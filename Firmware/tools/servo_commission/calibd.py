#!/usr/bin/env python3
"""calibd: calibration runs from the page (ADR-005 D7).

controld keeps the motors for a whole run and runs each session itself. calibd is commission.py's
orchestrator. It is a stack process, idle until the page starts a run, and a second client of
controld's socket (it reads the telemetry, it sends commands). For a run in CALIBRATING it:
- builds each session's manifest (manifests.py) and hands it to controld;
- reads the journal controld writes (journal.py);
- fits and designs between sessions while controld holds;
- finishes the run with its verdict.

What a run measures is the existing commissioning, its rules unchanged:
    VERIFY            check yaw                                 (~1 min)
    RERUN             pitch, then yaw --update                  (~20 min)
    RERUN+CROSSTALK   pitch, then yaw (a yaw motor change)      (~22 min)
    RERUN+ACCURACY    RERUN, then calibrate yaw                 (~40 min)
    RERUN+ACCURACY+CROSSTALK  both                              (~42 min)
    MEASURE           NEW's measurement: as RERUN+ACCURACY      (~40 min)
(measured on the station, 2026-10-24/25)

Each run is kept in <state>/calibration/runs/<run>-<kind>-<stamp>/, one directory per pipeline, with
run.json (the record, the kind, the verdict). Reports are kept; journals (~1 GB a full yaw run) only
for each record's latest installed run -- `calibrate yaw` reads its validation passes -- and the
three most recent runs (ADR-005 D3).

    python Firmware/tools/servo_commission/calibd.py [--socket PATH] [--state DIR]
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import shutil
import sys
import threading
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(FIRMWARE))

import journal  # noqa: E402

JOURNAL = {"yaw": "yaw-control.jsonl", "pitch": "sensorless-homing.jsonl"}
# What each pipeline leaves for the record when it is accepted (ADR-005 D7.6).
PRODUCES = {"pitch": "pitch_servo.json", "yaw": "yaw_servo.json", "calibrate": "yaw_accuracy.json"}
# What each kind runs: (pipeline, options) in order.
PLAN = {
    "VERIFY": [("check", {})],
    # Developer only, not on the page: the pitch servo's use-case check (commission.py check pitch).
    # It installs nothing; it is how controld's pitch executor is tried on the station.
    "CHECK-PITCH": [("check", {"axis": "pitch"})],
    "RERUN": [("pitch", {}), ("yaw", {"update": True})],
    "RERUN+CROSSTALK": [("pitch", {}), ("yaw", {"update": False})],
    "RERUN+ACCURACY": [("pitch", {}), ("yaw", {"update": True}), ("calibrate", {})],
    "RERUN+ACCURACY+CROSSTALK": [("pitch", {}), ("yaw", {"update": False}), ("calibrate", {})],
    "MEASURE": [("pitch", {}), ("yaw", {"update": True}), ("calibrate", {})],
}


class Stopped(RuntimeError):
    """The run ended under calibd: STOP, a fault, or controld gave up waiting."""


class Link:
    """controld's socket, as webd uses it: the latest telemetry and commands through its gate."""

    def __init__(self, socket_path: str):
        from web.webd.controld_client import ControldClient
        self.client = ControldClient(socket_path)
        self.client.start()

    def telemetry(self):
        return self.client.latest_telemetry()

    def send(self, name: str, arg: str = "") -> tuple[bool, str]:
        r = self.client.send_command(name, arg)
        return bool(r.ok), r.error or ""


class ControldStation:
    """commission.py's station interface (Station, SimStation), answered by controld (ADR-005 D7)."""

    name = "controld"

    def __init__(self, link, run_id: int, root: Path, log=print, start_timeout_s=10.0, poll_s=0.1):
        self.link, self.run_id, self.root, self.log = link, run_id, Path(root), log
        self.start_timeout_s, self.poll_s = start_timeout_s, poll_s

    def _run_state(self):
        t = self.link.telemetry()
        run = (t.calibration_run or {}) if t is not None else {}
        if run.get("id") != self.run_id or not run.get("active"):
            raise Stopped(run.get("verdict") or "the run ended")
        return t, run

    def run(self, axis, manifest, label):
        """One session: the manifest beside its journal, controld runs it, the journal comes back parsed."""
        from commission import Transient   # lazily, as the pipelines import commission (numpy, scipy)
        d = self.root / label
        d.mkdir(parents=True, exist_ok=True)
        bound = dict(manifest, session_label=label)
        bound.pop("output", None)
        path = d / "manifest.json"
        path.write_text(json.dumps(bound, indent=1), encoding="utf-8")
        ok, error = self.link.send("calibration_session", f"{self.run_id} {axis} {path}")
        if not ok:
            raise RuntimeError(f"{label}: controld refused the session: {error}")
        # Started: controld names it in calibration_run. Refused on the control thread: the ack says why.
        deadline = time.monotonic() + self.start_timeout_s
        while True:
            t, run = self._run_state()
            if (run.get("session") or {}).get("label") == label or str(run.get("last_session", "")).startswith(label + ":"):
                break
            if t.cmd_ack_command == "calibration_session" and t.cmd_ack_accepted == 0:
                raise RuntimeError(f"{label}: {t.cmd_ack_reason}")
            if time.monotonic() > deadline:
                raise Transient(f"{label}: controld did not start the session")
            time.sleep(self.poll_s)
        # Done: controld has handed the axis back and says how the session ended.
        while True:
            _, run = self._run_state()
            if (run.get("session") or {}).get("label") != label and str(run.get("last_session", "")).startswith(label + ":"):
                break
            time.sleep(self.poll_s)
        name = d / JOURNAL[axis]
        deadline = time.monotonic() + 20.0   # the footer is written on controld's writer thread
        while '"kind":"footer"' not in (name.read_text(encoding="utf-8")[-4096:] if name.exists() else ""):
            if time.monotonic() > deadline:
                raise Transient(f"{label}: the journal has no footer")
            time.sleep(self.poll_s)
        return (journal.yaw if axis == "yaw" else journal.pitch)(name)


class Runner:
    """One run: the kind's pipelines in order, notes for the page, a heartbeat, the verdict."""

    def __init__(self, link, state: Path, run: dict, record: dict, poll_s=0.1):
        self.link, self.run, self.record = link, run, record
        self.id, self.kind = run["id"], run["kind"]
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.root = state / "calibration" / "runs" / f"{self.id:04d}-{self.kind.lower()}-{stamp}"
        self.root.mkdir(parents=True, exist_ok=True)
        self.record_dir = state / "calibration" / "records" / f"{record['id']:04d}"
        self.poll_s = poll_s
        self.step = PLAN[self.kind][0][0]
        self.note = f"{self.step} | starting"
        self.done = threading.Event()

    def say(self, text: str):
        # "<step> | <line>": the page's run view marks the step and shows the line.
        text = " ".join(str(text).split())[:300]
        if text:
            self.note = f"{self.step} | {text}"
            self.link.send("calibration_note", f"{self.id} {self.note}")

    def heartbeat(self):
        # controld ends a run that hears nothing from calibd for 15 s (STAGE2.md); a fit or a design
        # between sessions can take longer than that on the Pi.
        while not self.done.wait(5.0):
            self.link.send("calibration_note", f"{self.id} {self.note}")

    def asset_for(self, axis: str) -> Path:
        """The servo a step works on: the one this run already accepted for the axis, so a later step
        measures what the run will install (MEASURE 2026-10-25 01:12: the accuracy campaign ran the
        record's bare-turret asset on the payload), else the record's."""
        accepted = self.root / axis / PRODUCES[axis]
        return accepted if accepted.exists() else self.record_dir / f"{axis}_servo.json"

    def pipeline(self, which: str, options: dict, station) -> dict:
        import commission
        out = self.root / which
        axis = options.get("axis", "pitch" if which == "pitch" else "yaw")
        args = SimpleNamespace(sim=None, out=str(out), no_write=True, no_build=True, resume=None, axis=axis,
                               update=options.get("update", False), from_prior=False, station=station,
                               on_log=self.say, asset=str(self.asset_for(axis)))
        commission.RUNS = self.root.parent
        try:
            {"check": commission.check, "pitch": commission.commission_pitch, "yaw": commission.commission_yaw,
             "calibrate": commission.calibrate}[which](args)
        except SystemExit:
            pass   # the pipeline's own STOPPED: its report says why
        report = out / "report.json"
        return json.loads(report.read_text(encoding="utf-8")) if report.exists() else {}

    def execute(self) -> str:
        """The verdict. A run whose every pipeline was accepted leaves its parts in install/, which
        controld puts into the record; anything less installs nothing (a payload set is one set)."""
        threading.Thread(target=self.heartbeat, daemon=True, name="calibd-heartbeat").start()
        verdicts, produced = [], []
        try:
            for which, options in PLAN[self.kind]:
                station = ControldStation(self.link, self.id, self.root / which / "sessions", log=self.say, poll_s=self.poll_s)
                self.step = which
                self.say("starting")
                report = self.pipeline(which, options, station)
                steps = report.get("steps", [])
                stopped = next((s.get("reason") for s in steps if s.get("step") == "stopped"), None)
                check = next((s for s in steps if s.get("step") == "check"), None)
                if stopped:
                    return f"NOT ACCEPTED: {which}: {stopped}"
                if check:
                    verdicts.append(check["verdict"])
                elif which in ("pitch", "yaw"):
                    if not report.get("accepted"):
                        return f"NOT ACCEPTED: {which} failed its validation"
                    verdicts.append(f"{which} ACCEPTED")
                else:
                    verdicts.append(f"{which} done")
                if which in PRODUCES and (self.root / which / PRODUCES[which]).exists():
                    produced.append(self.root / which / PRODUCES[which])
            if produced:
                install = self.root / "install"
                install.mkdir(exist_ok=True)
                for path in produced:
                    shutil.copy(path, install / path.name)
                verdicts.insert(0, "ACCEPTED")
            return "; ".join(verdicts) or "nothing ran"
        except Stopped as e:
            return f"STOPPED: {e}"
        finally:
            self.done.set()


KEEP_RECENT = 3


def prune(runs: Path, keep_recent=KEEP_RECENT) -> list:
    """Journals of runs nobody will read again: not a record's latest installed run, not recent.
    Returns the journals removed. Reports, logs and manifests stay."""
    done = []
    # Oldest first by when the run started: the name's trailing stamp, <id>-<kind>-YYYYMMDD-HHMMSS. Not
    # by name -- run ids restart with every controld, so the newest run can be 0001 beside an older
    # 0003, and sorting by name pruned the run that had just ended (2026-10-24 23:18).
    def started(d):
        stamp = d.name.rsplit("-", 2)[-2:]
        return "".join(stamp) if len(stamp) == 2 and all(s.isdigit() for s in stamp) else f"{d.stat().st_mtime:020.6f}"
    for d in sorted(runs.iterdir(), key=started) if runs.is_dir() else []:
        try:
            done.append((d, json.loads((d / "run.json").read_text(encoding="utf-8"))))
        except (OSError, ValueError):
            continue   # a run without run.json (in progress, or from before): left alone
    keep = {d for d, _ in done[-keep_recent:]}
    latest_installed = {}
    for d, info in done:
        if info.get("installed"):
            latest_installed[info.get("record")] = d
    keep |= set(latest_installed.values())
    removed = []
    for d, _ in done:
        if d in keep:
            continue
        for journal_file in d.rglob("*.jsonl"):
            journal_file.unlink()
            removed.append(journal_file)
    return removed


def serve(link, state: Path, poll_s=0.2, once=False):
    """Waits for runs; one at a time."""
    handled = set()
    while True:
        t = link.telemetry()
        run = t.calibration_run if t is not None else None
        if t is not None and t.calibrating and run and run.get("active") and run.get("state") == "waiting" \
                and run.get("id") not in handled:
            handled.add(run["id"])
            record = t.calibration_record or {}
            print(f"calibd: run {run['id']} {run['kind']} on #{record.get('id')} {record.get('name')}", flush=True)
            runner = Runner(link, state, run, record)
            try:
                verdict = runner.execute()
            except Exception as e:   # a bug here must not leave the station waiting 15 s for nothing
                traceback.print_exc()
                verdict = f"NOT ACCEPTED: calibd failed: {e}"
            print(f"calibd: run {run['id']}: {verdict}", flush=True)
            # A run that ended under calibd (STOP, a fault) is over: controld has recorded it already.
            ok, error = (False, "the run had ended") if verdict.startswith("STOPPED:") else \
                link.send("calibration_finish", f"{run['id']} {runner.root} {verdict[:400]}")
            if not ok:
                print(f"calibd: not finished: {error}", flush=True)
            # Whether controld installed it is its answer on the control thread (the ack), not the gate's.
            installed = False
            deadline = time.monotonic() + 3.0
            while ok and (runner.root / "install").is_dir() and time.monotonic() < deadline:
                t = link.telemetry()
                if t is not None and t.cmd_ack_command == "calibration_finish":
                    installed = "installed into" in (t.cmd_ack_reason or "")
                    break
                time.sleep(0.05)
            (runner.root / "run.json").write_text(json.dumps(
                {"run": run["id"], "kind": run["kind"], "record": record.get("id"), "verdict": verdict,
                 "installed": installed}, indent=1), encoding="utf-8")
            removed = prune(runner.root.parent)
            if removed:
                print(f"calibd: pruned {len(removed)} journals of older runs", flush=True)
            if once:
                return verdict
        time.sleep(poll_s)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--socket", default=os.environ.get("OTA_WEB_SOCKET", "/run/ota/controld-web.sock"))
    ap.add_argument("--state", type=Path, default=Path(os.environ.get("OTA_STATE_DIR", FIRMWARE.parent / "run/state")))
    a = ap.parse_args(argv)
    print(f"calibd: waiting for runs (controld {a.socket}, store {a.state / 'calibration'})", flush=True)
    serve(Link(a.socket), a.state)
    return 0


if __name__ == "__main__":
    sys.exit(main())
