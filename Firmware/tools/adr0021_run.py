#!/usr/bin/env python3
"""The Pi-local campaign runner: it obeys campaign.lock.json and decides nothing of its own.

Slice B of ADR-002.1 asks for one local runner, a frozen planner and scorer, automatic archiving, and a
payload binding — deliberately not a service, not a UI, not a database. This is that runner. It reads the
lock, checks that the machine in front of it is still the machine the lock was frozen against, walks the
candidate sequence, and archives every exchange. It does not choose a candidate, does not decide to stop
early, and does not report an improvement it cannot score.

Anything it cannot observe is a `BLOCKED_<reason>` in the manifest, never a guess with a number in it.

    python3 Firmware/tools/adr0021_run.py --lock campaign.lock.json --baseline runtime_snapshot.json \
        --out run/campaigns --socket /tmp/ota-stack-1000/control-web.sock
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time

import adr0021_acceptance as acc
import adr0021_plan as plan

FIELDS = ["yaw.current_kp_a_per_rad_s", "yaw.current_ki_a_per_rad_s", "yaw.velocity_rx_window_ms",
          "yaw.friction.positive_breakaway_a", "yaw.friction.negative_breakaway_a",
          "yaw.friction.positive_run_a", "yaw.friction.negative_run_a",
          "yaw.friction.output_slew_a_per_s"]


def baseline_from(snapshot: dict) -> list:
    """The values to return to between candidates, taken from what the station actually reported.

    A campaign's baseline is not what a config file says the machine should be; it is the set that was
    verified to be in the drives when the campaign started. The acceptance snapshot is where that was
    measured, so the runner inherits it rather than restating it.
    """
    rows = {row["parameter"]: row.get("requested") for row in snapshot.get("transcript", [])}
    first = rows.get("yaw.baseline") or ""
    values = [float(part) for part in first.split(":")] if first else []
    if len(values) != len(FIELDS):
        raise ValueError("the snapshot's yaw.baseline does not carry all eight fields; a campaign "
                         "cannot restore to a baseline it never measured")
    return values


class Runner:
    def __init__(self, station, lock: dict, inventory_sha: str, binary_sha: str, baseline: list,
                 archive_dir: str, score_command: str = "", now=time.time, scorer=None,
                 run_trials=False):
        self.station, self.lock = station, lock
        self.design = lock["design"]
        self.inventory_sha, self.binary_sha = inventory_sha, binary_sha
        self.baseline, self.archive_dir = baseline, archive_dir
        self.score_command, self.now = score_command, now
        self.scorer = scorer            # the frozen scorer module, when the campaign asked to be scored
        self.run_trials = run_trials    # drive a real trial between applying and archiving (:46)
        self.manifest = {"campaign_id": self.design["campaign_id"], "design_sha256": lock["design_sha256"],
                         "started_at": self.now(), "blocked": [], "refused": [], "exchanges": 0,
                         "applied": 0, "trials": [], "binding": lock["bound_to"]}

    def refuse(self, reason: str) -> bool:
        """Returns False on purpose. A refusal that came back as the manifest would have been a
        truthy object, and `if not self.check_bindings()` would have walked straight past it —
        which is what the first version of this method did, refusing and then running all eight
        candidates anyway. A refusal has to be shaped like a refusal."""
        self.manifest["blocked"].append(reason)
        return False

    def check_bindings(self) -> bool:
        """The lock is only a plan if the machine still matches the plan's assumptions."""
        if plan.sha256_text(plan.canonical(self.design)) != self.lock["design_sha256"]:
            self.refuse("BLOCKED_lock_tampered: the design does not hash to its own digest")
            return False
        if self.lock["bound_to"]["inventory_sha256"] != self.inventory_sha:
            self.refuse("BLOCKED_inventory_drift: parameters changed since the design was frozen; "
                               "a campaign spans one parameter set, not whatever the tree says now")
            return False
        if self.lock["bound_to"]["binary_sha256"] != self.binary_sha:
            self.refuse("BLOCKED_binary_drift: the running controld is not the one this campaign "
                               "was designed against; the evidence would be about a different machine")
            return False
        if self.design.get("scorer", {}).get("sha256") and self.score_command:
            digest = hashlib.sha256(open(self.score_command.split()[0], "rb").read()).hexdigest()
            if digest != self.design["scorer"]["sha256"]:
                self.refuse("BLOCKED_scorer_drift: the frozen scorer is not the file the design "
                            "was written against")
                return False
        return True

    def candidate_values(self, params: dict) -> list:
        values = list(self.baseline)
        for name, value in params.items():
            values[FIELDS.index(name)] = float(value)
        return values

    def exchange(self, command: str, arg: str = "") -> dict:
        """Send one command and wait for the ack that follows it — in that order, every time.

        Sampling the sequence counter after sending is the trap I fell into on hardware: the counter
        had already moved past the command being answered, so the ack looked like it never arrived,
        the restore was silently skipped, and the campaign still reported itself clean. Capture first,
        send second. A campaign that cannot return to its baseline is blocked, not finished.
        """
        before = self.station.seq()
        self.station.command(command, arg)
        return self.station.ack(before)

    def trial(self, candidate: dict, stage: str) -> dict:
        """One candidate: apply it, record the identity the controller confirms, restore the baseline."""
        values = self.candidate_values(candidate["params"])
        baseline_string = ":".join(f"{v:.9g}" for v in self.baseline)
        record = {"candidate_id": candidate["candidate_id"], "stage": stage,
                  "params": candidate["params"], "applied_string": ":".join(f"{v:.9g}" for v in values)}
        # Say which candidate this is, in the words the campaign archived it under, before anything is
        # written: docs/02 §5 wants the identity inside every trace record, not inferred afterwards.
        context = f"{self.manifest['campaign_id']}|{candidate['candidate_id']}|{stage}"[:39]
        record["context"] = self.exchange("param_context", context)
        prepared = self.exchange("param_prepare", record["applied_string"])
        self.manifest["exchanges"] += 1
        import re
        match = re.search(r"request_id=(\S+)", str(prepared.get("reason", "")))
        if not prepared.get("accepted") or not match:
            record["prepare"] = prepared
            if not prepared.get("accepted"):
                self.manifest["refused"].append({"candidate_id": candidate["candidate_id"],
                                                 "reason": prepared.get("reason")})
            return record
        applied = self.exchange("param_apply", match.group(1))
        self.manifest["exchanges"] += 1
        record["applied"] = applied
        if not applied.get("accepted"):
            self.manifest["refused"].append({"candidate_id": candidate["candidate_id"],
                                             "reason": applied.get("reason")})
            return record
        self.manifest["applied"] += 1
        # Ask for the trace window while this candidate is still what is running: §5 wants the identity
        # inside every record, and the only honest way to know it is there is to ask for the window and
        # count the records that carry the tag. A truncated read is a blocked run, not a short one.
        # 00_CODEX_START.md:46 puts a RUN between the applied write and the archived log: gains that were
        # never driven are not trials, they are configuration changes. The trial command is the firmware's
        # own guarded path, so a refusal here is a gate speaking, not a quality verdict — and the
        # candidate still has to be restored, because no candidate may be left in the machine.
        ran = True
        if self.run_trials:
            run = self.exchange("yaw_control_trial", record["applied_string"])
            record["run"] = run
            ran = bool(run.get("accepted"))
            if not ran:
                self.manifest["refused"].append({"candidate_id": candidate["candidate_id"],
                                                 "reason": "RUN refused: " + str(run.get("reason"))})
        if ran:
            window = self.station.trace_window(expect_context=context)
            record["trace_window"] = window
            if window.get("truncated"):
                self.refuse(f"BLOCKED_trace_truncated_{candidate['candidate_id']}: "
                            + str(window.get("reason")))
                return record
            if not window.get("records"):
                self.refuse(f"BLOCKED_trace_window_empty_{candidate['candidate_id']}: the station returned "
                            "no records to check, and an empty window satisfies a count comparison by "
                            "saying nothing")
                return record
            if window.get("records_with_context") == 0:
                self.refuse(f"BLOCKED_trace_identity_absent_{candidate['candidate_id']}: the campaign "
                            "announced itself and no record in the window carries the tag")
                return record
            if not window.get("contiguous_to_newest"):
                self.refuse(f"BLOCKED_trace_identity_missing_{candidate['candidate_id']}: "
                            f"{window.get('records_with_context')}/{window.get('records')} records carry the tag and "
                            "they do not run unbroken to the newest record; a window whose identity changes "
                            "mid-flight cannot say which candidate a given row belongs to")
                return record
            if self.scorer is not None:
                score = self.scorer.score_window(window.get("rows") or [], axis="yaw",
                                                 axes=tuple(window.get("axes") or ("pitch", "yaw")))
                record["score"] = {"classification": score["classification"],
                                   "metrics_sha256": score["metrics_sha256"],
                                   "metrics": {name: row["status"]
                                               for name, row in score["metrics"].items()}}
                frozen = self.lock["bound_to"].get("metrics_sha256")
                if frozen and score["metrics_sha256"] != frozen:
                    self.refuse("BLOCKED_metrics_version_drift: the campaign was frozen against " +
                                frozen[:12] + " and the scorer now says " +
                                score["metrics_sha256"][:12] + "; the classification of this candidate " +
                                "would not be the classification the lock promised")
                    return record
                tallied = self.manifest.setdefault("classifications", {})
                tallied[score["classification"]] = tallied.get(score["classification"], 0) + 1

        snapshot = self.exchange("param_snapshot")
        text = str(snapshot.get("reason", ""))
        grab = lambda key: (re.search(key + r"=(\S+)", text) or [None, None])[1]
        record["identity"] = {"revision": grab("revision"), "state": grab("state"),
                              "applied_hash": grab("applied_hash")}
        # Back to the baseline, and the restore is judged: a campaign that cannot return where it
        # started has not run a trial, it has moved the machine.
        restore_prepare = self.exchange("param_prepare", baseline_string)
        self.manifest["exchanges"] += 1
        r = re.search(r"request_id=(\S+)", str(restore_prepare.get("reason", "")))
        if not restore_prepare.get("accepted") or not r:
            record["restore"] = restore_prepare
            self.refuse(f"BLOCKED_restore_prepare_{candidate['candidate_id']}: "
                        + str(restore_prepare.get("reason"))[:120])
            return record
        restore = self.exchange("param_apply", r.group(1))
        self.manifest["exchanges"] += 1
        record["restore"] = restore
        if not restore.get("accepted"):
            self.refuse(f"BLOCKED_restore_failed_{candidate['candidate_id']}: the baseline could not "
                        "be re-applied, so the machine is left holding this candidate")
            return record
        self.manifest["applied"] += 1
        return record

    def score(self, record: dict):
        """Score only if a frozen scorer exists; an unscored trial is never called an improvement."""
        if not self.score_command:
            record["metrics"] = None
            record["unscored_reason"] = "no scorer was named: the runner records, it does not flatter"
            return None
        import subprocess
        path = os.path.join(self.archive_dir, f"{record['stage']}-{record['candidate_id']}.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(record, handle, sort_keys=True)
        outcome = subprocess.run(self.score_command.split() + [path], capture_output=True, text=True)
        if outcome.returncode != 0:
            record["metrics"] = None
            record["scorer_refused"] = outcome.stderr.strip()[-200:]
            return None
        try:
            record["metrics"] = json.loads(outcome.stdout)
        except json.JSONDecodeError:
            record["metrics"] = None
            record["scorer_refused"] = "the scorer did not answer with a JSON object"
            return None
        return record["metrics"]

    def run(self) -> dict:
        os.makedirs(self.archive_dir, exist_ok=True)
        if not self.check_bindings():
            return self._finish()
        frame = self.station.frame()
        want = str(self.design.get("payload_profile", ""))
        if frame.get("payload_profile_status") not in (want, None):
            self.refuse(f"BLOCKED_payload_profile_{frame.get('payload_profile_status')}: the campaign is "
                        f"bound to {want!r} and the station says otherwise")
            return self._finish()
        if frame.get("phase") != "hold":
            self.refuse(f"BLOCKED_phase_{frame.get('phase')}")
            return self._finish()
        stop = self.design["stop"]
        for candidate in self.design["coarse"]:
            if len(self.manifest["trials"]) >= int(stop["max_trials"]):
                self.manifest["stopped_by"] = "max_trials"
                break
            record = self.trial(candidate, "coarse")
            self.score(record)                      # fills in metrics, or why there are none
            self.manifest["trials"].append(record)
        if self.manifest.get("stopped_by") is None:
            self.manifest["stopped_by"] = "design_exhausted"
        return self._finish()

    def _finish(self) -> dict:
        self.manifest["ended_at"] = self.now()
        with open(os.path.join(self.archive_dir, "manifest.json"), "w", encoding="utf-8") as handle:
            json.dump(self.manifest, handle, indent=2, sort_keys=True)
            handle.write("\n")
        return self.manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--lock", required=True)
    parser.add_argument("--baseline", required=True, help="a runtime snapshot whose yaw.baseline was measured")
    parser.add_argument("--inventory", default=plan.INVENTORY)
    parser.add_argument("--binary", default="build/control/controld")
    parser.add_argument("--socket", default=os.environ.get("OTA_WEB_SOCKET", "/tmp/ota-stack-1000/control-web.sock"))
    parser.add_argument("--out", default="run/campaigns")
    parser.add_argument("--score-command", default="")
    parser.add_argument("--score", action="store_true",
                        help="score each candidate's trace window with the frozen scorer")
    parser.add_argument("--run-trials", action="store_true",
                        help="drive the firmware's own guarded trial after applying (00_CODEX_START.md:46)"
                             " — the axis moves, and that is the point of a trial")
    args = parser.parse_args()
    with open(args.lock, encoding="utf-8") as handle:
        lock = json.load(handle)
    inventory = plan.load_inventory(args.inventory)
    baseline = baseline_from(json.load(open(args.baseline, encoding="utf-8")))
    station = acc.Station(args.socket.replace("control-web.sock", "control-web.sock"))
    archive = os.path.join(args.out, lock["design"]["campaign_id"], str(int(time.time())))
    scorer = __import__("adr0021_scorer") if args.score else None
    manifest = Runner(station, lock, inventory["_sha256"], acc.sha256(args.binary), baseline,
                      archive, args.score_command, scorer=scorer,
                      run_trials=args.run_trials).run()
    print(f"campaign {manifest['campaign_id']}: exchanges={manifest['exchanges']} "
          f"applied={manifest['applied']} stopped_by={manifest.get('stopped_by', 'not_run')} "
          f"blocked={manifest['blocked'] or 'none'}")
    print(f"archived under {archive}")
    return 0 if not manifest["blocked"] else 1


if __name__ == "__main__":
    sys.exit(main())
