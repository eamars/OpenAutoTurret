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
                 archive_dir: str, score_command: str = "", now=time.time):
        self.station, self.lock = station, lock
        self.design = lock["design"]
        self.inventory_sha, self.binary_sha = inventory_sha, binary_sha
        self.baseline, self.archive_dir = baseline, archive_dir
        self.score_command, self.now = score_command, now
        self.manifest = {"campaign_id": self.design["campaign_id"], "design_sha256": lock["design_sha256"],
                         "started_at": self.now(), "blocked": [], "exchanges": 0,
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

    def trial(self, candidate: dict, stage: str) -> dict:
        """One candidate: apply it, record the identity the controller confirms, restore the baseline."""
        values = self.candidate_values(candidate["params"])
        record = {"candidate_id": candidate["candidate_id"], "stage": stage,
                  "params": candidate["params"], "applied_string": ":".join(f"{v:.9g}" for v in values)}
        before = self.station.seq()
        self.station.command("param_prepare", record["applied_string"])
        prepared = self.station.ack(before)
        self.manifest["exchanges"] += 1
        request_id = ""
        import re
        match = re.search(r"request_id=(\S+)", str(prepared.get("reason", "")))
        if not prepared.get("accepted") or not match:
            record["prepare"] = prepared
            return record
        before = self.station.seq()
        self.station.command("param_apply", match.group(1))
        applied = self.station.ack(before)
        self.manifest["exchanges"] += 1
        record["applied"] = applied
        if applied.get("accepted"):
            self.manifest["applied"] += 1
            frame = self.station.frame()
            record["identity"] = {"revision": frame.get("param_revision"),
                                  "applied_hash": frame.get("param_applied_hash")}
            import re as _re
            snap_before = self.station.seq()
            self.station.command("param_snapshot", "")
            text = str(self.station.ack(snap_before).get("reason", ""))
            grab = lambda key: (_re.search(key + r"=(\S+)", text) or [None, None])[1]
            record["identity"] = {"revision": grab("revision"), "state": grab("state"),
                                  "applied_hash": grab("applied_hash")}
            self.station.command("param_prepare", ":".join(f"{v:.9g}" for v in self.baseline))
            restore_id = ""
            r = _re.search(r"request_id=(\S+)", str(self.station.ack(self.station.seq()).get("reason", "")))
            if r:
                restore_id = r.group(1)
                before = self.station.seq()
                self.station.command("param_apply", restore_id)
                record["restore"] = self.station.ack(before)
                self.manifest["exchanges"] += 2
                if record["restore"].get("accepted"):
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
    args = parser.parse_args()
    with open(args.lock, encoding="utf-8") as handle:
        lock = json.load(handle)
    inventory = plan.load_inventory(args.inventory)
    baseline = baseline_from(json.load(open(args.baseline, encoding="utf-8")))
    station = acc.Station(args.socket.replace("control-web.sock", "control-web.sock"))
    archive = os.path.join(args.out, lock["design"]["campaign_id"], str(int(time.time())))
    manifest = Runner(station, lock, inventory["_sha256"], acc.sha256(args.binary), baseline,
                      archive, args.score_command).run()
    print(f"campaign {manifest['campaign_id']}: exchanges={manifest['exchanges']} "
          f"applied={manifest['applied']} stopped_by={manifest.get('stopped_by', 'not_run')} "
          f"blocked={manifest['blocked'] or 'none'}")
    print(f"archived under {archive}")
    return 0 if not manifest["blocked"] else 1


if __name__ == "__main__":
    sys.exit(main())
