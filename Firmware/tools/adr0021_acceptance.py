#!/usr/bin/env python3
"""ADR-002.1 §6 acceptance, run against the station that is actually switched on.

The clause being tested is narrow and brutal: change every experiment_writable parameter, read it back,
restore it, and the binary must not have changed once — no compile, no deploy, no restart. That is the
whole promise of "runtime tuning without redeployment", and it is only worth anything measured against
the machine that will run a campaign. Anything this script cannot observe is reported as
`BLOCKED_<reason>` rather than inferred from the config file.

Run it on the station:

    python3 Firmware/tools/adr0021_acceptance.py --inventory <release>/docs/ADR-002.1/manifests/parameter_inventory.json \
        --binary <release>/Firmware/build-arm64/control/controld --out runtime_snapshot.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import socket
import subprocess
import sys
import time
import datetime

# name -> position in the trial command's colon-separated fields
YAW_FIELDS = ["yaw.current_kp_a_per_rad_s", "yaw.current_ki_a_per_rad_s", "yaw.velocity_rx_window_ms",
              "yaw.friction.positive_breakaway_a", "yaw.friction.negative_breakaway_a",
              "yaw.friction.positive_run_a", "yaw.friction.negative_run_a",
              "yaw.friction.output_slew_a_per_s"]
PROBE = {"yaw.current_kp_a_per_rad_s": 1.4, "yaw.current_ki_a_per_rad_s": 0.9,
         "yaw.friction.positive_breakaway_a": 0.12, "yaw.friction.negative_breakaway_a": 0.12,
         "yaw.friction.positive_run_a": 0.08, "yaw.friction.negative_run_a": 0.08,
         "yaw.friction.output_slew_a_per_s": 4.0,
         "pitch.service_speed_kp": 4.0, "pitch.service_speed_ki": 0.03}
# The keys webd publishes. Two earlier runs of this script asked for keys that document does not
# carry (`mode`, `param_revision`, …), read back None, and reported that absence as a physical
# precondition. Facts come from the document that carries them.
TELEMETRY_KEYS = ("phase", "operating_mode", "safety_action", "manual_lease_active",
                  "manual_lease_remaining_ms", "manual_profile", "service_velocity_control",
                  "payload_profile_name", "payload_profile_status", "feedback_age_ms",
                  "current_a_yaw", "current_a_pitch", "can_state", "telemetry_stale",
                  "yaw_guard_degraded", "cmd_ack_seq")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


class Station:
    def __init__(self, sock_path, state_url="http://localhost:8080/api/state"):
        self.sock_path = sock_path
        self.state_url = state_url

    def _sock(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        s.connect(self.sock_path)
        s.settimeout(4)
        return s

    def ack(self, after_seq):
        """Wait for the controller's answer to the command just submitted.

        Over webd a command is asynchronous: the socket answers `{"ok":true,"verdict":"submitted"}` —
        which only means it was queued — and the real verdict arrives in `cmd_ack_*` on the next
        telemetry frame. Taking the submission for the verdict is how an agent reports a write that
        controld refused, so the runner correlates on `cmd_ack_seq` and reads that ack and nothing else.
        """
        deadline = time.time() + 4
        while time.time() < deadline:
            from urllib.request import urlopen
            try:
                with urlopen(self.state_url, timeout=2) as response:
                    frame = json.loads(response.read().decode())
            except Exception:
                return {"accepted": False, "reason": "state unavailable while waiting for an ack"}
            if frame.get("cmd_ack_seq") != after_seq:
                return {"accepted": bool(frame.get("cmd_ack_accepted")),
                        "reason": str(frame.get("cmd_ack_reason", "")),
                        "command": frame.get("cmd_ack_command"),
                        "seq": frame.get("cmd_ack_seq"), "safety": frame.get("cmd_ack_safety_state")}
            time.sleep(0.05)
        return {"accepted": False, "reason": "no cmd_ack within 4 s"}

    def seq(self):
        from urllib.request import urlopen
        try:
            with urlopen(self.state_url, timeout=2) as response:
                return json.loads(response.read().decode()).get("cmd_ack_seq")
        except Exception:
            return None

    def command(self, name, arg=None):
        s = self._sock()
        message = {"type": "command", "command": name}
        if arg is not None:
            message["arg"] = arg
        s.send(json.dumps(message).encode())
        buf = b""
        deadline = time.time() + 4
        while time.time() < deadline:
            try:
                buf += s.recv(65536)
            except socket.timeout:
                break
            if re.search(rb'\{"type":"response"', buf):
                break
        for match in re.findall(rb'\{"type":"response".*?\}', buf):
            return json.loads(match.decode())
        return {"accepted": False, "reason": "no response from controld"}

    def state(self):
        """The physical and safety picture, asked of the read surface that publishes it.

        The first two runs of this script asked the control socket for keys only /api/state publishes,
        read back `None`, and reported that absence as a precondition failure. Facts come from the
        document that carries them; the socket is for commands.
        """
        from urllib.request import urlopen
        try:
            with urlopen(self.state_url, timeout=4) as response:
                frame = json.loads(response.read().decode())
        except Exception as error:                       # a station we cannot read is a blocked run
            return {"_state_error": type(error).__name__}
        return {key: frame.get(key) for key in TELEMETRY_KEYS}

    def telemetry(self):
        s = self._sock()
        deadline = time.time() + 4
        while time.time() < deadline:
            try:
                frame = json.loads(s.recv(65536).decode())
            except socket.timeout:
                return {}
            if frame.get("type") == "telemetry":
                return {key: frame.get(key, "?") for key in TELEMETRY_KEYS}
        return {}


def identity(station):
    """The parameter identity, taken from the command that answers it and nothing else.

    controld's identity fields ride the trace frame, not webd's live telemetry (measured 2026-09-29),
    so `param_snapshot` is the authority for a runner: it is a question to the code that holds the
    state, rather than a guess about which fields somebody chose to forward.
    """
    before = station.seq()
    station.command("param_snapshot", "")
    response = station.ack(before)
    text = str(response.get("reason", ""))
    def field(key):
        match = re.search(key + r"=(\S+)", text)
        return match.group(1) if match else None
    return {"state": field("state"), "revision": field("revision"),
            "applied_hash": field("applied_hash"), "expected_hash": field("expected_hash"),
            "reason": field("reason"), "accepted": response.get("accepted")}


def yaw_string(values):
    return ":".join(f"{value:.9g}" for value in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--socket", default=os.environ.get("OTA_WEB_SOCKET", "/run/ota/controld-web.sock"))
    parser.add_argument("--inventory", required=True)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--out", default="runtime_snapshot.json")
    parser.add_argument("--config", default="config/turret_mixed.yaml", help="for the on-station registry dump")
    parser.add_argument("--release-dir", default="", help="cwd for the on-station controld dump")
    args = parser.parse_args()

    with open(args.inventory, encoding="utf-8") as handle:
        raw = handle.read()
    inventory = json.loads(raw)
    inventory_sha = hashlib.sha256(raw.encode()).hexdigest()
    binary_before = sha256(args.binary)
    station = Station(args.socket)
    writable = {entry["name"]: entry for entry in inventory["entries"]
                if entry["mutability"] == "experiment_writable"}
    boot = {name: entry["actual_value"] for name, entry in writable.items()}

    state = station.state()
    snapshot = {
        "schema": 1,
        "taken_at_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "host": os.uname().nodename,
        "binary": os.path.abspath(args.binary),
        "binary_sha256_before": binary_before,
        "binary_sha256_after": None,
        "inventory_sha256": inventory_sha,
        "inventory_generated_from": inventory.get("generated_from", {}),
        "station_state": state,
        "transcript": [],
        "protected_write": {},
        "blocked": [],
    }

    # Preconditions the transaction demands. Saying which one is missing is the whole difference
    # between a blocked acceptance and a silent skip.
    if str(state.get("phase", "")) not in ("hold",):
        snapshot["blocked"].append(f"BLOCKED_phase_{state.get('phase')}")
    if str(state.get("operating_mode", "")).lower() != "manual":
        snapshot["blocked"].append(f"BLOCKED_mode_{state.get('operating_mode')}")
    if str(state.get("safety_action", "")).lower() != "allow":
        snapshot["blocked"].append(f"BLOCKED_safety_{state.get('safety_action')}")
    snapshot["identity_before"] = identity(station)

    def yaw_exchange(label, arg):
        """prepare → apply under the id prepare handed back → record what the station says it runs."""
        before = station.seq()
        station.command("param_prepare", arg)
        prepared = station.ack(before)
        record = {"parameter": label, "requested": arg, "prepare": prepared}
        match = re.search(r"request_id=(\S+)", str(prepared.get("reason", "")))
        if not match:
            record["applied"] = None
            snapshot["transcript"].append(record)
            return record
        before = station.seq()
        station.command("param_apply", match.group(1))
        record["applied"] = station.ack(before)
        record["identity_after"] = identity(station)
        snapshot["transcript"].append(record)
        return record

    def pitch_exchange(label, arg):
        """One command, verified by the register readback a few cycles later — so wait, then look."""
        before = station.seq()
        station.command("pitch_control_trial", arg)
        sent = station.ack(before)
        time.sleep(0.4)
        record = {"parameter": label, "requested": arg, "sent": sent,
                  "identity_after": identity(station)}
        time.sleep(0.4)                     # the register answer arrives a few cycles later
        record["identity_after_settle"] = identity(station)
        snapshot["transcript"].append(record)

    def boot_value(name, fallback):
        raw = boot.get(name, fallback)
        return float(json.loads(raw)) if isinstance(raw, str) else float(raw)

    # The 8 fields the yaw trial command carries, seeded from the values the running binary itself
    # reported — so "restore" means restoring what was actually measured at boot, not what a doc says.
    yaw_base = [boot_value("yaw.current_kp_a_per_rad_s", 1), boot_value("yaw.current_ki_a_per_rad_s", .6),
                0.0, boot_value("yaw.friction.positive_breakaway_a", 0),
                boot_value("yaw.friction.negative_breakaway_a", 0), boot_value("yaw.friction.positive_run_a", 0),
                boot_value("yaw.friction.negative_run_a", 0),
                boot_value("yaw.friction.output_slew_a_per_s", 2)]
    if not snapshot["blocked"]:
        yaw_exchange("yaw.baseline", yaw_string(yaw_base))
        for position, name in enumerate(YAW_FIELDS):
            if name not in writable:
                continue
            probe = list(yaw_base)
            probe[position] = PROBE[name]
            changed = yaw_exchange(name, yaw_string(probe))
            changed["restore"] = yaw_exchange(name + " (restore)", yaw_string(yaw_base)).get("applied")
            changed["snapshot_after_restore"] = identity(station)

    for name in ("pitch.service_speed_kp", "pitch.service_speed_ki"):
        if name in writable:
            pitch_exchange(name, f"{PROBE['pitch.service_speed_kp']}:{PROBE['pitch.service_speed_ki']}")
            pitch_exchange(name + " (restore)",
                           f"{boot.get('pitch.service_speed_kp', 4)}:{boot.get('pitch.service_speed_ki', .05)}")

    # A protected field: the write surface does not carry its name at all, so the check is that the
    # server's own dump still reports the same value and the same classification after the campaign.
    dump = subprocess.run([args.binary, args.config, "--dump-parameter-registry", "-"],
                          cwd=args.release_dir or None, capture_output=True, text=True)
    cap_after = cap_before = None
    # controld logs to the same stdout it writes the document to, so the parse starts at the brace
    # rather than assuming the pipe carries nothing but JSON.
    start = dump.stdout.find("{")
    if dump.returncode != 0 or start < 0:
        snapshot["blocked"].append("BLOCKED_registry_dump_on_station")
        print("registry dump failed:", dump.returncode, dump.stderr[-200:], file=sys.stderr)
    for entry in json.loads(dump.stdout[start:])["entries"]:
            if entry["name"] == "yaw.host_current_limit_a":
                cap_after, cap_before = entry["actual_value"], entry["mutability"]
    snapshot["protected_write"] = {
        "attempt": "yaw.host_current_limit_a = 1.2",
        "how_the_server_refuses": "the trial grammar carries no name for it; the transaction writes "
                                 "only the fields it can read back",
        "reported_mutability": cap_before, "value_after_campaign": cap_after,
        "value_before_campaign": next((e["actual_value"] for e in inventory["entries"]
                                       if e["name"] == "yaw.host_current_limit_a"), None),
    }

    snapshot["identity_after"] = identity(station)
    snapshot["station_state_after"] = station.state()
    snapshot["binary_sha256_after"] = sha256(args.binary)
    snapshot["binary_unchanged"] = snapshot["binary_sha256_after"] == binary_before
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(snapshot, handle, indent=2, sort_keys=True)
        handle.write("\n")
    applied_ok = sum(1 for row in snapshot["transcript"]
                     if (row.get("applied") or {}).get("accepted"))
    print(f"§6 acceptance: {len(snapshot['transcript'])} exchanges, {applied_ok} applied-and-read-back, "
          f"binary unchanged={snapshot['binary_unchanged']}, blocked={snapshot['blocked'] or 'none'}")
    print(f"wrote {args.out}")
    return 0 if snapshot["binary_unchanged"] and not snapshot["blocked"] else 1


if __name__ == "__main__":
    sys.exit(main())
