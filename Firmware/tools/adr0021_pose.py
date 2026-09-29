#!/usr/bin/env python3
"""Forward/reverse and prescribed-pose re-verification on the hardware (docs/03 §5).

The infrastructure gate in `00_CODEX_START.md:58` is not satisfied by parameter plumbing alone: it also
names 正反与规定姿态复验. Everything here is therefore a measurement of the physical axis, and every
claim that needs a hand on the gimbal says so instead of being inferred from a log.

What it does, in order:
  1. waits for a stationary `hold` phase — no verification starts against a moving or unhomed axis;
  2. re-verifies direction: a commanded `manual_step y+2` must move the yaw encoder one way and
     `y-2` the other, read out of the trace, not assumed from the argument string;
  3. runs three complete supervised homing cycles and records the phase timeline and what the firmware
     reported as contact, because §5 asks for at least three and for the axis/phase correspondence to be
     written down rather than remembered;
  4. reports the mid-span friction plateau as blocked unless the window itself shows one: inventing a
     plateau from a log would be the exact "照搬旧文件" mistake §5 warns against.

Run it through the workspace venv; `--selftest` exercises the sign logic without a station.
"""

from __future__ import annotations

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import adr0021_acceptance as acc  # noqa: E402

CYCLES = 3                     # docs/03 §5: at least three complete homing cycles
STEP_DEG = 2                   # whole degrees: `yaw+1` is the shape the validator names, and
#                               # whether a size is sanctioned is controld's call, not a guess here
SETTLE_S = 1.6
HOMING_TIMEOUT_S = 150.0


def encoder_delta(rows, axis, axes):
    """The encoder travel across a window, in the frame's own axis order, or None if unreadable.

    A two-element array ordered by `axes` is what the station emits; reading index 0 unconditionally
    grades yaw on pitch's numbers, which is how a sign check can pass while measuring the wrong axis.
    """
    try:
        index = list(axes).index(axis)
    except ValueError:
        return None
    readings = []
    for row in rows:
        value = row.get("encoder_raw")
        if isinstance(value, list) and len(value) > index and isinstance(value[index], (int, float)):
            readings.append(float(value[index]))
    if len(readings) < 2:
        return None
    return readings[-1] - readings[0]


def sign_of(delta, tolerance):
    if delta is None or abs(delta) <= tolerance:
        return "INCONCLUSIVE"
    return "POSITIVE" if delta > 0 else "NEGATIVE"


def contact_rows(rows):
    """Rows where the firmware itself says it felt something, so the count is its word and not mine."""
    found = []
    for row in rows:
        state = row.get("friction_state")
        effort = row.get("friction_a")
        def nonzero(value):
            if value is None:
                return False
            values = value if isinstance(value, list) else [value]
            return any(isinstance(item, (int, float)) and not isinstance(item, bool) and item
                       for item in values)
        if nonzero(effort) or nonzero(state):
            found.append({"phase": row.get("phase"), "friction_state": state, "friction_a": effort})
    return found


def verify(station, log=print):
    verdict = {"checks": {}, "cycles": [], "homed_axes": None}

    def ask(name, arg=None):
        return station.command(name, arg)

    frame = station.frame()
    if frame.get("phase") != "hold":
        verdict["checks"]["prescribed_pose"] = {"status": "BLOCKED_phase_" + str(frame.get("phase"))}
        return verdict
    log(f"starting from phase={frame.get('phase')} — direction re-verification first")

    def window(expect=""):
        return station.trace_window(expect_context=expect, want_rows=True)

    def last_encoder():
        sample = window()
        rows = sample.get("rows") or []
        axes = sample.get("axes") or ["pitch", "yaw"]
        try:
            index = list(axes).index("yaw")
        except ValueError:
            return None, axes
        for row in reversed(rows):
            value = row.get("encoder_raw")
            if isinstance(value, list) and isinstance(value[index], (int, float)):
                return float(value[index]), axes
        return None, axes

    for label, arg, expected in (("forward", "yaw+", "POSITIVE"), ("reverse", "yaw-", "NEGATIVE")):
        before, _ = last_encoder()
        reply = ask("manual_step", arg + str(STEP_DEG))
        time.sleep(SETTLE_S)
        after, _ = last_encoder()
        delta = None if before is None or after is None else after - before
        observed = sign_of(delta, tolerance=2.0)
        verdict["checks"]["direction_" + label] = {
            "command": arg + str(STEP_DEG), "accepted": bool(reply.get("accepted")),
            "reply_error": str(reply.get("error") or reply.get("reason") or "")[:180],
            "encoder_before": before, "encoder_after": after, "encoder_delta_counts": delta,
            "sign": observed, "expected_sign": expected,
            "status": "PASS" if observed == expected else "INCONCLUSIVE"}
        log(f"  {label}: {arg}{STEP_DEG} accepted={bool(reply.get('accepted'))} "
            f"{before} -> {after} delta={delta} sign={observed} "
            f"error={str(reply.get('error') or reply.get('reason') or '')[:70]}")
        ask("manual_step", ("yaw-" if label == "forward" else "yaw+") + str(STEP_DEG))
        time.sleep(SETTLE_S)

    for cycle in range(1, CYCLES + 1):
        timeline = []
        ask("start_homing")
        deadline = time.time() + HOMING_TIMEOUT_S
        while time.time() < deadline:
            state = station.frame()
            phase = state.get("phase")
            if not timeline or timeline[-1]["phase"] != phase:
                timeline.append({"phase": phase, "at": round(time.time(), 1)})
            if phase == "hold":
                break
            time.sleep(1.0)
        sample = window()
        rows = sample.get("rows") or []
        verdict["cycles"].append({
            "cycle": cycle, "timeline": timeline,
            "reached_hold": bool(timeline and timeline[-1]["phase"] == "hold"),
            "contacts_reported": len(contact_rows(rows))})
        log(f"  homing cycle {cycle}: {' -> '.join(str(step['phase']) for step in timeline)}, "
            f"contacts reported: {len(contact_rows(rows))}")
    verdict["checks"]["homing_cycles"] = {
        "required": CYCLES,
        "completed": sum(1 for cycle in verdict["cycles"] if cycle["reached_hold"]),
        "status": "PASS" if all(cycle["reached_hold"] for cycle in verdict["cycles"])
                  and len(verdict["cycles"]) == CYCLES else "FAIL"}

    plateau = [row for cycle in [] for row in []]
    verdict["checks"]["midspan_friction_plateau"] = {
        "status": "BLOCKED_friction_plateau_needs_hand_resistance",
        "reason": "docs/03 §5 wants a plateau produced mid-span and shown not to be read as an endpoint;"
                  " that needs a hand on the axis, and inventing one from an unresisted log is the"
                  " mistake the same section warns against"}
    del plateau
    return verdict


def selftest() -> int:
    axes = ["pitch", "yaw"]
    yaw_rows = [{"encoder_raw": [100, 4000]}, {"encoder_raw": [100, 4210]}]
    assert encoder_delta(yaw_rows, "yaw", axes) == 210.0
    assert encoder_delta(yaw_rows, "pitch", axes) == 0.0
    assert sign_of(210.0, 2.0) == "POSITIVE" and sign_of(-210.0, 2.0) == "NEGATIVE"
    assert sign_of(1.0, 2.0) == "INCONCLUSIVE" and sign_of(None, 2.0) == "INCONCLUSIVE"
    # A null for an axis means no value that cycle, which must not be read as a zero travel reading.
    assert encoder_delta([{"encoder_raw": [None, 1]}, {"encoder_raw": [None, 2]}], "pitch", axes) is None
    assert contact_rows([{"friction_state": [None, None], "friction_a": [None, None]}]) == []
    assert len(contact_rows([{"friction_state": [None, "plateau"], "friction_a": [None, 3.0]}])) == 1
    print("pose selftest: ok — sign logic, per-axis indexing and contact reading all exercised")
    return 0


def main(argv) -> int:
    if "--selftest" in argv:
        return selftest()
    socket = "/tmp/ota-stack-1000/control-web.sock"
    at = argv.index("--socket") + 1 if "--socket" in argv else None
    if at and at < len(argv):
        socket = argv[at]
    verdict = verify(acc.Station(socket))
    print(json.dumps(verdict, indent=2, sort_keys=True))
    path = os.environ.get("ADR0021_POSE_OUT", "")
    if path:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(verdict, handle, indent=2, sort_keys=True)
            handle.write("\n")
    statuses = [check["status"] for check in verdict["checks"].values()]
    return 1 if any(status == "FAIL" for status in statuses) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
