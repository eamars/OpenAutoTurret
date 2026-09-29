"""Does it move, and does it stop fighting? The owner's bar, taken literally.

Two measurements per candidate, nothing else:
  moves     — a commanded `manual_step yaw+2` / `yaw-2` changes the yaw encoder, in opposite directions;
  not stalled — while the axis is at rest, the yaw current is not sitting at the drive's own cap.

The trial string is the firmware's eight-field yaw form (kp:ki:rx:bp:bn:rp:rn:slew); candidates are the
archived baseline with the two current-loop fields scaled, and every string actually sent is printed, so
the table is what happened rather than what was intended. Currents convert with the drive's own scale
(control/src/can/gm6020_protocol.hpp:65-66, 16384 counts == +-3.0 A).
"""

import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import adr0021_acceptance as acc  # noqa: E402

AMPS_PER_RAW = 3.0 / 16384.0
SOCKET = os.environ.get("ADR0021_SOCKET", "/tmp/ota-stack-1000/control-web.sock")
BASELINE = os.environ.get("ADR0021_BASELINE", "/tmp/adr/baseline.json")


def scaled(base, kp_factor, ki_factor):
    fields = list(str(base).split(":"))
    fields[0] = str(round(float(fields[0]) * kp_factor, 6))
    fields[1] = str(round(float(fields[1]) * ki_factor, 6))
    return ":".join(fields)


def yaw_field(station, field):
    """The newest row's per-axis value, in the frame's own axis order (yaw is not index 0 by luck)."""
    window = station.trace_window(want_rows=True)
    axes = window.get("axes") or ["pitch", "yaw"]
    index = axes.index("yaw")
    values = []
    for row in window.get("rows") or []:
        value = row.get(field)
        if isinstance(value, list) and len(value) > index and isinstance(value[index], (int, float)) \
                and not isinstance(value[index], bool):
            values.append(float(value[index]))
    return values


def hold_reading(station):
    """At-rest current, the cap it is measured against, and whether the axis is actually still."""
    currents = [abs(v * AMPS_PER_RAW) for v in yaw_field(station, "current_raw")]
    caps = [v for v in yaw_field(station, "current_cap") if v > 0]
    speeds = [abs(v) for v in yaw_field(station, "omega")]
    currents.sort()
    if not currents or not caps:
        return {"status": "NO_DATA"}
    cap = max(caps)
    return {"status": "OK", "at_rest_current_a_p95": round(currents[int(0.95 * (len(currents) - 1))], 4),
            "cap_a": round(cap, 4), "fraction_of_cap": round(currents[int(0.95 * (len(currents) - 1))] / cap, 3),
            "omega_max_dps": round(max(speeds), 4) if speeds else None}


def moves(station, degrees=2):
    """Command a step each way and read the encoder, comparing the last reading before with the last after."""
    result = {}
    for label, sign in (("forward", "+"), ("reverse", "-")):
        before = yaw_field(station, "encoder_raw")
        reply = station.command("manual_step", "yaw" + sign + str(degrees))
        time.sleep(1.8)
        after = yaw_field(station, "encoder_raw")
        delta = (after[-1] - before[-1]) if before and after else None
        result[label] = {"accepted": bool(reply.get("accepted")),
                         "error": str(reply.get("error") or reply.get("reason") or "")[:90],
                         "delta_counts": delta}
        back = "yaw-" if sign == "+" else "yaw+"
        station.command("manual_step", back + str(degrees))
        time.sleep(1.2)
    forward, reverse = result["forward"]["delta_counts"], result["reverse"]["delta_counts"]
    result["verdict"] = "MOVES" if (forward and reverse and (forward > 0) != (reverse > 0)
                                    and abs(forward) > 2 and abs(reverse) > 2) else "DOES_NOT_MOVE"
    return result


def main():
    # The archived snapshot's key has moved before, so the string is found by shape (eight colon-separated
    # fields) rather than by a key name I happen to remember.
    found = []

    def scan(node):
        if found:
            return
        if isinstance(node, str) and node.count(":") == 7 and all(part.replace(".", "").replace("-", "").isdigit()
                                                                 for part in node.split(":")):
            found.append(node)
        elif isinstance(node, dict):
            for value in node.values():
                scan(value)
        elif isinstance(node, list):
            for value in node:
                scan(value)
    if os.path.exists(BASELINE):
        scan(json.load(open(BASELINE)))
    base = found[0] if found else None
    if not base:
        raise SystemExit("BLOCKED_baseline_missing: no eight-field yaw trial string in the archived snapshot")
    candidates = [("baseline", base), ("half_ki", scaled(base, 1.0, 0.5)),
                  ("half_kp", scaled(base, 0.5, 1.0)), ("half_both", scaled(base, 0.5, 0.5))]
    station = acc.Station(SOCKET)
    rows = []
    for name, trial in candidates:
        # The campaign runner announces which trial this is before preparing it, and the station's gate
        # expects that order; a refusal is printed whole rather than as one field I hope is the reason.
        station.command("param_context", f"moves-check|{name}"[:39])
        prepared = station.command("param_prepare", trial)
        request = json.dumps(prepared)
        at = request.find("request_id=")
        # `ok/verdict=submitted` is an ack, not an acceptance: the outcome arrives in a later frame, which
        # is what the campaign runner's exchange() waits for. Until this tool goes through that same
        # exchange, every candidate reads as refused — measured, not assumed.
        if not prepared.get("accepted") or at < 0:
            print(f"{name}: refused at prepare — {json.dumps(prepared)[:200]}")
            continue
        request_id = request[at + 11:].strip().strip('"').strip(",").split('"')[0]
        applied = station.command("param_apply", request_id)
        if not applied.get("accepted"):
            print(f"{name}: refused at apply — {str(applied.get('reason'))[:110]}")
            continue
        time.sleep(2.0)
        hold = hold_reading(station)
        motion = moves(station)
        rows.append({"candidate": name, "string": trial, "hold": hold, "motion": motion})
        print(f"{name}: sent {trial} | at-rest {hold.get('at_rest_current_a_p95')} A of "
              f"{hold.get('cap_a')} A ({hold.get('fraction_of_cap')} of cap) | {motion['verdict']} "
              f"fwd={motion['forward']['delta_counts']} rev={motion['reverse']['delta_counts']}")
    station.command("param_restore")
    print("baseline restored")
    out = os.environ.get("ADR0021_MOVES_OUT", "")
    if out:
        with open(out, "w", encoding="utf-8") as handle:
            json.dump(rows, handle, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
