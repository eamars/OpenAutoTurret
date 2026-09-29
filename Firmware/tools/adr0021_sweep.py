"""Long-pulse proof that yaw moves, across the whole bearing, in degrees and not in counts.

Cross-roller bearings do not have uniform drag, so a 1° step proves very little: the owner asked for long
pulses (15/45/90°) and I agree. The obstacle is that the yaw encoder's unit is not stated anywhere in the
code — `gm6020::Feedback::angle_count` is a bare `uint16_t`, while pitch's CyberGear `MechPos` is
documented as a float mechanical angle. Rather than import a datasheet number silently, this measures the
scale on the machine against the station's own documented travel window (`turret_mixed.yaml:39`
`expected_travel_deg: -70..70`, 140° of sweep), and the sweep that produces the scale is also the harshest
long pulse there is: it drags the axis through every angle the software is allowed to visit.

Reported per segment: degrees travelled (using the measured scale), the current it cost as a fraction of
the drive's own cap, and whether any sample was stalled — no motion while the current sat at the cap.
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

SOCKET = os.environ.get("ADR0021_SOCKET", "/tmp/ota-stack-1000/control-web.sock")
AMPS_PER_RAW = 3.0 / 16384.0                     # gm6020_protocol.hpp:65-66, quoted not invented
DOCUMENTED_TRAVEL_DEG = 140.0                    # turret_mixed.yaml:39 yaw -70..+70
TARGETS = (15.0, 45.0, 90.0)
POLL_S = 0.5
STALL_FRACTION_OF_CAP = 0.9


def exchange(station, command, arg=""):
    before = station.seq()
    station.command(command, arg)
    return station.ack(before)


class YawEye:
    """Wrap-safe yaw angle, the current it is drawing, and whether it is moving."""

    def __init__(self, station):
        self.station = station
        self.last = None
        self.cumulative = 0.0

    def sample(self):
        window = self.station.trace_window(want_rows=True)
        axes = window.get("axes") or ["pitch", "yaw"]
        index = axes.index("yaw")
        rows = window.get("rows") or []
        angle = current = cap = speed = None
        for row in reversed(rows):
            def pick(field):
                value = row.get(field)
                if (isinstance(value, list) and len(value) > index
                        and isinstance(value[index], (int, float)) and not isinstance(value[index], bool)):
                    return float(value[index])
                return None
            if angle is None:
                angle = pick("encoder_raw")
            if current is None:
                value = pick("current_raw")
                current = None if value is None else abs(value * AMPS_PER_RAW)
            if cap is None:
                value = pick("current_cap")
                cap = None if not value or value <= 0 else value
            if speed is None:
                speed = pick("omega")
            if angle is not None and current is not None:
                break
        if angle is not None:
            if self.last is not None:
                step = ((angle - self.last + 32768.0) % 65536.0) - 32768.0   # uint16 wraps; travel must not
                self.cumulative += step
            self.last = angle
        fraction = None if current is None or not cap else current / cap
        return {"angle": angle, "cumulative": self.cumulative, "amps": current,
                "fraction_of_cap": fraction, "omega": speed}


def jog_to(eye, sign, target_deg, scale, seconds, log):
    """Jog until the requested sweep is banked, the segment runs out of time, or the axis stops answering."""
    start = eye.sample()["cumulative"]
    reply = exchange(eye.station, "manual_jog_start", "yaw" + sign)
    if not reply.get("accepted"):
        return {"status": "REFUSED", "reason": str(reply.get("reason", ""))[:120]}
    moved = 0.0
    stalled_samples = 0
    peak = 0.0
    deadline = time.time() + seconds
    while time.time() < deadline:
        sample = eye.sample()
        # The manual lease is the controller's business, not mine, but it is also not free: renew it while
        # a long segment is running rather than letting it lapse and calling the silence a limit.
        exchange(eye.station, "manual_jog_keepalive")
        moved = abs(sample["cumulative"] - start)
        if sample["fraction_of_cap"]:
            peak = max(peak, sample["fraction_of_cap"])
            if sample["fraction_of_cap"] >= STALL_FRACTION_OF_CAP and abs(sample["omega"] or 0.0) < 0.5:
                stalled_samples += 1
        if moved * scale >= target_deg:
            break
        time.sleep(POLL_S)
    exchange(eye.station, "manual_jog_stop")
    time.sleep(1.0)
    reached = moved * scale
    status = "REACHED" if reached >= target_deg * 0.95 else "LIMIT_OR_SHORT"
    log(f"    jog {sign}{target_deg:>4}°: moved {reached:5.1f}° "
        f"({moved:.0f} counts) peak {peak:.2f} of cap, stalled samples {stalled_samples} -> {status}")
    return {"status": status, "degrees": round(reached, 2), "counts": round(moved),
            "peak_fraction_of_cap": round(peak, 3), "stalled_samples": stalled_samples}


def main():
    log = lambda *a: print(*a, flush=True)
    station = acc.Station(SOCKET)
    frame = station.frame()
    if frame.get("phase") not in ("hold", "manual"):
        raise SystemExit(f"BLOCKED_phase_{frame.get('phase')}: the sweep starts from a stationary axis")
    eye = YawEye(station)
    eye.sample()

    log("calibrating the encoder scale by sweeping the documented travel window")
    positive = jog_to(eye, "+", 200.0, 1.0, 40.0, log)      # counts only; scale unknown yet
    negative = jog_to(eye, "-", 200.0, 1.0, 80.0, log)
    counts = abs(positive.get("counts", 0)) + abs(negative.get("counts", 0))
    if counts < 1000:
        raise SystemExit(f"BLOCKED_scale_unmeasurable: only {counts} counts of sweep, "
                         "too little to divide by the documented 140°")
    scale = counts / DOCUMENTED_TRAVEL_DEG
    log(f"  scale: {counts} counts over the documented {DOCUMENTED_TRAVEL_DEG}° => {scale:.2f} counts/deg")

    results = []
    for target in TARGETS:
        for sign in ("+", "-"):
            results.append(jog_to(eye, sign, target, scale, seconds=30.0, log))
    out = os.environ.get("ADR0021_SWEEP_OUT", "")
    if out:
        with open(out, "w", encoding="utf-8") as handle:
            json.dump({"scale_counts_per_deg": scale, "calibration": [positive] + [negative],
                       "segments": results}, handle, indent=2)
    stalled = [segment for segment in results if segment.get("stalled_samples")]
    log(f"segments with any stalled sample: {len(stalled)} of {len(results)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
