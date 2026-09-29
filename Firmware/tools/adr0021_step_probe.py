"""What the station itself says during a commanded yaw step, row by row, in its own field names.

The plateau (0.308 degrees of a 5 degree command, flat for six seconds) rules out a slow ramp, and the
current (0.04-0.21 A against a 1.62 A envelope) rules out the current clamp. So something upstream of the
current loop is deciding how hard the axis may push, and the trace already carries that decision as
`output_reason` / `safety` / the tracking error. This dumps those rows rather than inferring them.

Run on the station, in Manual/Hold (commission mode), after `manual_step yaw+5`.
"""

from __future__ import annotations

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import adr0021_acceptance as acc  # noqa: E402
from adr0021_moves import COUNTS_PER_DEGREE, exchange  # noqa: E402

SOCKET = os.environ.get("ADR0021_SOCKET", "/tmp/ota-stack-1000/control-web.sock")
FIELDS = ("encoder_raw", "ref", "track", "effort", "omega", "pi_integral", "output_requested",
          "output_reason")


def rows_for_yaw(station):
    window = station.trace_window(want_rows=True)
    index = (window.get("axes") or ["pitch", "yaw"]).index("yaw")

    def pick(row, field):
        value = row.get(field)
        if isinstance(value, list):
            value = value[index] if len(value) > index else None
        return value

    return [{field: pick(row, field) for field in FIELDS if field in row} for row in (window.get("rows") or [])]


def main():
    station = acc.Station(SOCKET)
    frame = station.frame()
    if frame.get("phase") not in ("hold", "manual"):
        raise SystemExit(f"BLOCKED_phase_{frame.get('phase')}: a step needs Manual/Hold")

    before = rows_for_yaw(station)[-1:]
    reply = exchange(station, "manual_step", "yaw+5")
    print("step verdict:", {k: reply.get(k) for k in ("accepted", "reason", "error") if k in reply})
    time.sleep(2.5)                       # mid-motion: the plateau is reached in three to four seconds
    rows = rows_for_yaw(station)

    origin = before[0].get("encoder_raw") if before else None
    print(f"captured {len(rows)} rows; the step boundary is where the angle starts counting:")
    for row in rows[-16:]:
        value = row.get("encoder_raw")
        angle = None if origin is None or not isinstance(value, (int, float)) else \
            round((value - origin) / COUNTS_PER_DEGREE, 3)
        print("    angle=%s ref=%s track=%s effort=%s omega=%s integral=%s out=%s reason=%s" % (
            angle, row.get("ref"), row.get("track"), row.get("effort"), row.get("omega"),
            row.get("pi_integral"), row.get("output_requested"), row.get("output_reason")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
