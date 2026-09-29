"""Who is writing into the control loop while the station is holding, and how hard.

The owner's report is physical, not statistical: after homing the station sits in AUTO and the motors are
stalled the whole time. Two hypotheses explain that and they look identical from outside — either two
sources are commanding the loop at once (hold plus calibration/contact), or one source is holding against
something with real current. The trace answers it, because every row carries the source of its own
output (`command_kind`, `output_reason`, `mode`) and the current it drew.

Current is reported in raw drive counts. No ampere conversion is invented here: the row also carries the
axis's own cap, so the useful quantity is the unit-free ratio of reading to cap, which is what "stalled"
would have to mean anyway.
"""

import json
import socket
import statistics
import sys
import time

SOCKET = "/tmp/ota-stack-1000/control-web.sock"
AXES = ["pitch", "yaw"]


def capture(want_seconds=6.0):
    """Ask the station for a control window after it has been running a moment."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.connect(SOCKET)
    sock.settimeout(10)
    latest = None
    deadline = time.time() + want_seconds
    while time.time() < deadline:
        try:
            sock.send(json.dumps({"type": "command", "command": "read_control_trace"}).encode())
        except OSError:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            sock.connect(SOCKET)
            sock.settimeout(10)
            continue
        for _ in range(10):
            chunk, _, flags, _ = sock.recvmsg(8 << 20, 0)
            if flags & socket.MSG_TRUNC:
                raise SystemExit("BLOCKED_trace_truncated: the window did not fit the buffer")
            frame = json.loads(chunk)
            if frame.get("type") == "control_trace":
                latest = frame
                break
        time.sleep(0.5)
    sock.close()
    if not latest or not latest.get("rows"):
        raise SystemExit("BLOCKED_trace_window_empty: no rows to reason about")
    return latest


def axis_value(row, field, index, default=None):
    value = row.get(field)
    if isinstance(value, list):
        return value[index] if len(value) > index else default
    return value


def distribution(rows, field, index):
    counts = {}
    for row in rows:
        key = json.dumps(axis_value(row, field, index))
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: -item[1]))


def quantiles(rows, field, index):
    values = [axis_value(row, field, index) for row in rows]
    values = [float(v) for v in values if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not values:
        return None
    ordered = sorted(values)
    pick = lambda q: ordered[min(len(ordered) - 1, int(q * len(ordered)))]
    return {"n": len(values), "p50": pick(0.5), "p95": pick(0.95), "max": ordered[-1],
            "mean": round(statistics.fmean(values), 3)}


def main():
    frame = capture()
    rows, axes = frame["rows"], frame.get("axes") or AXES
    print(f"window: {len(rows)} rows, axes={axes}, type={frame.get('type')}")
    for index, axis in enumerate(axes):
        print(f"\n--- {axis} ---")
        for field in ("command_kind", "output_reason", "mode", "phase", "safety", "enabled_state",
                      "param_state", "frozen"):
            counts = distribution(rows, field, index)
            if len(counts) == 1 and next(iter(counts)) == "null":
                continue                      # the axis simply had nothing to say about that field
            print(f"  {field}: {json.dumps(counts, sort_keys=False)[:200]}")
        current = quantiles(rows, "current_raw", index)
        cap = quantiles(rows, "current_cap", index)
        effort = quantiles(rows, "effort", index)
        speed = quantiles(rows, "omega", index)
        track = quantiles(rows, "track", index)
        for label, stats in (("current_raw", current), ("current_cap", cap), ("effort", effort),
                             ("omega", speed), ("track", track)):
            if stats:
                print(f"  {label}: {json.dumps(stats)}")
        if current and cap and cap.get("p50"):
            print(f"  current/cap at p50: {round(current['p50'] / cap['p50'], 3)}"
                  f"   at p95: {round(current['p95'] / cap['p50'], 3)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
