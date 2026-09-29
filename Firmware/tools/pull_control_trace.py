#!/usr/bin/env python3
"""Pull controld's per-cycle trace and show what it actually says.

The ring has existed for a long time and `read_control_trace` has answered for
about as long, yet **nothing in the stack ever asked**: the 2026-09-28 no-progress
trip left no per-cycle evidence behind, not because the rows weren't written but
because nobody read them inside the ~20 s before the ring wrapped. This is that
missing caller, and it deliberately stays a *reader*: it sends one read command,
never touches can0, and cannot ask for anything safety-relevant.

Transport is controld's own SOCK_SEQPACKET Unix socket (the same one webd uses),
so this works with no dependencies on the station and as whoever owns the run dir.

    Firmware/tools/pull_control_trace.py                       # summary of the live ring
    Firmware/tools/pull_control_trace.py --dump /tmp/trace.ndjson
    Firmware/tools/pull_control_trace.py --selftest            # offline, no station

`--dump` writes NDJSON with every absolute ns value as a **decimal string**:
docs/04_CONTRACTS.md §2 forbids letting a 64-bit ns timestamp become a JavaScript
Number, and a file that quietly breaks that rule is worse than no file, because
it looks analysable.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

# Same as the other tools in this directory: run as a script, the repo root is
# not on sys.path on its own, and ``common`` is where the shared reader lives.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.control_trace import (MAX_FRAME, TraceUnavailable,  # noqa: E402
                                  request_trace)

DEFAULT_SOCKET = "/tmp/ota-stack-1000/control-web.sock"

# The reader lives in ``common/control_trace.py``, shared with webd's
# ``/api/control_trace``: the frame-size floor and the "skip telemetry until the
# frame that says so" rule must not exist twice, because twice is how one copy
# starts returning a truncated megabyte while the other refuses to. MAX_FRAME is
# re-exported here so `--dump` users can still reason about the ceiling.


def summarise(reply: dict) -> tuple[str, list[dict]]:
    rows = reply.get("rows", [])
    phases = Counter(r.get("phase", "?") for r in rows)
    temps = [r["temp_raw"][1] for r in rows if "temp_raw" in r]
    span_ns = (rows[-1]["t"] - rows[0]["t"]) if len(rows) > 1 else 0
    # The flag belongs in the summary line: 256 live rows and 1024 frozen ones both
    # look like "a trace", and only one of them is the trip's own window.
    freeze = ""
    if reply.get("frozen"):
        freeze = f"FROZEN_AT={reply.get('frozen_t_ns')} "
    head = (
        f"{freeze}rows={len(rows)} span={span_ns / 1e9:.3f}s "
        f"phases={dict(phases)} "
        f"yaw_temp_raw={('none' if not temps else f'{min(temps)}..{max(temps)}')}"
    )
    return head, rows


def to_ndjson(rows: list[dict]) -> str:
    """ns and command_seq as decimal strings; everything else as-is."""
    out = []
    for r in rows:
        fixed = dict(r)
        for key in ("t", "ack", "rx"):
            if key in fixed:
                fixed[key] = [str(v) for v in fixed[key]] if isinstance(fixed[key], list) else str(fixed[key])
        out.append(json.dumps(fixed, separators=(",", ":")))
    return "\n".join(out) + ("\n" if out else "")


def selftest() -> int:
    synthetic = {
        "type": "control_trace",
        "axes": ["pitch", "yaw"],
        "rows": [
            {"t": 1_000_000_000_000_000_000, "ack": 18_446_744_073_709_551_615, "phase": "hold",
             "temp_raw": [-1, 28], "q": [0.1, -1.3353], "cmd": [0.0, 0.1745], "safety": 0,
             "period_us": 5000, "rx": [1, 2]},
            {"t": 1_000_000_005_000_000_000, "ack": 18_446_744_073_709_551_616, "phase": "hold",
             "temp_raw": [-1, 28], "q": [0.1, -1.3353], "cmd": [0.0, 0.1745], "safety": 0,
             "period_us": 5000, "rx": [1, 2]},
        ],
    }
    checks = []
    head, rows = summarise(synthetic)
    checks.append(("summary counts phases", "phases={'hold': 2}" in head))
    checks.append(("yaw byte is reported", "28..28" in head))
    ndjson = to_ndjson(rows)
    first = json.loads(ndjson.splitlines()[0])
    # 2^64-1 as a float loses its low digits; as a string it cannot.
    checks.append(("ns stays exact", first["t"] == "1000000000000000000"))
    checks.append(("command_seq stays exact", first["ack"] == "18446744073709551615"))
    checks.append(("per-axis ns stays exact", first["rx"] == ["1", "2"]))
    checks.append(("no row is dropped", len(ndjson.strip().splitlines()) == 2))
    frozen_head, _ = summarise({**synthetic, "frozen": True, "frozen_t_ns": "123"})
    checks.append(("a frozen window announces itself", frozen_head.startswith("FROZEN_AT=123")))
    for name, ok in checks:
        print(f"[{'ok' if ok else 'FAIL'}] {name}")
    return 0 if all(ok for _, ok in checks) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--socket", default=DEFAULT_SOCKET)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--dump", help="write the rows as NDJSON to this path")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    try:
        reply = request_trace(args.socket, args.timeout)
    except TraceUnavailable as exc:
        # Every reason already names the socket it tried, which is the one thing
        # an operator on the station needs first.
        print(f"unreadable: {exc}", file=sys.stderr)
        return 2
    head, rows = summarise(reply)
    print(head)
    if rows:
        newest = rows[-1]
        print("newest:", json.dumps(newest, separators=(",", ":")))
    if args.dump:
        Path(args.dump).write_text(to_ndjson(rows))
        print(f"wrote {len(rows)} rows -> {args.dump}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
