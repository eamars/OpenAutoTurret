#!/usr/bin/env python3
"""controld telemetry stream recorder — watch/record the §6.3 snapshot.

The web/log consumer's view of the daemon (§6.3, §42.1): connects to controld's
SOCK_SEQPACKET web socket, reads the published telemetry frames, and either
prints them human-readable or records a JSONL file for post-run analysis.

    python3 tools/telemetry_stream.py --seconds 30
    python3 tools/telemetry_stream.py --socket /run/ota/controld-web.sock \
        --jsonl build/probe/telemetry.jsonl --fields phase,payload_check_active
    python3 tools/telemetry_stream.py --changes-only     # print only transitions

SAFETY: read-only. It never sends a command, never opens CAN, never touches the
motor driver — the same constraint webd works under (§5.3).

Socket path precedence: --socket > $OTA_WEB_SOCKET > /run/ota/controld-web.sock
(the daemon's default, control/src/web/web_server.hpp).
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

DEFAULT_SOCKET = "/run/ota/controld-web.sock"


def socket_path(cli: str | None) -> str:
    return cli or os.environ.get("OTA_WEB_SOCKET") or DEFAULT_SOCKET


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="record controld telemetry (§6.3)")
    p.add_argument("--socket", default=None, help="controld web socket path")
    p.add_argument("--seconds", type=float, default=10.0, help="record window")
    p.add_argument("--jsonl", default="", help="write one JSON object per line here")
    p.add_argument("--fields", default="", help="comma-separated keys to print")
    p.add_argument("--changes-only", action="store_true",
                   help="print only when one of --fields changes value")
    p.add_argument("--count-transitions", action="store_true",
                   help="at the end, report how many times each field changed")
    args = p.parse_args(argv)

    path = socket_path(args.socket)
    if not os.path.exists(path):
        print(f"error: no controld socket at {path} (daemon running?)", file=sys.stderr)
        return 2
    fields = [f.strip() for f in args.fields.split(",") if f.strip()]

    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.connect(path)
    s.settimeout(2.0)
    out = open(args.jsonl, "w") if args.jsonl else None

    deadline = time.monotonic() + args.seconds
    prev: dict = {}
    changes: dict = {}
    n = 0
    try:
        while time.monotonic() < deadline:
            try:
                raw = s.recv(65536)
            except socket.timeout:
                continue
            if not raw:
                break
            try:
                d = json.loads(raw.decode(errors="replace"))
            except json.JSONDecodeError:
                continue
            if d.get("type") != "telemetry":
                continue
            n += 1
            if out:
                out.write(json.dumps(d) + "\n")
            shown = fields or sorted(k for k in d if k != "type")
            if args.changes_only:
                diff = {k: d.get(k) for k in shown if prev.get(k) != d.get(k)}
                if diff:
                    print(f"t={d.get('ts_ns', 0) / 1e9:12.3f} " +
                          " ".join(f"{k}={diff[k]}" for k in diff))
                for k in shown:
                    if prev.get(k) != d.get(k):
                        changes[k] = changes.get(k, 0) + 1
                prev = {k: d.get(k) for k in shown}
            else:
                print(f"t={d.get('ts_ns', 0) / 1e9:12.3f} " +
                      " ".join(f"{k}={d.get(k)}" for k in shown))
    finally:
        if out:
            out.close()
        s.close()

    print(f"--- {n} telemetry frames from {path}"
          + (f" -> {args.jsonl}" if args.jsonl else ""), file=sys.stderr)
    if args.count_transitions:
        for k, c in sorted(changes.items()):
            print(f"    {k}: {c} transitions", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
