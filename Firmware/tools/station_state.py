#!/usr/bin/env python3
"""Print the station's state as the deploy tool restores it: ``homed`` or ``shutdown``.

Owner ruling 2026-10-03: the station is Homed or Shutdown, and a deploy returns it to the one it
found. Homed means a reachable controller with valid limits that is neither shut down nor faulted.
Anything else -- no stack running, idle, a fault, an unreadable answer -- is ``shutdown``, and the
web's HOME is the way up (it also recovers a fault). Standard library only; runs on the station.
"""
from __future__ import annotations

import json
import os
import sys
import urllib.request


def classify(state: dict) -> str:
    up = (state.get("controld_connected") and state.get("soft_limits_valid")
          and state.get("phase") not in ("idle", "fault"))
    return "homed" if up else "shutdown"


def main() -> int:
    run_dir = os.environ.get("OTA_RUN_DIR", f"/tmp/ota-stack-{os.getuid()}")
    try:
        with open(os.path.join(run_dir, "web.port"), encoding="utf-8") as handle:
            port = handle.read().strip()
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/state", timeout=5) as response:
            print(classify(json.load(response)))
    except (OSError, ValueError):
        print("shutdown")
    return 0


if __name__ == "__main__":
    sys.exit(main())
