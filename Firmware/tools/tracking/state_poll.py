"""Record production's /api/state at a fixed rate on the station (ADR-003 3b evidence).

Runs on the station beside the live stack, reads only, and writes one JSON line per sample with
the station's monotonic receive time. Stops at SIGTERM/SIGINT or after `seconds`.

    state_poll.py OUT.jsonl SECONDS [RATE_HZ]
"""
import json
import os
import signal
import sys
import time
import urllib.request

out, seconds = sys.argv[1], float(sys.argv[2])
rate = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
run_dir = f"/tmp/ota-stack-{os.getuid()}"
port = open(os.path.join(run_dir, "web.port"), encoding="utf-8").read().strip()
url = f"http://127.0.0.1:{port}/api/state"
stop = False


def done(_signum, _frame):
    global stop
    stop = True


signal.signal(signal.SIGTERM, done)
signal.signal(signal.SIGINT, done)
end = time.monotonic() + seconds
period = 1.0 / rate
next_t = time.monotonic()
with open(out, "a", encoding="utf-8") as f:
    while not stop and time.monotonic() < end:
        next_t += period
        try:
            with urllib.request.urlopen(url, timeout=0.5) as r:
                state = json.loads(r.read())
            f.write(json.dumps({"rx_ns": time.monotonic_ns(), "state": state}) + "\n")
        except Exception as error:  # noqa: BLE001 - record the gap, keep polling
            f.write(json.dumps({"rx_ns": time.monotonic_ns(), "error": str(error)}) + "\n")
        f.flush()
        time.sleep(max(0.0, next_t - time.monotonic()))
