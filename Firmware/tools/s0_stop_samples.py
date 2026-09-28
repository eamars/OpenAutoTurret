#!/usr/bin/env python3
"""S0: stop from motion, and read back what the station wrote about it.

Two bins, because the station has two different verbs and 07 defines evidence per definition:

  halt      -- "stop_motion": a mode-level halt. The claim is only "the reference stopped", so no
               axis evidence is required by definition; what we check is that no readiness rule
               refused the request to stop.
  shutdown  -- "request_shutdown": the stop/park sequence, which owes per-axis evidence and a
               closing record. Each sample ends the process, so the harness restarts the stack
               and reads the rotated evidence file.

Printed as counts with denominators and failure cases, per 08 §4: successes alone prove nothing.
"""
import argparse, glob, json, os, subprocess, sys, time, urllib.request

RUN = os.environ.get("OTA_RUN_DIR", "/tmp/ota-stack-1000")
RELEASES = "/home/eamars/workspace/OpenAutoTurret/run/releases"


def post(payload):
    req = urllib.request.Request("http://127.0.0.1:8080/api/command",
                                 data=json.dumps(payload).encode(),
                                 headers={"content-type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except Exception as exc:                                   # noqa: BLE001
        return {"ok": False, "error": str(exc)}


def state():
    try:
        with urllib.request.urlopen("http://127.0.0.1:8080/api/state", timeout=5) as r:
            return json.loads(r.read())
    except Exception:                                          # noqa: BLE001
        return {}


def launch_script():
    for path in sorted(glob.glob(RELEASES + "/*/Firmware/scripts/run_application.sh"),
                       key=os.path.getmtime, reverse=True):
        return path
    raise SystemExit("no release launcher under " + RELEASES)


def newest_evidence(since):
    files = [f for f in glob.glob(RUN + "/logs-history/*/traces/stop-evidence.ndjson")
             if os.path.getmtime(f) >= since] + \
            [f for f in [RUN + "/traces/stop-evidence.ndjson"] if os.path.exists(f)]
    return max(files, key=os.path.getmtime) if files else None


def moving(seconds=8.0, threshold_deg_s=1.0):
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(0.2)
        s = state()
        speed = max(abs(s.get("v_yaw_rad_s") or 0.0), abs(s.get("v_pitch_rad_s") or 0.0)) * 57.2958
        if speed > threshold_deg_s:
            return speed
    return 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-samples", type=int, default=3)
    parser.add_argument("--halt-samples", type=int, default=3)
    parser.add_argument("--modes", default="auto_roam,auto_track,manual")
    args = parser.parse_args()
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    bins, failures = {}, []

    for index in range(args.halt_samples):                      # bin: halt, no evidence owed
        mode = modes[index % len(modes)]
        post({"command": "set_mode", "mode": mode})
        moving()
        reply = post({"command": "stop_motion"})
        stats = bins.setdefault("halt:" + mode, {"n": 0, "accepted": 0})
        stats["n"] += 1
        stats["accepted"] += 1 if reply.get("ok") else 0
        if not reply.get("ok"):
            failures.append({"bin": "halt", "mode": mode, "reply": reply})

    for index in range(args.evidence_samples):                  # bin: shutdown, evidence owed
        mode = modes[index % len(modes)]
        started = time.time() - 1
        post({"command": "set_mode", "mode": mode})
        speed = moving()
        post({"command": "request_shutdown"})
        time.sleep(4.0)
        subprocess.run(["bash", launch_script(), "start"], capture_output=True, text=True)
        # A restart is a new run: wait for the socket, then read what the last run left behind.
        for _ in range(60):
            time.sleep(1.0)
            if state().get("phase"):
                break
        path = newest_evidence(started)
        rows = [json.loads(l) for l in open(path)] if path else []
        joined = {r.get("stop_id") for r in rows if r.get("stop_id")}
        stats = bins.setdefault("shutdown:" + mode, {"n": 0, "evidence": 0, "moving": 0})
        stats["n"] += 1
        stats["moving"] += 1 if speed > 1.0 else 0
        stats["evidence"] += 1 if rows else 0
        if not rows or not joined:
            failures.append({"bin": "shutdown", "mode": mode, "moving_deg_s": round(speed, 2),
                             "file": path, "lines": len(rows)})
    print(json.dumps({"gate": "S0", "bins": bins, "failure_count": len(failures),
                      "failures": failures[:6]}, ensure_ascii=False, indent=1))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
