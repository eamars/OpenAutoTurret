#!/usr/bin/env python3
"""Run the OpenAutoTurret web UI (webd) against a *simulated* controld.

This lets the web service be inspected over the network WITHOUT the real
station — no CAN, no motor. It starts:

  * a ``FakeControld`` on a Unix socket (the controld stand-in) that publishes
    a realistic, slowly-panning tracking scenario (calibrated base, an active
    target, fresh feedback, ALLOW);
  * the webd (FastAPI/Uvicorn) bound to 0.0.0.0:<port>, reading that socket.

So the *telemetry* panels are simulated, but the *video* panel is REAL: its
on/off switch opens the physical IMX500 and streams an MJPEG preview (a separate
low-priority path, §42.3). When the switch is off the camera is released (no
CPU). If the camera is absent/busy the switch reports the error in the panel.

Commands clicked in the UI are acknowledged by the simulated controld
(``hold``/``stop_tracking`` pause the simulated tracking, ``start_tracking``
resumes it). They have NO hardware effect because no real controld/motor is
connected. This is a dev/inspection tool, not a control path.

Usage:  python3 tools/run_webd_inspection.py [--host 0.0.0.0] [--port 8080]
        [--socket /run/ota/controld-web.sock] [--hz 15]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

# Make the repo root importable regardless of the CWD we are started from.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from web.webd.app import WebdApp                 # noqa: E402
from web.webd.fake_controld import FakeControld  # noqa: E402
from web.webd.protocol import ResponseMessage    # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default=os.environ.get("OTA_WEB_HOST", "0.0.0.0"))
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("OTA_WEB_PORT", "8080")))
    ap.add_argument("--socket", default=os.environ.get(
        "OTA_WEB_SOCKET", "/run/ota/controld-web.sock"))
    ap.add_argument("--hz", type=int,
                    default=int(os.environ.get("OTA_WEB_HZ", "15")))
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.socket) or ".", exist_ok=True)

    # --- simulated scenario state ------------------------------------------
    state = {"mode": "tracking"}   # "tracking" | "hold"
    cur = {"y": 0.0, "p": 0.0}     # current yaw/pitch (eased)

    def handler(command: str, arg: str) -> ResponseMessage:
        if command in ("hold", "stop_tracking"):
            state["mode"] = "hold"
        elif command == "start_tracking":
            state["mode"] = "tracking"
        return ResponseMessage(command=command, ok=True, error="")

    fake = FakeControld(args.socket, telemetry_hz=float(args.hz))
    fake.set_command_handler(handler)
    fake.start()

    def sim() -> None:
        t0 = time.time()
        while True:
            t = time.time() - t0
            if state["mode"] == "tracking":
                az = 0.5 * math.sin(0.3 * t)          # target pans +/-28 deg
                el = 0.15 + 0.10 * math.sin(0.2 * t)  # elevation oscillates
                cur["y"] += (az - cur["y"]) * 0.20    # first-order follow
                cur["p"] += (el - cur["p"]) * 0.20
                qv = (az - cur["y"]) * 4.0            # ~velocity (from error)
                qvp = (el - cur["p"]) * 4.0
                conf = 0.85 + 0.08 * math.sin(1.5 * t) + 0.01 * math.sin(9.0 * t)
                ts, act = "tracking", True
            else:  # hold: ease to the ready pose, drop confidence
                az = el = 0.0
                cur["y"] += (az - cur["y"]) * 0.15
                cur["p"] += (el - cur["p"]) * 0.15
                qv = (az - cur["y"]) * 4.0
                qvp = (el - cur["p"]) * 4.0
                conf = 0.0
                ts, act = "ready_hold", False
            fake.set_telemetry(
                ts_ns=time.time_ns(),
                track_state=ts, tracking_active=act,
                target_confidence=max(0.0, min(1.0, conf)),
                q_yaw_rad=cur["y"], v_yaw_rad_s=qv, q_ref_yaw_rad=az,
                q_pitch_rad=cur["p"], v_pitch_rad_s=qvp, q_ref_pitch_rad=el,
                effort_yaw=0.3 * qv + 0.1 * math.sin(5.0 * t),
                effort_pitch=0.2 * qvp + 0.05 * math.sin(7.0 * t),
                target_az_world_rad=az, target_el_world_rad=el,
                base_roll_rad=0.01, base_pitch_rad=-0.02, base_yaw_rad=0.0,
                installation_calibrated=True,
                installation_source="visual_calibration",
                safety_action="ALLOW",
                feedback_age_ms=2,
                control_cycle_us=5000 + int(40 * math.sin(3.0 * t)),
            )
            time.sleep(0.05)

    threading.Thread(target=sim, name="sim-scenario", daemon=True).start()
    print(f"[inspection] FakeControld on {args.socket} @ {args.hz} Hz", flush=True)

    # --- run the web UI (blocking) -----------------------------------------
    os.environ["OTA_WEB_HOST"] = args.host
    os.environ["OTA_WEB_PORT"] = str(args.port)
    os.environ["OTA_WEB_SOCKET"] = args.socket
    os.environ["OTA_WEB_HZ"] = str(args.hz)
    # Video preview (real IMX500), a separate low-priority path (42.3).
    os.environ.setdefault("OTA_VIDEO_ENABLE", "1")
    os.environ.setdefault("OTA_VIDEO_WIDTH", "640")
    os.environ.setdefault("OTA_VIDEO_HEIGHT", "480")
    os.environ.setdefault("OTA_VIDEO_FPS", "15")
    os.environ.setdefault("OTA_VIDEO_QUALITY", "80")
    # This install: the IMX500 is mounted upside-down, so rotate_180 is
    # applied at capture (before processing / control) to both the preview
    # and the vision/control pipeline. The colour is neutral once the
    # BGRX byte order is decoded correctly, so no white-balance correction
    # is needed (OTA_VIDEO_WB stays at its default "off").
    os.environ.setdefault("OTA_VIDEO_ORIENTATION", "rotate_180")
    print(f"[inspection] webd listening on http://{args.host}:{args.port}",
          flush=True)
    WebdApp().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
