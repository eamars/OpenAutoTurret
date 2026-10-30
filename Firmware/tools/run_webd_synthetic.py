#!/usr/bin/env python3
"""The web page against a synthetic station: no controld, no CAN, no camera (owner, 2026-10-30: page
work is tested on a synthetic web, never the station's video).

A FakeControld publishes a still turret with a calibration record, and answers the commands the page
sends the way controld would for what it imitates: the operating modes (MODE), HOME / SHUTDOWN from
MENU, and the one-shot actuator (ADR-007, through web/webd/fake_actuator.py), with controld's
acknowledgement fields so refusals reach the page as they do on the station. Everything else is
accepted and does nothing. There is no video: the panes show their no-stream state.

    python3 tools/run_webd_synthetic.py [--port 8090] [--socket /tmp/ota-synthetic.sock]
        [--state shutdown|manual|roam] [--actuator ok|unconfigured|unavailable|none]

Runs where AF_UNIX SOCK_SEQPACKET exists (Linux, WSL), from Firmware/ with the test venv.
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from web.webd.app import WebdApp                                   # noqa: E402
from web.webd.fake_actuator import ACTUATOR_COMMANDS, FakeActuator  # noqa: E402
from web.webd.fake_controld import FakeControld                     # noqa: E402
from web.webd.protocol import ResponseMessage                       # noqa: E402

RECORD = {"id": 6, "name": "NO_LOAD", "unusable": "",
          "parts": {k: {"origin": "measured", "date": "2026-10-25", "run": "synthetic", "from": 0}
                    for k in ("accuracy", "pitch", "yaw")} | {
                    "extrinsic": {"origin": "inherited", "date": "", "run": "", "from": 1},
                    "bore": {"origin": "zeroed", "date": "2026-10-25", "run": "", "from": 0}},
          "bore": {"mode": "manual_depth", "range_m": 11,
                   "wide": {"trim_right_mrad": 0, "trim_up_mrad": 0, "right_mm": 0, "up_mm": 0, "forward_mm": 0,
                            "zeroed_m": 11, "zeroed_on": "2026-10-25"},
                   "detail": {"trim_right_mrad": 0, "trim_up_mrad": 0, "right_mm": 0, "up_mm": 0, "forward_mm": 0,
                              "zeroed_m": 11, "zeroed_on": "2026-10-25"}},
          "last_run": None}
STATES = {  # (phase, operating_mode, supervisory_state)
    "shutdown": ("idle", "MANUAL", "UNHOMED"),
    "manual": ("hold", "MANUAL", "HOMED"),
    "roam": ("hold", "AUTO_ROAM", "HOMED"),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--socket", default="/tmp/ota-synthetic.sock")
    ap.add_argument("--state", choices=sorted(STATES), default="manual")
    ap.add_argument("--actuator", choices=("ok", "unconfigured", "unavailable", "none"), default="ok")
    args = ap.parse_args()

    fake = FakeControld(args.socket, telemetry_hz=15.0)
    lock = threading.Lock()
    station = {"state": args.state, "fault": False, "seq": 0}

    def publish_state() -> None:
        phase, mode, sup = STATES[station["state"]]
        if station["fault"]:
            phase, sup = "fault", "FAULT"
        fake.set_telemetry(phase=phase, operating_mode=mode, supervisory_state=sup, mode_phase="HOLD",
                           fault="synthetic fault" if station["fault"] else "",
                           at_ready=phase == "hold", track_state="ready_hold")

    act = None
    if args.actuator != "none":
        act = FakeActuator(rest_us=None if args.actuator == "unconfigured" else 1650,
                           active_us=None if args.actuator == "unconfigured" else 1450,
                           available=args.actuator != "unavailable",
                           reason="no PWM chip at 1f00098000: the pwm overlay is not loaded (dtoverlay=pwm,pin=15,func=4)"
                           if args.actuator == "unavailable" else "",
                           on_change=lambda status: fake.set_telemetry(actuator=status))
        fake.set_telemetry(actuator=act.status())

    def ack(command: str, accepted: bool, reason: str) -> None:
        station["seq"] += 1
        fake.set_telemetry(cmd_ack_command=command, cmd_ack_accepted=1 if accepted else 0, cmd_ack_reason=reason,
                           cmd_ack_seq=station["seq"])

    def handler(command: str, arg: str) -> ResponseMessage:
        with lock:
            if act and (command in ACTUATOR_COMMANDS or command == "stop_motion"):
                ok, why = act.handle(command, arg, faulted=station["fault"])
                if command != "stop_motion":
                    ack(command, ok, why)
            if command == "set_mode" and arg in ("MANUAL", "AUTO_ROAM", "SURVEILLANCE", "AUTO_TRACK"):
                if STATES[station["state"]][0] == "hold":
                    station["state"] = "manual" if arg == "MANUAL" else "roam"
            elif command in ("hold", "stop_motion") and STATES[station["state"]][0] == "hold":
                station["state"] = "manual"
            elif command == "start_homing":
                station["fault"] = False
                station["state"] = "roam"
            elif command in ("request_shutdown", "request_park"):
                station["state"] = "shutdown"
            elif command == "synthetic_fault":  # the page never sends it: for a person driving this by hand
                station["fault"] = arg != "off"
            publish_state()
        return ResponseMessage(command=command, ok=True, error="", verdict="submitted")

    fake.set_command_handler(handler)
    fake.set_telemetry(calibration_record={"id": RECORD["id"], "name": RECORD["name"]},
                       calibration_records=[RECORD], calibration_progress=1.0,
                       installation_calibrated=True, safety_action="ALLOW", feedback_age_ms=1,
                       telemetry_stale=False, cmd_ack_seq=0)
    publish_state()
    fake.start()

    def tick() -> None:
        while True:
            fake.set_telemetry(ts_ns=time.time_ns())
            time.sleep(0.05)
    threading.Thread(target=tick, daemon=True).start()

    os.environ["OTA_WEB_HOST"] = args.host
    os.environ["OTA_WEB_PORT"] = str(args.port)
    os.environ["OTA_WEB_SOCKET"] = args.socket
    os.environ["OTA_VIDEO_ENABLE"] = "0"
    os.environ.setdefault("OTA_VIDEO_STREAMS_MANIFEST", "/nonexistent/streams.json")
    print(f"[synthetic] state={args.state} actuator={args.actuator}; http://{args.host}:{args.port}/", flush=True)
    WebdApp().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
