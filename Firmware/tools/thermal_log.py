#!/usr/bin/env python3
"""Thermal-equilibrium logger for the mixed station (experiment 2026-09-28).

One JSON line per sample: yaw raw temperature/current/speed straight off the
CAN bus (the truth, no firmware in the loop), pitch degrees parsed from the
controller heartbeat (the driver's own Celsius), station mode/phase from the
telemetry endpoint, and the Pi's CPU temperature as the ambient witness.

Run on the station. Stdlib only by intent: the experiment must outlive any
virtualenv mishap. --once is the self-test: one sample to stdout, exit 0.
"""
import argparse
import json
import re
import socket
import struct
import subprocess
import time
import urllib.request

CAN_RAW, CAN_RAW_FILTER = 3, 1
POLLIN = 1
import select


def yaw_sample(iface: str, timeout_s: float):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((iface, 0))
    s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FILTER, struct.pack("=II", 0x200, 0x7F0))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not select.select([s], [], [], 0.2)[0]:
            continue
        data = s.recv(16)
        can_id, dlc = struct.unpack_from("=IB3x", data)
        can_id &= socket.CAN_ERR_FLAG
        if can_id in (0x205, 0x206, 0x207, 0x208) and dlc == 8:
            body = data[8:16]
            speed = struct.unpack_from(">h", body, 2)[0]
            current = struct.unpack_from(">h", body, 4)[0]
            return {"frame_id": hex(can_id), "speed_rpm": speed,
                    "current_raw": current, "temp_raw": body[6]}
    return None


def pitch_and_mode(log_path: str):
    try:
        tail = subprocess.run(["tail", "-n", "200", log_path],
                              capture_output=True, text=True, timeout=5).stdout
        pitch = None
        for line in reversed(tail.splitlines()):
            m = re.search(r"temp_pitch=([0-9.]+)", line)
            if m:
                pitch = float(m.group(1))
                break
        return pitch
    except Exception:
        return None


def state(url: str):
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
            d = json.load(r)
        return {"phase": d.get("phase"), "mode": d.get("operating_mode"),
                "fault": d.get("fault")}
    except Exception:
        return None


def cpu_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


def sample(args):
    return {"t": time.time(), "iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "yaw": yaw_sample(args.iface, args.bus_timeout),
            "pitch_c": pitch_and_mode(args.log),
            "station": state(args.state_url), "pi_cpu_c": cpu_c()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="can0")
    p.add_argument("--log", default="/tmp/ota-stack-1000/controller.log")
    p.add_argument("--state-url", default="http://127.0.0.1:8080/api/state")
    p.add_argument("--interval", type=float, default=60.0)
    p.add_argument("--out", default="/tmp/ota-stack-1000/thermal_experiment.jsonl")
    p.add_argument("--once", action="store_true", help="self-test: one sample, stdout")
    a = p.parse_args()
    if a.once:
        print(json.dumps(sample(a), ensure_ascii=False))
        return
    while True:
        with open(a.out, "a") as f:
            f.write(json.dumps(sample(a), ensure_ascii=False) + "\n")
        time.sleep(a.interval)


if __name__ == "__main__":
    main()
