"""Closed-loop session simulation through the native servo-sim (axis_control_core/simulate.cpp).

The simulator runs the same Servo / PositionLoop classes and session timing as
commissiond against a plant model (config/servo/*.json "plant"). Requests carry
the manifest's 50 Hz reference table, so a simulated run and a station run of the
same manifest are directly comparable.
"""
import json
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
SERVO_SIM = Path(os.environ.get("OTA_SERVO_SIM", REPO / "run/adr0022-local/firmware/axis_control_core/servo-sim"))


def reference_arrays(table):
    """Manifest reference_samples -> the simulator's column form."""
    return {"t": [r["time_s"] for r in table], "q": [r["position_rad"] for r in table],
            "v": [r["velocity_rad_s"] for r in table], "a": [r["acceleration_rad_s2"] for r in table]}


def run(request):
    """Run one simulation request; returns (columns dict, status string).

    The columns dict also carries "learned": the servo parameters at the end (yaw).
    """
    with tempfile.TemporaryDirectory(prefix="servo-sim-") as tmp:
        req, out = Path(tmp) / "request.json", Path(tmp) / "out.npy"
        req.write_text(json.dumps(request), encoding="utf-8")
        done = subprocess.run([str(SERVO_SIM), str(req), str(out)], capture_output=True, text=True)
        if not done.stdout.strip():
            raise RuntimeError("servo-sim produced no result: " + done.stderr[-500:])
        line = json.loads(done.stdout.strip().splitlines()[-1])
        if line["status"] == "INVALID":
            raise ValueError("servo-sim: " + line.get("detail", ""))
        data = np.load(out)
    result = {name: data[:, k] for k, name in enumerate(line["columns"])}
    result["learned"] = line.get("learned")
    return result, line["status"]


def yaw(servo, plant, table, start=0.0, excitation=None, gain_schedule=None, hold_after=1.5, speed_limit=1.75,
        oscillation_limit=0.3):
    request = {"axis": "yaw", "servo": servo, "plant": plant, "reference": reference_arrays(table),
               "start_position_rad": start, "hold_after_s": hold_after, "speed_limit_rad_s": speed_limit,
               "oscillation_limit_A": oscillation_limit}
    if excitation:
        request["excitation"] = excitation
    if gain_schedule:
        request["gain_schedule"] = gain_schedule
    return run(request)


def pitch(loop, plant, table, centre=0.0, excitation=None, gain_schedule=None, hold_after=1.0):
    request = {"axis": "pitch", "loop": loop, "plant": plant, "reference": reference_arrays(table),
               "start_position_rad": centre, "hold_after_s": hold_after}
    if excitation:
        request["excitation"] = excitation
    if gain_schedule:
        request["gain_schedule"] = gain_schedule
    return run(request)
