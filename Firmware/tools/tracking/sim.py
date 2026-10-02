"""The native ADR-003 tracking simulator (tracking_core/tracking_sim.cpp) and the station's
geometry, servo assets and tracking parameters it is run with."""
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parents[1]
REPO = FIRMWARE.parent
TRACKING_SIM = Path(os.environ.get("OTA_TRACKING_SIM", REPO / "run/adr0022-local/firmware/tracking_core/tracking-sim"))
DEG = math.pi / 180


def _key_values(path):
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def intrinsics():
    kv = _key_values(FIRMWARE / "calibration/camera_intrinsics.yaml")
    return {"fx": float(kv["fx"]), "fy": float(kv["fy"]), "cx": float(kv["cx"]), "cy": float(kv["cy"]),
            "width": int(kv["width"]), "height": int(kv["height"])}


def extrinsics():
    """R_PC (camera -> pitch frame), row-major, from calibration/camera_extrinsics.yaml."""
    lines = [l for l in (FIRMWARE / "calibration/camera_extrinsics.yaml").read_text(encoding="utf-8").splitlines()
             if l and not l.startswith("#")]
    rows = []
    for l in lines:
        l = l.split("=", 1)[1] if "=" in l else l
        rows.append([float(x) for x in l.split()])
    matrix = [x for r in rows[1:4] for x in r]  # rows[0] is t_P_C
    assert len(matrix) == 9
    return matrix


def servo_assets():
    yaw = json.load(open(FIRMWARE / "config/servo/yaw_servo.json", encoding="utf-8"))
    pitch = json.load(open(FIRMWARE / "config/servo/pitch_servo.json", encoding="utf-8"))
    t = pitch["servo_trial"]
    return ({"servo": yaw["servo_parameters"], "plant": yaw["plant"]},
            {"loop": {k: t[k] for k in ("kp_per_s", "ki_per_s2", "integral_clamp_rad_s", "speed_limit_rad_s")},
             "plant": pitch["plant"]},
            [t["window_min_rad"], t["window_max_rad"]], t["center_rad"])


def tracking_parameters(name="tracking_prior.json"):
    return json.load(open(FIRMWARE / "config/tracking" / name, encoding="utf-8"))


def run(request):
    """One simulation: returns (ticks dict, frames dict, status, applied parameters)."""
    with tempfile.TemporaryDirectory(prefix="tracking-sim-") as tmp:
        req, ticks, frames = Path(tmp) / "request.json", Path(tmp) / "ticks.npy", Path(tmp) / "frames.npy"
        req.write_text(json.dumps(request), encoding="utf-8")
        done = subprocess.run([str(TRACKING_SIM), str(req), str(ticks), str(frames)], capture_output=True, text=True)
        if not done.stdout.strip():
            raise RuntimeError("tracking-sim produced no result: " + done.stderr[-500:])
        line = json.loads(done.stdout.strip().splitlines()[-1])
        if line["status"] == "INVALID":
            raise ValueError("tracking-sim: " + line.get("detail", ""))
        t, f = np.load(ticks), np.load(frames)
    tick = {name: t[:, k] for k, name in enumerate(line["tick_columns"])}
    frame = {name: f[:, k] for k, name in enumerate(line["frame_columns"])} if f.size else {n: np.zeros(0) for n in line["frame_columns"]}
    return tick, frame, line["status"], line["parameters"]
