"""ADR-003's 14 mandatory scenarios (docs/ADR-003/docs/03_STAGES_AND_ACCEPTANCE.md sec. 6) as
simulator requests, and the request builder shared with stage 2's lambda solve.

Targets are base-frame LOS trajectories with piecewise-constant angular acceleration, written
relative to where the camera looks at the start (the commissioned pitch centre, yaw 0). The
camera defaults to what production delivered on 2026-09-29 (about 9 fps, 60 ms capture to
control, 3 px anchor noise); stage 2 replaces these with measured values.
"""
import copy
import math

import numpy as np

import sim

DEG = math.pi / 180

CAMERA = {"frame_period_s": 0.111, "exposure_s": 0.01, "latency_s": 0.06, "latency_jitter_s": 0.01,
          "frame_jitter_s": 0.002, "pixel_noise_px": 3.0, "true_timing_offset_s": 0.0, "seed": 7}


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def boresight(yaw, pitch, r_pc):
    """Base-frame LOS (az, el) of the optical axis at a joint pose."""
    r = _rot_z(yaw) @ _rot_y(pitch) @ np.array(r_pc).reshape(3, 3) @ np.array([0.0, 0.0, 1.0])
    return math.atan2(r[1], r[0]), math.atan2(r[2], math.hypot(r[0], r[1]))


def request(params, segments, start_offset_deg=(0.0, 0.0), start_rate_dps=(0.0, 0.0), duration=10.0,
            camera=None, actuator="servo", **injections):
    """segments: list of dicts in degrees: {"duration", "accel": [az, el] deg/s^2, "rate": [..] deg/s,
    "jump": [..] deg, "identity": n}. Injections: drops, extra_delay ([t0, t1, s]), velocity_unavailable,
    saturate ([t0, t1, yaw A or pitch rad/s])."""
    yaw, pitch, window, centre = sim.servo_assets()
    r_pc = sim.extrinsics()
    az0, el0 = boresight(0.0, centre, r_pc)
    segs = []
    for s in segments:
        g = {"duration": s["duration"]}
        for key in ("accel", "rate", "jump"):
            if key in s:
                g[key] = [s[key][0] * DEG, s[key][1] * DEG]
        if "identity" in s:
            g["identity"] = s["identity"]
        segs.append(g)
    cam = dict(CAMERA, **(camera or {}))
    for key in ("drops", "extra_delay"):
        if key in injections:
            cam[key] = injections.pop(key)
    req = {"tracker": params,
           "geometry": {"intrinsics": sim.intrinsics(), "R_PC": r_pc, "sight": [0, 0, 1], "travel": {"pitch": window}},
           "camera": cam,
           "target": {"start": {"az": az0 + start_offset_deg[0] * DEG, "el": el0 + start_offset_deg[1] * DEG,
                                "rate": [start_rate_dps[0] * DEG, start_rate_dps[1] * DEG], "identity": 1},
                      "segments": segs},
           "turret": {"start": [0.0, centre]}, "actuator": actuator, "yaw": yaw, "pitch": pitch,
           "duration_s": duration, "control_period_s": params["level1"]["period_s"]}
    req.update(injections)
    return req


# Each scenario: the request arguments and the windows its checks use (seconds).
SCENARIOS = {
    "T01": dict(name="stationary acquisition", segments=[{"duration": 8.0}], start_offset_deg=(8.0, 4.0), duration=8.0),
    "T02": dict(name="stationary hold", segments=[{"duration": 15.0}], start_offset_deg=(0.3, 0.2), duration=15.0),
    "T03": dict(name="slow constant rate", segments=[{"duration": 20.0}], start_rate_dps=(2.0, 0.0), duration=20.0),
    "T04": dict(name="fast constant rate", segments=[{"duration": 8.0}], start_offset_deg=(-12.0, -4.0),
                start_rate_dps=(20.0, 4.0), duration=8.0),
    "T05": dict(name="accelerating", segments=[{"duration": 1.0}, {"duration": 1.0, "accel": [20.0, 4.0]}, {"duration": 4.0}],
                start_offset_deg=(-10.0, -3.0), duration=6.0),
    "T06": dict(name="decelerating", segments=[{"duration": 2.0}, {"duration": 1.0, "accel": [-20.0, -4.0]}, {"duration": 4.0}],
                start_offset_deg=(-15.0, -3.0), start_rate_dps=(20.0, 4.0), duration=7.0),
    "T07": dict(name="sudden stop", segments=[{"duration": 3.0}, {"duration": 4.0, "rate": [0.0, 0.0]}],
                start_offset_deg=(-15.0, 0.0), start_rate_dps=(20.0, 0.0), duration=7.0),
    "T08": dict(name="direction reversal", segments=[{"duration": 2.0}, {"duration": 1.0, "accel": [-30.0, -6.0]}, {"duration": 3.0}],
                start_offset_deg=(-10.0, -3.0), start_rate_dps=(15.0, 3.0), duration=6.0),
    "T09": dict(name="near zero with vision noise", segments=[{"duration": 15.0}], start_rate_dps=(0.1, 0.05),
                camera={"pixel_noise_px": 6.0}, duration=15.0),
    "T10": dict(name="velocity temporarily unavailable", segments=[{"duration": 12.0}], start_offset_deg=(-20.0, 0.0),
                start_rate_dps=(8.0, 0.0), duration=12.0, velocity_unavailable=[[4.0, 7.0]]),
    "T11": dict(name="delayed observations", segments=[{"duration": 12.0}], start_offset_deg=(-20.0, 0.0),
                start_rate_dps=(8.0, 0.0), duration=12.0, extra_delay=[[4.0, 8.0, 0.15]]),
    "T12": dict(name="irregular arrival and dropped frames", segments=[{"duration": 10.0}], start_offset_deg=(-15.0, -3.0),
                start_rate_dps=(8.0, 2.0), duration=10.0, camera={"frame_jitter_s": 0.03, "latency_jitter_s": 0.03},
                drops=[[3.0, 3.4], [6.0, 6.25]]),
    "T13": dict(name="actuator saturation and release", segments=[{"duration": 9.0}], start_offset_deg=(-15.0, 0.0),
                start_rate_dps=(10.0, 0.0), duration=9.0, saturate=[[3.0, 5.0, 0.1]]),
    "T14": dict(name="loss and reacquisition", segments=[{"duration": 8.0}, {"duration": 5.0, "jump": [5.0, 1.0], "rate": [0.0, 0.0], "identity": 2}],
                start_offset_deg=(-10.0, 0.0), start_rate_dps=(6.0, 0.0), duration=13.0, drops=[[3.0, 5.5]]),
}


def build(sid, params, motion=True, actuator="servo", camera=None):
    s = copy.deepcopy(SCENARIOS[sid])
    s.pop("name")
    p = copy.deepcopy(params)
    p["target_motion"] = motion
    # The estimator knows its own measurement noise (stage 2 measures it): a scenario that
    # injects more pixel noise tells the tracker so, as a calibrated station would.
    noise = s.get("camera", {}).get("pixel_noise_px")
    if noise:
        p["pixel_sigma_px"] = noise
    if camera:
        s["camera"] = dict(s.get("camera", {}), **camera)
    return request(p, actuator=actuator, **s)
