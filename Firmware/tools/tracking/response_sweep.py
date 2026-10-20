"""What limits the tracking response time today? (drive review 2026-10-19, item 2)

Closed-loop step responses through the production classes in `tracking-sim` (estimator, Level 1,
ADR-002.2 yaw servo on the identified yaw plant, pitch position loop on the identified pitch
plant), with the camera set to what the station measures today, while one thing at a time is
varied: Level 1's lambda, the yaw servo bandwidth (gains re-derived with the commissioning rule
kq = J wn^2, kv = J wn, ki = 1.5 kq), an ideal actuator (Level 1 alone), the camera latency, and
the pitch dead band. A stationary target starts `offset` degrees off the boresight; the metrics
are what item 0 measures on the station, so the two can be laid side by side.

    response_sweep.py [--out SUMMARY.json]

Run from this directory with the servo-commission venv in WSL (needs the native tracking-sim).
"""
import argparse
import copy
import json
import math

import numpy as np

import scenarios
import sim

DEG = math.pi / 180
# The station as measured on 2026-10-19 from the production trace: 30 fps, auto-exposure ~30 ms,
# optical time to use at the 200 Hz tick 46 ms p50 (so ~43.5 ms to arrival), 1 px box noise.
CAMERA = {"frame_period_s": 1 / 30.0159, "exposure_s": 0.030, "latency_s": 0.0435, "latency_jitter_s": 0.004,
          "frame_jitter_s": 0.001, "pixel_noise_px": 1.0, "true_timing_offset_s": 0.0, "seed": 11, "first_frame_s": 0.02}
# Pitch travel in the homed-low frame the extrinsic is written in (2026-10-17 homing: span 1.398 rad,
# 5 deg soft margins), and the ready pose at its centre.
PITCH_LOW, PITCH_HIGH = 5 * DEG, 1.398 - 5 * DEG
PITCH_CENTRE = 0.5 * (PITCH_LOW + PITCH_HIGH)


def gains(inertia, wn, zeta=0.5, integral_rate=1.5):
    kq = inertia * wn * wn
    return {"kq": kq, "kv": 2 * zeta * inertia * wn, "ki": integral_rate * kq}


def step_request(params, offset_deg, axis, yaw_asset, pitch_asset, actuator="servo", camera=None, duration=6.0):
    r_pc = sim.extrinsics()
    az0, el0 = scenarios.boresight(0.0, PITCH_CENTRE, r_pc)
    cam = dict(CAMERA, **(camera or {}))
    d_az = offset_deg * DEG if axis == "yaw" else 0.0
    d_el = offset_deg * DEG if axis == "pitch" else 0.0
    return {"tracker": params,
            "geometry": {"intrinsics": sim.intrinsics(), "R_PC": r_pc, "sight": [0, 0, 1],
                         "travel": {"pitch": [PITCH_LOW, PITCH_HIGH]}},
            "camera": cam,
            "target": {"start": {"az": az0 + d_az, "el": el0 + d_el, "rate": [0.0, 0.0], "identity": 1},
                       "segments": [{"duration": duration}]},
            "turret": {"start": [0.0, PITCH_CENTRE]}, "actuator": actuator, "yaw": yaw_asset, "pitch": pitch_asset,
            "duration_s": duration, "control_period_s": params["level1"]["period_s"]}


def metrics(tick, axis):
    k = 0 if axis == "yaw" else 1
    t = tick["t"]
    q_true = tick["qtrue_y"] if k == 0 else tick["qtrue_p"]
    q_goal = tick["qT_y"] if k == 0 else tick["qT_p"]
    e = (q_goal - q_true) / DEG                     # true pointing error, deg
    delta = e[0]
    s = math.copysign(1.0, delta)
    ea = e * s                                      # positive = still short of the target
    acc = tick["accepted"]
    t_obs = t[np.argmax(acc > 0)] if np.any(acc > 0) else float("nan")
    moved = np.abs(q_true - q_true[0]) > 0.05 * DEG
    t_move = t[np.argmax(moved)] if np.any(moved) else float("nan")
    def cross(level):
        idx = np.argmax(ea < level) if np.any(ea < level) else -1
        return t[idx] if idx >= 0 else float("nan")
    t10, t90 = cross(0.9 * abs(delta)), cross(0.1 * abs(delta))
    def settle(band):
        out = np.nonzero(np.abs(e) > band)[0]
        return t[out[-1] + 1] if len(out) and out[-1] + 1 < len(t) else (0.0 if not len(out) else float("nan"))
    tail = t >= t[-1] - 1.0
    return {"delta_deg": delta, "t_first_obs": t_obs, "t_first_motion": t_move, "rise_10_90": t90 - t10,
            "t90": t90, "settle_0p5": settle(0.5), "settle_0p2": settle(0.2), "overshoot_deg": float(max(0.0, -ea.min())),
            "steady_deg": float(e[tail].mean()), "jitter_deg": float(e[tail].std()),
            "peak_v_dps": float(np.max(np.abs(np.gradient(q_true, t))) / DEG)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    yaw, pitch, _, _ = sim.servo_assets()
    base = sim.tracking_parameters("tracking.json")
    base["level1"]["pitch"]["q_min_rad"], base["level1"]["pitch"]["q_max_rad"] = PITCH_LOW, PITCH_HIGH
    inertia = yaw["servo"]["inertia"]
    r_pc = sim.extrinsics()
    az0, el0 = scenarios.boresight(0.0, PITCH_CENTRE, r_pc)
    print(f"boresight at the ready pose: el {el0 / DEG:+.1f} deg; yaw wn deployed 27.5 (kq {yaw['servo']['kq']})")

    cases = []
    offsets = [5, 10, 20, 30] if not args.quick else [10]
    # A. lambda, deployed servo.
    for lam in ([4, 6, 8, 12, 16] if not args.quick else [4, 8]):
        for off in offsets:
            p = copy.deepcopy(base); p["level1"]["yaw"]["lambda_rad_s"] = lam
            cases.append((f"yaw lam={lam} wn=27.5", "yaw", off, p, yaw, "servo", None))
    # B. servo bandwidth at lambda 4, and an ideal actuator.
    for wn in [44.7, 60.0]:
        y = copy.deepcopy(yaw); y["servo"].update(gains(inertia, wn))
        for off in offsets:
            cases.append((f"yaw lam=4 wn={wn}", "yaw", off, copy.deepcopy(base), y, "servo", None))
    for off in offsets:
        cases.append(("yaw lam=4 ideal actuator", "yaw", off, copy.deepcopy(base), yaw, "ideal", None))
    # C. lambda 8 with a stiffer servo: do they compound?
    y = copy.deepcopy(yaw); y["servo"].update(gains(inertia, 44.7))
    for lam in [8, 12]:
        p = copy.deepcopy(base); p["level1"]["yaw"]["lambda_rad_s"] = lam
        for off in offsets:
            cases.append((f"yaw lam={lam} wn=44.7", "yaw", off, p, y, "servo", None))
    # D. camera latency: none, and the old 9 fps / 60 ms stack.
    for off in offsets:
        cases.append(("yaw lam=4 no camera latency", "yaw", off, copy.deepcopy(base), yaw, "servo", {"latency_s": 0.0, "latency_jitter_s": 0.0}))
        cases.append(("yaw lam=4 9 fps 60 ms", "yaw", off, copy.deepcopy(base), yaw, "servo", {"frame_period_s": 0.111, "latency_s": 0.06}))
    # E. negative steps (the friction model is asymmetric).
    for off in [-10, -30]:
        cases.append(("yaw lam=4 wn=27.5", "yaw", off, copy.deepcopy(base), yaw, "servo", None))
    # F. pitch: deployed (lambda 2.5, 3 deg band, centring), band off, lambda 4.
    for off in [5, -5, 10]:
        cases.append(("pitch deployed", "pitch", off, copy.deepcopy(base), yaw, "servo", None))
        p = copy.deepcopy(base); p["level1"]["pitch"].update({"dead_band_rad": 0.0, "centre_band_rad": 0.0, "feedforward_gain": 1.0})
        cases.append(("pitch no dead band", "pitch", off, p, yaw, "servo", None))
        p = copy.deepcopy(p); p["level1"]["pitch"]["lambda_rad_s"] = 4.0
        cases.append(("pitch lam=4 no band", "pitch", off, p, yaw, "servo", None))
        p = copy.deepcopy(base); p["level1"]["pitch"]["lambda_rad_s"] = 4.0
        cases.append(("pitch lam=4 with band", "pitch", off, p, yaw, "servo", None))

    rows = []
    print(f"{'case':30} {'axis':5} {'off':>5} {'obs':>6} {'move':>6} {'rise':>6} {'t90':>6} {'s0.5':>6} {'s0.2':>6} {'over':>6} {'steady':>7} {'jit':>6} {'vpk':>6} status")
    for name, axis, off, p, y, act, cam in cases:
        req = step_request(p, off, axis, y, pitch, actuator=act, camera=cam)
        tick, frame, status, _ = sim.run(req)
        m = metrics(tick, axis)
        m.update(case=name, axis=axis, offset_deg=off, status=status)
        rows.append(m)
        print(f"{name:30} {axis:5} {off:5.0f} {m['t_first_obs']:6.3f} {m['t_first_motion']:6.3f} {m['rise_10_90']:6.2f} {m['t90']:6.2f} "
              f"{m['settle_0p5']:6.2f} {m['settle_0p2']:6.2f} {m['overshoot_deg']:6.2f} {m['steady_deg']:+7.3f} {m['jitter_deg']:6.3f} {m['peak_v_dps']:6.1f} {status}")
    if args.out:
        json.dump(rows, open(args.out, "w", encoding="utf-8"), indent=1)


if __name__ == "__main__":
    main()
