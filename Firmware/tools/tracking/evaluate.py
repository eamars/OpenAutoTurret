"""Metrics of one tracking run (simulated or recorded) and the stage-1 checks of each scenario.

Every number is computed against an independent truth: in simulation the true target and the
true camera pose; on the station (stage 3) the recorded truth of the scenario. The filter's own
output is never its own reference. Error layers (ADR-003 sec. 9):
  framing   the true target's pixel at mid-exposure, from the framing pixel (what the photo shows)
  e_track   joint goal - reference      (Level 1: estimation, prediction, reference dynamics)
  e_servo   reference - axis            (Level 2: the ADR-002.2 servo)
  e_goal    true joint goal - joint goal (the estimator and the prediction)
"""
import math

import numpy as np

DEG = math.pi / 180


def peak_step(x):
    return float(np.max(np.abs(np.diff(x)))) if len(x) > 1 else 0.0


def window(x, t, a, b):
    k = (t >= a) & (t < b)
    return x[k]


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def peak(x):
    return float(np.max(np.abs(x))) if len(x) else float("nan")


def framing(f, a, b):
    """Framing error at exposure (px) of frames whose exposure mid falls in [a, b)."""
    ok = np.isfinite(f["err_px"])
    e = window(f["err_px"][ok], f["t_mid"][ok], a, b)
    blur = window(f["blur_px"][ok], f["t_mid"][ok], a, b)
    return {"framing_rms_px": rms(e), "framing_peak_px": peak(e), "blur_peak_px": peak(blur), "frames": int(len(e))}


def framing_axis(f, a, b, column):
    """One image axis of the framing error (err_u_px: horizontal, yaw; err_v_px: vertical, pitch): RMS and peak (px)."""
    ok = np.isfinite(f[column])
    e = window(f[column][ok], f["t_mid"][ok], a, b)
    return {"rms_px": rms(e), "peak_px": peak(e)}


def layers(t, a, b):
    out = {}
    for axis, s in (("yaw", "y"), ("pitch", "p")):
        tt = t["t"]
        valid = t["goal_valid"] > 0
        out[f"e_track_{axis}_rms_deg"] = rms(window((t[f"qt_{s}"] - t[f"qr_{s}"])[valid], tt[valid], a, b)) / DEG
        out[f"e_servo_{axis}_rms_deg"] = rms(window(t[f"qr_{s}"] - t[f"qtrue_{s}"], tt, a, b)) / DEG
        out[f"e_goal_{axis}_rms_deg"] = rms(window((t[f"qT_{s}"] - t[f"qt_{s}"])[valid], tt[valid], a, b)) / DEG
        out[f"e_total_{axis}_rms_deg"] = rms(window(t[f"qT_{s}"] - t[f"qtrue_{s}"], tt, a, b)) / DEG
    return out


def lag_s(t, a, b, rate_dps):
    """Moving lag along yaw: mean (true joint goal - axis) / target rate, in a steady window."""
    e = window(t["qT_y"] - t["qtrue_y"], t["t"], a, b)
    return float(np.mean(e) / (rate_dps * DEG)) if len(e) else float("nan")


def settle_s(f, t_event, threshold_px, end):
    """Time from an event until the framing error enters and stays within threshold_px."""
    ok = np.isfinite(f["err_px"]) & (f["t_mid"] >= t_event) & (f["t_mid"] < end)
    tm, e = f["t_mid"][ok], f["err_px"][ok]
    bad = np.where(e > threshold_px)[0]
    if not len(e):
        return float("nan")
    if not len(bad):
        return 0.0
    if bad[-1] == len(e) - 1:
        return float("inf")
    return float(tm[bad[-1] + 1] - t_event)


def settle_axes_s(f, t_event, threshold_u_px, threshold_v_px, end):
    """settle_s with a threshold per image axis (horizontal: yaw, vertical: pitch)."""
    ok = np.isfinite(f["err_u_px"]) & np.isfinite(f["err_v_px"]) & (f["t_mid"] >= t_event) & (f["t_mid"] < end)
    tm = f["t_mid"][ok]
    bad = np.where((np.abs(f["err_u_px"][ok]) > threshold_u_px) | (np.abs(f["err_v_px"][ok]) > threshold_v_px))[0]
    if not len(tm):
        return float("nan")
    if not len(bad):
        return 0.0
    if bad[-1] == len(tm) - 1:
        return float("inf")
    return float(tm[bad[-1] + 1] - t_event)


def consistency(t):
    """Largest departure of the published reference from its own integral (rad): must be ~0."""
    worst = 0.0
    for s in ("y", "p"):
        q, v, a, j = t[f"qr_{s}"], t[f"vr_{s}"], t[f"ar_{s}"], t[f"jr_{s}"]
        dt = np.diff(t["t"])
        pred = q[:-1] + v[:-1] * dt + a[:-1] * dt ** 2 / 2 + j[:-1] * dt ** 3 / 6
        # Segments that met a bound integrate piecewise; those are flagged and excluded.
        free = (t[f"flag_{s}"][:-1].astype(int) & 0b11) == 0
        if free.any():
            worst = max(worst, float(np.max(np.abs((pred - q[1:])[free]))))
    return worst


def bounds(t, limits):
    """Largest |v|, |a|, |j| of the reference relative to the Level-1 limits (<= 1 holds)."""
    out = 0.0
    for s, axis in (("y", "yaw"), ("p", "pitch")):
        c = limits[axis]
        out = max(out, peak(t[f"vr_{s}"]) / c["v_max_rad_s"], peak(t[f"ar_{s}"]) / c["a_max_rad_s2"],
                  peak(t[f"jr_{s}"]) / c["j_max_rad_s3"])
    return out


def sign_changes(x, weight, floor=0.05):
    """Sign changes of the feedforward while it is in use (weight above floor)."""
    s = np.sign(x[weight > floor])
    s = s[s != 0]
    return int(np.sum(s[1:] != s[:-1])) if len(s) > 1 else 0
