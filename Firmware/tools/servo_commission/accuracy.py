"""Yaw accuracy limits from measured tracking performance (owner ruling 2026-10-02).

ADR-003's photography spec had no framing budget. The owner ruled that the yaw accuracy
limits are calibrated from the real tracking performance of the feedforward + feedback
servo. This module holds the three fixed parts of that calibration, all decided before
the calibration data was taken:

* per-pass metrics: the worst value of each framing-relevant quantity over one use-case
  pass (the servo error q_ref - q_true, deg, with the measured crosstalk removed);
* the limit rule: limit = 1.25 x the worst pass among all calibration passes, rounded up
  to the metric's grid. 1.25 covers the bearing's measured friction drift (+-35% in a day)
  acting on the friction-dominated part of the error;
* the motor feedback-only variant used for ADR-003 sec. 7B (motor-model FF + FB against
  FB only, same gains, same references): every plant feedforward term set to zero.

The limits are a capability baseline for this mechanism: they gate later commissioning
runs (a new asset must do as well as the calibrated one) and they are the servo share of
the ADR-003 framing budget. A deliberate hardware change re-runs `commission.py calibrate`.
"""
import math

import numpy as np

DEG = math.pi / 180
MARGIN = 1.25

# name: (grid, unit, description). Angles are servo error, deg; speed jitter is deg/s of the
# 20 ms-averaged speed (what an exposure integrates: blur = fx * jitter * exposure).
METRICS = {
    "rest_error_deg": (0.01, "deg", "pointing error at rest: final error after every step, stop, long move and hold"),
    "overshoot_deg": (0.01, "deg", "largest overshoot past a step or long-move target"),
    "ramp_rms_slow_deg": (0.01, "deg", "error RMS while following 1-5 deg/s ramps"),
    "ramp_rms_fast_deg": (0.01, "deg", "error RMS while following 10-20 deg/s ramps and reversals"),
    "ramp_jitter_slow_deg": (0.01, "deg", "detrended P95-P5 of the error on steady 1-5 deg/s ramps"),
    "ramp_jitter_fast_deg": (0.01, "deg", "detrended P95-P5 of the error on steady 10-20 deg/s ramps"),
    "speed_jitter_dps": (0.01, "deg/s", "std of the 20 ms-averaged speed error on steady ramps (motion blur)"),
    "walker_rms_deg": (0.01, "deg", "error RMS on the 20 s multi-sine walking profile"),
    "moving_peak_deg": (0.01, "deg", "largest error while the reference moves (ramp starts, reversals, walker)"),
}


def _speed(label):
    try:
        return abs(float(label[4:]))
    except ValueError:
        return 10.0  # rev-a / rev-b: 10 deg/s reversal


def pass_metrics(rows):
    """Worst value per metric over one use-case pass (score.yaw rows)."""
    out = {k: 0.0 for k in METRICS}

    def worst(key, value):
        if value is not None and np.isfinite(value):
            out[key] = max(out[key], float(value))
    for r in rows:
        name = r["label"]
        if name.startswith(("step", "long", "stop")) or name in ("hold", "rev-stop", "final"):
            worst("rest_error_deg", abs(r.get("final_err", 0.0)))
        if name.startswith(("step", "long")):
            worst("overshoot_deg", r.get("overshoot_pct", 0.0) / 100 * abs(float(name[4:]) if name.startswith("step") else 90.0))
        if name.startswith("ramp") or (name.startswith("rev-") and name != "rev-stop"):
            fast = _speed(name) >= 10
            worst("ramp_rms_fast_deg" if fast else "ramp_rms_slow_deg", r["err_rms"])
            worst("moving_peak_deg", r["err_max"])
            if "pos_p95_5" in r:
                worst("ramp_jitter_fast_deg" if fast else "ramp_jitter_slow_deg", r["pos_p95_5"])
            if "v_jit" in r:
                worst("speed_jitter_dps", r["v_jit"])
        if name == "walker":
            worst("walker_rms_deg", r["err_rms"])
            worst("moving_peak_deg", r["err_max"])
    return out


def limits_from(passes):
    """The frozen rule: 1.25 x the worst calibration pass, rounded up to the metric grid."""
    limits = {}
    for key, (grid, _, _) in METRICS.items():
        worst = max(p[key] for p in passes)
        limits[key] = round(math.ceil(MARGIN * worst / grid - 1e-9) * grid, 4)
    return limits


def check(metrics, limits):
    """Failures of one pass against calibrated limits."""
    return [f"{k} {metrics[k]:.3f} > {limits[k]:.3f} {METRICS[k][1]}" for k in limits if k in metrics and metrics[k] > limits[k]]


def motor_feedback_only(servo):
    """The same servo with every plant feedforward term removed (ADR-003 sec. 7B, motor FB only).
    Gains, integral, stall recovery, crosstalk compensation and limits are unchanged."""
    s = dict(servo)
    for key in ("inertia", "coulomb_positive", "coulomb_negative", "stribeck_positive", "stribeck_negative",
                "viscous", "creep_drop", "load", "friction_correction_rate", "friction_learning_rate"):
        if key in s:
            s[key] = 0.0
    for key in ("friction_map_positive", "friction_map_negative"):
        if key in s:
            s[key] = [0.0] * len(s[key])
    return s


def pixels(limits, px_per_deg):
    """Angle limits in image pixels for the tracker frame (deg/s limits become px/s)."""
    return {k.replace("_deg", "_px").replace("_dps", "_px_s"): round(v * px_per_deg, 1) for k, v in limits.items()}


def compare(ff, fb):
    """Per metric (and stall recoveries per pass): FF+FB and FB-only means over the same angles, and the ratio."""
    out = {}
    for key in list(METRICS) + ["stalls"]:
        a = float(np.mean([p[key] for p in ff]))
        b = float(np.mean([p[key] for p in fb]))
        out[key] = {"ff_fb": round(a, 4), "fb_only": round(b, 4), "fb_over_ff": round(b / a, 2) if a > 0 else None}
    return out
