"""Score a session against its use-case segments, and the fixed acceptance limits.

Yaw truth is the 1 kHz GM6020 encoder with the *measured* current crosstalk removed
(the asset's identified table, not the servo's biased copy: the bias is part of
the tracking error). Pitch truth is the CyberGear type-2 position. Per segment:
tracking error RMS/max (deg); ramps add speed ratio and jitter of 20 ms-averaged
speed and detrended position P95-P5; steps add overshoot and settling; holds and
steps report the final error.

    python score.py yaw   JOURNAL [--asset ../../config/servo/yaw_servo.json]
    python score.py pitch JOURNAL
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

import accuracy
import journal
import usecase

DEG = math.pi / 180
ACCURACY_FILE = Path(__file__).resolve().parents[2] / "config/servo/yaw_accuracy.json"

# Gates (fail the run, no asset is written): the servo really tracks (speed ratio, so a barely-moving
# axis cannot pass on low jitter) and stalls were recovered. Conformance (reported, never blocks the
# asset): the accuracy limits in config/servo/yaw_accuracy.json, calibrated by `commission.py
# calibrate` from the real FF+FB tracking performance (owner ruling 2026-10-02, accuracy.py). A
# servo that works on a changed plant must still be written -- keeping the old asset would be
# worse -- and a conformance shortfall is a capability finding: inspect the hardware, or after a
# deliberate change re-run the calibration. YAW_LIMITS are the design targets the simulator's bias
# choice normalises by (design.predicted_cost). Speeds in deg/s, angles in deg.
YAW_GATES = {"speed_ratio": (0.9, 1.1), "max_stalls": 4}
PITCH_GATES = {}
YAW_LIMITS = {"speed_ratio": (0.95, 1.05), "ramp_p95_5": 0.15, "ramp_p95_5_fast": 0.25, "step_final": 0.25,
              "step_overshoot_deg": 0.1, "walker_rms": 0.2, "long_final": 0.35, "max_stalls": 2}
PITCH_LIMITS = {"ramp_rms": 0.1, "walker_rms": 0.1, "step_final": 0.05}


def segments(manifest):
    return [(s["begin_s"], s["end_s"], s["kind"], s["value"], s["label"]) for s in manifest["reference_segments_labels"]]


def yaw_result(j, table, delay):
    c = j["cycles"]
    f = j["feedback"]
    q = journal.true_position(j, table, delay) if table is not None else f["q"]
    tu = np.arange(max(0, f["t"][0]), f["t"][-1], 0.001)
    qu = np.interp(tu, f["t"], q)
    return {"t": c["t"], "qr": c["qr"], "vr": c["vr"], "q": c["q"], "v": c["v"], "u": c["u"],
            "true_t": tu, "true_q": qu, "true_v": savgol_filter(qu, 21, 2, deriv=1, delta=0.001)}


UNSCORED = ("goto",)  # a calibration's positioning move: not part of the use-case pass


def scored_stalls(j):
    """Stall recoveries that began inside the scored script."""
    c = j["cycles"]
    if not len(c["stalls"]):
        return 0
    onset = c["t"][np.where(np.diff(c["stalls"]) > 0)[0] + 1] - c["t"][0]
    skip = [(s["begin_s"], s["end_s"]) for s in j["manifest"]["reference_segments_labels"] if s["label"] in UNSCORED]
    return int(sum(1 for t in onset if not any(a <= t < b for a, b in skip)))


def yaw(j, table, delay):
    rows = usecase.metrics(yaw_result(j, table, delay), segments(j["manifest"]))
    total = int(j["cycles"]["stalls"][-1]) if len(j["cycles"]["stalls"]) else 0
    return rows, {"stalls": scored_stalls(j), "stalls_total": total, "peak_current": float(np.max(np.abs(j["cycles"]["u"])))}


def pitch(j):
    tr = j["trial"]
    rows = []
    for a, b, kind, val, label in segments(j["manifest"]):
        k = (tr["t"] >= a) & (tr["t"] < b)
        if k.sum() < 10:
            continue
        e = (tr["qr"][k] - tr["q"][k]) / DEG
        row = {"label": label, "err_rms": float(np.sqrt(np.mean(e ** 2))), "err_max": float(np.max(np.abs(e)))}
        if kind in ("hold", "step"):
            row["final_err"] = float(np.mean(e[-200:]))
        rows.append(row)
    return rows, {}


def calibrated_limits():
    """The measured accuracy limits, or None before the first calibration."""
    if not ACCURACY_FILE.exists():
        return None
    return json.load(open(ACCURACY_FILE, encoding="utf-8"))["limits"]


def tracking_fails(rows):
    """The axis really tracks: every steady ramp's speed within YAW_GATES of the reference."""
    lo, hi = YAW_GATES["speed_ratio"]
    return [f"{r['label']}: speed ratio {r['v_ratio']:.3f} (not tracking)" for r in rows
            if r["label"].startswith("ramp") and "v_ratio" in r and not lo <= r["v_ratio"] <= hi]


def gate_yaw(rows, extra):
    """Hard gates: the run fails on these."""
    fails = tracking_fails(rows)
    if extra.get("stalls", 0) > YAW_GATES["max_stalls"]:
        fails.append(f"{extra['stalls']} stalls")
    return fails


def conformance_yaw(rows):
    """Exceedances of the calibrated accuracy limits (empty before the first calibration)."""
    limits = calibrated_limits()
    return accuracy.check(accuracy.pass_metrics(rows), limits) if limits else []


def gate_pitch(rows, extra):
    return []


def accept_yaw(rows, extra):
    """Advisory: the provisional accuracy targets (YAW_LIMITS)."""
    fails = []
    for r in rows:
        name = r["label"]
        if name.startswith("ramp") and "v_ratio" in r:
            lo, hi = YAW_LIMITS["speed_ratio"]
            if not lo <= r["v_ratio"] <= hi:
                fails.append(f"{name}: speed ratio {r['v_ratio']:.3f}")
            speed = abs(float(name[4:]))
            limit = YAW_LIMITS["ramp_p95_5_fast"] if speed >= 20 else YAW_LIMITS["ramp_p95_5"]
            if r.get("pos_p95_5", 0) > limit:
                fails.append(f"{name}: P95-P5 {r['pos_p95_5']:.3f} deg")
        if name.startswith("step") and abs(float(name[4:])) >= 0.5:
            if abs(r.get("final_err", 0)) > YAW_LIMITS["step_final"]:
                fails.append(f"{name}: final error {r['final_err']:.3f} deg")
            overshoot = r.get("overshoot_pct", 0) / 100 * abs(float(name[4:]))
            if overshoot > YAW_LIMITS["step_overshoot_deg"]:
                fails.append(f"{name}: overshoot {overshoot:.3f} deg")
        if name.startswith("long") and abs(r.get("final_err", 0)) > YAW_LIMITS["long_final"]:
            fails.append(f"{name}: final error {r['final_err']:.3f} deg")
        if name == "walker" and r["err_rms"] > YAW_LIMITS["walker_rms"]:
            fails.append(f"walker: error RMS {r['err_rms']:.3f} deg")
    if extra.get("stalls", 0) > YAW_LIMITS["max_stalls"]:
        fails.append(f"{extra['stalls']} stalls")
    return fails


def accept_pitch(rows, extra):
    fails = []
    for r in rows:
        name = r["label"]
        if name.startswith("ramp") and r["err_rms"] > PITCH_LIMITS["ramp_rms"]:
            fails.append(f"{name}: error RMS {r['err_rms']:.3f} deg")
        if name == "walker" and r["err_rms"] > PITCH_LIMITS["walker_rms"]:
            fails.append(f"walker: error RMS {r['err_rms']:.3f} deg")
        if name.startswith("step") and abs(r.get("final_err", 0)) > PITCH_LIMITS["step_final"]:
            fails.append(f"{name}: final error {r['final_err']:.3f} deg")
    return fails


def summary(rows):
    """The few numbers a person compares between runs."""
    pick = lambda prefix, key: [r[key] for r in rows if r["label"].startswith(prefix) and key in r]
    out = {}
    for prefix, key, name in (("ramp", "v_ratio", "speed_ratio"), ("ramp", "pos_p95_5", "ramp_p95_5_deg"),
                              ("ramp", "err_rms", "ramp_rms_deg"), ("step", "final_err", "step_final_deg"),
                              ("long", "final_err", "long_final_deg"), ("walker", "err_rms", "walker_rms_deg")):
        v = pick(prefix, key)
        if v:
            out[name] = [round(float(min(v)), 3), round(float(max(v)), 3)] if len(v) > 1 else round(float(v[0]), 3)
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("axis", choices=("yaw", "pitch")); p.add_argument("journal")
    p.add_argument("--asset", help="yaw asset with identified.crosstalk (default: the servo's own table + recorded bias)")
    a = p.parse_args()
    if a.axis == "yaw":
        j = journal.yaw(a.journal)
        sp = j["manifest"]["servo_parameters"]
        if a.asset:
            x = json.load(open(a.asset, encoding="utf-8"))["identified"]["crosstalk"]
            table, delay = x["table"], x["delay_s"]
        else:
            bias = j["manifest"].get("servo_crosstalk_bias_rad_per_A", 0.0)
            table, delay = [g + bias for g in sp.get("crosstalk_map", [0.0] * 120)], sp.get("crosstalk_delay_s", 0.0)
        rows, extra = yaw(j, table, delay)
        fails = accept_yaw(rows, extra)
    else:
        j = journal.pitch(a.journal)
        rows, extra = pitch(j)
        fails = accept_pitch(rows, extra)
    print(f"footer {j['footer']['status'] if j.get('footer') else 'missing'} {json.dumps(extra)}")
    usecase.show(rows)
    print("ACCEPTED" if not fails else "NOT ACCEPTED: " + "; ".join(fails))
