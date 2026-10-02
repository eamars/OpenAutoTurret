"""Read commissiond servo journals (yaw-control.jsonl, sensorless-homing.jsonl) into arrays.

Times are seconds from yaw_control_begin (yaw) or from the trial script's time zero
(pitch: the end of the approach). Only the row kinds the tooling uses are decoded.
"""
import json
import math

import numpy as np

COUNT = 2 * math.pi / 8192


def _typed(value):
    """The journal header stores the manifest as YAML scalars (strings); restore numbers and booleans."""
    if isinstance(value, dict):
        return {k: _typed(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_typed(v) for v in value]
    if isinstance(value, str):
        if value in ("true", "false"):
            return value == "true"
        try:
            return int(value) if value.lstrip("-").isdigit() else float(value)
        except ValueError:
            return value
    return value


def _rows(path, kinds):
    prefixes = {k: '{"kind":"%s"' % k for k in kinds}
    out = {k: [] for k in kinds}
    with open(path, encoding="utf-8") as lines:
        for line in lines:
            for k, p in prefixes.items():
                if line.startswith(p):
                    out[k].append(json.loads(line))
                    break
    return out


def yaw(path):
    r = _rows(path, ("header", "yaw_control_begin", "yaw_feedback", "yaw_current_tx", "servo_cycle",
                     "servo_learned", "servo_gains", "footer", "register_read"))
    manifest = _typed(json.loads(r["header"][0]["manifest_yaml"]))
    begin = r["yaw_control_begin"][0]["time_ns"] if r["yaw_control_begin"] else None
    # The idle pitch (disabled during yaw sessions): its MechPos readbacks, for the decoupling record.
    posture = [x["value"] for x in r["register_read"] if x.get("index") == 0x7019 and x.get("value") is not None]
    out = {"manifest": manifest, "footer": r["footer"][0] if r["footer"] else None,
           "learned": r["servo_learned"][0]["parameters"] if r["servo_learned"] else None,
           "pitch_posture": np.array(posture, float), "begin_ns": begin}
    if begin is None or not r["yaw_feedback"]:
        return out
    fb = r["yaw_feedback"]
    first_raw = fb[0]["encoder_raw"]
    out["feedback"] = {
        "t": np.array([(x["kernel_monotonic_ns"] - begin) * 1e-9 for x in fb]),
        # absolute GM6020 angle, unwrapped from the first frame (the servo's coordinates)
        "q": np.array([(first_raw + x["encoder_unwrapped_counts"]) * COUNT for x in fb]),
        "current": np.array([x["current_A"] for x in fb], float),
        "temperature": np.array([x["temperature_raw"] for x in fb], float)}
    tx = [x for x in r["yaw_current_tx"] if x["success"]]
    out["tx"] = {"t": np.array([(x["kernel_accepted_ns"] - begin) * 1e-9 for x in tx]),
                 "u": np.array([x["successful_tx_A"] for x in tx]),
                 "excitation": np.array([x["phase"] == "excitation" for x in tx])}
    c = r["servo_cycle"]
    if c:
        out["cycles"] = {k: np.array([x.get(k, 0.0) for x in c], float) for k in
                         ("qr", "vr", "ar", "q", "v", "u", "req", "ff", "fr", "i", "rock", "stalls", "sat", "rms", "cap")}
        out["cycles"]["t"] = np.array([(x["time_ns"] - begin) * 1e-9 for x in c])
    out["gains"] = [(round((g["time_ns"] - begin) * 1e-9, 3), g["kq"], g["kv"], g["ki"]) for g in r["servo_gains"]]
    return out


def applied(tx, t, delay=0.0):
    """The successfully transmitted current in force at times t - delay (zero before the first)."""
    k = np.searchsorted(tx["t"], np.asarray(t) - delay, side="right") - 1
    return np.where(k >= 0, tx["u"][np.clip(k, 0, None)], 0.0)


def crosstalk(table, q):
    """Periodic table lookup (rad/A) at absolute angles q, as Servo::crosstalk_gain."""
    table = np.asarray(table, float)
    n = len(table)
    x = np.mod(q, 2 * math.pi) / (2 * math.pi) * n
    k0 = np.floor(x).astype(int) % n
    w = x - np.floor(x)
    return (1 - w) * table[k0] + w * table[(k0 + 1) % n]


def true_position(j, table, delay):
    """Encoder angle with the current-induced reading removed (the truth the scorer uses)."""
    f = j["feedback"]
    if table is None:
        return f["q"]
    return f["q"] - crosstalk(table, f["q"]) * applied(j["tx"], f["t"], delay)


def pitch(path):
    r = _rows(path, ("header", "pitch_trial", "pitch_trial_approach", "pitch_trial_window", "homing_endpoints",
                     "pitch_trial_gains", "footer", "can_rx"))
    manifest = _typed(json.loads(r["header"][0]["manifest_yaml"]))
    out = {"manifest": manifest, "footer": r["footer"][0] if r["footer"] else None,
           "window": r["pitch_trial_window"][0] if r["pitch_trial_window"] else None,
           "endpoints": r["homing_endpoints"][0] if r["homing_endpoints"] else None}
    # The idle yaw (zero current during pitch sessions): its encoder, for the decoupling record.
    counts = [x["bytes"][0] << 8 | x["bytes"][1] for x in r["can_rx"] if x.get("axis") == "yaw" and len(x.get("bytes", [])) == 8]
    if counts:
        unwrapped = np.cumsum(np.concatenate([[0], (np.diff(counts) + 4096) % 8192 - 4096]))
        out["yaw_motion_rad"] = float(np.ptp(unwrapped) * COUNT)
    rows = r["pitch_trial"]
    if not rows:
        return out
    app = r["pitch_trial_approach"][0] if r["pitch_trial_approach"] else None
    t0 = rows[0]["time_ns"] + (int(app["duration_s"] * 1e9) if app else 0)
    out["trial"] = {k: np.array([x.get(k, 0.0) for x in rows], float) for k in ("qr", "vr", "q", "cmd", "x", "i", "torque")}
    out["trial"]["t"] = np.array([(x["time_ns"] - t0) * 1e-9 for x in rows])
    out["trial"]["status_t"] = np.array([(x["status_ns"] - t0) * 1e-9 for x in rows])
    out["centre"] = app["center_rad"] if app else None
    out["gains"] = [(round((g["time_ns"] - t0) * 1e-9, 3), g["kp"], g["ki"]) for g in r["pitch_trial_gains"]]
    return out
