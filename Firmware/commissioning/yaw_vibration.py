"""Report actual-time yaw acceleration and local vibration; never arm or stop motors."""
from __future__ import annotations
from collections import Counter
import json
import numpy as np


def statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    return {"count": int(values.size), "mean": float(values.mean()),
            "sigma": float(values.std()), "rms": float(np.sqrt(np.mean(values**2))),
            "minimum": float(values.min()), "maximum": float(values.max()),
            "p99_absolute": float(np.quantile(np.abs(values), .99)),
            "maximum_absolute": float(np.max(np.abs(values)))}


def _detrend(time_ns, values):
    """The recorded-data centered linear observation, using actual sample times."""
    t = (np.asarray(time_ns, dtype=np.int64)-int(time_ns[0]))/1e9
    v = np.asarray(values, dtype=float)
    if v.ndim == 1:
        v = v[:, None]
    if len(t) < 3:
        return {"available": False, "reason": "fewer_than_three_actual_samples",
                "samples": len(t)}
    centered = t-t.mean()
    tt = float(centered @ centered)
    if tt <= 0:
        return {"available": False, "reason": "no_actual_time_span", "samples": len(t)}
    slope = centered @ (v-v.mean(axis=0))/tt
    residual = v-v.mean(axis=0)-centered[:, None]*slope
    increments = np.diff(v, axis=0)
    return {"available": True, "samples": len(t), "first_sample_ns": int(time_ns[0]),
            "last_sample_ns": int(time_ns[-1]), "actual_span_s": float(t[-1]),
            "mean": v.mean(axis=0).tolist(), "slope_per_s": slope.tolist(),
            "raw_vector_norm": statistics(np.linalg.norm(v, axis=1)),
            "detrended_component_rms": np.sqrt(np.mean(residual**2, axis=0)).tolist(),
            "detrended_vector_rms": float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
            "successive_increment_vector_rms": float(np.sqrt(np.mean(np.sum(increments**2, axis=1))))}


def _samples(rows, sensor, failures):
    records = []
    for row in rows:
        if row.get("kind") != "imu_raw":
            continue
        try:
            sample = json.loads(row["raw_json"])
            if sample.get("kind") != "sample" or sample.get("sensor") != sensor:
                continue
            values = np.asarray(sample["values"], dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all():
                failures.append({"sensor": sensor, "reason": "nonfinite_or_invalid_vector",
                                 "sample_ns": sample.get("sample_ns")})
                continue
            records.append(dict(sample, dequeue_ns=row["dequeue_ns"]))
        except (ValueError, KeyError, TypeError) as exc:
            failures.append({"sensor": sensor, "reason": "invalid_record", "detail": str(exc)})
    # Repeated sample timestamps are not independent measurements; retain their count/reason.
    unique = {}
    for row in records:
        key = int(row["sample_ns"])
        if key in unique:
            failures.append({"sensor": sensor, "reason": "duplicate_sample_timestamp", "sample_ns": key})
        else:
            unique[key] = row
    return [unique[key] for key in sorted(unique)]


def _timing(samples):
    t = np.asarray([r["sample_ns"] for r in samples], dtype=np.int64)
    rx = np.asarray([r["rx_ns"] for r in samples], dtype=np.int64)
    span = (t[-1]-t[0])/1e9 if len(t) > 1 else 0.
    return {"samples": len(t), "actual_sample_rate_Hz": (len(t)-1)/span if span else None,
            "sample_gap_s": statistics(np.diff(t)/1e9), "receipt_gap_s": statistics(np.diff(rx)/1e9),
            "receipt_minus_sample_s": statistics((rx-t)/1e9),
            "accuracy_status_counts": dict(Counter(str(r["status"]) for r in samples)),
            "reported_unreliable_samples": sum(r["status"] == 0 for r in samples),
            "status_meaning": "SH-2 status0 unreliable,1 low,2 medium,3 high; all retained, no status motor gate",
            "generation_counts": dict(Counter(str(r["generation"]) for r in samples))}


def _summary(observations):
    valid = [r for r in observations if r.get("acceleration_rad_s2") is not None]
    return {"observations": len(observations), "acceleration_samples": len(valid),
            "cap_assessment_available_count": sum(r.get("cap_exceedance_observed") is not None for r in valid),
            "cap_assessment_unavailable_count": sum(r.get("cap_exceedance_observed") is None for r in valid),
            "acceleration_rad_s2": statistics([r["acceleration_rad_s2"] for r in valid]),
            "yaw_rate_rad_s": statistics([r["yaw_rate_rad_s"] for r in valid]),
            "cap_exceedance_observed_count": sum(r.get("cap_exceedance_observed") is True for r in valid),
            "cap_exceedance_clear_of_noise_count": sum(r.get("cap_exceedance_clear_of_noise") is True for r in valid),
            "unavailable_acceleration_reasons": dict(Counter(r["acceleration_unavailable_reason"] for r in observations if "acceleration_unavailable_reason" in r)),
            "local_accel_detrended_vector_rms_m_s2": statistics([
                r["accel_vibration"]["detrended_vector_rms"] for r in observations
                if r["accel_vibration"].get("available")]),
            "local_off_axis_gyro_detrended_vector_rms_rad_s": statistics([
                r["off_axis_gyro_vibration"]["detrended_vector_rms"] for r in observations
                if r["off_axis_gyro_vibration"].get("available")]),
            "local_accel_mean_vector_norm_m_s2": statistics([
                np.linalg.norm(r["accel_vibration"]["mean"]) for r in observations
                if r["accel_vibration"].get("available")])}


def analyze_yaw_vibration(rows, *, manifest=None, calibration=None, acceleration_cap=None,
                          sustained_s=None, constraint_source=None):
    """Consume raw journal records, preserving unmeasured/insufficient facts as fields.

    Acceleration cap uses rad/s². Explicit overrides are analysis constraints,
    not statements about historical configured parameters. No PSD or motor gate.
    """
    failures = []
    if manifest is None:
        manifest = json.loads(next((r.get("manifest_yaml", "{}") for r in rows if r.get("kind") == "header"), "{}"))
    parameters = manifest.get("controller_parameters", {})
    readbacks = [r for r in rows if r.get("kind") == "controller_parameters_readback"]
    readback = readbacks[-1].get("parameters", {}) if readbacks else {}
    cap_source = constraint_source
    if "acceleration_cap" in readback:
        acceleration_cap = readback["acceleration_cap"]
        cap_source = "actual_controller_parameters_readback"
    elif acceleration_cap is None and "acceleration_cap" in parameters:
        acceleration_cap = parameters["acceleration_cap"]
        cap_source = "manifest_controller_parameters"
    sustained_source = "explicit_owner_analysis_constraint"
    if sustained_s is None:
        sustained_s = readback.get("sustained_s", parameters.get("sustained_s", .06))
        sustained_source = ("actual_controller_parameters_readback" if "sustained_s" in readback else
                            "manifest_controller_parameters" if "sustained_s" in parameters else
                            "existing_60ms_analysis_convention")
    sustained_s = float(sustained_s)
    if not np.isfinite(sustained_s) or sustained_s <= 0:
        raise ValueError("positive finite actual sustained observation support required")
    if acceleration_cap is not None:
        acceleration_cap = float(acceleration_cap)
        if not np.isfinite(acceleration_cap) or acceleration_cap <= 0:
            failures.append({"reason": "invalid_declared_acceleration_cap", "value": acceleration_cap})
            acceleration_cap = None
    calibration = calibration or manifest.get("gyro_calibration")
    if calibration and "gyro_calibration" in calibration:
        calibration = calibration["gyro_calibration"]
    column = projection = bias = None
    if calibration:
        column = np.asarray(calibration["yaw_column"], float)
        bias = np.asarray(calibration["baseline_sensor_bias"], float)
        if column.shape != (3,) or bias.shape != (3,) or not np.isfinite([column, bias]).all() or column @ column <= 0:
            failures.append({"reason": "invalid_frozen_yaw_calibration"})
            column = bias = None
        else:
            projection = column/(column @ column)
    if column is None:
        failures.append({"reason": "frozen_yaw_column_unavailable",
                         "detail": "Sensor-frame acceleration/gyro reporting retained; yaw cap/off-axis projection unavailable"})
    gyro = _samples(rows, "gyro", failures)
    accel = _samples(rows, "accel", failures)
    gt = np.asarray([r["sample_ns"] for r in gyro], np.int64)
    at = np.asarray([r["sample_ns"] for r in accel], np.int64)
    gv = np.asarray([r["values"] for r in gyro], float).reshape(-1, 3)
    av = np.asarray([r["values"] for r in accel], float).reshape(-1, 3)
    begin = next((r["time_ns"] for r in rows if r.get("kind") in ("excitation_begin", "yaw_control_begin")), None)
    start = next((r["time_ns"] for r in rows if r.get("kind") == "session_begin"), int(gt[0]) if len(gt) else 0)
    stop = next((r["time_ns"] for r in rows if r.get("kind") == "stop_observation_begin"), None)
    if begin is None:
        failures.append({"reason": "command_phase_begin_unavailable"})
        begin = start
    baseline_start = max(start, begin-round(2e9))
    gm = (gt >= baseline_start) & (gt < begin)
    am = (at >= baseline_start) & (at < begin)
    baseline_accel = _detrend(at[am], av[am]) if am.any() else {"available": False, "reason": "quiet_accel_samples_missing"}
    baseline_gyro = _detrend(gt[gm], gv[gm]) if gm.any() else {"available": False, "reason": "quiet_gyro_samples_missing"}
    yaw = (gv-bias) @ projection if projection is not None else None
    cross = gv-yaw[:, None]*column if yaw is not None else None
    if yaw is not None and gm.sum() >= 3:
        noise = max(float(yaw[gm].std()), float(np.linalg.norm(projection)/(512*np.sqrt(12))))
        baseline_cross = _detrend(gt[gm], cross[gm])
    else:
        noise = None
        baseline_cross = {"available": False, "reason": "quiet_yaw_noise_or_column_unavailable"}
        failures.append({"reason": "quiet_yaw_noise_unavailable", "actual_samples": int(gm.sum())})
    observations = []
    rest_since = None
    rest_confirmed = False
    startup_since = None
    for i, stamp in enumerate(gt):
        phase = "baseline" if stamp < begin else "final_zero_coast" if stop is not None and stamp >= stop else "command_body"
        left = max(0, int(np.searchsorted(gt, stamp-round(sustained_s*1e9), side="right"))-1)
        indices = np.arange(left, i+1)
        same_generation = all(gyro[j]["generation"] == gyro[i]["generation"] for j in indices)
        enough = stamp-gt[left] >= round(sustained_s*1e9) and len(indices) >= 3 and same_generation
        aw = (at >= gt[left]) & (at <= stamp)
        accel_fit = _detrend(at[aw], av[aw]) if aw.any() else {"available": False, "reason": "local_accel_samples_missing"}
        gyro_fit = _detrend(gt[indices], gv[indices])
        cross_fit = _detrend(gt[indices], cross[indices]) if cross is not None else {"available": False, "reason": "frozen_yaw_column_unavailable"}
        record = {"sample_ns": int(stamp), "rx_ns": gyro[i]["rx_ns"], "dequeue_ns": gyro[i]["dequeue_ns"],
                  "status": gyro[i]["status"], "generation": gyro[i]["generation"], "capture_phase": phase,
                  "actual_support_first_ns": int(gt[left]), "actual_support_s": float((stamp-gt[left])/1e9),
                  "gyro_samples": len(indices), "accel_vibration": accel_fit,
                  "sensor_frame_gyro_vibration": gyro_fit, "off_axis_gyro_vibration": cross_fit,
                  "motion_phase": "unresolved", "acceleration_rad_s2": None}
        if noise is not None and enough:
            t = (gt[indices]-stamp)/1e9
            centered = t-t.mean()
            tt = float(centered @ centered)
            rate = float(yaw[i])
            classification_rate = float(yaw[indices].mean())
            slope = float(centered @ (yaw[indices]-yaw[indices].mean())/tt)
            residual = yaw[indices]-yaw[indices].mean()-centered*slope
            sigma = max(noise, float(np.sqrt(np.mean(residual**2))))/np.sqrt(tt)
            changing = abs(slope) > 3*sigma
            moving = abs(classification_rate) > 3*noise
            speed_change = "acceleration" if classification_rate*slope > 0 else "deceleration"
            motion = speed_change if changing else "steady_rotation"
            if not moving:
                rest_since = int(stamp) if rest_since is None else rest_since
                rest_confirmed = stamp-rest_since >= round(sustained_s*1e9)
                startup_since = None
                motion = "rest"
            else:
                if rest_confirmed:
                    startup_since = int(stamp)
                rest_since = None
                rest_confirmed = False
                if startup_since is not None:
                    if changing and speed_change == "acceleration" or stamp-startup_since < round(sustained_s*1e9):
                        motion = "startup"
                    else:
                        startup_since = None
            record.update(yaw_rate_rad_s=rate, classification_window_mean_rate_rad_s=classification_rate,
                          acceleration_rad_s2=slope,
                          acceleration_noise_sigma_rad_s2=float(sigma), uncertainty_three_sigma_rad_s2=float(3*sigma),
                          motion_phase=motion, resolved_speed_change=speed_change if changing else "unresolved_change",
                          cap_exceedance_observed=bool(abs(slope)>acceleration_cap) if acceleration_cap is not None else None,
                          cap_exceedance_clear_of_noise=bool(abs(slope)-3*sigma>acceleration_cap) if acceleration_cap is not None else None)
            if i and gyro[i-1]["generation"] == gyro[i]["generation"]:
                record["raw_adjacent_acceleration_rad_s2"] = float((yaw[i]-yaw[i-1])/((gt[i]-gt[i-1])/1e9))
            if accel_fit.get("available") and baseline_accel.get("available"):
                denominator = baseline_accel["detrended_vector_rms"]
                record["accel_residual_to_own_baseline_ratio"] = accel_fit["detrended_vector_rms"]/denominator if denominator else None
        else:
            record["acceleration_unavailable_reason"] = "insufficient_actual_support_or_generation_change" if not enough else "quiet_noise_or_yaw_calibration_unavailable"
        observations.append(record)
    pitch = [r for r in rows if r.get("kind") == "register_read" and r.get("axis") == "pitch" and r.get("index") == 0x7019]
    runtime_cycles = [r.get("acceleration_control") for r in rows
                      if r.get("kind") == "yaw_control_cycle" and isinstance(r.get("acceleration_control"), dict)]
    runtime_fresh = [r for r in runtime_cycles if r.get("fresh") is True]
    return {"schema": "adr0022.yaw-acceleration-vibration-report/1", "status": "REPORT_COMPLETE",
            "terminal_capture_status": next((r.get("status") for r in reversed(rows) if r.get("kind") == "footer"), None),
            "constraints": {"acceleration_cap_rad_s2": acceleration_cap, "cap_source": cap_source,
                            "sustained_s": sustained_s, "same_cap_for_acceleration_and_deceleration": True,
                            "sustained_source": sustained_source,
                            "configured_acceleration_noise_sigma": readback.get("acceleration_noise_sigma", parameters.get("acceleration_noise_sigma")),
                            "configured_acceleration_sample_period_s": readback.get("acceleration_sample_period_s", parameters.get("acceleration_sample_period_s")),
                            "speed_constraint_changed": False},
            "calibration": {"frozen_column": column.tolist() if column is not None else None,
                            "frozen_bias": bias.tolist() if bias is not None else None, "refitted": False,
                            "scope": "single current-pose yaw column; full mounting/lever arm unknown"},
            "quiet_baseline": {"start_ns": baseline_start, "end_ns_exclusive": begin, "requested_s": 2.,
                               "accel": baseline_accel, "sensor_frame_gyro": baseline_gyro,
                               "off_axis_gyro": baseline_cross, "yaw_noise_sigma_rad_s": noise},
            "sensor_timing": {"gyro": _timing(gyro), "accel": _timing(accel)},
            "capture_phases": {key: _summary([r for r in observations if r["capture_phase"] == key])
                               for key in ("baseline", "command_body", "final_zero_coast")},
            "motion_phases": {key: _summary([r for r in observations if r["capture_phase"] != "baseline" and r["motion_phase"] == key])
                              for key in ("startup", "acceleration", "deceleration", "steady_rotation", "rest", "unresolved")},
            "raw_adjacent_acceleration_rad_s2": statistics([r["raw_adjacent_acceleration_rad_s2"] for r in observations if "raw_adjacent_acceleration_rad_s2" in r]),
            "observations": observations, "retained_measurement_failures": failures,
            "runtime_acceleration_control": {"cycles": len(runtime_cycles), "fresh_observations": len(runtime_fresh),
                "held_cycles": len(runtime_cycles)-len(runtime_fresh),
                "fresh_measured_rad_s2": statistics([r.get("measured_rad_s2", np.nan) for r in runtime_fresh]),
                "fresh_records": runtime_fresh,
                "scope": "Actual fresh=true runtime derivative observations only; held cycles are not new measurements"},
            "passive_pitch_readbacks": [{"receive_ns": r["receive_ns"], "MechPos_rad": r["value"]} for r in pitch],
            "units": {"accel": "m/s^2 including gravity/specific force", "gyro": "rad/s, sensor axes"},
            "interpretation": "Local linear detrending keeps constant centripetal acceleration and constant mounting residual in separate mean fields. Residuals/increments remain vibration proxies, with no tipping certificate or new vibration threshold. Overlapping local windows are not independent samples.",
            "uncertainty_scope": "3sigma uses existing observed-noise convention plus local rate residual scatter; sensor internal filter, timestamp alignment and high-frequency response remain unknown. Raw adjacent differences and causal slopes do not certify instantaneous physical cap compliance.",
            "timing_scope": "Actual unique SH-2-derived sample timestamps; receipt/dequeue retained. No invented uniform200Hz stream, PSD resampling requirement or PSD arming gate.",
            "motion_phase_rule": "Observed rest/rate change relative to existing3sigma noise; startup follows sustained observed rest, steady_rotation means slope unresolved against its local uncertainty. Classification reports measurements; it never arms, aborts, or commands HOLD.",
            "motor_action": "NONE", "qualification": False}
