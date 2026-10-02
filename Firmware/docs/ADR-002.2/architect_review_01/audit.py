#!/usr/bin/env python3
"""Read-only audit of the supplied ADR-002.2 ZIP. No project code or hardware needed.

Requires Python 3.10+, numpy and pandas. No model/controller is qualified here.
Usage: python audit.py ADR-002.2-yaw-physics-math-20261001.zip --out audit_output

The 0..150 ms current lag search is a descriptive telemetry alignment, NOT an
identification of physical Iq, torque, motor bandwidth, or precise transport delay.
"""
from __future__ import annotations
import argparse
import collections
import json
import math
from pathlib import Path, PurePosixPath
import zipfile
import numpy as np
import pandas as pd

DELAY_S = 0.06059310818541734
COUNT_RAD = 2 * math.pi / 8192
# Frozen calibration copied from candidate13's supplied control-manifest.json.
GYRO_COLUMN = np.array([-0.9964237359024551, 0.011703077994046572, 0.028933078779482667])
GYRO_BIAS = np.array([0.0001730617088607595, 0.00007416930379746836, -0.0001483386075949367])
# This common encoder phase is a registration diagnostic, not an assumed world zero.
COMMON_ENCODER_DATUM = 5768


def read_json(zf: zipfile.ZipFile, name: str):
    with zf.open(name) as stream:
        return json.load(stream)


def held(t: np.ndarray, times: np.ndarray, values: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(times, t, side="right") - 1
    return np.where(indices >= 0, values[np.maximum(indices, 0)], np.nan)


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def telemetry_lag(tq, iq, tu, u):
    """Same affine fit at each lag; use a common valid interval for all lags."""
    mask = tq >= tu[0] + 0.150
    t, y = tq[mask], iq[mask]
    best = None
    for delay_ms in range(151):
        x = held(t - delay_ms / 1000, tu, u)
        xc, yc = x - x.mean(), y - y.mean()
        denominator = float(np.dot(xc, xc))
        if denominator <= 0:
            continue
        gain = float(np.dot(xc, yc) / denominator)
        offset = float(y.mean() - gain * x.mean())
        error = rms(y - (gain * x + offset))
        if best is None or error < best["feedback_rmse_A"]:
            best = {"best_lag_ms": delay_ms, "feedback_gain": gain,
                    "feedback_intercept_A": offset, "feedback_rmse_A": error}
    if best is None:
        raise ValueError("No variable successful current input in journal")
    # These fixed-lag comparisons impose unity gain and zero offset, explicitly.
    for delay, label in [(0.0, "zero"), (0.001, "one_ms"), (DELAY_S, "frozen")]:
        best[f"unity_rmse_{label}_A"] = rms(y - held(t - delay, tu, u))
    return best


def audit_run(zf, experiment):
    name = experiment["raw_archive_path"]
    run_id = PurePosixPath(name).parts[1]
    count = collections.Counter()
    wire = {}
    feedback, tx, gyro, cycles = [], [], [], []
    origin, footer = None, None
    pairs, errors = 0, 0
    with zf.open(name) as stream:
        for number, line in enumerate(stream, 1):
            try:
                d = json.loads(line)
            except (ValueError, UnicodeError) as exc:
                raise ValueError(f"{name}:{number}: invalid JSON") from exc
            kind = d.get("kind")
            count[kind] += 1
            if kind == "session_begin":
                origin = d["time_ns"]
            elif kind == "can_rx" and d.get("axis") == "yaw":
                wire[d["kernel_monotonic_ns"]] = d
            elif kind == "yaw_feedback":
                feedback.append(d)
                raw = wire.pop(d["kernel_monotonic_ns"], None)
                if raw is not None:
                    b = bytes(raw["bytes"])
                    decoded = (int.from_bytes(b[:2], "big", signed=False),
                               int.from_bytes(b[2:4], "big", signed=True),
                               int.from_bytes(b[4:6], "big", signed=True))
                    expected = (d["encoder_raw"], d["speed_rpm"], d["current_raw"])
                    pairs += 1
                    errors += decoded != expected
            elif kind == "yaw_current_tx" and d.get("success") is True:
                tx.append(d)
            elif kind == "imu_raw":
                raw = json.loads(d["raw_json"])
                if raw.get("kind") == "sample" and raw.get("sensor") == "gyro":
                    gyro.append(raw)
            elif kind == "yaw_control_cycle":
                cycles.append(d)
            elif kind == "footer":
                footer = d
    if origin is None or not feedback or not tx:
        raise ValueError(f"Incomplete input: {name}")
    # Preserve capture order and explicitly check it before interpolation/ZOH.
    tq = (np.array([r["kernel_monotonic_ns"] for r in feedback], dtype=np.int64) - origin) * 1e-9
    tu = (np.array([r["kernel_accepted_ns"] for r in tx], dtype=np.int64) - origin) * 1e-9
    if np.any(np.diff(tq) < 0) or np.any(np.diff(tu) < 0):
        raise ValueError(f"Nonmonotonic recorded times: {name}")
    q = np.array([r["q_relative_rad"] for r in feedback])
    enc = np.array([r["encoder_raw"] for r in feedback])
    iq = np.array([r["current_A"] for r in feedback])
    u = np.array([r["successful_tx_A"] for r in tx])
    dq = ((np.diff(enc) + 4096) % 8192 - 4096) * COUNT_RAD
    reconstructed = np.r_[0.0, np.cumsum(dq)]
    phase = (q[0] - (enc[0] - COMMON_ENCODER_DATUM) * COUNT_RAD + math.pi) % (2 * math.pi) - math.pi
    result = {"run": run_id, "archive_member": name, "paired_can_records": pairs,
              "decoder_mismatches": errors, "yaw_feedback_records": len(feedback),
              "unpaired_decoded_records": len(feedback)-pairs,
              "encoder_unwrap_max_abs_error_rad": float(np.max(np.abs((q-q[0])-reconstructed))),
              "relative_vs_common_phase_offset_deg": math.degrees(phase),
              "yaw_sample_span_s": float(tq[-1]-tq[0]),
              "q_span_deg": float(np.ptp(np.rad2deg(q))),
              "current_min_A": float(iq.min()), "current_max_A": float(iq.max()),
              "temperature_raw_min": min(r["temperature_raw"] for r in feedback),
              "temperature_raw_max": max(r["temperature_raw"] for r in feedback),
              "experiment_index_counts_match": dict(count) == experiment["record_counts"]}
    result.update(telemetry_lag(tq, iq, tu, u))
    if gyro:
        tg = np.array([r["sample_ns"] for r in gyro], dtype=np.int64)
        vg = (np.array([r["values"] for r in gyro])-GYRO_BIAS) @ GYRO_COLUMN / (GYRO_COLUMN @ GYRO_COLUMN)
        if run_id == "yaw-feedback-13-speed-pos5":
            vg_by_time = dict(zip(tg.tolist(), vg.tolist()))
            projection_errors = [abs(c["observation"]["gyro_rate_rad_s"] - vg_by_time[c["observation"]["gyro_ns"]])
                                 for c in cycles if c["observation"]["gyro_ns"] in vg_by_time]
            result["candidate13_projection_compared_cycles"] = len(projection_errors)
            result["candidate13_projection_max_abs_error_rad_s"] = max(projection_errors, default=None)
        result.update({"native_gyro_records": len(gyro),
                       "native_gyro_median_period_ms": float(np.median(np.diff(tg)) * 1e-6),
                       "projected_gyro_peak_abs_deg_s": float(np.max(np.abs(np.rad2deg(vg))))})
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--out", type=Path, default=Path("audit_output"))
    args = parser.parse_args()
    if not args.archive.is_file():
        parser.error(f"Archive not found: {args.archive}")
    args.out.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.archive) as zf:
        experiments = read_json(zf, "EXPERIMENT_INDEX.json")
        rows = [audit_run(zf, e) for e in experiments if e.get("physical_data") is True]
        nf = read_json(zf, "evidence/yaw-feedback-local-update-06/numerical-fit.json")
        train = nf["training_run_descriptions"]
        predictions = [r["updated"] for r in nf["raw_prediction_errors"]]
        fit_windows = [{"run_id": w["run_id"], "journal": w["journal"],
                        "direction": w["direction"], "start_ns": w["start_ns"],
                        "end_ns": w["end_ns"],
                        "duration_s": (w["end_ns"]-w["start_ns"])*1e-9,
                        "encoder_events": w["encoder_events"], "gyro_events": w["gyro_events"]}
                       for w in train]
        train_ids = {w["run_id"] for w in train}
        prediction_ids = {w["run_id"] for w in predictions}
        durations = np.array([w["duration_s"] for w in fit_windows])
        summary = {
            "scope": "Read-only raw-journal audit; no new hardware run and no qualified model",
            "physical_journals": len(rows),
            "paired_can_records": sum(r["paired_can_records"] for r in rows),
            "decoder_mismatches": sum(r["decoder_mismatches"] for r in rows),
            "unpaired_decoded_records": sum(r["unpaired_decoded_records"] for r in rows),
            "all_experiment_index_event_counts_match": all(r["experiment_index_counts_match"] for r in rows),
            "telemetry_best_lag_ms_counts": dict(collections.Counter(r["best_lag_ms"] for r in rows)),
            "telemetry_lag_caution": "Descriptive decoded-current alignment, not calibrated physical torque delay",
            "fit_window_count": len(train), "fit_physical_journal_count": len({w["journal"] for w in train}),
            "prediction_window_count": len(predictions),
            "training_and_prediction_ids_identical": train_ids == prediction_ids,
            "fit_total_duration_s": float(durations.sum()),
            "fit_median_duration_s": float(np.median(durations)),
            "fit_min_duration_s": float(durations.min()), "fit_max_duration_s": float(durations.max()),
            "fit_windows_at_most_61ms": int(np.sum(durations <= 0.061)),
            "native_updated_prediction_failures": dict(collections.Counter(f for r in predictions for f in r["metric_failures"])),
            "integral_equations_by_direction": nf["equations_by_direction"],
            "observed_information": nf["observed_periodic_information"],
            "frozen_delay_s": nf["initializer_command_delay_s"],
            "local_q_support_rad": nf["local_q_support_rad"],
            "local_q_span_deg": math.degrees(np.ptp(nf["local_q_support_rad"])),
            "actual_posture_support_rad": nf["actual_posture_support_rad"],
            "posture_span_deg": math.degrees(np.ptp(nf["actual_posture_support_rad"])),
            "optimizer_evaluations": nf["optimizer_evaluations"],
            "optimizer_message": nf["optimizer_message"],
            "candidate14_physically_run": read_json(zf, "evidence/yaw-feedback-local-synthesis-05/retained-failure-mathematical-handoff.json")["candidate14_physically_run"],
            "qualification": "UNQUALIFIED; this audit does not produce deployment gains"
        }
    pd.DataFrame(rows).to_csv(args.out/"audit_runs.csv", index=False)
    pd.DataFrame(fit_windows).to_csv(args.out/"fit_window_inventory.csv", index=False)
    (args.out/"audit_summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
