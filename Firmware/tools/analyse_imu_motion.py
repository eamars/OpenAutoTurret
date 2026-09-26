#!/usr/bin/env python3
"""Summarize an IMU probe NDJSON file and optionally compare a motor CSV trace.

IMU rv values are interpreted as [x, y, z, w] quaternions q_W_S. Relative
rotation is q_first^-1 * q_last, so its axis is expressed in the initial sensor
frame. It is not a world/base installation pose.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter, defaultdict

MAX_AXIS_SAMPLE_AGE_NS = 100_000_000
MAX_AXIS_SAMPLE_GAP_NS = 250_000_000


def percentile(xs, p):
    if not xs:
        return None
    ys = sorted(xs)
    x = (len(ys) - 1) * p
    lo = int(x)
    hi = min(lo + 1, len(ys) - 1)
    return ys[lo] + (ys[hi] - ys[lo]) * (x - lo)


def vec_stats(vectors):
    if not vectors:
        return None
    n = len(vectors[0])
    means = [statistics.fmean(v[i] for v in vectors) for i in range(n)]
    rms = [math.sqrt(statistics.fmean(v[i] * v[i] for v in vectors)) for i in range(n)]
    return {"mean": means, "rms": rms, "count": len(vectors)}


def qnorm(q):
    return math.sqrt(sum(x * x for x in q))


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]


def relative_rotation(first, last):
    # Normalize to keep small sensor quantization errors from changing the angle.
    na, nb = qnorm(first), qnorm(last)
    if na < 1e-9 or nb < 1e-9:
        return None
    a = [x / na for x in first]
    b = [x / nb for x in last]
    # q and -q encode the same orientation; choose the short relative arc.
    if sum(x*y for x, y in zip(a, b)) < 0:
        b = [-x for x in b]
    inv_a = [-a[0], -a[1], -a[2], a[3]]
    d = qmul(inv_a, b)
    nd = qnorm(d)
    d = [x / nd for x in d]
    if d[3] < 0:
        d = [-x for x in d]
    angle = 2.0 * math.atan2(math.sqrt(sum(x*x for x in d[:3])), max(-1.0, min(1.0, d[3])))
    sn = math.sqrt(sum(x*x for x in d[:3]))
    axis = [x / sn for x in d[:3]] if sn > 1e-9 else None
    return {"angle_deg": math.degrees(angle), "axis_initial_sensor": axis}


def read_imu(path):
    by_sensor = defaultdict(list)
    rejected_by_sensor = Counter()
    ignored = malformed = 0
    with open(path, encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            row = None
            try:
                row = json.loads(line)
                if row.get("kind") != "sample" or row.get("sensor") not in ("accel", "gyro", "rv", "game_rv"):
                    ignored += 1
                    continue
                vals = [float(x) for x in row["values"]]
                expected = 4 if row["sensor"] in ("rv", "game_rv") else 3
                if len(vals) != expected:
                    raise ValueError("wrong values length")
                if not all(math.isfinite(x) for x in vals):
                    raise ValueError("nonfinite sample value")
                row["values"] = vals
                row["rx_ns"] = int(row["rx_ns"])
                row["sample_ns"] = int(row["sample_ns"])
                row["sequence"] = int(row["sequence"]) & 0xff
                row["status"] = int(row["status"])
                if not 0 <= row["status"] <= 3:
                    raise ValueError("status outside 0..3")
                by_sensor[row["sensor"]].append(row)
            except (ValueError, TypeError, KeyError, json.JSONDecodeError):
                malformed += 1
                if isinstance(row, dict) and row.get("kind") == "sample" and row.get("sensor") in ("accel", "gyro", "rv", "game_rv"):
                    rejected_by_sensor[row["sensor"]] += 1
                print(f"warning: skipped malformed IMU line {line_no}", file=sys.stderr)
    for rows in by_sensor.values():
        rows.sort(key=lambda r: r["sample_ns"])
    return by_sensor, {"ignored_lines": ignored, "malformed_lines": malformed,
                       "rejected_samples_by_sensor": dict(sorted(rejected_by_sensor.items()))}


def summarize_imu(by_sensor):
    result = {}
    for sensor, rows in sorted(by_sensor.items()):
        rx = [r["rx_ns"] for r in rows]
        sample = [r["sample_ns"] for r in rows]
        ages_ms = [(r["rx_ns"] - r["sample_ns"]) / 1e6 for r in rows]
        periods_ms = [(b-a)/1e6 for a, b in zip(rx, rx[1:]) if b > a]
        seq_gaps = dup_or_reorder = 0
        for a, b in zip(rows, rows[1:]):
            d = (b["sequence"] - a["sequence"]) & 0xff
            if 1 < d < 128:
                seq_gaps += d - 1
            elif d == 0 or d >= 128:
                dup_or_reorder += 1
        item = {
            "samples": len(rows),
            "rx_span_ms": (rx[-1] - rx[0]) / 1e6 if len(rx) > 1 else 0.0,
            "cadence_ms": {"median": percentile(periods_ms, .5), "p95": percentile(periods_ms, .95)},
            "sequence_missing_estimate": seq_gaps,
            "duplicate_or_reordered_pairs": dup_or_reorder,
            "sample_age_ms": {"median": percentile(ages_ms, .5), "p95": percentile(ages_ms, .95),
                              "max": max(ages_ms) if ages_ms else None},
            "accuracy_status_counts": dict(sorted(Counter(r["status"] for r in rows).items())),
            "sample_time_span_ms": (sample[-1] - sample[0]) / 1e6 if len(sample) > 1 else 0.0,
        }
        if sensor in ("rv", "game_rv"):
            norms = [qnorm(r["values"]) for r in rows]
            item["quaternion_norm"] = {"median": percentile(norms, .5),
                                       "max_abs_error_from_1": max(abs(x-1) for x in norms)}
            if len(rows) > 1:
                item["relative_rotation"] = relative_rotation(rows[0]["values"], rows[-1]["values"])
        else:
            item["vector"] = vec_stats([r["values"] for r in rows])
            if sensor == "gyro":
                norms = [math.sqrt(sum(x*x for x in r["values"])) for r in rows]
                item["rate_magnitude"] = {"mean": statistics.fmean(norms), "rms": math.sqrt(statistics.fmean(x*x for x in norms))}
        result[sensor] = item
    return result


def trapezoid_gyro(rows):
    total = [0.0, 0.0, 0.0]
    for a, b in zip(rows, rows[1:]):
        dt = (b["sample_ns"] - a["sample_ns"]) / 1e9
        if 0 < dt < 1.0:
            for i in range(3):
                total[i] += .5 * (a["values"][i] + b["values"][i]) * dt
    return total


def axis_sample_quality(rows, rejected_samples=0):
    """Require valid accuracy, quaternion norms, freshness and sample ordering."""
    reasons = []
    if len(rows) < 2:
        return False, ["fewer_than_two_samples"]
    if rejected_samples:
        reasons.append("invalid_samples_rejected_from_stream")
    ages = [r["rx_ns"] - r["sample_ns"] for r in rows]
    sample_gaps = [b["sample_ns"] - a["sample_ns"] for a, b in zip(rows, rows[1:])]
    rx_gaps = [b["rx_ns"] - a["rx_ns"] for a, b in zip(rows, rows[1:])]
    if any(r["status"] == 0 for r in rows):
        reasons.append("unreliable_accuracy_status")
    if any(not .9 <= qnorm(r["values"]) <= 1.1 for r in rows):
        reasons.append("quaternion_norm_out_of_range")
    if any(age < 0 or age > MAX_AXIS_SAMPLE_AGE_NS for age in ages):
        reasons.append("sample_age_out_of_range")
    if any(gap <= 0 or gap > MAX_AXIS_SAMPLE_GAP_NS for gap in sample_gaps):
        reasons.append("sample_time_not_monotonic_or_gap_too_large")
    if any(gap <= 0 for gap in rx_gaps):
        reasons.append("receive_time_not_monotonic")
    return not reasons, reasons


def compare_motor(by_sensor, path, rejected_by_sensor=None):
    rejected_by_sensor = rejected_by_sensor or {}
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    parsed = []
    for r in rows:
        try:
            r["time_ns"] = int(r["time_ns"])
            r["yaw_relative_deg"] = float(r["yaw_relative_deg"])
            if not math.isfinite(r["yaw_relative_deg"]):
                continue
            if r.get("angle_count") not in (None, ""):
                r["angle_count"] = int(r["angle_count"])
            parsed.append(r)
        except (ValueError, KeyError):
            continue
    out = {"rows": len(parsed)}
    if len(parsed) < 2:
        out["warning"] = "fewer than two usable motor rows"
        return out
    trace_lo, trace_hi = parsed[0]["time_ns"], parsed[-1]["time_ns"]
    out["phase_counts"] = dict(Counter(r.get("phase", "") for r in parsed))
    out["trace_span_ms"] = (trace_hi-trace_lo)/1e6
    sample_times = [r["sample_ns"] for rows_for_sensor in by_sensor.values() for r in rows_for_sensor]
    if not sample_times:
        out["warning"] = "no IMU samples to align with motor trace"
        out["yaw_motion_sufficient_for_axis_estimate"] = False
        out["axis_estimate_note"] = "no common IMU/motor time window"
        return out
    lo, hi = max(trace_lo, min(sample_times)), min(trace_hi, max(sample_times))
    if hi <= lo:
        out["warning"] = "IMU and motor traces do not overlap in host-monotonic time"
        out["yaw_motion_sufficient_for_axis_estimate"] = False
        out["axis_estimate_note"] = "no common IMU/motor time window"
        return out
    same_window_motor = [r for r in parsed if lo <= r["time_ns"] <= hi]
    if len(same_window_motor) < 2:
        out["warning"] = "fewer than two motor samples in common time window"
        out["yaw_motion_sufficient_for_axis_estimate"] = False
        out["axis_estimate_note"] = "insufficient common-window motor samples"
        return out
    out["common_window_ms"] = (hi-lo)/1e6
    out["yaw_relative_delta_deg"] = (same_window_motor[-1]["yaw_relative_deg"] -
                                     same_window_motor[0]["yaw_relative_deg"])
    counts = [r["angle_count"] for r in same_window_motor if isinstance(r.get("angle_count"), int)]
    out["angle_count_delta"] = counts[-1] - counts[0] if len(counts) >= 2 else None
    imu = {}
    axis_candidates = {}
    for sensor in ("rv", "game_rv"):
        subset = [r for r in by_sensor.get(sensor, []) if lo <= r["sample_ns"] <= hi]
        if len(subset) >= 2:
            quality_ok, quality_reasons = axis_sample_quality(
                subset, rejected_by_sensor.get(sensor, 0))
            rot = relative_rotation(subset[0]["values"], subset[-1]["values"])
            imu[sensor] = {"samples": len(subset), "relative_rotation": rot,
                           "axis_sample_quality": "usable" if quality_ok else "rejected",
                           "axis_sample_quality_reasons": quality_reasons}
            axis_candidates[sensor] = (quality_ok and rot is not None and
                                       rot["angle_deg"] >= .2)
    gyros = [r for r in by_sensor.get("gyro", []) if lo <= r["sample_ns"] <= hi]
    if len(gyros) >= 2:
        imu["gyro_integral_rad_input"] = trapezoid_gyro(gyros)
        imu["gyro_samples"] = len(gyros)
    out["imu_same_window"] = imu
    encoder_motion = abs(out["yaw_relative_delta_deg"]) >= .5
    out["encoder_yaw_motion_sufficient"] = encoder_motion
    out["yaw_motion_sufficient_for_axis_estimate"] = False
    if encoder_motion:
        # Normalize the observed attitude axis to the positive motor-yaw basis.
        sign = 1.0 if out["yaw_relative_delta_deg"] > 0 else -1.0
        for name in ("rv", "game_rv"):
            rot = imu.get(name, {}).get("relative_rotation")
            if not rot:
                continue
            if axis_candidates.get(name, False):
                out["yaw_motion_sufficient_for_axis_estimate"] = True
                rot["yaw_axis_estimate_initial_sensor"] = [sign*x for x in rot["axis_initial_sensor"]]
                rot["axis_estimate_note"] = "positive-yaw basis from >=0.5 deg encoder motion and >=0.2 deg usable IMU rotation; expressed in initial sensor coordinates"
            else:
                rot["axis_estimate_note"] = "withheld: requires >=0.2 deg IMU relative rotation and valid accuracy, quaternion and timing"
    if not encoder_motion:
        out["axis_estimate_note"] = "insufficient encoder yaw excursion (<0.5 deg); no yaw-axis/alignment inference"
    elif not out["yaw_motion_sufficient_for_axis_estimate"]:
        out["axis_estimate_note"] = "withheld: IMU rotation below 0.2 deg or IMU samples have unreliable quaternion, accuracy or timing"
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--imu", required=True, help="IMU probe NDJSON")
    ap.add_argument("--motor-csv", help="optional probe_mixed_hardware trace CSV")
    args = ap.parse_args()
    by_sensor, parse = read_imu(args.imu)
    report = {"imu_file": args.imu, "parse": parse, "sensors": summarize_imu(by_sensor)}
    if args.motor_csv:
        report["motor_comparison"] = compare_motor(
            by_sensor, args.motor_csv, parse.get("rejected_samples_by_sensor"))
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
