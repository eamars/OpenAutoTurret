"""Describe a yaw acquisition journal without qualifying motion or sensor noise."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import sys
from types import SimpleNamespace


def describe(values):
    values = list(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "minimum": min(values), "maximum": max(values),
            "mean": statistics.fmean(values), "median": statistics.median(values),
            "standard_deviation": statistics.pstdev(values)}


def load_capture(journal: Path, manifest: Path | None = None):
    journal = Path(journal)
    if journal.is_dir():
        directory = journal
        journal = directory / "capture.jsonl"
        if not journal.exists():
            journal = directory / "yaw-acquisition.jsonl"
        if manifest is None and (directory / "manifest.json").exists():
            manifest = directory / "manifest.json"
    rows = [json.loads(line) for line in journal.read_text().splitlines() if line.strip()]
    header = rows[0] if rows else {}
    config = json.loads(Path(manifest).read_text()) if manifest else json.loads(header.get("manifest_yaml", "{}"))
    return journal, config, rows


def summarize(journal: Path, manifest: Path | None = None) -> dict:
    journal, config, rows = load_capture(journal, manifest)
    footer = rows[-1] if rows else {}
    yaw = [row for row in rows if row.get("kind") == "yaw_feedback"]
    tx = [row for row in rows if row.get("kind") == "yaw_current_tx"]
    successful = [row for row in tx if row.get("success")]
    imu = [json.loads(row["raw_json"]) for row in rows if row.get("kind") == "imu_raw"]
    samples = [row for row in imu if row.get("kind") == "sample"]
    phase_commands = {}
    for phase in ("baseline", "excitation", "stop"):
        commands = [row for row in successful if row.get("phase") == phase]
        phase_commands[phase] = {
            "successful_transmissions": len(commands),
            "actual_current_A": describe(row["successful_tx_A"] for row in commands),
            "first_kernel_accepted_ns": commands[0]["kernel_accepted_ns"] if commands else None,
            "last_kernel_accepted_ns": commands[-1]["kernel_accepted_ns"] if commands else None,
        }
    stamps = [row["kernel_monotonic_ns"] for row in yaw]
    excitation = phase_commands["excitation"]["first_kernel_accepted_ns"]
    stopped = phase_commands["stop"]["first_kernel_accepted_ns"]
    before_motion = [row for row in yaw if excitation is not None and row["kernel_monotonic_ns"] <= excitation]
    before_stop = [row for row in yaw if stopped is not None and row["kernel_monotonic_ns"] <= stopped]
    after_stop = [row for row in yaw if stopped is not None and row["kernel_monotonic_ns"] >= stopped]
    first = yaw[0] if yaw else None
    motion_start = before_motion[-1] if before_motion else first
    motion_end = before_stop[-1] if before_stop else None
    final = yaw[-1] if yaw else None
    displacement = {}
    if first and final:
        displacement = {
            "first_encoder_raw": first["encoder_raw"], "final_encoder_raw": final["encoder_raw"],
            "total_relative_rad": final["q_relative_rad"] - first["q_relative_rad"],
            "first_receipt_ns": first["kernel_monotonic_ns"],
            "final_receipt_ns": final["kernel_monotonic_ns"],
        }
        if motion_start and motion_end:
            displacement["excitation_relative_rad"] = motion_end["q_relative_rad"] - motion_start["q_relative_rad"]
        if motion_end:
            displacement["drift_after_zero_rad"] = final["q_relative_rad"] - motion_end["q_relative_rad"]
            displacement["observed_after_zero_s"] = max(0, (final["kernel_monotonic_ns"] - stopped) / 1e9)
    return {
        "schema": "adr0022.yaw-data-summary/1", "provenance": config.get("provenance"),
        "journal": str(journal.resolve()), "manifest": str(Path(manifest).resolve()) if manifest else "capture header",
        "capture_complete": footer.get("status") == "COMPLETE" and footer.get("sequence_complete") is True,
        "capture_footer_status": footer.get("status"), "capture_footer_detail": footer.get("detail"),
        "footer": footer, "record_counts": dict(Counter(row.get("kind", "unknown") for row in rows)),
        "imu_sample_counts": dict(Counter(row["sensor"] for row in samples)),
        "gyro_accuracy_counts": dict(Counter(str(row["status"]) for row in samples if row["sensor"] == "gyro")),
        "yaw_feedback": {
            "encoder_raw": describe(row["encoder_raw"] for row in yaw),
            "current_raw": describe(row["current_raw"] for row in yaw),
            "reported_current_A": describe(row["current_A"] for row in yaw),
            "temperature_raw": describe(row["temperature_raw"] for row in yaw),
            "speed_rpm": describe(row["speed_rpm"] for row in yaw),
            "receipt_interval_s": describe((b-a)/1e9 for a, b in zip(stamps, stamps[1:])),
            "dequeue_delay_s": describe((row["dequeue_ns"]-row["kernel_monotonic_ns"])/1e9 for row in yaw),
            "stop_feedback_samples": len(after_stop),
        },
        "current_commands": phase_commands,
        "tx_duration_s": describe((row["kernel_accepted_ns"]-row["begin_ns"])/1e9 for row in successful),
        "displacement": displacement,
        "qualification": {"dynamics": False, "current_mapping": False, "stopping": False},
        "interpretation": "Acquisition completion records the requested sequence and zero-current observation. Dynamics, current mapping and stopping require subsequent physical analysis; raw noise and gyro accuracy zero are retained.",
    }


def regression(x, y):
    """Finite centered least-squares observation, including detrended scatter."""
    x, y = list(x), list(y)
    if len(x) < 2:
        return {"samples": len(x)}
    mx, my = statistics.fmean(x), statistics.fmean(y)
    xx = sum((v-mx)**2 for v in x)
    slope = sum((a-mx)*(b-my) for a, b in zip(x, y))/xx if xx else None
    intercept = my-slope*mx if slope is not None else my
    residuals = [b-(intercept+slope*a) for a, b in zip(x, y)] if slope is not None else [b-my for b in y]
    return {"samples": len(x), "slope": slope, "intercept": intercept,
            "detrended_sigma": math.sqrt(statistics.fmean(v*v for v in residuals))}


def timing(rows, stamp_key):
    stamps = [row[stamp_key] for row in rows]
    span = (stamps[-1]-stamps[0])/1e9 if len(stamps) > 1 else 0
    return {"timestamp_field": stamp_key, "samples": len(stamps),
            "first_ns": stamps[0] if stamps else None, "last_ns": stamps[-1] if stamps else None,
            "observed_span_s": span, "observed_rate_Hz": (len(stamps)-1)/span if span > 0 else None,
            "adjacent_interval_s": describe((b-a)/1e9 for a, b in zip(stamps, stamps[1:]))}


def yaw_window(rows):
    if not rows:
        return {"samples": 0}
    t0 = rows[0]["kernel_monotonic_ns"]
    times = [(row["kernel_monotonic_ns"]-t0)/1e9 for row in rows]
    fit = regression(times, (row["q_relative_rad"] for row in rows))
    fit["slope_unit"] = "rad/s"
    fit["detrended_sigma_unit"] = "rad"
    mean_rpm = statistics.fmean(row["speed_rpm"] for row in rows)
    return {"samples": len(rows), "timing": timing(rows, "kernel_monotonic_ns"),
            "encoder_counts": describe(row["encoder_unwrapped_counts"] for row in rows),
            "encoder_displacement_counts": rows[-1]["encoder_unwrapped_counts"]-rows[0]["encoder_unwrapped_counts"],
            "encoder_displacement_rad": rows[-1]["q_relative_rad"]-rows[0]["q_relative_rad"],
            "encoder_linear_observation": fit,
            "raw_current": describe(row["current_raw"] for row in rows),
            "protocol_current_A": describe(row["current_A"] for row in rows),
            "raw_speed_rpm": describe(row["speed_rpm"] for row in rows),
            "mean_rpm_as_rad_s": mean_rpm*2*math.pi/60,
            "encoder_regression_minus_mean_rpm_rad_s": fit["slope"]-mean_rpm*2*math.pi/60 if fit.get("slope") is not None else None,
            "temperature_raw": describe(row["temperature_raw"] for row in rows)}


def analyze_baseline(journal: Path, manifest: Path | None = None) -> dict:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from commissioning.adaptation import first_stimulus
    from commissioning.breakaway import estimate_interval
    from commissioning.contracts import Rejected

    journal, config, rows = load_capture(journal, manifest)
    yaw = [row for row in rows if row.get("kind") == "yaw_feedback"]
    tx = [row for row in rows if row.get("kind") == "yaw_current_tx" and row.get("success")]
    tx_times = [row["kernel_accepted_ns"] for row in tx]
    begin = next(row["time_ns"] for row in rows if row.get("kind") == "session_begin")
    excitation = next(row["time_ns"] for row in rows if row.get("kind") == "excitation_begin")
    stop = next(row["time_ns"] for row in rows if row.get("kind") == "stop_observation_begin")
    baseline_start = max(begin, excitation-int(2e9))
    baseline_yaw = [row for row in yaw if baseline_start <= row["kernel_monotonic_ns"] < excitation]
    stop_yaw = [row for row in yaw if stop <= row["kernel_monotonic_ns"] <= stop+int(2e9)]
    paired = []
    for row in yaw:
        index = bisect_right(tx_times, row["kernel_monotonic_ns"])-1
        if index >= 0:
            paired.append({"feedback": row, "command": tx[index],
                           "last_tx_age_s": (row["kernel_monotonic_ns"]-tx_times[index])/1e9})
    rate_window_s = estimate_interval.__kwdefaults__["sustained_s"]
    receipt_times = [row["kernel_monotonic_ns"] for row in yaw]
    encoder_velocity = []
    regression_uncertainty = []
    quantum = 2*math.pi/8192
    for index, row in enumerate(yaw):
        first = bisect_right(receipt_times, row["kernel_monotonic_ns"]-int(rate_window_s*1e9))
        local = yaw[first:index+1]
        local_t = [(value["kernel_monotonic_ns"]-row["kernel_monotonic_ns"])/1e9 for value in local]
        fit = regression(local_t, (value["q_relative_rad"] for value in local))
        encoder_velocity.append(fit.get("slope") or 0.0)
        if len(local_t) > 1:
            mean_t = statistics.fmean(local_t)
            sum_t2 = sum((value-mean_t)**2 for value in local_t)
            regression_uncertainty.append(quantum/math.sqrt(12*sum_t2))
    baseline_rates = [value for row, value in zip(yaw, encoder_velocity)
                      if baseline_start+int(rate_window_s*1e9) <= row["kernel_monotonic_ns"] < excitation]
    observed_rate_sigma = statistics.pstdev(baseline_rates)
    quantization_rate_sigma = statistics.median(regression_uncertainty)
    rate_sigma = observed_rate_sigma if observed_rate_sigma > 0 else quantization_rate_sigma
    sensors = {}
    samples = []
    for row in rows:
        if row.get("kind") == "imu_raw":
            sample = json.loads(row["raw_json"])
            if sample.get("kind") == "sample":
                samples.append(dict(sample, dequeue_ns=row["dequeue_ns"]))
    for sensor in ("gyro", "accel", "rv", "game_rv"):
        all_samples = [row for row in samples if row["sensor"] == sensor]
        selected = [row for row in all_samples if baseline_start <= row["sample_ns"] < excitation]
        per_axis = []
        for axis in range(3 if sensor in ("gyro", "accel") else 4):
            values = [row["values"][axis] for row in selected]
            times = [(row["sample_ns"]-baseline_start)/1e9 for row in selected]
            per_axis.append({"component": axis, "raw_values": describe(values),
                             "time_regression": regression(times, values)})
        sensors[sensor] = {"baseline_samples": len(selected), "components": per_axis,
            "generation_counts": dict(Counter(str(row["generation"]) for row in selected)),
            "accuracy_status_counts": dict(Counter(str(row["status"]) for row in selected)),
            "host_sample_timing": timing(selected, "sample_ns"),
            "host_receipt_timing": timing(selected, "rx_ns"),
            "producer_to_recorder_delay_s": describe((row["dequeue_ns"]-row["rx_ns"])/1e9 for row in selected),
            "native_sh2_interval_s": describe((b["sh2_us"]-a["sh2_us"])/1e6 for a, b in zip(selected, selected[1:]) if a["generation"] == b["generation"]),
            "native_sh2_span_us": selected[-1]["sh2_us"]-selected[0]["sh2_us"] if len(selected) > 1 else None,
            "receipt_minus_host_sample_s": describe((row["rx_ns"]-row["sample_ns"])/1e9 for row in selected),
            "full_capture_samples": len(all_samples)}
    pitch = [row for row in rows if row.get("kind") == "register_read" and
             row.get("axis") == "pitch" and row.get("index") == 0x7019]
    baseline_pitch = [row for row in pitch if baseline_start <= row["receive_ns"] < excitation]
    segments = []
    breakaway_intervals = []
    offset = 0.0
    for index, segment in enumerate(config["current_segments"]):
        duration = float(segment["duration_s"])
        start, end = excitation+int(offset*1e9), excitation+int((offset+duration)*1e9)
        segment_yaw = [row for row in yaw if start <= row["kernel_monotonic_ns"] < end]
        commands = [row for row in tx if start <= row["kernel_accepted_ns"] < end]
        segments.append({"index": index, "declared_start_A": float(segment["start_A"]),
                         "declared_end_A": float(segment["end_A"]), "duration_s": duration,
                         "observed": yaw_window(segment_yaw),
                         "successful_command_A": describe(row["successful_tx_A"] for row in commands)})
        start_A, end_A = float(segment["start_A"]), float(segment["end_A"])
        if abs(end_A) > abs(start_A):
            direction = 1 if end_A > 0 else -1
            indices = [i for i, row in enumerate(yaw) if start <= row["kernel_monotonic_ns"] < end]
            observed_commands = [tx[bisect_right(tx_times, receipt_times[i])-1]["successful_tx_A"] for i in indices]
            try:
                result = estimate_interval(
                    [(receipt_times[i]-start)/1e9 for i in indices], observed_commands,
                    [yaw[i]["q_relative_rad"] for i in indices], [encoder_velocity[i] for i in indices],
                    direction=direction, sigma_velocity=rate_sigma, encoder_quantum=quantum)
            except Rejected as exc:
                result = {"analysis_status": "UNANALYSABLE", "reason": exc.reason.value,
                    "detail": exc.detail, "censored": True, "not_moving_A": None,
                    "sustained_motion_A": None, "first_motion_s": None, "sustained_confirmed_s": None,
                    "start_ns": start, "end_ns_exclusive": end, "encoder_receipts": len(indices),
                    "actual_command_at_encoder_receipts_A": describe(observed_commands),
                    "actual_successful_TX": [{"kernel_accepted_ns": row["kernel_accepted_ns"],
                        "successful_tx_A": row["successful_tx_A"]} for row in commands],
                    "definition": "directed_total_current_not_increment",
                    "interpretation": "The estimator rejected this segment trace; no startup endpoint is inferred. Raw segment and whole-capture observations remain available."}
            breakaway_intervals.append({"segment_index": index, "direction": direction,
                                        "source": "commissioning.breakaway.estimate_interval", **result})
        offset += duration
    # Manifest pieces describe interpolation, not separate physical startup attempts.
    # Extract actual monotonic propulsive ramps, including flat quantized steps,
    # and end them at real current turning points or current-direction reversals.
    actual = [row for row in tx if excitation <= row["kernel_accepted_ns"] < stop]
    continuous_starts = []
    gyro_samples = [row for row in samples if row["sensor"] == "gyro"]
    gyro_bias = [component["raw_values"]["mean"] for component in sensors["gyro"]["components"]]
    gyro_sigma = math.sqrt(sum(component["raw_values"]["standard_deviation"]**2
                               for component in sensors["gyro"]["components"]))
    for direction in (1, -1):
        first = 0
        while first < len(actual)-1:
            value = direction*actual[first]["successful_tx_A"]
            difference = direction*(actual[first+1]["successful_tx_A"]-actual[first]["successful_tx_A"])
            if value < 0 or difference < -1e-9:
                first += 1
                continue
            last = first
            while last+1 < len(actual) and direction*actual[last+1]["successful_tx_A"] >= 0 and direction*(actual[last+1]["successful_tx_A"]-actual[last]["successful_tx_A"]) >= -1e-9:
                last += 1
            if direction*(actual[last]["successful_tx_A"]-actual[first]["successful_tx_A"]) <= 1e-9:
                first = last+1
                continue
            start = actual[first]["kernel_accepted_ns"]
            end = actual[last+1]["kernel_accepted_ns"] if last+1 < len(actual) else stop
            indices = [i for i, stamp in enumerate(receipt_times) if start <= stamp < end]
            observed_commands = [tx[bisect_right(tx_times, receipt_times[i])-1]["successful_tx_A"] for i in indices]
            try:
                result = estimate_interval([(receipt_times[i]-start)/1e9 for i in indices], observed_commands,
                    [yaw[i]["q_relative_rad"] for i in indices], [encoder_velocity[i] for i in indices],
                    direction=direction, sigma_velocity=rate_sigma, encoder_quantum=quantum)
            except Rejected as exc:
                result = {"not_moving_A": actual[last]["successful_tx_A"], "sustained_motion_A": None,
                          "censored": True, "first_motion_s": None, "sustained_confirmed_s": None,
                          "reason": exc.reason.value, "detail": exc.detail,
                          "definition": "directed_total_current_not_increment"}
            preceding = [i for i, stamp in enumerate(receipt_times) if start-int(rate_window_s*1e9) <= stamp < start]
            preceding_gyro = [row for row in gyro_samples if start-int(rate_window_s*1e9) <= row["sample_ns"] < start]
            rotation = gyro_rotation(preceding_gyro, gyro_bias)
            gyro_norms = [math.sqrt(sum((value-bias)**2 for value, bias in zip(row["values"], gyro_bias))) for row in preceding_gyro]
            rates = [encoder_velocity[i] for i in preceding]
            counts = [yaw[i]["encoder_unwrapped_counts"] for i in preceding]
            quiet_encoder = bool(rates) and max(abs(value) for value in rates) <= 3*rate_sigma and max(counts)-min(counts) <= 1
            quiet_gyro = bool(gyro_norms) and max(gyro_norms) <= 3*gyro_sigma
            moving_encoder = bool(rates) and max(abs(value) for value in rates) > 3*rate_sigma and max(counts)-min(counts) > 1
            moving_gyro = rotation["integrated_s"] > 0 and rotation["rotation_vector_norm_rad"] > 3*gyro_sigma*rotation["integrated_s"]
            from_rest = quiet_encoder and quiet_gyro
            context = "OBSERVED_REST" if from_rest else "ONGOING_MOTION" if moving_encoder or moving_gyro else "UNRESOLVED"
            support_pitch = [row for row in pitch if start <= row["receive_ns"] < end]
            proof_start = start+int(result["first_motion_s"]*1e9) if result["first_motion_s"] is not None else None
            proof_end = start+int(result["sustained_confirmed_s"]*1e9) if result["sustained_confirmed_s"] is not None else None
            proof_gyro = [row for row in gyro_samples if proof_start is not None and proof_start <= row["sample_ns"] <= proof_end]
            proof_rotation = gyro_rotation(proof_gyro, gyro_bias)
            independent_onset = (proof_rotation["integrated_s"] > 0 and
                proof_rotation["rotation_vector_norm_rad"] > 3*gyro_sigma*proof_rotation["integrated_s"])
            continuous_starts.append({"direction": direction, "source": "commissioning.breakaway.estimate_interval", **result,
                "start_ns": start, "end_ns_exclusive": end, "duration_s": (end-start)/1e9,
                "successful_command_A": describe(row["successful_tx_A"] for row in actual[first:last+1]),
                "successful_TX_count": last-first+1, "encoder_receipts": len(indices),
                "encoder_raw": describe(yaw[i]["encoder_raw"] for i in indices),
                "q_relative_rad": describe(yaw[i]["q_relative_rad"] for i in indices),
                "pitch_mechpos_rad": describe(row["value"] for row in support_pitch),
                "pitch_receipt_ns": [row["receive_ns"] for row in support_pitch],
                "start_context": context, "began_from_observed_rest": from_rest,
                "local_encoder_onset_interval_observed": from_rest and not result["censored"],
                "independently_supported_onset": from_rest and not result["censored"] and independent_onset,
                "preceding_observation": {"requested_window_s": rate_window_s,
                    "encoder_receipts": len(preceding), "encoder_velocity_rad_s": describe(rates),
                    "encoder_counts": describe(counts), "gyro_noise_norm_rad_s": gyro_sigma,
                    "gyro_rate_norm_rad_s": describe(gyro_norms), "body_rotation": rotation},
                "confirmation_gyro_observation": {"start_ns": proof_start, "end_ns": proof_end,
                    "body_rotation": proof_rotation, "baseline_sigma_vector_norm_rad_s": gyro_sigma,
                    "above_baseline_scatter": independent_onset,
                    "accuracy_status_counts": dict(Counter(str(row["status"]) for row in proof_gyro))},
                "boundary_rule": "actual directed monotonic successful TX; current turning points/direction reversal end the trace",
                "interpretation": "local_encoder_onset_interval_observed is encoder-defined onset only; it does not establish independent body motion or continued following. independently_supported_onset additionally records the existing three-sigma gyro rotation comparison in the actual confirmation window. Ongoing motion is not a startup threshold. Neither observation establishes continued following, running-load/plant qualification or spatial/posture coverage."})
            first = last+1
    directional = []
    for sign in (1, -1):
        commands = [row for row in tx if row["phase"] == "excitation" and sign*row["successful_tx_A"] > 0]
        if not commands:
            continue
        start, end = commands[0]["kernel_accepted_ns"], commands[-1]["kernel_accepted_ns"]
        observed = [row for row in yaw if start <= row["kernel_monotonic_ns"] <= end]
        previous = [row for row in yaw if row["kernel_monotonic_ns"] < start]
        reference = previous[-1] if previous else observed[0]
        directed_steps = [(a, b) for a, b in zip([reference]+observed, observed)
                          if sign*(b["encoder_unwrapped_counts"]-a["encoder_unwrapped_counts"]) > 0]
        onset = None
        if directed_steps:
            a, b = directed_steps[0]
            ia = bisect_right(tx_times, a["kernel_monotonic_ns"])-1
            ib = bisect_right(tx_times, b["kernel_monotonic_ns"])-1
            onset = {"last_receipt_before_step_ns": a["kernel_monotonic_ns"],
                     "first_directed_step_receipt_ns": b["kernel_monotonic_ns"],
                     "signed_command_A_interval": [tx[ia]["successful_tx_A"], tx[ib]["successful_tx_A"]],
                     "step_counts": b["encoder_unwrapped_counts"]-a["encoder_unwrapped_counts"]}
        directional.append({"direction": "positive" if sign > 0 else "negative",
            "observed_command_A": describe(row["successful_tx_A"] for row in commands),
            "encoder_observation": yaw_window(observed), "first_directed_encoder_step": onset,
            "no_directed_step_censored_observation": onset is None,
            "directed_steps": len(directed_steps),
            "opposite_steps": sum(sign*(b["encoder_unwrapped_counts"]-a["encoder_unwrapped_counts"]) < 0
                                  for a, b in zip([reference]+observed, observed)),
            "directed_step_span_s": (directed_steps[-1][1]["kernel_monotonic_ns"]-directed_steps[0][1]["kernel_monotonic_ns"])/1e9 if directed_steps else None,
            "sustained_motion_qualified": False,
            "interpretation": "The signed interval brackets the first encoder-resolvable step against last successful commands. A step or a brief ramp is not a qualified breakaway or sustained-motion measurement; receipt and actuator delays remain included."})
    receipts = [row for row in rows if row.get("kind") == "can_rx"]
    baseline_receipts = [row for row in receipts if baseline_start <= row["kernel_monotonic_ns"] < excitation]
    mapping = regression((row["command"]["successful_tx_A"] for row in paired),
                         (row["feedback"]["current_A"] for row in paired))
    starts_complete = all(any(row["direction"] == sign and not row["censored"]
                              for row in breakaway_intervals) for sign in (1, -1))
    selection = config.get("signal_selection", {})
    current_factor = int(selection.get("factor_index", 0))
    decision = {"method": "first_stimulus", "current_factor_index": current_factor,
                "sufficient_for_start_intervals": starts_complete,
                "full_plant_identification_sufficient": False,
                "basis": "Both signed monotonic upward ramps must supply uncensored intervals from the existing sustained-motion estimator."}
    if selection.get("method") == "choose_information":
        decision["method"] = "choose_information"
        decision["next_action"] = "CONTINUE_MEASURED_INFORMATION_ANALYSIS"
    elif not starts_complete:
        next_factor = current_factor+1
        decision["next_factor_index"] = next_factor
        decision["next_amplitude_A"] = float(first_stimulus(
            float(selection["approved_minimum_A"]), SimpleNamespace(current_a=float(config["yaw_current_bound_A"])),
            previous_factor=next_factor))
        decision["next_action"] = "ACQUIRE_FIXED_NEXT_FACTOR_SIGNED_RAMPS"
    else:
        decision["next_action"] = "CONTINUE_PROGRAM_INFORMATION_SELECTION"
    return {"schema": "adr0022.yaw-baseline-observations/1", "provenance": config["provenance"],
        "source_journal": str(journal.resolve()), "capture_footer": rows[-1],
        "baseline_window": {"start_ns": baseline_start, "excitation_begin_ns": excitation,
                            "requested_window_s": 2.0, "selection": "last two seconds before excitation"},
        "yaw_baseline": yaw_window(baseline_yaw),
        "encoder_quantization": {"counts_per_revolution": 8192, "radians_per_count": 2*math.pi/8192,
                                 "degrees_per_count": 360/8192, "uniform_quantization_sigma_rad": 2*math.pi/8192/math.sqrt(12)},
        "motor_rpm_quantization_rad_s": 2*math.pi/60,
        "encoder_velocity_observation": {"method": "causal local linear regression on actual encoder receipt timestamps",
            "window_s": rate_window_s, "window_source": "existing estimate_interval sustained_s default",
            "baseline_rate_rad_s": describe(baseline_rates), "measured_rate_sigma_rad_s": observed_rate_sigma,
            "quantization_regression_sigma_rad_s": quantization_rate_sigma,
            "estimator_sigma_velocity_rad_s": rate_sigma,
            "uncertainty_interpretation": "Use measured baseline velocity scatter; if that scatter is exactly zero, use encoder quantization propagated through the actual local regression time grid."},
        "pitch_actual_mechpos_rad": describe(row["value"] for row in baseline_pitch),
        "pitch_full_capture_mechpos_rad": describe(row["value"] for row in pitch),
        "sensor_frame_baseline": sensors,
        "sensor_bias_interpretation": "Gyro component means are observed baseline offsets in the sensor frame. Acceleration means include gravity and installation attitude; they are not accelerometer bias calibration. No calibrated yaw-axis claim is made.",
        "can_baseline_timing": {axis: timing([row for row in baseline_receipts if row["axis"] == axis], "kernel_monotonic_ns") for axis in ("yaw", "pitch")},
        "can_clock_bracket_uncertainty_ns": describe(row["clock_uncertainty_ns"] for row in baseline_receipts),
        "can_dequeue_delay_s": describe((row["dequeue_ns"]-row["kernel_monotonic_ns"])/1e9 for row in baseline_receipts),
        "current_mapping_observation": {"reported_A_per_raw_count": 3/16384,
            "last_successful_tx_vs_reported_A_regression": mapping,
            "paired_samples": len(paired), "last_tx_age_s": describe(row["last_tx_age_s"] for row in paired),
            "interpretation": "Reported amperes use the existing protocol conversion. The same-receipt command association is descriptive; it does not calibrate true current or estimate actuator delay."},
        "segments": segments, "directional_start_observations": directional,
        "breakaway_interval_observations": breakaway_intervals,
        "continuous_startup_interval_observations": continuous_starts,
        "initial_stimulus_decision": decision,
        "fixed_stop_observation": {"stop_begin_ns": stop, "duration_s": 2.0,
            "observed": yaw_window(stop_yaw),
            "actual_zero_current_tx_A": describe(row["successful_tx_A"] for row in tx if row["phase"] == "stop")},
        "qualification": {"dynamics": False, "current_mapping": False, "stopping": False,
                          "breakaway": False, "sustained_motion": False},
        "timing_interpretation": "Motor feedback timestamps are actual kernel receipt times, not motor sample times. IMU host sample/receipt and native SH2 intervals are reported separately; asynchronous streams are not resampled or assigned a fabricated common rate."}


def quaternion_angle(a, b):
    norm_a = math.sqrt(sum(value*value for value in a))
    norm_b = math.sqrt(sum(value*value for value in b))
    dot = sum(x*y for x, y in zip(a, b))/(norm_a*norm_b)
    return 2*math.acos(min(1.0, abs(dot)))


def gyro_rotation(samples, bias):
    integral = [0.0, 0.0, 0.0]
    integrated_s = 0.0
    for a, b in zip(samples, samples[1:]):
        if a["generation"] != b["generation"]:
            continue
        dt = (b["sample_ns"]-a["sample_ns"])/1e9
        if dt <= 0:
            continue
        integrated_s += dt
        for axis in range(3):
            integral[axis] += ((a["values"][axis]+b["values"][axis])/2-bias[axis])*dt
    return {"samples": len(samples), "integrated_s": integrated_s,
            "sensor_frame_rotation_vector_rad": integral,
            "rotation_vector_norm_rad": math.sqrt(sum(value*value for value in integral))}


def analyze_movement(journal: Path, manifest: Path | None = None) -> dict:
    """Postcapture encoder and independent body-rotation observations, no arming gate."""
    baseline = analyze_baseline(journal, manifest)
    journal, config, rows = load_capture(journal, manifest)
    yaw = [row for row in rows if row.get("kind") == "yaw_feedback"]
    excitation = baseline["baseline_window"]["excitation_begin_ns"]
    baseline_start = baseline["baseline_window"]["start_ns"]
    stop = baseline["fixed_stop_observation"]["stop_begin_ns"]
    window_s = baseline["encoder_velocity_observation"]["window_s"]
    sigma_velocity = baseline["encoder_velocity_observation"]["estimator_sigma_velocity_rad_s"]
    quantum = baseline["encoder_quantization"]["radians_per_count"]
    stamps = [row["kernel_monotonic_ns"] for row in yaw]
    motion_end = stamps[-1]
    velocities = []
    for index, row in enumerate(yaw):
        first = bisect_right(stamps, stamps[index]-int(window_s*1e9))
        local = yaw[first:index+1]
        times = [(value["kernel_monotonic_ns"]-stamps[index])/1e9 for value in local]
        velocities.append(regression(times, (value["q_relative_rad"] for value in local)).get("slope") or 0.0)
    samples = []
    for row in rows:
        if row.get("kind") == "imu_raw":
            sample = json.loads(row["raw_json"])
            if sample.get("kind") == "sample":
                samples.append(sample)
    sensor_samples = {sensor: [row for row in samples if row["sensor"] == sensor]
                      for sensor in ("gyro", "rv", "game_rv")}
    gyro_bias = [component["raw_values"]["mean"] for component in baseline["sensor_frame_baseline"]["gyro"]["components"]]
    gyro_sigma_norm = math.sqrt(sum(component["raw_values"]["standard_deviation"]**2
                                  for component in baseline["sensor_frame_baseline"]["gyro"]["components"]))
    quaternion_noise = {}
    for sensor in ("rv", "game_rv"):
        selected = [row for row in sensor_samples[sensor] if baseline_start <= row["sample_ns"] < excitation]
        times = [row["sample_ns"] for row in selected]
        rates = []
        for index, row in enumerate(selected):
            last = bisect_right(times, row["sample_ns"]+int(window_s*1e9))-1
            if last > index and selected[last]["generation"] == row["generation"]:
                dt = (times[last]-times[index])/1e9
                rates.append(quaternion_angle(row["values"], selected[last]["values"])/dt)
        stats = describe(rates)
        quaternion_noise[sensor] = {"baseline_window_rotation_rate_rad_s": stats,
            "observation_level_rad_s": stats.get("mean", 0)+3*stats.get("standard_deviation", 0)}

    def body_observation(first_ns, last_ns):
        observations = {}
        gyro = [row for row in sensor_samples["gyro"] if first_ns <= row["sample_ns"] <= last_ns]
        rotation = gyro_rotation(gyro, gyro_bias)
        rotation["baseline_sigma_vector_norm_rad_s"] = gyro_sigma_norm
        rotation["above_baseline_scatter"] = rotation["integrated_s"] > 0 and rotation["rotation_vector_norm_rad"] > 3*gyro_sigma_norm*rotation["integrated_s"]
        rotation["accuracy_status_counts"] = dict(Counter(str(row["status"]) for row in gyro))
        observations["gyro"] = rotation
        for sensor in ("rv", "game_rv"):
            selected = [row for row in sensor_samples[sensor] if first_ns <= row["sample_ns"] <= last_ns]
            generation = selected[0]["generation"] if selected else None
            same = [row for row in selected if row["generation"] == generation]
            dt = (same[-1]["sample_ns"]-same[0]["sample_ns"])/1e9 if len(same) > 1 else 0
            angle = quaternion_angle(same[0]["values"], same[-1]["values"]) if dt else 0
            observations[sensor] = {"samples": len(same), "actual_span_s": dt,
                "first_sample_ns": same[0]["sample_ns"] if same else None,
                "last_sample_ns": same[-1]["sample_ns"] if same else None,
                "relative_orientation_angle_rad": angle,
                "rotation_rate_rad_s": angle/dt if dt else None,
                "above_baseline_scatter": dt > 0 and angle/dt > quaternion_noise[sensor]["observation_level_rad_s"],
                "accuracy_status_counts": dict(Counter(str(row["status"]) for row in same))}
        return observations

    candidates = []
    for direction in (1, -1):
        active = []
        runs = []
        for index, row in enumerate(yaw):
            moving = excitation <= stamps[index] <= motion_end and direction*velocities[index] > 3*sigma_velocity
            if moving:
                active.append(index)
            elif active:
                runs.append(active)
                active = []
        if active:
            runs.append(active)
        for run in runs:
            first, last = run[0], run[-1]
            duration = (stamps[last]-stamps[first])/1e9
            displacement = yaw[last]["q_relative_rad"]-yaw[first]["q_relative_rad"]
            if duration < window_s or direction*displacement <= max(quantum, 3*sigma_velocity*window_s):
                continue
            # Encoder regression is causal. Its supporting data starts one
            # regression window earlier; use that same actual-time support for IMU.
            support_start = max(excitation, stamps[first]-int(window_s*1e9))
            body = body_observation(support_start, stamps[last])
            confirmed = any(value["above_baseline_scatter"] for value in body.values())
            candidates.append({"direction": direction, "first_encoder_motion_ns": stamps[first],
                "last_encoder_motion_ns": stamps[last], "encoder_motion_duration_s": duration,
                "recorded_zero_observation_motion_s": max(0.0, (stamps[last]-max(stamps[first], stop))/1e9),
                "encoder_displacement_rad": displacement,
                "encoder_regression_velocity_rad_s": describe(velocities[index] for index in run),
                "independent_imu_support_start_ns": support_start,
                "independent_body_rotation": body, "body_rotation_observed_in_same_time_support": confirmed})
    confirmed = [row for row in candidates if row["body_rotation_observed_in_same_time_support"]]
    directional = {str(sign): {"encoder_sustained_intervals": sum(row["direction"] == sign for row in candidates),
                              "independently_observed_body_intervals": sum(row["direction"] == sign for row in confirmed)}
                   for sign in (1, -1)}
    expected_directions = {1 if float(segment["end_A"]) > 0 else -1 for segment in config["current_segments"] if float(segment["end_A"]) != 0}
    directions_observed = all(directional[str(sign)]["independently_observed_body_intervals"] > 0 for sign in expected_directions)
    return {"schema": "adr0022.yaw-movement-evidence/1", "provenance": config["provenance"],
        "source_journal": str(journal.resolve()), "acquisition_complete": rows[-1]["status"] == "COMPLETE",
        "encoder_only_sustained_motion_observed": bool(candidates),
        "independent_body_rotation_observed": bool(confirmed),
        "requested_directions_observed": directions_observed,
        "acquisition_information_result": "OBSERVED_SUSTAINED_BODY_ROTATION" if confirmed else "ACQUISITION_INSUFFICIENT_SUSTAINED_BODY_ROTATION",
        "stage3_ready": False, "stage3_readiness_reason": "Requires identified model and independent holdout validation.",
        "motion_observation_window": {"start_ns": excitation, "final_zero_begin_ns": stop,
            "last_actual_yaw_receipt_ns": motion_end,
            "includes_final_zero_current_observation": True,
            "recorded_zero_observation_span_s": (motion_end-stop)/1e9},
        "encoder_uncertainty": baseline["encoder_velocity_observation"],
        "gyro_sensor_frame_bias_rad_s": gyro_bias, "gyro_baseline_sigma_norm_rad_s": gyro_sigma_norm,
        "quaternion_baseline_observations": quaternion_noise,
        "sustained_motion_intervals": candidates, "directional_observations": directional,
        "independently_observed_motion_duration_s": sum(row["encoder_motion_duration_s"] for row in confirmed),
        "independently_observed_encoder_coverage_rad": sum(abs(row["encoder_displacement_rad"]) for row in confirmed),
        "whole_excitation_body_rotation": body_observation(excitation, stop),
        "whole_excitation_scope": {"start_ns": excitation, "end_ns": stop,
                                   "includes_final_zero_current_observation": False},
        "sensor_actual_timing": {sensor: {"sample_timing": timing(selected, "sample_ns"),
             "receipt_timing": timing(selected, "rx_ns"),
             "accuracy_status_counts": dict(Counter(str(row["status"]) for row in selected)),
             "generation_counts": dict(Counter(str(row["generation"]) for row in selected))}
             for sensor, selected in sensor_samples.items()},
        "interpretation": "Postcapture evidence uses measured baseline scatter, encoder quantization and the existing 60 ms sustained-motion convention. Independent gyro/orientation observations share the actual time support of encoder motion. Sensor coordinates are not calibrated yaw coordinates. No travel minimum or arming gate is introduced, and a fitted load column is not used as motion proof."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, help="optional: otherwise use the journal header manifest")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline", action="store_true", help="analyze pre-excitation baseline and directional observations")
    parser.add_argument("--movement", action="store_true", help="report encoder and independent raw IMU movement evidence")
    args = parser.parse_args()
    result = analyze_movement(args.journal, args.manifest) if args.movement else analyze_baseline(args.journal, args.manifest) if args.baseline else summarize(args.journal, args.manifest)
    serialized = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized)
    print(serialized, end="")


if __name__ == "__main__":
    main()
