"""Preserve zero-command measurements as immutable calibration candidates.

No device I/O. Hardware scatter, quantization and offsets are observations, not
reasons to discard data. A zero-command Iqf mean does not establish sensor bias.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

from adr0022_baseline_assets import identity, summarize, write_asset
from adr0022_capture_review import need, strict_json
from adr0022_current_review import review_characterization


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def yaml_scalars(value):
    if isinstance(value, dict):
        return {k: yaml_scalars(v) for k, v in value.items()}
    if isinstance(value, list):
        return [yaml_scalars(v) for v in value]
    if value is None:
        return "null"
    if type(value) is bool:
        return str(value).lower()
    return str(value)


def scatter(values):
    result = summarize(values)
    result["sample_variance"] = statistics.variance(values) if len(values) > 1 else None
    mean = result["mean"]
    centered = [v-mean for v in values]
    denominator = sum(v*v for v in centered)
    result["lag1_observation_autocorrelation"] = (
        sum(a*b for a,b in zip(centered,centered[1:]))/denominator if denominator else None)
    result["autocorrelation_meaning"] = "Adjacent observations; irregular cadence and device filtering are retained."
    return result


def derive(capture, attempt, manifest, bound):
    report = review_characterization(capture)
    receipt = strict_json(Path(attempt).read_text(encoding="utf-8"))
    original = strict_json(Path(manifest).read_text(encoding="utf-8"))
    configured = strict_json(Path(bound).read_text(encoding="utf-8"))
    rows = [strict_json(line) for line in Path(capture).read_text(encoding="utf-8").splitlines()]
    need(receipt.get("schema") == "adr0022.neutral_characterization_attempt/1" and
         receipt.get("provenance") == report["provenance"] and receipt.get("motion_requested") is False and
         receipt.get("nonzero_current_requested") is False,
         "zero-command attempt identity/provenance differs")
    need(sha(manifest) == receipt["manifest_sha256"] and receipt["binaries"] == original["expected_binaries"] and
         {k:v for k,v in configured.items() if k != "imu_fd"} == original and
         strict_json(rows[0]["manifest_yaml"]) == yaml_scalars(configured),
         "capture/manifest/binary binding differs")
    need(report["pitch_uid_observed"] == original["expected_pitch_uid"], "pitch identity differs")
    observations = report["current_observations"]
    first, last = observations[0]["receive_ns"], observations[-1]["receive_ns"]
    # Analyze the actual observed zero-command window, with no rolling-tail cut.
    encoder = {}
    for axis, quantum in (("yaw",2*math.pi/8192),("pitch",25/65535)):
        selected = [(i,r) for i,r in enumerate(rows) if r.get("kind") == "can_rx" and
                    r.get("axis") == axis and "angle_raw" in r and first <= r["kernel_monotonic_ns"] <= last]
        counts = [r["angle_raw"] for _,r in selected]
        need(counts, "zero-command encoder observations missing")
        encoder[axis] = {"raw_count": scatter(counts), "capture_row_range": [selected[0][0],selected[-1][0]],
            "protocol_quantum_rad": quantum, "protocol_bin_half_width_rad": quantum/2,
            "empirical_variance_is_quantized": True, "zero_empirical_variance_is_valid": True,
            "output_ratio_verified": False, "output_frame_verified": False,
            "uncertainty_meaning": "Protocol angle bin only; physical output ratio, scale/sign and frame require measured mapping.",
            "observer_covariance": None}
    gyro = [(i,strict_json(r["raw_json"])) for i,r in enumerate(rows)
            if r.get("kind") == "imu_raw" and first <= r["dequeue_ns"] <= last]
    gyro = [(i,r) for i,r in gyro if r.get("kind") == "sample" and r.get("sensor") == "gyro"]
    need(gyro, "zero-command gyro observations missing")
    statuses = {}
    for _,r in gyro:
        statuses[str(r["status"])] = statuses.get(str(r["status"]),0)+1
    axes = [scatter([r["values"][axis] for _,r in gyro]) for axis in range(3)]
    gyro_candidate = {"coordinate": "RAW_SENSOR_FRAME", "unit": "rad/s", "axes": axes,
        "window_selection_clock":"IMU_CAPTURE_DEQUEUE_MONOTONIC_NS",
        "capture_row_range": [gyro[0][0],gyro[-1][0]], "status_histogram": statuses,
        "generation_observed": sorted({r["generation"] for _,r in gyro}),
        "stationary_assessment": "Zero command and bounded encoder observations; sub-bin motion remains unresolved.",
        "bias_candidate_rad_s": [a["mean"] for a in axes],
        "bias_applied": False, "mounting_rotation": None, "axis_velocity_noise_variance": None}
    currents = [o["value_A"] for o in observations]
    return {"schema": "adr0022.neutral_observations/1", "provenance": report["provenance"],
        "capture_integrity": "PASS", "pitch_uid": report["pitch_uid_observed"],
        "source": {"capture_sha256": report["capture_sha256"], "attempt_sha256":sha(attempt),
            "manifest_sha256":sha(manifest), "bound_manifest_sha256":sha(bound),
            "binaries":receipt["binaries"], "revision":receipt.get("expected_revision"),
            "source_sha256":receipt.get("source_sha256"),
            "reviewer_sha256":sha(Path(__file__).with_name("adr0022_current_review.py")),
            "extractor_sha256":sha(__file__), "independent_review_sha256":identity(report)},
        "observation_window_ns": [first,last], "commanded_current_A":0,
        "plant_input_semantics": "SUCCESSFUL_CURRENT_COMMAND_A_ZERO_ORDER_HOLD",
        "iqf": {"documented_unit":"A", "statistics":scatter(currents),
            "stream":report["streams"]["pitch_iqf"],
            "host_transaction_latency_s":summarize([(o["receive_ns"]-o["request_accepted_ns"])*1e-9 for o in observations]),
            "observation_request_sequence_range":[observations[0]["request_sequence"],observations[-1]["request_sequence"]],
            "feedback_sensor_bias_A":None, "bias_correction_applied":False, "scale_calibrated":False,
            "meaning":"Observed zero-command feedback response. Mean can contain actual current, sensor bias, filtering and transients; it is not a sensor-bias estimate.",
            "device_sample_time_calibrated":False, "filter_tau_s":None},
        "encoders":encoder, "gyro_calibration_candidates":gyro_candidate,
        "temperatures":report["temperatures"],
        "historical_diagnostic_comparison":{"bound_A":original["neutral_current_bound_A"],
            "satisfied":report["neutral_current_criterion_satisfied"], "rejects_observations":False},
        "adaptation_policy":"Retain measured imperfections; use calibrated mapping, noise/quantization uncertainty and identified load/dynamics in normalization, estimation and controller synthesis.",
        "pending_components":["Mounted gyro bias/axis-rate calibration from independent motion directions",
            "Measured encoder-to-mechPos scale/sign/output ratio/frame", "Device timing, causal response and filter bandwidth",
            "Synchronized dynamic observer process variance", "Nonzero successful-command motion runs and independent whole-run holdouts"],
        "physical_parameters_qualified":False, "plant_snapshot":None, "controller_candidate":None}


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("capture","attempt","manifest","bound","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    args=parser.parse_args()
    print(write_asset(args.output,derive(args.capture,args.attempt,args.manifest,args.bound)))
