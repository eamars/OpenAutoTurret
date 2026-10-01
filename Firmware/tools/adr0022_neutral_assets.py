"""Preserve current/encoder/IMU observations as unqualified calibration candidates.

No device I/O, hashing, ideal neutral-current comparison or observation rejection.
An Iqf mean remains an observed response rather than a sensor-bias correction.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import statistics
from adr0022_capture_review import strict_json
from adr0022_current_review import review_characterization, summary


def scatter(values):
    values = list(values)
    result = summary(values)
    centered = [v-result["mean"] for v in values] if values else []
    denominator = sum(v*v for v in centered)
    result["lag1_observation_autocorrelation"] = sum(a*b for a,b in zip(centered,centered[1:]))/denominator if denominator else None
    result["autocorrelation_meaning"] = "Adjacent observations; irregular cadence and device filtering are retained."
    return result


def derive(capture, attempt, manifest, bound):
    report = review_characterization(capture)
    receipt = strict_json(Path(attempt).read_text(encoding="utf-8"))
    original = strict_json(Path(manifest).read_text(encoding="utf-8"))
    configured = strict_json(Path(bound).read_text(encoding="utf-8"))
    rows = [strict_json(line) for line in Path(capture).read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    observations = report["current_observations"]
    times = [o["receive_ns"] for o in observations if type(o.get("receive_ns")) is int]
    first = min(times) if times else report.get("start_ns")
    last = max(times) if times else report.get("end_ns")
    def in_window(time):
        return type(time) is int and (first is None or time >= first) and (last is None or time <= last)
    encoder = {}
    for axis, quantum in (("yaw",2*math.pi/8192),("pitch",25/65535)):
        selected = []
        for i, row in enumerate(rows):
            if row.get("kind") != "can_rx" or row.get("axis") != axis or not in_window(row.get("kernel_monotonic_ns")):
                continue
            payload = row.get("bytes", [])
            if len(payload) != 8 or row.get("error") or row.get("rtr"):
                continue
            if axis == "pitch" and ((row.get("id",0) >> 24) & 31) != 2:
                continue
            if axis == "yaw" and row.get("id") != 0x205:
                continue
            selected.append((i, int.from_bytes(bytes(payload[:2]), "big")))
        counts = [count for _,count in selected]
        encoder[axis] = {"raw_count": scatter(counts),
            "capture_row_range": [selected[0][0],selected[-1][0]] if selected else None,
            "protocol_quantum_rad": quantum, "protocol_bin_half_width_rad": quantum/2,
            "empirical_variance_is_quantized": True, "zero_empirical_variance_is_valid": True,
            "output_ratio_verified": False, "output_frame_verified": False,
            "uncertainty_meaning": "Protocol angle bin; output ratio, scale/sign and frame still require measured mapping.",
            "observer_covariance": None}
    gyro = [(i,strict_json(row["raw_json"])) for i,row in enumerate(rows)
            if row.get("kind") == "imu_raw" and in_window(row.get("dequeue_ns"))]
    gyro = [(i,raw) for i,raw in gyro if raw.get("kind") == "sample" and raw.get("sensor") == "gyro"]
    axes = [scatter(raw["values"][axis] for _,raw in gyro if len(raw.get("values",[])) > axis) for axis in range(3)]
    complete = [raw["values"] for _,raw in gyro if len(raw.get("values",[])) == 3]
    covariance = None
    if len(complete)>1:
        means = [statistics.fmean(v[axis] for v in complete) for axis in range(3)]
        covariance = [[sum((v[a]-means[a])*(v[b]-means[b]) for v in complete)/(len(complete)-1)
                       for b in range(3)] for a in range(3)]
    gyro_candidate = {"coordinate": "RAW_SENSOR_FRAME", "unit": "rad/s", "axes": axes,
        "sample_covariance_sensor_rad2_s2": covariance,
        "window_selection_clock":"IMU_CAPTURE_DEQUEUE_MONOTONIC_NS",
        "capture_row_range": [gyro[0][0],gyro[-1][0]] if gyro else None,
        "status_histogram": {str(status):sum(raw.get("status")==status for _,raw in gyro)
                             for status in sorted({raw.get("status") for _,raw in gyro if type(raw.get("status")) is int})},
        "generation_observed": sorted({raw.get("generation") for _,raw in gyro if type(raw.get("generation")) is int}),
        "stationary_assessment": "Commands and encoder observations retained; physical stillness and sub-bin motion remain unresolved.",
        "bias_candidate_rad_s": [axis["mean"] for axis in axes], "bias_applied": False,
        "bias_qualified": False, "mounting_rotation": None, "axis_velocity_noise_variance": None}
    currents = [o["value_A"] for o in observations]
    references = [c["value"] for c in report["commands"] if c["axis"] == "pitch" and c["kind"] == 18 and
                  c.get("index") == 0x7006 and c["success"] is True]
    zero_command = bool(references) and all(value == 0 for value in references)
    latency = [(o["receive_ns"]-o["request_accepted_ns"])/1e9 for o in observations
               if type(o.get("request_accepted_ns")) is int and type(o.get("receive_ns")) is int]
    return {"schema": "adr0022.neutral_observations/1", "provenance": report["provenance"],
        "capture_integrity": report["capture_integrity"], "capture_complete": report["capture_complete"],
        "capture_footer_status": report["capture_footer_status"], "capture_footer_detail": report["capture_footer_detail"],
        "pitch_uid": report["pitch_uid_observed"],
        "source": {"capture_path":str(capture), "attempt_path":str(attempt), "manifest_path":str(manifest),
            "bound_manifest_path":str(bound), "attempt_schema":receipt.get("schema"),
            "manifest_schema":original.get("schema"), "bound_manifest_schema":configured.get("schema")},
        "observation_window_ns": [first,last], "commanded_current_A":0 if zero_command else None,
        "zero_current_command_observed": zero_command, "successful_current_reference_observations_A":references,
        "plant_input_semantics": "SUCCESSFUL_CURRENT_COMMAND_A_ZERO_ORDER_HOLD",
        "iqf": {"documented_unit":"A", "statistics":scatter(currents), "stream":report["streams"]["pitch_iqf"],
            "host_transaction_latency_s":summary(latency),
            "observation_request_sequence_range":[observations[0].get("request_sequence"),observations[-1].get("request_sequence")] if observations else None,
            "feedback_sensor_bias_A":None, "bias_correction_applied":False, "scale_calibrated":False,
            "meaning":"Observed feedback response; mean can contain actual current, sensor bias, filtering and transients.",
            "device_sample_time_calibrated":False, "filter_tau_s":None},
        "encoders":encoder, "gyro_calibration_candidates":gyro_candidate,
        "temperatures":report["temperatures"], "drive_faults":report["drive_faults"],
        "stop_observations":report["stop_observations"], "final_stop_confirmed":report["final_stop_confirmed"],
        "loss_observations":report["loss_observations"], "interface_loss_deltas":report["interface_loss_deltas"],
        "adaptation_policy":"Retain measured observations for calibration, estimation and controller synthesis.",
        "pending_components":["Mounted gyro calibration from independent motion directions",
            "Measured encoder-to-mechPos scale/sign/output ratio/frame", "Device timing, causal response and filter bandwidth",
            "Synchronized dynamic observer process variance", "Motion runs and independent whole-run holdouts"],
        "calibration_qualified":False, "protection_qualified":False, "controller_qualified":False,
        "physical_parameters_qualified":False, "physical_capabilities_qualified":False,
        "plant_snapshot":None, "controller_candidate":None}


def write_asset(output, document):
    path = Path(output)
    if path.suffix != ".json":
        path.mkdir(parents=True, exist_ok=True)
        path = path / "neutral-observations.json"
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as file:
        json.dump(document, file, indent=2, sort_keys=True, allow_nan=False)
        file.write("\n")
    return path


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("capture","attempt","manifest","bound","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    args=parser.parse_args()
    print(write_asset(args.output,derive(args.capture,args.attempt,args.manifest,args.bound)))
