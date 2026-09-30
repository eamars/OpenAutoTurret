"""Derive immutable Step 2 capability observations from a complete baseline.

No motor I/O or parameter fitting. Unexcited baseline data cannot become a PlantSnapshot.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import statistics

from adr0022_capture_review import need, review, strict_json


def identity(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def summarize(values):
    need(bool(values) and all(math.isfinite(v) for v in values), "finite observations required")
    return {"count": len(values), "min": min(values), "max": max(values),
            "mean": statistics.fmean(values),
            "sample_std": statistics.stdev(values) if len(values) > 1 else None}


def derive(capture, attempt, manifest, bound):
    report = review(capture)
    receipt = strict_json(Path(attempt).read_text(encoding="utf-8"))
    need(receipt.get("schema") == "adr0022.capture_attempt/1" and receipt["provenance"] == report["provenance"],
         "capture attempt identity/provenance differs")
    need(receipt.get("motion_requested") is False, "baseline must not contain excitation")
    rows = [strict_json(line) for line in Path(capture).read_text(encoding="utf-8").splitlines()]
    original = strict_json(Path(manifest).read_text(encoding="utf-8"))
    bound_config = strict_json(Path(bound).read_text(encoding="utf-8"))
    need(hashlib.sha256(Path(manifest).read_bytes()).hexdigest() == receipt["manifest_sha256"] and
         receipt["binaries"] == original["expected_binaries"] and
         {k: v for k, v in bound_config.items() if k != "imu_fd"} == original,
         "manifest/binary binding differs from attempt")
    def yaml_scalars(value):
        # C++ retains JSON scalar spellings and emits all YAML scalars quoted.
        if isinstance(value, dict): return {k: yaml_scalars(v) for k, v in value.items()}
        if isinstance(value, list): return [yaml_scalars(v) for v in value]
        if value is None: return "null"
        if type(value) is bool: return str(value).lower()
        return str(value)
    need(strict_json(rows[0]["manifest_yaml"]) == yaml_scalars(bound_config), "capture header does not match bound manifest")
    need(report["pitch_uid_observed"] == original["expected_pitch_uid"], "observed UID differs from manifest")
    registers = {}
    for index in sorted({r["index"] for r in rows if r["kind"] == "register_read"}):
        reads = [r for r in rows if r["kind"] == "register_read" and r["index"] == index]
        registers[hex(index)] = {"observed": summarize([r["value"] for r in reads]),
                                "context": "PITCH_DISABLED_BASELINE", "sample_time_calibrated": False,
                                "response_interval_s": summarize([(r["receive_ns"]-r["request_begin_ns"])*1e-9 for r in reads])}
    encoders = {}
    for axis in ("yaw", "pitch"):
        frames = [r for r in rows if r["kind"] == "can_rx" and r["axis"] == axis and "angle_raw" in r]
        encoders[axis] = {"raw_count": summarize([r["angle_raw"] for r in frames]),
                          "physical_zero_qualified": False, "direction_scale_qualified": False}
    imu = [strict_json(r["raw_json"]) for r in rows if r["kind"] == "imu_raw"]
    gyro = [r["values"] for r in imu if r.get("sensor") == "gyro"]
    gyro_observations = {"coordinate": "RAW_SENSOR_FRAME", "unit": "rad/s",
                         "axes": [summarize([v[i] for v in gyro]) for i in range(3)],
                         "bias_qualified": False, "mounting_qualified": False,
                         "meaning": "observed scatter; not independent-sample uncertainty or a mounted-axis calibration"}
    # Compare the documented finite type-2 mapping with independent register
    # observations only at this pose. One pose cannot identify a scale or sign.
    pitch = [r for r in rows if r["kind"] == "can_rx" and r["axis"] == "pitch" and "angle_raw" in r]
    times = [r["kernel_monotonic_ns"] for r in pitch]
    differences, separations = [], []
    for row in [r for r in rows if r["kind"] == "register_read" and r["index"] == 0x7019]:
        at = bisect.bisect_left(times, row["receive_ns"])
        candidates = [i for i in (at-1, at) if 0 <= i < len(times)]
        i = min(candidates, key=lambda i: abs(times[i]-row["receive_ns"]))
        differences.append(row["value"] - (-12.5 + pitch[i]["angle_raw"]*25/65535))
        separations.append(abs(times[i]-row["receive_ns"])*1e-9)
    encoder_comparison = {"scale_sign_range_qualified": False, "source": "NEAREST_HOST_RECEIPT_AT_BASELINE_POSE"}
    if differences:
        encoder_comparison.update(register_minus_type2_rad=summarize(differences),
                                  receipt_separation_s=summarize(separations))
    pending = [
        {"item": "pitch_current_mode", "reason": "MEASUREMENT_LIMITED",
         "next_operation": "Verify mode 3 and zero IqRef/readback under the sole owner, before enabling"},
        {"item": "yaw_current_feedback_scale", "reason": "MEASUREMENT_LIMITED",
         "next_operation": "Bind compatible existing current-interface evidence and qualify feedback scale; no command-scale substitution"},
        {"item": "stop_and_physical_envelope", "reason": "MEASUREMENT_LIMITED",
         "next_operation": "Qualify bounded motion/settling and actual zero/travel relationship using declared manual cutoff conditions"},
        {"item": "imu_mount_clock_filter", "reason": "INSUFFICIENT_EXCITATION",
         "next_operation": "Collect independent-axis calibrated motion and matched timing evidence using the existing Stage 1 estimator"},
        {"item": "plant_parameters", "reason": "INSUFFICIENT_EXCITATION",
         "next_operation": "Collect prescribed directional/posture excitation, breakaway and whole-run holdouts; call existing identifier and solver"}]
    return {"schema": "adr0022.baseline_capabilities/1", "provenance": report["provenance"],
            "source": {"capture_sha256": report["capture_sha256"], "attempt_sha256": hashlib.sha256(Path(attempt).read_bytes()).hexdigest(),
                       "binaries": receipt["binaries"], "manifest_sha256": receipt["manifest_sha256"],
                       "reviewer_sha256": hashlib.sha256(Path(__file__).with_name("adr0022_capture_review.py").read_bytes()).hexdigest(),
                       "extractor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
            "capture_integrity": "PASS", "pitch_uid": report["pitch_uid_observed"],
            "streams": report["streams"], "temperatures": report["temperatures"],
            "current_units": report["current_units"], "registers": registers,
            "unavailable_registers": report["measurement_limitations"],
            "encoders": encoders, "gyro_observations": gyro_observations,
            "pitch_encoder_comparison": encoder_comparison,
            "applicability": "This capture, operating point and disabled pitch context only; no extrapolation to enabled/current-mode behavior",
            "pending_parameters": {key: None for key in ("inertia", "velocity_resistance", "directional_load", "breakaway", "actuation_delay", "mounting_rotation")},
            "plant_snapshot": None, "controller_candidate": None, "physical_parameters_qualified": False,
            "next_steps": pending}


def write_asset(directory, document):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (identity(document) + ".json")
    encoded = json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if path.exists():
        need(path.read_text(encoding="utf-8") == encoded, "immutable capability asset collision")
    else:
        with path.open("x", encoding="utf-8", newline="\n") as output:
            output.write(encoded)
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--attempt", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--bound", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    asset = derive(args.capture, args.attempt, args.manifest, args.bound)
    print(write_asset(args.output, asset))
