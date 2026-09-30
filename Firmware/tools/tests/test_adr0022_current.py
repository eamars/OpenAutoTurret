"""Review real neutral C++ captures and reject unsupported/forged claims."""
import copy
import json
import math
import os
from pathlib import Path
import struct
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_current_review import IQF, IQREF, MODE, review, review_characterization
from adr0022_current_rehearsal import rehearse

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real Linux C++/UDP/pipe capture")
ROOT = TOOLS.parents[1]
BINARY = Path(os.environ.get("OTA_STAGE1_BUILD", ROOT / "run/adr0022-local/firmware")) / "axis_control_core/commissiond"


@pytest.fixture(scope="module")
def successful(tmp_path_factory):
    directory = tmp_path_factory.mktemp("current-review") / "normal"
    result = rehearse(BINARY, directory)
    return directory, result


def load(directory):
    return [json.loads(line) for line in (directory / "capture.jsonl").read_text().splitlines()]


def save(tmp_path, rows):
    path = tmp_path / "altered.jsonl"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return path


def is_pitch(row, kind):
    return row["kind"] == "can_rx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == kind


def test_independent_review_derives_neutral_scope_from_real_capture(successful):
    directory, result = successful
    report = review(directory / "capture.jsonl")
    assert result["returncode"] == 0 and report["capture_complete"]
    assert report["capability_scope"] == "neutral_transition_only" and report["neutral_transition_verified"]
    assert report["original_mode"] == report["restored_mode"] == 2 and report["normal_stop_confirmed"]
    assert report["pitch_current_max_abs_A"] == 0 and report["streams"]["pitch_iqf"]["count"] > 100
    assert report["streams"]["pitch_iqf"]["device_sample_ns"] is None
    assert not report["streams"]["pitch_iqf"]["sample_clock_calibrated"]
    assert report["current_units"]["yaw"]["scale_A_per_count"] is None
    assert report["temperatures"]["yaw"]["Celsius_mapping"] == "UNKNOWN"
    assert not report["neutral_transition_qualified"] and not report["physical_capabilities_qualified"]
    assert not report["physical_parameters_qualified"] and not report["motion_authorized"]
    assert all(not report["streams"][sensor]["mounting_calibrated"] for sensor in ("accel", "gyro", "rv", "game_rv"))


def test_write_echo_is_retained_without_becoming_readback(tmp_path):
    directory = tmp_path / "echo"
    rehearse(BINARY, directory, fault="write_echo")
    report = review(directory / "capture.jsonl")
    assert report["capture_complete"] and report["write_echoes_ignored"] > 0
    assert report["register_reads"] > 100


def test_labels_and_legacy_yaw_current_cannot_invent_a_physical_claim(tmp_path, successful):
    rows = load(successful[0])
    for row in rows:
        if row["kind"] == "preparation_state":
            row["state"] = -1
            row["name"] = "FORGED_COMPLETE"
        elif row["kind"] == "can_rx" and row["axis"] == "yaw":
            row["current_A"] = 500
    report = review(save(tmp_path, rows))
    assert report["capture_complete"] and not report["neutral_transition_qualified"]
    assert report["current_units"]["yaw"]["legacy_derived_ampere_fields_ignored"] == report["streams"]["yaw_feedback"]["count"]


@pytest.mark.parametrize("corruption", [
    "missing_footer", "duplicate_header", "footer_count", "loss", "missing_loss", "can_sequence", "can_generation",
    "can_clock", "can_age", "nonzero_yaw", "nonzero_pitch", "unrelated_write", "fault_clear", "wrong_uid",
    "missing_read_metadata", "echo_only", "read_value", "read_source", "read_sequence", "read_time", "sample_time_claim", "missing_sample_time", "negative_reply",
    "missing_pre_enable_zero", "wrong_pre_enable_mode", "missing_enabled_zero", "lost_enabled_status",
    "lost_stop_status", "missing_restore_read", "temperature", "pitch_motion", "yaw_motion", "overcurrent",
    "imu_sequence", "imu_generation", "imu_gap", "imu_status", "imu_quaternion", "imu_end", "qualification",
])
def test_reviewer_rejects_corrupt_raw_evidence(tmp_path, successful, corruption):
    rows = load(successful[0])
    tx = [r for r in rows if r["kind"] == "neutral_tx"]
    reads = [r for r in rows if r["kind"] == "register_read"]
    enable = next(r for r in tx if r["axis"] == "pitch" and (r["id"] >> 24) & 31 == 3)
    restores = [r for r in tx if r["axis"] == "pitch" and (r["id"] >> 24) & 31 == 18 and
                int.from_bytes(bytes.fromhex(r["data_hex"])[:2], "little") == MODE]
    final_stop = next(r for r in tx if r["axis"] == "pitch" and (r["id"] >> 24) & 31 == 4 and
                      enable["begin_ns"] < r["begin_ns"] < restores[-1]["begin_ns"])
    feedback = [r for r in rows if is_pitch(r, 2)]
    yaw = [r for r in rows if r["kind"] == "can_rx" and r["axis"] == "yaw"]
    if corruption == "missing_footer": rows.pop()
    elif corruption == "duplicate_header": rows.insert(1, copy.deepcopy(rows[0]))
    elif corruption == "footer_count": rows[-1]["pitch_frames"] += 1
    elif corruption == "loss": yaw[0]["drop_delta"] = 1
    elif corruption == "missing_loss": del rows[-1]["socket_drops"]
    elif corruption == "can_sequence": yaw[10]["sequence"] += 1
    elif corruption == "can_generation": yaw[10]["generation"] = 2
    elif corruption == "can_clock": yaw[10]["clock_uncertainty_ns"] = 5_000_000
    elif corruption == "can_age": yaw[10]["dequeue_ns"] += 100_000_000
    elif corruption == "nonzero_yaw": next(r for r in tx if r["axis"] == "yaw")["data_hex"] = "0001000000000000"
    elif corruption in ("nonzero_pitch", "unrelated_write"):
        r = next(r for r in tx if r["axis"] == "pitch" and r["operation"] == "zero_before_mode")
        r["data_hex"] = (struct.pack("<H2xf", IQREF, .2) if corruption == "nonzero_pitch" else struct.pack("<H2xf", 0x7010, 0.)).hex()
    elif corruption == "fault_clear": final_stop["data_hex"] = "0100000000000000"
    elif corruption == "wrong_uid": next(r for r in rows if is_pitch(r, 0))["bytes"][0] ^= 1
    elif corruption == "missing_read_metadata": rows.remove(reads[2])
    elif corruption == "echo_only":
        rows = [r for r in rows if r["kind"] != "register_read" and not is_pitch(r, 17)]
        pitch = [r for r in rows if r["kind"] == "can_rx" and r["axis"] == "pitch"]
        for seq, r in enumerate(pitch, 1): r["sequence"] = seq
        rows[-1]["pitch_frames"] = len(pitch)
    elif corruption == "read_value": reads[0]["value"] = 1
    elif corruption == "read_source": reads[0]["source"] = "type18_echo"
    elif corruption == "read_sequence": reads[0]["request_sequence"] = 5
    elif corruption == "read_time": reads[0]["request_begin_ns"] = reads[0]["receive_ns"] + 1
    elif corruption == "sample_time_claim": reads[0]["device_sample_ns"] = reads[0]["receive_ns"]
    elif corruption == "missing_sample_time": del reads[0]["device_sample_ns"]
    elif corruption == "negative_reply": next(r for r in rows if is_pitch(r, 17))["id"] |= 1 << 16
    elif corruption in ("missing_pre_enable_zero", "missing_enabled_zero", "wrong_pre_enable_mode"):
        selected = [r for r in reads if r["index"] == (MODE if corruption == "wrong_pre_enable_mode" else IQREF)]
        r = selected[1 if corruption in ("missing_enabled_zero", "wrong_pre_enable_mode") else 0]
        if corruption == "wrong_pre_enable_mode":
            r["value"] = 2
            raw = next(f for f in rows if is_pitch(f, 17) and f["kernel_monotonic_ns"] == r["receive_ns"])
            raw["bytes"][4] = 2
        else: rows.remove(r)
    elif corruption == "lost_enabled_status":
        for r in feedback:
            if enable["begin_ns"] <= r["kernel_monotonic_ns"] < final_stop["begin_ns"]:
                r["id"] &= ~(3 << 22)
    elif corruption == "lost_stop_status":
        for r in feedback:
            if r["kernel_monotonic_ns"] >= final_stop["begin_ns"]: r["id"] |= 2 << 22
    elif corruption == "missing_restore_read": rows.remove(reads[-1])
    elif corruption == "temperature":
        feedback[10]["bytes"][6:] = list(struct.pack(">H", 650))
        feedback[10]["temperature_raw"], feedback[10]["temperature_C"] = 650, 65.
    elif corruption == "pitch_motion":
        feedback[10]["bytes"][:2] = list(struct.pack(">H", 35000)); feedback[10]["angle_raw"] = 35000
    elif corruption == "yaw_motion":
        for r in yaw[10:]:
            r["angle_raw"] += 20; r["bytes"][:2] = list(struct.pack(">H", r["angle_raw"]))
    elif corruption == "overcurrent":
        r = next(r for r in reads if r["index"] == IQF)
        wire = struct.pack("<f", .2)
        r["value"] = struct.unpack("<f", wire)[0]
        raw = next(f for f in rows if is_pitch(f, 17) and f["kernel_monotonic_ns"] == r["receive_ns"])
        raw["bytes"][4:] = list(wire)
    elif corruption.startswith("imu_"):
        records = [r for r in rows if r["kind"] == "imu_raw" and json.loads(r["raw_json"]).get("sensor") == "rv"]
        r = records[10]; raw = json.loads(r["raw_json"])
        if corruption == "imu_sequence": raw["sequence"] += 1
        elif corruption == "imu_generation": raw["generation"] += 1
        elif corruption == "imu_gap": raw["sample_ns"] -= 300_000_000
        elif corruption == "imu_status": raw["status"] = 4
        elif corruption == "imu_quaternion": raw["values"] = [0, 0, 0, 2]
        elif corruption == "imu_end": raw["kind"] = "summary"
        r["raw_json"] = json.dumps(raw)
    elif corruption == "qualification": rows[-1]["motion_qualified"] = True
    with pytest.raises(ValueError):
        review(save(tmp_path, rows))


def test_measured_capture_requires_interface_loss_evidence(tmp_path, successful):
    rows = load(successful[0]); rows[0]["provenance"] = "MEASURED"
    manifest = json.loads(rows[0]["manifest_yaml"])
    manifest.update(provenance="MEASURED", transport="socketcan", yaw={"interface":"can0"}, pitch={"interface":"can1"})
    rows[0]["manifest_yaml"] = json.dumps(manifest)
    rows[-1].pop("interface_loss_deltas", None)
    with pytest.raises(ValueError, match="interface receive loss"):
        review(save(tmp_path, rows))


def test_invalid_capture_cannot_be_made_complete_by_footer(tmp_path):
    directory = tmp_path / "rejected-mode"
    result = rehearse(BINARY, directory, fault="mode_ignored")
    assert result["returncode"] != 0
    rows = load(directory)
    rows[-1].update(status="COMPLETE", detail="", normal_stop_confirmed=True, abort_stop_confirmed=False)
    with pytest.raises(ValueError):
        review(save(tmp_path, rows))


def test_pitch_count_scale_matches_the_active_protocol(tmp_path, successful):
    rows = load(successful[0])
    feedback = [r for r in rows if is_pitch(r, 2)]
    row = feedback[10]
    row["angle_raw"] += 1
    row["bytes"][:2] = list(struct.pack(">H", row["angle_raw"]))
    report = review(save(tmp_path, rows))
    assert math.isclose(report["displacement"]["pitch_max_abs_rad"], 25 / 65535, abs_tol=1e-12)


@pytest.fixture(scope="module")
def characterization(tmp_path_factory):
    directory = tmp_path_factory.mktemp("characterization-review") / "above-criterion"
    result = rehearse(BINARY, directory, fault="neutral_noise", characterize=True)
    return directory, result


def test_characterization_records_quality_failure_without_qualifying_current(characterization):
    directory, result = characterization
    report = review_characterization(directory / "capture.jsonl")
    assert result["returncode"] == 0 and report["capture_complete"]
    assert report["capability_scope"] == "neutral_current_measurement_characterization_only"
    assert report["protection_current_bound_A"] == 6.5 and report["pitch_neutral_current_bound_A"] == .1
    assert report["diagnostic_comparison_only"] and report["historical_diagnostic_bound_A"] == .1
    assert report["neutral_observation_s"] == 2.
    assert not report["neutral_current_criterion_satisfied"]
    samples = report["current_observations"]
    assert len(samples) == report["current_statistics"]["count"] > 100
    assert report["samples_above_neutral_criterion"] == len(samples)
    expected = struct.unpack("<f", struct.pack("<f", .2515))[0]
    assert report["current_statistics"]["mean_A"] == expected and report["current_statistics"]["sample_std_A"] == 0
    assert report["current_statistics"]["maximum_abs_A"] == expected
    assert all(r["value_A"] == expected and r["raw_register_response_hex"] == "1a700000" + struct.pack("<f", .2515).hex()
               and r["device_sample_ns"] is None for r in samples)
    assert 0 < report["initial_current_receipt_from_enable_s"] < .15
    assert report["initial_current_receipt_from_enable_s"] == samples[0]["receipt_from_enable_s"]
    assert not report["neutral_current_qualified"] and not report["neutral_transition_qualified"]
    assert not report["physical_current_mode_qualified"] and not report["physical_capabilities_qualified"]
    assert not report["physical_parameters_qualified"] and not report["motion_authorized"]
    assert not report["current_feedback_scale_calibrated"] and report["current_bias_correction"] is None
    assert report["plant_snapshot"] is None and report["controller_candidate"] is None


def test_characterization_keeps_manufacturer_protection_active(tmp_path):
    directory = tmp_path / "protection"
    result = rehearse(BINARY, directory, fault="overcurrent", characterize=True)
    assert result["returncode"] != 0 and "manufacturer current protection" in result["result"]["detail"]
    assert result["result"]["abort_stop_confirmed"]
    with pytest.raises(ValueError, match="capture failed"):
        review_characterization(directory / "capture.jsonl")


def test_strict_v1_still_aborts_above_its_original_neutral_guard(tmp_path):
    directory = tmp_path / "strict-v1"
    result = rehearse(BINARY, directory, fault="overcurrent")
    assert result["returncode"] != 0 and "nonneutral measured pitch current" in result["result"]["detail"]
    with pytest.raises(ValueError, match="capture failed"):
        review(directory / "capture.jsonl")


def test_purposes_cannot_be_exchanged(characterization, successful):
    with pytest.raises(ValueError, match="unsupported preparation schema/purpose"):
        review(characterization[0] / "capture.jsonl")
    with pytest.raises(ValueError, match="unsupported preparation schema/purpose"):
        review_characterization(successful[0] / "capture.jsonl")


@pytest.mark.parametrize("corruption", ["qualification", "criterion", "count", "peak", "guard", "current", "device_time", "observation"])
def test_characterization_rejects_forged_footer_or_current_evidence(tmp_path, characterization, corruption):
    rows = load(characterization[0])
    if corruption == "qualification": rows[-1]["neutral_current_qualified"] = True
    elif corruption == "criterion": rows[-1]["neutral_current_criterion_satisfied"] = True
    elif corruption == "count": rows[-1]["current_sample_count"] -= 1
    elif corruption == "peak": rows[-1]["observed_current_max_abs_A"] = 0.
    elif corruption == "guard":
        config = json.loads(rows[0]["manifest_yaml"])
        config["protection_current_bound_A"] = "23"
        rows[0]["manifest_yaml"] = json.dumps(config)
    elif corruption == "observation":
        config = json.loads(rows[0]["manifest_yaml"])
        config["neutral_observation_s"] = "10"
        rows[0]["manifest_yaml"] = json.dumps(config)
    else:
        read = next(r for r in rows if r["kind"] == "register_read" and r["index"] == IQF)
        if corruption == "device_time": read["device_sample_ns"] = read["receive_ns"]
        else:
            read["value"] = 7.
            reply = next(r for r in rows if is_pitch(r, 17) and r["kernel_monotonic_ns"] == read["receive_ns"])
            reply["bytes"][4:] = list(struct.pack("<f", 7.))
            rows[-1]["observed_current_max_abs_A"] = 7.
    with pytest.raises(ValueError):
        review_characterization(save(tmp_path, rows))
