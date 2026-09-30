"""Independently review executable homing evidence and reject forged claims."""
import json
import math
import os
from pathlib import Path
import struct
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_homing_review import COUNT_RAD, IQF, IQREF, MODE, POSITION, MECH_POSITION, SPEED, SPEED_LIMIT, review
from adr0022_homing_rehearsal import rehearse

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real Linux C++/UDP/pipe capture")
ROOT = TOOLS.parents[1]
BINARY = Path(os.environ.get("OTA_STAGE1_BUILD", ROOT / "run/adr0022-local/firmware")) / "axis_control_core/commissiond"


@pytest.fixture(scope="module")
def successful(tmp_path_factory):
    directory = tmp_path_factory.mktemp("homing-review") / "normal"
    result = rehearse(BINARY, directory)
    assert result["returncode"] == 0
    # This actual executable capture is the viability gate before corruption cases.
    report = review(directory / "capture.jsonl")
    return directory, report


def load(directory):
    return [json.loads(line) for line in (directory / "capture.jsonl").read_text().splitlines()]


def save(tmp_path, rows):
    path = tmp_path / "altered.jsonl"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return path


def is_pitch(row, kind):
    return row["kind"] == "can_rx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == kind


def tx_kind(row, kind):
    return row["kind"] == "homing_tx" and row["axis"] == "pitch" and (row["id"] >> 24) & 31 == kind


def register(row):
    return int.from_bytes(bytes.fromhex(row["data_hex"])[:2], "little")


def set_tx(row, value):
    row["data_hex"] = struct.pack("<H2xf", register(row), value).hex()


def set_read(rows, row, value):
    wire = struct.pack("<f", value)
    row["value"] = struct.unpack("<f", wire)[0]
    raw = next(r for r in rows if is_pitch(r, 17) and r["kernel_monotonic_ns"] == row["receive_ns"])
    raw["bytes"][4:] = list(wire)


def remove_transaction(rows, row):
    raw = next(r for r in rows if is_pitch(r, 17) and r["kernel_monotonic_ns"] == row["receive_ns"])
    request = next(r for r in reversed(rows[:rows.index(raw)]) if tx_kind(r, 17) and register(r) == row["index"])
    rows.remove(raw); rows.remove(request); rows.remove(row)
    for seq, r in enumerate((r for r in rows if r["kind"] == "can_rx" and r["axis"] == "pitch"), 1):
        r["sequence"] = seq
    for seq, r in enumerate((r for r in rows if r["kind"] == "register_read"), 1):
        r["request_sequence"] = seq


def test_actual_homing_reconstructs_geometry_without_qualification(successful):
    _, report = successful
    assert report["capture_complete"] and report["homing_observed"]
    assert report["capability_scope"] == "sensorless_geometry_observation_only"
    assert len(report["contacts"]) == 6 and len(report["backoffs"]) == 4 and len(report["endpoints"]) == 2
    assert 55 <= report["measured_travel_deg"] <= 65
    assert report["midpoint_rad"] == .5 * sum(e["angle_rad"] for e in report["endpoints"])
    assert report["encoder_resolution_rad"] == 25 / 65535
    assert report["midpoint_dwell_s"] >= .5 and report["normal_stop_confirmed"] and report["restored_settings_verified"]
    assert report["mode_transitions"] == 12 and report["register_reads"] > 1000
    assert len(report["enabled_acknowledgements"]) == 12
    assert all(a["neutral_until_first_motor"] and a["receipt_latency_s"] >= 0 for a in report["enabled_acknowledgements"])
    assert report["streams"]["pitch_iqf"]["device_sample_ns"] is None
    assert report["current_units"]["yaw"]["scale_A_per_count"] is None
    assert report["encoder_mechpos_receipt_pairs"] and report["encoder_mechpos_bias_correction"] is None
    assert all(not report[key] for key in ("motion_authorized", "motion_qualified", "current_mode_qualified",
        "physical_capabilities_qualified", "physical_parameters_qualified", "encoder_mechpos_agreement_qualified", "encoder_zero_command_sent"))
    assert report["plant_snapshot"] is report["controller_candidate"] is None


def test_phase_labels_do_not_replace_raw_phase_reconstruction(tmp_path, successful):
    rows = load(successful[0])
    for row in rows:
        if row["kind"] == "homing_executor_state": row["state"] = -999
        if row["kind"] == "homing_desired_state": row["message"] = "invented endpoint"
    report = review(save(tmp_path, rows))
    assert report["contacts"] == successful[1]["contacts"]


def test_geometry_annotations_allow_encoder_quantization_without_replacing_measurements(tmp_path, successful):
    rows = load(successful[0])
    for row in rows:
        if row["kind"] in ("homing_endpoints", "footer"):
            row["endpoint_a_rad"] += COUNT_RAD
            row["endpoint_b_rad"] -= COUNT_RAD
            row["midpoint_rad"] += COUNT_RAD
            row["measured_travel_deg"] += 2 * COUNT_RAD * 180 / math.pi * .99
    report = review(save(tmp_path, rows))
    assert report["endpoints"] == successful[1]["endpoints"]
    assert report["midpoint_rad"] == successful[1]["midpoint_rad"]


def test_real_native_position_offset_is_retained_without_calibration(tmp_path):
    directory = tmp_path / "native-position-offset"
    result = rehearse(BINARY, directory, fault="native_position_offset")
    assert result["returncode"] == 0
    report = review(directory / "capture.jsonl")
    assert .0004 < report["encoder_mechpos_max_abs_receipt_difference_rad"] <= .001 + 1e-6
    assert report["encoder_mechpos_declared_guard_rad"] == .001
    assert report["encoder_mechpos_bias_correction"] is None
    assert not report["encoder_mechpos_agreement_qualified"] and not report["physical_parameters_qualified"]
    assert all(pair["device_sample_ns"] is None for pair in report["encoder_mechpos_receipt_pairs"])


def test_real_delayed_reads_survive_inert_stop_polls(tmp_path):
    directory = tmp_path / "delayed-reads"
    result = rehearse(BINARY, directory, fault="delayed_reads")
    assert result["returncode"] == 0
    report = review(directory / "capture.jsonl")
    assert report["homing_observed"] and report["restored_settings_verified"]
    assert report["encoder_mechpos_receipt_pairs"]
    assert sum(a["pending_reset_feedback_count"] for a in report["enabled_acknowledgements"]) > 0


def test_native_annotations_cannot_replace_raw_evidence(tmp_path, successful):
    rows = [r for r in load(successful[0]) if r["kind"] != "homing_position_observation"]
    report = review(save(tmp_path, rows))
    assert report["encoder_mechpos_receipt_pairs"] == successful[1]["encoder_mechpos_receipt_pairs"]


def test_kernel_arrival_before_pin_can_be_processed_after_pin(tmp_path, successful):
    rows = load(successful[0])
    observation = next(r for r in rows if r["kind"] == "homing_position_observation")
    pin = next(r for r in rows if tx_kind(r, 18) and register(r) == POSITION and r["begin_ns"] > observation["read_receive_ns"])
    # Preserve raw journal order: this queued frame was unavailable to the
    # executor at pin time, although it had reached the kernel beforehand.
    queued = next(r for r in rows[rows.index(pin) + 1:] if r["kind"] == "can_rx" and r["axis"] == "pitch")
    assert is_pitch(queued, 2)
    old_time = queued["kernel_monotonic_ns"]
    queued["kernel_monotonic_ns"] = pin["begin_ns"] - 1
    queued["kernel_realtime_ns"] += queued["kernel_monotonic_ns"] - old_time
    report = review(save(tmp_path, rows))
    assert report["encoder_mechpos_receipt_pairs"][0]["encoder_receive_ns"] == observation["status_receive_ns"]


@pytest.fixture(scope="module")
def continuous_current_capture(tmp_path_factory):
    directory = tmp_path_factory.mktemp("homing-current-protection") / "continuous-rating"
    result = rehearse(BINARY, directory, fault="continuous_current_guard")
    assert result["returncode"] == 0
    return directory


def test_current_protection_is_independent_from_homing_command_cap(continuous_current_capture):
    rows = load(continuous_current_capture)
    report = review(continuous_current_capture / "capture.jsonl")
    assert report["homing_command_current_cap_A"] == 5
    assert report["measured_current_protection_bound_A"] == 6.5
    assert report["pitch_current_max_abs_A"] == pytest.approx(5.2)
    caps = [struct.unpack("<f", bytes.fromhex(r["data_hex"])[4:])[0] for r in rows if tx_kind(r, 18) and register(r) == 0x7018]
    assert caps and all(value == 5 for value in caps)
    assert report["normal_stop_confirmed"] and report["restored_settings_verified"]
    assert not report["current_mode_qualified"] and not report["physical_capabilities_qualified"]


@pytest.mark.parametrize("corruption", ["missing", "kind", "document", "sha256", "continuous_current_A", "extra", "lower_guard", "above_rating", "raised_command_cap"])
def test_measured_current_basis_is_exact_and_command_cap_remains_five(tmp_path, continuous_current_capture, corruption):
    rows = load(continuous_current_capture)
    manifest = json.loads(rows[0]["manifest_yaml"])
    rows[0]["provenance"] = manifest["provenance"] = "MEASURED"
    manifest["transport"] = "socketcan"
    manifest["yaw"]["interface"], manifest["pitch"]["interface"] = "can0", "can1"
    rows[-1]["interface_loss_deltas"] = {axis: {"rx_dropped": 0, "rx_errors": 0} for axis in ("yaw", "pitch")}
    basis = manifest["protection_limit_basis"]
    if corruption == "missing": del manifest["protection_limit_basis"]
    elif corruption == "extra": basis["invented_calibration"] = True
    elif corruption == "lower_guard": manifest["guards"]["current_bound_A"] = 5
    elif corruption == "above_rating": manifest["guards"]["current_bound_A"] = 7
    elif corruption == "raised_command_cap":
        manifest["homing"]["limit_cur_initial_a"] = manifest["homing"]["limit_cur_max_a"] = 5.2
        manifest["native_settings"]["homing_limit_cur_A"] = 5.2
    else: basis[corruption] = 5 if corruption == "continuous_current_A" else "forged"
    rows[0]["manifest_yaml"] = json.dumps(manifest)
    with pytest.raises(ValueError): review(save(tmp_path, rows))


def test_seven_ampere_actual_feedback_still_aborts_and_cannot_be_qualified(tmp_path):
    directory = tmp_path / "above-continuous-rating"
    result = rehearse(BINARY, directory, fault="overcurrent")
    assert result["returncode"] != 0
    rows = load(directory)
    assert any(r["kind"] == "register_read" and r["index"] == IQF and r["value"] == 7 for r in rows)
    rows[-1].update(status="COMPLETE", detail="", normal_stop_confirmed=True, abort_stop_confirmed=False, homing_observed=True)
    with pytest.raises(ValueError, match="measured pitch current guard exceeded"):
        review(save(tmp_path, rows))


@pytest.mark.parametrize("corruption", [
    "footer", "geometry", "qualification", "loss", "missing_loss", "sequence", "clock", "age", "can_fault",
    "nonzero_yaw", "nonzero_iq", "zero_encoder", "fault_clear", "read_value", "read_source", "sample_time",
    "snapshot", "before_enable", "enabled_pin", "mech_position", "position_default", "position_cap",
    "wrong_approach", "backoff_target", "hold_pin", "hold_readback", "native_tolerance_contract", "sensor_guard_contract",
    "position_annotation", "mapping_claim", "reset_after_motor", "positive_cap_before_native_reads",
    "contact_effort", "repeatability", "midpoint_dwell", "final_reset",
    "restore", "temperature", "torque", "encoder_jump", "overcurrent", "imu_sequence", "imu_generation", "imu_status",
])
def test_raw_evidence_corruption_is_rejected(tmp_path, successful, corruption):
    rows = load(successful[0])
    writes = [r for r in rows if tx_kind(r, 18)]
    reads = [r for r in rows if r["kind"] == "register_read"]
    feedback = [r for r in rows if is_pitch(r, 2)]
    modes = [r for r in writes if register(r) == MODE]
    enables = [r for r in rows if tx_kind(r, 3)]
    final_stop = next(r for r in rows if tx_kind(r, 4) and enables[-1]["begin_ns"] < r["begin_ns"] < modes[-1]["begin_ns"])
    if corruption == "footer": rows[-1]["normal_stop_confirmed"] = False
    elif corruption == "geometry":
        for row in rows:
            if row["kind"] in ("homing_endpoints", "footer"):
                row["endpoint_a_rad"] += .1
                row["midpoint_rad"] += .05
    elif corruption == "qualification": rows[-1]["motion_qualified"] = True
    elif corruption == "loss": feedback[10]["drop_delta"] = 1
    elif corruption == "missing_loss": del rows[-1]["socket_drops"]
    elif corruption == "sequence": feedback[10]["sequence"] += 1
    elif corruption == "clock": feedback[10]["clock_uncertainty_ns"] = 5_000_000
    elif corruption == "age": feedback[10]["dequeue_ns"] += 100_000_000
    elif corruption == "can_fault": feedback[10]["id"] |= 1 << 16
    elif corruption == "nonzero_yaw": next(r for r in rows if r["kind"] == "homing_tx" and r["axis"] == "yaw")["data_hex"] = "0001000000000000"
    elif corruption == "nonzero_iq": set_tx(next(r for r in writes if register(r) == IQREF), .1)
    elif corruption == "zero_encoder": next(r for r in writes if register(r) == IQREF)["id"] = (6 << 24) | 127
    elif corruption == "fault_clear": final_stop["data_hex"] = "0100000000000000"
    elif corruption == "read_value": reads[0]["value"] = 2
    elif corruption == "read_source": reads[0]["source"] = "type18_echo"
    elif corruption == "sample_time": reads[0]["device_sample_ns"] = reads[0]["receive_ns"]
    elif corruption == "snapshot": remove_transaction(rows, reads[0])
    elif corruption == "before_enable":
        remove_transaction(rows, next(r for r in reads if r["index"] == IQREF and r["receive_ns"] < enables[0]["begin_ns"]))
    elif corruption == "enabled_pin":
        remove_transaction(rows, next(r for r in reads if r["index"] == POSITION and enables[2]["begin_ns"] < r["receive_ns"] < modes[3]["begin_ns"]))
    elif corruption == "mech_position": remove_transaction(rows, next(r for r in reads if r["index"] == MECH_POSITION))
    elif corruption == "position_default": set_tx(next(r for r in writes if register(r) == POSITION), 0)
    elif corruption == "position_cap": set_tx(next(r for r in writes if register(r) == SPEED_LIMIT), .1)
    elif corruption == "wrong_approach":
        row = next(r for r in writes if register(r) == SPEED and struct.unpack("<f", bytes.fromhex(r["data_hex"])[4:])[0] != 0)
        set_tx(row, -struct.unpack("<f", bytes.fromhex(row["data_hex"])[4:])[0])
    elif corruption == "backoff_target":
        row = next(r for r in writes if r["operation"] == "position_target")
        set_tx(row, struct.unpack("<f", bytes.fromhex(row["data_hex"])[4:])[0] + .03)
    elif corruption in ("hold_pin", "hold_readback"):
        hold = next(r for r in writes if r["operation"] == "hold_measured_pose")
        if corruption == "hold_pin":
            remove_transaction(rows, next(r for r in reversed(reads) if r["index"] == MECH_POSITION and r["receive_ns"] < hold["begin_ns"]))
        else:
            remove_transaction(rows, next(r for r in reads if r["index"] == POSITION and r["request_begin_ns"] > hold["begin_ns"]))
    elif corruption in ("native_tolerance_contract", "sensor_guard_contract"):
        manifest = json.loads(rows[0]["manifest_yaml"])
        if corruption == "native_tolerance_contract": manifest["native_settings"]["position_reference_readback_tolerance_rad"] = "0.001"
        else: manifest["guards"]["encoder_mechpos_agreement_bound_rad"] = "0.1"
        rows[0]["manifest_yaml"] = json.dumps(manifest)
    elif corruption in ("position_annotation", "mapping_claim"):
        row = next(r for r in rows if r["kind"] == "homing_position_observation")
        if corruption == "position_annotation": row["register_minus_type2_rad"] += .001
        else: row["mapping_qualified"] = True
    elif corruption == "reset_after_motor":
        motor = [r for r in feedback if enables[1]["begin_ns"] < r["kernel_monotonic_ns"] < modes[2]["begin_ns"] and
                 (r["id"] >> 22) & 3 == 2]
        motor[1]["id"] &= ~(3 << 22)
    elif corruption == "positive_cap_before_native_reads":
        cap = next(r for r in writes if register(r) == SPEED_LIMIT and struct.unpack("<f", bytes.fromhex(r["data_hex"])[4:])[0] > 0)
        enable = next(r for r in reversed(enables) if r["begin_ns"] < cap["begin_ns"])
        first_motor = next(r for r in feedback if r["kernel_monotonic_ns"] > enable["begin_ns"] and (r["id"] >> 22) & 3 == 2)
        rows.remove(cap)
        cap["begin_ns"] = first_motor["kernel_monotonic_ns"] + 1
        cap["kernel_accepted_ns"] = cap["begin_ns"] + 1
        rows.insert(rows.index(first_motor) + 1, cap)
    elif corruption == "contact_effort":
        contact = successful[1]["contacts"][0]
        for row in feedback:
            if contact["begin_ns"] <= row["kernel_monotonic_ns"] <= contact["receive_ns"]:
                row["torque_raw"] = 32768; row["bytes"][4:6] = list(struct.pack(">H", 32768))
    elif corruption == "repeatability":
        manifest = json.loads(rows[0]["manifest_yaml"])
        manifest["homing"]["repeatability_rad"] = "0.0000001"
        # Raw repeated endpoint is one encoder count farther than the first;
        # metadata remains otherwise consistent so phase reconstruction must fail.
        contact = successful[1]["contacts"][2]
        for row in feedback:
            if contact["receive_ns"] - 350_000_000 <= row["kernel_monotonic_ns"] <= contact["receive_ns"]:
                row["angle_raw"] += 1; row["bytes"][:2] = list(struct.pack(">H", row["angle_raw"]))
        rows[0]["manifest_yaml"] = json.dumps(manifest)
    elif corruption == "midpoint_dwell":
        manifest = json.loads(rows[0]["manifest_yaml"]); manifest["guards"]["midpoint_dwell_s"] = "1"
        rows[0]["manifest_yaml"] = json.dumps(manifest)
    elif corruption == "final_reset":
        for row in feedback:
            if row["kernel_monotonic_ns"] >= final_stop["begin_ns"]: row["id"] |= 2 << 22
    elif corruption == "restore": remove_transaction(rows, reads[-1])
    elif corruption == "temperature":
        feedback[10]["temperature_raw"] = 650; feedback[10]["temperature_C"] = 65
        feedback[10]["bytes"][6:] = list(struct.pack(">H", 650))
    elif corruption == "torque":
        feedback[10]["torque_raw"] = 65535; feedback[10]["bytes"][4:6] = list(struct.pack(">H", 65535))
    elif corruption == "encoder_jump":
        feedback[10]["angle_raw"] += 100; feedback[10]["bytes"][:2] = list(struct.pack(">H", feedback[10]["angle_raw"]))
    elif corruption == "overcurrent": set_read(rows, next(r for r in reads if r["index"] == IQF), 6)
    elif corruption.startswith("imu_"):
        row = next(r for r in rows if r["kind"] == "imu_raw" and json.loads(r["raw_json"]).get("sensor") == "rv")
        raw = json.loads(row["raw_json"])
        raw[corruption.removeprefix("imu_")] = {"imu_sequence": 99, "imu_generation": 9, "imu_status": 4}[corruption]
        row["raw_json"] = json.dumps(raw)
    with pytest.raises(ValueError): review(save(tmp_path, rows))


@pytest.mark.parametrize("fault", ["mode_ignored", "sensor_disagreement"])
def test_real_abort_cannot_be_upgraded_by_a_complete_footer(tmp_path, fault):
    directory = tmp_path / fault
    result = rehearse(BINARY, directory, fault=fault)
    assert result["returncode"] != 0
    rows = load(directory)
    rows[-1].update(status="COMPLETE", detail="", normal_stop_confirmed=True, abort_stop_confirmed=False, homing_observed=True)
    with pytest.raises(ValueError): review(save(tmp_path, rows))
