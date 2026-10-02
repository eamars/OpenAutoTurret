"""Review executable homing evidence: raw observations, no acceptance gates or qualification."""
import json
import os
from pathlib import Path
import struct
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_homing_review import COUNT_RAD, CURRENT_LIMIT, review, strict_json
from adr0022_homing_rehearsal import rehearse

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real Linux C++/UDP/pipe capture")
ROOT = TOOLS.parents[1]
BINARY = Path(os.environ.get("OTA_STAGE1_BUILD", ROOT / "run/adr0022-local/firmware")) / "axis_control_core/commissiond"
UNQUALIFIED = ("motion_authorized", "motion_qualified", "current_mode_qualified", "physical_capabilities_qualified",
               "physical_parameters_qualified", "calibration_qualified", "protection_qualified", "controller_qualified",
               "encoder_mechpos_agreement_qualified", "encoder_zero_command_sent")


@pytest.fixture(scope="module")
def successful(tmp_path_factory):
    directory = tmp_path_factory.mktemp("homing-review") / "normal"
    result = rehearse(BINARY, directory)
    assert result["returncode"] == 0
    # This actual executable capture is the viability gate before the altered cases.
    report = review(directory / "capture.jsonl")
    return directory, report


def load(directory):
    return [json.loads(line) for line in (directory / "capture.jsonl").read_text().splitlines()]


def save(tmp_path, rows):
    path = tmp_path / "altered.jsonl"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return path


def assert_unqualified(report):
    assert all(report[key] is False for key in UNQUALIFIED), {key: report[key] for key in UNQUALIFIED}
    assert report["plant_snapshot"] is report["controller_candidate"] is None


def test_actual_homing_reconstructs_geometry_without_qualification(successful):
    _, report = successful
    assert report["schema"] == "adr0022.homing_review/1"
    assert report["capture_complete"] and report["capture_integrity"] == "PASS" and report["homing_observed"]
    assert report["capability_scope"] == "sensorless_measurement_observations_only"
    assert len(report["endpoints"]) == 2 and {e["direction"] for e in report["endpoints"]} == {1, -1}
    assert all(not e["qualified"] for e in report["endpoints"])
    assert all(not a["contact_qualified"] for a in report["approaches"])
    assert 55 <= report["measured_travel_deg"] <= 65
    assert report["midpoint_rad"] == .5 * sum(e["angle_rad"] for e in report["endpoints"])
    assert report["encoder_resolution_rad"] == COUNT_RAD == 25 / 65535
    assert report["normal_stop_confirmed"] and not report["abort_stop_confirmed"]
    assert report["restored_settings_verified"] and report["mode_transitions"] == 12
    assert report["streams"]["pitch_iqf"]["device_sample_ns"] is None
    assert report["current_units"]["yaw"]["scale_A_per_count"] is None
    assert report["encoder_mechpos_receipt_pairs"] and report["encoder_mechpos_bias_correction"] is None
    assert all(not pair["mapping_qualified"] for pair in report["encoder_mechpos_receipt_pairs"])
    assert_unqualified(report)


def test_phase_labels_do_not_replace_raw_phase_reconstruction(tmp_path, successful):
    rows = load(successful[0])
    for row in rows:
        if row["kind"] == "homing_executor_state": row["state"] = -999
        if row["kind"] == "homing_desired_state": row["message"] = "invented endpoint"
    report = review(save(tmp_path, rows))
    assert report["approaches"] == successful[1]["approaches"]
    assert report["endpoints"] == successful[1]["endpoints"]


def test_geometry_annotations_are_reported_beside_raw_measurements(tmp_path, successful):
    rows = load(successful[0])
    for row in rows:
        if row["kind"] in ("homing_endpoints", "footer") and "endpoint_a_rad" in row:
            row["endpoint_a_rad"] += .1
            row["midpoint_rad"] += .05
    report = review(save(tmp_path, rows))
    assert report["endpoints"] == successful[1]["endpoints"]
    assert report["midpoint_rad"] == successful[1]["midpoint_rad"]
    annotated = report["executor_measurement_annotations"]["homing_endpoints"]
    assert annotated and annotated[0]["endpoint_a_rad"] != successful[1]["executor_measurement_annotations"]["homing_endpoints"][0]["endpoint_a_rad"]


def test_real_native_position_offset_is_retained_without_calibration(tmp_path):
    directory = tmp_path / "native-position-offset"
    assert rehearse(BINARY, directory, fault="native_position_offset")["returncode"] == 0
    report = review(directory / "capture.jsonl")
    differences = [abs(p["register_minus_type2_rad"]) for p in report["encoder_mechpos_receipt_pairs"]
                   if p["register_minus_type2_rad"] is not None]
    assert .0004 < max(differences) <= .001 + 1e-6
    assert report["encoder_mechpos_bias_correction"] is None
    assert all(not pair["mapping_qualified"] for pair in report["encoder_mechpos_receipt_pairs"])
    assert report["homing_observed"]
    assert_unqualified(report)


def test_current_protection_is_independent_from_homing_command_cap(tmp_path):
    directory = tmp_path / "continuous-rating"
    assert rehearse(BINARY, directory, fault="continuous_current_guard")["returncode"] == 0
    rows = load(directory)
    report = review(directory / "capture.jsonl")
    caps = [struct.unpack("<f", bytes.fromhex(r["data_hex"])[4:])[0] for r in rows
            if r["kind"] == "homing_tx" and r["axis"] == "pitch" and (r["id"] >> 24) & 31 == 18 and
            int.from_bytes(bytes.fromhex(r["data_hex"])[:2], "little") == CURRENT_LIMIT]
    assert caps and all(value == 5 for value in caps)
    assert {c["value"] for c in report["homing_command_current_observations"]} == {5}
    assert report["pitch_current_max_abs_A"] == pytest.approx(5.2)
    assert report["normal_stop_confirmed"] and report["restored_settings_verified"]
    assert_unqualified(report)


def test_real_overcurrent_abort_is_reported_and_a_forged_footer_invents_no_geometry(tmp_path):
    directory = tmp_path / "above-continuous-rating"
    assert rehearse(BINARY, directory, fault="overcurrent")["returncode"] != 0
    report = review(directory / "capture.jsonl")
    assert not report["capture_complete"] and report["capture_footer_status"] != "COMPLETE"
    assert report["abort_stop_confirmed"] and not report["homing_observed"]
    assert report["pitch_current_max_abs_A"] == 7
    rows = load(directory)
    rows[-1].update(status="COMPLETE", detail="", normal_stop_confirmed=True, abort_stop_confirmed=False, homing_observed=True)
    forged = review(save(tmp_path, rows))
    assert not forged["homing_observed"] and forged["endpoints"] == [] and forged["midpoint_rad"] is None
    assert forged["homing_executor_reported_observed"] is True
    assert_unqualified(forged)


def test_malformed_or_foreign_capture_is_rejected(tmp_path, successful):
    rows = load(successful[0])
    foreign = [dict(rows[0], schema="adr0022.yaw-control/1"), *rows[1:]]
    with pytest.raises(ValueError, match="unsupported homing capture schema"):
        review(save(tmp_path, foreign))
    with pytest.raises(ValueError, match="capture header missing"):
        review(save(tmp_path, rows[1:]))
    bad = tmp_path / "bad.jsonl"
    bad.write_text(json.dumps(rows[0]) + "\n[1, 2]\n")
    with pytest.raises(ValueError, match="JSON objects"):
        review(bad)


@pytest.mark.parametrize("text", ['{"a": 1, "a": 2}', '{"a": NaN}', '{"a": Infinity}'])
def test_strict_json_rejects_duplicates_and_nonfinite_values(text):
    with pytest.raises(ValueError):
        strict_json(text)
