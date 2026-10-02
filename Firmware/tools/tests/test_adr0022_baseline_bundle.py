"""Packing, validation and extraction boundaries for a dedicated session release."""
import io
import json
from pathlib import Path
import shutil
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from adr0022_baseline_bundle import EXECUTABLES, SCHEMA, install, pack, validate
from adr0022_capture_launch import PROTECTION_DOCUMENT

LABEL = "yaw-fixture-01"


def arm64_build(tmp_path):
    """Dummy executables with only the ARM64 ELF identification the bundle checks."""
    build = tmp_path / "build"
    for key, relative in EXECUTABLES.items():
        path = build / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"\x7fELF" + bytes(14) + b"\xb7\x00" + key.encode())
    return build


def yaw_control_manifest():
    """Synthetic, unbound yaw-control session; no physical facts."""
    return {"schema": "adr0022.yaw-control/1", "purpose": "yaw_shared_core_3a", "provenance": "MEASURED",
            "transport": "socketcan", "yaw": {"interface": "can0"}, "pitch": {"interface": "can1"},
            "expected_pitch_uid": "7216313130333105", "pitch_supported_when_disabled": True,
            "baseline_s": 2., "stop_observation_s": 2., "pitch_maximum_temperature_C": 45., "yaw_current_bound_A": 3.,
            "candidate_label": "fixture", "servo_parameters": {"use_gyro": 0}, "gyro_calibration": {"yaw_column": [0, 0, 1]},
            "other_axis_posture_rad": 0., "reference_segments": [{"duration_s": 1., "target_position_rad": .1}],
            "limits": {"clock_uncertainty_s": .001, "dequeue_age_s": .1, "can_gap_s": .1, "imu_gap_s": .2,
                       "startup_s": 3., "duration_s": 20., "minimum_imu_status": 0, "read_timeout_s": .2,
                       "read_period_s": .01, "stop_period_s": .02}}


def homing_manifest(tmp_path):
    """Synthetic package fixture from the homing rehearsal; never a physical qualification."""
    from adr0022_homing_rehearsal import fixture
    manifest = fixture([31001, 31002], 31003, -1, tmp_path / "unused.jsonl")
    manifest.pop("imu_fd")
    manifest.pop("output")
    manifest.update(provenance="MEASURED", transport="socketcan", yaw={"interface": "can0"}, pitch={"interface": "can1"},
                    operator_attendance={"present_at_manual_cutoff": False, "operator_identity": "fixture",
                                         "manual_cutoff_evidence_identity": "fixture-manual-only-cutoff"},
                    session_authorization={"purpose": "pitch_sensorless_homing", "sensorless_homing_authorized": True,
                                           "authorization_identity": "fixture-authorization",
                                           "unattended_operation_authorized": True, "presence_required": False})
    manifest["guards"]["current_bound_A"] = 6.5
    return manifest


@pytest.fixture
def bundle(tmp_path):
    archive = tmp_path / "bundle.tar"
    pack(arm64_build(tmp_path), LABEL, yaw_control_manifest(), archive)
    return archive


def test_yaw_control_bundle_installs_executables_and_bound_manifest(tmp_path, bundle):
    record = validate(bundle, LABEL)
    assert record["schema"] == SCHEMA and record["launch_option"] == "--control-yaw"
    firmware, output = tmp_path / "Firmware", tmp_path / "observations"
    firmware.mkdir()
    manifest = install(bundle, LABEL, firmware, output)
    bound = json.loads(manifest.read_text())
    assert bound["output"] == str(output.resolve() / "yaw-control.jsonl")
    assert bound["session_label"] == LABEL
    for relative in EXECUTABLES.values():
        assert (firmware / "build" / relative).read_bytes() == (tmp_path / "build" / relative).read_bytes()
    with pytest.raises(ValueError, match="unused release"):
        install(bundle, LABEL, firmware, output)
    assert not (output / "yaw-control.jsonl").exists()


def test_bundle_cannot_be_relabelled_as_another_session(bundle):
    with pytest.raises(ValueError, match="session label differs"):
        validate(bundle, "another-session")


@pytest.mark.parametrize("corruption", ["truncated", "duplicate", "traversal", "symlink"])
def test_modified_or_unsafe_bundle_is_rejected_before_install(tmp_path, bundle, corruption):
    altered = tmp_path / "altered.tar"
    with tarfile.open(bundle) as source, tarfile.open(altered, "w") as target:
        for entry in source:
            data = source.extractfile(entry).read()
            if corruption == "truncated" and entry.name.endswith("commissiond"):
                data = data[:-1]
                entry.size = len(data)
            target.addfile(entry, io.BytesIO(data))
        if corruption != "truncated":
            name = "bundle.json" if corruption == "duplicate" else "../../escape"
            extra = tarfile.TarInfo(name)
            if corruption == "symlink":
                extra.type = tarfile.SYMTYPE
                extra.linkname = "/tmp/escape"
            target.addfile(extra, io.BytesIO())
    with pytest.raises(ValueError):
        install(altered, LABEL, tmp_path / "Firmware", tmp_path / "evidence")
    assert not (tmp_path / "Firmware").exists() and not (tmp_path / "evidence").exists()


@pytest.mark.parametrize("changed", ["bound_output", "synthetic", "label_mismatch", "authority", "stop_window",
                                     "no_reference", "short_session", "non_arm64"])
def test_yaw_control_bundle_refuses_invalid_session_before_writing(tmp_path, changed):
    build = arm64_build(tmp_path)
    manifest = yaw_control_manifest()
    if changed == "bound_output":
        manifest["output"] = "/tmp/yaw-control.jsonl"
    elif changed == "synthetic":
        manifest["provenance"] = "SYNTHETIC"
    elif changed == "label_mismatch":
        manifest["session_label"] = "other-session"
    elif changed == "authority":
        manifest["yaw_current_bound_A"] = 3.5
    elif changed == "stop_window":
        manifest["stop_observation_s"] = 1.
    elif changed == "no_reference":
        del manifest["reference_segments"]
    elif changed == "short_session":
        manifest["limits"]["duration_s"] = 8.
    else:
        (build / EXECUTABLES["imu"]).write_bytes(b"\x7fELF" + bytes(14) + b"\x3e\x00")
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(build, LABEL, manifest, archive)
    assert not archive.exists()


@pytest.mark.parametrize("schema", ["adr0022.capture/2", "adr0022.current-preparation/1",
                                    "adr0022.neutral-characterization/1", "adr0022.yaw-acquisition/1"])
def test_removed_acquisition_modes_cannot_be_packed(tmp_path, schema):
    manifest = dict(yaw_control_manifest(), schema=schema)
    archive = tmp_path / "removed.tar"
    with pytest.raises(ValueError, match="yaw-control or sensorless-homing"):
        pack(arm64_build(tmp_path), LABEL, manifest, archive)
    assert not archive.exists()


def test_homing_bundle_installs_with_homing_output(tmp_path):
    manifest = homing_manifest(tmp_path)
    archive = tmp_path / "homing.tar"
    assert pack(arm64_build(tmp_path), "homing-fixture", manifest, archive)["launch_option"] == "--establish-homing"
    firmware = tmp_path / "Firmware"
    firmware.mkdir()
    bound = json.loads(install(archive, "homing-fixture", firmware, tmp_path / "observations").read_text())
    assert bound["output"] == str((tmp_path / "observations").resolve() / "sensorless-homing.jsonl")
    assert bound["guards"]["current_bound_A"] == 6.5
    assert bound["native_settings"]["homing_limit_cur_A"] == 5


@pytest.mark.parametrize("missing", ["attendance", "authorization", "unattended_authorization", "authorization_purpose"])
def test_homing_bundle_refuses_missing_attendance_or_authorization(tmp_path, missing):
    manifest = homing_manifest(tmp_path)
    if missing == "unattended_authorization":
        manifest["session_authorization"]["unattended_operation_authorized"] = False
    elif missing == "authorization_purpose":
        manifest["session_authorization"]["purpose"] = "neutral_current_mode_verification"
    else:
        del manifest[{"attendance": "operator_attendance", "authorization": "session_authorization"}[missing]]
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(arm64_build(tmp_path), "homing-fixture", manifest, archive)
    assert not archive.exists()


@pytest.mark.parametrize("changed", ["guard_ceiling", "command_cap", "missing_basis", "peak_rating"])
def test_homing_bundle_keeps_protection_separate_from_command_cap(tmp_path, changed):
    manifest = homing_manifest(tmp_path)
    if changed == "guard_ceiling":
        manifest["guards"]["current_bound_A"] = 6.6
    elif changed == "command_cap":
        manifest["native_settings"]["homing_limit_cur_A"] = 6.5
    elif changed == "missing_basis":
        del manifest["protection_limit_basis"]
    else:
        manifest["protection_limit_basis"]["continuous_current_A"] = 23.
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(arm64_build(tmp_path), "homing-fixture", manifest, archive)
    assert not archive.exists()


def test_homing_missing_manufacturer_document_blocks_before_device_checks(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import adr0022_capture_launch
    import station_preflight
    firmware = tmp_path / "Firmware"
    shutil.copytree(arm64_build(tmp_path), firmware / "build")
    for relative in EXECUTABLES.values():
        (firmware / "build" / relative).chmod(0o755)
    manifest = homing_manifest(tmp_path)
    manifest["output"] = str(tmp_path / "unused-homing.jsonl")
    manifest["limits"]["startup_s"] = 3.
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    calls = []
    monkeypatch.setattr(adr0022_capture_launch.subprocess, "run",
                        lambda argv, **kwargs: calls.append(argv) or SimpleNamespace(returncode=0, stderr=""))
    monkeypatch.setattr(station_preflight, "_validate_can_spi_mapping",
                        lambda *args, **kwargs: pytest.fail("device checks reached before manufacturer document check"))
    assert not (firmware / PROTECTION_DOCUMENT).exists()
    with pytest.raises(ValueError, match="manufacturer protection document unavailable"):
        adr0022_capture_launch.preflight(manifest_path, firmware, establish_homing=True)
    assert calls == [[str(firmware / "build/axis_control_core/commissiond"), "--validate-homing", str(manifest_path)]]
    assert not (tmp_path / "unused-homing.attempt.json").exists()
