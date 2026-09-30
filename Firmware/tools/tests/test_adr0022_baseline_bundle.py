"""Evidence identity and extraction boundaries for the dedicated baseline release."""
import hashlib
import io
import json
from pathlib import Path
import shutil
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from adr0022_baseline_bundle import EXECUTABLES, install, pack, validate
from adr0022_capture_launch import (SOURCE_FILES, SOURCE_DIRECTORIES, source_identity, canonical_sha,
                                   validate_current_contract, PROTECTION_DOCUMENT, PROTECTION_DOCUMENT_SHA256)


@pytest.fixture
def bundle(tmp_path):
    build = tmp_path / "build"
    hashes = {}
    for key, relative in EXECUTABLES.items():
        path = build / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        data = b"\x7fELF" + bytes(14) + b"\xb7\x00" + key.encode()
        path.write_bytes(data)
        hashes[key] = hashlib.sha256(data).hexdigest()
    manifest = {"schema": "adr0022.capture/2", "provenance": "MEASURED", "expected_binaries": hashes}
    archive = tmp_path / "bundle.tar"
    revision = "a" * 40
    pack(build, revision, manifest, archive)
    return archive, revision


def test_install_preserves_binary_identity_and_existing_evidence(tmp_path, bundle):
    archive, revision = bundle
    firmware, output = tmp_path / "Firmware", tmp_path / "observations"
    firmware.mkdir()
    manifest = install(archive, revision, firmware, output)
    bound = json.loads(manifest.read_text())
    assert bound["output"] == str(output / "baseline.jsonl")
    for key, relative in EXECUTABLES.items():
        assert hashlib.sha256((firmware / "build" / relative).read_bytes()).hexdigest() == bound["expected_binaries"][key]
    with pytest.raises(ValueError, match="unused release"):
        install(archive, revision, firmware, output)
    assert not (output / "baseline.jsonl").exists()


def test_bundle_cannot_be_relabelled_as_another_revision(bundle):
    with pytest.raises(ValueError, match="revision differs"):
        validate(bundle[0], "b" * 40)


@pytest.mark.parametrize("corruption", ["changed_bytes", "duplicate", "traversal", "symlink"])
def test_modified_or_unsafe_bundle_is_rejected_before_install(tmp_path, bundle, corruption):
    altered = tmp_path / "altered.tar"
    with tarfile.open(bundle[0]) as source, tarfile.open(altered, "w") as target:
        for entry in source:
            data = source.extractfile(entry).read()
            if corruption == "changed_bytes" and entry.name.endswith("commissiond"):
                data = data[:-1] + bytes([data[-1] ^ 1])
            target.addfile(entry, io.BytesIO(data))
        if corruption != "changed_bytes":
            name = "bundle.json" if corruption == "duplicate" else "../../escape"
            extra = tarfile.TarInfo(name)
            if corruption == "symlink":
                extra.type = tarfile.SYMTYPE
                extra.linkname = "/tmp/escape"
            target.addfile(extra, io.BytesIO())
    with pytest.raises(ValueError):
        install(altered, bundle[1], tmp_path / "Firmware", tmp_path / "evidence")
    assert not (tmp_path / "Firmware").exists() and not (tmp_path / "evidence").exists()


def current_contract(tmp_path, characterization=False):
    """Deliberately synthetic file fixture for package checks; no physical facts."""
    firmware, build = tmp_path / "Firmware", tmp_path / "build"
    for name in SOURCE_FILES:
        path = firmware / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"source fixture\r\n")
    for name in SOURCE_DIRECTORIES:
        path = firmware / name / "fixture.hpp"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"header fixture\r\n")
    binaries = {}
    for key, relative in EXECUTABLES.items():
        path = build / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        data = b"\x7fELF" + bytes(14) + b"\xb7\x00" + key.encode()
        path.write_bytes(data)
        binaries[key] = hashlib.sha256(data).hexdigest()
    revision = "a" * 40
    identity = source_identity(firmware)
    report = {"status": "LOCAL_CURRENT_PREPARATION_PASS", "expected_binaries": binaries,
              "source_sha256": identity["source_sha256"], "revision": revision,
              "hardware_accessed": False, "provenance": "SYNTHETIC"}
    manifest = {"schema": "adr0022.current-preparation/1", "purpose": "neutral_current_mode_verification",
                "provenance": "MEASURED", "pitch_supported_when_disabled": True,
                "neutral_current_bound_A": .1, "transition_displacement_bound_rad": .01,
                "pitch_maximum_temperature_C": 60., "expected_binaries": binaries,
                "expected_source_sha256": identity["source_sha256"], "expected_revision": revision,
                "operator_attendance": {"present_at_manual_cutoff": False, "operator_identity": "fixture",
                                        "manual_cutoff_evidence_identity": "fixture-manual-only-cutoff"},
                "session_authorization": {"purpose": "neutral_current_mode_verification", "current_mode_enable_authorized": True,
                                          "authorization_identity": "fixture-authorization", "unattended_operation_authorized": True,
                                          "presence_required": False},
                "local_qualification": {"report": report, "sha256": canonical_sha(report)}}
    if characterization:
        manifest.update(schema="adr0022.neutral-characterization/1", purpose="neutral_current_measurement_characterization",
                        protection_current_bound_A=6.5, neutral_observation_s=2., limits={"startup_s": 3., "duration_s": 30.},
                        protection_limit_basis={"kind": "manufacturer_continuous_current_rating", "document": PROTECTION_DOCUMENT,
                                                "sha256": PROTECTION_DOCUMENT_SHA256, "continuous_current_A": 6.5})
        manifest["session_authorization"]["purpose"] = manifest["purpose"]
        report["status"] = "LOCAL_CURRENT_CHARACTERIZATION_PASS"
        manifest["local_qualification"]["sha256"] = canonical_sha(report)
        document = firmware / PROTECTION_DOCUMENT
        document.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(Path(__file__).resolve().parents[2] / PROTECTION_DOCUMENT, document)
    return firmware, build, revision, manifest


def test_current_bundle_binds_portable_qualification_and_normalized_source(tmp_path):
    firmware, build, revision, manifest = current_contract(tmp_path)
    archive = tmp_path / "current.tar"
    record = pack(build, revision, manifest, archive)
    assert record["launch_option"] == "--prepare-current"
    assert source_identity(firmware)["normalization"] == "CRLF_TO_LF"
    for path in firmware.rglob("*"):
        if path.is_file():
            path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n"))
    assert source_identity(firmware)["source_sha256"] == manifest["expected_source_sha256"]
    validate(archive, revision, firmware)
    bound = json.loads(install(archive, revision, firmware, tmp_path / "observations").read_text())
    assert bound["output"] == str(tmp_path / "observations/current-preparation.jsonl")
    assert bound["local_qualification"] == manifest["local_qualification"]
    assert bound["operator_attendance"]["present_at_manual_cutoff"] is False


@pytest.mark.parametrize("missing", ["attendance", "authorization", "qualification", "unattended_authorization"])
def test_current_bundle_refuses_missing_physical_contract_before_writing(tmp_path, missing):
    _, build, revision, manifest = current_contract(tmp_path)
    if missing == "unattended_authorization":
        manifest["session_authorization"]["unattended_operation_authorized"] = False
    else:
        del manifest[{"attendance": "operator_attendance", "authorization": "session_authorization",
                      "qualification": "local_qualification"}[missing]]
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(build, revision, manifest, archive)
    assert not archive.exists()


def test_current_bundle_refuses_changed_source_before_install(tmp_path):
    firmware, build, revision, manifest = current_contract(tmp_path)
    archive = tmp_path / "current.tar"
    pack(build, revision, manifest, archive)
    (firmware / SOURCE_FILES[0]).write_text("changed source\n")
    with pytest.raises(ValueError, match="source SHA-256 differs"):
        install(archive, revision, firmware, tmp_path / "observations")
    assert not (firmware / "build").exists() and not (tmp_path / "observations").exists()


def test_characterization_bundle_distinct_purpose_and_output(tmp_path):
    firmware, build, revision, manifest = current_contract(tmp_path, characterization=True)
    archive = tmp_path / "characterization.tar"
    assert pack(build, revision, manifest, archive)["launch_option"] == "--characterize-current"
    bound = json.loads(install(archive, revision, firmware, tmp_path / "observations").read_text())
    assert bound["output"] == str(tmp_path / "observations/current-characterization.jsonl")
    assert bound["local_qualification"]["report"]["status"] == "LOCAL_CURRENT_CHARACTERIZATION_PASS"
    assert bound["protection_current_bound_A"] == 6.5 and bound["neutral_current_bound_A"] == .1


@pytest.mark.parametrize("changed", ["peak_current", "quality_bound", "document_identity", "old_qualification", "old_authorization"])
def test_characterization_bundle_refuses_wrong_limit_or_reused_purpose(tmp_path, changed):
    _, build, revision, manifest = current_contract(tmp_path, characterization=True)
    if changed == "peak_current":
        manifest["protection_current_bound_A"] = 23.
    elif changed == "quality_bound":
        manifest["neutral_current_bound_A"] = float("nan")
    elif changed == "document_identity":
        manifest["protection_limit_basis"]["sha256"] = "0" * 64
    elif changed == "old_qualification":
        manifest["local_qualification"]["report"]["status"] = "LOCAL_CURRENT_PREPARATION_PASS"
        manifest["local_qualification"]["sha256"] = canonical_sha(manifest["local_qualification"]["report"])
    else:
        manifest["session_authorization"]["purpose"] = "neutral_current_mode_verification"
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(build, revision, manifest, archive)
    assert not archive.exists()


def test_characterization_bundle_accepts_explicit_positive_diagnostic_bound(tmp_path):
    _, build, revision, manifest = current_contract(tmp_path, characterization=True)
    manifest["neutral_current_bound_A"] = .3
    assert pack(build, revision, manifest, tmp_path / "diagnostic.tar")["launch_option"] == "--characterize-current"


def test_characterization_bundle_changed_manufacturer_document_prevents_install(tmp_path):
    firmware, build, revision, manifest = current_contract(tmp_path, characterization=True)
    archive = tmp_path / "characterization.tar"
    pack(build, revision, manifest, archive)
    (firmware / PROTECTION_DOCUMENT).write_bytes(b"changed manual")
    with pytest.raises(ValueError, match="changed manufacturer protection document"):
        install(archive, revision, firmware, tmp_path / "observations")
    assert not (firmware / "build").exists() and not (tmp_path / "observations").exists()


@pytest.mark.parametrize("observation", [None, 0, 61, float("inf")])
def test_characterization_bundle_requires_bounded_explicit_observation(tmp_path, observation):
    _, build, revision, manifest = current_contract(tmp_path, characterization=True)
    if observation is None:
        del manifest["neutral_observation_s"]
    else:
        manifest["neutral_observation_s"] = observation
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError, match="neutral_observation_s"):
        pack(build, revision, manifest, archive)
    assert not archive.exists()


def test_characterization_bundle_duration_covers_explicit_observation(tmp_path):
    _, build, revision, manifest = current_contract(tmp_path, characterization=True)
    manifest["neutral_observation_s"] = 10.
    manifest["limits"]["duration_s"] = 13.
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError, match="duration must cover"):
        pack(build, revision, manifest, archive)
    assert not archive.exists()


def homing_contract(tmp_path):
    """Synthetic package/evidence fixture; never an actual physical qualification."""
    from adr0022_homing_rehearsal import fixture
    firmware, build, revision, manifest = current_contract(tmp_path)
    manifest.update(fixture([31001, 31002], 31003, -1, tmp_path / "unused.jsonl"))
    manifest.pop("imu_fd")
    manifest.pop("output")
    manifest.update(provenance="MEASURED", transport="socketcan",
                    yaw={"interface": "can0"}, pitch={"interface": "can1"})
    manifest["session_authorization"].update(purpose=manifest["purpose"], sensorless_homing_authorized=True)
    manifest["session_authorization"].pop("current_mode_enable_authorized")
    report = manifest["local_qualification"]["report"]
    report["status"] = "LOCAL_SENSORLESS_HOMING_PASS"
    manifest["local_qualification"]["sha256"] = canonical_sha(report)
    asset = {"schema": "adr0022.baseline_capabilities/1", "provenance": "MEASURED",
             "capture_integrity": "PASS", "pitch_uid": manifest["expected_pitch_uid"],
             "source": {"capture_sha256": "b" * 64}, "registers": {}}
    for name, index in (("expected_original_mode", "0x7005"), ("original_limit_cur_A", "0x7018"),
                        ("original_position_kp", "0x701e"), ("original_speed_kp", "0x701f"), ("original_speed_ki", "0x7020")):
        original = manifest["native_settings"][name]
        asset["registers"][index] = {"context": "PITCH_DISABLED_BASELINE",
                                      "observed": {"count": 1, "min": original, "max": original}}
    manifest["native_settings_evidence"] = {"asset": asset, "sha256": canonical_sha(asset)}
    return firmware, build, revision, manifest


def test_homing_bundle_binds_own_qualification_and_measured_originals(tmp_path):
    firmware, build, revision, manifest = homing_contract(tmp_path)
    archive = tmp_path / "homing.tar"
    assert pack(build, revision, manifest, archive)["launch_option"] == "--establish-homing"
    bound = json.loads(install(archive, revision, firmware, tmp_path / "observations").read_text())
    assert bound["output"] == str(tmp_path / "observations/sensorless-homing.jsonl")
    assert bound["native_settings_evidence"] == manifest["native_settings_evidence"]
    assert bound["local_qualification"]["report"]["status"] == "LOCAL_SENSORLESS_HOMING_PASS"


@pytest.mark.parametrize("changed", ["absent", "wrong_hash", "synthetic", "enabled", "missing_gain",
                                    "wrong_gain", "old_qualification", "old_authorization", "malformed"])
def test_homing_bundle_refuses_unmeasured_originals_or_reused_purpose(tmp_path, changed):
    _, build, revision, manifest = homing_contract(tmp_path)
    evidence = manifest["native_settings_evidence"]
    asset = evidence["asset"]
    if changed == "absent":
        del manifest["native_settings_evidence"]
    elif changed == "wrong_hash":
        evidence["sha256"] = "0" * 64
    elif changed == "synthetic":
        asset["provenance"] = "SYNTHETIC"
    elif changed == "enabled":
        asset["registers"]["0x701f"]["context"] = "PITCH_ENABLED"
    elif changed == "missing_gain":
        del asset["registers"]["0x7020"]
    elif changed == "wrong_gain":
        manifest["native_settings"]["original_speed_kp"] += 1
    elif changed == "old_qualification":
        report = manifest["local_qualification"]["report"]
        report["status"] = "LOCAL_CURRENT_CHARACTERIZATION_PASS"
        manifest["local_qualification"]["sha256"] = canonical_sha(report)
    elif changed == "old_authorization":
        manifest["session_authorization"]["purpose"] = "neutral_current_mode_verification"
    else:
        manifest["native_settings_evidence"] = []
    if changed in ("synthetic", "enabled", "missing_gain"):
        evidence["sha256"] = canonical_sha(asset)
    archive = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(build, revision, manifest, archive)
    assert not archive.exists()


def test_homing_bundle_accepts_exact_measured_zero_original_gain(tmp_path):
    _, build, revision, manifest = homing_contract(tmp_path)
    manifest["native_settings"]["original_speed_ki"] = 0.
    evidence = manifest["native_settings_evidence"]
    evidence["asset"]["registers"]["0x7020"]["observed"].update(min=0., max=0.)
    evidence["sha256"] = canonical_sha(evidence["asset"])
    assert pack(build, revision, manifest, tmp_path / "zero-original.tar")["launch_option"] == "--establish-homing"


@pytest.mark.parametrize("registers,reads,poll", [([0x701e, 0x701e], True, True), ([0x1234], True, True),
                                              ([True], True, True), ([0x701f], False, True), ([0x701f], True, False)])
def test_baseline_bundle_refuses_unbounded_or_enabled_additional_reads(tmp_path, bundle, registers, reads, poll):
    archive, revision = bundle
    with tarfile.open(archive) as source:
        manifest = dict(json.load(source.extractfile("manifest.json")), additional_startup_registers=registers,
                        register_reads=reads, pitch_stop_poll=poll)
    output = tmp_path / "refused.tar"
    with pytest.raises(ValueError):
        pack(tmp_path / "build", revision, manifest, output)
    assert not output.exists()


def test_baseline_bundle_allows_only_explicit_disabled_native_setting_reads(tmp_path, bundle):
    archive, revision = bundle
    with tarfile.open(archive) as source:
        manifest = dict(json.load(source.extractfile("manifest.json")), additional_startup_registers=[0x701e, 0x701f, 0x7020, 0x7017],
                        register_reads=True, pitch_stop_poll=True)
    assert pack(tmp_path / "build", revision, manifest, tmp_path / "native-settings.tar")["launch_option"] == "--capture-baseline"
