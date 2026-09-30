"""Evidence identity and extraction boundaries for the dedicated baseline release."""
import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from adr0022_baseline_bundle import EXECUTABLES, install, pack, validate
from adr0022_capture_launch import SOURCE_FILES, SOURCE_DIRECTORIES, source_identity, canonical_sha, validate_current_contract


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


def current_contract(tmp_path):
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
