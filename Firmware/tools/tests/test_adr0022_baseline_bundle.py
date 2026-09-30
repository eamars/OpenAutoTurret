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
