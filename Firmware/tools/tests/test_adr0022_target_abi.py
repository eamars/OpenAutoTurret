"""Use real target ELF files and readelf, never fabricated readelf output."""
import hashlib
import os
from pathlib import Path
import shutil
import sys

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from adr0022_target_abi import audit

ROOT = TOOLS.parents[1]
TARGET = Path(os.environ.get("OTA_TARGET_BUILD", ROOT / "run/adr0022-debian13/firmware-make"))
SYSROOT = Path(os.environ.get("OTA_TARGET_SYSROOT", ROOT / "run/adr0022-debian13/root"))
BINARY = TARGET / "axis_control_core/commissiond"
pytestmark = pytest.mark.skipif(not BINARY.is_file() or shutil.which("readelf") is None,
                                reason="requires a locally cross-built ARM64 capture executable and readelf")


@pytest.fixture(scope="module")
def qualified_sysroot():
    return audit([BINARY, TARGET / "imu-bno085"], SYSROOT)


def sparse_copy(tmp_path, report):
    source_root = SYSROOT.resolve()
    target = tmp_path / "target"
    target.mkdir()
    for item in report["files"]:
        path = Path(item["path"])
        if path.is_relative_to(source_root):
            dest = target / path.relative_to(source_root)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, dest)
    for binding in report["bindings"]:
        provider = next(f for f in report["files"] if f["sha256"] == binding["provider_sha256"])
        path = target / Path(provider["path"]).relative_to(source_root)
        alias = path.with_name(binding["name"])
        if not alias.exists():
            alias.symlink_to(path.name)
    (target / "lib").symlink_to("usr/lib", target_is_directory=True)
    # The loader's interpreter name is a symlink in the original Debian package.
    (target / "usr/lib/ld-linux-aarch64.so.1").symlink_to("aarch64-linux-gnu/ld-linux-aarch64.so.1")
    return target


def test_actual_arm64_dependency_closure(qualified_sysroot):
    report = qualified_sysroot
    assert report["status"] == "SYSROOT_ABI_PASS"
    assert not report["station_accessed"] and not report["station_libraries_verified"]
    assert not report["physical_parameters_qualified"]
    target_hash = hashlib.sha256(BINARY.read_bytes()).hexdigest()
    assert any(b["consumer_sha256"] == target_hash and b["name"] == "libyaml-cpp.so.0.8"
               for b in report["bindings"])


def test_host_executable_is_rejected():
    with pytest.raises(ValueError, match="not a little-endian ARM64"):
        audit([Path(sys.executable)], SYSROOT)


def test_missing_library_fails_with_complete_other_libraries(tmp_path, qualified_sysroot):
    target = sparse_copy(tmp_path, qualified_sysroot)
    audit([BINARY], target)  # the sparse copy actually loads the same dependency set
    (target / "usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.8").unlink()
    with pytest.raises(ValueError, match="target dependency missing: libyaml-cpp"):
        audit([BINARY], target)


def test_real_elf_missing_required_version_is_rejected(tmp_path, qualified_sysroot):
    target = sparse_copy(tmp_path, qualified_sysroot)
    binding = next(b for b in qualified_sysroot["bindings"] if b["versions"])
    provider = next(f for f in qualified_sysroot["files"] if f["sha256"] == binding["provider_sha256"])
    path = target / Path(provider["path"]).relative_to(SYSROOT.resolve())
    name = binding["versions"][0].encode()
    data = path.read_bytes()
    assert name in data
    # Keep ELF offsets/string lengths intact while removing the real required
    # version label. This is a deliberately incompatible local library copy.
    path.write_bytes(data.replace(name, b"X" * len(name)))
    with pytest.raises(ValueError, match="lacks versions"):
        audit([BINARY], target)
