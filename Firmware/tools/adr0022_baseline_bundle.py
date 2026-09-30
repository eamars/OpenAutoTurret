"""Package locally verified acquisition executables for a separate release.

Deployment uses deploy_station.py; this module never contacts or starts a station.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

EXECUTABLES = {"commissiond": "axis_control_core/commissiond", "imu": "imu-bno085"}
SCHEMA = "adr0022.baseline_bundle/1"
CURRENT_SCHEMA = "adr0022.acquisition_bundle/1"


def launch_option(manifest):
    if manifest.get("schema") == "adr0022.sensorless-homing/1":
        from adr0022_capture_launch import validate_current_contract
        validate_current_contract(manifest, establish_homing=True)
        return "--establish-homing"
    if manifest.get("schema") == "adr0022.neutral-characterization/1":
        from adr0022_capture_launch import validate_current_contract
        validate_current_contract(manifest, characterize_current=True)
        return "--characterize-current"
    if manifest.get("schema") == "adr0022.current-preparation/1":
        from adr0022_capture_launch import validate_current_contract
        validate_current_contract(manifest)
        return "--prepare-current"
    if manifest.get("schema") == "adr0022.capture/2":
        from adr0022_capture_launch import validate_additional_registers
        validate_additional_registers(manifest)
        return "--capture-baseline"
    raise ValueError("supported physical acquisition manifest required")


def validate_manifest(manifest, revision):
    option = launch_option(manifest)
    if manifest.get("provenance") != "MEASURED" or "output" in manifest or "imu_fd" in manifest:
        raise ValueError("unbound measured acquisition manifest required")
    if option != "--capture-baseline" and manifest["expected_revision"] != revision:
        raise ValueError("current preparation source revision differs from committed release")
    return option


def digest(data):
    return hashlib.sha256(data).hexdigest()


def pack(build, revision, manifest, output):
    if len(revision) != 40 or any(c not in "0123456789abcdef" for c in revision):
        raise ValueError("full committed revision required")
    option = validate_manifest(manifest, revision)
    content = {}
    for key, relative in EXECUTABLES.items():
        data = (Path(build) / relative).read_bytes()
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00":
            raise ValueError("ARM64 ELF required: " + relative)
        if digest(data) != manifest["expected_binaries"][key]:
            raise ValueError("manifest binary hash mismatch: " + key)
        content["build/" + relative] = data
    content["manifest.json"] = (json.dumps(manifest, indent=2, allow_nan=False) + "\n").encode()
    record = {"schema": CURRENT_SCHEMA if option != "--capture-baseline" else SCHEMA, "revision": revision,
              "launch_option": option,
              "files": {name: {"sha256": digest(data), "bytes": len(data)} for name, data in content.items()}}
    content["bundle.json"] = (json.dumps(record, indent=2) + "\n").encode()
    with Path(output).open("xb") as target, tarfile.open(fileobj=target, mode="w") as archive:
        for name, data in content.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o755 if name.startswith("build/") else 0o644
            archive.addfile(info, io.BytesIO(data))
    return validate(output, revision)


def validate(bundle, revision, firmware=None):
    allowed = {"bundle.json", "manifest.json", *("build/" + p for p in EXECUTABLES.values())}
    with tarfile.open(bundle, "r:") as archive:
        members = archive.getmembers()
        if len(members) != len(allowed) or {m.name for m in members} != allowed or any(not m.isfile() for m in members):
            raise ValueError("unexpected, duplicate or non-regular bundle member")
        content = {m.name: archive.extractfile(m).read() for m in members}
    record = json.loads(content.pop("bundle.json"))
    if record.get("schema") not in (SCHEMA, CURRENT_SCHEMA) or record.get("revision") != revision:
        raise ValueError("bundle source revision differs from committed release")
    if set(record["files"]) != set(content):
        raise ValueError("bundle identity incomplete")
    for name, data in content.items():
        if record["files"][name] != {"sha256": digest(data), "bytes": len(data)}:
            raise ValueError("bundle file identity differs: " + name)
    manifest = json.loads(content["manifest.json"])
    option = validate_manifest(manifest, revision)
    expected_schema = CURRENT_SCHEMA if option != "--capture-baseline" else SCHEMA
    if record["schema"] != expected_schema or record.get("launch_option", "--capture-baseline") != option:
        raise ValueError("acquisition launch mode identity differs")
    if option != "--capture-baseline" and firmware is not None:
        from adr0022_capture_launch import source_identity, verify_protection_basis
        if source_identity(firmware)["source_sha256"] != manifest["expected_source_sha256"]:
            raise ValueError("acquisition source SHA-256 differs from qualified manifest")
        if option == "--characterize-current":
            verify_protection_basis(manifest, firmware)
    for key, relative in EXECUTABLES.items():
        data = content["build/" + relative]
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00" or digest(data) != manifest["expected_binaries"][key]:
            raise ValueError("bundle executable identity invalid: " + key)
    return record


def install(bundle, revision, firmware, output_directory):
    """Verify again on the receiving host; do not execute a shipped executable."""
    record = validate(bundle, revision, firmware)
    firmware, output_directory = Path(firmware).resolve(), Path(output_directory).resolve()
    if (firmware / "build").exists() or (firmware / "build").is_symlink():
        raise ValueError("baseline needs an unused release build directory")
    output_directory.mkdir(parents=True, exist_ok=False)
    with tarfile.open(bundle, "r:") as archive:
        for relative in EXECUTABLES.values():
            name = "build/" + relative
            path = firmware / name
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as target:
                target.write(archive.extractfile(name).read())
            path.chmod(0o755)
        manifest = json.load(archive.extractfile("manifest.json"))
    name = {"--establish-homing": "sensorless-homing.jsonl", "--prepare-current": "current-preparation.jsonl", "--characterize-current": "current-characterization.jsonl", "--capture-baseline": "baseline.jsonl"}[launch_option(manifest)]
    manifest["output"] = str(output_directory / name)
    path = output_directory / "manifest.json"
    with path.open("x") as target:
        json.dump(manifest, target, indent=2, allow_nan=False)
        target.write("\n")
    with (output_directory / "bundle.json").open("x") as target:
        json.dump(record, target, indent=2)
        target.write("\n")
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    packing = commands.add_parser("pack")
    packing.add_argument("--build", type=Path, required=True)
    packing.add_argument("--manifest", type=Path, required=True)
    packing.add_argument("--output", type=Path, required=True)
    installing = commands.add_parser("install")
    installing.add_argument("--bundle", type=Path, required=True)
    installing.add_argument("--revision", required=True)
    installing.add_argument("--firmware", type=Path, required=True)
    installing.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "pack":
        repo = Path(__file__).resolve().parents[2]
        status = subprocess.run(["git", "status", "--porcelain"], cwd=repo, check=True, capture_output=True).stdout
        if status.strip():
            raise SystemExit("Commit source before packaging the baseline release")
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo, check=True, capture_output=True, text=True).stdout.strip()
        manifest = json.loads(args.manifest.read_text())
        if launch_option(manifest) != "--capture-baseline":
            from adr0022_capture_launch import source_identity, verify_protection_basis
            if source_identity(repo / "Firmware")["source_sha256"] != manifest["expected_source_sha256"]:
                raise SystemExit("Acquisition source differs from local qualification")
            if launch_option(manifest) == "--characterize-current":
                verify_protection_basis(manifest, repo / "Firmware")
        print(json.dumps(pack(args.build, revision, manifest, args.output), indent=2))
    else:
        print(install(args.bundle, args.revision, args.firmware, args.output_directory))
