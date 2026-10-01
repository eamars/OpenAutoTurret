"""Package acquisition executables and an operation manifest for a separate release.

Deployment uses deploy_station.py; this module never contacts or starts a station.
"""
import argparse
import io
import json
from pathlib import Path
import re
import tarfile

EXECUTABLES = {"commissiond": "axis_control_core/commissiond", "imu": "imu-bno085"}
SCHEMA = "adr0022.baseline_bundle/1"
CURRENT_SCHEMA = "adr0022.acquisition_bundle/1"


def launch_option(manifest):
    if manifest.get("schema") == "adr0022.yaw-control/1":
        from adr0022_capture_launch import validate_yaw_control_contract
        validate_yaw_control_contract(manifest)
        return "--control-yaw"
    if manifest.get("schema") == "adr0022.yaw-acquisition/1":
        from adr0022_capture_launch import validate_yaw_contract
        validate_yaw_contract(manifest)
        return "--acquire-yaw"
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


def validate_session_label(session_label):
    if not isinstance(session_label, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", session_label):
        raise ValueError("session label must contain letters, digits, dots, underscores or hyphens")
    return session_label


def validate_manifest(manifest, session_label):
    option = launch_option(manifest)
    if manifest.get("provenance") != "MEASURED" or "output" in manifest or "imu_fd" in manifest:
        raise ValueError("unbound measured acquisition manifest required")
    if manifest.get("session_label", session_label) != session_label:
        raise ValueError("manifest session label differs from acquisition bundle")
    return option


def pack(build, session_label, manifest, output):
    session_label = validate_session_label(session_label)
    manifest = dict(manifest)
    manifest.setdefault("session_label", session_label)
    option = validate_manifest(manifest, session_label)
    content = {}
    for key, relative in EXECUTABLES.items():
        data = (Path(build) / relative).read_bytes()
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00":
            raise ValueError("ARM64 ELF required: " + relative)
        content["build/" + relative] = data
    content["manifest.json"] = (json.dumps(manifest, indent=2, allow_nan=False) + "\n").encode()
    record = {"schema": CURRENT_SCHEMA if option != "--capture-baseline" else SCHEMA, "session_label": session_label,
              "launch_option": option,
              "files": {name: {"bytes": len(data), "mode": 0o755 if name.startswith("build/") else 0o644}
                        for name, data in content.items()}}
    content["bundle.json"] = (json.dumps(record, indent=2) + "\n").encode()
    with Path(output).open("xb") as target, tarfile.open(fileobj=target, mode="w") as archive:
        for name, data in content.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o755 if name.startswith("build/") else 0o644
            archive.addfile(info, io.BytesIO(data))
    return validate(output, session_label)


def validate(bundle, session_label=None, firmware=None):
    allowed = {"bundle.json", "manifest.json", *("build/" + p for p in EXECUTABLES.values())}
    with tarfile.open(bundle, "r:") as archive:
        members = archive.getmembers()
        if len(members) != len(allowed) or {m.name for m in members} != allowed or any(not m.isfile() for m in members):
            raise ValueError("unexpected, duplicate or non-regular bundle member")
        content = {m.name: archive.extractfile(m).read() for m in members}
        modes = {m.name: m.mode for m in members}
    record = json.loads(content.pop("bundle.json"))
    label = validate_session_label(record.get("session_label"))
    if record.get("schema") not in (SCHEMA, CURRENT_SCHEMA) or (session_label is not None and label != session_label):
        raise ValueError("bundle session label differs from requested acquisition")
    if set(record["files"]) != set(content):
        raise ValueError("bundle identity incomplete")
    for name, data in content.items():
        expected_mode = 0o755 if name.startswith("build/") else 0o644
        if record["files"][name] != {"bytes": len(data), "mode": expected_mode} or modes[name] != expected_mode:
            raise ValueError("bundle file size or mode differs: " + name)
    manifest = json.loads(content["manifest.json"])
    option = validate_manifest(manifest, label)
    expected_schema = CURRENT_SCHEMA if option != "--capture-baseline" else SCHEMA
    if record["schema"] != expected_schema or record.get("launch_option", "--capture-baseline") != option:
        raise ValueError("acquisition launch mode identity differs")
    for key, relative in EXECUTABLES.items():
        data = content["build/" + relative]
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00":
            raise ValueError("ARM64 ELF required: " + key)
    return record


def install(bundle, session_label, firmware, output_directory):
    """Verify again on the receiving host; do not execute a shipped executable."""
    record = validate(bundle, session_label, firmware)
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
    name = {"--control-yaw": "yaw-control.jsonl", "--acquire-yaw": "yaw-acquisition.jsonl", "--establish-homing": "sensorless-homing.jsonl", "--prepare-current": "current-preparation.jsonl", "--characterize-current": "current-characterization.jsonl", "--capture-baseline": "baseline.jsonl"}[launch_option(manifest)]
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
    packing.add_argument("--session-label", required=True)
    packing.add_argument("--output", type=Path, required=True)
    installing = commands.add_parser("install")
    installing.add_argument("--bundle", type=Path, required=True)
    installing.add_argument("--session-label", required=True)
    installing.add_argument("--firmware", type=Path, required=True)
    installing.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "pack":
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        print(json.dumps(pack(args.build, args.session_label, manifest, args.output), indent=2))
    else:
        print(install(args.bundle, args.session_label, args.firmware, args.output_directory))
