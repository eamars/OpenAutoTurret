"""Package only the locally verified baseline executables for a separate release.

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


def digest(data):
    return hashlib.sha256(data).hexdigest()


def pack(build, revision, manifest, output):
    if len(revision) != 40 or any(c not in "0123456789abcdef" for c in revision):
        raise ValueError("full committed revision required")
    if manifest.get("schema") != "adr0022.capture/2" or manifest.get("provenance") != "MEASURED":
        raise ValueError("physical baseline manifest required")
    if "output" in manifest or "imu_fd" in manifest:
        raise ValueError("deployment and launcher bind output and IMU descriptor")
    content = {}
    for key, relative in EXECUTABLES.items():
        data = (Path(build) / relative).read_bytes()
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00":
            raise ValueError("ARM64 ELF required: " + relative)
        if digest(data) != manifest["expected_binaries"][key]:
            raise ValueError("manifest binary hash mismatch: " + key)
        content["build/" + relative] = data
    content["manifest.json"] = (json.dumps(manifest, indent=2, allow_nan=False) + "\n").encode()
    record = {"schema": SCHEMA, "revision": revision,
              "files": {name: {"sha256": digest(data), "bytes": len(data)} for name, data in content.items()}}
    content["bundle.json"] = (json.dumps(record, indent=2) + "\n").encode()
    with Path(output).open("xb") as target, tarfile.open(fileobj=target, mode="w") as archive:
        for name, data in content.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o755 if name.startswith("build/") else 0o644
            archive.addfile(info, io.BytesIO(data))
    return validate(output, revision)


def validate(bundle, revision):
    allowed = {"bundle.json", "manifest.json", *("build/" + p for p in EXECUTABLES.values())}
    with tarfile.open(bundle, "r:") as archive:
        members = archive.getmembers()
        if len(members) != len(allowed) or {m.name for m in members} != allowed or any(not m.isfile() for m in members):
            raise ValueError("unexpected, duplicate or non-regular bundle member")
        content = {m.name: archive.extractfile(m).read() for m in members}
    record = json.loads(content.pop("bundle.json"))
    if record.get("schema") != SCHEMA or record.get("revision") != revision:
        raise ValueError("bundle source revision differs from committed release")
    if set(record["files"]) != set(content):
        raise ValueError("bundle identity incomplete")
    for name, data in content.items():
        if record["files"][name] != {"sha256": digest(data), "bytes": len(data)}:
            raise ValueError("bundle file identity differs: " + name)
    manifest = json.loads(content["manifest.json"])
    if manifest.get("schema") != "adr0022.capture/2" or manifest.get("provenance") != "MEASURED" or "output" in manifest or "imu_fd" in manifest:
        raise ValueError("unbound measured baseline manifest required")
    for key, relative in EXECUTABLES.items():
        data = content["build/" + relative]
        if data[:4] != b"\x7fELF" or data[18:20] != b"\xb7\x00" or digest(data) != manifest["expected_binaries"][key]:
            raise ValueError("bundle executable identity invalid: " + key)
    return record


def install(bundle, revision, firmware, output_directory):
    """Verify again on the receiving host; do not execute a shipped executable."""
    record = validate(bundle, revision)
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
    manifest["output"] = str(output_directory / "baseline.jsonl")
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
        print(json.dumps(pack(args.build, revision, json.loads(args.manifest.read_text()), args.output), indent=2))
    else:
        print(install(args.bundle, args.revision, args.firmware, args.output_directory))
