"""Offline ARM64 ELF dependency/version audit against an explicit target sysroot.

Never executes the inspected ELF, contacts a station, or qualifies physical I/O.
The report identifies the inspected libraries; an OS release name alone is not
evidence that the station has these exact files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect(path: Path, readelf: str) -> dict:
    def read(*args):
        return subprocess.run([readelf, "-W", *args, str(path)], check=True,
                              capture_output=True, text=True).stdout
    header = read("-h")
    if not all(re.search(pattern, header) for pattern in (
            r"Class:\s+ELF64", r"Data:.*little endian", r"Machine:\s+AArch64",
            r"Type:\s+(?:DYN|EXEC)")):
        raise ValueError(f"not a little-endian ARM64 executable/shared library: {path}")
    dynamic = read("-d")
    # No deployment should depend on the build workstation's absolute paths.
    search = re.findall(r"\((?:RPATH|RUNPATH)\).*?\[([^]]*)\]", dynamic)
    if any(p and not p.startswith("$ORIGIN") for value in search for p in value.split(":")):
        raise ValueError(f"nonportable ELF library search path: {path}: {search}")
    if search:
        raise ValueError(f"custom library search paths need an explicit bundle audit: {path}")
    needed = re.findall(r"\(NEEDED\).*?\[([^]]+)\]", dynamic)
    versions = read("--version-info")
    definitions, requirements = set(), {}
    section, provider = "", None
    for line in versions.splitlines():
        if line.startswith("Version definition section"):
            section = "definitions"
        elif line.startswith("Version needs section"):
            section = "needs"
        elif line.startswith("Version symbols section"):
            section = "symbols"
        name = re.search(r"\bName:\s+(\S+)", line)
        if section == "definitions" and name:
            definitions.add(name[1])
        elif section == "needs":
            file = re.search(r"\bFile:\s+(\S+)", line)
            if file:
                provider = file[1]
                requirements[provider] = []
            elif name:
                if provider is None:
                    raise ValueError(f"unscoped version requirement: {path}")
                requirements[provider].append(name[1])
    program = read("-l")
    interpreter = re.search(r"Requesting program interpreter:\s*([^]]+)", program)
    return {"path": str(path), "sha256": sha(path), "bytes": path.stat().st_size,
            "needed": needed, "required_versions": requirements,
            "defined_versions": sorted(definitions),
            "interpreter": interpreter[1] if interpreter else None}


def audit(binaries: list[Path], sysroot: Path, *, readelf="readelf") -> dict:
    sysroot = sysroot.resolve(strict=True)
    if not sysroot.is_dir() or sysroot == Path("/"):
        raise ValueError("a separate explicit target sysroot is required")
    files, links = {}, []

    def confined(path):
        resolved = path.resolve(strict=True)
        if not resolved.is_relative_to(sysroot):
            raise ValueError(f"target library escapes the sysroot: {path}")
        return resolved

    def find(name):
        if "/" in name or name in ("", ".", ".."):
            raise ValueError(f"unsupported dependency name: {name}")
        # These are the Debian aarch64 loader's standard library locations.
        # Cross-compiler-only /usr/aarch64-linux-gnu/lib is intentionally absent.
        for folder in ("lib/aarch64-linux-gnu", "usr/lib/aarch64-linux-gnu", "lib", "usr/lib"):
            path = sysroot / folder / name
            if path.is_file():
                return confined(path)
        raise ValueError(f"target dependency missing: {name}")

    def visit(path):
        path = path.resolve(strict=True)
        key = str(path)
        if key in files:
            return files[key]
        info = inspect(path, readelf)
        files[key] = info
        if info["interpreter"]:
            if info["interpreter"] != "/lib/ld-linux-aarch64.so.1":
                raise ValueError(f"unexpected ARM64 ELF interpreter: {path}")
            visit(confined(sysroot / info["interpreter"].lstrip("/")))
        for name in info["needed"]:
            supplied = visit(find(name))
            required = set(info["required_versions"].get(name, []))
            missing = required - set(supplied["defined_versions"])
            if missing:
                raise ValueError(f"{path.name}: {name} lacks versions {sorted(missing)}")
            links.append({"consumer_sha256": info["sha256"], "name": name,
                          "provider_sha256": supplied["sha256"], "versions": sorted(required)})
        if set(info["required_versions"]) - set(info["needed"]):
            raise ValueError(f"unresolved version-provider metadata: {path}")
        return info

    if not binaries:
        raise ValueError("at least one target binary is required")
    for binary in binaries:
        visit(binary)
    return {"schema": "adr0022.target_abi/1", "status": "SYSROOT_ABI_PASS",
            "sysroot": str(sysroot), "binaries": [str(p.resolve()) for p in binaries],
            "files": list(files.values()), "bindings": links,
            "station_accessed": False, "station_libraries_verified": False,
            "physical_parameters_qualified": False,
            "limits": ["ELF class, dependency closure and version labels checked against this sysroot only.",
                       "Does not execute code or prove symbol semantics, sensor behavior or target timing.",
                       "Exact station library identity and physical qualification remain separate requirements."]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, action="append", required=True)
    parser.add_argument("--sysroot", type=Path, required=True)
    parser.add_argument("--readelf", default="readelf")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.binary, args.sysroot, readelf=args.readelf)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2)
        output.write("\n")
    print(json.dumps({"status": report["status"], "files": len(report["files"]),
                      "bindings": len(report["bindings"]), "station_accessed": False}))
