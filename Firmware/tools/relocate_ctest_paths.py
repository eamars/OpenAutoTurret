#!/usr/bin/env python3
"""Rebase generated CTest paths when running a prebuilt release on another machine."""

import argparse
import os
import sys
from pathlib import Path


def cache_paths(cache_file: Path) -> tuple[str, str]:
    values: dict[str, str] = {}
    for line in cache_file.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key, value = line.split("=", 1)
        name = key.split(":", 1)[0]
        if name in {"CMAKE_HOME_DIRECTORY", "CMAKE_CACHEFILE_DIR"}:
            values[name] = value
    missing = {"CMAKE_HOME_DIRECTORY", "CMAKE_CACHEFILE_DIR"} - values.keys()
    if missing:
        raise ValueError(f"{cache_file} is missing {', '.join(sorted(missing))}")
    return values["CMAKE_HOME_DIRECTORY"], values["CMAKE_CACHEFILE_DIR"]


def relocate(build_dir: Path, source_dir: Path) -> int:
    cache_file = build_dir / "CMakeCache.txt"
    old_source, old_build = cache_paths(cache_file)
    new_source = os.path.abspath(source_dir)
    new_build = os.path.abspath(build_dir)
    files = sorted(build_dir.rglob("CTestTestfile.cmake"))
    if not files:
        raise ValueError(f"no CTestTestfile.cmake files under {build_dir}")

    changed = 0
    for path in files:
        original = path.read_text(encoding="utf-8")
        updated = original.replace(old_build, new_build).replace(old_source, new_source)
        if updated != original:
            path.write_text(updated, encoding="utf-8", newline="")
            changed += 1

    print(
        f"CTest paths ready: {len(files)} generated files, {changed} rebased; "
        f"source {old_source} -> {new_source}; build {old_build} -> {new_build}"
    )
    return changed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=Path)
    parser.add_argument("source_dir", type=Path)
    args = parser.parse_args()
    try:
        relocate(args.build_dir, args.source_dir)
    except (OSError, ValueError) as exc:
        print(f"cannot prepare the prebuilt CTest suite: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
