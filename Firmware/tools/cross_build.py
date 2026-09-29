#!/usr/bin/env python3
"""Build the station's controller and test binaries here, for aarch64.

The station used to compile every release itself: roughly four minutes of each deploy, on a
machine whose job is holding a camera still. Nothing about compiling needs the hardware -- only
*running* the suite does, and that stays on the station (repo AGENTS.md).

GTest is built from the source Debian ships (/usr/src/googletest, from libgtest-dev), which is
the same version the station has installed (1.16.0-1, measured 2026-09-28) -- so no extra
arm64 package is needed for the tests, and the version cannot drift from the station's.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parent.parent
ARM64_LIBDIR = "/usr/lib/aarch64-linux-gnu"
GTEST_SOURCE = Path("/usr/src/googletest")


def run(cmd, **kwargs):
    """Run a command, and on failure print the command and the tail of its own output.

    A gate that fails silently is worse than no gate: the first cross-build failure here was a
    missing -lgtest, and the message that would have said so was swallowed by a shell redirect.
    """
    print("+", " ".join(str(part) for part in cmd), flush=True)
    proc = subprocess.run([str(part) for part in cmd], capture_output=True, text=True, **kwargs)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout[-4000:] or "")
        sys.stderr.write(proc.stderr[-4000:] or "")
        raise SystemExit(f"cross build failed (rc={proc.returncode}): {' '.join(map(str, cmd))}")
    return proc


def build_gtest(prefix: Path) -> None:
    """Configure, build and install GTest for the target, once, unless it is already there."""
    if (prefix / "lib/cmake/GTest/GTestConfig.cmake").exists():
        print(f"GTest for aarch64 already built: {prefix}")
        return
    if not GTEST_SOURCE.is_dir():
        raise SystemExit(
            f"no GTest source at {GTEST_SOURCE}: install libgtest-dev (amd64 is enough, it ships "
            "the source) or add libgtest-dev:arm64 and pass --system-gtest")
    source = prefix.parent / "gtest-build"
    run(["cmake", "-S", GTEST_SOURCE, "-B", source,
         "-DCMAKE_TOOLCHAIN_FILE=" + str(FIRMWARE / "cmake/aarch64-pi.toolchain.cmake"),
         "-DCMAKE_BUILD_TYPE=Release",
         f"-DCMAKE_INSTALL_PREFIX={prefix}", "-DBUILD_GMOCK=OFF"])
    run(["cmake", "--build", source, "--parallel", os.cpu_count() or 2])
    run(["cmake", "--install", source])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=FIRMWARE / "build-arm64",
                        help="where the aarch64 build tree lives (default Firmware/build-arm64)")
    parser.add_argument("--gtest", choices=("local", "system"), default="local",
                        help="build GTest from Debian's source here (default), or use the "
                             "system libgtest-dev:arm64 if the image grows it")
    parser.add_argument("--target", action="append",
                        help="build only this target (repeatable); default builds everything")
    args = parser.parse_args()

    build = args.build_dir
    deps = build / "_deps"
    if args.gtest == "local":
        build_gtest(deps / "gtest")

    # -DVAR=value is one token: split across two, CMake reads the second as a path and says so.
    configure = ["cmake", "-S", FIRMWARE, "-B", build, "-G", "Ninja",
                 "-DCMAKE_BUILD_TYPE=Release",
                 "-DCMAKE_TOOLCHAIN_FILE=" + str(FIRMWARE / "cmake/aarch64-pi.toolchain.cmake")]
    if args.gtest == "local":
        configure.append(f"-DGTest_DIR={deps / 'gtest/lib/cmake/GTest'}")
    # Pinned so find_package cannot quietly reach an amd64 config file and report a missing
    # library when the real problem is the architecture.
    for package in ("spdlog", "yaml-cpp", "fmt"):
        configure.append(f"-D{package}_DIR={ARM64_LIBDIR}/cmake/{package}")
    run(configure)

    if args.target:
        for target in args.target:
            run(["cmake", "--build", build, "--target", target, "--parallel", os.cpu_count() or 2])
    else:
        run(["cmake", "--build", build, "--parallel", os.cpu_count() or 2])

    # Verify what was actually produced rather than trusting the toolchain file's promises.
    shipped = sorted(p for p in build.rglob("*")
                     if p.is_file() and p.name.startswith(("controld", "test_", "probe_")))
    wrong = []
    for artifact in shipped:
        with artifact.open("rb") as handle:
            head = handle.read(20)
        # ELF, ET_EXEC/ET_DYN, EM_AARCH64 (183) at e_machine.
        if head[:4] != b"\x7fELF" or head[18:20] != b"\xb7\x00":
            wrong.append(artifact)
    for artifact in wrong:
        sys.stderr.write(f"not an aarch64 binary: {artifact}\n")
    if wrong:
        return 1
    runtime = [p for p in shipped if p.name == "controld"]
    if not runtime:
        sys.stderr.write("the build produced no controld; nothing to ship\n")
        return 1
    print(f"aarch64 build green: {len(shipped)} binaries, runtime: {runtime[0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
