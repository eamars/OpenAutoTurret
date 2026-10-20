#!/usr/bin/env python3
"""Cross-build a commit from a WSL-native mirror, not from the Windows checkout.

deploy_station.py --prebuilt on Windows used to run tools/cross_build.py inside WSL against
/mnt/c: the source, the 869 MB Debian sysroot and the build tree all sat behind WSL's 9p
filesystem, and a release that touched a widely included header took 3-4 minutes to compile
(2026-10-20). It also compiled the *working tree*, so an edit made during the build could ship
under the commit's name; deploy_station.py refused such a release after the fact.

This builds the commit itself, on the Linux filesystem:
  * a git mirror under ~/.cache/ota-cross/<name>/src, fetched from the checkout and checked out
    at exactly REVISION (git rewrites only the files that changed, so the build stays
    incremental and the station's `git archive` and the binaries are the same source);
  * the sysroot copied once to ~/.cache/ota-cross/<name>/sysroot (rsync, refreshed when the
    source copy changes);
  * the build tree in ~/.cache/ota-cross/<name>/build-arm64;
and packs the result into OUT (the tar deploy_station.py sends), inside WSL so the executable
bits survive.

Run inside WSL by deploy_station.py:
    cross_build_mirror.py --repo /mnt/c/.../OpenAutoTurret --revision SHA --out TAR
        [--sysroot /mnt/c/.../root] [--host-lib DIR]
"""
import argparse
import hashlib
import os
import subprocess
import sys
import time
from pathlib import Path


def run(cmd, **kwargs):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def mirror_source(repo: Path, revision: str, src: Path) -> None:
    if not (src / ".git").is_dir():
        src.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "init", "-q", src])
    # The checkout's HEAD is REVISION (deploy_station.py checked the tree was clean). Fetching it
    # by name rather than by hash needs no server-side settings.
    run(["git", "-C", src, "fetch", "-q", "--no-tags", "--depth=1", repo, "HEAD"])
    fetched = subprocess.run(["git", "-C", src, "rev-parse", "FETCH_HEAD"], check=True,
                             capture_output=True, text=True).stdout.strip()
    if fetched != revision:
        raise SystemExit(f"the checkout's HEAD is {fetched[:12]}, not {revision[:12]}: commit, "
                         "then deploy again")
    run(["git", "-C", src, "-c", "advice.detachedHead=false", "checkout", "-q", "--force", revision])
    run(["git", "-C", src, "clean", "-q", "-fdx"])


def mirror_sysroot(sysroot: Path, dest: Path) -> Path:
    # Refreshed only when the source copy's top-level listing changes: an rsync over 869 MB on
    # 9p takes longer than the build it is meant to speed up.
    stamp = hashlib.sha256("\n".join(
        f"{p.name}:{p.stat().st_mtime_ns}" for p in sorted(sysroot.iterdir())).encode()).hexdigest()
    marker = dest.parent / "sysroot.stamp"
    if not dest.is_dir() or not marker.is_file() or marker.read_text() != stamp:
        dest.mkdir(parents=True, exist_ok=True)
        run(["rsync", "-a", "--delete", f"{sysroot}/", f"{dest}/"])
        marker.write_text(stamp)
    return dest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--sysroot", type=Path)
    parser.add_argument("--host-lib", type=Path)
    parser.add_argument("--cache", type=Path, default=Path.home() / ".cache/ota-cross")
    args = parser.parse_args()

    started = time.monotonic()
    base = args.cache / args.repo.resolve().name
    src = base / "src"
    build = base / "build-arm64"
    mirror_source(args.repo.resolve(), args.revision, src)
    command = [sys.executable, src / "Firmware/tools/cross_build.py", "--build-dir", build]
    if args.sysroot:
        command += ["--sysroot", mirror_sysroot(args.sysroot.resolve(), base / "sysroot")]
    if args.host_lib:
        # A few small libraries the cross binutils load on every invocation: native too.
        host_lib = base / "host-lib"
        host_lib.mkdir(parents=True, exist_ok=True)
        run(["rsync", "-a", "--delete", f"{args.host_lib.resolve()}/", f"{host_lib}/"])
        command += ["--host-lib", host_lib]
    run(command, cwd=src)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    run(["tar", "-C", build.parent, "-cf", args.out,
         "--exclude=*.o", "--exclude=.ninja_deps", "--exclude=.ninja_log", "--exclude=_deps",
         build.name])
    print(f"cross build of {args.revision[:12]} from the native mirror: "
          f"{time.monotonic() - started:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
