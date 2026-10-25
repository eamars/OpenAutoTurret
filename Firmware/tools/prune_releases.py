#!/usr/bin/env python3
"""Remove old releases from the station's run/releases (owner, 2026-10-25: "each build takes heaps of
space"; 151 releases had reached 44 GB of the card). Standard library only; runs on the station.

Kept: the release run/current points at; the newest KEEP_ACTIVATED other releases that were once in
force (the deploy marks a release ACTIVATED when it goes live), to roll back to; and any --keep path
(the release a build-only deploy just made). Everything else directly under run/releases goes.
A release that failed its deploy, or a probe build, never pushes out the last good one.

It removes nothing unless run/current resolves to a directory directly under run/releases, and it
only ever removes real directories directly under run/releases (never a symlink, never a path
outside it).

    prune_releases.py ROOT [--keep RELEASE_DIR] [--dry-run]
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

KEEP_ACTIVATED = 2
MARKER = "ACTIVATED"


def plan(root: Path, keep: list[Path] = ()) -> tuple[list[Path], str]:
    """(the releases to remove, why nothing is removed when the list is empty for a reason)."""
    releases = (root / "run" / "releases").resolve()
    if not releases.is_dir():
        return [], f"no releases directory at {releases}"
    current = (root / "run" / "current").resolve()
    if current.parent != releases or not current.is_dir():
        return [], f"run/current ({current}) is not a release under {releases}: nothing removed"
    kept = {current} | {Path(k).resolve() for k in keep}
    children = [d for d in releases.iterdir() if d.is_dir() and not d.is_symlink()]
    # Newest in force by when each went live: the marker's own time. Not the directory's, which any
    # write inside it moves (writing the markers themselves did, and the first prune on the station
    # kept two older releases over the latest one, 2026-10-25).
    activated = sorted((d for d in children if (d / MARKER).is_file() and d not in kept),
                       key=lambda d: (d / MARKER).stat().st_mtime, reverse=True)
    kept |= set(activated[:KEEP_ACTIVATED])
    return [d for d in children if d not in kept], ""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", type=Path)
    ap.add_argument("--keep", type=Path, action="append", default=[])
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    doomed, why = plan(a.root, a.keep)
    if why:
        print(f"Old releases: none removed ({why})")
        return 0
    for d in doomed:
        if not a.dry_run:
            shutil.rmtree(d)
    print(f"Old releases {'to remove' if a.dry_run else 'removed'}: {len(doomed)} (kept run/current, the "
          f"{KEEP_ACTIVATED} newest that were in force, and this deploy's)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
