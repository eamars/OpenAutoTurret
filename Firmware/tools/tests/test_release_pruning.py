"""A deploy keeps the release in force, the newest that were in force, and a build-only deploy's own;
it removes the rest, and never anything outside run/releases (owner, 2026-10-25: "each build takes heaps
of space"; 151 releases had reached 44 GB on the station. Review 2026-10-25: the first version, a shell
one-liner, ran its delete loop in $HOME when its `cd` failed, and counted failed deploys as newest)."""
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import prune_releases  # noqa: E402

needs_symlinks = pytest.mark.skipif(os.name == "nt", reason="run/current is a symlink on the station")


def station(tmp_path, names, current, activated=()):
    """run/releases/<names>, oldest first, run/current -> current; ACTIVATED in `activated`."""
    root = tmp_path / "ota"
    releases = root / "run" / "releases"
    now = time.time()
    for i, name in enumerate(names):
        (releases / name / "Firmware").mkdir(parents=True)
        if name in activated:
            (releases / name / "ACTIVATED").touch()
        os.utime(releases / name, (now - 1000 + i * 10, now - 1000 + i * 10))
    (root / "run" / "current").symlink_to(releases / current)
    return root, releases


def left(releases):
    return sorted(p.name for p in releases.iterdir())


@needs_symlinks
def test_kept_the_one_in_force_the_two_newest_that_were_and_nothing_else(tmp_path):
    names = [f"r{i}" for i in range(8)]
    root, releases = station(tmp_path, names, "r6", activated=("r1", "r3", "r4", "r6"))
    assert prune_releases.main([str(root)]) == 0
    assert left(releases) == ["r3", "r4", "r6"], "r7 (newest, never in force) and r5 go; r3, r4 are the rollback"


@needs_symlinks
def test_newest_in_force_means_when_it_went_live_not_when_its_directory_last_changed(tmp_path):
    # The first prune on the station (2026-10-25): markers written by hand into three releases moved
    # their directories' times, in the wrong order, and the latest of them was removed.
    root, releases = station(tmp_path, ["A", "B", "C", "D"], "D", activated=("A", "B", "C", "D"))
    now = time.time()
    for name, live in (("A", now - 300), ("B", now - 200), ("C", now - 100)):
        os.utime(releases / name / "ACTIVATED", (live, live))
    os.utime(releases / "A", (now, now))   # something wrote into A last
    prune_releases.main([str(root)])
    assert left(releases) == ["B", "C", "D"]


@needs_symlinks
def test_failed_deploys_never_push_out_the_last_good_release(tmp_path):
    # A in force; B and C failed (never ACTIVATED); D just activated.
    root, releases = station(tmp_path, ["A", "B", "C", "D"], "D", activated=("A", "D"))
    prune_releases.main([str(root)])
    assert left(releases) == ["A", "D"]


@needs_symlinks
def test_a_build_only_deploy_keeps_its_release(tmp_path):
    root, releases = station(tmp_path, ["A", "B", "N"], "A", activated=("A",))
    prune_releases.main([str(root), "--keep", str(releases / "N")])
    assert left(releases) == ["A", "N"]


@needs_symlinks
def test_nothing_goes_unless_run_current_is_a_release_under_run_releases(tmp_path):
    root, releases = station(tmp_path, ["A", "B", "C"], "A")
    (root / "run" / "current").unlink()
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    (root / "run" / "current").symlink_to(elsewhere)
    prune_releases.main([str(root)])
    assert left(releases) == ["A", "B", "C"]
    (root / "run" / "current").unlink()
    prune_releases.main([str(root)])   # no run/current at all
    assert left(releases) == ["A", "B", "C"]


def test_no_releases_directory_removes_nothing_and_touches_nothing_else(tmp_path):
    root = tmp_path / "ota"
    (root / "run").mkdir(parents=True)
    keep = tmp_path / "home-like"
    keep.mkdir()
    assert prune_releases.plan(root) == ([], f"no releases directory at {(root / 'run' / 'releases').resolve()}")
    assert keep.exists()


@needs_symlinks
def test_a_symlinked_releases_directory_still_keeps_the_one_in_force(tmp_path):
    # run/releases moved off the card and linked back: run/current resolves through it.
    root, releases = station(tmp_path, ["A", "B", "C", "D"], "D", activated=("D",))
    moved = tmp_path / "bigdisk" / "releases"
    moved.parent.mkdir()
    releases.rename(moved)
    (root / "run" / "releases").symlink_to(moved)
    (root / "run" / "current").unlink()
    (root / "run" / "current").symlink_to(root / "run" / "releases" / "D")
    prune_releases.main([str(root)])
    assert left(moved) == ["D"]


@needs_symlinks
def test_links_inside_run_releases_are_not_followed(tmp_path):
    root, releases = station(tmp_path, ["A", "B"], "B", activated=("B",))
    precious = tmp_path / "precious"
    (precious / "data").mkdir(parents=True)
    (releases / "link").symlink_to(precious)
    prune_releases.main([str(root)])
    assert (precious / "data").exists()
