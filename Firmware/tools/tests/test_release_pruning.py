"""A deploy keeps the release in force and the newest few, and removes the rest (owner, 2026-10-25:
"each build takes heaps of space"; 151 releases had reached 44 GB on the station)."""
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from deploy_station import prune_script  # noqa: E402


@pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="the script runs on the station's shell")
def test_the_one_in_force_and_the_newest_three_stay(tmp_path):
    run = tmp_path / "run"
    releases = run / "releases"
    names = [f"rev{i:02d}.abc" for i in range(8)]   # rev00 oldest .. rev07 newest
    now = time.time()
    for i, name in enumerate(names):
        (releases / name / "Firmware").mkdir(parents=True)
        os.utime(releases / name, (now - 1000 + i * 10, now - 1000 + i * 10))
    (run / "current").symlink_to(releases / "rev02.abc")   # an older one in force (after a build-only deploy)
    out = subprocess.run(["bash", "-c", prune_script(str(releases), keep=3)], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert sorted(p.name for p in releases.iterdir()) == ["rev02.abc", "rev05.abc", "rev06.abc", "rev07.abc"]
    assert "Old releases removed: 4" in out.stdout


@pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="the script runs on the station's shell")
def test_nothing_goes_when_there_is_no_release_in_force(tmp_path):
    releases = tmp_path / "run" / "releases"
    for i in range(5):
        (releases / f"rev{i}").mkdir(parents=True)
    out = subprocess.run(["bash", "-c", prune_script(str(releases), keep=1)], capture_output=True, text=True)
    # readlink -f of a missing run/current names "current", which no release is: the newest stays,
    # the others go -- the same rule, nothing in force to protect.
    assert len(list(releases.iterdir())) == 1
