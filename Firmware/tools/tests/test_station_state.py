"""The deploy's view of the station (tools/station_state.py): Homed or Shutdown, and never through a
calibration run -- a restart would end it unfinished, so the deploy refuses to activate."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from station_state import classify  # noqa: E402

UP = {"controld_connected": True, "soft_limits_valid": True, "phase": "hold"}


def test_homed_and_shutdown():
    assert classify(UP) == "homed"
    assert classify(dict(UP, phase="idle")) == "shutdown"
    assert classify(dict(UP, phase="fault")) == "shutdown"
    assert classify(dict(UP, controld_connected=False)) == "shutdown"
    assert classify({}) == "shutdown"


def test_a_run_in_progress_is_its_own_answer():
    assert classify(dict(UP, calibrating=True)) == "calibrating"
    assert classify(dict(UP, calibrating=False)) == "homed"
    assert classify(dict(UP, controld_connected=False, calibrating=True)) == "shutdown", "no controller, no run"


def test_the_deploy_checks_for_a_run_whatever_start_state_says():
    src = (Path(__file__).resolve().parents[1] / "deploy_station.py").read_text(encoding="utf-8")
    probe = src.index('found = remote(')
    assert probe < src.index('if found == "calibrating":') < src.index("state = found if args.start_state")
