"""The deploy's view of the station (tools/station_state.py): Homed or Shutdown, and never through a
calibration run -- a restart would end it unfinished, so the deploy refuses to activate."""
import sys

import pytest
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


def test_the_deploy_refuses_during_a_run():
    from deploy_station import refuse_during_run
    with pytest.raises(SystemExit, match="calibration run is in progress.*The verified release is /r/x"):
        refuse_during_run("calibrating", "/r/x")
    refuse_during_run("homed")
    refuse_during_run("shutdown")


def test_the_probe_is_asked_only_where_it_exists():
    from deploy_station import station_found
    seen = []

    def remote(command, **_):
        seen.append(command)
        return type("Done", (), {"stdout": answer})()
    answer = "calibrating\n"
    assert station_found(remote, "/v/python", "/ota/run/current/Firmware/tools/station_state.py") == "calibrating"
    assert seen[0].startswith("if [ -f /ota/run/current/Firmware/tools/station_state.py ]")
    answer = ""
    assert station_found(remote, "/v/python", "/x") == "shutdown"


def test_the_deploy_asks_before_it_builds_and_again_before_the_restart():
    src = (Path(__file__).resolve().parents[1] / "deploy_station.py").read_text(encoding="utf-8")
    early = src.index("refuse_during_run(station_found(remote")
    assert early < src.index("mktemp -d") < src.index("refuse_during_run(found, release)") < src.index("ln -sfn")
