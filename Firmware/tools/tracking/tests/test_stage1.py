"""ADR-003 stage 1 as a test: all 14 scenarios pass their checks with the stage-1 prior, and
the Level-1 target feedforward beats feedback only wherever the subject moves steadily."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sim  # noqa: E402
import stage1  # noqa: E402

PRIOR = sim.tracking_parameters("tracking_prior.json")


def test_all_scenarios_pass():
    results = stage1.run_all(PRIOR)
    failed = [f"{sid}: {c['check']} = {c['value']} (limit {c['limit']})"
              for sid, r in results.items() for c in stage1.checks(sid, r, PRIOR) if not c["pass"]]
    assert not failed, "\n".join(failed)


def test_target_feedforward_removes_moving_lag():
    results = stage1.run_all(PRIOR, ["T04", "T08", "T11"])
    for sid, c in stage1.comparison(results).items():
        assert abs(c["lag_s"]["ff_fb"]) < 0.1 * abs(c["lag_s"]["fb_only"]), sid
        assert c["framing_rms_px"]["ff_fb"] < 0.2 * c["framing_rms_px"]["fb_only"], sid


def test_simulation_is_deterministic():
    import numpy as np
    request = stage1.scenarios.build("T12", PRIOR)
    a, b = sim.run(request), sim.run(request)
    assert np.array_equal(a[0]["qr_y"], b[0]["qr_y"]) and np.array_equal(a[1]["err_px"], b[1]["err_px"], equal_nan=True)
