"""ADR-005 D8: the gates VERIFY and a re-run decide on. The pitch gate used to pass anything; a yaw
check that ended early used to count."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import score  # noqa: E402


class PitchGate(unittest.TestCase):
    ROWS = [{"label": "ramp+5", "err_rms": 0.04}, {"label": "ramp-5", "err_rms": 0.045},
            {"label": "walker", "err_rms": 0.031}, {"label": "step+2.0", "err_rms": 0.3, "final_err": 0.004}]

    def test_the_accepted_asset_passes(self):
        self.assertEqual(score.gate_pitch(self.ROWS, {}), [])

    def test_a_loop_that_does_not_follow_fails(self):
        rows = [dict(r) for r in self.ROWS]
        rows[2]["err_rms"] = 0.9
        self.assertEqual(score.gate_pitch(rows, {}), ["walker: error RMS 0.900 deg (not tracking)"])

    def test_a_session_with_nothing_scored_fails(self):
        self.assertEqual(score.gate_pitch([], {}), ["no segments scored (the session ended early)"])

    def test_a_step_is_not_a_tracking_segment(self):
        # A step's error is large by design while it moves; its final error is the advisory's.
        rows = [dict(r) for r in self.ROWS]
        rows[3]["err_rms"] = 5.0
        self.assertEqual(score.gate_pitch(rows, {}), [])


class YawGate(unittest.TestCase):
    """Engineering gates (owner ruling 2026-10-25): a fault, a stuck or runaway axis, or an error that
    would take a tracked target out of the centre of the detail camera fails; a late start does not."""
    # MEASURE 2026-10-25 09:13, the accuracy campaign's first pass (rejected by the old 0.9-1.1 gate)
    ROWS = [{"label": "ramp+1", "v_ratio": 1.088, "err_max": 1.49}, {"label": "ramp-1", "v_ratio": 1.265, "err_max": 1.69},
            {"label": "ramp+20", "v_ratio": 1.006, "err_max": 1.2}, {"label": "rev-a", "v_ratio": 0.981, "err_max": 1.0},
            {"label": "rev-stop", "err_max": 1.6}, {"label": "walker", "err_max": 1.5},
            {"label": "step+0.5", "overshoot_pct": 83.0}, {"label": "long+90", "overshoot_pct": 0.4}]

    def test_a_late_start_that_catches_up_passes(self):
        self.assertEqual(score.gate_yaw(self.ROWS, {"stalls": 0}), [])

    def test_a_stuck_axis_fails(self):
        rows = [dict(r) for r in self.ROWS]
        rows[0]["v_ratio"] = 0.3
        self.assertEqual(score.gate_yaw(rows, {"stalls": 0}), ["ramp+1: speed ratio 0.300 (not tracking)"])

    def test_an_error_out_of_the_detail_frame_centre_fails(self):
        # MEASURE 09:58: a feedforward 0.3 A over threw ramp+1 3.9 deg ahead and the walker 3.6 off.
        rows = [dict(r) for r in self.ROWS]
        rows[0]["err_max"], rows[5]["err_max"] = 3.91, 3.64
        self.assertEqual(score.gate_yaw(rows, {"stalls": 0}),
                         ["ramp+1: error 3.91 deg (pointing)", "walker: error 3.64 deg (pointing)"])

    def test_overshoot_counts_in_degrees_not_percent(self):
        rows = [dict(r) for r in self.ROWS]
        rows[6]["overshoot_pct"] = 300.0  # 1.5 deg on a 0.5 deg step: inside the frame's centre
        rows[7]["overshoot_pct"] = 4.0  # 3.6 deg past a 90 deg move
        self.assertEqual(score.gate_yaw(rows, {"stalls": 0}), ["long+90: overshoot 3.60 deg (pointing)"])

    def test_more_than_four_stalls_fail(self):
        self.assertEqual(score.gate_yaw(self.ROWS, {"stalls": 5}), ["5 stalls"])


class YawCheck(unittest.TestCase):
    def test_check_counts_a_probe_that_ended_early_as_a_failure(self):
        text = (HERE / "commission.py").read_text(encoding="utf-8")
        self.assertIn('[] if status == "COMPLETE" else [f"session {status}"]', text)


if __name__ == "__main__":
    unittest.main()
