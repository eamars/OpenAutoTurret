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


class YawCheck(unittest.TestCase):
    def test_check_counts_a_probe_that_ended_early_as_a_failure(self):
        text = (HERE / "commission.py").read_text(encoding="utf-8")
        self.assertIn('[] if status == "COMPLETE" else [f"session {status}"]', text)


if __name__ == "__main__":
    unittest.main()
