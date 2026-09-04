"""Round 66: a ringing criterion that can fail, sized against measured jitter rather than a source constant."""
import unittest

from tools.probe_track_loop import oscillation_verdict

BOX = 378.0                     # BOX_H_NORM * 1080: the declared person box height


def old_rule_flips(series):
    """C4's current rule, for contrast inside the tests."""
    s = [1 if v > 0 else -1 for v in series if abs(v) > 1.0]
    return sum(1 for a, b in zip(s, s[1:]) if a != b)


class OscillationVerdict(unittest.TestCase):
    def test_jitter_inside_the_band_is_not_called_oscillation(self) -> None:
        # Round-53 shape: stationary target, jitter to ~49 px. The 1 px rule saw 11 flips here.
        series = [24.9 * (1 if i % 2 else -1) + (i % 7) for i in range(40)]
        v = oscillation_verdict(series, BOX)
        self.assertIn("NO OSCILLATION DETECTED", v)
        self.assertGreaterEqual(old_rule_flips(series), 10, "the old rule should still be flapping here")

    def test_a_monotone_approach_is_convergence_not_ringing(self) -> None:
        series = [200.0 - 6.0 * i for i in range(40)]          # sweeps through zero once
        v = oscillation_verdict(series, BOX)
        self.assertIn("converging (single crossing)", v)

    def test_repeated_large_reversals_are_flagged(self) -> None:
        series = []
        for k in range(4):                                      # decaying overshoot either side
            series += [150.0 - 40.0 * k] * 6
            series += [-150.0 + 40.0 * k] * 6
        v = oscillation_verdict(series, BOX)
        self.assertIn("RINGING SUSPECTED", v)
        self.assertIn("box heights", v)

    def test_reversals_below_one_third_box_are_reported_as_small(self) -> None:
        series = []
        for k in range(4):
            series += [60.0] * 6
            series += [-60.0] * 6
        v = oscillation_verdict(series, BOX)
        self.assertIn("reversals present but small", v)
        self.assertNotIn("RINGING SUSPECTED", v)

    def test_a_short_window_refuses_to_judge(self) -> None:
        self.assertIn("INSUFFICIENT DATA", oscillation_verdict([10.0] * 5, BOX))

    def test_the_band_is_an_external_ruler_and_moves_with_it(self) -> None:
        # A fixed ±60 px reversal: below a loose ruler (shimmer), above a tight one (real reversal).
        series = []
        for _ in range(4):
            series += [60.0] * 6
            series += [-60.0] * 6
        loose = oscillation_verdict(series, BOX, jitter_px=100.0)
        tight = oscillation_verdict(series, BOX, jitter_px=20.0)
        self.assertIn("NO OSCILLATION DETECTED", loose)
        self.assertIn("reversal", tight)
        self.assertNotEqual(loose, tight, "the ruler must change the answer, or it is decoration")

    def test_large_reversals_are_flagged_even_when_frequent(self) -> None:
        # ±150 px alternating is oscillation by any operator's eye: beyond the ruler and beyond a third
        # of the box. An earlier draft of these tests expected shimmer to be forgiven here, which was the
        # self-referential band doing what it always does - so the expectation, not the metric, was wrong.
        series = []
        for _ in range(4):
            series += [150.0] * 6
            series += [-150.0] * 6
        self.assertIn("RINGING SUSPECTED", oscillation_verdict(series, BOX))


if __name__ == "__main__":
    unittest.main()
