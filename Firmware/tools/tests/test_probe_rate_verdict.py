"""Round 56/57: a rate check must not PASS on data too thin to judge - the always-pass failure mode.

The first published-rate verdict printed PASS with a maximum of 0.2 deg/s, while an earlier run of the same tool
reported a maximum of 10.0 from the same field. Both statements were true of their inputs; only one input was
worth anything.
"""
import unittest

from tools.probe_track_loop import rate_verdict


class RateVerdict(unittest.TestCase):
    def test_empty_sample_set_cannot_pass(self) -> None:
        v, n = rate_verdict([], 10.0)
        self.assertIn("INSUFFICIENT DATA", v)
        self.assertEqual(n, 0)
        self.assertNotIn("PASS", v)          # the round-56 defect, asserted by name

    def test_thin_sample_set_cannot_pass(self) -> None:
        v, _ = rate_verdict([0.2] * 19, 10.0)
        self.assertIn("INSUFFICIENT DATA", v)

    def test_idle_samples_do_not_count_as_moving(self) -> None:
        # Ninety samples at rest say nothing about a 10 deg/s ceiling; this was the round-56 shape.
        v, n = rate_verdict([0.2] * 90, 10.0)
        self.assertIn("INSUFFICIENT DATA", v)
        self.assertEqual(n, 0)

    def test_a_populated_set_within_the_ceiling_passes_and_says_so(self) -> None:
        v, n = rate_verdict([9.0] * 30 + [10.0] * 10, 10.0)
        self.assertTrue(v.startswith("PASS"), v)
        self.assertEqual(n, 40)
        self.assertIn("max 10.0 deg/s", v)

    def test_a_populated_set_over_the_ceiling_fails(self) -> None:
        v, _ = rate_verdict([10.6] * 25, 10.0)
        self.assertTrue(v.startswith("FAIL"), v)
        self.assertIn("25 over", v)

    def test_missing_ceiling_is_its_own_answer_not_a_pass(self) -> None:
        for bad in (None, 0.0, -1.0):
            v, _ = rate_verdict([9.0] * 40, bad)
            self.assertIn("NO CEILING", v)

    def test_the_verdict_always_names_its_sample_count(self) -> None:
        for data in ([], [0.2] * 5, [9.0] * 40, [10.6] * 40):
            v, n = rate_verdict(data, 10.0)
            self.assertTrue(any(str(n) in part or "samples" in part for part in [v]),
                            "verdict must state its evidence: %s" % v)


if __name__ == "__main__":
    unittest.main()
