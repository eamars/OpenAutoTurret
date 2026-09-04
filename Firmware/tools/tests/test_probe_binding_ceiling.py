"""Round 47/49: a legality check against the wrong ceiling certifies darts the station cannot follow."""
import unittest

from tools.probe_track_loop import binding_ceiling_deg_s


class BindingCeiling(unittest.TestCase):
    def test_published_ceiling_wins_even_when_it_is_the_smaller_number(self) -> None:
        # The whole point of the fix: controld says 10, the old arithmetic said 20.1.
        ceil_val, src = binding_ceiling_deg_s([10.0, 10.0], 30.0, [0.67])
        self.assertEqual(ceil_val, 10.0)
        self.assertIn("controld", src)

    def test_fallback_only_when_nothing_was_published(self) -> None:
        ceil_val, src = binding_ceiling_deg_s([None, None], 30.0, [0.67])
        self.assertAlmostEqual(ceil_val, 20.1, places=1)
        self.assertIn("FALLBACK", src)

    def test_missing_samples_do_not_disable_the_published_value(self) -> None:
        ceil_val, src = binding_ceiling_deg_s([None, 10.0, None], 30.0, [])
        self.assertEqual(ceil_val, 10.0)
        self.assertIn("controld", src)

    def test_the_tightest_window_in_an_interval_is_used(self) -> None:
        ceil_val, _ = binding_ceiling_deg_s([20.0, 10.0, 18.0], 30.0, [1.0])
        self.assertEqual(ceil_val, 10.0)

    def test_no_derate_and_nothing_published_still_yields_the_configured_speed(self) -> None:
        ceil_val, src = binding_ceiling_deg_s([], 30.0, [])
        self.assertEqual(ceil_val, 30.0)
        self.assertIn("FALLBACK", src)


if __name__ == "__main__":
    unittest.main()
