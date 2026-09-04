"""Round 56/58/59: one unit, one accessor - the defect was rad/s meeting a deg/s ceiling."""
import math, unittest

from tools.probe_track_loop import rate_verdict, ref_rate_deg_s


class RateUnits(unittest.TestCase):
    def test_ten_deg_per_second_is_its_own_ceiling_in_the_right_unit(self) -> None:
        row = {"ref_v": math.radians(10.0)}            # exactly what controld publishes
        self.assertAlmostEqual(ref_rate_deg_s(row), 10.0, places=6)

    def test_the_round_56_defect_cannot_recur(self) -> None:
        # Raw rad/s against a deg/s ceiling could never fail: 0.1745 < 10.5. Converted, it sits at the
        # ceiling and must not be reported over it - and at 12 deg/s it must be.
        at_ceiling = [{"ref_v": math.radians(10.0)}] * 40
        v, n = rate_verdict([ref_rate_deg_s(r) for r in at_ceiling], 10.0)
        self.assertTrue(v.startswith("PASS"), v)
        self.assertEqual(n, 40)
        over = [{"ref_v": math.radians(12.0)}] * 40
        v2, _ = rate_verdict([ref_rate_deg_s(r) for r in over], 10.0)
        self.assertTrue(v2.startswith("FAIL"), v2)

    def test_missing_or_non_numeric_rate_is_none_not_zero(self) -> None:
        self.assertIsNone(ref_rate_deg_s({}))
        self.assertIsNone(ref_rate_deg_s({"ref_v": None}))
        self.assertIsNone(ref_rate_deg_s({"ref_v": "n/a"}))

    def test_sign_is_discarded_because_the_ceiling_is_on_magnitude(self) -> None:
        self.assertAlmostEqual(ref_rate_deg_s({"ref_v": -math.radians(4.0)}), 4.0, places=6)


class SitesAgree(unittest.TestCase):
    def test_both_print_sites_consume_the_same_converted_list(self) -> None:
        import pathlib
        src = pathlib.Path("tools/probe_track_loop.py").read_text()
        self.assertIn('rv = sorted(x for x in (ref_rate_deg_s(r) for r in all_rows) if x is not None)', src)
        self.assertNotIn('rv[-1] * r2d', src,
                         "the profile site must not convert again - that double conversion is what made the two sites disagree")


if __name__ == "__main__":
    unittest.main()
