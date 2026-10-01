"""Selection coverage after the exact local and actual full-motor probes."""
import copy
import unittest

from Firmware.commissioning.contracts import Rejected
from Firmware.tools.adr0022_position_response_synthesis_probe import (WN_GRID, ZETA_GRID,
    POSITION_RATIO_GRID, rank_local_candidates)


def records(feasible=()):
    return [{"WN_rad_s": wn, "damping_ratio": zeta, "Kpos_over_WN": ratio,
        "passed": (wn, zeta, ratio) in feasible,
        "points": [{"passed": (wn, zeta, ratio) in feasible} for _ in (-1, 1)]}
        for wn in WN_GRID for zeta in ZETA_GRID for ratio in POSITION_RATIO_GRID]


class PositionResponseSelectionTests(unittest.TestCase):
    def test_maximum_three_candidates_use_fastest_bandwidth_then_damping_then_position_ratio(self):
        values = records(((4., 2., .2), (4., 2., 1.), (4., 1.5, .5), (2., 1., 1.)))
        chosen = rank_local_candidates(values)
        self.assertEqual([(r["WN_rad_s"], r["damping_ratio"], r["Kpos_over_WN"]) for r in chosen],
            [(4., 1.5, .5), (4., 2., 1.), (4., 2., .2)])
        self.assertEqual(len(values), 72)  # selection keeps the complete retained curve

    def test_either_direction_failure_or_missing_operating_point_prevents_selection(self):
        values = records(((2., 1.5, .5), (1., 2., .5)))
        first = next(r for r in values if r["WN_rad_s"] == 2. and r["damping_ratio"] == 1.5 and r["Kpos_over_WN"] == .5)
        first["points"][1]["passed"] = False
        second = next(r for r in values if r["WN_rad_s"] == 1. and r["damping_ratio"] == 2. and r["Kpos_over_WN"] == .5)
        second["points"].pop()
        self.assertEqual(rank_local_candidates(values), [])
        self.assertEqual(rank_local_candidates(records()), [])

    def test_missing_duplicate_reordered_or_extra_grid_points_do_not_support_the_search_decision(self):
        for operation in ("missing", "duplicate", "reordered", "extra"):
            values = records()
            if operation == "missing": values.pop()
            elif operation == "duplicate": values[-1] = copy.deepcopy(values[-2])
            elif operation == "reordered": values[0], values[1] = values[1], values[0]
            else: values.append(copy.deepcopy(values[-1]))
            with self.subTest(operation=operation):
                with self.assertRaises(Rejected): rank_local_candidates(values)


if __name__ == "__main__": unittest.main()
