"""The frozen planner: a campaign design is computed once, refused loudly, and cannot be edited later.

The ADR-002 run's own report says the next numbers were chosen by hand, round by round, with no design
and no stop criterion — and that the folder named kp2-fine contained Kp=1. Freezing the design into a
hash-locked document is the mechanical answer to both, so these tests are mostly about refusals: the
ways a plan quietly stops being a plan.
"""

from __future__ import annotations

import copy
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
FIRMWARE = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)

import adr0021_plan as plan  # noqa: E402
import adr0021_scorer as scoring  # noqa: E402


def spec() -> dict:
    return {
        "campaign_id": "yaw-current-unittest",
        "objective": "jitter at a fixed velocity",
        "dimensions": [
            {"name": "yaw.current_kp_a_per_rad_s", "levels": [0.6, 1.0, 1.6, 2.4]},
            {"name": "yaw.current_ki_a_per_rad_s", "levels": [0.3, 0.6, 1.0, 1.5]},
        ],
        "refine": {"max_candidates": 8, "gate": "relative improvement > 0.15",
                   "levels_per_dimension": 2},
        "confirm": {"repeats": 3},
        "stop": {"max_trials": 30, "no_improvement_rounds": 2},
        "fixed": {"velocity_dps": 6.0, "hold_ms": 1200},
        # 00_CODEX_START.md:36: the scoring basis, the seed, the geometry identity and the retry rule are
        # part of the lock, so a fixture that omits them would be testing a planner that lets a real
        # campaign omit them too.
        "scorer": {"metric": "jitter_rad_s_pp", "worse_is_better": False,
                   "metrics_version": scoring.METRICS_VERSION,
                   "metrics_sha256": scoring.metrics_sha256()},
        "seed": 20260930,
        "geometry_calibration": "BLOCKED_geometry_identity_not_measured_this_session",
        "retry": {"allowed": 1, "same_parameters": True, "same_conditions": True},
        "payload_profile": "no_payload",
    }


class FreezingADesign(unittest.TestCase):
    def setUp(self):
        self.inventory = plan.load_inventory(plan.INVENTORY)

    def test_a_square_grid_freezes_to_the_number_the_adr_names(self):
        lock = plan.freeze(spec(), self.inventory)
        self.assertEqual(16, len(lock["design"]["coarse"]))
        self.assertEqual(["baseline", "coarse", "refine_if_gate_passes", "confirm", "report"],
                         lock["sequence"])

    def test_the_same_design_from_two_operators_hashes_the_same(self):
        left = plan.freeze(spec(), self.inventory)
        reordered = spec()
        reordered["dimensions"].reverse()                       # typed the other way round
        right = plan.freeze(reordered, self.inventory)
        self.assertEqual(left["design_sha256"], right["design_sha256"],
                         "an order-dependent hash would let two people claim the same design "
                         "while running different sequences")
        self.assertEqual(left["design"]["coarse"], right["design"]["coarse"])

    def test_every_candidate_states_every_dimension(self):
        lock = plan.freeze(spec(), self.inventory)
        for candidate in lock["design"]["coarse"]:
            self.assertEqual({"yaw.current_kp_a_per_rad_s", "yaw.current_ki_a_per_rad_s"},
                             set(candidate["params"]), candidate["candidate_id"])

    def test_the_lock_is_bound_to_the_inventory_and_binary_it_was_frozen_against(self):
        bound = plan.freeze(spec(), self.inventory)["bound_to"]
        for key in ("inventory_sha256", "binary_sha256", "source_rev", "config", "hardware_profile"):
            self.assertTrue(bound[key], f"{key} missing from the binding")
            if key.endswith("sha256"):
                self.assertEqual(64, len(bound[key]), f"{key} is not a digest")
        self.assertEqual(self.inventory["generated_from"]["binary_sha256"], bound["binary_sha256"])


class RefusalsAreThePoint(unittest.TestCase):
    def setUp(self):
        self.inventory = plan.load_inventory(plan.INVENTORY)

    def refuse(self, mutate) -> str:
        broken = spec()
        mutate(broken)
        with self.assertRaises(ValueError) as caught:
            plan.freeze(broken, self.inventory)
        return str(caught.exception)

    def test_the_tuner_may_not_widen_its_own_current_envelope(self):
        reason = self.refuse(lambda s: s["dimensions"].append(
            {"name": "yaw.host_current_limit_a", "levels": [0.8, 1.0]}))
        self.assertIn("protected_read_only", reason)
        self.assertIn("D7", reason, "the refusal should name the clause that protects the field")

    def test_a_field_that_is_not_in_the_inventory_at_all_is_refused(self):
        reason = self.refuse(lambda s: s["dimensions"].append(
            {"name": "yaw.drag_coefficient", "levels": [1, 2]}))
        self.assertIn("generated from the binary", reason)

    def test_a_design_resized_to_fit_the_grid_has_to_say_so(self):
        def shrink(s):
            s["dimensions"][1]["levels"] = [0.6, 1.0]     # 4 x 2 = 8, not the frozen 16
        reason = self.refuse(shrink)
        self.assertIn("16", reason)
        allowed = spec()
        allowed["dimensions"][1]["levels"] = [0.6, 1.0]
        allowed["coarse_count_reason"] = "single-axis sweep, second axis held at its verified value"
        lock = plan.freeze(allowed, self.inventory)
        self.assertEqual(8, len(lock["design"]["coarse"]))
        self.assertIn("single-axis sweep", lock["design"]["coarse_count_reason"])

    def test_a_dimension_that_is_not_varied_is_a_constant_somewhere_else(self):
        reason = self.refuse(lambda s: s["dimensions"][0].update(levels=[1.0]))
        self.assertIn("belongs in 'fixed'", reason)

    def test_an_unconditional_refine_stage_is_not_a_refine_stage(self):
        self.assertIn("conditional", self.refuse(lambda s: s["refine"].update(gate="  ")))

    def test_too_many_refine_candidates(self):
        self.assertIn("8", self.refuse(lambda s: s["refine"].update(max_candidates=12)))

    def test_one_confirmation_is_not_a_confirmation(self):
        self.assertIn("not a confirmation", self.refuse(lambda s: s["confirm"].update(repeats=1)))

    def test_no_stop_criterion_is_the_last_run_finding_repeated(self):
        reason = self.refuse(lambda s: s.pop("stop"))
        self.assertIn("stop", reason)

    def test_a_result_without_a_payload_binding_describes_an_unladen_station(self):
        self.assertIn("payload", self.refuse(lambda s: s.update(payload_profile="")))


class TheLockCannotBeEditedAfterwards(unittest.TestCase):
    def setUp(self):
        self.inventory = plan.load_inventory(plan.INVENTORY)
        self.lock = plan.freeze(spec(), self.inventory)

    def test_a_design_that_was_touched_after_freezing_does_not_hash_to_itself(self):
        tampered = copy.deepcopy(self.lock)
        tampered["design"]["coarse"][0]["params"]["yaw.current_kp_a_per_rad_s"] = 9.0
        self.assertNotEqual(self.lock["design_sha256"], plan.sha256_text(plan.canonical(tampered["design"])),
                            "an edit to a frozen design must show up as a different campaign, not as "
                            "an amendment")

    def test_inventory_drift_blocks_the_campaign_rather_than_silently_continuing(self):
        drifted = copy.deepcopy(self.inventory)
        drifted["_sha256"] = "0" * 64
        lock = plan.freeze(spec(), drifted)
        self.assertNotEqual(self.lock["bound_to"]["inventory_sha256"],
                            lock["bound_to"]["inventory_sha256"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
