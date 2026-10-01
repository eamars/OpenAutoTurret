"""Finite START policy contracts after actual four-point native discrimination."""
import copy
from dataclasses import replace
import unittest

from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.motor_feedforward import parameters_for_family
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters, dynamic_fixture
from Firmware.commissioning.family_sampled_analysis import LocalGains
from Firmware.tools.adr0022_start_policy_synthesis_probe import (EXCESS_GRID_A, PLATEAU_CASES,
    STEP_CASES, case_passed, select_minimum, start_policy)


def row(case, passed):
    return {"case": case, "completed": passed, "conditional_forward_passed": True,
        "START_dose_passed": True, "START_realized_command_dose_A2s": .01,
        "original_quality_passed": False, "revised_quality_passed": True,
        "candidate_case_passed": True,  # cannot override incomplete/faulted execution
        "report": {"maximum_successful_command_A": .30, "maximum_successful_slew_A_s": 2.}}


def candidates(passing=()):
    return [{"excess_A_effective": excess,
        "plateau_cases": [row(c, excess in passing) for c in PLATEAU_CASES],
        "step_cases": [row(c, True) for c in STEP_CASES] if excess in passing else []}
        for excess in EXCESS_GRID_A]


class StartPolicySynthesisTests(unittest.TestCase):
    def test_exact_declared_grid_maps_start_interval_without_changing_controller_limits(self):
        model, _, _, template, _, _, _ = dynamic_fixture()[0]
        gains = LocalGains(.8411435757340915, .4, .4, 3.)
        supplied = candidate_parameters(template, gains)
        for excess in EXCESS_GRID_A:
            policy = start_policy(excess, "synthetic-family-forecast")
            mapped = parameters_for_family(supplied, model, actuator_policy="STEADY_STATE_REFERENCE",
                actuation_memory_max_s=.03, start_policy=policy)
            self.assertEqual((mapped.kp, mapped.ki, mapped.kpos, mapped.kaw), (.8411435757340915, .4, .4, 3.))
            self.assertEqual((mapped.current_cap, mapped.slew, mapped.start_timeout_s), (.35, 2., .2))
            self.assertAlmostEqual(mapped.start_total[0], .02-(.164+excess))
            self.assertAlmostEqual(mapped.start_total[15], .02+.164+excess)
            self.assertEqual((policy.max_attempts, policy.max_command_dose_A2s), (1, .026))
            self.assertGreaterEqual(.026, .35**2*(.2+mapped.dt_max))
        for invalid in (0., .010, .081, .10, True):
            with self.assertRaises(Rejected): start_policy(invalid, "synthetic-family-forecast")

    def test_only_authorized_directional_pair_is_available(self):
        pair = start_policy(.06, "synthetic-family-forecast", positive_excess=.08)
        self.assertEqual((pair.negative_excess_A, pair.positive_excess_A), (.06, .08))
        self.assertEqual((pair.max_attempts, pair.max_attempt_s, pair.max_command_dose_A2s), (1, .2, .026))
        with self.assertRaises(Rejected): start_policy(.04, "synthetic-family-forecast", positive_excess=.08)

    def test_selection_uses_minimum_excess_passing_every_motor_case_under_revised_contract(self):
        decision = select_minimum(candidates((.04, .08)))
        self.assertEqual(decision["selected_excess_A_effective"], .04)
        self.assertIn("DEVELOPMENT", decision["qualification"])
        # Original metric failure is retained separately and cannot alter the declared revised gate.
        self.assertFalse(candidates((.04,))[1]["plateau_cases"][0]["original_quality_passed"])

    def test_fault_or_incomplete_case_cannot_be_laundered_by_quality_or_cached_pass(self):
        values = candidates((.06,))
        values[2]["step_cases"][0]["completed"] = False
        decision = select_minimum(values)
        self.assertEqual(decision["status"], "NO_SINGLE_EPISODE_ALL_DOMAIN_CANDIDATE")
        self.assertIsNone(decision["selected_excess_A_effective"])
        self.assertFalse(case_passed(values[2]["step_cases"][0]))
        self.assertEqual(select_minimum(candidates())["status"], "NO_SINGLE_EPISODE_ALL_DOMAIN_CANDIDATE")

    def test_missing_duplicate_or_unexecuted_required_cases_do_not_provide_domain_coverage(self):
        for edit in ("missing_candidate", "duplicate_plateau", "missing_step", "no_steps"):
            values = candidates((.08,))
            if edit == "missing_candidate": values.pop()
            elif edit == "duplicate_plateau": values[3]["plateau_cases"][0] = copy.deepcopy(values[3]["plateau_cases"][1])
            elif edit == "missing_step": values[3]["step_cases"].pop()
            else: values[3]["step_cases"] = []
            with self.subTest(edit=edit):
                with self.assertRaises(Rejected): select_minimum(values)

    def test_current_slew_dose_and_forward_failures_remain_disqualifying(self):
        for key, value in (("dose", .027), ("command", .351), ("slew", 2.01), ("forward", False)):
            values = candidates((.08,))
            trial = values[3]["step_cases"][0]
            if key == "dose": trial["START_realized_command_dose_A2s"] = value
            elif key == "command": trial["report"]["maximum_successful_command_A"] = value
            elif key == "slew": trial["report"]["maximum_successful_slew_A_s"] = value
            else: trial["conditional_forward_passed"] = value
            with self.subTest(key=key):
                self.assertIsNone(select_minimum(values)["selected_excess_A_effective"])


if __name__ == "__main__": unittest.main()
