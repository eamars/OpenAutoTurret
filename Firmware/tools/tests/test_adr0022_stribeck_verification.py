"""Narrow Stribeck experiment contract and actual hybrid native regression."""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from Firmware.commissioning.model_family import FamilyNative
from Firmware.tools.adr0022_stribeck_verification import (
    DEVELOPMENT_INPUT_SEEDS, FINAL_INPUT_SEEDS, INITIAL, MAX_NFEV, RECOVERY_LIMITS,
    BALANCE_GRID_POINTS, CHECKPOINT_RESUME_NFEV, JOINT_BOUNDS, JOINT_INITIAL, JOINT_METHOD_REVISION,
    JOINT_RECOVERY_INPUT_SEEDS, fit_joint_procedure, integrated_stribeck_initializer, joint_method_ready,
    SPEED_BOUNDS, fit_one, generate_truth, narrow_development_ready, numerical_gate,
    observations, parameter_recovery_gate, schedule, stribeck_moving_intervals, trajectory_gate, true_model)


class OriginalRecoveryGateTests(unittest.TestCase):
    def test_pristine_two_percent_and_noisy_five_percent_are_distinct(self):
        self.assertEqual(RECOVERY_LIMITS, {"pristine": .02, "noisy": .05})
        self.assertFalse(parameter_recovery_gate({"stribeck_negative": .03}, [], noisy=False)["passed"])
        self.assertTrue(parameter_recovery_gate({"stribeck_negative": .03}, [], noisy=True)["passed"])
        self.assertTrue(parameter_recovery_gate({"stribeck_negative": -.02}, [], noisy=False)["passed"])
        self.assertFalse(parameter_recovery_gate({"stribeck_negative": 0.}, ["stribeck_negative"], noisy=False)["passed"])
        with self.assertRaises(ValueError):
            parameter_recovery_gate({"stribeck_negative": 0.}, [], noisy=None)

    def test_fresh_joint_requires_passing_exact_bounded_method_freeze(self):
        freeze = {"method_revision": JOINT_METHOD_REVISION, "retained_checkpoint_probe_passed": True,
            "fresh_input_seeds": JOINT_RECOVERY_INPUT_SEEDS, "initializer_grid_points_each": BALANCE_GRID_POINTS,
            "initial_final_nfev_cap": 120, "single_checkpoint_resume_nfev_cap": CHECKPOINT_RESUME_NFEV,
            "relative_recovery_limits": RECOVERY_LIMITS, "selection_or_truth_used_for_resume": False}
        self.assertTrue(joint_method_ready(freeze))
        for change in ({"retained_checkpoint_probe_passed": False}, {"fresh_input_seeds": FINAL_INPUT_SEEDS},
                       {"initial_final_nfev_cap": 200}, {"single_checkpoint_resume_nfev_cap": 80},
                       {"relative_recovery_limits": {"pristine": .05, "noisy": .05}},
                       {"selection_or_truth_used_for_resume": True}):
            self.assertFalse(joint_method_ready({**freeze, **change}))


class JointInitializerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = FamilyNative(Path(os.environ["OTA_AXIS_CORE_LIBRARY"]))
        cls.acquisition = observations(generate_truth(DEVELOPMENT_INPUT_SEEDS[0]), False)
        cls.model = replace(true_model(), **JOINT_INITIAL)

    def test_moving_equations_use_supported_times_and_both_real_signs(self):
        intervals, metadata = stribeck_moving_intervals(self.model, self.acquisition)
        self.assertEqual(metadata["gyro_smoothing_window_samples"], 7)
        self.assertEqual({row["direction"] for row in intervals}, {-1, 1})
        self.assertGreater(metadata["negative_equations"], 0)
        self.assertGreater(metadata["positive_equations"], 0)
        for row in intervals:
            self.assertGreaterEqual(row["times"][0], self.acquisition.t[0])
            self.assertLessEqual(row["times"][-1], self.acquisition.t[-1])
            self.assertTrue(np.all(row["direction"]*row["velocity"] > metadata["minimum_abs_speed_rad_s"]))
        self.assertFalse(metadata["endpoint_extrapolation"])
        self.assertFalse(metadata["filter_derivative_used_for_final_residuals"])
        self.assertFalse(metadata["state_reset"])

    def test_stationary_or_unsupported_model_declares_information_deficiency(self):
        stationary = replace(self.acquisition, q=np.zeros_like(self.acquisition.q), v=np.zeros_like(self.acquisition.v),
            current=np.zeros_like(self.acquisition.current), tx_A=np.zeros_like(self.acquisition.tx_A))
        with self.assertRaisesRegex(ValueError, "INSUFFICIENT_BOTH_DIRECTION"):
            stribeck_moving_intervals(self.model, stationary)
        with self.assertRaisesRegex(ValueError, "fixed algebraic"):
            stribeck_moving_intervals(replace(self.model, actuator="first_order", actuator_tau=.02), self.acquisition)

    def test_profile_keeps_bounds_and_ignores_unfresh_gyro_poison(self):
        seed, metadata = integrated_stribeck_initializer(self.native, self.model, self.acquisition, JOINT_BOUNDS)
        poisoned = self.acquisition.v.copy()
        poisoned[~self.acquisition.v_new] = 1e6
        alternate, alternate_metadata = integrated_stribeck_initializer(self.native, self.model,
            replace(self.acquisition, v=poisoned), JOINT_BOUNDS)
        self.assertEqual(asdict(seed), asdict(alternate))
        self.assertEqual(metadata["selected_huber_cost"], alternate_metadata["selected_huber_cost"])
        self.assertEqual(metadata["native_candidate_evaluations"], 49)
        self.assertEqual(metadata["native_candidate_budget"], 49)
        self.assertEqual(metadata["candidates"][metadata["selected_index"]]["matrix_rank"], 4)
        for key, bounds in JOINT_BOUNDS.items():
            self.assertGreaterEqual(getattr(seed, key), bounds[0])
            self.assertLessEqual(getattr(seed, key), bounds[1])
        for key, value in asdict(self.model).items():
            if key not in JOINT_BOUNDS:
                self.assertEqual(getattr(seed, key), value)
        self.assertFalse(metadata["truth_used_to_choose_seed"])
        self.assertFalse(metadata["selection_or_holdout_used"])
        self.assertFalse(metadata["smooth_switch_acceptance"])

    def test_actual_bounded_resume_recovers_train_and_keeps_selection_outside_fit(self):
        selection = replace(self.acquisition, run_id="deliberately-contaminated-selection", q=self.acquisition.q+.02)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            report = fit_joint_procedure(self.native, self.acquisition, [selection], output=output, noisy=False)
            original = json.loads((output/f"{self.acquisition.run_id}-fit.json").read_text())
            resumed = json.loads((output/f"{self.acquisition.run_id}-resume"/f"{self.acquisition.run_id}-fit.json").read_text())
        self.assertFalse(original["optimizer"]["success"])
        self.assertEqual(original["optimizer"]["evaluations"], 120)
        self.assertTrue(report["optimizer"]["success"])
        self.assertEqual(resumed["initial_model"], original["fitted_model"])
        self.assertTrue(resumed["checkpoint_grid_initializer_disabled"])
        self.assertIsNone(resumed["initializer_stage"])
        self.assertTrue(report["checkpoint_resume"]["used"])
        self.assertTrue(report["checkpoint_resume"]["diagnostic"]["resume_allowed"])
        self.assertLessEqual(resumed["native_optimizer_evaluations_total"], 40)
        self.assertLessEqual(report["native_optimizer_evaluations_total"], 160)
        self.assertGreater(report["native_optimizer_evaluations_total"], 120)
        self.assertEqual(report["initializer_native_candidate_evaluations"], 49)
        self.assertEqual(report["recovery_limit"], .02)
        self.assertTrue(report["recovery_passed"])
        self.assertTrue(report["training_native_quality_passed"])
        self.assertTrue(report["predictions"][0]["passed"])
        self.assertFalse(report["predictions"][1]["passed"])
        self.assertFalse(report["passed"])
        self.assertFalse(report["selection_in_fit_or_initialization"])
        self.assertFalse(report["deployment_authorized"])


class StribeckVerificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = FamilyNative(Path(os.environ["OTA_AXIS_CORE_LIBRARY"]))
        cls.training = generate_truth(DEVELOPMENT_INPUT_SEEDS[0])
        cls.selection = generate_truth(DEVELOPMENT_INPUT_SEEDS[1])

    def test_partition_seeds_change_commands_and_duration_not_only_noise(self):
        self.assertFalse(set(DEVELOPMENT_INPUT_SEEDS) & set(FINAL_INPUT_SEEDS))
        # Final schedules remain unopened until the method-freeze record.
        commands = [schedule(seed) for seed in DEVELOPMENT_INPUT_SEEDS]
        for times, levels, duration in commands:
            self.assertTrue(np.all(np.diff(times) > 0))
            self.assertLess(times[0], -true_model().transport_delay)
            self.assertGreater(duration, times[-1])
            self.assertTrue(np.any(levels > 0) and np.any(levels < 0) and np.any(levels == 0))
        for index, first in enumerate(commands):
            for second in commands[index+1:]:
                self.assertFalse(np.array_equal(first[0], second[0]))
                self.assertFalse(np.array_equal(first[1], second[1]))

    def test_independent_hybrid_motion_covers_low_speed_and_retains_native_masks(self):
        for data in (self.training, self.selection):
            self.assertTrue(data["coverage"]["passed"])
            self.assertGreaterEqual(data["coverage"]["zero_crossings"], 2)
            self.assertEqual(data["coverage"]["breakaway_directions"], [-1, 1])
            run = observations(data, True)
            self.assertTrue(np.all(run.q_new) and np.all(run.current_new))
            np.testing.assert_array_equal(run.v_new, np.arange(len(run.t)) % 20 == 0)
            self.assertTrue(np.all(np.isnan(run.v[~run.v_new])))
            self.assertTrue(np.all(np.isfinite(run.v[run.v_new])))
            self.assertTrue(numerical_gate(self.native, data)["passed"])

    def test_probe_and_incomplete_evidence_cannot_authorize_joint_scope(self):
        summary = {"scope": "speeds", "partition": "development", "probe_only": False,
            "narrow_diagnostic_passed": True, "synthetic_scope_passed": True, "free_coordinate_count": 2,
            "cases": [{"training_run": f"stribeck-input3109-{noise}", "passed": True}
                      for noise in ("pristine", "noisy")]}
        contract = {"scope": "speeds", "partition": "development", "probe_only": False,
            "active_train_input_seed": 3109, "active_selection_input_seeds": [3461]}
        self.assertTrue(narrow_development_ready(summary, contract))
        for change in ({"probe_only": True}, {"partition": "final"}, {"free_coordinate_count": 6},
                       {"cases": summary["cases"][1:]}, {"narrow_diagnostic_passed": False}):
            self.assertFalse(narrow_development_ready({**summary, **change}, contract))
        for change in ({"probe_only": True}, {"active_selection_input_seeds": []},
                       {"active_train_input_seed": FINAL_INPUT_SEEDS[0]}):
            self.assertFalse(narrow_development_ready(summary, {**contract, **change}))

    def test_original_quality_gate_survives_large_declared_sensor_sigma(self):
        run = replace(observations(self.training, True), sigma_q=.1, sigma_v=.1)
        prediction = self.training["truth"].copy()
        prediction[:, 0] += .004
        prediction[:, 3] += .04
        gate = trajectory_gate(run, prediction)
        self.assertLess(gate["errors"]["q_rms_rad"], gate["synthetic_3sigma_limits"]["q_rms_rad"])
        self.assertGreater(gate["errors"]["q_rms_rad"], gate["original_whole_run_limits"]["q_rms_rad"])
        self.assertGreater(gate["errors"]["gyro_rms_rad_s"], gate["original_whole_run_limits"]["gyro_rms_rad_s"])
        self.assertFalse(gate["passed"])

    def test_actual_native_inverse_does_not_fit_contaminated_selection(self):
        train = observations(self.training, True)
        selected = observations(self.selection, True)
        selected = replace(selected, q=selected.q+.02)
        self.assertEqual(MAX_NFEV, 120)
        self.assertEqual(RECOVERY_LIMITS["noisy"], .05)
        for key in SPEED_BOUNDS:
            self.assertNotEqual(INITIAL[key], getattr(true_model(), key))
        with tempfile.TemporaryDirectory() as temporary:
            report = fit_one(self.native, train, [selected], scope="speeds", output=Path(temporary), noisy=True)
            self.assertEqual(len(list(Path(temporary).glob("*.npz"))), 2)
        self.assertTrue(report["optimizer"]["success"])
        self.assertTrue(report["recovery_passed"])
        self.assertTrue(report["training_native_quality_passed"])
        self.assertTrue(report["predictions"][0]["passed"])
        self.assertFalse(report["predictions"][1]["passed"])
        self.assertFalse(report["passed"])
        self.assertFalse(report["selection_in_fit_or_initialization"])
        self.assertFalse(report["smooth_sign_substitution"])
        self.assertLessEqual(report["native_optimizer_evaluations_total"], 120)
        self.assertEqual(report["free_coordinates"], list(SPEED_BOUNDS))
        for key, value in true_model().document().items():
            if key not in SPEED_BOUNDS:
                self.assertEqual(report["fitted_model"][key], value)
        self.assertEqual(report["physical_qualification"], "NOT_RUN")
        self.assertFalse(report["deployment_authorized"])


if __name__ == "__main__":
    unittest.main()
