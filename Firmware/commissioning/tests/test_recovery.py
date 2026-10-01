from __future__ import annotations

import unittest
import numpy as np

from Firmware.commissioning.model_family import FamilyRun
from Firmware.commissioning.recovery import (CandidateChecks, Check, Failure,
    comparison_recovery, diagnostic_report, recovery_decision)


class RecoveryTests(unittest.TestCase):
    def run_fixture(self, t, q, v, current, *, masks=None, quantum=0., sigma=1., initial=None):
        if masks is None:
            masks = [np.ones(len(t), dtype=bool)] * 3
        values = [np.where(mask, value, np.nan) for value, mask in zip((q, v, current), masks)]
        return FamilyRun(run_id="synthetic-regression", source_id="independent-fixture", t=np.asarray(t),
            q=values[0], v=values[1], current=values[2], q_new=masks[0], v_new=masks[1],
            current_new=masks[2], tx_t=np.array([-.1, .6, 1.2, 1.4]),
            tx_A=np.array([0., .3, 0., -.3]),
            initial=np.zeros(5) if initial is None else np.asarray(initial),
            sigma_q=sigma, sigma_v=sigma, sigma_current=sigma, encoder_quantum=quantum,
            provenance="SYNTHETIC")

    def test_no_motion_cannot_masquerade_as_baseline_recovery(self):
        t = np.linspace(0., 2., 201)
        run = self.run_fixture(t, .1 * t, np.full(len(t), .1), np.zeros(len(t)), sigma=.001)
        report = diagnostic_report(run, np.zeros((len(t), 6)))
        self.assertTrue(report["no_motion_model_on_moving_data"])
        self.assertEqual(report["displacement"]["predicted_span_rad"], 0.)
        self.assertEqual(report["predicted_motion"]["moving_duration_s"], 0.)
        self.assertGreater(report["observed_motion"]["moving_duration_s"], 1.9)
        self.assertEqual(report["baselines"]["constant_position_q_rms_rad"],
                         report["baselines"]["model_q_rms_rad"])
        self.assertFalse(report["baselines"]["absolute_angle_gate_passed"])

    def test_event_horizons_slice_uninterrupted_prediction_at_native_masks(self):
        t = np.linspace(0., 2., 201)
        v = np.where((t >= .6) & (t < 1.2), .1, np.where(t >= 1.4, -.1, 0.))
        q = np.cumsum(v) * .01
        masks = [(np.arange(len(t)) % stride == 0) for stride in (2, 5, 10)]
        run = self.run_fixture(t, q, v, np.zeros(len(t)), masks=masks, sigma=.001)
        pred = np.zeros((len(t), 6)); pred[:, 0] = q + .2; pred[:, 3] = v
        report = diagnostic_report(run, pred)
        kinds = {e["kind"] for e in report["event_anchored_horizons"]}
        self.assertIn("observed_motion_onset", kinds)
        self.assertIn("observed_reversal", kinds)
        self.assertIn("observed_stop", kinds)
        self.assertIn("successful_command_change", kinds)
        for comparison in report["transition_comparison"].values():
            self.assertEqual(comparison["unmatched_observed_count"], 0)
            self.assertTrue(all(pair["timing_error_s"] == 0 for pair in comparison["ordered_pairs"]))
        for anchor in report["event_anchored_horizons"]:
            for window in anchor["horizons"].values():
                if window["encoder"]["native_samples"]:
                    self.assertAlmostEqual(window["encoder"]["rms"], .2)
        self.assertEqual(report["state_resets"], 0)
        self.assertTrue(all(e["anchor_policy"].startswith("OUTCOME_TRIGGERED")
            for e in report["event_anchored_horizons"] if e["kind"].startswith("observed_")))

    def test_huber_breakdown_matches_exact_bin_and_sigma_objective(self):
        t = np.array([0., 1., 2.])
        run = self.run_fixture(t, np.zeros(3), np.zeros(3), np.zeros(3), quantum=2.)
        pred = np.zeros((3, 6)); pred[:, 0] = [0., 2., 4.]; pred[:, 3] = [.5, 2., -2.]
        report = diagnostic_report(run, pred)
        objective = report["objective"]
        self.assertAlmostEqual(objective["channels"]["encoder"]["huber_cost"], 3.)
        self.assertAlmostEqual(objective["channels"]["gyro"]["huber_cost"], 3.125)
        self.assertAlmostEqual(objective["total_huber_cost"], 6.125)
        for channel in objective["channels"].values():
            self.assertAlmostEqual(sum(r["huber_cost"] for r in channel["regimes"].values()),
                                   channel["huber_cost"])

    def test_unknown_and_unperformed_checks_do_not_become_failures(self):
        record = recovery_decision(CandidateChecks())
        self.assertEqual(record["checks"]["forward_numerics"], Check.UNKNOWN)
        self.assertEqual(record["checks"]["physical_stage3a"], Check.NOT_RUN)
        self.assertIsNone(record["first_failing_predicate"])
        self.assertTrue(record["promotion_blocked"])
        self.assertFalse(record["deployment_authorized"])

    def test_budget_and_trajectory_failures_route_separately(self):
        candidate = {"optimizer": {"success": False, "message": "The maximum number of function evaluations is exceeded."},
            "training": [{"passed": False}], "selection": [{"passed": False}]}
        record = comparison_recovery(candidate, [{"no_motion_model_on_moving_data": True}])
        self.assertEqual([r["condition"] for r in record["routes"]],
                         [Failure.BUDGET_EXHAUSTED, Failure.NO_MOTION_MODEL_ON_MOVING_DATA])
        self.assertEqual(record["checks"]["optimizer_converged"], Check.FAIL)
        self.assertEqual(record["checks"]["training_trajectory"], Check.FAIL)
        candidate["optimizer"] = {"success": True, "message": "xtol termination condition satisfied"}
        record = comparison_recovery(candidate)
        self.assertEqual(record["first_failing_predicate"], Failure.CONVERGED_BUT_TRAJECTORY_FAILS)
        self.assertEqual(record["checks"]["optimizer_converged"], Check.PASS)

    def test_hardware_fault_never_automatically_rearms_or_repeats_motion(self):
        record = recovery_decision(CandidateChecks(), [Failure.HARDWARE_FAULT], physical_state="FAULT")
        self.assertEqual(record["analysis_state"], "DIAGNOSE")
        self.assertTrue(record["promotion_blocked"])
        self.assertFalse(record["automatically_repeat_motion"])
        self.assertFalse(record["automatically_rearm"])
        self.assertIn("offline fault diagnosis", record["next_action"])

    def test_zero_motion_observation_is_not_falsely_called_a_contradiction(self):
        t = np.linspace(0., 1., 101)
        run = self.run_fixture(t, np.zeros(len(t)), np.zeros(len(t)), np.zeros(len(t)), sigma=.001)
        report = diagnostic_report(run, np.zeros((len(t), 6)))
        self.assertFalse(report["no_motion_model_on_moving_data"])
        self.assertEqual(report["observed_motion"]["events"], [])
        self.assertEqual(report["objective"]["total_huber_cost"], 0.)


if __name__ == "__main__":
    unittest.main()
