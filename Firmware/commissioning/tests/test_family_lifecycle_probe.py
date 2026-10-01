"""Native regressions after the actual fitted-receipt configuration lifecycle probe."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from Firmware.commissioning.applicability import assess_configuration
from Firmware.commissioning.family_assets import (QUALITY_GATES, bind_diagnostic_runtime,
    runtime_document)
from Firmware.commissioning.native import Native
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters, estimator_model
from Firmware.tools.adr0022_family_forecast_probe import contract
from Firmware.tools.adr0022_family_pipeline_probe import fitted_asset
from Firmware.tools.adr0022_family_lifecycle_probe import changed_facts, run_probe


class FamilyLifecycleProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        cls.native = Native()
        model = replace(estimator_model(), a=.113, viscous=.067, actuator="first_order",
            actuator_tau=.0073, gyro_delay=.0043, current_tau=.012, current_delay=.0029)
        fit = directory/"diagnostic-fit.json"
        fit.write_text(json.dumps({"model": model.document(), "role": "CONSUMED TEST FIXTURE",
            "free_fields": ["a", "viscous"], "gates": {**dict.fromkeys(QUALITY_GATES, "PASS"),
                "optimizer_converged": True}}), encoding="utf-8")
        cls.asset, support, _ = fitted_asset(fit, contract(duration=.6))
        template = controller_parameters()
        parameters, support, binding = bind_diagnostic_runtime(cls.asset, cls.native, template, support)
        receipt = directory/"runtime-receipt.json"
        receipt.write_text(json.dumps(runtime_document(cls.asset, template, parameters, support, binding)),
                           encoding="utf-8")
        cls.result = run_probe(receipt, cls.native.path, directory/"full-probe")

    def test_declared_boundaries_latch_before_any_new_command_and_keep_the_successful_receipt(self):
        changes = [row for row in self.result["cases"] if row["event"] in
                   ("PAYLOAD+", "PAYLOAD-", "FRICTION+", "FRICTION-")]
        self.assertEqual(len(changes), 4)
        for row in changes:
            with self.subTest(event=row["event"]):
                self.assertEqual(row["assessment"]["action"], "UPDATE_PARAMETERS_AND_REVALIDATE")
                self.assertEqual(row["runtime"]["native_status"], 5)
                self.assertEqual(row["runtime"]["new_command_count"], 0)
                self.assertEqual(row["runtime"]["accepted_receipt_before_invalidation"],
                                 row["runtime"]["accepted_receipt_after_invalidation"])
                self.assertAlmostEqual(row["runtime"]["last_command_retained_A"], .010)
                self.assertIsNone(row["replacement_model_revision"])

    def test_return_reuses_model_support_but_requires_explicit_fresh_reset_and_rejects_old_tokens(self):
        returned = [row for row in self.result["cases"] if row["event"] == "RETURN"]
        self.assertEqual(len(returned), 4)
        for row in returned:
            self.assertEqual(row["assessment"]["action"], "REUSE_WITHIN_SUPPORT")
            self.assertFalse(row["automatic_rearm"])
            self.assertTrue(row["old_generation_token_rejected"])
            self.assertEqual(row["native_generation"], row["previous_native_generation"]+1)
            self.assertFalse(row["model_revision_changed"])
            self.assertFalse(row["receipt_clock_reset"])
        self.assertEqual(self.result["model_revision_count"], 1)
        self.assertEqual(self.result["native_generation_count"], 5)
        self.assertEqual(self.result["configuration_generation_count"], 10)
        self.assertTrue(self.result["receipt_clock_monotonic"])
        # The second reset falls after the latest delayed gyro source sample.
        # Re-injecting that older source as valid caused the observed native
        # DATA_INVALID regression; wait for a sample newer than each reset.
        self.assertGreater(self.result["pre_restart_gyro_samples_skipped"], 3)

    def test_undeclared_change_at_rest_is_unobservable_and_information_budget_is_finite(self):
        hidden = next(row for row in self.result["cases"] if row["event"] == "UNDECLARED")
        self.assertFalse(hidden["change_detected"])
        self.assertEqual(hidden["status"], "INSUFFICIENT_EXCITATION_AT_REST")
        self.assertEqual([row["actual_fresh_samples"] for row in hidden["monitor_windows"]], [100]*3)
        self.assertEqual([row["response"] for row in hidden["monitor_windows"]], ["MONITOR_ONLY"]*3)
        self.assertEqual([row["normalized_residual_RMS"] for row in hidden["monitor_windows"]], [0.]*3)
        self.assertEqual(hidden["finite_recovery_routes"], ["SELECT_AT_MOST_THREE_INFORMATION_CASES",
            "SELECT_AT_MOST_THREE_INFORMATION_CASES", "STOP_INFORMATION_BUDGET_EXHAUSTED"])
        revealed = next(row for row in self.result["cases"] if row["event"] == "UNDECLARED_REVEALED")
        self.assertEqual(revealed["runtime"]["native_status"], 5)
        self.assertTrue(self.result["final_inhibited"])

    def test_full_native_history_agrees_with_independent_rest_flow_without_qualifying_the_model(self):
        result = self.result
        self.assertEqual(result["interface_status"], "PASS")
        self.assertEqual(result["actual_native_receipt_count"], 1205)
        self.assertEqual(result["numerical_verification"]["causal_rest_observation_max_error"], 0.)
        self.assertEqual(result["numerical_verification"]["plant_initializations"], 1)
        self.assertEqual(result["numerical_verification"]["core_owner_instances"], 1)
        self.assertTrue(result["numerical_verification"]["forward_pass"])
        self.assertFalse(result["qualification_attempt"]["accepted"])
        self.assertEqual(result["qualification_attempt"]["reason"], "MODEL_INADEQUATE")
        self.assertFalse(result["model_qualified"])
        self.assertFalse(result["controller_qualified"])
        self.assertFalse(result["deployment_authorized"])

    def test_unclassified_friction_cannot_create_a_supported_schedule_from_one_fitted_vector(self):
        baseline = self.asset.configuration_support.baseline
        self.assertNotIn("friction.condition", baseline.facts)
        for event in ("FRICTION+", "FRICTION-"):
            assessment = assess_configuration(self.asset.configuration_support,
                changed_facts(baseline, event), purpose="SYNTHETIC_CONTROL")
            self.assertEqual(assessment["changed_fields"], ["friction.condition"])
            self.assertFalse(assessment["supported"])
            self.assertIn("plant_parameters", assessment["invalidate"])
            self.assertIn("controller_candidate", assessment["invalidate"])
            self.assertEqual(assessment["preserve"], ["raw_history", "model_family", "identification_method"])


if __name__ == "__main__": unittest.main()
