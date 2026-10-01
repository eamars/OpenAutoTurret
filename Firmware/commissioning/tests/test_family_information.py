"""Contract regressions after the actual native fixed-template design probe."""
from dataclasses import replace
import os
from pathlib import Path
import unittest

import numpy as np

from Firmware.commissioning.adaptation import FailurePolicy
from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.family_information import select_family_supplemental
from Firmware.commissioning.model_family import FamilyNative
from Firmware.tools.adr0022_family_information_probe import fixture_inputs
from Firmware.tools.adr0022_family_synthesis_probe import dynamic_fixture
from Firmware.tools.adr0022_assembly_probe import fixture as coupled_fixture


class NeverPredict:
    def rollout(self, *args):
        raise AssertionError("unsupported request reached prediction")


class FamilyInformationContractTests(unittest.TestCase):
    def setUp(self):
        self.inputs = list(fixture_inputs(dynamic_fixture()[0][0]))
        self.policy = FailurePolicy()

    def rejected(self, inputs, reason):
        with self.assertRaises(Rejected) as exc:
            select_family_supplemental(NeverPredict(), *inputs, self.policy)
        self.assertEqual(exc.exception.reason, reason)
        self.assertEqual(self.policy.information_rounds, 0)

    def test_model_numerical_domain_cannot_supply_unknown_physical_travel(self):
        self.inputs[3] = replace(self.inputs[3], angle_min_rad=None, angle_max_rad=None)
        self.rejected(self.inputs, Reason.ENVELOPE_LIMITED)

    def test_qualification_must_cover_actual_stop_and_thermal_facts(self):
        for field in ("adequate_stop", "thermal", "supply", "winding"):
            inputs = self.inputs.copy()
            support = inputs[4]
            inputs[4] = replace(support, evidence={**support.evidence, field: ""})
            self.rejected(inputs, Reason.ENVELOPE_LIMITED)

    def test_synthetic_qualification_cannot_be_relabelled_measured(self):
        self.inputs[3] = replace(self.inputs[3], provenance="MEASURED")
        self.rejected(self.inputs, Reason.ENVELOPE_LIMITED)

    def test_qualified_physical_packet_cannot_promote_sampled_gradients(self):
        self.inputs[3] = replace(self.inputs[3], provenance="MEASURED")
        self.inputs[4] = replace(self.inputs[4], qualification="QUALIFIED_PHYSICAL_ENVELOPE")
        self.rejected(self.inputs, Reason.ENVELOPE_LIMITED)

    def test_nominal_must_be_in_complete_prediction_set(self):
        self.inputs[1] = self.inputs[1][1:]
        self.rejected(self.inputs, Reason.DATA_INVALID)

    def test_unknown_and_malformed_envelope_cannot_reach_arithmetic(self):
        for field, value, reason in (("current_a", None, Reason.ENVELOPE_LIMITED),
                                    ("current_a", True, Reason.DATA_INVALID),
                                    ("slew_a_s", "unknown", Reason.DATA_INVALID),
                                    ("duration_s", float("nan"), Reason.DATA_INVALID),
                                    ("acceleration_rad_s2", False, Reason.DATA_INVALID),
                                    ("angle_min_rad", float("inf"), Reason.DATA_INVALID),
                                    ("provenance", None, Reason.DATA_INVALID),
                                    ("stop_verified", 1, Reason.DATA_INVALID)):
            with self.subTest(field=field, value=value):
                inputs = self.inputs.copy()
                inputs[3] = replace(inputs[3])
                object.__setattr__(inputs[3], field, value)
                self.rejected(inputs, reason)

    def test_missing_explicit_support_or_measurement_packet_is_typed(self):
        for index, reason in ((4, Reason.ENVELOPE_LIMITED), (5, Reason.MEASUREMENT_LIMITED)):
            inputs = self.inputs.copy(); inputs[index] = None
            self.rejected(inputs, reason)

    def test_quantization_is_not_gaussian_variance_invention(self):
        self.inputs[5] = replace(self.inputs[5], encoder_quantum_rad=2*np.pi/8192)
        self.rejected(self.inputs, Reason.MEASUREMENT_LIMITED)

    def test_coupled_parent_is_explicitly_unsupported(self):
        self.inputs[0] = coupled_fixture()
        self.rejected(self.inputs, Reason.MODEL_INADEQUATE)

    def test_unknown_actuator_bias_gauge_is_not_identification(self):
        self.inputs[6] = ("actuator_bias",)
        self.rejected(self.inputs, Reason.INSUFFICIENT_EXCITATION)

    def test_exhausted_existing_budget_does_not_predict_another_case(self):
        self.policy.handle(Reason.INSUFFICIENT_EXCITATION)
        self.policy.handle(Reason.INSUFFICIENT_EXCITATION)
        with self.assertRaises(Rejected) as exc:
            select_family_supplemental(NeverPredict(), *self.inputs, self.policy)
        self.assertEqual(exc.exception.reason, Reason.INSUFFICIENT_EXCITATION)
        self.assertEqual(self.policy.information_rounds, 2)


class FamilyInformationNativeTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get("OTA_AXIS_CORE_LIBRARY"), "actual local native library required")
    def test_native_selection_retains_supported_stop_and_bounded_cases(self):
        inputs = fixture_inputs(dynamic_fixture()[0][0])
        policy = FailurePolicy()
        selected = select_family_supplemental(FamilyNative(Path(os.environ["OTA_AXIS_CORE_LIBRARY"])), *inputs, policy)
        self.assertEqual(len(selected["selected"]), 3)
        self.assertEqual(selected["local_rank"], 4)
        self.assertFalse(selected["physical_identifiability"])
        self.assertIn("NOT_CONTINUOUS_PHYSICAL_BOUND_PROOF", selected["feasibility_scope"])
        for row in selected["selected"]:
            self.assertTrue(all(row["checks"].values()))
            self.assertGreater(row["moving_duration_s"], 0.)
            self.assertGreaterEqual(row["time"][-1], row["stop_time_s"]+2.)
            self.assertIn(row["amplitude_divisor"], (1., 1.5, 2., 3.))
        self.assertEqual(policy.information_rounds, 1)


if __name__ == "__main__":
    unittest.main()
