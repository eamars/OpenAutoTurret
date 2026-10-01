"""Receipt contracts after the fitted-vector actual-runtime bridge probe."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.family_assets import (FamilyAsset, MODEL_UNITS, QUALITY_GATES,
    bind_diagnostic_runtime, bind_runtime_document, native_parameter_document, runtime_document)
from Firmware.commissioning.native import CONTROL_FIELDS, Native
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters, estimator_model
from Firmware.tools.adr0022_family_forecast_probe import contract
from Firmware.tools.adr0022_family_pipeline_probe import fitted_asset


class FamilyAssetTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        source = Path(self.directory.name)/"consumed-fit.json"
        # A selected fitted-vector fixture deliberately distinct from the original template.
        model = replace(estimator_model(), a=.113, viscous=.067,
            coulomb_negative=.117, coulomb_positive=.126, actuator_gain=1.7,
            actuator_bias=.023, current_gain=.91, current_bias=.014)
        source.write_text(json.dumps({"model": model.document(),
            "free_fields": ["a", "viscous", "coulomb_negative", "coulomb_positive"],
            "role": "CONSUMED DEVELOPMENT FIXTURE",
            "gates": {**dict.fromkeys(QUALITY_GATES, "PASS"), "optimizer_converged": True}}))
        self.asset, self.support, self.state = fitted_asset(source, contract(duration=.6))

    def test_roundtrip_keeps_estimated_vector_structure_units_and_separate_q_guards(self):
        document = json.loads(json.dumps(self.asset.document()))
        actual = FamilyAsset.from_document(document)
        self.assertEqual(actual.document(), document)
        self.assertEqual(actual.model.a, .113)
        self.assertEqual(actual.model.structure, self.asset.model.structure)
        self.assertEqual(document["model_units"], MODEL_UNITS)
        self.assertEqual(document["numerical_domain"]["q_min_rad"], -2.)
        self.assertEqual(self.support.q_min_rad, -1.)
        self.assertFalse(document["deployment_authorized"])
        self.assertEqual(self.state.static_balance_A, .023-.02)

    def test_caller_or_export_mutation_cannot_rewrite_receipt(self):
        gauges = copy.deepcopy(self.asset.document()["gauges"])
        asset = replace(self.asset, gauges=gauges)
        gauges["acquisition_state"]["value"][0] = 1.
        exported = asset.document()
        exported["gauges"]["input_current_map"]["bias_A_effective"] = .9
        self.assertEqual(asset.gauges["acquisition_state"]["value"][0], 0.)
        self.assertEqual(asset.gauges["input_current_map"]["bias_A_effective"], .023)
        with self.assertRaises(TypeError):
            asset.gauges["sensor_biases"]["gyro_rad_s"] = .2

    def test_all_numeric_gauge_bindings_reject_contradictory_model(self):
        for group, quantity in (("load_friction", "load_offset_A_effective"),
            ("load_friction", "load_slope_A_effective_rad"),
            ("input_current_map", "gain_A_effective_per_A_command"),
            ("input_current_map", "bias_A_effective"),
            ("reported_current_map", "gain_A_reported_per_A_effective"),
            ("reported_current_map", "bias_A_reported"), ("sensor_biases", "gyro_rad_s"),
            ("static_thresholds", "negative_A_effective"), ("static_thresholds", "positive_A_effective")):
            with self.subTest(group=group, quantity=quantity):
                gauges = self.asset.document()["gauges"]
                gauges[group][quantity] += .01
                with self.assertRaises(Rejected) as error: replace(self.asset, gauges=gauges)
                self.assertEqual(error.exception.reason, Reason.DATA_INVALID)

    def test_noise_clock_and_initial_state_are_explicit_supported_assumptions(self):
        for group, quantity, invalid in (("encoder_noise", "quantum_rad", 0.),
            ("gyro_noise", "iid", False), ("current_noise", "sigma_A_reported", -1.),
            ("source_clock", "scale", 0.), ("source_clock", "scale", 1.001),
            ("source_clock", "offset_s", .001), ("source_clock", "status", "UNKNOWN")):
            with self.subTest(group=group, quantity=quantity):
                assumptions = self.asset.document()["assumptions"]
                assumptions[group][quantity] = invalid
                with self.assertRaises(Rejected): replace(self.asset, assumptions=assumptions)
        for invalid in ([0.]*4, [3., 0., 0., 0., 0.], [0., float("nan"), 0., 0., 0.]):
            gauges = self.asset.document()["gauges"]
            gauges["acquisition_state"]["value"] = invalid
            with self.assertRaises(Rejected): replace(self.asset, gauges=gauges)

    def test_failed_gates_persist_diagnostically_but_cannot_qualify(self):
        quality = self.asset.document()["estimator_quality"]
        quality["synthetic_parameter_recovery"] = "FAIL"
        diagnostic = replace(self.asset, estimator_quality=quality)
        self.assertEqual(FamilyAsset.from_document(diagnostic.document()).estimator_quality[
            "synthetic_parameter_recovery"], "FAIL")
        with self.assertRaises(Rejected) as error:
            replace(diagnostic, qualification="QUALIFIED_SYNTHETIC_MODEL")
        self.assertEqual(error.exception.reason, Reason.MODEL_INADEQUATE)

    def test_consumed_receipt_cannot_be_relabelled_fresh_or_qualified(self):
        with self.assertRaises(Rejected): replace(self.asset, evidence_partition="FINAL_FRESH")
        with self.assertRaises(Rejected): replace(self.asset, qualification="QUALIFIED_SYNTHETIC_MODEL")
        # Even a new partition declaration does not supply the missing independent evidence/uncertainty.
        provenance = self.asset.document()["provenance"]
        provenance["evidence_partition"] = "FINAL_FRESH"
        with self.assertRaises(Rejected):
            replace(self.asset, evidence_partition="FINAL_FRESH", provenance=provenance,
                qualification="QUALIFIED_SYNTHETIC_MODEL")

    def test_supported_uncertainty_cannot_change_structure_fixed_gauges_or_context(self):
        valid = {"status": "SUPPORTED", "parameter_sets": [replace(self.asset.model, a=.12).document()],
            "basis": "SUPPORTED_JOINT_ESTIMATION_ENSEMBLE", "evidence_reference": "synthetic-ensemble.json",
            "model_revision": self.asset.model_revision,
            "configuration_id": self.asset.configuration_support.baseline.context_id}
        self.assertEqual(replace(self.asset, uncertainty=valid).uncertainty["status"], "SUPPORTED")
        for model in (replace(self.asset.model, actuator_gain=2.),
            replace(self.asset.model, load="affine", load_slope=.01)):
            invalid = {**valid, "parameter_sets": [model.document()]}
            with self.assertRaises(Rejected): replace(self.asset, uncertainty=invalid)
        with self.assertRaises(Rejected):
            replace(self.asset, uncertainty={**valid, "configuration_id": "different context"})

    def test_qualified_declaration_requires_fresh_references_joint_support_and_frozen_protocol(self):
        # Declared synthetic receipt contract only; this fixture is not estimator evidence.
        provenance = {**self.asset.document()["provenance"], "evidence_partition": "FINAL_FRESH",
            "fit_role": "FINAL_FRESH CONTRACT FIXTURE", "protocol_frozen_before_observations": True}
        quality = {**self.asset.document()["estimator_quality"], "fresh_excitation_and_noise": "PASS",
            "gate_evidence": dict.fromkeys((*QUALITY_GATES, "fresh_excitation_and_noise"), "fresh-contract-fixture.json")}
        uncertainty = {"status": "SUPPORTED", "parameter_sets": [self.asset.model.document()]*128,
            "basis": "SUPPORTED_JOINT_ESTIMATION_ENSEMBLE", "evidence_reference": "joint-contract-fixture.json",
            "model_revision": self.asset.model_revision,
            "configuration_id": self.asset.configuration_support.baseline.context_id}
        declared = replace(self.asset, evidence_partition="FINAL_FRESH", provenance=provenance,
            estimator_quality=quality, uncertainty=uncertainty, qualification="QUALIFIED_SYNTHETIC_MODEL")
        self.assertFalse(declared.document()["deployment_authorized"])
        with self.assertRaises(Rejected):
            replace(declared, provenance={**provenance, "protocol_frozen_before_observations": False})
        with self.assertRaises(Rejected): replace(declared, estimator_quality={**quality, "gate_evidence": {}})
        with self.assertRaises(Rejected): replace(declared, estimator_quality={**quality, "optimizer_converged": False})
        with self.assertRaises(Rejected): replace(declared, uncertainty={**uncertainty, "parameter_sets": uncertainty["parameter_sets"][:127]})

    def test_export_rejects_unit_travel_physical_and_extra_claims(self):
        edits = (("model_units", {**MODEL_UNITS, "a": "kg*m^2"}),
            ("numerical_domain", {"q_min_rad": -2., "q_max_rad": 2., "role": "VERIFIED_TRAVEL"}),
            ("deployment_authorized", True), ("physical_stage3a", "PASS"),
            ("production_qualified", True))
        for key, value in edits:
            with self.subTest(key=key):
                document = self.asset.document()
                document[key] = value
                with self.assertRaises(Rejected): FamilyAsset.from_document(document)

    def test_model_revision_configuration_and_fit_coordinates_are_required(self):
        with self.assertRaises(Rejected): replace(self.asset, model_revision="unrelated fit")
        for free in ([], ["a", "a"], ["max_step"], ["invented_torque"]):
            quality = self.asset.document()["estimator_quality"]
            quality["free_parameters"] = free
            with self.assertRaises(Rejected): replace(self.asset, estimator_quality=quality)
        provenance = self.asset.document()["provenance"]
        provenance["source_paths"] = []
        with self.assertRaises(Rejected): replace(self.asset, provenance=provenance)

    def test_complete_native_readback_maps_fitted_units_and_preserves_supplied_template(self):
        native, template = Native(), controller_parameters()
        before = native_parameter_document(template)
        parameters, support, report = bind_diagnostic_runtime(self.asset, native, template, self.support)
        mapped = native_parameter_document(parameters)
        self.assertEqual(native_parameter_document(template), before)
        self.assertEqual(mapped["observer"], before["observer"])
        for name in CONTROL_FIELDS: self.assertEqual(mapped[name], before[name])
        self.assertEqual(mapped["start_censored"], before["start_censored"])
        self.assertEqual(len(mapped["model"]["theta"]), 55)
        self.assertEqual(len(mapped["start_total"]), 48)
        self.assertEqual(mapped["model"]["theta"][0], .113/1.7)
        self.assertEqual(mapped["model"]["theta"][6], (.02-.117-.023)/1.7)
        self.assertEqual(mapped["start_total"][0], (.02-.16-.023)/1.7)
        self.assertEqual(report["native_abi"], 4)
        self.assertEqual(report["complete_parameter_readback"], "PASS")
        self.assertEqual(report["synthesis"], "NOT_RUN_FOR_THIS_ASSET")
        self.assertEqual(support.configuration_support.model_revision, self.asset.model_revision)

    def test_runtime_binding_rejects_frame_context_and_ff_support_outside_guard(self):
        native, template = Native(), controller_parameters()
        for support in (replace(self.support, frame="wrong frame"),
            replace(self.support, configuration_id="wrong context"),
            replace(self.support, q_min_rad=-3.)):
            with self.assertRaises(Rejected): bind_diagnostic_runtime(self.asset, native, template, support)

    def test_runtime_roundtrip_rechecks_actual_native_and_rejects_candidate_or_provenance_tampering(self):
        native, template = Native(), controller_parameters()
        parameters, support, report = bind_diagnostic_runtime(self.asset, native, template, self.support)
        receipt = json.loads(json.dumps(runtime_document(self.asset, template, parameters, support, report)))
        asset, actual, _, binding = bind_runtime_document(receipt, native)
        self.assertEqual(asset.document(), self.asset.document())
        self.assertEqual(native_parameter_document(actual), receipt["mapped_controller_candidate"])
        self.assertEqual(binding["complete_parameter_readback"], "PASS")
        for field in ("kp", "start_total", "observer", "model"):
            edited = copy.deepcopy(receipt)
            candidate = edited["mapped_controller_candidate"]
            if field == "kp": candidate[field] += .01
            elif field == "start_total": candidate[field][-1] += .01
            elif field == "observer": candidate[field]["process_variance"] += .01
            else: candidate[field]["theta"][-1] += .01
            with self.subTest(field=field):
                with self.assertRaises(Rejected) as error: bind_runtime_document(edited, native)
                self.assertEqual(error.exception.reason, Reason.INTEGRATION_MISMATCH)
        edited = copy.deepcopy(receipt)
        edited["actual_binding"]["native_abi"] = 3
        with self.assertRaises(Rejected): bind_runtime_document(edited, native)
        edited = copy.deepcopy(receipt)
        del edited["supplied_controller_template"]["start_censored"]
        with self.assertRaises(Rejected): bind_runtime_document(edited, native)


if __name__ == "__main__": unittest.main()
