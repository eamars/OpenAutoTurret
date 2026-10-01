"""Configuration contradictions, evidence binding and native inhibition regressions."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from Firmware.commissioning.applicability import (ConfigurationFact, ConfigurationFacts,
    ConfigurationResponse, ConfigurationSupport, FactStatus, OperatingDomain,
    applicable_rollback, assess_configuration, configuration_pool, supported_snapshot)
from Firmware.commissioning.contracts import Rejected
from Firmware.tools.adr0022_configuration_probe import context


class ConfigurationSupportTests(unittest.TestCase):
    def setUp(self):
        self.context = context(provenance="MEASURED", status=FactStatus.VERIFIED)
        self.support = ConfigurationSupport("model-A", self.context, tuple(self.context.facts),
                                            qualification="PREDICTIVE_MODEL")

    def run_record(self, name, facts, provenance="MEASURED"):
        return SimpleNamespace(run_id=name, configuration_facts=facts, provenance=provenance)

    def test_equal_mass_relocation_invalidates_parameters_and_both_certificates(self):
        changed = context(.2, provenance="MEASURED", status=FactStatus.VERIFIED)
        decision = assess_configuration(self.support, changed, purpose="PHYSICAL_MODEL")
        self.assertFalse(decision["compatible"])
        self.assertTrue({"plant_parameters", "3a", "3b"} <= set(decision["invalidate"]))
        self.assertIn("identification_method", decision["preserve"])

    def test_invariants_and_facts_cannot_be_removed_by_caller_mutation(self):
        names = list(self.context.facts)
        binding = replace(self.support, invariant_fields=names)
        names.remove("motor.settings")
        self.assertIn("motor.settings", binding.invariant_fields)
        with self.assertRaises(TypeError):
            self.context.facts["motor.settings"] = ConfigurationFact("new", FactStatus.VERIFIED, "readback")

    def test_unclassified_baseline_is_rejected_and_new_current_fact_invalidates(self):
        with self.assertRaises(Rejected):
            replace(self.support, invariant_fields=("payload.distribution",))
        changed = replace(self.context, facts={**self.context.facts,
            "clock.mapping": ConfigurationFact("new-clock", FactStatus.VERIFIED, "clock check")})
        self.assertFalse(assess_configuration(self.support, changed)["compatible"])

    def test_unknown_first_run_cannot_hide_later_known_conflicting_motor_settings(self):
        unknown = replace(self.context, facts={**self.context.facts,
            "motor.settings": ConfigurationFact(None, FactStatus.UNKNOWN, "historical missing readback")})
        other = replace(self.context, facts={**self.context.facts,
            "motor.settings": ConfigurationFact("new-settings", FactStatus.VERIFIED, "readback")})
        with self.assertRaises(Rejected):
            configuration_pool([self.run_record("unknown", unknown), self.run_record("A", self.context),
                                self.run_record("B", other)])

    def test_measured_run_cannot_bind_synthetic_configuration(self):
        with self.assertRaises(Rejected):
            configuration_pool([self.run_record("measured", context())])

    def test_legacy_unknown_context_remains_diagnostic(self):
        result = configuration_pool([self.run_record("legacy", None)])
        self.assertEqual(result["support_status"], "UNKNOWN_CONFIGURATION_DIAGNOSTIC_ONLY")
        self.assertFalse(result["physical_qualification"])

    def test_supported_temperature_variation_requires_explicit_interval(self):
        facts = replace(self.context, facts={**self.context.facts,
            "temperature_C": ConfigurationFact(20., FactStatus.VERIFIED, "calibrated thermometer")})
        binding = replace(self.support, baseline=facts, numeric_support={"temperature_C": (10., 30.)})
        warm = replace(facts, facts={**facts.facts,
            "temperature_C": ConfigurationFact(25., FactStatus.VERIFIED, "calibrated thermometer")})
        pooled = configuration_pool([self.run_record("cold", facts), self.run_record("warm", warm)], support=binding)
        self.assertEqual(pooled["support_status"], "EXPLICIT_DIAGNOSTIC_SUPPORT")
        outside = replace(warm, facts={**warm.facts,
            "temperature_C": ConfigurationFact(35., FactStatus.VERIFIED, "calibrated thermometer")})
        with self.assertRaises(Rejected):
            configuration_pool([self.run_record("hot", outside)], support=binding)

    def test_response_cannot_substitute_for_unknown_interface_settings(self):
        with self.assertRaises(Rejected):
            replace(self.support, unknown_with_response=("motor.settings",))
        partial = replace(self.context, facts={**self.context.facts,
            "motor.settings": ConfigurationFact(None, FactStatus.UNKNOWN, "missing readback")})
        binding = replace(self.support, baseline=partial)
        decision = assess_configuration(binding, partial, purpose="PHYSICAL_MODEL",
            response_evidence=(ConfigurationResponse("model-A", partial, True),))
        self.assertFalse(decision["supported"])

    def test_optional_unknown_mass_uses_exact_model_and_context_response(self):
        facts = replace(self.context, facts={**self.context.facts,
            "payload.mass_kg": ConfigurationFact(None, FactStatus.UNKNOWN, "unmeasured mass")})
        binding = replace(self.support, baseline=facts, invariant_fields=tuple(facts.facts),
                          unknown_with_response=("payload.mass_kg",))
        evidence = (ConfigurationResponse("model-A", facts, True),)
        self.assertTrue(assess_configuration(binding, facts, purpose="PHYSICAL_MODEL",
                                             response_evidence=evidence)["supported"])
        wrong = (ConfigurationResponse("another-model", facts, True),)
        self.assertFalse(assess_configuration(binding, facts, purpose="PHYSICAL_MODEL",
                                              response_evidence=wrong)["supported"])

    def test_supported_rollback_uses_descriptive_revision_and_its_own_response(self):
        identity = SimpleNamespace(hardware="fixture-hardware", measurement="fixture-sensors")
        snapshot = SimpleNamespace(identity=identity, fit_report={"model_revision": "model-A"})
        domain = OperatingDomain("fixture-hardware", "fixture-sensors", {}, {})
        other = context(.2, provenance="MEASURED", status=FactStatus.VERIFIED)
        with self.assertRaises(Rejected):
            applicable_rollback([snapshot], domain, {}, hardware=identity.hardware, measurement=identity.measurement,
                configuration_supports=[self.support], current_configuration=self.context,
                response_evidence=[ConfigurationResponse("model-A", other, True)])
        chosen = applicable_rollback([snapshot], domain, {}, hardware=identity.hardware, measurement=identity.measurement,
            configuration_supports=[self.support], current_configuration=self.context,
            response_evidence=[ConfigurationResponse("model-A", self.context, True)])
        self.assertIs(chosen, snapshot)

    def test_reported_settings_do_not_equal_verified_physical_support(self):
        reported = replace(self.context, facts={**self.context.facts,
            "motor.settings": ConfigurationFact("declared-current-map", FactStatus.REPORTED, "defaults screenshot")})
        decision = assess_configuration(self.support, reported, purpose="PHYSICAL_MODEL",
            response_evidence=[ConfigurationResponse("model-A", reported, True)])
        self.assertFalse(decision["supported"])

    def test_boolean_setting_cannot_match_numeric_response_setting(self):
        facts = replace(self.context, facts={**self.context.facts,
            "motor.settings": ConfigurationFact(True, FactStatus.VERIFIED, "readback")})
        numeric = replace(facts, facts={**facts.facts,
            "motor.settings": ConfigurationFact(1, FactStatus.VERIFIED, "readback")})
        binding = replace(self.support, baseline=facts)
        result = assess_configuration(binding, facts, purpose="PHYSICAL_MODEL",
            response_evidence=[ConfigurationResponse("model-A", numeric, True)])
        self.assertFalse(result["supported"])
        self.assertFalse(result["model_context_prediction_passed"])


if __name__ == "__main__":
    unittest.main()
