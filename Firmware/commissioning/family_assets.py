"""Synthetic selected-family receipts; no physical qualification or promotion API."""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass, fields
import math
from types import MappingProxyType

from .applicability import (ConfigurationFact, ConfigurationFacts, ConfigurationSupport,
                            FactStatus, assess_configuration)
from .contracts import Reason, require
from .model_family import FamilyModel
from .motor_feedforward import BoundedStartPolicy, FeedforwardSupport, MotorFeedforward, parameters_for_family
from .native import CParameters, Controller, Native


MODEL_UNITS = {
    "a": "A_effective*s^2/rad", "viscous": "A_effective*s/rad",
    **{k: "A_effective" for k in ("coulomb_negative", "coulomb_positive", "static_negative",
        "static_positive", "load_offset", "actuator_bias")}, "current_bias": "A_reported",
    "load_slope": "A_effective/rad", "actuator_gain": "A_effective/A_command",
    "current_gain": "A_reported/A_effective", "gyro_bias": "rad/s",
    **{k: "s" for k in ("actuator_tau", "transport_delay", "gyro_tau", "gyro_delay",
        "current_tau", "current_delay", "max_step")},
    **{k: "rad" for k in ("q_min", "q_max", "q_origin")},
    **{k: "rad/s" for k in ("stribeck_negative", "stribeck_positive")},
    "stribeck_power": "1"}
REQUIRED_GAUGES = ("load_friction", "input_current_map", "reported_current_map",
                   "sensor_biases", "static_thresholds", "acquisition_state")
REQUIRED_ASSUMPTIONS = ("encoder_noise", "gyro_noise", "current_noise", "source_clock")
QUALITY_GATES = ("data_integrity", "forward_numerics", "synthetic_parameter_recovery",
                 "training_trajectory", "historical_regression", "held_out_prescribed_input_prediction")
NUMERICAL_DOMAIN_ROLE = "NUMERICAL_ROLLOUT_GUARD; TRAVEL/FF_SUPPORT_SUPPLIED_SEPARATELY"
FITTABLE_FIELDS = set(MODEL_UNITS) - {"q_min", "q_max", "max_step"}


def _frozen(value):
    if isinstance(value, (dict, MappingProxyType)):
        require(all(type(k) is str and k.strip() for k in value), Reason.DATA_INVALID,
                "receipt keys must be nonempty strings")
        return MappingProxyType({k: _frozen(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)): return tuple(_frozen(v) for v in value)
    require(value is None or type(value) in (str, bool, int, float), Reason.DATA_INVALID,
            "receipt values must be serializable quantities, statuses or references")
    if type(value) is float:
        require(math.isfinite(value), Reason.DATA_INVALID, "finite receipt quantity required")
    return value


def _plain(value):
    if isinstance(value, (dict, MappingProxyType)): return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [_plain(v) for v in value]
    return value


def model_from_document(document):
    names = {field.name for field in fields(FamilyModel)}
    metadata = {"schema", "structure", "parameter_units", "q_domain_role", "qualification"}
    require(isinstance(document, (dict, MappingProxyType)) and
            set(document) <= names | metadata and names <= set(document), Reason.DATA_INVALID,
            "complete explicit selected-family model required")
    require(all(_number(document[k]) for k in MODEL_UNITS), Reason.DATA_INVALID,
            "model receipt quantities must be finite numeric values, never boolean labels")
    model = FamilyModel(**{k: document[k] for k in names}).validate()
    require(document.get("structure", model.structure) == model.structure, Reason.DATA_INVALID,
            "selected structure and parameter document disagree")
    return model


def _reference(value):
    return type(value) is str and bool(value.strip())


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _validate_assumptions(model, gauges, assumptions):
    bindings = (("load_friction", "load_offset_A_effective", "load_offset"),
        ("load_friction", "load_slope_A_effective_rad", "load_slope"),
        ("input_current_map", "gain_A_effective_per_A_command", "actuator_gain"),
        ("input_current_map", "bias_A_effective", "actuator_bias"),
        ("reported_current_map", "gain_A_reported_per_A_effective", "current_gain"),
        ("reported_current_map", "bias_A_reported", "current_bias"),
        ("sensor_biases", "gyro_rad_s", "gyro_bias"),
        ("static_thresholds", "negative_A_effective", "static_negative"),
        ("static_thresholds", "positive_A_effective", "static_positive"))
    for group, quantity, parameter in bindings:
        value = gauges[group].get(quantity)
        require(_number(value) and value == getattr(model, parameter), Reason.DATA_INVALID,
                f"{group}.{quantity} contradicts the selected model")
    require(_reference(gauges["load_friction"].get("rule")) and
            _reference(gauges["sensor_biases"].get("encoder_datum")) and
            _reference(gauges["static_thresholds"].get("status")), Reason.DATA_INVALID,
            "load/friction gauge, encoder datum and static-threshold status required")
    initial = gauges["acquisition_state"]
    require(isinstance(initial.get("value"), (list, tuple)) and len(initial["value"]) == 5 and
            all(_number(x) for x in initial["value"]) and model.q_min <= initial["value"][0] <= model.q_max and
            type(initial.get("count_per_run")) is int and initial["count_per_run"] == 1 and
            initial.get("status") == "SUPPLIED_SYNTHETIC", Reason.DATA_INVALID,
            "one supplied finite five-state synthetic acquisition state required")
    positive_noise = (("encoder_noise", "Gaussian_before_rounding_sigma_rad"),
        ("encoder_noise", "quantum_rad"), ("gyro_noise", "sigma_rad_s"),
        ("gyro_noise", "fresh_hz"), ("current_noise", "sigma_A_reported"),
        ("current_noise", "fresh_hz"))
    for group, quantity in positive_noise:
        value = assumptions[group].get(quantity)
        require(_number(value) and value > 0, Reason.DATA_INVALID,
                "explicit positive estimator noise and sampling quantities required")
    require(assumptions["gyro_noise"].get("iid") is True and
            assumptions["current_noise"].get("iid") is True, Reason.DATA_INVALID,
            "this receipt supports only the declared independent Gaussian noise experiment")
    clock = assumptions["source_clock"]
    require(_number(clock.get("scale")) and clock["scale"] == 1. and
            _number(clock.get("offset_s")) and clock["offset_s"] == 0. and
            clock.get("status") == "KNOWN_SYNTHETIC_COMMON_CLOCK" and
            _reference(clock.get("gyro_source_age")), Reason.DATA_INVALID,
            "known synthetic clock and filter/source-age convention required")


@dataclass(frozen=True, kw_only=True)
class FamilyAsset:
    model_revision: str
    model: FamilyModel
    frame: str
    configuration_support: ConfigurationSupport
    gauges: dict
    assumptions: dict
    estimator_quality: dict
    evidence_partition: str
    uncertainty: dict
    provenance: dict
    qualification: str = "DIAGNOSTIC_SYNTHETIC"

    def __post_init__(self):
        require(isinstance(self.model, FamilyModel), Reason.DATA_INVALID, "typed selected family required")
        require(all(_number(getattr(self.model, k)) for k in MODEL_UNITS), Reason.DATA_INVALID,
                "finite numeric model quantities required")
        self.model.validate()
        require(type(self.model_revision) is str and bool(self.model_revision) and
                type(self.frame) is str and bool(self.frame), Reason.DATA_INVALID, "revision/frame required")
        support = self.configuration_support
        require(isinstance(support, ConfigurationSupport) and support.model_revision == self.model_revision
                and support.baseline.provenance == "SYNTHETIC" and support.qualification == "SYNTHETIC_OFFLINE",
                Reason.OPERATING_POINT_CHANGED, "receipt binds exact synthetic model/configuration support")
        require(assess_configuration(support, support.baseline, purpose="SYNTHETIC_CONTROL")["supported"],
                Reason.OPERATING_POINT_CHANGED, "explicit supported synthetic configuration required")
        require(isinstance(self.gauges, (dict, MappingProxyType)) and
                isinstance(self.assumptions, (dict, MappingProxyType)) and
                set(REQUIRED_GAUGES) <= set(self.gauges) and set(REQUIRED_ASSUMPTIONS) <= set(self.assumptions) and
                all(isinstance(self.gauges[k], (dict, MappingProxyType)) for k in REQUIRED_GAUGES) and
                all(isinstance(self.assumptions[k], (dict, MappingProxyType)) for k in REQUIRED_ASSUMPTIONS),
                Reason.DATA_INVALID, "input/load/sensor/static/state gauges and noise/source-clock assumptions required")
        _validate_assumptions(self.model, self.gauges, self.assumptions)
        quality = self.estimator_quality
        require(isinstance(quality, (dict, MappingProxyType)) and set(QUALITY_GATES) <= set(quality) and
                all(_reference(quality[k]) for k in QUALITY_GATES) and
                type(quality.get("optimizer_converged")) is bool,
                Reason.DATA_INVALID, "separate estimator gates and optimizer predicate required")
        require(quality.get("physical_stage3a", "NOT_RUN") == quality.get("physical_stage3b", "NOT_RUN") == "NOT_RUN"
                and quality.get("deployment_authorized", False) is False, Reason.DATA_INVALID,
                "synthetic estimator receipts cannot contain physical or deployment qualification")
        free = quality.get("free_parameters")
        require(isinstance(free, (tuple, list)) and bool(free) and all(type(k) is str for k in free) and
                len(set(free)) == len(free) and set(free) <= FITTABLE_FIELDS, Reason.DATA_INVALID,
                "exact distinct fitted quantities must be declared")
        require(self.evidence_partition in ("DEVELOPMENT_CONSUMED", "FINAL_FRESH") and
                self.qualification in ("DIAGNOSTIC_SYNTHETIC", "QUALIFIED_SYNTHETIC_MODEL"),
                Reason.DATA_INVALID, "explicit synthetic partition and qualification required")
        require(isinstance(self.provenance, (dict, MappingProxyType)) and
                _reference(self.provenance.get("fit_evidence")) and
                isinstance(self.provenance.get("source_paths"), (tuple, list)) and bool(self.provenance["source_paths"]) and
                all(_reference(x) for x in self.provenance["source_paths"]) and
                _reference(self.provenance.get("source_revision")) and _reference(self.provenance.get("fit_role")) and
                self.provenance.get("evidence_partition") == self.evidence_partition and
                self.provenance.get("scope") == "SYNTHETIC_OFFLINE", Reason.DATA_INVALID,
                "fit/source/revision provenance required; physical claims unsupported")
        require(isinstance(self.uncertainty, (dict, MappingProxyType)) and
                self.uncertainty.get("status") in ("UNKNOWN", "SUPPORTED") and
                isinstance(self.uncertainty.get("parameter_sets"), (tuple, list)), Reason.DATA_INVALID,
                "supported uncertainty and diagnostic unknown must be separate")
        members = self.uncertainty["parameter_sets"]
        require(self.uncertainty["status"] != "UNKNOWN" or not members, Reason.DATA_INVALID,
                "unknown uncertainty cannot masquerade as supported parameter sets")
        require(_reference(self.uncertainty.get("basis")), Reason.DATA_INVALID,
                "uncertainty support basis or reason for unknown required")
        if self.uncertainty["status"] == "SUPPORTED":
            require(bool(members) and _reference(self.uncertainty.get("evidence_reference")) and
                    self.uncertainty.get("model_revision") == self.model_revision and
                    self.uncertainty.get("configuration_id") == support.baseline.context_id and
                    self.uncertainty.get("basis") == "SUPPORTED_JOINT_ESTIMATION_ENSEMBLE",
                    Reason.DATA_INVALID, "supported uncertainty needs joint-estimation evidence and exact context")
        for member in members:
            candidate = model_from_document(member)
            require(candidate.structure == self.model.structure and all(getattr(candidate, f.name) ==
                    getattr(self.model, f.name) for f in fields(FamilyModel) if f.name not in free),
                    Reason.DATA_INVALID, "uncertainty cannot change selected structure, fixed gauges or numerical guards")
        if self.qualification == "QUALIFIED_SYNTHETIC_MODEL":
            references = quality.get("gate_evidence", {})
            require(self.evidence_partition == "FINAL_FRESH" and quality["optimizer_converged"] and
                    all(quality[k] == "PASS" for k in QUALITY_GATES) and
                    quality.get("fresh_excitation_and_noise") == "PASS" and
                    self.uncertainty["status"] == "SUPPORTED" and len(members) == 128 and
                    isinstance(references, (dict, MappingProxyType)) and
                    all(_reference(references.get(k)) for k in (*QUALITY_GATES, "fresh_excitation_and_noise")) and
                    self.provenance.get("protocol_frozen_before_observations") is True,
                    Reason.MODEL_INADEQUATE,
                    "qualified model needs fresh independent gates and supported joint uncertainty")
        for name in ("gauges", "assumptions", "estimator_quality", "uncertainty", "provenance"):
            object.__setattr__(self, name, _frozen(getattr(self, name)))

    def document(self):
        support = self.configuration_support
        return {"schema": "adr0022.family-asset/1", "model_revision": self.model_revision,
            "model": self.model.document(), "model_units": MODEL_UNITS.copy(), "frame": self.frame,
            "configuration_support": {"model_revision": support.model_revision,
                "baseline": _plain(support.baseline.document()), "invariant_fields": list(support.invariant_fields),
                "numeric_support": _plain(support.numeric_support), "unknown_with_response": list(support.unknown_with_response),
                "qualification": support.qualification},
            **{k: _plain(getattr(self, k)) for k in ("gauges", "assumptions", "estimator_quality",
                "uncertainty", "provenance")}, "evidence_partition": self.evidence_partition,
            "qualification": self.qualification,
            "numerical_domain": {"q_min_rad": self.model.q_min, "q_max_rad": self.model.q_max,
                "role": NUMERICAL_DOMAIN_ROLE},
            "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}

    @classmethod
    def from_document(cls, document):
        names = {f.name for f in fields(cls)}
        require(isinstance(document, dict) and set(document) == names | {
                "schema", "model_units", "numerical_domain", "physical_stage3a", "physical_stage3b", "deployment_authorized"} and
                document.get("schema") == "adr0022.family-asset/1" and
                document.get("model_units") == MODEL_UNITS and
                document.get("physical_stage3a") == document.get("physical_stage3b") == "NOT_RUN" and
                document.get("deployment_authorized") is False, Reason.DATA_INVALID,
                "known schema/units and synthetic-only receipt required")
        model = model_from_document(document["model"])
        require(document["numerical_domain"] == {"q_min_rad": model.q_min, "q_max_rad": model.q_max,
                "role": NUMERICAL_DOMAIN_ROLE}, Reason.DATA_INVALID,
                "numerical rollout guard must match model and cannot become a travel certificate")
        raw = document["configuration_support"]
        require(isinstance(raw, dict) and set(raw) == {"model_revision", "baseline", "invariant_fields",
                "numeric_support", "unknown_with_response", "qualification"} and
                isinstance(raw["baseline"], dict) and set(raw["baseline"]) == {"context_id", "provenance", "facts"} and
                isinstance(raw["baseline"]["facts"], dict) and
                all(isinstance(v, dict) and set(v) == {"value", "status", "source"} and
                    v["status"] in {status.value for status in FactStatus}
                    for v in raw["baseline"]["facts"].values()), Reason.DATA_INVALID,
                "complete typed configuration support document required")
        baseline = raw["baseline"]
        facts = ConfigurationFacts(baseline["context_id"], baseline["provenance"],
            {k: ConfigurationFact(v["value"], FactStatus(v["status"]), v["source"])
             for k, v in baseline["facts"].items()})
        support = ConfigurationSupport(raw["model_revision"], facts, tuple(raw["invariant_fields"]),
            {k: tuple(v) for k, v in raw["numeric_support"].items()}, tuple(raw["unknown_with_response"]), raw["qualification"])
        return cls(model=model, configuration_support=support,
            **{k: document[k] for k in ("model_revision", "frame", "gauges", "assumptions", "estimator_quality",
                "evidence_partition", "uncertainty", "provenance", "qualification")})


def native_parameter_document(parameters):
    """Every native field/array, including inactive table slots; no padding bytes."""
    if isinstance(parameters, ct.Structure):
        return {name: native_parameter_document(getattr(parameters, name)) for name, *_ in parameters._fields_}
    if isinstance(parameters, ct.Array): return [native_parameter_document(x) for x in parameters]
    return parameters


def bind_diagnostic_runtime(asset, native, template, feedforward_support):
    """Verify one copied candidate using actual native readback and public FF."""
    from dataclasses import replace
    require(isinstance(asset, FamilyAsset) and isinstance(native, Native) and
            isinstance(template, CParameters) and isinstance(feedforward_support, FeedforwardSupport), Reason.DATA_INVALID,
            "typed asset and explicit diagnostic controller template required")
    require(feedforward_support.frame == asset.frame and
            feedforward_support.configuration_id == asset.configuration_support.baseline.context_id,
            Reason.OPERATING_POINT_CHANGED, "runtime frame/configuration differs from receipt")
    require(asset.model.q_min <= feedforward_support.q_min_rad < feedforward_support.q_max_rad <= asset.model.q_max,
            Reason.MODEL_INADEQUATE, "supplied FF support must fit inside the separate numerical rollout guard")
    support = replace(feedforward_support, configuration_support=asset.configuration_support)
    parameters = parameters_for_family(template, asset.model, actuator_policy=support.actuator_policy,
        actuation_memory_max_s=support.actuation_memory_max_s, start_policy=support.start_policy)
    with Controller(native, parameters) as controller:
        readback = controller.read_parameters()
        require(native_parameter_document(parameters) == native_parameter_document(readback),
                Reason.INTEGRATION_MISMATCH, "complete native parameter readback differs")
        MotorFeedforward(controller, asset.model, support)
    return parameters, support, {"native_abi": int(native.lib.ota_core_abi()), "native_library": str(native.path),
        "complete_parameter_readback": "PASS", "model_revision": asset.model_revision,
        "candidate_origin": "SUPPLIED_DIAGNOSTIC_TEMPLATE", "synthesis": "NOT_RUN_FOR_THIS_ASSET",
        "model_qualification": asset.qualification, "controller_qualification": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def _parameters_from_document(document):
    def load(structure, raw):
        require(isinstance(raw, dict) and set(raw) == {name for name, *_ in structure._fields_},
                Reason.DATA_INVALID, "every native parameter field must be serialized exactly")
        for name, *_ in structure._fields_:
            current, value = getattr(structure, name), raw[name]
            if isinstance(current, ct.Structure):
                load(current, value)
            elif isinstance(current, ct.Array):
                require(isinstance(value, list) and len(value) == len(current) and
                        all(_number(x) for x in value), Reason.DATA_INVALID, "complete finite native array required")
                for i, x in enumerate(value):
                    if type(current[i]) is int:
                        require(type(x) is int, Reason.DATA_INVALID, "integer native array requires integers")
                    current[i] = x
                    require(current[i] == x, Reason.DATA_INVALID, "native array value cannot overflow")
            else:
                require(_number(value) and (type(current) is not int or type(value) is int),
                        Reason.DATA_INVALID, "finite native scalar with exact integer type required")
                setattr(structure, name, value)
                require(getattr(structure, name) == value, Reason.DATA_INVALID, "native scalar cannot overflow")
    parameters = CParameters()
    load(parameters, document)
    return parameters


def runtime_document(asset, template, parameters, support, binding):
    """One serialized model/controller/FF receipt; actual binding is rechecked on load."""
    require(isinstance(asset, FamilyAsset) and isinstance(template, CParameters) and
            isinstance(parameters, CParameters) and isinstance(support, FeedforwardSupport),
            Reason.DATA_INVALID, "typed model and explicit runtime candidate required")
    require(getattr(support, "planned_start_program", None) is None, Reason.DATA_INVALID,
            "this diagnostic receipt supports the existing single-episode START policy only")
    raw_support = {}
    for f in fields(support):
        if f.name == "configuration_support": continue  # exact asset context is reused on load
        value = getattr(support, f.name)
        raw_support[f.name] = ({x.name: getattr(value, x.name) for x in fields(value)}
                              if isinstance(value, BoundedStartPolicy) else value)
    return {"schema": "adr0022.family-runtime-receipt/1", "asset": asset.document(),
        "supplied_controller_template": native_parameter_document(template),
        "mapped_controller_candidate": native_parameter_document(parameters),
        "feedforward_support": _plain(_frozen(raw_support)), "actual_binding": _plain(_frozen(binding))}


def bind_runtime_document(document, native):
    """Reload, remap and read back the actual core before exposing a candidate."""
    require(isinstance(document, dict) and set(document) == {"schema", "asset", "supplied_controller_template",
            "mapped_controller_candidate", "feedforward_support", "actual_binding"} and
            document["schema"] == "adr0022.family-runtime-receipt/1", Reason.DATA_INVALID,
            "complete synthetic family runtime receipt required")
    asset = FamilyAsset.from_document(document["asset"])
    template = _parameters_from_document(document["supplied_controller_template"])
    raw = document["feedforward_support"]
    require(isinstance(raw, dict) and set(raw) == {f.name for f in fields(FeedforwardSupport)} - {"configuration_support"}
            and raw.get("planned_start_program") is None, Reason.DATA_INVALID,
            "complete single-episode FF support required")
    raw = raw.copy()
    if raw["start_policy"] is not None:
        policy = raw["start_policy"]
        require(isinstance(policy, dict) and set(policy) == {f.name for f in fields(BoundedStartPolicy)},
                Reason.DATA_INVALID, "complete bounded START declaration required")
        raw["start_policy"] = BoundedStartPolicy(**{k: tuple(v) if k.endswith("interval_A") else v
                                                 for k, v in policy.items()})
    support = FeedforwardSupport(**raw, configuration_support=asset.configuration_support)
    parameters, support, binding = bind_diagnostic_runtime(asset, native, template, support)
    require(native_parameter_document(parameters) == document["mapped_controller_candidate"] and
            binding == document["actual_binding"], Reason.INTEGRATION_MISMATCH,
            "serialized candidate/native provenance differs from the actual rechecked binding")
    return asset, parameters, support, binding
