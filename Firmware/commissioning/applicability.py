"""Bounded operating domains, protection facts, and selective asset invalidation."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
from .contracts import Reason,digest,require


class FactStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    REPORTED = "REPORTED"
    VERIFIED = "VERIFIED"
    OWNER_CONFIRMED = "OWNER_CONFIRMED"
    SYNTHETIC = "SYNTHETIC"


def _fact_value(value):
    if isinstance(value, (tuple, list)):
        return tuple(_fact_value(item) for item in value)
    require(type(value) in (str, bool, int, float) and
            (not isinstance(value, (int, float)) or np.isfinite(value)),
            Reason.DATA_INVALID, "configuration facts need finite explicit scalar/tuple values")
    require(not isinstance(value, str) or bool(value.strip()), Reason.DATA_INVALID,
            "empty configuration value is unknown, not a usable fact")
    return value


def _same_fact_value(first, second):
    if isinstance(first, tuple) and isinstance(second, tuple):
        return len(first) == len(second) and all(_same_fact_value(a, b) for a, b in zip(first, second))
    if type(first) is bool or type(second) is bool:
        return type(first) is type(second) and first == second
    return first == second


@dataclass(frozen=True)
class ConfigurationFact:
    value: object
    status: FactStatus
    source: str

    def __post_init__(self):
        require(isinstance(self.status, FactStatus) and isinstance(self.source, str)
                and bool(self.source.strip()), Reason.DATA_INVALID,
                "configuration fact status and source must be explicit")
        if self.status == FactStatus.UNKNOWN:
            require(self.value is None, Reason.DATA_INVALID, "unknown configuration value must remain None")
        else:
            object.__setattr__(self, "value", _fact_value(self.value))


@dataclass(frozen=True)
class ConfigurationFacts:
    context_id: str  # physical/synthetic assembly context; never a capture procedure ID
    provenance: str
    facts: dict[str, ConfigurationFact]

    def __post_init__(self):
        require(isinstance(self.context_id, str) and bool(self.context_id.strip())
                and self.provenance in ("MEASURED", "SYNTHETIC") and bool(self.facts),
                Reason.DATA_INVALID, "descriptive configuration context, provenance and facts required")
        copied = dict(self.facts)
        require(all(isinstance(k, str) and k.strip() and isinstance(v, ConfigurationFact)
                    for k, v in copied.items()), Reason.DATA_INVALID, "typed configuration facts required")
        require(self.provenance == "SYNTHETIC" or
                all(v.status != FactStatus.SYNTHETIC for v in copied.values()),
                Reason.DATA_INVALID, "synthetic facts cannot be relabelled as measured")
        # Copy prevents mutation of a caller's dictionary from rewriting the binding.
        from types import MappingProxyType
        object.__setattr__(self, "facts", MappingProxyType(copied))

    def document(self):
        return {"context_id": self.context_id, "provenance": self.provenance,
                "facts": {name: {"value": fact.value, "status": fact.status.value,
                                  "source": fact.source} for name, fact in self.facts.items()}}


@dataclass(frozen=True)
class ConfigurationSupport:
    model_revision: str
    baseline: ConfigurationFacts
    invariant_fields: tuple[str, ...]
    numeric_support: dict[str, tuple[float, float]] = field(default_factory=dict)
    unknown_with_response: tuple[str, ...] = ()
    qualification: str = "DIAGNOSTIC_ONLY"

    def __post_init__(self):
        require(isinstance(self.model_revision, str) and bool(self.model_revision.strip())
                and isinstance(self.baseline, ConfigurationFacts)
                and self.qualification in ("DIAGNOSTIC_ONLY", "SYNTHETIC_OFFLINE", "PREDICTIVE_MODEL"),
                Reason.DATA_INVALID, "explicit model revision and configuration support required")
        object.__setattr__(self, "invariant_fields", tuple(self.invariant_fields))
        object.__setattr__(self, "unknown_with_response", tuple(self.unknown_with_response))
        require(len(set(self.invariant_fields)) == len(self.invariant_fields)
                and bool(self.invariant_fields) and all(isinstance(k, str) and k.strip() for k in self.invariant_fields)
                and not set(self.invariant_fields) & set(self.numeric_support)
                and set(self.unknown_with_response) <= set(self.invariant_fields), Reason.DATA_INVALID,
                "distinct configuration invariants/support coordinates required")
        require(set(self.baseline.facts) <= set(self.invariant_fields) | set(self.numeric_support),
                Reason.DATA_INVALID, "every baseline fact must have an invariant or supported interval")
        require(set(self.unknown_with_response) <= {"payload.mass_kg"}, Reason.DATA_INVALID,
                "response may cover unknown payload mass, never motor/settings/calibration/clock facts")
        copied = {}
        for name, bounds in self.numeric_support.items():
            require(isinstance(name, str) and name.strip() and len(bounds) == 2
                    and all(type(v) in (int, float) for v in bounds) and np.isfinite(bounds).all()
                    and bounds[0] <= bounds[1], Reason.DATA_INVALID, "invalid configuration support interval")
            copied[name] = tuple(bounds)
            baseline = self.baseline.facts.get(name)
            require(baseline is None or baseline.status == FactStatus.UNKNOWN or
                    (type(baseline.value) in (int, float) and bounds[0] <= baseline.value <= bounds[1]),
                    Reason.DATA_INVALID, "baseline configuration must lie in its declared support interval")
        from types import MappingProxyType
        object.__setattr__(self, "numeric_support", MappingProxyType(copied))


@dataclass(frozen=True)
class ConfigurationResponse:
    model_revision: str
    context: ConfigurationFacts
    prediction_passed: bool

    def __post_init__(self):
        require(isinstance(self.model_revision, str) and bool(self.model_revision.strip())
                and isinstance(self.context, ConfigurationFacts) and type(self.prediction_passed) is bool,
                Reason.DATA_INVALID, "response evidence must bind an exact model and context")


def _response_matches(evidence, support, current):
    # No global PASS, label-only match, digest or unspecified context wildcard.
    return any(isinstance(item, ConfigurationResponse) and item.prediction_passed
               and item.model_revision == support.model_revision and _same_context(item.context, current)
               for item in evidence)


def _same_context(first, second):
    return (first.context_id == second.context_id and first.provenance == second.provenance
            and set(first.facts) == set(second.facts) and
            all(first.facts[name].status == second.facts[name].status
                and first.facts[name].source == second.facts[name].source
                and _same_fact_value(first.facts[name].value, second.facts[name].value)
                for name in first.facts))


def _configuration_invalidations(fields):
    affected = set()
    for name in fields:
        if name.startswith("sensor.") or name.startswith("clock."):
            affected.update(("measurement_calibration", "observer", "plant_parameters", "controller_candidate", "3a", "3b"))
        else:
            affected.update(("plant_parameters", "controller_candidate", "dual_axis_validation", "3a", "3b"))
    return sorted(affected)


def assess_configuration(support, current, *, response_evidence=(), purpose="DIAGNOSTIC"):
    """Reuse assessment only: never arm, command, retry or certify physical safety."""
    require(isinstance(support, ConfigurationSupport) and isinstance(current, ConfigurationFacts)
            and purpose in ("DIAGNOSTIC", "SYNTHETIC_CONTROL", "PHYSICAL_MODEL"),
            Reason.DATA_INVALID, "typed configuration assessment and purpose required")
    changed, unknown = [], []
    classified = set(support.invariant_fields) | set(support.numeric_support)
    changed.extend(set(current.facts) - classified)
    if purpose == "PHYSICAL_MODEL":
        required = {"hardware.assembly", "payload.distribution", "mounting.geometry", "cable.route",
                    "transmission.mapping", "motor.settings", "sensor.calibration", "base.orientation"}
        unknown.extend(required - classified)
    credible = ({FactStatus.SYNTHETIC} if purpose == "SYNTHETIC_CONTROL" else
                {FactStatus.VERIFIED, FactStatus.OWNER_CONFIRMED})
    for name in support.invariant_fields:
        previous, present = support.baseline.facts.get(name), current.facts.get(name)
        if previous is not None and present is not None and previous.status != FactStatus.UNKNOWN \
                and present.status != FactStatus.UNKNOWN and not _same_fact_value(previous.value, present.value):
            changed.append(name)
        if previous is None or present is None or previous.status == FactStatus.UNKNOWN \
                or present.status == FactStatus.UNKNOWN or (purpose != "DIAGNOSTIC" and
                (previous.status not in credible or present.status not in credible)):
            unknown.append(name)
    for name, (low, high) in support.numeric_support.items():
        fact = current.facts.get(name)
        if fact is None or fact.status == FactStatus.UNKNOWN:
            unknown.append(name)
            continue
        require(type(fact.value) in (int, float), Reason.DATA_INVALID, f"numeric configuration fact required: {name}")
        if not low <= fact.value <= high:
            changed.append(name)
        if purpose != "DIAGNOSTIC" and fact.status not in credible:
            unknown.append(name)
    response_passed = _response_matches(response_evidence, support, current)
    unresolved = [name for name in unknown if name not in support.unknown_with_response or not response_passed]
    provenance_ok = current.provenance == support.baseline.provenance
    if purpose == "SYNTHETIC_CONTROL":
        provenance_ok = provenance_ok and current.provenance == "SYNTHETIC" and support.qualification == "SYNTHETIC_OFFLINE"
    if purpose == "PHYSICAL_MODEL":
        provenance_ok = provenance_ok and current.provenance == "MEASURED" and support.qualification == "PREDICTIVE_MODEL"
    compatible = provenance_ok and not changed
    if not compatible:
        action = "UPDATE_PARAMETERS_AND_REVALIDATE"
    elif unresolved:
        action = "DIAGNOSTIC_ASSUMPTION_ONLY" if purpose == "DIAGNOSTIC" else "NEEDS_CONFIGURATION_EVIDENCE"
    elif purpose == "PHYSICAL_MODEL" and not response_passed:
        action = "NEEDS_MODEL_CONTEXT_PREDICTION"
    else:
        action = "REUSE_WITHIN_SUPPORT"
    authorized = action == "REUSE_WITHIN_SUPPORT"
    return {"action": action, "compatible": compatible, "supported": authorized,
            "model_revision": support.model_revision, "context_id": current.context_id,
            "changed_fields": sorted(set(changed)), "unknown_fields": sorted(set(unknown)),
            "unresolved_fields": sorted(set(unresolved)), "model_context_prediction_passed": response_passed,
            "invalidate": _configuration_invalidations(changed),
            "preserve": ["raw_history", "model_family", "identification_method"],
            "physical_3a": "NOT_RUN", "physical_3b": "NOT_RUN", "deployment_authorized": False}


def configuration_pool(runs, *, support=None):
    """Legacy context stays diagnostic; known incompatible facts cannot be pooled."""
    entries = [(run.run_id, getattr(run, "configuration_facts", None)) for run in runs]
    provided = [facts for _, facts in entries if facts is not None]
    require(all(isinstance(facts, ConfigurationFacts) for facts in provided),
            Reason.DATA_INVALID, "typed run configuration facts required")
    require(all(facts is None or facts.provenance == run.provenance
                for run, (_, facts) in zip(runs, entries)), Reason.DATA_INVALID,
            "configuration provenance must match its observation run")
    if support is None and provided:
        names = tuple(sorted(set().union(*(facts.facts for facts in provided))))
        support = ConfigurationSupport("diagnostic-fixed-family", provided[0], names)
    if support is not None:
        for name in support.invariant_fields:
            known = [facts.facts[name].value for facts in provided if name in facts.facts
                     and facts.facts[name].status != FactStatus.UNKNOWN]
            require(not known or all(_same_fact_value(value, known[0]) for value in known), Reason.OPERATING_POINT_CHANGED,
                    f"conflicting known fixed-family configurations: {name}")
    reports = []
    for run_id, facts in entries:
        if facts is None or support is None:
            decision = {"action": "DIAGNOSTIC_ASSUMPTION_ONLY", "compatible": True,
                        "supported": False, "unknown_fields": ["physical_configuration"],
                        "deployment_authorized": False}
        else:
            decision = assess_configuration(support, facts)
            require(decision["compatible"], Reason.OPERATING_POINT_CHANGED,
                    f"cannot pool fixed-family run {run_id}: {decision['changed_fields']}")
        reports.append({"run_id": run_id, **decision})
    return {"runs": reports, "support_status": "EXPLICIT_DIAGNOSTIC_SUPPORT" if reports and
            all(row["supported"] for row in reports) else "UNKNOWN_CONFIGURATION_DIAGNOSTIC_ONLY",
            "physical_qualification": False}


def supported_snapshot(supports, current, *, response_evidence=()):
    """Select a supported descriptive model revision without applying it to hardware."""
    choices = [support.model_revision for support in supports if
               assess_configuration(support, current, response_evidence=response_evidence,
                                    purpose="PHYSICAL_MODEL")["supported"]]
    require(bool(choices), Reason.OPERATING_POINT_CHANGED,
            "no model has supported configuration and exact context-specific prediction evidence")
    return sorted(choices)[0]


@dataclass(frozen=True)
class OperatingDomain:
    hardware_hash: str
    measurement_hash: str
    categories: dict
    intervals: dict
    unknown_permitted_with_response: tuple[str,...] = ()

    def check(self,hardware,measurement,point,*,independent_prediction_passed):
        require(hardware==self.hardware_hash and measurement==self.measurement_hash,
                Reason.OPERATING_POINT_CHANGED,"hardware or measurement domain changed")
        for name,allowed in self.categories.items():
            require(point.get(name) in allowed,Reason.OPERATING_POINT_CHANGED,
                    f"uncovered operating category: {name}")
        for name,bounds in self.intervals.items():
            value=point.get(name)
            if value is None and name in self.unknown_permitted_with_response:
                require(independent_prediction_passed is True,Reason.OPERATING_POINT_CHANGED,
                        f"unknown {name} requires independent current response validation")
                continue
            require(type(value) in (int,float) and np.isfinite(value),Reason.DATA_INVALID,
                    f"unknown/nonfinite required applicability variable: {name}")
            require(len(bounds)==2 and np.isfinite(bounds).all() and bounds[0]<=value<=bounds[1],
                    Reason.OPERATING_POINT_CHANGED,f"outside identified {name} interval; no extrapolation")
        return True


def protection_decision(observation,bounds,*,elapsed_s):
    """Danger and inability to claim continuous qualification are different results.

    The result specifies a response to the output owner. It never invents a zero-current
    'hold' or sends an independent motor command.
    """
    require(np.isfinite(elapsed_s) and elapsed_s>=0,Reason.DATA_INVALID,"invalid elapsed time")
    if observation.get("estop") is True or observation.get("drive_fault") is True:
        return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    if observation.get("feedback_valid") is not True:
        return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    unknown=[]
    for name in ("current_A","temperature_C","bus_V","position_rad"):
        require(name in bounds,Reason.DATA_INVALID,f"missing injected protection bound {name}")
        value=observation.get(name);limits=bounds[name]
        require(len(limits)==2 and np.isfinite(limits).all() and limits[0]<limits[1],
                Reason.DATA_INVALID,f"invalid protection interval {name}")
        if value is None:
            unknown.append(name);continue
        require(type(value) in (int,float) and np.isfinite(value),Reason.DATA_INVALID,
                f"invalid supervision reading {name}")
        if not limits[0]<=value<=limits[1]:
            return {"reason":"HARD_ABORT","response":"OWNER_VERIFIED_STOP","continuous_qualified":False}
    if elapsed_s>bounds["approved_duration_s"]:
        return {"reason":"ENVELOPE_LIMITED","response":"END_QUALIFICATION_KEEP_VERIFIED_SUPPORT",
                "continuous_qualified":False}
    if unknown:
        finite=(unknown==["temperature_C"] and elapsed_s<=bounds.get("approved_unknown_temperature_window_s",-1))
        return {"reason":"MEASUREMENT_LIMITED","response":"CONTINUE_APPROVED_FINITE_WINDOW" if finite else
                "END_QUALIFICATION_KEEP_VERIFIED_SUPPORT","continuous_qualified":False}
    return {"reason":None,"response":"CONTINUE_WITHIN_APPROVED_ENVELOPE","continuous_qualified":False}


def thermal_equilibrium(time_s,temperature_C,*,temperature_max_C,required_margin_C):
    t=np.asarray(time_s,float);temperature=np.asarray(temperature_C,float)
    require(t.ndim==1 and len(t)>=30 and temperature.shape==t.shape and np.isfinite(t).all()
            and np.isfinite(temperature).all() and np.all(np.diff(t)>0),
            Reason.MEASUREMENT_LIMITED,"valid temperature samples required for thermal qualification")
    require(t[-1]-t[0]>=1800,Reason.MEASUREMENT_LIMITED,"less than minimum 30 minute observation")
    require(np.max(np.diff(t))<=60,Reason.MEASUREMENT_LIMITED,"temperature coverage has gaps over one minute")
    slopes=[]
    for end in (t[-1]-600,t[-1]):
        mask=(t>=end-600)&(t<=end)
        require(mask.sum()>=10,Reason.MEASUREMENT_LIMITED,"insufficient samples in thermal windows")
        slopes.append(float(np.polyfit((t[mask]-end)/60,temperature[mask],1)[0]))
    passed=all(abs(s)<.1 for s in slopes) and temperature.max()<=temperature_max_C-required_margin_C
    return {"passed":bool(passed),"window_slopes_C_per_min":slopes,
            "observed_margin_C":float(temperature_max_C-temperature.max())}


def invalidated_assets(changed_fields):
    affected=set()
    for field in changed_fields:
        if field.startswith("hardware.yaw"):
            affected.update(("yaw_capability","yaw_plant","dual_axis_validation","3a","3b"))
        elif field.startswith("hardware.pitch"):
            affected.update(("pitch_capability","pitch_plant","yaw_posture_family","dual_axis_validation","3a","3b"))
        elif field.startswith("measurement.installation"):
            affected.update(("measurement_calibration","observer","affected_plant_observations","3a","3b"))
        elif field.startswith("measurement.session"):
            affected.update(("session_mapping","session_readiness"))
        elif field.startswith("software.core"):
            affected.update(("core_build","observer","controller_candidate","3a","3b"))
        elif field.startswith("software.production"):
            affected.update(("end_to_end_latency","3b"))
        elif field.startswith("operating_point"):
            affected.update(("applicability_check","parameter_snapshot_if_residual_changed","3a","3b"))
        else:
            require(False,Reason.DATA_INVALID,f"unclassified identity change {field}")
    return {"invalidate":sorted(affected),"preserve":["immutable_raw_history","model_method","identification_solver"]}


def applicable_rollback(snapshots,domain,point,*,hardware,measurement,prediction_checks=None,
                        configuration_supports=(),current_configuration=None,response_evidence=()):
    require(isinstance(current_configuration, ConfigurationFacts) and bool(configuration_supports),
            Reason.OPERATING_POINT_CHANGED, "rollback requires snapshot-specific structured configuration support")
    selected = supported_snapshot(configuration_supports, current_configuration,
                                  response_evidence=response_evidence)
    domain.check(hardware,measurement,point,independent_prediction_passed=True)
    matching=[s for s in snapshots if s.identity.hardware==hardware and s.identity.measurement==measurement
              and s.fit_report.get("model_revision") == selected]
    require(bool(matching),Reason.OPERATING_POINT_CHANGED,
            "no applicable historical snapshot; retain verified support/stop instead of blind rollback")
    return matching[0]
