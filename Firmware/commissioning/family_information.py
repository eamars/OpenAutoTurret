"""Fixed-template information selection for a supplied scalar FamilyModel.

This is the existing 32-template/log-det selector with native family output
sensitivities. It neither commands hardware nor certifies a physical envelope.
"""
from dataclasses import dataclass, replace
import math
from numbers import Real

import numpy as np

from .adaptation import Envelope, FailurePolicy, template
from .contracts import Reason, Rejected, array, require
from .model_family import FamilyModel, MODEL_FIELDS


ENVELOPE_FACTS = ("current", "slew", "speed", "acceleration", "jerk", "travel",
                  "winding", "thermal", "supply", "timeout", "adequate_stop")
CHANNELS = (("q", 0, "rad"), ("gyro", 3, "rad/s"), ("current", 4, "A-reported"))


@dataclass(frozen=True, kw_only=True)
class FamilyInformationSupport:
    configuration_id: str
    model_revision: str
    qualification: str
    evidence: dict
    stop_command_A: float
    stop_hold_s: float
    rest_speed_rad_s: float
    max_stop_drift_rad: float

    def validate(self, envelope):
        require(isinstance(envelope, Envelope), Reason.DATA_INVALID, "existing Envelope required")
        require(type(envelope.provenance) is str and envelope.provenance in ("SYNTHETIC", "MEASURED")
                and type(envelope.stop_verified) is bool, Reason.DATA_INVALID,
                "strict envelope provenance and stop-verification types required")
        positive = ("current_a", "slew_a_s", "duration_s", "velocity_rad_s",
                    "acceleration_rad_s2", "jerk_rad_s3")
        travel = ("angle_min_rad", "angle_max_rad")
        require(all(getattr(envelope, key) is not None for key in positive+travel),
                Reason.ENVELOPE_LIMITED, "BLOCKED_ENVELOPE: unknown numerical envelope bound")
        require(all(isinstance(getattr(envelope, key), Real) and not isinstance(getattr(envelope, key), bool)
                    and math.isfinite(getattr(envelope, key)) for key in positive+travel)
                and all(getattr(envelope, key) > 0 for key in positive)
                and envelope.angle_min_rad < envelope.angle_max_rad,
                Reason.DATA_INVALID, "finite positive current/slew/duration/kinematic bounds and ordered travel required")
        require(isinstance(self.evidence, dict), Reason.ENVELOPE_LIMITED, "BLOCKED_ENVELOPE: qualification facts missing")
        qualification = "SYNTHETIC_FIXTURE" if envelope.provenance == "SYNTHETIC" else "QUALIFIED_PHYSICAL_ENVELOPE"
        missing = [key for key in ENVELOPE_FACTS if not isinstance(self.evidence.get(key), str)
                   or not self.evidence[key].strip()]
        require(self.qualification == qualification and not missing and envelope.stop_verified,
                Reason.ENVELOPE_LIMITED, "BLOCKED_ENVELOPE: missing qualified envelope/stop facts " + str(missing))
        require(type(self.configuration_id) is str and bool(self.configuration_id.strip())
                and type(self.model_revision) is str and bool(self.model_revision.strip()), Reason.DATA_INVALID,
                "configuration and model revision required")
        values = (self.stop_command_A, self.stop_hold_s, self.rest_speed_rad_s, self.max_stop_drift_rad)
        require(all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
                and abs(self.stop_command_A) <= envelope.current_a and self.stop_hold_s >= 2.
                and self.rest_speed_rad_s > 0 and self.max_stop_drift_rad > 0,
                Reason.DATA_INVALID, "supplied bounded stop command and complete two-second rest/drift criteria required")
        return self


@dataclass(frozen=True, kw_only=True)
class FamilyInformationNoise:
    sample_hz: float
    periods: tuple
    sigma: tuple
    source: str
    assumption: str = "INDEPENDENT_UNQUANTIZED_GAUSSIAN_NATIVE_SAMPLES"
    encoder_quantum_rad: float = 0.

    def validate(self):
        require(self.assumption == "INDEPENDENT_UNQUANTIZED_GAUSSIAN_NATIVE_SAMPLES"
                and self.encoder_quantum_rad == 0., Reason.MEASUREMENT_LIMITED,
                "unsupported quantized/correlated likelihood; supply expected-bin Fisher or supported whitening")
        require(isinstance(self.sample_hz, (int, float)) and not isinstance(self.sample_hz, bool)
                and math.isfinite(self.sample_hz) and self.sample_hz > 0 and bool(self.source),
                Reason.MEASUREMENT_LIMITED, "explicit native sampling and noise source required")
        require(len(self.periods) == len(self.sigma) == 3 and all(type(x) is int and x > 0 for x in self.periods)
                and all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and x > 0
                        for x in self.sigma), Reason.MEASUREMENT_LIMITED,
                "q/gyro/reported-current independent native periods and SI sigmas required")
        return self


@dataclass(frozen=True, kw_only=True)
class FamilyInformationCell:
    cell_id: str
    initial: tuple
    baseline_command_A: float
    direction: int
    regime: str

    def validate(self):
        array(self.initial, (5,), "one supplied acquisition state")
        require(bool(self.cell_id) and type(self.direction) is int and self.direction in (-1, 1)
                and self.regime in ("REST_TO_MOVING", "SLIDING")
                and np.isfinite(self.baseline_command_A), Reason.DATA_INVALID, "declared family cell/regime required")
        require(self.initial[1] == 0 if self.regime == "REST_TO_MOVING" else self.direction*self.initial[1] > 0,
                Reason.DATA_INVALID, "declared initial regime contradicts supplied velocity")
        return self


def _commands(case_id, cell, model, envelope, support, sample_hz, amplitude_divisor=1.):
    """Two-second baseline, same normalized template, slew-connected stop."""
    dt = 1. / sample_hz
    center = (model.load_offset + model.load_slope*(cell.initial[0]-model.q_origin)
              - model.actuator_bias) / model.actuator_gain
    amplitude = (envelope.current_a - abs(center))/amplitude_divisor
    if amplitude <= 0 or abs(cell.baseline_command_A) > envelope.current_a:
        return None
    # Reserve the worst cap-to-cap transitions; unused time is not more stimulus.
    reserved = 2. + support.stop_hold_s + 4*envelope.current_a/envelope.slew_a_s
    duration = math.floor(min(4., envelope.duration_s-reserved)*sample_hz)*dt
    if duration <= 0:
        return None
    signal_t, shape = template(case_id, duration, dt)
    t, u = list(np.arange(round(2./dt)+1)*dt), [float(cell.baseline_command_A)]*(round(2./dt)+1)

    def ramp(target):
        count = int(math.ceil(abs(target-u[-1])/envelope.slew_a_s/dt-1e-12))
        begin, old = t[-1], u[-1]
        for k in range(1, count+1):
            t.append(begin+k*dt); u.append(old+(target-old)*k/count)

    ramp(float(center))
    begin = t[-1]
    t.extend(begin+signal_t[1:]); u.extend(center+cell.direction*amplitude*shape[1:])
    stop_time = t[-1]
    ramp(support.stop_command_A)
    hold_begin = t[-1]
    count = int(math.ceil(support.stop_hold_s/dt-1e-12))
    t.extend(hold_begin+np.arange(1, count+1)*dt); u.extend([support.stop_command_A]*count)
    return np.asarray(t), np.asarray(u), stop_time, hold_begin


def _observations(trace, noise):
    return np.concatenate([trace[np.arange(len(trace)) % period == 0, column]/sigma
                           for (_, column, _), period, sigma in zip(CHANNELS, noise.periods, noise.sigma)])


def _feasible(trace, t, command, stop_time, hold_begin, envelope, support):
    """Sampled screening only: gradients do not bound continuous hybrid motion."""
    q, v = trace[:, 0], trace[:, 1]
    a = np.gradient(v, t)
    jerk = np.gradient(a, t)
    stop = (t >= stop_time) & (t <= stop_time+2.+1e-12)
    hold = (t >= hold_begin) & (t <= hold_begin+2.+1e-12)
    drift = float(np.max(np.abs(q[stop]-q[np.flatnonzero(stop)[0]])))
    checks = {"current": bool(np.max(np.abs(command)) <= envelope.current_a+1e-12),
        "occupancy": bool(t[-1] <= envelope.duration_s+1e-12),
        "slew": bool(np.max(np.abs(np.diff(command))/np.diff(t)) <= envelope.slew_a_s+1e-10),
        "travel": bool(q.min() >= envelope.angle_min_rad and q.max() <= envelope.angle_max_rad),
        "speed": bool(np.max(np.abs(v)) <= envelope.velocity_rad_s),
        "acceleration": bool(np.max(np.abs(a)) <= envelope.acceleration_rad_s2),
        "jerk": bool(np.max(np.abs(jerk)) <= envelope.jerk_rad_s3),
        "stop_window": bool(t[-1] >= stop_time+2. and drift <= support.max_stop_drift_rad),
        "rest": bool(np.max(np.abs(v[hold])) <= support.rest_speed_rad_s)}
    return checks, drift


def select_family_supplemental(native, model, supported_models, cells, envelope, support, noise,
                              fields, bounds, coordinate_scales, derivative_steps, prior_information,
                              failure_policy):
    """Return <=3 cases from the fixed selector, conditional on supplied facts.

    Prior/information use xi_j=theta_j/coordinate_scale_j (dimensionless).
    Derivative steps and bounds are physical parameter units. Gaussian output
    noise applies only to independent native samples, never held duplicates.
    """
    require(isinstance(model, FamilyModel), Reason.MODEL_INADEQUATE,
            "scalar selected FamilyModel required; coupled parent selection is unsupported")
    require(isinstance(support, FamilyInformationSupport), Reason.ENVELOPE_LIMITED,
            "BLOCKED_ENVELOPE: explicit qualified support packet required")
    require(isinstance(noise, FamilyInformationNoise), Reason.MEASUREMENT_LIMITED,
            "explicit native measurement/noise contract required")
    model.validate(); support.validate(envelope); noise.validate()
    require(envelope.provenance == "SYNTHETIC", Reason.ENVELOPE_LIMITED,
            "continuous physical envelope prediction unsupported; sampled hybrid/filter gradients cannot certify physical limits")
    require(isinstance(failure_policy, FailurePolicy), Reason.DATA_INVALID, "existing per-change FailurePolicy required")
    require(0 < len(fields) == len(set(fields)) and all(k in MODEL_FIELDS and k not in
            ("q_min", "q_max", "max_step", "q_origin", "stribeck_power") for k in fields),
            Reason.DATA_INVALID, "explicit admitted estimated fields required")
    require(not ("a" in fields and "actuator_gain" in fields), Reason.INSUFFICIENT_EXCITATION,
            "mechanical/input scale gauge unresolved; no unknown-gain identification promise")
    require("actuator_bias" not in fields, Reason.INSUFFICIENT_EXCITATION,
            "unknown input-bias/load/friction gauge requires a supported lumped parameterization")
    p = len(fields)
    scales, steps = array(coordinate_scales, (p,), "physical coordinate scales"), array(derivative_steps, (p,), "physical derivative steps")
    prior = array(prior_information, (p, p), "dimensionless-coordinate Fisher prior")
    require(np.all(scales > 0) and np.all(steps > 0) and np.allclose(prior, prior.T)
            and np.linalg.eigvalsh(prior).min() >= 0, Reason.DATA_INVALID, "positive scales/steps and PSD symmetric prior required")
    require(set(bounds) == set(fields), Reason.DATA_INVALID, "bounds must match selected fields")
    for k, step in zip(fields, steps):
        lo, hi = bounds[k]
        require(np.isfinite([lo, hi]).all() and lo <= getattr(model, k)-step < getattr(model, k)+step <= hi,
                Reason.DATA_INVALID, "central physical sensitivity steps must fit declared bounds: " + k)
    require(bool(supported_models) and bool(cells), Reason.DATA_INVALID, "explicit supported parameter set and cells required")
    require(model in supported_models, Reason.DATA_INVALID, "supported prediction set must include selected nominal model")
    for candidate in supported_models:
        require(isinstance(candidate, FamilyModel) and candidate.structure == model.structure,
                Reason.MODEL_INADEQUATE, "selected structure required for each supported model")
        candidate.validate()
    for cell in cells: cell.validate()
    require(len({c.cell_id for c in cells}) == len(cells), Reason.DATA_INVALID, "unique information cells required")
    require(failure_policy.information_rounds < 2, Reason.INSUFFICIENT_EXCITATION,
            "STOP_INFORMATION_BUDGET_EXHAUSTED")
    candidates, rejected, feasible_before_information = [], [], 0
    for cell in cells:
        for case_id in range(32):
            # Fixed descending factors from docs04, never an agent-selected amplitude.
            # The highest amplitude supported informative variant (smallest divisor)
            # retains one candidate per template/cell.
            for divisor in (1., 1.5, 2., 3.):
                rejected_id = {"cell_id": cell.cell_id, "case_id": case_id, "amplitude_divisor": divisor}
                plan = _commands(case_id, cell, model, envelope, support, noise.sample_hz, divisor)
                if plan is None:
                    rejected.append({**rejected_id, "reason": "COMMAND_OR_DURATION"}); continue
                t, u, stop_time, hold_begin = plan
                prehistory = -max(.1, *(m.transport_delay+m.current_delay for m in supported_models))
                tx_t, tx_A = np.r_[prehistory, t], np.r_[cell.baseline_command_A, u]
                try:
                    predictions = [native.rollout(m, t, tx_t, tx_A, cell.initial) for m in supported_models]
                    results = [_feasible(y, t, u, stop_time, hold_begin, envelope, support) for y in predictions]
                    failed = sorted({key for checks, _ in results for key, passed in checks.items() if not passed})
                    if failed:
                        rejected.append({**rejected_id, "reason": "PREDICTED_"+"_".join(failed)}); continue
                    feasible_before_information += 1
                    nominal = native.rollout(model, t, tx_t, tx_A, cell.initial)
                    columns = []
                    for field, step, scale in zip(fields, steps, scales):
                        plus = native.rollout(replace(model, **{field: getattr(model, field)+step}), t, tx_t, tx_A, cell.initial)
                        minus = native.rollout(replace(model, **{field: getattr(model, field)-step}), t, tx_t, tx_A, cell.initial)
                        columns.append((_observations(plus, noise)-_observations(minus, noise))*scale/(2*step))
                    jacobian = np.column_stack(columns)
                    information = jacobian.T@jacobian
                    if not np.isfinite(information).all() or np.linalg.norm(information) <= 1e-12:
                        rejected.append({**rejected_id, "reason": "NO_LOCAL_INFORMATION"}); continue
                except Rejected as exc:
                    rejected.append({**rejected_id, "reason": exc.reason.value}); continue
                candidates.append({"case_id": case_id, "cell_id": cell.cell_id, "direction": cell.direction,
                    "regime": cell.regime, "amplitude_divisor": divisor,
                    "selection_id": (cell.cell_id, cell.direction, case_id), "time": t,
                    "successful_tx_t": tx_t, "successful_tx": tx_A, "initial": np.asarray(cell.initial),
                    "information": information, "prediction": nominal, "stop_time_s": stop_time,
                    "stop_hold_begin_s": hold_begin, "predicted_stop_drift_rad": max(drift for _, drift in results),
                    "stop_verified": envelope.stop_verified, "checks": results[0][0],
                    "moving_duration_s": float(np.diff(t)@(np.abs(nominal[:-1, 1]) > support.rest_speed_rad_s)),
                    "predicted_directions": tuple(int(x) for x in (-1, 1)
                        if np.any(x*nominal[:, 1] > support.rest_speed_rad_s)),
                    "command_dose_A2s": float(np.diff(t)@(u[:-1]**2)),
                    "dose_role": "COMMAND_ONLY_PROXY; NOT_THERMAL_OR_PHYSICAL_SAFE_STOP_EVIDENCE"})
                break
    require(bool(candidates), Reason.INSUFFICIENT_EXCITATION if feasible_before_information else Reason.ENVELOPE_LIMITED,
            "no informative complete stimulus and predicted stop fits supplied qualified envelope; " + str(rejected[:8]))
    route = failure_policy.handle(Reason.INSUFFICIENT_EXCITATION)
    require(route == "SELECT_AT_MOST_THREE_INFORMATION_CASES", Reason.INSUFFICIENT_EXCITATION, route)
    # Preserve select_supplemental's context-first tie order, followed by case_id.
    selected, accumulated = [], prior.copy()+np.eye(p)*1e-12
    while candidates and len(selected) < 3:
        baseline = np.linalg.slogdet(accumulated)[1]
        for row in candidates:
            row["score"] = float((np.linalg.slogdet(accumulated+row["information"])[1]-baseline)/row["time"][-1])
        winner = min(candidates, key=lambda row: (-row["score"], row["selection_id"]))
        selected.append(winner); candidates.remove(winner); accumulated += winner["information"]
    data_information = sum((row["information"] for row in selected), np.zeros_like(prior))
    eigenvalues = np.linalg.eigvalsh(data_information)
    rank = int(np.sum(eigenvalues > max(float(eigenvalues[-1]), 1.)*1e-12))
    return {"selected": selected, "rejected": rejected, "information_round": failure_policy.information_rounds,
        "selector": "EXISTING_32_TEMPLATES_GREEDY_LOGDET_GAIN_PER_OCCUPANCY_TIME",
        "tie_policy": "CONTEXT_FIRST_CELL_ID_DIRECTION_CASE_ID; EXISTING_SELECT_SUPPLEMENTAL_ORDER",
        "feasibility_scope": "SAMPLED_SYNTHETIC_Q_V_AND_GRADIENT_SCREENING; NOT_CONTINUOUS_PHYSICAL_BOUND_PROOF",
        "fields": tuple(fields), "coordinate_scales": scales, "derivative_steps": steps,
        "information_units": "dimensionless xi=theta/physical_coordinate_scale; independent native Gaussian samples",
        "data_information": data_information, "local_rank": rank, "coordinate_count": p,
        "physical_identifiability": False, "calibrated_covariance": None,
        "qualification": "SUPPLIED_FAMILY_LOCAL_INFORMATION_ONLY", "deployment_authorized": False}
