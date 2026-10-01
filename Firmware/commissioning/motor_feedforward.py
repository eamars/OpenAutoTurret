"""Bounded offline yaw FF interface; current physical yaw models are unqualified.

The reference producer owns one shaped q/v/a trajectory. This module neither
integrates a second reference nor differentiates sensor/target measurements.
The default policy supports a zero-delay algebraic actuator. An explicit bounded
steady-state reference policy leaves dynamic/transport lag to the existing loop;
it performs no lag/delay inversion. Feedforward replaces one
term in the native controller; its observer, outer correction, PI, start policy,
current/slew arbitration and accepted-command accounting remain authoritative.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, replace
from enum import Enum
import math

from .model_family import FamilyModel
from .applicability import ConfigurationFacts, ConfigurationSupport, assess_configuration
from .native import CObservation, COutput, CReference, Controller
from .planned_start import PlannedStartProgram, PlannedStartLedger, PlannedStartRejected


class Failure(str, Enum):
    UNARMED = "UNARMED"
    INVALID_REFERENCE = "INVALID_REFERENCE"
    INVALID_STATE = "INVALID_STATE"
    UNSUPPORTED_MODEL = "UNSUPPORTED_MODEL"
    UNSUPPORTED_ACTUATOR = "UNSUPPORTED_ACTUATOR"
    OUTSIDE_SUPPORT = "OUTSIDE_SUPPORT"
    CORE_MISMATCH = "CORE_MISMATCH"
    CORE_FAULT = "CORE_FAULT"
    TX_FAILURE = "TX_FAILURE"


class FeedforwardRejected(ValueError):
    def __init__(self, reason, detail):
        self.reason = reason
        super().__init__(f"{reason.value}: {detail}")


def check(predicate, reason, detail):
    if not predicate:
        raise FeedforwardRejected(reason, detail)


def finite(*values):
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values)


def planned_call(function, *args, **kwargs):
    try:
        return function(*args, **kwargs)
    except PlannedStartRejected as exc:
        raise FeedforwardRejected(Failure(exc.reason), str(exc)) from exc


@dataclass(frozen=True, kw_only=True)
class ReferencePacket:
    q_ref_rad: float
    v_ref_rad_s: float
    a_ref_rad_s2: float
    time_s: float  # evaluation time of all three shaped components, on the control clock
    source_time_s: float  # causal upstream information used by the reference producer
    expires_at_s: float
    frame: str
    configuration_id: str
    trajectory_id: str  # all components must come from this same trajectory
    generation: int
    fresh: bool
    valid: bool
    trajectory_phase: str = "LEGACY"  # explicit synthetic declaration; legacy remains default
    planned_direction: int = 0
    departure_offset_s: float | None = None  # relative to THIS packet's source_time_s
    departure_position_rad: float | None = None


@dataclass(frozen=True, kw_only=True)
class CausalState:
    q_rad: float
    v_rad_s: float
    posture_rad: float
    winding_rad: float
    temperature: float
    time_s: float
    frame: str
    configuration_id: str
    generation: int
    friction_state: str  # strict STICKING/SLIDING, or opt-in synthetic POSTERIOR_POLICY
    static_balance_A: float | None  # explicit bounded static/hold policy; never inferred uniquely from v=0
    fresh: bool
    valid: bool
    provenance: str
    configuration_facts: ConfigurationFacts | None = None


@dataclass(frozen=True, kw_only=True)
class BoundedStartPolicy:
    """Synthetic candidate, distinct from the identified static threshold.

    Excess is above the interval's upper end in A-effective friction units.
    The dose bound covers at most max_attempts native START entries until logical inhibit;
    it does not certify physical current cutoff after an aborted send path.
    """
    configuration_id: str
    source: str
    q_min_rad: float
    q_max_rad: float
    static_negative_interval_A: tuple[float, float]
    static_positive_interval_A: tuple[float, float]
    negative_excess_A: float
    positive_excess_A: float
    max_attempt_s: float
    max_command_dose_A2s: float
    qualification: str = "SYNTHETIC_START_CANDIDATE"
    max_attempts: int = 1

    def validate(self, model, parameters):
        check(self.qualification == "SYNTHETIC_START_CANDIDATE" and self.configuration_id
              and isinstance(self.source, str) and self.source, Failure.UNSUPPORTED_MODEL,
              "explicit synthetic START candidate provenance required")
        check(type(self.max_attempts) is int and self.max_attempts > 0,
              Failure.OUTSIDE_SUPPORT, "explicit finite positive START attempt count required")
        check(finite(self.q_min_rad, self.q_max_rad, self.negative_excess_A, self.positive_excess_A,
                     self.max_attempt_s, self.max_command_dose_A2s)
              and self.q_min_rad < self.q_max_rad and min(self.negative_excess_A, self.positive_excess_A) > 0
              and .060 <= parameters.sustained_s < self.max_attempt_s <= .200,
              Failure.OUTSIDE_SUPPORT, "START needs positive interval excess, 60ms evidence and <=200ms attempt")
        for interval, point in ((self.static_negative_interval_A, model.static_negative),
                                (self.static_positive_interval_A, model.static_positive)):
            check(type(interval) is tuple and len(interval) == 2 and finite(*interval)
                  and 0 <= interval[0] <= point < interval[1], Failure.OUTSIDE_SUPPORT,
                  "static interval must retain its unresolved upper endpoint")
        for q in (self.q_min_rad, self.q_max_rad):
            load = model.load_offset + (model.load_slope*(q-model.q_origin) if model.load == "affine" else 0.)
            for sign, upper, excess in ((-1, self.static_negative_interval_A[1], self.negative_excess_A),
                                        (1, self.static_positive_interval_A[1], self.positive_excess_A)):
                total = (load+sign*(upper+excess)-model.actuator_bias)/model.actuator_gain
                check(abs(total) <= parameters.current_cap, Failure.OUTSIDE_SUPPORT,
                      "START interval policy exceeds existing command cap")
        duration = min(parameters.start_timeout_s, self.max_attempt_s)+parameters.dt_max
        check(self.max_command_dose_A2s >= self.max_attempts*parameters.current_cap**2*duration,
              Failure.OUTSIDE_SUPPORT, "total dose bound must cover each allowed attempt through its abort-detection tick")
        return self


@dataclass(frozen=True, kw_only=True)
class FeedforwardSupport:
    configuration_id: str
    frame: str
    q_min_rad: float
    q_max_rad: float
    velocity_max_rad_s: float
    acceleration_max_rad_s2: float
    fixed_posture_rad: float
    winding_min_rad: float
    winding_max_rad: float
    temperature_min: float
    temperature_max: float
    actuator_gain_min: float
    actuator_gain_max: float
    command_cap_A: float
    command_slew_A_s: float
    max_reference_source_age_s: float
    max_state_age_s: float
    max_ack_delay_s: float
    rest_speed_rad_s: float
    state_encoder_consistency_rad: float
    state_gyro_consistency_rad_s: float
    qualification: str
    configuration_support: ConfigurationSupport | None = None
    actuator_policy: str = "ZERO_DELAY_ALGEBRAIC"
    actuation_memory_max_s: float = 0.  # bound on tau+delay, not finite-time settling
    start_policy: BoundedStartPolicy | None = None
    planned_start_program: PlannedStartProgram | None = None


@dataclass(frozen=True)
class FeedforwardDemand:
    effective_current_A: float
    command_current_A: float
    load_A: float
    friction_A: float
    phase: str
    direction: int


def validate_model(model, *, actuator_policy="ZERO_DELAY_ALGEBRAIC", actuation_memory_max_s=0.):
    check(isinstance(model, FamilyModel), Failure.UNSUPPORTED_MODEL, "explicit FamilyModel required")
    try:
        model.validate()
    except (ValueError, TypeError) as exc:
        raise FeedforwardRejected(Failure.UNSUPPORTED_MODEL, str(exc)) from exc
    check(actuator_policy in ("ZERO_DELAY_ALGEBRAIC", "STEADY_STATE_REFERENCE"),
          Failure.UNSUPPORTED_ACTUATOR, "explicit supported causal actuator policy required")
    if actuator_policy == "ZERO_DELAY_ALGEBRAIC":
        check(model.actuator == "algebraic" and model.actuator_tau == 0 and model.transport_delay == 0
              and actuation_memory_max_s == 0, Failure.UNSUPPORTED_ACTUATOR,
              "dynamic actuator or delay requires a separately validated bounded reference policy")
    else:
        check(finite(actuation_memory_max_s) and actuation_memory_max_s > 0
              and model.actuator_tau+model.transport_delay <= actuation_memory_max_s
              and (model.actuator != "algebraic" or model.actuator_tau == 0),
              Failure.UNSUPPORTED_ACTUATOR, "declared characteristic lag-plus-transport time-scale bound exceeded")


def parameters_for_family(template, model, *, actuator_policy="ZERO_DELAY_ALGEBRAIC", actuation_memory_max_s=0.,
                          start_policy=None):
    """Map model units into the existing core's coefficients/start totals.

    Gains, observer, censor flags and all electrical/start limits are supplied in
    template and unchanged. This utility supplies no physical model or gains.
    """
    validate_model(model, actuator_policy=actuator_policy, actuation_memory_max_s=actuation_memory_max_s)
    p = copy.deepcopy(template)
    check(p.model.periodic == 0, Failure.CORE_MISMATCH, "periodic load mapping is outside this fixed-configuration interface")
    n = p.model.n
    check(n in (5, 8), Failure.CORE_MISMATCH, "unsupported core coefficient table")
    if start_policy is not None:
        check(isinstance(start_policy, BoundedStartPolicy), Failure.OUTSIDE_SUPPORT, "typed bounded START policy required")
        start_policy.validate(model, p)
        p.start_timeout_s = min(p.start_timeout_s, start_policy.max_attempt_s)
    gain, bias = model.actuator_gain, model.actuator_bias
    for k in range(3):
        p.model.theta[k] = model.a / gain
        p.model.theta[3+k] = model.viscous / gain
    for direction_index, sign in enumerate((-1, 1)):
        fc = model.coulomb_negative if sign < 0 else model.coulomb_positive
        fs = model.static_negative if sign < 0 else model.static_positive
        if start_policy is not None:
            fs = (start_policy.static_negative_interval_A[1]+start_policy.negative_excess_A if sign < 0 else
                  start_policy.static_positive_interval_A[1]+start_policy.positive_excess_A)
        for posture_index in range(3):
            for q_index in range(n):
                at = direction_index*3*n + posture_index*n + q_index
                load = model.load_offset + (model.load_slope*(p.model.q[q_index]-model.q_origin) if model.load == "affine" else 0.)
                p.model.theta[6+at] = (load+sign*fc-bias)/gain
                p.start_total[at] = (load+sign*fs-bias)/gain
    p.model.theta[6+6*n] = model.transport_delay
    p.start_policy_qualification = "SYNTHETIC_START_CANDIDATE" if start_policy is not None else "STATIC_POINT_DIAGNOSTIC_UNQUALIFIED"
    return p


class MotorFeedforward:
    """Sole offline adapter around an existing native Controller, initially inhibited.

    Physical model promotion, stop-chain certification, production wiring,
    model-switch bumpless transfer are unsupported. Dynamic/delayed actuation
    requires explicit STEADY_STATE_REFERENCE support and separate nonlinear gates.
    SYNTHETIC_OFFLINE support must not be relabelled as physical qualification.
    """
    def __init__(self, controller, model, support):
        check(isinstance(controller, Controller), Failure.CORE_MISMATCH, "existing native Controller required")
        self.controller, self.model, self.support = controller, model, support
        self.fault = None
        self.armed = False
        self.pending = None
        self.last_reference_time = None
        self.last_posterior = None
        self.start_attempt_count = 0
        self._last_native_motion = None
        self._departure_key = None
        self._last_trajectory_phase = None
        self._program_ledger = None
        self._program_clock = None
        controller.inhibit()
        check(isinstance(support, FeedforwardSupport), Failure.OUTSIDE_SUPPORT, "explicit FeedforwardSupport required")
        self._model_contract()
        if support.planned_start_program is not None:
            self._program_ledger = PlannedStartLedger(support.planned_start_program, self._core_parameters, support)

    def _model_contract(self):
        s, p = self.support, self.controller.read_parameters()
        validate_model(self.model, actuator_policy=s.actuator_policy,
                       actuation_memory_max_s=s.actuation_memory_max_s)
        numeric = [value for name, value in vars(s).items()
                   if name not in ("configuration_id", "frame", "qualification", "configuration_support", "actuator_policy", "start_policy", "planned_start_program")]
        check(finite(*numeric) and s.q_min_rad < s.q_max_rad and s.winding_min_rad <= s.winding_max_rad
              and s.temperature_min <= s.temperature_max and 0 < s.actuator_gain_min <= self.model.actuator_gain <= s.actuator_gain_max
              and min(s.velocity_max_rad_s, s.acceleration_max_rad_s2, s.command_cap_A, s.command_slew_A_s,
                      s.max_reference_source_age_s, s.max_state_age_s, s.max_ack_delay_s, s.rest_speed_rad_s) > 0,
              Failure.OUTSIDE_SUPPORT, "explicit finite supported envelope required")
        check(s.state_encoder_consistency_rad >= 0 and s.state_gyro_consistency_rad_s >= 0,
              Failure.OUTSIDE_SUPPORT, "supplied state/sensor consistency bounds required")
        check(s.configuration_id and s.frame and s.qualification == "SYNTHETIC_OFFLINE",
              Failure.UNSUPPORTED_MODEL, "no physical yaw model/stop chain is qualified by this adapter")
        check(isinstance(s.configuration_support, ConfigurationSupport)
              and s.configuration_support.baseline.context_id == s.configuration_id
              and s.configuration_support.qualification == "SYNTHETIC_OFFLINE",
              Failure.OUTSIDE_SUPPORT, "structured synthetic configuration support required")
        check(p.acceleration_cap == 0 and p.current_cap <= s.command_cap_A and p.slew <= s.command_slew_A_s
              and p.velocity_cap <= s.velocity_max_rad_s and p.rest_speed == s.rest_speed_rad_s,
              Failure.CORE_MISMATCH, "one pre-shaped reference and compatible supplied core limits required")
        expected = parameters_for_family(p, self.model, actuator_policy=s.actuator_policy,
                                         actuation_memory_max_s=s.actuation_memory_max_s, start_policy=s.start_policy)
        if s.start_policy is not None:
            check(s.start_policy.configuration_id == s.configuration_id
                  and s.start_policy.q_min_rad <= s.q_min_rad <= s.q_max_rad <= s.start_policy.q_max_rad
                  and p.start_timeout_s <= s.start_policy.max_attempt_s,
                  Failure.CORE_MISMATCH, "START policy context/envelope or stricter native timeout mismatch")
        check(all(math.isclose(p.model.theta[k], expected.model.theta[k], rel_tol=1e-12, abs_tol=1e-12)
                  for k in range(7+6*p.model.n))
              and all(math.isclose(p.start_total[k], expected.start_total[k], rel_tol=1e-12, abs_tol=1e-12)
                      for k in range(6*p.model.n)), Failure.CORE_MISMATCH,
              "core coefficients/start totals must use this model's host command map")
        check(p.model.q[0] <= s.q_min_rad and s.q_max_rad <= p.model.q[p.model.n-1]
              and p.model.z[0] <= s.fixed_posture_rad <= p.model.z[2], Failure.CORE_MISMATCH,
              "core evaluation domain must cover the separately supplied supported envelope")
        self._core_parameters = p
        if s.planned_start_program is not None:
            check(isinstance(s.planned_start_program, PlannedStartProgram), Failure.INVALID_REFERENCE,
                  "typed immutable planned START program required")
            planned_call(s.planned_start_program.validate, s, p)
        if self._program_ledger is not None:
            check(self._program_ledger.program == s.planned_start_program, Failure.INVALID_REFERENCE,
                  "bound planned START program cannot be replaced or translated again")
            check((self._program_ledger.current_cap_A, self._program_ledger.dt_max_s,
                   self._program_ledger.ack_max_s, self._program_ledger.rest_speed_rad_s,
                   self._program_ledger.ceiling_A2s) ==
                  (p.current_cap, p.dt_max, s.max_ack_delay_s, s.rest_speed_rad_s,
                   s.start_policy.max_command_dose_A2s), Failure.CORE_MISMATCH,
                  "bound planned program command-dose/timing limits cannot change")
        self.start_policy_qualification = ("SYNTHETIC_START_CANDIDATE" if s.start_policy else
                                           "STATIC_POINT_DIAGNOSTIC_UNQUALIFIED")

    def _state(self, state, now):
        check(isinstance(state, CausalState), Failure.INVALID_STATE, "explicit CausalState packet required")
        s = self.support
        check(isinstance(state.configuration_facts, ConfigurationFacts)
              and state.configuration_facts.context_id == state.configuration_id,
              Failure.INVALID_STATE, "current structured configuration facts required")
        configuration = assess_configuration(s.configuration_support, state.configuration_facts,
                                             purpose="SYNTHETIC_CONTROL")
        check(configuration["supported"], Failure.OUTSIDE_SUPPORT,
              "configuration facts changed or lack support: " + configuration["action"])
        check(state.valid is True and state.fresh is True and state.provenance == "SYNTHETIC"
              and finite(state.q_rad, state.v_rad_s, state.posture_rad, state.winding_rad, state.temperature, state.time_s, now)
              and 0 <= now-state.time_s <= s.max_state_age_s
              and isinstance(state.generation, int) and not isinstance(state.generation, bool) and state.generation > 0,
              Failure.INVALID_STATE, "fresh finite causal simulated state required")
        check(state.frame == s.frame and state.configuration_id == s.configuration_id
              and s.q_min_rad <= state.q_rad <= s.q_max_rad and abs(state.v_rad_s) <= s.velocity_max_rad_s
              and state.posture_rad == s.fixed_posture_rad and s.winding_min_rad <= state.winding_rad <= s.winding_max_rad
              and s.temperature_min <= state.temperature <= s.temperature_max,
              Failure.OUTSIDE_SUPPORT, "state/configuration outside independently declared support")
        sticking = state.friction_state == "STICKING"
        sliding = state.friction_state == "SLIDING"
        posterior_policy = state.friction_state == "POSTERIOR_POLICY"
        unresolved = state.friction_state == "LOW_SPEED_UNRESOLVED"
        check((sticking and abs(state.v_rad_s) <= s.rest_speed_rad_s and finite(state.static_balance_A)
               and -self.model.static_negative <= state.static_balance_A <= self.model.static_positive)
              or (sliding and abs(state.v_rad_s) > s.rest_speed_rad_s and state.static_balance_A is None)
              or ((posterior_policy or unresolved and abs(state.v_rad_s) <= s.rest_speed_rad_s)
                  and finite(state.static_balance_A)
                  and -self.model.static_negative <= state.static_balance_A <= self.model.static_positive),
              Failure.INVALID_STATE, "friction state/static balance inconsistent with causal motion")

    def _reference(self, reference, state, now):
        check(isinstance(reference, ReferencePacket), Failure.INVALID_REFERENCE, "explicit ReferencePacket required")
        s = self.support
        check(reference.valid is True and reference.fresh is True
              and finite(reference.q_ref_rad, reference.v_ref_rad_s, reference.a_ref_rad_s2,
                         reference.time_s, reference.source_time_s, reference.expires_at_s)
              and math.isclose(reference.time_s, now, abs_tol=1e-12, rel_tol=0)
              and 0 <= now-reference.source_time_s <= s.max_reference_source_age_s
              and now <= reference.expires_at_s and isinstance(reference.trajectory_id, str) and bool(reference.trajectory_id)
              and isinstance(reference.generation, int) and not isinstance(reference.generation, bool)
              and (self.last_reference_time is None or reference.time_s > self.last_reference_time),
              Failure.INVALID_REFERENCE, "q/v/a must share the current evaluation time and fresh causal source")
        check(reference.frame == s.frame and reference.configuration_id == s.configuration_id
              and reference.generation == state.generation and s.q_min_rad <= reference.q_ref_rad <= s.q_max_rad
              and abs(reference.v_ref_rad_s) <= s.velocity_max_rad_s
              and abs(reference.a_ref_rad_s2) <= s.acceleration_max_rad_s2,
              Failure.OUTSIDE_SUPPORT, "reference frame/configuration/generation/envelope mismatch")
        phase = reference.trajectory_phase
        check(phase in ("LEGACY", "HOLD", "DEPARTURE", "TRACKING", "BRAKING", "REVERSAL", "POSITION_CORRECTION"),
              Failure.INVALID_REFERENCE, "explicit supported synthetic trajectory phase required")
        if self._program_ledger is not None:
            planned_call(self._program_ledger.check_reference, reference)
        if phase != "DEPARTURE":
            check(type(reference.planned_direction) is int and reference.planned_direction == 0
                  and reference.departure_offset_s is None and reference.departure_position_rad is None,
                  Failure.INVALID_REFERENCE, "departure metadata is exclusive to a shaped DEPARTURE")
            if phase in ("HOLD", "POSITION_CORRECTION"):
                check(reference.v_ref_rad_s == 0 and reference.a_ref_rad_s2 == 0, Failure.INVALID_REFERENCE,
                      "stationary reference phase requires zero shaped v/a; native position correction remains active")
            if phase == "BRAKING":
                check(reference.v_ref_rad_s*reference.a_ref_rad_s2 < 0, Failure.INVALID_REFERENCE,
                      "BRAKING requires nonzero shaped deceleration; stationary references use HOLD")
            return 0
        check(s.start_policy is not None and type(reference.planned_direction) is int
              and reference.planned_direction in (-1, 1)
              and finite(reference.departure_offset_s, reference.departure_position_rad)
              and reference.departure_offset_s >= 0
              and s.q_min_rad <= reference.departure_position_rad <= s.q_max_rad,
              Failure.INVALID_REFERENCE, "DEPARTURE requires a bounded synthetic START policy and signed source-relative anchor")
        direction = reference.planned_direction
        anchor = reference.source_time_s+reference.departure_offset_s
        elapsed = now-anchor
        key = (reference.trajectory_id, reference.generation, reference.source_time_s,
               reference.departure_offset_s, reference.departure_position_rad, direction)
        continuing = self._last_trajectory_phase == "DEPARTURE" and self._departure_key == key
        check(self._last_trajectory_phase != "DEPARTURE" or continuing, Failure.INVALID_REFERENCE,
              "an active DEPARTURE must retain its source, original anchor and direction")
        check(continuing or key != self._departure_key, Failure.INVALID_REFERENCE,
              "a closed DEPARTURE cannot be replayed as a new START declaration")
        check(elapsed >= -1e-12 and (continuing or elapsed <= self._core_parameters.dt_max+1e-12),
              Failure.INVALID_REFERENCE, "first DEPARTURE must be evaluated within one supported tick of its original anchor")
        check(direction*(reference.q_ref_rad-reference.departure_position_rad) >= -1e-12
              and direction*reference.v_ref_rad_s >= 0 and direction*reference.a_ref_rad_s2 >= 0,
              Failure.INVALID_REFERENCE, "DEPARTURE q/v/a must coherently accelerate from its declared stationary anchor")
        if not continuing:
            age = max(0., elapsed)
            check(abs(reference.q_ref_rad-reference.departure_position_rad) <= .5*s.acceleration_max_rad_s2*age**2+1e-12
                  and abs(reference.v_ref_rad_s) <= s.acceleration_max_rad_s2*age+1e-12,
                  Failure.INVALID_REFERENCE, "initial DEPARTURE cannot relabel a position/speed step or stationary noise")
        return direction

    def _observation_consistency(self, observation, state):
        s = self.support
        # These supplied bounds must include the permitted estimation error,
        # sampling age and sensor filtering. They are not inferred from a fit.
        if observation.encoder_valid:
            check(finite(observation.position) and abs(observation.position-state.q_rad) <= s.state_encoder_consistency_rad,
                  Failure.INVALID_STATE, "causal position contradicts the supplied encoder-consistency bound")
        if observation.gyro_valid:
            check(finite(observation.gyro_rate) and abs(observation.gyro_rate-state.v_rad_s) <= s.state_gyro_consistency_rad_s,
                  Failure.INVALID_STATE, "causal friction motion contradicts the supplied gyro-consistency bound")

    def _posterior_state(self, posterior, state, observation):
        """Kinematics come only from this cycle's single native observer."""
        p, s = self._core_parameters, self.support
        check(finite(posterior.now, posterior.dt, posterior.position, posterior.velocity,
                     posterior.encoder_time, posterior.gyro_time)
              and posterior.now == observation.now and posterior.generation == state.generation
              and p.dt_min <= posterior.dt <= p.dt_max
              and 0 <= posterior.now-posterior.encoder_time <= min(p.observer.max_encoder_age_s, s.max_state_age_s)
              and (posterior.encoder_only and p.observer.encoder_only_verified
                   or 0 <= posterior.now-posterior.gyro_time <= min(p.observer.max_gyro_age_s, s.max_state_age_s)),
              Failure.INVALID_STATE, "current native posterior/source clocks outside supported freshness")
        result = replace(state, q_rad=posterior.position, v_rad_s=posterior.velocity, time_s=posterior.now)
        if state.friction_state == "POSTERIOR_POLICY":
            moving = abs(posterior.velocity) > s.rest_speed_rad_s
            result = replace(result, friction_state="SLIDING" if moving else "LOW_SPEED_UNRESOLVED",
                             static_balance_A=None if moving else state.static_balance_A)
        self._state(result, posterior.now)
        return result

    def _demand(self, reference, state):
        m, s = self.model, self.support
        wanted = reference.planned_direction if reference.trajectory_phase == "DEPARTURE" else (
            1 if reference.v_ref_rad_s > self._core_parameters.intent_threshold else (
            -1 if reference.v_ref_rad_s < -self._core_parameters.intent_threshold else 0)
        )
        unresolved = state.friction_state == "LOW_SPEED_UNRESOLVED"
        if state.friction_state in ("STICKING", "LOW_SPEED_UNRESOLVED") and not wanted:
            friction, direction, phase = state.static_balance_A, 0, "REST_UNRESOLVED" if unresolved else "REST"
        else:
            # At unresolved low speed retain a nonzero posterior's causal sign;
            # a reference reversal alone cannot flip its moving friction term.
            causal_direction = state.friction_state == "SLIDING" or unresolved and state.v_rad_s != 0
            direction = (1 if state.v_rad_s > 0 else -1) if causal_direction else wanted
            phase = ("START_UNRESOLVED" if unresolved and not causal_direction else
                     "REVERSE_UNRESOLVED" if unresolved and wanted != direction else
                     "SLIDE_UNRESOLVED" if unresolved else "START" if state.friction_state == "STICKING" else
                     "STOP" if not wanted else "REVERSE" if wanted != direction else "SLIDE")
            speed = abs(state.v_rad_s) if phase in ("STOP", "REVERSE", "REVERSE_UNRESOLVED") else abs(reference.v_ref_rad_s)
            fc = m.coulomb_positive if direction > 0 else m.coulomb_negative
            fs = m.static_positive if direction > 0 else m.static_negative
            magnitude = fc
            if m.friction == "stribeck":
                vs = m.stribeck_positive if direction > 0 else m.stribeck_negative
                magnitude += (fs-fc)*math.exp(-(speed/vs)**m.stribeck_power)
            friction = direction*magnitude
        load = m.load_offset + (m.load_slope*(state.q_rad-m.q_origin) if m.load == "affine" else 0.)
        effective = m.a*reference.a_ref_rad_s2 + load + m.viscous*reference.v_ref_rad_s + friction
        command = (effective-m.actuator_bias)/m.actuator_gain
        check(finite(effective, command) and abs(command) <= s.command_cap_A, Failure.OUTSIDE_SUPPORT,
              "FF demand outside supported algebraic command envelope")
        return FeedforwardDemand(effective, command, load, friction, phase, direction)

    def _inhibit(self, fault):
        if self._program_ledger is not None:
            self._program_ledger.abort(self._program_clock)
        self.fault = self.fault or fault
        self.armed = False
        self.pending = None
        self.last_posterior = None
        self.controller.inhibit()

    def reset(self, state, *, now, previous_current_A, accepted_time_s):
        """Explicit offline reinitialization; caller must supply a fresh stationary state."""
        try:
            if self._program_ledger is not None:
                self._program_clock = now
            self._model_contract()
            self._state(state, now)
            check(state.friction_state in ("STICKING", "POSTERIOR_POLICY")
                  and abs(state.v_rad_s) <= self.support.rest_speed_rad_s and state.time_s == now
                  and finite(previous_current_A, accepted_time_s) and accepted_time_s <= now,
                      Failure.INVALID_STATE, "stationary reset/current prehistory required")
            if self._program_ledger is not None:
                planned_call(self._program_ledger.initialize, now, state, previous_current_A, accepted_time_s)
            self.controller.reset(now, state.q_rad, state.v_rad_s, previous_current_A,
                                  generation=state.generation, accepted_time=accepted_time_s)
        except (ValueError, TypeError, ArithmeticError) as exc:
            fault = exc if isinstance(exc, FeedforwardRejected) else FeedforwardRejected(Failure.CORE_FAULT, str(exc))
            self._inhibit(fault)
            if fault is exc:
                raise
            raise fault from exc
        self.fault = None
        self.armed = True
        self.pending = None
        self.last_reference_time = None
        self.last_posterior = None
        self.start_attempt_count = 0
        self._last_native_motion = None
        self._departure_key = None
        self._last_trajectory_phase = None

    def step(self, observation, reference, state):
        if not self.armed:
            raise self.fault or FeedforwardRejected(Failure.UNARMED, "explicit reset is required")
        try:
            if self._program_ledger is not None and isinstance(observation, CObservation):
                self._program_clock = observation.now
            self._model_contract()
            check(isinstance(observation, CObservation), Failure.INVALID_STATE, "native observation packet required")
            if self._program_ledger is not None:
                planned_call(self._program_ledger.advance, observation.now)
            self._state(state, observation.now)
            planned_start_intent = self._reference(reference, state, observation.now)
            check(observation.generation == state.generation, Failure.INVALID_STATE, "observation generation mismatch")
            self._observation_consistency(observation, state)
            demands, posteriors = [], []

            def compute(posterior, native_reference):
                check((native_reference.position, native_reference.velocity, native_reference.acceleration,
                       native_reference.posture) == (reference.q_ref_rad, reference.v_ref_rad_s,
                       reference.a_ref_rad_s2, state.posture_rad), Failure.INVALID_REFERENCE,
                      "native FF must use the same authoritative reference packet")
                causal = self._posterior_state(posterior, state, observation)
                if self._program_ledger is not None:
                    if planned_call(self._program_ledger.before_callback, reference, posterior, self._last_native_motion):
                        self.start_attempt_count += 1
                elif self.support.start_policy is not None and posterior.motion == 1 and self._last_native_motion != 1:
                    check(self.start_attempt_count < self.support.start_policy.max_attempts,
                          Failure.OUTSIDE_SUPPORT, "bounded native START attempt count exhausted; explicit reset required")
                    self.start_attempt_count += 1
                demand = self._demand(reference, causal)
                demands.append(demand)
                posteriors.append(posterior)
                return demand.command_current_A

            out = self.controller.step_posterior_feedforward(observation,
                CReference(reference.q_ref_rad, reference.v_ref_rad_s, reference.a_ref_rad_s2, state.posture_rad),
                compute, planned_start_intent=planned_start_intent,
                reference_phase=1 if reference.trajectory_phase == "DEPARTURE" else
                                2 if reference.trajectory_phase == "BRAKING" else 0)
            check(out.status == 0, Failure.CORE_FAULT, f"native status={out.status}")
            check(len(demands) == 1 and out.position == posteriors[0].position
                  and out.velocity == posteriors[0].velocity, Failure.CORE_MISMATCH,
                  "exactly one FF evaluation must share the output posterior")
            demand = demands[0]
        except (ValueError, TypeError, ArithmeticError) as exc:
            fault = exc if isinstance(exc, FeedforwardRejected) else FeedforwardRejected(Failure.CORE_FAULT, str(exc))
            self._inhibit(fault)
            if fault is exc:
                raise
            raise fault from exc
        self.last_reference_time = reference.time_s
        self._last_trajectory_phase = reference.trajectory_phase
        if reference.trajectory_phase == "DEPARTURE":
            self._departure_key = (reference.trajectory_id, reference.generation, reference.source_time_s,
                                  reference.departure_offset_s, reference.departure_position_rad,
                                  reference.planned_direction)
        self.last_posterior = posteriors[0]
        self._last_native_motion = out.motion
        self.pending = (out.sequence, observation.now)
        if self._program_ledger is not None:
            self._program_ledger.output(out)
        return out, demand

    def acknowledge(self, output, *, successful, accepted_time_s, applied_current_A=None):
        if not self.armed or self.pending is None:
            return False
        try:
            check(isinstance(output, COutput), Failure.TX_FAILURE, "native command output required")
            if self._program_ledger is not None and finite(accepted_time_s) and accepted_time_s >= self.pending[1]:
                self._program_clock = accepted_time_s
            if output.sequence != self.pending[0]:
                if self._program_ledger is not None:
                    raise FeedforwardRejected(Failure.TX_FAILURE, "planned program ACK token mismatch")
                return False
            applied = output.limited if applied_current_A is None else applied_current_A
            check(isinstance(successful, bool) and finite(applied)
                  and abs(applied) <= self._core_parameters.current_cap,
                  Failure.TX_FAILURE, "explicit TX result and finite in-envelope applied command required")
            check(finite(accepted_time_s) and self.pending[1] <= accepted_time_s <= self.pending[1]+self.support.max_ack_delay_s,
                  Failure.TX_FAILURE, "accepted-command callback missed its supported deadline")
            if self._program_ledger is not None:
                self._program_clock = accepted_time_s
                planned_call(self._program_ledger.advance, accepted_time_s)
            accepted = self.controller.ack(output, successful=successful, applied=applied,
                                           accepted_time=accepted_time_s)
            check(accepted, Failure.TX_FAILURE, "failed/rejected TX cannot become applied current")
            if self._program_ledger is not None:
                self._program_ledger.receipt(output, accepted_time_s, applied)
        except (ValueError, TypeError, ArithmeticError) as exc:
            fault = exc if isinstance(exc, FeedforwardRejected) else FeedforwardRejected(Failure.CORE_FAULT, str(exc))
            self._inhibit(fault)
            if fault is exc:
                raise
            raise fault from exc
        self.pending = None
        return True

    def start_program_report(self):
        return self._program_ledger.report() if self._program_ledger is not None else None

    def account_start_dose_through(self, now):
        """Charge a known partial forecast tail without an observation or TX."""
        if self._program_ledger is None:
            return None
        if not self.armed:
            raise self.fault or FeedforwardRejected(Failure.UNARMED, "planned program is inhibited")
        try:
            self._program_clock = now
            last_control = self.last_reference_time if self.last_reference_time is not None else self._program_ledger.through_s
            check(finite(now) and now <= self.support.planned_start_program.expires_at_s
                  and now <= last_control+self._core_parameters.dt_max+1e-12 and self.pending is None,
                  Failure.INVALID_STATE, "known planned tail must remain in the next supported interval without pending TX")
            planned_call(self._program_ledger.advance, now)
        except (ValueError, TypeError, ArithmeticError) as exc:
            fault = exc if isinstance(exc, FeedforwardRejected) else FeedforwardRejected(Failure.CORE_FAULT, str(exc))
            self._inhibit(fault)
            if fault is exc:
                raise
            raise fault from exc
        return self.start_program_report()

    def inhibit_at(self, now, detail):
        """Censor an upstream offline forecast rejection at its logical clock."""
        check(finite(now), Failure.INVALID_STATE, "finite logical inhibit clock required")
        if self._program_ledger is not None:
            self._program_clock = now
        self._inhibit(FeedforwardRejected(Failure.INVALID_REFERENCE, detail))
