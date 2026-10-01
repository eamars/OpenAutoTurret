"""Offline local sampled analysis; sliding and inactive limits are prerequisites."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.linalg import expm
from scipy.optimize import brentq

from .contracts import Reason, require
from .family_analysis import linearize_sliding_family


@dataclass(frozen=True)
class LocalGains:
    kp: float
    ki: float
    kpos: float
    kaw: float

    def validate(self):
        require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in (self.kp, self.ki, self.kpos, self.kaw)) and
                min(self.kp, self.kpos, self.kaw) > 0 and self.ki >= 0,
                Reason.DATA_INVALID, "finite positive local gains; Ki may be zero")


OBSERVER_FIELDS = ("encoder_variance", "gyro_variance", "process_variance", "max_encoder_age_s",
                  "max_gyro_age_s", "initial_position_variance", "initial_velocity_variance")


@dataclass(frozen=True)
class LocalObserver:
    encoder_variance: float
    gyro_variance: float
    process_variance: float
    max_encoder_age_s: float
    max_gyro_age_s: float
    initial_position_variance: float
    initial_velocity_variance: float


def observer_snapshot(observer):
    values = tuple(getattr(observer, k, None) for k in OBSERVER_FIELDS)
    require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and x > 0
                for x in values), Reason.DATA_INVALID, "explicit positive native observer quantities required")
    return LocalObserver(*values)


def immutable(matrix):
    return np.frombuffer(matrix.tobytes(), dtype=matrix.dtype).reshape(matrix.shape)


@dataclass(frozen=True, kw_only=True)
class SampleSchedule:
    """Periodic packet timing with explicit source/availability distinction.

    Encoder age is its physical source age. Gyro availability age locates the
    reported sensor value; the model's pure gyro delay locates that value's
    physical source. Their sum enters native timestamp/covariance/freshness.
    Sensor filter states remain dynamics, not an additional clock shift.
    """
    dt_s: float
    encoder_period: int
    gyro_period: int
    encoder_age_s: float
    gyro_availability_age_s: float
    encoder_quantum_rad: float
    gyro_quantum_rad_s: float
    immediate_successful_ack: bool
    limits_inactive: bool

    def gyro_source_age_s(self, gyro_delay_s):
        require(isinstance(gyro_delay_s, (int, float)) and not isinstance(gyro_delay_s, bool) and
                math.isfinite(gyro_delay_s) and gyro_delay_s >= 0.,
                Reason.DATA_INVALID, "finite nonnegative gyro signal delay required")
        return self.gyro_availability_age_s+gyro_delay_s

    def validate(self, observer, *, gyro_delay_s):
        require(type(self.encoder_period) is type(self.gyro_period) is int and
                min(self.encoder_period, self.gyro_period) >= 1 and
                math.lcm(self.encoder_period, self.gyro_period) <= 64,
                Reason.DATA_INVALID, "positive bounded periodic sample cadence required")
        values = (self.dt_s, self.encoder_age_s, self.gyro_availability_age_s,
                  self.encoder_quantum_rad, self.gyro_quantum_rad_s)
        require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in values) and self.dt_s > 0 and min(values[1:]) >= 0,
                Reason.DATA_INVALID, "finite sample times/ages/quantization required")
        require(self.encoder_quantum_rad == self.gyro_quantum_rad_s == 0.,
                Reason.MEASUREMENT_LIMITED, "quantization has no smooth local derivative; nonlinear quantized forecast required")
        require(self.immediate_successful_ack is True and self.limits_inactive is True,
                Reason.INTEGRATION_MISMATCH, "local tangent requires successful same-tick ACK and inactive limiter/AW/guards")
        require(self.encoder_age_s+(self.encoder_period-1)*self.dt_s <= observer.max_encoder_age_s and
                self.gyro_source_age_s(gyro_delay_s)+(self.gyro_period-1)*self.dt_s <= observer.max_gyro_age_s,
                Reason.MEASUREMENT_LIMITED, "schedule exceeds native freshness bounds")
        return self


def periodic_observer(observer, schedule, *, gyro_source_age_s):
    """Converged scheduled covariance, distinct from native reset covariance."""
    P = np.diag([observer.initial_position_variance, observer.initial_velocity_variance])
    period = math.lcm(schedule.encoder_period, schedule.gyro_period)
    for _ in range(2000):
        before, phases = P.copy(), []
        for j in range(period):
            F, K, P = observer_tick(observer, schedule.dt_s, P,
                encoder_fresh=j%schedule.encoder_period == 0, gyro_fresh=j%schedule.gyro_period == 0,
                encoder_age_s=schedule.encoder_age_s, gyro_age_s=gyro_source_age_s)
            phases.append((F, K))
        if np.max(np.abs(P-before)) <= 1e-12*max(np.max(np.abs(P)), 1e-30):
            return tuple(phases), P
    require(False, Reason.MODEL_INADEQUATE, "periodic observer covariance did not converge")


class FamilySampledAnalysis:
    """Full family tangent and exact held-input/delayed-output sampled lift.

    State is immediately before a controller decision: current/past plant
    snapshots, prior posterior estimate, post-ACK integral/old error, and prior
    plant-drive command history. Observation precedes output, then ACK updates
    the integral, then the plant propagates to the next decision.
    """
    def __setattr__(self, name, value):
        if getattr(self, "_frozen", False): raise AttributeError("cached sampled analysis is immutable; rebuild for changed context")
        object.__setattr__(self, name, value)

    def __init__(self, model, point, support, observer, gains, schedule, *, ff_policy):
        self.local = linearize_sliding_family(model, point, support)
        require(isinstance(gains, LocalGains) and isinstance(schedule, SampleSchedule),
                Reason.DATA_INVALID, "typed local gains and sample schedule required")
        observer = observer_snapshot(observer)
        gains.validate(); schedule.validate(observer, gyro_delay_s=model.gyro_delay)
        require(ff_policy in ("SHARED_POSTERIOR_SLIDE_ALGEBRAIC", "SHARED_POSTERIOR_STEADY_STATE_REFERENCE",
                              "FROZEN_COMMAND_OFFSET"),
                Reason.INTEGRATION_MISMATCH, "explicit supported FF derivative policy required")
        if ff_policy == "SHARED_POSTERIOR_SLIDE_ALGEBRAIC":
            require(model.actuator == "algebraic" and model.transport_delay == 0.,
                    Reason.INTEGRATION_MISMATCH, "shared SLIDE FF supports only zero-delay algebraic actuation")
        # Steady-state reference FF is an algebraic command, not an inverse of
        # the selected actuator pole or transport delay. Those dynamics remain
        # in the forward plant/history and are therefore present in the lift.
        self.model, self.point, self.support, self.observer = model, point, support, observer
        self.gains, self.schedule, self.ff_policy = gains, schedule, ff_policy
        self.ff_state_q = self.local.load_gradient_A_rad/model.actuator_gain if ff_policy.startswith("SHARED") else 0.
        self.ff_reference = immutable(np.array([0., self.local.incremental_damping_A_s_rad/model.actuator_gain,
            model.a/model.actuator_gain]) if ff_policy.startswith("SHARED") else np.zeros(3))
        self.dt, self.n = schedule.dt_s, len(self.local.A)
        self.gyro_source_age_s = schedule.gyro_source_age_s(model.gyro_delay)
        self.delays = (schedule.encoder_age_s, self.gyro_source_age_s, model.current_delay)
        self.act_whole, self.act_fraction = self.split_delay(model.transport_delay)
        oldest = max(math.ceil(x/self.dt-1e-12) for x in self.delays)
        self.plant_count, self.command_count = oldest+1, oldest+self.act_whole+1
        self.controller_start = self.n*self.plant_count
        self.command_start = self.controller_start+4
        self.dimension = self.command_start+self.command_count
        require(self.dimension <= 128, Reason.ENVELOPE_LIMITED, "delayed family lift exceeds 128-state bounded domain")
        self._transitions = {}
        self.observer_phases, self.periodic_covariance = periodic_observer(observer, schedule,
            gyro_source_age_s=self.gyro_source_age_s)
        self.period = len(self.observer_phases)
        phases = []
        for j, (F, K) in enumerate(self.observer_phases):
            zeros = np.zeros(self.dimension)
            A = np.column_stack([self.open_step(x, 0., j, observer_pair=(F, K))[0]
                                 for x in np.eye(self.dimension)])
            B = self.open_step(zeros, 1., j, observer_pair=(F, K))[0]
            H = np.array([self.open_step(x, 0., j, observer_pair=(F, K))[1][-1]
                          for x in np.eye(self.dimension)])
            phases.append(tuple(immutable(matrix) for matrix in (A, B, H)))
        self.phases = tuple(phases)
        self.periodic_covariance = immutable(self.periodic_covariance)
        self._frozen = True

    def assert_context(self, *, model, support, observer, gains, schedule, ff_policy, frame, configuration_id, friction_state):
        self.local.assert_context(model=model, support=support, frame=frame,
            configuration_id=configuration_id, friction_state=friction_state)
        require(isinstance(gains, LocalGains) and isinstance(schedule, SampleSchedule), Reason.DATA_INVALID, "typed sampled context required")
        gains.validate(); schedule.validate(observer_snapshot(observer), gyro_delay_s=model.gyro_delay)
        require(gains == self.gains and schedule == self.schedule and ff_policy == self.ff_policy and
                observer_snapshot(observer) == self.observer, Reason.OPERATING_POINT_CHANGED,
                "gains/observer/sampling/FF changes require a new sampled analysis")

    def split_delay(self, delay):
        whole = int(math.floor(delay/self.dt+1e-12))
        fraction = delay-whole*self.dt
        if abs(fraction) < 1e-12: fraction = 0.
        require(fraction >= 0., Reason.DATA_INVALID, "negative fractional delay")
        return whole, fraction

    def transition(self, duration):
        if duration in self._transitions: return self._transitions[duration]
        augmented = np.zeros((self.n+1, self.n+1))
        augmented[:self.n, :self.n], augmented[:self.n, self.n:] = self.local.A, self.local.B
        result = expm(augmented*duration)
        self._transitions[duration] = (immutable(result[:self.n, :self.n]), immutable(result[:self.n, self.n]))
        return self._transitions[duration]

    def propagate(self, state, interval_back, duration, drive=None):
        plants = state[:self.controller_start].reshape(self.plant_count, self.n)
        commands = state[self.command_start:]
        m, fraction = self.act_whole, self.act_fraction
        new = drive if interval_back == 0 and m == 0 else commands[interval_back+m-1]
        old = commands[interval_back+m] if fraction > 0 else new
        first = min(duration, fraction)
        E, G = self.transition(first)
        mid = E@plants[interval_back]+G*old
        E, G = self.transition(duration-first)
        return E@mid+G*new

    def measurements(self, state):
        plants = state[:self.controller_start].reshape(self.plant_count, self.n)
        commands = state[self.command_start:]
        values = []
        for row, delay in enumerate(self.delays):
            whole, fraction = self.split_delay(delay)
            x = plants[whole] if fraction == 0 else self.propagate(state, whole+1, self.dt-fraction)
            total = self.model.transport_delay+delay
            back = max(1, int(math.ceil(total/self.dt-1e-12)))
            values.append(self.local.C[row]@x+self.local.D[row, 0]*commands[back-1])
        return np.array(values)

    def open_step(self, state, drive, phase, *, measurement_delta=(0., 0.), reference_delta=(0., 0., 0.), observer_pair=None):
        F, K = self.observer_phases[phase] if observer_pair is None else observer_pair
        measured = self.measurements(state)
        previous = state[self.controller_start:self.command_start]
        estimate = F@previous[:2]+K@(measured[:2]+measurement_delta)
        reference_delta = np.asarray(reference_delta)
        error = self.gains.kpos*(reference_delta[0]-estimate[0])+reference_delta[1]-estimate[1]
        command = self.gains.kp*error+previous[2]+self.ff_state_q*estimate[0]+self.ff_reference@reference_delta
        integral = previous[2]+self.gains.ki*self.dt*(error+previous[3])/2
        plant = self.propagate(state, 0, self.dt, drive)
        oldplants = state[:self.controller_start].reshape(self.plant_count, self.n)
        next_state = np.r_[plant, oldplants[:-1].ravel(), estimate, integral, error,
                           drive, state[self.command_start:-1]]
        return next_state, np.r_[estimate, previous[2], command], measured

    def closed_step(self, state, phase, **forcing):
        _, output, _ = self.open_step(state, 0., phase, **forcing)
        return self.open_step(state, output[-1], phase, **forcing)

    def lifted(self):
        transition, inputs, C, D, closed = np.eye(self.dimension), np.zeros((self.dimension, self.period)), [], [], np.eye(self.dimension)
        for j, (A, B, H) in enumerate(self.phases):
            C.append(H@transition); D.append(H@inputs)
            transition, inputs = A@transition, A@inputs
            inputs[:, j] += B
            closed = (A+np.outer(B, H))@closed
        return transition, inputs, np.asarray(C), np.asarray(D), closed

    def pole_diagnostics(self):
        _, _, _, _, closed = self.lifted()
        poles = np.linalg.eigvals(closed)
        require(np.isfinite(poles).all(), Reason.MODEL_INADEQUATE, "scheduled pole calculation failed")
        radius = float(np.max(np.abs(poles)))
        return {"status": "FINITE_SCHEDULED_TANGENT_POLES", "period_ticks": self.period,
            "period_s": self.period*self.dt, "spectral_radius_per_period": radius,
            "equivalent_radius_per_tick": radius**(1/self.period), "local_stable": radius < 1.-1e-10,
            "poles_per_period_real_imag": np.column_stack([poles.real, poles.imag]).tolist(),
            "physical_qualified": False, "native_internal_state_jacobian_verified": False}

    def margin_diagnostics(self, *, phase_required_deg=50., gain_required_db=6.):
        """Nearest scheduled stability boundary under common gain/phase stress.

        The break is downstream of successful ACK. The plant gain is varied,
        while accepted-command accounting remains unchanged and AW inactive.
        Direct monodromy continuation also retains DC and folded Nyquist
        crossings, where an open-loop transfer solve can be singular. These
        are numerical bidirectional reserves for this periodic tangent, not
        physical uncertainty, an irregular-timing proof or a delay inversion.
        """
        poles = self.pole_diagnostics()
        base = {"pole_diagnostics": poles, "phase_margin_deg": None, "gain_margin_db": None,
            "passed": False, "native_internal_state_jacobian_verified": False,
            "definition": "nearest bidirectional common plant-gain/phase stability boundary of periodic monodromy",
            "resolution_limit": "finite grid refinement and critical-pole checks; not a global uncertainty proof"}
        if not poles["local_stable"]: return {**base, "status": "NOMINAL_LOCAL_TANGENT_UNSTABLE"}
        require(math.isfinite(phase_required_deg) and math.isfinite(gain_required_db) and
                phase_required_deg > 0 and gain_required_db > 0, Reason.DATA_INVALID, "positive finite margin requirements")
        def stressed_poles(gain):
            closed = np.eye(self.dimension, dtype=complex)
            for Ai, Bi, Hi in self.phases: closed = (Ai+gain*np.outer(Bi, Hi))@closed
            result = np.linalg.eigvals(closed)
            require(np.isfinite(result).all(), Reason.MODEL_INADEQUATE, "stressed scheduled pole calculation failed")
            return result
        def radius_residual(coordinate, kind):
            gain = np.exp(1j*coordinate) if kind == "phase" else np.exp(coordinate)
            return float(np.max(np.abs(stressed_poles(gain)))-1.)
        def first_boundary(coordinates, kind):
            previous_value = radius_residual(coordinates[0], kind)
            for left, right in zip(coordinates, coordinates[1:]):
                current_value = radius_residual(right, kind)
                if previous_value <= 0 and current_value >= 0:
                    root = brentq(lambda x: radius_residual(x, kind), min(left, right), max(left, right), xtol=1e-11)
                    gain = np.exp(1j*root) if kind == "phase" else np.exp(root)
                    values = stressed_poles(gain)
                    critical = values[np.argmax(np.abs(values))]
                    return {"coordinate": float(root), "unit_pole_distance": float(abs(abs(critical)-1.)),
                        "critical_pole_real_imag": [float(critical.real), float(critical.imag)],
                        "folded_frequency_rad_s": float(np.angle(critical)/(self.period*self.dt))}
                previous_value = current_value
            return None
        previous, result = None, None
        for points in (32, 64, 128, 256):
            phase_crossings = [row for row in (first_boundary(np.linspace(0., -math.pi, points), "phase"),
                first_boundary(np.linspace(0., math.pi, points), "phase")) if row is not None]
            gain_crossings = [row for row in (first_boundary(np.linspace(0., -math.log(1e6), points), "gain"),
                first_boundary(np.linspace(0., math.log(1e6), points), "gain")) if row is not None]
            for row in phase_crossings: row["phase_distance_deg"] = abs(float(np.rad2deg(row["coordinate"])))
            for row in gain_crossings:
                row["critical_gain"] = math.exp(row["coordinate"])
                row["signed_gain_db"] = 20/math.log(10)*row["coordinate"]
            result = {**base, "stress_scan_points_per_direction": points, "gain_search_range": [1e-6, 1e6],
                "gain_crossings": gain_crossings, "phase_crossings": phase_crossings}
            if not gain_crossings or not phase_crossings:
                result["status"] = "UNDEFINED_REQUIRED_MARGIN"
                continue
            pm = min(row["phase_distance_deg"] for row in phase_crossings)
            gm = min(abs(row["signed_gain_db"]) for row in gain_crossings)
            result.update(phase_margin_deg=pm, gain_margin_db=gm)
            if max(row["unit_pole_distance"] for row in gain_crossings+phase_crossings) > 1e-6:
                return {**result, "status": "CRITICAL_CROSSING_POLE_CHECK_FAILED"}
            current = np.array([pm, gm, len(phase_crossings), len(gain_crossings)])
            if previous is not None and np.max(np.abs(current-previous)) < .02:
                return {**result, "status": "CONVERGED_NUMERICAL_SCHEDULED_STABILITY_BOUNDARIES",
                    "resolution_reserve": .02, "passed": pm-.02 >= phase_required_deg and gm-.02 >= gain_required_db}
            previous = current
        return {**result, "status": result.get("status", "UNRESOLVED_MARGIN_RESOLUTION")}

    def document(self):
        return {"qualification": "SYNTHETIC_OFFLINE_UNQUALIFIED", "family": self.model.structure,
            "state_dimension": self.dimension, "family_dynamic_states": list(self.local.state_names),
            "period_ticks": self.period, "dt_s": self.dt, "ff_policy": self.ff_policy,
            "ff_state_q_gradient_A_rad": self.ff_state_q, "ff_state_v_gradient_A_s_rad": 0.,
            "ff_reference_q_v_a_gradients": self.ff_reference.tolist(),
            "plant_transport_delay_s": self.model.transport_delay,
            "gyro_availability_age_s": self.schedule.gyro_availability_age_s,
            "gyro_signal_delay_s": self.model.gyro_delay, "gyro_source_age_s": self.gyro_source_age_s,
            "encoder_gyro_current_output_delays_s": list(self.delays),
            "delay_representation": "EXACT_ZOH_SPLIT_AND_HISTORY; NO_PADE_OR_DELAY_INVERSION",
            "ff_dynamic_policy": "STEADY_STATE_COMMAND_WITH_FORWARD_ACTUATOR_DYNAMICS" if
                self.ff_policy == "SHARED_POSTERIOR_STEADY_STATE_REFERENCE" else self.ff_policy,
            "order": "native observe; FF at posterior; command; successfulACK trapezoid PI; plant propagation",
            "excluded": ["active current/slew/integral/reference/acceleration limits or AW",
                "quantized derivative; nonlinear quantized forecast required", "rest/start/reversal or changed support",
                "irregular timing; periodic schedule is explicit", "dynamic FF inversion", "physical qualification or gains"],
            "native_evidence_limit": "finite-prefix observable response; covariance/old-error/internal state setter not exposed"}


def observer_tick(observer, dt, covariance, *, encoder_fresh, gyro_fresh, encoder_age_s=0., gyro_age_s=0.):
    """Native sequential encoder then gyro covariance and posterior tangent."""
    require(isinstance(dt, (int, float)) and not isinstance(dt, bool) and math.isfinite(dt) and dt > 0 and
            type(encoder_fresh) is type(gyro_fresh) is bool, Reason.DATA_INVALID, "positive time and explicit freshness required")
    observer = observer_snapshot(observer)
    require(np.shape(covariance) == (2, 2) and np.isfinite(covariance).all() and
            np.allclose(covariance, np.asarray(covariance).T, rtol=0., atol=1e-20) and
            np.min(np.linalg.eigvalsh(covariance)) >= -1e-20,
            Reason.DATA_INVALID, "finite positive semidefinite observer covariance required")
    require(0 <= encoder_age_s <= observer.max_encoder_age_s and 0 <= gyro_age_s <= observer.max_gyro_age_s,
            Reason.MEASUREMENT_LIMITED, "observer source ages outside declared freshness support")
    F = np.array([[1., dt], [0., 1.]])
    w = observer.process_variance
    P = F@covariance@F.T + w*np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
    M, K = np.eye(2), np.zeros((2, 2))
    for j, H, variance, fresh in ((0, np.array([1., -encoder_age_s]),
            observer.encoder_variance+w*encoder_age_s**4/4, encoder_fresh),
            (1, np.array([0., 1.]), observer.gyro_variance+w*gyro_age_s**2, gyro_fresh)):
        if not fresh: continue
        c = P@H
        gain = c/(H@c+variance)
        update = np.eye(2)-np.outer(gain, H)
        P -= np.outer(c, c)/(H@c+variance)
        P[0, 0], P[1, 1] = max(P[0, 0], 0.), max(P[1, 1], 0.)
        M, K = update@M, update@K
        K[:, j] += gain
    return M@F, K, P


def controller_tick(state, measurements, dt, gains, observer_transition, observer_input, *, command_ff=0.):
    """Posterior observer → command → successful immediate ACK; limits inactive.

    State contains posterior estimate q/v, integral after previous ACK, previous
    error. Feedforward is supplied in command A. Back-calculation is zero only
    because the accepted command equals the unconstrained request.
    """
    estimate = observer_transition@state[:2]+observer_input@measurements
    error = -gains.kpos*estimate[0]-estimate[1]
    command = gains.kp*error+state[2]+command_ff
    output = np.r_[estimate, state[2], command]
    return np.r_[estimate, state[2]+gains.ki*dt*(error+state[3])/2, error], output
