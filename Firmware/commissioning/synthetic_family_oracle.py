"""Independent synthetic hybrid plant oracle; never calls the native rollout.

The Coulomb continuous modes use a matrix exponential. Stribeck modes use
SciPy's DOP853 integrator and explicit zero-speed events. Successful-command
changes and first-order current breakaway times delimit modes. Sensor outputs
are sampled from the same uninterrupted state, at their delayed native times.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.optimize import brentq


@dataclass(frozen=True)
class OracleResult:
    trace: np.ndarray
    events: list[dict]
    diagnostics: dict


def independent_rollout(model, t, tx_t, tx_A, initial, *, rtol=1e-10,
                        atol=1e-12, max_step=.0005) -> OracleResult:
    """Return q,v,i,delayed gyro,delayed reported current,stick at supplied times.

    ``model`` has the FamilyModel parameter interface. All values are explicit
    synthetic fixture parameters; neither calibration nor deployment authority
    is implied. The initial state has five entries: q,v,i,gyro filter,current
    filter. Pre-run filtered sensor history is held at that supplied state.
    Unfiltered algebraic reported current instead uses full successful ZOH
    prehistory, including its separate sensor delay.
    """
    model.validate()
    t, tx_t, tx_A, initial = (np.asarray(x, dtype=float) for x in (t, tx_t, tx_A, initial))
    if not (t.ndim == tx_t.ndim == 1 and len(t) >= 2 and len(tx_t) >= 1 and
            tx_A.shape == tx_t.shape and initial.shape == (5,) and
            np.all(np.diff(t) > 0) and np.all(np.diff(tx_t) > 0) and
            all(np.isfinite(x).all() for x in (t, tx_t, tx_A, initial))):
        raise ValueError("finite increasing observation/TX times and one five-state initializer required")
    if not (0 < max_step <= .01 and 0 < rtol < 1 and 0 < atol < 1):
        raise ValueError("declared oracle tolerances and maximum event bracket step required")
    needed = t[0] - model.transport_delay
    if model.actuator == "algebraic" and model.current_tau == 0:
        needed -= model.current_delay
    if tx_t[0] > needed:
        raise ValueError("successful-input prehistory is missing")
    applied = tx_t + model.transport_delay
    gyro_times = np.maximum(t[0], t - model.gyro_delay)
    current_times = np.maximum(t[0], t - model.current_delay)
    anchors = np.unique(np.r_[t, gyro_times, current_times,
        applied[(applied >= t[0]) & (applied <= t[-1])]])
    y = initial.copy()
    command = max(0, int(np.searchsorted(applied, t[0], side="right") - 1))
    target = model.actuator_gain * tx_A[command] + model.actuator_bias
    if model.actuator == "algebraic": y[2] = target
    if model.gyro_tau == 0: y[3] = y[1]
    if model.current_tau == 0: y[4] = y[2]
    state_history = [y.copy()]
    events = []
    counters = {"matrix_exponential_propagations": 0, "adaptive_rhs_evaluations": 0,
                "continuous_segments": 0, "zero_crossings": 0, "breakaways": 0,
                "successful_command_discontinuities": 0}
    cache = {}

    def load_at(q):
        return model.load_offset + (model.load_slope * (q - model.q_origin)
                                   if model.load == "affine" else 0.)

    def friction(v, direction):
        d = "positive" if direction > 0 else "negative"
        fc, fs = getattr(model, "coulomb_" + d), getattr(model, "static_" + d)
        if model.friction == "coulomb": return direction * fc
        vs = getattr(model, "stribeck_" + d)
        return direction * (fc + (fs - fc) * math.exp(-(abs(v) / vs) ** model.stribeck_power))

    def rhs(state, desired, direction, stick):
        q, v, i, gyro, current = state
        return np.array([0. if stick else v,
            0. if stick else (i - load_at(q) - model.viscous * v - friction(v, direction)) / model.a,
            (desired - i) / model.actuator_tau if model.actuator == "first_order" else 0.,
            (v - gyro) / model.gyro_tau if model.gyro_tau > 0 else 0.,
            (i - current) / model.current_tau if model.current_tau > 0 else 0.])

    def linear_propagation(state, desired, duration, direction, stick):
        # q,v,i,gyro,current,1. The affine source stays separate from state.
        # Cached operators depend only on one declared mode/input/duration.
        key = (desired, direction, stick, round(float(duration), 15))
        operator = cache.get(key)
        if operator is None:
            matrix = np.zeros((6, 6))
            if not stick:
                matrix[0, 1] = 1
                matrix[1, 0] = -model.load_slope / model.a if model.load == "affine" else 0.
                matrix[1, 1] = -model.viscous / model.a
                matrix[1, 2] = 1 / model.a
                spatial_constant = model.load_offset - (model.load_slope * model.q_origin
                                                         if model.load == "affine" else 0.)
                matrix[1, 5] = -(spatial_constant + friction(0., direction)) / model.a
            if model.actuator == "first_order":
                matrix[2, 2] = -1 / model.actuator_tau
                matrix[2, 5] = desired / model.actuator_tau
            if model.gyro_tau > 0:
                matrix[3, 1] = 1 / model.gyro_tau
                matrix[3, 3] = -1 / model.gyro_tau
            if model.current_tau > 0:
                matrix[4, 2] = 1 / model.current_tau
                matrix[4, 4] = -1 / model.current_tau
            operator = expm(matrix * duration)
            cache[key] = operator
        result = (operator @ np.r_[state, 1.])[:5]
        if model.gyro_tau == 0: result[3] = result[1]
        if model.current_tau == 0: result[4] = result[2]
        counters["matrix_exponential_propagations"] += 1
        return result

    def holding(state, desired):
        net = state[2] - load_at(state[0])
        held = abs(state[1]) < 1e-12 and -model.static_negative <= net <= model.static_positive
        if held and model.actuator == "first_order":
            # A threshold reached with outward actuator response releases now.
            if (net >= model.static_positive - 1e-12 and desired - load_at(state[0]) > model.static_positive) or \
               (net <= -model.static_negative + 1e-12 and desired - load_at(state[0]) < -model.static_negative):
                held = False
        return held, net

    at = float(anchors[0])
    prior_stick = holding(y, target)[0]
    for end in anchors[1:]:
        transitions = 0
        while at < end - 1e-13:
            transitions += 1
            if transitions > 10000:
                raise RuntimeError("independent hybrid transition loop did not progress")
            while command + 1 < len(applied) and applied[command + 1] <= at + 1e-13:
                command += 1
                counters["successful_command_discontinuities"] += 1
            target = model.actuator_gain * tx_A[command] + model.actuator_bias
            if model.actuator == "algebraic": y[2] = target
            if model.current_tau == 0: y[4] = y[2]
            stick, net = holding(y, target)
            if stick: y[1] = 0.
            if prior_stick and not stick:
                events.append({"kind": "breakaway", "time_s": at, "direction": 1 if net > 0 else -1})
                counters["breakaways"] += 1
            duration = min(float(end - at), max_step)
            if command + 1 < len(applied): duration = min(duration, float(applied[command + 1] - at))
            threshold = None
            if stick and model.actuator == "first_order":
                load = load_at(y[0])
                if target - load > model.static_positive: threshold = load + model.static_positive
                elif target - load < -model.static_negative: threshold = load - model.static_negative
                if threshold is not None:
                    ratio = (threshold - target) / (y[2] - target)
                    if 0 < ratio < 1:
                        crossing = -model.actuator_tau * math.log(ratio)
                        if crossing < duration: duration = crossing
            direction = (1 if y[1] > 0 else -1) if abs(y[1]) >= 1e-12 else (1 if net > 0 else -1)
            counters["continuous_segments"] += 1
            crossed = False
            if model.friction == "coulomb" or stick:
                after = linear_propagation(y, target, duration, direction, stick)
                if not stick and y[1] * after[1] < 0:
                    # A small declared event bracket limits possible missed roots.
                    zero = brentq(lambda h: linear_propagation(y, target, h, direction, False)[1],
                                  0., duration, xtol=1e-14, rtol=1e-14)
                    after = linear_propagation(y, target, zero, direction, False)
                    duration = zero; crossed = True
            else:
                def zero_event(when, state):
                    # Avoid the trivial initial v=0 root during outward release.
                    return 1e-14 if when == at and abs(state[1]) < 1e-12 else direction * state[1]
                zero_event.terminal = True
                zero_event.direction = -1
                solved = solve_ivp(lambda when, state: rhs(state, target, direction, False),
                    (at, at + duration), y, method="DOP853", rtol=rtol, atol=atol,
                    max_step=max_step, events=zero_event)
                counters["adaptive_rhs_evaluations"] += solved.nfev
                if not solved.success: raise RuntimeError(solved.message)
                after = solved.y[:, -1]
                duration = float(solved.t[-1] - at)
                crossed = bool(len(solved.t_events[0]))
                if model.gyro_tau == 0: after[3] = after[1]
                if model.current_tau == 0: after[4] = after[2]
            if crossed:
                after[1] = 0.
                events.append({"kind": "zero_crossing", "time_s": at + duration,
                               "direction_before": direction})
                counters["zero_crossings"] += 1
            if not np.isfinite(after).all() or not model.q_min <= after[0] <= model.q_max:
                raise ValueError(f"independent state diverged outside numerical domain at {at + duration}")
            if duration <= 0: raise RuntimeError("independent event failed to advance time")
            y = after; at += duration; prior_stick = stick
        # Right-continuous held input at the requested native/event time.
        while command + 1 < len(applied) and applied[command + 1] <= end + 1e-13:
            command += 1
            counters["successful_command_discontinuities"] += 1
        target = model.actuator_gain * tx_A[command] + model.actuator_bias
        if model.actuator == "algebraic": y[2] = target
        if model.current_tau == 0: y[4] = y[2]
        state_history.append(y.copy())
    states = np.asarray(state_history)
    direct = states[np.searchsorted(anchors, t)]
    gyro = states[np.searchsorted(anchors, gyro_times), 3] + model.gyro_bias
    if model.actuator == "algebraic" and model.current_tau == 0:
        current_index = np.maximum(0, np.searchsorted(tx_t,
            t - model.current_delay - model.transport_delay + 1e-13, side="right") - 1)
        reported_current = model.actuator_gain * tx_A[current_index] + model.actuator_bias
    else:
        reported_current = states[np.searchsorted(anchors, current_times), 4]
    current = model.current_gain * reported_current + model.current_bias
    net = direct[:, 2] - np.array([load_at(q) for q in direct[:, 0]])
    stick = (np.abs(direct[:, 1]) < 1e-12) & (net >= -model.static_negative) & (net <= model.static_positive)
    trace = np.c_[direct[:, :3], gyro, current, stick.astype(float)]
    return OracleResult(trace, events, {"method": "independent exact linear modes/DOP853 hybrid events",
        "rtol": rtol, "atol": atol, "max_event_bracket_step_s": max_step,
        "filtered_sensor_prehistory": "held supplied five-state initializer before observation start",
        "input": "successful TX causal ZOH split at exact delayed application times",
        **counters, "provenance": "SYNTHETIC", "native_generator_used": False})
