"""Offline failure routing and native-observation diagnostics for ADR-002.2.

This module never commands hardware, retries motion, resets a prediction state,
changes an acceptance limit, or turns diagnostic success into deployment approval.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import numpy as np


class Check(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    NOT_RUN = "NOT_RUN"
    UNKNOWN = "UNKNOWN"


class AnalysisState(str, Enum):
    READY = "READY"
    RUNNING = "RUNNING"
    DIAGNOSE = "DIAGNOSE"
    RETEST = "RETEST"
    BLOCKED_EXTERNAL = "BLOCKED_EXTERNAL"


class Failure(str, Enum):
    BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
    CONVERGED_BUT_TRAJECTORY_FAILS = "CONVERGED_BUT_TRAJECTORY_FAILS"
    SYNTHETIC_REFERENCE_FAILS_AT_TRUE_PARAMETERS = "SYNTHETIC_REFERENCE_FAILS_AT_TRUE_PARAMETERS"
    PARAMETERS_RECOVER_BUT_CROSS_PREDICTION_FAILS = "PARAMETERS_RECOVER_BUT_CROSS_PREDICTION_FAILS"
    NO_MOTION_MODEL_ON_MOVING_DATA = "NO_MOTION_MODEL_ON_MOVING_DATA"
    NEEDS_DISCRIMINATING_DATA = "NEEDS_DISCRIMINATING_DATA"
    HARDWARE_FAULT = "HARDWARE_FAULT"
    UNRESOLVABLE_EXTERNAL_PREREQUISITE = "UNRESOLVABLE_EXTERNAL_PREREQUISITE"
    DATA_INTEGRITY_FAILURE = "DATA_INTEGRITY_FAILURE"
    FORWARD_NUMERICS_FAILURE = "FORWARD_NUMERICS_FAILURE"


@dataclass(frozen=True)
class CandidateChecks:
    data_integrity: Check = Check.UNKNOWN
    forward_numerics: Check = Check.UNKNOWN
    optimizer_termination_reason: str = "NOT_RUN"
    optimizer_converged: Check = Check.NOT_RUN
    synthetic_parameter_recovery: Check = Check.NOT_RUN
    training_trajectory: Check = Check.NOT_RUN
    selection_trajectory: Check = Check.NOT_RUN
    historical_regression: Check = Check.NOT_RUN
    prospective_prediction: Check = Check.NOT_RUN
    physical_stage3a: Check = Check.NOT_RUN
    physical_stage3b: Check = Check.NOT_RUN
    deployment_authorized: bool = False


_ROUTES = {
    Failure.BUDGET_EXHAUSTED: ("candidate promotion", "checkpoint -> improvement/scaling/event/derivative diagnosis -> justified resume or alternative"),
    Failure.CONVERGED_BUT_TRAJECTORY_FAILS: ("candidate promotion", "objective/profile/structure discrimination; retain numerical success separately"),
    Failure.SYNTHETIC_REFERENCE_FAILS_AT_TRUE_PARAMETERS: ("affected estimator/structure qualification", "forward integration/units/timing/state regression"),
    Failure.PARAMETERS_RECOVER_BUT_CROSS_PREDICTION_FAILS: ("synthetic/plant gate", "locate divergence; examine excitation, event sensitivity, noise and objective; do not waive gate"),
    Failure.NO_MOTION_MODEL_ON_MOVING_DATA: ("model adequacy", "threshold/load/state identification and baseline/event metrics"),
    Failure.NEEDS_DISCRIMINATING_DATA: ("physical model promotion", "WP5 approved bounded information-gain experiment -> WP4; offline work continues"),
    Failure.HARDWARE_FAULT: ("physical motion and rearm", "validated stop/latch -> offline fault diagnosis; explicit reentry tests"),
    Failure.UNRESOLVABLE_EXTERNAL_PREREQUISITE: ("dependent work only", "precise required fact/accuracy/approval request; continue independent work"),
    Failure.DATA_INTEGRITY_FAILURE: ("model promotion", "repair units/masks/sign/clock/provenance; replay preserved observations"),
    Failure.FORWARD_NUMERICS_FAILURE: ("affected candidate qualification", "minimal integration/units/timing/state regression; preserve first numerical failure"),
}


def recovery_decision(checks: CandidateChecks, failures=(), *, evidence=(),
                      physical_state="NOT_OBSERVED_OFFLINE", external_prerequisite=None):
    """Reject promotion while returning the next safe, informative analysis step.

    Unknown/unperformed checks are retained as such. They block promotion but do
    not invent a failed experiment. Physical authorization is a separate input.
    """
    failures = list(dict.fromkeys(Failure(f) for f in failures))
    if checks.data_integrity == Check.FAIL:
        failures.insert(0, Failure.DATA_INTEGRITY_FAILURE)
    elif checks.forward_numerics == Check.FAIL:
        failures.insert(0, Failure.FORWARD_NUMERICS_FAILURE)
    failures = list(dict.fromkeys(failures))
    gate_names = [n for n in asdict(checks) if n not in
                  ("optimizer_termination_reason", "deployment_authorized")]
    incomplete = [n for n in gate_names if getattr(checks, n) != Check.PASS]
    if external_prerequisite and not failures:
        failures.append(Failure.UNRESOLVABLE_EXTERNAL_PREREQUISITE)
    routes = [{"condition": f.value, "block": _ROUTES[f][0], "continue": _ROUTES[f][1]}
              for f in failures]
    state = AnalysisState.DIAGNOSE if failures else AnalysisState.RETEST
    if external_prerequisite and failures == [Failure.UNRESOLVABLE_EXTERNAL_PREREQUISITE]:
        state = AnalysisState.BLOCKED_EXTERNAL
    return {"schema": "adr0022.analysis-recovery/1", "software_policy_only": True,
        "analysis_state": state.value, "checks": asdict(checks),
        "first_failing_predicate": failures[0].value if failures else None,
        "promotion_blocked": bool(failures or incomplete or not checks.deployment_authorized),
        "incomplete_or_failed_checks": incomplete, "routes": routes,
        "next_action": routes[0]["continue"] if routes else "run remaining frozen gates before promotion",
        "evidence_references": list(evidence), "physical_state": physical_state,
        "external_prerequisite": external_prerequisite,
        "acceptance_criteria_changed": False, "deployment_authorized": False,
        "automatically_repeat_motion": False, "automatically_rearm": False}


@dataclass(frozen=True)
class DiagnosticPolicy:
    # These are descriptive partitions/detectors, not new qualification gates or
    # loss weights. The engineering gates remain in model_family.family_reports.
    minimum_moving_rad_s: float = float(np.deg2rad(.5))
    native_noise_multiplier: float = 3.
    motion_confirmation_s: float = .05
    significant_command_A: float = .01
    low_speed_upper_rad_s: float = float(np.deg2rad(5.))
    high_speed_lower_rad_s: float = float(np.deg2rad(30.))
    horizons_s: tuple = (.05, .1, .2, .5, 1.)


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None


def _motion(t, velocity, threshold, persistence, end):
    """Native sample ZOH durations; confirmed event times retain onset time.

    Confirmation uses later samples and is therefore explicitly retrospective.
    It filters short sign excursions for event summaries only. Duration reports
    the unfiltered threshold occupancy so persistence cannot erase hard samples.
    """
    signs = np.where(np.abs(velocity) >= threshold, np.sign(velocity), 0.).astype(int)
    intervals = np.diff(np.r_[t, end])
    events, state, pending, since = [], int(signs[0]), int(signs[0]), float(t[0])
    last_moving = state
    for timestamp, direction in zip(t[1:], signs[1:]):
        if direction != pending:
            pending, since = int(direction), float(timestamp)
        if pending != state and timestamp - since >= persistence:
            kind = "observed_motion_onset" if state == 0 and pending else \
                   "observed_stop" if pending == 0 else "observed_reversal"
            events.append({"kind": kind, "time_s": since, "confirmed_at_s": float(timestamp),
                           "from_direction": state, "to_direction": pending})
            if state == 0 and pending and last_moving and pending != last_moving:
                events.append({"kind": "observed_reversal", "time_s": since,
                    "confirmed_at_s": float(timestamp), "from_direction": last_moving,
                    "to_direction": pending, "passed_through_rest": True})
            if pending: last_moving = pending
            state = pending
    return {"moving_duration_s": float(intervals[signs != 0].sum()),
        "positive_duration_s": float(intervals[signs > 0].sum()),
        "negative_duration_s": float(intervals[signs < 0].sum()),
        "starts_already_moving": bool(signs[0]),
        "maximum_abs_velocity_rad_s": float(np.max(np.abs(velocity))), "events": events}


def _huber_cost(residual):
    absolute = np.abs(residual)
    return np.where(absolute <= 1., .5 * residual**2, absolute - .5)


def diagnostic_report(run, prediction, *, policy=DiagnosticPolicy()):
    """Slice one uninterrupted prediction at native observations and event times.

    Retrospective regime labels never enter the forward equations. The objective
    decomposition exactly mirrors the unmodified, sigma-scaled bin residual and
    Huber loss in fit_family; no per-run/channel/regime reweighting is performed.
    """
    run.validate()
    prediction = np.asarray(prediction, dtype=float)
    if prediction.shape != (len(run.t), 6) or not np.isfinite(prediction).all():
        raise ValueError("complete finite six-state uninterrupted prediction required")
    threshold = max(policy.minimum_moving_rad_s, policy.native_noise_multiplier * run.sigma_v)
    t, qmask, vmask, imask = run.t, run.q_new, run.v_new, run.current_new
    observed_q, predicted_q = run.q[qmask], prediction[qmask, 0]
    observed_v, predicted_v = run.v[vmask], prediction[vmask, 3]
    qt, vt = t[qmask], t[vmask]
    observed_motion = _motion(vt, observed_v, threshold, policy.motion_confirmation_s, t[-1])
    predicted_motion = _motion(vt, predicted_v, threshold, policy.motion_confirmation_s, t[-1])
    for event in predicted_motion["events"]:
        event["kind"] = event["kind"].replace("observed_", "predicted_")
    transitions = {}
    for kind in ("motion_onset", "reversal", "stop"):
        observed = [e for e in observed_motion["events"] if e["kind"] == "observed_" + kind]
        predicted = [e for e in predicted_motion["events"] if e["kind"] == "predicted_" + kind]
        transitions[kind] = {"observed_count": len(observed), "predicted_count": len(predicted),
            "unmatched_observed_count": max(len(observed) - len(predicted), 0),
            "extra_predicted_count": max(len(predicted) - len(observed), 0),
            "ordered_pairs": [{"observed_time_s": a["time_s"], "predicted_time_s": b["time_s"],
                "timing_error_s": b["time_s"] - a["time_s"]} for a, b in zip(observed, predicted)],
            "matching_policy": "ordered same-type retrospective diagnostic; not a formal transition gate"}
    # Causal pre-motion seed shared by the model and both trivial baselines.
    constant_position = np.full(len(observed_q), run.initial[0])
    constant_velocity = run.initial[0] + run.initial[1] * (qt - t[0])
    q_error = predicted_q - observed_q
    baseline = {"initial_position_rad": float(run.initial[0]),
        "initial_velocity_rad_s": float(run.initial[1]),
        "state_source": "same single run initial state used by continuous prediction; TRAIN may estimate it",
        "model_q_rms_rad": _rms(q_error),
        "constant_position_q_rms_rad": _rms(constant_position - observed_q),
        "constant_velocity_q_rms_rad": _rms(constant_velocity - observed_q),
        "beats_constant_position": bool(_rms(q_error) < _rms(constant_position - observed_q)),
        "beats_constant_velocity": bool(_rms(q_error) < _rms(constant_velocity - observed_q)),
        "improvement_over_constant_position_rad": _rms(constant_position - observed_q) - _rms(q_error),
        "improvement_exceeds_encoder_half_bin": bool(
            _rms(constant_position - observed_q) - _rms(q_error) > run.encoder_quantum / 2),
        "absolute_angle_gate_rad": float(np.deg2rad(.15)),
        "absolute_angle_gate_passed": bool(_rms(q_error) <= np.deg2rad(.15))}
    displacement = {"observed_span_rad": float(np.ptp(observed_q)),
        "predicted_span_rad": float(np.ptp(predicted_q)),
        "observed_net_rad": float(observed_q[-1] - observed_q[0]),
        "predicted_net_rad": float(predicted_q[-1] - predicted_q[0]),
        "observed_total_native_increment_rad": float(np.abs(np.diff(observed_q)).sum()),
        "predicted_total_native_increment_rad": float(np.abs(np.diff(predicted_q)).sum()),
        "native_increment_is_not_velocity": True}
    # Report a descriptive no-motion contradiction only beyond encoder-bin and
    # measured noise resolution; this remains independent of the absolute gate.
    no_motion = bool(displacement["predicted_span_rad"] <= max(1e-12, run.encoder_quantum / 2)
        and displacement["observed_span_rad"] > max(run.encoder_quantum, 6 * run.sigma_q))
    anchors = [{**event, "anchor_policy": "OUTCOME_TRIGGERED_RETROSPECTIVE_DIAGNOSIS"}
               for event in observed_motion["events"]]
    preceding = np.searchsorted(run.tx_t, t[0], side="right") - 1
    if preceding < 0:
        raise ValueError("successful command prehistory is required for diagnostics")
    previous = float(run.tx_A[preceding])
    for timestamp, current in zip(run.tx_t, run.tx_A):
        if timestamp < t[0] or timestamp > t[-1]: continue
        if abs(current - previous) > policy.significant_command_A:
            anchors.append({"kind": "successful_command_change", "time_s": float(timestamp),
                "delta_from_previous_anchor_A": float(current - previous),
                "anchor_policy": "REALIZED_INPUT_HISTORY_RETROSPECTIVE; NOT_PRERUN_REFERENCE"})
            previous = float(current)
    anchors.sort(key=lambda x: (x["time_s"], x["kind"]))
    # Raw channel errors serve trajectory diagnostics; q bin errors below serve
    # the fit objective. Neither horizon resets the trajectory to a measurement.
    channels = (("encoder", qmask, prediction[:, 0] - run.q, run.sigma_q),
                ("gyro", vmask, prediction[:, 3] - run.v, run.sigma_v),
                ("current", imask, prediction[:, 4] - run.current, run.sigma_current))
    for anchor in anchors:
        anchor["horizons"] = {}
        for horizon in policy.horizons_s:
            window = (t >= anchor["time_s"]) & (t <= anchor["time_s"] + horizon)
            anchor["horizons"][str(horizon)] = {name: {"native_samples": int((mask & window).sum()),
                "rms": _rms(error[mask & window])} for name, mask, error, _ in channels}
    # Native gyro ZOH is used only to label observed regimes, never to predict.
    gyro_index = np.searchsorted(vt, t, side="right") - 1
    regime_velocity = np.where(gyro_index >= 0, observed_v[np.maximum(gyro_index, 0)], run.initial[3])
    magnitude = np.abs(regime_velocity)
    regimes = np.full(len(t), "rest", dtype="U24")
    regimes[magnitude >= threshold] = "low_speed"
    regimes[magnitude >= policy.low_speed_upper_rad_s] = "moving_5_to_30_deg_s"
    regimes[magnitude >= policy.high_speed_lower_rad_s] = "high_speed"
    objective = {}
    for name, mask, error, sigma in channels:
        raw = error[mask]
        residual = raw
        if name == "encoder":
            residual = np.sign(raw) * np.maximum(np.abs(raw) - run.encoder_quantum / 2, 0.)
        normalized = residual / sigma
        costs = _huber_cost(normalized)
        objective[name] = {"native_samples": len(raw), "sigma": float(sigma),
            "raw_residual_rms": _rms(raw), "normalized_residual_rms": _rms(normalized),
            "huber_cost": float(costs.sum()), "regimes": {}, "event_windows": {}}
        for regime in ("rest", "low_speed", "moving_5_to_30_deg_s", "high_speed"):
            selection = regimes[mask] == regime
            objective[name]["regimes"][regime] = {"native_samples": int(selection.sum()),
                "raw_residual_rms": _rms(raw[selection]),
                "normalized_residual_rms": _rms(normalized[selection]),
                "huber_cost": float(costs[selection].sum())}
        for kind in ("successful_command_change", "observed_motion_onset", "observed_reversal", "observed_stop"):
            # These overlapping 200 ms diagnostic windows are an additional
            # factor breakdown, never additive weights or disjoint partitions.
            window = np.zeros(len(t), dtype=bool)
            for anchor in anchors:
                if anchor["kind"] == kind:
                    window |= (t >= anchor["time_s"]) & (t <= anchor["time_s"] + .2)
            selection = window[mask]
            objective[name]["event_windows"][kind] = {"native_samples": int(selection.sum()),
                "window_s": .2, "may_overlap_other_event_windows": True,
                "raw_residual_rms": _rms(raw[selection]),
                "normalized_residual_rms": _rms(normalized[selection]),
                "huber_cost": float(costs[selection].sum())}
    return {"schema": "adr0022.native-whole-run-diagnostics/1", "run_id": run.run_id,
        "prediction_kind": "INPUT_DRIVEN_WHOLE_RUN", "state_resets": 0,
        "diagnostic_policy": {**asdict(policy), "effective_moving_threshold_rad_s": threshold,
            "qualification_gate": False, "weights_changed": False},
        "baselines": baseline, "displacement": displacement,
        "no_motion_model_on_moving_data": no_motion,
        "observed_motion": observed_motion, "predicted_motion": predicted_motion,
        "transition_comparison": transitions,
        "event_anchored_horizons": anchors,
        "event_horizon_policy": "slice existing continuous prediction; no measured-state reinjection",
        "objective": {"loss": "huber", "f_scale": 1., "cost_convention": ".5 * sum(rho(r**2))",
            "encoder_residual": "sign(center_error) * max(abs(center_error)-quantum/2,0) / sigma_q",
            "gyro_prediction": "filtered measurement output, column 3; latent velocity is column 1",
            "current_prediction": "reported-current output, column 4",
            "independent_innovations_assumed": False, "channels": objective,
            "total_huber_cost": float(sum(c["huber_cost"] for c in objective.values()))},
        "qualification": "RETROSPECTIVE_DIAGNOSTIC_UNQUALIFIED", "deployment_authorized": False}


def comparison_checks(candidate, *, data_integrity=Check.UNKNOWN, forward_numerics=Check.UNKNOWN):
    optimizer = candidate.get("optimizer", {})
    numerical_success = bool(optimizer.get("success", False))
    training, selection = candidate.get("training", []), candidate.get("selection", [])
    return CandidateChecks(data_integrity=data_integrity, forward_numerics=forward_numerics,
        optimizer_termination_reason=optimizer.get("message", candidate.get("reason", "UNKNOWN")),
        optimizer_converged=Check.PASS if numerical_success else Check.FAIL if optimizer else Check.NOT_RUN,
        training_trajectory=Check.PASS if training and all(r["passed"] for r in training) else
            Check.FAIL if training else Check.NOT_RUN,
        selection_trajectory=Check.PASS if selection and all(r["passed"] for r in selection) else
            Check.FAIL if selection else Check.NOT_RUN)


def comparison_recovery(candidate, diagnostics=()):
    # Finite arrays/masks are a software subcheck. Current semantics, coordinate
    # registration and physical configuration compatibility remain unqualified.
    checks = comparison_checks(candidate)
    failures = []
    message = checks.optimizer_termination_reason.lower()
    if "maximum number" in message or "budget" in message or "max_nfev" in message:
        failures.append(Failure.BUDGET_EXHAUSTED)
    elif checks.optimizer_converged == Check.PASS and (
            checks.training_trajectory == Check.FAIL or checks.selection_trajectory == Check.FAIL):
        failures.append(Failure.CONVERGED_BUT_TRAJECTORY_FAILS)
    if any(d["no_motion_model_on_moving_data"] for d in diagnostics):
        failures.append(Failure.NO_MOTION_MODEL_ON_MOVING_DATA)
    result = recovery_decision(checks, failures)
    result["native_array_integrity"] = Check.PASS if diagnostics else Check.NOT_RUN
    result["data_integrity_scope"] = "physical input/clock/registration/configuration contract remains UNKNOWN"
    return result
