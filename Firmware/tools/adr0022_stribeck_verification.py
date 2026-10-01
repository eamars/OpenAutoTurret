"""Narrow independent-oracle Stribeck inverse verification; never operates hardware."""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields, replace
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import lsq_linear
from scipy.signal import savgol_filter

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun, _errors, fit_family
from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_closed_loop_estimator_probe import RECOVERY_GATES


DEVELOPMENT_INPUT_SEEDS = (3109, 3461)
FINAL_INPUT_SEEDS = (4517, 4903, 5231)
JOINT_RECOVERY_INPUT_SEEDS = (6211, 6841, 7307)
BALANCE_GRID_POINTS = 7
BALANCE_INTERVALS_S = (.06, .12, .20)
SPEED_BOUNDS = {"stribeck_negative": (.025, .14), "stribeck_positive": (.02, .12)}
JOINT_BOUNDS = {"a": (.06, .15), "viscous": (.015, .12), "coulomb_negative": (.07, .15),
                "coulomb_positive": (.07, .17), **SPEED_BOUNDS}
INITIAL = {"stribeck_negative": .095, "stribeck_positive": .033}
JOINT_INITIAL = {"a": .085, "viscous": .075, "coulomb_negative": .105, "coulomb_positive": .13, **INITIAL}
RECOVERY_LIMITS = {"pristine": RECOVERY_GATES["pristine_relative"], "noisy": RECOVERY_GATES["noisy_relative"]}
MAX_NFEV = 120
CHECKPOINT_RESUME_NFEV = 40
JOINT_METHOD_REVISION = "balance-grid-native120-single40/1"
QUANTUM = 2*np.pi/8192
NOISE = {"encoder_sigma_rad": .00015, "gyro_sigma_rad_s": .005, "current_sigma_A": .002}
NUMERICAL_GATES = {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9}
LEVELS = (.21, 0., -.15, 0., .265, 0., -.235, 0., .215, -.21, 0., .19, -.13, 0.)
HOLDS = (.48, .40, .55, .38, .42, .45, .46, .50, .40, .50, .60, .30, .30, .50)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def parameter_recovery_gate(relative_errors, bound_hits, *, noisy):
    if type(noisy) is not bool:
        raise ValueError("the declared pristine/noisy observation class must be explicit")
    limit = RECOVERY_LIMITS["noisy" if noisy else "pristine"]
    return {"limit": limit, "passed": not bound_hits and all(abs(value) <= limit for value in relative_errors.values())}


def narrow_development_ready(summary, contract):
    """A training-only viability probe cannot authorize the joint diagnostic."""
    expected_runs = {f"stribeck-input{DEVELOPMENT_INPUT_SEEDS[0]}-{noise}" for noise in ("pristine", "noisy")}
    cases = summary.get("cases", [])
    return (summary.get("scope") == contract.get("scope") == "speeds"
        and summary.get("partition") == contract.get("partition") == "development"
        and summary.get("probe_only") is False and contract.get("probe_only") is False
        and summary.get("narrow_diagnostic_passed") is True
        and summary.get("synthetic_scope_passed") is True
        and summary.get("free_coordinate_count") == 2
        and contract.get("active_train_input_seed") == DEVELOPMENT_INPUT_SEEDS[0]
        and tuple(contract.get("active_selection_input_seeds", ())) == DEVELOPMENT_INPUT_SEEDS[1:]
        and {case.get("training_run") for case in cases} == expected_runs
        and len(cases) == 2 and all(case.get("passed") is True for case in cases))


def joint_method_ready(freeze):
    return (freeze.get("method_revision") == JOINT_METHOD_REVISION
        and freeze.get("retained_checkpoint_probe_passed") is True
        and tuple(freeze.get("fresh_input_seeds", ())) == JOINT_RECOVERY_INPUT_SEEDS
        and freeze.get("initializer_grid_points_each") == BALANCE_GRID_POINTS
        and freeze.get("initial_final_nfev_cap") == MAX_NFEV
        and freeze.get("single_checkpoint_resume_nfev_cap") == CHECKPOINT_RESUME_NFEV
        and freeze.get("relative_recovery_limits") == RECOVERY_LIMITS
        and freeze.get("selection_or_truth_used_for_resume") is False)


def true_model():
    return FamilyModel(a=.1, viscous=.06, coulomb_negative=.11, coulomb_positive=.12,
        static_negative=.16, static_positive=.18, stribeck_negative=.07, stribeck_positive=.05,
        friction="stribeck", load_offset=.02, q_min=-20., q_max=20., actuator_gain=1., actuator_bias=0.,
        transport_delay=.0087, gyro_bias=0., gyro_tau=.015, gyro_delay=.0043,
        current_gain=1., current_bias=0., current_tau=.012, current_delay=.0029, max_step=.000125)


def schedule(seed):
    """Input seed changes dwell lengths/current levels, not merely sensor noise."""
    rng = np.random.default_rng(seed)
    tx_t, tx_A = [-.1], [0.]
    now = .30
    for level, dwell in zip(LEVELS, HOLDS, strict=True):
        tx_t.append(now)
        tx_A.append(level+rng.uniform(-.002, .002) if level else 0.)
        now += dwell*rng.uniform(.90, 1.10)
    tx_t.append(now)
    tx_A.append(0.)
    return np.asarray(tx_t), np.asarray(tx_A), now+.60


def coverage(truth, t, events):
    velocity = truth[:, 1]
    low_positive = int(np.sum((velocity > 1e-4) & (velocity < .1)))
    low_negative = int(np.sum((velocity < -1e-4) & (velocity > -.1)))
    rest = int(np.sum(truth[:, 5] > .5))
    slide = int(np.sum(truth[:, 5] < .5))
    zeros = sum(event["kind"] == "zero_crossing" for event in events)
    directions = sorted({event["direction"] for event in events if event["kind"] == "breakaway"})
    return {"low_speed_positive_samples": low_positive, "low_speed_negative_samples": low_negative,
        "rest_samples": rest, "sliding_samples": slide, "zero_crossings": zeros,
        "breakaway_directions": directions, "duration_s": float(t[-1]),
        "passed": low_positive >= 20 and low_negative >= 20 and rest >= 100 and slide >= 100
                  and zeros >= 2 and directions == [-1, 1]}


def generate_truth(seed):
    tx_t, tx_A, duration = schedule(seed)
    t = np.arange(int(np.ceil(duration/.001))+1)*.001
    oracle = independent_rollout(true_model(), t, tx_t, tx_A, np.zeros(5), max_step=.0005)
    return {"seed": seed, "t": t, "tx_t": tx_t, "tx_A": tx_A, "truth": oracle.trace,
            "oracle_diagnostics": oracle.diagnostics, "events": oracle.events,
            "coverage": coverage(oracle.trace, t, oracle.events)}


def observations(data, noisy):
    rng = np.random.default_rng(data["seed"]+70001)
    truth, t = data["truth"], data["t"]
    q, current = truth[:, 0].copy(), truth[:, 4].copy()
    mask_v = np.arange(len(t)) % 20 == 0
    gyro = np.full(len(t), np.nan)
    gyro[mask_v] = truth[mask_v, 3]
    if noisy:
        q = np.round((q+rng.normal(0, NOISE["encoder_sigma_rad"], len(t)))/QUANTUM)*QUANTUM
        gyro[mask_v] += rng.normal(0, NOISE["gyro_sigma_rad_s"], int(mask_v.sum()))
        current += rng.normal(0, NOISE["current_sigma_A"], len(t))
    label = f"stribeck-input{data['seed']}-{'noisy' if noisy else 'pristine'}"
    sigma = {"sigma_q": float(np.sqrt(NOISE["encoder_sigma_rad"]**2+QUANTUM**2/12)) if noisy else 1e-5,
             "sigma_v": NOISE["gyro_sigma_rad_s"] if noisy else 1e-4,
             "sigma_current": NOISE["current_sigma_A"] if noisy else 1e-4}
    run = FamilyRun(run_id=label, source_id="independent-DOP853-stribeck/"+label,
        t=t, q=q, v=gyro, current=current, q_new=np.ones(len(t), bool), v_new=mask_v,
        current_new=np.ones(len(t), bool), tx_t=data["tx_t"], tx_A=data["tx_A"], initial=np.zeros(5),
        provenance="SYNTHETIC", configuration_id="known-stribeck-fixed-nuisances",
        calibration_revision="known-synthetic-maps", encoder_quantum=QUANTUM if noisy else 0., **sigma).validate()
    return run


def errors(run, prediction):
    return {"q_rms_rad": float(np.sqrt(np.mean((prediction[run.q_new, 0]-run.q[run.q_new])**2))),
        "gyro_rms_rad_s": float(np.sqrt(np.mean((prediction[run.v_new, 3]-run.v[run.v_new])**2))),
        "current_rms_A": float(np.sqrt(np.mean((prediction[run.current_new, 4]-run.current[run.current_new])**2)))}


def trajectory_gate(run, prediction):
    e = errors(run, prediction)
    limits = {"q_rms_rad": max(3*run.sigma_q, NUMERICAL_GATES["q_rms_rad"]),
              "gyro_rms_rad_s": max(3*run.sigma_v, NUMERICAL_GATES["gyro_rms_rad_s"]),
              "current_rms_A": 3*run.sigma_current}
    original_limits = {"q_rms_rad": float(np.deg2rad(.15)),
        "gyro_rms_rad_s": max(float(np.deg2rad(.5)), .1*float(np.mean(np.abs(run.v[run.v_new]))))}
    return {"errors": e, "synthetic_3sigma_limits": limits, "original_whole_run_limits": original_limits,
        "passed": all(e[key] <= limit for key, limit in limits.items())
                  and all(e[key] <= limit for key, limit in original_limits.items())}


def numerical_gate(native, data):
    run = observations(data, False)
    prediction = native.rollout(true_model(), run.t, run.tx_t, run.tx_A, run.initial)
    e = errors(run, prediction)
    return {"errors": e, "limits": NUMERICAL_GATES,
            "passed": all(e[key] <= limit for key, limit in NUMERICAL_GATES.items())}


def retained_observation_run(path, *, noisy):
    """Read a declared fixture acquisition, never its latent oracle states."""
    with np.load(path, allow_pickle=False) as archive:
        data = {key: archive[key] for key in ("t", "q", "v", "current", "q_new", "v_new", "current_new", "tx_t", "tx_A", "initial")}
    sigma = {"sigma_q": float(np.sqrt(NOISE["encoder_sigma_rad"]**2+QUANTUM**2/12)) if noisy else 1e-5,
             "sigma_v": NOISE["gyro_sigma_rad_s"] if noisy else 1e-4,
             "sigma_current": NOISE["current_sigma_A"] if noisy else 1e-4}
    label = path.name.removesuffix("-observations.npz")
    return FamilyRun(run_id=label, source_id="retained-independent-DOP853-stribeck/"+label,
        provenance="SYNTHETIC", configuration_id="known-stribeck-fixed-nuisances",
        calibration_revision="known-synthetic-maps", encoder_quantum=QUANTUM if noisy else 0.,
        **data, **sigma).validate()


def stribeck_moving_intervals(model, run):
    """Initializer-only reconstructed velocity; final hybrid observations stay raw."""
    if model.actuator != "algebraic" or model.friction != "stribeck" or model.load != "constant":
        raise ValueError("moving-balance scope requires fixed algebraic input/Stribeck/constant-load nuisances")
    times = run.t[run.v_new]-model.gyro_delay
    gyro = run.v[run.v_new]-model.gyro_bias
    if len(times) < 9:
        raise ValueError("INSUFFICIENT_NATIVE_GYRO")
    spacing = float(np.median(np.diff(times)))
    if spacing <= 0 or np.max(np.abs(np.diff(times)-spacing)) > max(1e-8, .01*spacing):
        raise ValueError("INSUFFICIENT_UNIFORM_NATIVE_GYRO")
    window = max(5, int(round(.14/spacing)) | 1)
    if window > len(times):
        raise ValueError("INSUFFICIENT_NATIVE_GYRO_FOR_DECLARED_WINDOW")
    smooth = savgol_filter(gyro, window, 3, mode="interp")
    derivative = savgol_filter(gyro, window, 3, deriv=1, delta=spacing, mode="interp")
    velocity = smooth+model.gyro_tau*derivative
    command_times = run.tx_t+model.transport_delay
    q_times, q_values = run.t[run.q_new], run.q[run.q_new]
    minimum_speed = max(.012, 3*run.sigma_v)
    intervals = []
    for count in (max(3, int(round(duration/spacing))) for duration in BALANCE_INTERVALS_S):
        for start in range(window//2, len(times)-count-window//2, 2):
            end = start+count
            section = velocity[start:end+1]
            if times[start] < max(q_times[0], command_times[0]) or times[end] > q_times[-1] or not (
                    np.all(section > minimum_speed) or np.all(section < -minimum_speed)):
                continue
            direction = 1 if section[0] > 0 else -1
            inside = command_times[(command_times > times[start]) & (command_times < times[end])]
            edges = np.r_[times[start], inside, times[end]]
            held = np.searchsorted(command_times, edges[:-1]+1e-13, side="right")-1
            if np.any(held < 0):
                raise ValueError("UNSUPPORTED_INPUT_PREHISTORY")
            input_integral = float(np.diff(edges) @ (model.actuator_gain*run.tx_A[held]+model.actuator_bias))
            intervals.append({"times": times[start:end+1], "velocity": section, "direction": direction,
                "duration": float(times[end]-times[start]), "delta_v": float(velocity[end]-velocity[start]),
                "delta_q": float(np.interp(times[end], q_times, q_values)-np.interp(times[start], q_times, q_values)),
                "input_minus_load": input_integral-model.load_offset*(times[end]-times[start])})
    directions = {row["direction"] for row in intervals}
    if len(intervals) < 4 or directions != {-1, 1}:
        raise ValueError("INSUFFICIENT_BOTH_DIRECTION_MOVING_EQUATIONS")
    metadata = {"equations": len(intervals), "negative_equations": sum(row["direction"] < 0 for row in intervals),
        "positive_equations": sum(row["direction"] > 0 for row in intervals), "gyro_sample_spacing_s": spacing,
        "gyro_smoothing_window_samples": window, "gyro_smoothing_span_s": (window-1)*spacing,
        "minimum_abs_speed_rad_s": minimum_speed, "moving_intervals_s": BALANCE_INTERVALS_S,
        "endpoint_extrapolation": False, "filter_derivative_used_for_final_residuals": False,
        "state_reset": False, "selection_or_holdout_used": False}
    return intervals, metadata


def integrated_stribeck_initializer(native, model, run, bounds):
    """Profile fixed Vs grid; choose only by native TRAIN observations."""
    began = time.monotonic()
    intervals, metadata = stribeck_moving_intervals(model, run)
    fields = ("a", "viscous", "coulomb_negative", "coulomb_positive")
    lower, upper = ([bounds[key][index] for key in fields] for index in (0, 1))
    grids = {key: np.linspace(*bounds[key], BALANCE_GRID_POINTS) for key in SPEED_BOUNDS}
    candidates, selected = [], None
    for negative in grids["stribeck_negative"]:
        for positive in grids["stribeck_positive"]:
            matrix, rhs = [], []
            for row in intervals:
                direction = row["direction"]
                speed, static = (negative, model.static_negative) if direction < 0 else (positive, model.static_positive)
                exponential_integral = float(np.trapezoid(np.exp(-(np.abs(row["velocity"])/speed)**model.stribeck_power), row["times"]))
                drag_interval = row["duration"]-exponential_integral
                matrix.append([row["delta_v"], row["delta_q"], -drag_interval if direction < 0 else 0.,
                               drag_interval if direction > 0 else 0.])
                rhs.append(row["input_minus_load"]-direction*static*exponential_integral)
            matrix, rhs = np.asarray(matrix), np.asarray(rhs)
            norms = np.linalg.norm(matrix, axis=0)
            normalized = matrix/np.maximum(norms, 1e-30)
            rank = int(np.linalg.matrix_rank(normalized))
            record = {"stribeck_negative": float(negative), "stribeck_positive": float(positive),
                      "matrix_rank": rank, "matrix_normalized_condition": float(np.linalg.cond(normalized))}
            if rank < 4:
                record["status"] = "INSUFFICIENT_MECHANICS_RANK"
                candidates.append(record)
                continue
            solution = lsq_linear(matrix, rhs, bounds=(lower, upper))
            record.update(linear_solver_success=bool(solution.success), linear_solver_iterations=int(solution.nit),
                moving_equation_rms_A_s=float(np.sqrt(np.mean((matrix@solution.x-rhs)**2))))
            if not solution.success:
                record["status"] = "BOUNDED_LINEAR_SOLVER_FAILED"
                candidates.append(record)
                continue
            values = dict(zip(fields, map(float, solution.x)))
            candidate = replace(model, **values, stribeck_negative=float(negative), stribeck_positive=float(positive))
            record["mechanics_coordinates"] = values
            try:
                pred = native.rollout(candidate, run.t, run.tx_t, run.tx_A, run.initial)
                eq, ev, ei = _errors(run, pred)
                residual = np.r_[eq/run.sigma_q, ev/run.sigma_v, ei/run.sigma_current]
                absolute = np.abs(residual)
                cost = float(np.sum(np.where(absolute <= 1., .5*residual**2, absolute-.5)))
                record.update(status="NATIVE_TRAIN_OBJECTIVE", native_train_huber_cost=cost,
                              native_train_errors=errors(run, pred))
                if selected is None or cost < selected[0]:
                    selected = (cost, candidate, len(candidates))
            except Rejected as exc:
                record.update(status="NATIVE_TRAIN_INFEASIBLE", reason=str(exc))
            candidates.append(record)
    if selected is None:
        raise ValueError("NO_VALID_NATIVE_TRAIN_BALANCE_SEED")
    metadata.update(policy="fixed bounded Vs grid and constrained linear mechanics; choose native TRAIN Huber cost",
        scope="six mechanics/Stribeck coordinates; supplied static/input/filter/load nuisances",
        grids={key: list(map(float, grid)) for key, grid in grids.items()}, candidates=candidates,
        selected_index=selected[2], selected_huber_cost=selected[0], selected_model=selected[1].document(),
        native_candidate_evaluations=sum(row["status"].startswith("NATIVE_TRAIN_") for row in candidates),
        native_candidate_budget=BALANCE_GRID_POINTS**2, bounded_linear_solves=sum("linear_solver_success" in row for row in candidates),
        elapsed_s=time.monotonic()-began, truth_used_to_choose_seed=False, physical_identifiability=False,
        smooth_switch_acceptance=False, acceptance_uses_initializer=False)
    return selected[1], metadata


def checkpoint_tail_diagnostics(native, report, run):
    """Inspect TRAIN descent before a single documented checkpoint extension."""
    began = time.monotonic()
    optimizer = report["optimizer"]
    model = FamilyModel(**{field.name: report["fitted_model"][field.name] for field in fields(FamilyModel)})
    tail = optimizer["accepted_coordinate_steps"][-8:]
    rows = []
    for step in tail:
        candidate = replace(model, **dict(zip(JOINT_BOUNDS, step["coordinates"])))
        pred = native.rollout(candidate, run.t, run.tx_t, run.tx_A, run.initial)
        eq, ev, ei = _errors(run, pred)
        residual = np.r_[eq/run.sigma_q, ev/run.sigma_v, ei/run.sigma_current]
        absolute = np.abs(residual)
        rows.append({"coordinates": step["coordinates"], "characteristic_step_norm": step["characteristic_step_norm"],
            "native_train_huber_cost": float(np.sum(np.where(absolute <= 1., .5*residual**2, absolute-.5)))})
    coherent = len(rows) == 8 and all(rows[index+1]["native_train_huber_cost"] < rows[index]["native_train_huber_cost"] for index in range(7))
    reduction = 1-rows[-1]["native_train_huber_cost"]/rows[0]["native_train_huber_cost"] if rows and rows[0]["native_train_huber_cost"] > 0 else 0.
    recent_steps = [row["characteristic_step_norm"] for row in rows[-3:] if row["characteristic_step_norm"] is not None]
    median_step = float(np.median(recent_steps)) if recent_steps else 0.
    allowed = optimizer["termination_status"] == 0 and optimizer["evaluations"] == MAX_NFEV and coherent and reduction > .10 and median_step > 1e-5
    return {"rows": rows, "native_diagnostic_rollouts": len(rows), "coherent_cost_descent": coherent,
        "tail_cost_reduction_fraction": float(reduction), "median_last_three_characteristic_step": median_step,
        "saved_final_optimality": optimizer["optimality"], "resume_allowed": bool(allowed),
        "resume_rule": "status0 at120; eight strictly descending TRAIN costs; cost reduction>10%; median last3 step>1e-5",
        "selection_or_truth_used": False, "elapsed_s": time.monotonic()-began}


def fit_one(native, run, targets, *, scope, output, noisy, joint_strategy="direct", checkpoint_model=None):
    bounds = SPEED_BOUNDS if scope == "speeds" else JOINT_BOUNDS
    if checkpoint_model is not None and (scope != "joint" or joint_strategy != "direct"):
        raise ValueError("checkpoint resume requires joint scope with the grid/warm-up initializer disabled")
    supplied_initial = checkpoint_model if checkpoint_model is not None else replace(true_model(), **(INITIAL if scope == "speeds" else JOINT_INITIAL))
    phase_budget = CHECKPOINT_RESUME_NFEV if checkpoint_model is not None else MAX_NFEV
    initial, remaining_budget, initializer_stage = supplied_initial, phase_budget, None
    began = time.monotonic()
    if joint_strategy == "mechanics-warmup":
        if scope != "joint":
            raise ValueError("mechanics warm-up is only declared for the joint diagnostic")
        mechanics_bounds = {key: bound for key, bound in JOINT_BOUNDS.items() if key not in SPEED_BOUNDS}
        warmup_began = time.monotonic()
        warmup = fit_family(native, supplied_initial, [run], bounds=mechanics_bounds, max_nfev=30)
        initial = warmup["model"]
        remaining_budget -= warmup["optimizer"]["evaluations"]
        initializer_stage = {"policy": "training-only mechanics block at supplied nontruth Stribeck speed scales",
            "free_coordinates": list(mechanics_bounds), "speed_coordinates_fixed": INITIAL,
            "model": initial.document(), "optimizer": warmup["optimizer"], "budget_cap": 30,
            "elapsed_s": time.monotonic()-warmup_began,
            "selection_or_holdout_used": False, "acceptance_uses_warmup": False}
    elif joint_strategy == "moving-balance-grid":
        if scope != "joint":
            raise ValueError("moving-balance initialization is only declared for the joint diagnostic")
        initial, initializer_stage = integrated_stribeck_initializer(native, supplied_initial, run, bounds)
    final_fit_began = time.monotonic()
    fit = fit_family(native, initial, [run], bounds=bounds, max_nfev=remaining_budget)
    final_fit_elapsed = time.monotonic()-final_fit_began
    fitted, optimizer = fit["model"], fit["optimizer"]
    warm_optimizer = (initializer_stage or {}).get("optimizer")
    relative = {key: (getattr(fitted, key)/getattr(true_model(), key)-1) for key in bounds}
    recovery = parameter_recovery_gate(relative, optimizer["parameter_bound_hits"], noisy=noisy)
    predictions = []
    for target in [run, *targets]:
        pred = native.rollout(fitted, target.t, target.tx_t, target.tx_A, target.initial)
        np.savez_compressed(output/f"{run.run_id}-predict-{target.run_id}.npz", t=target.t, prediction=pred,
                            q_new=target.q_new, v_new=target.v_new, current_new=target.current_new)
        predictions.append({"run_id": target.run_id, "role": "training_whole_run" if target is run else "selection_whole_run",
                            **trajectory_gate(target, pred)})
    report = {"training_run": run.run_id, "scope": scope, "free_coordinates": list(bounds),
        "supplied_initial_model": supplied_initial.document(), "initial_model": initial.document(),
        "initializer_stage": initializer_stage, "joint_strategy": joint_strategy,
        "phase": "checkpoint-resume" if checkpoint_model is not None else "initial-final-fit",
        "checkpoint_grid_initializer_disabled": checkpoint_model is not None,
        "initializer_native_candidate_evaluations": (initializer_stage or {}).get("native_candidate_evaluations", 0),
        "initializer_native_candidate_budget_separate": (initializer_stage or {}).get("native_candidate_budget", 0),
        "final_native_optimizer_budget": remaining_budget,
        "native_optimizer_evaluations_total": phase_budget-remaining_budget+optimizer["evaluations"],
        "native_residual_evaluations_total": optimizer["residual_evaluations_including_jacobian_and_diagnostics"]
            + (warm_optimizer["residual_evaluations_including_jacobian_and_diagnostics"] if warm_optimizer else 0),
        "native_jacobian_evaluations_total": optimizer["jacobian_evaluations"]
            + (warm_optimizer["jacobian_evaluations"] if warm_optimizer else 0),
        "final_fit_elapsed_s": final_fit_elapsed,
        "native_optimizer_budget_total": phase_budget,
        "fitted_model": fitted.document(), "optimizer": optimizer,
        "relative_parameter_error": relative, "observation_class": "noisy" if noisy else "pristine",
        "recovery_limit": recovery["limit"], "recovery_passed": recovery["passed"],
        "training_native_quality_passed": all(row["passed"] for row in fit["training"]),
        "predictions": predictions, "elapsed_s": time.monotonic()-began,
        "selection_in_fit_or_initialization": False, "smooth_sign_substitution": False,
        "physical_qualification": "NOT_RUN", "deployment_authorized": False}
    report["passed"] = bool(optimizer["success"] and report["recovery_passed"] and report["training_native_quality_passed"]
                            and all(row["passed"] for row in predictions))
    save(output/f"{run.run_id}-fit.json", report)
    print(json.dumps({"case": run.run_id, "scope": scope, "relative_error": relative,
                      "optimizer_success": optimizer["success"], "nfev": optimizer["evaluations"], "passed": report["passed"]}), flush=True)
    return report


def fit_joint_procedure(native, run, targets, *, output, noisy):
    """Frozen third strategy: one grid, native120, one conditional native40."""
    initial = fit_one(native, run, targets, scope="joint", output=output, noisy=noisy,
                      joint_strategy="moving-balance-grid")
    if initial["optimizer"]["termination_status"] != 0:
        initial["checkpoint_resume"] = {"used": False, "reason": "initial optimizer terminated"}
        return initial
    diagnostic = checkpoint_tail_diagnostics(native, initial, run)
    save(output/f"{run.run_id}-tail.json", diagnostic)
    if not diagnostic["resume_allowed"]:
        initial["checkpoint_resume"] = {"used": False, "reason": "TRAIN tail does not authorize coherent checkpoint extension", "diagnostic": diagnostic}
        return initial
    checkpoint = FamilyModel(**{field.name: initial["fitted_model"][field.name] for field in fields(FamilyModel)})
    save(output/f"{run.run_id}-checkpoint.json", {"model": checkpoint.document(), "run_initial": run.initial.tolist(),
        "initial_fit_evidence": f"{run.run_id}-fit.json", "tail_evidence": f"{run.run_id}-tail.json"})
    resume_dir = output/f"{run.run_id}-resume"
    resume_dir.mkdir()
    resumed = fit_one(native, run, targets, scope="joint", output=resume_dir, noisy=noisy,
                      joint_strategy="direct", checkpoint_model=checkpoint)
    combined = dict(resumed)
    combined.update(joint_strategy="moving-balance-grid", method_revision=JOINT_METHOD_REVISION,
        initializer_stage=initial["initializer_stage"], original_phase_optimizer=initial["optimizer"],
        original_phase_failed_preserved=True, initial_fit_evidence=f"{run.run_id}-fit.json",
        initializer_native_candidate_evaluations=initial["initializer_native_candidate_evaluations"],
        initializer_native_candidate_budget_separate=BALANCE_GRID_POINTS**2,
        native_optimizer_evaluations_total=initial["native_optimizer_evaluations_total"]+resumed["native_optimizer_evaluations_total"],
        native_residual_evaluations_total=initial["native_residual_evaluations_total"]+resumed["native_residual_evaluations_total"],
        native_jacobian_evaluations_total=initial["native_jacobian_evaluations_total"]+resumed["native_jacobian_evaluations_total"],
        native_optimizer_budget_total=MAX_NFEV+CHECKPOINT_RESUME_NFEV,
        checkpoint_tail_diagnostic_native_rollouts=diagnostic["native_diagnostic_rollouts"],
        checkpoint_resume={"used": True, "budget": CHECKPOINT_RESUME_NFEV, "diagnostic": diagnostic,
                           "initialization_disabled": True, "single_extension": True},
        elapsed_s=initial["elapsed_s"]+diagnostic["elapsed_s"]+resumed["elapsed_s"])
    save(output/f"{run.run_id}-combined-fit.json", combined)
    return combined


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--partition", choices=("development", "final", "joint-fresh"), default="development")
    parser.add_argument("--scope", choices=("speeds", "joint"), default="speeds")
    parser.add_argument("--probe-only", action="store_true")
    parser.add_argument("--narrow-evidence", type=Path)
    parser.add_argument("--joint-strategy", choices=("direct", "mechanics-warmup", "moving-balance-grid"), default="direct")
    parser.add_argument("--joint-method-freeze", type=Path)
    args = parser.parse_args()
    if args.joint_strategy != "direct" and args.scope != "joint":
        parser.error("a joint strategy requires --scope joint")
    if args.partition == "joint-fresh":
        if args.scope != "joint" or args.joint_strategy != "moving-balance-grid" or args.joint_method_freeze is None:
            parser.error("joint-fresh requires the frozen moving-balance joint procedure")
        if not joint_method_ready(json.loads(args.joint_method_freeze.read_text(encoding="utf-8"))):
            parser.error("joint-fresh method freeze is incomplete or differs from the declared procedure")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("retain all prior evidence; select a fresh output directory")
    if args.scope == "joint":
        if args.narrow_evidence is None:
            parser.error("joint mechanics+Stribeck requires a passing narrow diagnostic first")
        narrow = json.loads(args.narrow_evidence.read_text(encoding="utf-8"))
        narrow_contract = json.loads((args.narrow_evidence.parent/"predeclared-contract.json").read_text(encoding="utf-8"))
        if not narrow_development_ready(narrow, narrow_contract):
            parser.error("joint scope requires full pristine/noisy two-speed development with whole-run selection")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    seeds = {"development": DEVELOPMENT_INPUT_SEEDS, "final": FINAL_INPUT_SEEDS, "joint-fresh": JOINT_RECOVERY_INPUT_SEEDS}[args.partition]
    if args.probe_only:
        seeds = seeds[:1]
    bounds = SPEED_BOUNDS if args.scope == "speeds" else JOINT_BOUNDS
    contract = {"schema": "adr0022.stribeck-estimator-verification/1", "provenance": "SYNTHETIC",
        "partition": args.partition, "scope": args.scope, "probe_only": args.probe_only,
        "development_input_seeds": DEVELOPMENT_INPUT_SEEDS, "final_input_seeds": FINAL_INPUT_SEEDS,
        "fresh_joint_input_seeds": JOINT_RECOVERY_INPUT_SEEDS,
        "active_train_input_seed": seeds[0], "active_selection_input_seeds": seeds[1:],
        "input_algorithm": {"levels_A": LEVELS, "nominal_dwells_s": HOLDS,
            "level_jitter_A": [-.002, .002], "dwell_factors": [.90, 1.10], "baseline_s": .30, "final_hold_s": .60},
        "input_seed_changes_motion": True, "true_model": asdict(true_model()),
        "initial_coordinates": INITIAL if args.scope == "speeds" else JOINT_INITIAL, "bounds": bounds,
        "known_nuisance_coordinates": [key for key in asdict(true_model()) if key not in bounds],
        "optimizer_budget": MAX_NFEV, "relative_recovery_limits": RECOVERY_LIMITS,
        "sensor_noise": NOISE, "encoder_quantum_rad": QUANTUM,
        "quality_limits": "unchanged original 3-sigma numerical/sensor and 0.15deg angle / max(0.5deg/s,10% mean|v|) whole-run gates",
        "numerical_gates": NUMERICAL_GATES, "native_masks": "1kHz encoder/current; 50Hz gyro only, no interpolated residuals",
        "oracle": "independent hybrid DOP853 with explicit sticking, breakaway and zero crossings",
        "narrow_prerequisite": str(args.narrow_evidence) if args.narrow_evidence else None,
        "strategy_branch": "bounded whole-run hybrid output error with repaired central derivative",
        "joint_strategy": args.joint_strategy,
        "joint_method_revision": JOINT_METHOD_REVISION if args.joint_strategy == "moving-balance-grid" else None,
        "joint_method_freeze": str(args.joint_method_freeze) if args.joint_method_freeze else None,
        "initializer_grid_native_candidate_budget_separate": BALANCE_GRID_POINTS**2 if args.joint_strategy == "moving-balance-grid" else 0,
        "single_checkpoint_resume_budget": CHECKPOINT_RESUME_NFEV if args.joint_strategy == "moving-balance-grid" else 0,
        "joint_warmup_budget_cap": 30 if args.joint_strategy == "mechanics-warmup" else 0,
        "budget_accounting": "grid49 native candidate calls separate; native120 plus one authorized checkpoint max40; all actual work reported" if args.joint_strategy == "moving-balance-grid" else "warm-up and final native optimizer evaluations together cannot exceed120",
        "strategy_review_limit": 3, "automatic_retry": False, "physical_actions": False}
    save(args.output_dir/"predeclared-contract.json", contract)
    native = FamilyNative(args.library)
    all_truth, numerics = [], []
    for seed in seeds:
        data = generate_truth(seed)
        all_truth.append(data)
        numerical = numerical_gate(native, data)
        numerics.append({"input_seed": seed, "coverage": data["coverage"], "numerical_gate": numerical,
                         "oracle_diagnostics": data["oracle_diagnostics"], "events": data["events"]})
        np.savez_compressed(args.output_dir/f"input-{seed}-independent.npz", t=data["t"], tx_t=data["tx_t"],
                            tx_A=data["tx_A"], initial=np.zeros(5), truth=data["truth"])
        save(args.output_dir/"generation.json", numerics)
        print(json.dumps({"generated_input_seed": seed, "coverage": data["coverage"], "numerical": numerical}), flush=True)
    cases = []
    for noisy in ((True,) if args.probe_only else (False, True)):
        runs = [observations(data, noisy) for data in all_truth]
        for run in runs:
            np.savez_compressed(args.output_dir/f"{run.run_id}-observations.npz", t=run.t, q=run.q, v=run.v,
                current=run.current, q_new=run.q_new, v_new=run.v_new, current_new=run.current_new,
                tx_t=run.tx_t, tx_A=run.tx_A, initial=run.initial)
        if args.joint_strategy == "moving-balance-grid":
            case = fit_joint_procedure(native, runs[0], runs[1:], output=args.output_dir, noisy=noisy)
        else:
            case = fit_one(native, runs[0], runs[1:], scope=args.scope, output=args.output_dir,
                           noisy=noisy, joint_strategy=args.joint_strategy)
        cases.append(case)
        save(args.output_dir/"cases.json", cases)
    passed = all(case["passed"] for case in cases) and all(row["coverage"]["passed"] and row["numerical_gate"]["passed"] for row in numerics)
    summary = {"schema": contract["schema"], "partition": args.partition, "scope": args.scope,
        "probe_only": args.probe_only, "probe_viability_passed": passed if args.probe_only else None,
        "narrow_diagnostic_passed": passed and not args.probe_only if args.scope == "speeds" else None,
        "synthetic_scope_passed": passed, "free_coordinate_count": len(bounds),
        "cases": [{key: case[key] for key in ("training_run", "passed", "relative_parameter_error", "elapsed_s")} for case in cases],
        "strategy_branch": args.joint_strategy,
        "strategy_branches_evaluated": {"direct": 1, "mechanics-warmup": 2, "moving-balance-grid": 3}[args.joint_strategy],
        "decision": "SYNTHETIC_SCOPE_PASS" if passed else "DIAGNOSE_FIRST_FAILURE_NO_PROMOTION",
        "full_family_qualification": "NOT_RUN", "closed_loop_feedback_bias": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(args.output_dir/"summary.json", summary)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
