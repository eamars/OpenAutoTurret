"""Offline training-only diagnosis of a retained hybrid synthetic fit failure."""
from __future__ import annotations
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyNative, _errors, fit_family
from Firmware.commissioning import model_family
from Firmware.tools.adr0022_closed_loop_estimator_probe import (
    estimator_model, FREE_BOUNDS, FIXTURE, measurement_errors, trajectory_gate)
from Firmware.tools.adr0022_seed41_diagnose import run_from_case, save


def integral_initializer(run, model, bounds):
    """Training-only moving integral equations; final score is whole-run OE."""
    times = run.t[run.v_new]-model.gyro_delay
    gyro = run.v[run.v_new]-model.gyro_bias
    spacing = float(np.median(np.diff(times)))
    window = max(5, int(round(.22/spacing)) | 1)
    smooth = savgol_filter(gyro, window, 3, mode="interp")
    derivative = savgol_filter(gyro, window, 3, deriv=1, delta=spacing, mode="interp")
    velocity = smooth + model.gyro_tau*derivative
    # Exact ZOH integral, not noisy reported motor-current observations.
    command_times = run.tx_t+model.transport_delay
    def input_integral(start, end):
        inside = command_times[(command_times > start) & (command_times < end)]
        edges = np.r_[start, inside, end]
        held = np.searchsorted(command_times, edges[:-1]+1e-13, side="right")-1
        return float(np.diff(edges) @ (model.actuator_gain*run.tx_A[held]+model.actuator_bias))
    q_times, q_values = run.t[run.q_new], run.q[run.q_new]
    matrix, rhs = [], []
    for count in (max(3, int(round(duration/spacing))) for duration in (.12, .2, .4)):
        for start in range(0, len(times)-count, 3):
            end = start+count
            section = velocity[start:end+1]
            if times[start] < run.t[0] or not (np.all(section > .05) or np.all(section < -.05)):
                continue
            direction = 1 if section[0] > 0 else -1
            duration = times[end]-times[start]
            delta_q = np.interp(times[end], q_times, q_values)-np.interp(times[start], q_times, q_values)
            matrix.append([velocity[end]-velocity[start], delta_q,
                -duration if direction < 0 else 0., duration if direction > 0 else 0.])
            rhs.append(input_integral(times[start],times[end])-model.load_offset*duration)
    matrix, rhs = np.asarray(matrix), np.asarray(rhs)
    fields = ["a", "viscous", "coulomb_negative", "coulomb_positive"]
    result = lsq_linear(matrix, rhs, bounds=([bounds[key][0] for key in fields],
                                           [bounds[key][1] for key in fields]))
    values = dict(zip(fields, map(float, result.x)))
    return replace(model, **values), {"parameters":values,"equations":len(rhs),
        "matrix_rank":int(np.linalg.matrix_rank(matrix)),"matrix_condition":float(np.linalg.cond(matrix)),
        "equation_residual_rms_A_s":float(np.sqrt(np.mean((matrix @ result.x-rhs)**2))),
        "window_s":.22,"moving_interval_s":[.12,.2,.4],"minimum_abs_velocity_rad_s":.05,
        "training_only":True,"physical_identifiability":False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--derivative-factor", type=float, default=1.)
    parser.add_argument("--initialize-all-source-cases", action="store_true")
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("fresh empty output required")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.derivative_factor != 1.:
        original_jacobian = model_family._bounded_central_jacobian
        # Isolated diagnostic intervention in this offline process only. The
        # production fit function/default is unchanged, and records distinguish
        # actual perturbed physical steps from its default metadata.
        model_family._bounded_central_jacobian = lambda objective, x, lower, upper, steps: \
            original_jacobian(objective, x, lower, upper, steps*args.derivative_factor)
    with np.load(args.case, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    data["noisy"] = True
    run = run_from_case(args.case.stem, data)
    native = FamilyNative(args.library)
    truth = estimator_model()
    fields = list(FREE_BOUNDS)
    seed = replace(truth, a=.085, viscous=.075, coulomb_negative=.105, coulomb_positive=.13)
    saved = json.loads(args.checkpoint.read_text(encoding="utf-8"))
    saved_model = saved.get("fitted_model", saved.get("model"))
    point = np.array([saved_model[key] for key in fields])
    def candidate(x, step=.00025):
        return replace(truth, max_step=step, **dict(zip(fields, x)))
    def rollout(x, step=.00025):
        return native.rollout(candidate(x, step), run.t, run.tx_t, run.tx_A, run.initial)
    def residual(pred):
        eq, ev, ei = _errors(run, pred)
        return np.r_[eq/run.sigma_q, ev/run.sigma_v, ei/run.sigma_current]
    def cost(pred):
        r = residual(pred)
        return float(np.sum(np.where(np.abs(r) <= 1, .5*r*r, np.abs(r)-.5)))
    def event_times(pred):
        return run.t[1:][np.diff(pred[:, 5]) != 0]
    starts = {"original": [.085, .075, .105, .13],
              "inertial-start": [.12, .04, .10, .14],
              "drag-start": [.075, .09, .14, .09],
              "retained-point": point.tolist()}
    integral_seed, integral_metadata = integral_initializer(run, seed, FREE_BOUNDS)
    starts["moving-integral-start"] = [getattr(integral_seed,key) for key in fields]
    contract = {"fitting_data": str(args.case), "checkpoint": str(args.checkpoint),
        "start_set": starts, "bounds": FREE_BOUNDS, "budget_per_start":120,
        "loss": "huber", "selection_rule": "TRAINING_OBJECTIVE_ONLY; no cross-runs inspected to pick a fit",
        "diagnostic_jacobian_step_multiplier":args.derivative_factor,
        "partition_role": "DEVELOPMENT; failure observed in former verification partition",
        "quality_gates": {"relative_parameter":.05,"q_rms_rad":.0008023127185209158},
        "provenance": "SYNTHETIC", "physical_actions":False}
    save(args.output_dir / "predeclared-diagnosis.json", contract)
    save(args.output_dir / "moving-integral-initializer.json", integral_metadata)
    if args.initialize_all_source_cases:
        results = []
        paths = sorted(path for path in args.case.parent.glob("*.npz")
                       if "-predict-" not in path.stem and path.stem.startswith(
                           ("original_multisine-seed-", "quintic_hold_reverse-seed-", "chirp-seed-")))
        for path in paths:
            with np.load(path, allow_pickle=False) as archive:
                source = {key:archive[key] for key in archive.files}
            source["noisy"] = True
            source_run = run_from_case(path.stem, source)
            initialized, metadata = integral_initializer(source_run, seed, FREE_BOUNDS)
            fit = fit_family(native, initialized, [source_run], bounds=FREE_BOUNDS, max_nfev=120)
            fitted = fit.pop("model")
            pred = native.rollout(fitted,source_run.t,source_run.tx_t,source_run.tx_A,source_run.initial)
            errors = measurement_errors(source,pred)
            result = {"case":path.stem,"initializer":metadata,"model":fitted.document(),
                "optimizer":fit["optimizer"],"trajectory_gate":trajectory_gate(source,errors),
                "relative_parameter_errors":{key:float(getattr(fitted,key)/FIXTURE[key]-1) for key in fields}}
            results.append(result)
            save(args.output_dir / "consumed-development-initializers.json",results)
            print(json.dumps({"case":path.stem,"passed":result["trajectory_gate"]["passed"],
                "optimizer_converged":result["optimizer"]["success"],
                "nfev":result["optimizer"]["evaluations"],"relative":result["relative_parameter_errors"]}),flush=True)
        return
    base = rollout(point)
    ladder = []
    for integration_step in (.0005, .00025, .000125, .0000625):
        pred = rollout(point, integration_step)
        ladder.append({"max_step":integration_step,"training_cost":cost(pred),
            "q_rms_change_from_retained":float(np.sqrt(np.mean((pred[:, 0]-base[:, 0])**2))),
            "events":event_times(pred).tolist()})
    derivatives = []
    for k, field in enumerate(fields):
        for h in (1e-5, 1e-6, 1e-7, 1e-8, 1e-9):
            plus, minus = point.copy(), point.copy()
            plus[k] += h; minus[k] -= h
            positive, negative = rollout(plus), rollout(minus)
            derivatives.append({"coordinate":field,"absolute_step":h,
                "raw_normalized_derivative_norm":float(np.linalg.norm((residual(positive)-residual(negative))/(2*h))),
                "cost_plus":cost(positive),"cost_minus":cost(negative),
                "q_max_jump_rad":float(np.max(np.abs(positive[:, 0]-negative[:, 0]))),
                "events_plus":event_times(positive).tolist(),"events_minus":event_times(negative).tolist()})
    save(args.output_dir / "event-derivative-ladder.json", {"integration":ladder,"derivatives":derivatives})
    results = []
    for label, values in starts.items():
        began = time.monotonic()
        fit = fit_family(native, candidate(values), [run], bounds=FREE_BOUNDS, max_nfev=120)
        model = fit.pop("model")
        pred = native.rollout(model, run.t, run.tx_t, run.tx_A, run.initial)
        errors = measurement_errors(data, pred)
        result = {"label":label,"start":values,"model":model.document(),
            "optimizer":fit["optimizer"],"errors":errors,"trajectory_gate":trajectory_gate(data, errors),
            "relative_parameter_errors":{key:float(getattr(model,key)/FIXTURE[key]-1) for key in fields},
            "wall_time_s":time.monotonic()-began,"events":event_times(pred).tolist()}
        result["actual_optimizer_absolute_steps"] = (np.asarray(fit["optimizer"]["absolute_derivative_steps"])*args.derivative_factor).tolist()
        results.append(result)
        save(args.output_dir / "training-start-results.json", results)
        print(json.dumps({key:result[key] for key in ("label","relative_parameter_errors","trajectory_gate")})+
            " "+json.dumps({key:result["optimizer"][key] for key in ("success","evaluations","cost","optimality")}), flush=True)
    save(args.output_dir / "diagnosis.json", {"contract":contract,"integration":ladder,
        "derivatives":derivatives,"starts":results,"promotion_blocked":True})


if __name__ == "__main__":
    main()
