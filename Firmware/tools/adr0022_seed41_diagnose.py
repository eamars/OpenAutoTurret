"""Offline frozen seed-41 objective/derivative diagnosis; no station operations.

Use retained successful inputs and native observations. Cross-run data score
predictions after fitting only seed 41; never choose a fit using those scores.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyNative, FamilyRun, _errors, fit_family
from Firmware.tools.adr0022_closed_loop_estimator_probe import (
    FREE_BOUNDS, FIXTURE, estimator_model, measurement_errors, sensor_scales, trajectory_gate)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_cases(source):
    cases = {}
    for seed in (17, 41, 83):
        label = f"noisy-seed-{seed}"
        with np.load(source / f"{label}.npz", allow_pickle=False) as archive:
            data = {key: archive[key] for key in archive.files}
        data["noisy"] = True
        data["seed"] = seed
        cases[label] = data
    return cases


def run_from_case(label, data):
    return FamilyRun(run_id=label, source_id=f"independent-analytic-oracle/{label}",
        t=data["t"], q=data["q"], v=data["v"], current=data["current"],
        q_new=data["q_new"], v_new=data["v_new"], current_new=data["current_new"],
        tx_t=data["tx_t"], tx_A=data["tx_A"], initial=np.zeros(5),
        provenance="SYNTHETIC", configuration_id="declared-coulomb-analytic-fixture",
        calibration_revision="known-synthetic-sensors", encoder_quantum=FIXTURE["encoder_quantum"],
        **sensor_scales(data)).validate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-nfev", type=int, default=120)
    parser.add_argument("--mode", choices=("diagnose", "replay"), default="diagnose")
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("new empty output directory required; frozen evidence is preserved")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    native = FamilyNative(args.library)
    cases = load_cases(args.source)
    data = cases["noisy-seed-41"]
    run = run_from_case("noisy-seed-41", data)
    fields = list(FREE_BOUNDS)
    lower = np.array([FREE_BOUNDS[key][0] for key in fields])
    upper = np.array([FREE_BOUNDS[key][1] for key in fields])
    initial_model = replace(estimator_model(), a=.085, viscous=.075,
                            coulomb_negative=.105, coulomb_positive=.13)
    initial = np.array([getattr(initial_model, key) for key in fields])
    checkpoint_json = json.loads((args.source / "noisy-seed-41-fit.json").read_text(encoding="utf-8"))
    checkpoint = np.array([checkpoint_json["model"][key] for key in fields])
    scales = np.maximum(np.abs(initial), (upper-lower)*.05)
    calls = 0
    def model_from(x):
        return replace(initial_model, **dict(zip(fields, x)))
    def prediction(x):
        return native.rollout(model_from(x), run.t, run.tx_t, run.tx_A, run.initial)
    def residual(x):
        nonlocal calls
        calls += 1
        eq, ev, ei = _errors(run, prediction(x))
        return np.r_[eq / run.sigma_q, ev / run.sigma_v, ei / run.sigma_current]
    def huber_cost(r):
        absolute = np.abs(r)
        return float(np.sum(np.where(absolute <= 1, .5*r*r, absolute-.5)))
    def events(pred):
        return {"stick_transitions": int(np.sum(np.diff(pred[:, 5]) != 0)),
                "moving_sign_changes": int(np.sum(np.diff(np.sign(pred[:, 1])) != 0)),
                "stick_indices": np.flatnonzero(np.diff(pred[:, 5]) != 0).tolist()}
    def score(x):
        model = model_from(x)
        scores = {}
        for label, target in cases.items():
            pred = native.rollout(model, target["t"], target["tx_t"], target["tx_A"], np.zeros(5))
            errors = measurement_errors(target, pred)
            oracle = native.rollout(estimator_model(), target["t"], target["tx_t"], target["tx_A"], np.zeros(5))
            delta_q = pred[:, 0]-oracle[:, 0]
            deviated = np.flatnonzero(np.abs(delta_q) > 1e-4)
            at = int(deviated[0]) if len(deviated) else None
            scores[label] = {"errors": errors, "gate": trajectory_gate(target, errors),
                "truth_comparison_same_realized_inputs_and_initial": {
                    "first_abs_q_error_above_1e-4_s": None if at is None else float(target["t"][at]),
                    "oracle_latent_velocity_at_first_error": None if at is None else float(oracle[at, 1]),
                    "oracle_sticking_at_first_error": None if at is None else bool(oracle[at, 5]),
                    "latent_q_rms_rad": float(np.sqrt(np.mean(delta_q**2))),
                    "latent_velocity_rms_rad_s": float(np.sqrt(np.mean((pred[:, 1]-oracle[:, 1])**2))),
                    "filtered_gyro_rms_rad_s": float(np.sqrt(np.mean((pred[target["v_new"], 3]-oracle[target["v_new"], 3])**2)))}}
        return scores
    def loss_breakdown(x):
        pred = prediction(x)
        interval, gyro, current = _errors(run, pred)
        result = {}
        for channel, raw, normalized, mask in (
                ("encoder", pred[run.q_new, 0]-run.q[run.q_new], interval/run.sigma_q, run.q_new),
                ("gyro", gyro, gyro/run.sigma_v, run.v_new),
                ("current", current, current/run.sigma_current, run.current_new)):
            truth_motion = np.abs(data["truth"][mask, 1])
            result[channel] = {"native_samples": len(raw), "center_rms": float(np.sqrt(np.mean(raw**2))),
                "normalized_rms": float(np.sqrt(np.mean(normalized**2))),
                "huber_loss": huber_cost(normalized),
                "regimes": {regime: {"samples": int(keep.sum()),
                    "huber_loss": huber_cost(normalized[keep])} for regime, keep in
                    (("sticking", truth_motion < 1e-12),
                     ("moving", truth_motion >= 1e-12),
                     ("low_speed", (truth_motion >= 1e-12) & (truth_motion <= .05)),
                     ("higher_speed", truth_motion > .05))}}
        return result
    if args.mode == "replay":
        fit = fit_family(native, initial_model, [run], bounds=FREE_BOUNDS, max_nfev=args.max_nfev)
        model = fit.pop("model")
        values = np.array([getattr(model, key) for key in fields])
        report = {"schema": "adr0022.seed41-repair-replay/1", "argv": sys.argv,
            "source": str(args.source), "library": str(args.library),
            "fitting_data": ["noisy-seed-41"], "cross_runs_not_used_to_choose_fit": True,
            "before": {"checkpoint": checkpoint_json["optimizer"], "scores": score(checkpoint),
                       "loss_breakdown": loss_breakdown(checkpoint)},
            "after": {**fit, "model": model.document(), "scores": score(values),
                      "loss_breakdown": loss_breakdown(values)},
            "unchanged": {"bounds": FREE_BOUNDS, "loss": "huber", "max_nfev": args.max_nfev,
                "noise": sensor_scales(data), "known_zero_initial_state": True,
                "free_coordinates": fields, "relative_parameter_gate": .05},
            "promotion_blocked": True}
        residual_vectors = {}
        for label, point in (("before", checkpoint), ("after", values)):
            pred = prediction(point)
            interval, gyro, current = _errors(run, pred)
            residual_vectors.update({f"{label}_encoder_center_rad": pred[run.q_new, 0]-run.q[run.q_new],
                f"{label}_encoder_interval_rad": interval, f"{label}_gyro_rad_s": gyro,
                f"{label}_current_A": current})
        np.savez_compressed(args.output_dir / "native-residuals-before-after.npz",
            encoder_t=run.t[run.q_new], gyro_t=run.t[run.v_new], current_t=run.t[run.current_new],
            sigma_q=run.sigma_q, sigma_v=run.sigma_v, sigma_current=run.sigma_current,
            **residual_vectors)
        # Two bounded starts declared before evaluating their quality. Fit only
        # seed 41 and retain both outcomes; cross-run results never pick a winner.
        report["predeclared_multistart"] = []
        for label, values in (("inertial-start", [.12, .04, .10, .14]),
                              ("drag-start", [.075, .09, .14, .09])):
            started_model = model_from(np.asarray(values))
            result = fit_family(native, started_model, [run], bounds=FREE_BOUNDS, max_nfev=args.max_nfev)
            candidate = result.pop("model")
            point = np.array([getattr(candidate, key) for key in fields])
            report["predeclared_multistart"].append({"label": label, "start": values,
                "optimizer": result["optimizer"], "fitted": candidate.document(),
                "scores": score(point), "role": "TRAINING_ONLY_DIAGNOSTIC; NOT_SELECTED_BY_CROSS_RUNS"})
        save(args.output_dir / "replay.json", report)
        print(json.dumps({"optimizer": fit["optimizer"], "scores": report["after"]["scores"]}), flush=True)
        return
    truth = np.array([FIXTURE[key] for key in fields])
    diagnostics = {"schema": "adr0022.seed41-estimator-diagnosis/1", "argv": sys.argv,
        "source": str(args.source), "library": str(args.library),
        "fields": fields, "parameter_scales": scales.tolist(),
        "fitting_data": ["noisy-seed-41"], "cross_runs_not_used_to_choose_fit": True,
        "parameter_relative_gate": .05, "frozen_q_gate_rad": trajectory_gate(data,
            measurement_errors(data, prediction(truth)))["thresholds"]["q"],
        "before": {"checkpoint": checkpoint_json["optimizer"], "scores": score(checkpoint),
                   "loss_breakdown": loss_breakdown(checkpoint)},
        "oracle": {"cost": huber_cost(residual(truth)), "scores": score(truth),
                   "loss_breakdown": loss_breakdown(truth)}}
    derivative_diagnostics = []
    for key_index, key in enumerate(fields):
        previous = None
        base_events = events(prediction(checkpoint))
        for physical_step in (1e-4, 1e-5, 1e-6, 1e-7):
            positive, negative = checkpoint.copy(), checkpoint.copy()
            positive[key_index] += physical_step
            negative[key_index] -= physical_step
            plus, minus = residual(positive), residual(negative)
            column = (plus-minus)/(2*physical_step)
            derivative_diagnostics.append({"coordinate": key, "absolute_step": physical_step,
                "raw_normalized_column_norm": float(np.linalg.norm(column)),
                "relative_change_from_previous_step": None if previous is None else
                    float(np.linalg.norm(column-previous)/np.linalg.norm(previous)),
                "positive_events": events(prediction(positive)), "negative_events": events(prediction(negative)),
                "base_events": base_events,
                "cost_plus": huber_cost(plus), "cost_minus": huber_cost(minus)})
            previous = column
    diagnostics["physical_step_derivatives"] = derivative_diagnostics
    # Predeclared bracket around the retained point, training objective only.
    profiles = []
    for k, key in enumerate(fields):
        for fractional_scale in (-.02, -.01, -.005, 0., .005, .01, .02):
            x = checkpoint.copy()
            x[k] = np.clip(x[k] + fractional_scale*scales[k], lower[k], upper[k])
            profiles.append({"coordinate": key, "value": float(x[k]),
                             "huber_cost": huber_cost(residual(x))})
    diagnostics["training_profiles"] = profiles
    pair_profiles = []
    for pair in ((0, 1), (2, 3)):
        for shift1 in (-.01, 0., .01):
            for shift2 in (-.01, 0., .01):
                x = checkpoint.copy()
                for k, shift in zip(pair, (shift1, shift2)):
                    x[k] = np.clip(x[k] + shift*scales[k], lower[k], upper[k])
                pair_profiles.append({"coordinates": [fields[k] for k in pair],
                    "values": [float(x[k]) for k in pair], "huber_cost": huber_cost(residual(x))})
    diagnostics["training_pair_profiles"] = pair_profiles
    diagnostics["retained_seed_cross_evaluations"] = []
    for seed in (17, 83):
        saved = json.loads((args.source / f"noisy-seed-{seed}-fit.json").read_text(encoding="utf-8"))
        values = np.array([saved["model"][key] for key in fields])
        diagnostics["retained_seed_cross_evaluations"].append({"seed": seed,
            "role": "DIAGNOSTIC_ONLY; NOT_CHOSEN_FIT", "seed41_training_cost": huber_cost(residual(values)),
            "parameters": dict(zip(fields, map(float, values))), "scores": score(values)})
    save(args.output_dir / "pre-fit-diagnostics.json", diagnostics)
    results = []
    # One repair at a time: compare solver relative FD with bounded absolute
    # symmetric FD, using exactly the same scales, residuals, bounds and loss.
    for label, start, derivative, step in (("original-relative-forward", initial, "2-point", 1e-4),
            ("checkpoint-relative-forward", checkpoint, "2-point", 1e-4),
            ("original-absolute-forward", initial, "absolute-forward", 1e-6),
            ("original-absolute-central-coarse", initial, "absolute-central", 1e-5),
            ("original-absolute-central", initial, "absolute-central", 1e-6),
            ("original-absolute-central-fine", initial, "absolute-central", 1e-7),
            ("checkpoint-absolute-central", checkpoint, "absolute-central", 1e-6)):
        started, before_calls = time.monotonic(), calls
        if derivative.startswith("absolute-"):
            def jacobian(x):
                columns = []
                base = residual(x) if derivative == "absolute-forward" else None
                for k in range(len(x)):
                    h = step
                    plus, minus = x.copy(), x.copy()
                    plus[k], minus[k] = min(x[k]+h, upper[k]), max(x[k]-h, lower[k])
                    if derivative == "absolute-forward" and plus[k] != x[k]:
                        columns.append((residual(plus)-base)/(plus[k]-x[k]))
                    else:
                        columns.append((residual(plus)-residual(minus))/(plus[k]-minus[k]))
                return np.column_stack(columns)
            jac = jacobian
        else:
            jac = derivative
        fit = least_squares(residual, start, bounds=(lower, upper), loss="huber", f_scale=1.,
            method="trf", max_nfev=args.max_nfev, x_scale=scales, diff_step=step, jac=jac)
        result = {"label": label, "start": start.tolist(), "derivative": derivative,
            "derivative_step": step, "fitted": dict(zip(fields, map(float, fit.x))), "cost": float(fit.cost),
            "optimality": float(fit.optimality), "optimizer_converged": bool(fit.success),
            "termination": str(fit.message), "nfev": int(fit.nfev), "njev": int(fit.njev),
            "residual_calls_including_jacobian": calls-before_calls, "elapsed_s": time.monotonic()-started,
            "relative_errors": dict(zip(fields, map(float, fit.x/truth-1))),
            "parameter_recovery": bool(np.all(np.abs(fit.x/truth-1) <= .05)),
            "predictions": score(fit.x), "events": events(prediction(fit.x))}
        results.append(result)
        save(args.output_dir / "fit-comparisons.json", results)
        print(json.dumps({k: result[k] for k in ("label", "fitted", "cost", "optimality", "optimizer_converged",
            "nfev", "residual_calls_including_jacobian", "elapsed_s")}), flush=True)
    diagnostics["fits"] = results
    diagnostics["promotion_blocked"] = True
    save(args.output_dir / "diagnosis.json", diagnostics)


if __name__ == "__main__":
    main()
