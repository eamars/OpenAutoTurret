"""Prepare fair yaw ablations and diagnose thresholds using frozen TRAIN only.

The profile operation evaluates a bounded, declared grid, retaining complete
uninterrupted predictions. It never runs a full fit, reads SELECTION/HOLDOUT
observations, commands hardware, changes controller gains, or approves deployment.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict, fields, replace
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun, family_reports
from Firmware.commissioning.recovery import diagnostic_report


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def fresh_output(path):
    path = Path(path)
    if path.exists() and any(path.iterdir()):
        raise ValueError("retained evidence exists; use a new output directory")
    path.mkdir(parents=True, exist_ok=True)
    return path


def prepare_fair_plans(constant_path, affine_path, output):
    output = fresh_output(output)
    constant = json.loads(Path(constant_path).read_text(encoding="utf-8"))
    affine = json.loads(Path(affine_path).read_text(encoding="utf-8"))
    same_keys = ("schema", "splits", "frozen_calibration_json", "calibration_revision",
        "encoder_datum_count", "maximum_timing_search_s", "training_noise", "calibration_scope")
    if any(constant[k] != affine[k] for k in same_keys):
        raise ValueError("input/noise/calibration policies differ; cannot make a clean ablation")
    original_pairs = list(zip(constant["candidates"], affine["candidates"], strict=True))
    q_bound = max(max(abs(c["initial_model"][k]) for k in ("q_min", "q_max"))
                  for c in affine["candidates"])
    plans = {}
    for load in ("constant", "affine"):
        for measurement in ("fixed", "free"):
            name = f"{load}-current-{measurement}"
            plan = deepcopy(constant)
            for target, (base, alternative) in zip(plan["candidates"], original_pairs, strict=True):
                target["label"] = base["label"].removesuffix("-constant") + f"-{load}-current-{measurement}"
                target["initial_model"].update(load=load, q_min=-q_bound, q_max=q_bound,
                    load_slope=0., current_tau=0., current_delay=0.)
                for coordinate in ("load_slope", "current_tau", "current_delay"):
                    target["bounds"].pop(coordinate, None)
                if load == "affine":
                    target["bounds"]["load_slope"] = deepcopy(alternative["bounds"]["load_slope"])
                if measurement == "free":
                    for coordinate in ("current_tau", "current_delay"):
                        target["bounds"][coordinate] = deepcopy(alternative["bounds"][coordinate])
            plan["numerical_domain_contract"] = {"common_numerical_q_domain_rad": [-q_bound, q_bound],
                "basis": "existing affine worst-case mathematical bound encloses all four crossed plans",
                "source": str(affine_path), "physical_travel_limit": False,
                "not_a_parameter_scale_or_applicability_claim": True}
            plan["fair_ablation_contract"] = {"load": load, "current_measurement": measurement,
                "common_successful_TX_and_prehistory": True, "common_single_initial_state_policy": True,
                "common_noise_and_loss_weights": True, "parameter_gauge": "load_offset=0; directional totals are A-equivalent",
                "motor_command_mode": "owner-confirmed CAN current control; PWM excluded",
                "fit_execution": "NOT_RUN; WP2 independent estimator verification required first",
                "full_optimizer_budget": 200, "acceptance_criteria_changed": False,
                "deployment_authorized": False}
            plans[name] = plan
            write_json(output / (name + ".json"), plan)
    # Verify every comparison changes only the intended declared coordinates.
    checks = []
    for measurement in ("fixed", "free"):
        for a, b in zip(plans[f"constant-current-{measurement}"]["candidates"],
                        plans[f"affine-current-{measurement}"]["candidates"], strict=True):
            model_diff = sorted(k for k in a["initial_model"] if a["initial_model"][k] != b["initial_model"][k])
            bound_diff = sorted(k for k in set(a["bounds"]) | set(b["bounds"])
                                if a["bounds"].get(k) != b["bounds"].get(k))
            if model_diff != ["load"] or bound_diff != ["load_slope"]:
                raise ValueError("load ablation has an unintended seed or parameter-freedom difference")
            checks.append({"measurement": measurement, "candidate": a["label"],
                           "model_differences": model_diff, "bound_differences": bound_diff})
    for load in ("constant", "affine"):
        for a, b in zip(plans[f"{load}-current-fixed"]["candidates"],
                        plans[f"{load}-current-free"]["candidates"], strict=True):
            if a["initial_model"] != b["initial_model"]:
                raise ValueError("measurement ablation seeds differ")
            different = sorted(k for k in set(a["bounds"]) | set(b["bounds"])
                               if a["bounds"].get(k) != b["bounds"].get(k))
            if different != ["current_delay", "current_tau"]:
                raise ValueError("measurement ablation changes unrelated freedom")
    manifest = {"schema": "adr0022.fair-load-measurement-ablation/1",
        "plans": list(plans), "common_numerical_q_domain_rad": [-q_bound, q_bound],
        "clean_load_ablation_checks": checks, "current_ablation_checks": "PASS",
        "full_fit": "NOT_RUN", "physical_activity": "NOT_RUN", "deployment_authorized": False}
    write_json(output / "plan-verification.json", manifest)
    return manifest


def load_retained_train(source, candidate, plan):
    noise = plan["training_noise"]
    runs, retained_predictions = [], {}
    for record in plan["splits"]["train"]:
        run_id = record["physical_run_id"]
        path = source / "trajectories" / candidate["label"] / (run_id + ".npz")
        with np.load(path, allow_pickle=False) as data:
            run = FamilyRun(run_id=run_id, physical_run_id=run_id, source_id=str(path.resolve()),
                configuration_id=record["configuration_id"], calibration_revision=plan["calibration_revision"],
                t=data["t"], q=data["encoder_q"], v=data["gyro_v"], current=data["decoded_current"],
                q_new=data["q_new"], v_new=data["v_new"], current_new=data["current_new"],
                tx_t=data["tx_t"], tx_A=data["tx_A"], initial=data["initial"],
                sigma_q=noise["sigma_q_rad"], sigma_v=noise["sigma_v_rad_s"],
                sigma_current=noise["sigma_current_A"], encoder_quantum=2*np.pi/8192)
            retained_predictions[run_id] = data["prediction"].copy()
        run.validate()
        runs.append(run)
    return runs, retained_predictions


def threshold_grid(model):
    """Bounded physical-unit perturbations, not relative machine-epsilon steps."""
    yield "retained", model, {"kind": "retained_checkpoint"}
    for directions in (("negative",), ("positive",), ("negative", "positive")):
        tag = "both" if len(directions) == 2 else directions[0]
        for excess in (0., .005, .01, .02, .04, .08):
            changes = {"static_" + direction: getattr(model, "coulomb_" + direction) + excess
                       for direction in directions}
            yield f"static-{tag}-{excess:g}", replace(model, **changes), {
                "kind": "static_excess_profile", "directions": directions, "excess_A": excess}
        # Lowering Fs below Fc is inadmissible. This bracket moves the supported
        # directional moving+static combination together under the fixed gauge.
        for delta in (-.15, -.10, -.05, -.02, -.005, .02, .05):
            changes = {}
            for direction in directions:
                fc = getattr(model, "coulomb_" + direction) + delta
                if fc < 0 or fc > .9:
                    break
                changes["coulomb_" + direction] = fc
                changes["static_" + direction] = fc
            else:
                yield f"total-{tag}-{delta:g}", replace(model, **changes), {
                    "kind": "directional_total_profile", "directions": directions,
                    "moving_total_delta_A": delta, "excess_A": 0.,
                    "gauge": "load_offset remains zero; not separately calibrated friction"}


def compact_diagnostic(report):
    return {key: report[key] for key in ("baselines", "displacement", "observed_motion",
        "predicted_motion", "transition_comparison", "objective", "no_motion_model_on_moving_data")}


def bounded_threshold_outer_search(profiles):
    """Rank declared training-only grid points; quality gates still block promotion.

    This diagnostic outer search holds moving nuisances and one initial state per
    whole run fixed. It is not a global fit, an inner smooth refit, or estimator
    qualification. Selection and holdout never choose the grid point.
    """
    finite = [p for p in profiles if p["forward_status"] == "PASS"]
    if not finite:
        return {"status": "NO_VALID_GRID_POINT", "deployment_authorized": False}
    best = min(finite, key=lambda p: (p["training_huber_cost"], p["profile_id"]))
    return {"status": "DIAGNOSTIC_GRID_POINT_ONLY", "best_profile_id": best["profile_id"],
        "best_training_huber_cost": best["training_huber_cost"], "best_model": best["model"],
        "whole_training_gates_passed": all(r["engineering_gates"]["passed"] for r in best["runs"]),
        "selection_used": False, "holdout_used": False, "inner_moving_fit": "NOT_RUN",
        "estimator_qualification": "UNVERIFIED", "deployment_authorized": False}


def profile_thresholds(source, library, output):
    source, output = Path(source), fresh_output(output)
    comparison = json.loads((source / "comparison-result.json").read_text(encoding="utf-8"))
    plan = json.loads((source / "comparison-plan.json").read_text(encoding="utf-8"))
    native = FamilyNative(library)
    candidates = [c for c in comparison["comparisons"] if c.get("model", {}).get("friction") == "coulomb"
                  and c["model"]["load"] == "constant"]
    if len(candidates) != 2:
        raise ValueError("expected retained algebraic and first-order constant Coulomb candidates")
    contract = {"schema": "adr0022.training-threshold-profiles/1", "source": str(source),
        "library": str(library), "source_command_mode": "owner-confirmed CAN current control",
        "training_runs": [r["physical_run_id"] for r in plan["splits"]["train"]],
        "retained_single_training_initial_states": True, "native_sample_masks": True,
        "objective": "unchanged channel sigma, encoder bin residual, Huber f_scale=1",
        "grid": {"static_excess_A": [0., .005, .01, .02, .04, .08],
                 "directional_moving_total_change_A": [-.15, -.10, -.05, -.02, -.005, .02, .05],
                 "directions": ["negative", "positive", "both"]},
        "gauge": "load_offset fixed to zero; Fc/Fs are directional input-equivalent totals",
        "selection_observations": "NOT_READ", "holdout_observations": "NOT_READ",
        "full_refit": "NOT_RUN", "physical_activity": "NOT_RUN",
        "acceptance_criteria_changed": False, "deployment_authorized": False}
    write_json(output / "frozen-profile-contract.json", contract)
    began = time.monotonic(); summaries = []
    for candidate in candidates:
        model = FamilyModel(**{f.name: candidate["model"][f.name] for f in fields(FamilyModel)})
        if model.load_offset != 0:
            raise ValueError("this diagnostic requires the retained zero-offset gauge")
        runs, retained_predictions = load_retained_train(source, candidate, plan)
        profiles, samples = [], {r.run_id: [] for r in runs}
        profile_ids = []
        for profile_id, variant, change in threshold_grid(model):
            variant.validate()
            row = {"profile_id": profile_id, "change": change, "model": variant.document(),
                "forward_status": "PASS", "training_huber_cost": 0., "runs": []}
            for run in runs:
                try:
                    prediction = native.rollout(variant, run.t, run.tx_t, run.tx_A, run.initial)
                    report = diagnostic_report(run, prediction)
                except ValueError as exc:
                    row["forward_status"] = "FAIL"
                    row["runs"].append({"run_id": run.run_id, "reason": str(exc)})
                    samples[run.run_id].append(None)
                    continue
                gates = family_reports(native, variant, [run])[0]
                run_result = {"run_id": run.run_id, **compact_diagnostic(report), "engineering_gates": gates}
                if profile_id == "retained":
                    run_result["retained_prediction_replay_max_difference"] = {
                        "encoder_rad": float(np.max(np.abs(prediction[run.q_new, 0] -
                            retained_predictions[run.run_id][run.q_new, 0]))),
                        "gyro_rad_s": float(np.max(np.abs(prediction[run.v_new, 3] -
                            retained_predictions[run.run_id][run.v_new, 3]))),
                        "reported_current_A": float(np.max(np.abs(prediction[run.current_new, 4] -
                            retained_predictions[run.run_id][run.current_new, 4])))}
                if run.run_id in ("yaw-physical-probe-01", "yaw-30deg-01", "yaw-information-case4-01"):
                    run_result["event_anchored_horizons"] = report["event_anchored_horizons"]
                row["runs"].append(run_result)
                row["training_huber_cost"] += report["objective"]["total_huber_cost"]
                # Save only the actual observation predictions. Internal solver
                # states and shared observations are available in retained input.
                samples[run.run_id].append((prediction[run.q_new, 0], prediction[run.v_new, 3],
                    prediction[run.current_new, 4], prediction[run.v_new, 1]))
            if row["forward_status"] != "PASS": row["training_huber_cost"] = None
            profiles.append(row); profile_ids.append(profile_id)
            if len(profiles) % 10 == 0:
                print(json.dumps({"candidate": candidate["label"], "profiles_evaluated": len(profiles),
                                  "elapsed_s": time.monotonic() - began}), flush=True)
        target = output / candidate["label"]
        for run in runs:
            arrays = {}
            for column, name, mask in ((0, "encoder_prediction_rad", run.q_new),
                    (1, "gyro_prediction_rad_s", run.v_new), (2, "current_prediction_A", run.current_new),
                    (3, "latent_velocity_at_gyro_rad_s", run.v_new)):
                arrays[name] = np.array([s[column] if s is not None else np.full(int(mask.sum()), np.nan)
                                        for s in samples[run.run_id]])
            target.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(target / (run.run_id + "-native-predictions.npz"),
                profile_ids=np.array(profile_ids), encoder_t_s=run.t[run.q_new],
                gyro_t_s=run.t[run.v_new], current_t_s=run.t[run.current_new], **arrays)
        # Profiles are finite differences over declared absolute brackets. Report
        # their outcome changes instead of pretending flat local columns measure
        # absence of a physical threshold.
        baseline = profiles[0]
        base_by_run = {r["run_id"]: r for r in baseline["runs"]}
        sensitivities = []
        for row in profiles[1:]:
            if row["forward_status"] != "PASS": continue
            sensitivities.append({"profile_id": row["profile_id"],
                "training_cost_change": row["training_huber_cost"] - baseline["training_huber_cost"],
                "run_changes": [{"run_id": r["run_id"],
                    "predicted_span_change_rad": r["displacement"]["predicted_span_rad"] -
                        base_by_run[r["run_id"]]["displacement"]["predicted_span_rad"],
                    "predicted_start_count_change": len([e for e in r["predicted_motion"]["events"]
                        if e["kind"] == "predicted_motion_onset"]) - len([e for e in
                        base_by_run[r["run_id"]]["predicted_motion"]["events"] if e["kind"] == "predicted_motion_onset"]),
                    "no_motion_contradiction": r["no_motion_model_on_moving_data"]}
                    for r in row["runs"]]})
        result = {"candidate": candidate["label"], "profiles": profiles,
            "absolute_step_outcome_sensitivity": sensitivities,
            "outer_search": bounded_threshold_outer_search(profiles),
            "unqualified_directional_totals": True, "deployment_authorized": False}
        write_json(target / "profiles.json", result)
        summaries.append({"candidate": candidate["label"], "profiles_evaluated": len(profiles),
            "all_native_whole_runs": len(runs), "retained_training_huber_cost": baseline["training_huber_cost"],
            "outer_search": result["outer_search"], "profile_evidence": str(target / "profiles.json")})
    summary = {**contract, "candidates": summaries, "elapsed_s": time.monotonic() - began}
    write_json(output / "profile-summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    prepare = commands.add_parser("prepare")
    prepare.add_argument("--constant-plan", type=Path, required=True)
    prepare.add_argument("--affine-plan", type=Path, required=True)
    prepare.add_argument("--output", type=Path, required=True)
    profile = commands.add_parser("profile")
    profile.add_argument("--retained-comparison", type=Path, required=True)
    profile.add_argument("--library", type=Path, required=True)
    profile.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.operation == "prepare":
        result = prepare_fair_plans(args.constant_plan, args.affine_plan, args.output)
        print(json.dumps({"prepared_plans": result["plans"], "full_fit": "NOT_RUN"}, indent=2))
    else:
        result = profile_thresholds(args.retained_comparison, args.library, args.output)
        print(json.dumps({"profiled_candidates": len(result["candidates"]),
                          "elapsed_s": result["elapsed_s"], "deployment_authorized": False}, indent=2))


if __name__ == "__main__":
    main()
