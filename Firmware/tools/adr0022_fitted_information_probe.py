"""Conditional fitted joint information; no confidence ensemble or new fitting."""
from dataclasses import replace
import argparse
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np
from scipy.special import erf

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.family_assets import MODEL_UNITS, model_from_document
from Firmware.commissioning.model_family import FamilyNative
from Firmware.tools.adr0022_nuisance_verification import (BIN_FINAL_TRAIN, FIT_INITIAL,
    GROUPS, admit_bin_final_dataset, expected_encoder_information, gaussian_bin_deviance,
    load_dataset)
from Firmware.tools.adr0022_closed_loop_estimator_probe import FIXTURE


SEED = 10103
LABEL = BIN_FINAL_TRAIN + "-noisy-" + str(SEED)
FIELDS = tuple(GROUPS["mechanics_and_nuisance"])
GATES = {"derivative_relative_change": .01, "normalized_rank_relative_cutoff": 1e-8,
    "normalized_condition_max": 1e6, "encoder_probability_mass_error": 1e-12,
    "saved_prediction_max_error": 1e-12}


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def inputs(source):
    result = json.loads((source/("mechanics_and_nuisance-fit-"+LABEL+".json")).read_text())
    data = load_dataset(source/(LABEL+".npz"))
    admit_bin_final_dataset(data, BIN_FINAL_TRAIN, True, SEED)
    if result["training_case"] != LABEL or tuple(result["free_fields"]) != FIELDS:
        raise ValueError("chronological original joint10 fit required; no winner selection")
    model = model_from_document(result["model"])
    optimizer = result["optimizer"]
    if tuple(optimizer["coordinates"]) != FIELDS or optimizer["encoder_objective"] != "gaussian_bins":
        raise ValueError("original exact-bin coordinate ordering required")
    scales = np.asarray(optimizer["coordinate_scales"])
    steps = np.asarray(optimizer["absolute_derivative_steps"])
    if not np.array_equal(scales, np.array([FIT_INITIAL[k] for k in FIELDS])) or \
            not np.allclose(steps, scales*1e-6, rtol=1e-15, atol=0.):
        raise ValueError("original physical scales and central steps required")
    path = source/("mechanics_and_nuisance-fit-"+LABEL+"-predict-"+LABEL+".npz")
    with np.load(path, allow_pickle=False) as archive:
        prediction = archive["prediction"]
        if any(not np.array_equal(archive[k], data[k]) for k in ("t", "q_new", "v_new", "current_new")):
            raise ValueError("retained fitted prediction times/native masks differ")
    return result, data, model, prediction, scales, steps


def declaration(source, library, model, scales, steps):
    return {"scope": "CONDITIONAL_LOCAL_INFORMATION_ONLY", "uncertainty": "UNKNOWN",
        "chronological_training_case": LABEL, "selection": "first chronological10103; no winner",
        "source_directory": str(source), "native_library": str(library),
        "fitted_model": model.document(), "fields": FIELDS,
        "units": {k: MODEL_UNITS[k] for k in FIELDS}, "coordinate_scales": scales.tolist(),
        "physical_central_steps": steps.tolist(), "decade_steps": (steps*.1).tolist(),
        "native_max_steps_s": [model.max_step, model.max_step/2], "gates": GATES,
        "information": "expected score Fisher: native physical mean derivatives; exact preset encoder-bin Fisher; Gaussian fresh gyro/current",
        "score": "encoder signed deviance times analytic slope is negative-log-likelihood mean score; score covariance uses expected Fisher",
        "noise_diagnostics": "retained independent truth, native modality freshness masks; phase-aware encoder bin scores/counts; no uniform quantization approximation",
        "noise_levels": {k: FIXTURE[k] for k in ("encoder_noise", "encoder_quantum", "gyro_noise", "current_noise")},
        "noise_diagnostic_acceptance": "descriptive consumed-data checks only; no calibrated confidence coverage gate",
        "fixed_gauges": "same model gain/bias/load/static/clock/prehistory/one-zero5-state acquisition",
        "new_noise_draws": 0, "inverse_fits": 0, "confidence_ensemble": "NOT_CONSTRUCTED",
        "physical_actions": False, "promotion": False}


def reconstruction(data, prediction):
    qmask, vmask, imask = (data[k] for k in ("q_new", "v_new", "current_new"))
    sigma, quantum = FIXTURE["encoder_noise"], FIXTURE["encoder_quantum"]
    residual, slope = gaussian_bin_deviance(prediction[qmask, 0], data["q"][qmask], quantum, sigma)
    information, mass = expected_encoder_information(prediction[qmask, 0])
    r = np.r_[residual, (prediction[vmask, 3]-data["v"][vmask])/FIXTURE["gyro_noise"],
        (prediction[imask, 4]-data["current"][imask])/FIXTURE["current_noise"]]
    return {"samples": {"encoder": int(qmask.sum()), "gyro": int(vmask.sum()), "current": int(imask.sum())},
        "exact_objective_cost": float(.5*(r@r)), "normalized_residual_rms": float(np.sqrt(np.mean(r*r))),
        "encoder_fitted_score_sum_per_rad": float(np.sum(residual*slope)),
        "encoder_expected_information_range_per_rad2": [float(information.min()), float(information.max())],
        "encoder_probability_mass_max_error": float(np.max(abs(mass-1))),
        "finite": bool(np.isfinite(r).all() and np.isfinite(slope).all())}


def noise_diagnostics(source, data, prediction):
    """Use independently retained true means; never infer uniform bin noise."""
    base = load_dataset(source/(BIN_FINAL_TRAIN+"-independent-latent.npz"))
    admit_bin_final_dataset(base, BIN_FINAL_TRAIN, False, 0)
    if any(not np.array_equal(base[k], data[k]) for k in
           ("t", "tx_t", "tx_A", "initial", "truth", "q_new", "v_new", "current_new")):
        raise ValueError("independent latent truth must be the exact retained TRAIN source")
    result = {"scope": "CONSUMED_GENERATOR_DIAGNOSTICS; NOT_CONFIDENCE_COVERAGE",
        "independent_truth_source": str(source/(BIN_FINAL_TRAIN+"-independent-latent.npz")),
        "held_samples_used_as_new": False}
    for name, channel, mask, sigma in (("gyro", 3, "v_new", FIXTURE["gyro_noise"]),
                                      ("current", 4, "current_new", FIXTURE["current_noise"])):
        measured = data["v" if name == "gyro" else "current"][data[mask]]
        standardized = (measured-base["truth"][data[mask], channel])/sigma
        result[name] = {"fresh_count": len(standardized), "preset_sigma": sigma,
            "normalized_mean": float(standardized.mean()), "normalized_sample_sd": float(standardized.std(ddof=1)),
            "mean_standard_errors": float(standardized.sum()/np.sqrt(len(standardized))),
            "squared_noise_over_expected": float(np.mean(standardized**2)),
            "lag1": float(np.corrcoef(standardized[:-1], standardized[1:])[0, 1]),
            "cross_channel_independence": "PRESET_GENERATOR_ASSUMPTION; NOT_CALIBRATED"}
    mask = data["q_new"]
    mean, observed = base["truth"][mask, 0], data["q"][mask]
    quantum, sigma = FIXTURE["encoder_quantum"], FIXTURE["encoder_noise"]
    residual, slope = gaussian_bin_deviance(mean, observed, quantum, sigma)
    information, mass = expected_encoder_information(mean)
    score = residual*slope
    phase = mean-np.round(mean/quantum)*quantum
    offsets = np.rint((observed-np.round(mean/quantum)*quantum)/quantum).astype(int)
    p0 = float(erf(quantum/(2*sigma*np.sqrt(2.))))
    phases = np.minimum(np.floor((phase/quantum+.5)*8).astype(int), 7)
    bins = []
    for cell in range(8):
        selected = phases == cell
        row = {"phase_cell": cell, "phase_fraction_bounds": [cell/8-.5, (cell+1)/8-.5],
            "count": int(selected.sum()), "bin_offsets": [],
            "true_score_mean_standard_errors": float(score[selected].sum()/np.sqrt(information[selected].sum()))
                if selected.any() else None}
        for offset in range(-3, 4):
            deviance, _ = gaussian_bin_deviance(phase[selected], offset*quantum, quantum, sigma)
            probability = p0*np.exp(-.5*deviance**2)
            expected, variance = float(probability.sum()), float(np.sum(probability*(1-probability)))
            count = int(np.sum(offsets[selected] == offset))
            row["bin_offsets"].append({"offset": offset, "observed": count, "expected": expected,
                "count_standard_errors": float((count-expected)/np.sqrt(variance)) if variance>0 else None,
                "normal_count_approximation": "DIAGNOSTIC_ONLY" if min(expected, row["count"]-expected)>=5 else "SPARSE_NOT_INTERPRETED"})
        bins.append(row)
    result["encoder"] = {"fresh_count": len(mean), "preset_sigma_before_rounding_rad": sigma,
        "quantum_rad": quantum, "probability_mass_max_error": float(np.max(abs(mass-1))),
        "true_score_sum_per_rad": float(score.sum()),
        "true_score_mean_standard_errors": float(score.sum()/np.sqrt(information.sum())),
        "observed_squared_score_over_expected": float((score@score)/information.sum()),
        "true_score_lag1": float(np.corrcoef(score[:-1], score[1:])[0, 1]),
        "observed_offsets_outside_seven_bin_support": int(np.sum(abs(offsets)>3)),
        "phase_aware_bins": bins,
        "no_uniform_quantization_variance_or_continuous_normal_residual_assumed": True}
    fitted_residual, fitted_slope = gaussian_bin_deviance(prediction[mask, 0], observed, quantum, sigma)
    result["fitted_encoder"] = {"score_sum_per_rad": float(np.sum(fitted_residual*fitted_slope)),
        "expected_information_reference": "fitted mean; generating mean kept separate"}
    return result


def mean_column(native, model, data, field, step, qweight):
    plus = native.rollout(replace(model, **{field: getattr(model, field)+step}),
        data["t"], data["tx_t"], data["tx_A"], data["initial"])
    minus = native.rollout(replace(model, **{field: getattr(model, field)-step}),
        data["t"], data["tx_t"], data["tx_A"], data["initial"])
    difference = (plus-minus)/(2*step)
    return np.r_[difference[data["q_new"], 0]*qweight,
        difference[data["v_new"], 3]/FIXTURE["gyro_noise"],
        difference[data["current_new"], 4]/FIXTURE["current_noise"]]


def early(source, library, output):
    output.mkdir(parents=True, exist_ok=False)
    result, data, model, prediction, scales, steps = inputs(source)
    save(output/"predeclared-contract.json", declaration(source, library, model, scales, steps))
    report = reconstruction(data, prediction)
    report["retained_optimizer_cost_error"] = abs(report["exact_objective_cost"]-result["optimizer"]["cost"])
    native = FamilyNative(library)
    current = native.rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
    report["native_saved_prediction_max_error"] = float(np.max(abs(current-prediction)))
    weight = np.sqrt(expected_encoder_information(prediction[data["q_new"], 0])[0])
    field = "gyro_delay"; step = steps[FIELDS.index(field)]
    a = mean_column(native, model, data, field, step, weight)
    b = mean_column(native, model, data, field, step*.1, weight)
    c = mean_column(native, replace(model, max_step=model.max_step/2), data, field, step, weight)
    report.update(derivative_field=field, physical_step=float(step),
        derivative_decade_relative_change=float(np.linalg.norm(b-a)/np.linalg.norm(a)),
        derivative_native_halfstep_relative_change=float(np.linalg.norm(c-a)/np.linalg.norm(a)))
    report["passed"] = bool(report["finite"] and report["native_saved_prediction_max_error"]<=GATES["saved_prediction_max_error"]
        and report["encoder_probability_mass_max_error"]<=GATES["encoder_probability_mass_error"]
        and max(report["derivative_decade_relative_change"], report["derivative_native_halfstep_relative_change"])<=GATES["derivative_relative_change"])
    save(output/"early-result.json", report)
    print(json.dumps(report), flush=True)


def geometry(jacobian):
    norms = np.linalg.norm(jacobian, axis=0)
    if not np.isfinite(jacobian).all() or np.any(norms<=0):
        return {"passed": False, "detail": "nonfinite or zero information column"}
    normalized = jacobian/norms
    singular = np.linalg.svd(normalized, compute_uv=False)
    rank = int(np.sum(singular > singular[0]*GATES["normalized_rank_relative_cutoff"]))
    condition = float(singular[0]/singular[-1])
    return {"passed": bool(rank==len(FIELDS) and condition<=GATES["normalized_condition_max"]),
        "column_norms": norms.tolist(), "normalized_singular_values": singular.tolist(),
        "normalized_rank": rank, "normalized_condition": condition}


def full(source, library, output, *, numerical_max_step=None):
    if (output/"decision.json").exists():
        raise ValueError("completed numerical evidence must remain immutable; use a fresh declared pair")
    result, data, model, prediction, scales, steps = inputs(source)
    if json.loads((output/"predeclared-contract.json").read_text()) != json.loads(json.dumps(declaration(source, library, model, scales, steps))) \
            or not json.loads((output/"early-result.json").read_text())["passed"]:
        raise ValueError("same declared context and actual early prerequisite PASS required")
    save(output/"full-numerical-contract.json", {
        **declaration(source, library, model, scales, steps),
        "source_fitting_model_max_step_s": model.max_step,
        "diagnostic_native_max_steps_s": [numerical_max_step or model.max_step, (numerical_max_step or model.max_step)/2],
        "source_fitted_physical_vector_unchanged": True,
        "noise_phase_cells": 8, "encoder_bins": [-3, -2, -1, 0, 1, 2, 3],
        "noise_checks": "preset generator means/SD/fresh lag1 and exact phase-aware bin-score/count diagnostics; descriptive only",
        "numerical_checks_before_inverse": "all10 fullTRAIN physical mean columns at original steps, decade and half native mesh; no frozen truth parameter substitution",
        "refined_information": "half-mesh center has its own exact-bin Fisher; derivative comparisons use common coarse Fisher weight",
        "inverse_residual_limit": 1e-8, "inverse_residual_coordinate": "dimensionless column-normalized information",
        "failure": "retain numerical diagnostic; covariance NOT_COMPUTED if derivative/rank/condition/SPD gates fail",
        "confidence_calibration": "NOT_RUN; information inverse is conditional local asymptotic covariance only"})
    began = time.monotonic()
    noise = noise_diagnostics(source, data, prediction)
    save(output/"noise-score-diagnostics.json", noise)
    native = FamilyNative(library)
    source_center = native.rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
    saved_error = float(np.max(abs(source_center-prediction)))
    source_model = model
    if numerical_max_step is not None:
        model = replace(model, max_step=numerical_max_step)
        current = native.rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
    else:
        current = source_center
    qmask = data["q_new"]
    qinfo, mass = expected_encoder_information(current[qmask, 0])
    weight = np.sqrt(qinfo)
    refined_model = replace(model, max_step=model.max_step/2)
    refined_center = native.rollout(refined_model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
    refined_qinfo, refined_mass = expected_encoder_information(refined_center[qmask, 0])
    count = int(qmask.sum()+data["v_new"].sum()+data["current_new"].sum())
    original = np.empty((count, len(FIELDS)))
    decade = np.empty_like(original)
    refined = np.empty_like(original)
    comparisons = []
    for index, (field, step) in enumerate(zip(FIELDS, steps)):
        original[:, index] = mean_column(native, model, data, field, step, weight)
        decade[:, index] = mean_column(native, model, data, field, step*.1, weight)
        refined[:, index] = mean_column(native, refined_model, data, field, step, weight)
        norm = np.linalg.norm(original[:, index])
        row = {"field": field, "physical_step": float(step), "unit": MODEL_UNITS[field],
            "column_norm": float(norm),
            "decade_relative_change": float(np.linalg.norm(decade[:, index]-original[:, index])/norm),
            "native_halfstep_relative_change": float(np.linalg.norm(refined[:, index]-original[:, index])/norm)}
        row["passed"] = bool(max(row["decade_relative_change"], row["native_halfstep_relative_change"])<=GATES["derivative_relative_change"])
        comparisons.append(row)
        save(output/"derivative-checkpoint.json", {"columns": comparisons, "covariance": "NOT_COMPUTED"})
        print(json.dumps({"stage": "physical-mean-derivative", **row, "elapsed_s": time.monotonic()-began}), flush=True)
    coarse_geometry = geometry(original)
    decade_geometry = geometry(decade)
    refined[:int(qmask.sum())] *= np.sqrt(refined_qinfo/qinfo)[:, None]
    refined_geometry = geometry(refined)
    information, finer_information, refined_information = (j.T@j for j in (original, decade, refined))
    diagnostics = {"fields": FIELDS, "derivatives": comparisons, "original": coarse_geometry,
        "decade": decade_geometry, "native_halfstep": refined_geometry,
        "saved_prediction_max_error": saved_error,
        "source_fitting_model_max_step_s": source_model.max_step,
        "diagnostic_native_max_steps_s": [model.max_step, refined_model.max_step],
        "diagnostic_center_vs_source_max_channel_errors": np.max(abs(current-source_center), axis=0).tolist(),
        "encoder_probability_mass_max_error": float(max(np.max(abs(mass-1)), np.max(abs(refined_mass-1)))),
        "refined_center_max_channel_errors": np.max(abs(refined_center-current), axis=0).tolist(),
        "refined_encoder_fisher_max_relative_change": float(np.max(abs(refined_qinfo/qinfo-1))),
        "information_decade_relative_change": float(np.linalg.norm(finer_information-information)/np.linalg.norm(information)),
        "information_native_halfstep_relative_change": float(np.linalg.norm(refined_information-information)/np.linalg.norm(information)),
        "information_matrices_computed": True, "covariance": "NOT_COMPUTED_YET"}
    diagnostics["passed"] = bool(all(x["passed"] for x in comparisons) and
        all(x["passed"] for x in (coarse_geometry, decade_geometry, refined_geometry)) and
        saved_error<=GATES["saved_prediction_max_error"] and
        diagnostics["encoder_probability_mass_max_error"]<=GATES["encoder_probability_mass_error"] and
        diagnostics["refined_encoder_fisher_max_relative_change"]<=GATES["derivative_relative_change"])
    diagnostics["failed_checks"] = [x["field"]+": physical derivative consistency" for x in comparisons if not x["passed"]]
    if diagnostics["refined_encoder_fisher_max_relative_change"]>GATES["derivative_relative_change"]:
        diagnostics["failed_checks"].append("pointwise exact-bin Fisher mesh consistency")
    for name, record in (("original", coarse_geometry), ("decade", decade_geometry), ("native_halfstep", refined_geometry)):
        if not record["passed"]: diagnostics["failed_checks"].append(name+": normalized rank/condition")
    # Numerical admission is saved before any inverse is attempted.
    save(output/"numerical-diagnostics-before-inverse.json", diagnostics)
    matrices = {"fields": FIELDS, "units": {k: MODEL_UNITS[k] for k in FIELDS},
        "expected_score_information": information.tolist(),
        "decade_expected_score_information": finer_information.tolist(),
        "refined_native_expected_score_information": refined_information.tolist(),
        "covariance": None, "correlation": None, "uncertainty": "UNKNOWN"}
    decision = {"scope": "CONDITIONAL_LOCAL_INFORMATION_ONLY", "uncertainty": "UNKNOWN",
        "chronological_case": LABEL, "numerical_prerequisite_passed": diagnostics["passed"],
        "information_method": "expected exact-bin encoder score Fisher plus known Gaussian fresh gyro/current; full joint10 physical native mean derivatives",
        "no_Gauss_Newton_or_prior_covariance_substitution": True,
        "confidence_coverage": "NOT_CALIBRATED", "parameter_sets": "NOT_CONSTRUCTED",
        "inverse_fits": 0, "new_noise_draws": 0, "physical_actions": False,
        "promotion": False, "elapsed_s": None}
    decision.update(diagnostic_native_max_steps_s=[model.max_step, refined_model.max_step],
        source_fitting_model_max_step_s=source_model.max_step, failed_checks=diagnostics["failed_checks"],
        max_derivative_decade_change=max(x["decade_relative_change"] for x in comparisons),
        max_derivative_native_halfstep_change=max(x["native_halfstep_relative_change"] for x in comparisons),
        refined_pointwise_encoder_fisher_change=diagnostics["refined_encoder_fisher_max_relative_change"],
        normalized_rank=coarse_geometry.get("normalized_rank"), normalized_condition=coarse_geometry.get("normalized_condition"))
    if diagnostics["passed"]:
        norms = np.asarray(coarse_geometry["column_norms"])
        normalized_information = information/norms[:, None]/norms[None, :]
        try:
            np.linalg.cholesky(normalized_information)
            normalized_covariance = np.linalg.solve(normalized_information, np.eye(len(FIELDS)))
            covariance = normalized_covariance/norms[:, None]/norms[None, :]
            inverse_error = float(np.max(abs(normalized_information@normalized_covariance-np.eye(len(FIELDS)))))
            if inverse_error>1e-8 or np.any(np.diag(covariance)<=0):
                raise np.linalg.LinAlgError("conditional covariance inverse residual/diagonal failed")
            standard_errors = np.sqrt(np.diag(covariance))
            correlation = covariance/standard_errors[:, None]/standard_errors[None, :]
            matrices.update(covariance=covariance.tolist(), correlation=correlation.tolist(),
                standard_errors=dict(zip(FIELDS, map(float, standard_errors))), inverse_identity_max_error=inverse_error,
                inverse_identity_coordinate="dimensionless column-normalized information",
                physical_information_covariance_product_max_error=float(np.max(abs(information@covariance-np.eye(len(FIELDS))))),
                covariance_role="CONDITIONAL_LOCAL_ASYMPTOTIC_ONLY; NO_SUPPORTED_CONFIDENCE_SET")
            encoder, slope = gaussian_bin_deviance(current[qmask, 0], data["q"][qmask],
                FIXTURE["encoder_quantum"], FIXTURE["encoder_noise"])
            score = np.r_[encoder*slope/weight,
                (current[data["v_new"], 3]-data["v"][data["v_new"]])/FIXTURE["gyro_noise"],
                (current[data["current_new"], 4]-data["current"][data["current_new"]])/FIXTURE["current_noise"]]
            joint_score = original.T@score
            matrices["fitted_negative_log_likelihood_gradient"] = dict(zip(FIELDS, map(float, joint_score)))
            matrices["fitted_score_local_information_norm"] = float(np.sqrt(joint_score@covariance@joint_score))
            decision.update(status="FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS",
                standard_errors=matrices["standard_errors"], inverse_identity_max_error=inverse_error,
                fitted_score_local_information_norm=matrices["fitted_score_local_information_norm"])
        except np.linalg.LinAlgError as exc:
            decision.update(status="CONDITIONAL_INFORMATION_INVERSE_FAILED", detail=str(exc))
    else:
        decision.update(status="CONDITIONAL_INFORMATION_NUMERICAL_PREREQUISITE_FAILED",
            covariance="NOT_COMPUTED; no numerical gate relaxation")
    decision["elapsed_s"] = time.monotonic()-began
    save(output/"information-matrices.json", matrices)
    save(output/"decision.json", decision)
    print(json.dumps(decision), flush=True)
    return decision


def refine(source, library, output):
    """At most two frozen adjacent-mesh checks; preserve the original failure."""
    result, data, model, prediction, scales, steps = inputs(source)
    if model.max_step != .000125 or json.loads((output/"decision.json").read_text())["numerical_prerequisite_passed"]:
        raise ValueError("this refinement starts only from the retained original125us failure")
    plan_path = output/"bounded-refinement-plan.json"
    if plan_path.exists(): raise ValueError("new bounded refinement plan requires unconsumed output paths")
    pairs = ((.0000625, .00003125), (.00003125, .000015625))
    save(plan_path, {"scope": "NUMERICAL_RESOLUTION_PREREQUISITE_ONLY", "source_case": LABEL,
        "source_fitting_model_max_step_s": model.max_step, "fitted_model": model.document(),
        "physical_coordinate_values": result["optimizer"]["coordinate_values"],
        "coordinate_scales": scales.tolist(), "physical_steps": steps.tolist(),
        "adjacent_pairs_s": pairs, "maximum_further_pairs": 2, "gates": GATES,
        "pointwise_exact_bin_fisher_relative_gate": GATES["derivative_relative_change"],
        "stop": "first pair passing all unchanged numerical gates; no inverse otherwise",
        "source_native_equality": "125us native source model rechecked against retained prediction for each pair",
        "new_fits": 0, "new_noise_draws": 0, "physical_actions": False,
        "original_failure": "decision.json/numerical-diagnostics-before-inverse.json/information-matrices.json preserved"})
    results = []
    for index, (coarse, fine) in enumerate(pairs, 1):
        folder = output/("refinement-pair-"+str(index))
        folder.mkdir(exist_ok=False)
        for name in ("predeclared-contract.json", "early-result.json"):
            shutil.copyfile(output/name, folder/name)
        decision = full(source, library, folder, numerical_max_step=coarse)
        results.append({"pair_number": index, "coarse_s": coarse, "fine_s": fine,
            "decision": decision, "evidence": str(folder)})
        save(output/"refinement-checkpoint.json", {"completed_pairs": results, "maximum_pairs": 2,
            "uncertainty": "UNKNOWN"})
        if decision["status"] == "FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS": break
    save(output/"bounded-refinement-result.json", {"original_failure_preserved": True,
        "completed_pairs": results, "first_passing_pair": next((r["pair_number"] for r in results if
            r["decision"]["status"]=="FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS"), None),
        "remaining_pairs": "NOT_RUN; first passing pair" if len(results)<2 else "NONE",
        "scope": "CONDITIONAL_LOCAL_INFORMATION_ONLY", "uncertainty": "UNKNOWN",
        "calibrated_joint_coverage": "NOT_RUN", "new_fits": 0, "new_noise_draws": 0})
    export_compact(output)


def export_compact(output):
    """Architect evidence only; raw diagnostics stay in the local folders."""
    records = [("original", output)]
    bounded = json.loads((output/"bounded-refinement-result.json").read_text())
    records += [("refinement-pair-"+str(r["pair_number"]), Path(r["evidence"])) for r in bounded["completed_pairs"]]
    summaries, selected = [], None
    for label, folder in records:
        d = json.loads((folder/"numerical-diagnostics-before-inverse.json").read_text())
        decision = json.loads((folder/"decision.json").read_text())
        item = {"case": label, "native_max_steps_s": d.get("diagnostic_native_max_steps_s", [.000125, .0000625]),
            "status": decision["status"], "all10_decade_derivatives_passed": all(x["decade_relative_change"]<=.01 for x in d["derivatives"]),
            "max_decade_relative_change": max(x["decade_relative_change"] for x in d["derivatives"]),
            "max_native_halfstep_relative_change": max(x["native_halfstep_relative_change"] for x in d["derivatives"]),
            "transport_delay_halfstep_relative_change": next(x["native_halfstep_relative_change"] for x in d["derivatives"] if x["field"]=="transport_delay"),
            "pointwise_encoder_fisher_halfstep_relative_change": d["refined_encoder_fisher_max_relative_change"],
            "joint_information_halfstep_relative_change": d["information_native_halfstep_relative_change"],
            "normalized_rank": d["original"].get("normalized_rank"), "normalized_condition": d["original"].get("normalized_condition"),
            "saved_source_native_error": d["saved_prediction_max_error"],
            "halfstep_center_q_max_error_rad": d["refined_center_max_channel_errors"][0],
            "covariance": "COMPUTED_LOCAL_INFORMATION_INVERSE" if decision["status"]=="FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS" else "NOT_COMPUTED"}
        summaries.append(item)
        if decision["status"]=="FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS": selected = folder
    noise = json.loads((output/"noise-score-diagnostics.json").read_text())
    encoder = noise["encoder"]
    compact = {"scope": "CONDITIONAL_LOCAL_INFORMATION_ONLY", "uncertainty": "UNKNOWN",
        "chronological_fit": LABEL, "source_fitting_model_max_step_s": .000125,
        "fitted_physical_coordinates_changed": False, "coordinate_order": FIELDS,
        "numerical_gate": {"full_derivatives_and_pointwise_bin_fisher_relative": .01,
            "normalized_rank": 10, "normalized_condition_max": 1e6},
        "retained_pairs": summaries, "first_passing_pair": bounded["first_passing_pair"],
        "unused_pairs": bounded["remaining_pairs"],
        "information": "joint expected score Fisher at the same retained fitted vector; no prior/Gauss-Newton substitution",
        "covariance_scope": "conditional local information inverse; source vector was fitted at125us and was not refitted at finer resolution",
        "generator_diagnostics": {"scope": "known independent truth, fresh masks; descriptive consumed data only",
            "encoder": {k: encoder[k] for k in ("fresh_count", "probability_mass_max_error", "true_score_mean_standard_errors",
                "observed_squared_score_over_expected", "true_score_lag1", "observed_offsets_outside_seven_bin_support")},
            "encoder_largest_nonsparse_phase_bin_count_standard_errors": max(abs(b["count_standard_errors"])
                for row in encoder["phase_aware_bins"] for b in row["bin_offsets"] if b["normal_count_approximation"]=="DIAGNOSTIC_ONLY"),
            **{k: {field: noise[k][field] for field in ("fresh_count", "normalized_sample_sd", "mean_standard_errors", "lag1")}
                for k in ("gyro", "current")}},
        "calibrated_joint_coverage": "NOT_RUN", "parameter_uncertainty_ensemble": "NOT_CONSTRUCTED",
        "next_required_evidence": "frozen joint-estimation ensemble under supported noise/gauges plus independent empirical simultaneous parameter and event/horizon trajectory coverage/width; retain optimizer/numerical failures",
        "one_truth_one_train_design": True, "closed_loop_data_statistics": "UNQUALIFIED",
        "inverse_fits": 0, "new_noise_draws": 0, "physical_actions": False, "deployment": False}
    if selected is not None:
        matrix = json.loads((selected/"information-matrices.json").read_text())
        save(output/"conditional-joint-matrices.json", {k: matrix[k] for k in ("fields", "units",
            "expected_score_information", "covariance", "correlation", "standard_errors", "inverse_identity_max_error",
            "inverse_identity_coordinate", "covariance_role", "fitted_score_local_information_norm", "uncertainty")})
        compact["conditional_standard_errors"] = matrix["standard_errors"]
        compact["fitted_score_local_information_norm"] = matrix["fitted_score_local_information_norm"]
        compact["information_matrices_reference"] = "conditional-joint-matrices.json"
    else:
        compact["covariance"] = "NOT_COMPUTED; no frozen numerical pair passed"
    save(output/"compact-decision.json", compact)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "full", "refine"), default="early")
    args = parser.parse_args()
    {"early": early, "full": full, "refine": refine}[args.stage](args.source, args.library, args.output)
