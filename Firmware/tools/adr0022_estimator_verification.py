"""Predeclared offline verification of the repaired four-coordinate estimator.

Independent analytic mechanics and actual native feedback are reused from the
frozen probe. No physical calibration, hardware access, or controller gains result.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.tools import adr0022_closed_loop_estimator_probe as probe
from Firmware.commissioning.model_family import FamilyNative, FamilyRun, fit_family
from Firmware.commissioning.native import Native

FINAL_SEEDS = (1019, 1237, 1423)
DEVELOPMENT_SEEDS = (17, 41, 83, 131, 227, 419)
REFERENCE_NAMES = ("original_multisine", "quintic_hold_reverse", "chirp")
NEW_REFERENCE_NAMES = ("multisine_alternate", "quintic_alternate", "chirp_descending")


def quintic_hold_reverse(t):
    # Positions/knots are fixed before noise or optimization; derivative pairs
    # come from the same polynomial reference, including complete holds.
    knots = ((0., 0.), (.5, 0.), (2., .30), (3., .30), (5., -.30),
             (5.5, -.30), (7., .15), (10., .15))
    for (start, q0), (end, q1) in zip(knots, knots[1:]):
        if start <= t < end:
            duration = end - start
            s = (t - start) / duration
            delta = q1 - q0
            return np.array([q0 + delta*(10*s**3 - 15*s**4 + 6*s**5),
                delta*(30*s**2 - 60*s**3 + 30*s**4)/duration,
                delta*(60*s - 180*s**2 + 120*s**3)/duration**2, 0.])
    return np.array([knots[-1][1], 0., 0., 0.])


def chirp(t):
    if t <= .5:
        return np.zeros(4)
    x = t - .5
    phase = 2*math.pi*(.12*x + .5*.03*x*x)
    omega, alpha, amplitude = 2*math.pi*(.12 + .03*x), 2*math.pi*.03, .26
    return np.array([amplitude*math.sin(phase), amplitude*omega*math.cos(phase),
        amplitude*(alpha*math.cos(phase)-omega*omega*math.sin(phase)), 0.])


def multisine_alternate(t):
    if t <= .5:
        return np.zeros(4)
    x = t-.5
    frequencies = 2*math.pi*np.array([.13, .43, .91])
    amplitudes = np.array([.27, .045, .02])
    return np.array([float(amplitudes @ np.sin(frequencies*x)),
        float((amplitudes*frequencies) @ np.cos(frequencies*x)),
        float((-amplitudes*frequencies**2) @ np.sin(frequencies*x)), 0.])


def quintic_alternate(t):
    knots = ((0., 0.), (.5, 0.), (2.25, .24), (3.5, .24), (5.75, -.20),
             (6.25, -.20), (7.75, .32), (10., .32))
    for (start, q0), (end, q1) in zip(knots, knots[1:]):
        if start <= t < end:
            duration, delta = end-start, q1-q0
            s = (t-start)/duration
            return np.array([q0+delta*(10*s**3-15*s**4+6*s**5),
                delta*(30*s**2-60*s**3+30*s**4)/duration,
                delta*(60*s-180*s**2+120*s**3)/duration**2, 0.])
    return np.array([knots[-1][1], 0., 0., 0.])


def chirp_descending(t):
    if t <= .5:
        return np.zeros(4)
    x = t-.5
    phase = 2*math.pi*(.39*x-.5*.025*x*x)
    omega, alpha, amplitude = 2*math.pi*(.39-.025*x), -2*math.pi*.025, .26
    return np.array([amplitude*math.sin(phase), amplitude*omega*math.cos(phase),
        amplitude*(alpha*math.cos(phase)-omega*omega*math.sin(phase)), 0.])


REFERENCES = {"original_multisine": probe.reference,
              "quintic_hold_reverse": quintic_hold_reverse, "chirp": chirp,
              "multisine_alternate": multisine_alternate, "quintic_alternate": quintic_alternate,
              "chirp_descending": chirp_descending}


def fixed_input_noisy(data, seed):
    """Inject full noise after freezing pristine successful TX; no input feedback."""
    out = {key: value.copy() if isinstance(value, np.ndarray) else value for key, value in data.items()}
    rng = np.random.default_rng(seed)
    for k, now in enumerate(out["t"]):
        q_noise = rng.normal(0, probe.FIXTURE["encoder_noise"])
        out["q"][k] = np.round((out["truth"][k, 0] + q_noise) / probe.FIXTURE["encoder_quantum"]) * probe.FIXTURE["encoder_quantum"]
        out["current"][k] = out["truth"][k, 2] + rng.normal(0, probe.FIXTURE["current_noise"])
        if out["v_new"][k]:
            gyro_truth = np.interp(now-probe.FIXTURE["gyro_delay"], out["t"], out["truth"][:, 3])
            out["v"][k] = gyro_truth + rng.normal(0, probe.FIXTURE["gyro_noise"])
        else:
            out["v"][k] = out["v"][k-1]
    out.update(seed=seed, noisy=True, noise_sources=np.asarray(probe.NOISE_SOURCES, dtype="U32"))
    return out


def fit_one(native, label, data, budget, targets, output):
    true = probe.estimator_model()
    initial = replace(true, a=.085, viscous=.075, coulomb_negative=.105, coulomb_positive=.13)
    run = FamilyRun(run_id=label, source_id="independent-analytic-oracle/"+label,
        t=data["t"], q=data["q"], v=data["v"], current=data["current"],
        q_new=data["q_new"], v_new=data["v_new"], current_new=data["current_new"],
        tx_t=data["tx_t"], tx_A=data["tx_A"], initial=np.zeros(5), provenance="SYNTHETIC",
        configuration_id="declared-coulomb-analytic-fixture", calibration_revision="known-synthetic-sensors",
        encoder_quantum=probe.FIXTURE["encoder_quantum"] if "encoder_quantization" in set(data["noise_sources"]) else 0.,
        **probe.sensor_scales(data))
    fit = fit_family(native, initial, [run], bounds=probe.FREE_BOUNDS, max_nfev=budget)
    model = fit["model"]
    relative = {key: (getattr(model, key)-probe.FIXTURE[key])/probe.FIXTURE[key] for key in probe.FREE_BOUNDS}
    oracle_prediction = native.rollout(true, data["t"], data["tx_t"], data["tx_A"], np.zeros(5))
    oracle = probe.numerical_oracle_errors(data, oracle_prediction)
    predictions = []
    for target_label, target in targets.items():
        prediction = native.rollout(model, target["t"], target["tx_t"], target["tx_A"], np.zeros(5))
        errors = probe.measurement_errors(target, prediction)
        row = {"source": target_label, "role": "training_whole_run" if target_label == label else "blocked_whole_seeded_run_holdout",
               "prediction_errors": errors, "trajectory_gate": probe.trajectory_gate(target, errors)}
        predictions.append(row)
        np.savez_compressed(output / (label+"-predict-"+target_label+".npz"), prediction=prediction)
    recovery = probe.RECOVERY_GATES["noisy_relative"]
    gates = probe.case_gates(data, fit["optimizer"], relative, recovery, fit["optimizer"]["parameter_bound_hits"], predictions, oracle)
    passed = all(gates[key] == "PASS" for key in ("data_integrity", "forward_numerics", "synthetic_parameter_recovery", "training_trajectory", "selection_trajectory")) and gates["optimizer_converged"]
    result = {"case": label, "fitted_model": model.document(), "relative_parameter_error": relative,
              "optimizer": fit["optimizer"], "independent_truth_forward_numerics": oracle,
              "gates": gates, "predictions": predictions, "passed": passed}
    probe.save(output / (label+"-fit.json"), result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=list(FINAL_SEEDS))
    parser.add_argument("--partition", choices=("development", "final"), default="final")
    parser.add_argument("--references", nargs="+", choices=tuple(REFERENCES), default=list(NEW_REFERENCE_NAMES))
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(seed < 0 for seed in args.seeds):
        parser.error("Use unique nonnegative predeclared seeds")
    if args.partition == "final" and set(args.seeds) & set(DEVELOPMENT_SEEDS):
        parser.error("Consumed development seeds cannot be labelled independent final verification")
    if len(set(args.references)) != len(args.references):
        parser.error("Use unique predeclared references")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("Use a fresh output directory")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    probe.save(args.output_dir / "predeclared-contract.json", {
        "provenance": "SYNTHETIC", "development_seed": 41, "development_duration_s": 8,
        "development_noise_ablations": list(probe.NOISE_SOURCES)+["all_feedback", "all_fixed_input"],
        "final_seeds": args.seeds, "final_references": args.references, "final_duration_s": 10,
        "partition_role": args.partition,
        "reference_partition": ("Fresh reference cases not used for estimator repair" if set(args.references) <= set(NEW_REFERENCE_NAMES)
                                else "Includes development-consumed reference cases; fresh seeds alone do not make their waveforms untouched"),
        "final_partition": ("Seeds not used to select estimator repair; procedure frozen before generation" if args.partition == "final" else
                            "Consumed development regressions; this replay is not independent final verification"),
        "quality_gates": probe.RECOVERY_GATES, "free_bounds": probe.FREE_BOUNDS,
        "initial_free_parameters": {"a": .085, "viscous": .075, "coulomb_negative": .105, "coulomb_positive": .13},
        "optimizer_budget": 120, "cross_prediction": "Each final fit uses one training trajectory, then predicts all declared final trajectories without resetting state",
        "native_library": str(args.library), "physical_actions": False, "deployment_authorized": False,
        "limits": "Four mechanics coordinates only; known fixed Coulomb/static/current/sensor nuisances; does not qualify other families, both-axis coupling or physical plant"})
    control = Native(args.library)
    native = FamilyNative(args.library)
    development = {}
    for source in probe.NOISE_SOURCES:
        development["ablation-"+source] = probe.generate(control, 8., 41, True, noise_sources=(source,))
    development["all_feedback"] = probe.generate(control, 8., 41, True)
    development["all_fixed_input"] = fixed_input_noisy(probe.generate(control, 8., 17, False), 41)
    for label, data in development.items():
        np.savez_compressed(args.output_dir / (label+".npz"), **data)
    dev_results = [fit_one(native, label, data, 120, {label: data}, args.output_dir) for label, data in development.items()]
    probe.save(args.output_dir / "development-results.json", dev_results)
    # No optimization/procedure change between development and final generation.
    final = {}
    for reference in args.references:
        for seed in args.seeds:
            label = reference+"-seed-"+str(seed)
            final[label] = probe.generate(control, 10., seed, True, reference_function=REFERENCES[reference])
            np.savez_compressed(args.output_dir / (label+".npz"), **final[label])
    results = []
    for label, data in final.items():
        result = fit_one(native, label, data, 120, final, args.output_dir)
        results.append(result)
        probe.save(args.output_dir / "final-results.json", results)
        print(json.dumps({"case": label, "passed": result["passed"], "evaluations": result["optimizer"]["evaluations"], "gates": result["gates"]}), flush=True)
    passed = all(r["passed"] for r in results)
    probe.save(args.output_dir / "summary.json", {
        "schema": "adr0022.estimator-independent-verification/1", "provenance": "SYNTHETIC",
        "partition_role": args.partition,
        "development_passes": sum(r["passed"] for r in dev_results), "development_cases": len(dev_results),
        "final_passes": sum(r["passed"] for r in results), "final_cases": len(results),
        "final_trajectory_passes": sum(p["trajectory_gate"]["passed"] for r in results for p in r["predictions"]),
        "final_trajectory_comparisons": sum(len(r["predictions"]) for r in results),
        "final_verification_passed": passed if args.partition == "final" else "NOT_RUN",
        "development_retest_passed": passed if args.partition == "development" else "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False})
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
