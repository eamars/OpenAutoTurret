"""Native fixed-template FamilyModel information-selection probe; no hardware."""
from dataclasses import asdict, replace
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import Envelope, FailurePolicy
from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.family_information import (FamilyInformationCell, FamilyInformationNoise,
    FamilyInformationSupport, select_family_supplemental)
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_family_synthesis_probe import dynamic_candidate_fixture


def fixture(candidate):
    _, fixtures = dynamic_candidate_fixture(candidate)
    return fixture_inputs(fixtures[0][0])


def fixture_inputs(model):
    models = (model, replace(model, a=model.a*.95, viscous=model.viscous*1.05),
              replace(model, a=model.a*1.05, viscous=model.viscous*.95))
    # All are supplied synthetic bounds, never allowed station values.
    envelope = Envelope(.35, 2., .8, 3., 300., -1., 1., 9., True, "SYNTHETIC")
    source = "predeclared synthetic family-information fixture; not station authorization"
    support = FamilyInformationSupport(configuration_id="synthetic-family-forecast",
        model_revision="supplied-dynamic-stribeck-information-fixture-1", qualification="SYNTHETIC_FIXTURE",
        evidence={k: source for k in ("current", "slew", "speed", "acceleration", "jerk", "travel",
            "winding", "thermal", "supply", "timeout", "adequate_stop")},
        stop_command_A=0., stop_hold_s=2., rest_speed_rad_s=.02, max_stop_drift_rad=float(np.deg2rad(.15)))
    noise = FamilyInformationNoise(sample_hz=100., periods=(1, 5, 2), sigma=(.00015, .005, .002),
        source="declared independent unquantized Gaussian output noise; known filter/timing nuisances")
    cells = tuple(FamilyInformationCell(cell_id=f"rest-q0-direction{direction}", initial=(0., 0., 0., 0., 0.),
        baseline_command_A=0., direction=direction, regime="REST_TO_MOVING") for direction in (-1, 1))
    fields = ("a", "viscous", "coulomb_negative", "coulomb_positive")
    bounds = {"a": (.06, .15), "viscous": (.015, .12), "coulomb_negative": (.07, .15), "coulomb_positive": (.07, .15)}
    scales, steps, prior = np.array([.1, .06, .12, .12]), np.full(4, 1e-5), np.eye(4)*.01
    return model, models, cells, envelope, support, noise, fields, bounds, scales, steps, prior


def run(library, candidate, output):
    output.mkdir(parents=True, exist_ok=False)
    args = fixture(candidate)
    model, models, cells, envelope, support, noise, fields, bounds, scales, steps, prior = args
    declaration = {"schema": "adr0022.family-information-probe/1", "qualification": "SYNTHETIC_OFFLINE_ONLY",
        "candidate": str(candidate), "library": str(library), "model": model.document(),
        "supported_models": [m.document() for m in models], "supported_set_role": "DECLARED_FINITE_SYNTHETIC_SET; NOT_CALIBRATED_CONFIDENCE",
        "cells": [asdict(c) for c in cells], "envelope": asdict(envelope), "support": asdict(support), "noise": asdict(noise),
        "fields": fields, "parameter_units": ["A-effective*s2/rad", "A-effective*s/rad", "A-effective", "A-effective"],
        "coordinate_scales": scales.tolist(), "absolute_derivative_steps": steps.tolist(), "bounds": bounds,
        "prior_information": prior.tolist(), "prior_units": "xi=physical_parameter/coordinate_scale; dimensionless",
        "selector": "existing template0..31, marginal logdet gain / complete occupancy time",
        "tie_policy": "context-first (cell_id, direction, case_id), preserving existing select_supplemental",
        "feasibility_scope": "sampled synthetic q/v and 100Hz gradient a/jerk screening; not continuous physical-bound proof",
        "amplitude_factors": [1., 1.5, 2., 3.],
        "amplitude_policy": "highest feasible informative descending factor; supplied envelope unchanged",
        "budget": {"maximum_information_rounds_per_change": 2, "maximum_cases_per_round": 3},
        "oracle_gates": {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9},
        "physical_identification": "NOT_RUN", "physical_safe_stop": "NOT_RUN", "deployment_authorized": False}
    (output/"predeclared-contract.json").write_text(json.dumps(declaration, indent=2)+"\n")
    native, policy = FamilyNative(library), FailurePolicy()
    result = select_family_supplemental(native, *args, policy)
    selected, reports = result["selected"], []
    for index, row in enumerate(selected):
        oracle = independent_rollout(model, row["time"], row["successful_tx_t"], row["successful_tx"], row["initial"]).trace
        errors = {name: float(np.sqrt(np.mean((row["prediction"][::period, column]-oracle[::period, column])**2)))
            for name, column, period in (("q_rms_rad", 0, 1), ("gyro_rms_rad_s", 3, 5), ("current_rms_A", 4, 2))}
        reports.append({"case_id": row["case_id"], "cell_id": row["cell_id"], "score": row["score"],
            "regime": row["regime"], "direction": row["direction"], "amplitude_divisor": row["amplitude_divisor"],
            "occupancy_time_s": float(row["time"][-1]),
            "checks": row["checks"], "predicted_stop_drift_rad": row["predicted_stop_drift_rad"],
            "moving_duration_s": row["moving_duration_s"], "predicted_directions": row["predicted_directions"],
            "command_dose_A2s": row["command_dose_A2s"], "dose_role": row["dose_role"],
            "independent_forward_errors": errors, "independent_forward_passed": all(errors[k] <= v for k, v in declaration["oracle_gates"].items())})
        np.savez_compressed(output/f"selected-{index}.npz", **{k: v for k, v in row.items() if isinstance(v, np.ndarray)},
                            independent_forward=oracle)
    blocked = []
    for label, changed in (("unknown-stop", replace(support, evidence={**support.evidence, "adequate_stop": ""})),
                           ("inadequate-predicted-stop", replace(support, stop_command_A=-.15)),
                           ("quantized-noise", noise)):
        altered = list(args)
        if label == "quantized-noise": altered[5] = replace(noise, encoder_quantum_rad=2*np.pi/8192)
        else: altered[4] = changed
        try:
            select_family_supplemental(native, *altered, FailurePolicy())
        except Rejected as exc:
            blocked.append({"case": label, "reason": exc.reason.value, "detail": str(exc)})
        else:
            raise AssertionError("unsupported fixture unexpectedly selected: "+label)
    repeat = select_family_supplemental(native, *args, policy)
    deterministic = [r["selection_id"] for r in repeat["selected"]] == [r["selection_id"] for r in selected]
    try:
        select_family_supplemental(native, *args, policy)
    except Rejected as exc:
        budget = {"reason": exc.reason.value, "detail": str(exc), "rounds": policy.information_rounds}
    else: raise AssertionError("third information round was accepted")
    report = {"selector": result["selector"], "tie_policy": result["tie_policy"],
        "feasibility_scope": result["feasibility_scope"], "selected": reports, "rejected_candidates": result["rejected"],
        "selected_count": len(selected), "deterministic_two_rounds": deterministic, "budget_exhaustion": budget,
        "unsupported_cases": blocked, "local_rank": result["local_rank"], "coordinate_count": result["coordinate_count"],
        "information_units": result["information_units"], "physical_identifiability": False,
        "physical_safe_stop": "NOT_RUN", "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN",
        "deployment_authorized": False, "passed": bool(len(selected) <= 3 and deterministic
            and all(r["independent_forward_passed"] for r in reports) and policy.information_rounds == 2)}
    (output/"result.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k: v for k, v in report.items() if k != "rejected_candidates"}), flush=True)
    return 0 if report["passed"] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    return run(args.library, args.candidate, args.output_dir)


if __name__ == "__main__":
    raise SystemExit(main())
