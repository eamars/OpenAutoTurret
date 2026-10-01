"""Offline independent-forward verification of declared synthetic families."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.model_family import FamilyModel, FamilyNative
from Firmware.commissioning.synthetic_family_oracle import independent_rollout


NATIVE_STEPS = (.001, .0005, .00025, .000125, .0000625)
NUMERICAL_GATES = {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9}
ORACLE_REFINEMENT_GATES = {"q_rms_rad": 1e-8, "gyro_rms_rad_s": 1e-7, "current_rms_A": 1e-10}


def declared_fixtures():
    """Fixed independent fixtures; no physical parameter or authority claim."""
    base = FamilyModel(a=.10, viscous=.06, coulomb_negative=.12, coulomb_positive=.12,
        static_negative=.16, static_positive=.16, load_offset=.02,
        q_min=-2., q_max=2., actuator_gain=1., actuator_bias=0., transport_delay=.0087,
        gyro_bias=0., gyro_tau=.015, gyro_delay=.0043,
        current_gain=1., current_bias=0., current_tau=.012, current_delay=.0029, max_step=.00025)
    # Both signs, no-start hold, rest/restart, reversal, and final stop are all declared.
    times = np.array([-.10, .0513, .1937, .2621, .3932, .5311, .7039, .8577,
                       1.0093, 1.1137, 1.2311, 1.4])
    commands = np.array([0., .23, 0., -.23, 0., .13, .24, -.24, 0., .23, 0., 0.])
    return [
        {"label": "first_order_coulomb", "model": replace(base, actuator="first_order", actuator_tau=.0073),
         "t": np.arange(1601) * .001, "tx_t": times, "tx_A": commands, "initial": np.zeros(5)},
        {"label": "algebraic_stribeck", "model": replace(base, friction="stribeck", stribeck_negative=.07,
             stribeck_positive=.05, static_negative=.16, static_positive=.18,
             coulomb_negative=.11, coulomb_positive=.12),
         "t": np.arange(1601) * .001, "tx_t": times, "tx_A": commands, "initial": np.zeros(5)}]


def error_metrics(left, right):
    return {name: {"rms": float(np.sqrt(np.mean((left[:, index] - right[:, index]) ** 2))),
                   "max_abs": float(np.max(np.abs(left[:, index] - right[:, index])))}
            for name, index in [("q", 0), ("latent_v", 1), ("effective_current", 2),
                                ("gyro", 3), ("reported_current", 4)]}


def passes(errors, gates):
    return errors["q"]["rms"] <= gates["q_rms_rad"] and \
        errors["gyro"]["rms"] <= gates["gyro_rms_rad_s"] and \
        errors["reported_current"]["rms"] <= gates["current_rms_A"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output_dir
    if output.exists() and any(output.iterdir()): raise SystemExit("refusing to overwrite retained oracle evidence")
    output.mkdir(parents=True, exist_ok=True)
    fixtures = declared_fixtures()
    contract = {"schema": "adr0022.independent-family-oracle-contract/1", "provenance": "SYNTHETIC",
        "scope": "independent forward verification only; not estimator, model selection, physical qualification, or gains",
        "fixtures": [{"label": f["label"], "parameters": asdict(f["model"]),
            "observation_dt_s": .001, "duration_s": 1.6, "tx_t_s": f["tx_t"].tolist(),
            "tx_A": f["tx_A"].tolist(), "one_initial_state": f["initial"].tolist()} for f in fixtures],
        "native_steps_s": NATIVE_STEPS, "native_numerical_gates": NUMERICAL_GATES,
        "oracle_refinement_gates": ORACLE_REFINEMENT_GATES,
        "oracle_primary": {"rtol": 1e-10, "atol": 1e-12, "max_step_s": .0005},
        "oracle_refinement": {"rtol": 1e-12, "atol": 1e-14, "max_step_s": .000125},
        "budget": {"fixtures": 2, "oracle_rollouts_per_fixture": 2, "native_rollouts_per_fixture": 5},
        "sampling": "1 kHz encoder/current, 50 Hz gyro freshness retained in saved fixture; continuous trace also retained",
        "noisy_estimation": "NOT_RUN; noiseless numerical oracle comparison is separate from estimator recovery",
        "plant_sources": ["independent exact matrix exponential Coulomb modes with root-solved zero crossings",
                          "independent DOP853 Stribeck integration with directional zero-speed and static-release events"]}
    (output / "predeclared-contract.json").write_text(json.dumps(contract, indent=2) + "\n")
    native = FamilyNative(args.library)
    results = []
    began = time.monotonic()
    for fixture in fixtures:
        f, model = fixture, fixture["model"]
        oracle = independent_rollout(model, f["t"], f["tx_t"], f["tx_A"], f["initial"])
        refined = independent_rollout(model, f["t"], f["tx_t"], f["tx_A"], f["initial"],
                                      rtol=1e-12, atol=1e-14, max_step=.000125)
        refinement = error_metrics(oracle.trace, refined.trace)
        filename = f"{f['label']}-independent.npz"
        np.savez_compressed(output / filename, t=f["t"], tx_t=f["tx_t"], tx_A=f["tx_A"],
            initial=f["initial"], truth=refined.trace, primary_truth=oracle.trace,
            q=refined.trace[:, 0], v=refined.trace[:, 3], current=refined.trace[:, 4],
            q_new=np.ones(len(f["t"]), dtype=bool), v_new=np.arange(len(f["t"])) % 20 == 0,
            current_new=np.ones(len(f["t"]), dtype=bool))
        comparisons = []
        for step in NATIVE_STEPS:
            prediction = native.rollout(replace(model, max_step=step), f["t"], f["tx_t"], f["tx_A"], f["initial"])
            errors = error_metrics(prediction, refined.trace)
            np.savez_compressed(output / f"{f['label']}-native-step-{step:.7f}.npz",
                                t=f["t"], prediction=prediction)
            comparisons.append({"max_step_s": step, "errors": errors,
                                "numerical_gate_passed": passes(errors, NUMERICAL_GATES)})
        at_default = next(row for row in comparisons if row["max_step_s"] == model.max_step)
        result = {"label": f["label"], "independent_data": filename,
            "oracle_primary_diagnostics": oracle.diagnostics, "oracle_refined_diagnostics": refined.diagnostics,
            "oracle_events": refined.events, "oracle_refinement_errors": refinement,
            "oracle_refinement_passed": passes(refinement, ORACLE_REFINEMENT_GATES),
            "native_comparisons": comparisons, "native_default_numerics_passed": at_default["numerical_gate_passed"],
            "independent_forward_verified": passes(refinement, ORACLE_REFINEMENT_GATES) and at_default["numerical_gate_passed"],
            "estimator_qualification": "NOT_RUN", "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN",
            "deployment_authorized": False}
        results.append(result)
        (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
        print(json.dumps({"label": f["label"], "oracle_refinement_passed": result["oracle_refinement_passed"],
              "default_errors": at_default["errors"], "independent_forward_verified": result["independent_forward_verified"]}), flush=True)
    (output / "summary.json").write_text(json.dumps({"scope": contract["scope"], "elapsed_s": time.monotonic() - began,
        "independent_forward_verified": all(r["independent_forward_verified"] for r in results),
        "provenance": "SYNTHETIC", "deployment_authorized": False}, indent=2) + "\n")
    return 0 if all(r["independent_forward_verified"] for r in results) else 2


if __name__ == "__main__": raise SystemExit(main())
