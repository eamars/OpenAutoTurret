"""Fitted family receipt -> actual native readback/FF -> diagnostic forecast."""
from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.family_assets import (FamilyAsset, bind_diagnostic_runtime,
    bind_runtime_document, model_from_document, runtime_document)
from Firmware.commissioning.family_forecast import IndependentForecastPlant, forecast
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters
from Firmware.tools.adr0022_family_forecast_probe import contract, feedforward_fixture


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def fitted_asset(source, c):
    fit = json.loads(source.read_text())
    model = model_from_document(fit["model"])
    support, state = feedforward_fixture(c, model, actuator_policy="STEADY_STATE_REFERENCE",
        actuation_memory_max_s=.060)
    revision = "exact-bin-7103-consumed-fit"
    config = replace(support.configuration_support, model_revision=revision)
    quality = {**fit["gates"], "fresh_excitation_and_noise": "NOT_RUN",
        "free_parameters": fit["free_fields"]}
    asset = FamilyAsset(model_revision=revision, model=model, frame=c.frame, configuration_support=config,
        gauges={"load_friction": {"rule": "fixed total-load offset with separate directional moving friction",
            "load_offset_A_effective": model.load_offset, "load_slope_A_effective_rad": model.load_slope},
            "input_current_map": {"gain_A_effective_per_A_command": model.actuator_gain, "bias_A_effective": model.actuator_bias},
            "reported_current_map": {"gain_A_reported_per_A_effective": model.current_gain, "bias_A_reported": model.current_bias},
            "sensor_biases": {"gyro_rad_s": model.gyro_bias, "encoder_datum": "known synthetic acquisition datum"},
            "static_thresholds": {"negative_A_effective": model.static_negative, "positive_A_effective": model.static_positive,
                "status": "SUPPLIED_SYNTHETIC; NOT_ESTIMATED_IN_THIS_FIT"},
            "acquisition_state": {"value": [0., 0., 0., 0., 0.], "count_per_run": 1, "status": "SUPPLIED_SYNTHETIC"}},
        assumptions={"encoder_noise": {"Gaussian_before_rounding_sigma_rad": .00015, "quantum_rad": 2*np.pi/8192},
            "gyro_noise": {"sigma_rad_s": .005, "fresh_hz": 50., "iid": True},
            "current_noise": {"sigma_A_reported": .002, "fresh_hz": 1000., "iid": True},
            "source_clock": {"scale": 1., "offset_s": 0., "status": "KNOWN_SYNTHETIC_COMMON_CLOCK",
                "gyro_source_age": "model gyro_delay; sensor filter remains dynamic state"}},
        estimator_quality=quality, evidence_partition="DEVELOPMENT_CONSUMED",
        uncertainty={"status": "UNKNOWN", "parameter_sets": [],
            "basis": "one consumed TRAIN fit/local information is not calibrated joint supported uncertainty"},
        provenance={"fit_evidence": str(source.resolve()), "source_paths": [str(Path(__file__).resolve()),
            str(Path("Firmware/tools/adr0022_nuisance_verification.py").resolve())],
            "source_revision": "current dirty review02 implementation",
            "scope": "SYNTHETIC_OFFLINE", "fit_role": fit["role"],
            "evidence_partition": "DEVELOPMENT_CONSUMED"})
    # Known rest balance under the supplied acquisition state, not inferred from quiet observations.
    state = replace(state, configuration_facts=config.baseline,
        static_balance_A=model.actuator_bias-model.load_offset)
    return asset, support, state


def step_packet(c, now):
    from Firmware.commissioning.motor_feedforward import ReferencePacket
    start, duration, step = .1, .3, float(np.deg2rad(1.))
    x = min(max((now-start)/duration, 0.), 1.)
    q = step*(10*x**3-15*x**4+6*x**5)
    v = step/duration*(30*x**2-60*x**3+30*x**4)
    a = step/duration**2*(60*x-180*x**2+120*x**3)
    return ReferencePacket(q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a, time_s=now,
        source_time_s=0., expires_at_s=c.duration_s, frame=c.frame, configuration_id=c.configuration_id,
        trajectory_id=c.trajectory_id, generation=1, fresh=True, valid=True,
        trajectory_phase="HOLD" if now <= start or now >= start+duration else "BRAKING" if a < 0 else "TRACKING")


def run_probe(source, library, output, duration=.6):
    output.mkdir(parents=True, exist_ok=False)
    c = replace(contract(duration=duration), configuration_id="fitted-7103-diagnostic-pipeline",
        trajectory_id="frozen-one-degree-quintic-step")
    asset, ff_support, ff_state = fitted_asset(source, c)
    save(output/"family-asset.json", asset.document())
    loaded = FamilyAsset.from_document(json.loads((output/"family-asset.json").read_text()))
    assert loaded.document() == asset.document()
    native, plant = Native(library), FamilyNative(library)
    template = controller_parameters()
    parameters, ff_support, binding = bind_diagnostic_runtime(loaded, native, template, ff_support)
    receipt = runtime_document(loaded, template, parameters, ff_support, binding)
    save(output/"runtime-family-asset.json", receipt)
    loaded, parameters, ff_support, binding = bind_runtime_document(
        json.loads((output/"runtime-family-asset.json").read_text()), native)
    assert runtime_document(loaded, template, parameters, ff_support, binding) == receipt
    initial = np.asarray(loaded.gauges["acquisition_state"]["value"], dtype=float)
    declaration = {"asset_revision": loaded.model_revision, "estimated_parameters_reused": True,
        "model_qualification": loaded.qualification, "uncertainty": "UNKNOWN",
        "candidate": binding, "runtime_receipt": "runtime-family-asset.json", "forecast": vars(c),
        "reference": "one1deg quintic step0.1..0.4s; coherent q/v/a",
        "full_quality": "NOT_RUN; thin prefix lacks fixed2s stop/full manoeuvre coverage",
        "physical_travel": "UNKNOWN", "FF_support_rad": [ff_support.q_min_rad, ff_support.q_max_rad],
        "numerical_rollout_guard_rad": [loaded.model.q_min, loaded.model.q_max],
        "numerical_gates": {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9},
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"predeclared-contract.json", declaration)
    rows = []
    for name, backend in (("native", plant), ("independent", IndependentForecastPlant())):
        data = forecast(native, backend, loaded.model, parameters, c, lambda now: step_packet(c, now),
            initial=initial, feedforward_support=ff_support, feedforward_state=ff_state)
        comparison = (independent_rollout(loaded.model, data["t"], data["tx_t"], data["tx_A"], initial).trace
            if name == "native" else plant.rollout(loaded.model, data["t"], data["tx_t"], data["tx_A"], initial))
        errors = {"q_rms_rad": float(np.sqrt(np.mean((data["truth"][:,0]-comparison[:,0])**2))),
            "gyro_rms_rad_s": float(np.sqrt(np.mean((data["truth"][data["v_new"],3]-comparison[data["v_new"],3])**2))),
            "current_rms_A": float(np.sqrt(np.mean((data["truth"][:,4]-comparison[:,4])**2)))}
        np.savez_compressed(output/(name+".npz"), **{k: v for k, v in data.items() if isinstance(v, np.ndarray)},
            conditional_other_backend_truth=comparison)
        row = {"backend": name, "outcome": data["report"]["outcome"], "forward_errors": errors,
            "numerical_interface_pass": all(errors[k] <= value for k, value in declaration["numerical_gates"].items()),
            "causal_replay_max_error": data["report"]["causal_sensor_replay_max_error"],
            "successful_TX_count": len(data["tx_t"])-1, "one_initial_state": data["report"]["plant_state_initializations"],
            "future_TX_substituted": data["report"]["future_realized_inputs_substituted"],
            "future_measured_state_substituted": data["report"]["future_measured_state_substituted"]}
        rows.append(row)
        print(json.dumps(row), flush=True)
    summary = {"scope": "FITTED_RECEIPT_NATIVE_FF_FORECAST_INTERFACE_ONLY", "cases": rows,
        "receipt_roundtrip": "PASS", "runtime_receipt_roundtrip_rebind": "PASS",
        "runtime_receipt": "runtime-family-asset.json", "actual_runtime_readback": binding,
        "estimated_parameters_reused": True, "model_qualified": False, "controller_qualified": False,
        "full_quality": "NOT_RUN", "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN",
        "deployment_authorized": False}
    save(output/"summary.json", summary)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run_probe(args.fit, args.library, args.output)
