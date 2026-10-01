"""Fresh fitted family receipts -> selected-family synthesis -> diagnostic Product B.

All three retained fits are mandatory.  Their uncertainty remains UNKNOWN; the
controller trials are consumed development, conditional on each fitted plant.
"""
from dataclasses import asdict, replace
import argparse
import ctypes as ct
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, require
from Firmware.commissioning.family_analysis import SlidingPoint, SlidingSupport
from Firmware.commissioning.family_assets import (FamilyAsset, QUALITY_GATES,
    bind_diagnostic_runtime, bind_runtime_document, model_from_document,
    native_parameter_document, runtime_document)
from Firmware.commissioning.family_forecast import forecast, synthetic_motion_metrics
from Firmware.commissioning.family_sampled_analysis import LocalGains, SampleSchedule
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import CParameters, Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.commissioning.synthesis import selected_family_solve
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters
from Firmware.tools.adr0022_family_forecast_probe import (contract, feedforward_fixture,
    phased_planned_packet, velocity_plan)
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters
from Firmware.tools.adr0022_start_policy_synthesis_probe import (FORWARD_GATES,
    ORIGINAL_JITTER_RAD, REVISED_JITTER_RAD, PLATEAU_CASES, case_label, case_passed, start_policy)


METHOD = "adr0022.fresh-family-control-development/1"
FIT_CASES = tuple(f"joint-final-mixed-720s-noisy-{seed}" for seed in (10103, 10301, 10613))
FREE_FIELDS = ("a", "viscous", "coulomb_negative", "coulomb_positive", "actuator_tau",
    "transport_delay", "gyro_tau", "gyro_delay", "current_tau", "current_delay")
WN_GRID = (.5, 1., 2., 4.)
DAMPING_GRID = (1., 1.5, 2., 2.5, 3., 4.)
FF_POLICY = "SHARED_POSTERIOR_STEADY_STATE_REFERENCE"


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def load_fits(source):
    records = json.loads(source.read_text())
    require(isinstance(records, list) and tuple(r.get("training_case") for r in records) == FIT_CASES,
        Reason.DATA_INVALID, "all three exact retained fresh fits, in frozen order, are required")
    for record in records:
        require(tuple(record.get("free_fields", ())) == FREE_FIELDS and
            record.get("synthetic_scope_verified") is True and
            record.get("inverse_probe_passed") is True and
            record["gates"].get("optimizer_converged") is True and
            all(record["gates"].get(k) == "PASS" for k in QUALITY_GATES) and
            record["gates"].get("physical_stage3a") == record["gates"].get("physical_stage3b") == "NOT_RUN" and
            record["gates"].get("deployment_authorized") is False,
            Reason.DATA_INVALID, "each retained fit must carry the actual separate fresh estimator predicates")
        model = model_from_document(record["model"])
        require(model.structure == "rigid-yaw/first_order/coulomb/constant" and model.max_step == .000125,
            Reason.DATA_INVALID, "retain fitted Coulomb family and its declared numerical resolution")
    return records


def context(record):
    seed = int(record["training_case"].rsplit("-", 1)[1])
    return replace(contract(seed=101, noisy=True), frame="output_shaft_rad",
        configuration_id=f"fresh-fit-{seed}-synthetic-control",
        trajectory_id="frozen-signed-five-degree-per-second-plateaux")


def fitted_asset(source, record):
    model = model_from_document(record["model"])
    c = context(record)
    policy = start_policy(.060, c.configuration_id, positive_excess=.080)
    support, state = feedforward_fixture(c, model, actuator_policy="STEADY_STATE_REFERENCE",
        actuation_memory_max_s=.03, start_policy=policy, planned_start_program=None)
    revision = "fresh-exact-bin-"+record["training_case"].rsplit("-", 1)[1]
    config = replace(support.configuration_support, model_revision=revision)
    quality = {**record["gates"], "fresh_excitation_and_noise": "PASS",
        "free_parameters": record["free_fields"], "synthetic_scope_verified": True,
        "gate_evidence": {k: str(source.resolve())+"#"+record["training_case"] for k in
            (*QUALITY_GATES, "fresh_excitation_and_noise")}}
    asset = FamilyAsset(model_revision=revision, model=model, frame=c.frame, configuration_support=config,
        gauges={"load_friction": {"rule": "fixed total-load offset with separate directional moving friction",
            "load_offset_A_effective": model.load_offset, "load_slope_A_effective_rad": model.load_slope},
            "input_current_map": {"gain_A_effective_per_A_command": model.actuator_gain, "bias_A_effective": model.actuator_bias},
            "reported_current_map": {"gain_A_reported_per_A_effective": model.current_gain, "bias_A_reported": model.current_bias},
            "sensor_biases": {"gyro_rad_s": model.gyro_bias, "encoder_datum": "known synthetic acquisition datum"},
            "static_thresholds": {"negative_A_effective": model.static_negative, "positive_A_effective": model.static_positive,
                "status": "SUPPLIED_SYNTHETIC; NOT_ESTIMATED_IN_THIS_FIT"},
            "acquisition_state": {"value": [0.]*5, "count_per_run": 1, "status": "SUPPLIED_SYNTHETIC"}},
        assumptions={"encoder_noise": {"Gaussian_before_rounding_sigma_rad": .00015, "quantum_rad": 2*np.pi/8192},
            "gyro_noise": {"sigma_rad_s": .005, "fresh_hz": 50., "iid": True},
            "current_noise": {"sigma_A_reported": .002, "fresh_hz": 1000., "iid": True},
            "source_clock": {"scale": 1., "offset_s": 0., "status": "KNOWN_SYNTHETIC_COMMON_CLOCK",
                "gyro_source_age": "fitted model gyro_delay; filter remains a dynamic state"}},
        estimator_quality=quality, evidence_partition="FINAL_FRESH",
        uncertainty={"status": "UNKNOWN", "parameter_sets": [],
            "basis": "three independent noise fits at one synthetic truth point are not a calibrated joint confidence set"},
        provenance={"fit_evidence": str(source.resolve())+"#"+record["training_case"],
            "source_paths": [str(Path(__file__).resolve()),
                str(Path("Firmware/tools/adr0022_nuisance_verification.py").resolve())],
            "source_revision": "adr0022.joint10.known-gaussian-bins/1 frozen fresh final",
            "scope": "SYNTHETIC_OFFLINE", "fit_role": "FINAL_FRESH_TRAIN_FIT_WITH_HELD_OUT_PRODUCT_A_GATES",
            "evidence_partition": "FINAL_FRESH", "protocol_frozen_before_observations": True,
            "control_evidence_partition": "DEVELOPMENT_CONSUMED"})
    support = replace(support, configuration_support=config)
    state = replace(state, configuration_facts=config.baseline,
        static_balance_A=model.actuator_bias-model.load_offset)
    return asset, support, state


def synthesis_fixture(asset, support):
    template = controller_parameters()
    nominal = LocalGains(template.kp, template.ki, template.kpos, template.kaw)
    schedule = SampleSchedule(dt_s=.005, encoder_period=1, gyro_period=4,
        encoder_age_s=0., gyro_availability_age_s=0., encoder_quantum_rad=0.,
        gyro_quantum_rad_s=0., immediate_successful_ack=True, limits_inactive=True)
    points, supports = [], []
    for direction in (-1, 1):
        points.append(SlidingPoint(q_rad=0., v_rad_s=direction*float(np.deg2rad(5.)),
            configuration_id=support.configuration_id, frame=asset.frame))
        supports.append(SlidingSupport(configuration_id=support.configuration_id, frame=asset.frame,
            q_min_rad=-1., q_max_rad=1., v_min_rad_s=-.3 if direction < 0 else .02,
            v_max_rad_s=-.02 if direction < 0 else .3))
    return template, nominal, schedule, tuple(points), tuple(supports)


def local_solve(asset, support):
    template, nominal, schedule, points, supports = synthesis_fixture(asset, support)
    result = selected_family_solve(asset.model, points, supports, template.observer, nominal, schedule,
        wn_grid=WN_GRID, ff_policy=FF_POLICY, damping_ratio_grid=DAMPING_GRID,
        phase_required_deg=45., gain_required_db=6.)
    return result, template


def declaration(source, library, records, assets):
    return {"schema": METHOD, "scope": "PRODUCT_B_FITTED_PLANT_CONDITIONAL_DEVELOPMENT_ONLY",
        "fit_source": str(source.resolve()), "fit_cases": list(FIT_CASES),
        "models": [r["model"] for r in records], "model_revisions": [a.model_revision for a in assets],
        "library": str(library.resolve()), "mandatory_control_cases_per_asset": list(PLATEAU_CASES),
        "WN_grid_rad_s": list(WN_GRID), "damping_ratio_grid": list(DAMPING_GRID),
        "phase_required_deg": 45., "gain_required_db": 6., "ff_policy": FF_POLICY,
        "actuation_memory_max_s": .03, "negative_START_excess_A": .060, "positive_START_excess_A": .080,
        "START_static_intervals_A": [.156, .164], "START_attempt_s": .2, "START_attempts": 1,
        "START_command_dose_A2s": .026, "current_cap_A": .35, "slew_A_s": 2.,
        "initial": [0.]*5, "prehistory_A": 0., "sample_control_gyro_period_s": [.001, .005, .020],
        "forward_gates": FORWARD_GATES, "causal_sensor_replay_max_error": 1e-12,
        "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "owner_revised_position_jitter_limit_rad": REVISED_JITTER_RAD,
        "other_quality_gates": "unchanged existing synthetic_motion_metrics",
        "uncertainty": "UNKNOWN for every asset; three draws are not joint calibrated support",
        "numerical_q_guard_rad": [-100., 100.], "FF_q_support_rad": [-1., 1.], "physical_travel": "UNKNOWN",
        "plant_role": "each fitted FamilyModel is both FF model and forecast plant; independent oracle replays own TX",
        "forecast_against_independent_unknown_true_plant": "NOT_RUN",
        "control_evidence_partition": "DEVELOPMENT_CONSUMED; same retained control seeds and excitation",
        "no_truth_or_winner_selection": True, "no_tuning_or_restarts": True,
        "model_qualified": False, "controller_qualified": False,
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def runtime_receipt(asset, support, native, template, output):
    parameters, support, binding = bind_diagnostic_runtime(asset, native, template, support)
    receipt = runtime_document(asset, template, parameters, support, binding)
    save(output/"runtime-family-asset.json", receipt)
    reloaded = bind_runtime_document(json.loads((output/"runtime-family-asset.json").read_text()), native)
    require(reloaded[0].document() == asset.document() and
        native_parameter_document(reloaded[1]) == native_parameter_document(parameters),
        Reason.INTEGRATION_MISMATCH, "serialized actual runtime receipt must remap/read back identically")
    return parameters, support, binding


def early(source, library, output):
    records = load_fits(source)
    output.mkdir(parents=True, exist_ok=False)
    triples = [fitted_asset(source, r) for r in records]
    for asset, _, _ in triples:
        path = output/asset.model_revision
        path.mkdir()
        save(path/"family-asset.json", asset.document())
        require(FamilyAsset.from_document(json.loads((path/"family-asset.json").read_text())).document() == asset.document(),
            Reason.INTEGRATION_MISMATCH, "complete family receipt roundtrip required")
    save(output/"predeclared-contract.json", declaration(source, library, records, [t[0] for t in triples]))
    asset, support, _ = triples[0]
    began = time.monotonic()
    result, template = local_solve(asset, support)
    path = output/asset.model_revision
    save(path/"local-result.json", result.document())
    if result.gains is not None:
        template = candidate_parameters(template, result.gains)
    native = Native(library)
    parameters, _, binding = runtime_receipt(asset, support, native, template, path)
    report = {"method": METHOD, "first_asset": asset.model_revision, "fit_case": records[0]["training_case"],
        "synthesis": result.document(), "complete_actual_native_parameter_readback": binding["complete_parameter_readback"],
        "receipt_roundtrip_rebind": "PASS", "native_abi": binding["native_abi"],
        "controller_template_role": "fresh synthesized gains" if result.gains else "nominal template for mapping/readback only; no forecast run",
        "native_START_totals_A": list(parameters.start_total)[:2], "elapsed_s": time.monotonic()-began,
        "all_three_asset_receipts_frozen": True, "uncertainty": "UNKNOWN", "control_trials": "NOT_RUN",
        "model_qualified": False, "controller_qualified": False, "deployment_authorized": False}
    save(output/"early-result.json", report)
    print(json.dumps({**{k: v for k, v in report.items() if k != "synthesis"},
        "synthesis_status": result.status.value, "selected_wn_rad_s": result.selected_wn_rad_s,
        "selected_damping_ratio": result.selected_damping_ratio,
        "gains": asdict(result.gains) if result.gains else None}), flush=True)


def control_case(native, family, asset, support, state, parameters, case, output, *, plant_model=None):
    """Frozen complete plateau through the actual shared posterior native path."""
    output.mkdir(parents=True, exist_ok=False)
    times, refs, timing = velocity_plan(case["speed_deg_s"])
    c = replace(contract(case["seed"], case["noisy"], round(float(times[-1]), 9)),
        frame=asset.frame, configuration_id=support.configuration_id,
        trajectory_id="frozen-single-episode-start-"+case_label(case))
    support = replace(support, max_reference_source_age_s=c.duration_s+.01)
    require(parameters.current_cap == .35 and parameters.slew == 2. and parameters.start_timeout_s == .2,
        Reason.INTEGRATION_MISMATCH, "frozen current/slew/START limits changed")
    save(output/"predeclared-contract.json", {"asset_revision": asset.model_revision, "case": case,
        "forecast_contract": asdict(c), "reference_timing": timing, "start_policy": asdict(support.start_policy),
        "forward_gates": FORWARD_GATES, "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "owner_revised_position_jitter_limit_rad": REVISED_JITTER_RAD,
        "plant_role": ("fitted FamilyModel; separate true plant not substituted" if plant_model is None else
                       "separate supplied synthetic plant; fitted FamilyModel remains controller FF"),
        "separate_plant_model": plant_model.document() if plant_model is not None else None,
        "uncertainty": "UNKNOWN"})
    expected = native_parameter_document(parameters)
    readbacks = []
    original_readback = native.controller_parameters

    def capture_readback(handle, destination):
        accepted = original_readback(handle, destination)
        if accepted:
            actual = native_parameter_document(ct.cast(destination, ct.POINTER(CParameters)).contents)
            require(actual == expected, Reason.INTEGRATION_MISMATCH,
                "actual forecast controller complete parameter readback differs from frozen candidate")
            readbacks.append(actual)
        return accepted

    began = time.monotonic()
    native.controller_parameters = capture_readback
    try:
        data = forecast(native, family, asset.model, parameters, c,
            lambda now: phased_planned_packet(c, now, times, refs, timing),
            initial=np.asarray(asset.gauges["acquisition_state"]["value"], dtype=float),
            feedforward_support=support, feedforward_state=state, plant_model=plant_model)
    finally:
        native.controller_parameters = original_readback
    require(bool(readbacks), Reason.INTEGRATION_MISMATCH,
        "actual MotorFeedforward controller readback must be observed during this forecast")
    save(output/"actual-controller-readback.json", {"complete_native_parameters": readbacks[0],
        "successful_readback_count": len(readbacks), "all_equal_frozen_candidate": True})
    comparison = independent_rollout(asset.model if plant_model is None else plant_model,
        data["t"], data["tx_t"], data["tx_A"], data["initial"]).trace
    errors = {name: float(np.sqrt(np.mean((data["truth"][mask, column]-comparison[mask, column])**2)))
        for name, column, mask in (("q_rms_rad", 0, slice(None)),
            ("gyro_rms_rad_s", 3, data["v_new"]), ("current_rms_A", 4, slice(None)))}
    completed = data["report"]["outcome"]["status"] == "COMPLETED"
    original = revised = {"status": "NOT_RUN", "detail": "fault truncated the complete motion/stop window"}
    if completed:
        original = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.)
        revised = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.,
            position_jitter_limit_rad=REVISED_JITTER_RAD)
    starts = data["commands"][:, 9] == 1
    dose = float(np.sum(data["commands"][starts, 2]**2)*.005)
    numerical = all(errors[k] <= FORWARD_GATES[k] for k in errors) and data["report"]["causal_sensor_replay_max_error"] <= 1e-12
    row = {"case": case, "model_revision": asset.model_revision, "elapsed_s": time.monotonic()-began,
        "report": data["report"], "completed": completed, "forward_errors": errors,
        "conditional_forward_passed": numerical, "successful_ACK_count": len(data["tx_t"])-1,
        "actual_controller_complete_readback": "PASS", "actual_controller_readback_count": len(readbacks),
        "START_successful_tick_count": int(np.sum(starts)), "START_realized_command_dose_A2s": dose,
        "START_dose_passed": dose <= .026, "max_abs_command_A": float(np.max(np.abs(data["tx_A"]))),
        "original_quality": original, "revised_quality": revised,
        "original_quality_passed": original.get("metrics", {}).get("passed", False) is True,
        "revised_quality_passed": revised.get("metrics", {}).get("passed", False) is True,
        "uncertainty": "UNKNOWN", "model_qualified": False, "controller_qualified": False}
    row["candidate_case_passed"] = case_passed(row)
    save(output/"result.json", row)
    np.savez_compressed(output/"trace.npz", **{k: v for k, v in data.items() if isinstance(v, np.ndarray)},
        independent_same_successful_input=comparison)
    print(json.dumps({"model_revision": asset.model_revision, "case": case,
        "outcome": data["report"]["outcome"], "original_quality_passed": row["original_quality_passed"],
        "owner_revised_quality_passed": row["revised_quality_passed"], "forward_passed": numerical,
        "complete_readback_count": len(readbacks), "elapsed_s": row["elapsed_s"]}), flush=True)
    return row


def full(source, library, output, frozen):
    records = load_fits(source)
    triples = [fitted_asset(source, r) for r in records]
    require(json.loads((frozen/"predeclared-contract.json").read_text()) ==
        declaration(source, library, records, [t[0] for t in triples]), Reason.DATA_INVALID,
        "control protocol and all fitted vectors must match the already frozen declaration")
    require((frozen/"early-result.json").is_file(), Reason.DATA_INVALID,
        "actual first synthesis and native readback prerequisite must be retained")
    output.mkdir(parents=True, exist_ok=False)
    save(output/"predeclared-contract.json", declaration(source, library, records, [t[0] for t in triples]))
    native, family = Native(library), FamilyNative(library)
    rows = []
    for record, (asset, support, state) in zip(records, triples):
        path = output/asset.model_revision
        path.mkdir()
        save(path/"family-asset.json", asset.document())
        result, template = local_solve(asset, support)
        save(path/"local-result.json", result.document())
        row = {"model_revision": asset.model_revision, "training_case": record["training_case"],
            "synthesis_status": result.status.value, "gains": asdict(result.gains) if result.gains else None,
            "selected_wn_rad_s": result.selected_wn_rad_s, "selected_damping_ratio": result.selected_damping_ratio,
            "cases": [], "uncertainty": "UNKNOWN", "model_qualified": False, "controller_qualified": False}
        if result.gains is not None:
            template = candidate_parameters(template, result.gains)
            parameters, support, binding = runtime_receipt(asset, support, native, template, path)
            row["complete_native_readback"] = binding["complete_parameter_readback"]
            for case in PLATEAU_CASES:
                row["cases"].append(control_case(native, family, asset, support, state, parameters, case, path/case_label(case)))
                save(output/"partial-results.json", {"assets": rows+[row]})
        else:
            row["control_trials"] = "NOT_RUN; no candidate in frozen supported synthesis grid"
        rows.append(row)
        save(output/"partial-results.json", {"assets": rows})
    summary = {"method": METHOD, "assets": rows, "mandatory_assets": 3, "mandatory_cases": 18,
        "synthesis_pass_count": sum(r["gains"] is not None for r in rows),
        "completed_case_count": sum(c["completed"] for r in rows for c in r["cases"]),
        "case_pass_count": sum(case_passed(c) for r in rows for c in r["cases"]),
        "original_quality_pass_count": sum(c["original_quality_passed"] for r in rows for c in r["cases"]),
        "owner_revised_quality_pass_count": sum(c["revised_quality_passed"] for r in rows for c in r["cases"]),
        "independent_forward_pass_count": sum(c["conditional_forward_passed"] for r in rows for c in r["cases"]),
        "uncertainty": "UNKNOWN", "full_domain_coverage": "NOT_RUN",
        "forecast_against_independent_unknown_true_plant": "NOT_RUN",
        "scope": "PRODUCT_B_FITTED_PLANT_CONDITIONAL_DEVELOPMENT_ONLY",
        "model_qualified": False, "controller_qualified": False,
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"result.json", summary)
    print(json.dumps({k: v for k, v in summary.items() if k != "assets"}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fits", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "full"), required=True)
    parser.add_argument("--frozen", type=Path)
    args = parser.parse_args()
    if args.stage == "early": early(args.fits, args.library, args.output)
    else:
        parser.error("--frozen is required for full") if args.frozen is None else full(args.fits, args.library, args.output, args.frozen)
