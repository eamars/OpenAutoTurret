"""Frozen fitted-family position coverage; reversal coverage is separately declared."""
from dataclasses import asdict, replace
import argparse
import copy
import csv
import ctypes as ct
import io
import json
from pathlib import Path
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, Rejected, require
from Firmware.commissioning.family_assets import bind_diagnostic_runtime, native_parameter_document, runtime_document
from Firmware.commissioning.family_forecast import forecast, synthetic_motion_metrics
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import CParameters, Native
from Firmware.commissioning.planned_start import PlannedStartLeg, PlannedStartProgram
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_family_forecast_probe import contract
from Firmware.tools.adr0022_fresh_family_control_probe import FIT_CASES, load_fits, fitted_asset, runtime_receipt, save
from Firmware.tools.adr0022_fitted_candidate_qualification_probe import mapped_gains, controller_parameters_for
from Firmware.tools.adr0022_fitted_start_policy_probe import FIXED_CURVE, source_margin_records, policy_support
from Firmware.tools.adr0022_manoeuvre_forecast_probe import manoeuvre
from Firmware.tools.adr0022_start_policy_synthesis_probe import FORWARD_GATES, ORIGINAL_JITTER_RAD, REVISED_JITTER_RAD


METHOD = "adr0022.fitted-manoeuvre-development/1"
STEP_CASES = tuple({"kind": "step", "direction": direction, "step_deg": distance, "noisy": noisy, "seed": 101}
    for direction in (-1, 1) for distance in (.5, 1., 5.) for noisy in (False, True))
EARLY_CASE = STEP_CASES[0]
REVERSAL_CASES = tuple({"kind": "reversal", "direction": direction, "noisy": noisy, "seed": 101}
    for direction in (-1, 1) for noisy in (False, True))


def label(case):
    if case["kind"] == "reversal":
        return f"reversal{case['direction']:+d}-{'noisy' if case['noisy'] else 'pristine'}-{case['seed']}"
    return f"step{case['direction']*case['step_deg']:g}-{'noisy' if case['noisy'] else 'pristine'}-{case['seed']}"


def reversal_program(c, direction):
    return PlannedStartProgram(configuration_id=c.configuration_id, frame=c.frame, trajectory_id=c.trajectory_id,
        generation=1, source_time_s=0., expires_at_s=c.duration_s,
        legs=(PlannedStartLeg(direction=direction, departure_offset_s=2., departure_position_rad=0.),
              PlannedStartLeg(direction=-direction, departure_offset_s=4., departure_position_rad=direction*.125)))


def load_context(source, margins, start):
    records = load_fits(source)
    triples = [fitted_asset(source, record) for record in records]
    local = source_margin_records(margins, triples)
    decision = json.loads((start/"decision.json").read_text())
    require(decision["status"] == "FAMILY_SPECIFIC_START_PAIR_CONDITIONAL_DEVELOPMENT_PASS" and
        decision["fixed_curve"] == list(FIXED_CURVE) and decision["selected_pair_A"] == {"negative": .02, "positive": .02} and
        decision["combined_passed"] is True and decision["combined_case_count"] == 18 and
        decision["combined_original_quality_pass_count"] == decision["combined_owner_quality_pass_count"] == 18 and
        decision["uncertainty"] == "UNKNOWN" and decision["controller_qualified"] is False,
        Reason.DATA_INVALID, "exact completed18-case20/20 START evidence with ownWN2/zeta1.5 is required")
    require(json.loads((start/"frozen-fit-records.json").read_text()) == records and
        json.loads((start/"frozen-local-margin-records.json").read_text()) == local,
        Reason.INTEGRATION_MISMATCH, "START branch must bind identical fitted vectors and local curves")
    triples = [(asset, policy_support(support, .02), state) for asset, support, state in triples]
    return records, triples, local, decision


def protocol(source, library, margins, start, records, triples, local):
    references = []
    for case in STEP_CASES:
        duration, timing, _ = manoeuvre("step", case["direction"], case["step_deg"])
        references.append({"case": case, "duration_s": duration, "timing": timing})
    return {"schema": METHOD, "scope": "FITTED_PLANT_CONDITIONAL_POSITION_DEVELOPMENT_ONLY",
        "fit_source": str(source.resolve()), "margin_source": str(margins.resolve()), "START_source": str(start.resolve()),
        "library": str(library.resolve()), "fit_cases": list(FIT_CASES), "models": [r["model"] for r in records],
        "fixed_curve": list(FIXED_CURVE), "gains_by_asset": [{"model_revision": asset.model_revision,
            "gains": asdict(mapped_gains(asset, l, FIXED_CURVE))} for (asset, _, _), l in zip(triples, local)],
        "negative_excess_A": .02, "positive_excess_A": .02, "known_rest_balance_A": -.02,
        "mandatory_models": 3, "mandatory_cases_each_model": list(STEP_CASES), "mandatory_total_cases": 36,
        "references": references, "early_case": EARLY_CASE, "early_result_reused_once": True,
        "reference_generator": "unchanged existing manoeuvre(step)/shaped_step; coherent q/v/a and fixed anchors",
        "initial": [0.]*5, "successful_TX_prehistory_A": 0., "sample_control_gyro_period_s": [.001, .005, .020],
        "START_static_intervals_A": [.156, .164], "START_attempt_s": .2, "START_attempts": 1,
        "START_command_dose_A2s": .026, "current_cap_A": .35, "slew_A_s": 2.,
        "phase_required_deg": 45., "gain_required_db": 6., "forward_gates": FORWARD_GATES,
        "causal_sensor_replay_max_error": 1e-12,
        "original_jitter_limit_rad": ORIGINAL_JITTER_RAD, "owner_jitter_limit_rad": REVISED_JITTER_RAD,
        "step_limits": "existing finalerror0.15deg/overshootmax(0.15deg,10%step)/settlingdeadline/fixed2sstop unchanged",
        "position_and_reversal_coverage_separate": True, "reversal_stage": "NOT_RUN; separately declared only",
        "seed_role": "CONSUMED DEVELOPMENT; pristine and originalnoisy101", "uncertainty": "UNKNOWN",
        "independent_unknown_true_plant_forecast": "NOT_RUN", "full_domain_coverage": "NOT_RUN",
        "model_qualified": False, "controller_qualified": False,
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def motor_passed(row):
    return bool(row["completed"] and row["forward_passed"] and row["actual_complete_readback"] == "PASS" and
        row["causal_replay_max_error"] <= 1e-12 and row["owner_quality_passed"] and row["START_dose_A2s"] <= .026 and
        row["maximum_command_A"] <= .35+1e-12 and row["maximum_slew_A_s"] <= 2.+1e-12)


def run_case(native, family, asset, support, state, parameters, case, output):
    output.mkdir(parents=True, exist_ok=False)
    reversal = case["kind"] == "reversal"
    duration, timing, producer = manoeuvre(case["kind"], case["direction"], case.get("step_deg", .5),
        planned_start_program=reversal)
    c = replace(contract(case["seed"], case["noisy"], duration), frame=asset.frame,
        configuration_id=support.configuration_id, trajectory_id="frozen-fitted-"+label(case))
    support = replace(support, max_reference_source_age_s=duration+.01)
    if reversal:
        support = replace(support, planned_start_program=reversal_program(c, case["direction"]))
    save(output/"predeclared-contract.json", {"case": case, "model_revision": asset.model_revision,
        "forecast_contract": asdict(c), "timing": timing, "START_policy": asdict(support.start_policy)})
    if reversal: save(output/"planned-start-program.json", asdict(support.planned_start_program))
    expected = native_parameter_document(parameters)
    original_readback = native.controller_parameters
    readback_count, actual_first = 0, None

    def capture(handle, destination):
        nonlocal readback_count, actual_first
        accepted = original_readback(handle, destination)
        if accepted:
            actual = native_parameter_document(ct.cast(destination, ct.POINTER(CParameters)).contents)
            require(actual == expected, Reason.INTEGRATION_MISMATCH, "actual complete core candidate readback differs")
            readback_count += 1
            if actual_first is None: actual_first = actual
        return accepted

    began = time.monotonic()
    native.controller_parameters = capture
    try:
        data = forecast(native, family, asset.model, parameters, c, lambda now: producer(c, now),
            initial=np.asarray(asset.gauges["acquisition_state"]["value"], dtype=float),
            feedforward_support=support, feedforward_state=state)
    finally:
        native.controller_parameters = original_readback
    require(readback_count > 0, Reason.INTEGRATION_MISMATCH, "actual forecast core readback must be observed")
    independent = independent_rollout(asset.model, data["t"], data["tx_t"], data["tx_A"], data["initial"]).trace
    errors = {name: float(np.sqrt(np.mean((data["truth"][mask, column]-independent[mask, column])**2)))
        for name, column, mask in (("q_rms_rad", 0, slice(None)), ("gyro_rms_rad_s", 3, data["v_new"]),
            ("current_rms_A", 4, slice(None)))}
    completed = data["report"]["outcome"]["status"] == "COMPLETED"
    original = revised = {"status": "NOT_RUN; incomplete fixed manoeuvre windows"}
    if completed and not reversal:
        original = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.)
        revised = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.,
            position_jitter_limit_rad=REVISED_JITTER_RAD)
    commands = data["commands"]
    starts = commands[:, 9] == 1
    row = {"case": case, "model_revision": asset.model_revision, "elapsed_s": time.monotonic()-began,
        "report": data["report"], "completed": completed, "forward_errors": errors,
        "forward_passed": all(errors[k] <= FORWARD_GATES[k] for k in errors),
        "actual_complete_readback": "PASS", "actual_readback_count": readback_count,
        "causal_replay_max_error": data["report"]["causal_sensor_replay_max_error"],
        "original_quality": original, "owner_quality": revised,
        "original_quality_passed": original.get("metrics", {}).get("passed", False) is True,
        "owner_quality_passed": revised.get("metrics", {}).get("passed", False) is True,
        "START_dose_A2s": float(np.sum(commands[starts, 2]**2)*.005),
        "maximum_command_A": data["report"]["maximum_successful_command_A"],
        "maximum_slew_A_s": data["report"]["maximum_successful_slew_A_s"]}
    row["motor_case_passed"] = motor_passed(row)
    if reversal:
        for key in ("original_quality", "owner_quality"):
            row[key] = {"status": "NOT_RUN", "detail": "full reversal tracking quality contract not declared"}
        t, q = data["t"], data["q"]
        stop = (t >= timing["zero_reference_time"]) & (t <= timing["zero_reference_time"]+2.)
        full_stop = t[-1] >= timing["zero_reference_time"]+2.-1e-12
        drift = float(np.max(np.abs(q[stop]-q[np.flatnonzero(stop)[0]]))) if full_stop else None
        ledger = data["report"]["planned_start_program"]
        # Reconstruct only actual accepted-current holds, with their native START state.
        tx_t, tx_A = data["tx_t"], data["tx_A"]
        accepted = commands[:len(tx_t)-1]
        require(np.array_equal(accepted[:, 0], tx_t[1:]) and np.array_equal(accepted[:, 2], tx_A[1:]),
            Reason.INTEGRATION_MISMATCH, "receipt dose must use actual successful commands and times")
        held_dt = np.diff(np.r_[tx_t[1:], ledger["accounted_through_s"]])
        dose = float(np.sum(tx_A[1:]**2 * held_dt * (accepted[:, 9] == 1)))
        row.update({"motor_case_passed": False, "full_reversal_tracking_quality": "NOT_RUN",
            "fixed_final_two_second_stop_complete": bool(full_stop), "fixed_final_two_second_observed_anchor_drift_rad": drift,
            "fixed_final_two_second_drift_passed": bool(drift <= ORIGINAL_JITTER_RAD) if drift is not None else "NOT_RUN",
            "successful_ACK_count": len(tx_t)-1, "actual_accepted_START_dose_A2s": dose,
            "ledger_dose_reconstruction_error_A2s": abs(dose-ledger["dose_A2s"]),
            "original_quality_passed": False, "owner_quality_passed": False})
        row["reversal_interface_stop_passed"] = reversal_interface_passed(row)
    save(output/"actual-controller-readback.json", {"complete_native_parameters": actual_first,
        "successful_readback_count": readback_count, "all_equal_frozen_candidate": True})
    save(output/"result.json", row)
    np.savez_compressed(output/"trace.npz", **{k:v for k,v in data.items() if isinstance(v, np.ndarray)},
        independent_same_successful_input=independent)
    print(json.dumps({"asset": asset.model_revision, "case": case, "completed": completed,
        "original_quality_passed": row["original_quality_passed"], "owner_quality_passed": row["owner_quality_passed"],
        "forward_passed": row["forward_passed"], "readbacks": readback_count, "elapsed_s": row["elapsed_s"]}), flush=True)
    return row


def reversal_interface_passed(row):
    ledger = row["report"]["planned_start_program"]
    return bool(row["completed"] and row["forward_passed"] and row["actual_complete_readback"] == "PASS" and
        row["causal_replay_max_error"] <= 1e-12 and row["fixed_final_two_second_drift_passed"] is True and
        row["ledger_dose_reconstruction_error_A2s"] <= 1e-12 and row["actual_accepted_START_dose_A2s"] <= .026 and
        len(ledger["admissions"]) == 2 and tuple(entry["leg"] for entry in ledger["admissions"]) == (0, 1) and
        len(ledger["accepted_MOVE_times_s"]) == 2 and ledger["unknown_future_hold"] is False and
        row["maximum_command_A"] <= .35+1e-12 and row["maximum_slew_A_s"] <= 2.+1e-12)


def early(source, library, margins, start, output):
    records, triples, local, start_decision = load_context(source, margins, start)
    output.mkdir(parents=True, exist_ok=False)
    save(output/"frozen-fit-records.json", records)
    save(output/"frozen-local-margin-records.json", local)
    save(output/"frozen-START-decision.json", start_decision)
    save(output/"predeclared-contract.json", protocol(source, library, margins, start, records, triples, local))
    for asset, _, _ in triples:
        folder = output/"frozen-assets"/asset.model_revision
        folder.mkdir(parents=True); save(folder/"family-asset.json", asset.document())
    asset, support, state = triples[0]
    gains = mapped_gains(asset, local[0], FIXED_CURVE)
    folder = output/"early"/asset.model_revision
    folder.mkdir(parents=True)
    native, family = Native(library), FamilyNative(library)
    parameters, support, _ = runtime_receipt(asset, support, native, controller_parameters_for(asset, gains), folder)
    row = run_case(native, family, asset, support, state, parameters, EARLY_CASE, folder/label(EARLY_CASE))
    save(output/"early-result.json", {"asset_revision": asset.model_revision, "case": EARLY_CASE,
        "result_path": str((folder/label(EARLY_CASE)/"result.json").resolve()),
        "motor_case_passed": motor_passed(row), "all36_cases_frozen_before_motion": True})


def validate_coverage(revisions, rows):
    require(len(revisions) == 3 and len(set(revisions)) == 3 and len(rows) == 36 and
        tuple((r["model_revision"], r["case"]) for r in rows) ==
            tuple((revision, case) for revision in revisions for case in STEP_CASES),
        Reason.DATA_INVALID, "exact all-three-model/all12-step cases coverage required")
    return all(motor_passed(row) for row in rows)


def compact_row(row):
    return {"model_revision": row["model_revision"], **row["case"],
        **{k:row[k] for k in ("completed", "forward_passed", "original_quality_passed", "owner_quality_passed",
            "motor_case_passed", "actual_readback_count", "causal_replay_max_error", "START_dose_A2s",
            "maximum_command_A", "maximum_slew_A_s")}, **row["forward_errors"],
        **{k:row["original_quality"].get("metrics", {}).get(k, "") for k in
            ("step_error_rad", "overshoot_rad", "settled_before_deadline", "stop_drift_rad", "sustained_start_s")}}


def write_csv(path, rows):
    stream=io.StringIO(newline="");writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    path.write_text(stream.getvalue())


def full(source, library, margins, start, output):
    records, triples, local, start_decision = load_context(source, margins, start)
    require(json.loads((output/"frozen-fit-records.json").read_text()) == records and
        json.loads((output/"frozen-local-margin-records.json").read_text()) == local and
        json.loads((output/"frozen-START-decision.json").read_text()) == start_decision and
        json.loads((output/"predeclared-contract.json").read_text()) == protocol(source, library, margins, start, records, triples, local) and
        all(json.loads((output/"frozen-assets"/a.model_revision/"family-asset.json").read_text()) == a.document() for a,_,_ in triples),
        Reason.INTEGRATION_MISMATCH, "all exact fitted models/gains/START/reference windows must stay frozen")
    native, family=Native(library),FamilyNative(library)
    asset,support,_=triples[0]
    parameters,support,binding=bind_diagnostic_runtime(asset,native,controller_parameters_for(asset,mapped_gains(asset,local[0],FIXED_CURVE)),support)
    early_receipt=json.loads((output/"early-result.json").read_text())
    folder=output/"early"/asset.model_revision;path=folder/label(EARLY_CASE)
    first=json.loads((path/"result.json").read_text());readback=json.loads((path/"actual-controller-readback.json").read_text())
    require(early_receipt["asset_revision"]==asset.model_revision and early_receipt["case"]==EARLY_CASE and
        Path(early_receipt["result_path"]).resolve()==(path/"result.json").resolve() and
        early_receipt["all36_cases_frozen_before_motion"] is True and first["case"]==EARLY_CASE and first["model_revision"]==asset.model_revision and
        motor_passed(first)==first["motor_case_passed"]==early_receipt["motor_case_passed"] and
        json.loads((folder/"runtime-family-asset.json").read_text())==runtime_document(asset,controller_parameters_for(asset,mapped_gains(asset,local[0],FIXED_CURVE)),parameters,support,binding) and
        readback["complete_native_parameters"]==native_parameter_document(parameters) and readback["all_equal_frozen_candidate"] is True and
        readback["successful_readback_count"]==first["actual_readback_count"]>0,
        Reason.INTEGRATION_MISMATCH,"retained early step must bind exact actual candidate/readback, without replay")
    rows=[]
    for i,(asset,support,state) in enumerate(triples):
        folder=output/"position-steps"/asset.model_revision;folder.mkdir(parents=True,exist_ok=False)
        gains=mapped_gains(asset,local[i],FIXED_CURVE)
        parameters,support,_=runtime_receipt(asset,support,native,controller_parameters_for(asset,gains),folder)
        for case in STEP_CASES:
            row=first if i==0 and case==EARLY_CASE else run_case(native,family,asset,support,state,parameters,case,folder/label(case))
            rows.append(row);write_csv(output/"case-results.csv",[compact_row(r) for r in rows]);save(output/"partial-results.json",{"cases":rows})
    passing=validate_coverage(tuple(a.model_revision for a,_,_ in triples),rows)
    decision={"schema":METHOD,"status":"POSITION_STEPS_CONDITIONAL_DEVELOPMENT_PASS" if passing else "POSITION_STEP_MOTOR_QUALITY_FAILED",
        "fixed_curve":list(FIXED_CURVE),"START_excess_A":{"negative":.02,"positive":.02},"required_position_cases":36,
        "complete_case_count":sum(r["completed"] for r in rows),"original_quality_pass_count":sum(r["original_quality_passed"] for r in rows),
        "owner_quality_pass_count":sum(r["owner_quality_passed"] for r in rows),"motor_case_pass_count":sum(motor_passed(r) for r in rows),
        "independent_forward_pass_count":sum(r["forward_passed"] for r in rows),"actual_complete_readbacks":sum(r["actual_readback_count"] for r in rows),
        "position_coverage":"signed.5/1/5deg pristine and noisy101, all3fitted models","reversal_coverage":"NOT_RUN; separate declaration required",
        "failure_cases":[{"model_revision":r["model_revision"],"case":r["case"]} for r in rows if not motor_passed(r)],
        "all_motor_reference_quality_limits":"UNCHANGED","uncertainty":"UNKNOWN","model_qualified":False,"controller_qualified":False,
        "independent_unknown_true_plant_forecast":"NOT_RUN","full_domain_coverage":"NOT_RUN","physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
    save(output/"decision.json",decision);print(json.dumps(decision),flush=True)


def reversal_contract(records, triples, local):
    references=[]
    for case in REVERSAL_CASES:
        duration,timing,_=manoeuvre("reversal",case["direction"],.5,planned_start_program=True)
        asset=triples[0][0]
        c=replace(contract(case["seed"],case["noisy"],duration),frame=asset.frame,
            configuration_id=asset.model_revision,trajectory_id="frozen-fitted-"+label(case))
        references.append({"case":case,"duration_s":duration,"timing":timing,
            "ordered_legs":[asdict(leg) for leg in reversal_program(c,case["direction"]).legs]})
    return {"schema":"adr0022.fitted-planned-reversal-development/1",
        "scope":"FITTED_PLANT_CONDITIONAL_INTERFACE_AND_STOP_ONLY", "models":[r["model"] for r in records],
        "model_revisions":[a.model_revision for a,_,_ in triples],"fixed_curve":list(FIXED_CURVE),
        "own_gains":[asdict(mapped_gains(a,l,FIXED_CURVE)) for (a,_,_),l in zip(triples,local)],
        "mandatory_cases_each_model":list(REVERSAL_CASES),"required_total_cases":12,"references":references,
        "reference_generator":"unchanged manoeuvre(reversal,planned_start_program=True):8*direction*x^3*(1-x)^3;x=(t-2)/4",
        "ordered_original_departure_anchors_s":[2.,4.],"stop_anchor_s":6.,"fixed_stop_window_s":[6.,8.],
        "two_leg_ledger":"immutable once-bound context; one START attempt per leg; actual accepted-command aggregate dose",
        "START_excess_A":{"negative":.02,"positive":.02},"aggregate_START_dose_ceiling_A2s":.026,
        "attempt_duration_s":.2,"current_cap_A":.35,"slew_A_s":2.,"body_authority":"UNCHANGED",
        "known_rest_balance_A":-.02,"initial":[0.]*5,"input_prehistory_A":0.,
        "sample_control_gyro_period_s":[.001,.005,.020],"noise":"original pristine/noisy101 unchanged",
        "numeric_forward_gates":FORWARD_GATES,"causal_sensor_replay_max_error":1e-12,
        "accepted_dose_reconstruction_max_error_A2s":1e-12,"observed_fixed_anchor_drift_limit_rad":ORIGINAL_JITTER_RAD,
        "original_full_reversal_tracking_quality":"NOT_RUN; no complete tracking predicate declared",
        "revised_full_reversal_tracking_quality":"NOT_RUN; no complete tracking predicate declared",
        "early_case":REVERSAL_CASES[0],"early_result_reused_once":True,
        "uncertainty":"UNKNOWN","model_qualified":False,"controller_qualified":False,
        "independent_unknown_true_plant_forecast":"NOT_RUN","full_domain_coverage":"NOT_RUN",
        "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}


def admitted_cycle09(source,library,margins,start,output):
    records,triples,local,start_decision=load_context(source,margins,start)
    require(json.loads((output/"predeclared-contract.json").read_text())==protocol(source,library,margins,start,records,triples,local),
        Reason.INTEGRATION_MISMATCH,"separate reversal stage must retain all exact fitted models/gains/START")
    rows=json.loads((output/"partial-results.json").read_text())["cases"]
    validate_coverage(tuple(a.model_revision for a,_,_ in triples),rows)
    require((output/"decision.json").is_file(),Reason.DATA_INVALID,"complete frozen36 position evidence must precede reversals")
    return records,triples,local


def reversal_early(source,library,margins,start,output):
    records,triples,local=admitted_cycle09(source,library,margins,start,output)
    target=output/"reversals";target.mkdir(parents=True,exist_ok=False)
    save(target/"predeclared-contract.json",reversal_contract(records,triples,local))
    asset,support,state=triples[0];folder=target/"early"/asset.model_revision;folder.mkdir(parents=True)
    native,family=Native(library),FamilyNative(library)
    parameters,support,_=runtime_receipt(asset,support,native,controller_parameters_for(asset,mapped_gains(asset,local[0],FIXED_CURVE)),folder)
    row=run_case(native,family,asset,support,state,parameters,REVERSAL_CASES[0],folder/label(REVERSAL_CASES[0]))
    save(target/"early-result.json",{"model_revision":asset.model_revision,"case":REVERSAL_CASES[0],
        "result_path":str((folder/label(REVERSAL_CASES[0])/"result.json").resolve()),
        "all12_cases_frozen_before_motion":True,"reversal_interface_stop_passed":reversal_interface_passed(row)})


def reversal_full(source,library,margins,start,output):
    records,triples,local=admitted_cycle09(source,library,margins,start,output)
    target=output/"reversals"
    require(json.loads((target/"predeclared-contract.json").read_text())==reversal_contract(records,triples,local),
        Reason.INTEGRATION_MISMATCH,"all exact reversal models, gains, ordered legs and fixed stop must stay frozen")
    native,family=Native(library),FamilyNative(library)
    asset,support,_=triples[0]
    parameters,support,binding=bind_diagnostic_runtime(asset,native,controller_parameters_for(asset,mapped_gains(asset,local[0],FIXED_CURVE)),support)
    receipt=json.loads((target/"early-result.json").read_text());folder=target/"early"/asset.model_revision
    path=folder/label(REVERSAL_CASES[0]);first=json.loads((path/"result.json").read_text())
    readback=json.loads((path/"actual-controller-readback.json").read_text())
    require(receipt["model_revision"]==asset.model_revision and receipt["case"]==REVERSAL_CASES[0] and
        Path(receipt["result_path"]).resolve()==(path/"result.json").resolve() and receipt["all12_cases_frozen_before_motion"] is True and
        first["model_revision"]==asset.model_revision and first["case"]==REVERSAL_CASES[0] and
        reversal_interface_passed(first)==receipt["reversal_interface_stop_passed"]==first["reversal_interface_stop_passed"] and
        json.loads((folder/"runtime-family-asset.json").read_text())==runtime_document(asset,controller_parameters_for(asset,mapped_gains(asset,local[0],FIXED_CURVE)),parameters,support,binding) and
        readback["complete_native_parameters"]==native_parameter_document(parameters) and readback["all_equal_frozen_candidate"] is True and
        readback["successful_readback_count"]==first["actual_readback_count"]>0,
        Reason.INTEGRATION_MISMATCH,"retained first reversal binds exact actual candidate/readback without replay")
    rows=[]
    for i,(asset,support,state) in enumerate(triples):
        folder=target/"cases"/asset.model_revision;folder.mkdir(parents=True,exist_ok=False)
        parameters,support,_=runtime_receipt(asset,support,native,controller_parameters_for(asset,mapped_gains(asset,local[i],FIXED_CURVE)),folder)
        for case in REVERSAL_CASES:
            row=first if i==0 and case==REVERSAL_CASES[0] else run_case(native,family,asset,support,state,parameters,case,folder/label(case))
            rows.append(row)
            compact=[]
            for r in rows:
                ledger=r["report"]["planned_start_program"]
                compact.append({"model_revision":r["model_revision"],**r["case"],"outcome":r["report"]["outcome"]["status"],
                    "complete":r["completed"],"interface_stop_passed":r["reversal_interface_stop_passed"],
                    "full_original_quality":"NOT_RUN","full_revised_quality":"NOT_RUN", "forward_passed":r["forward_passed"],
                    "actual_readback_count":r["actual_readback_count"],"successful_ACK_count":r["successful_ACK_count"],
                    "admitted_legs":len(ledger["admissions"]),"accepted_MOVE_legs":len(ledger["accepted_MOVE_times_s"]),
                    "actual_accepted_START_dose_A2s":r["actual_accepted_START_dose_A2s"],
                    "dose_reconstruction_error_A2s":r["ledger_dose_reconstruction_error_A2s"],
                    "stop_window_complete":r["fixed_final_two_second_stop_complete"],
                    "stop_drift_rad":r["fixed_final_two_second_observed_anchor_drift_rad"],
                    "stop_drift_passed":r["fixed_final_two_second_drift_passed"],
                    "maximum_command_A":r["maximum_command_A"],"maximum_slew_A_s":r["maximum_slew_A_s"],
                    "causal_replay_max_error":r["causal_replay_max_error"],**r["forward_errors"]})
            write_csv(target/"case-results.csv",compact);save(target/"partial-results.json",{"cases":rows})
    require(tuple((r["model_revision"],r["case"]) for r in rows)==tuple((a.model_revision,c) for a,_,_ in triples for c in REVERSAL_CASES),
        Reason.DATA_INVALID,"exact all-three-model/all-four-reversal cases coverage required")
    passing=all(reversal_interface_passed(r) for r in rows)
    decision={"schema":"adr0022.fitted-planned-reversal-development/1", "status":"REVERSAL_INTERFACE_AND_FIXED_STOP_PASS" if passing else "REVERSAL_INTERFACE_OR_FIXED_STOP_FAILED",
        "required_case_count":12,"completed_case_count":sum(r["completed"] for r in rows),
        "independent_forward_pass_count":sum(r["forward_passed"] for r in rows),
        "interface_and_fixed_stop_pass_count":sum(reversal_interface_passed(r) for r in rows),
        "actual_complete_readbacks":sum(r["actual_readback_count"] for r in rows),
        "full_original_reversal_tracking_quality":"NOT_RUN; no complete predicate declared",
        "full_revised_reversal_tracking_quality":"NOT_RUN; no complete predicate declared",
        "uncertainty":"UNKNOWN","model_qualified":False,"controller_qualified":False,
        "independent_unknown_true_plant_forecast":"NOT_RUN","full_domain_coverage":"NOT_RUN",
        "physical_stage3a":"NOT_RUN","physical_stage3b":"NOT_RUN","deployment_authorized":False}
    save(target/"decision.json",decision);print(json.dumps(decision),flush=True)


def compact_evidence(output):
    rows=json.loads((output/"partial-results.json").read_text())["cases"]
    revisions=tuple(dict.fromkeys(r["model_revision"] for r in rows));validate_coverage(revisions,rows)
    frozen=json.loads((output/"predeclared-contract.json").read_text())
    target=output/"compact";target.mkdir(parents=True,exist_ok=False)
    decision=json.loads((output/"decision.json").read_text());decision.pop("failure_cases")
    decision.update({"fault_truncated_case_count":sum(not r["completed"] for r in rows),
        "original_full_position_quality":"NOT_RUN; all36 fault-truncated before fixed windows",
        "revised_full_position_quality":"NOT_RUN; all36 fault-truncated before fixed windows",
        "evaluated_original_quality_case_count":sum("metrics" in r["original_quality"] for r in rows),
        "evaluated_revised_quality_case_count":sum("metrics" in r["owner_quality"] for r in rows),
        "unchanged_step_settling_deadline":"command+1s for abs(step)<=1deg; zero_reference+1s for5deg",
        "unchanged_final_position_error_limit_deg":.15,
        "unchanged_overshoot_limit_deg":"max(0.15,0.1*abs(step_deg))"})
    faults=[]
    for index,row in enumerate(rows):
        case=row["case"]
        path=(output/"early"/row["model_revision"] if index==0 else output/"position-steps"/row["model_revision"])/label(case)
        data=np.load(path/"trace.npz")
        timing=next(ref["timing"] for ref in frozen["references"] if ref["case"]==case)
        outcome=row["report"]["outcome"]
        faults.append({**compact_row(row),"original_full_position_quality":"NOT_RUN" if not row["completed"] else "EVALUATED",
            "revised_full_position_quality":"NOT_RUN" if not row["completed"] else "EVALUATED",
            "fault_status":outcome["status"],"fault_time_s":outcome["time_s"],"fault_reason":outcome.get("reason",""),
            "fault_detail":outcome.get("detail",""),"fault_minus_original_HOLD_anchor_s":outcome["time_s"]-timing["zero_reference_time"],
            "latent_error_at_fault_deg":float(np.rad2deg(data["truth"][-1,0]-timing["step_rad"])),
            "observed_error_at_fault_deg":float(np.rad2deg(data["q"][-1]-timing["step_rad"]))})
    write_csv(target/"case-results.csv",faults)
    require(all(r["fault_status"]=="MOTOR_FF_FAULT" and r["fault_reason"]=="OUTSIDE_SUPPORT" and
        "attempt count exhausted" in r["fault_detail"] and abs(r["fault_minus_original_HOLD_anchor_s"])<=1e-12 for r in faults),
        Reason.DATA_INVALID,"diagnosis must retain actual all36 causal outcomes, not assume a shared fault")
    first=rows[0];path=output/"early"/first["model_revision"]/label(first["case"]);data=np.load(path/"trace.npz")
    timing=frozen["references"][0]["timing"];commands=data["commands"];reference=data["references"]
    motion=commands[:,9].astype(int);changes=np.flatnonzero(np.r_[True,np.diff(motion)!=0])
    events=[(float(commands[i,0]),"entered_"+first["report"]["native_motion_names"][str(motion[i])])
        for i in changes if commands[i,0]>=timing["command_time"]]
    over=np.flatnonzero((data["t"]>=timing["command_time"]) &
        (first["case"]["direction"]*(data["truth"][:,0]-timing["step_rad"])>np.deg2rad(.15)))
    if len(over):events.append((float(data["t"][over[0]]),"first_latent_target_overshoot_above_0.15deg"))
    events.extend([(float(commands[-1,0]),"last_successful_command"),
        (first["report"]["outcome"]["time_s"],"HOLD_correction_rejected_START_attempt_exhausted")])
    event_rows=[]
    for when,event in sorted(events):
        k=int(np.argmin(np.abs(data["t"]-when)));j=int(np.searchsorted(commands[:,0],when,side="right")-1)
        is_command=abs(commands[j,0]-when)<=1e-12
        event_rows.append({"time_s":when,"event":event,"latent_q_rad":float(data["truth"][k,0]),
            "latent_v_rad_s":float(data["truth"][k,1]),"observed_q_rad":float(data["q"][k]),
            "reference_q_rad":float(np.interp(when,reference[:,0],reference[:,1])) if when<timing["zero_reference_time"] else timing["step_rad"],
            "reference_v_rad_s":float(np.interp(when,reference[:,0],reference[:,2])) if when<timing["zero_reference_time"] else 0.,
            "last_accepted_command_A":float(commands[j,2]),
            "posterior_q_rad":float(commands[j,5]) if is_command else "NOT_SAVED_AT_EVENT",
            "posterior_v_rad_s":float(commands[j,6]) if is_command else "NOT_SAVED_AT_EVENT"})
    write_csv(target/"event-window.csv",event_rows)
    diagnosis={"schema":"adr0022.fitted-step-causal-slice/1","case":first["case"],"model_revision":first["model_revision"],
        "actual_outcome":first["report"]["outcome"],"original_HOLD_anchor_s":timing["zero_reference_time"],
        "target_deg":float(np.rad2deg(timing["step_rad"])),"latent_q_at_fault_deg":float(np.rad2deg(data["truth"][-1,0])),
        "latent_position_error_at_fault_deg":faults[0]["latent_error_at_fault_deg"],
        "actual_START_dose_A2s":first["START_dose_A2s"],"dose_ceiling_A2s":.026,
        "native_START_attempts_allowed":1,"actual_complete_readbacks":first["actual_readback_count"],
        "forward_passed":first["forward_passed"],"forward_errors":first["forward_errors"],
        "causal_slice":"START2s -> MOVE2.18s at latent-0.05065rad/s versus reference-0.02224 -> overshoot -> correctionREVERSE2.43 while reference stillnegative -> rest beyondtarget -> HOLD2.56 requests second START, rejected by unchanged count1 guard",
        "all36_faults":"same actual count-exhausted rejection exactly at each originalHOLD anchor; percase latent errors retained inCSV",
        "interpretation":"first-case low-amplitude departure and correction expose a single-attempt controller/reference/START interaction; does not uniquely explain every amplitude or establish a repair",
        "quality_windows":"NOT_RUN; no complete original/revised settling or stop verdict",
        "reference_limits_and_guards":"UNCHANGED; no resets, extra attempts, gain/ref/FF/START tuning",
        "reversal_coverage":"separate declared program; see separate decision", "uncertainty":"UNKNOWN",
        "model_qualified":False,"controller_qualified":False,"physical_stages":"NOT_RUN"}
    save(target/"decision.json",decision);save(target/"first-failure-diagnosis.json",diagnosis)
    reversal=output/"reversals"
    if (reversal/"decision.json").is_file():
        (target/"reversal-decision.json").write_text((reversal/"decision.json").read_text())
        (target/"reversal-case-results.csv").write_text((reversal/"case-results.csv").read_text())
    print(json.dumps({"compact_files":[p.name for p in target.iterdir()],"fault_truncated":36,"quality_evaluated":0}),flush=True)


def reversal_protocol_tests(output):
    target=output/"reversals"
    require((target/"early-result.json").is_file(),Reason.DATA_INVALID,"actual reversal runtime must precede its protocol tests")
    receipt=json.loads((target/"early-result.json").read_text());first=json.loads(Path(receipt["result_path"]).read_text())
    class Protocol(unittest.TestCase):
        def test_actual_interface_not_tracking_promotion(self):
            self.assertTrue(reversal_interface_passed(first));self.assertFalse(first["motor_case_passed"])
            self.assertEqual(first["original_quality"]["status"],"NOT_RUN");self.assertEqual(first["owner_quality"]["status"],"NOT_RUN")
        def test_exact12_cases(self):
            self.assertEqual(len(REVERSAL_CASES)*3,12)
            self.assertEqual({(r["direction"],r["noisy"]) for r in REVERSAL_CASES},{(-1,False),(-1,True),(1,False),(1,True)})
        def test_missing_leg_rejected(self):
            r=copy.deepcopy(first);r["report"]["planned_start_program"]["admissions"].pop();self.assertFalse(reversal_interface_passed(r))
        def test_accepted_dose_and_reconstruction_required(self):
            for key,value in (("actual_accepted_START_dose_A2s",.027),("ledger_dose_reconstruction_error_A2s",1e-8)):
                r=copy.deepcopy(first);r[key]=value;self.assertFalse(reversal_interface_passed(r))
        def test_complete_fixed_stop_required(self):
            r=copy.deepcopy(first);r["fixed_final_two_second_drift_passed"]="NOT_RUN";self.assertFalse(reversal_interface_passed(r))
        def test_original_immutable_anchors(self):
            for direction in (-1,1):
                p=reversal_program(contract(duration=8.),direction)
                self.assertIsInstance(p.legs,tuple);self.assertEqual(tuple(l.departure_offset_s for l in p.legs),(2.,4.))
                self.assertEqual(tuple(l.direction for l in p.legs),(direction,-direction))
                self.assertEqual(p.legs[1].departure_position_rad,direction*.125)
    with (target/"protocol-tests.txt").open("w") as stream:
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Protocol))
    print((target/"protocol-tests.txt").read_text());return result.wasSuccessful()


def protocol_tests(output):
    require((output/"early-result.json").is_file(),Reason.DATA_INVALID,"actual early runtime must precede source protocol tests")
    receipt=json.loads((output/"early-result.json").read_text());early=json.loads(Path(receipt["result_path"]).read_text())
    revisions=("fresh-exact-bin-10103","fresh-exact-bin-10301","fresh-exact-bin-10613")
    def rows():
        out=[]
        for revision in revisions:
            for case in STEP_CASES:
                r=copy.deepcopy(early);r["model_revision"]=revision;r["case"]=case
                r["completed"]=True;r["owner_quality_passed"]=True;out.append(r)
        return out
    class Protocol(unittest.TestCase):
        def test_exact36(self): self.assertEqual(len(STEP_CASES),12);self.assertTrue(validate_coverage(revisions,rows()))
        def test_missing_reordered_duplicate_rejected(self):
            for bad in (rows()[:-1],list(reversed(rows())),rows()[:-1]+[rows()[0]]):
                with self.assertRaises(Rejected):validate_coverage(revisions,bad)
        def test_failed_numerics_not_hidden(self):
            r=copy.deepcopy(early);r["forward_passed"]=False;r["motor_case_passed"]=True;self.assertFalse(motor_passed(r))
        def test_last_model_must_pass(self):
            data=rows();data[-1]["owner_quality_passed"]=False;self.assertFalse(validate_coverage(revisions,data))
        def test_motor_guard_not_waived(self):
            r=copy.deepcopy(early);r["START_dose_A2s"]=.027;self.assertFalse(motor_passed(r))
        def test_reference_deadlines_unchanged(self):
            for case in STEP_CASES:
                duration,timing,_=manoeuvre("step",case["direction"],case["step_deg"])
                self.assertEqual(timing["command_time"],2.);self.assertGreaterEqual(duration,timing["zero_reference_time"]+2.);self.assertEqual(abs(timing["step_rad"]),np.deg2rad(case["step_deg"]))
    with (output/"protocol-tests.txt").open("w") as stream:
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Protocol))
    print((output/"protocol-tests.txt").read_text());return result.wasSuccessful()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fits",type=Path,required=True);parser.add_argument("--library",type=Path,required=True)
    parser.add_argument("--margins",type=Path,required=True);parser.add_argument("--START",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--stage",choices=("early","full","protocol-tests","reversal-early","reversal-full","reversal-protocol-tests","compact"),required=True)
    args=parser.parse_args()
    if args.stage=="early":early(args.fits,args.library,args.margins,args.START,args.output)
    elif args.stage=="full":full(args.fits,args.library,args.margins,args.START,args.output)
    elif args.stage=="reversal-early":reversal_early(args.fits,args.library,args.margins,args.START,args.output)
    elif args.stage=="reversal-full":reversal_full(args.fits,args.library,args.margins,args.START,args.output)
    elif args.stage=="reversal-protocol-tests":raise SystemExit(not reversal_protocol_tests(args.output))
    elif args.stage=="compact":compact_evidence(args.output)
    else:raise SystemExit(not protocol_tests(args.output))
