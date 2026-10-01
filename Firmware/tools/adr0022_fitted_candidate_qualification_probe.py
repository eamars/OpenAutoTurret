"""Bounded automatic nonlinear qualification of fitted-family local candidates.

The complete original curve is evaluated and frozen before any new trajectory.
At most three new, common locally feasible entries are tried in deterministic
order. All three fitted assets must pass all six complete plateaux before a
candidate can be selected as conditional synthetic development evidence.
"""
from dataclasses import asdict
import argparse
import csv
import io
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, require
from Firmware.commissioning.family_assets import (bind_diagnostic_runtime,
    native_parameter_document, runtime_document)
from Firmware.commissioning.family_sampled_analysis import LocalGains
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters
from Firmware.tools.adr0022_fresh_family_control_probe import (FIT_CASES, METHOD as BRIDGE_METHOD,
    WN_GRID, DAMPING_GRID, FF_POLICY, load_fits, fitted_asset, local_solve,
    runtime_receipt, control_case, save)
from Firmware.tools.adr0022_start_policy_synthesis_probe import (PLATEAU_CASES,
    FORWARD_GATES, ORIGINAL_JITTER_RAD, REVISED_JITTER_RAD, case_label, case_passed)


METHOD = "adr0022.fitted-candidate-nonlinear-qualification/1"
CONSUMED_CURVE = (4., 1.)
MAX_NEW_CURVES = 3
EARLY_CASE = PLATEAU_CASES[0]


def curve_id(curve):
    return f"wn{curve[0]:g}-zeta{curve[1]:g}"


def curve_keys(local):
    return tuple((entry["wn_rad_s"], entry["damping_ratio"]) for entry in local["candidates"])


def shared_curves(locals_by_asset):
    require(len(locals_by_asset) == 3 and all(curve_keys(local) ==
        tuple((w, z) for w in WN_GRID for z in DAMPING_GRID) for local in locals_by_asset),
        Reason.INTEGRATION_MISMATCH, "all three exact original 24-entry local curves are required")
    feasible = set.intersection(*({(c["wn_rad_s"], c["damping_ratio"])
        for c in local["candidates"] if c["passed"] is True} for local in locals_by_asset))
    ordered = sorted(feasible, key=lambda item: (-item[0], item[1]))
    return tuple(ordered), tuple(c for c in ordered if c != CONSUMED_CURVE)[:MAX_NEW_CURVES]


def local_entry(local, curve):
    return next(entry for entry in local["candidates"] if
        (entry["wn_rad_s"], entry["damping_ratio"]) == curve)


def mapped_gains(asset, local, curve):
    entry = local_entry(local, curve)
    require(entry["passed"] is True, Reason.ENVELOPE_LIMITED,
        "only frozen locally feasible candidates may enter nonlinear qualification")
    model = asset.model
    minimum_damping = local["minimum_incremental_damping_A_s_rad"]
    gains = LocalGains((2*curve[1]*model.a*curve[0]-minimum_damping)/model.actuator_gain,
        model.a*curve[0]**2/model.actuator_gain, curve[0]/5., 3.)
    require(asdict(gains) == entry["gains"], Reason.INTEGRATION_MISMATCH,
        "each asset uses its own exact frozen analytic gain rule and nominal Kaw")
    return gains


def write_csv(path, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ["status"])
    writer.writeheader(); writer.writerows(rows)
    path.write_text(stream.getvalue())


def local_margin_rows(triples, locals_by_asset):
    rows = []
    for (asset, _, _), local in zip(triples, locals_by_asset):
        for entry in local["candidates"]:
            diagnostics = [p.get("diagnostics") for p in entry["points"]]
            valid = bool(diagnostics) and all(d is not None for d in diagnostics)
            rows.append({"asset_revision": asset.model_revision, "wn_rad_s": entry["wn_rad_s"],
                "damping_ratio": entry["damping_ratio"], "local_passed": entry["passed"],
                "kp": entry["gains"]["kp"], "ki": entry["gains"]["ki"],
                "kpos": entry["gains"]["kpos"], "kaw": entry["gains"]["kaw"],
                "worst_phase_margin_deg": min(d["phase_margin_deg"] for d in diagnostics) if valid else "",
                "worst_gain_margin_db": min(d["gain_margin_db"] for d in diagnostics) if valid else "",
                "worst_spectral_radius_per_period": max(d["pole_diagnostics"]["spectral_radius_per_period"]
                    for d in diagnostics) if valid else ""})
    return rows


def frozen_contract(source, library, records, triples, common, selected):
    return {"schema": METHOD, "scope": "FITTED_PLANT_CONDITIONAL_PRODUCT_B_DEVELOPMENT_ONLY",
        "fit_source": str(source.resolve()), "fit_cases": list(FIT_CASES),
        "model_revisions": [a.model_revision for a, _, _ in triples],
        "models": [record["model"] for record in records], "bridge_method": BRIDGE_METHOD,
        "library": str(library.resolve()), "WN_grid_rad_s": list(WN_GRID),
        "damping_ratio_grid": list(DAMPING_GRID), "phase_required_deg": 45., "gain_required_db": 6.,
        "ff_policy": FF_POLICY, "consumed_curve_excluded": list(CONSUMED_CURVE),
        "all_common_locally_feasible_curves": [list(c) for c in common],
        "maximum_new_curves": MAX_NEW_CURVES, "new_curve_order": [list(c) for c in selected],
        "ordering": "fastest WN, then lowest damping ratio, locally feasible in ALL three fitted models",
        "stop_rule": "first candidate passing all18 complete cases; otherwise exhaust at most three declared entries",
        "mandatory_cases_each_curve": list(PLATEAU_CASES), "mandatory_assets_each_curve": 3,
        "early_discriminator": {"fit_case": FIT_CASES[0], "case": EARLY_CASE},
        "early_result_reused_without_redraw": True,
        "START_directional_excess_A": {"negative": .060, "positive": .080},
        "START_static_intervals_A": [.156, .164], "START_attempt_s": .2,
        "START_attempts": 1, "START_command_dose_A2s": .026, "actuation_memory_max_s": .03,
        "current_cap_A": .35, "slew_A_s": 2., "sample_control_gyro_period_s": [.001, .005, .020],
        "initial": [0.]*5, "successful_TX_prehistory_A": 0.,
        "reference": "same complete shaped signed5deg/s plateaux; fixed anchors and two-second stop",
        "forward_gates": FORWARD_GATES, "causal_sensor_replay_max_error": 1e-12,
        "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "owner_revised_position_jitter_limit_rad": REVISED_JITTER_RAD,
        "all_other_motion_gates": "unchanged synthetic_motion_metrics",
        "rejection": "ANY full-case motor/quality/numerical/readback/START/current/slew failure rejects candidate",
        "uncertainty": "UNKNOWN; no calibrated joint set or parameter/member selection",
        "noise_and_input_role": "same consumed control cases; no fresh verification claim",
        "model_qualified": False, "controller_qualified": False,
        "independent_unknown_true_plant_forecast": "NOT_RUN", "full_domain_coverage": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def freeze_and_early(source, library, output):
    records = load_fits(source)
    triples = [fitted_asset(source, record) for record in records]
    output.mkdir(parents=True, exist_ok=False)
    save(output/"frozen-fit-records.json", records)
    locals_by_asset = []
    for asset, support, _ in triples:
        result, _ = local_solve(asset, support)
        local = result.document()
        require(local["phase_required_deg"] == 45. and local["gain_required_db"] == 6.,
            Reason.INTEGRATION_MISMATCH, "owner margin requirements must remain unchanged")
        locals_by_asset.append(local)
        path = output/"frozen-assets"/asset.model_revision
        path.mkdir(parents=True)
        save(path/"family-asset.json", asset.document())
        save(path/"local-margin-result.json", local)
    common, selected = shared_curves(locals_by_asset)
    save(output/"frozen-local-margin-records.json", locals_by_asset)
    write_csv(output/"local-margins.csv", local_margin_rows(triples, locals_by_asset))
    protocol = frozen_contract(source, library, records, triples, common, selected)
    save(output/"predeclared-contract.json", protocol)
    require(bool(selected), Reason.ENVELOPE_LIMITED, "no new common locally feasible curve entry")
    curve = selected[0]
    asset, support, state = triples[0]
    gains = mapped_gains(asset, locals_by_asset[0], curve)
    _, template = local_solve(asset, support)
    path = output/"early"/curve_id(curve)/asset.model_revision
    path.mkdir(parents=True)
    native, family = Native(library), FamilyNative(library)
    parameters, support, _ = runtime_receipt(asset, support, native, candidate_parameters(template, gains), path)
    row = control_case(native, family, asset, support, state, parameters, EARLY_CASE, path/case_label(EARLY_CASE))
    receipt = {"curve": list(curve), "asset_revision": asset.model_revision, "case": EARLY_CASE,
        "result_path": str((path/case_label(EARLY_CASE)/"result.json").resolve()),
        "candidate_case_passed": case_passed(row), "all72_local_margin_outcomes_frozen_before_motion": True}
    save(output/"early-result.json", receipt)
    print(json.dumps(receipt), flush=True)


def compact_case(curve, asset, row):
    m = row.get("original_quality", {}).get("metrics", {})
    return {"curve": curve_id(curve), "asset_revision": asset.model_revision,
        "speed_deg_s": row["case"]["speed_deg_s"], "noisy": row["case"]["noisy"], "seed": row["case"]["seed"],
        "completed": row["completed"], "candidate_case_passed": case_passed(row),
        "original_quality_passed": row["original_quality_passed"],
        "owner_quality_passed": row["revised_quality_passed"], "forward_passed": row["conditional_forward_passed"],
        "actual_complete_readback_count": row["actual_controller_readback_count"],
        "q_forward_rms_rad": row["forward_errors"]["q_rms_rad"],
        "gyro_forward_rms_rad_s": row["forward_errors"]["gyro_rms_rad_s"],
        "current_forward_rms_A": row["forward_errors"]["current_rms_A"],
        "START_command_dose_A2s": row["START_realized_command_dose_A2s"],
        "max_command_A": row["report"]["maximum_successful_command_A"],
        "max_slew_A_s": row["report"]["maximum_successful_slew_A_s"],
        **{name: m.get(name, "") for name in ("tracking_ratio", "active_fraction", "speed_rms_rad_s",
            "speed_jitter_rad_s", "position_jitter_rad", "position_range_rad", "gyro_band_rms_rad_s",
            "stop_drift_rad", "sustained_start_s")}}


def attempt_passed(revisions, rows):
    require(len(revisions) == 3 and len(set(revisions)) == 3 and len(rows) == 18 and
        tuple((r["model_revision"], r["case"]) for r in rows) ==
            tuple((revision, case) for revision in revisions for case in PLATEAU_CASES),
        Reason.DATA_INVALID, "candidate decision requires the exact ordered three-model, six-case coverage")
    return all(case_passed(row) for row in rows)


def full(source, library, output):
    records = load_fits(source)
    require(json.loads((output/"frozen-fit-records.json").read_text()) == records,
        Reason.INTEGRATION_MISMATCH, "the exact parsed retained source records must remain frozen")
    triples = [fitted_asset(source, record) for record in records]
    locals_by_asset = json.loads((output/"frozen-local-margin-records.json").read_text())
    common, selected = shared_curves(locals_by_asset)
    require(json.loads((output/"predeclared-contract.json").read_text()) ==
        frozen_contract(source, library, records, triples, common, selected), Reason.INTEGRATION_MISMATCH,
        "all fitted vectors, curve ordering, masks and motor gates must match the frozen protocol")
    for (asset, _, _), local in zip(triples, locals_by_asset):
        folder = output/"frozen-assets"/asset.model_revision
        require(json.loads((folder/"family-asset.json").read_text()) == asset.document() and
            json.loads((folder/"local-margin-result.json").read_text()) == local,
            Reason.INTEGRATION_MISMATCH, "frozen candidate margins must bind exact current fitted asset")
    early = json.loads((output/"early-result.json").read_text())
    require(early["curve"] == list(selected[0]) and early["asset_revision"] == triples[0][0].model_revision and
        early["case"] == EARLY_CASE and early["all72_local_margin_outcomes_frozen_before_motion"] is True,
        Reason.INTEGRATION_MISMATCH, "chronological early discriminator prerequisite changed")
    early_row = json.loads(Path(early["result_path"]).read_text())
    expected_early_path = output/"early"/curve_id(selected[0])/triples[0][0].model_revision/case_label(EARLY_CASE)
    require(Path(early["result_path"]).resolve() == (expected_early_path/"result.json").resolve() and
        early_row["case"] == EARLY_CASE and early_row["model_revision"] == early["asset_revision"] and
        case_passed(early_row) == early_row["candidate_case_passed"] == early["candidate_case_passed"],
        Reason.INTEGRATION_MISMATCH, "early result must be retained and reused without replay/redraw")
    native, family = Native(library), FamilyNative(library)
    first_asset, first_support, _ = triples[0]
    first_gains = mapped_gains(first_asset, locals_by_asset[0], selected[0])
    first_template = controller_parameters_for(first_asset, first_gains)
    first_parameters, first_support, first_binding = bind_diagnostic_runtime(
        first_asset, native, first_template, first_support)
    saved_early_runtime = json.loads((expected_early_path.parent/"runtime-family-asset.json").read_text())
    saved_early_readback = json.loads((expected_early_path/"actual-controller-readback.json").read_text())
    require(saved_early_runtime == runtime_document(first_asset, first_template, first_parameters, first_support, first_binding) and
        saved_early_readback["complete_native_parameters"] == native_parameter_document(first_parameters) and
        saved_early_readback["all_equal_frozen_candidate"] is True and
        saved_early_readback["successful_readback_count"] == early_row["actual_controller_readback_count"] > 0,
        Reason.INTEGRATION_MISMATCH, "early trajectory must bind the first exact mapped candidate and its actual complete native readback")
    attempts, case_rows, selected_candidate = [], [], None
    for curve in selected:
        attempt = {"curve": list(curve), "gains_by_asset": [], "cases": []}
        for i, (asset, support, state) in enumerate(triples):
            gains = mapped_gains(asset, locals_by_asset[i], curve)
            attempt["gains_by_asset"].append({"model_revision": asset.model_revision, "gains": asdict(gains)})
            path = output/"trials"/curve_id(curve)/asset.model_revision
            path.mkdir(parents=True, exist_ok=False)
            template = controller_parameters_for(asset, gains)
            parameters, support, _ = runtime_receipt(asset, support, native, template, path)
            for case in PLATEAU_CASES:
                if curve == selected[0] and i == 0 and case == EARLY_CASE:
                    row = early_row
                else:
                    row = control_case(native, family, asset, support, state, parameters, case, path/case_label(case))
                attempt["cases"].append(row)
                case_rows.append(compact_case(curve, asset, row))
                write_csv(output/"case-results.csv", case_rows)
                save(output/"partial-results.json", {"attempts": attempts+[attempt]})
        attempt["candidate_passed"] = attempt_passed(tuple(asset.model_revision for asset, _, _ in triples), attempt["cases"])
        attempts.append(attempt)
        save(output/"partial-results.json", {"attempts": attempts})
        if attempt["candidate_passed"]:
            selected_candidate = curve
            break
    decision = decision_document(common, selected, attempts, selected_candidate)
    save(output/"decision.json", decision)
    print(json.dumps(decision), flush=True)


def controller_parameters_for(asset, gains):
    # Same supplied nominal template as the frozen bridge; no gain borrowing.
    from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters
    return candidate_parameters(controller_parameters(), gains)


def decision_document(common, declared, attempts, winner):
    allcases = [row for attempt in attempts for row in attempt["cases"]]
    return {"schema": METHOD, "status": "SELECTED_CONDITIONAL_SYNTHETIC_DEVELOPMENT_CANDIDATE" if winner else
        "NO_FEASIBLE_CANDIDATE_IN_THIS_BOUNDED_NONLINEAR_SELECTION",
        "selected_curve": list(winner) if winner else None, "maximum_new_curves": MAX_NEW_CURVES,
        "declared_curve_order": [list(c) for c in declared],
        "attempts": [{"curve": a["curve"], "candidate_passed": a["candidate_passed"],
            "gains_by_asset": a["gains_by_asset"], "complete_cases": sum(r["completed"] for r in a["cases"]),
            "original_quality_passes": sum(r["original_quality_passed"] for r in a["cases"]),
            "owner_quality_passes": sum(r["revised_quality_passed"] for r in a["cases"]),
            "motor_case_passes": sum(case_passed(r) for r in a["cases"]),
            "independent_forward_passes": sum(r["conditional_forward_passed"] for r in a["cases"])} for a in attempts],
        "remaining_locally_feasible_curves": [{"curve": list(c), "nonlinear_status": "NOT_RUN"}
            for c in common if c != CONSUMED_CURVE and list(c) not in [a["curve"] for a in attempts]],
        "consumed_curve": {"curve": list(CONSUMED_CURVE), "status": "PREVIOUS18/18_NONLINEAR_FAIL; EXCLUDED_NO_RESTART"},
        "original_jitter_limit_rad": ORIGINAL_JITTER_RAD, "owner_jitter_limit_rad": REVISED_JITTER_RAD,
        "all_other_gates": "UNCHANGED", "total_complete_cases": sum(r["completed"] for r in allcases),
        "total_case_passes": sum(case_passed(r) for r in allcases),
        "total_independent_forward_passes": sum(r["conditional_forward_passed"] for r in allcases),
        "total_actual_complete_readbacks": sum(r["actual_controller_readback_count"] for r in allcases),
        "uncertainty": "UNKNOWN", "scope": "FITTED_PLANT_CONDITIONAL_PRODUCT_B_DEVELOPMENT_ONLY",
        "global_infeasibility_proven": False, "model_qualified": False, "controller_qualified": False,
        "independent_unknown_true_plant_forecast": "NOT_RUN", "full_domain_coverage": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fits", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "full"), required=True)
    args = parser.parse_args()
    if args.stage == "early": freeze_and_early(args.fits, args.library, args.output)
    else: full(args.fits, args.library, args.output)
