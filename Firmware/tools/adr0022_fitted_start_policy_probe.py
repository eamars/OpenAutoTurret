"""Bounded family-specific START excess discriminator, offline development only."""
from dataclasses import asdict, replace
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
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters
from Firmware.tools.adr0022_fresh_family_control_probe import (FIT_CASES, WN_GRID, DAMPING_GRID,
    load_fits, fitted_asset, runtime_receipt, control_case, save)
from Firmware.tools.adr0022_fitted_candidate_qualification_probe import (mapped_gains,
    curve_keys, controller_parameters_for, attempt_passed, compact_case)
from Firmware.tools.adr0022_start_policy_synthesis_probe import (EXCESS_GRID_A, PLATEAU_CASES,
    FORWARD_GATES, ORIGINAL_JITTER_RAD, REVISED_JITTER_RAD, case_label, case_passed, start_policy)


METHOD = "adr0022.fitted-family-start-policy-development/1"
FIXED_CURVE = (2., 1.5)
EARLY_EXCESS_A = .060
EARLY_CASE = next(case for case in PLATEAU_CASES if case["direction"] == 1 and
    case["noisy"] is True and case["seed"] == 101)


def symmetric_id(excess):
    return f"symmetric-{round(excess*1000):d}mA"


def policy_support(support, negative, positive=None):
    positive = negative if positive is None else positive
    require(negative in EXCESS_GRID_A and positive in EXCESS_GRID_A, Reason.DATA_INVALID,
        "only the original20/40/60/80mA grid amounts are permitted")
    policy = replace(start_policy(negative, support.configuration_id), positive_excess_A=positive)
    return replace(support, start_policy=policy)


def source_margin_records(margins, triples):
    locals_by_asset = json.loads((margins/"frozen-local-margin-records.json").read_text())
    require(len(locals_by_asset) == 3 and all(curve_keys(local) ==
        tuple((w, z) for w in WN_GRID for z in DAMPING_GRID) and
        local["phase_required_deg"] == 45. and local["gain_required_db"] == 6.
        for local in locals_by_asset), Reason.INTEGRATION_MISMATCH,
        "retain all original local curves with unchanged owner45-degree/6-dB requirements")
    for (asset, _, _), local in zip(triples, locals_by_asset):
        frozen_asset = json.loads((margins/"frozen-assets"/asset.model_revision/"family-asset.json").read_text())
        require(frozen_asset == asset.document(), Reason.INTEGRATION_MISMATCH,
            "completed nonlinear curve must bind the exact fitted FamilyAsset")
        mapped_gains(asset, local, FIXED_CURVE)
    return locals_by_asset


def frozen_contract(source, library, margins, records, triples, locals_by_asset):
    return {"schema": METHOD, "scope": "FITTED_COULOMB_FAMILY_START_DISCRIMINATOR_DEVELOPMENT",
        "fit_source": str(source.resolve()), "margin_source": str(margins.resolve()),
        "fit_cases": list(FIT_CASES), "models": [record["model"] for record in records],
        "library": str(library.resolve()), "fixed_curve": list(FIXED_CURVE),
        "gains_by_asset": [{"model_revision": asset.model_revision,
            "gains": asdict(mapped_gains(asset, local, FIXED_CURVE))}
            for (asset, _, _), local in zip(triples, locals_by_asset)],
        "phase_required_deg": 45., "gain_required_db": 6.,
        "excess_grid_A": list(EXCESS_GRID_A), "symmetric_variant_count": 4,
        "mandatory_models_each_variant": 3, "mandatory_cases_each_model": list(PLATEAU_CASES),
        "mandatory_symmetric_grid_cases": 72,
        "early_discriminator": {"asset_revision": triples[0][0].model_revision,
            "case": EARLY_CASE, "symmetric_excess_A": EARLY_EXCESS_A},
        "early_result_reused_without_redraw": True,
        "directional_selection": "minimum grid excess passing ALL9 cases of that sign across all models/noise draws",
        "combined_pair": "freeze chosen negative/positive excesses, then actually rerun ALL18; no reuse for final combined verdict",
        "FF_policy": "STEADY_STATE_REFERENCE", "actuation_memory_max_s": .03,
        "START_static_intervals_A": [.156, .164], "START_attempt_s": .2,
        "START_attempts": 1, "START_command_dose_A2s": .026,
        "current_cap_A": .35, "slew_A_s": 2., "initial": [0.]*5,
        "known_static_rest_balance_A": -.02, "successful_TX_prehistory_A": 0.,
        "sample_control_gyro_period_s": [.001, .005, .020],
        "reference": "same complete shaped signed5deg/s plateaux, unchanged anchors and fixed two-second stop",
        "forward_gates": FORWARD_GATES, "causal_sensor_replay_max_error": 1e-12,
        "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "owner_revised_position_jitter_limit_rad": REVISED_JITTER_RAD,
        "all_other_quality_gates": "UNCHANGED synthetic_motion_metrics",
        "other_amounts_restarts_extra_attempts_reference_changes": "NOT_PERMITTED",
        "noise_and_input_role": "CONSUMED DEVELOPMENT; same101/307 control draws; no fresh validation claim",
        "uncertainty": "UNKNOWN", "model_qualified": False, "controller_qualified": False,
        "independent_unknown_true_plant_forecast": "NOT_RUN", "full_domain_coverage": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def write_csv(path, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader(); writer.writerows(rows)
    path.write_text(stream.getvalue())


def compact_start_case(variant, negative, positive, asset, parameters, row):
    return {"variant": variant, "negative_excess_A": negative, "positive_excess_A": positive,
        "native_START_negative_total_A": parameters.start_total[0],
        "native_START_positive_total_A": parameters.start_total[3*parameters.model.n],
        **compact_case(FIXED_CURVE, asset, row)}


def freeze_and_early(source, library, margins, output):
    records = load_fits(source)
    triples = [fitted_asset(source, record) for record in records]
    locals_by_asset = source_margin_records(margins, triples)
    output.mkdir(parents=True, exist_ok=False)
    save(output/"frozen-fit-records.json", records)
    save(output/"frozen-local-margin-records.json", locals_by_asset)
    for asset, _, _ in triples:
        folder = output/"frozen-assets"/asset.model_revision
        folder.mkdir(parents=True)
        save(folder/"family-asset.json", asset.document())
    save(output/"predeclared-contract.json", frozen_contract(source, library, margins, records, triples, locals_by_asset))
    asset, support, state = triples[0]
    support = policy_support(support, EARLY_EXCESS_A)
    gains = mapped_gains(asset, locals_by_asset[0], FIXED_CURVE)
    template = controller_parameters_for(asset, gains)
    folder = output/"early"/symmetric_id(EARLY_EXCESS_A)/asset.model_revision
    folder.mkdir(parents=True)
    native, family = Native(library), FamilyNative(library)
    parameters, support, _ = runtime_receipt(asset, support, native, template, folder)
    row = control_case(native, family, asset, support, state, parameters, EARLY_CASE, folder/case_label(EARLY_CASE))
    receipt = {"asset_revision": asset.model_revision, "case": EARLY_CASE, "symmetric_excess_A": EARLY_EXCESS_A,
        "native_START_total_A": {"negative": parameters.start_total[0],
            "positive": parameters.start_total[3*parameters.model.n]},
        "result_path": str((folder/case_label(EARLY_CASE)/"result.json").resolve()),
        "candidate_case_passed": case_passed(row), "all72_grid_cases_frozen_before_motion": True}
    save(output/"early-result.json", receipt)
    print(json.dumps(receipt), flush=True)


def admit_early(native, output, triples, locals_by_asset):
    asset, support, _ = triples[0]
    support = policy_support(support, EARLY_EXCESS_A)
    gains = mapped_gains(asset, locals_by_asset[0], FIXED_CURVE)
    template = controller_parameters_for(asset, gains)
    parameters, support, binding = bind_diagnostic_runtime(asset, native, template, support)
    receipt = json.loads((output/"early-result.json").read_text())
    folder = output/"early"/symmetric_id(EARLY_EXCESS_A)/asset.model_revision
    path = folder/case_label(EARLY_CASE)
    row = json.loads((path/"result.json").read_text())
    readback = json.loads((path/"actual-controller-readback.json").read_text())
    require(receipt["asset_revision"] == asset.model_revision and receipt["case"] == EARLY_CASE and
        receipt["symmetric_excess_A"] == EARLY_EXCESS_A and receipt["all72_grid_cases_frozen_before_motion"] is True and
        Path(receipt["result_path"]).resolve() == (path/"result.json").resolve() and
        row["model_revision"] == asset.model_revision and row["case"] == EARLY_CASE and
        case_passed(row) == row["candidate_case_passed"] == receipt["candidate_case_passed"] and
        json.loads((folder/"runtime-family-asset.json").read_text()) == runtime_document(asset, template, parameters, support, binding) and
        readback["complete_native_parameters"] == native_parameter_document(parameters) and
        readback["all_equal_frozen_candidate"] is True and
        readback["successful_readback_count"] == row["actual_controller_readback_count"] > 0,
        Reason.INTEGRATION_MISMATCH, "early result must bind exact newly remapped60/60 START parameters and actual native readback")
    return row


def select_directional(grid, revisions):
    require(tuple(entry["excess_A"] for entry in grid) == EXCESS_GRID_A,
        Reason.DATA_INVALID, "all four ordered original START variants are required")
    for entry in grid:
        attempt_passed(revisions, entry["cases"])  # validates exact coverage even on a quality failure
    selection = {}
    reports = []
    for direction in (-1, 1):
        passing = []
        for entry in grid:
            cases = [row for row in entry["cases"] if row["case"]["direction"] == direction]
            require(len(cases) == 9, Reason.DATA_INVALID, "every sign needs all three models and three cases")
            passes = sum(case_passed(row) for row in cases)
            reports.append({"direction": direction, "excess_A": entry["excess_A"],
                "motor_case_passes": passes, "required_cases": 9,
                "original_quality_passes": sum(row["original_quality_passed"] for row in cases),
                "owner_quality_passes": sum(row["revised_quality_passed"] for row in cases)})
            if passes == 9: passing.append(entry["excess_A"])
        selection[direction] = min(passing) if passing else None
    return selection, reports


def full(source, library, margins, output):
    records = load_fits(source)
    require(json.loads((output/"frozen-fit-records.json").read_text()) == records,
        Reason.INTEGRATION_MISMATCH, "exact retained fits must remain frozen")
    triples = [fitted_asset(source, record) for record in records]
    locals_by_asset = source_margin_records(margins, triples)
    require(json.loads((output/"frozen-local-margin-records.json").read_text()) == locals_by_asset and
        json.loads((output/"predeclared-contract.json").read_text()) ==
            frozen_contract(source, library, margins, records, triples, locals_by_asset) and
        all(json.loads((output/"frozen-assets"/asset.model_revision/"family-asset.json").read_text()) == asset.document()
            for asset, _, _ in triples), Reason.INTEGRATION_MISMATCH,
        "fixed gains, family receipts, original grid and motor gates must match declaration")
    native, family = Native(library), FamilyNative(library)
    early = admit_early(native, output, triples, locals_by_asset)
    grid, case_rows = [], []
    for excess in EXCESS_GRID_A:
        entry = {"excess_A": excess, "cases": []}
        for i, (asset, support, state) in enumerate(triples):
            support = policy_support(support, excess)
            gains = mapped_gains(asset, locals_by_asset[i], FIXED_CURVE)
            folder = output/"symmetric-grid"/symmetric_id(excess)/asset.model_revision
            folder.mkdir(parents=True, exist_ok=False)
            parameters, support, _ = runtime_receipt(asset, support, native, controller_parameters_for(asset, gains), folder)
            for case in PLATEAU_CASES:
                row = early if excess == EARLY_EXCESS_A and i == 0 and case == EARLY_CASE else control_case(
                    native, family, asset, support, state, parameters, case, folder/case_label(case))
                entry["cases"].append(row)
                case_rows.append(compact_start_case(symmetric_id(excess), excess, excess, asset, parameters, row))
                write_csv(output/"case-results.csv", case_rows)
                save(output/"partial-grid-results.json", {"variants": grid+[entry]})
        attempt_passed(tuple(asset.model_revision for asset, _, _ in triples), entry["cases"])
        grid.append(entry)
        save(output/"partial-grid-results.json", {"variants": grid})
    revisions = tuple(asset.model_revision for asset, _, _ in triples)
    selected, direction_rows = select_directional(grid, revisions)
    write_csv(output/"directional-selection.csv", direction_rows)
    combined = []
    pair = None
    if selected[-1] is not None and selected[1] is not None:
        pair = {"negative": selected[-1], "positive": selected[1]}
        save(output/"combined-pair-freeze.json", {"pair_A": pair, "fixed_curve": list(FIXED_CURVE),
            "required_combined_cases": 18, "selection_basis": "minimum full9/9 of each sign across all models/seeds",
            "all_other_parameters_and_gates": "UNCHANGED", "combined_status": "NOT_RUN_YET"})
        for i, (asset, support, state) in enumerate(triples):
            support = policy_support(support, pair["negative"], pair["positive"])
            gains = mapped_gains(asset, locals_by_asset[i], FIXED_CURVE)
            folder = output/"combined-pair"/asset.model_revision
            folder.mkdir(parents=True, exist_ok=False)
            parameters, support, _ = runtime_receipt(asset, support, native, controller_parameters_for(asset, gains), folder)
            for case in PLATEAU_CASES:
                row = control_case(native, family, asset, support, state, parameters, case, folder/case_label(case))
                combined.append(row)
                case_rows.append(compact_start_case("combined-pair", pair["negative"], pair["positive"], asset, parameters, row))
                write_csv(output/"case-results.csv", case_rows)
                save(output/"partial-combined-results.json", {"pair_A": pair, "cases": combined})
        combined_passed = attempt_passed(revisions, combined)
    else:
        combined_passed = False
    allrows = [row for entry in grid for row in entry["cases"]]+combined
    decision = {"schema": METHOD, "status": "FAMILY_SPECIFIC_START_PAIR_CONDITIONAL_DEVELOPMENT_PASS" if combined_passed else
        "COMBINED_PAIR_MOTOR_QUALITY_FAILED" if pair else "NO_FAMILY_SPECIFIC_START_PAIR_IN_GRID",
        "fixed_curve": list(FIXED_CURVE), "selected_pair_A": pair, "directional_results": direction_rows,
        "grid_case_count": sum(len(entry["cases"]) for entry in grid),
        "combined_case_count": len(combined), "combined_passed": combined_passed,
        "combined_original_quality_pass_count": sum(row["original_quality_passed"] for row in combined),
        "combined_owner_quality_pass_count": sum(row["revised_quality_passed"] for row in combined),
        "combined_status": "COMPLETE" if combined else "NOT_RUN; one or both signs lacked9/9 support",
        "grid_original_quality_pass_count": sum(row["original_quality_passed"] for entry in grid for row in entry["cases"]),
        "grid_owner_quality_pass_count": sum(row["revised_quality_passed"] for entry in grid for row in entry["cases"]),
        "total_complete_cases": sum(row["completed"] for row in allrows),
        "total_independent_forward_pass_count": sum(row["conditional_forward_passed"] for row in allrows),
        "total_actual_complete_readbacks": sum(row["actual_controller_readback_count"] for row in allrows),
        "early_reused_once_no_redraw": True, "unavailable_directions": [direction for direction in (-1, 1) if selected[direction] is None],
        "other_START_amounts": "NOT_RUN; outside frozen grid", "other_controller_curves": "NOT_RUN; fixedWN2/zeta1.5",
        "original_jitter_limit_rad": ORIGINAL_JITTER_RAD, "owner_jitter_limit_rad": REVISED_JITTER_RAD,
        "all_other_gates": "UNCHANGED", "uncertainty": "UNKNOWN",
        "scope": "FITTED_PLANT_CONDITIONAL_PRODUCT_B_DEVELOPMENT_ONLY", "global_infeasibility_proven": False,
        "model_qualified": False, "controller_qualified": False,
        "independent_unknown_true_plant_forecast": "NOT_RUN", "full_domain_coverage": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"decision.json", decision)
    print(json.dumps(decision), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fits", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--margins", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "full"), required=True)
    args = parser.parse_args()
    if args.stage == "early": freeze_and_early(args.fits, args.library, args.margins, args.output)
    else: full(args.fits, args.library, args.margins, args.output)
