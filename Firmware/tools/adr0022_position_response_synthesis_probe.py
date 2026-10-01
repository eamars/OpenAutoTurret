"""Bounded position-response curve with actual selected-family sampled dynamics."""
from dataclasses import asdict
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, require
from Firmware.commissioning.family_sampled_analysis import FamilySampledAnalysis, LocalGains
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.tools.adr0022_family_synthesis_probe import dynamic_candidate_fixture
from Firmware.tools.adr0022_start_policy_synthesis_probe import (PLATEAU_CASES, STEP_CASES,
    case_label, case_passed, declaration, run_case)


WN_GRID = (.5, 1., 2., 4.)
ZETA_GRID = (1., 1.5, 2., 2.5, 3., 4.)
POSITION_RATIO_GRID = (.2, .5, 1.)
MAX_NATIVE_CANDIDATES = 3


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def predeclare(candidate, library):
    base = declaration(candidate, library)
    gains, fixtures = dynamic_candidate_fixture(candidate)
    return {**base, "scope": "SYNTHETIC_BOUNDED_POSITION_RESPONSE_GAIN_CURVE_DEVELOPMENT",
        "early_gains": {**asdict(gains), "kpos": 1.}, "WN_rad_s": list(WN_GRID),
        "damping_ratio": list(ZETA_GRID), "Kpos_over_WN": list(POSITION_RATIO_GRID),
        "curve_candidates": 72, "maximum_actual_candidates": MAX_NATIVE_CANDIDATES,
        "exact_local_method": "full selected-family scheduled plant/observer/controller/FF/ACK tangent; all states/delays",
        "local_requirements": {"phase_deg": 45., "gain_db": 6., "all_closed_poles_stable": True},
        "gain_formula": "Kp=(2*zeta*a*WN-minimum signed incremental damping)/actuator gain; Ki=a*WN^2/g; Kpos=ratio*WN; supplied Kaw",
        "selection": "fastest WN, lowest zeta, largest Kpos/WN; maximum3 locally feasible candidates",
        "fixed_directional_START_excess_A": {"negative": .060, "positive": .080},
        "excess_grid_A_effective": None, "early_discriminator": "signed pristine0.5deg step after local prerequisites",
        "pristine_step_cases_if_all_plateaus_pass": None, "mandatory_step_cases": list(STEP_CASES),
        "early_cases_reused_without_new_draws": False,
        "actual_stop_rule": "run all six plateaus and six steps for each selected candidate; stop only after all12 pass revised motor gates",
        "no_HOLD_retries_or_motor_guard_changes": True}


def evaluate_local(gains, fixtures):
    points = []
    for model, point, support, parameters, schedule, _, policy in fixtures:
        analysis = FamilySampledAnalysis(model, point, support, parameters.observer, gains, schedule, ff_policy=policy)
        margins = analysis.margin_diagnostics(phase_required_deg=45., gain_required_db=6.)
        points.append({"point": asdict(point), "diagnostics": margins, "passed": bool(margins["passed"])})
    return {"gains": asdict(gains), "points": points, "passed": all(p["passed"] for p in points)}


def early(candidate, library, output):
    output.mkdir(parents=True, exist_ok=False)
    save(output/"predeclared-contract.json", predeclare(candidate, library))
    gains, fixtures = dynamic_candidate_fixture(candidate)
    changed = LocalGains(gains.kp, gains.ki, 1., gains.kaw)
    began = time.monotonic()
    local = evaluate_local(changed, fixtures)
    save(output/"early-local.json", local)
    print(json.dumps({"stage": "early-local", "gains": asdict(changed), "passed": local["passed"],
        "margins": [{k: p["diagnostics"].get(k) for k in ("status", "phase_margin_deg", "gain_margin_db")}
                    for p in local["points"]], "elapsed_s": time.monotonic()-began}), flush=True)
    cases = []
    if local["passed"]:
        native, family = Native(library), FamilyNative(library)
        for case in STEP_CASES:
            if case["step_deg"] != .5: continue
            cases.append(run_case(native, family, changed, fixtures, case, .060,
                output/"early"/case_label(case), positive_excess=.080))
    record = {"status": "EARLY_LOCAL_AND_NATIVE_DISCRIMINATOR_RETAINED" if local["passed"] else
        "EARLY_LOCAL_MARGIN_FAILED_NATIVE_STEP_NOT_RUN", "reason": None if local["passed"] else Reason.ENVELOPE_LIMITED.value,
        "local": local, "native_cases": cases, "qualification": "DEVELOPMENT_ONLY; NOT_PROMOTED"}
    save(output/"early-result.json", record)


def rank_local_candidates(records):
    expected = tuple((wn, zeta, ratio) for wn in WN_GRID for zeta in ZETA_GRID for ratio in POSITION_RATIO_GRID)
    require(tuple((r["WN_rad_s"], r["damping_ratio"], r["Kpos_over_WN"]) for r in records) == expected,
            Reason.DATA_INVALID, "all72 unique ordered prescribed local candidates must remain in the decision")
    feasible = [r for r in records if r["passed"] and len(r["points"]) == 2 and all(p["passed"] for p in r["points"])]
    return sorted(feasible, key=lambda r: (-r["WN_rad_s"], r["damping_ratio"], -r["Kpos_over_WN"]))[:MAX_NATIVE_CANDIDATES]


def synthesize(candidate, library, output):
    require(json.loads((output/"predeclared-contract.json").read_text()) == predeclare(candidate, library),
            Reason.INTEGRATION_MISMATCH, "position-response declaration differs from frozen early context")
    supplied, fixtures = dynamic_candidate_fixture(candidate)
    damping = min(FamilySampledAnalysis(model, point, support, params.observer, supplied, schedule,
        ff_policy=policy).local.incremental_damping_A_s_rad for model, point, support, params, schedule, _, policy in fixtures)
    model = fixtures[0][0]
    records = []
    began = time.monotonic()
    for wn in WN_GRID:
        for zeta in ZETA_GRID:
            for ratio in POSITION_RATIO_GRID:
                gains = LocalGains((2*zeta*model.a*wn-damping)/model.actuator_gain,
                    model.a*wn**2/model.actuator_gain, ratio*wn, supplied.kaw)
                record = {"WN_rad_s": wn, "damping_ratio": zeta, "Kpos_over_WN": ratio,
                    **evaluate_local(gains, fixtures)}
                records.append(record)
        print(json.dumps({"stage": "local-curve", "through_WN_rad_s": wn, "candidates": len(records),
            "feasible": sum(r["passed"] for r in records), "elapsed_s": time.monotonic()-began}), flush=True)
    save(output/"all-local-results.json", {"candidates": records,
        "minimum_signed_incremental_damping_A_s_rad": damping, "gates": {"phase_deg": 45., "gain_db": 6.}})
    selected = rank_local_candidates(records)
    save(output/"preselected-native-candidates.json", {"candidates": selected,
        "selection": "fastest WN, lowest zeta, largest Kpos/WN; maximum3; frozen before all selected native runs"})
    native, family = Native(library), FamilyNative(library)
    native_results, accepted = [], None
    for number, record in enumerate(selected, 1):
        gains = LocalGains(**record["gains"])
        folder = output/f"native-candidate-{number}"
        cases = []
        for case in (*PLATEAU_CASES, *STEP_CASES):
            cases.append(run_case(native, family, gains, fixtures, case, .060,
                folder/case_label(case), positive_excess=.080))
            save(output/"native-results.json", {"finished_candidates": native_results,
                "active_candidate": number, "active_cases": cases, "physical_qualification": "NOT_RUN"})
        result = {"candidate_number": number, "local_candidate": record, "cases": cases,
            "all_declared_cases_passed": all(case_passed(r) for r in cases),
            "original_quality_pass_count": sum(r["original_quality_passed"] for r in cases),
            "revised_quality_pass_count": sum(r["revised_quality_passed"] for r in cases)}
        native_results.append(result)
        save(output/"native-results.json", {"finished_candidates": native_results, "physical_qualification": "NOT_RUN"})
        if result["all_declared_cases_passed"]:
            accepted = number
            break
    decision = {"status": "BOUNDED_POSITION_RESPONSE_ALL_DECLARED_CASES_PASS" if accepted is not None else
        "NO_POSITION_RESPONSE_ALL_DECLARED_DOMAIN_CANDIDATE" if selected else "NO_LOCAL_FEASIBLE_POSITION_RESPONSE_CANDIDATE",
        "reason": None if accepted is not None else Reason.ENVELOPE_LIMITED.value,
        "accepted_candidate_number": accepted, "local_curve_count": len(records),
        "local_feasible_count": sum(r["passed"] for r in records),
        "preselected_candidates": selected, "executed_candidates": native_results,
        "unexecuted_preselected_candidates": len(selected)-len(native_results),
        "unexecuted_reason": "earlier candidate passed every declared case" if accepted is not None else None,
        "fresh_control_validation": "NOT_RUN; supplied model and consumed development cases",
        "qualification": "SYNTHETIC_DEVELOPMENT_ONLY; FULL_STAGE1/UNCERTAINTY/PHYSICAL_NOT_QUALIFIED",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"decision-detailed.json", decision)
    print(json.dumps({k: v for k, v in decision.items() if k not in ("executed_candidates", "preselected_candidates")}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "synthesize"), default="early")
    args = parser.parse_args()
    if args.stage == "early": early(args.candidate, args.library, args.output)
    else: synthesize(args.candidate, args.library, args.output)
