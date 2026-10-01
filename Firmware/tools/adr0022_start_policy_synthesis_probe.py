"""Finite single-episode START excess synthesis under an explicit revised contract."""
from dataclasses import asdict, replace
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.contracts import Reason, require
from Firmware.commissioning.family_forecast import forecast, synthetic_motion_metrics
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.motor_feedforward import BoundedStartPolicy, parameters_for_family
from Firmware.commissioning.native import Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_family_forecast_probe import (contract, feedforward_fixture,
    phased_planned_packet, velocity_plan)
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters, dynamic_candidate_fixture
from Firmware.tools.adr0022_manoeuvre_forecast_probe import manoeuvre


EXCESS_GRID_A = (.020, .040, .060, .080)
FORWARD_GATES = {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9}
ORIGINAL_JITTER_RAD, REVISED_JITTER_RAD = float(np.deg2rad(.15)), float(np.deg2rad(.16))
PLATEAU_CASES = tuple({"kind": "plateau", "direction": d, "speed_deg_s": d*5.,
    "noisy": noisy, "seed": seed} for d in (-1, 1) for noisy, seed in ((False, 101), (True, 101), (True, 307)))
STEP_CASES = tuple({"kind": "step", "direction": d, "step_deg": step, "noisy": False, "seed": 101}
    for d in (-1, 1) for step in (.5, 1., 5.))
EARLY_CASE = next(c for c in PLATEAU_CASES if c["direction"] == 1 and c["noisy"] and c["seed"] == 101)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def case_label(case):
    motion = f"plateau{case['speed_deg_s']:g}" if case["kind"] == "plateau" else f"step{case['direction']*case['step_deg']:g}"
    return f"{motion}-{'noisy' if case['noisy'] else 'pristine'}-{case['seed']}"


def start_policy(excess, configuration_id, *, positive_excess=None):
    require(type(excess) in (int, float) and excess in EXCESS_GRID_A, Reason.DATA_INVALID,
            "only the predeclared 20/40/60/80 mA START excesses are supported")
    require(positive_excess is None or (excess, positive_excess) == (.060, .080), Reason.DATA_INVALID,
            "only the separately authorized consumed negative60/positive80 mA pair is supported")
    return BoundedStartPolicy(configuration_id=configuration_id,
        source="Frozen four-point synthetic single-episode START excess curve; unqualified",
        q_min_rad=-1., q_max_rad=1., static_negative_interval_A=(.156, .164),
        static_positive_interval_A=(.156, .164), negative_excess_A=excess,
        positive_excess_A=excess if positive_excess is None else positive_excess,
        max_attempt_s=.200, max_command_dose_A2s=.026, max_attempts=1)


def declaration(candidate, library):
    local = json.loads(candidate.read_text())
    require(local.get("phase_required_deg") == 45. and local.get("gain_required_db") == 6. and
            local.get("selected_wn_rad_s") == 2. and local.get("selected_damping_ratio") == 1.5,
            Reason.DATA_INVALID, "this frozen branch requires the revised45-degree WN2/zeta1.5 candidate")
    gains, fixtures = dynamic_candidate_fixture(candidate)
    return {"scope": "SYNTHETIC_SINGLE_EPISODE_START_POLICY_SYNTHESIS_DEVELOPMENT",
        "candidate": str(candidate.resolve()), "library": str(library.resolve()), "gains": asdict(gains),
        "model": fixtures[0][0].document(), "excess_grid_A_effective": list(EXCESS_GRID_A),
        "static_intervals_A_effective": {"negative": [.156, .164], "positive": [.156, .164]},
        "max_attempt_s": .200, "max_attempts": 1, "max_START_command_dose_A2s": .026,
        "current_cap_A": .35, "slew_A_s": 2., "planned_start_program": None,
        "plateau_cases_per_candidate": list(PLATEAU_CASES),
        "pristine_step_cases_if_all_plateaus_pass": list(STEP_CASES),
        "early_discriminator": EARLY_CASE, "early_cases_reused_without_new_draws": True,
        "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "revised_position_jitter_limit_rad": REVISED_JITTER_RAD,
        "performance_relaxation": "owner diagnostic 0.16deg position jitter only; original0.15deg assessed separately",
        "selection": "minimum declared symmetric excess passing every plateau and six signed pristine step cases",
        "forward_gates": FORWARD_GATES, "other_motion_metrics_and_guards": "unchanged existing evaluator/native core",
        "initial": [0.]*5, "prehistory_A": 0., "control_encoder_gyro_period_s": [.005, .001, .020],
        "fresh_noise_or_validation": "NOT_RUN; retained declared development seeds only",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}


def run_case(native, family, gains, fixtures, case, excess, output, *, positive_excess=None):
    output.mkdir(parents=True, exist_ok=False)
    model, _, _, template, _, _, _ = fixtures[0]
    if case["kind"] == "plateau":
        times, refs, timing = velocity_plan(case["speed_deg_s"])
        duration = round(float(times[-1]), 9)
        producer = lambda c, now: phased_planned_packet(c, now, times, refs, timing)
    else:
        duration, timing, producer = manoeuvre("step", case["direction"], case["step_deg"])
    c = replace(contract(case["seed"], case["noisy"], duration), frame="output_shaft_rad",
        trajectory_id="frozen-single-episode-start-"+case_label(case))
    policy = start_policy(excess, c.configuration_id, positive_excess=positive_excess)
    support, state = feedforward_fixture(c, model, actuator_policy="STEADY_STATE_REFERENCE",
        actuation_memory_max_s=.03, start_policy=policy, planned_start_program=None)
    parameters = parameters_for_family(candidate_parameters(template, gains), model,
        actuator_policy=support.actuator_policy, actuation_memory_max_s=support.actuation_memory_max_s,
        start_policy=policy)
    require(parameters.current_cap == .35 and parameters.slew == 2. and parameters.start_timeout_s == .2,
            Reason.INTEGRATION_MISMATCH, "frozen current/slew/start limits changed")
    save(output/"predeclared-contract.json", {"case": case, "start_policy": asdict(policy),
        "gains": asdict(gains), "forecast_contract": asdict(c), "reference_timing": timing,
        "original_position_jitter_limit_rad": ORIGINAL_JITTER_RAD,
        "revised_position_jitter_limit_rad": REVISED_JITTER_RAD, "forward_gates": FORWARD_GATES})
    began = time.monotonic()
    data = forecast(native, family, model, parameters, c, lambda now: producer(c, now),
        initial=np.zeros(5), feedforward_support=support, feedforward_state=state)
    comparison = independent_rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"]).trace
    errors = {name: float(np.sqrt(np.mean((data["truth"][mask, j]-comparison[mask, j])**2)))
        for name, j, mask in (("q_rms_rad", 0, slice(None)),
            ("gyro_rms_rad_s", 3, data["v_new"]), ("current_rms_A", 4, slice(None)))}
    completed = data["report"]["outcome"]["status"] == "COMPLETED"
    original = revised = {"status": "NOT_RUN", "detail": "fault truncated the complete motion/stop window"}
    if completed:
        original = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.)
        revised = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.,
            position_jitter_limit_rad=REVISED_JITTER_RAD)
    commands = data["commands"]
    start_rows = commands[:, 9] == 1
    start_dose = float(np.sum(commands[start_rows, 2]**2)*.005)
    numerical = all(errors[k] <= FORWARD_GATES[k] for k in errors) and data["report"]["causal_sensor_replay_max_error"] <= 1e-12
    row = {"case": case, "excess_A_effective": excess, "elapsed_s": time.monotonic()-began,
        "START_excess_A_effective": {"negative": policy.negative_excess_A, "positive": policy.positive_excess_A},
        "report": data["report"], "completed": completed, "forward_errors": errors,
        "conditional_forward_passed": numerical, "successful_ACK_count": len(data["tx_t"])-1,
        "START_successful_tick_count": int(np.sum(start_rows)), "START_realized_command_dose_A2s": start_dose,
        "START_dose_passed": start_dose <= .026, "max_abs_command_A": float(np.max(np.abs(data["tx_A"]))),
        "original_quality": original, "revised_quality": revised,
        "original_quality_passed": original.get("metrics", {}).get("passed", False) is True,
        "revised_quality_passed": revised.get("metrics", {}).get("passed", False) is True}
    row["candidate_case_passed"] = completed and numerical and row["START_dose_passed"] and row["revised_quality_passed"]
    save(output/"result.json", row)
    np.savez_compressed(output/"trace.npz", **{k: v for k, v in data.items() if isinstance(v, np.ndarray)},
        independent_same_successful_input=comparison)
    print(json.dumps({"case": case, "excess_A_effective": excess, "outcome": data["report"]["outcome"],
        "original_quality_passed": row["original_quality_passed"], "revised_quality_passed": row["revised_quality_passed"],
        "forward_passed": numerical, "dose_A2s": start_dose, "elapsed_s": row["elapsed_s"]}), flush=True)
    return row


def case_passed(row):
    return (row["completed"] and row["conditional_forward_passed"] and row["START_dose_passed"] and
        row["START_realized_command_dose_A2s"] <= .026 and row["revised_quality_passed"] and
        row["report"]["maximum_successful_command_A"] <= .35+1e-12 and
        row["report"]["maximum_successful_slew_A_s"] <= 2.+1e-12)


def select_minimum(candidate_rows):
    require(len(candidate_rows) == len(EXCESS_GRID_A) and
            tuple(row["excess_A_effective"] for row in candidate_rows) == EXCESS_GRID_A,
            Reason.DATA_INVALID, "all ordered predeclared START candidates must remain in the decision")
    for row in candidate_rows:
        require(tuple(r["case"] for r in row["plateau_cases"]) == PLATEAU_CASES and
                (not row["step_cases"] or tuple(r["case"] for r in row["step_cases"]) == STEP_CASES),
                Reason.DATA_INVALID, "all exact prescribed cases must be retained; duplicates do not provide coverage")
        require(not all(case_passed(r) for r in row["plateau_cases"]) or len(row["step_cases"]) == 6,
                Reason.DATA_INVALID, "a plateau-feasible candidate requires all six declared step results")
    feasible = [r for r in candidate_rows if len(r["step_cases"]) == 6 and
        all(case_passed(c) for c in (*r["plateau_cases"], *r["step_cases"]))]
    return {"status": "SELECTED_SYNTHETIC_SINGLE_EPISODE_CANDIDATE" if feasible else "NO_SINGLE_EPISODE_ALL_DOMAIN_CANDIDATE",
        "reason": None if feasible else Reason.ENVELOPE_LIMITED.value,
        "selected_excess_A_effective": feasible[0]["excess_A_effective"] if feasible else None,
        "qualification": "DEVELOPMENT_CANDIDATE_ONLY; PHYSICAL/FRESH/UNCERTAINTY_NOT_RUN"}


def run_directional(candidate, library, output):
    output.mkdir(parents=True, exist_ok=False)
    expected = declaration(candidate, library)
    expected.update(scope="SYNTHETIC_FIXED_DIRECTIONAL_START_PAIR_DEVELOPMENT",
        excess_grid_A_effective=None, fixed_directional_excess_A_effective={"negative": .060, "positive": .080},
        selection="fixed from consumed symmetric-grid directional evidence; actual combined-policy replay required",
        pristine_step_cases_if_all_plateaus_pass=None, mandatory_step_cases=list(STEP_CASES),
        early_discriminator=None, early_cases_reused_without_new_draws=False)
    save(output/"predeclared-contract.json", expected)
    gains, fixtures = dynamic_candidate_fixture(candidate)
    native, family = Native(library), FamilyNative(library)
    rows = []
    for case in (*PLATEAU_CASES, *STEP_CASES):
        rows.append(run_case(native, family, gains, fixtures, case, .060, output/case_label(case), positive_excess=.080))
        save(output/"all-results.json", {"cases": rows, "physical_qualification": "NOT_RUN"})
    passing = all(case_passed(r) for r in rows)
    decision = {"status": "FIXED_DIRECTIONAL_PAIR_ALL_DECLARED_CASES_PASS" if passing else
        "FIXED_DIRECTIONAL_PAIR_DECLARED_DOMAIN_FAILED", "reason": None if passing else Reason.ENVELOPE_LIMITED.value,
        "negative_excess_A_effective": .060,
        "positive_excess_A_effective": .080, "cases": rows,
        "original_quality_pass_count": sum(r["original_quality_passed"] for r in rows),
        "revised_quality_pass_count": sum(r["revised_quality_passed"] for r in rows),
        "fresh_validation": "NOT_RUN; pair selected from consumed development results",
        "qualification": "DEVELOPMENT_ONLY; FULL_STAGE1_DOMAIN/UNCERTAINTY/PHYSICAL_NOT_QUALIFIED",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"decision-detailed.json", decision)
    print(json.dumps({k: v for k, v in decision.items() if k != "cases"}), flush=True)


def run(candidate, library, output, *, stage):
    expected = declaration(candidate, library)
    if stage == "early":
        output.mkdir(parents=True, exist_ok=False)
        save(output/"predeclared-contract.json", expected)
    else:
        require(json.loads((output/"predeclared-contract.json").read_text()) == expected,
                Reason.INTEGRATION_MISMATCH, "early declaration differs from frozen synthesis context")
    gains, fixtures = dynamic_candidate_fixture(candidate)
    native, family = Native(library), FamilyNative(library)
    if stage == "early":
        early = [run_case(native, family, gains, fixtures, EARLY_CASE, excess,
            output/"early"/f"excess-{int(round(excess*1000))}mA") for excess in EXCESS_GRID_A]
        save(output/"early-results.json", {"cases": early, "all_startup_guards_preserved": True})
        return
    early = json.loads((output/"early-results.json").read_text())["cases"]
    candidates = []
    for excess, retained in zip(EXCESS_GRID_A, early):
        require(retained["case"] == EARLY_CASE and retained["excess_A_effective"] == excess,
                Reason.INTEGRATION_MISMATCH, "early case/grid association changed")
        plateaus = [retained if case == EARLY_CASE else run_case(native, family, gains, fixtures, case, excess,
            output/f"excess-{int(round(excess*1000))}mA"/case_label(case)) for case in PLATEAU_CASES]
        steps = [run_case(native, family, gains, fixtures, case, excess,
            output/f"excess-{int(round(excess*1000))}mA"/case_label(case)) for case in STEP_CASES] \
            if all(r["candidate_case_passed"] for r in plateaus) else []
        candidates.append({"excess_A_effective": excess, "plateau_cases": plateaus, "step_cases": steps,
            "steps_status": "RUN" if steps else "NOT_RUN; at least one declared plateau gate failed"})
        save(output/"all-candidate-results.json", {"candidates": candidates, "physical_qualification": "NOT_RUN"})
    decision = {**select_minimum(candidates), "candidates": candidates,
        "original_contract": "0.15deg jitter; all original assessments retained",
        "revised_contract": "owner 0.16deg jitter / 45deg local phase margin; all other gates unchanged",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"decision-detailed.json", decision)
    print(json.dumps({k: v for k, v in decision.items() if k != "candidates"}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("early", "synthesize", "directional"), required=True)
    args = parser.parse_args()
    if args.stage == "directional": run_directional(args.candidate, args.library, args.output)
    else: run(args.candidate, args.library, args.output, stage=args.stage)
