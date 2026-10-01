"""Selected dynamic candidate through stationary, step and reversal forecasts."""
from dataclasses import asdict, replace
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import Envelope
from Firmware.commissioning.family_forecast import forecast, synthetic_motion_metrics
from Firmware.commissioning.metrics import shaped_step
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.motor_feedforward import BoundedStartPolicy, parameters_for_family
from Firmware.commissioning.native import Native
from Firmware.commissioning.planned_start import PlannedStartLeg, PlannedStartProgram
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_family_forecast_probe import contract, feedforward_fixture, packet
from Firmware.tools.adr0022_family_synthesis_probe import candidate_parameters, dynamic_candidate_fixture


def save(path, record):
    path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")


def manoeuvre(kind, direction, step_deg, *, planned_start_program=False):
    """One frozen q/v/a generator; no observed-state or future-input dependence."""
    start = 2.
    if kind == "stationary":
        return 4., {"command_time": start, "zero_reference_time": start, "step_rad": 0.}, \
            lambda c, now: replace(packet(c, now), q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0.,
                                  trajectory_phase="HOLD")
    if kind == "step":
        envelope = Envelope(.35, 2., .8, np.deg2rad(30.), 3., -1., 1., 20., False, "SYNTHETIC")
        distance = direction * float(np.deg2rad(step_deg))
        times, refs, timing = shaped_step(distance, envelope, .005)
        times, refs = np.r_[0., times], np.vstack((np.zeros(4), refs))
        midpoint = (timing["command_time"] + timing["zero_reference_time"]) / 2

        def reference(c, now):
            q, v, a = [float(np.interp(now, times, refs[:, j])) for j in range(3)]
            result = replace(packet(c, now), q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a)
            if now < start or now >= timing["zero_reference_time"]:
                return replace(result, trajectory_phase="HOLD", v_ref_rad_s=0., a_ref_rad_s2=0.)
            if now <= midpoint and direction*a >= 0:
                return replace(result, trajectory_phase="DEPARTURE", planned_direction=direction,
                    departure_offset_s=start, departure_position_rad=0.)
            return replace(result, trajectory_phase="BRAKING" if v*a < 0 else "TRACKING")

        return round(float(times[-1]), 9), timing, reference
    # Polynomial excursion: continuous q/v/a, one reversal and exact final rest.
    # These dimensions are diagnostic coverage, not a new physical authority.
    duration, coefficient, stop = 4., direction*8., 6.
    release = (5.-np.sqrt(5.))/10.  # first acceleration zero, x in (0,.5)

    def reference(c, now):
        x = float(np.clip((now-start)/duration, 0., 1.))
        q = coefficient*x**3*(1-x)**3
        v = coefficient/duration*(3*x**2-12*x**3+15*x**4-6*x**5)
        a = coefficient/duration**2*(6*x-36*x**2+60*x**3-30*x**4)
        result = replace(packet(c, now), q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a)
        if now < start or now >= stop:
            return replace(result, trajectory_phase="HOLD", v_ref_rad_s=0., a_ref_rad_s2=0.)
        if x <= release:
            return replace(result, trajectory_phase="DEPARTURE", planned_direction=direction,
                departure_offset_s=start, departure_position_rad=0.)
        if planned_start_program and .5 <= x <= 1.-release:
            return replace(result, trajectory_phase="DEPARTURE", planned_direction=-direction,
                departure_offset_s=4., departure_position_rad=direction*.125)
        return replace(result, trajectory_phase="BRAKING" if v*a < 0 else "REVERSAL" if v == 0 else "TRACKING")

    return 8., {"command_time": start, "zero_reference_time": stop}, reference


def run(library, candidate, output, *, kind, direction, step_deg, noisy_seed,
        planned_start_program=False, negative_excess_mA=60, positive_excess_mA=60):
    if planned_start_program and kind != "reversal":
        raise ValueError("distinct planned legs require the declared reversal generator")
    if negative_excess_mA not in (20, 40, 60, 80) or positive_excess_mA not in (20, 40, 60, 80):
        raise ValueError("existing finite startup-excess grid required")
    if output.exists() and any(output.iterdir()):
        raise ValueError("fresh evidence directory required")
    output.mkdir(parents=True, exist_ok=True)
    gains, fixtures = dynamic_candidate_fixture(candidate)
    model, _, _, template, _, _, _ = fixtures[0]
    duration, timing, producer = manoeuvre(kind, direction, step_deg,
                                           planned_start_program=planned_start_program)
    c = replace(contract(101 if noisy_seed is None else noisy_seed, noisy_seed is not None, duration),
        frame="output_shaft_rad", trajectory_id=f"declared-{kind}-{direction}-{step_deg:g}")
    start = BoundedStartPolicy(configuration_id=c.configuration_id,
        source="Frozen synthetic development directional grid; unqualified START policy", q_min_rad=-1., q_max_rad=1.,
        static_negative_interval_A=(.156, .164), static_positive_interval_A=(.156, .164),
        negative_excess_A=negative_excess_mA/1000., positive_excess_A=positive_excess_mA/1000., max_attempt_s=.200,
        max_command_dose_A2s=.026, max_attempts=1)
    support, state = feedforward_fixture(c, model, actuator_policy="STEADY_STATE_REFERENCE",
        actuation_memory_max_s=.03, start_policy=start)
    if planned_start_program:
        support = replace(support, planned_start_program=PlannedStartProgram(
            configuration_id=c.configuration_id, frame=c.frame, trajectory_id=c.trajectory_id,
            generation=1, source_time_s=0., expires_at_s=c.duration_s,
            legs=(PlannedStartLeg(direction=direction, departure_offset_s=2., departure_position_rad=0.),
                  PlannedStartLeg(direction=-direction, departure_offset_s=4.,
                                  departure_position_rad=direction*.125))))
    parameters = parameters_for_family(candidate_parameters(template, gains), model,
        actuator_policy=support.actuator_policy, actuation_memory_max_s=support.actuation_memory_max_s,
        start_policy=start)
    limits = {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9}
    declaration = {"scope": "SYNTHETIC_SELECTED_DYNAMIC_COMPLETE_MANOEUVRE",
        "kind": kind, "direction": direction, "step_deg": step_deg if kind == "step" else None,
        "candidate": str(candidate), "library": str(library), "gains": asdict(gains),
        "model": model.document(), "forecast_contract": asdict(c), "reference_timing": timing,
        "reference_generator": "existing shaped_step" if kind == "step" else
            "constant0 q/v/a" if kind == "stationary" else "8*direction*x^3*(1-x)^3; x=(t-2)/4",
        "start_policy": asdict(start), "planned_start_program": asdict(support.planned_start_program)
            if planned_start_program else None, "initial": [0.]*5, "input_prehistory_A": 0.,
        "parameter_recovery": "NOT_RUN; supplied synthetic model",
        "numeric_forward_gates": limits, "original_quality_gates": "existing motion_metrics unchanged",
        "reversal_deadline": "NONE_DECLARED; transitions and fixed final2s drift diagnosed",
        "cases_are_development": True, "uncertainty_coverage": "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"predeclared-contract.json", declaration)
    began = time.monotonic()
    data = forecast(Native(library), FamilyNative(library), model, parameters, c,
        lambda now: producer(c, now), initial=np.zeros(5),
        feedforward_support=support, feedforward_state=state)
    independent = independent_rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"]).trace
    errors = {name: float(np.sqrt(np.mean((data["truth"][mask, j]-independent[mask, j])**2)))
        for name, j, mask in (("q_rms_rad", 0, slice(None)),
                             ("gyro_rms_rad_s", 3, data["v_new"]), ("current_rms_A", 4, slice(None)))}
    complete = data["report"]["outcome"]["status"] == "COMPLETED"
    quality = {"status": "NOT_RUN", "detail": ("full reversal-quality contract not declared"
        if complete and kind == "reversal" else "incomplete full manoeuvre/window")}
    if complete and kind != "reversal":
        quality = synthetic_motion_metrics(data, c, **timing, gyro_bandwidth_hz=10.)
    t, q = data["t"], data["q"]
    stop = (t >= timing["zero_reference_time"]) & (t <= timing["zero_reference_time"]+2.)
    full_stop = t[-1] >= timing["zero_reference_time"]+2.-1e-12
    drift = float(np.max(np.abs(q[stop]-q[np.flatnonzero(stop)[0]]))) if full_stop else None
    commands = data["commands"]
    motion = commands[:, 9].astype(int)
    transitions = [{"time_s": float(commands[i, 0]), "motion": int(motion[i])}
                   for i in np.flatnonzero(np.r_[True, np.diff(motion) != 0])] if len(motion) else []
    before = t < timing["command_time"]
    report = {"kind": kind, "direction": direction, "noisy_seed": noisy_seed,
        "elapsed_s": time.monotonic()-began, "native_report": data["report"], "complete_manoeuvre": complete,
        "forward_errors_conditional_on_own_successful_input": errors,
        "forward_passed": all(errors[k] <= limits[k] for k in limits), "original_quality": quality,
        "original_quality_passed": quality.get("metrics", {}).get("passed", "NOT_RUN"),
        "motion_transitions": transitions, "successful_ACK_count": len(data["tx_t"])-1,
        "maximum_precommand_observed_displacement_rad": float(np.max(np.abs(q[before]-q[0]))),
        "maximum_abs_latent_velocity_rad_s": float(np.max(np.abs(data["truth"][:, 1]))),
        "fixed_final_two_second_observed_anchor_drift_rad": drift,
        "fixed_final_two_second_drift_passed": bool(drift <= np.deg2rad(.15)) if drift is not None else "NOT_RUN",
        "reversal_reference_time_s": 4. if kind == "reversal" else None,
        "observed_displacement_rad": float(q[-1]-q[0]),
        "coverage_status": "DEVELOPMENT_SINGLE_CASE; NOT_FULL_STAGE1",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    report["numerical_interface_passed"] = bool(complete and report["forward_passed"]
        and data["report"]["causal_sensor_replay_max_error"] <= 1e-12)
    report["exit_code_semantics"] = "0 requires full completion and numerical interface; original quality is separate"
    save(output/"result.json", report)
    np.savez_compressed(output/"trace.npz", **{k:v for k,v in data.items() if isinstance(v, np.ndarray)},
        independent_same_successful_input=independent)
    print(json.dumps(report), flush=True)
    return 0 if report["numerical_interface_passed"] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--case", choices=("stationary", "step", "reversal"), required=True)
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--step-deg", type=float, choices=(.5, 1., 5.), default=1.)
    parser.add_argument("--noisy-seed", type=int)
    parser.add_argument("--planned-start-program", action="store_true")
    parser.add_argument("--negative-excess-mA", type=int, choices=(20, 40, 60, 80), default=60)
    parser.add_argument("--positive-excess-mA", type=int, choices=(20, 40, 60, 80), default=60)
    args = parser.parse_args()
    return run(args.library, args.candidate, args.output_dir, kind=args.case, direction=args.direction,
               step_deg=args.step_deg, noisy_seed=args.noisy_seed,
               planned_start_program=args.planned_start_program,
               negative_excess_mA=args.negative_excess_mA, positive_excess_mA=args.positive_excess_mA)


if __name__ == "__main__":
    raise SystemExit(main())
