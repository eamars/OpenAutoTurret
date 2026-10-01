"""Offline synthetic selected-family/shared-core forecast and oracle probe."""
from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.family_forecast import (ForecastContract, IndependentForecastPlant, forecast,
                                                   synthetic_motion_metrics)
from Firmware.commissioning.adaptation import Envelope
from Firmware.commissioning.metrics import shaped_velocity
from Firmware.commissioning.contracts import Rejected
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.motor_feedforward import (BoundedStartPolicy, CausalState, FeedforwardSupport, ReferencePacket,
                                                     parameters_for_family)
from Firmware.commissioning.applicability import (ConfigurationFact, ConfigurationFacts,
                                                ConfigurationSupport, FactStatus)
from Firmware.commissioning.native import Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters, estimator_model


def packet(contract, now, coefficient=8.):
    # A predeclared analytic trajectory: smooth departure, reversal and two-second rest.
    if now <= .1 or now >= 1.7:
        q = v = a = 0.
    else:
        x = (now - .1) / 1.6
        # One smooth excursion; q,v,a all vanish at either endpoint.
        q = coefficient * x**3 * (1-x)**3
        v = coefficient / 1.6 * (3*x**2 - 12*x**3 + 15*x**4 - 6*x**5)
        a = coefficient / 1.6**2 * (6*x - 36*x**2 + 60*x**3 - 30*x**4)
    return ReferencePacket(q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a, time_s=now,
        source_time_s=0., expires_at_s=contract.duration_s, frame=contract.frame,
        configuration_id=contract.configuration_id, trajectory_id=contract.trajectory_id,
        generation=1, fresh=True, valid=True)


def contract(seed=101, noisy=False, duration=3.7):
    return ForecastContract(duration_s=duration, sample_dt_s=.001, control_period_samples=5,
        gyro_period_samples=20, encoder_noise_rad=.00015 if noisy else 0.,
        encoder_quantum_rad=2*np.pi/8192 if noisy else 0., gyro_noise_rad_s=.005 if noisy else 0.,
        current_noise_A=.002 if noisy else 0., seed=seed, configuration_id="synthetic-family-forecast",
        trajectory_id="declared-smooth-excursion-and-fixed-stop", frame="output-shaft-rad")


def feedforward_fixture(c, model, **policy_fields):
    """Declared synthetic context; these values provide no physical authority."""
    facts = ConfigurationFacts(c.configuration_id, "SYNTHETIC", {
        "payload.distribution": ConfigurationFact("fixed lumped synthetic fixture", FactStatus.SYNTHETIC,
                                                    "declared forecast plant"),
        "motor.settings": ConfigurationFact((model.actuator_gain, model.actuator_bias,
                                              model.actuator_tau, model.transport_delay),
                                             FactStatus.SYNTHETIC, "declared forecast current path"),
        "sensor.calibration": ConfigurationFact((model.gyro_tau, model.gyro_delay,
                                                  c.encoder_quantum_rad, c.gyro_noise_rad_s),
                                                 FactStatus.SYNTHETIC, "declared forecast sensors")})
    binding = ConfigurationSupport("declared-forecast-family", facts, tuple(facts.facts),
                                   qualification="SYNTHETIC_OFFLINE")
    support = FeedforwardSupport(configuration_id=c.configuration_id, frame=c.frame,
        q_min_rad=-1., q_max_rad=1., velocity_max_rad_s=1., acceleration_max_rad_s2=3.,
        fixed_posture_rad=0., winding_min_rad=-1., winding_max_rad=1., temperature_min=10., temperature_max=30.,
        actuator_gain_min=.5, actuator_gain_max=3., command_cap_A=.35, command_slew_A_s=2.,
        max_reference_source_age_s=c.duration_s+.01, max_state_age_s=.060, max_ack_delay_s=.005,
        rest_speed_rad_s=.020, state_encoder_consistency_rad=.02, state_gyro_consistency_rad_s=.10,
        qualification="SYNTHETIC_OFFLINE", configuration_support=binding, **policy_fields)
    state = CausalState(q_rad=0., v_rad_s=0., posture_rad=0., winding_rad=0., temperature=20., time_s=0.,
        frame=c.frame, configuration_id=c.configuration_id, generation=1,
        friction_state="POSTERIOR_POLICY", static_balance_A=0., fresh=True, valid=True,
        provenance="SYNTHETIC", configuration_facts=facts)
    return support, state


def velocity_plan(speed_deg_s):
    if not np.isfinite(speed_deg_s) or speed_deg_s == 0:
        raise ValueError("finite nonzero signed synthetic plateau speed required")
    envelope = Envelope(.35, 2., .8, np.deg2rad(30.), 3., -1., 1., 20., False, "SYNTHETIC")
    times, refs, timing = shaped_velocity(np.deg2rad(speed_deg_s), envelope, .005)
    return np.r_[0., times], np.vstack((np.zeros(4), refs)), timing


def planned_packet(c, now, times, refs):
    q, v, a = [float(np.interp(now, times, refs[:, column])) for column in range(3)]
    return replace(packet(c, now), q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a)


def phased_planned_packet(c, now, times, refs, timing):
    """Phase from the same frozen shaped trajectory, without measured-state input."""
    reference = planned_packet(c, now, times, refs)
    anchor = float(timing["command_time"])
    direction = 1 if float(np.max(refs[:, 1])) > 0 else -1
    accelerating = times[direction * refs[:, 2] > 0]
    if now < anchor - 1e-12:
        return replace(reference, trajectory_phase="HOLD")
    if len(accelerating) and now <= float(accelerating[-1]) + 1e-12:
        return replace(reference, trajectory_phase="DEPARTURE", planned_direction=direction,
                       departure_offset_s=anchor-reference.source_time_s,
                       departure_position_rad=float(np.interp(anchor, times, refs[:, 0])))
    if reference.v_ref_rad_s == 0 and reference.a_ref_rad_s2 == 0:
        return replace(reference, trajectory_phase="HOLD")
    return replace(reference, trajectory_phase=("BRAKING" if reference.v_ref_rad_s * reference.a_ref_rad_s2 < 0
                                               else "TRACKING"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--trajectory-coefficient", type=float, default=8.)
    parser.add_argument("--control-plant", choices=("native", "independent"), default="native")
    parser.add_argument("--duration-s", type=float, default=3.7)
    parser.add_argument("--only-noisy-seed", type=int)
    parser.add_argument("--motor-ff-policy", choices=("LEGACY", "STEADY_STATE_REFERENCE"), default="LEGACY")
    parser.add_argument("--reference", choices=("excursion", "velocity-plateau"), default="excursion")
    parser.add_argument("--speed-deg-s", type=float, default=5.)
    parser.add_argument("--start-policy", choices=("LEGACY", "PLANNED_DEPARTURE_DIAGNOSTIC"), default="LEGACY")
    args = parser.parse_args()
    if args.start_policy != "LEGACY" and (args.reference != "velocity-plateau" or
                                          args.motor_ff_policy != "STEADY_STATE_REFERENCE"):
        parser.error("planned departure diagnostic requires velocity-plateau and steady-state reference FF")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    native, family = Native(args.library), FamilyNative(args.library)
    backend = family if args.control_plant == "native" else IndependentForecastPlant()
    cases = ((False, 101), (True, 101), (True, 307)) if args.only_noisy_seed is None else ((True, args.only_noisy_seed),)
    model = replace(estimator_model(), actuator="first_order", actuator_tau=.0073,
        transport_delay=.0087, current_tau=.012, current_delay=.0029, friction="stribeck",
        stribeck_negative=.07, stribeck_positive=.05)
    plan = velocity_plan(args.speed_deg_s) if args.reference == "velocity-plateau" else None
    duration = round(float(plan[0][-1]), 9) if plan is not None else args.duration_s
    declared = {"scope": "SYNTHETIC_INTERFACE_AND_NUMERICAL_VERIFICATION_ONLY",
        "trajectory_coefficient": args.trajectory_coefficient,
        "trajectory_rule": ("existing shaped_velocity: 2s baseline, 2.5s plateau, fixed2s stop"
                            if plan is not None else "coefficient*x^3*(1-x)^3 over.1..1.7s, then fixed2s stop"),
        "reference": args.reference, "speed_deg_s": args.speed_deg_s if plan is not None else None,
        "reference_timing": plan[2] if plan is not None else {"zero_reference_time": 1.7},
        "plant": model.document(), "cases": [{"noisy": noisy, "seed": seed} for noisy, seed in cases],
        "control_plant_backend": args.control_plant, "duration_s": duration,
        "control_policy": "existing declared benchmark core gains; no new gain synthesis",
        "motor_ff_policy": args.motor_ff_policy,
        "start_policy": args.start_policy,
        "planned_departure_policy_scope": ("Frozen60mA above supplied synthetic static interval; full motion-quality gates failed; no qualified START policy"
                                          if args.start_policy != "LEGACY" else "NOT_RUN"),
        "actuation_memory_max_s": .03 if args.motor_ff_policy == "STEADY_STATE_REFERENCE" else "NOT_RUN",
        "forecast": "shared core generates own future current from simulated native sensors",
        "forward_limits": {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9},
        "physical_gate": "NOT_RUN; synthetic numerical agreement does not establish tracking qualification"}
    (args.output_dir / "contract.json").write_text(json.dumps(declared, indent=2) + "\n")
    results, histories = [], []
    for noisy, seed in cases:
        label = ("noisy" if noisy else "pristine") + str(seed)
        c = contract(seed, noisy, duration)
        if plan is not None:
            c = replace(c, trajectory_id="existing-shaped-velocity-plateau-and-fixed-stop")
        began = time.monotonic()
        ff = {}
        parameters = controller_parameters()
        if args.motor_ff_policy == "STEADY_STATE_REFERENCE":
            start_policy = None
            if args.start_policy == "PLANNED_DEPARTURE_DIAGNOSTIC":
                start_policy = BoundedStartPolicy(configuration_id=c.configuration_id,
                    source="Frozen synthetic development60mA candidate; not qualified", q_min_rad=-1., q_max_rad=1.,
                    static_negative_interval_A=(.156, .164), static_positive_interval_A=(.156, .164),
                    negative_excess_A=.060, positive_excess_A=.060, max_attempt_s=.200,
                    max_command_dose_A2s=.026, max_attempts=1)
            support, state = feedforward_fixture(c, model, actuator_policy="STEADY_STATE_REFERENCE",
                                                actuation_memory_max_s=.03, start_policy=start_policy)
            parameters = parameters_for_family(parameters, model, actuator_policy=support.actuator_policy,
                                                actuation_memory_max_s=support.actuation_memory_max_s,
                                                start_policy=start_policy)
            ff = {"feedforward_support": support, "feedforward_state": state}
        if plan is None:
            reference = lambda now: packet(c, now, args.trajectory_coefficient)
        elif args.start_policy != "LEGACY":
            reference = lambda now: phased_planned_packet(c, now, plan[0], plan[1], plan[2])
        else:
            reference = lambda now: planned_packet(c, now, plan[0], plan[1])
        data = forecast(native, backend, model, parameters, c, reference, initial=np.zeros(5), **ff)
        independent = independent_rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
        errors = {key: float(np.sqrt(np.mean((data["truth"][:, index] - independent.trace[:, index])**2)))
                  for key, index in (("q_rms_rad", 0), ("gyro_rms_rad_s", 3), ("current_rms_A", 4))}
        native_replay = family.rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
        native_errors = {key: float(np.sqrt(np.mean((native_replay[:, index] - independent.trace[:, index])**2)))
                        for key, index in (("q_rms_rad", 0), ("gyro_rms_rad_s", 3), ("current_rms_A", 4))}
        verified = all(errors[key] <= value for key, value in declared["forward_limits"].items())
        verified = verified and all(native_errors[key] <= value for key, value in declared["forward_limits"].items())
        report = {"case": label, **data["report"], "independent_forward_errors": errors,
            "independent_forward_passed": verified, "independent_oracle_events": independent.events,
            "native_realized_input_forward_errors": native_errors,
            "elapsed_s": time.monotonic() - began}
        refs = data["references"]
        indices = np.rint(refs[:, 0] / c.sample_dt_s).astype(int)
        zero = plan[2]["zero_reference_time"] if plan is not None else 1.7
        stop = (data["t"] >= zero) & (data["t"] <= zero + 2.)
        full_stop = data["t"][-1] >= zero + 2. - 1e-12
        report["tracking_diagnostics"] = {
            "control_sample_q_reference_rms_rad": float(np.sqrt(np.mean((data["truth"][indices, 0] - refs[:, 1])**2))),
            "maximum_abs_latent_velocity_rad_s": float(np.max(np.abs(data["truth"][:, 1]))),
            "fixed_two_second_zero_reference_latent_position_span_rad": float(np.ptp(data["truth"][stop, 0])) if full_stop else "NOT_RUN",
            "fixed_two_second_zero_reference_observed_anchor_drift_rad": float(np.max(np.abs(
                data["q"][stop] - data["q"][np.flatnonzero(stop)[0]]))) if full_stop else "NOT_RUN",
            "interpretation": "supplied controller baseline; no synthesis or physical acceptance claimed"}
        if plan is not None:
            data["reference_plan_t"], data["reference_plan"] = plan[:2]
            try:
                if report["outcome"]["status"] != "COMPLETED":
                    raise ValueError("complete forecast required before frozen motion metrics")
                report["frozen_synthetic_motion_metrics"] = synthetic_motion_metrics(data, c, **plan[2], gyro_bandwidth_hz=10.)
            except (ValueError, Rejected) as exc:
                report["frozen_synthetic_motion_metrics"] = {"status": "NOT_RUN", "detail": str(exc)}
        data.pop("report")
        np.savez_compressed(args.output_dir / (label + ".npz"), **data, independent=independent.trace, native_replay=native_replay)
        (args.output_dir / (label + ".json")).write_text(json.dumps(report, indent=2) + "\n")
        histories.append(data["tx_A"])
        results.append(report)
        print(json.dumps({"case": label, "outcome": report["outcome"], "forward": errors,
                          "causal_replay": report["causal_sensor_replay_max_error"]}), flush=True)
    summary = {"cases": len(results), "completed_forecasts": sum(row["outcome"]["status"] == "COMPLETED" for row in results),
        "independent_forward_passes": sum(row["independent_forward_passed"] for row in results),
        "feedback_noise_changes_successful_input": not np.array_equal(histories[0], histories[1]) if len(histories) >= 2 else "NOT_RUN",
        "distinct_noise_changes_successful_input": not np.array_equal(histories[1], histories[2]) if len(histories) >= 3 else "NOT_RUN",
        "synthetic_forecast_interface": "PASS" if all(row["outcome"]["status"] == "COMPLETED"
            and row["independent_forward_passed"] and row["causal_sensor_replay_max_error"] < 1e-10 for row in results) else "FAIL",
        "unknown_parameter_recovery": "NOT_RUN", "dynamic_motor_ff_policy": args.motor_ff_policy,
        "frozen_synthetic_metrics_evaluated": sum("metrics" in row.get("frozen_synthetic_motion_metrics", {})
                                                   for row in results) if plan is not None else "NOT_RUN",
        "frozen_synthetic_metric_passes": sum(row.get("frozen_synthetic_motion_metrics", {}).get("metrics", {}).get("passed", False)
                                              for row in results) if plan is not None else "NOT_RUN",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["synthetic_forecast_interface"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
