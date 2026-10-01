"""Finite-prefix native controller perturbations, synthetic/offline only."""
from __future__ import annotations

import argparse
import csv
import ctypes as ct
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.family_sampled_analysis import LocalGains, SampleSchedule, controller_tick, observer_tick
from Firmware.commissioning.family_analysis import SlidingPoint, SlidingSupport
from Firmware.commissioning.model_family import FamilyModel, FamilyNative
from Firmware.commissioning.synthesis import selected_family_sampled_analysis
from Firmware.commissioning.native import CModel, CObserver, CObservation, CParameters, CReference, Controller, Native


def fixture():
    theta = [.1]*3+[.06]*3+[.02-.12]*15+[.02+.12]*15+[0.]
    p = CParameters()
    p.model = CModel(5, 0, (ct.c_double*8)(-10., -5., 0., 5., 10.),
        (ct.c_double*3)(-1., 0., 1.), (ct.c_double*55)(*theta))
    p.observer = CObserver(1e-8, 4e-6, .1, .03, .03, 1e-8, 4e-6, 0)
    for name, value in dict(kp=.5, ki=.8, kpos=1.2, kaw=3., current_cap=10., slew=1e6,
            integral_cap=10., velocity_cap=2., dt_min=.004, dt_max=.006, intent_threshold=1e-4,
            rest_speed=1e-5, sustained_s=.02, start_timeout_s=1.).items(): setattr(p, name, value)
    for k in range(30): p.start_total[k] = .02+(-.16 if k < 15 else .16)
    return p


def native_controller_prefix(native, params, sign, *, count=160, eps=1e-6):
    dt, velocity, ff = .005, .1, .146
    rows, gyro_seq, last_gyro_time, gyro_rate = [], 0, 0., velocity
    with Controller(native, params) as core:
        core.reset(0., 0., velocity, ff, accepted_time=-.1)
        for k in range(1, count+1):
            fresh = k == 1 or k%4 == 0
            if fresh:
                gyro_seq += 1; last_gyro_time = k*dt
                gyro_rate = velocity+sign*eps*(.7*np.cos(k*.31) if k >= 80 else 0.)
            position = velocity*k*dt+sign*eps*(np.sin(k*.47) if k >= 80 else 0.)
            observation = CObservation(k*dt, k*dt, last_gyro_time, position, gyro_rate,
                k, gyro_seq, 1, 1, int(fresh))
            out = core.step_feedforward(observation, CReference(velocity*k*dt, velocity, 0., 0.), ff)
            assert out.status == 0 and out.limited == out.requested and out.start_increment == 0.
            if k >= 80: assert out.motion == 2, f"not steady MOVE: {out.motion}"
            rows.append([out.position, out.velocity, out.integral, out.requested])
            assert core.ack(out, accepted_time=k*dt)
    return np.asarray(rows)


def first_probe(library):
    native, params = Native(Path(library)), fixture()
    plus, minus = native_controller_prefix(native, params, 1), native_controller_prefix(native, params, -1)
    observed = (plus-minus)/(2e-6)
    state = np.zeros(4)
    P = np.diag([params.observer.initial_position_variance, params.observer.initial_velocity_variance])
    expected = []
    gains = LocalGains(params.kp, params.ki, params.kpos, params.kaw)
    for k in range(1, 161):
        fresh = k == 1 or k%4 == 0
        F, K, P = observer_tick(params.observer, .005, P, encoder_fresh=True, gyro_fresh=fresh)
        measurements = np.array([np.sin(k*.47), .7*np.cos(k*.31)]) if k >= 80 else np.zeros(2)
        state, output = controller_tick(state, measurements, .005, gains, F, K)
        expected.append(output)
    error = float(np.max(np.abs(observed-np.asarray(expected))))
    assert error < 1e-8, error
    return {"pass": True, "scope": "SYNTHETIC_FINITE_PREFIX_LOCAL_CONTROLLER",
        "native_prefix_ticks": 160, "gyro_hz": 50, "control_hz": 200,
        "max_posterior_integral_command_error": error, "gate": 1e-8,
        "limitations": "covariance and old-error not exposed; no native internal-state Jacobian/pole proof"}


def family_fixture(direction, *, dynamic=False, affine=False, filtered=False, fractional=False, aged=False,
                   stribeck=False, ff_policy=None, speed_rad_s=.1):
    model = FamilyModel(a=.1, viscous=.06, coulomb_negative=.12, coulomb_positive=.12,
        static_negative=.16, static_positive=.16, q_min=-10., q_max=10., actuator_gain=1.,
        actuator_bias=0., transport_delay=.00637 if fractional else 0., gyro_bias=0.,
        gyro_tau=.012 if filtered else 0., gyro_delay=.00313 if fractional else 0.,
        current_gain=.91, current_bias=.003, current_tau=.019 if filtered else 0.,
        current_delay=.00217 if fractional else 0., actuator="first_order" if dynamic else "algebraic",
        actuator_tau=.007 if dynamic else 0., load="affine" if affine else "constant",
        load_offset=.02, load_slope=.03 if affine else 0., max_step=.000005,
        friction="stribeck" if stribeck else "coulomb", stribeck_positive=.07, stribeck_negative=.07)
    point = SlidingPoint(q_rad=0., v_rad_s=direction*speed_rad_s, configuration_id="synthetic-sampled", frame="output_shaft_rad")
    support = SlidingSupport(configuration_id=point.configuration_id, frame=point.frame,
        q_min_rad=-2., q_max_rad=2., v_min_rad_s=.001 if direction > 0 else -.5,
        v_max_rad_s=.5 if direction > 0 else -.001)
    params = fixture()
    schedule = SampleSchedule(dt_s=.005, encoder_period=1, gyro_period=4,
        encoder_age_s=.00113 if aged else 0., gyro_availability_age_s=.00217 if aged else 0., encoder_quantum_rad=0., gyro_quantum_rad_s=0.,
        immediate_successful_ack=True, limits_inactive=True)
    policy = ff_policy or ("FROZEN_COMMAND_OFFSET" if dynamic or fractional else "SHARED_POSTERIOR_SLIDE_ALGEBRAIC")
    gains = LocalGains(params.kp, params.ki, params.kpos, params.kaw)
    return model, point, support, params, schedule, gains, policy


def family_native_prefix(native, family_native, model, params, schedule, direction, policy, sign,
                         *, count=280, inject=200, eps=1e-6, reference_perturb=False, causal_tick=False,
                         speed_rad_s=.1, steady_initial_both=False, initial_position_rad=0.,
                         allow_warmup_limits=False):
    dt, velocity = .005, direction*speed_rad_s
    def friction(v):
        fc = model.coulomb_positive if v > 0 else model.coulomb_negative
        fs = model.static_positive if v > 0 else model.static_negative
        vs = model.stribeck_positive if v > 0 else model.stribeck_negative
        return math.copysign(fc+(fs-fc)*np.exp(-(abs(v)/vs)**model.stribeck_power) if model.friction == "stribeck" else fc, v)
    baseline_effective = model.load_offset+model.load_slope*(initial_position_rad-model.q_origin)+model.viscous*velocity+friction(velocity)
    baseline_ff = (baseline_effective-model.actuator_bias)/model.actuator_gain
    steady = steady_initial_both or (model.friction == "stribeck" and direction > 0)
    initial = np.array([initial_position_rad, velocity, baseline_effective, velocity, baseline_effective]) if steady else np.array([initial_position_rad, 0., 0., 0., 0.])
    if causal_tick:
        assert model.transport_delay == model.gyro_delay == model.current_delay == 0.
        assert schedule.encoder_age_s == schedule.gyro_availability_age_s == 0.
    oracle_state = initial.copy()
    tx_t, tx_A, rows, plant_rows, gyro_seq, gyro_time = [-.1], [baseline_ff if steady else 0.], [], [], 0, 0.
    clock = {"pre_initial_source_skipped_ticks": [], "first_fresh_gyro_tick": None, "first_gyro_source_time_s": None,
        "initialization": "ONE_SUPPLIED_STEADY_SLIDING_INITIAL" if steady else "ONE_ZERO_REST_INITIAL",
        "initial_position_rad": initial_position_rad, "reference_speed_rad_s": velocity,
        "warmup_output_limited_ticks": 0,
        "start_transition_qualified": False}
    with Controller(native, params) as core:
        core.reset(0., initial_position_rad, initial[1], tx_A[0], accepted_time=-.1)
        for k in range(1, count+1):
            enc_at, gyro_at = k*dt-schedule.encoder_age_s, k*dt-schedule.gyro_availability_age_s
            gyro_source_at = gyro_at-model.gyro_delay
            t = np.unique(np.r_[np.arange(k+1)*dt, enc_at, gyro_at])
            if causal_tick:
                # Carry only the preceding native oracle's causal plant state.
                # No core state is reset or injected and no future observation
                # enters this synthetic closed-loop simulation.
                trace = family_native.rollout(model, [(k-1)*dt, k*dt], [tx_t[-1]], [tx_A[-1]], oracle_state)
                last = trace[-1]
                oracle_state = np.array([last[0], last[1], last[2], last[3]-model.gyro_bias,
                    (last[4]-model.current_bias)/model.current_gain])
                enc_row = gyro_row = last
            else:
                trace = family_native.rollout(model, t, tx_t, tx_A, initial)
                enc_row, gyro_row = trace[np.searchsorted(t, enc_at)], trace[np.searchsorted(t, gyro_at)]
            scheduled = k == 1 or k%4 == 0
            fresh = scheduled and gyro_source_at >= 0.
            if scheduled and not fresh: clock["pre_initial_source_skipped_ticks"].append(k)
            if fresh:
                gyro_seq += 1; gyro_time = gyro_source_at
                if clock["first_fresh_gyro_tick"] is None:
                    clock.update(first_fresh_gyro_tick=k, first_gyro_source_time_s=gyro_time)
            perturb = np.array([np.sin(k*.47), .7*np.cos(k*.31)]) if k >= inject else np.zeros(2)
            observation = CObservation(k*dt, enc_at, gyro_time,
                enc_row[0]+sign*eps*perturb[0], gyro_row[3]+sign*eps*perturb[1],
                k, gyro_seq, 1, 1, int(fresh))
            rdelta = reference_delta(k, inject) if reference_perturb and k >= inject else np.zeros(3)
            reference = CReference(initial_position_rad+velocity*k*dt+sign*eps*rdelta[0], velocity+sign*eps*rdelta[1], sign*eps*rdelta[2], 0.)
            if policy.startswith("SHARED"):
                def ff(posterior, ref):
                    effective = (model.a*ref.acceleration+model.load_offset+model.load_slope*(posterior.position-model.q_origin)+
                        model.viscous*ref.velocity+friction(ref.velocity))
                    return (effective-model.actuator_bias)/model.actuator_gain
                out = core.step_posterior_feedforward(observation, reference, ff)
            else: out = core.step_feedforward(observation, reference, baseline_ff)
            assert out.status == 0, (k, out.status)
            if k < inject and out.limited != out.requested:
                clock["warmup_output_limited_ticks"] += 1
            if not allow_warmup_limits or k >= inject:
                assert out.limited == out.requested, (k, "output limiter active")
            if k >= inject:
                assert out.motion == 2 and out.start_increment == 0., (k, out.motion)
                assert direction*trace[-1, 1] > .001
            rows.append([out.position, out.velocity, out.integral, out.requested])
            plant_rows.append([enc_row[0], gyro_row[3], trace[-1, 4], trace[-1, 1]])
            assert core.ack(out, accepted_time=k*dt)
            tx_t.append(k*dt); tx_A.append(out.limited)
    return np.asarray(rows), np.asarray(plant_rows), clock


def reference_delta(k, inject=200):
    # One causal shaped quintic trajectory, with matching q/v/a derivatives.
    duration, amplitude = .4, .2
    s = min(max((k-inject)*.005/duration, 0.), 1.)
    return amplitude*np.array([10*s**3-15*s**4+6*s**5,
        (30*s**2-60*s**3+30*s**4)/duration,
        (60*s-180*s**2+120*s**3)/duration**2])


def integrated_family_case(library, family_library, direction, *, eps=1e-5, max_step=None, reference_perturb=False,
                           inject=200, causal_tick=False, require_pass=True, steady_initial_both=False, **structure):
    native, family_native = Native(Path(library)), FamilyNative(family_library)
    model, point, support, params, schedule, gains, policy = family_fixture(direction, **structure)
    if max_step is not None: model = replace(model, max_step=max_step)
    analysis = selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule, ff_policy=policy)
    count = inject+80
    plus, plus_plant, plus_clock = family_native_prefix(native, family_native, model, params, schedule, direction, policy, 1,
        eps=eps, reference_perturb=reference_perturb, count=count, inject=inject, causal_tick=causal_tick,
        speed_rad_s=abs(point.v_rad_s), steady_initial_both=steady_initial_both)
    minus, minus_plant, minus_clock = family_native_prefix(native, family_native, model, params, schedule, direction, policy, -1,
        eps=eps, reference_perturb=reference_perturb, count=count, inject=inject, causal_tick=causal_tick,
        speed_rad_s=abs(point.v_rad_s), steady_initial_both=steady_initial_both)
    assert plus_clock == minus_clock
    observed = (plus-minus)/(2*eps)
    state, expected, expected_measurements = np.zeros(analysis.dimension), [], []
    P = np.diag([params.observer.initial_position_variance, params.observer.initial_velocity_variance])
    for k in range(1, count+1):
        fresh = (k == 1 or k%4 == 0) and k*schedule.dt_s-analysis.gyro_source_age_s >= 0.
        F, K, P = observer_tick(params.observer, .005, P, encoder_fresh=True, gyro_fresh=fresh,
            encoder_age_s=schedule.encoder_age_s, gyro_age_s=analysis.gyro_source_age_s)
        perturb = np.array([np.sin(k*.47), .7*np.cos(k*.31)]) if k >= inject else np.zeros(2)
        state, output, measured = analysis.closed_step(state, k%analysis.period,
            measurement_delta=perturb, observer_pair=(F, K),
            reference_delta=reference_delta(k, inject) if reference_perturb and k >= inject else (0., 0., 0.))
        expected.append(output); expected_measurements.append(measured)
    error = float(np.max(np.abs(observed-np.array(expected))))
    observed_measurements = (plus_plant[:,:3]-minus_plant[:,:3])/(2*eps)
    measurement_error = float(np.max(np.abs(observed_measurements-np.array(expected_measurements))))
    numerical_agreement = max(error, measurement_error) < 1e-7
    baseline_deviation = float(np.max(np.abs((plus_plant[inject-1:,3]+minus_plant[inject-1:,3])/2-point.v_rad_s)))
    warmup_deviation = float(np.max(np.abs((plus_plant[inject-21:inject-1,3]+minus_plant[inject-21:inject-1,3])/2-point.v_rad_s)))
    operating_point_established = model.friction != "stribeck" or warmup_deviation < 1e-8
    passed = numerical_agreement and operating_point_established
    if require_pass: assert passed, (structure, direction, error, measurement_error, warmup_deviation)
    return {"direction": direction, **structure, "ff_policy": policy, "native_ticks": count,
        "max_posterior_integral_command_error": error, "max_family_measurement_error": measurement_error,
        "gate": 1e-7, "eps": eps, "native_max_step_s": model.max_step,
        "reference_perturb": reference_perturb, "pass": passed, "injection_tick": inject,
        "numerical_agreement": numerical_agreement, "operating_point_established": operating_point_established,
        "baseline_velocity_max_deviation_rad_s": baseline_deviation, "causal_native_oracle_tick": causal_tick,
        "pre_injection_baseline_velocity_max_deviation_rad_s": warmup_deviation,
        "gyro_availability_age_s": schedule.gyro_availability_age_s, "gyro_signal_delay_s": model.gyro_delay,
        "gyro_source_age_s": analysis.gyro_source_age_s,
        **plus_clock,
        "pole_diagnostics": analysis.pole_diagnostics(), "margin_diagnostics": analysis.margin_diagnostics()}


def steady_state_policy_probe(library, family_library, output):
    """Separate additive policy evidence; prior corrected 14 cases untouched."""
    import json
    output.mkdir(parents=True, exist_ok=False)
    policy = "SHARED_POSTERIOR_STEADY_STATE_REFERENCE"
    structures = (dict(dynamic=True, affine=True, filtered=True, fractional=True, aged=True),
        dict(dynamic=True, filtered=True, fractional=True, aged=True, stribeck=True),
        dict(dynamic=True, filtered=True, stribeck=True),
        dict(filtered=True, fractional=True, aged=True, stribeck=True))
    contract = {"scope": "SYNTHETIC_LOCAL_SLIDING_NATIVE_PREFIX_ONLY", "policy": policy,
        "structures": list(structures), "directions": [-1, 1], "speed_rad_s": .1,
        "coherent_reference": "one quintic position perturbation with analytic matching velocity/acceleration",
        "derivatives": "dFF/dqpost=L'/g; dFF/dvref=(B+F')/g; dFF/daref=a/g; dFF/dvpost=0",
        "dynamic_policy": "steady-state algebraic command; actuator pole and transport remain forward dynamics",
        "native_step_s": 5e-6, "central_epsilon": 1e-5, "comparison_gate": 1e-7,
        "initialization": "positive supplied sliding initial; negative zero/rest initial; actual native transitions unchanged",
        "source_clock": "availability plus model gyro delay; preinitial sources skipped; signal delayed once",
        "limits": "smooth sliding, fresh periodic samples, no quantization, same-tick ACK, inactive limits/AW",
        "full_start_reversal_stop": "NOT_RUN", "native_hidden_state_jacobian": "NOT_VERIFIED",
        "physical_qualification": "NOT_RUN"}
    (output/"contract.json").write_text(json.dumps(contract, indent=2)+"\n")
    rows = []
    for direction in contract["directions"]:
        for structure in structures:
            causal_tick = bool(structure.get("stribeck") and not structure.get("fractional"))
            inject = 3800 if causal_tick else 200
            try:
                case = integrated_family_case(library, family_library, direction, **structure, ff_policy=policy,
                    reference_perturb=True, steady_initial_both=direction > 0, require_pass=False,
                    inject=inject, causal_tick=causal_tick)
            except AssertionError as exc:
                rows.append({"direction": direction, **structure, "ff_policy": policy, "pass": False,
                    "status": "NATIVE_SLIDING_PREREQUISITE_NOT_ESTABLISHED", "first_guard": str(exc),
                    "injection_tick": inject, "local_stable": None, "margin_pass": None})
                print(json.dumps(rows[-1]), flush=True)
                continue
            poles, margins = case.pop("pole_diagnostics"), case.pop("margin_diagnostics")
            row = {**case, "period_radius": poles["spectral_radius_per_period"],
                "local_stable": poles["local_stable"], "phase_margin_deg": margins["phase_margin_deg"],
                "gain_margin_db": margins["gain_margin_db"], "margin_pass": margins["passed"],
                "margin_status": margins["status"]}
            rows.append(row)
            print(json.dumps({"direction": direction, **structure, "pass": row["pass"],
                "controller_error": row["max_posterior_integral_command_error"],
                "measurement_error": row["max_family_measurement_error"]}), flush=True)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (output/"native_prefix_cases.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(rows)
    summary = {"scope": contract["scope"], "policy": policy, "cases": len(rows),
        "native_agreement_passes": sum(row["pass"] for row in rows),
        "worst_native_error": max(max(row["max_posterior_integral_command_error"],
            row["max_family_measurement_error"]) for row in rows if "max_family_measurement_error" in row),
        "gate": contract["comparison_gate"],
        "local_stable_count": sum(row["local_stable"] is True for row in rows),
        "margin_pass_count": sum(row["margin_pass"] is True for row in rows),
        "native_hidden_state_jacobian": "NOT_VERIFIED", "full_start_reversal_stop": "NOT_RUN",
        "physical_qualification": "NOT_RUN", "prior_evidence": "cycle-03 sampled-analysis remains unchanged"}
    (output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary), flush=True)
    return summary


if __name__ == "__main__":
    import json
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--library", required=True)
    parser.add_argument("--family-library"); parser.add_argument("--integrated", action="store_true")
    parser.add_argument("--output", type=Path, help="write compact synthetic local evidence; no source or hashes")
    parser.add_argument("--steady-state-ff", action="store_true", help="separate dynamic steady-state FF policy evidence")
    args = parser.parse_args()
    if args.steady_state_ff:
        if args.output is None: parser.error("--steady-state-ff requires fresh --output")
        steady_state_policy_probe(args.library, args.family_library or args.library, args.output)
        sys.exit(0)
    if args.output:
        family_library = args.family_library or args.library
        first, cases, retained = first_probe(args.library), [], []
        for direction in (-1, 1):
            for structure in (dict(affine=True), dict(filtered=True), dict(dynamic=True, filtered=True, fractional=True),
                              dict(filtered=True, fractional=True), dict(dynamic=True, filtered=True, fractional=True, aged=True)):
                case = integrated_family_case(args.library, family_library, direction, **structure)
                cases.append(case)
                print(json.dumps({"direction": direction, **structure, "native_agreement": case["pass"]}), flush=True)
            for dynamic in (False, True):
                case = integrated_family_case(args.library, family_library, direction, filtered=True, stribeck=True,
                    dynamic=dynamic, inject=3800, causal_tick=True, reference_perturb=not dynamic)
                cases.append(case)
                print(json.dumps({"direction": direction, "stribeck": True, "dynamic": dynamic, "native_agreement": case["pass"]}), flush=True)
        for eps in (1e-6, 1e-5):
            retained.append(integrated_family_case(args.library, family_library, -1, filtered=True, eps=eps,
                max_step=25e-6, require_pass=False))
        retained.append(integrated_family_case(args.library, family_library, 1, filtered=True, stribeck=True,
            reference_perturb=True, require_pass=False))
        retained.append(integrated_family_case(args.library, family_library, -1, filtered=True, stribeck=True,
            reference_perturb=True, inject=3000, causal_tick=True, require_pass=False))
        args.output.mkdir(parents=True, exist_ok=True)
        def compact(case):
            poles, margins = case["pole_diagnostics"], case["margin_diagnostics"]
            return {**{k: v for k, v in case.items() if k not in ("pole_diagnostics", "margin_diagnostics")},
                "period_radius": poles["spectral_radius_per_period"], "local_stable": poles["local_stable"],
                "phase_margin_deg": margins["phase_margin_deg"], "gain_margin_db": margins["gain_margin_db"],
                "margin_pass": margins["passed"], "margin_status": margins["status"]}
        for name, rows in (("native_prefix_cases.csv", cases), ("retained_failures.csv", retained)):
            compact_rows = [compact(row) for row in rows]
            fields = list(dict.fromkeys(key for row in compact_rows for key in row))
            with (args.output/name).open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(compact_rows)
        representative = [next(c for c in cases if c["direction"] == 1 and all(c.get(k) for k in fields))
            for fields in (("affine",), ("dynamic", "filtered", "fractional", "aged"), ("stribeck", "filtered"))]
        summary = {"scope": "SYNTHETIC_OFFLINE_LOCAL_SLIDING", "first_probe": first,
            "native_prefix_pass_count": sum(c["pass"] for c in cases), "native_prefix_case_count": len(cases),
            "max_accepted_error": max(max(c["max_posterior_integral_command_error"], c["max_family_measurement_error"]) for c in cases),
            "representative_diagnostics": [{"structure": {k: v for k, v in c.items() if k in
                ("affine", "dynamic", "filtered", "fractional", "aged", "stribeck", "ff_policy")},
                "period_radius": c["pole_diagnostics"]["spectral_radius_per_period"],
                "local_stable": c["pole_diagnostics"]["local_stable"],
                "margins": {k: c["margin_diagnostics"][k] for k in
                    ("phase_margin_deg", "gain_margin_db", "passed", "status", "stress_scan_points_per_direction", "resolution_reserve")},
                "critical_unit_pole_max_distance": max(row["unit_pole_distance"] for kind in ("gain", "phase")
                    for row in c["margin_diagnostics"][kind+"_crossings"])} for c in representative],
            "margin_definition": "nearest bidirectional common plant-gain/phase stability boundary of periodic monodromy",
            "margin_requirements": {"phase_deg": 50., "gain_db": 6.},
            "test_count": 17, "physical_qualified": False, "native_internal_state_jacobian_verified": False,
            "clock_contract": "gyro source_age=availability_age+model gyro_delay; observer uses source timestamp; signal delayed once; preinitial sources skipped",
            "previous_evidence": "before-source-clock; prior gyro-delay cases used availability timestamp and are superseded",
            "limits": ["periodic 200Hz controller, fresh 50Hz gyro; linear continuous unquantized measurements",
                "same-tick successful ACK; inactive limits/AW; local sliding only",
                "Stribeck startup unchanged; bounded 3800-tick warm-up, preceding native oracle state carried only for zero delays",
                "finite-prefix native observables; hidden covariance/old-error states not set or independently differentiated",
                "finite-grid common gain/phase stress boundaries; not physical/global/irregular-timing qualification",
                "dynamic/delayed cases use frozen FF command offset; no dynamic FF inversion",
                "nonlinear quantization, START/reversal and manoeuvre forecasts remain separate gates"]}
        (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        print(json.dumps({"accepted_cases": len(cases), "max_error": summary["max_accepted_error"], "retained_failures": len(retained)}))
    else:
        print(json.dumps(integrated_family_case(args.library, args.family_library or args.library,
            1, affine=True) if args.integrated else first_probe(args.library), indent=2))
