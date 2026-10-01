"""Coupled supplied-parent / two-native-owner runtime probe; no station access."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.assembly_dynamics import ActuatorMap, RigidBody
from Firmware.commissioning.assembly_forecast import CausalParent, CoupledControllerPair, JointReference, JointSupport
from Firmware.commissioning.native import CObservation, CParameters, CReference, Controller, Native
from Firmware.tools.adr0022_assembly_probe import fixture, serializable, zero_load
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters


def support_fixture(duration_s):
    return JointSupport(q_min_rad=np.full(2, -1.), q_max_rad=np.full(2, 1.),
        velocity_max_rad_s=np.full(2, .8), acceleration_max_rad_s2=np.full(2, np.deg2rad(30.)),
        max_reference_source_age_s=duration_s, qualification="SYNTHETIC_OFFLINE")


def native_fixture(assembly, initial_q, actuator=None):
    """Copied benchmark gains/observer/guards, explicit frictionless nominal model."""
    inertia = assembly.dynamics(initial_q, np.zeros(2)).M_kg_m2
    if actuator is None:
        actuator = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    zero_torque_current = actuator.command_for_torque(np.zeros(2))[1]
    result = []
    for index in range(2):
        params = CParameters.from_buffer_copy(controller_parameters())
        params.model.z[:] = (-2., 0., 2.)  # numerical posture nodes for this supplied geometry only
        for node in range(3):
            params.model.theta[node] = inertia[index, index]/(
                actuator.torque_per_effective_amp_Nm[index]*actuator.command_gain[index])
            params.model.theta[3+node] = 0.
        for entry in range(30):
            params.model.theta[6+entry] = zero_torque_current[index]
            params.start_total[entry] = zero_torque_current[index]  # zero torque, no friction kick
            params.start_censored[entry] = 0
        params.model.theta[36] = 0.
        result.append(params)
    return tuple(result)


def vector_plan(now, initial_q, *, move_s, duration_s, configuration_id):
    departure = .2
    displacement = np.array([.07, -.04])
    phase = np.clip((now-departure)/move_s, 0., 1.)
    s = 10*phase**3-15*phase**4+6*phase**5
    ds = (30*phase**2-60*phase**3+30*phase**4)/move_s
    dds = (60*phase-180*phase**2+120*phase**3)/move_s**2
    return JointReference(q_ref_rad=initial_q+displacement*s, v_ref_rad_s=displacement*ds,
        a_ref_rad_s2=displacement*dds, time_s=now, source_time_s=0., expires_at_s=duration_s,
        frame="logical-output-joint-rad", configuration_id=configuration_id,
        trajectory_id="simultaneous-quintic-two-joint-displacement", generation=1, fresh=True, valid=True)


def filter_oracle_probe():
    """Independent exact filter solution with a constant coupled inertia fixture."""
    body = RigidBody(mass_kg=1., com_in_body_m=np.zeros(3), inertia_com_body_kg_m2=np.eye(3)*.1,
                    mount_translation_m=np.zeros(3), mount_rotation=np.eye(3))
    assembly = replace(fixture(), yaw_carriage=body,
        pitch_payload=replace(body, inertia_com_body_kg_m2=np.eye(3)*.2),
        pitch_axis_in_yaw=np.array([0., 0., 1.]), pitch_origin_in_yaw_m=np.zeros(3),
        gravity_world_m_s2=np.zeros(3))
    actuator = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    tau, delay = .012, .0073
    parent = CausalParent(assembly, actuator, np.zeros(4), zero_load(assembly), max_step_s=.0025,
                         transport_delay_s=delay, gyro_filter_tau_s=tau, max_load_age_s=0., prehistory_command_A=np.zeros(2))
    accepted_t = np.array([0., .065, .110])
    commands = np.array([[.03, 0.], [-.01, .02], [.015, -.005]])
    # This independent known inverse is for M=[[.3,.2],[.2,.2]].
    accelerations = commands@np.array([[10., -10.], [-10., 15.]])
    delta_acceleration = np.diff(np.vstack((np.zeros(2), accelerations)), axis=0)
    effective_t = accepted_t+delay
    parent.accept(0., commands[0])
    for index in (1, 2):
        parent.advance(float(accepted_t[index]))
        parent.accept(float(accepted_t[index]), commands[index])
    parent.advance(.2)

    def oracle(at):
        elapsed = np.maximum(0., at-effective_t)
        velocity = elapsed@delta_acceleration
        filtered = (elapsed-tau*(-np.expm1(-elapsed/tau)))@delta_acceleration
        return velocity, filtered

    knots = np.asarray(parent.times)
    actual_filter = np.asarray(parent.filtered)
    exact = np.array([oracle(t)[1] for t in knots])
    knot_error = float(np.max(np.abs(actual_filter-exact)))
    velocity_error = float(np.max(np.abs(np.asarray(parent.states)[:, 2:]-np.array([oracle(t)[0] for t in knots]))))
    breakpoints_present = all(np.min(np.abs(knots-event)) <= 1e-15 for event in effective_t)
    sensor_errors, sensor_bounds, queries = [], [], []
    for available_s in np.arange(.005, .201, .020):
        source = float(available_s-.004)
        right = min(int(np.searchsorted(knots, source, side="right")), len(knots)-1)
        left = max(0, right-1)
        begin, end = knots[left], knots[right]
        # The sample uses only the available prefix. For this known common tau,
        # f'' is an exponential between acceleration events, so the largest
        # absolute curvature occurs at an endpoint or transport event.
        candidates = [begin, end] + [float(x) for x in effective_t if begin <= x <= end]
        curvature = np.zeros(2)
        for at in candidates:
            for side in ("left", "right"):
                active = effective_t < at if side == "left" else effective_t <= at
                value = ((np.exp(-(at-effective_t[active])/tau)/tau)@delta_acceleration[active]
                         if active.any() else np.zeros(2))
                curvature = np.maximum(curvature, np.abs(value))
        bound = float(np.max(curvature)*(end-begin)**2/8)
        error = float(np.max(np.abs(parent.gyro_at(source)-oracle(source)[1])))
        sensor_errors.append(error)
        sensor_bounds.append(bound)
        queries.append(dict(availability_s=float(available_s), source_s=source, bracket_s=[float(begin), float(end)],
                            analytic_interpolation_bound_rad_s=bound, observed_error_rad_s=error))
        if end > available_s+1e-15 or error > bound+1e-12:
            raise AssertionError("nonaligned gyro interpolation exceeded its analytic causal bound")
    return dict(oracle="constant coupled inertia; analytic piecewise-ramp first-order filter",
        transport_delay_s=delay, gyro_filter_tau_s=tau, accepted_event_times_s=accepted_t.tolist(),
        effective_event_times_s=effective_t.tolist(), retained_filter_knots=len(knots),
        supplied_step_s=.0025, successful_commands_A=commands.tolist(),
        independent_inverse_inertia=[[10., -10.], [-10., 15.]], retained_filter_times_s=knots.tolist(),
        bound_formula="max_abs_filter_second_derivative_on_bracket * bracket_width_s**2 / 8",
        delayed_sensor_queries=queries,
        all_transport_breakpoints_retained=bool(breakpoints_present),
        maximum_filter_knot_error=knot_error, maximum_velocity_error=velocity_error,
        maximum_delayed_interpolation_error=max(sensor_errors), maximum_analytic_interpolation_bound=max(sensor_bounds),
        all_delayed_sensor_reads_within_causal_bound=True,
        passed=bool(knot_error < 1e-12 and velocity_error < 1e-12 and breakpoints_present))


def fault_probe(native, case):
    """Execute real native faults; no ACK or parent advance follows a pair fault."""
    assembly = replace(fixture(), gravity_world_m_s2=np.zeros(3))
    actuator = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    initial = np.array([.3, .4])
    params = native_fixture(assembly, initial)
    with Controller(native, params[0]) as yaw, Controller(native, params[1]) as pitch:
        cores = (yaw, pitch)
        for axis, core in enumerate(cores):
            core.reset(1., initial[axis], 0., 0., accepted_time=.9)
        pair = CoupledControllerPair(cores, assembly, actuator, core_epoch_s=1.,
            frame="logical-output-joint-rad", trajectory_id="simultaneous-quintic-two-joint-displacement",
            max_load_age_s=0., support=support_fixture(3.))
        ref = vector_plan(.005, initial, move_s=2., duration_s=3., configuration_id=assembly.configuration_id)
        observations = [CObservation(1.005, 1.005, 1.005, initial[k], 0., 1, 1, 1, 1, 1) for k in range(2)]
        load = zero_load(assembly)(.005, initial, np.zeros(2))
        if case == "pitch-native-nonfinite":
            observations[1].gyro_rate = float("nan")
        elif case == "yaw-native-stale":
            observations[0].encoder_time = observations[1].encoder_time = .9
        elif case == "paired-source-clock":
            observations[1].gyro_time = 1.004
        elif case == "invalid-load":
            load = replace(load, valid=False)
        elif case == "reference-context":
            ref = replace(ref, configuration_id="wrong-configuration")
        elif case == "reference-source-age":
            ref = replace(ref, source_time_s=-3.1)
        elif case == "reference-velocity-envelope":
            ref = replace(ref, v_ref_rad_s=np.array([.81, 0.]))
        elif case == "reference-position-envelope":
            ref = replace(ref, q_ref_rad=np.array([1.1, .4]))
        elif case == "reference-acceleration-envelope":
            ref = replace(ref, a_ref_rad_s2=np.array([.6, 0.]))
        else:
            raise ValueError("unknown native pair fault case")
        try:
            pair.step(observations, ref, load)
        except Exception as exc:
            detail = str(exc)
        else:
            raise AssertionError("injected pair fault unexpectedly succeeded")
        postfault = [core.step(CObservation(1.010, 1.010, 1.010, initial[k], 0., 2, 2, 1, 1, 1),
                              CReference(initial[k], 0., 0., initial[1-k])) for k, core in enumerate(cores)]
        statuses = [out.status for out in postfault]
        tokens = [out.sequence for out in postfault]
        no_ack = all("ACK" not in event for event in pair.trace)
        return dict(case=case, observed_detail=detail, postfault_native_status=statuses,
                    postfault_command_tokens=tokens, pair_fault_latched=pair.faulted,
                    synthetic_current_accepted=False, postfault_parent_advanced=False,
                    no_ACK_after_partial_output=no_ack,
                    passed=pair.faulted and statuses == [5, 5] and tokens == [0, 0] and no_ack)


def partial_ack_probe(native):
    assembly = replace(fixture(), gravity_world_m_s2=np.zeros(3))
    actuator = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    initial = np.array([.3, .4, 0., 0.])
    loads = zero_load(assembly)
    parent = CausalParent(assembly, actuator, initial, loads, max_step_s=.0025,
        transport_delay_s=0., gyro_filter_tau_s=0., max_load_age_s=0., prehistory_command_A=np.zeros(2))
    parent.advance(.005)
    params = native_fixture(assembly, initial[:2])
    with Controller(native, params[0]) as yaw, Controller(native, params[1]) as pitch:
        for axis, core in enumerate((yaw, pitch)):
            core.reset(1., initial[axis], 0., 0., accepted_time=.9)
        pair = CoupledControllerPair((yaw, pitch), assembly, actuator, core_epoch_s=1.,
            frame="logical-output-joint-rad", trajectory_id="partial-accepted-command-discriminator",
            max_load_age_s=0., support=support_fixture(.1))
        ref = JointReference(q_ref_rad=initial[:2], v_ref_rad_s=np.array([.04, -.03]),
            a_ref_rad_s2=np.array([.05, -.04]), time_s=.005, source_time_s=0., expires_at_s=.1,
            frame=pair.frame, configuration_id=assembly.configuration_id, trajectory_id=pair.trajectory,
            generation=1, fresh=True, valid=True)
        observations = tuple(CObservation(1.005, 1.005, 1.005, initial[k], 0., 1, 1, 1, 1, 1) for k in range(2))
        result = pair.step(observations, ref, loads(.005, initial[:2], np.zeros(2)))
        result.outputs[1].sequence += 1  # real native ACK rejects this unmatched token
        try:
            pair.acknowledge(result, record_receipt=parent.accept_receipt)
        except Exception as exc:
            detail = str(exc)
        else:
            raise AssertionError("wrong-token pitch ACK unexpectedly succeeded")
        retained = parent.commands[-1].copy()
        before = parent.state.copy()
        after = parent.advance(.010)
        return dict(case="first-native-ACK-success-second-native-ACK-rejection", detail=detail,
            successful_receipts=[asdict(r) for r in pair.receipts], retained_input_A=retained.tolist(),
            pair_fault_latched=pair.faulted, unaccepted_pitch_retains_prior_input=bool(retained[1] == 0.),
            conditional_postfault_velocity_delta_rad_s=(after[2:]-before[2:]).tolist(),
            postfault_prediction="offline conditional on retained accepted yaw input; physical stopping NOT_QUALIFIED",
            passed=bool(pair.faulted and len(pair.receipts) == 1 and pair.receipts[0].axis == 0
                        and retained[0] > 0 and retained[1] == 0. and np.any(after[2:] != before[2:])))


def run(native, output_dir, *, move_s=2., step_s=.0025, timing_case="pristine", actuator_case="identity"):
    output_dir.mkdir(parents=True, exist_ok=False)
    assembly = replace(fixture(), gravity_world_m_s2=np.zeros(3))
    if actuator_case == "identity":
        actuator = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    elif actuator_case == "nonunit-bias":
        actuator = ActuatorMap(torque_per_effective_amp_Nm=np.array([.5, .4]),
            command_gain=np.array([2., 1.5]), command_bias_A=np.array([.01, -.02]))
    else:
        raise ValueError("explicit positive synthetic actuator case required")
    loads = zero_load(assembly)
    initial = np.array([.3, .4, 0., 0.])
    params = native_fixture(assembly, initial[:2], actuator)
    prehistory_command = actuator.command_for_torque(np.zeros(2))[1]
    if np.max(np.abs(actuator.torque_for_command(prehistory_command))) > 1e-12:
        raise AssertionError("supplied current prehistory must match zero initial acceleration/load")
    dt, duration, epoch = .005, .2+move_s+.8, 1.
    if timing_case not in ("pristine", "causal-filter-delay", "causal-offmesh"):
        raise ValueError("explicit known timing case required")
    delayed = timing_case != "pristine"
    transport_delay, gyro_tau, gyro_delay, gyro_period = (.0075, .012, .004, 4) if delayed else (0., 0., 0., 1)
    if timing_case == "causal-offmesh":
        transport_delay, gyro_delay = .0073, .0041
    times = np.arange(round(duration/dt)+1)*dt
    times[-1] = duration
    contract = dict(schema="adr0022.coupled-native-parent-probe/1", qualification="SYNTHETIC_OFFLINE_ONLY",
        parent=serializable(asdict(assembly)), actuator=serializable(asdict(actuator)), initial_q_v=initial.tolist(),
        native_current_domain="actual host command amperes; torque map only in callback and parent",
        prehistory_host_command_A=prehistory_command.tolist(), native_reset_applied_A=prehistory_command.tolist(),
        torque_per_host_command_amp_Nm=(actuator.torque_per_effective_amp_Nm*actuator.command_gain).tolist(),
        support=serializable(asdict(support_fixture(duration))),
        load_torque_Nm=[0., 0.], source_clock="common local monotonic fixture", core_epoch_s=epoch,
        command_acceptance="synthetic acceptance at reference time; both outputs before any ACK",
        control_dt_s=dt, numerical_step_s=step_s, sensor_filter_tau_s=gyro_tau, sensor_delay_s=gyro_delay,
        gyro_period_cycles=gyro_period, current_transport_delay_s=transport_delay,
        delay_policy="known supplied forward transport; no feedforward delay inversion",
        noise="NONE", move_s=move_s, duration_s=duration,
        vector_reference=dict(rule="simultaneous quintic position/velocity/acceleration, 0.2s initial hold",
                              displacement_rad=[.07, -.04], source_time_s=0., expires_at_s=duration),
        native_limits=[dict(current_cap_A=p.current_cap, slew_A_s=p.slew, start_timeout_s=p.start_timeout_s,
            gains=[p.kp, p.ki, p.kpos, p.kaw],
            observer={name: getattr(p.observer, name) for name, _ in p.observer._fields_},
            dt_min_s=p.dt_min, dt_max_s=p.dt_max, rest_speed_rad_s=p.rest_speed,
            sustained_s=p.sustained_s, intent_threshold_rad_s=p.intent_threshold,
            velocity_cap_rad_s=p.velocity_cap, integral_cap_A=p.integral_cap,
            start_total_A=list(p.start_total[:30]),
            model_role="explicit frictionless diagonal and zero-torque command baseline; no identification") for p in params],
        physical_identification="NOT_RUN", production_adoption="NOT_RUN", physical_qualification="NOT_RUN",
        acceptance=dict(no_fault=True, command_cap_A=.35, slew_A_s=2., paired_callback_order=True,
            causal_replay_max_rad=1e-12, step_refinement_max_state_error=1e-8,
            filtered_gyro_refinement_max_rad_s=1e-6, nonzero_M01=True))
    (output_dir/"contract.json").write_text(json.dumps(contract, indent=2)+"\n", encoding="utf-8")
    # Validate the entire frozen planned domain before observing any outputs.
    plan = [vector_plan(float(t), initial[:2], move_s=move_s, duration_s=duration,
                        configuration_id=assembly.configuration_id) for t in times]
    for p in params:
        if not all(p.model.q[0] <= q <= p.model.q[p.model.n-1] and p.model.z[0] <= q <= p.model.z[2]
                   for packet in plan for q in packet.q_ref_rad):
            raise ValueError("frozen vector reference exceeds supplied numerical domain")
    parent = CausalParent(assembly, actuator, initial, loads, max_step_s=step_s,
                         transport_delay_s=transport_delay, gyro_filter_tau_s=gyro_tau, max_load_age_s=0.,
                         prehistory_command_A=prehistory_command)
    state = initial.copy()
    states, accepted, references, posterior_rows, command_rows, intervals = [state.copy()], [prehistory_command.copy()], [], [], [], []
    ff_rows, event_orders, native_motion, sensor_rows = [], [], [], []
    outcome = dict(status="COMPLETED", time_s=duration)
    with Controller(native, params[0]) as yaw, Controller(native, params[1]) as pitch:
        cores = (yaw, pitch)
        for k, core in enumerate(cores):
            core.reset(epoch, initial[k], initial[k+2], float(prehistory_command[k]), generation=1, accepted_time=epoch-.1)
        pair = CoupledControllerPair(cores, assembly, actuator, core_epoch_s=epoch,
            frame="logical-output-joint-rad", trajectory_id="simultaneous-quintic-two-joint-displacement",
            max_load_age_s=0., support=support_fixture(duration))
        gyro_source, gyro, gyro_sequence = 0., initial[2:].copy(), 0
        for index, now in enumerate(times[1:], 1):
            # The interval's command was already jointly accepted last cycle.
            intervals.append(accepted[-1].copy())
            state = parent.advance(float(now))
            states.append(state.copy())
            ref = vector_plan(float(now), initial[:2], move_s=move_s, duration_s=duration,
                              configuration_id=assembly.configuration_id)
            fresh_gyro = index % gyro_period == 0
            if fresh_gyro:
                gyro_source = float(now)-gyro_delay
                gyro = parent.gyro_at(gyro_source)
                gyro_sequence += 1
            observations = tuple(CObservation(now+epoch, now+epoch, gyro_source+epoch, state[k], gyro[k],
                index, gyro_sequence, 1, 1, int(fresh_gyro)) for k in range(2))
            try:
                before = len(pair.trace)
                prior_receipts = len(pair.receipts)
                result = pair.step(observations, ref, loads(now, state[:2], state[2:]))
                command = pair.acknowledge(result, record_receipt=parent.accept_receipt)
                event_orders.append(pair.trace[before:])
            except Exception as exc:
                accepted_after_failure = len(pair.receipts)-prior_receipts
                outcome = dict(status="PAIRED_FAULT", time_s=float(now), detail=str(exc),
                               both_inhibited=pair.faulted, accepted_fault_command=bool(accepted_after_failure),
                               successful_per_axis_receipts_before_fault=accepted_after_failure,
                               retained_input_A=parent.commands[-1].tolist())
                break
            accepted.append(command.copy())
            sensor_rows.append(np.r_[now, state[:2], gyro, gyro_source, int(fresh_gyro)])
            references.append(np.r_[ref.q_ref_rad, ref.v_ref_rad_s, ref.a_ref_rad_s2])
            posterior_rows.append(np.r_[[p.position for p in result.posteriors], [p.velocity for p in result.posteriors]])
            ff_rows.append(result.demand["command_current_A"].copy())
            command_rows.append([[o.requested, o.limited, o.feedforward, o.integral, o.sequence, o.status,
                                  o.motion, o.start_increment] for o in result.outputs])
            native_motion.append([o.motion for o in result.outputs])
    t = times[:len(states)]
    states, intervals = np.asarray(states), np.asarray(intervals)
    def replay_parent(integration_step):
        repeated = CausalParent(assembly, actuator, initial, loads, max_step_s=integration_step,
            transport_delay_s=transport_delay, gyro_filter_tau_s=gyro_tau, max_load_age_s=0.,
            prehistory_command_A=prehistory_command)
        rows = [initial.copy()]
        # Conditional input-driven replay contains every actual accepted event,
        # including a partial receipt at fault time. Future events are read only
        # when their transport-shifted timestamp is reached; no state injection.
        repeated.tx_times = parent.tx_times.copy()
        repeated.commands = [value.copy() for value in parent.commands]
        for index, now in enumerate(t[1:], 1):
            rows.append(repeated.advance(float(now)))
        return np.asarray(rows), repeated
    accepted = np.asarray(accepted)
    replay, replay_parent_state = replay_parent(step_s)
    fine, fine_parent = replay_parent(step_s/2)
    causal_error = float(np.max(np.abs(replay-states)))
    refinement = float(np.max(np.abs(fine-replay)))
    sensor_error = max((float(np.max(np.abs(replay_parent_state.gyro_at(float(row[5]))-row[3:5])))
                        for row in sensor_rows), default=0.)
    filter_refinement = max((float(np.max(np.abs(fine_parent.gyro_at(float(row[5]))-row[3:5])))
                             for row in sensor_rows), default=0.)
    maximum_current = float(np.max(np.abs(accepted)))
    maximum_slew = float(np.max(np.abs(np.diff(accepted, axis=0))/dt)) if len(accepted)>1 else 0.
    coupling = np.asarray([assembly.dynamics(row[:2], row[2:]).M_kg_m2[0, 1] for row in states])
    correct_order = all(row == ["VECTOR_FF", "PITCH_OUTPUT", "YAW_OUTPUT", "YAW_ACK", "PITCH_ACK"] for row in event_orders)
    report = dict(outcome=outcome, lower_callback_probe="NESTED_DISTINCT_NATIVE_HANDLES",
        same_cycle_vector_FF_calls=len(ff_rows), paired_accepted_commands=len(accepted)-1,
        native_library=str(native.path), paired_output_before_ACK=correct_order,
        causal_prefix_replay_max_error=causal_error, causal_gyro_replay_max_error=sensor_error,
        identical_history_step_refinement_max_error=refinement,
        identical_history_filtered_gyro_refinement_max_error=filter_refinement,
        maximum_successful_current_A=maximum_current, maximum_successful_slew_A_s=maximum_slew,
        successful_per_axis_receipts=len(pair.receipts),
        receipt_host_effective_map_max_error=max((abs(r.effective_current_A-(
            actuator.command_gain[r.axis]*r.command_current_A+actuator.command_bias_A[r.axis])) for r in pair.receipts), default=0.),
        M01_min_kg_m2=float(coupling.min()), M01_max_kg_m2=float(coupling.max()),
        reference_domain="[-2,2] absolute q and [-2,2] numerical posture nodes; complete reference checked",
        state_span_rad=np.ptp(states[:, :2], axis=0).tolist(),
        passed=bool(outcome["status"] == "COMPLETED" and correct_order and causal_error <= 1e-12 and sensor_error <= 1e-12
            and refinement <= 1e-8 and filter_refinement <= 1e-6
            and maximum_current <= .35 and maximum_slew <= 2.+1e-10
            and np.min(np.abs(coupling)) > 1e-4),
        scope="lower callback + supplied parent numerical forecast; no model/controller qualification",
        full_motion_predicates="NOT_RUN", production_adoption="NOT_RUN", physical_qualification="NOT_RUN")
    np.savez_compressed(output_dir/"trace.npz", t_s=t, parent_q_v=states, accepted_A=accepted,
        interval_commands_A=intervals, references=np.asarray(references), posterior_q_v=np.asarray(posterior_rows),
        vector_ff_A=np.asarray(ff_rows), native_outputs=np.asarray(command_rows), replay_q_v=replay,
        refined_replay_q_v=fine, M01=coupling, control_sensors=np.asarray(sensor_rows),
        parent_native_times_s=np.asarray(parent.times), parent_filtered_gyro=np.asarray(parent.filtered),
        successful_event_t_s=np.asarray(parent.tx_times), successful_event_command_A=np.asarray(parent.commands),
        per_axis_receipts=np.asarray([[r.axis, r.accepted_time_s, r.command_current_A, r.effective_current_A,
                                      r.torque_Nm, r.command_token, r.generation] for r in pair.receipts]))
    (output_dir/"result.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--move-s", type=float, default=2.)
    parser.add_argument("--max-step-s", type=float, default=.0025)
    parser.add_argument("--actuator-case", choices=("identity", "nonunit-bias"), default="identity")
    parser.add_argument("--timing-case", choices=("pristine", "causal-filter-delay", "causal-offmesh"), default="pristine")
    parser.add_argument("--fault-probes", action="store_true")
    args = parser.parse_args()
    native = Native(args.library)
    if args.fault_probes:
        args.output_dir.mkdir(parents=True, exist_ok=False)
        cases = ("pitch-native-nonfinite", "yaw-native-stale", "paired-source-clock", "invalid-load", "reference-context",
                 "reference-source-age", "reference-position-envelope", "reference-velocity-envelope", "reference-acceleration-envelope")
        report = dict(execution="SYNTHETIC_LOCAL_ONLY", native_faults=[fault_probe(native, case) for case in cases],
                      partial_native_ACK=partial_ack_probe(native), independent_filter_oracle=filter_oracle_probe(),
                      physical_protection="NOT_QUALIFIED")
        (args.output_dir/"result.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    else:
        run(native, args.output_dir, move_s=args.move_s, step_s=args.max_step_s,
            timing_case=args.timing_case, actuator_case=args.actuator_case)


if __name__ == "__main__":
    main()
