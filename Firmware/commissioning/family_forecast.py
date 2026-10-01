"""Synthetic closed-loop family forecast through the existing native controller.

The core owns its observer, reference corrections, PI, limiter and successful
command accounting. The optional motor adapter evaluates its single FF term
from that same observer posterior. This runner supplies simulated sensors;
it does not qualify controller gains.
"""
from dataclasses import asdict, dataclass, replace
import math

import numpy as np

from .model_family import FamilyModel
from .motor_feedforward import (CausalState, FeedforwardRejected, FeedforwardSupport,
                                MotorFeedforward, ReferencePacket)
from .native import CObservation, CParameters, CReference, Controller


class ForecastRejected(ValueError):
    pass


class IndependentForecastPlant:
    """Independent oracle plant for feedback-generated synthetic observations."""
    provenance = "INDEPENDENT_ORACLE"

    def rollout(self, model, t, tx_t, tx_A, initial):
        from .synthetic_family_oracle import independent_rollout
        return independent_rollout(model, t, tx_t, tx_A, initial).trace


def require(test, detail):
    if not test:
        raise ForecastRejected(detail)


@dataclass(frozen=True, kw_only=True)
class ForecastContract:
    duration_s: float
    sample_dt_s: float
    control_period_samples: int
    gyro_period_samples: int
    encoder_noise_rad: float
    encoder_quantum_rad: float
    gyro_noise_rad_s: float
    current_noise_A: float
    seed: int
    configuration_id: str
    trajectory_id: str
    frame: str
    qualification: str = "SYNTHETIC_OFFLINE"

    def validate(self):
        require(self.qualification == "SYNTHETIC_OFFLINE", "synthetic-only forecast required")
        floats = (self.duration_s, self.sample_dt_s, self.encoder_noise_rad,
                  self.encoder_quantum_rad, self.gyro_noise_rad_s, self.current_noise_A)
        require(all(isinstance(x, (int, float)) and not isinstance(x, bool)
                    and math.isfinite(x) for x in floats), "finite forecast sampling/noise required")
        require(self.duration_s > 0 and self.sample_dt_s > 0 and min(floats[2:]) >= 0,
                "positive duration/sample period and nonnegative sensor noise required")
        for value in (self.control_period_samples, self.gyro_period_samples):
            require(type(value) is int and value > 0, "integer native sample periods required")
        require(type(self.seed) is int and self.seed >= 0, "explicit nonnegative noise seed required")
        require(all(type(x) is str and x for x in (self.configuration_id, self.trajectory_id, self.frame)),
                "explicit configuration, trajectory and frame required")
        samples = self.duration_s / self.sample_dt_s
        require(abs(samples - round(samples)) < 1e-9 and round(samples) >= self.control_period_samples,
                "duration must contain whole samples and a control cycle")
        return self


def _reference(packet, now, contract):
    require(isinstance(packet, ReferencePacket), "one authoritative reference packet required")
    require(packet.valid is True and packet.fresh is True and type(packet.generation) is int
            and packet.generation == 1, "valid fresh reference in the simulated generation required")
    require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
            for x in (packet.q_ref_rad, packet.v_ref_rad_s, packet.a_ref_rad_s2,
            packet.time_s, packet.source_time_s, packet.expires_at_s)), "finite reference required")
    require(packet.time_s == now and packet.source_time_s <= now <= packet.expires_at_s,
            "reference time, source causality or expiry invalid")
    require((packet.configuration_id, packet.trajectory_id, packet.frame)
            == (contract.configuration_id, contract.trajectory_id, contract.frame),
            "reference context changed")
    return CReference(packet.q_ref_rad, packet.v_ref_rad_s, packet.a_ref_rad_s2, 0.)


def forecast(native, plant_native, model, parameters, contract, reference, *, initial,
             prehistory_time_s=-.1, prehistory_command_A=0.,
             feedforward_support=None, feedforward_state=None, send=None, plant_model=None):
    """Generate future successful TX using simulated native sensors only.

    Each plant prefix is replayed from the same acquisition state and causal
    accepted history. No prefix is reinitialized from measured or future state.
    Prefix replay is an offline implementation choice, not a real-time claim.
    Current follows the actual accepted-event timestamp; the default receipt
    occurs on the control tick. q/gyro are continuous at that event and are
    checked against the final complete rollout.
    A core fault ends this forecast. Holding its last command is not labelled
    a physically safe stop and no automatic rearm is attempted.
    An optional separate plant supplies all simulated motion and sensor-source
    delays. The selected model remains the controller's feedforward model;
    simulator parameters are never injected into its adapter or reference.
    """
    contract.validate()
    require(isinstance(model, FamilyModel), "explicit selected family required")
    model.validate()
    separate_plant = plant_model is not None
    plant_model = model if plant_model is None else plant_model
    require(isinstance(plant_model, FamilyModel), "explicit compatible synthetic plant family required")
    plant_model.validate()
    require(isinstance(parameters, CParameters), "existing core parameters required")
    require(callable(reference), "declared reference producer required")
    require(send is None or callable(send), "optional simulated receipt callback must be callable")
    initial = np.asarray(initial, dtype=float)
    require(initial.shape == (5,) and np.isfinite(initial).all(), "one declared initial state required")
    require(math.isfinite(prehistory_time_s) and prehistory_time_s < 0
            and math.isfinite(prehistory_command_A), "causal finite TX prehistory required")
    dt = contract.sample_dt_s * contract.control_period_samples
    require(parameters.dt_min <= dt <= parameters.dt_max, "control sample period outside core support")
    require(abs(prehistory_command_A) <= parameters.current_cap, "prehistory exceeds supplied core cap")
    use_motor_ff = feedforward_support is not None or feedforward_state is not None
    if use_motor_ff:
        require(isinstance(feedforward_support, FeedforwardSupport)
                and isinstance(feedforward_state, CausalState), "motor FF needs explicit support and context")
        require((feedforward_support.configuration_id, feedforward_support.frame)
                == (contract.configuration_id, contract.frame), "motor FF forecast context mismatch")
        require(feedforward_state.time_s == 0. and feedforward_state.generation == 1
                and feedforward_state.q_rad == initial[0] and feedforward_state.v_rad_s == initial[1],
                "motor FF context must share the acquisition initial state")
    n = int(round(contract.duration_s / contract.sample_dt_s)) + 1
    t = np.arange(n) * contract.sample_dt_s
    # Preserve the declared expiry endpoint when floating multiplication lands
    # one ULP above it. This does not extend the reference's validity window.
    t[-1] = float(contract.duration_s)
    rng = np.random.default_rng(contract.seed)
    q_noise = rng.normal(0., contract.encoder_noise_rad, n)
    current_noise = rng.normal(0., contract.current_noise_A, n)
    gyro_noise = rng.normal(0., contract.gyro_noise_rad_s,
                            (n - 1) // contract.gyro_period_samples + 1)

    def encoder(value, k):
        value = value + q_noise[k]
        return (float(np.round(value / contract.encoder_quantum_rad) * contract.encoder_quantum_rad)
                if contract.encoder_quantum_rad else float(value))

    tx_t, tx_A = [float(prehistory_time_s)], [float(prehistory_command_A)]
    # The core's observation clock is a nonnegative monotonic clock. Local
    # acquisition t=0 can contain older source samples/prehistory: translate
    # every core timestamp by the same epoch, without changing any age/delay.
    core_epoch = max(0., -prehistory_time_s, plant_model.gyro_delay) + 1.
    commands, sensors, references, control_gyro_valid = [], [], [], []
    ff_demands, ff_posteriors, ff_phases = [], [], []
    outcome = {"status": "COMPLETED", "time_s": contract.duration_s}
    program_report = None
    with Controller(native, parameters) as core:
        initial_trace = plant_native.rollout(plant_model, t[:2], np.asarray(tx_t), np.asarray(tx_A), initial)
        if use_motor_ff:
            if feedforward_support.planned_start_program is not None:
                feedforward_support = replace(feedforward_support,
                    planned_start_program=feedforward_support.planned_start_program.with_epoch(core_epoch))
            adapter = MotorFeedforward(core, model, feedforward_support)
            advisory = replace(feedforward_state, time_s=core_epoch,
                q_rad=encoder(initial_trace[0, 0], 0), v_rad_s=float(initial_trace[0, 1]))
            adapter.reset(advisory, now=core_epoch, previous_current_A=prehistory_command_A,
                          accepted_time_s=prehistory_time_s + core_epoch)
        else:
            core.reset(core_epoch, encoder(initial_trace[0, 0], 0), float(initial_trace[0, 1]), prehistory_command_A,
                       generation=1, accepted_time=prehistory_time_s + core_epoch)
        for k in range(contract.control_period_samples, n, contract.control_period_samples):
            now = float(t[k])
            prefix = plant_native.rollout(plant_model, t[:k+1], np.asarray(tx_t), np.asarray(tx_A), initial)
            gyro_index = k // contract.gyro_period_samples * contract.gyro_period_samples
            gyro = float(prefix[gyro_index, 3] + gyro_noise[gyro_index // contract.gyro_period_samples])
            q = encoder(prefix[k, 0], k)
            gyro_source_time = float(t[gyro_index] - plant_model.gyro_delay)
            # Reset installs the declared acquisition initial velocity at t=0.
            # An older delayed sample must not replace that newer information.
            # Wait for the first source sample at/after acquisition initialization;
            # the existing core's normal freshness deadline remains authoritative.
            gyro_valid = gyro_source_time >= 0.
            obs = CObservation(now + core_epoch, now + core_epoch, gyro_source_time + core_epoch, q, gyro,
                               k + 1, gyro_index + 1, 1, True, gyro_valid)
            try:
                packet = reference(now)
                ref = _reference(packet, now, contract)
            except ForecastRejected as exc:
                if use_motor_ff:
                    adapter.inhibit_at(now + core_epoch, str(exc))
                outcome = {"status": "INVALID_REFERENCE", "time_s": now, "detail": str(exc)}
                n = k + 1
                break
            try:
                if use_motor_ff:
                    shifted_packet = replace(packet, time_s=packet.time_s + core_epoch,
                        source_time_s=packet.source_time_s + core_epoch,
                        expires_at_s=packet.expires_at_s + core_epoch)
                    advisory = replace(advisory, time_s=now + core_epoch)
                    output, demand = adapter.step(obs, shifted_packet, advisory)
                    posterior = adapter.last_posterior
                    advisory = replace(advisory, q_rad=posterior.position, v_rad_s=posterior.velocity)
                    ff_demands.append([now, demand.effective_current_A, demand.command_current_A,
                                       demand.load_A, demand.friction_A, demand.direction])
                    ff_phases.append(demand.phase)
                    ff_posteriors.append([now, posterior.position, posterior.velocity,
                        posterior.encoder_time - core_epoch, posterior.gyro_time - core_epoch,
                        posterior.accepted_current, posterior.accepted_time - core_epoch])
                else:
                    output = core.step(obs, ref)
            except FeedforwardRejected as exc:
                outcome = {"status": "MOTOR_FF_FAULT", "time_s": now,
                           "reason": exc.reason.value, "detail": str(exc)}
                n = k + 1
                break
            references.append([now, ref.position, ref.velocity, ref.acceleration])
            sensors.append([now, q, gyro, gyro_source_time, prefix[k, 0], prefix[gyro_index, 3]])
            control_gyro_valid.append(gyro_valid)
            commands.append([now, output.requested, output.limited, output.feedforward,
                output.integral, output.position, output.velocity, output.sequence,
                output.status, output.motion, output.start_increment])
            if output.status not in (0, 3):
                outcome = {"status": "CORE_FAULT", "time_s": now, "core_status": int(output.status)}
                n = k + 1
                break
            receipt_time = now
            try:
                successful, accepted_time, applied = ((True, now, float(output.limited)) if send is None
                                                     else send(output, now))
                if (isinstance(accepted_time, (int, float)) and not isinstance(accepted_time, bool)
                        and math.isfinite(accepted_time) and now <= accepted_time <= contract.duration_s):
                    receipt_time = float(accepted_time)
                require(type(successful) is bool and math.isfinite(accepted_time) and math.isfinite(applied)
                        and now <= accepted_time <= min(now+dt, contract.duration_s)
                        and accepted_time > tx_t[-1],
                        "simulated receipt must precede the next observation and declared horizon")
                accepted = (adapter.acknowledge(output, successful=successful,
                                accepted_time_s=accepted_time + core_epoch, applied_current_A=applied)
                            if use_motor_ff else core.ack(output, successful=successful, applied=applied,
                                                         accepted_time=accepted_time + core_epoch))
            except FeedforwardRejected as exc:
                outcome = {"status": "MOTOR_FF_FAULT", "time_s": now,
                           "reason": exc.reason.value, "detail": str(exc)}
                if send is not None:
                    outcome["receipt_time_s"] = receipt_time
                n = k + 1
                break
            except (ValueError, TypeError, ArithmeticError) as exc:
                if use_motor_ff:
                    adapter.inhibit_at(receipt_time + core_epoch, str(exc))
                else:
                    core.inhibit()
                outcome = {"status": "INVALID_RECEIPT", "time_s": now, "detail": str(exc)}
                if send is not None:
                    outcome["receipt_time_s"] = receipt_time
                n = k + 1
                break
            if not accepted:
                outcome = {"status": "ACK_REJECTED", "time_s": now}
                if send is not None:
                    outcome["receipt_time_s"] = receipt_time
                n = k + 1
                break
            tx_t.append(float(accepted_time))
            tx_A.append(float(applied))
        if use_motor_ff and feedforward_support.planned_start_program is not None:
            if outcome["status"] == "COMPLETED":
                try:
                    adapter.account_start_dose_through(contract.duration_s + core_epoch)
                except FeedforwardRejected as exc:
                    outcome = {"status": "MOTOR_FF_FAULT", "time_s": contract.duration_s,
                               "reason": exc.reason.value, "detail": str(exc),
                               "stage": "FINAL_HELD_COMMAND_ACCOUNTING"}
            program_report = adapter.start_program_report()
            for key in ("accounted_through_s", "last_accepted_time_s"):
                if program_report[key] is not None:
                    program_report[key] -= core_epoch
            program_report["admissions"] = [{**entry, "time_s": entry["time_s"]-core_epoch}
                                             for entry in program_report["admissions"]]
            program_report["accepted_MOVE_times_s"] = {key: value-core_epoch
                for key, value in program_report["accepted_MOVE_times_s"].items()}
    t = t[:n]
    tx_t, tx_A = np.asarray(tx_t), np.asarray(tx_A)
    truth = plant_native.rollout(plant_model, t, tx_t, tx_A, initial)
    q = np.asarray([encoder(value, k) for k, value in enumerate(truth[:, 0])])
    gyro_indices = np.arange(n) // contract.gyro_period_samples * contract.gyro_period_samples
    v = truth[gyro_indices, 3] + gyro_noise[gyro_indices // contract.gyro_period_samples]
    current = truth[:, 4] + current_noise[:n]
    sensors = np.asarray(sensors, dtype=float).reshape(-1, 6)
    if len(sensors):
        indices = np.rint(sensors[:, 0] / contract.sample_dt_s).astype(int)
        causal_error = max(float(np.max(np.abs(q[indices] - sensors[:, 1]))),
                           float(np.max(np.abs(v[indices] - sensors[:, 2]))))
    else:
        causal_error = 0.
    successful_slew = np.abs(np.diff(tx_A)) / (dt if send is None else np.diff(tx_t))
    result = {"t": t, "q": q, "v": v, "current": current, "truth": truth,
        "q_new": np.ones(n, dtype=bool), "v_new": np.arange(n) % contract.gyro_period_samples == 0,
        "current_new": np.ones(n, dtype=bool), "tx_t": tx_t, "tx_A": tx_A,
        "initial": initial, "commands": np.asarray(commands, dtype=float).reshape(-1, 11),
        "references": np.asarray(references, dtype=float).reshape(-1, 4), "causal_sensors": sensors,
        "ff_demands": np.asarray(ff_demands, dtype=float).reshape(-1, 6),
        "ff_posteriors": np.asarray(ff_posteriors, dtype=float).reshape(-1, 7),
        "ff_phases": np.asarray(ff_phases, dtype="U32"),
        "control_gyro_valid": np.asarray(control_gyro_valid, dtype=bool),
        "report": {"outcome": outcome, "contract": asdict(contract),
            "controller_policy": ("SUPPLIED_EXISTING_NATIVE_CORE_WITH_SHARED_POSTERIOR_MOTOR_FF"
                                  if use_motor_ff else "SUPPLIED_EXISTING_NATIVE_CORE_WITH_LEGACY_FF"),
            "motor_ff_policy": (getattr(feedforward_support, "actuator_policy", "ALGEBRAIC_ZERO_DELAY")
                                if use_motor_ff else "NOT_RUN"),
            "motor_ff_phase_counts": {phase: ff_phases.count(phase) for phase in sorted(set(ff_phases))},
            "command_columns": ["time_s", "requested_A", "limited_A", "feedforward_A", "integral_A",
                                "posterior_q_rad", "posterior_v_rad_s", "sequence", "status", "motion",
                                "start_increment_A"],
            "ending_native_motion": int(commands[-1][9]) if commands else "NOT_RUN",
            "native_motion_names": {0: "REST", 1: "START", 2: "MOVE", 3: "STOP", 4: "REVERSE"},
            "ff_demand_columns": ["time_s", "effective_current_A", "command_current_A", "load_A",
                                  "friction_A", "direction"],
            "ff_posterior_columns": ["time_s", "q_rad", "v_rad_s", "encoder_source_time_s",
                                     "gyro_source_time_s", "accepted_current_A", "accepted_time_s"],
            "control_plant_backend": getattr(plant_native, "provenance", "NATIVE_FAMILY"),
            "future_realized_inputs_substituted": False, "future_measured_state_substituted": False,
            "plant_state_initializations": 1, "prefix_replay": "SAME_INITIAL_STATE_AND_CAUSAL_HISTORY",
            "current_sampling_order": ("AFTER_SAME_TIMESTAMP_ACCEPTED_COMMAND" if send is None
                else "ACTUAL_ACCEPTED_EVENT_TIMESTAMPS; PRECEDING_CURRENT_UNTIL_RECEIPT"),
            "gyro_timestamp_policy": ("SOURCE_SAMPLE_TIME=FRESH_AVAILABILITY_TIME-PLANT_GYRO_DELAY; KNOWN_SYNTHETIC_CLOCK"
                if separate_plant else "SOURCE_SAMPLE_TIME=FRESH_AVAILABILITY_TIME-MODEL_GYRO_DELAY; KNOWN_SYNTHETIC_CLOCK"),
            "initial_gyro_policy": "SUPPLIED_INITIAL_VELOCITY; PREACQUISITION_SOURCE_SAMPLES_NOT_INJECTED",
            "core_clock_epoch_translation_s": core_epoch,
            "causal_sensor_columns": ["availability_time_s", "encoder_q_rad", "gyro_rad_s",
                "gyro_source_time_s", "plant_q_rad", "plant_filtered_gyro_rad_s"],
            "causal_sensor_replay_max_error": causal_error,
            "maximum_successful_command_A": float(np.max(np.abs(tx_A))),
            "maximum_successful_slew_A_s": float(successful_slew.max()) if len(successful_slew) else 0.,
            "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}}
    if program_report is not None:
        result["report"]["planned_start_program"] = program_report
    if separate_plant:
        result["report"]["separate_synthetic_plant"] = {
            "controller_model": model.document(), "plant_model": plant_model.document(),
            "plant_parameters_injected_into_controller": False,
            "gyro_timestamp_policy": "SOURCE_SAMPLE_TIME=FRESH_AVAILABILITY_TIME-PLANT_GYRO_DELAY; KNOWN_SYNTHETIC_CLOCK"}
    if send is not None:
        receipt_end = outcome.get("receipt_time_s", float(t[-1]))
        result["report"]["recorded_plant_horizon_s"] = float(t[-1])
        result["report"]["known_prior_command_hold_after_trace"] = (
            {"start_s": float(t[-1]), "through_s": receipt_end, "command_A": float(tx_A[-1]),
             "plant_and_sensor_samples": "NOT_GENERATED; QUALITY_NOT_EVALUATED"}
            if receipt_end > t[-1] else None)
    return result


def synthetic_motion_metrics(data, contract, *, command_time, zero_reference_time, gyro_bandwidth_hz,
                             step_rad=None, position_jitter_limit_rad=None):
    """Apply frozen sensor-based metrics to a completed synthetic forecast.

    Offline gyro interpolation matches the recorded-data evaluator. The core
    consumed the separately retained causal held samples; interpolation never
    enters its observer. Numerical oracle agreement is a separate check.
    """
    from .metrics import motion_metrics
    contract.validate()
    t, commands = data["t"], data["commands"]
    require(data["report"]["outcome"]["status"] == "COMPLETED",
            "complete forecast required before frozen motion metrics")
    require(commands.ndim == 2 and commands.shape[1] == 11 and len(commands) > 1,
            "recorded current, startup and status columns required")
    require(np.array_equal(data["tx_t"][1:], commands[:, 0])
            and np.array_equal(data["tx_A"][1:], commands[:, 2]),
            "one successful same-tick command required for each evaluated output")
    require(math.isfinite(gyro_bandwidth_hz) and 0 < gyro_bandwidth_hz
            <= 1. / (contract.sample_dt_s * contract.gyro_period_samples * 5.),
            "synthetic gyro evaluation bandwidth must respect native sample support")
    indices = np.rint(commands[:, 0] / contract.sample_dt_s).astype(int)
    fresh = data["v_new"]
    gyro = np.interp(commands[:, 0], t[fresh], data["v"][fresh])
    trace = np.column_stack((data["q"][indices], gyro, commands[:, 5], commands[:, 6],
        commands[:, 1], commands[:, 2], commands[:, 4], commands[:, 3], commands[:, 10],
        commands[:, 9], commands[:, 8], data["tx_A"][1:]))
    refs = np.column_stack((data["references"][:, 1:], np.zeros(len(commands))))
    result = motion_metrics(commands[:, 0], refs, trace, command_time=command_time,
                            zero_reference_time=zero_reference_time, gyro_bandwidth_hz=gyro_bandwidth_hz,
                            step_rad=step_rad, position_jitter_limit_rad=position_jitter_limit_rad)
    return {"qualification": "SYNTHETIC_OFFLINE", "metrics": result,
            "sensor_evaluation": "OBSERVED_ENCODER_AND_OFFLINE_INTERPOLATED_NATIVE_GYRO",
            "causal_control_samples": "RETAINED_SEPARATELY; NO_OFFLINE_INTERPOLATION_IN_CONTROL",
            "startup_anchor_s": command_time, "zero_reference_anchor_s": zero_reference_time,
            "gyro_bandwidth_hz": gyro_bandwidth_hz,
            "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
