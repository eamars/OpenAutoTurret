"""Offline coupled-parent forecast using two existing native control owners.

This lower callback experiment supplies no station adapter or qualified gains.
One parent state and immutable vector reference feed both handles; a pair fault
inhibits both before the synthetic transport accepts any command.
"""
from dataclasses import dataclass
import math

import numpy as np

from .assembly_dynamics import ActuatorMap, CausalJointLoad, SerialAssembly, vector
from .native import CObservation, CPosterior, CReference, Controller


class AssemblyForecastRejected(ValueError):
    pass


def require(condition, detail):
    if not condition:
        raise AssemblyForecastRejected(detail)


@dataclass(frozen=True, kw_only=True)
class JointReference:
    q_ref_rad: np.ndarray
    v_ref_rad_s: np.ndarray
    a_ref_rad_s2: np.ndarray
    time_s: float
    source_time_s: float
    expires_at_s: float
    frame: str
    configuration_id: str
    trajectory_id: str
    generation: int
    fresh: bool
    valid: bool

    def __post_init__(self):
        for name in ("q_ref_rad", "v_ref_rad_s", "a_ref_rad_s2"):
            # A bytes backing store cannot be made writable by flipping NumPy's
            # array flags while either native callback is running.
            frozen = np.frombuffer(vector(getattr(self, name), (2,), name).tobytes(), dtype=float)
            object.__setattr__(self, name, frozen)
        require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in (self.time_s, self.source_time_s, self.expires_at_s)),
                "finite common reference clock required")
        require(self.source_time_s <= self.time_s <= self.expires_at_s,
                "reference source/expiry chronology invalid")
        require(self.fresh is True and self.valid is True and type(self.generation) is int
                and self.generation > 0, "fresh valid vector reference in an explicit generation required")
        require(all(type(x) is str and x for x in (self.frame, self.configuration_id, self.trajectory_id)),
                "explicit vector reference identities required")


@dataclass(frozen=True)
class PairedOutput:
    outputs: tuple
    posteriors: tuple
    demand: dict
    reference: JointReference


@dataclass(frozen=True)
class CommandReceipt:
    axis: int
    accepted_time_s: float
    command_current_A: float
    effective_current_A: float
    torque_Nm: float
    command_token: int
    generation: int


@dataclass(frozen=True, kw_only=True)
class JointSupport:
    q_min_rad: np.ndarray
    q_max_rad: np.ndarray
    velocity_max_rad_s: np.ndarray
    acceleration_max_rad_s2: np.ndarray
    max_reference_source_age_s: float
    qualification: str

    def __post_init__(self):
        for name in ("q_min_rad", "q_max_rad", "velocity_max_rad_s", "acceleration_max_rad_s2"):
            object.__setattr__(self, name, vector(getattr(self, name), (2,), name))
        require(np.all(self.q_min_rad < self.q_max_rad) and np.all(self.velocity_max_rad_s > 0)
                and np.all(self.acceleration_max_rad_s2 > 0), "finite supported joint envelopes required")
        require(isinstance(self.max_reference_source_age_s, (int, float))
                and not isinstance(self.max_reference_source_age_s, bool)
                and math.isfinite(self.max_reference_source_age_s) and self.max_reference_source_age_s >= 0
                and self.qualification == "SYNTHETIC_OFFLINE", "explicit synthetic reference age/envelope required")


class CoupledControllerPair:
    """Nest two distinct posterior callbacks; defer all ACKs until both return."""
    def __init__(self, cores, assembly, actuator, *, core_epoch_s, frame, trajectory_id, max_load_age_s, support):
        require(len(cores) == 2 and all(isinstance(c, Controller) for c in cores)
                and cores[0].handle != cores[1].handle, "two distinct existing native handles required")
        require(isinstance(assembly, SerialAssembly) and isinstance(actuator, ActuatorMap),
                "supplied parent mechanics and actuator map required")
        require(np.all(actuator.torque_per_effective_amp_Nm*actuator.command_gain > 0),
                "this offline runner requires positive logical command-to-torque polarity")
        require(isinstance(support, JointSupport), "explicit supported reference/state envelope required")
        require(math.isfinite(core_epoch_s) and core_epoch_s >= 0 and math.isfinite(max_load_age_s)
                and max_load_age_s >= 0, "explicit common clock epoch/load age required")
        self.cores, self.assembly, self.actuator = tuple(cores), assembly, actuator
        self.epoch, self.frame, self.trajectory = core_epoch_s, frame, trajectory_id
        self.max_load_age = max_load_age_s
        self.support = support
        self.faulted = False
        self.pending = None
        self.trace = []
        self.receipts = []

    def inhibit(self):
        self.faulted = True
        self.pending = None
        for core in self.cores:
            core.inhibit()

    def step(self, observations, reference, loads):
        posteriors, outputs, demand = [None, None], [None, None], {}
        try:
            require(not self.faulted and self.pending is None, "pair faulted or prior ACK still pending")
            require(isinstance(reference, JointReference) and isinstance(loads, CausalJointLoad),
                    "one immutable vector reference and explicit causal loads required")
            require((reference.configuration_id, reference.frame, reference.trajectory_id)
                    == (self.assembly.configuration_id, self.frame, self.trajectory),
                    "vector reference context mismatch")
            require(reference.time_s-reference.source_time_s <= self.support.max_reference_source_age_s,
                    "vector reference source is older than its declared support")
            require(np.all(reference.q_ref_rad >= self.support.q_min_rad)
                    and np.all(reference.q_ref_rad <= self.support.q_max_rad)
                    and np.all(np.abs(reference.v_ref_rad_s) <= self.support.velocity_max_rad_s)
                    and np.all(np.abs(reference.a_ref_rad_s2) <= self.support.acceleration_max_rad_s2),
                    "vector q/v/a reference exceeds its declared support")
            require(len(observations) == 2 and all(isinstance(o, CObservation) for o in observations),
                    "two native observations required")
            now = reference.time_s + self.epoch
            require(all(o.now == now and o.generation == reference.generation for o in observations),
                    "both native observations must share the reference clock and generation")
            require(observations[0].encoder_time == observations[1].encoder_time
                    and observations[0].gyro_time == observations[1].gyro_time,
                    "declared paired source clocks disagree")
            refs = tuple(CReference(reference.q_ref_rad[k], reference.v_ref_rad_s[k],
                                    reference.a_ref_rad_s2[k], reference.q_ref_rad[1-k]) for k in range(2))
            for index, core in enumerate(self.cores):
                model = core.params.model
                require((model.periodic or model.q[0] <= refs[index].position <= model.q[model.n-1])
                        and model.z[0] <= refs[index].posture <= model.z[2],
                        "reference exceeds supplied native numerical model domain")

            def capture(index, posterior, packet):
                require(posterior.now == now and posterior.generation == reference.generation,
                        "native posterior clock/generation mismatch")
                require((packet.position, packet.velocity, packet.acceleration, packet.posture)
                        == (refs[index].position, refs[index].velocity, refs[index].acceleration, refs[index].posture),
                        "native callback reference differs from authoritative vector")
                posteriors[index] = CPosterior.from_buffer_copy(posterior)
                require(self.support.q_min_rad[index] <= posterior.position <= self.support.q_max_rad[index]
                        and abs(posterior.velocity) <= self.support.velocity_max_rad_s[index],
                        "native posterior exceeds its supplied diagnostic support")
                other = posteriors[1-index]
                if other is not None:
                    require(posterior.encoder_time == other.encoder_time
                            and posterior.gyro_time == other.gyro_time,
                            "same-cycle native posterior source clocks disagree")

            def inner(posterior, packet):
                capture(1, posterior, packet)
                # Same-cycle native posterior positions enter rigid-body M/C/G
                # geometry. External loads are the explicit causal packet;
                # planned v/a remain from the one vector trajectory.
                current_q = np.array([p.position for p in posteriors])
                demand.update(self.assembly.current_demand(current_q, reference.v_ref_rad_s,
                    reference.a_ref_rad_s2, self.actuator, loads, now_s=reference.time_s,
                    max_load_age_s=self.max_load_age))
                self.trace.append("VECTOR_FF")
                return float(demand["command_current_A"][1])

            def outer(posterior, packet):
                capture(0, posterior, packet)
                outputs[1] = self.cores[1].step_posterior_feedforward(observations[1], refs[1], inner)
                require(outputs[1].status == 0, f"pitch native fault {outputs[1].status}")
                self.trace.append("PITCH_OUTPUT")
                return float(demand["command_current_A"][0])

            outputs[0] = self.cores[0].step_posterior_feedforward(observations[0], refs[0], outer)
            require(outputs[0].status == 0 and outputs[1] is not None and outputs[1].status == 0,
                    f"paired native fault {[getattr(o, 'status', None) for o in outputs]}")
            self.trace.append("YAW_OUTPUT")
            self.pending = PairedOutput(tuple(outputs), tuple(posteriors), demand, reference)
            return self.pending
        except BaseException:
            self.inhibit()
            raise

    def acknowledge(self, result, *, record_receipt):
        """Both outputs precede transport; preserve every individual receipt.

        Sequential receipt accounting is not atomic physical two-axis TX. A
        later failure latches the pair but cannot erase an already accepted
        command. The supplied recorder updates the one offline parent input.
        """
        try:
            require(not self.faulted and result is self.pending, "no matching pending pair")
            require(callable(record_receipt), "explicit successful per-axis receipt recorder required")
            accepted_time = result.reference.time_s+self.epoch
            for axis, (core, out) in enumerate(zip(self.cores, result.outputs)):
                require(core.ack(out, accepted_time=accepted_time),
                        f"axis {axis} ACK failed; earlier successful receipts remain applied")
                effective = self.actuator.command_gain[axis]*out.limited+self.actuator.command_bias_A[axis]
                receipt = CommandReceipt(axis, result.reference.time_s, out.limited, float(effective),
                    float(self.actuator.torque_per_effective_amp_Nm[axis]*effective),
                    int(out.sequence), result.reference.generation)
                self.receipts.append(receipt)
                record_receipt(receipt)
                self.trace.append("YAW_ACK" if axis == 0 else "PITCH_ACK")
            self.pending = None
            return vector([out.limited for out in result.outputs], (2,), "accepted command pair")
        except BaseException:
            self.inhibit()
            raise


def replay_history(assembly, times_s, interval_commands_A, initial, actuator, load_function,
                   *, max_step_s, max_load_age_s):
    """One coupled parent state, subdivided only at numerical integration points."""
    times = np.asarray(times_s, dtype=float)
    commands = vector(interval_commands_A, (len(times)-1, 2), "successful ZOH interval commands")
    state = vector(initial, (4,), "single parent initial state")
    require(math.isfinite(max_step_s) and max_step_s > 0, "positive supplied integration step required")
    history = [state.copy()]
    for index, (begin, end) in enumerate(zip(times[:-1], times[1:])):
        divisions = max(1, math.ceil((end-begin)/max_step_s-1e-12))
        fine = np.linspace(begin, end, divisions+1)
        trace = assembly.rollout(fine, np.tile(commands[index], (divisions, 1)), state[:2], state[2:],
                                 actuator, load_function, max_load_age_s=max_load_age_s)
        state = trace[-1]
        history.append(state.copy())
    return np.asarray(history)


class CausalParent:
    """One supplied parent with causal accepted events and gyro filter memory.

    The verified synthetic transport delay shifts accepted command events. No
    inversion or future trajectory prediction is used to compensate this delay.
    Mechanical state advances only with already accepted current; sensor delay
    reads the retained simulated prefix, never future plant state.
    """
    def __init__(self, assembly, actuator, initial, load_function, *, max_step_s,
                 transport_delay_s, gyro_filter_tau_s, max_load_age_s, prehistory_command_A):
        require(all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in (max_step_s, transport_delay_s, gyro_filter_tau_s, max_load_age_s))
                and max_step_s > 0 and min(transport_delay_s, gyro_filter_tau_s, max_load_age_s) >= 0,
                "explicit finite parent integration, delay, filter and load-age values required")
        self.assembly, self.actuator, self.loads = assembly, actuator, load_function
        self.step_s, self.delay_s, self.tau_s, self.load_age_s = (
            max_step_s, transport_delay_s, gyro_filter_tau_s, max_load_age_s)
        self.state = vector(initial, (4,), "one causal parent initial state").copy()
        self.time_s = 0.
        self.filtered_v = self.state[2:].copy()
        self.times, self.states, self.filtered = [0.], [self.state.copy()], [self.filtered_v.copy()]
        self.tx_times, self.commands = [-max(.1, transport_delay_s)], [
            vector(prehistory_command_A, (2,), "supplied accepted command prehistory")]

    def accept(self, at_s, command_A):
        require(at_s == self.time_s and at_s > self.tx_times[-1],
                "accepted pair must be current-time and uniquely increasing")
        self.tx_times.append(float(at_s))
        self.commands.append(vector(command_A, (2,), "accepted two-axis command"))

    def accept_receipt(self, receipt):
        require(isinstance(receipt, CommandReceipt) and receipt.axis in (0, 1)
                and receipt.accepted_time_s == self.time_s and receipt.accepted_time_s >= self.tx_times[-1],
                "per-axis successful receipt must be causal and current-time")
        command = self.commands[-1].copy()
        command[receipt.axis] = receipt.command_current_A
        self.tx_times.append(receipt.accepted_time_s)
        self.commands.append(vector(command, (2,), "per-axis merged accepted input"))

    def advance(self, until_s):
        require(math.isfinite(until_s) and until_s > self.time_s,
                "causal parent time must advance monotonically")
        while self.time_s < until_s:
            begin = self.time_s
            event = int(np.searchsorted(self.tx_times, begin-self.delay_s+1e-13, side="right"))-1
            require(event >= 0, "accepted prehistory does not cover transport delay")
            end = min(until_s, begin+self.step_s)
            if event+1 < len(self.tx_times):
                delayed_event = self.tx_times[event+1]+self.delay_s
                if delayed_event > begin+1e-13:
                    end = min(end, delayed_event)
            require(end > begin, "integration clock cannot represent a positive substep")
            old_v = self.state[2:].copy()
            self.state = self.assembly.rollout([begin, end], [self.commands[event]],
                self.state[:2], self.state[2:], self.actuator, self.loads,
                max_load_age_s=self.load_age_s)[-1]
            dt = end-begin
            if self.tau_s:
                # Exact first-order filter update for a linear interpolation
                # between the two simulated endpoint velocities.
                loss = -math.expm1(-dt/self.tau_s)
                slope = (self.state[2:]-old_v)/dt
                self.filtered_v += loss*(old_v-self.filtered_v)+slope*(dt-self.tau_s*loss)
            else:
                self.filtered_v = self.state[2:].copy()
            self.time_s = end
            self.times.append(end)
            self.states.append(self.state.copy())
            self.filtered.append(self.filtered_v.copy())
        return self.state.copy()

    def gyro_at(self, source_s):
        require(math.isfinite(source_s) and 0 <= source_s <= self.time_s,
                "delayed sensor source must lie in the available simulated prefix")
        available = np.asarray(self.filtered)
        return np.array([np.interp(source_s, self.times, available[:, axis]) for axis in range(2)])
