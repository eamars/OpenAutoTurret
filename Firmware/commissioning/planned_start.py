"""Opt-in synthetic distinct-leg START admission and accepted-command dose accounting.

The ledger bounds START intervals through logical inhibit under supplied timing
support. It retains the last accepted command and does not certify physical
current cutoff or winding energy after an aborted command path.
"""
from dataclasses import dataclass, replace
import math


class PlannedStartRejected(ValueError):
    def __init__(self, reason, detail):
        self.reason = reason
        super().__init__(detail)


def require(condition, reason, detail):
    if not condition:
        raise PlannedStartRejected(reason, detail)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True, kw_only=True)
class PlannedStartLeg:
    direction: int
    departure_offset_s: float
    departure_position_rad: float


@dataclass(frozen=True, kw_only=True)
class PlannedStartProgram:
    configuration_id: str
    frame: str
    trajectory_id: str
    generation: int
    source_time_s: float
    expires_at_s: float
    legs: tuple[PlannedStartLeg, ...]
    qualification: str = "SYNTHETIC_PLANNED_START_PROGRAM"
    clock_translated: bool = False

    def with_epoch(self, epoch_s):
        """Translate the immutable declaration once, before creating its ledger."""
        require(finite(epoch_s) and epoch_s >= 0 and self.clock_translated is False,
                "INVALID_REFERENCE", "planned program clock may be translated exactly once before binding")
        return replace(self, source_time_s=self.source_time_s+epoch_s,
                       expires_at_s=self.expires_at_s+epoch_s, clock_translated=True)

    def validate(self, support, parameters):
        policy = support.start_policy
        require(self.qualification == "SYNTHETIC_PLANNED_START_PROGRAM"
                and support.qualification == "SYNTHETIC_OFFLINE" and policy is not None
                and policy.max_attempts == 1,
                "OUTSIDE_SUPPORT", "planned program requires the unchanged bounded single-attempt policy")
        require(self.configuration_id == support.configuration_id and self.frame == support.frame
                and isinstance(self.trajectory_id, str) and bool(self.trajectory_id)
                and type(self.generation) is int and self.generation > 0
                and finite(self.source_time_s) and finite(self.expires_at_s)
                and self.source_time_s < self.expires_at_s and type(self.clock_translated) is bool,
                "INVALID_REFERENCE", "immutable planned program context/source/expiry required")
        require(type(self.legs) is tuple and bool(self.legs),
                "INVALID_REFERENCE", "immutable nonempty planned leg table required")
        previous = None
        for leg in self.legs:
            require(isinstance(leg, PlannedStartLeg) and type(leg.direction) is int
                    and leg.direction in (-1, 1) and finite(leg.departure_offset_s)
                    and leg.departure_offset_s >= 0 and finite(leg.departure_position_rad)
                    and support.q_min_rad <= leg.departure_position_rad <= support.q_max_rad
                    and self.source_time_s+leg.departure_offset_s <= self.expires_at_s,
                    "INVALID_REFERENCE", "planned leg needs supported direction, original anchor and position")
            if previous is not None:
                require(leg.departure_offset_s > previous.departure_offset_s
                        and leg.direction == -previous.direction,
                        "INVALID_REFERENCE", "distinct planned legs require ordered anchors and opposite directions")
            previous = leg
        require(policy.max_command_dose_A2s > 0 and parameters.dt_max > 0,
                "OUTSIDE_SUPPORT", "positive aggregate command-dose/timing support required")
        return self


class PlannedStartLedger:
    def __init__(self, program, parameters, support):
        self.program = program
        self.ceiling_A2s = support.start_policy.max_command_dose_A2s
        self.current_cap_A = parameters.current_cap
        self.dt_max_s = parameters.dt_max
        self.ack_max_s = support.max_ack_delay_s
        self.rest_speed_rad_s = support.rest_speed_rad_s
        self.initialized = self.terminal = False
        self.dose_A2s = 0.
        self.through_s = None
        self.held_current_A = None
        self.held_start = False
        self.accepted_time_s = None
        self.active_leg = None
        self.admissions = []
        self.move_times = {}
        self.receipts = []
        self.pending_output = None
        self.unknown_future_hold = False

    def initialize(self, now, state, previous_current_A, accepted_time_s):
        require(not self.initialized and not self.terminal,
                "UNARMED", "a planned program ledger cannot reset or rearm")
        require(state.generation == self.program.generation
                and now <= self.program.source_time_s+self.program.legs[0].departure_offset_s,
                "INVALID_REFERENCE", "planned program must bind once before its first original anchor")
        self.initialized = True
        self.through_s, self.held_current_A, self.accepted_time_s = now, previous_current_A, accepted_time_s

    def check_reference(self, reference):
        p = self.program
        require((reference.configuration_id, reference.frame, reference.trajectory_id,
                 reference.generation, reference.source_time_s, reference.expires_at_s) ==
                (p.configuration_id, p.frame, p.trajectory_id, p.generation, p.source_time_s, p.expires_at_s),
                "INVALID_REFERENCE", "planned program reference context/source/expiry cannot be relabeled")
        if reference.trajectory_phase == "DEPARTURE":
            self.reference_leg(reference)

    def reference_leg(self, reference):
        key = (reference.planned_direction, reference.departure_offset_s, reference.departure_position_rad)
        indices = [i for i, leg in enumerate(self.program.legs)
                   if key == (leg.direction, leg.departure_offset_s, leg.departure_position_rad)]
        require(len(indices) == 1, "INVALID_REFERENCE", "DEPARTURE is absent from the immutable planned leg table")
        return indices[0]

    def advance(self, now, *, enforce=True):
        require(self.initialized and finite(now) and now >= self.through_s,
                "INVALID_STATE", "planned command-dose clock must increase causally")
        if self.held_start:
            self.dose_A2s += self.held_current_A**2*(now-self.through_s)
        self.through_s = now
        if enforce:
            require(self.dose_A2s <= self.ceiling_A2s+1e-15,
                    "OUTSIDE_SUPPORT", "aggregate successful START command-dose ceiling exhausted")

    def before_callback(self, reference, posterior, prior_motion):
        require(not self.terminal, "UNARMED", "planned program is terminal after inhibit")
        if reference.trajectory_phase == "DEPARTURE":
            declared = self.reference_leg(reference)
            require(declared <= len(self.admissions)
                    and (declared == len(self.admissions) or declared == self.active_leg),
                    "OUTSIDE_SUPPORT", "closed or out-of-order planned leg cannot be redeclared")
            if declared == len(self.admissions):
                require(abs(posterior.velocity) <= self.rest_speed_rad_s
                        and (declared == 0 or declared-1 in self.move_times),
                        "OUTSIDE_SUPPORT", "next planned leg requires causal rest and previous accepted MOVE evidence")
        entered = posterior.motion == 1 and prior_motion != 1
        if entered:
            require(reference.trajectory_phase == "DEPARTURE",
                    "OUTSIDE_SUPPORT", "unplanned HOLD/correction/restart cannot consume a planned leg")
            index = self.reference_leg(reference)
            require(index == len(self.admissions),
                    "OUTSIDE_SUPPORT", "closed, replayed or out-of-order planned leg cannot retry START")
            anchor = self.program.source_time_s+self.program.legs[index].departure_offset_s
            require(-1e-12 <= posterior.now-anchor <= self.dt_max_s+1e-12,
                    "INVALID_REFERENCE", "planned START must occur at its original supported anchor tick")
            require(abs(posterior.velocity) <= self.rest_speed_rad_s
                    and (index == 0 or index-1 in self.move_times),
                    "OUTSIDE_SUPPORT", "next planned leg requires causal rest and previous accepted MOVE evidence")
        if posterior.motion == 1 or self.held_start:
            # Reserve a cap-bounded next supported control interval plus the
            # acknowledged-send window of the currently held START command.
            reserve = self.current_cap_A**2*self.dt_max_s
            if self.held_start:
                reserve += self.held_current_A**2*self.ack_max_s
            require(self.dose_A2s+reserve <= self.ceiling_A2s+1e-15,
                    "OUTSIDE_SUPPORT", "aggregate START dose cannot reserve the next supported command interval")
        if entered:
            self.active_leg = index
            self.admissions.append({"leg": index, "time_s": posterior.now,
                                    "direction": self.program.legs[index].direction})
        return entered

    def output(self, output):
        self.pending_output = (int(output.sequence), int(output.motion), self.active_leg)

    def receipt(self, output, accepted_time_s, applied_current_A):
        # Called only after the existing native ACK succeeds. Preserve that
        # actual receipt before any later fault can censor the programme.
        self.advance(accepted_time_s, enforce=False)
        _, motion, leg = self.pending_output
        self.held_current_A, self.accepted_time_s = applied_current_A, accepted_time_s
        self.held_start = motion == 1
        self.receipts.append({"sequence": int(output.sequence), "time_s": accepted_time_s,
                              "current_A": applied_current_A, "START_interval": self.held_start, "leg": leg})
        if motion == 2 and leg is not None:
            self.move_times.setdefault(leg, accepted_time_s)
        self.pending_output = None

    def abort(self, now):
        if self.initialized and finite(now) and now >= self.through_s:
            self.advance(now, enforce=False)
        self.terminal = True
        self.pending_output = None
        self.unknown_future_hold = True

    def report(self):
        return {"qualification": "SYNTHETIC_PLANNED_START_PROGRAM",
                "dose_A2s": self.dose_A2s, "ceiling_A2s": self.ceiling_A2s,
                "accounted_through_s": self.through_s, "admissions": [dict(entry) for entry in self.admissions],
                "accepted_MOVE_times_s": dict(self.move_times), "successful_receipt_count": len(self.receipts),
                "last_accepted_current_A": self.held_current_A, "last_accepted_time_s": self.accepted_time_s,
                "last_accepted_START_interval": self.held_start, "terminal": self.terminal,
                "unknown_future_hold": self.unknown_future_hold,
                "physical_current_cutoff": "NOT_ESTABLISHED"}
