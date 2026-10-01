"""Bounded offline configuration reuse and invalidation through the native FF owner.

All configuration declarations are synthetic. The fitted diagnostic receipt is
reused without assigning a new model revision or manufacturing uncertainty.
"""
from dataclasses import replace
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.adaptation import ChangeMonitor, FailurePolicy
from Firmware.commissioning.applicability import (ConfigurationFact, ConfigurationFacts,
    FactStatus, assess_configuration)
from Firmware.commissioning.contracts import Reason, Rejected, require
from Firmware.commissioning.family_assets import FamilyAsset, bind_runtime_document
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.motor_feedforward import (CausalState, FeedforwardRejected,
    MotorFeedforward, ReferencePacket)
from Firmware.commissioning.native import CObservation, CReference, Controller, Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout


def changed_facts(baseline, event):
    """Describe an out-of-family event; never infer its replacement coefficients."""
    values = dict(baseline.facts)
    if event in ("PAYLOAD+", "PAYLOAD-"):
        values["payload.distribution"] = ConfigurationFact(
            (event, "different mass distribution and axis offset; coefficients not identified"),
            FactStatus.SYNTHETIC, "predeclared offline configuration boundary")
    elif event in ("FRICTION+", "FRICTION-"):
        # This receipt has no supported friction-condition coordinate. A newly
        # declared condition is unclassified, not permission to widen support.
        values["friction.condition"] = ConfigurationFact(event, FactStatus.SYNTHETIC,
            "predeclared offline friction condition; outside receipt support")
    else:
        raise ValueError("known declared configuration event required")
    return ConfigurationFacts(baseline.context_id, "SYNTHETIC", values)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


class RestRuntime:
    """One native owner, one uninterrupted synthetic plant acquisition.

    The independently supplied rest balance leaves every accepted command at
    10mA, below both static thresholds. Source ages still use the fitted delay.
    The entire received input/sensor record is checked through both plant paths.
    """
    def __init__(self, asset, parameters, support, native):
        self.asset, self.parameters, self.support = asset, parameters, support
        self.core = Controller(native, parameters)
        self.adapter = MotorFeedforward(self.core, asset.model, support)
        self.epoch, self.local_time, self.sequence = 10., 0., 0
        self.reset_local_time = 0.
        self.configuration_generation, self.native_generation = 0, 1
        self.command_A = .010
        self.tx_t, self.tx_A = [-.1], [self.command_A]
        self.native_receipts, self.observed, self.outputs = [], [], []
        self.pre_restart_gyro_samples_skipped = 0
        self.state = CausalState(q_rad=0., v_rad_s=0., posture_rad=support.fixed_posture_rad,
            winding_rad=0., temperature=20., time_s=self.epoch,
            frame=support.frame, configuration_id=support.configuration_id, generation=1,
            friction_state="POSTERIOR_POLICY", static_balance_A=(asset.model.actuator_gain*self.command_A+
                asset.model.actuator_bias-asset.model.load_offset), fresh=True, valid=True,
            provenance="SYNTHETIC", configuration_facts=asset.configuration_support.baseline)
        require(-asset.model.static_negative <= self.state.static_balance_A <= asset.model.static_positive,
            Reason.ENVELOPE_LIMITED, "10mA rest balance must fit the declared synthetic static interval")
        self.adapter.reset(self.state, now=self.epoch, previous_current_A=self.command_A,
                           accepted_time_s=self.epoch+self.tx_t[-1])

    def close(self):
        self.core.close()

    def tick(self, facts=None):
        self.sequence += 1
        self.local_time = round(self.sequence*.005, 9)
        now = self.epoch+self.local_time
        source = np.floor((self.local_time+1e-12)/.020)*.020-self.asset.model.gyro_delay
        if source < self.reset_local_time:
            self.pre_restart_gyro_samples_skipped += 1
        observation = CObservation(now, now, self.epoch+source, 0., 0.,
            self.sequence+1, int(round((source+self.asset.model.gyro_delay)/.020))+1,
            self.native_generation, True, bool(source >= self.reset_local_time))
        state = replace(self.state, time_s=now, generation=self.native_generation,
            configuration_facts=facts or self.state.configuration_facts)
        reference = ReferencePacket(q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0., time_s=now,
            source_time_s=now, expires_at_s=now+.050, frame=self.support.frame,
            configuration_id=self.support.configuration_id, trajectory_id="declared-static-balance",
            generation=self.native_generation, fresh=True, valid=True, trajectory_phase="HOLD")
        return observation, reference, state

    def accept_hold(self, stale_output=None):
        observation, reference, state = self.tick()
        output, demand = self.adapter.step(observation, reference, state)
        require(abs(output.limited-self.command_A) <= 1e-12 and output.status == 0 and output.motion == 0,
            Reason.INTEGRATION_MISMATCH, "supplied stationary balance must produce a 10mA REST command")
        if stale_output is not None:
            require(not self.core.ack(stale_output, accepted_time=observation.now),
                Reason.INTEGRATION_MISMATCH, "previous-generation command token was replayed")
        require(self.adapter.acknowledge(output, successful=True, accepted_time_s=observation.now),
            Reason.INTEGRATION_MISMATCH, "actual native acknowledgement failed")
        self.tx_t.append(self.local_time)
        self.tx_A.append(float(output.limited))
        self.native_receipts.append({"native_generation": self.native_generation,
            "configuration_generation": self.configuration_generation,
            "sequence": int(output.sequence), "accepted_time_s": observation.now,
            "command_A": float(output.limited)})
        self.observed.append((self.local_time, 0., 0.))
        self.outputs.append(output)
        return output

    def invalidate(self, facts):
        before = dict(self.native_receipts[-1])
        count = len(self.tx_t)
        observation, reference, state = self.tick(facts)
        try:
            self.adapter.step(observation, reference, state)
        except FeedforwardRejected as error:
            rejection = {"reason": error.reason.value, "detail": str(error)}
        else:
            raise Rejected(Reason.INTEGRATION_MISMATCH, "unsupported configuration emitted a command")
        inhibited = self.core.step(observation, CReference(0., 0., 0., 0.))
        require((inhibited.status, inhibited.sequence, inhibited.requested, inhibited.limited) == (5, 0, 0., 0.)
            and not self.adapter.armed and len(self.tx_t) == count,
            Reason.INTEGRATION_MISMATCH, "configuration invalidation did not latch before another command")
        require(not self.core.ack(self.outputs[-1], accepted_time=observation.now),
            Reason.INTEGRATION_MISMATCH, "old command receipt was accepted after inhibit")
        return {"rejection": rejection, "native_status": int(inhibited.status),
            "new_command_count": 0, "accepted_receipt_before_invalidation": before,
            "accepted_receipt_after_invalidation": dict(self.native_receipts[-1]),
            "last_command_retained_A": self.tx_A[-1], "logical_time_s": observation.now,
            "physical_current_cutoff": "UNKNOWN; no new zero command is treated as accepted"}

    def return_to_baseline(self):
        self.configuration_generation += 1
        assessment = assess_configuration(self.asset.configuration_support,
            self.asset.configuration_support.baseline, purpose="SYNTHETIC_CONTROL")
        observation, reference, state = self.tick()
        try:
            self.adapter.step(observation, reference, state)
        except FeedforwardRejected:
            remains_inhibited = True
        else:
            remains_inhibited = False
        require(remains_inhibited, Reason.INTEGRATION_MISMATCH,
            "compatible RETURN must not automatically clear the existing fault")
        require(assessment["supported"], Reason.OPERATING_POINT_CHANGED,
            "exact baseline RETURN must remain in the unchanged model support")
        previous_generation = self.native_generation
        self.native_generation += 1
        # Explicit synthetic restart, never a physical restart authorization.
        # Entire uninterrupted plant history is checked below for true rest.
        self.state = replace(self.state, time_s=observation.now, generation=self.native_generation,
            configuration_facts=self.asset.configuration_support.baseline)
        self.reset_local_time = self.local_time
        self.adapter.reset(self.state, now=observation.now, previous_current_A=self.tx_A[-1],
            accepted_time_s=self.epoch+self.tx_t[-1])
        last = self.outputs[-1]
        first = self.accept_hold(stale_output=last)
        return {"event": "RETURN", "assessment": assessment, "automatic_rearm": False,
            "explicit_synthetic_stationary_reset": True, "configuration_generation": self.configuration_generation,
            "previous_native_generation": previous_generation, "native_generation": self.native_generation,
            "new_receipt_sequence": int(first.sequence), "old_generation_token_rejected": True,
            "model_revision_changed": False, "receipt_clock_reset": False,
            "physical_reentry": "UNQUALIFIED; safe-state and configuration confirmation required"}


def plant_verification(runtime, family, hidden=None):
    t = np.linspace(0., runtime.local_time, int(round(runtime.local_time/.001))+1)
    initial = np.asarray(runtime.asset.gauges["acquisition_state"]["value"], float)
    require(np.all(initial == 0.), Reason.DATA_INVALID, "this rest lifecycle requires the supplied zero acquisition state")
    trace = family.rollout(runtime.asset.model, t, runtime.tx_t, runtime.tx_A, initial)
    oracle = independent_rollout(runtime.asset.model, t, runtime.tx_t, runtime.tx_A, initial).trace
    errors = {"q_rms_rad": float(np.sqrt(np.mean((trace[:, 0]-oracle[:, 0])**2))),
        "gyro_rms_rad_s": float(np.sqrt(np.mean((trace[:, 3]-oracle[:, 3])**2))),
        "current_rms_A": float(np.sqrt(np.mean((trace[:, 4]-oracle[:, 4])**2)))}
    limits = {"q_rms_rad": 1e-5, "gyro_rms_rad_s": 1e-4, "current_rms_A": 1e-9}
    require(all(errors[k] <= limits[k] for k in errors), Reason.INTEGRATION_MISMATCH,
        "rest lifecycle native/independent forward gate failed")
    samples = np.rint(np.asarray(runtime.observed)[:, 0]/.001).astype(int)
    rest_error = float(np.max(np.abs(trace[samples][:, (0, 3)])))
    require(rest_error == 0., Reason.INTEGRATION_MISMATCH,
        "native received history contradicts the supplied rest observations/restart states")
    changed = family.rollout(hidden, t, runtime.tx_t, runtime.tx_A, initial) if hidden else None
    return t, trace, changed, {"forward_errors": errors, "forward_limits": limits,
        "forward_pass": True, "causal_rest_observation_max_error": rest_error,
        "plant_initializations": 1, "core_owner_instances": 1,
        "future_measured_state_substituted": False,
        "scope": "10mA sticking fixture only; not tracking or changed-configuration prediction"}


def run_probe(receipt, library, output, early=False):
    output.mkdir(parents=True, exist_ok=False)
    native, family = Native(library), FamilyNative(library)
    raw = json.loads(receipt.read_text(encoding="utf-8"))
    asset, parameters, support, binding = bind_runtime_document(raw, native)
    events = ("PAYLOAD+",) if early else ("PAYLOAD+", "PAYLOAD-", "FRICTION+", "FRICTION-")
    declaration = {"scope": "SYNTHETIC_DIAGNOSTIC_CONFIGURATION_BOUNDARIES_ONLY",
        "events": ["BASELINE", *[v for event in events for v in (event, "RETURN")], "UNDECLARED"],
        "model_revision": asset.model_revision, "evidence_partition": asset.evidence_partition,
        "uncertainty": asset.uncertainty["status"], "controller_synthesis": binding["synthesis"],
        "rest_command_A": .010, "current_cap_A": parameters.current_cap,
        "slew_A_s": parameters.slew, "unchanged_native_parameters": True,
        "hidden_change": "a and viscous +20%, Coulomb and static friction +10%; only rest observability is tested",
        "monitor": "three nonoverlapping2s windows; fresh20ms samples of the actual hidden-model rollout; original ChangeMonitor predicate",
        "sensor_noise": "NOISELESS_STRUCTURAL_OBSERVABILITY_PROBE; no calibrated false-alarm claim",
        "qualification_attempt": "must reject diagnostic consumed model with UNKNOWN uncertainty",
        "changed_configuration_identification": "NOT_RUN; no new identified revision supplied",
        "physical_stage3a": "NOT_RUN", "physical_stage3b": "NOT_RUN", "deployment_authorized": False}
    save(output/"predeclared-contract.json", declaration)
    runtime = RestRuntime(asset, parameters, support, native)
    policy, cases = FailurePolicy(), []
    try:
        runtime.accept_hold()
        cases.append({"event": "BASELINE", "assessment": assess_configuration(asset.configuration_support,
            asset.configuration_support.baseline, purpose="SYNTHETIC_CONTROL"),
            "actual_receipt": runtime.native_receipts[-1], "model_qualified": False})
        for event in events:
            runtime.configuration_generation += 1
            facts = changed_facts(asset.configuration_support.baseline, event)
            assessment = assess_configuration(asset.configuration_support, facts, purpose="SYNTHETIC_CONTROL")
            require(not assessment["supported"] and assessment["action"] == "UPDATE_PARAMETERS_AND_REVALIDATE",
                Reason.OPERATING_POINT_CHANGED, "changed configuration must invalidate its dependent assets")
            cases.append({"event": event, "configuration_generation": runtime.configuration_generation,
                "native_generation": runtime.native_generation, "assessment": assessment,
                "runtime": runtime.invalidate(facts),
                "failure_policy": policy.handle(Reason.OPERATING_POINT_CHANGED),
                "automatic_identification": "REQUIRED_NOT_RUN; same method, no manual gain substitution",
                "replacement_model_revision": None})
            cases.append(runtime.return_to_baseline())
        undeclared_start = runtime.local_time
        for _ in range(1200): runtime.accept_hold()
        hidden = replace(asset.model, a=asset.model.a*1.2, viscous=asset.model.viscous*1.2,
            coulomb_negative=asset.model.coulomb_negative*1.1,
            coulomb_positive=asset.model.coulomb_positive*1.1,
            static_negative=asset.model.static_negative*1.1, static_positive=asset.model.static_positive*1.1)
        t, trace, changed, numerical = plant_verification(runtime, family, hidden)
        monitor, windows = ChangeMonitor(), []
        for number in range(3):
            # Use one explicit event-local monitor epoch. Absolute decimal
            # subtraction produced a 1.9999999999999996s INVALID_WINDOW;
            # exact 0..2,2..4,4..6s duration retains the original >=2s gate.
            start, end = number*2., (number+1)*2.
            source_start, source_end = undeclared_start+start, undeclared_start+end
            selected = (t > source_start+1e-12) & (t <= source_end+1e-12) & (np.arange(len(t)) % 20 == 0)
            residual = np.c_[(changed[selected, 0]-trace[selected, 0])/.00015,
                (changed[selected, 3]-trace[selected, 3])/.005,
                (changed[selected, 4]-trace[selected, 4])/.002]
            response = monitor.window(start=start, end=end, normalized_residual=residual.ravel(),
                actual_samples=int(selected.sum()), effective_hz=50.)
            windows.append({"start_s": start, "end_s": end, "source_window_start_s": source_start,
                "source_window_end_s": source_end, "monitor_epoch_acquisition_s": undeclared_start,
                "actual_fresh_samples": int(selected.sum()),
                "normalized_residual_RMS": float(np.sqrt(np.mean(residual**2))), "response": response})
        cases.append({"event": "UNDECLARED", "assessment_with_unchanged_facts": assess_configuration(
            asset.configuration_support, asset.configuration_support.baseline, purpose="SYNTHETIC_CONTROL"),
            "actual_model_change": declaration["hidden_change"], "monitor_windows": windows,
            "change_detected": any(row["response"] == "CHANGE_DETECTED" for row in windows),
            "status": "INSUFFICIENT_EXCITATION_AT_REST", "false_invalidation": False,
            "needed_evidence": "authorized informative moving trajectory; qualified residual noise/false-alarm policy",
            "finite_recovery_routes": [policy.handle(Reason.INSUFFICIENT_EXCITATION) for _ in range(3)]})
        # Revelation is a different event, with declared evidence. The native
        # owner must inhibit before any further accepted command is added.
        runtime.configuration_generation += 1
        revealed = changed_facts(asset.configuration_support.baseline, "FRICTION+")
        cases.append({"event": "UNDECLARED_REVEALED", "configuration_generation": runtime.configuration_generation,
            "assessment": assess_configuration(asset.configuration_support, revealed, purpose="SYNTHETIC_CONTROL"),
            "runtime": runtime.invalidate(revealed), "model_revision_changed": False})
        qualification = asset.document()
        qualification["qualification"] = "QUALIFIED_SYNTHETIC_MODEL"
        try: FamilyAsset.from_document(qualification)
        except Rejected as error:
            qualified_attempt = {"accepted": False, "reason": error.reason.value, "detail": error.detail}
        else: raise Rejected(Reason.INTEGRATION_MISMATCH, "unknown uncertainty was promoted")
        result = {"schema": "adr0022.configuration-lifecycle-probe/1", "interface_status": "PASS",
            "early_probe": early, "cases": cases, "actual_binding": {"native_abi": binding["native_abi"],
                "complete_parameter_readback": binding["complete_parameter_readback"], "synthesis": binding["synthesis"]},
            "model_revision": asset.model_revision, "model_revision_count": 1,
            "configuration_generation_count": runtime.configuration_generation+1,
            "native_generation_count": runtime.native_generation, "actual_native_receipt_count": len(runtime.native_receipts),
            "pre_restart_gyro_samples_skipped": runtime.pre_restart_gyro_samples_skipped,
            "receipt_clock_monotonic": bool(np.all(np.diff([row["accepted_time_s"] for row in runtime.native_receipts]) > 0)),
            "final_accepted_receipt": runtime.native_receipts[-1], "final_inhibited": not runtime.adapter.armed,
            "numerical_verification": numerical, "qualification_attempt": qualified_attempt,
            "changed_configuration_identification": "NOT_RUN; no fitted revision or supported uncertainty supplied",
            "model_qualified": False, "controller_qualified": False, "physical_stage3a": "NOT_RUN",
            "physical_stage3b": "NOT_RUN", "deployment_authorized": False,
            "blockers": ["Changed payload/friction require automatic identification plus independent model/controller gates before reuse.",
                "The supplied receipt has UNKNOWN joint uncertainty and is consumed development evidence.",
                "Undeclared friction/inertia changes are unobservable in this sticking fixture; resting residuals cannot qualify the family.",
                "Synthetic RETURN reset does not establish a physical safe state or authorize physical re-entry."]}
        save(output/"decision.json", result)
        print(json.dumps({"interface_status": result["interface_status"], "actual_receipts": len(runtime.native_receipts),
            "events": [row["event"] for row in cases], "undeclared_rest_observable": False,
            "forward_errors": numerical["forward_errors"], "qualification": qualified_attempt["reason"]}), flush=True)
        return result
    finally:
        runtime.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--early", action="store_true")
    args = parser.parse_args()
    run_probe(args.receipt, args.library, args.output, args.early)
