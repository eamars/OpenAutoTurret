"""Offline configuration/reuse probe through fitting and the actual native FF path."""
from __future__ import annotations
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.applicability import (ConfigurationFact, ConfigurationFacts,
    ConfigurationResponse, ConfigurationSupport, FactStatus, assess_configuration,
    configuration_pool, supported_snapshot)
from Firmware.commissioning.contracts import ModelSpec, Rejected
from Firmware.commissioning.measurement import ObserverSpec
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun, fit_family
from Firmware.commissioning.motor_feedforward import (CausalState, FeedforwardRejected,
    FeedforwardSupport, MotorFeedforward, ReferencePacket, parameters_for_family)
from Firmware.commissioning.native import CObservation, CReference, Controller, Native, parameters


def context(radius=.1, *, provenance="SYNTHETIC", status=FactStatus.SYNTHETIC):
    return ConfigurationFacts("same-assembly-label", provenance, {
        "hardware.assembly": ConfigurationFact("synthetic-two-axis", status, "declared fixture"),
        "payload.distribution": ConfigurationFact((2., radius, .03), status, "declared geometry"),
        "mounting.geometry": ConfigurationFact("declared-mount", status, "declared fixture"),
        "cable.route": ConfigurationFact("declared-route", status, "declared fixture"),
        "transmission.mapping": ConfigurationFact("declared-transmission", status, "declared fixture"),
        "motor.settings": ConfigurationFact("declared-current-map", status, "declared motor fixture"),
        "sensor.calibration": ConfigurationFact("declared-sensors", status, "declared sensor fixture"),
        "base.orientation": ConfigurationFact((0., 0., 1.), status, "declared fixture")})


def run_probe(library):
    library = Path(library)
    baseline, changed = context(), context(.2)
    binding = ConfigurationSupport("configuration-probe-model-1", baseline,
                                   tuple(baseline.facts), qualification="SYNTHETIC_OFFLINE")
    update = assess_configuration(binding, changed, purpose="SYNTHETIC_CONTROL")
    assert update["action"] == "UPDATE_PARAMETERS_AND_REVALIDATE"
    assert set(("3a", "3b", "plant_parameters")) <= set(update["invalidate"])
    model = FamilyModel(a=.1, viscous=.06, coulomb_negative=.12, coulomb_positive=.12,
        static_negative=.16, static_positive=.16, q_min=-1e6, q_max=1e6,
        actuator_gain=2., actuator_bias=.03, transport_delay=0., gyro_bias=0., gyro_tau=0., gyro_delay=0.,
        current_gain=1., current_bias=0., current_tau=0., current_delay=0., load_offset=.02)
    native = FamilyNative(library)
    t = np.arange(0, .301, .001)
    tx_t, tx_A = np.array([-.1, .01, .15]), np.array([0., .25, -.25])
    trace = native.rollout(model, t, tx_t, tx_A, np.zeros(5))
    fresh = np.ones(len(t), dtype=bool)
    def family_run(name, facts):
        return FamilyRun(run_id=name, source_id="synthetic/"+name, t=t, q=trace[:, 0],
            v=trace[:, 3], current=trace[:, 4], q_new=fresh, v_new=fresh, current_new=fresh,
            tx_t=tx_t, tx_A=tx_A, initial=np.zeros(5), sigma_q=.00015,
            sigma_v=.005, sigma_current=.002, provenance="SYNTHETIC", configuration_facts=facts)
    try:
        fit_family(native, replace(model, a=.085),
                   [family_run("train-A", baseline), family_run("train-B", changed)],
                   bounds={"a": (.05, .15)}, max_nfev=10)
        raise AssertionError("conflicting configuration pooled")
    except Rejected as exc:
        pooling = {"rejected": True, "reason": str(exc)}
    aligned = configuration_pool([family_run("procedure-A", baseline), family_run("procedure-B", baseline)])
    legacy = configuration_pool([family_run("unknown-legacy", None)])
    spec = ModelSpec("yaw", (-100., -50., 0., 50., 100.), (-.3, 0., .3))
    theta = np.r_[np.full(3, .1), np.full(3, .06), np.full(15, -.12), np.full(15, .12), 0.]
    observer = ObserverSpec(4e-10, 6.4e-9, .1, .03, .03, 4e-10, 6.4e-9,
                            False, "synthetic-config-sensors", "SYNTHETIC")
    values = dict(kp=1., ki=1., kpos=1., kaw=1., current_cap=.9, slew=1., integral_cap=.5,
        velocity_cap=1., dt_min=.0001, dt_max=.03, intent_threshold=.00001,
        rest_speed=.001, sustained_s=.06, start_timeout_s=.15)
    starts = np.r_[np.full(15, -.16), np.full(15, .16)].reshape(2, 3, 5)
    params = parameters_for_family(parameters(spec, theta, observer, values, starts,
                                              np.zeros((2, 3, 5), bool)), model)
    support = FeedforwardSupport(configuration_id=baseline.context_id, frame="output-shaft-rad",
        q_min_rad=-1., q_max_rad=1., velocity_max_rad_s=1., acceleration_max_rad_s2=3.,
        fixed_posture_rad=0., winding_min_rad=-2., winding_max_rad=2., temperature_min=10., temperature_max=30.,
        actuator_gain_min=.5, actuator_gain_max=3., command_cap_A=.9, command_slew_A_s=1.,
        max_reference_source_age_s=.02, max_state_age_s=.02, max_ack_delay_s=.02,
        rest_speed_rad_s=.001, state_encoder_consistency_rad=.01, state_gyro_consistency_rad_s=.03,
        qualification="SYNTHETIC_OFFLINE", configuration_support=binding)
    state = CausalState(q_rad=0., v_rad_s=0., posture_rad=0., winding_rad=0., temperature=20., time_s=0.,
        frame=support.frame, configuration_id=baseline.context_id, generation=1, friction_state="STICKING",
        static_balance_A=.01, fresh=True, valid=True, provenance="SYNTHETIC", configuration_facts=baseline)
    ref = ReferencePacket(q_ref_rad=.5, v_ref_rad_s=.8, a_ref_rad_s2=2., time_s=.005,
        source_time_s=0., expires_at_s=.02, frame=support.frame, configuration_id=baseline.context_id,
        trajectory_id="predeclared-fixture", generation=1, fresh=True, valid=True)
    with Controller(Native(library), params) as core:
        adapter = MotorFeedforward(core, model, support)
        adapter.reset(state, now=0., previous_current_A=0., accepted_time_s=-.1)
        out, _ = adapter.step(CObservation(.005, .005, .005, 0., 0., 1, 1, 1, 1, 1), ref,
                              replace(state, time_s=.005))
        assert out.sequence > 0 and core.ack(out)
        try:
            adapter.step(CObservation(.01, .01, .01, 0., 0., 2, 2, 1, 1, 1),
                         replace(ref, time_s=.01), replace(state, time_s=.01, configuration_facts=changed))
            raise AssertionError("unchanged label hid relocated payload")
        except FeedforwardRejected as exc:
            ff_fault = str(exc)
        inhibited = core.step(CObservation(.015, .015, .015, 0., 0., 3, 3, 1, 1, 1), CReference(0., .8, 0., 0.))
        assert (inhibited.status, inhibited.sequence, inhibited.requested, inhibited.limited) == (5, 0, 0., 0.)
        assert not adapter.armed
        ff = {"fault": ff_fault, "native_status": inhibited.status, "sequence": inhibited.sequence,
              "requested_A": inhibited.requested, "limited_A": inhibited.limited, "armed": adapter.armed}
    physical = context(provenance="MEASURED", status=FactStatus.VERIFIED)
    other = context(.2, provenance="MEASURED", status=FactStatus.VERIFIED)
    historical = ConfigurationSupport("old-supported-model", physical, tuple(physical.facts), qualification="PREDICTIVE_MODEL")
    try:
        supported_snapshot([historical], physical,
                           response_evidence=[ConfigurationResponse(historical.model_revision, other, True)])
        raise AssertionError("another configuration's response authorized rollback")
    except Rejected:
        wrong_context_rejected = True
    selected = supported_snapshot([historical], physical,
                                 response_evidence=[ConfigurationResponse(historical.model_revision, physical, True)])
    return {"schema": "adr0022.configuration-probe/1", "provenance": "SYNTHETIC_SOFTWARE_PROBE",
        "baseline_configuration": baseline.document(), "changed_configuration": changed.document(),
        "same_label_geometry": {"mass_kg": 2., "old_axis_inertia_kg_m2": .03+2*.1**2,
                                "changed_axis_inertia_kg_m2": .03+2*.2**2},
        "update": update, "conflicting_train": pooling,
        "same_context_different_procedures": aligned, "unknown_legacy": legacy,
        "native_ff_inhibition": ff, "wrong_context_response_rejected": wrong_context_rejected,
        "supported_historical_model": selected, "physical_3a": "NOT_RUN", "physical_3b": "NOT_RUN",
        "deployment_authorized": False, "passed": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("use a fresh output path")
    result = run_probe(args.library)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
