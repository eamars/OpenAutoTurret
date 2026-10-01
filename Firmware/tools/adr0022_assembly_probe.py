"""Independent synthetic probes of the supplied two-joint Stage 1 parent model."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from Firmware.commissioning.assembly_dynamics import (ActuatorMap, CausalJointLoad, RigidBody,
    SerialAssembly, axis_inertia, axis_rotation)


def fixture():
    yaw = RigidBody(mass_kg=1., com_in_body_m=np.array([0., 0., .05]),
        inertia_com_body_kg_m2=np.diag([.02, .02, .015]), mount_translation_m=np.zeros(3), mount_rotation=np.eye(3))
    pitch = RigidBody(mass_kg=2., com_in_body_m=np.array([.3, .1, .05]),
        inertia_com_body_kg_m2=np.diag([.03, .05, .04]), mount_translation_m=np.zeros(3), mount_rotation=np.eye(3))
    return SerialAssembly(yaw_carriage=yaw, pitch_payload=pitch, yaw_axis_in_base=np.array([0., 0., 1.]),
        yaw_origin_in_base_m=np.zeros(3), pitch_axis_in_yaw=np.array([0., 1., 0.]),
        pitch_origin_in_yaw_m=np.array([.1, 0., .2]), base_rotation_world=np.eye(3),
        base_origin_world_m=np.zeros(3), gravity_world_m_s2=np.array([0., 0., -9.81]),
        configuration_id="SYNTHETIC_TWO_JOINT_FIXTURE")


def analytic_oracle(q, base_tilt=0.):
    # Closed-form mechanics derived independently for this particular fixture.
    yaw, pitch = q
    x = .3*math.cos(pitch)+.05*math.sin(pitch)
    z = -.3*math.sin(pitch)+.05*math.cos(pitch)
    mass = np.array([[.015+2*((.1+x)**2+.1**2)+.03*math.sin(pitch)**2+.04*math.cos(pitch)**2, -.2*z],
                     [-.2*z, .235]])
    gravity = np.array([19.62*math.sin(base_tilt)*(math.cos(yaw)*(.1+x)-.1*math.sin(yaw)),
        19.62*(math.sin(base_tilt)*math.sin(yaw)*z-math.cos(base_tilt)*x)])
    return mass, gravity


def zero_load(assembly):
    return lambda t, q, v: CausalJointLoad(cable_torque_Nm=np.zeros(2), friction_torque_Nm=np.zeros(2),
        external_load_torque_Nm=np.zeros(2), time_s=t, configuration_id=assembly.configuration_id, valid=True)


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    return value


def run_probes():
    assembly = fixture()
    q, v = np.array([.3, .4]), np.array([.6, -.2])
    d = assembly.dynamics(q, v)
    expected_m, expected_g = analytic_oracle(q)
    tilted = replace(assembly, base_rotation_world=axis_rotation(np.array([1., 0., 0.]), math.radians(20.)))
    dt = tilted.dynamics(q, v)
    tilted_m, tilted_g = analytic_oracle(q, math.radians(20.))
    oracle_error = max(np.max(np.abs(d.M_kg_m2-expected_m)), np.max(np.abs(d.G_Nm-expected_g)),
                       np.max(np.abs(dt.M_kg_m2-tilted_m)), np.max(np.abs(dt.G_Nm-tilted_g)))
    assert oracle_error < 1e-12
    small = RigidBody(mass_kg=2., com_in_body_m=np.zeros(3), inertia_com_body_kg_m2=np.eye(3)*.01,
        mount_translation_m=np.array([.1, 0., 0.]), mount_rotation=np.eye(3))
    moved = replace(small, mount_translation_m=np.array([.4, 0., 0.]))
    relocation = axis_inertia(moved, [0., 0., 1.], [0., 0., 0.])-axis_inertia(small, [0., 0., 1.], [0., 0., 0.])
    assert abs(relocation-.3) < 1e-14

    derivative_error = gravity_error = energy_identity_error = 0.
    for position, velocity in ((q, v), (np.array([8*math.pi+.3, -.7]), np.array([-.4, .7])),
                               (np.array([-6*math.pi-.8, 1.2]), np.array([.8, .2]))):
        actual = tilted.dynamics(position, velocity)
        for k in range(2):
            h = np.eye(2)[k]*1e-6
            plus, minus = tilted.dynamics(position+h, velocity), tilted.dynamics(position-h, velocity)
            derivative_error = max(derivative_error, float(np.max(np.abs((plus.M_kg_m2-minus.M_kg_m2)/2e-6-actual.dM_dq[k]))))
            gravity_error = max(gravity_error, abs((plus.potential_J-minus.potential_J)/2e-6-actual.G_Nm[k]))
        mdot = np.einsum("k,kij->ij", velocity, actual.dM_dq)
        skew_check = mdot-2*actual.C_kg_m2_s
        energy_identity_error = max(energy_identity_error, float(np.max(np.abs(skew_check+skew_check.T))))
    assert derivative_error < 1e-8 and gravity_error < 1e-8 and energy_identity_error < 1e-12

    actuator = ActuatorMap(torque_per_effective_amp_Nm=np.array([.5, -.4]),
        command_gain=np.array([2., 1.5]), command_bias_A=np.array([.01, -.02]))
    loads = CausalJointLoad(cable_torque_Nm=np.array([.03, -.02]), friction_torque_Nm=np.array([.01, .02]),
        external_load_torque_Nm=np.array([-.04, .01]), time_s=0., configuration_id=tilted.configuration_id, valid=True)
    target_acceleration = np.array([.2, -.3])
    demand = tilted.current_demand(q, v, target_acceleration, actuator, loads, now_s=0., max_load_age_s=0.)
    recovered = tilted.acceleration(q, v, demand["command_current_A"], actuator, loads, now_s=0., max_load_age_s=0.)
    inverse_error = float(np.max(np.abs(recovered-target_acceleration)))
    assert inverse_error < 1e-12

    unit_map = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))
    trajectories = []
    for step in (.004, .002):
        times = np.arange(round(1./step)+1)*step
        trajectory = tilted.rollout(times, np.zeros((len(times)-1, 2)), q, v, unit_map, zero_load(tilted), max_load_age_s=0.)
        energy = np.array([x.kinetic_J+x.potential_J for x in (tilted.dynamics(row[:2], row[2:]) for row in trajectory)])
        trajectories.append((times, trajectory, energy))
    energy_drift = float(np.max(np.abs(trajectories[1][2]-trajectories[1][2][0])))
    step_error = float(np.max(np.abs(trajectories[0][1]-trajectories[1][1][::2])))
    assert energy_drift < 1e-6 and step_error < 1e-6

    point_yaw = RigidBody(mass_kg=1., com_in_body_m=np.zeros(3), inertia_com_body_kg_m2=np.eye(3)*.1,
        mount_translation_m=np.zeros(3), mount_rotation=np.eye(3))
    point_pitch = replace(point_yaw, inertia_com_body_kg_m2=np.eye(3)*.2)
    constant = replace(assembly, yaw_carriage=point_yaw, pitch_payload=point_pitch,
                       pitch_origin_in_yaw_m=np.zeros(3), gravity_world_m_s2=np.zeros(3))
    times = np.linspace(0., 1., 101)
    initial_q, initial_v = np.array([8*math.pi+.1, -.3]), np.array([.2, -.1])
    independent = constant.rollout(times, np.tile([.3, .4], (100, 1)), initial_q, initial_v, unit_map,
                                  zero_load(constant), max_load_age_s=0.)
    expected_q = initial_q+times[:, None]*initial_v+.5*times[:, None]**2*np.array([1., 2.])
    expected_v = initial_v+times[:, None]*np.array([1., 2.])
    constant_error = float(np.max(np.abs(independent-np.c_[expected_q, expected_v])))
    assert constant_error < 1e-12
    report = {"execution": "SYNTHETIC_LOCAL_ONLY", "qualification": "RUNNABLE_MATHEMATICS_ONLY",
        "scope": "two-revolute rigid assembly; supplied SI masses/inertia/mounting and causal loads",
        "fixture_contract": serializable(asdict(assembly)),
        "probe_initial_q_rad": q.tolist(), "probe_initial_velocity_rad_s": v.tolist(),
        "tilted_base_rotation_world": tilted.base_rotation_world.tolist(),
        "inverse_probe_actuator_map": serializable(asdict(actuator)),
        "conservative_rollout_contract": {"duration_s": 1., "steps_s": [.004, .002],
            "command_current_A": [0., 0.], "effective_torque_per_amp_Nm": [1., 1.],
            "command_gain": [1., 1.], "command_bias_A": [0., 0.],
            "cable_friction_external_load_Nm": [0., 0.]},
        "independent_fixture_oracle_max_error": float(oracle_error),
        "same_mass_relocated_axis_inertia_change_kg_m2": float(relocation),
        "M_kg_m2": d.M_kg_m2.tolist(), "vertical_yaw_gravity_Nm": d.G_Nm.tolist(),
        "tilted_base_gravity_Nm": dt.G_Nm.tolist(), "cross_coupling_kg_m2": float(d.M_kg_m2[0, 1]),
        "mass_derivative_finite_difference_max_error": derivative_error,
        "gravity_potential_gradient_max_error_Nm": gravity_error,
        "Mdot_minus_2C_skew_identity_max_error": energy_identity_error,
        "inverse_forward_acceleration_max_error_rad_s2": inverse_error,
        "conservative_rollout_energy_max_drift_J": energy_drift,
        "step_refinement_max_state_error": step_error,
        "independent_constant_inertia_rollout_max_error": constant_error,
        "unwrapped_yaw_initial_rad": float(initial_q[0]),
        "physical_identification": "NOT_RUN", "native_controller_coupling": "NOT_RUN",
        "production_adoption": "NOT_RUN", "fixed_pitch_yaw_family": "UNQUALIFIED_BOUNDED_REDUCTION"}
    data = {"t_s": trajectories[1][0], "q_v": trajectories[1][1], "energy_J": trajectories[1][2]}
    return report, data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if any(args.output_dir.iterdir()):
        parser.error("preserve prior evidence: choose an empty output directory")
    report, data = run_probes()
    (args.output_dir/"assembly-probe.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    np.savez_compressed(args.output_dir/"assembly-rollout.npz", **data)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
