"""Independent mechanics and domain checks for the supplied two-axis parent."""
from dataclasses import replace
import math
import unittest

import numpy as np

from Firmware.commissioning.assembly_dynamics import (ActuatorMap, AssemblyInvalid, CausalJointLoad,
    RigidBody, axis_inertia, axis_rotation)
from Firmware.tools.adr0022_assembly_probe import analytic_oracle, fixture, zero_load


class AssemblyDynamicsTests(unittest.TestCase):
    def setUp(self):
        self.assembly = fixture()
        self.q, self.v = np.array([.3, .4]), np.array([.6, -.2])
        self.unit_map = ActuatorMap(torque_per_effective_amp_Nm=np.ones(2), command_gain=np.ones(2), command_bias_A=np.zeros(2))

    def test_closed_form_fixture_and_general_base_tilt(self):
        for tilt in (0., math.radians(20.)):
            model = replace(self.assembly, base_rotation_world=axis_rotation([1., 0., 0.], tilt))
            for q in (self.q, np.array([8*math.pi+.3, -.7]), np.array([-.8, 1.2])):
                d = model.dynamics(q, self.v)
                expected_m, expected_g = analytic_oracle(q, tilt)
                np.testing.assert_allclose(d.M_kg_m2, expected_m, rtol=0, atol=1e-14)
                np.testing.assert_allclose(d.G_Nm, expected_g, rtol=0, atol=1e-14)
                if tilt == 0:
                    self.assertEqual(d.G_Nm[0], 0.)
        self.assertGreater(abs(self.assembly.dynamics(self.q, self.v).M_kg_m2[0, 1]), .01)

    def test_parallel_axis_mass_relocation_and_rotated_com_tensor(self):
        body = RigidBody(mass_kg=2., com_in_body_m=np.zeros(3), inertia_com_body_kg_m2=np.eye(3)*.01,
            mount_translation_m=np.array([.1, 0., 0.]), mount_rotation=np.eye(3))
        moved = replace(body, mount_translation_m=np.array([.4, 0., 0.]))
        self.assertAlmostEqual(axis_inertia(moved, [0., 0., 1.], [0., 0., 0.])-
                               axis_inertia(body, [0., 0., 1.], [0., 0., 0.]), .3)
        r = axis_rotation([.3, .4, .1], .7)
        body = replace(body, com_in_body_m=np.array([.02, -.04, .07]),
            inertia_com_body_kg_m2=np.diag([.03, .04, .05]), mount_rotation=r)
        axis, origin = np.array([1., 2., 3.]), np.array([.03, .01, -.02])
        e = axis/np.linalg.norm(axis)
        com = body.mount_translation_m+r@body.com_in_body_m-origin
        expected = e@(r@body.inertia_com_body_kg_m2@r.T)@e+body.mass_kg*np.linalg.norm(np.cross(e, com))**2
        self.assertAlmostEqual(axis_inertia(body, axis, origin), expected)

    def test_arbitrary_joint_axes_mounts_and_analytic_derivatives(self):
        model = replace(self.assembly, yaw_axis_in_base=np.array([1., .2, .4]), pitch_axis_in_yaw=np.array([-.1, .8, .6]),
            yaw_origin_in_base_m=np.array([.02, -.01, .05]), pitch_origin_in_yaw_m=np.array([.05, .03, -.08]),
            base_rotation_world=axis_rotation([.2, -.3, .7], .8),
            yaw_carriage=replace(self.assembly.yaw_carriage, mount_rotation=axis_rotation([.1, .5, .2], .4)),
            pitch_payload=replace(self.assembly.pitch_payload, mount_rotation=axis_rotation([-.2, .2, .9], -.7),
                                  mount_translation_m=np.array([-.03, .01, .04])))
        actual = model.dynamics(self.q, self.v)
        finite_derivatives = []
        for k in range(2):
            h = np.eye(2)[k]*1e-6
            plus, minus = model.dynamics(self.q+h, self.v), model.dynamics(self.q-h, self.v)
            derivative = (plus.M_kg_m2-minus.M_kg_m2)/2e-6
            finite_derivatives.append(derivative)
            np.testing.assert_allclose(actual.dM_dq[k], derivative, rtol=0, atol=1e-9)
            self.assertAlmostEqual(actual.G_Nm[k], (plus.potential_J-minus.potential_J)/2e-6, delta=1e-8)
        # Independent finite-difference Euler-Lagrange inertial force.
        derivatives = np.array(finite_derivatives)
        mdot = np.einsum("k,kij->ij", self.v, derivatives)
        kinetic_gradient = np.array([.5*self.v@derivatives[k]@self.v for k in range(2)])
        np.testing.assert_allclose(actual.C_kg_m2_s@self.v, mdot@self.v-kinetic_gradient, rtol=0, atol=1e-9)
        difference = np.einsum("k,kij->ij", self.v, actual.dM_dq)-2*actual.C_kg_m2_s
        np.testing.assert_allclose(difference+difference.T, np.zeros((2, 2)), rtol=0, atol=1e-14)

    def test_world_translation_changes_only_potential_datum(self):
        moved = replace(self.assembly, base_origin_world_m=np.array([10., -20., 3.]))
        a, b = self.assembly.dynamics(self.q, self.v), moved.dynamics(self.q, self.v)
        np.testing.assert_allclose(a.M_kg_m2, b.M_kg_m2, rtol=0, atol=1e-14)
        np.testing.assert_allclose(a.C_kg_m2_s, b.C_kg_m2_s, rtol=0, atol=1e-14)
        np.testing.assert_allclose(a.G_Nm, b.G_Nm, rtol=0, atol=1e-13)
        self.assertAlmostEqual(b.potential_J-a.potential_J, 3.*9.81*3.)

    def test_supplied_signed_actuator_map_and_explicit_joint_loads(self):
        actuator = ActuatorMap(torque_per_effective_amp_Nm=np.array([.5, -.4]), command_gain=np.array([2., 1.5]),
            command_bias_A=np.array([.01, -.02]))
        loads = CausalJointLoad(cable_torque_Nm=np.array([.03, -.02]), friction_torque_Nm=np.array([.01, .02]),
            external_load_torque_Nm=np.array([-.04, .01]), time_s=0., configuration_id=self.assembly.configuration_id, valid=True)
        a = np.array([.2, -.3])
        demand = self.assembly.current_demand(self.q, self.v, a, actuator, loads, now_s=0., max_load_age_s=0.)
        dynamics = self.assembly.dynamics(self.q, self.v)
        expected = dynamics.M_kg_m2@a+dynamics.C_kg_m2_s@self.v+dynamics.G_Nm+np.array([0., .01])
        np.testing.assert_allclose(demand["torque_Nm"], expected, rtol=0, atol=1e-14)
        np.testing.assert_allclose(actuator.torque_for_command(demand["command_current_A"]), expected, rtol=0, atol=1e-14)
        recovered = self.assembly.acceleration(self.q, self.v, demand["command_current_A"], actuator, loads, now_s=0., max_load_age_s=0.)
        np.testing.assert_allclose(recovered, a, rtol=0, atol=1e-14)

    def test_missing_invalid_future_stale_or_changed_configuration_loads_rejected(self):
        loads = zero_load(self.assembly)(0., self.q, self.v)
        for candidate in (None, replace(loads, valid=False), replace(loads, time_s=.1),
                          replace(loads, time_s=-1.), replace(loads, configuration_id="changed")):
            with self.subTest(candidate=candidate):
                with self.assertRaises(AssemblyInvalid):
                    self.assembly.current_demand(self.q, self.v, [0., 0.], self.unit_map, candidate, now_s=0., max_load_age_s=.01)

    def test_unwrapped_winding_load_and_independent_forward_solution(self):
        body = RigidBody(mass_kg=1., com_in_body_m=np.zeros(3), inertia_com_body_kg_m2=np.eye(3)*.1,
            mount_translation_m=np.zeros(3), mount_rotation=np.eye(3))
        model = replace(self.assembly, yaw_carriage=body, pitch_payload=replace(body, inertia_com_body_kg_m2=np.eye(3)*.2),
            pitch_origin_in_yaw_m=np.zeros(3), gravity_world_m_s2=np.zeros(3))
        initial = np.array([4*math.pi+.1, .2])
        seen = []
        def cable(t, q, v):
            seen.append(q[0])
            return CausalJointLoad(cable_torque_Nm=np.array([.1*q[0], 0.]), friction_torque_Nm=np.zeros(2),
                external_load_torque_Nm=np.zeros(2), time_s=t, configuration_id=model.configuration_id, valid=True)
        t = np.linspace(0., .5, 101)
        trace = model.rollout(t, np.zeros((100, 2)), initial, [0., 0.], self.unit_map, cable, max_load_age_s=0.)
        rate = math.sqrt(.1/.3)
        np.testing.assert_allclose(trace[:, 0], initial[0]*np.cos(rate*t), rtol=0, atol=1e-11)
        np.testing.assert_allclose(trace[:, 2], -initial[0]*rate*np.sin(rate*t), rtol=0, atol=1e-11)
        self.assertGreater(min(seen), 2*math.pi)

    def test_invalid_geometry_units_or_actuator_scale_rejected(self):
        for axis in ([0., 0., 0.], [True, False, False], ["1", "0", "0"]):
            with self.assertRaises(AssemblyInvalid):
                replace(self.assembly, yaw_axis_in_base=np.array(axis))
        for inertia in (np.diag([10., 1., 1.]), np.diag([-.1, .1, .1]), np.array([[.1, .01, 0.], [0., .1, 0.], [0., 0., .1]])):
            with self.assertRaises(AssemblyInvalid):
                replace(self.assembly.yaw_carriage, inertia_com_body_kg_m2=inertia)
        with self.assertRaises(AssemblyInvalid):
            replace(self.assembly, base_rotation_world=np.diag([1., 1., -1.]))
        with self.assertRaises(AssemblyInvalid):
            ActuatorMap(torque_per_effective_amp_Nm=np.array([0., 1.]), command_gain=np.ones(2), command_bias_A=np.zeros(2))
        with self.assertRaises(AssemblyInvalid):
            self.assembly.dynamics([float("nan"), 0.], [0., 0.])


if __name__ == "__main__":
    unittest.main()
