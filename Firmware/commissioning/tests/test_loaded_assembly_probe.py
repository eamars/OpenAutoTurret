"""Small contracts added after actual loaded native-pair runtime evidence."""
import math
from dataclasses import replace
import unittest

import numpy as np

from Firmware.commissioning.assembly_forecast import (AssemblyForecastRejected, CausalParent,
    CoupledControllerPair, JointReference, JointSupport)
from Firmware.commissioning.family_assets import native_parameter_document
from Firmware.commissioning.native import CObservation, Controller, Native
from Firmware.tools.adr0022_assembly_probe import fixture, zero_load
from Firmware.tools.adr0022_loaded_assembly_probe import (BALANCED_COM_M, CURRENT_CAP_A,
    GRAVITY_TABLE_COMMAND_ERROR_A, INITIAL_Q, Q_HIGH, Q_LOW, equilibrium_current,
    gravity_table_error, known_gravity_parameters, supplied_fixture, upright_gravity_table_bound)


class LoadedAssemblyProbeTests(unittest.TestCase):
    def test_original_and_balanced_current_envelopes_use_host_units(self):
        for balanced in (False, True):
            with self.subTest(balanced=balanced):
                assembly, actuator = supplied_fixture(balanced=balanced)
                cx, _, cz = assembly.pitch_payload.com_in_body_m
                torque = -2*9.81*(cx*math.cos(INITIAL_Q[1])+cz*math.sin(INITIAL_Q[1]))
                expected = np.array([-.01/2., (torque/.4+.02)/1.5])
                actual = equilibrium_current(assembly, actuator)['command_current_A']
                np.testing.assert_allclose(actual, expected, rtol=0, atol=2e-15)
                self.assertEqual(bool(np.all(abs(actual) <= CURRENT_CAP_A)), balanced)

    def test_separate_balance_fixture_preserves_supplied_mass_and_inertia(self):
        original = fixture()
        balanced, _ = supplied_fixture(balanced=True)
        self.assertNotEqual(balanced.configuration_id, original.configuration_id)
        self.assertEqual(balanced.pitch_payload.mass_kg, original.pitch_payload.mass_kg)
        for name in ('inertia_com_body_kg_m2', 'mount_translation_m', 'mount_rotation'):
            np.testing.assert_array_equal(getattr(balanced.pitch_payload, name), getattr(original.pitch_payload, name))
        np.testing.assert_array_equal(balanced.pitch_payload.com_in_body_m, BALANCED_COM_M)
        for name in ('gravity_world_m_s2', 'pitch_origin_in_yaw_m', 'base_rotation_world'):
            np.testing.assert_array_equal(getattr(balanced, name), getattr(original, name))

    def test_zero_torque_prehistory_is_not_a_loaded_rest_equilibrium(self):
        assembly, actuator = supplied_fixture(balanced=True)
        loads = zero_load(assembly)(0., INITIAL_Q, np.zeros(2))
        holding = equilibrium_current(assembly, actuator)['command_current_A']
        balanced_accel = assembly.acceleration(INITIAL_Q, np.zeros(2), holding, actuator,
            loads, now_s=0., max_load_age_s=0.)
        zero_torque_accel = assembly.acceleration(INITIAL_Q, np.zeros(2),
            actuator.command_for_torque(np.zeros(2))[1], actuator, loads, now_s=0., max_load_age_s=0.)
        self.assertLess(np.max(abs(balanced_accel)), 1e-14)
        self.assertGreater(abs(zero_torque_accel[1]), .1)

    def test_nominal_gravity_table_is_bounded_and_not_the_zero_gravity_table(self):
        assembly, actuator = supplied_fixture(balanced=True)
        parameters = known_gravity_parameters(assembly, actuator)
        self.assertLessEqual(max(gravity_table_error(assembly, actuator, parameters)), GRAVITY_TABLE_COMMAND_ERROR_A)
        for axis, parameter in enumerate(parameters):
            np.testing.assert_array_equal(parameter.model.theta[6:21], parameter.model.theta[21:36])
            np.testing.assert_array_equal(parameter.start_total[:30], parameter.model.theta[6:36])
            self.assertGreater(min(parameter.model.theta[:3]), 0.)
            self.assertEqual(parameter.model.theta[36], .0073)
        # Pitch gravity varies over the bounded table and has not been replaced
        # by the actuator bias cancellation used in the old zero-gravity probe.
        self.assertGreater(np.ptp(list(parameters[1].model.theta[6:11])), .0001)

    def test_loaded_transport_retains_known_equilibrium_until_actual_edge(self):
        assembly, actuator = supplied_fixture(balanced=True)
        initial = np.r_[INITIAL_Q, np.zeros(2)]
        holding = equilibrium_current(assembly, actuator)['command_current_A']
        parent = CausalParent(assembly, actuator, initial, zero_load(assembly), max_step_s=.00125,
            transport_delay_s=.0073, gyro_filter_tau_s=.012, max_load_age_s=0., prehistory_command_A=holding)
        parent.advance(.005)
        parent.accept(.005, holding+np.array([0., .01]))
        np.testing.assert_allclose(parent.advance(.012), initial, rtol=0, atol=1e-14)
        self.assertGreater(parent.advance(.015)[3], 1e-5)
        for source in (-.001, .016):
            with self.assertRaises(AssemblyForecastRejected):
                parent.gyro_at(source)

    def test_continuous_remainder_is_independent_of_sample_grid_and_rejects_other_gravity(self):
        assembly, actuator = supplied_fixture(balanced=True)
        params = known_gravity_parameters(assembly, actuator)
        remainder = upright_gravity_table_bound(assembly, actuator, params)
        expected_second = 2*9.81*math.hypot(.001, .0005)/(.4*1.5)
        expected = expected_second*((.45-.30)/4)**2/8
        self.assertAlmostEqual(remainder['pitch_command_remainder_A'], expected, places=16)
        self.assertLess(expected, GRAVITY_TABLE_COMMAND_ERROR_A)
        self.assertEqual(remainder['yaw_command_remainder_A'], 0.)
        self.assertLessEqual(gravity_table_error(assembly, actuator, params)[1], expected)
        with self.assertRaisesRegex(ValueError, 'upright geometry'):
            upright_gravity_table_bound(replace(assembly, gravity_world_m_s2=np.array([1.,0.,-9.81])), actuator, params)

    def test_actual_native_loaded_pair_keeps_gravity_balance_before_ACK(self):
        assembly, actuator = supplied_fixture(balanced=True)
        parameters = known_gravity_parameters(assembly, actuator)
        holding = equilibrium_current(assembly, actuator)['command_current_A']
        parent = CausalParent(assembly, actuator, np.r_[INITIAL_Q, np.zeros(2)], zero_load(assembly),
            max_step_s=.00125, transport_delay_s=.0073, gyro_filter_tau_s=.012, max_load_age_s=0.,
            prehistory_command_A=holding)
        parent.advance(.005)
        support = JointSupport(q_min_rad=Q_LOW, q_max_rad=Q_HIGH, velocity_max_rad_s=np.full(2, .8),
            acceleration_max_rad_s2=np.full(2, np.deg2rad(30.)), max_reference_source_age_s=.02,
            qualification='SYNTHETIC_OFFLINE')
        native = Native()
        with Controller(native, parameters[0]) as yaw, Controller(native, parameters[1]) as pitch:
            for axis, core in enumerate((yaw, pitch)):
                core.reset(1., INITIAL_Q[axis], 0., float(holding[axis]), accepted_time=.9)
                self.assertEqual(native_parameter_document(core.read_parameters()), native_parameter_document(parameters[axis]))
            pair = CoupledControllerPair((yaw, pitch), assembly, actuator, core_epoch_s=1.,
                frame='logical-output-joint-rad', trajectory_id='loaded-contract', max_load_age_s=0., support=support)
            observations = tuple(CObservation(1.005, 1.005, 1., INITIAL_Q[axis], 0., 1, 0, 1, 1, 0) for axis in range(2))
            ref = JointReference(q_ref_rad=INITIAL_Q, v_ref_rad_s=np.zeros(2), a_ref_rad_s2=np.zeros(2),
                time_s=.005, source_time_s=0., expires_at_s=.02, frame=pair.frame,
                configuration_id=assembly.configuration_id, trajectory_id=pair.trajectory,
                generation=1, fresh=True, valid=True)
            result = pair.step(observations, ref, zero_load(assembly)(.005, INITIAL_Q, np.zeros(2)))
            np.testing.assert_allclose([o.feedforward for o in result.outputs], holding, rtol=0, atol=1e-14)
            self.assertEqual(pair.receipts, [])
            self.assertEqual(pair.trace, ['VECTOR_FF', 'PITCH_OUTPUT', 'YAW_OUTPUT'])
            np.testing.assert_allclose(pair.acknowledge(result, record_receipt=parent.accept_receipt), holding, rtol=0, atol=1e-14)
            np.testing.assert_allclose(parent.commands[-1], holding, rtol=0, atol=1e-14)
            self.assertEqual(len(pair.receipts), 2)
            self.assertTrue(all(r.accepted_time_s == .005 for r in pair.receipts))


if __name__ == '__main__':
    unittest.main()
