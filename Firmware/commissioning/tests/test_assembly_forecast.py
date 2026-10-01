"""Focused real-native contracts added after the coupled runtime probe passed."""
from dataclasses import replace
import unittest

import numpy as np

from Firmware.commissioning.assembly_dynamics import ActuatorMap
from Firmware.commissioning.assembly_forecast import (AssemblyForecastRejected, CausalParent,
    CoupledControllerPair, JointReference)
from Firmware.commissioning.native import CObservation, Controller, Native
from Firmware.tools.adr0022_assembly_probe import analytic_oracle, fixture, zero_load
from Firmware.tools.adr0022_assembly_forecast_probe import (fault_probe, filter_oracle_probe,
    native_fixture, partial_ack_probe, support_fixture, vector_plan)


class AssemblyForecastTests(unittest.TestCase):
    def setUp(self):
        self.native = Native()
        self.assembly = replace(fixture(), gravity_world_m_s2=np.zeros(3))
        self.initial = np.array([.3, .4, 0., 0.])
        self.actuator = ActuatorMap(torque_per_effective_amp_Nm=np.array([.5, .4]),
            command_gain=np.array([2., 1.5]), command_bias_A=np.array([.01, -.02]))

    def test_vector_reference_is_immutable_and_detached(self):
        supplied = self.initial[:2].copy()
        ref = vector_plan(.005, supplied, move_s=2., duration_s=3.,
                          configuration_id=self.assembly.configuration_id)
        supplied[0] = .8
        self.assertEqual(ref.q_ref_rad[0], .3)
        with self.assertRaises(ValueError):
            ref.q_ref_rad.flags.writeable = True
        with self.assertRaises(ValueError):
            ref.v_ref_rad_s[0] = .1

    def test_one_vector_inverse_uses_coupling_and_host_command_units(self):
        params = native_fixture(self.assembly, self.initial[:2], self.actuator)
        prehistory = self.actuator.command_for_torque(np.zeros(2))[1]
        parent = CausalParent(self.assembly, self.actuator, self.initial, zero_load(self.assembly),
            max_step_s=.00125, transport_delay_s=0., gyro_filter_tau_s=0., max_load_age_s=0.,
            prehistory_command_A=prehistory)
        parent.advance(.005)
        with Controller(self.native, params[0]) as yaw, Controller(self.native, params[1]) as pitch:
            for axis, core in enumerate((yaw, pitch)):
                core.reset(1., self.initial[axis], 0., float(prehistory[axis]), accepted_time=.9)
            pair = CoupledControllerPair((yaw, pitch), self.assembly, self.actuator, core_epoch_s=1.,
                frame="logical-output-joint-rad", trajectory_id="coupled-units-probe", max_load_age_s=0.,
                support=support_fixture(.1))
            ref = JointReference(q_ref_rad=self.initial[:2], v_ref_rad_s=np.zeros(2),
                a_ref_rad_s2=np.array([.05, -.04]), time_s=.005, source_time_s=0., expires_at_s=.1,
                frame=pair.frame, configuration_id=self.assembly.configuration_id, trajectory_id=pair.trajectory,
                generation=1, fresh=True, valid=True)
            observations = tuple(CObservation(1.005, 1.005, 1.005, self.initial[k], 0., 1, 1, 1, 1, 1)
                                 for k in range(2))
            result = pair.step(observations, ref, zero_load(self.assembly)(.005, self.initial[:2], np.zeros(2)))
            expected_mass, _ = analytic_oracle(self.initial[:2])
            expected_torque = expected_mass@ref.a_ref_rad_s2
            expected_host = (expected_torque/np.array([.5, .4])-np.array([.01, -.02]))/np.array([2., 1.5])
            np.testing.assert_allclose([o.feedforward for o in result.outputs], expected_host, rtol=0, atol=1e-14)
            self.assertGreater(np.max(np.abs(expected_torque-np.diag(expected_mass)*ref.a_ref_rad_s2)), 1e-4)
            self.assertEqual(pair.trace, ["VECTOR_FF", "PITCH_OUTPUT", "YAW_OUTPUT"])
            self.assertEqual(len(pair.receipts), 0)
            np.testing.assert_array_equal(parent.commands[-1], prehistory)
            accepted = pair.acknowledge(result, record_receipt=parent.accept_receipt)
            self.assertEqual(len(pair.receipts), 2)
            np.testing.assert_array_equal(parent.commands[-1], accepted)
            for receipt in pair.receipts:
                self.assertEqual(receipt.command_current_A, result.outputs[receipt.axis].limited)
                self.assertAlmostEqual(receipt.effective_current_A,
                    [2., 1.5][receipt.axis]*receipt.command_current_A+[.01, -.02][receipt.axis], places=14)

    def test_native_and_reference_faults_inhibit_both_before_ACK(self):
        for case in ("pitch-native-nonfinite", "yaw-native-stale", "paired-source-clock", "invalid-load",
                     "reference-context", "reference-source-age", "reference-position-envelope",
                     "reference-velocity-envelope", "reference-acceleration-envelope"):
            with self.subTest(case=case):
                result = fault_probe(self.native, case)
                self.assertTrue(result["passed"])
                self.assertFalse(result["synthetic_current_accepted"])

    def test_partial_native_ACK_preserves_actual_yaw_current(self):
        result = partial_ack_probe(self.native)
        self.assertTrue(result["passed"])
        self.assertEqual(len(result["successful_receipts"]), 1)
        self.assertGreater(result["retained_input_A"][0], 0.)
        self.assertEqual(result["retained_input_A"][1], 0.)

    def test_piecewise_offmesh_filter_has_independent_analytic_support(self):
        result = filter_oracle_probe()
        self.assertTrue(result["passed"])
        self.assertTrue(result["all_transport_breakpoints_retained"])
        self.assertLess(result["maximum_filter_knot_error"], 1e-12)
        self.assertTrue(all(q["observed_error_rad_s"] <= q["analytic_interpolation_bound_rad_s"]+1e-12
                            for q in result["delayed_sensor_queries"]))

    def test_delayed_sensor_cannot_read_future_or_preinitial_state(self):
        prehistory = self.actuator.command_for_torque(np.zeros(2))[1]
        parent = CausalParent(self.assembly, self.actuator, self.initial, zero_load(self.assembly),
            max_step_s=.00125, transport_delay_s=.0073, gyro_filter_tau_s=.012, max_load_age_s=0.,
            prehistory_command_A=prehistory)
        parent.advance(.005)
        for source in (-.001, .0051):
            with self.assertRaises(AssemblyForecastRejected):
                parent.gyro_at(source)


if __name__ == "__main__":
    unittest.main()
