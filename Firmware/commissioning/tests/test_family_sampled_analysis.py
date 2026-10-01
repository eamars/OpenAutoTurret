"""Contracts and independent loop identities after native prefix verification."""
from dataclasses import replace
import math
import unittest

import numpy as np

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.family_sampled_analysis import LocalGains, controller_tick, observer_tick
from Firmware.commissioning.synthesis import selected_family_sampled_analysis
from Firmware.tools.adr0022_family_sampled_probe import family_fixture


class FamilySampledTests(unittest.TestCase):
    def fixture(self, **structure):
        model, point, support, params, schedule, gains, policy = family_fixture(1, **structure)
        analysis = selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule, ff_policy=policy)
        return model, point, support, params, schedule, gains, policy, analysis

    def test_native_order_output_uses_posterior_and_old_integral_before_ack(self):
        gains = LocalGains(.5, .8, 1.2, 3.)
        state, output = controller_tick(np.array([0., 0., .3, -.1]), np.array([.2, .1]), .005,
            gains, np.eye(2), np.eye(2), command_ff=.04)
        error = -.34
        self.assertAlmostEqual(output[-1], .5*error+.3+.04)
        self.assertAlmostEqual(output[2], .3)
        self.assertAlmostEqual(state[2], .3+.8*.005*(error-.1)/2)
        self.assertAlmostEqual(state[3], error)

    def test_old_gyro_measurement_is_not_reused_as_fresh(self):
        *_, analysis = self.fixture()
        state = np.zeros(analysis.dimension)
        _, held, _ = analysis.closed_step(state, 1, measurement_delta=(0., 1.))
        _, fresh, _ = analysis.closed_step(state, 0, measurement_delta=(0., 1.))
        np.testing.assert_array_equal(held, np.zeros(4))
        self.assertGreater(abs(fresh[1]), .001)

    def test_affine_ff_uses_same_new_posterior_q(self):
        model, point, support, params, schedule, gains, _, shared = self.fixture(affine=True)
        frozen = selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule,
            ff_policy="FROZEN_COMMAND_OFFSET")
        state = np.zeros(shared.dimension)
        _, a, _ = shared.open_step(state, 0., 0, measurement_delta=(1., 0.))
        _, b, _ = frozen.open_step(state, 0., 0, measurement_delta=(1., 0.))
        self.assertGreater(a[0], 0.)
        self.assertAlmostEqual(a[-1]-b[-1], model.load_slope/model.actuator_gain*a[0], places=14)

    def test_fractional_transport_and_sensor_delays_match_closed_form_impulse(self):
        model, *_, analysis = self.fixture(fractional=True)
        state = np.zeros(analysis.dimension)
        state, _, _ = analysis.open_step(state, 1., 0)
        self.assertEqual(state[0], 0.)
        self.assertEqual(state[1], 0.)
        state, _, _ = analysis.open_step(state, 0., 1)
        def response(at):
            rate = model.viscous/model.a
            velocity = model.actuator_gain/model.viscous*(-math.expm1(-rate*at))
            position = model.actuator_gain/model.viscous*(at+math.expm1(-rate*at)/rate)
            return position, velocity
        q, v = response(2*analysis.dt-model.transport_delay)
        self.assertAlmostEqual(state[0], q, places=13)
        self.assertAlmostEqual(state[1], v, places=13)
        measured = analysis.measurements(state)
        self.assertAlmostEqual(measured[1], response(2*analysis.dt-model.transport_delay-model.gyro_delay)[1], places=13)
        self.assertAlmostEqual(measured[2], model.current_gain*model.actuator_gain, places=14)

    def test_full_actuator_and_both_sensor_dynamic_states_are_retained(self):
        *_, analysis = self.fixture(dynamic=True, filtered=True, fractional=True)
        self.assertEqual(analysis.local.state_names, ("q_rad", "v_rad_s", "effective_current_A", "gyro_filter_rad_s", "current_filter_A"))
        self.assertEqual(analysis.ff_state_q, 0.)
        np.testing.assert_array_equal(analysis.ff_reference, np.zeros(3))
        self.assertEqual(analysis.ff_policy, "FROZEN_COMMAND_OFFSET")

    def test_dynamic_ff_inversion_is_not_silently_enabled(self):
        model, point, support, params, schedule, gains, _, _ = self.fixture(dynamic=True)
        with self.assertRaises(Rejected):
            selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule,
                ff_policy="SHARED_POSTERIOR_SLIDE_ALGEBRAIC")

    def test_steady_state_dynamic_ff_uses_posterior_and_reference_gradients(self):
        model, point, support, params, schedule, gains, _, shared = self.fixture(dynamic=True,
            affine=True, filtered=True, fractional=True, stribeck=True,
            ff_policy="SHARED_POSTERIOR_STEADY_STATE_REFERENCE")
        frozen = selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule,
            ff_policy="FROZEN_COMMAND_OFFSET")
        state = np.zeros(shared.dimension)
        reference = np.array([.3, -.2, .7])
        _, a, _ = shared.open_step(state, 0., 0, measurement_delta=(1., .4), reference_delta=reference)
        _, b, _ = frozen.open_step(state, 0., 0, measurement_delta=(1., .4), reference_delta=reference)
        expected = (model.load_slope*a[0]+shared.local.incremental_damping_A_s_rad*reference[1]+
            model.a*reference[2])/model.actuator_gain
        self.assertAlmostEqual(a[-1]-b[-1], expected, places=14)
        self.assertLess(shared.ff_reference[1], 0.)
        self.assertEqual(shared.dimension, frozen.dimension)
        self.assertEqual(shared.local.state_names, frozen.local.state_names)

    def test_steady_state_policy_keeps_actuator_pole_and_transport_forward(self):
        model, *_, analysis = self.fixture(dynamic=True, fractional=True,
            ff_policy="SHARED_POSTERIOR_STEADY_STATE_REFERENCE")
        state, _, _ = analysis.open_step(np.zeros(analysis.dimension), 1., 0)
        np.testing.assert_array_equal(state[:3], np.zeros(3))
        state, _, _ = analysis.open_step(state, 0., 1)
        duration = 2*analysis.dt-model.transport_delay
        expected = model.actuator_gain*(-math.expm1(-duration/model.actuator_tau))
        self.assertAlmostEqual(state[2], expected, places=14)
        self.assertLess(state[2], model.actuator_gain)
        self.assertEqual(analysis.document()["ff_dynamic_policy"],
            "STEADY_STATE_COMMAND_WITH_FORWARD_ACTUATOR_DYNAMICS")

    def test_new_policy_preserves_legacy_algebraic_shared_matrices(self):
        model, point, support, params, schedule, gains, _, legacy = self.fixture(affine=True)
        steady = selected_family_sampled_analysis(model, point, support, params.observer, gains, schedule,
            ff_policy="SHARED_POSTERIOR_STEADY_STATE_REFERENCE")
        for before, after in zip(legacy.phases, steady.phases):
            for original, additive in zip(before, after):
                np.testing.assert_array_equal(original, additive)

    def test_quantization_active_limits_and_delayed_ack_reject_local_derivative(self):
        model, point, support, params, schedule, gains, policy, _ = self.fixture()
        for changed in (replace(schedule, encoder_quantum_rad=.001), replace(schedule, gyro_quantum_rad_s=.002),
                        replace(schedule, limits_inactive=False), replace(schedule, immediate_successful_ack=False)):
            with self.assertRaises(Rejected):
                selected_family_sampled_analysis(model, point, support, params.observer, gains, changed, ff_policy=policy)

    def test_freshness_bounds_and_boolean_cadence_are_rejected(self):
        model, point, support, params, schedule, gains, policy, _ = self.fixture()
        for changed in (replace(schedule, gyro_period=9), replace(schedule, encoder_age_s=.04),
                        replace(schedule, gyro_period=True), replace(schedule, dt_s=True)):
            with self.assertRaises(Rejected):
                selected_family_sampled_analysis(model, point, support, params.observer, gains, changed, ff_policy=policy)

    def test_gyro_source_age_combines_availability_and_signal_delay_once(self):
        model, _, _, _, schedule, _, _, analysis = self.fixture(fractional=True, aged=True)
        self.assertAlmostEqual(analysis.gyro_source_age_s, .00530)
        self.assertEqual(analysis.delays[1], schedule.gyro_availability_age_s+model.gyro_delay)
        document = analysis.document()
        self.assertEqual(document["gyro_source_age_s"], analysis.delays[1])
        self.assertEqual(document["gyro_signal_delay_s"], model.gyro_delay)

    def test_gyro_source_age_controls_covariance_and_freshness(self):
        model, point, support, params, schedule, gains, policy, analysis = self.fixture(fractional=True, aged=True)
        F, K, _ = observer_tick(params.observer, schedule.dt_s, analysis.periodic_covariance,
            encoder_fresh=True, gyro_fresh=True, encoder_age_s=schedule.encoder_age_s,
            gyro_age_s=analysis.gyro_source_age_s)
        np.testing.assert_allclose(analysis.observer_phases[0][0], F, rtol=1e-10, atol=1e-14)
        np.testing.assert_allclose(analysis.observer_phases[0][1], K, rtol=1e-10, atol=1e-14)
        _, availability_K, _ = observer_tick(params.observer, schedule.dt_s, analysis.periodic_covariance,
            encoder_fresh=True, gyro_fresh=True, encoder_age_s=schedule.encoder_age_s,
            gyro_age_s=schedule.gyro_availability_age_s)
        self.assertGreater(np.max(abs(K-availability_K)), .001)
        changed = replace(schedule, gyro_period=6)
        self.assertLess(schedule.gyro_availability_age_s+5*schedule.dt_s, params.observer.max_gyro_age_s)
        with self.assertRaises(Rejected):
            selected_family_sampled_analysis(model, point, support, params.observer, gains, changed, ff_policy=policy)

    def test_preinitial_gyro_source_is_not_consumed_by_transient_observer(self):
        _, _, _, params, schedule, _, _, analysis = self.fixture(fractional=True, aged=True)
        source_time = schedule.dt_s-analysis.gyro_source_age_s
        self.assertLess(source_time, 0.)
        F, K, _ = observer_tick(params.observer, schedule.dt_s,
            np.diag([params.observer.initial_position_variance, params.observer.initial_velocity_variance]),
            encoder_fresh=True, gyro_fresh=source_time >= 0., encoder_age_s=schedule.encoder_age_s,
            gyro_age_s=analysis.gyro_source_age_s)
        _, output, _ = analysis.closed_step(np.zeros(analysis.dimension), 1,
            observer_pair=(F, K), measurement_delta=(0., 1.))
        np.testing.assert_array_equal(output, np.zeros(4))

    def test_failed_gains_are_reported_without_valid_margin_sentinels(self):
        model, point, support, params, schedule, _, policy, _ = self.fixture()
        bad = selected_family_sampled_analysis(model, point, support, params.observer, LocalGains(.01, .8, 1.2, 3.), schedule, ff_policy=policy)
        result = bad.margin_diagnostics()
        self.assertFalse(result["passed"])
        self.assertEqual(result["status"], "NOMINAL_LOCAL_TANGENT_UNSTABLE")
        self.assertIsNone(result["gain_margin_db"]); self.assertIsNone(result["phase_margin_deg"])

    def test_stability_boundaries_satisfy_independent_lifted_transfer_identity(self):
        *_, analysis = self.fixture(affine=True)
        result = analysis.margin_diagnostics()
        self.assertEqual(result["status"], "CONVERGED_NUMERICAL_SCHEDULED_STABILITY_BOUNDARIES")
        self.assertTrue(result["passed"])
        A, B, C, D, _ = analysis.lifted()
        for kind in ("gain", "phase"):
            for boundary in result[kind+"_crossings"]:
                z = complex(*boundary["critical_pole_real_imag"])
                transfer = C@np.linalg.solve(z*np.eye(analysis.dimension)-A, B)+D
                gain = np.exp(boundary["coordinate"]*(1j if kind == "phase" else 1.))
                self.assertLess(np.min(np.abs(np.linalg.eigvals(transfer)-1/gain)), 1e-8)
                self.assertLess(boundary["unit_pole_distance"], 1e-8)

    def test_negative_stribeck_damping_can_be_stable_but_fail_margin_requirement(self):
        *_, analysis = self.fixture(stribeck=True, filtered=True)
        self.assertLess(analysis.local.incremental_damping_A_s_rad, 0.)
        self.assertEqual(analysis.ff_reference[1], analysis.local.incremental_damping_A_s_rad/analysis.model.actuator_gain)
        result = analysis.margin_diagnostics()
        self.assertTrue(result["pole_diagnostics"]["local_stable"])
        self.assertEqual(result["status"], "CONVERGED_NUMERICAL_SCHEDULED_STABILITY_BOUNDARIES")
        self.assertLess(result["phase_margin_deg"], 50.)
        self.assertFalse(result["passed"])

    def test_observer_copy_and_public_matrices_cannot_change_cached_analysis(self):
        _, _, _, params, _, _, _, analysis = self.fixture()
        before = analysis.observer.gyro_variance
        params.observer.gyro_variance *= 2
        self.assertEqual(analysis.observer.gyro_variance, before)
        with self.assertRaises(AttributeError): analysis.ff_policy = "FROZEN_COMMAND_OFFSET"
        for matrix in analysis.phases[0]+(analysis.ff_reference, analysis.periodic_covariance)+analysis.transition(analysis.dt):
            with self.assertRaises(ValueError): matrix.setflags(write=True)

    def test_context_change_requires_rebuilding_the_sampled_tangent(self):
        model, point, support, params, schedule, gains, policy, analysis = self.fixture()
        context = dict(model=model, support=support, observer=params.observer, gains=gains, schedule=schedule,
            ff_policy=policy, frame=point.frame, configuration_id=point.configuration_id, friction_state="SLIDING")
        analysis.assert_context(**context)
        for changed in ({"gains": replace(gains, kp=.6)}, {"schedule": replace(schedule, gyro_period=3)},
                        {"ff_policy": "FROZEN_COMMAND_OFFSET"}, {"configuration_id": "changed"}):
            with self.assertRaises(Rejected) as caught: analysis.assert_context(**{**context, **changed})
            self.assertEqual(caught.exception.reason, Reason.OPERATING_POINT_CHANGED)

    def test_invalid_observer_covariance_never_becomes_a_gain(self):
        _, _, _, params, *_ = self.fixture()
        for covariance in (np.array([[1., 2.], [2., 1.]]), np.array([[1., .1], [0., 1.]]), np.full((2, 2), np.nan)):
            with self.assertRaises(Rejected): observer_tick(params.observer, .005, covariance, encoder_fresh=True, gyro_fresh=True)


if __name__ == "__main__": unittest.main()
