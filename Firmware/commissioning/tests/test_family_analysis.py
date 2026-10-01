"""Selected-family tangent contracts; native boundary evidence is in its probe."""
from dataclasses import replace
import math
import unittest

import numpy as np

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.synthesis import selected_family_linearization
from Firmware.tools.adr0022_family_linearization_probe import model_and_context


class FamilyAnalysisTests(unittest.TestCase):
    def fixture(self, direction=1, actuator="algebraic", friction="coulomb", load="affine", filters=(True, True)):
        model, point, support, _, _ = model_and_context(direction, actuator, friction, load, filters)
        return model, point, support, selected_family_linearization(model, point, support)

    def context(self, model, point, support):
        return dict(model=model, support=support, frame=point.frame,
                    configuration_id=point.configuration_id, friction_state="SLIDING")

    def test_current_mapping_units_and_filters_match_closed_form_transfer(self):
        omega = np.array([.2, 2., 11., 120.])
        for direction in (-1, 1):
            for actuator in ("algebraic", "first_order"):
                model, _, _, local = self.fixture(direction, actuator)
                s = 1j*omega
                actuator_response = model.actuator_gain/(1+s*model.actuator_tau)
                q = actuator_response/(model.a*s*s+model.viscous*s+model.load_slope)
                expected = np.column_stack((q, s*q/(1+s*model.gyro_tau),
                    model.current_gain*actuator_response/(1+s*model.current_tau)))
                np.testing.assert_allclose(local.frequency_response(omega), expected, rtol=2e-14, atol=2e-14)
                self.assertEqual(local.frequency_response(float(omega[0])).shape, (3,))

    def test_signed_stribeck_force_has_same_negative_differential_sign_in_both_directions(self):
        for direction in (-1, 1):
            model, point, _, local = self.fixture(direction=direction, friction="stribeck")
            fc = model.coulomb_positive if direction > 0 else model.coulomb_negative
            fs = model.static_positive if direction > 0 else model.static_negative
            vs = model.stribeck_positive if direction > 0 else model.stribeck_negative
            def force(v):
                return math.copysign(fc+(fs-fc)*math.exp(-(abs(v)/vs)**model.stribeck_power), v)
            h = 1e-6
            derivative = (force(point.v_rad_s+h)-force(point.v_rad_s-h))/(2*h)
            self.assertAlmostEqual(local.friction_derivative_A_s_rad, derivative, delta=1e-9)
            self.assertLess(local.incremental_damping_A_s_rad, 0.)
            self.assertGreater(local.A[1, 1], 0.)

    def test_biases_change_operating_input_but_not_perturbation_transfer(self):
        model, point, support, local = self.fixture(actuator="first_order")
        changed = replace(model, actuator_bias=-.3, gyro_bias=1.1, current_bias=.7, load_offset=.8)
        other = selected_family_linearization(changed, point, support)
        np.testing.assert_array_equal(local.frequency_response([.4, 9.]), other.frequency_response([.4, 9.]))

    def test_algebraic_unfiltered_current_keeps_direct_feedthrough(self):
        model, _, _, local = self.fixture(filters=(False, False))
        self.assertEqual(local.state_names, ("q_rad", "v_rad_s"))
        self.assertEqual(local.D[2, 0], model.current_gain*model.actuator_gain)
        self.assertEqual(local.C[1, 1], 1.)

    def test_channel_delays_are_retained_exactly(self):
        model, point, support, plain = self.fixture(actuator="first_order")
        delayed = selected_family_linearization(replace(model, transport_delay=.0137,
            gyro_delay=.0213, current_delay=.0307), point, support)
        omega = np.array([.1, 7., 121.])
        expected = plain.frequency_response(omega)*np.exp(-1j*omega[:, None]*np.array([.0137, .035, .0444]))
        np.testing.assert_allclose(delayed.frequency_response(omega), expected, rtol=1e-14, atol=1e-14)
        self.assertEqual(delayed.document()["delay_representation"], "EXACT_TRANSFER_FACTORS; NO_PADE_OR_DELAY_INVERSION")

    def test_rest_and_support_crossing_zero_are_rejected(self):
        model, point, support, _ = self.fixture()
        for changed in (replace(point, v_rad_s=0.), replace(point, friction_state="STICKING"),
                        replace(point, v_rad_s=-point.v_rad_s)):
            with self.assertRaises(Rejected): selected_family_linearization(model, changed, support)
        with self.assertRaises(Rejected):
            selected_family_linearization(model, point, replace(support, v_min_rad_s=-.1))

    def test_motor_frame_sign_and_physical_claim_are_rejected(self):
        model, point, support, _ = self.fixture()
        for changed in (replace(support, frame="motor_shaft_rad"), replace(support, qualification="PHYSICAL_QUALIFIED")):
            with self.assertRaises(Rejected): selected_family_linearization(model, point, changed)
        with self.assertRaises(Rejected): selected_family_linearization(replace(model, actuator_gain=-1.), point, support)

    def test_point_requires_two_sided_neighbourhood_within_rollout_domain(self):
        model, point, support, _ = self.fixture()
        for changed in (replace(point, q_rad=support.q_min_rad), replace(point, v_rad_s=support.v_max_rad_s)):
            with self.assertRaises(Rejected): selected_family_linearization(model, changed, support)
        with self.assertRaises(Rejected):
            selected_family_linearization(model, point, replace(support, q_min_rad=model.q_min-1.))

    def test_cached_tangent_rejects_model_support_configuration_and_state_changes(self):
        model, point, support, local = self.fixture()
        context = self.context(model, point, support)
        local.assert_context(**context)
        changes = ({"model": replace(model, current_gain=1.1)},
            {"support": replace(support, q_max_rad=.9)}, {"configuration_id": "another-mount"},
            {"frame": "motor_shaft_rad"}, {"friction_state": "STICKING"})
        for change in changes:
            with self.assertRaises(Rejected) as caught: local.assert_context(**{**context, **change})
            self.assertEqual(caught.exception.reason, Reason.OPERATING_POINT_CHANGED)

    def test_trace_rejects_rest_reversal_departure_and_invalid_quantities(self):
        model, point, support, local = self.fixture()
        context = self.context(model, point, support)
        local.assert_trajectory([point.q_rad, point.q_rad+.001], [point.v_rad_s]*2, **context)
        for q, v in (([point.q_rad]*2, [point.v_rad_s, 0.]),
                     ([point.q_rad]*2, [point.v_rad_s, -point.v_rad_s]),
                     ([point.q_rad, support.q_max_rad], [point.v_rad_s]*2),
                     ([point.q_rad], [np.nan]), ([False], [True]), ([], [])):
            with self.assertRaises(Rejected): local.assert_trajectory(q, v, **context)

    def test_matrix_invariants_cannot_be_changed_after_creation(self):
        _, _, _, local = self.fixture(actuator="first_order")
        for matrix in (local.A, local.B, local.C, local.D):
            with self.assertRaises(ValueError): matrix.flat[0] = 3.
            with self.assertRaises(ValueError): matrix.setflags(write=True)

    def test_invalid_frequency_never_becomes_a_margin(self):
        _, _, _, local = self.fixture()
        for frequency in (0., -1., np.nan, np.inf, True, ["1"], [], [[1.]]):
            with self.assertRaises(Rejected): local.frequency_response(frequency)

    def test_boolean_model_quantities_and_unselected_unused_actuator_tau_rejected(self):
        model, point, support, local = self.fixture()
        for changed in (replace(model, actuator_gain=True), replace(model, a=".1"),
                        replace(model, actuator_tau=.1), replace(model, friction="bristle_state")):
            with self.assertRaises(Rejected): selected_family_linearization(changed, point, support)
        # bool(False)==0 must not bypass a cached model's quantity checks.
        with self.assertRaises(Rejected):
            local.assert_context(**self.context(replace(model, transport_delay=False), point, support))

    def test_high_speed_friction_derivative_is_finite_limiting_value(self):
        model, point, support, _ = self.fixture(friction="stribeck")
        local = selected_family_linearization(replace(model, stribeck_power=1e200),
            replace(point, v_rad_s=1e100), replace(support, v_max_rad_s=1e101))
        self.assertEqual(local.friction_derivative_A_s_rad, 0.)
        self.assertTrue(np.isfinite(local.A).all())


if __name__ == "__main__": unittest.main()
