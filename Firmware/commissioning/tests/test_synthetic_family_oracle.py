from dataclasses import replace
import os
import unittest

import numpy as np

from Firmware.commissioning.model_family import FamilyModel, FamilyNative
from Firmware.commissioning.synthetic_family_oracle import independent_rollout


def fixture(**changes):
    model = FamilyModel(a=.1, viscous=.06, coulomb_negative=.12, coulomb_positive=.12,
        static_negative=.16, static_positive=.16, load_offset=.02, q_min=-2., q_max=2.,
        actuator_gain=1., actuator_bias=0., transport_delay=0., gyro_bias=0., gyro_tau=.015,
        gyro_delay=0., current_gain=1., current_bias=0., current_tau=0., current_delay=0.,
        max_step=.00025)
    return replace(model, **changes)


class IndependentOracleTests(unittest.TestCase):
    def test_coulomb_moving_solution_and_gyro_are_independent_closed_form(self):
        m = fixture()
        t = np.array([0., .0031, .0197, .0502, .1073])
        initial = np.array([.1, .2, .3, 0., .3])
        result = independent_rollout(m, t, [-.1], [.3], initial)
        rate = m.viscous / m.a
        equilibrium = (.3 - m.load_offset - m.coulomb_positive) / m.viscous
        difference = initial[1] - equilibrium
        velocity = equilibrium + difference * np.exp(-rate * t)
        position = initial[0] + equilibrium * t + difference * (-np.expm1(-rate * t)) / rate
        gyro = equilibrium + (initial[3] - equilibrium) * np.exp(-t / m.gyro_tau) + \
            difference * (np.exp(-rate * t) - np.exp(-t / m.gyro_tau)) / (1 - rate * m.gyro_tau)
        np.testing.assert_allclose(result.trace[:, 0], position, atol=2e-13, rtol=0)
        np.testing.assert_allclose(result.trace[:, 1], velocity, atol=2e-13, rtol=0)
        np.testing.assert_allclose(result.trace[:, 3], gyro, atol=2e-13, rtol=0)

    def test_exact_stop_is_followed_by_static_holding(self):
        m = fixture()
        t = np.array([0., .01, .05, .1, .15, .2])
        initial = np.array([0., .2, 0., 0., 0.])
        result = independent_rollout(m, t, [-.1], [0.], initial)
        rate = m.viscous / m.a
        equilibrium = -(m.load_offset + m.coulomb_positive) / m.viscous
        crossing = -np.log(-equilibrium / (initial[1] - equilibrium)) / rate
        elapsed = np.minimum(t, crossing)
        position = equilibrium * elapsed + (initial[1] - equilibrium) * (-np.expm1(-rate * elapsed)) / rate
        velocity = np.maximum(0., equilibrium + (initial[1] - equilibrium) * np.exp(-rate * t))
        np.testing.assert_allclose(result.trace[:, 0], position, atol=2e-13, rtol=0)
        np.testing.assert_allclose(result.trace[:, 1], velocity, atol=2e-13, rtol=0)
        self.assertEqual(result.trace[-1, 5], 1.)
        event = next(e for e in result.events if e["kind"] == "zero_crossing")
        self.assertAlmostEqual(event["time_s"], crossing, places=12)


class FilterNumericalRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = FamilyNative(os.environ["OTA_AXIS_CORE_LIBRARY"])

    def test_fractional_delayed_current_is_exact_for_cascade_equal_and_zero_tau(self):
        t = np.arange(101) * .001
        tx_t, tx_A = np.array([-.1, .0173]), np.array([0., .23])
        elapsed = np.maximum(0., t - .0029 - .0173 - .0087)
        cases = [
            ("algebraic", 0., .012),
            ("first_order", .0073, .012),
            ("first_order", .0073, .0073),
            ("first_order", .0073, .0073 * (1 + 1e-8)),
            ("first_order", .0073, 0.),
            ("algebraic", 0., 0.)]
        for actuator, ta, tc in cases:
            with self.subTest(actuator=actuator, actuator_tau=ta, current_tau=tc):
                m = fixture(actuator=actuator, actuator_tau=ta, current_tau=tc,
                    static_negative=.8, static_positive=.8, transport_delay=.0087,
                    current_delay=.0029)
                actual = self.native.rollout(m, t, tx_t, tx_A, np.zeros(5))[:, 4]
                oracle = independent_rollout(m, t, tx_t, tx_A, np.zeros(5)).trace[:, 4]
                np.testing.assert_allclose(actual, oracle, atol=2e-12, rtol=0)
                if ta == tc and ta > 0:
                    expected = .23 * (1 - (1 + elapsed / ta) * np.exp(-elapsed / ta))
                elif ta == 0 and tc > 0: expected = .23 * (-np.expm1(-elapsed / tc))
                elif tc == 0 and ta > 0: expected = .23 * (-np.expm1(-elapsed / ta))
                else: continue
                np.testing.assert_allclose(actual, expected, atol=2e-12, rtol=0)

    def test_current_gain_bias_and_supplied_pre_run_filter_state_remain_explicit(self):
        m = fixture(actuator="first_order", actuator_tau=.0073, current_tau=.012,
            current_delay=.0029, current_gain=1.03, current_bias=-.004,
            static_negative=.8, static_positive=.8)
        t = np.array([0., .0007, .0029, .0031, .0097, .0173, .0331])
        initial = np.array([0., 0., .11, 0., -.02])
        oracle = independent_rollout(m, t, [-.1], [.23], initial).trace
        actual = self.native.rollout(m, t, [-.1], [.23], initial)
        np.testing.assert_allclose(actual[:, 4], oracle[:, 4], atol=2e-12, rtol=0)
        self.assertAlmostEqual(actual[0, 4], 1.03 * -.02 - .004, places=13)

    def test_pure_sensor_delay_changes_never_change_plant_or_other_sensor_states(self):
        # Delayed observation queries must not repartition the accepted plant
        # integration grid and create spurious mechanics/Jacobian information.
        t = np.arange(1601) * .001
        tx_t = np.array([-.1, .0513, .1937, .2621, .3932, .5311, .7039,
                         .8577, 1.0093, 1.1137, 1.2311, 1.4])
        tx_A = np.array([0., .23, 0., -.23, 0., .13, .24, -.24, 0., .23, 0., 0.])
        for friction in ("coulomb", "stribeck"):
            m = fixture(actuator="first_order", actuator_tau=.0073, friction=friction,
                stribeck_negative=.07, stribeck_positive=.05, current_tau=.012,
                transport_delay=.0087, current_delay=.0029, gyro_delay=.0043)
            baseline = self.native.rollout(m, t, tx_t, tx_A, np.zeros(5))
            for field, unchanged in (("gyro_delay", [0, 1, 2, 4, 5]),
                                     ("current_delay", [0, 1, 2, 3, 5])):
                for shift in (-1e-4, 1e-4, -1e-8, 1e-8):
                    with self.subTest(friction=friction, field=field, shift=shift):
                        changed = self.native.rollout(replace(m, **{field: getattr(m, field) + shift}),
                                                      t, tx_t, tx_A, np.zeros(5))
                        np.testing.assert_array_equal(baseline[:, unchanged], changed[:, unchanged])

    def test_filtered_current_query_at_command_edge_does_not_use_future_interval_target(self):
        # An instantaneous command change at a query's right endpoint has zero
        # effect on a continuous electrical cascade over its preceding interval.
        t = np.arange(1001) * .001
        tx_t = np.r_[-.1, np.arange(1000) * .001]
        tx_A = .32 * np.sin(2 * np.pi * 5 * np.maximum(0., tx_t - .1))
        for transport, sensor_delay, tc in ((.015, .008, .021), (.008, .004, .021),
                                             (.015, .008, .013), (.0087, .0029, .021)):
            with self.subTest(transport=transport, current_delay=sensor_delay, current_tau=tc):
                m = fixture(actuator="first_order", actuator_tau=.013, current_tau=tc,
                    static_negative=.8, static_positive=.8, transport_delay=transport,
                    current_delay=sensor_delay, max_step=.000125)
                actual = self.native.rollout(m, t, tx_t, tx_A, np.zeros(5))[:, 4]
                oracle = independent_rollout(m, t, tx_t, tx_A, np.zeros(5)).trace[:, 4]
                np.testing.assert_allclose(actual, oracle, atol=2e-12, rtol=0)


if __name__ == "__main__": unittest.main()
