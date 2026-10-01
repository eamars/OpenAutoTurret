"""Executable causal forecasts, independent forward checks and stop-on-fault."""
from dataclasses import replace
import unittest

import numpy as np

from Firmware.commissioning.family_forecast import ForecastRejected, forecast, synthetic_motion_metrics
from Firmware.commissioning.motor_feedforward import parameters_for_family
from Firmware.commissioning.model_family import FamilyNative
from Firmware.commissioning.native import Native
from Firmware.commissioning.synthetic_family_oracle import independent_rollout
from Firmware.tools.adr0022_closed_loop_estimator_probe import controller_parameters, estimator_model
from Firmware.tools.adr0022_family_forecast_probe import contract, packet, feedforward_fixture


class FamilyForecastTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = Native()
        cls.plant = FamilyNative(cls.native.path)
        cls.model = replace(estimator_model(), actuator="first_order", actuator_tau=.0073,
            transport_delay=.0087, current_tau=.012, current_delay=.0029, friction="stribeck",
            stribeck_negative=.07, stribeck_positive=.05)

    def run_forecast(self, c, *, ref=None, params=None):
        return forecast(self.native, self.plant, self.model,
                        params if params is not None else controller_parameters(), c,
                        ref if ref is not None else lambda now: packet(c, now), initial=np.zeros(5))

    def test_independent_oracle_start_reverse_and_stop(self):
        c = contract(duration=3.7)
        result = self.run_forecast(c)
        independent = independent_rollout(self.model, result["t"], result["tx_t"], result["tx_A"], result["initial"])
        error = result["truth"] - independent.trace
        for column, limit in ((0, 1e-5), (3, 1e-4), (4, 1e-9)):
            self.assertLess(float(np.sqrt(np.mean(error[:, column]**2))), limit)
        self.assertEqual([event["direction"] for event in independent.events if event["kind"] == "breakaway"], [1, -1])
        self.assertEqual(result["report"]["outcome"]["status"], "COMPLETED")
        self.assertLess(result["report"]["causal_sensor_replay_max_error"], 1e-10)
        self.assertLessEqual(result["report"]["maximum_successful_command_A"], .35+1e-12)
        self.assertLessEqual(result["report"]["maximum_successful_slew_A_s"], 2.+1e-10)
        self.assertEqual(result["report"]["plant_state_initializations"], 1)
        self.assertFalse(result["report"]["future_realized_inputs_substituted"])

    def test_feedback_noise_changes_input_and_native_gyro_freshness_is_preserved(self):
        c = contract(duration=.5)
        pristine = self.run_forecast(c)
        noisy = self.run_forecast(replace(c, encoder_noise_rad=.00015, encoder_quantum_rad=2*np.pi/8192,
                                          gyro_noise_rad_s=.005, current_noise_A=.002))
        self.assertFalse(np.array_equal(pristine["tx_A"], noisy["tx_A"]))
        expected = np.arange(len(noisy["t"])) % 20 == 0
        np.testing.assert_array_equal(noisy["v_new"], expected)
        self.assertLess(noisy["report"]["causal_sensor_replay_max_error"], 1e-10)
        indices = np.arange(len(noisy["t"])) // 20 * 20
        np.testing.assert_array_equal(noisy["v"], noisy["v"][indices])
        sensors = noisy["causal_sensors"]
        expected_source = np.floor(np.rint(sensors[:, 0] / .001).astype(int) / 20) * .02 - self.model.gyro_delay
        np.testing.assert_allclose(sensors[:, 3], expected_source, rtol=0, atol=1e-15)
        np.testing.assert_array_equal(noisy["control_gyro_valid"], sensors[:, 3] >= 0.)
        self.assertTrue(noisy["control_gyro_valid"][3])
        self.assertFalse(noisy["control_gyro_valid"][:3].any())

    def test_reproducible_forecast_does_not_change_supplied_parameters(self):
        c = contract(307, True, duration=.1)
        parameters = controller_parameters()
        before = bytes(parameters)
        first = self.run_forecast(c, params=parameters)
        second = self.run_forecast(c, params=parameters)
        self.assertEqual(bytes(parameters), before)
        for key in ("truth", "tx_t", "tx_A", "q", "v", "current", "commands"):
            np.testing.assert_array_equal(first[key], second[key])

    def test_invalid_midrun_reference_ends_forecast_before_next_command(self):
        c = contract(duration=.1)
        calls = []

        def reference(now):
            calls.append(now)
            return replace(packet(c, now), source_time_s=now+.001) if now >= .025 else packet(c, now)

        result = self.run_forecast(c, ref=reference)
        self.assertEqual(result["report"]["outcome"]["status"], "INVALID_REFERENCE")
        self.assertEqual(calls[-1], .025)
        self.assertEqual(result["t"][-1], .025)
        self.assertLess(result["tx_t"][-1], .025)

    def test_native_censored_start_fault_ends_without_rearm_or_new_accepted_tx(self):
        c = contract(duration=.5)
        parameters = controller_parameters()
        for i in range(15, 30):
            parameters.start_censored[i] = 1
        result = self.run_forecast(c, params=parameters)
        self.assertEqual(result["report"]["outcome"]["status"], "CORE_FAULT")
        self.assertEqual(result["commands"][-1, 8], 5)
        self.assertEqual(result["commands"][-1, 7], 0)
        self.assertLess(result["tx_t"][-1], result["report"]["outcome"]["time_s"])
        self.assertFalse(result["report"]["deployment_authorized"])

    def test_physical_relabel_and_invalid_sampling_rejected_before_execution(self):
        c = contract(duration=.1)
        for bad in (replace(c, qualification="PHYSICAL"), replace(c, gyro_period_samples=True),
                    replace(c, sample_dt_s=.0009), replace(c, encoder_noise_rad=-.001)):
            with self.assertRaises(ForecastRejected):
                self.run_forecast(bad)

    def test_invalid_reference_numeric_values_are_typed_failures(self):
        c = contract(duration=.1)
        for value in (float("nan"), True, "bad"):
            result = self.run_forecast(c, ref=lambda now: replace(packet(c, now), a_ref_rad_s2=value))
            self.assertEqual(result["report"]["outcome"]["status"], "INVALID_REFERENCE")
            self.assertEqual(len(result["tx_t"]), 1)

    def test_supplied_nonzero_initial_velocity_reaches_the_native_observer(self):
        c = contract(duration=.01)
        for velocity in (-.1, .1):
            data = forecast(self.native, self.plant, self.model, controller_parameters(), c,
                            lambda now: packet(c, now), initial=np.array([0., velocity, 0., velocity, 0.]))
            self.assertEqual(data["report"]["outcome"]["status"], "COMPLETED")
            self.assertFalse(data["control_gyro_valid"].any())
            self.assertGreater(np.sign(velocity)*data["commands"][0, 6], .08)

    def test_declared_final_reference_expiry_is_not_extended_or_rejected_by_grid_roundoff(self):
        c = contract(duration=.35)  # .001 * 350 lands one ULP above .35.
        data = self.run_forecast(c)
        self.assertEqual(data["t"][-1], c.duration_s)
        self.assertEqual(data["report"]["outcome"]["status"], "COMPLETED")
        expired = self.run_forecast(c, ref=lambda now: replace(packet(c, now), expires_at_s=c.duration_s-.001))
        self.assertEqual(expired["report"]["outcome"]["status"], "INVALID_REFERENCE")
        self.assertLess(expired["tx_t"][-1], c.duration_s)

    def test_shared_motor_ff_forecast_uses_one_posterior_and_matches_independent_plant(self):
        c = contract(duration=.35)
        model = replace(self.model, actuator="algebraic", actuator_tau=0., transport_delay=0.)
        support, state = feedforward_fixture(c, model)
        data = forecast(self.native, self.plant, model, parameters_for_family(controller_parameters(), model),
            c, lambda now: packet(c, now), initial=np.zeros(5),
            feedforward_support=support, feedforward_state=state)
        self.assertEqual(data["report"]["outcome"]["status"], "COMPLETED")
        np.testing.assert_array_equal(data["commands"][:, 5:7], data["ff_posteriors"][:, 1:3])
        np.testing.assert_allclose(data["ff_posteriors"][1:, 5], data["tx_A"][1:-1], rtol=0, atol=0)
        np.testing.assert_allclose(data["ff_posteriors"][1:, 6], data["tx_t"][1:-1], rtol=0, atol=1e-15)
        oracle = independent_rollout(model, data["t"], data["tx_t"], data["tx_A"], data["initial"])
        for column, limit in ((0, 1e-5), (3, 1e-4), (4, 1e-9)):
            self.assertLess(float(np.sqrt(np.mean((data["truth"][:, column]-oracle.trace[:, column])**2))), limit)
        self.assertEqual(data["report"]["causal_sensor_replay_max_error"], 0.)

    def test_motor_ff_native_fault_ends_without_a_new_accepted_command(self):
        c = contract(duration=.5)
        model = replace(self.model, actuator="algebraic", actuator_tau=0., transport_delay=0.)
        support, state = feedforward_fixture(c, model)
        params = parameters_for_family(controller_parameters(), model)
        for i in range(15, 30):
            params.start_censored[i] = 1
        data = forecast(self.native, self.plant, model, params, c, lambda now: packet(c, now),
            initial=np.zeros(5), feedforward_support=support, feedforward_state=state)
        self.assertEqual(data["report"]["outcome"]["status"], "MOTOR_FF_FAULT")
        self.assertEqual(data["report"]["outcome"]["reason"], "CORE_FAULT")
        self.assertLess(data["tx_t"][-1], data["report"]["outcome"]["time_s"])
        self.assertFalse(data["report"]["deployment_authorized"])

    def test_frozen_metric_band_cannot_treat_held_gyro_as_control_rate_samples(self):
        c = contract(duration=.1)
        data = self.run_forecast(c)
        with self.assertRaises(ForecastRejected):
            synthetic_motion_metrics(data, c, command_time=0., zero_reference_time=.05,
                                     gyro_bandwidth_hz=20.)  # 50 Hz source supports at most 10 Hz here.

    def test_stationary_metric_requires_the_explicit_zero_step_and_full_stop_window(self):
        c = contract(duration=2.05)
        data = self.run_forecast(c, ref=lambda now: replace(packet(c, now),
            q_ref_rad=0., v_ref_rad_s=0., a_ref_rad_s2=0., trajectory_phase="HOLD"))
        result = synthetic_motion_metrics(data, c, command_time=.05, zero_reference_time=.05,
                                          gyro_bandwidth_hz=10., step_rad=0.)
        self.assertTrue(result["metrics"]["passed"])
        self.assertEqual(result["metrics"]["stop_drift_rad"], 0.)
        self.assertEqual(result["metrics"]["step_error_rad"], 0.)
        with self.assertRaises(ValueError):
            synthetic_motion_metrics(data, c, command_time=.05, zero_reference_time=.05,
                                     gyro_bandwidth_hz=10.)
        with self.assertRaises(ValueError):
            synthetic_motion_metrics(data, c, command_time=.05, zero_reference_time=.05,
                gyro_bandwidth_hz=10., step_rad=0., position_jitter_limit_rad=float("nan"))

    def test_separate_plant_drives_sensors_and_source_clock_without_changing_ff_model(self):
        c = contract(duration=.35)
        selected = replace(self.model, actuator="algebraic", actuator_tau=0., transport_delay=0.)
        actual = replace(selected, actuator_gain=.7, viscous=.12, gyro_delay=.017)
        support, state = feedforward_fixture(c, selected)
        params = parameters_for_family(controller_parameters(), selected)
        before = bytes(params)
        data = forecast(self.native, self.plant, selected, params, c, lambda now: packet(c, now),
            initial=np.zeros(5), feedforward_support=support, feedforward_state=state, plant_model=actual)
        separation = data["report"]["separate_synthetic_plant"]
        self.assertEqual(separation["controller_model"], selected.document())
        self.assertEqual(separation["plant_model"], actual.document())
        self.assertFalse(separation["plant_parameters_injected_into_controller"])
        self.assertIn("-PLANT_GYRO_DELAY", data["report"]["gyro_timestamp_policy"])
        self.assertEqual(bytes(params), before)
        sensors = data["causal_sensors"]
        expected_source = np.floor(np.rint(sensors[:, 0]/.001).astype(int)/20)*.02-actual.gyro_delay
        np.testing.assert_allclose(sensors[:, 3], expected_source, rtol=0, atol=1e-15)
        np.testing.assert_array_equal(data["control_gyro_valid"], sensors[:, 3] >= 0.)
        oracle = independent_rollout(actual, data["t"], data["tx_t"], data["tx_A"], data["initial"])
        for column, limit in ((0, 1e-5), (3, 1e-4), (4, 1e-9)):
            self.assertLess(float(np.sqrt(np.mean((data["truth"][:, column]-oracle.trace[:, column])**2))), limit)
        wrong_plant = self.plant.rollout(selected, data["t"], data["tx_t"], data["tx_A"], data["initial"])
        self.assertGreater(float(np.max(np.abs(data["truth"]-wrong_plant))), 1e-5)
        self.assertEqual(data["report"]["causal_sensor_replay_max_error"], 0.)

    def test_explicit_identical_plant_preserves_default_arrays(self):
        c = contract(307, True, duration=.1)
        default = self.run_forecast(c)
        explicit = forecast(self.native, self.plant, self.model, controller_parameters(), c,
            lambda now: packet(c, now), initial=np.zeros(5), plant_model=self.model)
        for key, value in default.items():
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(explicit[key], value)
        with self.assertRaises(ForecastRejected):
            forecast(self.native, self.plant, self.model, controller_parameters(), c,
                lambda now: packet(c, now), initial=np.zeros(5), plant_model="unsupported")


if __name__ == "__main__":
    unittest.main()
