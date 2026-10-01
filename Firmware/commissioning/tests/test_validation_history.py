"""Validation retains actual pre-window commands across the native boundary."""
from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from Firmware.commissioning.contracts import ModelSpec
from Firmware.commissioning.identification import residuals, validation_report
from Firmware.commissioning.native import Native


class ValidationHistoryTests(unittest.TestCase):
    def setUp(self):
        self.native = Native()
        self.spec = ModelSpec("yaw", (-1., -.5, 0., .5, 1.), (-1., 0., 1.))
        # Unit inertia with zero friction/load and a fractional-sample input delay.
        self.theta = np.r_[np.ones(3), np.zeros(self.spec.size - 4), .073]
        t = np.arange(101) * .005
        after_change = np.maximum(t - self.theta[-1], 0.)
        # Exact solution: acceleration is 1 before the delayed change, then 2.
        self.run = SimpleNamespace(
            hash="local-probe-label", t=t,
            q=.1 + .2*t + .5*t*t + .5*after_change**2,
            v=.2 + t + after_change,
            tx=np.full(len(t), 2.), z=np.zeros(len(t)),
            direction=np.ones(len(t)),
            q_new=np.arange(len(t)) % 2 == 0,
            v_new=np.arange(len(t)) % 3 == 0,
            sigma_q=.0001, sigma_v=.0001, gyro_filter_tau_s=0.,
            tx_history_t=np.array([-.2, 0.]), tx_history_A=np.array([1., 2.]))

    def assert_exact_validation(self):
        report, = validation_report(self.native, self.spec, self.theta, [self.run])
        self.assertEqual(report["run_hash"], "local-probe-label")
        self.assertTrue(report["passed"])
        for key in ("q_rms_rad", "v_rms_rad_s", "one_observation_q_rms_rad",
                    "one_observation_v_rms_rad_s"):
            self.assertLess(report[key], 1e-12)
        self.assertLess(np.max(np.abs(residuals(self.native, self.spec, self.theta,
                                              [self.run]))), 1e-8)

    def test_validation_uses_actual_pre_window_current_history(self):
        # A first-window-current hold has a measurable error on this same input.
        legacy = self.native.rollout(self.spec, self.theta, self.run.t, self.run.tx,
                                     self.run.z, self.run.direction, (.1, .2))
        self.assertGreater(np.sqrt(np.mean((legacy[:, 0] - self.run.q)**2)), .01)
        self.assertGreater(np.sqrt(np.mean((legacy[:, 1] - self.run.v)**2)), .05)
        self.assert_exact_validation()

    def test_legacy_run_without_history_attributes_retains_first_current_hold(self):
        del self.run.tx_history_t
        del self.run.tx_history_A
        t = self.run.t
        self.run.q = .1 + .2*t + t*t
        self.run.v = .2 + 2*t
        self.assert_exact_validation()


if __name__ == "__main__":
    unittest.main()
