from __future__ import annotations
from dataclasses import replace
import os
from pathlib import Path
import unittest
import numpy as np

from Firmware.commissioning.contracts import Reason, Rejected
from Firmware.commissioning.model_family import FamilyModel, FamilyNative, FamilyRun, check_family_split, fit_family, compare_families
from Firmware.commissioning.probe_model_family import probe


class FamilyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = FamilyNative(os.environ["OTA_AXIS_CORE_LIBRARY"])

    def model(self, **changes):
        base = FamilyModel(a=.2, viscous=0., coulomb_negative=.1, coulomb_positive=.1,
            static_negative=.15, static_positive=.15, q_min=-10., q_max=10.,
            actuator_gain=1., actuator_bias=0., transport_delay=0., gyro_bias=0., gyro_tau=0.,
            gyro_delay=0., current_gain=1., current_bias=0., current_tau=0., current_delay=0.)
        return replace(base, **changes)

    def test_complete_native_probe(self):
        self.assertTrue(probe(self.native)["passed"])

    def test_fractional_delayed_current_stays_zoh(self):
        m = self.model(transport_delay=.013, current_delay=.0075)
        t = np.array([0., .1199, .12049, .1205, .12051, .121])
        pred = self.native.rollout(m, t, [-.1, .1], [0., .3], [0., 0., 0., 0., 0.])
        np.testing.assert_array_equal(pred[:, 4], [0., 0., 0., .3, .3, .3])

    def test_sliding_constant_acceleration_has_analytical_trajectory(self):
        m = self.model()
        t = np.linspace(0., .5, 101)
        pred = self.native.rollout(m, t, [-.1], [.3], [0., .1, .3, .1, .3])
        np.testing.assert_allclose(pred[:, 1], .1 + t, atol=1e-12)
        np.testing.assert_allclose(pred[:, 0], .1 * t + .5 * t ** 2, atol=1e-12)

    def test_no_future_observations_enter_native_signature(self):
        m = self.model()
        complete = self.native.rollout(m, [0., .25, .5], [-.1], [.3], [0., 0., .3, 0., .3])
        sparse = self.native.rollout(m, [0., .5], [-.1], [.3], [0., 0., .3, 0., .3])
        np.testing.assert_allclose(complete[-1], sparse[-1], atol=1e-10)

    def test_actual_prehistory_and_domain_are_required(self):
        with self.assertRaises(Rejected) as error:
            self.native.rollout(self.model(transport_delay=.1), [0., .2], [0.], [.3], [0., 0., .3, 0., .3])
        self.assertEqual(error.exception.reason, Reason.DATA_INVALID)
        with self.assertRaises(Rejected) as error:
            self.native.rollout(self.model(q_min=-.01, q_max=.01), [0., .2], [-.1], [.3], [0., 0., .3, 0., .3])
        self.assertEqual(error.exception.reason, Reason.MODEL_INADEQUATE)

    def test_holding_friction_balances_spatial_load(self):
        m = self.model(load_offset=.3)
        pred = self.native.rollout(m, [0., .2, .4], [-.1], [.25], [0., 0., .25, 0., .25])
        np.testing.assert_array_equal(pred[:, 0:2], np.zeros((3, 2)))
        np.testing.assert_array_equal(pred[:, 5], np.ones(3))

    def test_run_split_rejects_windows_from_one_journal(self):
        def run(name, source):
            return FamilyRun(name, source, np.arange(3.), np.zeros(3), np.zeros(3), np.zeros(3),
                np.ones(3, bool), np.ones(3, bool), np.ones(3, bool), np.array([-.1]), np.array([0.]),
                np.zeros(5), .001, .001, .001, provenance="SYNTHETIC")
        with self.assertRaises(Rejected) as error:
            check_family_split([run("train", "same-journal")], [run("selection", "same-journal")], [run("holdout", "third")])
        self.assertEqual(error.exception.reason, Reason.DATA_INVALID)

    def test_infeasible_seed_restored_inside_bounds_then_fits_observations(self):
        t = np.linspace(0., .5, 101)
        run = FamilyRun("analytical", "analytical-source", t, .1*t + .5*t**2, .1+t,
            np.full(len(t), .3), np.ones(len(t), bool), np.ones(len(t), bool), np.ones(len(t), bool),
            np.array([-.1]), np.array([.3]), np.array([0., .1, .3, .1, .3]),
            .0001, .0001, .0001, provenance="SYNTHETIC")
        result = fit_family(self.native, self.model(a=.02, q_min=-.3, q_max=.3), [run],
                            bounds={"a": (.01, .4)}, max_nfev=80)
        self.assertAlmostEqual(result["model"].a, .2, places=7)
        self.assertTrue(result["optimizer"]["success"])
        self.assertTrue(result["optimizer"]["initialization"]["original_invalid_runs"])
        self.assertEqual(result["optimizer"]["unmodified_objective_sensitivity"]["status"],
                         "COMPUTED_FROM_OBSERVATIONS")

    def test_inconsistent_training_registration_blocks_even_passing_selection(self):
        t = np.linspace(0., .5, 101)
        def run(name, angle_bias=0.):
            return FamilyRun(name, name+'-source', t, .1*t + .5*t**2 + angle_bias, .1+t,
                np.full(len(t), .3), np.ones(len(t), bool), np.ones(len(t), bool),
                np.ones(len(t), bool), np.array([-.1]), np.array([.3]),
                np.array([0., .1, .3, .1, .3]), .0001, .0001, .0001, provenance="SYNTHETIC")
        result = compare_families(self.native,
            [("analytical", self.model(), {"a": (.1999, .2001)})],
            [run("train", .01)], [run("selection")], [run("holdout")], max_nfev=50)
        comparison = result["comparisons"][0]
        self.assertTrue(comparison["selection"][0]["passed"])
        self.assertFalse(comparison["training_passed"])
        self.assertFalse(comparison["selection_passed"])
        self.assertIsNone(result["selected_label"])
        self.assertEqual(result["final_holdout"], [])

    def test_zero_seed_sensor_offset_has_nonzero_absolute_derivative_step(self):
        t = np.linspace(0., .5, 101)
        initial = np.array([0., .1, .3, .1, .3])
        truth = self.native.rollout(self.model(gyro_bias=.003), t, [-.1], [.3], initial)
        fresh = np.ones(len(t), bool)
        run = FamilyRun("offset", "synthetic-offset", t, truth[:, 0], truth[:, 3], truth[:, 4],
            fresh, fresh, fresh, np.array([-.1]), np.array([.3]), initial,
            .0001, .0001, .0001, provenance="SYNTHETIC")
        fit = fit_family(self.native, self.model(gyro_bias=0.), [run],
                         bounds={"gyro_bias": (-.02, .02)}, max_nfev=50)
        self.assertTrue(fit["optimizer"]["success"])
        self.assertGreater(fit["optimizer"]["absolute_derivative_steps"][0], 0.)
        self.assertAlmostEqual(fit["model"].gyro_bias, .003, places=10)

    def test_short_native_gyro_keeps_supplied_initializer(self):
        from Firmware.commissioning.model_family import moving_integral_initializer
        model = self.model(viscous=.01)
        t = np.linspace(0.,.06,7)
        zeros, fresh = np.zeros(len(t)),np.ones(len(t),bool)
        bounds = {"a":(.1,.3),"viscous":(.001,.1),
                  "coulomb_negative":(.05,.15),"coulomb_positive":(.05,.15)}
        for count in (1,2,4):
            gyro_fresh = np.zeros(len(t),bool)
            gyro_fresh[:count] = True
            run = FamilyRun("short-gyro","synthetic-short-gyro",t,zeros,zeros,zeros,
                fresh,gyro_fresh,fresh,np.array([-.1]),np.array([0.]),np.zeros(5),
                .0001,.0001,.0001,provenance="SYNTHETIC")
            seeded, metadata = moving_integral_initializer(model,[run],bounds)
            self.assertIsNone(seeded)
            self.assertEqual(metadata["status"],"INSUFFICIENT_NATIVE_GYRO")
            if count>=2:  # FamilyRun's complete-fit contract requires at least2.
                fit = fit_family(self.native,model,[run],bounds=bounds,max_nfev=5)
                self.assertTrue(fit["optimizer"]["success"])
                self.assertEqual(fit["model"],model)
                self.assertEqual(fit["optimizer"]["initialization"]["moving_integral_initializer"]["status"],
                                 "INSUFFICIENT_NATIVE_GYRO")

    def test_noisy_seed41_recovers_and_predicts_other_successful_input_histories(self):
        # Retain the original deterministic failure as an executable regression:
        # actual native feedback supplies TX, independent analytic mechanics and
        # sampled/quantized sensors supply observations. The old relative forward
        # derivative exhausted120 and failed the two cross-run position gates.
        from Firmware.tools.adr0022_closed_loop_estimator_probe import (
            generate, estimator_model, FREE_BOUNDS, FIXTURE, measurement_errors, trajectory_gate)
        from Firmware.tools.adr0022_seed41_diagnose import run_from_case
        from Firmware.commissioning.native import Native
        controller_native = Native(Path(os.environ["OTA_AXIS_CORE_LIBRARY"]))
        data = generate(controller_native, 8., 41, True)
        seed = replace(estimator_model(), a=.085, viscous=.075,
                       coulomb_negative=.105, coulomb_positive=.13)
        fit = fit_family(self.native, seed, [run_from_case("noisy-seed-41", data)],
                         bounds=FREE_BOUNDS, max_nfev=120)
        self.assertTrue(fit["optimizer"]["success"])
        self.assertFalse(fit["optimizer"]["parameter_bound_hits"])
        for field in FREE_BOUNDS:
            self.assertLessEqual(abs(getattr(fit["model"], field)/FIXTURE[field]-1), .05)
        for noise_seed in (17, 41, 83):
            target = data if noise_seed == 41 else generate(controller_native, 8., noise_seed, True)
            prediction = self.native.rollout(fit["model"], target["t"], target["tx_t"],
                                             target["tx_A"], np.zeros(5))
            gate = trajectory_gate(target, measurement_errors(target, prediction))
            self.assertAlmostEqual(gate["thresholds"]["q"], .0008023127185209158)
            self.assertTrue(gate["passed"], noise_seed)

    def test_moving_initializer_escapes_quintic_grazing_branch(self):
        from Firmware.tools.adr0022_closed_loop_estimator_probe import (
            generate, estimator_model, FREE_BOUNDS, FIXTURE, measurement_errors, trajectory_gate)
        from Firmware.tools.adr0022_estimator_verification import quintic_hold_reverse
        from Firmware.tools.adr0022_seed41_diagnose import run_from_case
        from Firmware.commissioning.native import Native
        data = generate(Native(Path(os.environ["OTA_AXIS_CORE_LIBRARY"])),10.,227,True,
                        reference_function=quintic_hold_reverse)
        # This supplied point is the retained failure. Its initial OE cost is
        # smaller than the moving-equation seed, but the local fit gets trapped
        # at a genuine static/moving grazing branch unless reinitialized.
        seed = replace(estimator_model(),a=.09098439731143511,viscous=.07748679036557453,
                       coulomb_negative=.11551826876458504,coulomb_positive=.11827877789470029)
        fit = fit_family(self.native,seed,[run_from_case("quintic-seed-227",data)],
                         bounds=FREE_BOUNDS,max_nfev=120)
        initializer = fit["optimizer"]["initialization"]["moving_integral_initializer"]
        self.assertTrue(initializer["selected"])
        self.assertFalse(initializer["selection_or_holdout_used"])
        self.assertEqual(initializer["matrix_rank"],4)
        self.assertTrue(fit["optimizer"]["success"])
        for field in FREE_BOUNDS:
            self.assertLessEqual(abs(getattr(fit["model"],field)/FIXTURE[field]-1),.05)
        pred = self.native.rollout(fit["model"],data["t"],data["tx_t"],data["tx_A"],np.zeros(5))
        self.assertTrue(trajectory_gate(data,measurement_errors(data,pred))["passed"])


if __name__ == "__main__":
    unittest.main()
