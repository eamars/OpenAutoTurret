"""Opt-in planned START contracts through the actual shared native controller.

The unit sensor sequence is declared test input, not a physical plant forecast.
Real coupled actuator/sensor forecasts precede these regression tests.
"""
from dataclasses import FrozenInstanceError, replace
import unittest

from Firmware.commissioning.motor_feedforward import (BoundedStartPolicy, Failure, FeedforwardRejected,
    MotorFeedforward, ReferencePacket, parameters_for_family)
from Firmware.commissioning.native import CObservation, CReference, Controller, Native
from Firmware.commissioning.planned_start import PlannedStartLeg, PlannedStartProgram, PlannedStartRejected
from Firmware.tools.adr0022_family_forecast_probe import contract, feedforward_fixture
from Firmware.tools.adr0022_family_synthesis_probe import dynamic_fixture


class PlannedStartTests(unittest.TestCase):
    def setUp(self):
        self.model, _, _, template, _, _, _ = dynamic_fixture()[0]
        self.contract = replace(contract(duration=1.), frame="output_shaft_rad", trajectory_id="unit-two-leg-program")
        start = BoundedStartPolicy(configuration_id=self.contract.configuration_id, source="declared unit fixture",
            q_min_rad=-1., q_max_rad=1., static_negative_interval_A=(.156, .164),
            static_positive_interval_A=(.156, .164), negative_excess_A=.060, positive_excess_A=.060,
            max_attempt_s=.200, max_command_dose_A2s=.026, max_attempts=1)
        self.program = PlannedStartProgram(configuration_id=self.contract.configuration_id, frame=self.contract.frame,
            trajectory_id=self.contract.trajectory_id, generation=1, source_time_s=0., expires_at_s=1.,
            legs=(PlannedStartLeg(direction=1, departure_offset_s=.1, departure_position_rad=0.),
                  PlannedStartLeg(direction=-1, departure_offset_s=.5, departure_position_rad=.005)))
        self.support, self.state = feedforward_fixture(self.contract, self.model,
            actuator_policy="STEADY_STATE_REFERENCE", actuation_memory_max_s=.03,
            start_policy=start, planned_start_program=self.program)
        self.parameters = parameters_for_family(template, self.model, actuator_policy=self.support.actuator_policy,
            actuation_memory_max_s=self.support.actuation_memory_max_s, start_policy=start)
        self.core = Controller(Native(), self.parameters)
        self.addCleanup(self.core.close)
        self.adapter = MotorFeedforward(self.core, self.model, self.support)
        self.adapter.reset(self.state, now=0., previous_current_A=0., accepted_time_s=-.1)
        self.outputs = []

    def reference(self, t):
        q, v, a, phase, direction, anchor, position = 0., 0., 0., "HOLD", 0, None, None
        if .1 <= t < .3 or .5 <= t < .7:
            direction = 1 if t < .3 else -1
            anchor, position = (.1, 0.) if direction == 1 else (.5, .005)
            x = (t-anchor)/.2
            q = position+direction*.005*(10*x**3-15*x**4+6*x**5)
            v = direction*.025*(30*x*x-60*x**3+30*x**4)
            a = direction*.125*(60*x-180*x*x+120*x**3)
            phase = "DEPARTURE" if direction*a >= 0 else "BRAKING"
            if phase != "DEPARTURE":
                direction, anchor, position = 0, None, None
        elif .3 <= t < .5:
            q = .005
        return ReferencePacket(q_ref_rad=q, v_ref_rad_s=v, a_ref_rad_s2=a, time_s=t,
            source_time_s=0., expires_at_s=1., frame=self.contract.frame,
            configuration_id=self.contract.configuration_id, trajectory_id=self.contract.trajectory_id,
            generation=1, fresh=True, valid=True, trajectory_phase=phase, planned_direction=direction,
            departure_offset_s=anchor, departure_position_rad=position)

    def tick(self, k, *, reference=None, acknowledge=True):
        t = k*.005
        if .1 < t < .3:
            q, rate = .025*(t-.1), .025
        elif .3 <= t <= .5:
            q, rate = .005, 0.
        elif .5 < t < .7:
            q, rate = .005-.025*(t-.5), -.025
        else:
            q, rate = 0., 0.
        observation = CObservation(t, t, t, q, rate, k, k, 1, True, True)
        self.state = replace(self.state, time_s=t)
        out, _ = self.adapter.step(observation, reference or self.reference(t), self.state)
        self.state = replace(self.state, q_rad=out.position, v_rad_s=out.velocity)
        self.outputs.append(out)
        if acknowledge:
            self.assertTrue(self.adapter.acknowledge(out, successful=True, accepted_time_s=t+.001))
        return out

    def run_until(self, last):
        for k in range(1, last+1):
            self.tick(k)

    def test_distinct_legs_share_one_ledger_and_require_native_move(self):
        self.run_until(180)
        report = self.adapter.start_program_report()
        self.assertEqual([a["direction"] for a in report["admissions"]], [1, -1])
        self.assertEqual(set(report["accepted_MOVE_times_s"]), {0, 1})
        self.assertLess(report["dose_A2s"], .026)
        self.assertEqual(report["successful_receipt_count"], 180)
        self.assertEqual(self.support.start_policy.max_attempts, 1)
        self.assertFalse(report["terminal"])
        report["admissions"][0]["direction"] = 99
        self.assertEqual(self.adapter.start_program_report()["admissions"][0]["direction"], 1)

    def test_wrong_ack_charges_censored_prior_current_and_cannot_rearm(self):
        self.run_until(21)
        out = self.tick(22, acknowledge=False)
        out.sequence += 10000
        with self.assertRaises(FeedforwardRejected) as failure:
            self.adapter.acknowledge(out, successful=True, accepted_time_s=.111)
        self.assertEqual(failure.exception.reason, Failure.TX_FAILURE)
        report = self.adapter.start_program_report()
        self.assertAlmostEqual(report["dose_A2s"], .01**2*.005+.02**2*.005, places=14)
        self.assertAlmostEqual(report["last_accepted_current_A"], .02)
        self.assertAlmostEqual(report["last_accepted_time_s"], .106)
        self.assertTrue(report["terminal"] and report["unknown_future_hold"])
        with self.assertRaises(FeedforwardRejected):
            self.adapter.reset(replace(self.state, time_s=.111, q_rad=0., v_rad_s=0.),
                now=.111, previous_current_A=0., accepted_time_s=.111)
        self.assertAlmostEqual(self.adapter.start_program_report()["dose_A2s"], report["dose_A2s"])
        inhibited = self.core.step(CObservation(.115, .115, .115, 0., 0., 23, 23, 1, True, True),
            CReference(0., .1, 0., 0.))
        self.assertEqual((inhibited.status, inhibited.sequence), (5, 0))

    def test_known_partial_tail_charges_held_start_without_a_fake_receipt(self):
        self.run_until(23)
        out = self.tick(24, acknowledge=False)
        self.assertEqual(out.motion, 1)
        self.assertTrue(self.adapter.acknowledge(out, successful=True, accepted_time_s=.122))
        before = self.adapter.start_program_report()
        after = self.adapter.account_start_dose_through(.123)
        self.assertAlmostEqual(after["dose_A2s"]-before["dose_A2s"], out.limited**2*.001, places=14)
        self.assertEqual(after["accounted_through_s"], .123)
        self.assertEqual(after["successful_receipt_count"], before["successful_receipt_count"])
        self.assertEqual(after["last_accepted_time_s"], .122)
        self.assertEqual(after["last_accepted_current_A"], out.limited)
        self.assertFalse(after["terminal"])

    def test_hold_correction_does_not_create_a_planned_episode(self):
        self.run_until(180)
        with self.assertRaises(FeedforwardRejected) as failure:
            self.tick(181, reference=replace(self.reference(.905), q_ref_rad=.05))
        self.assertEqual(failure.exception.reason, Failure.OUTSIDE_SUPPORT)
        self.assertEqual(len(self.adapter.start_program_report()["admissions"]), 2)
        self.assertFalse(self.adapter.armed)

    def test_reference_relabel_and_closed_leg_cannot_replenish(self):
        self.run_until(180)
        with self.assertRaises(FeedforwardRejected) as failure:
            self.tick(181, reference=replace(self.reference(.905), trajectory_id="new-caller-id"))
        self.assertEqual(failure.exception.reason, Failure.INVALID_REFERENCE)
        self.assertEqual(len(self.adapter.start_program_report()["admissions"]), 2)
        self.assertFalse(self.adapter.armed)

    def test_program_is_immutable_and_clock_translation_is_once_only(self):
        with self.assertRaises(FrozenInstanceError):
            self.program.source_time_s = 1.
        shifted = self.program.with_epoch(1.1)
        self.assertEqual(shifted.source_time_s, 1.1)
        self.assertEqual(shifted.legs, self.program.legs)
        with self.assertRaises(PlannedStartRejected):
            shifted.with_epoch(1.1)
        invalid = replace(self.program, legs=(self.program.legs[0], replace(self.program.legs[1], direction=1)))
        with self.assertRaises(PlannedStartRejected):
            invalid.validate(self.support, self.parameters)


if __name__ == "__main__":
    unittest.main()
