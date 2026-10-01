"""Native controller fault regressions with explicit, hash-free synthetic values."""
import unittest

import numpy as np

from Firmware.commissioning.contracts import ModelSpec, Rejected
from Firmware.commissioning.measurement import ObserverSpec
from Firmware.commissioning.native import CObservation, CReference, Controller, Native, parameters


class ControllerFaultTests(unittest.TestCase):
    def setUp(self):
        self.native = Native()
        spec = ModelSpec("yaw", (-100., -50., 0., 50., 100.), (-.3, 0., .3))
        theta = np.r_[np.full(3, .1), np.full(3, .06), np.full(15, -.12), np.full(15, .12), .008]
        observer = ObserverSpec(4e-10, 6.4e-9, .1, .03, .03, 4e-10, 6.4e-9,
                                False, "synthetic-fault-regression", "SYNTHETIC")
        values = dict(kp=1., ki=1., kpos=1., kaw=1., current_cap=.9, slew=1., integral_cap=.5,
                      velocity_cap=1., dt_min=.0001, dt_max=.03, intent_threshold=.00001,
                      rest_speed=.001, sustained_s=.06, start_timeout_s=.15)
        starts = np.r_[np.full(15, -.16), np.full(15, .16)].reshape(2, 3, 5)
        self.params = parameters(spec, theta, observer, values, starts, np.zeros((2, 3, 5), bool))

    def observation(self, t, seq, generation=1):
        return CObservation(t, t, t, 0., 0., seq, seq, generation, 1, 1)

    def assert_aborted(self, output):
        self.assertEqual(output.status, 5)
        self.assertEqual((output.requested, output.limited, output.sequence), (0., 0., 0))

    def test_timeout_latches_before_any_further_command(self):
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0.)
            fault_time = None
            last_sequence = 0
            for k in range(1, 61):
                out = c.step(self.observation(k*.005, k), CReference(0., .1, 0., 0.))
                if out.status == 0:
                    self.assertIsNone(fault_time)
                    last_sequence = out.sequence
                    self.assertTrue(c.ack(out))
                else:
                    self.assert_aborted(out)
                    self.assertFalse(c.ack(out))
                    fault_time = fault_time or k*.005
            self.assertAlmostEqual(fault_time, .160)
            with self.assertRaises(Rejected):
                c.reset(.3, 0., 0., 0., generation=2, accepted_time=.4)
            self.assert_aborted(c.step(self.observation(.305, 61, 2), CReference(0., -.1, 0., 0.)))
            self.assertFalse(c.switch(self.params, CReference()))
            c.reset(.3, 0., 0., 0., generation=2)
            out = c.step(self.observation(.305, 1, 2), CReference(0., -.1, 0., 0.))
            self.assertEqual(out.status, 0)
            self.assertGreater(out.sequence, last_sequence)

    def test_censored_start_emits_no_command_and_requires_reset(self):
        for k in range(15, 30):
            self.params.start_censored[k] = 1
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0.)
            out = c.step(self.observation(.005, 1), CReference(0., .1, 0., 0.))
            self.assert_aborted(out)
            self.assertFalse(c.ack(out))
            self.assert_aborted(c.step(self.observation(.01, 2), CReference(0., -.1, 0., 0.)))
            c.reset(.01, 0., 0., 0., generation=2)
            self.assertEqual(c.step(self.observation(.015, 1, 2), CReference(0., -.1, 0., 0.)).status, 0)

    def test_old_ack_cannot_accept_new_generation_output(self):
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0., generation=1)
            old = c.step(self.observation(.005, 1), CReference(0., .1, 0., 0.))
            c.reset(.005, 0., 0., 0., generation=2)
            new = c.step(self.observation(.010, 1, 2), CReference(0., -.1, 0., 0.))
            self.assertGreater(new.sequence, old.sequence)
            self.assertFalse(c.ack(old, accepted_time=.011))
            self.assertTrue(c.ack(new, accepted_time=.012))
            next_output = c.step(self.observation(.015, 2, 2), CReference(0., -.1, 0., 0.))
            self.assertEqual(next_output.status, 0)
            self.assertLessEqual(abs(next_output.limited-new.limited), .005+1e-12)

    def test_same_generation_reset_does_not_reuse_command_token(self):
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0.)
            old = c.step(self.observation(.005, 1), CReference(0., .1, 0., 0.))
            c.reset(.005, 0., 0., 0.)
            new = c.step(self.observation(.010, 1), CReference(0., -.1, 0., 0.))
            self.assertGreater(new.sequence, old.sequence)
            self.assertFalse(c.ack(old))
            self.assertTrue(c.ack(new))

    def test_delayed_ack_retains_pending_token_and_duplicate_ack_is_refused(self):
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0., accepted_time=-.1)
            out = c.step(self.observation(.005, 1), CReference(0., .1, 0., 0.))
            self.assertFalse(c.ack(out, accepted_time=.004))
            self.assertTrue(c.ack(out, accepted_time=.008))
            self.assertFalse(c.ack(out, accepted_time=.009))
            self.assertEqual(c.step(self.observation(.010, 2), CReference(0., .1, 0., 0.)).status, 0)

    def test_failed_send_inhibits_future_output_and_late_ack(self):
        with Controller(self.native, self.params) as c:
            c.reset(0., 0., 0., 0.)
            out = c.step(self.observation(.005, 1), CReference(0., .1, 0., 0.))
            self.assertFalse(c.ack(out, successful=False, accepted_time=.006))
            next_output = c.step(self.observation(.010, 2), CReference(0., .1, 0., 0.))
            self.assertNotEqual(next_output.status, 0)
            self.assertEqual((next_output.requested, next_output.limited, next_output.sequence), (0., 0., 0))
            self.assertFalse(c.ack(out, accepted_time=.011))


if __name__ == "__main__":
    unittest.main()
