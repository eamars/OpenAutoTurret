"""CAN health (§55 / §54.4): the server fields. The panel is the HUD stats overlay's CAN section,
tested in test_stats_and_settings.py (both buses, BUS-OFF, absence said as absence).

The transport has always counted rx/tx/error frames and the controller's error
state; the control loop read those counters and dropped them on the floor. The
consequence was not a missing row in a report, it was that a station climbing
toward error-passive looked exactly like a station at error-active until the
feedback went stale and the supervisor reacted to the *symptom*.

The row that matters most in this file is the boring one: a simulated backend
must render "no CAN link", never rx=0/tx=0, because a sim run is what everyone
runs first and a table of zeros is read as a healthy bus.
"""
from __future__ import annotations

import unittest

from ..protocol import Telemetry, telemetry_to_json


class CanTelemetryShapeTest(unittest.TestCase):

    def test_sim_run_carries_no_bus_claims(self):
        # What controld --sim publishes: available=false with the sentinel ages.
        t = Telemetry(phase="homing")
        obj = __import__("json").loads(telemetry_to_json(t))
        self.assertFalse(obj["can_available"])
        self.assertEqual(obj["can_state"], -1)
        self.assertEqual(obj["can_last_rx_age_ms"], -1)


if __name__ == "__main__":
    unittest.main()
