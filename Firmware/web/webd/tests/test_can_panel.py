"""The CAN health panel (§55 / §54.4) — server fields and the UI wiring.

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

from ..dashboard import DASHBOARD_HTML
from ..protocol import Telemetry, telemetry_to_json


class CanPanelSurfaceTest(unittest.TestCase):

    def test_panel_and_renderer_exist(self):
        for token in ('id="p-can"', 'id="can-state"', "renderCan(t)",
                      "t.can_state", "t.can_rx_error_frames", "CAN_STATE"):
            self.assertIn(token, DASHBOARD_HTML, token)

    def test_bus_off_is_the_loud_one(self):
        """§54.4 lists active/warning/passive/bus-off as things the HIL run must
        observe. BUS-OFF has to be visually unmistakable."""
        self.assertIn("BUS-OFF", DASHBOARD_HTML)
        self.assertIn('"err"', DASHBOARD_HTML[DASHBOARD_HTML.index("CAN_STATE"):
                                              DASHBOARD_HTML.index("CAN_STATE") + 400])

    def test_yousee_gets_no_permanent_unknown_badge(self):
        """This station's primary PHY is the yousee adapter, which exposes no
        controller error state (yousee_transport.hpp). A plain state map would
        sit on "unknown" forever and read as a fault; the panel must say the
        state is not exposed and name the signals that DO exist there."""
        note = DASHBOARD_HTML[DASHBOARD_HTML.index("function renderCan"):]
        note = note[:note.index("function render(t)")]
        self.assertIn('t.can_kind === "yousee"', note)
        self.assertIn("not exposed by adapter", note)
        self.assertIn("RX error frames", note)

    def test_simulated_backend_is_labelled_as_absence(self):
        note = DASHBOARD_HTML[DASHBOARD_HTML.index("function renderCan"):]
        note = note[:note.index("function render(t)")]
        self.assertIn("no CAN link", note)
        self.assertIn("absence, not health", note)


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
