"""``GET /api/control_trace`` — the ring, reachable from a browser.

The route exists because the ring wraps in ~20 s and the only reader was a shell
on the station. What these tests pin down is the part a route can get wrong on
its own: it must hand the frame over unchanged (the ns stamps are strings on
purpose), and it must answer "unreadable" with a 503 that names the socket
instead of a 200 with an empty list — "no anomalies" and "could not read" cannot
share a shape.
"""
from __future__ import annotations

import os
import tempfile
import time
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..config import WebConfig
from ..controld_client import ControldClient
from ..fake_controld import FakeControld

TRACE = {
    "type": "control_trace",
    "axes": ["pitch", "yaw"],
    "frozen": True,
    "frozen_t_ns": "1234567890123456789",
    "clock": "monotonic_raw",
    "rows": [
        {"t": "1000000000000000000", "phase": "homing", "cmd": [0.0, 0.8],
         "period_us": 5000, "rx": [1, 2]},
        {"t": "1000000005000000000", "phase": "hold", "cmd": [0.0, 0.0],
         "period_us": 5000, "rx": [3, 4]},
    ],
}


class ControlTraceRouteTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_trace_")
        self.sock_path = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock_path, telemetry_hz=50.0)
        self.fake.set_trace_frame(TRACE)
        self.fake.start()
        self.client = ControldClient(self.sock_path, reconnect_interval=0.05)
        self.app = create_app(self.client, WebConfig(
            host="127.0.0.1", port=0, socket_path=self.sock_path))
        self.tc = TestClient(self.app)
        self.tc.__enter__()
        # The fake publishes telemetry on the same socket the trace arrives on;
        # give the client a moment to be connected so a failure here is about the
        # route, not about startup.
        deadline = time.time() + 4.0
        while time.time() < deadline and not self.client.connected():
            time.sleep(0.02)

    def tearDown(self) -> None:
        self.tc.__exit__(None, None, None)
        self.client.stop()
        self.fake.stop()
        self._tmp.cleanup()

    def test_the_ring_comes_through_unchanged(self):
        r = self.tc.get("/api/control_trace")
        self.assertEqual(r.status_code, 200, r.text)
        body = r.json()
        self.assertEqual(body["type"], "control_trace")
        self.assertTrue(body["frozen"], "a frozen window must still announce itself")
        self.assertEqual(len(body["rows"]), 2)
        # Decimal strings, not floats: docs/04_CONTRACTS.md §2 forbids a 64-bit ns
        # timestamp becoming a JS Number, and a route that "helpfully" re-encodes
        # the frame is where that rule would quietly die.
        self.assertEqual(body["frozen_t_ns"], "1234567890123456789")
        self.assertEqual(body["rows"][0]["t"], "1000000000000000000")

    def test_a_station_with_no_controld_says_which_socket_it_tried(self):
        dead_path = os.path.join(self._tmp.name, "not-running.sock")
        dead_app = create_app(ControldClient(dead_path, reconnect_interval=0.05),
                              WebConfig(host="127.0.0.1", port=0, socket_path=dead_path))
        with TestClient(dead_app) as tc:
            r = tc.get("/api/control_trace")
        self.assertEqual(r.status_code, 503, r.text)
        self.assertIn("not-running.sock", r.json()["error"])
        self.assertNotIn("rows", r.json(), "an unreadable ring must not look like an empty one")


if __name__ == "__main__":
    unittest.main()
