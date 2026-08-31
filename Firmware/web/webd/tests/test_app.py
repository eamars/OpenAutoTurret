"""FastAPI app tests (TestClient) against a FakeControld (no CAN, no camera)."""
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
from ..protocol import ResponseMessage


class AppTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_app_")
        self.sock_path = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock_path, telemetry_hz=50.0)
        self.fake.set_telemetry(
            track_state="tracking",
            q_yaw_rad=0.25,
            safety_action="ALLOW",
            installation_source="visual_calibration",
            installation_calibrated=True,
        )
        self.fake.start()
        self.client = ControldClient(self.sock_path, reconnect_interval=0.05)
        self.config = WebConfig(
            host="127.0.0.1",
            port=0,
            socket_path=self.sock_path,
        )
        self.app = create_app(self.client, self.config)
        self.tc = TestClient(self.app)
        self.tc.__enter__()
        # Wait until controld is connected AND we have a telemetry snapshot
        # (so /api/state is deterministic across all tests).
        deadline = time.time() + 4.0
        while time.time() < deadline:
            r = self.tc.get("/api/state")
            if r.status_code == 200:
                break
            time.sleep(0.02)
        self.assertEqual(self.tc.get("/api/state").status_code, 200)

    def tearDown(self) -> None:
        try:
            self.tc.__exit__(None, None, None)
        finally:
            self.client.stop()
            self.fake.stop()
            self._tmp.cleanup()

    def test_health(self) -> None:
        r = self.tc.get("/api/health")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertTrue(body["ok"])
        self.assertTrue(body["controld_connected"])
        self.assertIn("browser_clients", body)

    def test_state(self) -> None:
        r = self.tc.get("/api/state")
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(body["type"], "telemetry")
        self.assertEqual(body["track_state"], "tracking")
        self.assertAlmostEqual(body["q_yaw_rad"], 0.25)
        self.assertEqual(body["installation_source"], "visual_calibration")
        self.assertEqual(body["safety_action"], "ALLOW")

    def test_command_accept(self) -> None:
        r = self.tc.post("/api/command", json={"command": "hold"})
        self.assertEqual(r.status_code, 200)
        self.assertTrue(r.json()["ok"])

    def test_command_reject(self) -> None:
        def handler(command: str, arg: str) -> ResponseMessage:
            if command == "start_tracking":
                return ResponseMessage(command, ok=False, error="not homed")
            return ResponseMessage(command, ok=True)

        self.fake.set_command_handler(handler)
        r = self.tc.post(
            "/api/command", json={"command": "start_tracking"}
        )
        self.assertEqual(r.status_code, 200)
        self.assertFalse(r.json()["ok"])
        self.assertEqual(r.json()["error"], "not homed")

    def test_dashboard_html(self) -> None:
        r = self.tc.get("/")
        self.assertEqual(r.status_code, 200)
        self.assertIn("<!DOCTYPE html>", r.text)
        self.assertIn("Developer controls", r.text)
        self.assertIn("/ws", r.text)

    def test_websocket_streams_telemetry(self) -> None:
        with self.tc.websocket_connect("/ws") as ws:
            # The first frame is the latest snapshot sent on connect.
            msg = ws.receive_json()
            self.assertEqual(msg["type"], "telemetry")
            self.assertEqual(msg["track_state"], "tracking")
            # Then live frames keep coming (the fake publishes at 50 Hz).
            deadline = time.time() + 3.0
            got = False
            while time.time() < deadline and not got:
                try:
                    ws.receive_json(mode="text")
                    got = True
                except Exception:
                    pass
            self.assertTrue(got, "no live telemetry frame over websocket")

    def test_multi_client_health_counts(self) -> None:
        # Two simultaneous websocket clients should both be counted (§54.5:
        # multiple clients must not break the control-facing service).
        with self.tc.websocket_connect("/ws") as ws1, self.tc.websocket_connect(
            "/ws"
        ) as ws2:
            # Give the hub a moment to register both.
            deadline = time.time() + 2.0
            n = 0
            while time.time() < deadline:
                n = self.tc.get("/api/health").json()["browser_clients"]
                if n >= 2:
                    break
                time.sleep(0.02)
            self.assertGreaterEqual(n, 2)
            # Both should be receiving telemetry.
            self.assertEqual(ws1.receive_json()["type"], "telemetry")


if __name__ == "__main__":
    unittest.main()
