"""ControldClient tests against a FakeControld (no CAN, no camera)."""
from __future__ import annotations

import os
import tempfile
import time
import unittest

from ..controld_client import ControldClient
from ..fake_controld import FakeControld
from ..protocol import ResponseMessage, Telemetry


class ControldClientTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_")
        self.sock_path = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock_path, telemetry_hz=50.0)
        self.fake.set_telemetry(track_state="tracking", q_yaw_rad=0.42)
        self.fake.start()
        self.client = ControldClient(self.sock_path, reconnect_interval=0.05)
        self.client.start()
        # Wait for the connection to establish.
        self.assertTrue(self.client.wait_connected(timeout=3.0))

    def tearDown(self) -> None:
        self.client.stop()
        self.fake.stop()
        self._tmp.cleanup()

    def test_receives_telemetry(self) -> None:
        deadline = time.time() + 3.0
        got = None
        while time.time() < deadline:
            got = self.client.latest_telemetry()
            if got is not None:
                break
            time.sleep(0.02)
        self.assertIsNotNone(got, "no telemetry received")
        self.assertIsInstance(got, Telemetry)
        self.assertEqual(got.track_state, "tracking")
        self.assertAlmostEqual(got.q_yaw_rad, 0.42)

    def test_command_accept(self) -> None:
        resp = self.client.send_command("hold")
        self.assertIsInstance(resp, ResponseMessage)
        self.assertTrue(resp.ok, resp.error)
        self.assertEqual(resp.command, "hold")

    def test_command_reject(self) -> None:
        # The fake can be told to reject a specific command.
        def handler(command: str, arg: str) -> ResponseMessage:
            if command == "start_tracking":
                return ResponseMessage(command, ok=False, error="not homed")
            return ResponseMessage(command, ok=True)

        self.fake.set_command_handler(handler)
        resp = self.client.send_command("start_tracking")
        self.assertFalse(resp.ok)
        self.assertEqual(resp.error, "not homed")

    def test_command_not_connected(self) -> None:
        self.client.stop()  # drop the connection
        # After stop, send_command should report not connected (no hang).
        resp = self.client.send_command("hold", timeout=0.2)
        self.assertFalse(resp.ok)

    def test_reconnect_after_fake_restart(self) -> None:
        # Kill the fake, confirm we lose the connection, then confirm the
        # client reconnects when it comes back.
        self.fake.stop()
        # Wait for the client to notice the drop.
        deadline = time.time() + 3.0
        while time.time() < deadline and self.client.connected():
            time.sleep(0.02)
        self.assertFalse(self.client.connected())
        # Bring a new fake up on the same path.
        fake2 = FakeControld(self.sock_path, telemetry_hz=50.0)
        fake2.set_telemetry(track_state="coasting")
        fake2.start()
        try:
            self.assertTrue(self.client.wait_connected(timeout=4.0))
            deadline = time.time() + 3.0
            while time.time() < deadline:
                t = self.client.latest_telemetry()
                if t is not None and t.track_state == "coasting":
                    break
                time.sleep(0.02)
            self.assertEqual(
                self.client.latest_telemetry().track_state, "coasting"
            )
        finally:
            fake2.stop()


if __name__ == "__main__":
    unittest.main()
