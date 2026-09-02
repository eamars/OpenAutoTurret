"""The payload-profile picker's server side (§28.5 / §31.3 / §42.2).

The daemon has accepted `select_payload_profile` for a while (and `station_ipc`
could send it), but the dashboard — the thing an operator actually stands at —
had no way to name a profile. That gap matters more than it looks: `select_
payload_profile` applies motion CAPS immediately while leaving the status at
`no_profile` until a verification commissions it, so an operator who cannot see
and choose profiles in the UI will reach for a config file and a restart
instead — a slower, less visible path to the same change.
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
from ..dashboard import DASHBOARD_HTML
from ..fake_controld import FakeControld


class ProfileListingTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_profiles_")
        self.sock_path = os.path.join(self._tmp.name, "controld.sock")
        self.profile_dir = os.path.join(self._tmp.name, "payload_profiles")
        os.makedirs(self.profile_dir, exist_ok=True)
        self.fake = FakeControld(self.sock_path, telemetry_hz=50.0)
        self.fake.set_telemetry(payload_profile_name="conservative",
                                payload_profile_status="ok")
        self.fake.start()
        self.client = ControldClient(self.sock_path, reconnect_interval=0.05)
        self.config = WebConfig(host="127.0.0.1", port=0,
                                socket_path=self.sock_path,
                                payload_profile_dir=self.profile_dir)
        self.app = create_app(self.client, self.config)
        self.tc = TestClient(self.app)
        self.tc.__enter__()
        deadline = time.time() + 4.0
        while time.time() < deadline and self.tc.get("/api/state").status_code != 200:
            time.sleep(0.02)

    def tearDown(self) -> None:
        try:
            self.tc.__exit__(None, None, None)
        finally:
            self.client.stop()
            self.fake.stop()
            self._tmp.cleanup()

    # -- listing ----------------------------------------------------------
    def test_lists_yaml_basenames_sorted(self):
        for name in ("heavy.yaml", "conservative.yaml", "notes.md",
                     ".hidden.yaml"):
            with open(os.path.join(self.profile_dir, name), "w") as f:
                f.write("# fixture\n")
        body = self.tc.get("/api/payload_profiles").json()
        self.assertEqual(body["profiles"], ["conservative", "heavy"])
        self.assertEqual(body["dir"], os.path.abspath(self.profile_dir))
        self.assertEqual(body["error"], "")

    def test_missing_dir_is_reported_with_the_path_not_swallowed(self):
        self.config.payload_profile_dir = os.path.join(self._tmp.name, "nope")
        body = self.tc.get("/api/payload_profiles").json()
        self.assertEqual(body["profiles"], [])
        self.assertIn("nope", body["error"])

    def test_empty_dir_points_at_the_real_cause(self):
        # The usual reason is webd and controld running from different working
        # directories, so the message names the knob to compare.
        body = self.tc.get("/api/payload_profiles").json()
        self.assertEqual(body["profiles"], [])
        self.assertIn("payload.profile_dir", body["error"])

    # -- command path -----------------------------------------------------
    def test_command_reaches_controld_with_its_argument(self):
        seen = []

        def handler(command, arg):
            seen.append((command, arg))
            from ..protocol import ResponseMessage
            if arg == "does_not_exist":
                return ResponseMessage(command=command, ok=False,
                                       error="no payload profile named "
                                             "'does_not_exist'")
            return ResponseMessage(command=command, ok=True, error="")

        self.fake.set_command_handler(handler)
        r = self.tc.post("/api/command",
                         json={"command": "select_payload_profile",
                               "arg": "heavy"}).json()
        self.assertTrue(r["ok"])
        self.assertEqual(seen[-1], ("select_payload_profile", "heavy"))

        r = self.tc.post("/api/command",
                         json={"command": "select_payload_profile",
                               "arg": "does_not_exist"}).json()
        self.assertFalse(r["ok"])
        # The reason must survive the relay: that string is the operator's only
        # explanation of why nothing changed.
        self.assertIn("does_not_exist", r["error"])


class DashboardSurfaceTest(unittest.TestCase):
    """The UI wiring is plain text in dashboard.py; assert it stays wired."""

    def test_profile_picker_is_present_and_wired(self):
        for token in ('data-cmd="select_payload_profile"',
                      'id="profile-select"', '"/api/payload_profiles"',
                      'wireCommands("p-payload")'):
            self.assertIn(token, DASHBOARD_HTML, token)

    def test_the_picker_explains_caps_vs_commissioning(self):
        """Silence here causes the wrong mental model: selecting a profile is
        not the same as commissioning verified payload data (§31.3)."""
        self.assertIn("CAPS", DASHBOARD_HTML)
        self.assertIn("Start payload verification", DASHBOARD_HTML)


if __name__ == "__main__":
    unittest.main()
