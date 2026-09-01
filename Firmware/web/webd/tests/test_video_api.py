"""Video API endpoint tests (TestClient) against a FakeControld + fake camera.

These exercise the on/off video endpoints and the MJPEG stream path end-to-end
without any real hardware: the telemetry path comes from FakeControld, the video
path from the fake picamera2. No CAN, no real camera.
"""
from __future__ import annotations

import os
import tempfile
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..config import WebConfig
from ..controld_client import ControldClient
from ..fake_controld import FakeControld
from . import fake_camera


class VideoApiTest(unittest.TestCase):
    def setUp(self) -> None:
        self._prev_cam = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_vapi_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.config = WebConfig(host="127.0.0.1", port=0, socket_path=self.sock)
        self.app = create_app(self.client, self.config)
        self.tc = TestClient(self.app)
        self.tc.__enter__()

    def tearDown(self) -> None:
        try:
            self.tc.__exit__(None, None, None)
        finally:
            self.client.stop()
            self.fake.stop()
            self._tmp.cleanup()
            fake_camera.restore(self._prev_cam)

    def _start(self, **kw):
        r = self.tc.post("/api/video/start", json=kw)
        self.assertEqual(r.status_code, 200)
        return r.json()

    def test_state_initially_off(self) -> None:
        r = self.tc.get("/api/video/state")
        self.assertEqual(r.status_code, 200)
        self.assertFalse(r.json()["running"])

    def test_stream_409_when_off(self) -> None:
        self.assertEqual(self.tc.get("/api/video").status_code, 409)

    def test_start_stop_and_stream(self) -> None:
        j = self._start()
        self.assertTrue(j["ok"])
        self.assertTrue(j["running"])
        self.assertEqual(j["camera"], "fake-imx500")
        self.assertEqual((j["width"], j["height"]),
                         (self.config.video_width, self.config.video_height))

        # State reflects the running source.
        st = self.tc.get("/api/video/state").json()
        self.assertTrue(st["running"])
        self.assertGreater(st["frames_published"], 0)

        # The MJPEG stream yields real JPEG frames (bounded via ?limit so the
        # response completes under TestClient).
        with self.tc.stream("GET", "/api/video?limit=3") as r:
            self.assertEqual(r.status_code, 200)
            self.assertIn("multipart/x-mixed-replace",
                          r.headers["content-type"])
            self.assertIn("boundary=frame", r.headers["content-type"])
            data = b"".join(r.iter_bytes())
        self.assertIn(b"\xff\xd8", data, "no JPEG frame in MJPEG stream")
        self.assertIn(b"image/jpeg", data)
        self.assertGreaterEqual(data.count(b"\xff\xd8"), 3, "expected >=3 frames")

        # Stop releases the camera and the stream goes away.
        j = self.tc.post("/api/video/stop").json()
        self.assertTrue(j["ok"])
        self.assertFalse(j["running"])
        self.assertEqual(self.tc.get("/api/video").status_code, 409)

    def test_start_custom_size(self) -> None:
        j = self._start(width=160, height=120, fps=5)
        self.assertTrue(j["running"])
        self.assertEqual((j["width"], j["height"]), (160, 120))

    def test_disabled_feature(self) -> None:
        # Rebuild the app with the video feature disabled (§53 kill switch).
        self.config.video_enabled = False
        app2 = create_app(self.client, self.config)
        with TestClient(app2) as tc2:
            r = tc2.post("/api/video/start", json={})
            j = r.json()
            self.assertFalse(j["ok"])
            self.assertFalse(j["running"])
            self.assertIn("disabled", j["error"])


if __name__ == "__main__":
    unittest.main()
