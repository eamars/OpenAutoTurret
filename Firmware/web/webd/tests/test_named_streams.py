"""The named-stream surface: three promises the (b) split makes to whoever opens the HUD.

Kept small and blunt on purpose. Each case is a lie this API could tell: serve the wide stream
under a name nobody published, fall back silently when a role is misspelled, or report a camera
with no identity because the daemon's manifest is missing.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from fastapi.testclient import TestClient  # noqa: E402

from web.webd.app import create_app  # noqa: E402
from web.webd.config import WebConfig  # noqa: E402


class _NoClient:
    """Enough of ControldClient for these routes: no telemetry, not connected."""

    def latest_telemetry(self):
        return None

    def connected(self):
        return False

    def start(self):
        pass

    def stop(self):
        pass


def _manifest(box: str, roles=("wide",)) -> str:
    path = os.path.join(box, "video_streams.json")
    streams = {role: {"role": role, "camera_id": f"cam-{role}", "identity_source": "fwnode",
                      "durable": True, "path": os.path.join(box, f"{role}.jpg"),
                      "transport": "atomic-jpeg-file", "width": 1920, "height": 1080,
                      "delivered_fps": None, "dropped": 0, "updated_ns": 1}
               for role in roles}
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"version": 1, "producer": "visiond", "streams": streams}, handle)
    return path


class NamedStreamApi(unittest.TestCase):
    def _client(self, manifest_path: str) -> TestClient:
        config = WebConfig(stream_manifest=manifest_path, imu_trace="")
        app = create_app(_NoClient(), config)    # type: ignore[arg-type]
        return TestClient(app)

    def test_an_unknown_role_is_refused_with_the_list_instead_of_defaulting(self):
        with tempfile.TemporaryDirectory() as box:
            client = self._client(_manifest(box))
            r = client.get("/api/video?camera=left")
            self.assertEqual(r.status_code, 400)
            self.assertEqual(r.json()["roles"], ["wide", "detail"])

    def test_a_stream_nobody_published_is_not_started_anyway(self):
        with tempfile.TemporaryDirectory() as box:
            client = self._client(_manifest(box, roles=("wide",)))
            r = client.post("/api/video/start?camera=detail")
            body = r.json()
            self.assertFalse(body["ok"])
            self.assertIn("no detail stream published", body["error"])

    def test_the_legacy_default_still_works_and_says_which_path_it_took(self):
        # No manifest at all: `wide` must still start on the pre-(b) path (the tapped JPEG file),
        # and the response must say it fell back rather than pretending to be a published stream.
        with tempfile.TemporaryDirectory() as box:
            tap = os.path.join(box, "preview.jpg")
            with open(tap, "wb") as handle:
                handle.write(bytes([0xff, 0xd8]) + b"\x00fake jpeg" + bytes([0xff, 0xd9]))
            os.environ["OTA_VISION_FRAME_TAP"] = tap
            try:
                body = self._client(os.path.join(box, "absent.json")).post("/api/video/start").json()
            finally:
                del os.environ["OTA_VISION_FRAME_TAP"]
            self.assertTrue(body.get("ok"), body)
            self.assertIn("no manifest entry", body["manifest_fallback"])

    def test_no_telemetry_means_no_state_rather_than_a_state_full_of_zeroes(self):
        with tempfile.TemporaryDirectory() as box:
            client = self._client(_manifest(box))
            r = client.get("/api/state")
            self.assertEqual(r.status_code, 503, "没有遥测时不该端出一台假站")


if __name__ == "__main__":
    unittest.main()


class PinnedPreviewContract(unittest.TestCase):
    """The owner's ruling of 2026-09-29: always open, pinned to the corner, no movable window.

    A toggle and a measured placement were both tried and both taken out. The pane moved when the
    window moved -- his words were "咋还乱跑呢" -- and the button existed only to reopen a pane that
    the placement had hidden, so it was a symptom of the placement rather than a feature. This test
    is here so nobody re-adds either one later.
    """

    def test_the_pane_is_pinned_and_there_is_no_toggle(self):
        from ..hud import HUD_HTML
        self.assertIn("top: 88px", HUD_HTML)          # below the mode block, a constant on purpose
        self.assertIn("left: 1%", HUD_HTML)           # the same column the mode block is in
        for gone in ("pipopen", "placePip", "pictureBox", "ResizeObserver", "pipclose"):
            self.assertNotIn(gone, HUD_HTML,
                             f"{gone} came back: the secondary preview is pinned and always open")
        self.assertIn("otaSwapPip", HUD_HTML)         # swap survives: it does something, it isn't chrome
