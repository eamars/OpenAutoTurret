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
        self.assertIn("bottom: 65px", HUD_HTML)           # where the mode buttons sat, a constant on purpose
        self.assertIn("left: 50%", HUD_HTML)           # the same column the mode block is in
        for gone in ("pipopen", "placePip", "pictureBox", "ResizeObserver", "pipclose"):
            self.assertNotIn(gone, HUD_HTML,
                             f"{gone} came back: the secondary preview is pinned and always open")
        self.assertIn("otaSwapPip", HUD_HTML)
        self.assertIn("translateX(-50%)", HUD_HTML)   # centred by a constant, not by measuring
        self.assertEqual(HUD_HTML.count('addEventListener("error"'), 2,
                         "both the main preview and the secondary pane must re-ask when a "
                         "stream dies; only the main one did, so the HQ feed needed a reload")
        # ...and so does the secondary pane: a deploy ends the multipart response, and a browser does
        # not retry a broken <img>, which is why the HQ feed used to need a page reload.
         # swap survives: it does something, it isn't chrome
    def test_the_preview_paints_under_the_chrome(self):
        """Chrome wins over a preview: the pane may be covered, it must not cover a control.

        The pane was raised to z-index 40 and sat on top of the aim pad; the owner kept the pinned
        position and asked for the stacking order to be inverted instead of moving anything.
        """
        from ..hud import HUD_HTML
        pane = HUD_HTML[HUD_HTML.index("#pip {"):HUD_HTML.index("#pip img")]
        self.assertIn("z-index: 15", pane)
        self.assertNotIn("z-index: 4", pane)          # nothing back above the chrome layer



class ServedPageIsWellFormed(unittest.TestCase):
    """Every inline script in the served page has to parse.

    A replacement in the PIP block once swallowed its closing `</script>`, so the rest of the
    document -- the SVG overlay, the style, everything -- arrived inside the script and the browser
    threw before the HUD drew a pixel. Token greps ("is `top: 88px` in the page?") all passed; the
    page was still dead. The document is the artifact, so the artifact is what gets checked.
    """

    def test_inline_scripts_parse(self):
        import shutil
        import subprocess
        import tempfile
        import re
        from ..hud import HUD_HTML
        self.assertEqual(HUD_HTML.count("<script"), HUD_HTML.count("</script>"),
                         "an inline script was left unterminated: everything after it is JS as far "
                         "as the browser is concerned")
        node = shutil.which("node")
        if node is None:
            self.skipTest("no node on this host")
        for i, body in enumerate(re.findall(r"<script>(.*?)</script>", HUD_HTML, re.S)):
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
                fh.write(body)
                path = fh.name
            proc = subprocess.run([node, "--check", path], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0,
                             f"inline script {i} does not parse: {proc.stderr[:400]}")
