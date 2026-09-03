"""The HUD page is served as the operator view, and its geometry is checked where it runs.

Two things this is trying to be honest about:

1. `docs/open_auto_turret_v3_2_apache_hud_ui_revision.md` s3 forbids the header-plus-card
   layout and makes the camera dominant. Checking that means asserting on what the served
   document contains and omits, not on how it looks to me.

2. The one part of the browser side that can be silently wrong is the projection from the
   detector's normalised coordinates to CSS pixels. A Python re-implementation of that math
   in a test proves nothing, because the page does not execute Python. So the test executes
   THE ACTUAL BYTES the browser receives, under node, and compares them against an
   independent computation. If somebody edits the page's layout math, this test is the one
   that complains.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
import tempfile
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..config import WebConfig
from ..controld_client import ControldClient
from ..fake_controld import FakeControld
from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS
from . import fake_camera


class HudServedTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_hud_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.config = WebConfig(host="127.0.0.1", port=0, socket_path=self.sock)
        self.app = create_app(self.client, self.config)
        # Entering the TestClient is what runs the lifespan, and the lifespan is what starts the
        # controld client; without it /api/state honestly reports 503 and nothing is connected.
        self.http = TestClient(self.app)
        self.http.__enter__()

    def tearDown(self) -> None:
        try:
            self.http.__exit__(None, None, None)
        finally:
            self.client.stop()
        self.fake.stop()
        fake_camera.restore(self.tc)
        self._tmp.cleanup()

    def test_operator_view_is_the_hud_not_the_card_dashboard(self):
        r = self.http.get("/")
        self.assertEqual(r.status_code, 200)
        body = r.text

        # s3's hierarchy: camera dominant, symbology overlaid.
        for marker in ('id="video"', 'id="overlay"', 'id="mode-block"', 'id="health"', 'id="strip"'):
            self.assertIn(marker, body, "HUD is missing " + marker)

        # s18 layering. The HTML layers are z-indexed in the stylesheet; the three overlay
        # layers live in one SVG, where z-index has no effect on children, so what has to be
        # asserted is that they are PAINTED in that order - document order is the mechanism.
        for z in ("z-index: 0", "z-index: 10", "z-index: 20"):
            self.assertIn(z, HUD_CSS, "layering spec lost: " + z)
        order = [body.index(m) for m in ('id="video"', 'id="g-candidates"', 'id="g-selected"',
                                         'id="g-reticle"')]
        self.assertEqual(order, sorted(order), "overlay layers must paint candidates < selected < reticle")
        self.assertNotIn('style="z-index', body,
                         "z-index on an SVG group is decoration; the renderer ignores it")

        # s15 tokens and s16 typography, verbatim.
        self.assertIn("--hud-green: #95f58b", HUD_CSS)
        self.assertIn("--hud-amber: #f2b329", HUD_CSS)
        self.assertIn('"IBM Plex Mono"', HUD_CSS)

        # s7: the reticle is the optical axis and says so when it cannot know where that is.
        self.assertIn("hudAxisNorm", HUD_GEOMETRY_JS)
        self.assertIn("RETICLE UNCALIBRATED", HUD_HTML)

        # s25: stale telemetry must stop looking current.
        self.assertIn("TELEMETRY STALE / DISCONNECTED", HUD_CSS)

        # The whole frame stays visible: cropping would hide the frame edge the operator has
        # to judge against. This is a behaviour assertion, not a style preference.
        self.assertIn("object-fit: contain", HUD_CSS)

        # And what s3 rules out stays out of the operator view.
        self.assertNotIn('class="card"', body)

    def test_page_starts_the_preview_and_says_why_if_it_cannot(self):
        """Restarting webd leaves /api/video answering 409 until something asks for the stream.
        That shipped a black video panel behind correct symbology exactly once, so the page now
        asks for the preview, re-asks when the <img> errors, and reports the reason instead of
        letting the operator infer a missing target from a missing picture."""
        self.assertIn("/api/video/start", HUD_JS)
        self.assertIn("VIDEO UNAVAILABLE", HUD_JS)
        self.assertIn('/api/video/state', HUD_JS)          # self-heal, not a single attempt
        self.assertIn('addEventListener("error"', HUD_JS)  # a stopped stream has to be noticed

    def test_legacy_engineering_page_survives_at_its_own_path(self):
        """The HUD is not allowed to delete the numbers it has not replaced yet."""
        r = self.http.get("/dashboard")
        self.assertEqual(r.status_code, 200)
        self.assertNotIn("--hud-green", r.text)

    def test_api_surface_is_unchanged(self):
        """The presentation revision must not disturb what other clients depend on."""
        self.assertEqual(self.http.get("/api/health").status_code, 200)
        # /api/state answers 503 until the first snapshot arrives from controld, which is the
        # honest answer and stays. Give the fake daemon a moment to produce one.
        deadline = time.time() + 5.0
        code = 503
        while time.time() < deadline:
            code = self.http.get("/api/state").status_code
            if code == 200:
                break
            time.sleep(0.1)
        self.assertEqual(code, 200)


class HudProjectionTest(unittest.TestCase):
    """Run the page's own geometry under node. No Python re-implementation allowed."""

    @classmethod
    def setUpClass(cls):
        cls.node = shutil.which("node")

    def run_geometry(self, script: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS + "\n" + script + "\n")
            path = fh.name
        try:
            out = subprocess.run([self.node, path], capture_output=True, text=True, timeout=20)
        finally:
            os.unlink(path)
        self.assertEqual(out.returncode, 0, out.stderr)
        return json.loads(out.stdout)

    def setUp(self):
        if not self.node:
            self.skipTest("node not available; the page's JS cannot be executed here")

    def test_contain_layout_and_projection_match_independent_arithmetic(self):
        # 1920x1080 into a viewport that is wider and shorter than 16:9, so the letterbox is
        # on the top and bottom and a naive "fill the box" mapping would be visibly wrong.
        vw, vh, iw, ih = 1600, 700, 1920, 1080
        got = self.run_geometry(
            'const lay = hudLayout(%d, %d, %d, %d);' % (vw, vh, iw, ih) +
            'const pts = [[0,0],[1,1],[0.5,0.5],[0.25,0.75],[1.3,-0.2]];' +
            'console.log(JSON.stringify({lay: lay, pts: pts.map(p => hudProject(p[0], p[1], lay))}));'
        )
        s = min(vw / iw, vh / ih)
        w, h = iw * s, ih * s
        ox, oy = (vw - w) / 2.0, (vh - h) / 2.0
        self.assertAlmostEqual(got["lay"]["s"], s, places=9)
        self.assertAlmostEqual(got["lay"]["ox"], ox, places=9)
        self.assertAlmostEqual(got["lay"]["oy"], oy, places=9)
        for (u, v), p in zip([(0, 0), (1, 1), (0.5, 0.5), (0.25, 0.75), (1.3, -0.2)], got["pts"]):
            self.assertTrue(p["ok"])
            self.assertAlmostEqual(p["x"], ox + u * w, places=9)
            self.assertAlmostEqual(p["y"], oy + v * h, places=9)

        # Off-frame coordinates come back unsaturated, because "the target left the frame"
        # has to be distinguishable from "the target is at the edge".
        self.assertGreater(got["pts"][4]["x"], ox + w)
        self.assertLess(got["pts"][4]["y"], oy)

    def test_optical_axis_is_the_principal_point_not_the_viewport_centre(self):
        """s7 depends on this. Today cx/cy happen to be the centre; a measured principal
        point that is not must move the reticle, and must not be replaced by 0.5."""
        moved = self.run_geometry(
            "const a = hudAxisNorm({cx: 1010, cy: 512, width: 1920, height: 1080});"
            "const missing = hudAxisNorm(null);"
            "console.log(JSON.stringify({a: a, m: missing}));")
        self.assertAlmostEqual(moved["a"]["u"], 1010.0 / 1920.0, places=12)
        self.assertAlmostEqual(moved["a"]["v"], 512.0 / 1080.0, places=12)
        self.assertIsNotNone(moved["a"])
        self.assertIsNone(moved["m"])

    def test_degenerate_layouts_are_refused_not_guessed(self):
        got = self.run_geometry(
            "console.log(JSON.stringify(["
            "hudLayout(0, 0, 0, 0),"
            "hudProject(0.5, 0.5, hudLayout(800, 600, 0, 480)"
            ")]));")
        self.assertFalse(got[0]["ok"])
        self.assertFalse(got[1]["ok"])


if __name__ == "__main__":
    unittest.main()
