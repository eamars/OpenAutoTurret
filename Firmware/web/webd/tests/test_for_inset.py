"""§11 field-of-regard inset: proportions, scale honesty, and the block that feeds it.

The inset is mostly geometry, and geometry is where this revision is unusually testable: §11.2 gives
proportions as percentages and §11.3 says which numbers derive from which. So those are asserted
literally, under node, against the page's own functions.

The test that matters most is the one that does not look like a §11 test at all. Fitting the yaw and
pitch spans independently would fill the box attractively and would make the white FOV rectangle
misreport the camera's field - a shape defect no proportion test can see. The aspect assertion is
there to keep that shortcut from ever looking like an improvement.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import time
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..controld_client import ControldClient
from ..config import WebConfig
from ..fake_controld import FakeControld
from ..hud import HUD_GEOMETRY_JS, HUD_JS
from ..protocol import Telemetry
from . import fake_camera

C_TOKENS = {
    "green": "#95f58b", "dim": "rgba(149,245,139,.56)", "faint": "rgba(149,245,139,.22)",
    "amber": "#f2b329", "red": "#ff5d5d", "white": "#edf2eb", "black": "rgba(3,6,5,.80)",
    "line": "rgba(230,245,230,.24)",
}

# This turret's measured envelope, in joint degrees, as controld publishes it.
ENV = [[-22.573, -74.712], [320.144, -74.712], [320.144, -4.891], [-22.573, -4.891]]
LOS = [174.1, -43.6]
HFOV, VFOV = 69.3002, 40.4171
VIEW = {"vw": 1920.0, "vh": 1080.0}


def _base(**over):
    o = {"pts": ENV, "hfovDeg": HFOV, "vfovDeg": VFOV, "los": LOS}
    o.update(VIEW)
    o.update(over)
    return o


@unittest.skipUnless(shutil.which("node"), "node not installed; the inset cannot be executed")
class ForInsetGeometry(unittest.TestCase):
    maxDiff = None

    def _eval(self, expr: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudForInset };\n")
            geo = fh.name
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
                fh.write("const T = require(%r);\nconsole.log(JSON.stringify(%s));" % (geo, expr))
                main = fh.name
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(geo)
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        return None if out == "null" else json.loads(out)

    def test_it_refuses_to_draw_without_a_real_envelope(self) -> None:
        # An inset with no envelope would be a frame around nothing, and the operator would read the
        # frame as a limit. Every one of these inputs has to be present and sane.
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(pts=[]))))
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(pts=ENV[:2]))))
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(hfovDeg=0))))
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(vfovDeg=-1))))
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(los=None))))
        self.assertIsNone(self._eval("T.hudForInset(%s)" % json.dumps(_base(pts=ENV[:3] + [[None, 1]]))))

    def test_it_occupies_the_band_the_revision_specifies(self) -> None:
        # §11.2: width 25-27% of viewport, height 20-23%. Asserted as the literal band, at two viewport
        # sizes, because "compact" is not a number and a card that creeps to 35% still looks compact.
        for vw, vh in ((1920.0, 1080.0), (1280.0, 720.0), (2560.0, 1440.0)):
            g = self._eval("T.hudForInset(%s)" % json.dumps(_base(vw=vw, vh=vh)))
            self.assertGreaterEqual(g["w"] / vw, 0.25, "inset too narrow at %sx%s" % (vw, vh))
            self.assertLessEqual(g["w"] / vw, 0.27, "§11.2 upper bound exceeded: scene stops dominant")
            self.assertGreaterEqual(g["h"] / vh, 0.20)
            self.assertLessEqual(g["h"] / vh, 0.23)

    def test_it_is_at_the_lower_left(self) -> None:
        g = self._eval("T.hudForInset(%s)" % json.dumps(_base()))
        self.assertLess(g["x"] / VIEW["vw"], 0.05, "§11 places the inset at the lower left")
        self.assertGreater(g["y"] + g["h"], VIEW["vh"] * 0.70)
        self.assertLess(g["y"] + g["h"], VIEW["vh"], "it must clear the §12 status strip")

    def test_one_scale_serves_both_axes(self) -> None:
        # The shortcut this test exists to block: scale yaw by width/spanY and pitch by height/spanP
        # independently, which fills the box beautifully and makes the FOV rectangle the wrong shape.
        # The rectangle's whole purpose is comparing "what the camera sees" against "where the turret
        # may point", and that comparison needs one scale.
        g = self._eval("T.hudForInset(%s)" % json.dumps(_base()))
        self.assertAlmostEqual(g["fov"]["w"] / g["fov"]["h"], HFOV / VFOV, places=6,
                               msg="the FOV rectangle no longer has the camera's aspect ratio")
        # degrees round-trip: the rectangle's width IS the effective HFOV at the inset's scale
        self.assertAlmostEqual(g["fov"]["w"] / g["scale"], HFOV, places=6)
        self.assertAlmostEqual(g["fov"]["h"] / g["scale"], VFOV, places=6)
        # and one scale, not two, is what makes that true
        self.assertNotIn("spanY *", HUD_GEOMETRY_JS.replace("plot.w / spanY", ""),
                         "a second per-axis scale appeared; §11's comparison needs one")

    def test_the_fov_rectangle_is_centred_on_the_los_marker(self) -> None:
        # §11.3: "camera FOV center derives from actual yaw/pitch".
        g = self._eval("T.hudForInset(%s)" % json.dumps(_base()))
        self.assertAlmostEqual(g["fov"]["x"] + g["fov"]["w"] / 2, g["los"]["x"], places=6)
        self.assertAlmostEqual(g["fov"]["y"] + g["fov"]["h"] / 2, g["los"]["y"], places=6)

    def test_yaw_drives_x_and_pitch_drives_y(self) -> None:
        # The axis-mixup test. Two envelope vertices share a pitch, so their pixel rows must be equal;
        # two share a yaw, so their columns must be equal. A transposed pair of coordinates would keep
        # the rectangle looking like a rectangle while pointing at the wrong part of the sky, which is
        # precisely the error this station cannot see for itself.
        g = self._eval("T.hudForInset(%s)" % json.dumps(_base()))
        e = g["envPx"]
        self.assertEqual(len(e), len(ENV), "one vertex per published point")
        self.assertAlmostEqual(e[0]["y"], e[1]["y"], places=6, msg="same pitch, different rows")
        self.assertGreater(abs(e[1]["x"] - e[0]["x"]), 1.0, "same span, no horizontal movement")
        self.assertAlmostEqual(e[1]["x"], e[2]["x"], places=6, msg="same yaw, different columns")
        self.assertGreater(abs(e[2]["y"] - e[1]["y"]), 1.0)
        # pitch increasing must move UP the screen, so row decreasing
        self.assertLess(e[2]["y"], e[1]["y"], "higher pitch drew lower")

    def test_markers_beyond_the_map_are_pinned_and_flagged(self) -> None:
        # Dropping an unreachable target would make "the axis cannot get there" look like "no target".
        far = self._eval("T.hudForInset(%s)" % json.dumps(_base(target=[-140.0, 40.0],
                                                                pred=[900.0, -43.6])))
        self.assertTrue(far["target"]["off"])
        self.assertTrue(far["pred"]["off"])
        p = far["plot"]
        self.assertGreaterEqual(far["target"]["x"], p["x"] - 1e-6)
        self.assertLessEqual(far["target"]["x"], p["x"] + p["w"] + 1e-6)
        self.assertGreaterEqual(far["target"]["y"], p["y"] - 1e-6)
        self.assertLessEqual(far["target"]["y"], p["y"] + p["h"] + 1e-6)

    def test_markers_that_do_not_exist_are_not_invented(self) -> None:
        g = self._eval("T.hudForInset(%s)" % json.dumps(_base()))
        self.assertIsNone(g["target"])
        self.assertIsNone(g["pred"])


@unittest.skipUnless(shutil.which("node"), "node not installed; the inset cannot be executed")
class ForInsetMarkup(unittest.TestCase):
    maxDiff = None

    def _svg(self, opts: dict) -> str:
        js = ("const g = T.hudForInset(%s);\n"
              "console.log(g ? S.hudForInsetSvg(g, %s) : '');"
              % (json.dumps(_base(**opts)), json.dumps(C_TOKENS)))
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudForInset, hudForInsetSvg };\n")
            geo = fh.name
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
                fh.write("const T = require(%r);\nconst S = T;\n%s" % (geo, js))
                main = fh.name
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(geo)
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout[:-1] if r.stdout.endswith("\n") else r.stdout

    def test_the_words_the_revision_asks_for_are_present(self) -> None:
        svg = self._svg({})
        self.assertIn("FIELD OF REGARD", svg)
        self.assertIn("SAFE ENVELOPE", svg)
        self.assertIn("<polygon", svg, "§11: green safe-envelope polygon")
        self.assertIn("FOV", svg)
        self.assertIn("TARGET", svg)
        self.assertIn("PRED", svg, "§11: short legend")

    def test_the_predicted_marker_is_amber_not_green(self) -> None:
        # Same rule as §10, enforced here too: two green symbols side by side would put an intention
        # and a measurement in the same uniform.
        svg = self._svg({"pred": [180.0, -40.0], "target": [170.0, -45.0]})
        pred = svg[svg.rindex("<path"):]
        self.assertIn(C_TOKENS["amber"], pred)
        self.assertNotIn(C_TOKENS["green"], pred)
        tgt = svg[svg.index("<circle"):svg.index("<path")]
        self.assertIn(C_TOKENS["green"], tgt)

    def test_envelope_is_green_and_the_fov_rectangle_is_white(self) -> None:
        svg = self._svg({})
        poly = svg[svg.index("<polygon"):svg.index("SAFE ENVELOPE")]
        self.assertIn(C_TOKENS["green"], poly)
        fov = svg[svg.index("<rect", svg.index("SAFE ENVELOPE")):svg.index("<line", svg.index("SAFE ENVELOPE"))]
        self.assertIn(C_TOKENS["white"], fov, "§11: white current-camera FOV rectangle")

    def test_the_box_is_low_opacity_not_a_card(self) -> None:
        # §11: "not a full dashboard card", and §11.2 keeps the scene dominant.
        svg = self._svg({})
        self.assertIn('fill-opacity=".28"', svg)
        self.assertNotIn("<filter", svg)

    def test_it_draws_nothing_when_the_daemon_has_no_envelope(self) -> None:
        self.assertEqual(self._svg({"pts": [[0, 0], [1, 0]]}), "")


class ForEnvelopeCrossesTheBoundary(unittest.TestCase):
    """controld's block -> ControldClient -> protocol -> /api/state, keys intact."""

    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_for_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.app = create_app(self.client, WebConfig(host="127.0.0.1", port=0, socket_path=self.sock))
        self.http = TestClient(self.app)
        self.http.__enter__()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            # latest_telemetry() hands back a Telemetry object, not a dict, so the block is watched
            # where it is actually consumed: the payload the page reads.
            body = self.http.get("/api/state").json()
            if isinstance(body.get("field_of_regard"), dict):
                break
            time.sleep(0.05)

    def tearDown(self) -> None:
        try:
            self.http.__exit__(None, None, None)
        finally:
            self.client.stop()
        self.fake.stop()
        fake_camera.restore(self.tc)
        self._tmp.cleanup()

    def test_protocol_declares_the_block(self) -> None:
        # Not trivia: telemetry_from_json drops keys it has not seen, so an undeclared field is
        # indistinguishable from one the daemon never sent. This landed on the project twice already.
        self.assertIn("field_of_regard", Telemetry.__dataclass_fields__)

    def test_the_polygon_arrives_whole(self) -> None:
        body = self.http.get("/api/state").json()
        f = body.get("field_of_regard")
        self.assertIsInstance(f, dict, "the block vanished somewhere between daemon and page")
        pts = f.get("safe_envelope_points")
        self.assertIsInstance(pts, list)
        self.assertGreaterEqual(len(pts), 3, "a region needs at least three vertices")
        self.assertEqual(f.get("coordinate_frame"), "joint_deg", "§11.3")
        self.assertTrue(f.get("valid"))

    def test_the_page_will_not_draw_an_unlabelled_frame(self) -> None:
        # If the server ever changes the frame the polygon is in, the inset must go blank rather than
        # keep drawing joint travel under a heading-shaped label.
        self.assertIn('forB.coordinate_frame === "joint_deg"', HUD_JS)
        self.assertIn("forB.valid === true", HUD_JS)


if __name__ == "__main__":
    unittest.main()
