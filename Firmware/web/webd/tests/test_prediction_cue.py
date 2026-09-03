"""§10 prediction cue: the contract path, and the rule that it must not be green.

Two halves again. The data-contract half matters because `telemetry_from_json` silently drops keys it
has not seen - twice on this project that has produced a confident report that "controld does not
publish X" when the truth was that webd's own parser discarded it. So the block is pushed through the
real boundary (fake daemon -> client -> protocol -> /api/state) and checked for every key §20 names.

The visual half is executed under node from the page's own source. §10 is unusual in this revision in
that one of its rules is a safety statement rather than a style preference: the prediction "must not
be green", because green on this display means *measured*.
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
from ..fake_controld import FakeControld
from ..hud import HUD_GEOMETRY_JS, HUD_HTML, HUD_JS
from . import fake_camera

C_TOKENS = {
    "green": "#95f58b", "dim": "rgba(149,245,139,.56)", "faint": "rgba(149,245,139,.22)",
    "amber": "#f2b329", "red": "#ff5d5d", "white": "#edf2eb", "black": "rgba(3,6,5,.80)",
    "line": "rgba(230,245,230,.24)",
}

_PRED = {
    "valid": True,
    "predicted_los_yaw_deg": 3.4,
    "predicted_los_pitch_deg": -1.1,
    "predicted_anchor_norm": [0.531, 0.492],
    "anchor_in_frame": True,
    "horizon_ms": 40,
}

_CONTRACT_KEYS = ("valid", "predicted_los_yaw_deg", "predicted_los_pitch_deg",
                  "predicted_anchor_norm", "horizon_ms")


class PredictionBlockCrossesTheBoundary(unittest.TestCase):
    """fake controld -> ControldClient -> protocol -> /api/state, with every §20 key intact."""

    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_pred_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.set_telemetry(prediction=dict(_PRED))
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        from ..config import WebConfig
        self.app = create_app(self.client, WebConfig(host="127.0.0.1", port=0, socket_path=self.sock))
        self.http = TestClient(self.app)
        self.http.__enter__()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            body = self.http.get("/api/state").json() if self.client.latest_telemetry() else {}
            if isinstance(body.get("prediction"), dict):
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

    def test_every_section_20_prediction_key_arrives(self) -> None:
        body = self.http.get("/api/state").json()
        pred = body.get("prediction")
        self.assertIsInstance(pred, dict, "the whole block vanished - that is what a parser that "
                                          "drops unknown keys looks like from the outside")
        for key in _CONTRACT_KEYS:
            self.assertIn(key, pred, "§20 names prediction.%s and the page never received it" % key)
        self.assertEqual(pred["predicted_anchor_norm"], _PRED["predicted_anchor_norm"])
        self.assertEqual(pred["horizon_ms"], 40)
        self.assertTrue(pred["valid"])

    def test_the_page_gates_on_both_validity_fields(self) -> None:
        # `valid` (a prediction exists) and `anchor_in_frame` (it is on the picture) are different
        # facts and the page has to check both: an off-edge anchor is a real prediction that must not
        # be painted as a box half off the bezel.
        self.assertIn("pred.valid === true", HUD_JS)
        self.assertIn("anchor_in_frame === true", HUD_JS)

    def test_the_cue_disappears_when_stale_or_invalid(self) -> None:
        # §10/§661: prediction disappears when invalid or stale. Three gates have to be present in
        # the render path, and the idle default of the fake daemon has to be "not predicting" rather
        # than a zero at the centre of the frame, which would draw a confident cue on an idle station.
        block = HUD_JS[HUD_JS.index("const pred ="):HUD_JS.index("§5 + §6 travel tapes")]
        for gate in ("pred.valid === true", "anchor_in_frame === true", "!stale"):
            self.assertIn(gate, block, "the prediction block is missing its %s gate" % gate)
        idle = FakeControld(self.sock)._telemetry.prediction
        self.assertFalse(idle["valid"], "an idle daemon must not claim to be predicting")
        self.assertFalse(idle["anchor_in_frame"])


@unittest.skipUnless(shutil.which("node"), "node not installed; the cue cannot be executed")
class PredictionCueExecuted(unittest.TestCase):
    maxDiff = None

    def _node(self, script: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS +
                     "\nmodule.exports = { hudPredictionBox, hudPredictionSvg };\n")
            geo = fh.name
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
                fh.write("const T = require(%r);\n%s" % (geo, script))
                main = fh.name
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        finally:
            os.unlink(geo)
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        try:
            return json.loads(out)
        except json.JSONDecodeError:
            return out

    def _svg(self, opts: str, box: str) -> str:
        return self._node("console.log(T.hudPredictionSvg(T.hudPredictionBox(%s), %s, %s));"
                          % (box, json.dumps(C_TOKENS), opts))

    _NEAR_BOX = "{ cx: 600, cy: 400, w: 80, h: 120, box: [560, 340, 640, 460] }"

    def test_it_is_amber_and_never_green(self) -> None:
        svg = self._svg("{ near: [600, 400] }", self._NEAR_BOX)
        self.assertIn(C_TOKENS["amber"], svg)
        self.assertNotIn(C_TOKENS["green"], svg,
                         "§10: the prediction cue must not be green - green is measured data")
        self.assertNotIn("rgba(149,245", svg, "a dim/faint green token would still read as green")

    def test_dashed_square_cross_and_label(self) -> None:
        svg = self._svg("{}", self._NEAR_BOX)
        self.assertIn("stroke-dasharray", svg, "§10: dashed square")
        self.assertIn(">PRED<", svg, "§10: small PRED label")
        self.assertIn("<rect", svg)
        # the small amber + at the centre is two short strokes crossing at the cue's centre
        self.assertEqual(svg.count('stroke-width="1.2"'), 3,
                         "square + two cross strokes; a missing stroke is a missing '+'")

    def test_it_does_not_touch_the_measured_box(self) -> None:
        # §10: "placed near the selected target but not touching its box". Tested at the worst case -
        # the prediction landing exactly on the target, which is what a settled loop does.
        box = [560, 340, 640, 460]
        gap = 8.0

        def touching(b):
            # Rectangles inflated by the gap. The rule is "not touching", which is true in any
            # direction: the first version of this test demanded that the cue end up to the RIGHT of
            # the box, and failed on an anchor a tenth of a pixel above the box centre, where the cue
            # correctly moved straight up and cleared the box vertically. That assertion was testing
            # my expectation of how the rule should be implemented rather than the rule.
            return (b["x"] < box[2] + gap and b["x"] + b["w"] > box[0] - gap and
                    b["y"] < box[3] + gap and b["y"] + b["h"] > box[1] - gap)

        for anchor in ([600, 400], [600, 400.0001], [600, 399.9], [600, 400.0], [599.99, 400.02]):
            b = self._node("console.log(JSON.stringify(T.hudPredictionBox(%s)));"
                           % json.dumps({"cx": anchor[0], "cy": anchor[1], "w": 80, "h": 120,
                                         "box": box, "gap": gap}))
            self.assertFalse(touching(b), "cue %r touches or overlaps the measured box" % b)
            self.assertTrue(b["shifted"])
            # and it must still be a cue of the size it was given, not collapsed by the shove
            self.assertAlmostEqual(b["w"], 80.0)
            self.assertAlmostEqual(b["h"], 120.0)

    def test_a_far_prediction_gets_no_error_vector(self) -> None:
        # §10: "a long error vector across the image is not shown by default", so a distant cue draws
        # the square alone: exactly the two cross strokes, no connector. The connector is the subtle
        # one at opacity .28, so its absence is what is being counted here.
        far = self._svg("{ near: [520, 300] }", "{ cx: 960, cy: 540, w: 60, h: 60 }")
        self.assertNotIn('opacity=".28"', far)
        near = self._svg("{ near: [880, 520] }", "{ cx: 960, cy: 540, w: 60, h: 60 }")
        self.assertIn('opacity=".28"', near, "a close pair should get the subtle connector")
        self.assertLess(far.count("<line"), near.count("<line"))

    def test_layer_order_puts_the_cue_under_the_reticle(self) -> None:
        order = [HUD_HTML.index(i) for i in ('id="g-selected"', 'id="g-prediction"',
                                            'id="g-reticle"')]
        self.assertEqual(order, sorted(order),
                         "§18: prediction is z=12, the reticle z=20, and this page has no SVG z-index")

    def test_no_invented_helpers_are_called(self) -> None:
        # Every mapping the cue uses has to exist. An earlier draft called hudMapNorm/hudMapU/hudMapV,
        # names that sounded plausible and were not in the file, which in a page script means the
        # whole render throws and the HUD draws nothing while the server returns 200.
        helpers = set()
        for name in ("hudProject", "hudLayout", "hudPredictionBox", "hudPredictionSvg"):
            if name + "(" in HUD_JS or name + "(" in HUD_GEOMETRY_JS:
                helpers.add(name)
        for name in helpers:
            self.assertIn("function " + name, HUD_GEOMETRY_JS + HUD_JS,
                          "%s is called but never defined" % name)


if __name__ == "__main__":
    unittest.main()
