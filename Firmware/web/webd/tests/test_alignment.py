"""Real HUD JavaScript and Python telemetry contract; no video or camera service."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from ..hud import HUD_GEOMETRY_JS
from ..protocol import telemetry_from_json, telemetry_to_json


class AlignmentHudTest(unittest.TestCase):
    def setUp(self):
        self.data = {
            "type": "telemetry",
            "alignment": {"mode": "manual_depth", "range_source": "manual",
                          "range_measured": False, "assumed_depth_m": 10,
                          "wide": {"valid": True, "reason": "assumed depth",
                                   "x_norm": 949.5825/1920, "y_norm": 551.0025/1080,
                                   "camera_offset": {"right_mm": 75, "up_mm": 75, "forward_mm": 0,
                                                     "right_deg": 0, "up_deg": 0}},
                          "detail": {"valid": True, "reason": "assumed depth",
                                     "x_norm": 0.6, "y_norm": 0.4,
                                     "camera_offset": {"right_mm": -40, "up_mm": 30, "forward_mm": 0,
                                                       "right_deg": 0, "up_deg": 0}}},
            "aim_point_policy": {"revision": 1, "mode": "box_fraction", "x_fraction": .5, "y_fraction": .22},
            "target_aim_valid": True, "target_aim_source": "box_fraction",
            "target_aim_x_norm": .5, "target_aim_y_norm": .276,
            "target_aim_box_clipped": False,
        }
        self.temp = tempfile.TemporaryDirectory()
        self.program = Path(self.temp.name)/"alignment.js"
        self.program.write_text(HUD_GEOMETRY_JS + '''
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
const lay = hudLayout(input.width || 960, input.height || 540, 1920, 1080);
const mark = hudBoreMark(input.t, !!input.stale, input.main || "wide");
// The measurement point takes its palette as an argument, like the tape and the FoR do: a geometry
// module that quietly owned page colours would be a second place to declare them.
const C = {"green": "#95f58b", "dim": "rgba(149,245,139,.56)", "faint": "rgba(149,245,139,.22)", "amber": "#f2b329", "red": "#ff5d5d", "white": "#edf2eb", "black": "rgba(3,6,5,.80)", "stroke": "#05070a", "text": "#c5d0c5", "text_dim": "#8c998c"};
console.log(JSON.stringify({mark, pixel: mark ? hudProject(mark.u, mark.v, lay) : null,
  point: hudMeasurementPointSvg(input.t, lay, !!input.stale, C),
  reticle: hudReticle(input.t, !!input.stale, input.main || "wide", !!input.calibrating)}));
''', encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def render(self, data=None, **kwargs):
        result = subprocess.run(["node", str(self.program)],
                                input=json.dumps({"t": data or self.data, **kwargs}),
                                text=True, capture_output=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_protocol_preserves_policy_and_alignment(self):
        out = json.loads(telemetry_to_json(telemetry_from_json(self.data)))
        for key in ("alignment", "aim_point_policy", "target_aim_source", "target_aim_box_clipped"):
            self.assertEqual(out[key], self.data[key])
        self.assertEqual(self.render(out)["mark"]["label"], "RANGE 10 m")

    def test_crosshair_uses_controller_projection_and_contain_scaling(self):
        rendered = self.render(width=960, height=800)
        self.assertAlmostEqual(rendered["pixel"]["x"], 949.5825/2)
        self.assertAlmostEqual(rendered["pixel"]["y"], 130+551.0025/2)
        # §8 of the 09-30 revision: the measured point carries no word -- the green reticle is the
        # caption. The marker itself must still be drawn, and must not be captioned.
        self.assertIn("<path", rendered["point"])
        self.assertNotIn("MEASURE", rendered["point"])

    def test_no_virtual_bore_claim_when_disabled_invalid_or_stale(self):
        for key, value in (("mode", "off"), ("wide.valid", False), ("wide.x_norm", 1.1),
                           ("wide.y_norm", None), ("assumed_depth_m", 0),
                           ("range_measured", True), ("range_source", "sensor"), ("wide", None)):
            data = copy.deepcopy(self.data)
            owner, leaf = data["alignment"], key
            if "." in key:
                camera, leaf = key.split(".")
                owner = owner[camera]
            owner[leaf] = value
            with self.subTest(key=key):
                self.assertIsNone(self.render(data)["mark"])
        rendered = self.render(stale=True)
        self.assertIsNone(rendered["mark"])
        self.assertEqual(rendered["point"], "")

    def test_mark_follows_the_camera_on_the_main_display(self):
        # Each camera has its own offset from the bore, so each has its own mark.
        wide = self.render()["mark"]
        detail = self.render(main="detail")["mark"]
        self.assertAlmostEqual(wide["u"], 949.5825/1920)
        self.assertAlmostEqual(detail["u"], 0.6)
        self.assertAlmostEqual(detail["v"], 0.4)
        self.data["alignment"]["detail"]["valid"] = False
        self.assertIsNone(self.render(main="detail")["mark"], "no wide mark drawn on the detail picture")

    def test_calibration_aids_show_only_in_calibration(self):
        # Owner, 2026-10-25: "I can still see the RANGE 11 m marker outside of the calibration mode".
        # The reticle stays on the bore point -- the tracker centres targets there -- but the amber, the
        # working-distance caption and the optical-axis circle belong to MENU > CALIBRATION.
        out = self.render()["reticle"]
        self.assertAlmostEqual(out["u"], 949.5825/1920)
        self.assertEqual((out["amber"], out["label"], out["axis"], out["warning"]), (False, None, None, None))
        cal = self.render(calibrating=True)["reticle"]
        self.assertAlmostEqual(cal["u"], out["u"])
        self.assertEqual((cal["amber"], cal["label"]), (True, "RANGE 10 m"))
        self.assertEqual(cal["axis"], {"u": 0.5, "v": 0.5}, "no intrinsics here: the assumed centre")
        # A bore that cannot be drawn is degraded operation, so it says so whether calibrating or not.
        self.data["alignment"]["wide"]["valid"] = False
        for calibrating in (False, True):
            broken = self.render(calibrating=calibrating)["reticle"]
            self.assertEqual((broken["u"], broken["amber"], broken["warning"]), (0.5, False, "BORE ALIGNMENT UNAVAILABLE"))
        self.data["alignment"]["mode"] = "off"
        self.assertEqual(self.render(calibrating=True)["reticle"]["warning"], None)

    def test_fallback_and_clipping_are_visible(self):
        self.data["target_aim_source"] = "invalid_box_anchor_fallback"
        self.data["target_aim_box_clipped"] = True
        point = self.render()["point"]
        self.assertIn("ANCHOR", point, "a fallback anchor is not a box measurement, and says so")
        self.assertIn("CLIPPED", point)
        self.assertNotIn("BOX CLIPPED", point, "one word, and clipped is a caution not a name")
        clipped = point[point.index("CLIPPED") - 200:point.index("CLIPPED")]
        self.assertIn("#f2b329", clipped, "clipping is degraded-but-running: amber, §15")
        self.data["target_aim_valid"] = False
        self.assertEqual(self.render()["point"], "")
