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
            "alignment": {"mode": "manual_depth", "valid": True, "range_source": "manual",
                          "range_measured": False, "assumed_depth_m": 10,
                          "x_norm": 949.5825/1920, "y_norm": 551.0025/1080,
                          "camera_from_laser_mm": {"right": 75, "up": 75, "forward": 0}},
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
const mark = hudLaserMark(input.t, !!input.stale);
console.log(JSON.stringify({mark, pixel: mark ? hudProject(mark.u, mark.v, lay) : null,
  point: hudMeasurementPointSvg(input.t, lay, !!input.stale)}));
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
        self.assertEqual(self.render(out)["mark"]["label"], "ASSUMED 10.0 m")

    def test_crosshair_uses_controller_projection_and_contain_scaling(self):
        rendered = self.render(width=960, height=800)
        self.assertAlmostEqual(rendered["pixel"]["x"], 949.5825/2)
        self.assertAlmostEqual(rendered["pixel"]["y"], 130+551.0025/2)
        self.assertIn("MEASURE", rendered["point"])

    def test_no_virtual_laser_claim_when_disabled_invalid_or_stale(self):
        for key, value in (("mode", "off"), ("valid", False), ("x_norm", 1.1),
                           ("y_norm", None), ("assumed_depth_m", 0),
                           ("range_measured", True), ("range_source", "sensor")):
            data = copy.deepcopy(self.data)
            data["alignment"][key] = value
            with self.subTest(key=key):
                self.assertIsNone(self.render(data)["mark"])
        rendered = self.render(stale=True)
        self.assertIsNone(rendered["mark"])
        self.assertEqual(rendered["point"], "")

    def test_fallback_and_clipping_are_visible(self):
        self.data["target_aim_source"] = "invalid_box_anchor_fallback"
        self.data["target_aim_box_clipped"] = True
        self.assertIn("ANCHOR / BOX CLIPPED", self.render()["point"])
        self.data["target_aim_valid"] = False
        self.assertEqual(self.render()["point"], "")
