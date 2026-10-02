"""The owner's four UI points of 2026-10-02, executed under node against the page's own functions.

1. The pitch value box sits where the yaw one does: against the caret, on the picture side.
2. Displayed pitch is up-positive: negative means the camera points down.
3. A yaw axis with no envelope gets a full-circle ruler. The ruler used to cycle on the +/-90 deg
   reference band, so at +104 deg it had run out of scale.
4. The field of regard shows that circle, -180..180, instead of the +/-90 band. With the turret
   at +104 deg, the line of sight used to be pinned off the map.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from ..hud import HUD_GEOMETRY_JS

C = {"green": "#0f0", "dim": "#080", "white": "#fff", "amber": "#fa0", "black": "#000",
     "line": "#444", "stroke": "#111"}


@unittest.skipUnless(shutil.which("node"), "node not installed; the page's rules cannot be executed")
class FullCircleAndPitchUp(unittest.TestCase):
    def _node(self, expr: str):
        with tempfile.TemporaryDirectory() as box:
            geo = os.path.join(box, "geo.js")
            with open(geo, "w", encoding="utf-8") as fh:
                fh.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudTravelTape, hudTravelTapeSvg, "
                         "hudYawTapeRange, hudForInset, hudPitch, hudPitchRange, hudWrapDeg };\n")
            main = os.path.join(box, "main.js")
            with open(main, "w", encoding="utf-8") as fh:
                fh.write("const T = require(%s);\nconsole.log(JSON.stringify(%s));\n"
                         % (json.dumps(geo), expr))
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(r.stdout)

    # -- 2: pitch is up-positive ------------------------------------------------------------
    def test_pointing_the_camera_down_reads_negative(self):
        # Positive joint pitch points this camera down (direction_contract), so a joint value above
        # the travel's centre is below the horizon of the display.
        t = "{soft_limits_valid: true, q_soft_min_pitch_rad: -1.4, q_soft_max_pitch_rad: -0.2}"
        self.assertLess(self._node("T.hudPitch(%s, -0.5)" % t), 0)
        self.assertGreater(self._node("T.hudPitch(%s, -1.0)" % t), 0)
        span = self._node("T.hudPitchRange(%s)" % t)
        self.assertAlmostEqual(span["lo"], -0.6)
        self.assertAlmostEqual(span["hi"], 0.6)

    def test_up_the_pitch_tape_is_up_the_scale(self):
        tape = self._node("T.hudTravelTape({horizontal: false, x: 900, y: 100, length: 400, "
                          "minDeg: -35, maxDeg: 35, valueDeg: 0, windowDeg: 40, valid: true})")
        labelled = [(tk["pos"], tk["deg"]) for tk in tape["ticks"] if tk["label"]]
        top, bottom = min(labelled)[1], max(labelled)[1]
        self.assertGreater(top, bottom, "the higher label on the screen is the higher pitch")

    # -- 1: the pitch value box is against its caret ----------------------------------------
    def test_the_pitch_box_sits_beside_the_caret_like_the_yaw_box_sits_under_it(self):
        svg = self._node("T.hudTravelTapeSvg(T.hudTravelTape({horizontal: false, x: 900, y: 100, "
                         "length: 400, minDeg: -35, maxDeg: 35, valueDeg: 3, windowDeg: 40, "
                         "valid: true}), %s, {title: 'PITCH', value: '+3', vw: 1000, vh: 800})"
                         % json.dumps(C))
        rect = svg[svg.index('<rect x="'):]
        x = float(rect.split('x="')[1].split('"')[0])
        y = float(rect.split('y="')[1].split('"')[0])
        self.assertAlmostEqual(y + 34 / 2, 300.0, msg="vertically centred on the caret (tape middle)")
        self.assertLess(x + 96, 900 - 12, "on the picture side of the caret, not over the tape")

    # -- 3: a continuous yaw ruler is the whole circle --------------------------------------
    def test_unbounded_yaw_is_ranged_as_a_circle_not_as_the_reference_band(self):
        r = self._node("T.hudYawTapeRange({soft_limits_valid: true, yaw_envelope: 'none', "
                       "yaw_band_min_rad: -1.5708, yaw_band_max_rad: 1.5708})")
        self.assertEqual((r["minDeg"], r["maxDeg"], r["continuous"]), (-180, 180, True))

    def test_at_104_degrees_the_ruler_still_has_scale_on_both_sides(self):
        tape = self._node("T.hudTravelTape({horizontal: true, x: 0, y: 50, length: 1000, "
                          "minDeg: -180, maxDeg: 180, valueDeg: 104.2, windowDeg: 69.3, "
                          "valid: true, continuous: true})")
        labels = {tk["label"] for tk in tape["ticks"] if tk["label"]}
        self.assertIn("+90", labels)
        self.assertIn("+120", labels, "the scale goes on past the old +90 band")
        self.assertFalse(any(tk["endpoint"] for tk in tape["ticks"]),
                         "a continuous axis has no ends to draw")

    def test_across_180_the_ruler_reads_on_round_the_circle(self):
        tape = self._node("T.hudTravelTape({horizontal: true, x: 0, y: 50, length: 1000, "
                          "minDeg: -180, maxDeg: 180, valueDeg: 175, windowDeg: 69.3, "
                          "valid: true, continuous: true})")
        labels = [tk["label"] for tk in sorted(tape["ticks"], key=lambda tk: tk["pos"]) if tk["label"]]
        self.assertIn("180", labels)
        self.assertIn("-160", labels, "past 180 the scale continues at -175, -170, ...")
        self.assertEqual(self._node("T.hudWrapDeg(361)"), 1)
        self.assertEqual(self._node("T.hudWrapDeg(-180)"), 180)

    # -- 4: the field of regard is the circle -----------------------------------------------
    BAND = [[-90, -35], [90, -35], [90, 35], [-90, 35]]

    def _inset(self, los, **extra):
        o = {"pts": self.BAND, "hfovDeg": 69.3, "vfovDeg": 40.4, "los": los, "vw": 1600,
             "vh": 900, "continuousYaw": True}
        o.update(extra)
        return self._node("T.hudForInset(%s)" % json.dumps(o))

    def test_the_map_is_the_whole_circle_and_104_degrees_is_on_it(self):
        g = self._inset([104.2, 0])
        xs = [p["x"] for p in g["envPx"]]
        self.assertAlmostEqual((max(xs) - min(xs)) / g["scale"], 360.0, places=6)
        self.assertFalse(g["los"]["off"], "the line of sight is on the map, not pinned to its edge")

    def test_a_line_of_sight_several_turns_from_home_is_drawn_where_it_points(self):
        a, b = self._inset([30.0, 0]), self._inset([30.0 + 720.0, 0])
        self.assertAlmostEqual(a["los"]["x"], b["los"]["x"], places=6)

    def test_the_view_across_180_is_drawn_at_both_edges(self):
        g = self._inset([175.0, 0])
        self.assertIsNotNone(g["fovWrap"], "the overhang past 180 reappears at -180")
        self.assertIsNone(self._inset([0.0, 0])["fovWrap"])


if __name__ == "__main__":
    unittest.main()
