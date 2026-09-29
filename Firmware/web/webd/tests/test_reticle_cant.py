"""The reticle's cant line: one line, one angle, and a box that cannot be moved by either.

The whole point of building the line as `centre + k*(ux, uy)` sampled at two intervals is that the two
visible halves cannot drift apart -- so this test does not look at a screenshot, it asks the geometry:

* every endpoint lies on the same line through the centre (collinearity, at several angles);
* the distance from the centre is unchanged by the angle, so the gap around the box and the reach of
  the arms are the same at 5 deg as at 0 -- a rotation cannot uncover or cover the aiming box;
* +d and -d are mirror images across the horizontal through the centre;
* the function returns the two layers in paint order (dark under, green over) and no box of its own,
  because the box is the aiming reference, drawn elsewhere, unrotated.

Run under node from the page's own geometry module, so what is asserted is what the browser executes.
"""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from ..hud import HUD_GEOMETRY_JS

CX, CY, REACH, INNER = 480.0, 270.0, 38.0, 16.0
COLORS = {"stroke": "#05070a", "line": "#95f58b"}

PROGRAM = HUD_GEOMETRY_JS + r"""
const input = JSON.parse(require("fs").readFileSync(0, "utf8"));
const layers = hudReticleCantSvg(input.cx, input.cy, input.reach, input.inner, input.deg, input.colors);
// Pull the endpoints back out of the markup: the assertion is about the numbers the page emits.
// Attribute names carry digits (x1, y2), so each value is read by its own attribute; the exponent
// branch is there because a 90-degree rotation emits cos(90deg) as 6.1e-17 and JS prints it that way.
const at = (tag, name) => Number(tag.match(new RegExp(name + '="([-\\d.eE+]+)"'))[1]);
const lines = (markup) => (markup.match(/<line[^>]*>/g) || []).map(
  (tag) => [[at(tag, "x1"), at(tag, "y1")], [at(tag, "x2"), at(tag, "y2")]]);
console.log(JSON.stringify({layers, dark: lines(layers[0]), green: lines(layers[1])}));
"""


def render(deg):
    with tempfile.TemporaryDirectory() as box:
        path = Path(box) / "cant.js"
        path.write_text(PROGRAM, encoding="utf-8")
        result = subprocess.run(["node", str(path)], timeout=20, text=True, capture_output=True,
                                input=json.dumps({"cx": CX, "cy": CY, "reach": REACH,
                                                  "inner": INNER, "deg": deg, "colors": COLORS}))
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


ANGLES = (0, 2, -2, 5, -5, 37.5, -90)


class ReticleCantGeometry(unittest.TestCase):
    def test_default_zero_is_a_flat_pair_of_arms(self):
        out = render(0)
        self.assertEqual(len(out["green"]), 2, "one line, two visible halves")
        for (x1, y1), (x2, y2) in out["green"]:
            self.assertAlmostEqual(y1, CY, 6)
            self.assertAlmostEqual(y2, CY, 6)
        self.assertIn(COLORS["stroke"], out["layers"][0], "dark under-stroke goes down first")
        self.assertIn(COLORS["line"], out["layers"][1])

    def test_every_endpoint_stays_on_the_line_through_the_centre(self):
        for deg in ANGLES:
            with self.subTest(deg=deg):
                out = render(deg)
                (x1, y1), (x2, y2) = out["green"][0]
                dx, dy = (x2 - x1), (y2 - y1)
                self.assertTrue(dx or dy, "a segment of zero length proves nothing")
                for half in out["green"] + out["dark"]:
                    for x, y in half:
                        self.assertAlmostEqual((x - CX) * dy - (y - CY) * dx, 0.0, places=4,
                                               msg="%r is off the line" % ((x, y),))

    def test_rotation_preserves_the_gap_and_the_reach(self):
        def radii(deg):
            return sorted(((x - CX) ** 2 + (y - CY) ** 2) ** .5
                          for half in render(deg)["green"] for x, y in half)
        zero = radii(0)
        # Sorted, so this reads as "two ends on the box gap, two on the arm reach" -- the shape the
        # drawing is supposed to have, asserted before any rotation is compared against it.
        self.assertEqual([round(r, 6) for r in zero], sorted([INNER, INNER, REACH, REACH]),
                         "near ends sit on the box gap, far ends on the arm reach")
        for deg in ANGLES:
            with self.subTest(deg=deg):
                for expected, actual in zip(zero, radii(deg)):
                    self.assertAlmostEqual(actual, expected, places=6,
                                           msg="a rotated line must not uncover or cover the box")

    def test_positive_and_negative_are_mirror_images(self):
        for pair in ((2, -2), (5, -5)):
            with self.subTest(pair=pair):
                up, down = render(pair[0])["green"], render(pair[1])["green"]
                self.assertEqual(len(up), len(down))
                for a_half, b_half in zip(up, down):
                    for (ax, ay), (bx, by) in zip(a_half, b_half):
                        self.assertAlmostEqual(ax, bx, 6)
                        self.assertAlmostEqual(ay + by, 2 * CY, 6)

    def test_the_line_carries_no_box_of_its_own(self):
        # The aiming reference is drawn by the caller and stays axis-aligned; if the cant geometry
        # grew a rect or a path, a rotation would start moving the thing the operator aims with.
        joined = "".join(render(3)["layers"])
        for fragment in ("<rect", "<path", "transform"):
            self.assertNotIn(fragment, joined,
                             "no group rotation and no box: the angle lives in the coordinates")

    def test_a_missing_or_broken_angle_falls_back_to_horizontal(self):
        # JSON carries neither NaN nor a unit-suffixed string as a number, so these are the shapes a
        # caller can actually hand over by mistake -- and none of them may tilt the line.
        for bad in (None, "0", "5deg", True):
            with self.subTest(bad=str(bad)):
                out = render(bad)
                self.assertTrue(all(abs(y - CY) < 1e-6
                                    for half in out["green"] for _, y in half))


class ReticleCantIsParameterised(unittest.TestCase):
    def test_the_page_has_one_cant_value_and_a_dev_handle(self):
        from ..hud import HUD_HTML
        self.assertIn("let reticleCantDeg = 0;", HUD_HTML,
                      "the default must be 0, and it must be the one value the renderer reads")
        self.assertIn("window.otaSetReticleCant", HUD_HTML,
                      "verifiable by hand at 0/+2/-2/+5/-5 without inventing an operator control")
        self.assertNotIn("artificial-horizon", HUD_HTML, "§16.8: no attitude UI came along with this")


if __name__ == "__main__":
    unittest.main()
