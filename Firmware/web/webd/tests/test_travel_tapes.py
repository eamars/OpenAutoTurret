"""§5 yaw tape and §6 pitch tape, executed under node from the page's own source.

The revision specifies these tapes numerically: "upper 10-15% of the viewport", "middle 55-60% of the
image width", "approximately middle 40-45% of viewport height", and endpoints that "always show the
software-safe travel limits". A specification that specific is an invitation to check it, and every
one of those claims is asserted below against the function the page actually calls.

What this does NOT establish is that the tapes look right. There is no browser here: geometry, colour
tokens and draw order are measurable, resemblance is §24 and belongs to a named person with the page
in front of them.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS

_EXPORTS = (
    "\nmodule.exports = { hudTravelTape, hudTravelTapeSvg, hudTickSteps, hudDegLabel,"
    " hudUnrangedNote };\n"
)

# The colour tokens the page passes in, mirrored so a change in the page's palette shows up here as
# a failing colour assertion rather than as a tape that quietly stopped matching §6.3.
C_TOKENS = {
    "green": "#95f58b",
    "dim": "rgba(149,245,139,.56)",
    "faint": "rgba(149,245,139,.22)",
    "amber": "#f2b329",
    "red": "#ff5d5d",
    "white": "#edf2eb",
    "black": "rgba(3,6,5,.80)",
    "line": "rgba(230,245,230,.24)",
}

YAW = dict(horizontal=True, x=408.0, y=135.0, length=1104.0,
           minDeg=-80.0, maxDeg=80.0, valueDeg=22.4, valid=True)
PITCH = dict(horizontal=False, x=1814.4, y=310.1, length=459.0,
             minDeg=-45.0, maxDeg=55.0, valueDeg=-6.8, valid=True)


@unittest.skipUnless(shutil.which("node"), "node not installed; the tapes cannot be executed")
class TravelTapesExecuted(unittest.TestCase):
    maxDiff = None

    def _node(self, script: str):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write(HUD_GEOMETRY_JS + _EXPORTS)
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

    # --- §5.1 / §6.1 placement ---------------------------------------------------------------

    def test_yaw_tape_sits_in_the_bands_the_revision_names(self) -> None:
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(YAW))
        vw, vh = 1920.0, 1080.0
        span = (got["x1"] - got["x"]) / vw
        self.assertGreaterEqual(span, 0.55, "§5.1: 'roughly the middle 55-60%% of the image width'")
        self.assertLessEqual(span, 0.60)
        self.assertAlmostEqual((got["x"] + got["x1"]) / 2.0, vw / 2.0, places=6,
                               msg="§5.1: horizontally centered")
        self.assertGreaterEqual(got["y"] / vh, 0.10, "§5.1: 'upper 10-15%% of the viewport'")
        self.assertLessEqual(got["y"] / vh, 0.15)

    def test_pitch_tape_occupies_the_middle_band_and_the_right_edge(self) -> None:
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(PITCH))
        vw, vh = 1920.0, 1080.0
        span = (got["y1"] - got["y"]) / vh
        self.assertGreaterEqual(span, 0.40, "§6.1: 'approximately middle 40-45%% of viewport height'")
        self.assertLessEqual(span, 0.45)
        self.assertGreater(got["x"], vw - 140.0, "§6.1: 'close to the right image edge'")
        self.assertGreater(got["y"], vh * 0.25, "the tape should be centred, not top-aligned")

    # --- §5.2 / §6.2 content ------------------------------------------------------------------

    def test_endpoints_always_show_the_soft_travel_limits(self) -> None:
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(YAW))
        ends = [t for t in got["ticks"] if t["endpoint"]]
        self.assertEqual(len(ends), 2, "§5.2: endpoints always shown")
        self.assertEqual({t["deg"] for t in ends}, {-80.0, 80.0})
        for t in ends:
            self.assertTrue(t["label"].endswith("\u00b0"),
                            "the reference draws the endpoints with a degree sign: %r" % t["label"])

    def test_limited_travel_is_not_invented(self) -> None:
        # Before homing, soft_limits_valid is false and the bounds are unset. Drawing a tape with
        # made-up endpoints would name a limit this machine was never homed to, and the operator
        # would see a travel range that does not exist.
        for bad in (dict(YAW, valid=False), dict(YAW, minDeg=0.0, maxDeg=0.0),
                    dict(YAW, minDeg=30.0, maxDeg=-30.0)):
            self.assertIsNone(self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));"
                                         % json.dumps(bad)),
                              "an unranged or impossible axis must produce no tape, not a confident one")
        self.assertIn("UNRANGED", self._node(
            "console.log(T.hudUnrangedNote(960, 135, 'YAW / PITCH'));"))

    def test_ticks_fine_and_coarse_and_monotonic(self) -> None:
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(YAW))
        positions = [t["pos"] for t in got["ticks"]]
        self.assertEqual(positions, sorted(positions), "ticks must be ordered along the tape")
        coarse = [t for t in got["ticks"] if t["coarse"]]
        self.assertGreaterEqual(len(coarse), 3, "a tape with two labels is a scale, not a readout")
        gaps = [coarse[i + 1]["pos"] - coarse[i]["pos"] for i in range(len(coarse) - 1)]
        self.assertGreater(min(gaps), 52.0,
                           "labels were allowed close enough to collide; §5.2 wants them readable")
        fine_only = [t for t in got["ticks"] if not t["coarse"]]
        self.assertTrue(all(t["label"] == "" for t in fine_only),
                        "§5.2: fine ticks carry no labels")

    def test_marker_maps_the_current_value_and_clamps_within_the_tape(self) -> None:
        # Hand arithmetic, not a re-run of the function: -80..80 across x=412..1516 puts +22.4 deg at
        # 62.75%% of the span.
        want = 408.0 + (22.4 + 80.0) / 160.0 * 1104.0
        got = self._node("console.log(T.hudTravelTape(%s).marker);" % json.dumps(YAW))
        self.assertAlmostEqual(got, want, places=6)
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);"
                       % json.dumps(dict(PITCH, valueDeg=-6.8))),
            310.1 + (55.0 + 6.8) / 100.0 * 459.0, places=6,
            msg="§6.2: pitch increases upward, so +55 is the top of the tape")
        # Out of range (a value beyond the soft limit, or an un-homed zero) must not point off the
        # tape into empty sky.
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);" % json.dumps(dict(YAW, valueDeg=-999.0))),
            408.0, places=9)
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);"
                       % json.dumps(dict(PITCH, valueDeg=120.0))),
            310.1, places=9)

    def test_no_cardinal_letters_appear_anywhere(self) -> None:
        # §5.3: logical joint travel, not compass heading; N/E/S/W forbidden without a validated
        # world-heading source. Checked on rendered output because that is where a stray label hides.
        svg = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'YAW', value:'X'}));"
            % (json.dumps(YAW), json.dumps(C_TOKENS)))
        for tok in (">N<", ">E<", ">S<", ">W<", ">N ", ">NE", "cardinal"):
            self.assertNotIn(tok, svg)

    # --- §6.3 hierarchy -----------------------------------------------------------------------

    def test_colour_hierarchy_follows_section_6_3(self) -> None:
        svg = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'PITCH', value:'-6.8'}));"
            % (json.dumps(PITCH), json.dumps(C_TOKENS)))
        fine = C_TOKENS["dim"]
        self.assertIn(fine, svg, "§6.3: fine ticks are dim green")
        self.assertIn('fill="%s"' % C_TOKENS["black"], svg,
                      "§6.3: value box has a dark translucent fill")
        self.assertIn('stroke="%s"' % C_TOKENS["green"], svg,
                      "§6.3: value box has a thin green outline")
        # The caret is filled bright, not dim: it is the thing the operator is reading.
        seg = svg[svg.index("<path"):]
        caret = seg[:seg.index("/>") + 2]
        self.assertIn('fill="%s"' % C_TOKENS["green"], caret)

    def test_each_tape_states_what_its_scale_is(self) -> None:
        # §5.3 says logical joint travel, not heading - and on this station the joint numbers are
        # surprising enough to be misread (yaw -22.6..+320.2 on a continuous axis; pitch entirely
        # negative, which is not elevation). The camera-to-axis boresight is not separable from the
        # principal point at the spans the theodolite probe has, so no world-elevation offset has
        # ever been measured. The tape has to say which scale it is on.
        yaw = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'YAW', value:'x',"
            " note:'JOINT TRAVEL, NOT HEADING'}));" % (json.dumps(YAW), json.dumps(C_TOKENS)))
        pitch = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'PITCH', value:'x',"
            " note:'JOINT, NOT ELEVATION'}));" % (json.dumps(PITCH), json.dumps(C_TOKENS)))
        self.assertIn("NOT HEADING", yaw)
        self.assertIn("NOT ELEVATION", pitch)

    def test_the_renderer_draws_caret_ticks_and_box(self) -> None:
        svg = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'YAW', value:'+22.4',"
            " vw:1920, vh:1080}));" % (json.dumps(YAW), json.dumps(C_TOKENS)))
        self.assertIn("<line", svg)
        self.assertIn("<path", svg, "§5.2: the current marker is a caret/triangle")
        self.assertIn("<rect", svg, "§5.2: the numeric value sits in an outlined box")
        self.assertIn("YAW", svg)
        self.assertIn("+22.4", svg)
        self.assertGreater(svg.count("<text"), 10, "endpoint and coarse labels should all be present")


class TapesWiredIntoThePage(unittest.TestCase):
    """Page-level facts that node cannot see: layer order, placement constants, typography."""

    def test_layer_order_matches_section_18(self) -> None:
        # §18 puts candidate (10), selected (11) and prediction (12) under the reticle and the tapes
        # (all 20). This page has no z-index on SVG groups - document order IS the order - so the
        # groups have to appear in that sequence in the markup.
        html = HUD_HTML
        order = [html.index(i) for i in
                 ('id="g-candidates"', 'id="g-selected"', 'id="g-reticle"', 'id="g-tapes"')]
        self.assertEqual(order, sorted(order),
                         "§18: tapes must not be drawn under the target overlays or the reticle")

    def test_placement_uses_the_revisions_own_bands(self) -> None:
        for token in ("0.575", "0.125", "0.425"):
            self.assertIn(token, HUD_JS,
                          "the tape placement fractions should be visible in the render path")

    def test_typography_is_one_monospace_stack(self) -> None:
        # §16: a narrow monospaced sensor-display appearance, not a proportional UI font.
        self.assertIn("IBM Plex Mono", HUD_CSS)
        self.assertIn("monospace", HUD_CSS)
        for cls in ("text.tlbl", "text.tval", "text.lbl"):
            self.assertIn(cls, HUD_CSS, "%s should share the global stack, not redeclare it" % cls)
        self.assertEqual(HUD_CSS.count("IBM Plex Mono"), 1,
                         "the font stack is set once; three copies is how they drift")


if __name__ == "__main__":
    unittest.main()
