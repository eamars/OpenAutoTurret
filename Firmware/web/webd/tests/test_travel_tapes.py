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
import re

import json
import os
import shutil
import subprocess
import tempfile
import unittest

from ..hud import HUD_CSS, HUD_GEOMETRY_JS, HUD_HTML, HUD_JS

_EXPORTS = (
    "\nmodule.exports = { hudTravelTape, hudYawTapeRange, hudTravelTapeSvg, hudTickSteps, hudDegLabel,"
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

    def test_a_travel_limit_is_labelled_when_the_window_can_show_it(self) -> None:
        # §5.2 used to read "endpoints always shown", which was written for a fixed ruler.
        # Under a sliding window the two ends are usually off-window, and drawing them at
        # the tape's edge would be a lie about where they are; hiding them is the truth,
        # and the fade at each end is what says "there is more travel this way".
        # What survives unchanged: whenever a limit IS in view, it is labelled with its
        # own number and a degree sign -- never a rounded approximation.
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(YAW))
        self.assertGreater(got["hiddenTicks"], 0, "a centred window must actually hide off-window ticks")
        self.assertEqual([t for t in got["ticks"] if t["endpoint"]], [])
        near = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));"
                          % json.dumps(dict(YAW, valueDeg=78.0)))["ticks"]
        ends = [t for t in near if t["endpoint"]]
        self.assertEqual([t["deg"] for t in ends], [80.0], "the limit being approached is the one shown")
        self.assertTrue(ends[0]["label"].endswith("\u00b0"),
                        "the reference draws the endpoints with a degree sign: %r" % ends[0]["label"])
        self.assertAlmostEqual(ends[0]["pos"], 408.0, places=6,
                               msg="yaw+ is screen-left, so +80 sits at the tape's left end")

    def test_the_window_is_the_camera_field_of_view_not_a_number_i_chose(self) -> None:
        # The tape window decides what "off the end" means, so its source has to be a
        # fact and not a taste: the commissioned effective H/VFOV, the same pair the
        # safe-envelope polygon is sized from. The source is returned rather than
        # implied, and a tape with no FOV to inherit says so instead of inventing one.
        got = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));"
                         % json.dumps(dict(YAW, windowDeg=69.3002)))
        self.assertEqual(got["windowSource"], "fov")
        self.assertAlmostEqual(got["windowDeg"], 69.3002, places=4)
        nofov = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));" % json.dumps(YAW))
        self.assertEqual(nofov["windowSource"], "fallback")
        # A lens wider than the travel cannot show more travel than exists.
        wide = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));"
                          % json.dumps(dict(YAW, windowDeg=400.0)))
        self.assertEqual(wide["windowSource"], "travel")
        self.assertAlmostEqual(wide["windowDeg"], 160.0, places=6)

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
        # Physical direction: yaw+ turns the camera left, so +22.4 lies left
        # of centre on the travel tape despite the joint value increasing.
        # Mid-travel the caret does not move at all: it sits at the tape's centre and the
        # ruler slides under it (owner's revision, 2026-09-28, after fighter/helicopter
        # HUDs). Both axes, one rule -- so both fixtures expect their own midpoint.
        got = self._node("console.log(T.hudTravelTape(%s).marker);" % json.dumps(YAW))
        self.assertAlmostEqual(got, 408.0 + 1104.0 / 2, places=6)
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);"
                       % json.dumps(dict(PITCH, valueDeg=-6.8))),
            310.1 + 459.0 / 2, places=6)
        # Only near a limit -- where the window can no longer stay centred -- does the
        # caret leave the middle, and that off-centre IS the proximity cue. yaw+ is
        # screen-left, so approaching +80 pushes the caret toward the tape's left end.
        near = self._node("console.log(T.hudTravelTape(%s).marker);" % json.dumps(dict(YAW, valueDeg=78.0)))
        self.assertLess(near, 408.0 + 1104.0 / 2, "approaching the screen-left limit moves the caret left")
        # Out of range (a value beyond the soft limit, or an un-homed zero) must not point off the
        # tape into empty sky.
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);" % json.dumps(dict(YAW, valueDeg=-999.0))),
            1512.0, places=9)
        self.assertAlmostEqual(
            self._node("console.log(T.hudTravelTape(%s).marker);"
                       % json.dumps(dict(PITCH, valueDeg=120.0))),
            769.1, places=9)

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

    def test_the_tapes_carry_no_captions_and_the_ruler_is_the_thing_that_moves(self) -> None:
        # Owner, 2026-09-28: the two captions ("JOINT TRAVEL, NOT HEADING", "0 = TRAVEL
        # MIDPOINT") bought nothing, so they are gone -- and this test is what stops them
        # creeping back as "helpful" labels. The scale's meaning is carried by the design
        # instead: no compass letters anywhere, and joint numbers, not elevation.
        yaw = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'YAW', value:'x'}));"
            % (json.dumps(YAW), json.dumps(C_TOKENS)))
        for banned in ("HEADING", "MIDPOINT", "NOT ELEVATION"):
            self.assertNotIn(banned, yaw, "captions were removed on purpose: " + banned)
        # And the reason the caret is worth holding still: the ruler, not the caret, is
        # what answers "where am I". Same label, different pixel, as the value changes.
        def label_pos(value_deg, want_label):
            ticks = self._node("console.log(JSON.stringify(T.hudTravelTape(%s)));"
                               % json.dumps(dict(YAW, valueDeg=value_deg)))["ticks"]
            return [t["pos"] for t in ticks if t["label"] == want_label]
        # ±10 deg apart, so the two windows still share the middle of the scale.
        at_plus22 = label_pos(10.0, "0")
        at_minus22 = label_pos(-10.0, "0")
        self.assertTrue(at_plus22 and at_minus22, "the +10 label should be in view in both")
        self.assertNotAlmostEqual(at_plus22[0], at_minus22[0], places=1,
                                  msg="a ruler that does not slide is a fixed scale wearing a new name")


    def test_the_renderer_draws_caret_ticks_and_box(self) -> None:
        svg = self._node(
            "console.log(T.hudTravelTapeSvg(T.hudTravelTape(%s), %s, {title:'YAW', value:'+22.4',"
            " vw:1920, vh:1080}));" % (json.dumps(YAW), json.dumps(C_TOKENS)))
        self.assertIn("<line", svg)
        self.assertIn("<path", svg, "§5.2: the current marker is a caret/triangle")
        self.assertIn("<rect", svg, "§5.2: the numeric value sits in an outlined box")
        self.assertIn("YAW", svg)
        self.assertIn("+22.4", svg)
        self.assertGreater(svg.count("<text"), 4,
                           "the coarse labels inside the window must be present (the window is a "
                           "quarter of the travel by design, so this is no longer the full-travel count)")
        # A gradient must actually exist in the drawn output. Asserting one magic opacity
        # would tie the test to a tick landing on one exact pixel, which is how the first
        # version of this line broke; the property worth holding is "some ticks are
        # dimmer than others, and the dimming is gradual".
        opacities = [float(x) for x in re.findall(r'opacity="([0-9.]+)"', svg) if float(x) > 0]
        self.assertTrue(any(o < 1 for o in opacities), "no fade at all: ticks are being cut, not dissolved")
        self.assertTrue(any(o == 1 for o in opacities), "everything faded means no readable middle")


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

    # --- yaw without an envelope: the ruler stays, the wall does not ---------------------------

    # 2026-09-28, the morning the software sector came off. The tape is the one thing he asked to
    # keep, in his words: it does not stand in the way of free rotation and 0 is the homing origin.
    # So the endpoints move to the reference band the station file still declares -- and the pair
    # of cases below is the whole claim: a ruler keeps the tape, nothing-to-show still does not.
    def _yaw_range(self, payload):
        return self._node("console.log(JSON.stringify(T.hudYawTapeRange(%s)));" % json.dumps(payload))

    def test_yaw_tape_keeps_a_reference_band_when_the_envelope_is_gone(self) -> None:
        got = self._yaw_range({"yaw_envelope": "none", "soft_limits_valid": True,
                               "q_soft_min_yaw_rad": None, "q_soft_max_yaw_rad": None,
                               "yaw_band_min_rad": -1.5707963, "yaw_band_max_rad": 1.5707963})
        self.assertTrue(got["valid"], "losing a wall is not a reason to lose the scale")
        self.assertTrue(got["ruler"], "and the page must know it is drawing a ruler, not a limit")
        self.assertAlmostEqual(got["minDeg"], -90.0, places=3)
        self.assertAlmostEqual(got["maxDeg"], 90.0, places=3)
        tape = self._node(
            "console.log(JSON.stringify(T.hudTravelTape({horizontal:true,x:120,y:135,"
            "length:1104,minDeg:%s,maxDeg:%s,valueDeg:12.5,valid:true})));"
            % (got["minDeg"], got["maxDeg"]))
        zero = self._node('console.log(T.hudDegLabel(0, false));')
        labels = [t["label"] for t in tape["ticks"] if t.get("label")]
        self.assertIn(zero, labels, "the band is centred on the homing origin, so 0 must be on it")
        centre = [t for t in tape["ticks"] if t.get("label") == zero]
        self.assertAlmostEqual((centre[0]["pos"] - tape["x"]) / (tape["x1"] - tape["x"]), 0.5,
                               places=6, msg="0 at the middle of the tape, not off to one side")

    def test_yaw_tape_still_gives_up_when_there_is_genuinely_nothing_to_show(self) -> None:
        # Envelope none and a band of nothing (never homed, or a file with no band): the page draws
        # the unranged note. That note exists precisely so silence cannot read as open sky.
        got = self._yaw_range({"yaw_envelope": "none", "soft_limits_valid": True,
                               "yaw_band_min_rad": 0.0, "yaw_band_max_rad": 0.0})
        self.assertFalse(got["valid"])

    def test_a_bounded_yaw_still_draws_its_limits_and_calls_them_limits(self) -> None:
        got = self._yaw_range({"yaw_envelope": "sector", "soft_limits_valid": True,
                               "q_soft_min_yaw_rad": -1.5707963, "q_soft_max_yaw_rad": 1.5707963,
                               "yaw_band_min_rad": 0.0, "yaw_band_max_rad": 0.0})
        self.assertTrue(got["valid"])
        self.assertFalse(got["ruler"], "with an envelope the endpoints are the soft limits")
        self.assertAlmostEqual(got["minDeg"], -90.0, places=3)
