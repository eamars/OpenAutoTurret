"""§15 palette, §16 typography, §18 layering - conformance for the things a browser would show and a
review would not.

Round 15 found by reading the stylesheet that three rules read `var(--hud-mono)` inside a `font:`
shorthand while nothing declared the token. An undefined custom property inside a shorthand makes the
*whole declaration* invalid at computed-value time, so the dock's buttons would have rendered in the
user-agent's serif button font at the user-agent's size - not "slightly wrong font", but "the §16
typography is absent". Nothing in a static read of the file shows that; only the token-usage check below
would have, which is why it exists and why it is tested against a stylesheet that is deliberately broken.

None of this asserts appearance. There is still no real-browser paint in the evidence for this HUD: these
are checks on the stylesheet's own consistency and on §18's numbers. Visual fidelity is §24 and stays
operator-signed.
"""

from __future__ import annotations

import re
import unittest

from ..hud import HUD_CSS, HUD_HTML, HUD_JS


def tokens_used(css: str) -> set:
    return set(re.findall(r"var\(\s*(--[\w-]+)", css))


def tokens_declared(css: str) -> set:
    return set(re.findall(r"(--[\w-]+)\s*:", css))


class EveryTokenReadIsDeclared(unittest.TestCase):
    maxDiff = None

    def test_the_stylesheet_declares_every_token_it_reads(self) -> None:
        missing = tokens_used(HUD_CSS) - tokens_declared(HUD_CSS)
        self.assertEqual(missing, set(),
                         "undefined var() inside a font:/border: shorthand drops the ENTIRE declaration, "
                         "so the rule does not degrade, it disappears: %s" % sorted(missing))

    def test_the_check_fails_on_the_mistake_it_exists_for(self) -> None:
        # A check that has never failed is a decoration. This is exactly the stylesheet that shipped for
        # three rounds: mono read, mono never declared.
        broken = ".a { font:500 8px/1 var(--hud-mono); } :root { --hud-green: #95f58b; }"
        self.assertEqual(tokens_used(broken) - tokens_declared(broken), {"--hud-mono"})

    def test_the_dock_drawer_and_safety_rules_use_the_declared_token(self) -> None:
        for sel in (".dockbtn", "#drawer {", "#safety {"):
            at = HUD_CSS.index(sel)
            self.assertIn("var(--hud-mono)", HUD_CSS[at:at + 400],
                          "%s without the mono token falls back to the browser's UI font" % sel)


class LayeringMatchesSection18(unittest.TestCase):
    maxDiff = None

    def test_no_layer_invents_a_z_index_outside_the_revision(self) -> None:
        declared = {int(z) for z in re.findall(r"z-index:\s*(\d+)", HUD_CSS)}
        # 60 is critical dialog: reserved by §18, not implemented, so absent here rather than faked.
        self.assertTrue(declared <= {0, 10, 20, 30, 40, 50},
                        "unexpected layer values %s; §18 lists 0,1,10,11,12,20,30,40,60" % sorted(declared))

    def test_the_named_layers_sit_where_the_table_puts_them(self) -> None:
        self.assertIn("z-index: 0", HUD_CSS)      # video
        self.assertIn("z-index: 20", HUD_CSS)     # symbology and chrome
        self.assertLess(HUD_CSS.index("#dock"), HUD_CSS.index("#drawer"),
                        "document order and numbers must tell the same story")
        self.assertIn("z-index:30", HUD_CSS)      # dock
        self.assertIn("z-index:40", HUD_CSS)      # drawer

    def test_the_safety_layer_is_documented_rather_than_squeezed_in(self) -> None:
        # §18 has no row for the safety indication. At 20 a drawer would cover a FAULT banner, which
        # makes §22's "interrupt normal operation" false the moment the operator opens DIAG; at 60 it
        # would sit on the layer §18 reserves for critical dialogs. 50 is the gap between them, and the
        # reason lives in the stylesheet.
        at = HUD_CSS.index("#safety {")
        self.assertIn("z-index:50", HUD_CSS[at:at + 400])
        self.assertNotIn("z-index:45", HUD_CSS, "the undocumented intermediate value must not survive")
        self.assertLess(HUD_CSS.index("z-index:40"), HUD_CSS.index("z-index:50"))

    def test_svg_layers_are_ordered_because_svg_has_no_z_index(self) -> None:
        # §18's 10/11/12/20 for candidate, selected, prediction, reticle. Inside one <svg>, child
        # elements have no z-index; document order is the layering. Putting z-index on a <g> would look
        # compliant in a diff and be ignored by the renderer.
        order = ["g-candidates", "g-selected", "g-prediction", "g-reticle", "g-for", "g-tapes"]
        places = [HUD_HTML.index('id="%s"' % g) for g in order]
        self.assertEqual(places, sorted(places),
                         "§18 order is candidates, selected, prediction, reticle, then FOR and tapes")
        self.assertNotIn("g#candidates", HUD_CSS)
        for g in order:
            self.assertNotIn("#%s { z-index" % g, HUD_CSS,
                             "z-index on an SVG group is decoration the renderer ignores")

    def test_the_sensor_filter_layer_is_absent_and_that_is_a_choice(self) -> None:
        # §17 makes presentation filters optional; §18 gives them z=1. Not implementing them should be a
        # stated absence, not an unnoticed gap in the layer table.
        self.assertNotIn("z-index: 1\n", HUD_CSS)
        self.assertNotIn("backdrop-filter", HUD_CSS, "heavy post-processing is what §17 warns reduces "
                                                    "target readability")


class TypographyMatchesSection16(unittest.TestCase):
    maxDiff = None

    STACK = ["IBM Plex Mono", "Roboto Mono", "SFMono-Regular", "Consolas", "monospace"]

    def _size(self, selector: str) -> float:
        at = HUD_CSS.index(selector)
        m = re.search(r"font(?:-size)?:\s*(?:\S+\s+)?([0-9.]+)px", HUD_CSS[at:at + 320])
        self.assertIsNotNone(m, "no font size found for %s" % selector)
        return float(m.group(1))

    def test_the_recommended_stack_is_the_one_declared(self) -> None:
        line = HUD_CSS[HUD_CSS.index("--hud-mono:"):]
        line = line[:line.index(";")]
        for i, fam in enumerate(self.STACK):
            self.assertIn(fam, line)
            if i:
                self.assertLess(line.index(self.STACK[i - 1]), line.index(fam),
                                "§16's order is part of the recommendation")

    def test_the_current_axis_value_is_the_strongest_numeric_text(self) -> None:
        # §16's ladder: yaw/pitch current values strongest numeric; strip small; FOR legend smallest.
        tval = self._size("text.tval")
        self.assertGreater(tval, self._size("#strip"))
        self.assertGreater(tval, self._size("text.tlbl"))
        self.assertGreater(tval, self._size("#drawer .dtitle"), "the FOR legend is the smallest normal "
                                                               "readable size, so the value out-sizes it")
        self.assertLessEqual(self._size("#mode-block .m2"), tval,
                             "mode phase is 'medium'; it must not out-size the strongest numeric")
        self.assertLessEqual(self._size("#mode-block .m3"), tval)

    def test_the_mode_name_stays_the_strongest_text_overall(self) -> None:
        self.assertGreater(self._size("#mode-block .m1"), self._size("#mode-block .m2"))
        self.assertGreater(self._size("#mode-block .m1"), self._size("text.tval"))


class PaletteDiscipline(unittest.TestCase):
    maxDiff = None

    def test_the_reference_values_are_the_ones_declared(self) -> None:
        # §15 says the values are references, not a hard requirement; the relationship is the requirement.
        # The relationship is only testable if the tokens mean what the document says they mean.
        want = {"--hud-green": "#95f58b", "--hud-amber": "#f2b329", "--hud-red": "#ff5d5d",
                "--hud-white": "#edf2eb"}
        for tok, val in want.items():
            self.assertIn("%s: %s" % (tok, val), HUD_CSS, "%s drifted from §15" % tok)

    def test_red_is_restricted_to_fault_and_stop(self) -> None:
        offenders = []
        for m in re.finditer(r"([^{}]+)\{([^}]*)\}", HUD_CSS):
            sel, body = m.group(1).strip(), m.group(2)
            if sel.startswith(":root") or sel in ("html, body", "html", "body"):
                continue   # declaring the token is not spending it; §15 test above pins the value
            if "#ff5d5d" in body or "var(--hud-red)" in body:
                if not any(w in sel for w in ("red", "fault", "stop", ".s1")):
                    offenders.append(sel)
        self.assertEqual(offenders, [],
                         "§15: red only for fault/stop. A panel that spends red on anything inconvenient "
                         "trains the operator to ignore it when FAULT appears: %s" % offenders)

    def test_a_refused_command_is_a_caution_not_a_fault(self) -> None:
        at = HUD_CSS.index("#drawer .dack.bad")
        self.assertIn("var(--hud-amber)", HUD_CSS[at:at + 60],
                      "a refused command is inconvenient, not a fault")

    def test_the_prediction_cue_is_never_green(self) -> None:
        # Restated here because §15 and §10 agree on it and it is the easiest colour to get wrong: green
        # means "healthy and here", the cue means "where it is going, if we are guessing".
        at = HUD_JS.index("hudPredictionSvg")
        self.assertIn("C.amber", HUD_JS[at:at + 700])


if __name__ == "__main__":
    unittest.main()
