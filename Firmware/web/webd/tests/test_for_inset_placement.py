"""§13 / §24: "FOR inset is compact and located at lower left".

That sentence is two claims, and both are numbers, so both are testable without a browser:

* LOWER LEFT - the card's left edge must sit against the left margin and its body must fall in the lower
  part of the viewport, above the bottom status strip that §12 owns.
* COMPACT - the card must stay within the fraction of the viewport the revision names, so it can never
  grow into the picture it is annotating.

The point of asserting these as RATIOS of the viewport rather than as pixel constants is that the HUD is
laid out against window size: an inset that is lower-left at 1920x1080 and drifts to mid-screen at 1280x800
would pass a pixel test and fail the sentence. hudForInset is pure, so it is driven through node over the
real JavaScript rather than reimplemented here - a reimplementation would test this file, not the HUD.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import unittest

from ..hud import HUD_GEOMETRY_JS


class ForInsetPlacement(unittest.TestCase):

    def setUp(self) -> None:
        self._mod = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._mod.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudForInset };\n")
        self._mod.close()

    def tearDown(self) -> None:
        os.unlink(self._mod.name)

    def _call(self, opts) -> object:
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\nconsole.log(JSON.stringify(T.hudForInset(%s)));\n"
                     % (self._mod.name, json.dumps(opts)))
            main = fh.name
        try:
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=20)
        finally:
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        return json.loads(out) if out and out != "null" else None

    @staticmethod
    def _opts(vw: int, vh: int) -> dict:
        return {"pts": [[-20.0, -10.0], [20.0, -10.0], [20.0, 10.0], [-20.0, 10.0]],
                "hfovDeg": 69.3, "vfovDeg": 40.4, "los": [0.0, 0.0], "vw": vw, "vh": vh}

    def test_it_is_drawn_against_the_left_margin_and_the_lower_half(self) -> None:
        g = self._call(self._opts(1920, 1080))
        self.assertIsNotNone(g, "a valid FOR polygon must produce an inset, not silence")
        self.assertLess(g["x"] / 1920.0, 0.05, "the card must sit at the left margin, not float inward")
        bottom = g["y"] + g["h"]
        self.assertGreater(g["y"] / 1080.0, 0.55, "the card belongs in the lower part of the viewport")
        self.assertLessEqual(bottom, 1080.0, "the card may not run past the bottom edge")
        self.assertGreater((1080.0 - bottom) / 1080.0, 0.01,
                           "and it must clear the strip that §12 puts along the bottom")

    def test_it_stays_compact_at_any_window_size(self) -> None:
        for vw, vh in ((1920, 1080), (1280, 800), (2560, 1440), (1024, 768)):
            g = self._call(self._opts(vw, vh))
            self.assertIsNotNone(g, "no inset at %dx%d" % (vw, vh))
            wf, hf = g["w"] / float(vw), g["h"] / float(vh)
            self.assertGreaterEqual(wf, 0.20, "below this the inset is unreadable, not compact")
            self.assertLessEqual(wf, 0.28, "the inset may not grow into the picture it annotates")
            self.assertGreaterEqual(hf, 0.16)
            self.assertLessEqual(hf, 0.25)
            self.assertLess(wf * hf, 0.07, "combined area is what 'compact' really means")

    def test_the_plot_lies_inside_the_card_it_belongs_to(self) -> None:
        g = self._call(self._opts(1920, 1080))
        p = g["plot"]
        self.assertGreaterEqual(p["x"], g["x"])
        self.assertGreaterEqual(p["y"], g["y"])
        self.assertLessEqual(p["x"] + p["w"], g["x"] + g["w"] + 1e-6)
        self.assertLessEqual(p["y"] + p["h"], g["y"] + g["h"] + 1e-6)

    def test_a_window_with_no_size_produces_no_inset(self) -> None:
        # Before the first resize event, or with the video element not yet measured, the geometry is 0.
        # Returning null is the honest answer; drawing a card at a guessed size would put furniture on
        # the screen that claims the picture exists.
        self.assertIsNone(self._call(self._opts(0, 0)))
        self.assertIsNone(self._call({"pts": [], "hfovDeg": 69.3, "vfovDeg": 40.4,
                                      "los": [0.0, 0.0], "vw": 1920, "vh": 1080}))
        self.assertIsNone(self._call({"pts": [[0.0, 0.0]], "hfovDeg": 0, "vfovDeg": 0,
                                      "los": [0.0, 0.0], "vw": 1920, "vh": 1080}))


if __name__ == "__main__":
    unittest.main()
