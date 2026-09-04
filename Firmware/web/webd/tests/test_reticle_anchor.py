"""§7: the centre reticle marks the OPTICAL AXIS, which is the principal point, not the middle of the image.

These two coincide on a well-centred lens and on this station's own calibration (cx 960, cy 540 of a
1920x1080 frame), which is exactly the trap: a build that simply drew the geometric centre would be
indistinguishable from a correct one on this hardware. So the mapping is tested with a deliberately
off-centre principal point, where "follows cx/cy" and "hard-coded 0.5" give different answers, and the
station's own calibration is asserted separately so the coincidence is on the record rather than
silently relied upon.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import unittest

from ..hud import HUD_GEOMETRY_JS, HUD_JS


class ReticleSitsOnTheOpticalAxis(unittest.TestCase):

    def setUp(self) -> None:
        self._mod = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False)
        self._mod.write(HUD_GEOMETRY_JS + "\nmodule.exports = { hudAxisNorm };\n")
        self._mod.close()

    def tearDown(self) -> None:
        os.unlink(self._mod.name)

    def _axis(self, intr):
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
            fh.write("const T = require(%r);\nconsole.log(JSON.stringify(T.hudAxisNorm(%s)));\n"
                     % (self._mod.name, json.dumps(intr)))
            main = fh.name
        try:
            r = subprocess.run(["node", main], capture_output=True, text=True, timeout=20)
        finally:
            os.unlink(main)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout.strip()
        return json.loads(out) if out and out != "null" else None

    def test_an_off_centre_principal_point_moves_the_anchor(self) -> None:
        # The only case that distinguishes following the calibration from hard-coding the centre.
        g = self._axis({"cx": 1010.0, "cy": 505.0, "width": 1920, "height": 1080})
        self.assertIsNotNone(g, "valid intrinsics must produce an anchor")
        self.assertAlmostEqual(g["u"], 1010.0 / 1920.0, places=9)
        self.assertAlmostEqual(g["v"], 505.0 / 1080.0, places=9)
        self.assertGreater(abs(g["u"] - 0.5), 0.01, "an off-centre lens must not be drawn centred")
        self.assertGreater(abs(g["v"] - 0.5), 0.01)

    def test_this_station_is_centred_by_luck_not_by_proof(self) -> None:
        # Recorded so nobody reads the coincidence as evidence: with the loaded calibration the optical
        # axis IS the frame centre, so a wrong build would pass on this hardware.
        g = self._axis({"cx": 960.0, "cy": 540.0, "width": 1920, "height": 1080})
        self.assertIsNotNone(g)
        self.assertAlmostEqual(g["u"], 0.5, places=9)
        self.assertAlmostEqual(g["v"], 0.5, places=9)

    def test_missing_or_rubbish_intrinsics_yield_no_anchor_rather_than_a_guessed_one(self) -> None:
        for bad in (None, {}, {"cx": 960, "cy": 540}, {"cx": -1, "cy": 540, "width": 1920, "height": 1080},
                    {"cx": 960, "cy": 540, "width": 0, "height": 1080}):
            self.assertIsNone(self._axis(bad), "no anchor may be invented from %r" % (bad,))

    def test_the_fallback_says_so_on_screen(self) -> None:
        # Without intrinsics the render path centres the reticle, which is a guess; §7's honesty clause
        # is that a guess must be labelled. This is a code-shape assertion, not a painted one - it can
        # show the note is emitted when intrinsics are absent, not that it renders legibly.
        self.assertIn("hudAxisNorm", HUD_JS, "the render path must consult the calibration")
        self.assertIn("{ u: 0.5, v: 0.5 }", HUD_JS, "the fallback must be an explicit centred default")
        self.assertRegex(HUD_JS, r"intr\s*\?\s*\"\"\s*:",
                         "the uncalibrated case must emit its own on-screen note, not draw silently")


if __name__ == "__main__":
    unittest.main()
