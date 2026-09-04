"""Round 47/48: containment has to be judged on the declared box, not on the anchor point.

C1 read "the target never leaves the frame" and for two rounds of dart runs reported PASS on the strength of a
predicate over a single point. The fixture declares a box height of BOX_H_NORM (about 378 px of 1080), so roughly
189 px of target may hang outside a frame edge that the anchor has not crossed. These tests pin the corrected
metric, including the case the old one passed.
"""
import unittest

from tools.probe_track_loop import BOX_H_NORM, box_extends_past_frame

FRAME_H = 1080
BOX_PX = BOX_H_NORM * FRAME_H          # ~378 px declared height: ~189 px above and below the anchor


class BoxContainment(unittest.TestCase):
    def test_centred_target_is_inside(self) -> None:
        self.assertFalse(box_extends_past_frame(540.0, BOX_PX, FRAME_H))

    def test_anchor_just_inside_with_the_box_hanging_out_is_out(self) -> None:
        # The exact case the old point predicate passed.
        self.assertTrue(box_extends_past_frame(1079.0, BOX_PX, FRAME_H))
        self.assertTrue(box_extends_past_frame(1.0, BOX_PX, FRAME_H))

    def test_threshold_is_the_declared_edge_not_an_invented_margin(self) -> None:
        # Sitting exactly on the edge is not out; a tenth of a pixel FURTHER OUT is. The first draft of this
        # test moved the anchor inward and expected a failure, which would have "fixed" the metric by breaking it.
        self.assertFalse(box_extends_past_frame(1080.0 - BOX_PX / 2.0, BOX_PX, FRAME_H))
        self.assertTrue(box_extends_past_frame(1080.0 - BOX_PX / 2.0 + 0.1, BOX_PX, FRAME_H))
        self.assertFalse(box_extends_past_frame(BOX_PX / 2.0, BOX_PX, FRAME_H))
        self.assertTrue(box_extends_past_frame(BOX_PX / 2.0 - 0.1, BOX_PX, FRAME_H))

    def test_zero_height_box_degrades_to_the_point_test(self) -> None:
        # Guards against the helper silently becoming an always-fail, which would "fix" C1 by breaking it.
        self.assertFalse(box_extends_past_frame(1080.0, 0.0, FRAME_H))
        self.assertTrue(box_extends_past_frame(1080.1, 0.0, FRAME_H))

    def test_declared_box_is_the_half_height_the_fixture_publishes(self) -> None:
        # Line 258 of the probe builds the fixture box from BOX_H_NORM * 0.5; the metric must agree with it,
        # or the check measures a target that was never sent.
        self.assertAlmostEqual(BOX_PX, 0.35 * FRAME_H)


if __name__ == "__main__":
    unittest.main()
