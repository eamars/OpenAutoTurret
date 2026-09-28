"""The preview half of WP4's coordinate closure, checked with the real transform.

A preview that has been through the orientation transform and back must land exactly where it
started -- the residual is zero, not small, because both directions are the same permutation.
That is a weaker claim than full raw->model->preview closure (which needs the intrinsics and the
letterbox, and no single function composes them yet), and it is stated as such rather than
inflated. The second check exists because an unknown orientation string that is silently treated
as "no rotation" would make the first check pass for the wrong reason.
"""
import unittest

import numpy as np
import yaml

from common.image_corrections import (ORIENTATIONS, apply_orientation_bbox,
                                      apply_orientation_image, read_install_orientation,
                                      validate_orientation)


class TestPreviewClosure(unittest.TestCase):
    def setUp(self):
        # A 3x2 image with distinct values, so any permutation is visible.
        self.pixels = np.arange(6, dtype=np.uint8).reshape(3, 2)

    def test_the_preview_path_closes_for_every_orientation_it_implements(self):
        # Every orientation the image path accepts must be its own inverse, because the preview
        # path closes only by undoing itself: all four implemented transforms are reflections or a
        # half turn. The loop walks ORIENTATIONS itself instead of a copied tuple -- F-WP4-1 was
        # answered on 2026-09-29 by narrowing the vocabulary to what the code does, and a copied
        # list would let the next person add rotate_90 to the vocabulary while this test kept
        # certifying only the old four. If someone does add a quarter turn, this test fails, and
        # it should: a quarter turn is not its own inverse, so closure would need a real inverse,
        # not the same permutation applied twice.
        for orientation in ORIENTATIONS:
            once = apply_orientation_image(self.pixels[..., None], orientation)
            twice = apply_orientation_image(once, orientation)
            np.testing.assert_array_equal(
                np.asarray(twice), self.pixels[..., None],
                err_msg=f"{orientation} does not undo itself, so preview cannot close")

    def test_the_picture_and_the_geometry_close_the_same_way(self):
        """The picture and the boxes must agree, or a target is drawn where nothing is.

        Both transforms are applied twice; all four implemented orientations are their own
        inverse, so a mismatch here means the two paths disagree about what a direction means --
        the failure F-WP4-1 is about, caught at the point where it would be visible.
        """
        box = [10.0, 20.0, 30.0, 44.0]
        width, height = 100, 60
        for orientation in ORIENTATIONS:
            once = apply_orientation_bbox(box, orientation, width, height)
            twice = apply_orientation_bbox(once, orientation, width, height)
            self.assertEqual([round(v, 9) for v in twice], [round(v, 9) for v in box],
                             f"{orientation} does not undo itself on the geometry path")
            # The intermediate box must stay inside the frame: a half turn that maps a box outside
            # the image would round-trip by accident while being wrong on the way.
            self.assertTrue(all(0.0 <= v <= (width if i % 2 == 0 else height)
                                for i, v in enumerate(once)),
                            f"{orientation} pushed the box out of frame: {once}")

    def test_the_shipped_orientation_is_one_the_transform_knows(self):
        """The shipped orientation must be one the code can actually perform.

        This used to assert membership in a tuple copied out of the module: that copy listed
        rotate_90 and rotate_270, which the image path refuses, and omitted flip_horizontal and
        flip_vertical, which it performs. So a valid install choice raised a false alarm here,
        while a value that would crash the pipeline at the first frame would have passed. F-WP4-1
        was answered by narrowing the vocabulary; this is the second copy of that vocabulary going
        away. Now the module is asked, once, and any refusal it raises is the message.
        """
        orientation, source = read_install_orientation()
        self.assertIn(validate_orientation(orientation), ORIENTATIONS,
                      msg=f"{source}: shipped orientation {orientation!r} is not implementable")


if __name__ == "__main__":
    unittest.main()
