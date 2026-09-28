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

from common.image_corrections import apply_orientation_image, read_install_orientation


class TestPreviewClosure(unittest.TestCase):
    def setUp(self):
        # A 3x2 image with distinct values, so any permutation is visible.
        self.pixels = np.arange(6, dtype=np.uint8).reshape(3, 2)

    def test_the_preview_path_closes_for_every_orientation_it_implements(self):
        # Every orientation the image path accepts must be its own inverse, because the preview
        # path closes only by undoing itself: all four implemented transforms are reflections or a
        # half turn. rotate_90/rotate_270 are deliberately not asserted here -- the image path does
        # not implement them at all (see F-WP4-1 in the WP4 report), and claiming closure for a
        # rotation that raises would be the test lying about the code.
        for orientation in ("none", "rotate_180", "flip_horizontal", "flip_vertical"):
            once = apply_orientation_image(self.pixels[..., None], orientation)
            twice = apply_orientation_image(once, orientation)
            np.testing.assert_array_equal(
                np.asarray(twice), self.pixels[..., None],
                err_msg=f"{orientation} does not undo itself, so preview cannot close")

    def test_the_shipped_orientation_is_one_the_transform_knows(self):
        orientation, source = read_install_orientation()
        self.assertIn(orientation, ("none", "rotate_90", "rotate_180", "rotate_270"),
                      msg=f"{source}: shipped orientation {orientation!r} would be applied as a no-op")


if __name__ == "__main__":
    unittest.main()
