"""Unit tests for the shared install-level camera corrections.

Pure-logic tests (no camera, no CAN, no motor): orientation of images and
detection boxes, gray-world white-balance gains, and the WB apply/clamp.
"""
from __future__ import annotations

import unittest

import numpy as np

from common import image_corrections as ic


class OrientationValidationTest(unittest.TestCase):
    def test_accepts_all_valid(self) -> None:
        for o in ic.ORIENTATIONS:
            self.assertEqual(ic.validate_orientation(o), o)

    def test_rejects_unknown_orientation(self) -> None:
        with self.assertRaises(ValueError):
            ic.validate_orientation("rotate_90")

    def test_accepts_all_wb_modes(self) -> None:
        for m in ic.WHITE_BALANCES:
            self.assertEqual(ic.validate_white_balance(m), m)

    def test_rejects_unknown_wb(self) -> None:
        with self.assertRaises(ValueError):
            ic.validate_white_balance("manual")


class ImageOrientationTest(unittest.TestCase):
    def setUp(self) -> None:
        # (H, W, C) = (2, 2, 1); flattened reading order is 1,2 / 3,4.
        self.arr = np.array([[[1], [2]], [[3], [4]]], dtype=np.uint8)

    def _flat(self, a) -> list:
        return a.reshape(-1).tolist()

    def test_none_is_identity(self) -> None:
        self.assertEqual(self._flat(ic.apply_orientation_image(self.arr, "none")), [1, 2, 3, 4])

    def test_rotate_180(self) -> None:
        self.assertEqual(self._flat(ic.apply_orientation_image(self.arr, "rotate_180")), [4, 3, 2, 1])

    def test_flip_horizontal(self) -> None:
        self.assertEqual(self._flat(ic.apply_orientation_image(self.arr, "flip_horizontal")), [2, 1, 4, 3])

    def test_flip_vertical(self) -> None:
        self.assertEqual(self._flat(ic.apply_orientation_image(self.arr, "flip_vertical")), [3, 4, 1, 2])

    def test_is_a_view_not_a_copy(self) -> None:
        # A rotate_180 of a small image is a strided view (no data copied).
        out = ic.apply_orientation_image(self.arr, "rotate_180")
        self.assertFalse(out.flags["C_CONTIGUOUS"])  # reversed strides

    def test_double_rotate_180_is_identity(self) -> None:
        once = ic.apply_orientation_image(self.arr, "rotate_180")
        twice = ic.apply_orientation_image(once, "rotate_180")
        self.assertEqual(self._flat(twice), [1, 2, 3, 4])


class BboxOrientationTest(unittest.TestCase):
    # Image is width=10, height=8; a box that is NOT symmetric so the three
    # flips are distinguishable.
    W, H = 10, 8
    BOX = (1.0, 1.0, 4.0, 5.0)

    def test_none(self) -> None:
        self.assertEqual(ic.apply_orientation_bbox(self.BOX, "none", self.W, self.H), (1.0, 1.0, 4.0, 5.0))

    def test_rotate_180(self) -> None:
        self.assertEqual(ic.apply_orientation_bbox(self.BOX, "rotate_180", self.W, self.H), (6.0, 3.0, 9.0, 7.0))

    def test_flip_horizontal(self) -> None:
        self.assertEqual(ic.apply_orientation_bbox(self.BOX, "flip_horizontal", self.W, self.H), (6.0, 1.0, 9.0, 5.0))

    def test_flip_vertical(self) -> None:
        self.assertEqual(ic.apply_orientation_bbox(self.BOX, "flip_vertical", self.W, self.H), (1.0, 3.0, 4.0, 7.0))

    def test_full_frame_maps_to_itself(self) -> None:
        full = (0.0, 0.0, float(self.W), float(self.H))
        for o in ("rotate_180", "flip_horizontal", "flip_vertical"):
            self.assertEqual(ic.apply_orientation_bbox(full, o, self.W, self.H), full)

    def test_double_rotate_180_is_identity(self) -> None:
        once = ic.apply_orientation_bbox(self.BOX, "rotate_180", self.W, self.H)
        twice = ic.apply_orientation_bbox(once, "rotate_180", self.W, self.H)
        self.assertEqual(twice, tuple(self.BOX))


class GrayWorldGainsTest(unittest.TestCase):
    def test_neutral_image_is_noop(self) -> None:
        rgb = np.full((4, 4, 3), 128, dtype=np.uint8)
        g = ic.gray_world_gains(rgb)
        for x in g:
            self.assertAlmostEqual(x, 1.0, places=5)

    def test_red_cast_pushes_red_down_and_green_blue_up(self) -> None:
        # The measured IMX500 cast: R~254, G~96, B~98.
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        rgb[..., 0] = 254
        rgb[..., 1] = 96
        rgb[..., 2] = 98
        r, g, b = ic.gray_world_gains(rgb)
        self.assertLess(r, 1.0)      # red pulled down
        self.assertGreater(g, 1.0)   # green boosted
        self.assertGreater(b, 1.0)   # blue boosted
        self.assertAlmostEqual(r, 149.333 / 254.0, places=3)

    def test_gains_are_clamped(self) -> None:
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        rgb[..., 0] = 255
        rgb[..., 1] = 1
        rgb[..., 2] = 1
        r, g, b = ic.gray_world_gains(rgb)
        self.assertEqual(g, 4.0)  # clamped at hi
        self.assertEqual(b, 4.0)
        self.assertGreaterEqual(r, 0.25)  # clamped at lo

    def test_black_image_is_safe_noop(self) -> None:
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        self.assertEqual(ic.gray_world_gains(rgb), (1.0, 1.0, 1.0))


class GrayWorldCorrectionTest(unittest.TestCase):
    def test_neutral_image_is_noop(self) -> None:
        rgb = np.full((4, 4, 3), 120, dtype=np.uint8)
        m = ic.gray_world_correction(rgb)
        for x in m:
            self.assertAlmostEqual(x, 1.0, places=5)

    def test_black_image_is_safe_noop(self) -> None:
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        self.assertEqual(ic.gray_world_correction(rgb), (1.0, 1.0, 1.0))

    def test_saturated_red_with_bright_other_channels_neutralizes(self) -> None:
        # The real IMX500 install: R is 100% saturated at 255 and G/B carry a
        # heavy bright tail. Plain gray-world gains clip G/B when applied, so the
        # means do not actually equalize. gray_world_correction adds a de-saturation
        # scale so the result IS neutral.
        rng = np.random.default_rng(7)
        H, W = 32, 48
        G = np.where(rng.random((H, W)) < 0.26, rng.uniform(200, 255, (H, W)), rng.uniform(0, 150, (H, W)))
        B = np.where(rng.random((H, W)) < 0.24, rng.uniform(200, 255, (H, W)), rng.uniform(0, 150, (H, W)))
        rgb = np.zeros((H, W, 3), dtype=np.uint8)
        rgb[..., 0] = 255                       # fully saturated red
        rgb[..., 1] = np.clip(G, 0, 255).astype(np.uint8)
        rgb[..., 2] = np.clip(B, 0, 255).astype(np.uint8)
        m = ic.gray_world_correction(rgb)
        out = ic.apply_white_balance(rgb, m)
        r, g, b = float(out[..., 0].mean()), float(out[..., 1].mean()), float(out[..., 2].mean())
        self.assertLess(abs(r - g), 2.0, "R and G means must equalize")
        self.assertLess(abs(g - b), 2.0, "G and B means must equalize")
        self.assertLessEqual(int(out.max()), 255)

    def test_no_clip_beats_plain_gray_world_on_saturated_red(self) -> None:
        # On the saturated-red frame, plain gray-world leaves a residual warm tint
        # (clipping drags G/B means down); the correction removes it.
        rng = np.random.default_rng(7)
        H, W = 32, 48
        G = np.where(rng.random((H, W)) < 0.26, rng.uniform(200, 255, (H, W)), rng.uniform(0, 150, (H, W)))
        B = np.where(rng.random((H, W)) < 0.24, rng.uniform(200, 255, (H, W)), rng.uniform(0, 150, (H, W)))
        rgb = np.zeros((H, W, 3), dtype=np.uint8)
        rgb[..., 0] = 255
        rgb[..., 1] = np.clip(G, 0, 255).astype(np.uint8)
        rgb[..., 2] = np.clip(B, 0, 255).astype(np.uint8)
        plain = ic.apply_white_balance(rgb, ic.gray_world_gains(rgb))
        fixed = ic.apply_white_balance(rgb, ic.gray_world_correction(rgb))
        plain_spread = abs(float(plain[..., 0].mean()) - float(plain[..., 1].mean()))
        fixed_spread = abs(float(fixed[..., 0].mean()) - float(fixed[..., 1].mean()))
        self.assertLess(fixed_spread, plain_spread)


class ApplyWhiteBalanceTest(unittest.TestCase):
    def test_identity_gains(self) -> None:
        rgb = np.full((3, 3, 3), 100, dtype=np.uint8)
        out = ic.apply_white_balance(rgb, (1.0, 1.0, 1.0))
        self.assertEqual(out.dtype, np.uint8)
        self.assertTrue(np.array_equal(out, rgb))

    def test_gain_applied(self) -> None:
        rgb = np.full((3, 3, 3), 100, dtype=np.uint8)
        out = ic.apply_white_balance(rgb, (2.0, 1.0, 1.0))
        self.assertEqual(int(out[0, 0, 0]), 200)   # red doubled
        self.assertEqual(int(out[0, 0, 1]), 100)   # green unchanged
        self.assertEqual(int(out[0, 0, 2]), 100)

    def test_clamps_to_255(self) -> None:
        rgb = np.full((2, 2, 3), 200, dtype=np.uint8)
        out = ic.apply_white_balance(rgb, (2.0, 2.0, 2.0))
        self.assertTrue((out <= 255).all())
        self.assertTrue((out == 255).all())

    def test_neutralizes_a_red_cast(self) -> None:
        # Applying the gray-world gains to a red-cast frame should bring R~G~B.
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        rgb[..., 0] = 254
        rgb[..., 1] = 96
        rgb[..., 2] = 98
        out = ic.apply_white_balance(rgb, ic.gray_world_gains(rgb))
        r, g, b = float(out[..., 0].mean()), float(out[..., 1].mean()), float(out[..., 2].mean())
        self.assertLess(abs(r - g), 3.0)
        self.assertLess(abs(g - b), 3.0)


if __name__ == "__main__":
    unittest.main()
