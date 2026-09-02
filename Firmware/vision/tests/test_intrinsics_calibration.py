"""§28.2 intrinsics fitting, checked against ground truth.

The point of these tests is not coverage, it is that a calibration tool nobody
validated against a known answer is a rumour mill: it will happily write a
confident-looking `fx` that is 8 % off, and every bearing downstream inherits the
error with no symptom. So the tests hand the pipeline images rendered through a
KNOWN pinhole and require it to recover that pinhole on this machine, with this
OpenCV build (measured: warp-quantised renders give ~0.5 % on fx, 0.4 px on the
principal point, at 0.4 px corner rms).
"""
from __future__ import annotations

import os
import tempfile
import unittest

import numpy as np

from vision.installation_calibration import (BoardSpec, CameraIntrinsics,
                                             parse_key_value_file)
from vision.intrinsics_calibration import (Fit, collect_views, intrinsics_text,
                                           render_synthetic_views, solve,
                                           write_intrinsics)

try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except Exception:  # noqa: BLE001
    HAVE_CV2 = False

TRUTH = Fit(fx=610.0, fy=612.0, cx=322.0, cy=241.0)


def _spec() -> BoardSpec:
    return BoardSpec(marker_cols=6, marker_rows=6,
                     square_length_m=0.024, marker_length_m=0.016)


@unittest.skipUnless(HAVE_CV2, "needs OpenCV")
class IntrinsicsFitTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.images = render_synthetic_views(_spec(), TRUTH, n_views=16,
                                            size=(640, 480))

    def test_recovers_a_known_pinhole(self):
        views, skipped, per_view, size = collect_views(self.images, _spec())
        self.assertEqual(size, (640, 480))
        self.assertEqual(len(views), 16, "the synthetic set must be usable")
        report = solve(views, size, per_view, skipped)
        self.assertLess(abs(report.pinhole.fx - TRUTH.fx) / TRUTH.fx, 0.015)
        self.assertLess(abs(report.pinhole.fy - TRUTH.fy) / TRUTH.fy, 0.015)
        self.assertLess(abs(report.pinhole.cx - TRUTH.cx), 2.0)
        self.assertLess(abs(report.pinhole.cy - TRUTH.cy), 2.0)
        # Sub-pixel fit quality on rendered data: if this regresses, the
        # corner/geometry plumbing is broken, not the lens.
        self.assertLess(report.pinhole.rms_px, 1.5, report.summary())
        self.assertEqual(report.pinhole.n_views, 16)

    def test_views_with_too_few_corners_are_skipped_not_fitted(self):
        blank = [np.full((480, 640, 3), 255, dtype=np.uint8) for _ in range(5)]
        views, skipped, per_view, size = collect_views(
            blank + self.images[:4], _spec())
        self.assertEqual(skipped, 5)
        self.assertEqual(len(views), 4)
        self.assertEqual(per_view[:5], [0, 0, 0, 0, 0])

    def test_mixed_resolutions_are_refused(self):
        other = [cv2.resize(im, (800, 600)) for im in self.images[:3]]
        with self.assertRaises(ValueError) as cm:
            collect_views(self.images[:3] + other, _spec())
        self.assertIn("per stream geometry", str(cm.exception))

    def test_too_few_views_raises_instead_of_confident_garbage(self):
        views, skipped, per_view, size = collect_views(self.images[:2], _spec())
        with self.assertRaises(ValueError) as cm:
            solve(views, size, per_view, skipped)
        self.assertIn("at least", str(cm.exception))

    def test_phantom_distortion_is_not_reported_as_edge_error(self):
        """Rendered data has NO distortion. An unconstrained k1 will still find
        ~0.4 px of warp noise to chase, and then predict degrees of phantom
        bearing error at the frame edge. The report must not launder that."""
        views, skipped, per_view, size = collect_views(self.images, _spec())
        report = solve(views, size, per_view, skipped)
        self.assertEqual(report.edge_error_deg, 0.0, report.summary())
        self.assertIn("NOT resolvable", " ".join(report.notes))


@unittest.skipUnless(HAVE_CV2, "needs OpenCV")
class IntrinsicsFileTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        views, skipped, per_view, size = collect_views(
            render_synthetic_views(_spec(), TRUTH, n_views=12, size=(640, 480)),
            _spec())
        cls.report = solve(views, size, per_view, skipped)

    def test_written_file_is_the_cxx_contract(self):
        text = intrinsics_text(self.report, meta=["test run"])
        self.assertIn("# ota-camera-intrinsics v1", text)
        kv = parse_key_value_file_from_text(text)
        for key in ("fx", "fy", "cx", "cy", "width", "height"):
            self.assertIn(key, kv, "controld applies NOTHING if one is missing")
        self.assertEqual(kv["width"], str(self.report.width))
        self.assertNotIn("\nfx:", text)          # the legacy YAML shape
        self.assertIn("# test run", text)

    def test_write_is_atomic_and_readable(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "camera_intrinsics.yaml")
            text = write_intrinsics(path, self.report, meta=["x"])
            with open(path) as f:
                self.assertEqual(f.read(), text)
            # No temp litter left behind (the .intr_*.tmp must be renamed away).
            self.assertEqual(sorted(os.listdir(d)), ["camera_intrinsics.yaml"])
            back = CameraIntrinsics.load(path)
            self.assertAlmostEqual(back.fx, self.report.pinhole.fx, places=6)
            self.assertEqual(back.width, self.report.width)

    def test_summary_says_what_the_numbers_come_from(self):
        s = self.report.summary()
        for token in ("pinhole fx=", "rms", "views"):
            self.assertIn(token, s)


def parse_key_value_file_from_text(text: str) -> dict:
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


if __name__ == "__main__":
    unittest.main()
