"""The two commissioning CLIs, exercised end to end (no camera, no station).

P9 was blocked twice by the same class of gap: the maths existed, the writer
existed, the docs told the operator to run a command — and there was no command.
These tests run the tools themselves over synthetic board views and check the
artifacts they claim to produce, so "run the tool" can never again be a
sentence with nothing behind it.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import unittest

import numpy as np

FIRMWARE = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))


def _load_tool(name: str):
    path = os.path.join(FIRMWARE, "tools", name + ".py")
    spec = importlib.util.spec_from_file_location("tool_" + name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except Exception:  # noqa: BLE001
    HAVE_CV2 = False


@unittest.skipUnless(HAVE_CV2, "needs OpenCV")
class CalibrationCliTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import cv2
        from vision.intrinsics_calibration import (Fit, render_synthetic_views)
        from vision.installation_calibration import BoardSpec
        cls.spec = BoardSpec(marker_cols=6, marker_rows=6,
                             square_length_m=0.024,
                             marker_length_m=0.016)
        cls.images = render_synthetic_views(cls.spec,
                                            Fit(610.0, 612.0, 322.0, 241.0),
                                            n_views=14, size=(640, 480))
        cls.view_dir = cls._write_views(cv2)

    @staticmethod
    def _write_views(cv2) -> str:
        import tempfile
        d = tempfile.mkdtemp(prefix="ota_cal_views_")
        for i, im in enumerate(CalibrationCliTest.images):
            cv2.imwrite(os.path.join(d, f"view_{i:03d}.png"), im)
        return d

    def _tmp(self) -> str:
        import tempfile
        return tempfile.mkdtemp(prefix="ota_cal_out_")

    # -- tools/calibrate_camera_intrinsics.py -------------------------------
    def test_intrinsics_cli_self_test_passes(self):
        tool = _load_tool("calibrate_camera_intrinsics")
        self.assertEqual(tool.main(["--self-test"]), 0)

    def test_intrinsics_cli_writes_a_file_controld_accepts(self):
        tool = _load_tool("calibrate_camera_intrinsics")
        out = os.path.join(self._tmp(), "camera_intrinsics.yaml")
        rc = tool.main(["--images", self.view_dir, "--out", out,
                        "--min-views", "6", "--width", "640",
                        "--height", "480"])
        self.assertEqual(rc, 0)
        with open(out) as f:
            text = f.read()
        self.assertIn("# ota-camera-intrinsics v1", text)
        for key in ("fx=", "fy=", "cx=", "cy=", "width=640", "height=480"):
            self.assertIn(key, text)
        # And the numbers are the ones we rendered with (0.6 % on this data).
        vals = {ln.split("=")[0]: ln.split("=")[1]
                for ln in text.splitlines() if "=" in ln and not
                ln.startswith("#")}
        self.assertAlmostEqual(float(vals["fx"]), 610.0, delta=610.0 * 0.02)

    def test_intrinsics_cli_refuses_a_different_stream_size(self):
        tool = _load_tool("calibrate_camera_intrinsics")
        out = os.path.join(self._tmp(), "camera_intrinsics.yaml")
        rc = tool.main(["--images", self.view_dir, "--out", out,
                        "--min-views", "6", "--expect-size", "1920x1080"])
        self.assertEqual(rc, 1, "a 640x480 fit must not be labelled 1920x1080")

    def test_intrinsics_cli_fails_loudly_without_views(self):
        import tempfile
        empty = tempfile.mkdtemp(prefix="ota_cal_empty_")
        tool = _load_tool("calibrate_camera_intrinsics")
        with self.assertRaises(SystemExit):
            tool.main(["--images", empty, "--out", "/tmp/whatever.yaml"])

    # -- tools/calibrate_installation_pose.py -------------------------------
    def test_pose_cli_refuses_without_intrinsics(self):
        tool = _load_tool("calibrate_installation_pose")
        rc = tool.main(["--images", self.view_dir,
                        "--intrinsics", "/nonexistent/camera_intrinsics.yaml",
                        "--extrinsics", "/nonexistent/extrinsics.yaml",
                        "--out", os.path.join(self._tmp(), "pose.yaml")])
        self.assertEqual(rc, 1)

    def test_pose_cli_commits_a_pose_file_in_the_cpp_format(self):
        intr_tool = _load_tool("calibrate_camera_intrinsics")
        out_dir = self._tmp()
        intr = os.path.join(out_dir, "camera_intrinsics.yaml")
        self.assertEqual(intr_tool.main(
            ["--images", self.view_dir, "--out", intr, "--min-views", "6",
             "--width", "640", "--height", "480"]), 0)

        tool = _load_tool("calibrate_installation_pose")
        pose = os.path.join(out_dir, "installation_pose.yaml")
        rc = tool.main(["--images", self.view_dir, "--intrinsics", intr,
                        "--extrinsics", os.path.join(out_dir, "none.yaml"),
                        "--out", pose])
        self.assertEqual(rc, 0)
        lines = open(pose).read().splitlines()
        self.assertEqual(lines[0], "# ota-installation-pose v1")
        self.assertEqual(lines[1], "source=visual_calibration")
        self.assertEqual(lines[2], "valid=1")
        # NB: the ANGLE is meaningless for random synthetic poses (the real
        # run needs a levelled board); what this pins is the plumbing.
        matrix = [ln for ln in lines if ln and not ln.startswith("#")
                  and "=" not in ln]
        self.assertEqual(len(matrix), 3)
        for row in matrix:
            self.assertEqual(len(row.split()), 3)
        R = np.array([[float(x) for x in r.split()] for r in matrix])
        np.testing.assert_allclose(R @ R.T, np.identity(3), atol=1e-9)

    def test_pose_cli_reports_instead_of_writing_with_no_commit(self):
        intr_tool = _load_tool("calibrate_camera_intrinsics")
        out_dir = self._tmp()
        intr = os.path.join(out_dir, "camera_intrinsics.yaml")
        intr_tool.main(["--images", self.view_dir, "--out", intr,
                        "--min-views", "6", "--width", "640",
                        "--height", "480"])
        tool = _load_tool("calibrate_installation_pose")
        pose = os.path.join(out_dir, "installation_pose.yaml")
        rc = tool.main(["--images", self.view_dir, "--intrinsics", intr,
                        "--extrinsics", "/nope.yaml", "--no-commit"])
        self.assertEqual(rc, 0)
        self.assertFalse(os.path.exists(pose))


if __name__ == "__main__":
    unittest.main()
