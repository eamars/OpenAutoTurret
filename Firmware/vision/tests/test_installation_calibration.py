"""Tests for the installation-orientation calibration (architecture §29).

The core math (R_W_B derivation, multi-frame outlier rejection, atomic commit)
is tested on synthetic data; the cv2 detection path is smoke-tested against a
rendered ChArUco board. No camera, no CAN, no motor.
"""
import math
import os
import tempfile
import unittest

import numpy as np

from vision import installation_calibration as ic


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def _ang_deg(R_a, R_b):
    c = float(np.clip((np.trace(R_a.T @ R_b) - 1.0) / 2.0, -1, 1))
    return math.degrees(math.acos(c))


def _pose(R_C_D, t=(0.0, 0.0, 0.40), err=0.4, ts=0):
    return ic.FramePose(
        np.asarray(R_C_D, float), np.asarray(t, float), err, ts, 16)


class TestDeriveRWB(unittest.TestCase):
    def test_identity(self):
        R = ic.derive_R_W_B(np.identity(3), np.identity(3))
        self.assertLess(_ang_deg(R, np.identity(3)), 1e-9)

    def test_tilted_base_with_camera_extrinsic(self):
        # Pick a true base tilt and a non-trivial camera extrinsic; derive the
        # board pose that would produce it and check the formula inverts it.
        R_W_B_true = _rot_z(0.10) @ _rot_y(0.15)
        R_P_C = _rot_x(0.12) @ _rot_z(0.05)
        # R_W_B = (R_P_C @ R_C_D)^T  =>  R_C_D = (R_W_B @ R_P_C)^T
        R_C_D = (R_W_B_true @ R_P_C).T
        R_est = ic.derive_R_W_B(R_C_D, R_P_C)
        self.assertLess(_ang_deg(R_est, R_W_B_true), 1e-6)

    def test_nonidentity_R_W_D(self):
        # A board plumb/rotated 90 deg about Z relative to the world.
        R_W_D = _rot_z(math.pi / 2)
        R_W_B_true = _rot_y(0.2)
        R_P_C = np.identity(3)
        # R_W_B = R_W_D (R_P_C R_C_D)^T  =>  R_C_D = (R_W_D^T R_W_B R_P_C)^T
        R_C_D = (R_W_D.T @ R_W_B_true @ R_P_C).T
        R_est = ic.derive_R_W_B(R_C_D, R_P_C, R_W_D)
        self.assertLess(_ang_deg(R_est, R_W_B_true), 1e-6)


class TestSolveBoardPose(unittest.TestCase):
    def _board(self):
        spec = ic.BoardSpec(marker_cols=3, marker_rows=3)
        board, obj = spec.make_cv_board()
        self.assertIsNotNone(board)
        return obj

    def test_roundtrip(self):
        obj = self._board()
        K = np.array([[500, 0, 300], [0, 500, 220], [0, 0, 1]], float)
        intr = ic.CameraIntrinsics(500.0, 500.0, 300.0, 220.0,
                                   (0.0, 0.0, 0.0, 0.0, 0.0), 600, 440)
        R_C_D = _rot_y(0.26) @ _rot_z(0.14)
        t_C_D = np.array([0.03, -0.02, 0.40])
        import cv2
        rvec, _ = cv2.Rodrigues(R_C_D)
        proj, _ = cv2.projectPoints(obj, rvec, t_C_D, intr.K, intr.dist)
        img_pts = proj.reshape(-1, 2)
        pose = ic.solve_board_pose(obj, img_pts, intr, timestamp_ns=42)
        self.assertLess(_ang_deg(pose.R_C_D, R_C_D), 1e-3)
        self.assertLess(float(np.linalg.norm(pose.t_C_D - t_C_D)), 1e-5)
        self.assertLess(pose.reprojection_error_px, 1e-3)
        self.assertEqual(pose.timestamp_ns, 42)


class TestMultiFrame(unittest.TestCase):
    def _poses(self, n, outlier_idx=(), noise_deg=0.4, out_deg=25.0, seed=0):
        rng = np.random.default_rng(seed)
        R_W_B_true = _rot_z(0.08) @ _rot_y(0.12)
        R_P_C = _rot_x(0.1) @ _rot_z(0.04)
        R_C_D_true = (R_W_B_true @ R_P_C).T
        poses = []
        for i in range(n):
            if i in outlier_idx:
                noise = _rot_y(out_deg * math.pi / 180)
            else:
                ang = noise_deg * math.pi / 180
                noise = _rot_z(rng.uniform(-ang, ang)) @ _rot_x(
                    rng.uniform(-ang, ang))
            R_C_D = R_C_D_true @ noise  # perturb the board pose
            poses.append(_pose(R_C_D, ts=1000 + i))
        return poses, R_W_B_true, R_P_C

    def test_clean_frames(self):
        poses, R_W_B_true, R_P_C = self._poses(10)
        res = ic.calibrate_from_poses(poses, R_P_C, max_outlier_deg=3.0)
        self.assertEqual(res.n_frames, 10)
        self.assertEqual(res.n_rejected, 0)
        self.assertTrue(res.valid)
        self.assertLess(_ang_deg(res.R_W_B, R_W_B_true), 1.0)

    def test_outliers_rejected(self):
        poses, R_W_B_true, R_P_C = self._poses(10, outlier_idx=(2, 7))
        res = ic.calibrate_from_poses(poses, R_P_C, max_outlier_deg=3.0)
        self.assertEqual(res.n_rejected, 2)
        self.assertEqual(res.n_frames, 8)
        self.assertLess(_ang_deg(res.R_W_B, R_W_B_true), 1.0)

    def test_too_few_frames_invalid(self):
        poses, _, R_P_C = self._poses(2)
        res = ic.calibrate_from_poses(poses, R_P_C, min_frames=3)
        self.assertFalse(res.valid)


class TestPersistence(unittest.TestCase):
    def _result(self):
        res = ic.CalibrationResult(
            R_W_B=_rot_y(0.12) @ _rot_z(0.05),
            covariance=1e-4,
            n_frames=15,
            n_rejected=1,
            reprojection_error_px=0.42,
            timestamp_ns=987654321,
        )
        return res

    def test_commit_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "installation_pose.yaml")
            res = self._result()
            ic.commit_R_W_B(res, path)
            self.assertTrue(os.path.exists(path))
            self.assertFalse(os.path.exists(path + ".tmp"))
            R = ic.load_R_W_B(path)
            self.assertIsNotNone(R)
            self.assertLess(_ang_deg(R, res.R_W_B), 1e-9)

    def test_commit_writes_cpp_compatible_format(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "installation_pose.yaml")
            ic.commit_R_W_B(self._result(), path)
            with open(path) as f:
                text = f.read()
            lines = [l for l in text.splitlines() if l and not l.startswith("#")]
            # metadata key=value lines, then exactly 3 matrix rows of 3 values.
            meta = [l for l in lines if "=" in l]
            matrix = [l for l in lines if "=" not in l]
            self.assertIn("source=visual_calibration", meta)
            self.assertIn("valid=1", meta)
            self.assertEqual(len(matrix), 3)
            for row in matrix:
                self.assertEqual(len(row.split()), 3)

    def test_load_missing_is_none(self):
        self.assertIsNone(ic.load_R_W_B("/nonexistent/path/pose.yaml"))


class TestDetection(unittest.TestCase):
    def test_marker_detection_and_pose(self):
        """Render ArUco markers on a white background at known 3D positions,
        detect them with cv2, and solve a pose from the detected centers."""
        import cv2
        dict_ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dict_, cv2.aruco.DetectorParameters())
        marker_px = 80
        img = np.full((480, 480), 255, np.uint8)
        centers_3d = []
        centers_px = []
        for i, id_ in enumerate([0, 1, 4, 5, 8, 9, 2, 3]):
            r, c = divmod(i, 3)
            cx = 90 + c * 130
            cy = 90 + r * 130
            m = cv2.aruco.generateImageMarker(dict_, id_, marker_px)
            img[cy:cy + marker_px, cx:cx + marker_px] = m
            centers_px.append((cx + marker_px / 2, cy + marker_px / 2))
            centers_3d.append((c * 0.04, r * 0.04, 0.0))
        res = detector.detectMarkers(img)
        ids = res[1]
        self.assertIsNotNone(ids)
        self.assertEqual(len(ids), 8)
        # Solve a pose from the detected marker centers (a valid PnP set).
        K = np.array([[400, 0, 240], [0, 400, 240], [0, 0, 1]], float)
        intr = ic.CameraIntrinsics(400.0, 400.0, 240.0, 240.0, (0.0,) * 5, 480, 480)
        img_pts = np.array(centers_px, float)
        obj_pts = np.array(centers_3d, float)
        pose = ic.solve_board_pose(obj_pts, img_pts, intr, timestamp_ns=1)
        self.assertLess(pose.reprojection_error_px, 1e-3)
        self.assertEqual(pose.n_corners, 8)

    def test_detect_charuco_rendered_board(self):
        """Exercise the production charuco path on a rendered board. Detection
        quality on the synthetic image is OpenCV-build dependent, so this is
        lenient: when the detector finds enough corners it must yield a solvable,
        low-reprojection pose."""
        spec = ic.BoardSpec(marker_cols=3, marker_rows=3)
        board, obj, detector = ic.make_detector(spec)
        import cv2
        img = np.asarray(board.generateImage((480, 480)))
        obj_pts, img_pts, n = ic.detect_charuco(img, board, detector)
        if img_pts is None:
            self.skipTest("charuco detection returned no corners on this build")
        self.assertEqual(obj_pts.shape[0], img_pts.shape[0])
        intr = ic.CameraIntrinsics(400.0, 400.0, 240.0, 240.0,
                                   (0.0,) * 5, 480, 480)
        pose = ic.solve_board_pose(obj_pts, img_pts, intr, timestamp_ns=1)
        self.assertLess(pose.reprojection_error_px, 5.0)


class TestEndToEnd(unittest.TestCase):
    def test_run_calibration_frontal(self):
        # Render the board frontally; the pipeline collects valid frames and
        # produces a stable, valid R_W_B. Detection on the synthetic image is
        # OpenCV-build dependent, so if the detector finds no corners we skip
        # (the per-stage math is covered by TestMultiFrame / TestDetection).
        spec = ic.BoardSpec(marker_cols=3, marker_rows=3)
        board, obj, detector = ic.make_detector(spec)
        import cv2
        img = np.asarray(board.generateImage((480, 480)))
        R_P_C = _rot_x(0.06) @ _rot_z(-0.03)
        cfg = ic.CalibrationConfig(
            board=spec,
            intrinsics=ic.CameraIntrinsics(400.0, 400.0, 240.0, 240.0,
                                           (0.0,) * 5, 480, 480),
            extrinsics=ic.CameraExtrinsics(R_P_C=R_P_C, t_P_C=np.zeros(3)),
            n_frames=6,
        )

        def capture():
            return img

        ts = [0]
        res = ic.run_calibration(capture, lambda: (ts.__setitem__(0, ts[0] + 1) or ts[0]), cfg)
        if res.n_frames < 3:
            self.skipTest("charuco detection returned no frames on this build")
        self.assertTrue(res.valid)
        # Frontal render: board aligned with camera => R_W_B == R_P_C^T.
        self.assertLess(_ang_deg(res.R_W_B, R_P_C.T), 2.0)


if __name__ == "__main__":
    unittest.main()
