#!/usr/bin/env python3
"""P9 step 1 — derive the installation orientation R_W_B from the board (§29).

This is the entry point the queue's P9 row was missing: `vision/
installation_calibration.py` had the maths and the atomic writer, but no way to
run it at a station.

    cd Firmware
    # 1. intrinsics first (they bias every bearing the PnP computes):
    python3 -m tools.calibrate_camera_intrinsics --live --frames 40 \
        --orientation rotate_180
    # 2. then this, with the printed board lying LEVEL on the ground in view:
    python3 -m tools.calibrate_installation_pose --live --frames 25 \
        --orientation rotate_180 --commit calibration/installation_pose.yaml

The board is the gravity reference: it is levelled on the ground, so its frame
IS the world frame (§29.1). The result is committed atomically and controld
loads it at boot; the log line to expect afterwards is

    installation pose: source=stored calibrated=true

Why the intrinsics file is required: R_W_B comes from a PnP pose of the board,
and PnP turns a pixel into a ray using fx/cx. A wrong fx does not fail — it
tilts the answer smoothly, and the station aims consistently wrong. Refusing is
cheaper than discovering that at P8 with motors armed. Pass --allow-uncalibrated
only to see the shape of the output, never to commission.
"""
from __future__ import annotations

import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.installation_calibration import (  # noqa: E402
    BoardSpec, CameraExtrinsics, CameraIntrinsics, CalibrationConfig,
    commit_R_W_B, run_calibration)


def _load_images(directory: str):
    exts = ("png", "jpg", "jpeg", "bmp", "pgm")
    paths = sorted(p for e in exts for p in glob.glob(
        os.path.join(directory, f"*.{e}")))
    if not paths:
        raise SystemExit(f"error: no images in {directory}")
    import cv2
    out = []
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"  skipping unreadable {p}")
            continue
        out.append(img)
    if not out:
        raise SystemExit(f"error: nothing decoded from {directory}")
    return out


def _capture_iterator(images):
    it = iter(images)

    def capture():
        return next(it, None)

    return capture


def _capture_live(args):
    from vision.frame_source import Picamera2FrameSource

    src = Picamera2FrameSource(config_path=args.image_config,
                               detector_rpk_path="",
                               image_size=(args.width, args.height),
                               orientation=args.orientation)
    print(f"camera: {args.width}x{args.height} orientation={args.orientation}")
    print("Lay the printed board LEVEL on the ground in view of the camera "
          "(it is the gravity reference). Not tilted, not held up.")
    src.start()
    frames = []
    try:
        for i in range(args.frames):
            cap = src.capture()
            img = cap.image
            if img is None:
                continue
            img = np.asarray(img)
            if img.ndim == 3 and img.shape[2] == 4:
                img = img[..., :3]
            frames.append(img)
            if (i + 1) % 5 == 0:
                print(f"  {i + 1}/{args.frames} captured")
            time.sleep(0.15)          # let the board and exposure settle
    finally:
        src.stop()
    return _capture_iterator(frames)


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true",
                     help="capture from the real camera (/usr/bin/python3)")
    src.add_argument("--images", help="fit from a directory of saved views")
    ap.add_argument("--out", "--commit", dest="out",
                    default="calibration/installation_pose.yaml",
                    help="where to atomically commit R_W_B (default: %(default)s)")
    ap.add_argument("--no-commit", action="store_true",
                    help="report only; leave the stored pose alone")
    ap.add_argument("--intrinsics", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--extrinsics", default="calibration/camera_extrinsics.yaml")
    ap.add_argument("--allow-uncalibrated", action="store_true",
                    help="run with default pinhole intrinsics (NEVER to "
                         "commission: R_W_B inherits the fx error smoothly)")
    ap.add_argument("--frames", type=int, default=25,
                    help="live frames to capture (default: %(default)s)")
    ap.add_argument("--n-frames", type=int, default=20,
                    help="valid board observations to calibrate from")
    ap.add_argument("--max-outlier-deg", type=float, default=3.0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--orientation", default="none",
                    help="must match the deployment (this station: rotate_180)")
    ap.add_argument("--image-config", default="")
    ap.add_argument("--grid-cols", type=int, default=6,
                    help="board squares across, as printed on the sheet")
    ap.add_argument("--grid-rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=24.0)
    ap.add_argument("--marker-mm", type=float, default=16.0)
    ap.add_argument("--dict", default="DICT_4X4_50")
    args = ap.parse_args(argv)

    spec = BoardSpec(marker_cols=args.grid_cols, marker_rows=args.grid_rows,
                     square_length_m=args.square_mm / 1000.0,
                     marker_length_m=args.marker_mm / 1000.0,
                     dictionary=args.dict)

    if os.path.exists(args.intrinsics):
        intr = CameraIntrinsics.load(args.intrinsics)
        print(f"intrinsics: {args.intrinsics} -> fx={intr.fx:.2f} "
              f"fy={intr.fy:.2f} cx={intr.cx:.2f} cy={intr.cy:.2f} "
              f"{intr.width}x{intr.height}")
    elif args.allow_uncalibrated:
        intr = CameraIntrinsics(1000.0, 1000.0, args.width / 2.0,
                                args.height / 2.0, (0.0,) * 5,
                                args.width, args.height)
        print(f"WARNING: no {args.intrinsics}; using fx=fy=1000 defaults. "
              "The R_W_B you get is NOT commissionable — run "
              "tools/calibrate_camera_intrinsics.py first.")
    else:
        print(f"error: {args.intrinsics} not found. R_W_B is derived from a "
              "PnP pose, so a wrong fx/cx silently tilts the result. Produce "
              "intrinsics first:\n"
              "  python3 -m tools.calibrate_camera_intrinsics --live "
              "--frames 40 --orientation " + args.orientation + "\n"
              "(or pass --allow-uncalibrated to see the output shape only)",
              file=sys.stderr)
        return 1

    if os.path.exists(args.extrinsics):
        ext = CameraExtrinsics.load(args.extrinsics)
        print(f"extrinsics: {args.extrinsics} (R_P_C loaded)")
    else:
        ext = CameraExtrinsics.aligned()
        print(f"extrinsics: {args.extrinsics} absent -> aligned default "
              "(§28.3; correct if the camera is mounted on the nominal axis)")

    if args.images:
        capture = _capture_iterator(_load_images(args.images))
        print(f"views: {args.images}")
    else:
        capture = _capture_live(args)

    cfg = CalibrationConfig(
        board=spec, intrinsics=intr, extrinsics=ext, R_W_D=None,
        n_frames=args.n_frames, max_outlier_deg=args.max_outlier_deg)
    result = run_calibration(capture, lambda: int(time.monotonic_ns()), cfg)

    print(f"frames: {result.n_frames} used, {result.n_rejected} rejected as "
          f"outliers (> {args.max_outlier_deg:.1f} deg)")
    if result.per_frame_error_px:
        errs = np.asarray(result.per_frame_error_px, dtype=np.float64)
        print(f"per-frame reprojection: min {errs.min():.3f} median "
              f"{np.median(errs):.3f} max {errs.max():.3f} px")
    print(f"reprojection: {result.reprojection_error_px:.3f} px   "
          f"attitude spread (covariance): {result.covariance:.3e} rad^2 "
          f"({np.degrees(np.sqrt(max(result.covariance, 0.0))):.3f} deg)")
    if not result.valid:
        print("error: NOT VALID — the board was never seen well enough. Check "
              "light, focus, and that the printed geometry matches "
              "--square-mm/--marker-mm/--grid-cols/--grid-rows. Nothing "
              "written.", file=sys.stderr)
        return 1

    roll, pitch, yaw = _euler_deg(result.R_W_B)
    print(f"R_W_B: roll {roll:+.3f} deg  pitch {pitch:+.3f} deg  "
          f"yaw {yaw:+.3f} deg")

    if args.no_commit:
        print("--no-commit: nothing written")
        return 0
    commit_R_W_B(result, args.out)
    print(f"committed: {os.path.abspath(args.out)}")
    print("restart controld and expect: "
          "`installation pose: source=stored calibrated=true`")
    return 0


def _euler_deg(R: np.ndarray) -> tuple:
    """ZYX euler angles in degrees, for the human reading the console."""
    R = np.asarray(R, dtype=np.float64)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-9
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return (np.degrees(x), np.degrees(y), np.degrees(z))


if __name__ == "__main__":
    sys.exit(main())
