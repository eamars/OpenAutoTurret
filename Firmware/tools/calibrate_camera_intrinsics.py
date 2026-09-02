#!/usr/bin/env python3
"""Fit §28.2 camera intrinsics from the ChArUco board -> camera_intrinsics.yaml.

Until this file produces `calibration/camera_intrinsics.yaml`, controld logs
`UNCALIBRATED` and runs on a made-up pinhole (fx=fy=1000, centre principal
point), so every bearing the tracker computes carries an unknown bias — P8 and
P9 can only be read qualitatively. This is the commissioning step that closes
that gap with the board you already print via `tools/make_charuco_board.py`.

Station use (system python — the venv has no picamera2):

    cd Firmware
    python3 -m tools.calibrate_camera_intrinsics --live --frames 40 \
        --orientation rotate_180 --save-dir build/intrinsics_views

    # or from images captured any other way:
    python3 -m tools.calibrate_camera_intrinsics --images build/intrinsics_views

    # prove the toolchain on this machine before trusting it on real glass:
    python3 -m tools.calibrate_camera_intrinsics --self-test

Capture discipline (this is what decides the fit, not the maths):
  * The camera must be running the SAME orientation and stream size as the
    deployment: intrinsics describe the corrected frame visiond publishes, so
    `--orientation` here must equal the unit's, and the size must equal what
    visiond will run (`--expect-size` refuses a mismatch).
  * Move the board, do not just stand still: 12+ views, near and far, corners
    of the frame, tilted both ways. A flat single-distance set cannot separate
    fx from distortion and the fit will look fine while lying.
  * The board geometry here MUST match the printed sheet (the sheet says it).
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.installation_calibration import BoardSpec  # noqa: E402
from vision.intrinsics_calibration import (  # noqa: E402
    Fit, collect_views, render_synthetic_views, solve, write_intrinsics)


def _load_images(directory: str):
    exts = ("png", "jpg", "jpeg", "bmp", "pgm")
    paths = sorted(pth for e in exts
                   for pth in glob.glob(os.path.join(directory, f"*.{e}")))
    if not paths:
        raise SystemExit(f"error: no images found in {directory}")
    import cv2
    out = []
    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_COLOR)
        if img is None:
            print(f"  skipping unreadable {p}")
            continue
        out.append(img)
    if not out:
        raise SystemExit(f"error: {directory} had files but none decoded")
    return out, paths


def _capture_live(args):
    from vision.frame_source import Picamera2FrameSource
    src = Picamera2FrameSource(
        config_path=args.image_config,
        detector_rpk_path="",
        image_size=(args.width, args.height),
        orientation=args.orientation)
    print(f"camera: {args.width}x{args.height} @ {src.__class__.__name__} "
          f"orientation={args.orientation}")
    print("Move the board: near/far, all corners of the frame, tilted both ways.")
    print(f"Capturing {args.frames} frames, keeping the ones with >= "
          f"{args.min_corners} board corners...")
    src.start()
    images, kept = [], 0
    try:
        for i in range(args.frames):
            cap = src.capture()
            img = cap.image
            if img is None:
                continue
            if img.ndim == 3 and img.shape[2] == 4:
                img = img[..., :3]
            images.append(np.asarray(img))
            if args.save_dir:
                os.makedirs(args.save_dir, exist_ok=True)
                import cv2
                cv2.imwrite(os.path.join(
                    args.save_dir, f"view_{i:03d}.png"),
                    np.asarray(img)[..., :3] if np.asarray(img).ndim == 3
                    else np.asarray(img))
            kept += 1
            if (i + 1) % 5 == 0:
                print(f"  {i + 1}/{args.frames} captured")
    finally:
        src.stop()
    print(f"captured {kept} frames")
    return images


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--images", help="directory of board views to fit from")
    src.add_argument("--live", action="store_true",
                     help="capture from the real camera (needs picamera2: "
                          "/usr/bin/python3 on the station)")
    src.add_argument("--self-test", action="store_true",
                     help="fit synthetic views rendered through a KNOWN pinhole "
                          "and report the recovery error (no camera needed)")
    ap.add_argument("--out", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--frames", type=int, default=40,
                    help="live frames to attempt (default: %(default)s)")
    ap.add_argument("--save-dir", default="",
                    help="also write captured views here, to re-fit offline")
    ap.add_argument("--min-views", type=int, default=8)
    ap.add_argument("--min-corners", type=int, default=12)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--expect-size", default="",
                    help="WxH the deployment will actually run at; the fit is "
                         "refused if it differs (intrinsics are per geometry)")
    ap.add_argument("--orientation", default="none",
                    help="must match the deployment (this station: rotate_180)")
    ap.add_argument("--image-config", default="")
    ap.add_argument("--grid-cols", type=int, default=6,
                    help="board squares across, as printed on the sheet")
    ap.add_argument("--grid-rows", type=int, default=6)
    ap.add_argument("--square-mm", type=float, default=24.0)
    ap.add_argument("--marker-mm", type=float, default=16.0)
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--dry-run", action="store_true",
                    help="report only; do not write the file")
    args = ap.parse_args(argv)

    spec = BoardSpec(marker_cols=args.grid_cols, marker_rows=args.grid_rows,
                     square_length_m=args.square_mm / 1000.0,
                     marker_length_m=args.marker_mm / 1000.0,
                     dictionary=args.dict)

    if args.self_test:
        truth = Fit(fx=610.0, fy=612.0, cx=322.0, cy=241.0)
        views_imgs = render_synthetic_views(spec, truth, n_views=16,
                                            size=(640, 480))
        views, skipped, per_view, size = collect_views(views_imgs, spec,
                                                       args.min_corners)
        rep = solve(views, size, per_view, skipped)
        ef = abs(rep.pinhole.fx - truth.fx) / truth.fx * 100.0
        ey = abs(rep.pinhole.fy - truth.fy) / truth.fy * 100.0
        ecx = abs(rep.pinhole.cx - truth.cx)
        ecy = abs(rep.pinhole.cy - truth.cy)
        print("SELF-TEST (synthetic views through a known pinhole)")
        print(f"  truth : fx={truth.fx:.2f} fy={truth.fy:.2f} "
              f"cx={truth.cx:.2f} cy={truth.cy:.2f}")
        print(f"  fitted: {rep.summary()}")
        print(f"  error : fx {ef:.3f}%  fy {ey:.3f}%  cx {ecx:.2f}px  "
              f"cy {ecy:.2f}px")
        ok = ef < 1.5 and ey < 1.5 and ecx < 3.0 and ecy < 3.0
        print(f"  verdict: {'PASS' if ok else 'FAIL'} "
              f"(limits 1.5% / 3 px; warp-quantised renders cost ~0.5%)")
        return 0 if ok else 1

    if args.images:
        images, paths = _load_images(args.images)
        meta = [f"views from {args.images} ({len(paths)} files)"]
    elif args.live:
        images = _capture_live(args)
        meta = [f"live capture, orientation={args.orientation}, "
                f"{args.width}x{args.height}"]
    else:
        print("error: choose --images DIR, --live, or --self-test",
              file=sys.stderr)
        return 2

    views, skipped, per_view, size = collect_views(images, spec,
                                                   args.min_corners)
    print(f"views: {len(views)} usable, {skipped} skipped "
          f"(corners/view: {', '.join(str(n) for n in per_view[:24])}"
          f"{'...' if len(per_view) > 24 else ''})")
    if len(views) < args.min_views:
        print(f"error: need at least {args.min_views} usable views; got "
              f"{len(views)}. More light, closer board, more angles.",
              file=sys.stderr)
        return 1
    rep = solve(views, size, per_view, skipped)

    if args.expect_size:
        try:
            want = tuple(int(x) for x in
                         args.expect_size.lower().strip("x ").replace("x", " ")
                         .split())
        except ValueError:
            print(f"error: --expect-size wants WxH, got {args.expect_size!r}",
                  file=sys.stderr)
            return 2
        if want != (rep.width, rep.height):
            print(f"error: fitted {rep.width}x{rep.height} but the deployment "
                  f"runs {want[0]}x{want[1]}. Intrinsics are per stream "
                  "geometry: fx/cx from another resolution are wrong by the "
                  "scale factor, silently. Recapture at the deployment size.",
                  file=sys.stderr)
            return 1

    print(f"fit   : {rep.summary()}")
    for note in rep.notes:
        print(f"note  : {note}")
    if args.orientation == "none":
        print("note  : --orientation none. If the deployed visiond runs "
              "--orientation rotate_180 (this station does), fit THAT frame: "
              "the principal point moves with the rotation.")

    if args.dry_run:
        print("dry run: nothing written")
        return 0
    text = write_intrinsics(args.out, rep, meta=meta)
    print(f"wrote : {os.path.abspath(args.out)} "
          f"({len(text.splitlines())} lines)")
    print("        controld picks it up on restart; the boot log must then say")
    print("        `camera intrinsics: fx=... (loaded)` instead of UNCALIBRATED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
