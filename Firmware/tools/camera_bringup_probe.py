#!/usr/bin/env python3
"""P7/P8 bring-up probe on REAL glass — camera only, no CAN, no motor driver.

Answers, with measurements rather than opinions, the three questions that gate
running tracking on real frames:

  1. Does the shipped `Picamera2FrameSource` (not a copy of it) stream on this
     station's picamera2 0.3.37, and are its `sensor_timestamp_ns` values in the
     CLOCK_MONOTONIC domain controld interpolates the motor history against?
     (§6.2/§11 — a domain mistake here silently kills every measurement.)
  2. Does the frame format that actually comes off this sensor (1920x1080
     XRGB8888, i.e. HxWx4) survive the bridge detector's luma path?
  3. Does the bridge detector fire on REAL pixels when something moves? The
     probe cannot wave a target, so it draws a bright patch at a known moving
     position ON TOP of real frames: real background, controlled target. That
     tests the format/geometry path honestly; it does NOT certify the detector's
     real-world accuracy (only a supervised session with a real target can).

Run:  python3 tools/camera_bringup_probe.py [--frames 60]
Stop anything else that owns the IMX500 first (webd's video feed, visiond).
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from vision.frame_source import Picamera2FrameSource
from vision.simple_detector import MotionBlobConfig, MotionBlobDetector


def hr(title: str) -> None:
    print(f"\n=== {title} ===", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--framerate", type=float, default=30.0)
    ap.add_argument("--orientation", default="rotate_180",
                    choices=["none", "rotate_180", "flip_horizontal", "flip_vertical"],
                    help="this station's IMX500 is mounted upside-down")
    args = ap.parse_args(argv)

    hr("start camera (shipped Picamera2FrameSource)")
    fs = Picamera2FrameSource("", "", orientation=args.orientation,
                              framerate_hz=args.framerate)
    fs.start()
    print(f"  stream size      : {fs.image_size()}")
    print(f"  detection backend: {fs.detection_backend!r} "
          f"({fs.detection_note or 'no RPK attempt'})")

    hr("1) capture + timestamp domain")
    stamps: list[int] = []
    walls: list[int] = []
    shapes: set[tuple] = set()
    dtypes: set[str] = set()
    lumas: list[float] = []
    first: np.ndarray | None = None
    try:
        for _ in range(args.frames):
            cap = fs.capture()
            stamps.append(cap.sensor_timestamp_ns)
            walls.append(int(time.monotonic_ns()))
            shapes.add(None if cap.image is None else tuple(cap.image.shape))
            dtypes.add(None if cap.image is None else str(cap.image.dtype))
            if cap.image is not None:
                lumas.append(float(cap.image[:, :, :3].mean()))
                if first is None:
                    first = cap.image.copy()
    finally:
        fs.stop()

    n = len(stamps)
    d_stamps = np.diff(stamps)
    drift = [w - s for w, s in zip(walls, stamps)]
    print(f"  frames captured  : {n}")
    print(f"  frames WITHOUT a real SensorTimestamp (monotonic fallback used): "
          f"{fs.frames_without_sensor_timestamp}")
    print(f"  image shapes     : {shapes}  dtypes {dtypes}")
    print(f"  mean luma        : min {min(lumas):.1f} max {max(lumas):.1f} "
          f"(a live scene, not a frozen buffer)")
    print(f"  stamp advance    : min {d_stamps.min()/1e6:.3f} ms  "
          f"max {d_stamps.max()/1e6:.3f} ms  "
          f"monotonic={bool((d_stamps > 0).all())}")
    print(f"  target frame gap : {1000.0/args.framerate:.3f} ms")
    print(f"  |wall - sensor|  : min {min(drift)/1e6:.3f} ms  "
          f"max {max(drift)/1e6:.3f} ms   <- SAME CLOCK DOMAIN if milliseconds")
    ok_domain = max(abs(min(drift)), abs(max(drift))) < 250e6  # < 250 ms
    ok_mono = bool((d_stamps > 0).all())
    print(f"  VERDICT domain   : {'PASS' if ok_domain else 'FAIL'} "
          f"(a CLOCK_REALTIME stamp would be ~1.7e9 s off, not ms)")
    print(f"  VERDICT monotonic: {'PASS' if ok_mono else 'FAIL'}")

    hr("2/3) bridge detector on REAL pixels with a controlled target")
    if first is None:
        print("  no image captured; cannot run the detector leg")
        return 1
    cfg = MotionBlobConfig()
    det = MotionBlobDetector(cfg)
    h, w = first.shape[0], first.shape[1]
    path = [(int(0.15 * w) + i * (int(0.6 * w) // 20)) for i in range(20)]
    seen: list[tuple[int, float, float, int]] = []
    for i, cx in enumerate(path):
        frame = first.copy()
        y0, y1 = int(0.4 * h), int(0.6 * h)
        x0, x1 = cx, cx + int(0.08 * w)
        frame[y0:y1, x0:x1, :3] = 245          # bright target on the real scene
        dets = det.detect(frame)
        if dets:
            bx, by = dets[0].centre_px
            seen.append((cx, bx, by, dets[0].class_id))
    print(f"  target size      : {int(0.08*w)} x {int(0.2*h)} px on {w}x{h}")
    print(f"  frames reporting : {len(seen)}/{len(path)} "
          f"(warmup + min-area legitimately suppress the first/last few)")
    for cx, bx, by, cls in seen[-5:]:
        print(f"    target cx={cx:5d} -> reported cx={bx:7.1f} cy={by:7.1f} "
              f"class={cls}  err={bx-cx:+7.1f} px")
    if seen:
        errs = [bx - (cx + int(0.04 * w)) for cx, bx, _, _ in seen]
        moved = seen[-1][1] - seen[0][1]
        want = path[-1] - path[0]
        print(f"  reported centre error vs target CENTRE: mean "
              f"{np.mean(errs):+.1f} px  spread {np.std(errs):.1f} px")
        print(f"  reported motion  : {moved:.1f} px for {want} px commanded "
              f"({100*moved/max(1,want):.0f}%)")
        ok_det = len(seen) >= 8 and moved > 0.6 * want and abs(np.mean(errs)) < 0.15 * w
        print(f"  VERDICT detector : {'PASS' if ok_det else 'CHECK'}")
    else:
        print("  VERDICT detector : FAIL (no detection on a moving patch)")

    hr("summary")
    ok = ok_domain and ok_mono and bool(seen)
    print(("  Camera path + timestamp domain + bridge detector: OK. "
           "P7's remaining gap is the NN detector stack, not the camera.")
          if ok else "  See the FAIL/CHECK lines above before P8.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
