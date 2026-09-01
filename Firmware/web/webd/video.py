"""Low-priority camera video source for the web UI (architecture §42.3).

The video feed is a **separate, low-priority path**: it is produced directly
from the IMX500 *inside the webd process* and NEVER traverses the controld
control IPC socket (§6.1 / doc "video frames should not traverse the critical
control IPC path"). Per §42.3, "control timing wins over browser video", so:

  * the stream is **OFF by default** (the camera is only opened when a client
    turns it on, and released when it is turned off — zero CPU/bandwidth while
    off);
  * the publish **FPS is capped** (reduce resolution/FPS when CPU matters);
  * the capture runs in the camera's own callback thread and only JPEG-encodes
    a frame when it is due (skipped frames cost just a timestamp check).

The camera (picamera2/libcamera) is opened lazily and defensively: webd starts
and serves the telemetry dashboard even if no camera is present; a missing or
busy camera is reported as an error on the video endpoints, never a crash.
"""
from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

import numpy as np

from common import image_corrections as ic


@dataclass
class VideoState:
    """Snapshot of the video source state (returned by the API)."""

    running: bool = False
    width: int = 0
    height: int = 0
    fps: float = 0.0
    quality: int = 0
    camera: str = ""
    error: str = ""
    frames_published: int = 0
    wb_gains: tuple = (1.0, 1.0, 1.0)
    orientation: str = "none"
    white_balance: str = "off"

    def to_dict(self) -> dict:
        return asdict(self)


class VideoSource:
    """A single-owner, on/off MJPEG source fed by the IMX500.

    One capture thread (the picamera2 request callback) produces the *latest*
    JPEG into a shared slot. Any number of browser clients read that slot; each
    only re-sends a frame when it changes, so N viewers cost the same capture.
    """

    def __init__(self, enabled: bool = True, orientation: str = "none",
                 white_balance: str = "off") -> None:
        self._enabled = enabled
        self._orientation = ic.validate_orientation(orientation)
        self._wb_mode = ic.validate_white_balance(white_balance)
        self._frame_lock = threading.Lock()   # protects _latest/_seq/_ts/_count
        self._lifecycle_lock = threading.Lock()  # serializes start/stop + camera
        self._latest: bytes = b""
        self._seq: int = 0
        self._ts: float = 0.0
        self._count: int = 0
        self._running: bool = False
        self._camera = None
        self._state = VideoState()
        self._first_frame = threading.Event()
        self._min_publish_s = 1.0 / 15.0
        self._last_publish = 0.0
        self._open_error = ""
        # Smoothed gray-world white-balance gains (EMA across frames: no flicker).
        self._wb_gains = (1.0, 1.0, 1.0)
        self._wb_alpha = 0.2

    # -- introspection ------------------------------------------------------
    def is_running(self) -> bool:
        return self._running

    def state(self) -> VideoState:
        with self._frame_lock:
            st = VideoState(
                running=self._running,
                width=self._state.width,
                height=self._state.height,
                fps=self._state.fps,
                quality=self._state.quality,
                camera=self._state.camera,
                error=self._state.error or self._open_error,
                frames_published=self._count,
                wb_gains=self._wb_gains,
                orientation=self._orientation,
                white_balance=self._wb_mode,
            )
        return st

    def latest(self) -> Tuple[bytes, int, float]:
        """Latest JPEG frame + sequence number + publish timestamp."""
        with self._frame_lock:
            return self._latest, self._seq, self._ts

    # -- lifecycle ----------------------------------------------------------
    def start(self, width: int, height: int, fps: float,
              quality: int) -> VideoState:
        """Open the camera and begin producing frames (idempotent)."""
        with self._lifecycle_lock:
            if self._running:
                return self.state()
            if not self._enabled:
                self._state = VideoState(error="video disabled (OTA_VIDEO_ENABLE=0)")
                return self.state()

            # Reset publish bookkeeping.
            self._min_publish_s = 1.0 / max(1.0, float(fps))
            self._last_publish = 0.0
            self._seq = 0
            self._count = 0
            self._open_error = ""
            self._wb_gains = (1.0, 1.0, 1.0)
            self._first_frame.clear()
            with self._frame_lock:
                self._latest = b""
                self._ts = 0.0

            try:
                # Lazy import: webd runs even if picamera2/the camera is absent.
                from picamera2 import Picamera2
                from PIL import Image
            except Exception as e:  # noqa: BLE001
                self._state = VideoState(error=f"video stack unavailable: {e}")
                return self.state()

            cam = None
            try:
                cam = Picamera2()
                cfg = cam.create_video_configuration(
                    main={"size": (int(width), int(height))}, buffer_count=3
                )
                if self._wb_mode == "auto":
                    # Disable the camera's own auto-white-balance so it cannot
                    # drift the per-channel balance out from under our software
                    # gray-world (which re-equalizes every frame). With the
                    # camera AWB off the raw balance is stable, so the correction
                    # fully neutralizes the red cast instead of chasing it.
                    try:
                        cfg["controls"]["AwbEnable"] = 0
                    except Exception:  # noqa: BLE001 - control name varies
                        pass
                cam.request_callback = self._make_callback(Image, quality)
                cam.configure(cfg)
                cam.start()  # blocks until the pipeline is up
            except Exception as e:  # noqa: BLE001
                if cam is not None:
                    self._safe_close(cam)
                msg = f"camera open failed: {e}"
                self._open_error = msg
                self._state = VideoState(error=msg)
                return self.state()

            # Confirm the camera is actually producing frames.
            if not self._first_frame.wait(timeout=5.0):
                self._safe_close(cam)
                self._camera = None
                msg = "camera started but produced no frames"
                self._open_error = msg
                self._state = VideoState(error=msg)
                return self.state()

            self._camera = cam
            self._running = True
            model = ""
            try:
                model = cam.camera_properties.get("Model", "")
            except Exception:  # noqa: BLE001
                pass
            self._state = VideoState(
                running=True, width=int(width), height=int(height),
                fps=float(fps), quality=int(quality), camera=model,
            )
            return self.state()

    def stop(self) -> VideoState:
        """Stop producing and release the camera (idempotent)."""
        with self._lifecycle_lock:
            self._running = False
            if self._camera is not None:
                self._safe_close(self._camera)
                self._camera = None
            with self._frame_lock:
                self._latest = b""
                self._ts = 0.0
            self._state = VideoState()  # clean slate
            return self.state()

    # -- internals ----------------------------------------------------------
    def _safe_close(self, cam) -> None:
        for op in ("stop", "close"):
            try:
                getattr(cam, op)()
            except Exception:  # noqa: BLE001
                pass

    def _make_callback(self, Image, quality: int):
        """Build the picamera2 request callback (runs on the camera thread).

        The install-level orientation correction is applied HERE, on the raw frame,
        before any encoding — i.e. before the frame enters processing (doc §42.3) —
        so the preview shows the corrected image. White balance is off by default: the
        sensor feed is already neutral once the BGRX byte order is decoded correctly,
        so it is only needed for a genuinely mis-balanced install.
        """
        state = self
        orientation = state._orientation
        wb_mode = state._wb_mode

        def _cb(request) -> None:
            # FPS cap: only JPEG-encode a frame when one is due. Skipped frames
            # cost just this comparison — the main §42.3 "low CPU" lever.
            now = time.monotonic()
            if now - state._last_publish < state._min_publish_s:
                return
            state._last_publish = now
            try:
                arr = request.make_array("main")
                # 1) Orientation (install-level) on the raw frame, first.
                if orientation != "none":
                    arr = ic.apply_orientation_image(arr, orientation)
                # 2) XBGR8888 -> RGB. V4L2_PIX_FMT_XBGR32 is BGRX-8-8-8-8
                #    (byte0=B, byte1=G, byte2=R, byte3=X). Take the first 3
                #    bytes and reverse -> [R, G, B], dropping the trailing X.
                #    (Getting this order wrong puts the constant-255 X byte into
                #    "red" and drops blue -> a false red cast.)
                rgb = arr[..., :3][..., ::-1]
                # 3) White balance (install-level): auto gray-world, smoothed.
                if wb_mode == "auto":
                    g = ic.gray_world_correction(rgb)
                    a, prev = state._wb_alpha, state._wb_gains
                    state._wb_gains = (
                        a * g[0] + (1 - a) * prev[0],
                        a * g[1] + (1 - a) * prev[1],
                        a * g[2] + (1 - a) * prev[2],
                    )
                    rgb = ic.apply_white_balance(rgb, state._wb_gains)
                buf = io.BytesIO()
                Image.fromarray(np.ascontiguousarray(rgb)).save(
                    buf, "JPEG", quality=quality)
                jpeg = buf.getvalue()
            except Exception as e:  # noqa: BLE001
                state._open_error = f"frame encode failed: {e}"
                return
            with state._frame_lock:
                state._latest = jpeg
                state._seq += 1
                state._ts = now
                state._count += 1
            if not state._first_frame.is_set():
                state._first_frame.set()

        return _cb


def mjpeg_boundary() -> str:
    return "frame"


def mjpeg_frame(jpeg: bytes) -> bytes:
    """One multipart boundary block for an MJPEG response."""
    return (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n"
        b"\r\n" + jpeg + b"\r\n"
    )
