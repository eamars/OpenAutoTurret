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
import logging
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

import numpy as np

from common import image_corrections as ic

log = logging.getLogger(__name__)


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
    # What the stream ACTUALLY delivers, measured over a short trailing window, kept apart from
    # `fps`, which is what was ASKED for. They were conflated until 2026-09-06: the panel showed
    # `fps: 15.00` while the server published exactly 10.00 fps - measured twice at 200 frames in
    # 20.0 s (idle, and with a client draining 1 MiB/s), and again ~10.1 fps at both 1080p and
    # 720p. No frame rate is ever requested from the camera: create_video_configuration is given
    # only a size, so 15 was never asked for, let alone measured. Reporting a wish in a field a
    # §20 contract calls `fps` is how a panel comes to lie quietly; both numbers are here now.
    fps_published: float = 0.0
    # Rate at which completed requests ARRIVE from the camera - see measured_arrival_fps().
    sensor_fps: float = 0.0
    wb_gains: tuple = (1.0, 1.0, 1.0)
    orientation: str = "none"
    white_balance: str = "off"
    # The negotiated main-stream pixel format, and the verdict of the one-time check that
    # our own decode agrees with the camera library's encoder. Both are reported on the
    # page because colour errors in a preview are invisible by construction: a permuted
    # picture still looks exactly like a picture.
    pixel_format: str = ""
    colour_check: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


class VideoSource:
    """A single-owner, on/off MJPEG source fed by the IMX500.

    One capture thread PULLS completed camera requests and produces the *latest*
    JPEG into a shared slot. Any number of browser clients read that slot; each
    only re-sends a frame when it changes, so N viewers cost the same capture.

    Why pull rather than the request callback: on this station's picamera2
    (0.3.37) `request_callback` is deprecated and silently maps onto
    `post_callback`, which fired only ~0.8 times per second (measured: 2
    callbacks in 2.5 s) and left `stop()` blocked forever on a job that never
    completed — i.e. a slideshow UI that could not be switched off. Pulling with
    `capture_request()` delivered 40 requests in 2.0 s and stopped cleanly, on
    the same camera in the same process. Same finding killed the callback design
    in `vision/frame_source.py`; both now use the pull pattern.
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
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()
        self._state = VideoState()
        self._first_frame = threading.Event()
        self._pixel_format = ""
        self._colour_check = ""
        self._colour_checked = False
        self._min_publish_s = 1.0 / 15.0
        # Publish timestamps over a trailing window, to measure the rate actually delivered.
        self._recent: list = []
        # When each COMPLETED REQUEST arrived from the camera, before any processing. Separates
        # 'the sensor delivers 10 fps to this path' from 'we process at 10 fps'.
        self._arrived: list = []
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
                fps_published=self.measured_fps(),
                sensor_fps=self.measured_arrival_fps(),
                wb_gains=self._wb_gains,
                orientation=self._orientation,
                white_balance=self._wb_mode,
                pixel_format=self._pixel_format,
                colour_check=self._colour_check,
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
            self._recent = []
            self._arrived = []
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
                # Ask for the rate on a control this configuration ACTUALLY advertises. Round 6 set
                # controls["FrameRate"], which the probe in round 8 showed is not exposed here at all
                # (the advertised controls are FrameDurationLimits and NoiseReductionMode) - so that
                # change was a silent no-op, and the `except Exception: pass` wrapped around it turned
                # a write that did nothing into a change that looked like it did something. Rate is
                # requested in microseconds per frame, min and max pinned together; FrameDurationLimits
                # is a paired control, and giving only one bound lets the sensor wander.
                try:
                    dur_us = int(round(1e6 / max(1.0, float(fps))))
                    cfg["controls"]["FrameDurationLimits"] = (dur_us, dur_us)
                except Exception as e:  # noqa: BLE001 - logged, never swallowed
                    print(f"video: could not request frame duration at {fps} fps: {e}")
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
                # The channel order is a property of the negotiated format, so the format
                # is read from the configuration instead of being assumed. An empty value is
                # left empty and refused downstream: guessing here is how a preview ends up
                # with red and blue exchanged and nothing else wrong with it.
                self._pixel_format = str((cfg.get("main") or {}).get("format", "") or "")
                log.info("video main stream negotiated as %r, orientation %s",
                         self._pixel_format, self._orientation)
                cam.configure(cfg)
                cam.start()  # blocks until the pipeline is up
                # OUR thread pulls requests (see the class docstring for why the
                # callback attribute must NOT be set on this picamera2).
                self._stop_evt.clear()
                self._thread = threading.Thread(
                    target=self._pull_loop, args=(cam, Image, quality),
                    name="webd-video-capture", daemon=True)
                self._thread.start()
            except Exception as e:  # noqa: BLE001
                if cam is not None:
                    self._safe_close(cam)
                msg = f"camera open failed: {e}"
                self._open_error = msg
                self._state = VideoState(error=msg)
                return self.state()

            # Confirm the camera is actually producing frames.
            if not self._first_frame.wait(timeout=5.0):
                # Keep a reason that is already known: "no frames" is a symptom, and the reason
                # (an undecodable format, say) was recorded by the thread that hit it.
                self._join_thread()
                self._safe_close(cam)
                self._camera = None
                # Keep a reason that is already known. "Produced no frames" is a symptom;
                # the cause (an undecodable pixel format, say) was recorded by the capture
                # thread, and losing it here is how a colour defect becomes a mystery.
                msg = self._open_error or "camera started but produced no frames"
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
            # The puller must be gone BEFORE the camera is stopped, or it is
            # blocked in capture_request() on a camera that no longer exists.
            self._join_thread()
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

    def _join_thread(self) -> None:
        """Ask the capture thread to finish and wait for it (bounded)."""
        self._stop_evt.set()
        th, self._thread = self._thread, None
        if th is not None and th.is_alive():
            # One pull iteration costs about a frame period; 3 s is generous and
            # bounded on purpose — a hung join would hang webd's shutdown.
            th.join(timeout=3.0)

    def _pull_loop(self, cam, Image, quality: int) -> None:
        """Capture thread: pull completed requests, publish JPEGs at the cap.

        `capture_request()` blocks for roughly one frame period, so a stop is
        noticed within ~1 frame and every request is released — holding one
        starves the pipeline, which is how the callback variant wedged.
        """
        while not self._stop_evt.is_set():
            try:
                request = cam.capture_request()
            except Exception as e:  # noqa: BLE001
                # Camera died / was stopped under us. Record it; the MJPEG
                # handler keeps serving the last frame and the UI shows the error.
                self._open_error = f"camera request failed: {e}"
                return
            now = time.monotonic()
            self._arrived.append(now)
            try:
                # FPS cap: only JPEG-encode a frame when one is due. Skipping is
                # cheap — and the request is ALWAYS released (the main §42.3
                # "low CPU" lever must not come at the cost of the pipeline).
                if now - self._last_publish >= self._min_publish_s:
                    self._encode_request(request, Image, quality, now)
            except Exception as e:  # noqa: BLE001
                self._open_error = f"frame encode failed: {e}"
            finally:
                try:
                    request.release()
                except Exception:  # noqa: BLE001
                    pass

    def measured_arrival_fps(self, now=None, window_s: float = 5.0) -> float:
        """Rate at which COMPLETED REQUESTS arrive from the camera, before any processing.

        Compared against measured_fps() this is the whole diagnosis in two numbers: arrival ~15 and published ~10 puts
        the loss inside this file's per-frame work; arrival ~10 puts it upstream (mode selection, or the single-
        in-flight capture_request pattern), where copying a config argument could never have helped - as rounds 6 and
        9 already hinted by applying changes that moved nothing.
        """
        now = time.monotonic() if now is None else now
        cut = now - window_s
        self._arrived = [t for t in self._arrived if t >= cut]
        return (len(self._arrived) / window_s) if self._arrived else 0.0

    def measured_fps(self, now=None, window_s: float = 5.0) -> float:
        """Frames per second over the trailing window; 0.0 when nothing has been published.

        The window is short on purpose: it must fall to zero within seconds of the stream dying, so
        a frozen pane cannot go on advertising the rate it had when things were healthy.
        """
        now = time.monotonic() if now is None else now
        cut = now - window_s
        self._recent = [t for t in self._recent if t >= cut]
        return (len(self._recent) / window_s) if self._recent else 0.0

    def _encode_request(self, request, Image, quality: int, now: float) -> None:
        """Turn one completed request into the newest JPEG in the shared slot.

        The install-level orientation correction is applied HERE, on the raw frame,
        before any encoding — i.e. before the frame enters processing (doc §42.3) —
        so the preview shows the corrected image. White balance is off by default: the
        sensor feed is already neutral once the BGRX byte order is decoded correctly,
        so it is only needed for a genuinely mis-balanced install.
        """
        state = self
        orientation = state._orientation
        wb_mode = state._wb_mode
        state._last_publish = now
        self._recent.append(now)
        arr = request.make_array("main")
        # The colour-order check (below, once) needs the decode WITHOUT the install orientation:
        # request.make_image() knows nothing about how the lens is mounted, so comparing a
        # rotated frame against an unrotated one compares two different pictures. On this
        # station — rotate_180 — that mistake made every channel order score the same mean
        # difference (59.7 against 59.7), and the check cheerfully reported "ok" while having
        # distinguished nothing.
        raw_rgb = None
        if not state._colour_checked:
            try:
                raw_rgb = ic.rgb_from_stream(arr, state._pixel_format)
            except ValueError:
                raw_rgb = None  # the refusal below is the more important report
        # 1) Orientation (install-level) on the raw frame, first.
        if orientation != "none":
            arr = ic.apply_orientation_image(arr, orientation)
        # 2) The stream's channels -> RGB, by the format the pipeline actually negotiated.
        #    This line used to read  rgb = arr[..., :3][..., ::-1]  and was justified by the
        #    comment below it: V4L2_PIX_FMT_XBGR32 is 0x00RRGGBB in host byte order, so the
        #    bytes were assumed to arrive B,G,R,X. That reasoning was WRONG and it lived here
        #    a long time, because picamera2's own encoder treats XBGR8888 as "RGBX" in memory
        #    order; reversing on top of that exchanges red and blue. The symptom is not a crash
        #    and not a cast: white stays white, so the scene looks normal until something
        #    saturated shows up. The operator at this station described it exactly -- "the
        #    yellow is rendered as blue", "not just yellow blue swap, but also pink purple
        #    swap" -- which is R<->B precisely. Measured on the live frame: 1.71% blue-dominant
        #    pixels against 0.02% red-dominant, near-whites dead neutral (a cast moves white, a
        #    swap does not). The order now comes from the format name; see
        #    common/image_corrections.py for where that correspondence comes from.
        try:
            rgb = ic.rgb_from_stream(arr, state._pixel_format)
        except ValueError as exc:
            state._open_error = "cannot decode camera frame: %s" % exc
            log.error("%s", state._open_error)
            return
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
        Image.fromarray(np.ascontiguousarray(rgb)).save(buf, "JPEG", quality=quality)
        jpeg = buf.getvalue()
        with state._frame_lock:
            state._latest = jpeg
            state._seq += 1
            state._ts = now
            state._count += 1
        if not state._first_frame.is_set():
            state._first_frame.set()
        if not state._colour_checked:
            state._colour_checked = True
            if raw_rgb is not None:
                state._check_colour_order(request, raw_rgb, state._pixel_format)

    def _check_colour_order(self, request, rgb, pixel_format: str) -> None:
        """Once per start: compare our decode against the camera library's own encoder.

        picamera2 can hand us the same frame as a PIL image through a path it owns end to end.
        If our hand-rolled channel order is right, the two agree; if red and blue have been
        exchanged, the swapped comparison is decisively closer. That makes this a check of the
        thing static reasoning could not settle -- what the bytes on this kernel with this sensor
        actually are -- and it costs one extra encode, once, at startup.

        Two things make this comparison valid rather than decorative, and both were got wrong
        first: it runs on the decode BEFORE the install orientation (the reference frame is not
        rotated, so a rotated comparison is a comparison of two different pictures), and it
        refuses to conclude anything from a colourless frame, which cannot tell one channel
        order from another.
        """
        try:
            ref = np.asarray(request.make_image("main").convert("RGB"), dtype=np.int16)
            ours = np.asarray(rgb, dtype=np.int16)
            if ref.shape != ours.shape:
                self._colour_check = "skipped (reference frame is %r, ours is %r)" % (
                    tuple(ref.shape), tuple(ours.shape))
                return
            if int((ours.max(axis=2) - ours.min(axis=2)).mean()) < 12:
                self._colour_check = (
                    "inconclusive: the first frame had almost no colour in it, which cannot "
                    "tell one channel order from another")
                log.warning("colour-order check inconclusive for %s (%s)",
                            pixel_format, self._colour_check)
                return
            # Every permutation, not just the one I thought of. The first version of this
            # check compared the decoded frame only against its red/blue exchange, and a
            # different mistake — taking the padding byte as one of the colour channels —
            # sailed straight through it, because "as decoded is closer than an R/B swap" is
            # true of a great many wrong answers. Testing all six makes the verdict mean
            # something: the decode we ran has to be the best of the six, not merely better
            # than one rival.
            names = {(0, 1, 2): "as decoded (R,G,B)", (0, 2, 1): "green and blue exchanged",
                     (1, 0, 2): "red and green exchanged", (1, 2, 0): "channels rotated",
                     (2, 0, 1): "channels rotated the other way",
                     (2, 1, 0): "red and blue exchanged"}
            scores = {}
            for perm in names:
                scores[perm] = float(np.abs(ours[..., list(perm)] - ref).mean())
            identity = float(scores[(0, 1, 2)])
            best = min(scores, key=lambda k: scores[k])
            if best != (0, 1, 2) and scores[best] + 6.0 < identity:
                self._colour_check = (
                    "FAILED: our decode disagrees with picamera2's encoder for %s — it looks "
                    "like %s (mean difference %.1f as decoded, %.1f that way)"
                    % (pixel_format, names[best], identity, scores[best]))
                log.error("camera colour order is wrong: %s", self._colour_check)
            else:
                rival = min((v for k, v in scores.items() if k != (0, 1, 2)))
                self._colour_check = (
                    "ok: matches picamera2's encoder for %s (mean difference %.1f; the closest "
                    "other channel order would be %.1f)" % (pixel_format, identity, rival))
                log.info("colour-order check: %s", self._colour_check)
        except Exception as exc:  # noqa: BLE001 - a failed diagnostic must not stop the video
            self._colour_check = "not performed (%s)" % exc
            log.warning("colour-order check could not run: %s", exc)


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
