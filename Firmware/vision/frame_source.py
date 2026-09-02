"""Frame sources (architecture §5.1).

A ``FrameSource`` yields captured frames plus the camera ``SensorTimestamp`` and
frame sequence, and the detector detections for that frame. It is the ONLY place
the (real) camera is touched. For tests and for running without a camera, use
``SyntheticFrameSource`` (deterministic, no hardware, no CAN, no motor driver).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from typing import List, Optional, Protocol, Tuple

from common import image_corrections as ic


@dataclass
class Detection:
    """One detector output, in image pixels (architecture §10.1)."""

    class_id: int
    confidence: float
    bbox_x_min_px: float
    bbox_y_min_px: float
    bbox_x_max_px: float
    bbox_y_max_px: float

    @property
    def centre_px(self) -> Tuple[float, float]:
        return (
            (self.bbox_x_min_px + self.bbox_x_max_px) / 2.0,
            (self.bbox_y_min_px + self.bbox_y_max_px) / 2.0,
        )

    def as_normalized(self, width: int, height: int) -> Tuple[float, float, float, float]:
        """Return (x_min, y_min, x_max, y_max) as fractions of [0,1]."""
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        return (
            self.bbox_x_min_px / width,
            self.bbox_y_min_px / height,
            self.bbox_x_max_px / width,
            self.bbox_y_max_px / height,
        )


@dataclass
class FrameCapture:
    """One captured frame plus its metadata and detections."""

    width: int
    height: int
    sensor_timestamp_ns: int  # capture time of THIS frame (mandatory, §6.2)
    frame_sequence: int
    detections: List[Detection] = field(default_factory=list)
    image: Optional[bytes] = None  # optional raw image (not sent on control IPC, §6.1)


class FrameSource(Protocol):
    """Interface for frame capture. Real (Picamera2) and synthetic both implement it."""

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def image_size(self) -> Tuple[int, int]: ...

    def capture(self) -> FrameCapture: ...


class SyntheticFrameSource:
    """Deterministic camera stand-in: a target that moves across the image.

    Used for unit tests and for running the vision daemon WITHOUT the real
    IMX500. It never touches CAN or the motor driver. The target moves by
    ``velocity_px_per_frame`` each capture; when it exits the image it wraps so
    the association layer has a continuous track to maintain.

    TIMESTAMPS (§6.2/§11): ``sensor_timestamp_ns`` is published on the HOST
    MONOTONIC clock (``time.monotonic_ns()``, i.e. CLOCK_MONOTONIC — the same
    domain controld timestamps the motor history with, see
    ``control/src/common/time.hpp``). A measurement stamped in any other domain
    can never be interpolated against that history, so controld would reject
    every frame. ``pace=True`` reproduces a real sensor's frame cadence (and
    keeps the timestamps honest: they never run ahead of the host clock).
    """

    def __init__(
        self,
        width: int = 1920,
        height: int = 1080,
        framerate_hz: float = 30.0,
        target_class_id: int = 1,
        target_size_px: float = 120.0,
        velocity_px_per_frame: float = 25.0,
        initial_center_px: Tuple[float, float] = (300.0, 400.0),
        confidence: float = 0.9,
        num_background_boxes: int = 0,
        background_class_id: int = 2,
        background_size_px: float = 60.0,
        pace: bool = True,
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        if framerate_hz <= 0:
            raise ValueError("framerate must be positive")
        self._w = int(width)
        self._h = int(height)
        self._framerate = float(framerate_hz)
        self._period_ns = int(1e9 / self._framerate)
        self._class_id = int(target_class_id)
        self._size = float(target_size_px)
        self._vel = float(velocity_px_per_frame)
        self._cx, self._cy = float(initial_center_px[0]), float(initial_center_px[1])
        self._confidence = float(confidence)
        self._num_bg = int(num_background_boxes)
        self._bg_class = int(background_class_id)
        self._bg_size = float(background_size_px)
        self._pace = bool(pace)

        self._seq = 0
        self._t0_ns = 0  # host monotonic time at start() (the frame-clock origin)
        self._started = False

    def start(self) -> None:
        self._started = True
        self._t0_ns = time.monotonic_ns()

    def stop(self) -> None:
        self._started = False

    def image_size(self) -> Tuple[int, int]:
        return (self._w, self._h)

    def capture(self) -> FrameCapture:
        if not self._started:
            raise RuntimeError("SyntheticFrameSource.start() was not called")

        # Frame schedule on the host monotonic clock. Never in the future: a
        # capture timestamp the motor history cannot reach is rejected by the
        # §11 timestamp alignment, so a slow host clamps to `now` instead.
        scheduled_ns = self._t0_ns + self._seq * self._period_ns
        now_ns = time.monotonic_ns()
        if self._pace and scheduled_ns > now_ns:
            time.sleep((scheduled_ns - now_ns) / 1e9)
            now_ns = scheduled_ns
        capture_ns = max(scheduled_ns, now_ns)

        # Advance the target (wrap around when it exits the image).
        self._cx += self._vel
        self._cy += self._vel * 0.2  # slight diagonal
        if self._cx > self._w + self._size:
            self._cx = -self._size
        if self._cy > self._h + self._size:
            self._cy = -self._size

        dets: List[Detection] = []
        half = self._size / 2.0
        dets.append(
            Detection(
                class_id=self._class_id,
                confidence=self._confidence,
                bbox_x_min_px=self._cx - half,
                bbox_y_min_px=self._cy - half,
                bbox_x_max_px=self._cx + half,
                bbox_y_max_px=self._cy + half,
            )
        )
        # Optional deterministic background boxes (to exercise class gating).
        for i in range(self._num_bg):
            bx = ((self._seq * 97 + i * 131) % (self._w - int(self._bg_size)))
            by = ((self._seq * 61 + i * 73) % (self._h - int(self._bg_size)))
            dets.append(
                Detection(
                    class_id=self._bg_class,
                    confidence=0.5,
                    bbox_x_min_px=bx,
                    bbox_y_min_px=by,
                    bbox_x_max_px=bx + self._bg_size,
                    bbox_y_max_px=by + self._bg_size,
                )
            )

        cap = FrameCapture(
            width=self._w,
            height=self._h,
            sensor_timestamp_ns=capture_ns,
            frame_sequence=self._seq,
            detections=dets,
        )
        self._seq += 1
        return cap


def parse_objects_metadata(meta: dict) -> List[Detection]:
    """Decode AI-stack ``Objects`` frame metadata into Detections.

    Two shapes have been seen in the wild and both are accepted:
      A. tuple/list  ``(score, label, x, y, w, h)`` — the libcamera / RPi
         examples;
      B. dict        ``{"Score", "Label", "BoundingBox": [x, y, w, h]}`` —
         picamera2 AI configuration drafts.
    Anything malformed yields NO detection for that object: the safe direction
    is "no target" (the selector publishes valid=false, §12), never a
    half-decoded box the control loop would chase.
    """
    dets: List[Detection] = []
    for obj in (meta or {}).get("Objects", []) or []:
        try:
            if isinstance(obj, dict):
                bbox = obj["BoundingBox"]  # [x, y, w, h] px
                cls, score = int(obj["Label"]), float(obj["Score"])
                x, y = float(bbox[0]), float(bbox[1])
                w_px, h_px = float(bbox[2]), float(bbox[3])
            else:
                score, cls = float(obj[0]), int(obj[1])
                x, y = float(obj[2]), float(obj[3])
                w_px, h_px = float(obj[4]), float(obj[5])
        except (TypeError, IndexError, KeyError, ValueError):
            continue
        dets.append(
            Detection(
                class_id=cls,
                confidence=score,
                bbox_x_min_px=x,
                bbox_y_min_px=y,
                bbox_x_max_px=x + w_px,
                bbox_y_max_px=y + h_px,
            )
        )
    return dets


class Picamera2FrameSource:
    """Real IMX500 frame source via Picamera2 (architecture §5.1).

    Written against the picamera2 that is actually installed on the station
    (0.3.37 — see `docs/research_vision_readiness_p7.md`): the camera is
    STARTED once and frames arrive on picamera2's request-callback thread, which
    publishes the newest frame into a one-slot mailbox; `capture()` takes the
    newest and never blocks the camera thread. That is the same pattern webd's
    video source uses successfully on this box (`web/webd/video.py`).

    Detection is OPTIONAL and never assumed:
      * ``detector_rpk_path`` set -> attempt the IMX500 AI-stack configuration.
        If the platform has no such API (today's station) the source still
        starts, sets ``detection_backend = "none"`` and yields frames WITHOUT
        detections. The selector then publishes valid=false measurements and
        controld never tracks — the safe outcome (§12/§34).
      * For P8 bring-up without the AI stack, wrap this source in
        ``DetectedFrameSource`` (``visiond --detector simple``).

    ``image`` is a numpy array. The install orientation (the IMX500 is mounted
    upside-down here) is applied to the FRAME and to any detector box together,
    so pixels and control geometry live in the same corrected frame (§29.1).
    """

    def __init__(self, config_path: str, detector_rpk_path: str,
                 image_size: Tuple[int, int] = (1920, 1080),
                 orientation: str = "none", framerate_hz: float = 30.0) -> None:
        self._orientation = ic.validate_orientation(orientation)
        try:
            from picamera2 import Picamera2  # noqa: F401
        except ImportError as e:  # pragma: no cover - requires picamera2
            raise RuntimeError(
                "Picamera2 is not available; use SyntheticFrameSource for "
                "camera-free runs (no motor driver is involved either way)."
            ) from e
        self._Picamera2 = Picamera2
        self._image_size = (int(image_size[0]), int(image_size[1]))
        self._config_path = config_path
        self._detector_rpk_path = detector_rpk_path
        self._framerate = float(framerate_hz)
        self._seq = 0
        self._started = False
        self._picam = None
        # Newest-frame mailbox written by the camera thread (§46-style: the
        # camera thread never blocks on a consumer, the consumer sees stale).
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[int, int, object, dict]] = None
        self.frames_dropped = 0            # consumer too slow (overwrites)
        self.detection_backend = "none"    # "none" | "rpk"
        self.detection_note = ""

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:  # pragma: no cover - requires the real camera
        import json

        self._picam = self._Picamera2()
        if self._config_path:
            with open(self._config_path) as f:
                cfg = json.load(f)
            self._picam.configure(cfg)
        else:
            cfg = self._picam.create_video_configuration(
                main={"size": self._image_size}, buffer_count=4
            )
            try:
                cfg["controls"]["FrameRate"] = self._framerate
            except Exception:  # noqa: BLE001 - control availability varies
                pass
            self._picam.request_callback = self._on_request
            self._picam.configure(cfg)

        if self._detector_rpk_path:
            self._try_configure_rpk()   # optional, never fatal

        self._picam.start()
        # Adopt the size the sensor actually negotiated (the IMX500 picks its
        # own RAW size; the main stream follows the requested one).
        try:
            sz = self._picam.stream_configuration.get("size")
            if sz:
                self._image_size = (int(sz[0]), int(sz[1]))
        except Exception:  # noqa: BLE001
            pass
        self._started = True

    def _try_configure_rpk(self) -> None:  # pragma: no cover - needs the AI stack
        """Try the IMX500 AI (RPK) detection API. Never fatal: no AI stack on
        this platform means no detections, not a crashed vision daemon."""
        picam = self._picam
        attempts = (
            lambda: picam.configure(self._detector_rpk_path, detect_objects=True),
            lambda: picam.postprocessing_config.load(self._detector_rpk_path),
        )
        for attempt in attempts:
            try:
                attempt()
                self.detection_backend = "rpk"
                self.detection_note = ""
                return
            except Exception as e:  # noqa: BLE001
                self.detection_note = f"RPK detection API unavailable: {e}"
        print(f"warning: {self.detection_note}; streaming WITHOUT detections",
              flush=True)

    def _on_request(self, request) -> None:  # pragma: no cover - real camera
        """picamera2 request callback (camera thread): keep only the newest."""
        try:
            meta = dict(request.get_metadata("main") or {})
            arr = request.make_array("main")
        except Exception:  # noqa: BLE001
            return
        seq = self._seq
        self._seq += 1
        with self._lock:
            if self._latest is not None:
                self.frames_dropped += 1
            self._latest = (seq, int(meta.get("SensorTimestamp", 0) or 0),
                            arr, meta)

    def stop(self) -> None:  # pragma: no cover - requires the real camera
        self._started = False
        if self._picam is not None:
            for op in ("stop", "close"):
                try:
                    getattr(self._picam, op)()
                except Exception:  # noqa: BLE001
                    pass
            self._picam = None

    def image_size(self) -> Tuple[int, int]:
        return self._image_size

    # -- capture ------------------------------------------------------------
    def capture(self) -> FrameCapture:  # pragma: no cover - requires the real camera
        if not self._started:
            raise RuntimeError("Picamera2FrameSource.start() was not called")
        deadline = time.monotonic() + 2.0
        latest = None
        while latest is None and time.monotonic() < deadline:
            with self._lock:
                latest = self._latest
            if latest is None:
                time.sleep(0.002)
        if latest is None:
            raise RuntimeError("camera produced no frames within 2 s")

        seq, sensor_ns, arr, meta = latest
        # libcamera's SensorTimestamp is host CLOCK_MONOTONIC — the same domain
        # controld stamps the motor history with (§6.2/§11). If a frame carries
        # none, stamp it with the monotonic clock; NEVER CLOCK_REALTIME.
        meta = dict(meta)
        if sensor_ns <= 0:
            sensor_ns = int(time.monotonic_ns())
            meta["SensorTimestamp"] = sensor_ns

        dets = self._parse_detections(meta)
        if self._orientation != "none":
            arr = ic.apply_orientation_image(arr, self._orientation)
            dets = [self._reorient(d) for d in dets]
            meta["ImageOrientationApplied"] = self._orientation

        return FrameCapture(
            width=self._image_size[0],
            height=self._image_size[1],
            sensor_timestamp_ns=sensor_ns,
            frame_sequence=seq,
            detections=dets,
            image=arr,
        )

    def _parse_detections(self, meta: dict) -> List[Detection]:  # pragma: no cover
        return parse_objects_metadata(meta)

    def _reorient(self, det: Detection) -> Detection:  # pragma: no cover
        """Re-express one detection box in the install-corrected image frame."""
        x0, y0, x1, y1 = ic.apply_orientation_bbox(
            (det.bbox_x_min_px, det.bbox_y_min_px,
             det.bbox_x_max_px, det.bbox_y_max_px),
            self._orientation, self._image_size[0], self._image_size[1],
        )
        return replace(
            det,
            bbox_x_min_px=x0, bbox_y_min_px=y0,
            bbox_x_max_px=x1, bbox_y_max_px=y1,
        )


class DetectedFrameSource:
    """FrameSource decorator: classical detections for a source that has none.

    Used by ``visiond --detector simple`` for P8 bring-up — see
    ``vision/simple_detector.py`` for what it is and, more importantly, what it
    is NOT (it is not the §10.1 detector and never ships as one). If the wrapped
    source already reports detections (a real NN/RPK), they are kept: this only
    fills a platform gap, it never overrides a real detector.
    """

    def __init__(self, inner: "FrameSource", detector) -> None:
        self._inner = inner
        self._det = detector
        self.frames = 0
        self.blobs = 0

    def start(self) -> None:
        self._inner.start()

    def stop(self) -> None:
        self._inner.stop()

    def image_size(self) -> Tuple[int, int]:
        return self._inner.image_size()

    def capture(self) -> FrameCapture:
        cap = self._inner.capture()
        self.frames += 1
        if cap.detections:
            return cap          # a real detector already spoke for this frame
        if cap.image is None:
            raise RuntimeError("DetectedFrameSource needs FrameCapture.image")
        dets = self._det.detect(cap.image)
        if dets:
            self.blobs += 1
        return replace(cap, detections=dets)
