"""Frame sources (architecture §5.1).

A ``FrameSource`` yields captured frames plus the camera ``SensorTimestamp`` and
frame sequence, and the detector detections for that frame. It is the ONLY place
the (real) camera is touched. For tests and for running without a camera, use
``SyntheticFrameSource`` (deterministic, no hardware, no CAN, no motor driver).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, Protocol, Tuple


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

        self._seq = 0
        self._t_ns = 0  # synthetic monotonic clock (capture time)
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def image_size(self) -> Tuple[int, int]:
        return (self._w, self._h)

    def capture(self) -> FrameCapture:
        if not self._started:
            raise RuntimeError("SyntheticFrameSource.start() was not called")

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
            sensor_timestamp_ns=self._t_ns,
            frame_sequence=self._seq,
            detections=dets,
        )
        # Advance the synthetic clock and sequence (capture time is the START of
        # processing this frame, matching a real sensor timestamp).
        self._t_ns += self._period_ns
        self._seq += 1
        return cap


class Picamera2FrameSource:
    """Real IMX500 frame source via Picamera2 (architecture §5.1).

    NOT used in tests or in the synthetic path — it requires the real camera.
    Import is deferred so the module loads (and tests run) without picamera2.
    """

    def __init__(self, config_path: str, detector_rpk_path: str, image_size: Tuple[int, int] = (1920, 1080)) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as e:  # pragma: no cover - requires the real camera
            raise RuntimeError(
                "Picamera2 is not available; use SyntheticFrameSource for "
                "camera-free runs (no motor driver is involved either way)."
            ) from e
        self._picam = Picamera2()
        self._image_size = image_size
        self._config_path = config_path
        self._detector_rpk_path = detector_rpk_path
        self._seq = 0
        self._t0_ns = 0
        self._started = False

    def start(self) -> None:  # pragma: no cover - requires the real camera
        import json
        with open(self._config_path) as f:
            cfg = json.load(f)
        # Load the IMX500 post-processing (detector) model.
        with open(self._detector_rpk_path) as f:
            detector_cfg = json.load(f)
        self._picam.configure(detector_cfg, detect_objects=True)
        self._picam.start()
        self._t0_ns = time.time_ns()
        self._started = True

    def stop(self) -> None:  # pragma: no cover - requires the real camera
        if self._started:
            self._picam.stop()
            self._started = False

    def image_size(self) -> Tuple[int, int]:
        return self._image_size

    def capture(self) -> FrameCapture:  # pragma: no cover - requires the real camera
        frame = self._picam.capture_file("/dev/null", format="bgr8")
        # Real sensor timestamp: use the frame metadata when available; fall
        # back to the host monotonic clock.
        try:
            meta = frame.metadata
            sensor_ns = int(meta.get("SensorTimestamp", 0))
        except Exception:
            sensor_ns = time.time_ns() - self._t0_ns
        dets = self._parse_detections(frame)
        return FrameCapture(
            width=self._image_size[0],
            height=self._image_size[1],
            sensor_timestamp_ns=sensor_ns,
            frame_sequence=self._seq,
            detections=dets,
        )

    def _parse_detections(self, frame) -> List[Detection]:  # pragma: no cover
        dets: List[Detection] = []
        try:
            for obj in frame.metadata.get("Objects", []):
                bbox = obj.get("BoundingBox")  # [x, y, w, h] in px
                dets.append(
                    Detection(
                        class_id=int(obj.get("Label", 0)),
                        confidence=float(obj.get("Score", 0.0)),
                        bbox_x_min_px=float(bbox[0]),
                        bbox_y_min_px=float(bbox[1]),
                        bbox_x_max_px=float(bbox[0] + bbox[2]),
                        bbox_y_max_px=float(bbox[1] + bbox[3]),
                    )
                )
        except Exception:
            pass
        self._seq += 1
        return dets
