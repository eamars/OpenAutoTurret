"""The shape a detection result carries between the perception process and everything else.

Written against measurements, not aspirations. The dual-camera -> one Hailo validation on the
station (2026-09-29) showed that with two sensors feeding one accelerator, the only way a
downstream consumer can tell "who saw this and when did the scene happen" is if **both travel
with the result** -- a shared `frame` counter or a per-camera side channel both lose it the
moment queues are bounded and the newest frame wins.

Field decisions, and why each one is the way it is:

``camera_id``
    A stable identity derived from the device's firmware node path, **not** the enumeration
    index. The station's own ``visiond`` refuses to trust ``Num`` for exactly this reason
    (``identity ... source=by-path``), and the indices were observed to move on this machine.
    Use :func:`camera_id_from_path`; a caller passing ``"0"`` is asking to be wrong someday.

``capture_timestamp_ns``
    ``SensorTimestamp``: **start of exposure**, in the sensor's monotonic domain. It is *not*
    delivery time and not application-receipt time -- those would understate the latency a
    gimbal has to survive by one ISP traversal. ``None`` when the frame arrived without it;
    never ``0``, because 0 is a plausible-looking time and a missing value is not.

``inference_timestamp_ns``
    Same monotonic domain, taken when the accelerator result was complete. Measured values for
    ``inference - capture`` on this station: 29 ms p50 / 40 ms p99 for the HQ camera and
    46/56 ms for the wide through the ISP inference stream -- and that figure *includes* the
    33 ms exposure the auto-exposure had reached, so it is conservative for a moving target.

``bbox``
    Normalised ``(x, y, w, h)`` in the **source** frame, because the inference stream and the
    preview stream have different sizes; the sizes travel alongside so a consumer can convert
    either way without guessing.

This module deliberately has no dependencies on picamera2, Hailo, or the transport: it is the
contract, and the contract has to be importable by a test on a machine with no camera.
"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha1
from typing import Any, Iterable


class FrameShapeError(ValueError):
    """The payload does not describe a frame this system can reason about."""


def camera_id_from_path(fwnode_path: str, model: str) -> str:
    """Stable id from the firmware node path (the thing ``/dev/v4l/by-path`` encodes).

    ``/base/axi/pcie@1000120000/rp1/i2c@88000/imx500@1a`` and the same device enumerated as a
    different ``Num`` must produce the same id, so the enumeration index is not in the input.
    """
    path = fwnode_path.strip().strip("/")
    if not path:
        raise FrameShapeError("empty fwnode path: a camera identity cannot be invented")
    sensor = path.rsplit("/", 1)[-1]
    if "@" not in sensor:
        raise FrameShapeError(f"fwnode path does not end in a <driver>@<addr> node: {path!r}")
    digest = sha1(f"{path}|{model}".encode("utf-8")).hexdigest()[:8]
    return f"cam-{digest}"


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    source: str

    def __post_init__(self) -> None:
        x, y, w, h = self.bbox
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 < w <= 1.0 and 0.0 < h <= 1.0):
            raise FrameShapeError(f"bbox outside the normalised frame: {self.bbox!r}")
        if x + w > 1.0001 or y + h > 1.0001:
            raise FrameShapeError(f"bbox runs off the frame: {self.bbox!r}")
        if not (0.0 <= self.confidence <= 1.0):
            raise FrameShapeError(f"confidence outside 0..1: {self.confidence!r}")
        if not self.source:
            raise FrameShapeError("a detection must say which detector produced it")


@dataclass(frozen=True)
class DetectionFrame:
    camera_id: str
    capture_timestamp_ns: int | None
    inference_timestamp_ns: int | None
    detections: tuple[Detection, ...]
    source_width: int
    source_height: int
    inference_source: str
    frame_sequence: int | None = None

    def __post_init__(self) -> None:
        if not self.camera_id:
            raise FrameShapeError("a detection frame with no camera_id cannot be attributed")
        if self.source_width <= 0 or self.source_height <= 0:
            raise FrameShapeError("source size must be positive; normalised boxes need a frame")
        if isinstance(self.detections, list):
            object.__setattr__(self, "detections", tuple(self.detections))

    def latency_ms(self) -> float | None:
        """Scene -> usable-detection latency, or ``None`` when it cannot be known.

        Returns ``None`` rather than 0.0 for a missing stamp: a zero here reads as "the pipeline
        is instantaneous" and would be believed.
        """
        if self.capture_timestamp_ns is None or self.inference_timestamp_ns is None:
            return None
        delta_ms = (self.inference_timestamp_ns - self.capture_timestamp_ns) / 1e6
        if delta_ms < 0:
            raise FrameShapeError(
                f"inference timestamp precedes exposure start by {-delta_ms:.3f} ms: the two "
                "clocks are not comparable (capture is CLOCK_MONOTONIC-domain SensorTimestamp)")
        return round(delta_ms, 3)

    def pixel_boxes(self) -> list[tuple[int, int, int, int]]:
        """Normalised boxes in source pixels -- what the gimbal's FOV maths wants."""
        w, h = self.source_width, self.source_height
        return [(round(x * w), round(y * h), round(bw * w), round(bh * h))
                for (x, y, bw, bh) in (d.bbox for d in self.detections)]

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "capture_timestamp_ns": self.capture_timestamp_ns,
            "inference_timestamp_ns": self.inference_timestamp_ns,
            "latency_ms": self.latency_ms(),
            "source_size": [self.source_width, self.source_height],
            "inference_source": self.inference_source,
            "frame_sequence": self.frame_sequence,
            "detections": [{"class_id": d.class_id, "label": d.label,
                            "confidence": round(d.confidence, 6),
                            "bbox": [round(v, 6) for v in d.bbox], "source": d.source}
                           for d in self.detections],
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "DetectionFrame":
        size = payload.get("source_size") or [0, 0]
        return cls(
            camera_id=payload["camera_id"],
            capture_timestamp_ns=payload.get("capture_timestamp_ns"),
            inference_timestamp_ns=payload.get("inference_timestamp_ns"),
            detections=tuple(Detection(class_id=d["class_id"], label=d["label"],
                                       confidence=d["confidence"], bbox=tuple(d["bbox"]),
                                       source=d["source"])
                             for d in payload.get("detections", ())),
            source_width=int(size[0]), source_height=int(size[1]),
            inference_source=payload.get("inference_source", ""),
            frame_sequence=payload.get("frame_sequence"),
        )


def empty_frame(camera_id: str, capture_timestamp_ns: int | None,
                inference_timestamp_ns: int | None, source_size: tuple[int, int],
                inference_source: str, detections: Iterable[Detection] = ()) -> DetectionFrame:
    """Name it so call sites do not spell ``detections=()`` and ``None`` five times."""
    return DetectionFrame(camera_id=camera_id, capture_timestamp_ns=capture_timestamp_ns,
                          inference_timestamp_ns=inference_timestamp_ns,
                          detections=tuple(detections), source_width=source_size[0],
                          source_height=source_size[1], inference_source=inference_source)
