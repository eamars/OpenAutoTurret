"""The detection output contract (§13) and the geometry every consumer shares.

One structure leaves every model adapter, whatever the network: a ``DetectionSet`` whose
coordinates are **normalized against the visible stream** after the inference-to-stream
mapping, with its source dimensions, coordinate convention and validity present (§13's
closing rule). That single shape is what lets §7's bake-off swap YOLO11n for NanoDet
without a line changing in ``TrackManager``, and what makes §9.2's oracle meaningful —
the comparison is between *detections*, not between tensor layouts.

The geometry helpers live here, next to the type they measure, rather than in a
"detection/nms.py" that each consumer would otherwise re-implement:

* ``iou`` (§16.1) and ``containment`` (§16.2) are used by the deduplicator, the
  DuplicateTrackResolver (§25) and the association cost (§21). Three implementations of
  IoU is three chances for the deduplicator and the tracker to disagree about whether two
  boxes are the same person — and the disagreement shows up as an identity split, which
  is the exact defect this document is about.
* Every accessor validates range (§14). A box with ``x_max <= x_min`` is not a noisy
  measurement, it is a different quantity wearing the field's name; the previous system
  published an impossible value in a field named radians, so "the reader will notice" is
  a claim this codebase has already tested and rejected.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from ..errors import ValidationError

_EPS = 1e-9


def is_finite(value: float) -> bool:
    """True for a real, finite number. ``NaN`` fails, and so does ``Decimal``-like junk."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


class AnchorSource(str, Enum):
    """Where the aim anchor came from (§35).

    Published rather than inferred: an operator looking at a jittery target needs to know
    whether the anchor is a shoulder midpoint or the fallback centre of a half-visible
    box, because those have different expected jitter and the control subsystem may want
    to treat them differently.
    """

    BBOX_TORSO = "bbox_torso"
    POSE_SHOULDERS = "pose_shoulders"
    POSE_TORSO = "pose_torso"
    BBOX_CENTER_FALLBACK = "bbox_center_fallback"


@dataclass
class PointNorm:
    """A point in normalized image space, [0,1] on both axes (§13, §14)."""

    x: float = 0.0
    y: float = 0.0

    def is_valid(self) -> bool:
        return (is_finite(self.x) and is_finite(self.y)
                and -_EPS <= self.x <= 1.0 + _EPS
                and -_EPS <= self.y <= 1.0 + _EPS)

    def validate(self, what: str = "anchor") -> "PointNorm":
        if not self.is_valid():
            raise ValidationError(
                f"{what} must be finite and inside [0,1], got ({self.x!r}, {self.y!r})")
        return self

    def distance_to(self, other: "PointNorm") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_dict(self) -> Dict[str, float]:
        return {"x": float(self.x), "y": float(self.y)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PointNorm":
        return cls(float(data["x"]), float(data["y"]))


@dataclass
class BBox:
    """A normalized bounding box, x-right / y-down, in [0,1] (§13, §14, §60).

    Normalized on the wire and in memory: the overlay, the detector and the recorder do
    not necessarily agree on a resolution, and a conversion step somebody has to remember
    is a conversion step somebody forgets.
    """

    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 0.0
    y_max: float = 0.0

    # -- construction -------------------------------------------------------
    @classmethod
    def from_px(cls, x_min: float, y_min: float, x_max: float, y_max: float,
                width: int, height: int) -> "BBox":
        """Pixels -> normalized, against the *visible stream* size (§13).

        Non-positive dimensions raise rather than divide: an unnormalized box that
        silently became 0.0004 wide would survive every downstream range check.
        """
        if width <= 0 or height <= 0:
            raise ValidationError(
                f"source dimensions must be positive, got {width}x{height}")
        return cls(x_min / float(width), y_min / float(height),
                   x_max / float(width), y_max / float(height))

    # -- validity (§14) -----------------------------------------------------
    def is_well_formed(self) -> bool:
        return (all(is_finite(v) for v in (self.x_min, self.y_min, self.x_max, self.y_max))
                and -_EPS <= self.x_min < self.x_max <= 1.0 + _EPS
                and -_EPS <= self.y_min < self.y_max <= 1.0 + _EPS)

    def validate(self, what: str = "bbox") -> "BBox":
        if not all(is_finite(v) for v in (self.x_min, self.y_min, self.x_max, self.y_max)):
            raise ValidationError(f"{what} has a non-finite coordinate: {self!r}")
        if not (-_EPS <= self.x_min <= 1.0 + _EPS and -_EPS <= self.y_min <= 1.0 + _EPS
                and -_EPS <= self.x_max <= 1.0 + _EPS
                and -_EPS <= self.y_max <= 1.0 + _EPS):
            raise ValidationError(f"{what} lies outside the unit image: {self!r}")
        if self.x_max <= self.x_min:
            raise ValidationError(f"{what} has x_max <= x_min: {self!r}")
        if self.y_max <= self.y_min:
            raise ValidationError(f"{what} has y_max <= y_min: {self!r}")
        return self

    # -- geometry -----------------------------------------------------------
    @property
    def width(self) -> float:
        return max(0.0, self.x_max - self.x_min)

    @property
    def height(self) -> float:
        return max(0.0, self.y_max - self.y_min)

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center(self) -> PointNorm:
        return PointNorm((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    @property
    def aspect(self) -> float:
        """width / height. A person box is taller than wide; aspect change is a §21 term."""
        return self.width / self.height if self.height > _EPS else float("inf")

    def intersection(self, other: "BBox") -> float:
        ix = min(self.x_max, other.x_max) - max(self.x_min, other.x_min)
        iy = min(self.y_max, other.y_max) - max(self.y_min, other.y_min)
        return 0.0 if ix <= 0.0 or iy <= 0.0 else ix * iy

    def iou(self, other: "BBox") -> float:
        """Intersection over union (§16.1, §21)."""
        inter = self.intersection(other)
        if inter <= 0.0:
            return 0.0
        union = self.area + other.area - inter
        return inter / union if union > _EPS else 0.0

    def containment(self, other: "BBox") -> float:
        """intersection / min(area) (§16.2).

        IoU alone cannot see a nested pair: a tight box inside a loose one can score a
        moderate IoU while obviously being one person. Containment answers the question
        §16.2 actually asks — how much of the *smaller* box is explained by the larger.
        """
        inter = self.intersection(other)
        if inter <= 0.0:
            return 0.0
        smaller = min(self.area, other.area)
        return inter / smaller if smaller > _EPS else 0.0

    def scale_change(self, other: "BBox") -> float:
        """Symmetric log-free scale ratio, 0.0 when the areas match (§21 cost term)."""
        a, b = self.area, other.area
        if a <= _EPS or b <= _EPS:
            return 1.0
        return abs(a - b) / max(a, b)

    def aspect_change(self, other: "BBox") -> float:
        """Relative aspect change as ``max/min - 1``: 0 identical, 1.0 means doubled.

        Ratio-based rather than the symmetric fraction ``scale_change`` uses, because this
        number feeds §21's hard gate as well as the cost. ``|a-b|/max(a,b)`` can never
        exceed 1, so a gate written as ``> 1.0`` against it is dead code that looks like a
        guard — the worst kind, because it survives review. A person's box does not go
        from portrait to landscape between two frames; a degenerate box is ``inf`` and is
        therefore gated out rather than treated as a perfect match.
        """
        a, b = self.aspect, other.aspect
        if not (is_finite(a) and is_finite(b)) or a <= _EPS or b <= _EPS:
            return float("inf")
        high, low = (a, b) if a >= b else (b, a)
        return high / low - 1.0

    def anchor_at_height(self, fraction: float) -> PointNorm:
        """Point at the box's horizontal centre, ``fraction`` down from its top (§35).

        The aim anchor is deliberately not the box centre: the centre of a full-body box
        is around the hips/thighs, and the initial candidate of §35 (0.42–0.48) biases
        toward the upper torso, which is both a more stable point and a more meaningful
        one for a aiming device.
        """
        return PointNorm(self.center.x, self.y_min + fraction * self.height)

    def expanded(self, margin: float) -> "BBox":
        """Grown by ``margin`` of its own size on each axis, clamped to the unit image."""
        mx, my = self.width * margin, self.height * margin
        return BBox(max(0.0, self.x_min - mx), max(0.0, self.y_min - my),
                    min(1.0, self.x_max + mx), min(1.0, self.y_max + my))

    def to_dict(self) -> Dict[str, float]:
        return {"x_min": float(self.x_min), "y_min": float(self.y_min),
                "x_max": float(self.x_max), "y_max": float(self.y_max)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "BBox":
        return cls(float(data["x_min"]), float(data["y_min"]),
                   float(data["x_max"]), float(data["y_max"]))

    def __str__(self) -> str:  # compact for logs
        return (f"[{self.x_min:.3f},{self.y_min:.3f},"
                f"{self.x_max:.3f},{self.y_max:.3f}]")


@dataclass
class Keypoint:
    """One pose keypoint in normalized image space (§13, §12)."""

    x: float = 0.0
    y: float = 0.0
    score: float = 0.0

    def is_valid(self) -> bool:
        return (is_finite(self.x) and is_finite(self.y) and is_finite(self.score)
                and -_EPS <= self.x <= 1.0 + _EPS
                and -_EPS <= self.y <= 1.0 + _EPS
                and -_EPS <= self.score <= 1.0 + _EPS)

    def to_dict(self) -> Dict[str, float]:
        return {"x": float(self.x), "y": float(self.y), "score": float(self.score)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Keypoint":
        return cls(float(data["x"]), float(data["y"]), float(data.get("score", 0.0)))


@dataclass
class Detection:
    """One §13 detection: a class, a detector score, a box and an aim anchor.

    ``detector_score`` is exactly the model's number and nothing else (§37). Track
    confidence, association quality and ambiguity are *separate* fields on the track —
    folding them in here is the mistake §3.5 records as already having been made.
    """

    detection_id_in_frame: int = 0
    class_id: int = 0
    class_name: str = ""
    detector_score: float = 0.0
    bbox: BBox = field(default_factory=BBox)
    measured_anchor: PointNorm = field(default_factory=lambda: PointNorm(0.5, 0.5))
    anchor_source: AnchorSource = AnchorSource.BBOX_CENTER_FALLBACK
    keypoints: Tuple[Keypoint, ...] = ()
    pose_score: Optional[float] = None

    # -- validity -----------------------------------------------------------
    def is_valid(self) -> bool:
        return (is_finite(self.detector_score) and 0.0 <= self.detector_score <= 1.0
                and self.bbox.is_well_formed() and self.measured_anchor.is_valid())

    def validate(self, what: str = "detection") -> "Detection":
        if not is_finite(self.detector_score) or not (0.0 <= self.detector_score <= 1.0):
            raise ValidationError(
                f"{what} detector_score must be finite and in [0,1], got "
                f"{self.detector_score!r}")
        self.bbox.validate(f"{what}.bbox")
        self.measured_anchor.validate(f"{what}.measured_anchor")
        return self

    # -- helpers ------------------------------------------------------------
    @property
    def has_pose(self) -> bool:
        return bool(self.keypoints)

    def with_anchor(self, anchor: PointNorm,
                    source: AnchorSource) -> "Detection":
        """A copy with the aim anchor replaced (§35: identity box and anchor differ)."""
        return replace(self, measured_anchor=anchor, anchor_source=source)

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "detection_id_in_frame": int(self.detection_id_in_frame),
            "class_id": int(self.class_id),
            "class_name": str(self.class_name),
            "detector_score": float(self.detector_score),
            "bbox": self.bbox.to_dict(),
            "measured_anchor": self.measured_anchor.to_dict(),
            "anchor_source": self.anchor_source.value,
        }
        if self.keypoints:
            out["keypoints"] = [k.to_dict() for k in self.keypoints]
        if self.pose_score is not None:
            out["pose_score"] = float(self.pose_score)
        return out

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Detection":
        source = data.get("anchor_source", AnchorSource.BBOX_CENTER_FALLBACK.value)
        try:
            anchor_source = AnchorSource(source)
        except ValueError:
            anchor_source = AnchorSource.BBOX_CENTER_FALLBACK
        return cls(
            detection_id_in_frame=int(data.get("detection_id_in_frame", 0)),
            class_id=int(data.get("class_id", 0)),
            class_name=str(data.get("class_name", "")),
            detector_score=float(data.get("detector_score", 0.0)),
            bbox=BBox.from_dict(data.get("bbox") or {}),
            measured_anchor=(PointNorm.from_dict(data["measured_anchor"])
                             if data.get("measured_anchor")
                             else PointNorm(0.5, 0.5)),
            anchor_source=anchor_source,
            keypoints=tuple(Keypoint.from_dict(k) for k in data.get("keypoints") or ()),
            pose_score=(None if data.get("pose_score") is None
                        else float(data["pose_score"])),
        )


@dataclass
class DetectionCounters:
    """§16.4's counters: no silent removal, anywhere in the detection path.

    Every stage that can drop a box records that it did, so "why did the tracker only
    see one of the two boxes I can see?" is a lookup and not an archaeology session.
    """

    raw_outputs: int = 0                    # tensors/rows the model produced
    malformed_rejected: int = 0             # §14: refused for invalid geometry
    class_filtered: int = 0                 # §15: not a permitted class
    post_model_nms: int = 0                 # surviving on-sensor/NMS output
    host_duplicates_suppressed: int = 0     # §16.1 host class-aware NMS
    containment_suppressed: int = 0         # §16.2 nested-box suppression
    pose_duplicates_suppressed: int = 0     # §16.3 keypoint-similarity suppression
    capacity_dropped: int = 0               # §26's detection-side drop counter
    degenerate_rejected: int = 0           # §36: a box too small to be a measurement

    def total_suppressed(self) -> int:
        return (self.host_duplicates_suppressed + self.containment_suppressed
                + self.pose_duplicates_suppressed)

    def to_dict(self) -> Dict[str, int]:
        return {
            "raw_outputs": int(self.raw_outputs),
            "malformed_rejected": int(self.malformed_rejected),
            "class_filtered": int(self.class_filtered),
            "post_model_nms": int(self.post_model_nms),
            "host_duplicates_suppressed": int(self.host_duplicates_suppressed),
            "containment_suppressed": int(self.containment_suppressed),
            "pose_duplicates_suppressed": int(self.pose_duplicates_suppressed),
            "capacity_dropped": int(self.capacity_dropped),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DetectionCounters":
        return cls(**{k: int((data or {}).get(k, 0)) for k in (
            "raw_outputs", "malformed_rejected", "class_filtered", "post_model_nms",
            "host_duplicates_suppressed", "containment_suppressed",
            "pose_duplicates_suppressed", "capacity_dropped")})


@dataclass
class DetectionSet:
    """§13 DetectionSet: one inference result, fully described, ready to be recorded.

    ``model_generation`` is bumped whenever a model is (re)loaded into the sensor. It is
    carried on every set because identities formed under different models are not
    comparable: after a reload the appearance statistics, the score semantics and — in
    the bake-off (§7) — the class map all change, and a track that silently survived that
    would be an identity claim nobody can audit.

    ``sensor_timestamp_ns`` is mandatory. A detection without a capture time cannot be
    associated across a moving camera, cannot be aged by §19's time-based lifecycle, and
    cannot be interpolated against the motor history — so it is published as an explicit
    zero plus a counted miss, never as a guessed "now".
    """

    model_id: str = ""
    model_generation: int = 0
    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    publish_timestamp_ns: int = 0
    stream_width: int = 0
    stream_height: int = 0
    roi: Optional[Tuple[int, int, int, int]] = None   # x, y, w, h in stream pixels
    preserve_aspect_ratio: bool = False
    detections: List[Detection] = field(default_factory=list)
    counters: DetectionCounters = field(default_factory=DetectionCounters)

    # -- validity -----------------------------------------------------------
    def is_valid(self) -> bool:
        if self.stream_width <= 0 or self.stream_height <= 0:
            return False
        if self.sensor_timestamp_ns <= 0:
            return False
        return all(d.is_valid() for d in self.detections)

    def validate(self) -> "DetectionSet":
        """Fail fast before publication (§14). Raises ``ValidationError``."""
        if self.stream_width <= 0 or self.stream_height <= 0:
            raise ValidationError(
                f"stream dimensions must be positive, got "
                f"{self.stream_width}x{self.stream_height}")
        if self.sensor_timestamp_ns <= 0:
            raise ValidationError(
                "sensor_timestamp_ns is mandatory (§13/§6.2); refusing to publish a "
                "detection whose capture time is unknown")
        for det in self.detections:
            det.validate(f"detection {det.detection_id_in_frame}")
        return self

    # -- access -------------------------------------------------------------
    def __len__(self) -> int:
        return len(self.detections)

    def __bool__(self) -> bool:
        """Always true. A frame with nothing in it is still a frame that was measured.

        ``set or empty_set()`` would be indistinguishable from "the model said nothing", and
        §16.4's counters ride on the object, not on its detections.
        """
        return True

    @property
    def empty(self) -> bool:
        return not self.detections

    def sorted_by_score(self) -> List[Detection]:
        """Score-descending, ties by id: the order NMS and §16.1 specify, deterministically."""
        return sorted(self.detections,
                      key=lambda d: (-d.detector_score, d.detection_id_in_frame))

    def with_detections(self, detections: Sequence[Detection]) -> "DetectionSet":
        """A copy carrying a different detection list, with its OWN counters.

        The counters are copied rather than shared. Every stage downstream is going to add
        to them, and a shallow copy would have the deduplicator editing the counters of the
        set the recorder just wrote out — two consumers of one frame disagreeing about how
        many boxes existed, which is exactly the audit trail §16.4 exists to keep.
        """
        return replace(self, detections=list(detections), counters=replace(self.counters))

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "model_id": self.model_id,
            "model_generation": int(self.model_generation),
            "frame_sequence": int(self.frame_sequence),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "publish_timestamp_ns": int(self.publish_timestamp_ns),
            "stream_width": int(self.stream_width),
            "stream_height": int(self.stream_height),
            "preserve_aspect_ratio": bool(self.preserve_aspect_ratio),
            "detections": [d.to_dict() for d in self.detections],
            "counters": self.counters.to_dict(),
        }
        if self.roi is not None:
            out["roi"] = [int(v) for v in self.roi]
        return out

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DetectionSet":
        roi = data.get("roi")
        return cls(
            model_id=str(data.get("model_id", "")),
            model_generation=int(data.get("model_generation", 0)),
            frame_sequence=int(data.get("frame_sequence", 0)),
            sensor_timestamp_ns=int(data.get("sensor_timestamp_ns", 0)),
            publish_timestamp_ns=int(data.get("publish_timestamp_ns", 0)),
            stream_width=int(data.get("stream_width", 0)),
            stream_height=int(data.get("stream_height", 0)),
            roi=tuple(int(v) for v in roi) if roi else None,
            preserve_aspect_ratio=bool(data.get("preserve_aspect_ratio", False)),
            detections=[Detection.from_dict(d) for d in data.get("detections") or ()],
            counters=DetectionCounters.from_dict(data.get("counters") or {}),
        )


def iter_valid(detections: Iterable[Detection]) -> Tuple[List[Detection], int]:
    """Split into (valid, count_rejected) — the §14 filter every adapter must apply.

    Returning the count instead of logging is deliberate: §16.4 forbids silent removal,
    and a malformed box that merely printed a line would be invisible in a replay.
    """
    kept: List[Detection] = []
    rejected = 0
    for det in detections:
        if det.is_valid():
            kept.append(det)
        else:
            rejected += 1
    return kept, rejected


CLASS_PERSON = 0            # COCO's person label id
PERSON_CLASS_NAME = "person"


def display_label_for_class(class_name: str) -> str:
    """The human prefix of §27's "Person #17".

    Derived from ``class_name`` rather than hard-coded per class so a profile that also
    permits ``dog`` produces "Dog #3" without a code change — but the *number* still comes
    from the session's monotonic allocator, which is the part §27 actually forbids
    recycling.
    """
    name = (class_name or "target").strip().replace("_", " ")
    return name[:1].upper() + name[1:] if name else "Target"
