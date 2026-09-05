"""§13 / §14 — turning model rows into trustworthy, normalized detections.

Every coordinate in this subsystem is normalized against the **visible stream** after the
inference-to-stream mapping, and this module is where that mapping happens. Two mapping
errors are worth naming because both are common and neither is visible in a live demo:

**Letterboxing.** A 640×640 model input fed from a 1920×1080 stream is not a resize, it is
a resize plus black bars. Undoing it requires the same uniform scale the model saw and the
symmetric padding, and skipping the padding step produces boxes that are correct at the
centre of the frame and drift toward the middle at the edges — a bug that looks like
detector noise on a bench and like a tracker that cannot hold a target on the turret.

**ROI.** When the model ran on a crop, the crop's top-left is not the stream's top-left.
Carrying the offset is the difference between "the target is at 0.4 of the picture" and
"the target is at 0.4 of the crop, which the overlay draws somewhere else entirely".

Validity is enforced, not assumed (§14: "Do not publish a number with a validity flag
omitted"). A box that lands partly off-frame is clamped — a person standing half out of
view genuinely has a box that ends at the frame edge — but a box that is wholly outside,
inverted, or born of a non-finite score is **counted and dropped**. The two look similar
in code and are opposite in meaning: one is a real object clipped by the sensor, the other
is a model output that does not describe an object at all.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

from ..config import AnchorConfig
from ..errors import ValidationError
from .anchor import compute_anchor
from .types import (BBox, Detection, DetectionCounters, DetectionSet, PointNorm,
                    is_finite)

BoxTuple = Tuple[float, float, float, float]
#: A box may not shrink below this fraction of its own frame before it is considered gone.
#: Anything smaller is a clamp artefact, not a detection of a person.
_MIN_EDGE = 1e-4


def letterbox_scale_pad(input_width: int, input_height: int, region_width: int,
                        region_height: int) -> Tuple[float, float, float]:
    """Uniform scale and symmetric padding of a letterboxed input for a scene region.

    ``scale`` is model-input pixels per scene pixel, so scene→input multiplies by ``scale``
    and gains ``pad``, while input→scene — what ``unmap_point`` does — subtracts the pad and
    divides. Getting that direction backwards scales every box by the inverse of the letter
    factor, which looks like a detector that cannot decide how big a person is.
    """
    if min(input_width, input_height, region_width, region_height) <= 0:
        raise ValidationError("letterbox geometry needs positive dimensions")
    scale = min(input_width / region_width, input_height / region_height)
    pad_x = (input_width - region_width * scale) / 2.0
    pad_y = (input_height - region_height * scale) / 2.0
    return scale, pad_x, pad_y


@dataclass
class InferenceGeometry:
    """The mapping from model-input coordinates to normalized stream coordinates."""

    input_width: int
    input_height: int
    stream_width: int
    stream_height: int
    preserve_aspect_ratio: bool = False
    roi: Optional[Tuple[int, int, int, int]] = None      # x, y, w, h in stream pixels
    bbox_normalized: bool = True
    bbox_order: str = "xy"

    def __post_init__(self) -> None:
        if min(self.input_width, self.input_height) <= 0:
            raise ValidationError(
                f"model input dimensions must be positive, got "
                f"{self.input_width}x{self.input_height}")
        if min(self.stream_width, self.stream_height) <= 0:
            raise ValidationError(
                f"stream dimensions must be positive, got "
                f"{self.stream_width}x{self.stream_height}")
        if self.roi is not None:
            if len(self.roi) != 4:
                raise ValidationError(f"roi must be (x, y, w, h), got {self.roi!r}")
            if self.roi[2] <= 0 or self.roi[3] <= 0:
                raise ValidationError(f"roi must have positive extent, got {self.roi!r}")

    # -- the region of the scene the model input actually covers ------------
    @property
    def region_size(self) -> Tuple[int, int]:
        if self.roi is None:
            return self.stream_width, self.stream_height
        return int(self.roi[2]), int(self.roi[3])

    @property
    def region_offset(self) -> Tuple[int, int]:
        if self.roi is None:
            return 0, 0
        return int(self.roi[0]), int(self.roi[1])

    # -- mapping ------------------------------------------------------------
    def input_to_region_px(self, x_px: float, y_px: float) -> Tuple[float, float]:
        """Model-input PIXELS → pixels within the scene region the input covers.

        Pixels in, always. Whether the model's rows carried fractions or pixels is answered
        once, in :func:`parse_row_box`; answering it here as well double-scales every box by
        the model's input size, which produces coordinates like ``x_min = 64.0`` in a field
        that is supposed to be a fraction of the picture — the impossible-value-in-a-named-
        field failure §14 was written about, arriving one layer earlier than the check.
        """
        region_w, region_h = self.region_size
        if self.preserve_aspect_ratio:
            scale, pad_x, pad_y = letterbox_scale_pad(self.input_width,
                                                      self.input_height,
                                                      region_w, region_h)
            return (x_px - pad_x) / scale, (y_px - pad_y) / scale
        return (x_px * region_w / self.input_width,
                y_px * region_h / self.input_height)

    def unmap_point(self, x_px: float, y_px: float) -> PointNorm:
        """Model-input pixels → normalized full-stream coordinates (§13)."""
        offset_x, offset_y = self.region_offset
        region_x, region_y = self.input_to_region_px(x_px, y_px)
        return PointNorm((region_x + offset_x) / self.stream_width,
                         (region_y + offset_y) / self.stream_height)

    def unmap_box(self, box: Sequence[float]) -> BBox:
        """A model-input box (four coordinates, per ``bbox_order``) → normalized stream."""
        x0, y0, x1, y1 = parse_row_box(
            box, self.bbox_order, input_width=self.input_width,
            input_height=self.input_height, normalized=self.bbox_normalized)
        near = self.unmap_point(x0, y0)
        far = self.unmap_point(x1, y1)
        return BBox(near.x, near.y, far.x, far.y)


def parse_row_box(values: Sequence[float], order: str, *, input_width: int,
                  input_height: int, normalized: bool) -> BoxTuple:
    """Four box coordinates in a model row → ``(x_min, y_min, x_max, y_max)`` in input px.

    ``normalized`` says whether the row carries fractions of the model input or pixels of
    it — §6's YOLO11n ``_pp`` baseline is the former, several older zoo exports the latter,
    and mixing the two up scales every box by 640 in one direction or 1/640 in the other.
    The order names which pair comes first, and an unknown name raises rather than being
    treated as ``"xy"``: guessing a coordinate convention is how an impossible value ends
    up in a field named radians (§14).
    """
    if values is None or len(values) < 4:
        raise ValidationError(f"box needs 4 coordinates, got {values!r}")
    a, b, c, d = (float(values[0]), float(values[1]),
                 float(values[2]), float(values[3]))
    for value in (a, b, c, d):
        if not is_finite(value):
            raise ValidationError(f"box coordinate is not finite: {values!r}")

    scale_x = float(input_width) if normalized else 1.0
    scale_y = float(input_height) if normalized else 1.0

    if order == "xy":
        x_min, y_min, x_max, y_max = a, b, c, d
    elif order == "yxyx":
        y_min, x_min, y_max, x_max = a, b, c, d
    elif order == "cxcywh":
        half_w, half_h = c / 2.0, d / 2.0
        x_min, y_min, x_max, y_max = a - half_w, b - half_h, a + half_w, b + half_h
    else:
        raise ValidationError(
            f"unknown bbox_order {order!r}; expected 'xy', 'yxyx' or 'cxcywh'. Refusing "
            f"to guess: the two plausible readings differ by an axis swap, and the "
            f"result would still be a plausible-looking box.")
    return (x_min * scale_x, y_min * scale_y, x_max * scale_x, y_max * scale_y)


def clamp_to_frame(bbox: BBox) -> Optional[BBox]:
    """Clip a box to the unit image, or return ``None`` when nothing of it is left."""
    x_min = max(0.0, min(bbox.x_min, 1.0))
    y_min = max(0.0, min(bbox.y_min, 1.0))
    x_max = max(0.0, min(bbox.x_max, 1.0))
    y_max = max(0.0, min(bbox.y_max, 1.0))
    if x_max - x_min <= _MIN_EDGE or y_max - y_min <= _MIN_EDGE:
        return None
    return BBox(x_min, y_min, x_max, y_max)


def _class_name(label_map: Any, class_id: int) -> Optional[str]:
    """Label lookup that refuses an id the profile's label table does not contain."""
    if label_map is None:
        return None
    if isinstance(label_map, Mapping):
        value = label_map.get(class_id)
        return None if value is None else str(value)
    try:
        if class_id < 0 or class_id >= len(label_map):
            return None
        return str(label_map[class_id])
    except TypeError:
        return None


def normalize_rows(rows: Sequence[Sequence[float]], *, geometry: InferenceGeometry,
                   model_id: str, model_generation: int, frame_sequence: int,
                   sensor_timestamp_ns: int, publish_timestamp_ns: int,
                   label_map: Any, score_index: int = 0, class_index: int = 1,
                   box_index: int = 2,
                   anchor_cfg: Optional[AnchorConfig] = None) -> DetectionSet:
    """Model output rows → a validated ``DetectionSet`` (§13, §14, §16.4).

    The row layout is the model's own — ``[score, class_id, x1, y1, x2, y2]`` unless the
    caller says otherwise — so this function is the single place where a per-model quirk
    stops leaking downstream. §9.2's compatibility oracle compares the output of this
    function against the upstream reference example's boxes, which is only meaningful
    because the conversion lives in one place.
    """
    anchor_cfg = anchor_cfg or AnchorConfig()
    counters = DetectionCounters(raw_outputs=len(rows))
    detections = []

    for index, row in enumerate(rows):
        try:
            if row is None or len(row) <= box_index + 3:
                raise ValidationError("row is too short")
            score = float(row[score_index])
            class_id = int(row[class_index])
            if not is_finite(score) or not 0.0 <= score <= 1.0:
                raise ValidationError(f"score out of range: {score!r}")
            class_name = _class_name(label_map, class_id)
            if class_name is None:
                raise ValidationError(f"class id {class_id} is not in the label map")
            bbox = clamp_to_frame(geometry.unmap_box(row[box_index:box_index + 4]))
            if bbox is None:
                raise ValidationError("box falls outside the visible stream")
        except (ValidationError, TypeError, ValueError, IndexError):
            # Counted, never clamped into validity, and never logged per row: §16.4's
            # counters are the record, and a per-row print at 17 Hz would bury the
            # pipeline's own diagnostics in its own noise.
            counters.malformed_rejected += 1
            continue

        anchor, source = compute_anchor(bbox, (), anchor_cfg)
        detections.append(Detection(
            detection_id_in_frame=index, class_id=class_id, class_name=class_name,
            detector_score=score, bbox=bbox, measured_anchor=anchor,
            anchor_source=source))

    # What the model's own NMS plus normalization left: the input to the host dedup stage.
    counters.post_model_nms = len(detections)
    out = DetectionSet(
        model_id=model_id, model_generation=model_generation,
        frame_sequence=frame_sequence, sensor_timestamp_ns=sensor_timestamp_ns,
        publish_timestamp_ns=publish_timestamp_ns,
        stream_width=geometry.stream_width, stream_height=geometry.stream_height,
        roi=geometry.roi, preserve_aspect_ratio=geometry.preserve_aspect_ratio,
        detections=detections, counters=counters)
    out.validate()      # §14: an adapter may not hand a broken set downstream
    return out
