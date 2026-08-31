"""TargetMeasurement schema + fixed binary protocol (architecture §6.2).

This is the wire format for the ``visiond -> controld`` latest-value IPC. The
C++ side mirrors it in ``control/src/tracking/target_measurement.hpp`` (same
field order, little-endian, no padding). Keep the two in lockstep; the test
``test_protocol`` round-trips the encoding and a C++ test decodes the same
bytes.

The vision daemon NEVER touches CAN or the motor driver; this module only
defines and (de)serializes the measurement message.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional

# Fixed little-endian layout, no alignment padding. Field order (13 fields):
#   Q frame_sequence, Q sensor_timestamp_ns, b valid, i class_id,
#   f confidence, f bbox_x_min_norm, f bbox_y_min_norm, f bbox_x_max_norm,
#   f bbox_y_max_norm, f anchor_u_px, f anchor_v_px, b has_track_id,
#   Q visual_track_id
# The optional visual_track_id is always present in the wire format (0 when
# absent) so the message length is constant (58 bytes) and the C++ side can
# parse it without length negotiation.
_FORMAT = "<QQbi" + "f" * 7 + "bQ"
_SIZE = struct.calcsize(_FORMAT)
assert _SIZE == 58, f"unexpected protocol size {_SIZE}"

# Class ids (MobileNet COCO subset; the v1 tracker only follows 'person').
CLASS_NONE = 0
CLASS_PERSON = 1
CLASS_CAR = 2


@dataclass
class TargetMeasurement:
    """One selected-target measurement (architecture §6.2).

    Bounding-box fields are normalized to [0,1] (image fraction). The anchor is
    the tracking point in pixels (default: bbox centre, §10.1).
    ``sensor_timestamp_ns`` is the capture time of the frame the detection came
    from (mandatory, §6.2) — used for camera/motor time alignment (§11).
    """

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    valid: bool = False
    class_id: int = CLASS_NONE
    confidence: float = 0.0
    bbox_x_min_norm: float = 0.0
    bbox_y_min_norm: float = 0.0
    bbox_x_max_norm: float = 0.0
    bbox_y_max_norm: float = 0.0
    anchor_u_px: float = 0.0
    anchor_v_px: float = 0.0
    visual_track_id: Optional[int] = None  # None => not set

    def encode(self) -> bytes:
        return struct.pack(
            _FORMAT,
            self.frame_sequence,
            self.sensor_timestamp_ns,
            1 if self.valid else 0,
            self.class_id,
            self.confidence,
            self.bbox_x_min_norm,
            self.bbox_y_min_norm,
            self.bbox_x_max_norm,
            self.bbox_y_max_norm,
            self.anchor_u_px,
            self.anchor_v_px,
            1 if self.visual_track_id is not None else 0,
            self.visual_track_id if self.visual_track_id is not None else 0,
        )

    @classmethod
    def decode(cls, data: bytes) -> "TargetMeasurement":
        if len(data) != _SIZE:
            raise ValueError(f"TargetMeasurement must be {_SIZE} bytes, got {len(data)}")
        (
            frame_sequence,
            sensor_timestamp_ns,
            valid,
            class_id,
            confidence,
            bx_min,
            by_min,
            bx_max,
            by_max,
            anchor_u,
            anchor_v,
            has_track,
            visual_track_id,
        ) = struct.unpack(_FORMAT, data)
        return cls(
            frame_sequence=frame_sequence,
            sensor_timestamp_ns=sensor_timestamp_ns,
            valid=bool(valid),
            class_id=class_id,
            confidence=confidence,
            bbox_x_min_norm=bx_min,
            bbox_y_min_norm=by_min,
            bbox_x_max_norm=bx_max,
            bbox_y_max_norm=by_max,
            anchor_u_px=anchor_u,
            anchor_v_px=anchor_v,
            visual_track_id=visual_track_id if has_track else None,
        )


# An empty (valid=False) measurement, used as the "no target" publish.
def empty_measurement() -> TargetMeasurement:
    return TargetMeasurement()


PROTOCOL_SIZE = _SIZE
