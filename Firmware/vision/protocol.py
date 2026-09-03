"""Vision wire formats: TargetMeasurement (§6.2) and TrackSet (§9/§59/§60).

This is the wire format for the ``visiond -> controld`` latest-value IPC. The
C++ side mirrors it in ``control/src/tracking/target_measurement.hpp`` (same
field order, little-endian, no padding). Keep the two in lockstep; the test
``test_protocol`` round-trips the encoding and a C++ test decodes the same
bytes.

v3 (§59) replaces the single "latest target" message with a TrackSet: the browser,
the target selector and reacquisition all need every candidate, and controld — not
visiond — decides which one is selected. Both formats are served on the same
SOCK_SEQPACKET socket and distinguished BY LENGTH, because SEQPACKET preserves message
boundaries and adding a type byte would have changed the v1 message that is still in
the field. The rule is explicit and total: 58 bytes is a TargetMeasurement,
``TRACK_SET_SIZE`` bytes is a TrackSet, anything else is refused. controld has to be
updated before visiond, and not the other way round — an old controld will refuse
TrackSet datagrams and therefore see no target, which is the failure direction this
project prefers (a turret that stops tracking is confusing; a turret that keeps
tracking something the operator never selected is not).

The vision daemon NEVER touches CAN or the motor driver; this module only defines and
(de)serializes these messages.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

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


# §9 carries class_name alongside class_id, and §10's human labels ("Person #2") are
# built from it. Duplicated rather than derived from a model's label table because the
# label has to be stable across a detector swap: when the IMX500 RPK replaces the
# classical bridge detector, "Person #2" must not silently become "pedestrian #2" in
# the middle of an operator having chosen it.
_CLASS_NAMES = {CLASS_NONE: "none", CLASS_PERSON: "person", CLASS_CAR: "car"}


def class_name_for(class_id: int) -> str:
    return _CLASS_NAMES.get(class_id, f"class{class_id}")


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


# --- v3 TrackSet (§9, §59, §60) -------------------------------------------

# §60: "fixed maximum track count ... suggested max_visible_tracks = 32". The cap is a
# contract, not a hint: the message has a constant size so the receiving side never has
# to negotiate a length or allocate while someone else is on a deadline.
K_MAX_TRACKS = 32
_CLASS_NAME_LEN = 12

_TRACK_FORMAT = "QQHH12sB" + "f" * 10 + "HHH"
_TRACK_SIZE = struct.calcsize("<" + _TRACK_FORMAT)
assert _TRACK_SIZE == 79, f"unexpected per-track size {_TRACK_SIZE}"

# Header: one unsigned 64-bit counter plus two signed 64-bit stamps, then resolution
# and count. §9's camera SensorTimestamp stays mandatory: without it "how old is this
# observation?" is a guess, and §61 exists so that question has a measured answer.
# Q + q + q = the three 64-bit stamps, I + I = resolution, H = count. Written out
# because "one too many q" here costs 8 bytes of silent misalignment in every record
# after the header, on one side of the boundary only.
_HEADER_FORMAT = "QqqIIH"
_TS_FORMAT = "<" + _HEADER_FORMAT + _TRACK_FORMAT * K_MAX_TRACKS
TRACK_SET_SIZE = struct.calcsize(_TS_FORMAT)
assert TRACK_SET_SIZE == 34 + 79 * K_MAX_TRACKS, TRACK_SET_SIZE


@dataclass
class BBoxNorm:
    """§9 bbox_norm, normalized to [0,1] image fraction (§60).

    Normalized on the wire, not in pixels: the overlay may be running at a different
    resolution than the detector, and a conversion step somebody has to remember is a
    conversion step somebody forgets.
    """

    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 0.0
    y_max: float = 0.0

    @classmethod
    def from_px(cls, x_min: float, y_min: float, x_max: float, y_max: float,
                width: int, height: int) -> "BBoxNorm":
        w, h = float(max(width, 1)), float(max(height, 1))
        return cls(x_min / w, y_min / h, x_max / w, y_max / h)


@dataclass
class Track:
    """One §9 track record. Also used to carry a raw detection into TrackManager,
    which is why `state` and the counters are plain ints rather than derived."""

    track_uuid: Tuple[int, int] = (0, 0)
    display_index: int = 0
    class_id: int = CLASS_NONE
    class_name: str = ""
    state: int = 0  # vision.track_manager.TrackState; int on the wire

    detector_confidence: float = 0.0
    track_confidence: float = 0.0

    bbox: BBoxNorm = field(default_factory=BBoxNorm)
    anchor_x: float = 0.0  # §9 anchor_norm: the point tracking follows
    anchor_y: float = 0.0
    velocity_x_norm_s: float = 0.0
    velocity_y_norm_s: float = 0.0

    age_frames: int = 0
    visible_frames: int = 0
    missing_frames: int = 0


@dataclass
class TrackSet:
    """§9 TrackSet: one detector frame's worth of candidate tracks."""

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    publish_timestamp_ns: int = 0
    width: int = 0
    height: int = 0
    tracks: List[Track] = field(default_factory=list)

    def encode(self) -> bytes:
        """Constant-length encoding. Tracks past §60's cap are dropped, not packed
        into an oversized datagram: the receiver refuses oversized datagrams, so
        growing here would silently turn a busy scene into total blindness."""
        kept = self.tracks[:K_MAX_TRACKS]
        args: List = [
            self.frame_sequence,
            self.sensor_timestamp_ns,
            self.publish_timestamp_ns,
            self.width,
            self.height,
            len(kept),
        ]
        for t in kept:
            args.extend([
                t.track_uuid[0], t.track_uuid[1],
                t.display_index, t.class_id,
                t.class_name.encode("utf-8", "replace")[:_CLASS_NAME_LEN]
                .ljust(_CLASS_NAME_LEN, b"\0"),
                t.state & 0xFF,
                t.detector_confidence, t.track_confidence,
                t.bbox.x_min, t.bbox.y_min, t.bbox.x_max, t.bbox.y_max,
                t.anchor_x, t.anchor_y,
                t.velocity_x_norm_s, t.velocity_y_norm_s,
                t.age_frames, t.visible_frames, t.missing_frames,
            ])
        # Pad the unused tail so the length never varies with how busy the scene is.
        for _ in range(K_MAX_TRACKS - len(kept)):
            args.extend([0, 0, 0, 0, b"\0" * _CLASS_NAME_LEN, 0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0])
        return struct.pack(_TS_FORMAT, *args)

    @classmethod
    def decode(cls, data: bytes) -> "TrackSet":
        if len(data) != TRACK_SET_SIZE:
            raise ValueError(
                f"TrackSet must be {TRACK_SET_SIZE} bytes, got {len(data)}")
        vals = struct.unpack(_TS_FORMAT, data)
        header, rest = vals[:6], vals[6:]
        out = cls(
            frame_sequence=header[0],
            sensor_timestamp_ns=header[1],
            publish_timestamp_ns=header[2],
            width=header[3],
            height=header[4],
        )
        count = min(header[5], K_MAX_TRACKS)
        per = len(_TRACK_FORMAT) and 19  # fields per track record
        for i in range(count):
            f = rest[i * per:(i + 1) * per]
            out.tracks.append(cls._track_from_fields(f))
        return out

    @staticmethod
    def _track_from_fields(f: Tuple) -> Track:
        return Track(
            track_uuid=(f[0], f[1]),
            display_index=f[2],
            class_id=f[3],
            class_name=f[4].rstrip(b"\0").decode("utf-8", "replace"),
            state=f[5],
            detector_confidence=f[6],
            track_confidence=f[7],
            bbox=BBoxNorm(f[8], f[9], f[10], f[11]),
            anchor_x=f[12],
            anchor_y=f[13],
            velocity_x_norm_s=f[14],
            velocity_y_norm_s=f[15],
            age_frames=f[16],
            visible_frames=f[17],
            missing_frames=f[18],
        )
