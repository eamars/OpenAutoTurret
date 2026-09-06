"""The visiond → controld wire messages, byte-exact (§6.2, §59, §60).

controld binds ``/tmp/ota_vision.sock`` (SOCK_SEQPACKET) and reads fixed-size datagrams. It
accepts two, distinguished by length: the 58-byte v1 :class:`TargetMeasurement` (a single target
visiond chose) or the 2562-byte v3 ``TrackSet`` (a frame's tracks).

The live daemon defaults to :mod:`native_wire`, which wraps the candidate list and
authoritative selected observation in one datagram. These encoders remain for
explicit legacy compatibility and boundary probes. A native connection cannot
downgrade to either legacy authority.

The format must stay in lockstep with ``control/src/tracking/target_measurement.hpp`` (v1) and
``control/src/tracks/track_wire.hpp`` (v3), both of which assert the sizes at compile time. The
Python side is pinned to the same bytes by cross-language tests; keep ``_FORMAT``/``_TRACK_LAYOUT``
in lockstep with those headers.
"""
from __future__ import annotations

import socket
import struct
import math
from dataclasses import dataclass
from typing import Any, Optional, Sequence, Tuple


#: ``<QQbi`` + 7 floats + ``bQ`` = 8+8+1+4 + 28 + 1+8 = 58 bytes (little-endian, no padding).
_FORMAT = "<QQbi" + "f" * 7 + "bQ"
_SIZE = struct.calcsize(_FORMAT)

#: v3 TrackSet wire (§60): 34-byte header + 32 × 79-byte tracks = 2562 bytes.
_TRACKSET_HEADER = "<QQQIIH"                 # frame_seq u64, sensor i64, publish i64, w u32, h u32, count u16
_TRACK_LAYOUT = "<QQHH12sBff4f2f2fHHH"        # uuid_hi/lo, disp, class, name, state, confs, bbox, anchor, vel, counters
_TRACKSET_SIZE = struct.calcsize(_TRACKSET_HEADER) + 32 * struct.calcsize(_TRACK_LAYOUT)
_MAX_TRACKS = 32
_CLASS_NAME_LEN = 12
#: Wire state bytes (track_wire.hpp ::state_from_wire). 0=Tentative 1=Confirmed 2=Occluded 3=Lost.
_WIRE_STATE = {"TENTATIVE": 0, "CONFIRMED_VISIBLE": 1, "OCCLUDED": 2, "LOST_REACQUIRABLE": 3}
_WIRE_STATE_RETIRED = 99

#: The perception class index and controld's selectable class id are DIFFERENT enumerations.
#: The detector labels "person" as COCO index 0; controld's SelectionManager only selects
#: class id 1 (target_selection_manager.hpp ``allowed_classes{1,...}``, and control_loop's
#: ``kPreferredClassId = 1  // v1 preferred_class_id: 'person'``). Sending the raw COCO index 0
#: makes ``select_target`` refuse with "class id 0 is not selectable by configuration (§14)".
#: This is the one translation point between the perception class namespace and controld's; it
#: lives IN THE ENCODER so every published track and measurement agrees on the id controld accepts.
_WIRE_CLASS_ID = {"person": 1}


def _wire_state(state: Any) -> int:
    """The wire byte for a track state, or the RETIRED sentinel (which the encoder drops)."""
    name = getattr(state, "name", str(state))
    if str(name) == "RETIRED":
        return _WIRE_STATE_RETIRED
    return _WIRE_STATE.get(str(name), 0)


@dataclass
class TargetMeasurement:
    """The frame's selected target, as the controller follows it.

    ``bbox_*`` are normalized to the image (fractions). ``anchor_u_px``/``anchor_v_px`` are the
    aim point in *stream* pixels (usually the bbox torso centre). ``visual_track_id`` is the track
    identity the controller should treat as "the one being followed"; ``None`` means "not set"
    and is encoded as 0 (the controller then derives the identity from the box alone).
    """

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    valid: bool = False
    class_id: int = 0
    confidence: float = 0.0
    bbox_x_min_norm: float = 0.0
    bbox_y_min_norm: float = 0.0
    bbox_x_max_norm: float = 0.0
    bbox_y_max_norm: float = 0.0
    anchor_u_px: float = 0.0
    anchor_v_px: float = 0.0
    visual_track_id: Optional[int] = None

    def encode(self) -> bytes:
        """58 bytes in controld's wire layout. Raise on malformed input rather than send it."""
        for name in ("bbox_x_min_norm", "bbox_y_min_norm", "bbox_x_max_norm", "bbox_y_max_norm",
                     "confidence", "anchor_u_px", "anchor_v_px"):
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"{name} must be finite")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if self.valid:
            if not (0 <= self.bbox_x_min_norm < self.bbox_x_max_norm <= 1
                    and 0 <= self.bbox_y_min_norm < self.bbox_y_max_norm <= 1):
                raise ValueError("valid measurement requires a normalized, nonempty box")
            if self.sensor_timestamp_ns <= 0 or min(self.anchor_u_px, self.anchor_v_px) < 0:
                raise ValueError("valid measurement requires capture time and nonnegative pixels")
        track_id = int(self.visual_track_id or 0)
        if track_id < 0 or track_id > 0xFFFFFFFFFFFFFFFF:
            raise ValueError(f"visual_track_id out of range: {track_id}")
        return struct.pack(
            _FORMAT,
            int(self.frame_sequence) & 0xFFFFFFFFFFFFFFFF,
            int(self.sensor_timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
            1 if self.valid else 0,
            int(self.class_id),
            float(self.confidence),
            float(self.bbox_x_min_norm),
            float(self.bbox_y_min_norm),
            float(self.bbox_x_max_norm),
            float(self.bbox_y_max_norm),
            float(self.anchor_u_px),
            float(self.anchor_v_px),
            1 if self.visual_track_id is not None else 0,
            track_id,
        )

    @classmethod
    def decode(cls, data: bytes) -> "TargetMeasurement":
        if len(data) != _SIZE:
            raise ValueError(f"expected {_SIZE} bytes, got {len(data)}")
        (frame_sequence, sensor_timestamp_ns, valid, class_id, confidence, bx0, by0, bx1, by1,
         au, av, has_track_id, track_id) = struct.unpack(_FORMAT, data)
        return cls(frame_sequence=int(frame_sequence),
                   sensor_timestamp_ns=int(sensor_timestamp_ns),
                   valid=bool(valid), class_id=int(class_id), confidence=float(confidence),
                   bbox_x_min_norm=float(bx0), bbox_y_min_norm=float(by0),
                   bbox_x_max_norm=float(bx1), bbox_y_max_norm=float(by1),
                   anchor_u_px=float(au), anchor_v_px=float(av),
                   visual_track_id=int(track_id) if has_track_id else None)

    def to_dict(self) -> dict:
        return {"frame_sequence": self.frame_sequence, "sensor_timestamp_ns": self.sensor_timestamp_ns,
                "valid": self.valid, "class_id": self.class_id, "confidence": round(self.confidence, 4),
                "bbox_norm": [round(self.bbox_x_min_norm, 4), round(self.bbox_y_min_norm, 4),
                              round(self.bbox_x_max_norm, 4), round(self.bbox_y_max_norm, 4)],
                "anchor_px": [round(self.anchor_u_px, 1), round(self.anchor_v_px, 1)],
                "visual_track_id": self.visual_track_id}


class SocketPublisher:
    """Publish datagrams to controld's SOCK_SEQPACKET socket.

    One connection, reconnected on failure. The socket is controld's; this class is the
    client. A failed send is *reported, not retried into a spin*: if controld is down, the
    next message (or the reconnect on the following send) recovers, and a 200 Hz retry loop
    would just busy-wait on a dead peer.
    """

    def __init__(self, path: str = "/tmp/ota_vision.sock") -> None:
        self.path = path
        self._sock: Optional[socket.socket] = None
        self.sent = 0
        self.failures = 0
        self.last_error = ""

    def connect(self) -> None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        # Bound a slow/disconnected consumer; it must not stall the camera indefinitely.
        sock.settimeout(0.05)
        try:
            sock.connect(self.path)
        except OSError:
            sock.close()
            raise
        self._sock = sock

    def send(self, payload: bytes) -> bool:
        if self._sock is None:
            try:
                self.connect()
            except OSError as exc:
                self.failures += 1
                self.last_error = str(exc)
                return False
        try:
            self._sock.send(payload)
            self.sent += 1
            self.last_error = ""
            return True
        except OSError as exc:
            self.failures += 1
            self.last_error = str(exc)
            self.close()
            return False

    def stats(self) -> dict:
        return {"sent": self.sent, "failures": self.failures, "last_error": self.last_error}

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self) -> "SocketPublisher":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


def measurement_from_selection(selection: Any, *, frame_sequence: int,
                               sensor_timestamp_ns: int, stream_size: Tuple[int, int],
                               confidence: float = 0.0) -> TargetMeasurement:
    """Build the wire measurement from a ``SelectedTargetObservation`` (or a like-shaped dict).

    ``selection`` is the subsystem's own selected-target record. A ``target_state_name``/state of
    ``NO_TARGET`` (or ``measurement_valid`` false) yields a valid=False measurement the controller
    treats as "no target" (§38's explicit NO_TARGET), rather than the lie of a stale zero. This is
    the only place the subsystem's selection becomes a wire value, so the mapping stays in one
    place and both sides agree on what NO_TARGET looks like.

    ``bbox`` is already normalized; ``measured_anchor`` is a normalized aim point and is scaled to
    *stream* pixels here because the controller works in image pixels (§10.1), not fractions.
    """
    state_name = _field(selection, "target_state_name", "target_state") or ""
    if hasattr(state_name, "name"):                       # a TargetState enum, not a str
        state_name = state_name.name
    # Retaining an identity does not create a new camera measurement. In particular, a
    # missing-tensor frame must not stamp the old box with this frame's capture time.
    if (not _field(selection, "measurement_valid", default=False)
            or state_name not in ("CONFIRMED_VISIBLE", "CONFIRMED_TARGET", "OCCLUDED")
            or bool(_field(selection, "ambiguity", default=0.0))):
        return TargetMeasurement(frame_sequence=int(frame_sequence),
                                 sensor_timestamp_ns=int(sensor_timestamp_ns), valid=False)

    bbox = _bbox_value(selection)
    anchor = _anchor_px(selection, stream_size)
    track_id = _track_u64(selection)
    # Same perception-index -> controld-id translation as _pack_track (person = 1).
    class_name = str(_field(selection, "class_name", default="") or "").lower()
    class_id = _WIRE_CLASS_ID.get(class_name, int(_field(selection, "class_id", default=0) or 0))
    return TargetMeasurement(
        frame_sequence=int(frame_sequence), sensor_timestamp_ns=int(sensor_timestamp_ns),
        valid=True, class_id=int(class_id),
        confidence=float(confidence or _field(selection, "detector_score", "confidence",
                                    default=0.0) or 0.0),
        bbox_x_min_norm=float(bbox[0]), bbox_y_min_norm=float(bbox[1]),
        bbox_x_max_norm=float(bbox[2]), bbox_y_max_norm=float(bbox[3]),
        anchor_u_px=float(anchor[0]), anchor_v_px=float(anchor[1]),
        visual_track_id=track_id)


def encode_track_set(tracks: Sequence[Any], *, frame_sequence: int, sensor_timestamp_ns: int,
                     publish_timestamp_ns: int, width: int, height: int) -> Optional[bytes]:
    """Serialize a frame's tracks into controld's 2562-byte v3 ``TrackSet`` datagram (§60).

    This is the message that actually populates ``controld``'s ``SelectionManager``: with the
    tracks in hand, ``select_target <display_index>`` can pick the confirmed person and the turret
    leaves ``LOST_HOLD``. The v1 measurement alone never sets the selection, which is exactly why
    the motor never followed before.

    The wire struct is a FIXED 2562 bytes: a 34-byte header plus 32 ``TrackWire`` slots, never
    fewer. Unused slots are zeroed and ``count`` says how many are real — an oversized *variable*
    datagram is refused by controld and a crowded scene becomes blindness, so the length is sacred.
    RETIRED tracks are omitted. Exceeding the 32-track capacity is an explicit error.
    """
    real = [track for track in tracks
            if _wire_state(getattr(track, "state", None)) != _WIRE_STATE_RETIRED]
    if len(real) > _MAX_TRACKS:
        raise ValueError("TrackSet exceeds the controller's 32-track capacity")
    head = struct.pack(_TRACKSET_HEADER,
                       int(frame_sequence) & 0xFFFFFFFFFFFFFFFF,
                       int(sensor_timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
                       int(publish_timestamp_ns) & 0xFFFFFFFFFFFFFFFF,
                       int(width) & 0xFFFFFFFF, int(height) & 0xFFFFFFFF,
                       len(real) & 0xFFFF)
    body = b"".join(_pack_track(track) for track in real)
    return head + body.ljust(_MAX_TRACKS * struct.calcsize(_TRACK_LAYOUT), b"\x00")


def _pack_track(track: Any) -> bytes:
    name = str(getattr(track, "class_name", "") or "")[: _CLASS_NAME_LEN - 1]
    tname = name.encode("utf-8", "replace").ljust(_CLASS_NAME_LEN, b"\x00")
    uuid_text = str(getattr(track, "track_uuid", "") or "").replace("-", "")
    hi = int(uuid_text[:16], 16) if len(uuid_text) >= 16 else 0
    lo = int(uuid_text[16:32], 16) if len(uuid_text) >= 32 else 0
    wire_state = _wire_state(track.state)
    # v3 has no measurement-valid or ambiguity bits. Withhold visibility so the
    # legacy controller cannot consume an ambiguous or retained box as fresh data.
    if wire_state == 1 and (not getattr(track, "measurement_valid", False)
                            or getattr(track, "ambiguous", False)):
        wire_state = 2
    bbox = getattr(track, "bbox", None)
    anchor = getattr(track, "anchor", None)
    # Translate the perception class index to controld's selectable class id (person = 1) so
    # `select_target` is accepted; see _WIRE_CLASS_ID. Default keeps the raw index for any
    # class the encoder does not know, so an unknown class fails loudly on controld's side
    # rather than silently becoming "person".
    class_id = _WIRE_CLASS_ID.get(str(getattr(track, "class_name", "") or "").lower(),
                                  int(getattr(track, "class_id", 0) or 0))
    display_index = int(getattr(track, "display_index", 0) or 0)
    if not 0 <= display_index <= 0xFFFF:
        raise ValueError("display index exceeds legacy wire capacity; refusing label reuse")
    return struct.pack(
        _TRACK_LAYOUT,
        hi & 0xFFFFFFFFFFFFFFFF, lo & 0xFFFFFFFFFFFFFFFF,
        display_index,
        class_id & 0xFFFF,
        tname, wire_state,
        float(getattr(track, "detector_score", 0.0) or 0.0),
        float(getattr(track, "identity_confidence", 0.0) or 0.0),
        float(getattr(bbox, "x_min", 0.0)), float(getattr(bbox, "y_min", 0.0)),
        float(getattr(bbox, "x_max", 0.0)), float(getattr(bbox, "y_max", 0.0)),
        float(getattr(anchor, "x", 0.5)), float(getattr(anchor, "y", 0.5)),
        float(getattr(track, "velocity_x", 0.0) or 0.0),
        float(getattr(track, "velocity_y", 0.0) or 0.0),
        min(0xFFFF, int(getattr(track, "observations", 0) or 0)
            + int(getattr(track, "miss_observations", 0) or 0)),
        min(0xFFFF, int(getattr(track, "observations", 0) or 0)),
        min(0xFFFF, int(getattr(track, "miss_observations", 0) or 0)))


def _bbox_value(selection: Any) -> Tuple[float, float, float, float]:
    """(x_min, y_min, x_max, y_max) in normalized image fractions."""
    bbox = _field(selection, "bbox_norm") or _field(selection, "bbox")
    if bbox is None:
        return (0.0, 0.0, 0.0, 0.0)
    # An object (a BBox) rather than a sequence.
    if not isinstance(bbox, (tuple, list)):
        return (float(getattr(bbox, "x_min", 0.0)), float(getattr(bbox, "y_min", 0.0)),
                float(getattr(bbox, "x_max", 0.0)), float(getattr(bbox, "y_max", 0.0)))
    return (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))


def _anchor_px(selection: Any, stream_size: Tuple[int, int]) -> Tuple[float, float]:
    """The aim point, in stream pixels. Accepts normalized anchor + scales it."""
    anchor = _field(selection, "anchor_px")
    if anchor is not None and isinstance(anchor, (tuple, list)) and len(anchor) == 2:
        return (float(anchor[0]), float(anchor[1]))
    measured = _field(selection, "measured_anchor")
    if measured is not None:
        # PointNorm: a normalized aim point in [0,1].
        ax = float(getattr(measured, "x", _index(measured, 0, 0.5)))
        ay = float(getattr(measured, "y", _index(measured, 1, 0.5)))
        return (ax * float(stream_size[0]), ay * float(stream_size[1]))
    centre = _field(selection, "anchor")
    if centre is not None and isinstance(centre, (tuple, list)) and len(centre) == 2:
        return (float(centre[0]) * float(stream_size[0]),
                float(centre[1]) * float(stream_size[1]))
    bbox = _bbox_value(selection)
    return ((bbox[0] + bbox[2]) / 2.0 * float(stream_size[0]),
            (bbox[1] + bbox[3]) / 2.0 * float(stream_size[1]))


def _index(value: Any, position: int, default: float) -> float:
    if isinstance(value, (tuple, list)) and len(value) > position:
        return float(value[position])
    return default


def _track_u64(selection: Any) -> Optional[int]:
    """A stable u64 identity for the controller, from the subsystem's track UUID."""
    track_id = _field(selection, "track_uuid", "identity_uuid", "uuid", "visual_track_id",
                      default=None)
    if track_id is None:
        return None
    if isinstance(track_id, int):
        return track_id & 0xFFFFFFFFFFFFFFFF
    if isinstance(track_id, bytes):
        return int.from_bytes(track_id[:8], "little")
    text = str(track_id).replace("-", "").strip()
    if len(text) >= 16:
        try:
            # The controller's TrackSet adapter uses uuid.lo for the legacy estimator.
            return int(text[-16:], 16) & 0xFFFFFFFFFFFFFFFF
        except ValueError:
            pass
    # Not a hex UUID; fold it to a stable u64 so identical UUIDs map to the same id.
    import hashlib
    return int.from_bytes(hashlib.blake2b(str(track_id).encode(), digest_size=8).digest(),
                          "little")


def _field(obj: Any, *names: str, default: Any = None) -> Any:
    """The first present attribute/key, or the explicit default.

    ``default`` is ``None`` by default: a call that does not know whether the field exists must
    be able to distinguish "absent" from "present and zero", because the optional fields of a
    selection (``measured_anchor``, ``bbox``, the track id) are exactly what ``is not None``
    branches on. A default of ``0.0`` here made an absent ``measured_anchor`` look present and
    silently aimed at the frame centre.
    """
    for name in names:
        value = getattr(obj, name, None)
        if value is None and isinstance(obj, dict):
            value = obj.get(name)
        if value is not None:
            return value
    return default


def _bbox_centre(bbox: Any, stream_size: Tuple[int, int]) -> Tuple[float, float]:
    if len(bbox) >= 4:
        cx = (float(bbox[0]) + float(bbox[2])) / 2.0
        cy = (float(bbox[1]) + float(bbox[3])) / 2.0
        width, height = stream_size
        return (cx * width, cy * height)
    return (0.0, 0.0)
