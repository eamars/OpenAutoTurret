"""``SelectedTargetObservation`` — the controller-facing output boundary (§34).

This is the entire surface the control subsystem sees of perception, and its omissions are
the point: no motor ID, no CAN frame, no CyberGear term, no servo gain, no drive mode, and
**no motor lead** (§36, §54). The previous arrangement let prediction happen on both sides
of this line, which double-counts the same latency and produces an aim that consistently
overshoots in the direction of travel.

Three fields carry the safety semantics:

``measurement_valid``
    There is a measurement here that may be acted on. It is ``False`` for ``LOST``,
    for ``AMBIGUOUS`` (§32: "controller receives no valid new measurement") and for
    ``NO_TARGET``. The controller may still coast on its own estimator across
    ``OCCLUDED``, but it must never invent a measurement we did not publish.

``target_state``
    The perception lifecycle (§38). It is NOT the AutoTrack phase and must never be
    merged with it — §3.6's ``LOST_HOLD`` ambiguity came from exactly that translation.

``selection_generation``
    Bumped on every *effective* selection change (§30). A repeated selection of the same
    UUID does not bump it, which is what lets the controller tell "the operator picked
    again" from "the script re-ran" and stop resetting acquisition mid-dwell.

The four quality numbers stay separate (§37): a low ``detector_score`` with a high
``identity_confidence`` is a target the model is unsure about and the tracker is sure
about, and those two states want different behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict

from ..detection.types import AnchorSource, BBox, PointNorm, is_finite
from ..errors import ValidationError

PROTOCOL_VERSION = 1


class TargetState(IntEnum):
    """§34's ``target_state`` enumeration. Wire values; append, never renumber."""

    NO_TARGET = 0          # nothing selected at all (§31's NO_SELECTION)
    CONFIRMED_VISIBLE = 1  # measured this frame, confirmed identity
    OCCLUDED = 2           # identity held, no fresh high-score measurement (§18.3)
    LOST = 3               # identity retained for reacquisition, no measurement (§31)
    AMBIGUOUS = 4          # §32: two plausible reacquisition candidates, chose neither


_STATE_NAMES: Dict[int, str] = {int(state): state.name for state in TargetState}


@dataclass
class SelectedTargetObservation:
    """§34 verbatim, with §14's sanity guarantees enforced at construction."""

    protocol_version: int = PROTOCOL_VERSION
    selection_generation: int = 0
    session_uuid: str = ""
    track_uuid: str = ""

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    publish_timestamp_ns: int = 0

    target_state: TargetState = TargetState.NO_TARGET
    measurement_valid: bool = False

    bbox: BBox = field(default_factory=BBox)
    measured_anchor: PointNorm = field(default_factory=lambda: PointNorm(0.5, 0.5))
    anchor_source: AnchorSource = AnchorSource.BBOX_CENTER_FALLBACK

    velocity_x_norm_s: float = 0.0     # §36: optional, for the controller's own use
    velocity_y_norm_s: float = 0.0

    detector_score: float = 0.0
    association_quality: float = 0.0
    identity_confidence: float = 0.0

    ambiguity: float = 0.0             # §32: explicit, never folded into another score
    just_reacquired: bool = False      # §34: the first measurement after a LOST spell

    class_id: int = 0
    class_name: str = ""

    # -- constructors -------------------------------------------------------
    @classmethod
    def no_target(cls, selection_generation: int, session_uuid: str,
                  frame_sequence: int = 0, sensor_timestamp_ns: int = 0,
                  publish_timestamp_ns: int = 0) -> "SelectedTargetObservation":
        """The explicit "nothing is selected" publish (§31's NO_SELECTION).

        An explicit no-target message, not the absence of one: a consumer that simply
        stops receiving cannot tell "operator cleared the selection" from "visiond died",
        and those two lead to different controller actions (§34).
        """
        return cls(
            selection_generation=int(selection_generation),
            session_uuid=session_uuid,
            frame_sequence=int(frame_sequence),
            sensor_timestamp_ns=int(sensor_timestamp_ns),
            publish_timestamp_ns=int(publish_timestamp_ns),
            target_state=TargetState.NO_TARGET,
            measurement_valid=False,
        )

    @classmethod
    def from_track(cls, track, *, selection_generation: int, session_uuid: str,
                   frame_sequence: int, sensor_timestamp_ns: int,
                   publish_timestamp_ns: int,
                   target_state: TargetState,
                   measurement_valid: bool,
                   ambiguity: float = 0.0) -> "SelectedTargetObservation":
        """Project one identity into the controller's view of it.

        The anchor is copied through untouched. §36 is explicit that visiond must not
        smooth it heavily: the controller owns target-motion prediction, and a pre-smoothed
        anchor plus a controller-side lead is the same filter applied twice, with the
        phase lag of both and the benefit of neither.
        """
        return cls(
            selection_generation=int(selection_generation),
            session_uuid=session_uuid,
            track_uuid=track.track_uuid,
            frame_sequence=int(frame_sequence),
            sensor_timestamp_ns=int(sensor_timestamp_ns),
            publish_timestamp_ns=int(publish_timestamp_ns),
            target_state=target_state,
            measurement_valid=bool(measurement_valid),
            bbox=track.bbox,
            measured_anchor=track.anchor,
            anchor_source=track.anchor_source,
            velocity_x_norm_s=float(track.velocity_x),
            velocity_y_norm_s=float(track.velocity_y),
            detector_score=float(track.detector_score),
            association_quality=float(track.association_quality),
            identity_confidence=float(track.identity_confidence),
            ambiguity=float(ambiguity),
            just_reacquired=bool(track.just_reacquired),
            class_id=int(track.class_id),
            class_name=str(track.class_name),
        )

    # -- validity (§14) -----------------------------------------------------
    def validate(self) -> "SelectedTargetObservation":
        if self.protocol_version != PROTOCOL_VERSION:
            raise ValidationError(
                f"unsupported observation protocol {self.protocol_version}")
        if self.measurement_valid:
            if not self.track_uuid:
                raise ValidationError(
                    "measurement_valid with no track_uuid: a measurement nobody owns")
            if self.sensor_timestamp_ns <= 0:
                raise ValidationError(
                    "measurement_valid with no sensor timestamp: the control loop "
                    "interpolates against this value (§11), so guessing it is worse "
                    "than refusing to publish")
            if not self.bbox.is_well_formed():
                raise ValidationError(f"invalid bbox in observation: {self.bbox}")
            if not self.measured_anchor.is_valid():
                raise ValidationError(
                    f"invalid anchor in observation: {self.measured_anchor}")
        for name in ("detector_score", "association_quality", "identity_confidence",
                     "ambiguity"):
            value = getattr(self, name)
            if not is_finite(value) or not (-1e-9 <= value <= 1.0 + 1e-9):
                raise ValidationError(
                    f"{name} must be finite and in [0,1], got {value!r}")
        if not (is_finite(self.velocity_x_norm_s)
                and is_finite(self.velocity_y_norm_s)):
            raise ValidationError("observation velocity is not finite")
        return self

    @property
    def state_name(self) -> str:
        return _STATE_NAMES.get(int(self.target_state), f"STATE_{int(self.target_state)}")

    @property
    def has_measurement(self) -> bool:
        return bool(self.measurement_valid)

    # -- serialization ------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        return {
            "protocol_version": int(self.protocol_version),
            "selection_generation": int(self.selection_generation),
            "session_uuid": self.session_uuid,
            "track_uuid": self.track_uuid,
            "frame_sequence": int(self.frame_sequence),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "publish_timestamp_ns": int(self.publish_timestamp_ns),
            "target_state": int(self.target_state),
            "target_state_name": self.state_name,
            "measurement_valid": bool(self.measurement_valid),
            "bbox": self.bbox.to_dict(),
            "measured_anchor": self.measured_anchor.to_dict(),
            "anchor_source": self.anchor_source.value,
            "velocity": {"x": float(self.velocity_x_norm_s),
                         "y": float(self.velocity_y_norm_s)},
            "detector_score": float(self.detector_score),
            "association_quality": float(self.association_quality),
            "identity_confidence": float(self.identity_confidence),
            "ambiguity": float(self.ambiguity),
            "just_reacquired": bool(self.just_reacquired),
            "class_id": int(self.class_id),
            "class_name": self.class_name,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SelectedTargetObservation":
        velocity = data.get("velocity") or {}
        try:
            anchor_source = AnchorSource(data.get("anchor_source",
                                                  AnchorSource.BBOX_CENTER_FALLBACK.value))
        except ValueError:
            anchor_source = AnchorSource.BBOX_CENTER_FALLBACK
        return cls(
            protocol_version=int(data.get("protocol_version", PROTOCOL_VERSION)),
            selection_generation=int(data.get("selection_generation", 0)),
            session_uuid=str(data.get("session_uuid", "")),
            track_uuid=str(data.get("track_uuid", "")),
            frame_sequence=int(data.get("frame_sequence", 0)),
            sensor_timestamp_ns=int(data.get("sensor_timestamp_ns", 0)),
            publish_timestamp_ns=int(data.get("publish_timestamp_ns", 0)),
            target_state=TargetState(int(data.get("target_state", TargetState.NO_TARGET))),
            measurement_valid=bool(data.get("measurement_valid", False)),
            bbox=BBox.from_dict(data.get("bbox") or {}),
            measured_anchor=PointNorm.from_dict(
                data.get("measured_anchor") or {"x": 0.5, "y": 0.5}),
            anchor_source=anchor_source,
            velocity_x_norm_s=float(velocity.get("x", 0.0)),
            velocity_y_norm_s=float(velocity.get("y", 0.0)),
            detector_score=float(data.get("detector_score", 0.0)),
            association_quality=float(data.get("association_quality", 0.0)),
            identity_confidence=float(data.get("identity_confidence", 0.0)),
            ambiguity=float(data.get("ambiguity", 0.0)),
            just_reacquired=bool(data.get("just_reacquired", False)),
            class_id=int(data.get("class_id", 0)),
            class_name=str(data.get("class_name", "")),
        )

    def to_json(self) -> str:
        from .jsonio import dumps
        return dumps(self.to_dict())

    @classmethod
    def from_json(cls, text: str) -> "SelectedTargetObservation":
        from .jsonio import loads
        return cls.from_dict(loads(text))
