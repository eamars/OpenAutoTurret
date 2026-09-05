"""``Track`` — one identity, and the state it is allowed to claim (§10, §18, §33).

Three decisions in this file are load-bearing:

**The identity is a UUID string, minted once, never reused (§10, §27).** The retired
path let the operator select by display index and let that index be recycled when a
track retired, which produced the handover's ``select_target 2 -> "selected Person #1"``
(§3.4). A label is a *rendering*, so it lives beside the UUID instead of standing in for
it. The UUID is a ``uuid4`` hex string, not a counter: a counter restarts at 1 when the
daemon restarts, which would hand a number the browser may still be holding to a
different human being. ``session_uuid`` (on the TrackSet) disambiguates across restarts.

**State is the perception lifecycle, not the controller's phase (§38).** §18's five
states answer "does this identity currently have a measurement, and how much may we
believe it?". The AutoTrack phase (``ACQUIRING`` / ``TRACKING`` / ``COASTING`` …) is a
different question owned by controld. §3.6 records what happens when one FSM is
translated into another's vocabulary: ``LOST_HOLD`` stops meaning anything to a reader,
because it is not clear whose state machine they are looking at. The two are published
side by side and never merged.

**Every age is derived from a timestamp, never counted in frames (§19).** ``miss_ms()``
and ``visible_ms()`` take "now" as an argument, so the same track answers correctly at
17 inference results per second and at 30, and so a replay (§43) ages identities by the
recorded ``SensorTimestamp`` rather than by how many lines the file happens to contain.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple

from ..detection.types import (AnchorSource, BBox, Keypoint, PointNorm,
                               display_label_for_class)
from ..measure import clamp_age_ms, ms_from_ns


class TrackState(IntEnum):
    """§18's lifecycle. Values are wire values: append, never renumber."""

    TENTATIVE = 0            # §18.1: seen, not yet believed. NOT selectable.
    CONFIRMED_VISIBLE = 1    # §18.2: currently measured and confirmed. The only
                             #         state that is drawn as a live person (§Appendix B).
    OCCLUDED = 2             # §18.3: no high-score measurement, a short miss or a
                             #         plausible low-score association.
    LOST_REACQUIRABLE = 3    # §18.4: no measurement; identity retained for a bounded
                             #         TTL for reacquisition. Never drawn as a person.
    RETIRED = 4              # §18.5: identity and appearance data discarded.

    @property
    def label(self) -> str:
        return _STATE_LABELS[self]

    def is_visible(self) -> bool:
        """§18.2/Appendix B: only CONFIRMED_VISIBLE is a visible candidate."""
        return self is TrackState.CONFIRMED_VISIBLE

    def may_hold_selection(self) -> bool:
        """§31: a selection survives OCCLUDED and LOST_REACQUIRABLE, nothing else."""
        return self in (TrackState.CONFIRMED_VISIBLE, TrackState.OCCLUDED,
                        TrackState.LOST_REACQUIRABLE)


_STATE_LABELS: Dict[TrackState, str] = {
    TrackState.TENTATIVE: "TENTATIVE",
    TrackState.CONFIRMED_VISIBLE: "CONFIRMED_VISIBLE",
    TrackState.OCCLUDED: "OCCLUDED",
    TrackState.LOST_REACQUIRABLE: "LOST_REACQUIRABLE",
    TrackState.RETIRED: "RETIRED",
}


def new_track_uuid() -> str:
    """A fresh immutable identity (§10)."""
    return uuid.uuid4().hex


def new_session_uuid() -> str:
    """§27: a process restart creates a new ``session_uuid``."""
    return uuid.uuid4().hex


@dataclass
class Track:
    """One identity, from first sight to retirement.

    Used both as TrackManager's live state (mutated in place, one owner, one thread) and
    — through :meth:`copy` — as the published record inside a TrackSet. Publishing a copy
    rather than the live object matters: the overlay, the recorder and the selector would
    otherwise be reading a data structure that the next association pass is editing.
    """

    track_uuid: str = ""
    display_index: int = 0                 # §27: monotonic per class within a session
    class_id: int = 0
    class_name: str = ""
    state: TrackState = TrackState.TENTATIVE

    # --- measurement ------------------------------------------------------
    bbox: BBox = field(default_factory=BBox)
    anchor: PointNorm = field(default_factory=lambda: PointNorm(0.5, 0.5))
    anchor_source: AnchorSource = AnchorSource.BBOX_CENTER_FALLBACK
    velocity_x: float = 0.0                # normalized image units / second (§22)
    velocity_y: float = 0.0
    keypoints: Tuple[Keypoint, ...] = ()
    pose_score: Optional[float] = None
    measurement_valid: bool = False        # §34: a measurement exists to be believed

    # --- the four separate quality fields (§37) ---------------------------
    detector_score: float = 0.0            # the model's own number, unmodified
    association_quality: float = 0.0       # how well this detection matched the track
    identity_confidence: float = 0.0       # continuity/uniqueness of the identity
    measurement_quality: float = 0.0       # bbox/keypoint suitability for aim

    # --- identity claims the selector must see ----------------------------
    ambiguous: bool = False                # §32
    ambiguity_candidates: Tuple[str, ...] = ()
    duplicate_resolving: bool = False      # §25: a merge is being watched, not decided
    selectable: bool = False               # §37.1's gate, computed at update time
    just_reacquired: bool = False          # §34: first measurement after a LOST spell

    # --- time (§19): stamps in ns, ages derived ---------------------------
    created_ns: int = 0
    first_measurement_ns: int = 0
    last_measurement_ns: int = 0           # SENSOR time of the newest measurement
    last_receive_ns: int = 0               # HOST time the manager last saw it
    visible_ms_total: float = 0.0          # accumulated time with a measurement
    last_visible_span_start_ns: int = 0    # open span, closed on the next miss
    observations: int = 0                  # §19's minimum observation count
    miss_observations: int = 0             # consecutive association misses
    low_score_only_observations: int = 0   # §20 pass-2 rescues, for diagnostics
    last_model_generation: int = 0

    # --- bookkeeping ------------------------------------------------------
    alias_of: Optional[str] = None         # §25.1: set when this identity merged away
    appearance: Any = None                 # §24's ephemeral descriptor (memory only)
    last_association: Any = None           # §41's per-track trace, when debug is on

    # -- constructors -------------------------------------------------------
    @classmethod
    def create(cls, detection_id: int, class_id: int, class_name: str, bbox: BBox,
               anchor: PointNorm, anchor_source: AnchorSource, detector_score: float,
               sensor_timestamp_ns: int, receive_timestamp_ns: int,
               display_index: int, model_generation: int = 0,
               keypoints: Tuple[Keypoint, ...] = (),
               pose_score: Optional[float] = None) -> "Track":
        """A TENTATIVE track from an unmatched high-score detection (§18.1, §20 pass 3)."""
        return cls(
            track_uuid=new_track_uuid(),
            display_index=int(display_index),
            class_id=int(class_id),
            class_name=str(class_name),
            state=TrackState.TENTATIVE,
            bbox=bbox,
            anchor=anchor,
            anchor_source=anchor_source,
            detector_score=float(detector_score),
            created_ns=int(sensor_timestamp_ns),
            last_receive_ns=int(receive_timestamp_ns),
            last_model_generation=int(model_generation),
            keypoints=tuple(keypoints),
            pose_score=pose_score,
        )

    # -- derived state ------------------------------------------------------
    @property
    def display_label(self) -> str:
        """§27's "Person #17". Rendering only; never a selection key (§29)."""
        return f"{display_label_for_class(self.class_name)} #{self.display_index}"

    @property
    def is_live(self) -> bool:
        return self.state is not TrackState.RETIRED

    @property
    def is_visible_candidate(self) -> bool:
        """Appendix B verbatim: confirmed, selectable, and not mid-merge."""
        return (self.state.is_visible() and self.selectable
                and not self.duplicate_resolving)

    @property
    def speed_norm_s(self) -> float:
        return (self.velocity_x ** 2 + self.velocity_y ** 2) ** 0.5

    def miss_age_ms(self, now_ns: int) -> float:
        """Milliseconds since the last MEASUREMENT (§19), floored at zero."""
        if self.last_measurement_ns <= 0:
            return -1.0   # never measured: age is unknown, not "0 ms"
        return clamp_age_ms(ms_from_ns(now_ns, self.last_measurement_ns))

    def identity_age_ms(self, now_ns: int) -> float:
        """Milliseconds since this identity first appeared (§26's TTL, §19)."""
        return clamp_age_ms(ms_from_ns(now_ns, self.created_ns))

    def visible_ms(self, now_ns: int) -> float:
        """Total time this identity has had a measurement, open span included (§19).

        Accumulated rather than ``now - created`` because a track that spent two seconds
        behind a pillar has not been visible for two seconds, and §18.2's confirmation
        rule is about being seen.
        """
        open_span = 0.0
        if self.last_visible_span_start_ns > 0 and self.last_measurement_ns > 0:
            open_span = clamp_age_ms(
                ms_from_ns(self.last_measurement_ns, self.last_visible_span_start_ns))
        return self.visible_ms_total + open_span

    def predicted_anchor(self, dt_s: float) -> PointNorm:
        """Constant-velocity prediction to the next capture (§22)."""
        if dt_s <= 0.0:
            return PointNorm(self.anchor.x, self.anchor.y)
        return PointNorm(self.anchor.x + self.velocity_x * dt_s,
                         self.anchor.y + self.velocity_y * dt_s)

    # -- serialization ------------------------------------------------------
    def copy(self) -> "Track":
        """A published snapshot. ``appearance`` stays shared on purpose (§24)."""
        return replace(self,
                       ambiguity_candidates=tuple(self.ambiguity_candidates),
                       keypoints=tuple(self.keypoints))

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "track_uuid": self.track_uuid,
            "display_index": int(self.display_index),
            "display_label": self.display_label,
            "class_id": int(self.class_id),
            "class_name": self.class_name,
            "state": int(self.state),
            "state_name": self.state.label,
            "bbox": self.bbox.to_dict(),
            "anchor": self.anchor.to_dict(),
            "anchor_source": self.anchor_source.value,
            "velocity": {"x": float(self.velocity_x), "y": float(self.velocity_y)},
            "measurement_valid": bool(self.measurement_valid),
            "detector_score": float(self.detector_score),
            "association_quality": float(self.association_quality),
            "identity_confidence": float(self.identity_confidence),
            "measurement_quality": float(self.measurement_quality),
            "ambiguous": bool(self.ambiguous),
            "ambiguity_candidates": list(self.ambiguity_candidates),
            "duplicate_resolving": bool(self.duplicate_resolving),
            "selectable": bool(self.selectable),
            "just_reacquired": bool(self.just_reacquired),
            "created_ns": int(self.created_ns),
            "first_measurement_ns": int(self.first_measurement_ns),
            "last_measurement_ns": int(self.last_measurement_ns),
            "visible_ms": round(self.visible_ms(self.last_receive_ns), 3),
            "observations": int(self.observations),
            "miss_observations": int(self.miss_observations),
            "low_score_only_observations": int(self.low_score_only_observations),
            "last_model_generation": int(self.last_model_generation),
        }
        if self.alias_of:
            out["alias_of"] = self.alias_of
        if self.keypoints:
            out["keypoints"] = [k.to_dict() for k in self.keypoints]
        if self.pose_score is not None:
            out["pose_score"] = float(self.pose_score)
        return out

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Track":
        """Rebuild a published track from :meth:`to_dict` (replay, tests, diagnostics).

        The lifecycle fields round-trip so a replayed TrackSet can drive the selector and
        the renderer without re-running association — which is how §52's deterministic
        selection tests get to be deterministic.
        """
        try:
            anchor_source = AnchorSource(data.get("anchor_source",
                                                  AnchorSource.BBOX_CENTER_FALLBACK.value))
        except ValueError:
            anchor_source = AnchorSource.BBOX_CENTER_FALLBACK
        velocity = data.get("velocity") or {}
        return cls(
            track_uuid=str(data.get("track_uuid", "")),
            display_index=int(data.get("display_index", 0)),
            class_id=int(data.get("class_id", 0)),
            class_name=str(data.get("class_name", "")),
            state=TrackState(int(data.get("state", TrackState.TENTATIVE))),
            bbox=BBox.from_dict(data.get("bbox") or {}),
            anchor=PointNorm.from_dict(data.get("anchor") or {"x": 0.5, "y": 0.5}),
            anchor_source=anchor_source,
            velocity_x=float(velocity.get("x", 0.0)),
            velocity_y=float(velocity.get("y", 0.0)),
            keypoints=tuple(Keypoint.from_dict(k)
                            for k in data.get("keypoints") or ()),
            pose_score=(None if data.get("pose_score") is None
                        else float(data["pose_score"])),
            measurement_valid=bool(data.get("measurement_valid", False)),
            detector_score=float(data.get("detector_score", 0.0)),
            association_quality=float(data.get("association_quality", 0.0)),
            identity_confidence=float(data.get("identity_confidence", 0.0)),
            measurement_quality=float(data.get("measurement_quality", 0.0)),
            ambiguous=bool(data.get("ambiguous", False)),
            ambiguity_candidates=tuple(data.get("ambiguity_candidates") or ()),
            duplicate_resolving=bool(data.get("duplicate_resolving", False)),
            selectable=bool(data.get("selectable", False)),
            just_reacquired=bool(data.get("just_reacquired", False)),
            created_ns=int(data.get("created_ns", 0)),
            first_measurement_ns=int(data.get("first_measurement_ns", 0)),
            last_measurement_ns=int(data.get("last_measurement_ns", 0)),
            last_receive_ns=int(data.get("last_measurement_ns", 0)),
            visible_ms_total=float(data.get("visible_ms", 0.0)),
            observations=int(data.get("observations", 0)),
            miss_observations=int(data.get("miss_observations", 0)),
            low_score_only_observations=int(
                data.get("low_score_only_observations", 0)),
            last_model_generation=int(data.get("last_model_generation", 0)),
            alias_of=data.get("alias_of"),
        )


def visible_candidates(tracks) -> List[Track]:
    """Appendix B's rendering rule, in one place so the HUD and the recorder agree."""
    return [t for t in tracks if t.is_visible_candidate]
