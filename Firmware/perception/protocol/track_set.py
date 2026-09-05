"""``TrackSet`` — every identity the subsystem currently knows, once per detector frame.

Why the whole set, not the "latest target" the retired path published: the browser needs
the candidate list to render it, the selector needs it to validate a click against
something authoritative, and the operator needs it to be the same list the turret is
acting on. Publishing a single target made the browser invent its own candidate book from
whatever it saw, which is how a click on "Person #2" could end up meaning somebody else
(§3.4).

Two counters exist specifically because §26 forbids silent truncation
(``detections_dropped_capacity``, ``tracks_evicted``) and §16.4 forbids silent duplicate
removal (inherited from the ``DetectionSet`` the pipeline reports alongside). A scene with
twenty people and sixteen slots is a *reported* condition, not a scene with sixteen people.

``track_set_sequence`` is monotonic per session and is what a selection request echoes
back (§29's ``track_set_sequence_seen_by_ui``). Without it, "the browser clicked a track
that retired three frames ago" and "the browser clicked the right track at the right
time" look identical from the daemon.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ..tracking.track import Track, TrackState, visible_candidates
from .jsonio import dumps, loads

TRACK_SET_SCHEMA_VERSION = 1


@dataclass
class TrackSetCounters:
    """§26's published capacity figures plus the lifecycle tallies §42 emits as events.

    Cumulative since ``visiond`` start, not per-frame: an operator asking "has this been
    happening all run or only since the sun came out?" needs both the rate and the total,
    and the total cannot be reconstructed from a stream of rates that were never stored.
    """

    track_capacity: int = 0
    track_capacity_used: int = 0
    detections_in: int = 0
    detections_dropped_capacity: int = 0
    detections_refused_duplicate: int = 0
    tracks_created: int = 0
    tracks_confirmed: int = 0
    tracks_occluded: int = 0
    tracks_lost: int = 0
    tracks_reacquired: int = 0
    tracks_retired: int = 0
    tracks_merged: int = 0
    tracks_evicted: int = 0
    low_score_associations: int = 0
    ambiguous_frames: int = 0

    def to_dict(self) -> Dict[str, int]:
        return {
            "track_capacity": int(self.track_capacity),
            "track_capacity_used": int(self.track_capacity_used),
            "detections_in": int(self.detections_in),
            "detections_dropped_capacity": int(self.detections_dropped_capacity),
            "detections_refused_duplicate": int(self.detections_refused_duplicate),
            "tracks_created": int(self.tracks_created),
            "tracks_confirmed": int(self.tracks_confirmed),
            "tracks_occluded": int(self.tracks_occluded),
            "tracks_lost": int(self.tracks_lost),
            "tracks_reacquired": int(self.tracks_reacquired),
            "tracks_retired": int(self.tracks_retired),
            "tracks_merged": int(self.tracks_merged),
            "tracks_evicted": int(self.tracks_evicted),
            "low_score_associations": int(self.low_score_associations),
            "ambiguous_frames": int(self.ambiguous_frames),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TrackSetCounters":
        keys = ("track_capacity", "track_capacity_used", "detections_in",
                "detections_dropped_capacity", "detections_refused_duplicate",
                "tracks_created", "tracks_confirmed",
                "tracks_occluded", "tracks_lost", "tracks_reacquired",
                "tracks_retired", "tracks_merged", "tracks_evicted",
                "low_score_associations", "ambiguous_frames")
        return cls(**{k: int((data or {}).get(k, 0)) for k in keys})


@dataclass
class TrackSet:
    """§9's TrackSet, carrying §18's lifecycle states and §27's non-reusable labels."""

    protocol_version: int = TRACK_SET_SCHEMA_VERSION
    session_uuid: str = ""
    track_set_sequence: int = 0
    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    publish_timestamp_ns: int = 0
    stream_width: int = 0
    stream_height: int = 0
    model_id: str = ""
    model_generation: int = 0
    tracks: List[Track] = field(default_factory=list)
    counters: TrackSetCounters = field(default_factory=TrackSetCounters)
    events: List[Dict[str, Any]] = field(default_factory=list)   # §42, bounded

    # -- lookups ------------------------------------------------------------
    def by_uuid(self, track_uuid: str) -> Optional[Track]:
        """§29: the selection API looks identities up by UUID, never by label."""
        if not track_uuid:
            return None
        for track in self.tracks:
            if track.track_uuid == track_uuid:
                return track
        return None

    def exists(self, track_uuid: str) -> bool:
        return self.by_uuid(track_uuid) is not None

    def visible(self) -> List[Track]:
        """Appendix B. The renderer draws these and only these as people."""
        return visible_candidates(self.tracks)

    def live(self) -> List[Track]:
        return [t for t in self.tracks if t.is_live]

    def selectable(self) -> List[Track]:
        """§37.1's gate, already evaluated by TrackManager on each track."""
        return [t for t in self.tracks
                if t.state is TrackState.CONFIRMED_VISIBLE and t.selectable
                and not t.duplicate_resolving and not t.ambiguous]

    def __len__(self) -> int:
        return len(self.tracks)

    def __bool__(self) -> bool:
        """Always true. An empty TrackSet is a statement, not an absence.

        "Nobody is visible" is the document §61's age counters and the UI's clearing depend on;
        if it were falsy, every ``set or fallback`` on the way to publishing would quietly
        substitute something else — and a consumer could not tell "empty" from "missing".
        """
        return True

    # -- validity -----------------------------------------------------------
    def validate(self) -> "TrackSet":
        from ..errors import ValidationError

        if self.protocol_version != TRACK_SET_SCHEMA_VERSION:
            raise ValidationError(
                f"unsupported TrackSet schema {self.protocol_version}")
        if self.stream_width <= 0 or self.stream_height <= 0:
            raise ValidationError(
                f"TrackSet needs source dimensions, got "
                f"{self.stream_width}x{self.stream_height}")
        for track in self.tracks:
            if not track.track_uuid:
                raise ValidationError("TrackSet contains a track with no UUID")
            if not track.bbox.is_well_formed():
                raise ValidationError(
                    f"track {track.track_uuid} has an invalid bbox {track.bbox}")
        return self

    # -- serialization ------------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        return {
            "protocol_version": int(self.protocol_version),
            "session_uuid": self.session_uuid,
            "track_set_sequence": int(self.track_set_sequence),
            "frame_sequence": int(self.frame_sequence),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "publish_timestamp_ns": int(self.publish_timestamp_ns),
            "stream_width": int(self.stream_width),
            "stream_height": int(self.stream_height),
            "model_id": self.model_id,
            "model_generation": int(self.model_generation),
            "tracks": [t.to_dict() for t in self.tracks],
            "counters": self.counters.to_dict(),
            "events": list(self.events),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TrackSet":
        return cls(
            protocol_version=int(data.get("protocol_version", TRACK_SET_SCHEMA_VERSION)),
            session_uuid=str(data.get("session_uuid", "")),
            track_set_sequence=int(data.get("track_set_sequence", 0)),
            frame_sequence=int(data.get("frame_sequence", 0)),
            sensor_timestamp_ns=int(data.get("sensor_timestamp_ns", 0)),
            publish_timestamp_ns=int(data.get("publish_timestamp_ns", 0)),
            stream_width=int(data.get("stream_width", 0)),
            stream_height=int(data.get("stream_height", 0)),
            model_id=str(data.get("model_id", "")),
            model_generation=int(data.get("model_generation", 0)),
            tracks=[Track.from_dict(t) for t in data.get("tracks") or ()],
            counters=TrackSetCounters.from_dict(data.get("counters") or {}),
            events=list(data.get("events") or ()),
        )

    def to_json(self) -> str:
        return dumps(self.to_dict())

    @classmethod
    def from_json(cls, text: str) -> "TrackSet":
        return cls.from_dict(loads(text))


def empty_track_set() -> TrackSet:
    """A well-formed "nothing is known" set.

    Publishing an explicit empty set matters: a consumer that receives nothing cannot
    distinguish "no people" from "visiond died", and the whole safety argument for §61's
    age counters depends on the difference being observable.
    """
    return TrackSet()
