"""§41 — a bounded, per-frame association trace.

The handover's central complaint about identity loss is not that identities die but that
nobody can say why. Two kinds of explanation were available and both were wrong: a log
line per frame, which is unreadable and fills an SD card, and no record at all, which is
what the station had. §41's answer is a ring buffer of *decisions* — for each detection,
the candidates it considered and why each was gated or chosen; for each track, the state it
was in, the state it moved to, and the numbers that decided it — kept in bounded memory and
dumped when something breaks.

Two properties are load-bearing:

**Bounded.** ``capacity`` frames, then the oldest are dropped and ``dropped`` counts it. A
diagnostic that leaks is a diagnostic that is turned off, and this one is explicitly not
allowed to be turned off during a long run (§41's "not permanently enabled in all builds"
refers to cost on the frame path, not to whether it may exist).

**Cheap when disabled.** ``enabled=False`` makes every ``record_*`` call a comparison and a
return, so TrackManager can call it unconditionally. A trace that has to be threaded
through ``if`` statements at every call site is a trace that will be missing the one call
site that mattered.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence

from ..protocol.jsonio import atomic_write_text, dumps


@dataclass
class CandidateNote:
    """One track the detector considered for one detection."""

    track_uuid: str
    outcome: str                 # "matched" | "gated" | "outvoted" | "considered"
    reason: str = ""             # the §21 gate name, when there was one
    cost: float = 0.0
    quality: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"track_uuid": self.track_uuid, "outcome": self.outcome}
        if self.reason:
            out["reason"] = self.reason
        if self.cost:
            out["cost"] = round(float(self.cost), 5)
        if self.quality:
            out["quality"] = round(float(self.quality), 5)
        return out


@dataclass
class DetectionTrace:
    """Everything association decided about one detection in one frame."""

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    detection_id: int = 0
    class_name: str = ""
    detector_score: float = 0.0
    candidates: List[CandidateNote] = field(default_factory=list)
    assigned_track: Optional[str] = None
    created_track: Optional[str] = None
    dedup_reason: str = ""          # why the deduplicator kept/dropped it (§16.4)
    suppressed: bool = False

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "kind": "detection",
            "frame_sequence": int(self.frame_sequence),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "detection_id": int(self.detection_id),
            "class_name": self.class_name,
            "detector_score": round(float(self.detector_score), 4),
            "candidates": [c.to_dict() for c in self.candidates],
            "suppressed": bool(self.suppressed),
        }
        if self.assigned_track:
            out["assigned_track"] = self.assigned_track
        if self.created_track:
            out["created_track"] = self.created_track
        if self.dedup_reason:
            out["dedup_reason"] = self.dedup_reason
        return out


@dataclass
class TrackTrace:
    """Everything association decided about one identity in one frame."""

    frame_sequence: int = 0
    sensor_timestamp_ns: int = 0
    track_uuid: str = ""
    state_before: str = ""
    state_after: str = ""
    matched_detection: Optional[int] = None
    miss_age_ms: float = 0.0
    reacquisition_score: float = 0.0
    ambiguous: bool = False
    merge_decision: str = ""

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "kind": "track",
            "frame_sequence": int(self.frame_sequence),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "track_uuid": self.track_uuid,
            "state_before": self.state_before,
            "state_after": self.state_after,
            "miss_age_ms": round(float(self.miss_age_ms), 3),
        }
        if self.matched_detection is not None:
            out["matched_detection"] = int(self.matched_detection)
        if self.reacquisition_score:
            out["reacquisition_score"] = round(float(self.reacquisition_score), 5)
        if self.ambiguous:
            out["ambiguous"] = True
        if self.merge_decision:
            out["merge_decision"] = self.merge_decision
        return out


class AssociationDiagnostics:
    """§41's bounded ring of per-frame association decisions."""

    def __init__(self, capacity: int = 512, enabled: bool = True,
                 persist_path: str = "") -> None:
        if capacity < 8:
            raise ValueError("diagnostics capacity must be at least 8 frames")
        self.capacity = int(capacity)
        self.enabled = bool(enabled)
        self.persist_path = persist_path
        self._frames: Deque[Dict[str, Any]] = deque(maxlen=self.capacity)
        self._current: Optional[Dict[str, Any]] = None
        self.frames_recorded = 0
        self.frames_dropped = 0
        self.last_fault_reason = ""

    # -- frame lifecycle ----------------------------------------------------
    def begin_frame(self, frame_sequence: int, sensor_timestamp_ns: int) -> None:
        if not self.enabled:
            return
        self._current = {"frame_sequence": int(frame_sequence),
                         "sensor_timestamp_ns": int(sensor_timestamp_ns),
                         "detections": [], "tracks": []}

    def end_frame(self) -> None:
        if not self.enabled or self._current is None:
            return
        if len(self._frames) == self.capacity:
            self.frames_dropped += 1
        self._frames.append(self._current)
        self.frames_recorded += 1
        self._current = None

    # -- records ------------------------------------------------------------
    def record_detection(self, detection, *, candidates: Sequence[CandidateNote] = (),
                         assigned_track: Optional[str] = None,
                         created_track: Optional[str] = None,
                         dedup_reason: str = "", suppressed: bool = False) -> None:
        if not self.enabled or self._current is None:
            return
        trace = DetectionTrace(
            frame_sequence=self._current["frame_sequence"],
            sensor_timestamp_ns=self._current["sensor_timestamp_ns"],
            detection_id=detection.detection_id_in_frame,
            class_name=detection.class_name,
            detector_score=detection.detector_score,
            candidates=list(candidates),
            assigned_track=assigned_track, created_track=created_track,
            dedup_reason=dedup_reason, suppressed=suppressed)
        self._current["detections"].append(trace.to_dict())

    def record_track(self, track, *, state_before: str, state_after: str,
                     matched_detection: Optional[int] = None, miss_age_ms: float = 0.0,
                     reacquisition_score: float = 0.0, ambiguous: bool = False,
                     merge_decision: str = "") -> None:
        if not self.enabled or self._current is None:
            return
        trace = TrackTrace(
            frame_sequence=self._current["frame_sequence"],
            sensor_timestamp_ns=self._current["sensor_timestamp_ns"],
            track_uuid=track.track_uuid, state_before=state_before,
            state_after=state_after, matched_detection=matched_detection,
            miss_age_ms=miss_age_ms, reacquisition_score=reacquisition_score,
            ambiguous=ambiguous, merge_decision=merge_decision)
        self._current["tracks"].append(trace.to_dict())

    def record_merge(self, merged_uuid: str, survivor_uuid: str, *, reason: str,
                     persistent: bool, evidence: float = 0.0) -> None:
        """§25's merge decision, including the ones that were *not* taken.

        A "waited, not persistent yet" record is the interesting half: when an operator
        asks why one person was two identities for a third of a second, the answer is that
        the resolver deliberately refused to merge on a single frame's evidence.
        """
        if not self.enabled or self._current is None:
            return
        self._current["tracks"].append({
            "kind": "merge",
            "frame_sequence": self._current["frame_sequence"],
            "sensor_timestamp_ns": self._current["sensor_timestamp_ns"],
            "merged_uuid": merged_uuid,
            "survivor_uuid": survivor_uuid,
            "decision": "merged" if persistent else "held",
            "reason": reason,
            "evidence": round(float(evidence), 5),
        })

    # -- queries ------------------------------------------------------------
    def recent(self, limit: int = 20) -> List[Dict[str, Any]]:
        frames = list(self._frames)
        return frames[-int(limit):] if limit else frames

    def for_track(self, track_uuid: str, limit: int = 50) -> List[Dict[str, Any]]:
        """Every recorded decision that mentions one identity, oldest first.

        Answers §53's question — "why did this identity stop receiving measurements?" — by
        returning the gate names that were applied to it, in order, rather than an
        inference from what happened afterwards.
        """
        if not track_uuid:
            return []
        hits: List[Dict[str, Any]] = []
        for frame in self._frames:
            for record in frame["tracks"] + frame["detections"]:
                mentioned = (record.get("track_uuid") == track_uuid
                             or record.get("assigned_track") == track_uuid
                             or record.get("created_track") == track_uuid
                             or any(c.get("track_uuid") == track_uuid
                                    for c in record.get("candidates", ()))
                             or record.get("survivor_uuid") == track_uuid
                             or record.get("merged_uuid") == track_uuid)
                if mentioned:
                    hits.append(record)
        return hits[-int(limit):] if limit else hits

    def dump(self, path: str) -> int:
        """Atomically write the whole ring (§41's "dumpable"). Returns records written."""
        frames = list(self._frames)
        if self._current is not None:
            frames.append(self._current)
        payload = {"dumped_at_ns": int(time.monotonic_ns()),
                   "capacity": self.capacity,
                   "frames_recorded": self.frames_recorded,
                   "frames_dropped": self.frames_dropped,
                   "frames": frames}
        atomic_write_text(path, dumps(payload, indent=2))
        return sum(len(f["detections"]) + len(f["tracks"]) for f in frames)

    def persist_on_fault(self, reason: str) -> Optional[str]:
        """Dump to the configured path when something has just gone wrong."""
        if not self.enabled or not self.persist_path:
            return None
        path = f"{self.persist_path}.{int(time.time())}"
        try:
            self.dump(path)
        except OSError:
            return None      # a full card must not turn a fault into a crash
        self.last_fault_reason = reason
        return path
