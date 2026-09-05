"""§20 + Appendix C — the two-stage association, and the three ways a detection may act.

ByteTrack's insight is that a detector's low-score outputs are not noise: they are the same
person, seen through an occlusion or at an awkward angle. Deleting them at a fixed
confidence floor is what makes a target vanish mid-stride and reappear as a new identity
after the occlusion. So the low-score rows are kept — but with **asymmetric rights**, which
is the part an implementation gets wrong:

* a high-score detection may **create** an identity (§20 pass 3), **confirm** one, and
  **update** one (§20 pass 1);
* a low-score detection may only **rescue** an identity that already exists (§20 pass 2).
  It can never create one, and it may not confirm one.

The asymmetry is the safety property. Without it, the ``low_association`` threshold alone
lets a detector's worst frame mint a new identity, and the operator watches a person appear
out of nothing — the same failure class as the duplicate tracks this subsystem exists to
remove.

One deliberate departure from §20's literal wording: pass 1 also matches ``TENTATIVE``
tracks. §18.1's confirmation requires three observations and 120 ms of visibility, and a
tentative track can only accumulate either if something measures it; a literal reading of
"pass 1 matches CONFIRMED_VISIBLE and OCCLUDED" leaves no transition into CONFIRMED at all.
Low-score detections still cannot touch a tentative track, so they still cannot confirm.

Index convention, because two of them were colliding in the first draft of this file: track
references are positions in the caller's ``tracks`` list; detection references are always
``detection_id_in_frame``, never a position in a score-split list.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import ScoreThresholds, TrackingConfig
from ..detection.types import Detection, PointNorm
from ..tracking.association import AssociationCost, Assignment, assign
from ..tracking.diagnostics import CandidateNote
from ..tracking.track import Track, TrackState

HIGH_PASS = "high_score"
LOW_PASS = "low_score_rescue"


@dataclass
class ScoreSplit:
    """§20's three score bands."""

    high: List[Detection] = field(default_factory=list)          # >= confirmed_update
    low: List[Detection] = field(default_factory=list)           # >= low_association
    ignored: List[Detection] = field(default_factory=list)       # below low_association

    def counts(self) -> Dict[str, int]:
        return {"high": len(self.high), "low": len(self.low), "ignored": len(self.ignored)}


@dataclass
class PassRecord:
    """One association pass, as §41 wants to report it."""

    name: str
    detections: int = 0
    tracks: int = 0
    matches: int = 0
    gated: Dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        return {"name": self.name, "detections": self.detections, "tracks": self.tracks,
                "matches": self.matches, "gated": dict(self.gated)}


@dataclass
class AssociationOutcome:
    """Everything TrackManager needs to advance state, and §41 needs to explain."""

    high_matches: List[Tuple[int, int, float]] = field(default_factory=list)
    low_matches: List[Tuple[int, int, float]] = field(default_factory=list)
    reacquired: List[Tuple[int, int, float]] = field(default_factory=list)
    unmatched_high: List[int] = field(default_factory=list)     # high-band residual
    new_track_candidates: List[int] = field(default_factory=list)   # §20 pass 3
    unmatched_low: List[int] = field(default_factory=list)      # counted, never created
    ignored: List[int] = field(default_factory=list)
    unmatched_tracks: List[int] = field(default_factory=list)
    passes: List[PassRecord] = field(default_factory=list)
    cells: Dict[Tuple[int, int], AssociationCost] = field(default_factory=dict)
    rejected: List[Tuple[int, int, str]] = field(default_factory=list)
    split_counts: Dict[str, int] = field(default_factory=dict)

    def match_for_track(self, track_index: int) -> Optional[Tuple[int, float, str]]:
        """The detection that measured this track, and which pass decided it."""
        for bucket, name in ((self.high_matches, HIGH_PASS), (self.low_matches, LOW_PASS),
                             (self.reacquired, "reacquired")):
            for ti, detection_id, quality in bucket:
                if ti == track_index:
                    return detection_id, quality, name
        return None

    def to_dict(self) -> Dict[str, int]:
        return {"high_matches": len(self.high_matches),
                "low_matches": len(self.low_matches),
                "reacquired": len(self.reacquired),
                "new_track_candidates": len(self.new_track_candidates),
                "ignored_detections": len(self.ignored)}


def split_by_score(detections: Sequence[Detection],
                   thresholds: ScoreThresholds) -> ScoreSplit:
    """§20's bands. A missing threshold means nothing qualifies, not that all does (§50).

    Treating an unresolved ``COMMISSION`` as zero would turn the subsystem into a machine
    that mints identities from detector noise, and it would do so while looking configured.
    """
    split = ScoreSplit()
    confirmed = thresholds.confirmed_update
    low = thresholds.low_association
    for detection in detections:
        if confirmed is None or low is None:
            split.ignored.append(detection)
            continue
        if detection.detector_score >= confirmed:
            split.high.append(detection)
        elif detection.detector_score >= low:
            split.low.append(detection)
        else:
            split.ignored.append(detection)
    return split


def _run_pass(tracks: Sequence[Track], detections: Sequence[Detection], *,
              dt_s: float, cfg: TrackingConfig, sensor_timestamp_ns: int,
              include_states: Sequence[TrackState], name: str,
              appearance_scorer, shift: Optional[PointNorm] = None,
              ) -> Tuple[Assignment, PassRecord]:
    assignment = assign(tracks, detections, dt_s, cfg,
                        sensor_timestamp_ns=sensor_timestamp_ns,
                        include_states=include_states,
                        appearance_scorer=appearance_scorer, shift=shift)
    gated: Dict[str, int] = {}
    for _ti, _di, reason in assignment.rejected:
        gated[reason] = gated.get(reason, 0) + 1
    record = PassRecord(name=name, detections=len(detections),
                        tracks=len({ti for ti, _d, _q in assignment.matches})
                        + len(assignment.unmatched_tracks),
                        matches=len(assignment.matches), gated=gated)
    return assignment, record


def two_stage_associate(tracks: Sequence[Track], detections: Sequence[Detection], *,
                        dt_s: float, cfg: TrackingConfig, thresholds: ScoreThresholds,
                        sensor_timestamp_ns: int,
                        appearance_scorer=None,
                        shift: Optional[PointNorm] = None) -> AssociationOutcome:
    """Appendix C: high score against live identities, low score against rescue-able ones."""
    outcome = AssociationOutcome()
    split = split_by_score(detections, thresholds)
    outcome.split_counts = split.counts()
    outcome.ignored = [d.detection_id_in_frame for d in split.ignored]

    # ---- pass 1: high score, against every identity that can still be measured -------
    pass1, record1 = _run_pass(
        tracks, split.high, dt_s=dt_s, cfg=cfg,
        sensor_timestamp_ns=sensor_timestamp_ns,
        include_states=(TrackState.TENTATIVE, TrackState.CONFIRMED_VISIBLE,
                        TrackState.OCCLUDED, TrackState.LOST_REACQUIRABLE),
        name=HIGH_PASS, appearance_scorer=appearance_scorer, shift=shift)
    outcome.passes.append(record1)
    # Pass 1 sees the caller's own *track* numbering, but its detection columns are
    # positions in ``split.high``; every index leaving this function is a detection id.
    outcome.cells.update({(ti, split.high[position].detection_id_in_frame): payload
                          for (ti, position), payload in pass1.cells.items()})
    outcome.rejected.extend((ti, split.high[position].detection_id_in_frame, reason)
                            for ti, position, reason in pass1.rejected)
    for ti, position, quality in pass1.matches:
        detection_id = split.high[position].detection_id_in_frame
        if tracks[ti].state is TrackState.LOST_REACQUIRABLE:
            outcome.reacquired.append((ti, detection_id, quality))
        else:
            outcome.high_matches.append((ti, detection_id, quality))
    matched_tracks = {ti for ti, _position, _q in pass1.matches}

    # ---- pass 2: low score, rescue only (§20). Already-measured tracks are excluded so
    #      one person cannot be measured twice in one frame by two thresholds. ----------
    # §20 names OCCLUDED and LOST_REACQUIRABLE, which is ByteTrack's wording for "tracks
    # that pass 1 did not measure". A CONFIRMED_VISIBLE track left unmeasured this frame is
    # that same situation, and excluding it would cost an identity a one-frame score dip —
    # the exact failure this pass exists to remove. Tentative candidates are deliberately
    # left out: a low-score detection must not help an unconfirmed candidate stay alive.
    rescue_positions = [index for index, track in enumerate(tracks)
                        if index not in matched_tracks
                        and track.state in (TrackState.CONFIRMED_VISIBLE,
                                            TrackState.OCCLUDED,
                                            TrackState.LOST_REACQUIRABLE)]
    if rescue_positions and split.low:
        rescue_tracks = [tracks[index] for index in rescue_positions]
        pass2, record2 = _run_pass(
            rescue_tracks, split.low, dt_s=dt_s, cfg=cfg,
            sensor_timestamp_ns=sensor_timestamp_ns,
            include_states=(TrackState.CONFIRMED_VISIBLE, TrackState.OCCLUDED,
                            TrackState.LOST_REACQUIRABLE),
            name=LOW_PASS, appearance_scorer=appearance_scorer, shift=shift)
        outcome.passes.append(record2)
        for position, det_pos, quality in pass2.matches:
            original = rescue_positions[position]
            detection_id = split.low[det_pos].detection_id_in_frame
            outcome.low_matches.append((original, detection_id, quality))
            matched_tracks.add(original)
        for (position, det_pos), payload in pass2.cells.items():
            outcome.cells[(rescue_positions[position],
                           split.low[det_pos].detection_id_in_frame)] = payload
        outcome.rejected.extend((rescue_positions[position],
                                 split.low[det_pos].detection_id_in_frame, reason)
                                for position, det_pos, reason in pass2.rejected)

    # Pass 3 is not a pass: the caller turns the creation candidates below into new
    # tentative tracks, subject to §26's capacity ceiling.
    matched_high_ids = {split.high[position].detection_id_in_frame
                        for _ti, position, _q in pass1.matches}
    outcome.unmatched_high = [d.detection_id_in_frame for d in split.high
                              if d.detection_id_in_frame not in matched_high_ids]
    matched_low_ids = {detection_id for _ti, detection_id, _q in outcome.low_matches}
    outcome.unmatched_low = [d.detection_id_in_frame for d in split.low
                             if d.detection_id_in_frame not in matched_low_ids]

    # ---- pass 3: creation. Any unmatched detection at or above ``new_track`` may start a
    #      tentative identity — including one from the low band, which is why that threshold
    #      is separate from ``confirmed_update`` in the first place. The gate that keeps this
    #      safe is §18.1: a new identity is tentative until three observations and 120 ms,
    #      and §37.1 keeps a tentative identity unselectable. 
    matched_any = matched_high_ids | matched_low_ids
    new_threshold = thresholds.new_track
    if new_threshold is None:
        outcome.new_track_candidates = []
    else:
        outcome.new_track_candidates = [
            detection.detection_id_in_frame for detection in detections
            if detection.detection_id_in_frame not in matched_any
            and detection.detector_score >= new_threshold]
    outcome.unmatched_tracks = [index for index in range(len(tracks))
                                if index not in matched_tracks]
    return outcome


def candidate_notes(tracks: Sequence[Track],
                    outcome: AssociationOutcome) -> Dict[int, List[CandidateNote]]:
    """§41's per-detection candidate list, rebuilt from the assignment's own bookkeeping."""
    notes: Dict[int, List[CandidateNote]] = {}
    for (ti, detection_id), payload in outcome.cells.items():
        notes.setdefault(detection_id, []).append(
            CandidateNote(track_uuid=tracks[ti].track_uuid, outcome="considered",
                          cost=payload.cost, quality=payload.quality))
    for ti, detection_id, reason in outcome.rejected:
        notes.setdefault(detection_id, []).append(
            CandidateNote(track_uuid=tracks[ti].track_uuid, outcome="gated", reason=reason))
    for bucket in (outcome.high_matches, outcome.low_matches, outcome.reacquired):
        for ti, detection_id, quality in bucket:
            for note in notes.get(detection_id, ()):
                if note.track_uuid == tracks[ti].track_uuid:
                    note.outcome = "matched"
                    note.quality = quality
    return notes
