"""§28 and §37.1 — who may be selected, and who may be picked without being asked.

Two questions live here, and they must not share an answer:

**"May the operator select this?"** (§37.1) — a *gate*: confirmed, currently visible, not
ambiguous, not mid-merge, and past the model's ``selectable`` score and the configured
identity-confidence floor. It is a permission, not a ranking. §28.3 refuses a "best target"
ranking outright, so nothing in this module computes which person is most worth shooting at.

**"May the system pick this one on its own?"** (§28.2) — a *dwell machine*: exactly one
selectable candidate, held alone for ``auto_select_single_dwell_ms``, passing separate and
higher confidence thresholds than the manual gate. The dwell is not ceremony. A detector
emits a single confident box for a moment at the edge of frame; a selector that acted on the
first frame would take ownership of the turret for that moment.

This module is consulted by ``TargetSelectionManager`` and by ``TrackManager``'s published
``selectable`` flag, from the same code, so the browser's greyed-out row, the daemon's
refusal and the tracker's own claim cannot disagree — which is the failure mode that makes an
interface feel broken when nothing is actually broken.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence

from ..config import ScoreThresholds, SelectionConfig, SelectionPolicy
from ..measure import ms_from_ns
from ..selection.protocol import SelectionReason
from ..tracking.track import Track, TrackState


@dataclass
class Selectability:
    """The §37.1 verdict, with the reason an operator would be told."""

    allowed: bool = False
    reason: SelectionReason = SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE
    detail: str = ""

    def __bool__(self) -> bool:
        return bool(self.allowed)


def evaluate_selectability(track: Track, *, thresholds: ScoreThresholds,
                           selection: SelectionConfig) -> Selectability:
    """§37.1, in the order an operator's question is usually asked.

    The order matters for the *reason code*: "never confirmed", "not visible right now" and
    "identity unresolved" are three different truths, and collapsing them into one rejection
    is what makes §32's ambiguity look like a dropout in the logs afterwards.
    """
    if track.state is TrackState.TENTATIVE:
        return Selectability(False, SelectionReason.TRACK_NOT_CONFIRMED,
                             f"{track.display_label} has not been confirmed yet (§18.1)")
    if track.state is TrackState.RETIRED:
        return Selectability(False, SelectionReason.TRACK_NOT_FOUND,
                             f"{track.display_label} no longer exists")
    if track.state is not TrackState.CONFIRMED_VISIBLE:
        return Selectability(False, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE,
                             f"{track.display_label} is {track.state.label.lower()}, "
                             f"not currently visible")
    if track.ambiguous:
        candidates = ", ".join(uuid[:8] for uuid in track.ambiguity_candidates)
        return Selectability(False, SelectionReason.TRACK_IDENTITY_UNRESOLVED,
                             f"two identities could explain this detection (§32); "
                             f"rivals: {candidates or 'unknown'}")
    if track.duplicate_resolving:
        return Selectability(False, SelectionReason.TRACK_IDENTITY_UNRESOLVED,
                             "a duplicate merge is being watched (§25)")

    selectable = thresholds.selectable
    if selectable is None:
        # §50: an uncommissioned threshold is a refusal, not a zero. A station that silently
        # defaulted this would be selecting whatever the detector liked least.
        return Selectability(False, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE,
                             "selectable threshold is COMMISSION (§50)")
    if track.detector_score < selectable:
        return Selectability(False, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE,
                             f"detector_score {track.detector_score:.2f} below "
                             f"selectable {selectable:.2f}")
    if track.identity_confidence < selection.select_min_identity_confidence:
        return Selectability(False, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE,
                             f"identity_confidence {track.identity_confidence:.2f} below "
                             f"{selection.select_min_identity_confidence:.2f}")
    return Selectability(True, SelectionReason.ACCEPTED, "selectable")


@dataclass
class AutoDecision:
    """What the dwell machine decided this frame, and why — including "nothing"."""

    track_uuid: str = ""
    reason: str = "no_policy"
    candidates: int = 0
    dwell_ms: float = 0.0

    def to_dict(self) -> Dict[str, object]:
        return {"track_uuid": self.track_uuid, "reason": self.reason,
                "candidates": int(self.candidates), "dwell_ms": round(self.dwell_ms, 1)}


class AutoSelector:
    """§28.2's ``auto_select_single``: one candidate, alone, for a dwell.

    The machine tracks *the identity of the sole candidate*, not merely "there was one
    candidate". A counter that reset only when the count exceeded one would let a person
    walking out of frame and a different person walking in share a dwell clock, and the
    system would auto-select the newcomer on the strength of the departure's timing.
    """

    def __init__(self, selection: SelectionConfig, thresholds: ScoreThresholds) -> None:
        self.cfg = selection
        self.thresholds = thresholds
        self._sole_uuid: str = ""
        self._sole_since_ns: int = 0
        self.triggered = 0
        self.suppressed_by_existing_selection = 0
        self.denied_missing_thresholds = 0

    @property
    def enabled(self) -> bool:
        return self.cfg.policy is SelectionPolicy.AUTO_SELECT_SINGLE

    def reset(self, reason: str = "") -> None:
        self._sole_uuid = ""
        self._sole_since_ns = 0

    def evaluate(self, tracks: Sequence[Track], sensor_timestamp_ns: int,
                 *, selection_active: bool) -> AutoDecision:
        """One step of the machine. Returns the identity to auto-select, if any."""
        if not self.enabled:
            return AutoDecision(reason="explicit_only_policy")
        if selection_active:
            # §28.2's auto-selection never takes the target away from an operator. If that
            # ever needs to change, the change belongs in a policy the operator opted into,
            # not in a corner of this machine.
            self.reset()
            self.suppressed_by_existing_selection += 1
            return AutoDecision(reason="selection_already_active")
        if self.thresholds.selectable is None \
                or self.cfg.auto_select_min_detector_score is None \
                or self.cfg.auto_select_min_identity_confidence is None:
            # Auto-selecting on borrowed thresholds is how a system ends up aiming at a
            # stranger because nobody finished the commissioning sheet (§50).
            self.reset()
            self.denied_missing_thresholds += 1
            return AutoDecision(reason="thresholds_uncommissioned")

        candidates = [track for track in tracks if _auto_eligible(
            track, thresholds=self.thresholds, selection=self.cfg)]
        if len(candidates) != 1:
            # A single OCCLUDED sole candidate is a skipped tensor (gap), not a lost one: §23
            # retires a track only after its window, so keep the dwell. But a genuine SECOND
            # candidate (len >= 2) still cancels — §28.2's rule is "exactly one, alone", and a
            # second person entering the frame is a real ambiguity, not a hardware cadence.
            if len(candidates) >= 2:
                self.reset()
                return AutoDecision(reason="not_a_single_candidate",
                                    candidates=len(candidates))
            # len(candidates) == 0 and the previous sole candidate is still alive.
            ongoing = next((t for t in tracks if t.track_uuid == self._sole_uuid), None)
            bar_ok = ongoing is not None and (
                ongoing.detector_score >= self.cfg.auto_select_min_detector_score
                and ongoing.identity_confidence >= self.cfg.auto_select_min_identity_confidence
                and ongoing.state in (TrackState.CONFIRMED_VISIBLE, TrackState.OCCLUDED))
            if not bar_ok:
                self.reset()
                return AutoDecision(reason="no_candidate", candidates=0)
            candidates = [ongoing]

        sole = candidates[0]
        if sole.track_uuid != self._sole_uuid:
            self._sole_uuid = sole.track_uuid
            self._sole_since_ns = sensor_timestamp_ns
            return AutoDecision(reason="dwell_started", candidates=1, dwell_ms=0.0)

        dwell_ms = ms_from_ns(sensor_timestamp_ns, self._sole_since_ns)
        if dwell_ms < self.cfg.auto_select_single_dwell_ms:
            return AutoDecision(reason="dwelling", candidates=1, dwell_ms=dwell_ms)
        self.reset()
        self.triggered += 1
        return AutoDecision(track_uuid=sole.track_uuid, reason="dwell_elapsed",
                            candidates=1, dwell_ms=dwell_ms)

    def to_dict(self) -> Dict[str, object]:
        return {"enabled": self.enabled, "policy": self.cfg.policy.value,
                "dwell_ms": self.cfg.auto_select_single_dwell_ms,
                "sole_uuid": self._sole_uuid, "triggered": self.triggered,
                "suppressed_by_existing_selection": self.suppressed_by_existing_selection,
                "denied_missing_thresholds": self.denied_missing_thresholds}


def _auto_eligible(track: Track, *, thresholds: ScoreThresholds,
                   selection: SelectionConfig) -> bool:
    """§28.2's bar: the §37.1 gate plus the separate, higher auto-select thresholds.

    Two thresholds rather than one because the two decisions carry different risk. A human
    looking at the overlay can see that the box they clicked is the person they meant; an
    unattended auto-select has no such witness, so §28.2 asks for more evidence, and the
    commissioning sheet leaves both blank rather than inventing a number.
    """
    if not evaluate_selectability(track, thresholds=thresholds,
                                  selection=selection).allowed:
        return False
    if selection.auto_select_min_detector_score is None \
            or selection.auto_select_min_identity_confidence is None:
        return False
    if track.detector_score < selection.auto_select_min_detector_score:
        return False
    if track.identity_confidence < selection.auto_select_min_identity_confidence:
        return False
    return True
