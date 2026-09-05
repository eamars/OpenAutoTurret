"""§25 — the duplicate-track resolver: one person, two identities, merged on evidence.

The detector's host dedup (§16) tries to stop two boxes for one person before they reach
here. This module is what happens when one gets through anyway: two tracks, one human being,
both live, both plausible. The failure it prevents is the one an operator notices first —
"Person #3" is the person "Person #1" was, and the selection is now split across them.

**A dwell, not a frame.** §25's rule is that a duplicate must persist before anything is
merged, and the reason is that a single frame's geometry is genuinely ambiguous: two people
who brush past each other produce one frame of near-perfect overlap, and a resolver that
merged on that would fuse two strangers into one identity whose UUID history is a lie to
both of them. Waiting 300 ms of *scene time* (§19: milliseconds, not frames) costs an
operator a third of a second of a visible duplicate and spares them an unrecoverable merge.

**Merges are recorded as evidence, never asserted as truth.** ``evidence`` carries the IoU,
containment and centre separation that justified it, and a held (not-yet-merged) pair is
recorded too. §41's promise is that every identity decision has an observable cause, and a
merge that happened "because the resolver said so" breaks it.

**The survivor follows a fixed order** (§25.1), with the operator's selection first. Once a
selection points at the merged-away UUID, §25.1 requires it to follow the alias atomically —
the merge must never be the reason the turret stops pointing at the person it was pointing
at, and that is the one property here that a test must not be allowed to regress.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from ..config import DedupConfig, TrackingConfig
from ..tracking.diagnostics import AssociationDiagnostics
from ..tracking.track import Track, TrackState

PairKey = Tuple[str, str]


def _scale_ratio(a, b) -> float:
    area_a, area_b = a.area, b.area
    if area_a <= 0.0 or area_b <= 0.0:
        return float("inf")
    return max(area_a, area_b) / min(area_a, area_b)


def _pair_key(a: str, b: str) -> PairKey:
    """Order-independent key, so ``a->b`` and ``b->a`` are the same pending duplicate."""
    return (a, b) if a <= b else (b, a)


@dataclass
class DuplicatePair:
    """One suspected duplicate pair and the measurements behind the suspicion."""

    a_uuid: str
    b_uuid: str
    iou: float = 0.0
    containment: float = 0.0
    centre_distance: float = 1.0
    scale_ratio: float = 1.0
    first_seen_ns: int = 0
    last_seen_ns: int = 0
    observations: int = 0

    @property
    def age_ms(self) -> float:
        return (self.last_seen_ns - self.first_seen_ns) / 1_000_000.0

    def to_dict(self) -> Dict[str, object]:
        return {"a": self.a_uuid, "b": self.b_uuid, "iou": round(self.iou, 4),
                "containment": round(self.containment, 4),
                "centre_distance": round(self.centre_distance, 4),
                "scale_ratio": round(self.scale_ratio, 3),
                "age_ms": round(self.age_ms, 1),
                "observations": self.observations}


@dataclass
class MergeDecision:
    """A merge to apply. The caller (§ ``TrackManager``) owns the alias and the event."""

    survivor_uuid: str
    merged_uuid: str
    reason: str
    evidence: DuplicatePair
    #: Which §25.1 tie-break settled the survivor, for the audit trail.
    survivor_rule: str = ""


class DuplicateTrackResolver:
    """Watches for persistent duplicates and proposes merges with the evidence attached."""

    def __init__(self, cfg: TrackingConfig, dedup: Optional[DedupConfig] = None,
                 *, diagnostics: Optional[AssociationDiagnostics] = None) -> None:
        self.cfg = cfg
        self.dedup = dedup or DedupConfig()
        self.diagnostics = diagnostics
        self._pending: Dict[PairKey, DuplicatePair] = {}
        self._selected_uuid: Optional[str] = None
        self.pairs_observed = 0
        self.held = 0            # seen, not yet persistent enough to merge
        self.merged = 0
        #: Set when the commissioned thresholds the resolver borrows from §16 are missing.
        self.disabled_reason = ""

    # -- thresholds ---------------------------------------------------------
    def _thresholds(self) -> Optional[Tuple[float, float, float]]:
        """The resolver borrows §16's thresholds: it asks the same question of boxes.

        Two boxes-overlap numbers in one subsystem would disagree about whether a pair is a
        duplicate — the deduplicator would suppress it and the resolver would call it two
        people, which is a contradiction the operator experiences as flicker. With the
        numbers uncommissioned the resolver declines and counts that, rather than running on
        a borrowed default (§50).
        """
        iou = self.dedup.nms_iou
        containment = self.dedup.containment_ratio
        centre = self.dedup.center_distance_norm
        if iou is None or containment is None or centre is None:
            self.disabled_reason = ("duplicate resolver disabled: §16 thresholds "
                                    "(nms_iou/containment_ratio/center_distance_norm) "
                                    "are COMMISSION")
            return None
        return iou, containment, centre

    def duplicate_of_live(self, bbox, class_name: str,
                          tracks: Sequence[Track]) -> Optional[Track]:
        """Is this detection the same person as an identity that already exists?

        The question §25 asks of two tracks, asked of one detection instead — with the same
        numbers, deliberately. It is what stops the merge/re-creation oscillation: when the
        detector keeps emitting a second box for a person who already has an identity, the
        leftover box must not mint a fresh one, or the tracker merges every dwell and mints a
        new label every frame afterwards. §27's monotonic labels make that failure visible as
        a person whose number changes twice a second, which is the exact symptom this
        subsystem was written to remove.

        The same centre-distance and scale terms §16.2 uses guard the cost: two people
        standing one behind the other stay two identities.
        """
        thresholds = self._thresholds()
        if thresholds is None:
            return None
        iou_threshold, containment_threshold, centre_limit = thresholds
        for track in tracks:
            if track.class_name != class_name or track.alias_of:
                continue
            overlapping = (bbox.iou(track.bbox) >= iou_threshold
                           or bbox.containment(track.bbox) >= containment_threshold)
            if not overlapping:
                continue
            if bbox.center.distance_to(track.bbox.center) > centre_limit:
                continue
            if _scale_ratio(bbox, track.bbox) > self.cfg.gates.max_scale_ratio:
                continue
            return track
        return None

    # -- observation --------------------------------------------------------
    def observe(self, tracks: Sequence[Track],
                sensor_timestamp_ns: int) -> List[MergeDecision]:
        """Update the duplicate watch and return the merges that have become justified."""
        thresholds = self._thresholds()
        if thresholds is None:
            return []
        iou_threshold, containment_threshold, centre_limit = thresholds

        live = [track for track in tracks if track.state is not TrackState.RETIRED
                and not track.alias_of]
        seen_now: Dict[PairKey, DuplicatePair] = {}
        decisions: List[MergeDecision] = []

        for index, a in enumerate(live):
            for b in live[index + 1:]:
                if a.class_id != b.class_id:
                    continue                     # different classes are not one person
                pair = self._measure(a, b, sensor_timestamp_ns)
                if pair is None:
                    continue
                iou = a.bbox.iou(b.bbox)
                containment = a.bbox.containment(b.bbox)
                overlapping = iou >= iou_threshold or containment >= containment_threshold
                if not overlapping:
                    continue
                if pair.centre_distance > centre_limit:
                    continue                     # one behind the other, not one of each
                if pair.scale_ratio > self.cfg.gates.max_scale_ratio:
                    continue                     # a person cannot triple in one frame

                previous = self._pending.get(_pair_key(a.track_uuid, b.track_uuid))
                if previous is not None:
                    pair.first_seen_ns = previous.first_seen_ns
                    pair.observations = previous.observations + 1
                else:
                    pair.first_seen_ns = sensor_timestamp_ns
                    pair.observations = 1
                    self.pairs_observed += 1
                seen_now[_pair_key(a.track_uuid, b.track_uuid)] = pair

                age_ms = (sensor_timestamp_ns - pair.first_seen_ns) / 1_000_000.0
                persistent = age_ms >= self.cfg.duplicate_dwell_ms and pair.observations >= 2
                self._record(a, b, pair, persistent)
                if not persistent:
                    self.held += 1
                    continue
                decision = self._decide(a, b, pair)
                decisions.append(decision)
                self.merged += 1

        # A pair that stopped overlapping has to be forgotten, not aged: keeping it would
        # let two people who crossed a minute ago merge on a dwell started then.
        self._pending = seen_now
        return decisions

    def _measure(self, a: Track, b: Track, sensor_timestamp_ns: int) -> Optional[DuplicatePair]:
        area_a, area_b = a.bbox.area, b.bbox.area
        if area_a <= 0.0 or area_b <= 0.0:
            return None
        return DuplicatePair(
            a_uuid=a.track_uuid, b_uuid=b.track_uuid,
            iou=a.bbox.iou(b.bbox),
            containment=a.bbox.containment(b.bbox),
            centre_distance=a.bbox.center.distance_to(b.bbox.center),
            scale_ratio=max(area_a, area_b) / min(area_a, area_b),
            first_seen_ns=sensor_timestamp_ns, last_seen_ns=sensor_timestamp_ns)

    def prefer(self, a: Optional[Track], b: Optional[Track]) -> str:
        """Which of two suspected duplicates should lose its identity, without merging.

        The same §25.1 order the merge uses, exposed for the *temporary suppression* §25
        allows: a pair that has not reached the dwell must not be selectable on the weaker
        side, and the weaker side has to be chosen by the rule that would choose the
        survivor — otherwise the suppressed candidate flickers between two identities.
        """
        if a is None:
            return b.track_uuid if b is not None else ""
        if b is None:
            return a.track_uuid
        return self._decide(a, b, DuplicatePair(a_uuid=a.track_uuid,
                                               b_uuid=b.track_uuid)).merged_uuid

    def _decide(self, a: Track, b: Track, pair: DuplicatePair) -> MergeDecision:
        """§25.1's survivor order, evaluated one rule at a time."""
        selected = self._selected_uuid
        if selected == b.track_uuid:
            # Swap so the selected identity is always the survivor, whichever side it fell.
            a, b = b, a
            pair = DuplicatePair(a_uuid=a.track_uuid, b_uuid=b.track_uuid, iou=pair.iou,
                                 containment=pair.containment,
                                 centre_distance=pair.centre_distance,
                                 scale_ratio=pair.scale_ratio,
                                 first_seen_ns=pair.first_seen_ns,
                                 last_seen_ns=pair.last_seen_ns,
                                 observations=pair.observations)
            rule = "selected_identity"
        elif selected == a.track_uuid:
            rule = "selected_identity"
        elif abs(a.identity_confidence - b.identity_confidence) > 1e-9:
            if a.identity_confidence < b.identity_confidence:
                a, b = b, a
            rule = "higher_identity_confidence"
        elif abs(a.visible_ms_total - b.visible_ms_total) > 1e-9:
            if a.visible_ms_total < b.visible_ms_total:
                a, b = b, a
            rule = "more_visible_time"
        elif a.created_ns != b.created_ns:
            if a.created_ns > b.created_ns:
                a, b = b, a
            rule = "older_identity"
        else:
            if a.display_index > b.display_index:
                a, b = b, a
            rule = "lower_display_index"
        return MergeDecision(survivor_uuid=a.track_uuid, merged_uuid=b.track_uuid,
                             reason="persistent_duplicate_overlap", evidence=pair,
                             survivor_rule=rule)

    def _record(self, a: Track, b: Track, pair: DuplicatePair,
                persistent: bool) -> None:
        if self.diagnostics is None or not self.diagnostics.enabled:
            return
        self.diagnostics.record_merge(
            merged_uuid=b.track_uuid, survivor_uuid=a.track_uuid,
            reason=(f"iou={pair.iou:.2f} containment={pair.containment:.2f} "
                    f"centre={pair.centre_distance:.3f} age_ms={pair.age_ms:.0f}"),
            persistent=persistent, evidence=pair.iou)

    # -- control ------------------------------------------------------------
    def note_selected(self, track_uuid: Optional[str]) -> None:
        """§25.1: the selected identity wins a merge, so the resolver has to know it."""
        self._selected_uuid = track_uuid

    def forget(self, track_uuid: str) -> None:
        """Drop pending pairs involving a retired identity (§25.1's alias cleanup)."""
        for key in [key for key in self._pending
                    if track_uuid in (key[0], key[1])]:
            del self._pending[key]

    def pending_pairs(self) -> List[DuplicatePair]:
        return list(self._pending.values())

    @property
    def disabled(self) -> bool:
        return bool(self.disabled_reason)

    def stats(self) -> Dict[str, object]:
        return {"pairs_observed": self.pairs_observed, "held": self.held,
                "merged": self.merged, "pending": len(self._pending),
                "disabled": bool(self.disabled_reason)}
