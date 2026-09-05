"""§16 — the host-side deduplication pipeline, and the counters that make it auditable.

Three stages, in the order the evidence gets more specific:

1. **class-aware NMS** (§16.1) — overlapping boxes of one class;
2. **containment** (§16.2) — nested boxes of one class, which IoU can score as a
   disagreement rather than a duplicate;
3. **pose NMS** (§16.3) — two skeletons describing one body, only for profiles that carry
   keypoints.

The stage that a generic implementation would skip is the third one's precondition, and
the rule the whole module obeys is §16.4's: **no silent duplicate removal**. Every removal
is counted per method and emitted as a ``DETECTION_DUPLICATE_SUPPRESSED`` event with the
measure that justified it. That is what turns "the tracker sees one person, I see two"
from an argument into a query.

The other rule is §50's. When a stage's threshold is still the literal ``COMMISSION``
placeholder, the stage does not run and the result says so in ``stages_skipped``. A dedup
stage silently disabled is a duplicate-detection problem wearing the costume of a working
pipeline — and the duplicate then shows up downstream as a second identity, where it costs
far more to diagnose than this one line of reporting.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from ..config import AnchorConfig, DedupConfig
from ..events import EventLog, EventType
from .containment import containment_suppression
from .nms import SuppressedPair, class_aware_nms
from .pose_nms import pose_nms
from .types import DetectionSet


@dataclass
class DedupResult:
    """The deduplicated set plus the accounting for what was removed (§16.4)."""

    detections: DetectionSet
    suppressed: List[SuppressedPair] = field(default_factory=list)
    #: Stages that could not run because §50's placeholder is still unresolved, or because
    #: the frame gave them nothing to work with. Named, never empty-by-omission.
    stages_skipped: List[str] = field(default_factory=list)

    def summary(self) -> Dict[str, int]:
        return {
            "in": int(self.detections.counters.post_model_nms),
            "out": len(self.detections.detections),
            "suppressed_total": len(self.suppressed),
            **{key: value for key, value in self.detections.counters.to_dict().items()
               if "suppressed" in key},
        }


class DetectionDeduplicator:
    """The §16 host layer: defensive, ordered, and loud about what it removed."""

    def __init__(self, cfg: DedupConfig, *, anchor_cfg: Optional[AnchorConfig] = None,
                 event_log: Optional[EventLog] = None) -> None:
        self.cfg = cfg
        self.anchor_cfg = anchor_cfg or AnchorConfig()
        self.events = event_log
        self.last_result: Optional[DedupResult] = None

    def run(self, dset: DetectionSet) -> DedupResult:
        """Suppress duplicates without touching the caller's set."""
        out = dset.with_detections(dset.detections)     # our copy, our counters
        pairs: List[SuppressedPair] = []
        skipped: List[str] = []
        detections = list(dset.detections)

        detections, stage_pairs, skipped_reason = self._run_nms(detections)
        if skipped_reason:
            skipped.append(skipped_reason)
        pairs.extend(stage_pairs)
        out.counters.host_duplicates_suppressed = (
            int(dset.counters.host_duplicates_suppressed) + _count_method(stage_pairs, "nms"))

        detections, stage_pairs, skipped_reason = self._run_containment(detections)
        if skipped_reason:
            skipped.append(skipped_reason)
        pairs.extend(stage_pairs)
        out.counters.containment_suppressed = (
            int(dset.counters.containment_suppressed)
            + _count_method(stage_pairs, "containment"))

        if self.cfg.pose_nms and any(d.has_pose for d in detections):
            detections, stage_pairs, skipped_reason = self._run_pose(detections)
            if skipped_reason:
                skipped.append(skipped_reason)
            pairs.extend(stage_pairs)
            out.counters.pose_duplicates_suppressed = (
                int(dset.counters.pose_duplicates_suppressed)
                + _count_method(stage_pairs, "pose"))

        out.detections = detections
        for pair in pairs:
            self._emit(dset, pair)

        result = DedupResult(detections=out, suppressed=pairs, stages_skipped=skipped)
        self.last_result = result
        return result

    # -- stages -------------------------------------------------------------
    def _run_nms(self, detections):
        if not self.cfg.class_aware_nms:
            return detections, [], ""
        if self.cfg.nms_iou is None:
            # §16.1's threshold is commissioned from the station's own recordings. The
            # alternative — a 0.45 out of a demo — is what makes a model swap look like a
            # tracker regression, because the suppression silently changed underneath it.
            return detections, [], "class_aware_nms: nms_iou is COMMISSION"
        kept, pairs = class_aware_nms(detections, self.cfg.nms_iou)
        return kept, pairs, ""

    def _run_containment(self, detections):
        if not self.cfg.containment:
            return detections, [], ""
        if self.cfg.containment_ratio is None or self.cfg.center_distance_norm is None:
            return detections, [], "containment: containment_ratio/center_distance_norm are COMMISSION"
        kept, pairs = containment_suppression(detections, self.cfg)
        return kept, pairs, ""

    def _run_pose(self, detections):
        if self.cfg.nms_iou is None:
            return detections, [], "pose_nms: nms_iou is COMMISSION"
        kept, pairs = pose_nms(detections, self.cfg.nms_iou, self.cfg.pose_oks_sigma)
        return kept, pairs, ""

    # -- reporting ----------------------------------------------------------
    def _emit(self, dset: DetectionSet, pair: SuppressedPair) -> None:
        if self.events is None:
            return
        self.events.emit(
            EventType.DETECTION_DUPLICATE_SUPPRESSED,
            sensor_timestamp_ns=dset.sensor_timestamp_ns,
            frame_sequence=dset.frame_sequence,
            method=pair.method,
            measure=round(float(pair.measure), 4),
            kept_id=pair.kept.detection_id_in_frame,
            kept_score=round(float(pair.kept.detector_score), 4),
            dropped_id=pair.dropped.detection_id_in_frame,
            dropped_score=round(float(pair.dropped.detector_score), 4),
            model_id=dset.model_id,
        )


def _count_method(pairs: List[SuppressedPair], method: str) -> int:
    return sum(1 for pair in pairs if pair.method == method)
