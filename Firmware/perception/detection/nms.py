"""§16.1 — class-aware non-maximum suppression on the host.

The on-sensor ``_pp`` output and ``nms=True`` exports are preferred (§16's opening
sentence), and this layer still runs. That is not belt-and-braces theatre: the bake-off
(§7) compares models whose packaged post-processing differs, and a profile whose
on-sensor NMS is absent or wrong would otherwise feed every duplicate box straight into
identity formation, where a duplicate becomes a *second person* — the exact defect this
document is titled after.

Two rules:

**Suppression is per class.** Two boxes of different classes over the same pixels are a
disagreement about what is there, not a duplicate of one thing. Suppressing across
classes turns "the model is undecided between person and dog" into "the model saw a
person", which is a confidence the detector never had.

**The threshold is commissioned, not imported.** §16.1: "Do not copy an arbitrary IoU
threshold from a generic demo." This function therefore takes the number as an argument
and the caller (``dedup.py``) refuses to run the stage when §50's placeholder is still
unresolved.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

from .types import Detection


@dataclass
class SuppressedPair:
    """One removal, with the number that justified it (§16.4: no silent removal).

    ``method`` is ``"nms"``, ``"containment"`` or ``"pose"``, so a station can tell which
    suppression rule is doing the work — a duplicate rate that rises after a model swap is
    a threshold problem if it shows up in ``nms`` and a packaging problem if it shows up
    in ``containment``.
    """

    kept: Detection
    dropped: Detection
    method: str
    measure: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "method": self.method,
            "measure": round(float(self.measure), 5),
            "kept": {"id": self.kept.detection_id_in_frame,
                     "class": self.kept.class_name,
                     "score": round(float(self.kept.detector_score), 4),
                     "bbox": self.kept.bbox.to_dict()},
            "dropped": {"id": self.dropped.detection_id_in_frame,
                        "class": self.dropped.class_name,
                        "score": round(float(self.dropped.detector_score), 4),
                        "bbox": self.dropped.bbox.to_dict()},
        }


def class_aware_nms(detections: Sequence[Detection],
                    iou_threshold: float) -> Tuple[List[Detection], List[SuppressedPair]]:
    """Greedy score-descending suppression, independently per class.

    Ties break on ``detection_id_in_frame`` rather than on arrival order: the same frame
    replayed from a recording (§43) must suppress the same boxes, or a tracker tuned on a
    replay is tuned on a different input than the live run produced.
    """
    if not 0.0 < iou_threshold <= 1.0:
        raise ValueError(f"nms_iou must be in (0,1], got {iou_threshold!r}")

    by_class: Dict[int, List[Detection]] = {}
    order: List[int] = []
    for detection in detections:
        if detection.class_id not in by_class:
            by_class[detection.class_id] = []
            order.append(detection.class_id)
        by_class[detection.class_id].append(detection)

    kept: List[Detection] = []
    suppressed: List[SuppressedPair] = []
    for class_id in order:
        group = sorted(by_class[class_id],
                       key=lambda d: (-d.detector_score, d.detection_id_in_frame))
        accepted: List[Detection] = []
        for candidate in group:
            winner = None
            best_iou = 0.0
            for existing in accepted:
                iou = candidate.bbox.iou(existing.bbox)
                if iou >= iou_threshold and (winner is None or iou > best_iou):
                    winner, best_iou = existing, iou
            if winner is None:
                accepted.append(candidate)
            else:
                suppressed.append(SuppressedPair(kept=winner, dropped=candidate,
                                                 method="nms", measure=best_iou))
        kept.extend(accepted)

    kept.sort(key=lambda d: d.detection_id_in_frame)
    return kept, suppressed
