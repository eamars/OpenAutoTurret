"""§16.2 — containment suppression: the duplicate IoU cannot see.

Two boxes can describe one person with only a moderate IoU when one is nested inside the
other — a tight box on the torso inside a loose box on the whole body, which is exactly
what an anchor-based head and a body head disagree about. IoU punishes that pair for
disagreeing about size and therefore keeps both boxes, and both boxes reach
``TrackManager`` as two candidate identities for one human being.

``intersection / min(area)`` asks the question instead: how much of the *smaller* box is
explained by the larger one.

The condition that is easy to leave out is the centre distance, and leaving it out is the
dangerous version. A person standing directly behind another produces a near-perfect
containment with a large centre separation only when the depths differ; at close depth
difference the boxes are almost concentric, and a rule that merges on containment alone
would delete one of the two people — silently replacing "two candidates for one person"
with "one candidate for two people". Losing a person is worse than duplicating one, so the
centre-distance and scale-ratio conditions are load-bearing, not refinement.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

from ..config import DedupConfig
from .nms import SuppressedPair
from .types import Detection


def _centre_distance(a: Detection, b: Detection) -> float:
    return a.bbox.center.distance_to(b.bbox.center)


def _scale_ratio(a: Detection, b: Detection) -> float:
    """Larger area over smaller area: 1.0 identical, 4.0 means four times the pixels."""
    area_a, area_b = a.bbox.area, b.bbox.area
    if area_a <= 0.0 or area_b <= 0.0:
        return float("inf")
    return max(area_a, area_b) / min(area_a, area_b)


def is_contained_duplicate(a: Detection, b: Detection,
                           cfg: DedupConfig) -> Tuple[bool, float]:
    """True when ``a`` and ``b`` are one physical object described twice.

    All four of §16.2's conditions, with the class test first because it is free: equal
    class, containment at or above the commissioned ratio, centres close enough that the
    pair cannot be two people at different depths, and a scale relationship that a single
    body could actually produce.
    """
    if a.class_id != b.class_id:
        return False, 0.0
    if cfg.containment_ratio is None or cfg.center_distance_norm is None:
        return False, 0.0
    containment = a.bbox.containment(b.bbox)
    if containment < cfg.containment_ratio:
        return False, containment
    if _centre_distance(a, b) > cfg.center_distance_norm:
        return False, containment
    if _scale_ratio(a, b) > cfg.containment_max_scale_ratio:
        return False, containment
    return True, containment


def containment_suppression(detections: Sequence[Detection],
                            cfg: DedupConfig) -> Tuple[List[Detection], List[SuppressedPair]]:
    """Drop the lower-quality box of each contained pair, in a deterministic pass.

    Score-descending like §16.1, and for the same reason: which box survives has to be
    decided by evidence, not by the order the model emitted rows in.
    """
    if not cfg.containment:
        return list(detections), []

    ordered = sorted(detections,
                     key=lambda d: (-d.detector_score, d.detection_id_in_frame))
    kept: List[Detection] = []
    suppressed: List[SuppressedPair] = []
    for candidate in ordered:
        winner = None
        best = 0.0
        for existing in kept:
            duplicate, containment = is_contained_duplicate(candidate, existing, cfg)
            if duplicate and containment >= best:
                winner, best = existing, containment
        if winner is None:
            kept.append(candidate)
        else:
            suppressed.append(SuppressedPair(kept=winner, dropped=candidate,
                                             method="containment", measure=best))
    kept.sort(key=lambda d: d.detection_id_in_frame)
    return kept, suppressed
