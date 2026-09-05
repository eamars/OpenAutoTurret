"""§16.3 — pose-aware duplicate suppression (§12.2).

Two pose outputs can describe the same skeleton while their boxes disagree, because a box
is a loose wrapper around a body and a skeleton is the body. Object Keypoint Similarity
(OKS) is therefore a *better* duplicate test than IoU for a pose profile: it is scale
normalization-aware and it does not care whether one head drew the box a little wide.

Scale normalization is the part that has to be right. The deviation of each keypoint pair
is divided by ``(2 · sigma · diagonal)²`` of the *pair's combined* extent, so a two-pixel
disagreement on a person across the room counts for the same amount as a two-pixel
disagreement on a person at the edge of the frame. Normalize by anything smaller and every
far-away duplicate survives.

This module is only reached when a profile actually carries keypoints (§12.3 lists when it
should not), which is why the caller checks before paying for it.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .nms import SuppressedPair
from .types import Detection, Keypoint


def keypoint_set_diagonal(sets: Sequence[Sequence[Keypoint]]) -> float:
    """Extent of the union of the given skeletons, in normalized image units."""
    xs: List[float] = []
    ys: List[float] = []
    for keypoints in sets:
        for keypoint in keypoints:
            if keypoint.score > 0.0:
                xs.append(keypoint.x)
                ys.append(keypoint.y)
    if len(xs) < 2:
        return 0.0
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def object_keypoint_similarity(a: Sequence[Keypoint], b: Sequence[Keypoint],
                               diagonal: float, sigma: float = 0.10) -> float:
    """OKS of two skeletons in [0,1]; 1.0 is the same skeleton, 0.0 is unrelated.

    Only keypoints confident on BOTH sides contribute. A keypoint the model refused to
    place on one skeleton carries no evidence either way, and counting it as a mismatch
    would suppress a genuine duplicate every time the model is uncertain — which is the
    opposite of what a duplicate-suppression rule is for.
    """
    if not a or not b or diagonal <= 0.0 or sigma <= 0.0:
        return 0.0
    pairs = min(len(a), len(b))
    denominator = 2.0 * (sigma * diagonal) ** 2
    if denominator <= 0.0:
        return 0.0
    total = 0.0
    counted = 0
    for ka, kb in zip(a[:pairs], b[:pairs]):
        if ka.score <= 0.0 or kb.score <= 0.0:
            continue
        d2 = (ka.x - kb.x) ** 2 + (ka.y - kb.y) ** 2
        total += math.exp(-d2 / denominator)
        counted += 1
    return total / counted if counted else 0.0


def pose_nms(detections: Sequence[Detection], iou_threshold: float,
             sigma: float = 0.10) -> Tuple[List[Detection], List[SuppressedPair]]:
    """Keep one description per skeleton, highest score first (§12.2's list of evidence).

    A candidate is suppressed when its OKS against an already-kept detection clears the
    same commissioned IoU knob (§16.1's threshold is a "these are the same object"
    threshold regardless of which measurement expressed it). Detections without keypoints
    pass through untouched: a mixed frame — a pose head plus a fallback box — must not lose
    the box merely because it has no skeleton to compare.
    """
    if not 0.0 < iou_threshold <= 1.0:
        raise ValueError(f"pose NMS threshold must be in (0,1], got {iou_threshold!r}")

    ordered = sorted((d for d in detections if d.has_pose),
                     key=lambda d: (-d.detector_score, d.detection_id_in_frame))
    plain = [d for d in detections if not d.has_pose]

    kept: List[Detection] = []
    suppressed: List[SuppressedPair] = []
    for candidate in ordered:
        winner = None
        best = 0.0
        for existing in kept:
            diagonal = keypoint_set_diagonal((candidate.keypoints, existing.keypoints))
            oks = object_keypoint_similarity(candidate.keypoints, existing.keypoints,
                                             diagonal, sigma)
            if oks >= iou_threshold and oks >= best:
                winner, best = existing, oks
        if winner is None:
            kept.append(candidate)
        else:
            suppressed.append(SuppressedPair(kept=winner, dropped=candidate,
                                             method="pose", measure=best))

    result = kept + plain
    result.sort(key=lambda d: d.detection_id_in_frame)
    return result, suppressed
