"""§15 — class filtering, immediately after normalization.

Filtering happens here, before the tracker, rather than at selection time: a track for a
potted plant costs a slot out of §26's 16-track capacity, appears in the operator's
candidate list, and can be selected. Every one of those consequences is cheaper to avoid
than to undo, and the architecture document lists exactly them (§15's closing list:
tracker load, selector clutter, irrelevant false identities).

The fail-safe direction is stated once, here: **an empty permitted-class list permits
nothing.** A profile with no class semantics — §5's ``SIMPLE_DIAGNOSTIC``, whose blob has
never seen a class — therefore produces no tracks at all. The alternative reading ("empty
means everything") would make a mis-typed or blank configuration the one moment in this
subsystem where arbitrary motion is allowed to become a target.
"""
from __future__ import annotations

from typing import Iterable, Optional, Sequence, Set

from .types import Detection, DetectionSet


def permitted_name_set(permitted: Optional[Iterable[str]]) -> Set[str]:
    return {str(name).strip().lower() for name in (permitted or ()) if str(name).strip()}


def is_permitted(detection: Detection, names: Set[str],
                 ids: Optional[Set[int]] = None) -> bool:
    """Name match (case-insensitive), or id match when the caller supplied ids.

    Names rather than ids as the default because the id map is per-model: COCO's person is
    0, and a profile that swaps in a model with a different table would otherwise silently
    track whatever class 0 turned out to be. §9.3 exists to make that disagreement loud;
    filtering on names keeps it loud here too.
    """
    if not names and not ids:
        return False
    if ids and detection.class_id in ids:
        return True
    return detection.class_name.strip().lower() in names


def filter_permitted(detections: DetectionSet, permitted: Optional[Sequence[str]], *,
                     permitted_ids: Optional[Sequence[int]] = None) -> DetectionSet:
    """Drop everything the profile does not permit and COUNT what was dropped (§16.4)."""
    names = permitted_name_set(permitted)
    ids = set(permitted_ids) if permitted_ids else None
    kept = []
    dropped = 0
    for detection in detections.detections:
        if is_permitted(detection, names, ids):
            kept.append(detection)
        else:
            dropped += 1
    out = detections.with_detections(kept)
    # Cumulative within this DetectionSet, not overwritten: §13's counters describe the
    # set as published, and a stage that replaced the running total would erase the
    # evidence that anything was filtered at all.
    out.counters.class_filtered = int(detections.counters.class_filtered) + dropped
    return out
