"""What the turret follows: HUMAN, HEAD, CAT or DOG (owner, 2026-10-20; docs/DESIGN_TARGET_TYPE_2026-10-20.md).

One type at a time. Each names the model profile that sees it, the classes kept (everything else is
dropped before tracking, so it is neither listed, overlaid nor selectable) and the aim point. HUMAN and
HEAD share the pose model and differ only in the aim; CAT and DOG share the COCO-80 objects model.

The wire carries the APPLIED type as a small code (protocol/native_wire.py, flag bits 2-4); 0 means "no
type applied" (a pinned developer profile, or a publisher from before this existed).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class TargetType:
    name: str                  # wire/UI name, upper case
    code: int                  # native header flag bits 2-4 (1..7; 0 = none)
    profile: str               # perception_v1.json vision.models key
    classes: Tuple[str, ...]   # kept classes, by label name
    anchor: str                # AnchorConfig.target: "torso" | "head"


TARGET_TYPES: Dict[str, TargetType] = {t.name: t for t in (
    TargetType("HUMAN", 1, "hailo_yolov8s_pose", ("person",), "torso"),
    TargetType("HEAD", 2, "hailo_yolov8s_pose", ("person",), "head"),
    # The objects model has no keypoints: "torso" is then the box at its torso fraction (0.45 of the
    # height from the top), about the middle of an animal's body.
    TargetType("CAT", 3, "hailo_yolov8n", ("cat",), "torso"),
    TargetType("DOG", 4, "hailo_yolov8n", ("dog",), "torso"),
)}

#: Today's behaviour (the pose profile aiming at the head) is the default (owner, 2026-10-20).
DEFAULT_TARGET_TYPE = "HEAD"

BY_CODE: Dict[int, TargetType] = {t.code: t for t in TARGET_TYPES.values()}


def target_type(name: Optional[str]) -> Optional[TargetType]:
    """The type for a name, case-insensitively; None for anything else (never a guess)."""
    return TARGET_TYPES.get(str(name or "").strip().upper())


def code_of(name: Optional[str]) -> int:
    t = target_type(name)
    return t.code if t else 0
