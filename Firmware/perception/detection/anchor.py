"""§35 / §12.1 — where the aim anchor sits, and which evidence decided it.

The identity box and the aim anchor are separate things (§35). The centre of a full-body
box is roughly at the hips or the thighs — a point that is both unstable (it moves with
stride) and unmotivated (it is not what an operator means when they point at a person).
§35's initial candidate is 0.42–0.48 of the box height from its top: upper torso, which is
steadier under a partial occlusion of the legs and stays inside the body when the person
turns. §12.1's keypoint priority replaces the guessed fraction with a measured shoulder
line whenever a pose profile can supply one.

Publishing ``anchor_source`` alongside the point is the other half of this module. A torso
anchor derived from two confident shoulder keypoints and a box-centre fallback do not have
the same expected jitter, and an operator or a downstream filter cannot tell them apart
unless the pipeline says which one it used.

Clamping is deliberate and limited: an anchor outside its own box means the keypoint and
the box disagree, and the *box* is what the association and the aim geometry are built on.
The source is left as the pose value, because "the anchor is a clamped shoulder midpoint"
is more informative than a silent downgrade to a box centre.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

from ..config import AnchorConfig
from .types import AnchorSource, BBox, Keypoint, PointNorm

# COCO-17 skeleton indices. Named, because "keypoints[5]" appears in three places in this
# file and an off-by-one there puts the aim anchor on an elbow.
COCO_SHOULDER_LEFT = 5
COCO_SHOULDER_RIGHT = 6
COCO_HIP_LEFT = 11
COCO_HIP_RIGHT = 12

_MIN_INDEXED_KEYPOINTS = COCO_HIP_RIGHT + 1


def _confident(keypoints: Sequence[Keypoint], index: int,
               min_score: float) -> Optional[Keypoint]:
    if index >= len(keypoints):
        return None
    keypoint = keypoints[index]
    return keypoint if keypoint.score >= min_score else None


def _weighted_midpoint(a: Keypoint, b: Keypoint) -> PointNorm:
    """§12.1's "use confidence-weighted keypoints".

    A keypoint the model half-believes pulls the midpoint less than one it is sure about,
    which is what weighting is for. Both scores zero cannot happen here — the callers only
    reach this with at least one confident endpoint — but the guard returns the confident
    side rather than dividing by zero.
    """
    weight_sum = a.score + b.score
    if weight_sum <= 0.0:
        return PointNorm(a.x if a.score >= b.score else b.x,
                         a.y if a.score >= b.score else b.y)
    return PointNorm((a.x * a.score + b.x * b.score) / weight_sum,
                     (a.y * a.score + b.y * b.score) / weight_sum)


def _midpoint(a: PointNorm, b: PointNorm) -> PointNorm:
    return PointNorm((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)


def _clamp_to_box(anchor: PointNorm, bbox: BBox) -> PointNorm:
    return PointNorm(min(max(anchor.x, bbox.x_min), bbox.x_max),
                     min(max(anchor.y, bbox.y_min), bbox.y_max))


def box_anchor(bbox: BBox, cfg: AnchorConfig) -> Tuple[PointNorm, AnchorSource]:
    """The box half of the policy: torso fraction, or centre when the box cannot be trusted."""
    if not bbox.is_well_formed():
        return bbox.center, AnchorSource.BBOX_CENTER_FALLBACK
    anchor = bbox.anchor_at_height(cfg.torso_fraction)
    if not anchor.is_valid():
        return bbox.center, AnchorSource.BBOX_CENTER_FALLBACK
    return anchor, AnchorSource.BBOX_TORSO


def compute_anchor(bbox: BBox, keypoints: Sequence[Keypoint],
                   cfg: AnchorConfig) -> Tuple[PointNorm, AnchorSource]:
    """§12.1's priority order: shoulders → shoulder/hip midpoint → hips → box.

    Each step down is chosen only when the step above it has no confident evidence — not
    when it disagrees. A pose profile that confidently places one shoulder and not the
    other uses the weighted midpoint of what it has; a profile that places neither falls
    back to the box, and says so in ``anchor_source``.
    """
    if not cfg.use_pose_anchors or len(keypoints) < _MIN_INDEXED_KEYPOINTS:
        return box_anchor(bbox, cfg)

    min_score = cfg.min_keypoint_score
    shoulder_left = _confident(keypoints, COCO_SHOULDER_LEFT, min_score)
    shoulder_right = _confident(keypoints, COCO_SHOULDER_RIGHT, min_score)
    hip_left = _confident(keypoints, COCO_HIP_LEFT, min_score)
    hip_right = _confident(keypoints, COCO_HIP_RIGHT, min_score)

    shoulder_centre: Optional[PointNorm] = None
    if shoulder_left is not None and shoulder_right is not None:
        shoulder_centre = _weighted_midpoint(shoulder_left, shoulder_right)
        return _clamp_to_box(shoulder_centre, bbox), AnchorSource.POSE_SHOULDERS
    if shoulder_left is not None or shoulder_right is not None:
        # One shoulder only: still the upper-torso line, biased to the side that was seen.
        sole = shoulder_left or shoulder_right
        assert sole is not None
        shoulder_centre = PointNorm(sole.x, sole.y)

    hip_centre: Optional[PointNorm] = None
    if hip_left is not None and hip_right is not None:
        hip_centre = _weighted_midpoint(hip_left, hip_right)
    elif hip_left is not None or hip_right is not None:
        sole = hip_left or hip_right
        assert sole is not None
        hip_centre = PointNorm(sole.x, sole.y)

    if shoulder_centre is not None and hip_centre is not None:
        return _clamp_to_box(_midpoint(shoulder_centre, hip_centre),
                             bbox), AnchorSource.POSE_TORSO
    if hip_centre is not None:
        return _clamp_to_box(hip_centre, bbox), AnchorSource.POSE_TORSO
    if shoulder_centre is not None:
        # Hips absent but a shoulder seen: the shoulder line is still a better torso point
        # than a guessed fraction of a box, and it is measured rather than assumed.
        return _clamp_to_box(shoulder_centre, bbox), AnchorSource.POSE_SHOULDERS
    return box_anchor(bbox, cfg)
