"""§23 — camera motion compensation, pluggable and optional.

A neural detector stays semantically valid while the turret moves: a person is still a
person in every frame of a pan. What breaks under motion is *association*, because every
box jumps at once and the predicted-position gate that §21 relies on was written for the
motion of targets, not of cameras. This module exists to move the camera's contribution out
of the prediction and into a separate, explicit quantity.

The interface deliberately contains no motor brand, no CAN id and no drive mode (§23.2).
The vision subsystem must also run with the provider absent — §23's rule is that camera
motion may never be *mistaken* for a semantic detection, not that it must be compensated.
A hint that is stale, oversized or unsupported therefore declines to compensate at all,
because a wrong compensation manufactures motion on every track at once: better one
fragmented identity than a system that confidently drags all of them sideways.

Sign convention, stated once: the shift returned here is what must be ADDED to a track's
predicted position to express where the same scene point now lands. A camera that yaws
right moves the scene left, so a positive ``delta_yaw`` produces a negative ``x``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from ..config import CameraMotionConfig, CameraMotionProviderName
from ..detection.types import PointNorm, is_finite
from ..errors import ConfigError
from ..measure import ms_from_ns

#: A hint that would move the scene by more than this (in normalized image units) is
#: treated as invalid rather than applied. Real turret motion between two inference
#: results is a few degrees; a hint of ninety is a torn packet or a unit mismatch.
MAX_SHIFT_NORM = 0.75


@dataclass
class CameraMotionHint:
    """§23.2's hint: a timestamp and two deltas. Nothing else."""

    timestamp_ns: int = 0
    delta_yaw_rad: float = 0.0
    delta_pitch_rad: float = 0.0
    receive_timestamp_ns: int = 0

    def is_valid(self) -> bool:
        return (self.timestamp_ns > 0 and is_finite(self.delta_yaw_rad)
                and is_finite(self.delta_pitch_rad))


class CameraMotionProvider:
    """Anything that can say how far the scene moved between two captures."""

    name = "none"

    def shift_norm(self, sensor_timestamp_ns: int, now_ns: int) -> Optional[PointNorm]:
        """The scene shift to add to a predicted anchor, or ``None`` for "do not"."""
        raise NotImplementedError

    def reset(self) -> None:
        """Forget accumulated motion (model reload, replay restart, stream gap)."""


class NoCameraMotion(CameraMotionProvider):
    """§23.3's baseline: stationary-camera association, no compensation whatsoever."""

    name = CameraMotionProviderName.NONE.value

    def shift_norm(self, sensor_timestamp_ns: int, now_ns: int) -> Optional[PointNorm]:
        return None

    def reset(self) -> None:
        return None


class ExternalPoseHintProvider(CameraMotionProvider):
    """§23.2: consume generic yaw/pitch deltas from whatever produces them.

    The producer is not this subsystem's business — that is the point of the interface. The
    provider accumulates deltas and hands the whole accumulated shift to the next
    association, because the association runs at inference cadence while pose updates run
    faster: dropping the deltas that arrived between two inferences would under-compensate
    by exactly the amount the operator can see.

    It declines (returns ``None``) when:
      * no hint has arrived since the last consumption — nothing is known;
      * the newest hint is older than ``hint_max_age_ms`` — a stale delta describes a move
        that has already been compensated, and applying it twice is how a track is dragged
        off its target by a dead encoder;
      * ``focal_px`` is not commissioned — without a focal length a yaw in radians cannot
        become a shift in image fractions at all, and guessing one would silently invent
        the station's optics (§50).
    """

    name = CameraMotionProviderName.EXTERNAL_POSE_HINT.value

    def __init__(self, cfg: CameraMotionConfig, *, stream_width: int,
                 stream_height: int) -> None:
        self.cfg = cfg
        self.stream_width = int(stream_width)
        self.stream_height = int(stream_height)
        self._pending_yaw = 0.0
        self._pending_pitch = 0.0
        self._newest_hint_ns = 0
        self.hints_received = 0
        self.hints_declined_stale = 0
        self.hints_declined_no_focal = 0

    def push_hint(self, hint: CameraMotionHint) -> bool:
        """Queue one delta. Returns False when the hint was unusable (and counted)."""
        if not hint.is_valid():
            return False
        if hint.receive_timestamp_ns <= 0:
            return False
        self._pending_yaw += float(hint.delta_yaw_rad)
        self._pending_pitch += float(hint.delta_pitch_rad)
        self._newest_hint_ns = max(self._newest_hint_ns, int(hint.receive_timestamp_ns))
        self.hints_received += 1
        return True

    def shift_norm(self, sensor_timestamp_ns: int, now_ns: int) -> Optional[PointNorm]:
        if self._newest_hint_ns <= 0:
            return None
        age_ms = ms_from_ns(now_ns, self._newest_hint_ns)
        if age_ms > self.cfg.hint_max_age_ms:
            self.hints_declined_stale += 1
            self.reset()
            return None
        if self.cfg.focal_px is None or self.cfg.focal_px <= 0.0:
            self.hints_declined_no_focal += 1
            self.reset()
            return None
        shift = self._to_shift(self._pending_yaw, self._pending_pitch)
        self.reset()
        return shift

    def _to_shift(self, yaw: float, pitch: float) -> Optional[PointNorm]:
        focal = float(self.cfg.focal_px)
        # tan(), not the angle itself: the image-plane displacement of a rotating pinhole
        # is focal·tan(angle). The small-angle approximation they share under ~10° is why
        # the difference is invisible in a demo and wrong at the end of a fast pan.
        dx = -math.tan(yaw) * focal / max(self.stream_width, 1)
        dy = math.tan(pitch) * focal / max(self.stream_height, 1)
        if not (is_finite(dx) and is_finite(dy)):
            return None
        if abs(dx) > MAX_SHIFT_NORM or abs(dy) > MAX_SHIFT_NORM:
            # Oversized: refuse the whole hint rather than clamping one axis. A clamped
            # shift is a partial compensation, which splits identity worse than none.
            return None
        return PointNorm(dx, dy)

    def reset(self) -> None:
        self._pending_yaw = 0.0
        self._pending_pitch = 0.0
        self._newest_hint_ns = 0


class ImageGmcProvider(CameraMotionProvider):
    """§23.1's sparse-flow option: declared, not implemented in this revision.

    Building it is §51's Vision-7 step, after panning and roaming recordings exist — the
    method (optical flow versus affine versus homography), the cost, and the
    person-region exclusion all have to be chosen against measured CPU numbers on this
    station, not guessed at. Refusing loudly at construction is the honest failure: a
    provider that quietly returned ``None`` would leave the configuration claiming
    compensation it never applied.
    """

    name = CameraMotionProviderName.IMAGE_GMC.value

    def __init__(self, cfg: CameraMotionConfig) -> None:
        raise ConfigError(
            "camera_motion.provider=image_gmc is not implemented in Vision 1.0 (§23.1 is "
            "Vision-7 work). Use provider=none or provider=external_pose_hint.")

    def shift_norm(self, sensor_timestamp_ns: int, now_ns: int) -> Optional[PointNorm]:
        raise NotImplementedError("see __init__: IMAGE_GMC is Vision-7 scope")

    def reset(self) -> None:
        return None


def build_camera_motion_provider(cfg: CameraMotionConfig, *, stream_width: int,
                                 stream_height: int) -> CameraMotionProvider:
    if cfg.provider is CameraMotionProviderName.NONE:
        return NoCameraMotion()
    if cfg.provider is CameraMotionProviderName.EXTERNAL_POSE_HINT:
        return ExternalPoseHintProvider(cfg, stream_width=stream_width,
                                        stream_height=stream_height)
    return ImageGmcProvider(cfg)


def accumulated_hint_debug(provider: CameraMotionProvider) -> Tuple[float, float, int]:
    """(pending yaw, pending pitch, hints received) for diagnostics and tests."""
    if isinstance(provider, ExternalPoseHintProvider):
        return provider._pending_yaw, provider._pending_pitch, provider.hints_received
    return 0.0, 0.0, 0
