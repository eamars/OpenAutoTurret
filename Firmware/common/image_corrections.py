"""Install-level camera corrections, applied at capture — BEFORE any processing
or control (doc §42.3 for the preview, §5.1 for the vision/control pipeline).

The IMX500 is physically mounted in a fixed orientation (this install: upside
down) and its colour balance is a fixed offset. Both are properties of the
*install*, not of any one consumer, so the correction is applied at the earliest
point in every pipeline that touches the camera:

  * webd low-priority video preview  ->  web/webd/video.py        (image + colour)
  * vision daemon control pipeline   ->  vision/frame_source.py   (detection boxes)

Keeping the transform in ONE place guarantees the preview, the detector input and
the control-loop geometry all agree on the same corrected image — the preview can
never drift out of sync with what the control loop sees.
"""
from __future__ import annotations

from typing import Sequence, Tuple

# Orientation of the mounted sensor relative to the world. "none" is the safe,
# portable default; the physical install selects the correction it needs.
ORIENTATIONS: Tuple[str, ...] = (
    "none",
    "rotate_180",       # camera mounted upside-down (top<->bottom AND left<->right)
    "flip_horizontal",  # mirror left/right
    "flip_vertical",    # mirror top/bottom
)

# White-balance mode: "off" = pass through, "auto" = per-frame gray-world balance.
WHITE_BALANCES: Tuple[str, ...] = ("off", "auto")


def validate_orientation(orientation: str) -> str:
    if orientation not in ORIENTATIONS:
        raise ValueError(f"orientation must be one of {ORIENTATIONS}, got {orientation!r}")
    return orientation


def validate_white_balance(mode: str) -> str:
    if mode not in WHITE_BALANCES:
        raise ValueError(f"white-balance mode must be one of {WHITE_BALANCES}, got {mode!r}")
    return mode


def apply_orientation_image(arr, orientation: str):
    """Return ``arr`` (an (H, W, C) image) re-oriented per ``orientation``.

    Returns a strided *view* (no data copy) so it is cheap and happens before any
    downstream processing. Works for any array with the channel on the last axis
    (e.g. the IMX500 XBGR8888 main stream, or an extracted RGB image).
    """
    validate_orientation(orientation)
    if orientation == "none":
        return arr
    if orientation == "rotate_180":
        return arr[::-1, ::-1, :]
    if orientation == "flip_horizontal":
        return arr[:, ::-1, :]
    if orientation == "flip_vertical":
        return arr[::-1, :, :]
    raise ValueError(orientation)  # pragma: no cover - guarded by validate


def apply_orientation_bbox(bbox: Sequence[float], orientation: str,
                           width: int, height: int) -> Tuple[float, float, float, float]:
    """Map a ``[x_min, y_min, x_max, y_max]`` box to the corrected image frame.

    The IMX500 detector reports boxes in the RAW sensor frame. When the install
    orientation corrects that frame, the control-loop geometry (LOS -> joint) only
    agrees if the boxes are corrected the same way, so control aims at the right
    place.
    """
    validate_orientation(orientation)
    x0, y0, x1, y1 = (float(v) for v in bbox)
    if orientation == "none":
        return (x0, y0, x1, y1)
    if orientation == "rotate_180":
        return (width - x1, height - y1, width - x0, height - y0)
    if orientation == "flip_horizontal":
        return (width - x1, y0, width - x0, y1)
    if orientation == "flip_vertical":
        return (x0, height - y1, x1, height - y0)
    raise ValueError(orientation)  # pragma: no cover - guarded by validate


def gray_world_gains(rgb, lo: float = 0.25, hi: float = 4.0) -> Tuple[float, float, float]:
    """Per-channel (R, G, B) gains that balance ``rgb`` to neutral (gray-world).

    ``rgb`` is an (H, W, 3) array. The gain for each channel scales its mean to
    the overall mean, so a neutral image is (a no-op, ~1.0 each) while a cast is
    removed. Gains are clamped to ``[lo, hi]`` so a near-black channel is not
    amplified into noise and a saturated channel is not crushed.
    """
    import numpy as np  # lazy: the box/orientation path above stays numpy-free

    if rgb.size == 0:
        return (1.0, 1.0, 1.0)
    r = float(np.asarray(rgb)[..., 0].mean())
    g = float(np.asarray(rgb)[..., 1].mean())
    b = float(np.asarray(rgb)[..., 2].mean())
    target = (r + g + b) / 3.0
    if target <= 0.0:
        return (1.0, 1.0, 1.0)

    def _clamp(x: float) -> float:
        return max(lo, min(hi, x))

    return (
        _clamp(target / r) if r > 0.0 else 1.0,
        _clamp(target / g) if g > 0.0 else 1.0,
        _clamp(target / b) if b > 0.0 else 1.0,
    )


def gray_world_correction(rgb, lo: float = 0.25, hi: float = 4.0) -> tuple[float, float, float]:
    """Per-channel multipliers that neutralize the average color cast.

    This is gray-world (equalize the per-channel means) combined with a
    common de-saturation scale so that applying the multipliers never clips a
    channel above 255. The scale is what makes the correction actually work on
    a saturated channel: without it a maxed-out red channel forces the red
    gain low, and boosting green/blue to match then clips their bright
    pixels, pulling their means back down and leaving a residual tint. With
    the scale, every channel lands on the same (lower) mean.

    Returns a length-3 ``(r, g, b)`` tuple of multipliers, clamped to
    ``[lo, hi]`` before the de-saturation scale is applied.
    """
    import numpy as np  # lazy: the box/orientation path above stays numpy-free

    x = np.asarray(rgb)
    raw = x.reshape(-1, 3)
    means = raw.mean(axis=0)
    total = float(means.sum())
    if total <= 1e-6:
        return (1.0, 1.0, 1.0)
    target = total / 3.0
    gains = [
        min(hi, max(lo, target / float(means[c]))) if float(means[c]) > 1e-6 else 1.0
        for c in range(3)
    ]
    scale = 1.0
    for c in range(3):
        mx = float(raw[:, c].max())
        if mx > 1e-6:
            scale = min(scale, 255.0 / (mx * gains[c]))
    scale = max(0.0, min(1.0, scale))
    return (scale * gains[0], scale * gains[1], scale * gains[2])


def apply_white_balance(rgb, gains: Sequence[float]):
    """Scale an (H, W, 3) image by per-channel ``gains`` (clamped to 0..255).

    Returns a NEW C-contiguous array of the same dtype as ``rgb`` (safe to hand to
    PIL / JPEG encoding).
    """
    import numpy as np  # lazy

    out = np.asarray(rgb, dtype=np.float32).copy()
    out[..., 0] *= float(gains[0])
    out[..., 1] *= float(gains[1])
    out[..., 2] *= float(gains[2])
    return np.clip(out, 0.0, 255.0).astype(rgb.dtype)
