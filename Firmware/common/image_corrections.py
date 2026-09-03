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

import pathlib
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


# ── the installed camera orientation, read from the station rather than remembered ─────
#
# The IMX500 on this station is mounted upside-down. Getting that correction in place has
# been a hand job on every bring-up — an environment variable for the web daemon, a
# command-line flag for the detector — and forgetting it does not crash anything, which is
# precisely the problem: the preview looks entirely normal while the geometry the controller
# reasons about is 180 degrees away from the picture, and the archive notes record somebody
# losing hours that way (docs/archive/post_homing_test_queue.md). A parameter that only
# exists if the person launching a process remembers it is not a parameter. So the mount is
# described once, in one file, and read by every process that turns sensor pixels into
# geometry: webd's preview and visiond's detector. An explicit flag still wins, and says so
# when it disagrees — because overriding the station's description of its own hardware is
# worth a line in the log.
# Anchored to the checkout, not to the working directory. A daemon started from elsewhere must
# not silently conclude that nobody described the mount and run the camera uncorrected; that is
# the same silent 180-degree error this file exists to prevent, wearing a costume.
INSTALL_ORIENTATION_FILE = (
    pathlib.Path(__file__).resolve().parent.parent / "config" / "camera_install.yaml")
UNCONFIGURED_ORIENTATION = "none"


def read_install_orientation(path=None, explicit=None, explicit_source="an explicit option"):
    """Resolve the installed orientation. Returns (orientation, where_it_came_from).

    Order of authority: an explicit option (flag or environment variable), then the station
    file, then "none" — and the third case is reported rather than assumed, because "none"
    is what an *aligned* camera wants and what a forgotten configuration leaves behind.
    Raises ValueError on a value outside ORIENTATIONS, naming the choices: a typo in a mount
    description must stop the process that is about to reason about geometry wrongly, not
    continue quietly with a picture that looks fine.
    """
    if explicit is not None and str(explicit).strip() != "":
        return validate_orientation(str(explicit).strip()), "%s" % explicit_source

    wanted = pathlib.Path(path) if path else pathlib.Path(INSTALL_ORIENTATION_FILE)
    if not wanted.exists():
        return UNCONFIGURED_ORIENTATION, ("not configured (%s absent; assuming an aligned "
                                         "camera)" % wanted)
    value = None
    for lineno, raw in enumerate(wanted.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        key, sep, val = line.partition(":")
        if not sep:
            key, sep, val = line.partition("=")
        if not sep or key.strip() != "orientation":
            continue
        value = val.strip().strip("\"'\"")
        where = "%s:%d" % (wanted, lineno)
        break
    if value is None:
        return UNCONFIGURED_ORIENTATION, ("not configured (%s has no `orientation` key)" % wanted)
    try:
        return validate_orientation(value), where
    except ValueError as exc:
        raise ValueError("%s (%s)" % (exc, wanted)) from exc


# ── channel order of a camera stream, taken from the library that writes it ────────────
#
# The web preview used to hardcode `arr[..., :3][..., ::-1]` on the reasoning that
# V4L2_PIX_FMT_XBGR32 is 0x00RRGGBB in host byte order, so the bytes must arrive B,G,R,X.
# That derivation is defensible on paper and wrong on this stack: picamera2's own encoder
# (`request.py`, the path every picamera2 example uses and the one that produces correct
# colour in the wild) encodes an XBGR8888 buffer with colourspace tag "RGBX" — it treats the
# buffer as R,G,B,X in memory. Doing the reversal anyway swaps red and blue, which is not an
# obvious defect: white stays white, so the scene looks fine until something saturated
# appears. An operator at this station described it precisely — "the yellow is rendered as
# blue", "not just yellow blue swap, but also pink purple swap" — which is R<->B exactly:
# yellow (255,255,0) becomes cyan, pink (255,192,203) becomes lavender, green is untouched.
# Measurement agreed: 1.71% of the live frame was blue-dominant against 0.02% red-dominant,
# with near-white pixels dead neutral (a cast moves white; a swap does not).
#
# So the order comes from the format name, using the same correspondence the installed
# picamera2 uses, and an unrecognised format is refused rather than guessed at. The table's
# agreement with the installed library is asserted by a test that reads the library's source,
# because a hand-copied table is exactly the kind of list that goes on certifying a world
# that no longer exists.
_RGB_INDEX_FOR_FORMAT = {
    # format name -> which channels of the buffer are R, G, B, in memory order.
    # Derived from picamera2's FORMAT_TABLE: {"XBGR8888": "RGBX", "XRGB8888": "BGRX",
    # "BGR888": "RGB", "RGB888": "BGR"} — the colourspace tag names the memory order.
    "XBGR8888": (0, 1, 2),   # encodes as "RGBX"
    "XRGB8888": (2, 1, 0),   # encodes as "BGRX"
    "BGR888": (0, 1, 2),     # encodes as "RGB"  — the fourcc name is NOT the memory order
    "RGB888": (2, 1, 0),     # encodes as "BGR"  — ditto. My first draft of this table read the
                             # names instead of the tags and got both of these backwards; the
                             # test that compares this table against picamera2's source caught
                             # it within one run, which is why that test exists.
}
_CHANNELS_FOR_FORMAT = {"XBGR8888": 4, "XRGB8888": 4, "BGR888": 3, "RGB888": 3}


def rgb_from_stream(arr, pixel_format: str):
    """Return an (H, W, 3) RGB view of a camera buffer in the format named by ``pixel_format``.

    Raises ValueError for a format this function does not know, and for a buffer whose channel
    count contradicts its own format name. Both cases are refusals on purpose: the alternative
    is to keep running and produce a picture whose colours are quietly permuted, and that
    failure has already cost this station an afternoon once.
    """
    if pixel_format not in _RGB_INDEX_FOR_FORMAT:
        raise ValueError(
            "unknown camera pixel format %r; known formats are %s — refusing to guess a "
            "channel order, because a wrong guess is a picture with red and blue exchanged "
            "and nothing else visibly wrong"
            % (pixel_format, sorted(_RGB_INDEX_FOR_FORMAT)))
    want = _CHANNELS_FOR_FORMAT[pixel_format]
    if arr.ndim != 3 or arr.shape[2] != want:
        raise ValueError(
            "format %s expects %d channels per pixel but the buffer has shape %r"
            % (pixel_format, want, tuple(arr.shape)))
    return arr[..., list(_RGB_INDEX_FOR_FORMAT[pixel_format])]
