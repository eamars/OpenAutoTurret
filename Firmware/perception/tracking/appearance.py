"""§24 — the ephemeral appearance descriptor, and the policy that keeps it ephemeral.

A coarse HSV histogram, split into upper and lower body regions, as a *last* association
term. It is not person identification, it is not a face embedding, and calling it either
would be a lie about capability and a policy problem at the same time (§24's closing
paragraphs). Coarse is the point: 48 bins per region cannot survive a change of clothes
and is not the sort of data anyone could reconstruct a face from.

The policy is enforced in code, not in a comment:

* **memory only** — there is no write path here at all. No file, no cache, no pickle. A
  descriptor is dropped when its track retires and disappears entirely on restart, which is
  why :class:`AppearanceExtractor` keeps no per-identity state: the state lives on the
  ``Track``, so retirement frees it by construction.
* **no crop persistence** — the crop is a local in one function. The image never leaves it.
* **off by default** — §54 lists "run a heavy ReID model before lightweight association is
  tested" as a refused approach, and the same logic applies to the cheap version: the cost
  is real on a Pi 5 and the benefit is unmeasured until §46's crossing test fails without it.

The distance is cosine, and it is a *soft* signal: with §21's appearance weight at its
default 0.0 it contributes nothing at all, and even when enabled it cannot veto a match —
a person who walks behind a pillar for 300 ms comes back with a different histogram
(partial occlusion, different lighting on the visible strip), and an appearance term that
could veto would then destroy the identity the occlusion rule exists to protect.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from ..config import AppearanceConfig
from ..detection.types import BBox

try:  # numpy is the station's own dependency; the subsystem still imports without it
    import numpy as _np
except ImportError:  # pragma: no cover - depends on the interpreter
    _np = None

HUE_BINS = 12
SAT_BINS = 4
BINS_PER_REGION = HUE_BINS * SAT_BINS


@dataclass
class HsvDescriptor:
    """Concatenated upper/lower body histograms, L2-normalized, in one flat vector."""

    values: Sequence[float] = ()
    confidence: float = 0.0    # fraction of the region that was in-focus enough to count

    @property
    def is_empty(self) -> bool:
        return not len(self.values) or float(sum(abs(v) for v in self.values)) <= 0.0

    def __len__(self) -> int:
        return len(self.values)

    def __bool__(self) -> bool:
        """Always true: a descriptor with no samples yet is still the descriptor."""
        return True


def _to_float_image(image) -> Optional["object"]:
    if _np is None or image is None:
        return None
    array = _np.asarray(image)
    if array.ndim != 3 or array.shape[2] < 3:
        return None          # greyscale or an unexpected layout: no colour evidence
    rgb = array[:, :, :3].astype(_np.float32)
    if rgb.max() > 1.5:      # uint8 input, which is what Picamera2 hands over
        rgb = rgb / 255.0
    return rgb


def _rgb_to_hsv(rgb):
    """A small vectorized RGB→HSV, so this module does not drag in a second dependency.

    OpenCV would do it in C, and OpenCV is not something the perception subsystem imports:
    the descriptor is computed at most once per detection per frame, and an import that can
    fail on a station whose wheel broke is a reason for the whole pipeline to not start.
    """
    maximum = rgb.max(axis=2)
    minimum = rgb.min(axis=2)
    delta = maximum - minimum
    safe = _np.where(delta <= 1e-6, 1.0, delta)

    red, green, blue = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    hue = _np.zeros_like(maximum)
    mask_r = (maximum == red) & (delta > 1e-6)
    mask_g = (maximum == green) & (delta > 1e-6) & ~mask_r
    mask_b = (delta > 1e-6) & ~mask_r & ~mask_g
    hue[mask_r] = (((green - blue) / safe) % 6.0)[mask_r]
    hue[mask_g] = (((blue - red) / safe) + 2.0)[mask_g]
    hue[mask_b] = (((red - green) / safe) + 4.0)[mask_b]
    hue = hue / 6.0

    saturation = _np.where(maximum <= 1e-6, 0.0, delta / _np.maximum(maximum, 1e-6))
    return hue, saturation, maximum


def _region_histogram(rgb, y0: int, y1: int, x0: int, x1: int):
    crop = rgb[y0:y1, x0:x1]
    if crop.size == 0:
        return None, 0.0
    hue, saturation, _value = _rgb_to_hsv(crop)
    # Grey pixels carry no identity evidence at all — a black jacket in low light is the
    # same colour as a shadow. Weighting by saturation keeps "no colour" from reading as
    # "the same colour as everything else".
    weights = saturation
    hue_index = _np.clip((hue * HUE_BINS).astype(_np.int32), 0, HUE_BINS - 1)
    sat_index = _np.clip((saturation * SAT_BINS).astype(_np.int32), 0, SAT_BINS - 1)
    flat = _np.bincount((hue_index * SAT_BINS + sat_index).ravel(),
                        weights=weights.ravel(), minlength=BINS_PER_REGION)
    total = float(flat.sum())
    if total <= 0.0:
        return None, 0.0
    usable = float(_np.count_nonzero(saturation > 0.15)) / float(saturation.size)
    return flat / total, usable


def descriptor_from_image(image, bbox: BBox, *,
                          anchor_y: Optional[float] = None) -> Optional[HsvDescriptor]:
    """Upper/lower body histograms for one box, or ``None`` when there is nothing to see.

    The split is the torso line, not the geometric middle: an anchor is available in every
    real frame (§35), and splitting a half-visible person — a torso above, empty chair
    below — at the geometric middle mixes the background into the body descriptor and makes
    the appearance term fire on the wallpaper.
    """
    rgb = _to_float_image(image)
    if rgb is None or not bbox.is_well_formed():
        return None
    height, width = rgb.shape[0], rgb.shape[1]
    x0 = max(0, min(width - 1, int(bbox.x_min * width)))
    x1 = max(x0 + 1, min(width, int(bbox.x_max * width)))
    y0 = max(0, min(height - 1, int(bbox.y_min * height)))
    y1 = max(y0 + 1, min(height, int(bbox.y_max * height)))
    split = y0 + int((anchor_y - bbox.y_min) * (y1 - y0)) if anchor_y is not None \
        else y0 + (y1 - y0) // 2
    split = max(y0 + 1, min(y1 - 1, split))

    upper, upper_confidence = _region_histogram(rgb, y0, split, x0, x1)
    lower, lower_confidence = _region_histogram(rgb, split, y1, x0, x1)
    if upper is None or lower is None:
        return None
    values = _np.concatenate((_np.asarray(upper), _np.asarray(lower)))
    norm = float(_np.linalg.norm(values))
    if norm <= 0.0:
        return None
    return HsvDescriptor(values=(values / norm).tolist(),
                         confidence=min(upper_confidence, lower_confidence))


def descriptor_distance(a: Optional[HsvDescriptor],
                        b: Optional[HsvDescriptor]) -> Optional[float]:
    """Cosine distance in [0,2], or ``None`` when either side has no evidence.

    ``None`` rather than 1.0 matters: "no descriptor yet" and "maximally different
    clothing" are different statements, and §21's cost is only allowed to see the second
    one. Reporting the first as a mismatch would punish every new detection on its first
    frame — the frame in which it is, by definition, new.
    """
    if a is None or b is None or a.is_empty or b.is_empty:
        return None
    if len(a) != len(b):
        return None
    if _np is not None:
        dot = float(_np.dot(_np.asarray(a.values), _np.asarray(b.values)))
    else:  # pragma: no cover - only without numpy
        dot = float(sum(x * y for x, y in zip(a.values, b.values)))
    return max(0.0, 1.0 - min(1.0, max(-1.0, dot)))


def ema_update(previous: Optional[HsvDescriptor], incoming: HsvDescriptor,
               alpha: float = 0.4) -> HsvDescriptor:
    """Blend a new observation into the track's descriptor.

    An EMA rather than a replacement, because one bad frame — a person crossing a shaft of
    sunlight — would otherwise re-point the appearance term at a highlight and make the next
    association look like a different person.
    """
    if previous is None or previous.is_empty:
        return incoming
    if _np is not None:
        blended = (1.0 - alpha) * _np.asarray(previous.values) + alpha * _np.asarray(incoming.values)
        norm = float(_np.linalg.norm(blended))
        return HsvDescriptor(values=(blended / norm).tolist() if norm > 0.0 else blended.tolist(),
                             confidence=min(previous.confidence, incoming.confidence))
    paired = [(1.0 - alpha) * x + alpha * y for x, y in zip(previous.values, incoming.values)]
    return HsvDescriptor(values=paired,
                         confidence=min(previous.confidence, incoming.confidence))


class AppearanceExtractor:
    """The §24 gate: nothing is computed at all unless the profile asked for it."""

    def __init__(self, cfg: AppearanceConfig) -> None:
        self.cfg = cfg
        self.frames_computed = 0
        self.frames_unavailable = 0

    @property
    def enabled(self) -> bool:
        return bool(self.cfg.enabled) and self.cfg.type == "hsv_upper_lower"

    def descriptor(self, image, bbox: BBox,
                   anchor_y: Optional[float] = None) -> Optional[HsvDescriptor]:
        if not self.enabled or image is None:
            return None
        descriptor = descriptor_from_image(image, bbox, anchor_y=anchor_y)
        if descriptor is None:
            self.frames_unavailable += 1
        else:
            self.frames_computed += 1
        return descriptor

    def describe_detections(self, image, detections) -> dict:
        """One frame's descriptors, keyed by ``detection_id_in_frame``.

        Computed once per frame and looked up by the association's appearance term, so the
        cost is O(detections) per frame instead of O(tracks × detections) — the difference
        is the whole reason this term can be afforded on a Pi 5 at all. The image is passed
        in by the pipeline and used only here; it is never stored (§24).
        """
        if not self.enabled:
            return {}
        out = {}
        for detection in detections:
            out[detection.detection_id_in_frame] = self.descriptor(
                image, detection.bbox,
                anchor_y=(detection.measured_anchor.y
                          if detection.measured_anchor else None))
        return out

    @staticmethod
    def scorer_for(descriptors: dict):
        """The callable §21's ``assign`` wants, reading from this frame's descriptors."""
        if not descriptors:
            return None

        def score(track, detection):
            return descriptor_distance(getattr(track, "appearance", None),
                                       descriptors.get(detection.detection_id_in_frame))

        return score
