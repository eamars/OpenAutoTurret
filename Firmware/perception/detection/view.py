"""The detail camera's detections, expressed in the wide camera's frame.

Only the camera on the operator's main display feeds the Hailo (owner, 2026-10-02), and the control
layer knows one camera model: the wide one (`calibration/camera_intrinsics.yaml`). So a detail frame
is published in *virtual wide coordinates*: the detail picture is a window 1/scale the size of the wide
frame, centred on it. Everything downstream -- the tracker, the selected-target observation, controld's
pixel-to-ray, the HUD -- keeps working in one frame, and a camera swap does not change the frame the
turret's estimator lives in.

Why centred, not where the detail optic actually looks: the two optics are a few centimetres apart
and not perfectly parallel, so where the narrow view sits inside the wide one depends on the subject's
distance (measured 2026-10-02: 0.49/0.45 of the wide frame for a subject at about 1 m). Centring the
window means the turret puts the subject in the middle of the picture the operator is watching, which
is the point of the swap; the price is that the estimator sees the residual misalignment (about 1
degree) as a step when the source changes, which Level 1 smooths like any other.

The scale is depth-independent (it is the focal-length ratio), so it is one number in configuration,
measured by `tools/register_detail_view.py` from the two live previews.
"""
from __future__ import annotations

from dataclasses import replace
from typing import Tuple

from .types import BBox, PointNorm


def narrow_to_wide(value: float, scale: float) -> float:
    """One normalised coordinate of the detail picture, in the wide frame."""
    return 0.5 + (float(value) - 0.5) / float(scale)


def to_wide_frame(detection_set, scale: float, canvas: Tuple[int, int]):
    """Map every box, anchor and keypoint of a detail-camera set into the wide frame.

    ``canvas`` is the wide picture's size, which the set then declares: the TrackSet built from it is
    validated by controld against the wide intrinsics, and that is now true of its coordinates.
    """
    scale = float(scale)
    if not scale > 1.0:
        raise ValueError(f"the detail view must be narrower than the wide one, got scale {scale}")

    def m(v: float) -> float:
        return narrow_to_wide(v, scale)

    detections = []
    for detection in detection_set.detections:
        b, a = detection.bbox, detection.measured_anchor
        detections.append(replace(
            detection,
            bbox=BBox(m(b.x_min), m(b.y_min), m(b.x_max), m(b.y_max)),
            measured_anchor=PointNorm(m(a.x), m(a.y)),
            keypoints=tuple(replace(p, x=m(p.x), y=m(p.y)) for p in detection.keypoints)))
    # The ROI is in the detail stream's pixels and has no meaning on the wide canvas.
    return replace(detection_set.with_detections(detections),
                   stream_width=int(canvas[0]), stream_height=int(canvas[1]), roi=None)
