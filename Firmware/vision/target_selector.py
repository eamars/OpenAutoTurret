"""Target selection / association (architecture §12).

Turns a set of raw detections into ONE selected target per frame (the "motor
control path receives one selected target at a time", §12.2). v1 association is
lightweight: class gating, confidence threshold, IoU + centroid-proximity
association against the current track, and automatic reacquisition. This module
only consumes detections and emits a TargetMeasurement — it does not touch the
camera hardware, CAN, or the motor driver.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .frame_source import Detection, FrameCapture
from .protocol import CLASS_PERSON, TargetMeasurement


@dataclass
class TargetSelectorConfig:
    """Association / selection parameters (§12.1–§12.2)."""

    preferred_class_id: int = CLASS_PERSON
    confidence_threshold: float = 0.5
    iou_threshold: float = 0.05
    centroid_gate_px: float = 300.0
    max_lost_frames: int = 10
    fallback_to_best: bool = True  # if no preferred-class target, use best detection
    track_id_start: int = 1


def _iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ax_min, ay_min, ax_max, ay_max = a
    bx_min, by_min, bx_max, by_max = b
    ix_min = max(ax_min, bx_min)
    iy_min = max(ay_min, by_min)
    ix_max = min(ax_max, bx_max)
    iy_max = min(ay_max, by_max)
    inter = max(0.0, ix_max - ix_min) * max(0.0, iy_max - iy_min)
    area_a = max(0.0, ax_max - ax_min) * max(0.0, ay_max - ay_min)
    area_b = max(0.0, bx_max - bx_min) * max(0.0, by_max - by_min)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def _centroid(d: Detection) -> Tuple[float, float]:
    return d.centre_px


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


@dataclass
class _Track:
    track_id: int
    class_id: int
    last_bbox_px: Tuple[float, float, float, float]
    last_centroid_px: Tuple[float, float]
    last_confidence: float
    last_frame: int
    lost_frames: int = 0


class TargetSelector:
    """Maintains one selected target and associates detections to it."""

    def __init__(self, config: Optional[TargetSelectorConfig] = None) -> None:
        self.cfg = config or TargetSelectorConfig()
        self._track: Optional[_Track] = None
        self._next_track_id = self.cfg.track_id_start

    @property
    def has_target(self) -> bool:
        return self._track is not None

    def reset(self) -> None:
        self._track = None
        self._next_track_id = self.cfg.track_id_start

    def _candidates(self, dets: List[Detection]) -> List[Detection]:
        """Class gating + confidence threshold (§12.1)."""
        out = [
            d
            for d in dets
            if d.confidence >= self.cfg.confidence_threshold
            and (d.class_id == self.cfg.preferred_class_id or not self.cfg.preferred_class_id)
        ]
        return out

    def _score(self, det: Detection) -> float:
        """Association score against the current track (higher = better)."""
        if self._track is None:
            return det.confidence
        iou = _iou(self._track.last_bbox_px, (det.bbox_x_min_px, det.bbox_y_min_px, det.bbox_x_max_px, det.bbox_y_max_px))
        cd = _dist(_centroid(det), self._track.last_centroid_px)
        # Combine IoU and centroid proximity; both must be plausibly close.
        s = iou
        if cd <= self.cfg.centroid_gate_px:
            s += 1.0
        return s

    def update(self, cap: FrameCapture) -> TargetMeasurement:
        """Advance association by one frame; return the selected target.

        An empty (valid=False) measurement is returned when there is no target,
        which the daemon publishes so controld knows the target was lost.
        """
        w, h = cap.width, cap.height
        dets = cap.detections

        # 1) Class-gated candidates.
        cands = self._candidates(dets)

        # 2) Pick the best candidate (association score, then confidence).
        best: Optional[Detection] = None
        if cands:
            best = max(cands, key=lambda d: (self._score(d), d.confidence))

        # 3) Associate or reacquire.
        if best is not None:
            iou = 0.0
            cd = float("inf")
            if self._track is not None:
                iou = _iou(self._track.last_bbox_px, (best.bbox_x_min_px, best.bbox_y_min_px, best.bbox_x_max_px, best.bbox_y_max_px))
                cd = _dist(_centroid(best), self._track.last_centroid_px)
                keep = (
                    iou >= self.cfg.iou_threshold
                    or cd <= self.cfg.centroid_gate_px
                )
                if keep:
                    # Continue the existing track.
                    self._track.lost_frames = 0
                    self._update_track(best, cap.frame_sequence)
                else:
                    # Not the same object -> reacquire a fresh track.
                    self._track = self._new_track(best, cap.frame_sequence)
            else:
                self._track = self._new_track(best, cap.frame_sequence)
        else:
            # No candidate this frame.
            if self._track is not None:
                self._track.lost_frames += 1
                if self._track.lost_frames > self.cfg.max_lost_frames:
                    self._track = None  # reacquisition: drop the lost track

        # 4) Fallback: if we have no track but a fallback is allowed, use the
        #    best overall detection (e.g. a non-preferred class).
        if self._track is None and self.cfg.fallback_to_best and dets:
            overall = max(dets, key=lambda d: d.confidence)
            if overall.confidence >= self.cfg.confidence_threshold:
                self._track = self._new_track(overall, cap.frame_sequence)

        return self._make_measurement(cap)

    def _new_track(self, det: Detection, frame: int) -> _Track:
        t = _Track(
            track_id=self._next_track_id,
            class_id=det.class_id,
            last_bbox_px=(det.bbox_x_min_px, det.bbox_y_min_px, det.bbox_x_max_px, det.bbox_y_max_px),
            last_centroid_px=_centroid(det),
            last_confidence=det.confidence,
            last_frame=frame,
        )
        self._next_track_id += 1
        return t

    def _update_track(self, det: Detection, frame: int) -> None:
        assert self._track is not None
        self._track.class_id = det.class_id
        self._track.last_bbox_px = (det.bbox_x_min_px, det.bbox_y_min_px, det.bbox_x_max_px, det.bbox_y_max_px)
        self._track.last_centroid_px = _centroid(det)
        self._track.last_confidence = det.confidence
        self._track.last_frame = frame
        self._track.lost_frames = 0

    def _make_measurement(self, cap: FrameCapture) -> TargetMeasurement:
        if self._track is None:
            return TargetMeasurement(
                frame_sequence=cap.frame_sequence,
                sensor_timestamp_ns=cap.sensor_timestamp_ns,
                valid=False,
            )
        t = self._track
        w, h = cap.width, cap.height
        x_min, y_min, x_max, y_max = t.last_bbox_px
        cx, cy = t.last_centroid_px
        return TargetMeasurement(
            frame_sequence=cap.frame_sequence,
            sensor_timestamp_ns=cap.sensor_timestamp_ns,
            valid=True,
            class_id=t.class_id,
            confidence=t.last_confidence,
            bbox_x_min_norm=x_min / w,
            bbox_y_min_norm=y_min / h,
            bbox_x_max_norm=x_max / w,
            bbox_y_max_norm=y_max / h,
            anchor_u_px=cx,
            anchor_v_px=cy,
            visual_track_id=t.track_id,
        )
