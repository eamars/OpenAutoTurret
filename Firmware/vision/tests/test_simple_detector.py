"""Tests for the P8 bridge detector and the AI-metadata decoder.

These are the only parts of the real-camera path that can be tested without the
camera: the classical detector is pure numpy, and the `Objects` decoder is a
pure function. Both matter because they decide whether controld sees a target
(§6.2 measurement) or nothing at all.
"""
from __future__ import annotations

import numpy as np
import pytest

from vision.frame_source import (
    DetectedFrameSource,
    Detection,
    FrameCapture,
    FrameSource,
    parse_objects_metadata,
)
from vision.simple_detector import MotionBlobConfig, MotionBlobDetector

H, W = 180, 320


def _frame(cx: int, cy: int, size: int = 40) -> np.ndarray:
    """Dark frame with a bright square centered at (cx, cy)."""
    img = np.full((H, W, 3), 20, dtype=np.uint8)
    half = size // 2
    y0, y1 = max(0, cy - half), min(H, cy + half)
    x0, x1 = max(0, cx - half), min(W, cx + half)
    img[y0:y1, x0:x1] = 235
    return img


def _centre(det: Detection) -> tuple:
    return ((det.bbox_x_min_px + det.bbox_x_max_px) / 2.0,
            (det.bbox_y_min_px + det.bbox_y_max_px) / 2.0)


class TestMotionBlobDetector:
    def test_dark_target_on_bright_background(self):
        # Polarity must not matter: the two edge bands are stitched the same way
        # (a real target is often DARK against a bright wall).
        def dark_frame(cx, cy, size=40):
            img = np.full((H, W, 3), 235, dtype=np.uint8)
            half = size // 2
            img[cy - half : cy + half, cx - half : cx + half] = 15
            return img

        d = MotionBlobDetector()
        dets = None
        for i in range(8):
            dets = d.detect(dark_frame(60 + 15 * i, 90))
        assert dets and len(dets) == 1
        bx, _ = _centre(dets[0])
        assert abs(bx - 150) <= 25, bx           # same accuracy, other polarity

    def test_warmup_reports_nothing(self):
        d = MotionBlobDetector()
        for i in range(MotionBlobConfig().warmup_frames):
            assert d.detect(_frame(80 + 10 * i, 90)) == []

    def test_moving_blob_is_found_at_the_target(self):
        d = MotionBlobDetector()
        # Move the square so every frame differs from the last two.
        path = [(60 + 20 * i, 90) for i in range(8)]
        dets: list = []
        for cx, cy in path:
            dets = d.detect(_frame(cx, cy))
        assert len(dets) == 1
        cx, cy = path[-1]
        bx, by = _centre(dets[0])
        # Accuracy is bounded by the coarse grid + the per-frame step: at 20 px
        # per frame, half a step (10 px) plus a cell or two of slop. Anything
        # looser means the blob is not the target.
        assert abs(bx - cx) <= 30, (bx, cx)
        assert abs(by - cy) <= 12, (by, cy)      # no bias across the motion
        assert dets[0].bbox_x_min_px < dets[0].bbox_x_max_px
        assert dets[0].bbox_y_min_px < dets[0].bbox_y_max_px

    def test_centre_follows_the_target_across_the_sweep(self):
        # The measurement must move WITH the target (a stale/anchored estimate
        # would look like a detection but drive the loop nowhere).
        d = MotionBlobDetector()
        seen = []
        for i in range(8):
            dets = d.detect(_frame(60 + 20 * i, 90))
            if dets:
                seen.append(_centre(dets[0])[0])
        assert len(seen) >= 5
        assert all(b2 > b1 for b1, b2 in zip(seen, seen[1:])), seen
        assert seen[-1] - seen[0] >= 0.8 * 20 * (len(seen) - 1), seen

    def test_reports_the_target_class_with_a_bounded_confidence(self):
        # The §12.1 class gate must be exercised even by a classical detector:
        # it reports the target class id, at a confidence inside the band.
        cfg = MotionBlobConfig()
        d = MotionBlobDetector(cfg)
        det = None
        for i in range(8):
            dets = d.detect(_frame(60 + 20 * i, 90))
            if dets:
                det = dets[0]
        assert det is not None
        assert det.class_id == cfg.target_class_id
        assert cfg.confidence_min <= det.confidence <= cfg.confidence_max

    def test_static_scene_is_not_a_target(self):
        # Motion detector: a target that stops moving must NOT keep producing
        # measurements (the §34 coast/lost path is the correct behaviour then).
        d = MotionBlobDetector()
        for _ in range(8):
            assert d.detect(_frame(160, 90)) == []

    def test_moving_then_stopped_stops_reporting(self):
        d = MotionBlobDetector()
        moving = False
        for i in range(6):
            moving = bool(d.detect(_frame(60 + 20 * i, 90)))
        assert moving, "the moving target must be detected"
        for _ in range(4):
            assert d.detect(_frame(160, 90)) == []

    def test_noise_only_scene_is_rejected(self):
        rng = np.random.default_rng(7)
        d = MotionBlobDetector()
        hits = 0
        for _ in range(10):
            frame = rng.integers(0, 6, size=(H, W, 3), dtype=np.uint8)
            hits += len(d.detect(frame))
        assert hits == 0, "sensor noise must not look like a target"

    def test_garbage_input_is_no_detection(self):
        d = MotionBlobDetector()
        assert d.detect(None) == []
        assert d.detect(np.zeros((4, 4), dtype=np.uint8)) == []

    def test_reset_rebuilds_the_reference(self):
        d = MotionBlobDetector()
        for i in range(6):
            d.detect(_frame(60 + 20 * i, 90))
        d.reset()
        assert d.detect(_frame(200, 90)) == []  # warmup again, not a stale blob


class _StubSource(FrameSource):
    """Fixed captures: one with detections, one without (image only)."""

    def __init__(self, caps):
        self._caps = list(caps)
        self.started = self.stopped = 0

    def start(self):
        self.started += 1

    def stop(self):
        self.stopped += 1

    def image_size(self):
        return (W, H)

    def capture(self) -> FrameCapture:
        if not self._caps:
            raise IndexError("no captures left")
        return self._caps.pop(0)


class TestDetectedFrameSource:
    def test_real_detections_are_never_overridden(self):
        real = FrameCapture(
            width=W, height=H, sensor_timestamp_ns=1234, frame_sequence=0,
            detections=[Detection(class_id=2, confidence=0.9, bbox_x_min_px=1,
                                 bbox_y_min_px=2, bbox_x_max_px=3,
                                 bbox_y_max_px=4)],
            image=_frame(100, 90),
        )
        src = DetectedFrameSource(_StubSource([real]), MotionBlobDetector())
        cap = src.capture()
        assert cap.detections == real.detections  # untouched
        assert src.blobs == 0

    def test_bridge_fills_the_gap_and_lifecycle_delegates(self):
        plain = FrameCapture(width=W, height=H, sensor_timestamp_ns=7,
                             frame_sequence=0, detections=[],
                             image=_frame(100, 90))
        inner = _StubSource([plain])
        det = MotionBlobDetector()
        # Pre-warm so the blob is visible on the single captured frame.
        for i in range(6):
            det.detect(_frame(60 + 20 * i, 90))
        src = DetectedFrameSource(inner, det)
        src.start()
        cap = src.capture()
        src.stop()
        assert inner.started == 1 and inner.stopped == 1
        assert len(cap.detections) == 1
        assert cap.sensor_timestamp_ns == 7 and cap.frame_sequence == 0
        assert src.blobs == 1

    def test_missing_image_is_an_error_not_a_silent_zero(self):
        plain = FrameCapture(width=W, height=H, sensor_timestamp_ns=7,
                             frame_sequence=0, detections=[], image=None)
        src = DetectedFrameSource(_StubSource([plain]), MotionBlobDetector())
        with pytest.raises(RuntimeError):
            src.capture()


class TestParseObjectsMetadata:
    def test_tuple_shape(self):
        dets = parse_objects_metadata({"Objects": [(0.8, 1, 10, 20, 30, 40)]})
        assert len(dets) == 1
        d = dets[0]
        assert (d.class_id, d.confidence) == (1, pytest.approx(0.8))
        assert (d.bbox_x_min_px, d.bbox_y_min_px) == (10.0, 20.0)
        assert (d.bbox_x_max_px, d.bbox_y_max_px) == (40.0, 60.0)

    def test_dict_shape(self):
        dets = parse_objects_metadata(
            {"Objects": [{"Score": 0.5, "Label": 0,
                          "BoundingBox": [1, 2, 3, 4]}]}
        )
        assert len(dets) == 1
        assert dets[0].class_id == 0
        assert dets[0].bbox_x_max_px == 4.0

    def test_malformed_objects_are_dropped_not_guessed(self):
        dets = parse_objects_metadata({"Objects": [
            None, "junk", (0.5,), {"Score": 0.9}, {}, [1, 2, 3],
            (0.7, 1, 5, 5, 10, 10),
        ]})
        assert len(dets) == 1
        assert dets[0].bbox_x_max_px == 15.0

    @pytest.mark.parametrize("meta", [{}, {"Objects": None}, {"Objects": []}, None])
    def test_no_objects_yields_nothing(self, meta):
        assert parse_objects_metadata(meta) == []
