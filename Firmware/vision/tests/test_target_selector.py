"""Target selector / association tests (camera-free)."""
import unittest

from vision.frame_source import Detection, FrameCapture
from vision.protocol import CLASS_CAR, CLASS_PERSON
from vision.target_selector import TargetSelector, TargetSelectorConfig


def det(class_id, confidence, cx, cy, size=100.0):
    h = size / 2.0
    return Detection(
        class_id=class_id,
        confidence=confidence,
        bbox_x_min_px=cx - h,
        bbox_y_min_px=cy - h,
        bbox_x_max_px=cx + h,
        bbox_y_max_px=cy + h,
    )


def cap(seq, t_ns, dets, w=1920, h=1080):
    return FrameCapture(width=w, height=h, sensor_timestamp_ns=t_ns, frame_sequence=seq, detections=dets)


class TestTargetSelector(unittest.TestCase):
    def test_selects_preferred_class(self):
        ts = TargetSelector(TargetSelectorConfig(preferred_class_id=CLASS_PERSON, fallback_to_best=False))
        m = ts.update(cap(0, 0, [det(CLASS_CAR, 0.9, 100, 100), det(CLASS_PERSON, 0.7, 500, 500)]))
        self.assertTrue(m.valid)
        self.assertEqual(m.class_id, CLASS_PERSON)
        self.assertAlmostEqual(m.anchor_u_px, 500.0)

    def test_confidence_threshold_rejects_low(self):
        ts = TargetSelector(TargetSelectorConfig(confidence_threshold=0.6, fallback_to_best=False))
        m = ts.update(cap(0, 0, [det(CLASS_PERSON, 0.3, 500, 500)]))
        self.assertFalse(m.valid)

    def test_track_continuity_same_id(self):
        ts = TargetSelector(TargetSelectorConfig(fallback_to_best=False))
        ids = set()
        for i in range(20):
            m = ts.update(cap(i, i * 1000, [det(CLASS_PERSON, 0.9, 300.0 + i * 25.0, 400.0)]))
            self.assertTrue(m.valid)
            ids.add(m.visual_track_id)
        self.assertEqual(ids, {1})  # one stable track over the whole sequence

    def test_reacquisition_drops_lost_track(self):
        cfg = TargetSelectorConfig(max_lost_frames=3, fallback_to_best=False)
        ts = TargetSelector(cfg)
        for i in range(5):
            ts.update(cap(i, i, [det(CLASS_PERSON, 0.9, 400.0, 400.0)]))
        self.assertTrue(ts.has_target)
        # Now the target disappears for more than max_lost_frames.
        for i in range(10):
            ts.update(cap(100 + i, 100 + i, []))
        self.assertFalse(ts.has_target)
        # It reappears -> a fresh track is started (new id).
        m = ts.update(cap(200, 200, [det(CLASS_PERSON, 0.9, 400.0, 400.0)]))
        self.assertTrue(m.valid)
        self.assertNotEqual(m.visual_track_id, 1)

    def test_empty_when_no_detections(self):
        ts = TargetSelector(TargetSelectorConfig(fallback_to_best=False))
        m = ts.update(cap(0, 0, []))
        self.assertFalse(m.valid)
        self.assertEqual(m.frame_sequence, 0)

    def test_anchor_is_bbox_centre(self):
        ts = TargetSelector(TargetSelectorConfig(fallback_to_best=False))
        m = ts.update(cap(0, 0, [det(CLASS_PERSON, 0.9, 480.0, 270.0, size=100.0)]))
        self.assertTrue(m.valid)
        self.assertAlmostEqual(m.anchor_u_px, 480.0)
        self.assertAlmostEqual(m.anchor_v_px, 270.0)

    def test_normalized_bbox(self):
        ts = TargetSelector(TargetSelectorConfig(fallback_to_best=False))
        # bbox centre (960, 540), size 100 -> [910, 490, 1010, 590]
        m = ts.update(cap(0, 0, [det(CLASS_PERSON, 0.9, 960.0, 540.0, size=100.0)]))
        self.assertAlmostEqual(m.bbox_x_min_norm, 910.0 / 1920.0, places=5)
        self.assertAlmostEqual(m.bbox_y_min_norm, 490.0 / 1080.0, places=5)
        self.assertAlmostEqual(m.bbox_x_max_norm, 1010.0 / 1920.0, places=5)
        self.assertAlmostEqual(m.bbox_y_max_norm, 590.0 / 1080.0, places=5)

    def test_fallback_to_best_when_no_preferred(self):
        cfg = TargetSelectorConfig(preferred_class_id=CLASS_PERSON, fallback_to_best=True)
        ts = TargetSelector(cfg)
        # Only a car is present; the fallback picks it (best detection).
        m = ts.update(cap(0, 0, [det(CLASS_CAR, 0.8, 300.0, 300.0)]))
        self.assertTrue(m.valid)
        self.assertEqual(m.class_id, CLASS_CAR)


if __name__ == "__main__":
    unittest.main()
