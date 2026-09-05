"""§52 — bbox order, normalization, ROI/aspect mapping, validity, and the aim anchor.

The geometry expectations are hand-computed rather than snapshot-compared: the whole point
of these tests is that the numbers are right in absolute terms, and a snapshot only proves
the code still disagrees with itself in the same way it always did.
"""
from __future__ import annotations

import math
import unittest

from perception.config import AnchorConfig
from perception.detection.anchor import (COCO_HIP_LEFT, COCO_HIP_RIGHT,
                                         COCO_SHOULDER_LEFT, COCO_SHOULDER_RIGHT,
                                         compute_anchor)
from perception.detection.normalize import (InferenceGeometry, clamp_to_frame,
                                            letterbox_scale_pad, normalize_rows,
                                            parse_row_box)
from perception.detection.types import AnchorSource, BBox, Keypoint
from perception.errors import ValidationError

LABELS = [f"class{i}" for i in range(80)]
LABELS[0] = "person"


def geometry(**kwargs):
    base = dict(input_width=640, input_height=640, stream_width=1920,
                stream_height=1080)
    base.update(kwargs)
    return InferenceGeometry(**base)


def keypoints(shoulder=0.9, hip=0.9, count=17, sx=(0.44, 0.56), sy=0.30,
              hx=(0.45, 0.55), hy=0.55):
    points = [Keypoint(0.5, 0.5, 0.0) for _ in range(count)]
    if count > COCO_SHOULDER_RIGHT:
        points[COCO_SHOULDER_LEFT] = Keypoint(sx[0], sy, shoulder)
        points[COCO_SHOULDER_RIGHT] = Keypoint(sx[1], sy, shoulder)
    if count > COCO_HIP_RIGHT:
        points[COCO_HIP_LEFT] = Keypoint(hx[0], hy, hip)
        points[COCO_HIP_RIGHT] = Keypoint(hx[1], hy, hip)
    return points


class TestLetterbox(unittest.TestCase):
    def test_scale_and_padding_for_1080p_into_640(self):
        scale, pad_x, pad_y = letterbox_scale_pad(640, 640, 1920, 1080)
        self.assertAlmostEqual(scale, 640.0 / 1920.0, places=12)
        self.assertAlmostEqual(pad_x, 0.0, places=9)
        self.assertAlmostEqual(pad_y, 140.0, places=9)   # (640 - 1080·scale)/2

    def test_frame_centre_round_trips_through_both_geometries(self):
        # Model-input pixels go in, normalized stream fractions come out — the
        # normalized-or-pixel question is answered once, in parse_row_box.
        for preserve in (True, False):
            geo = geometry(preserve_aspect_ratio=preserve)
            point = geo.unmap_point(320.0, 320.0)
            self.assertAlmostEqual(point.x, 0.5, places=9, msg=f"preserve={preserve}")
            self.assertAlmostEqual(point.y, 0.5, places=9, msg=f"preserve={preserve}")

    def test_letterbox_padding_is_undone_at_the_frame_edge(self):
        # The visible scene occupies input rows 140..500 of a 640-high input; the rest is
        # black padding. A mapping that ignores the pad reports the top edge of the world
        # as y = 0.219, and every box above the middle of the frame is then wrong.
        geo = geometry(preserve_aspect_ratio=True)
        bottom = geo.unmap_point(320.0, 500.0)
        self.assertAlmostEqual(bottom.y, 1.0, places=9)
        top = geo.unmap_point(320.0, 140.0)
        self.assertAlmostEqual(top.y, 0.0, places=9)

    def test_roi_offset_is_carried_into_stream_coordinates(self):
        geo = geometry(input_width=320, input_height=320, preserve_aspect_ratio=False,
                       roi=(200, 100, 800, 600))
        point = geo.unmap_point(160.0, 160.0)
        self.assertAlmostEqual(point.x, (200 + 400) / 1920.0, places=9)
        self.assertAlmostEqual(point.y, (100 + 300) / 1080.0, places=9)

    def test_geometry_rejects_impossible_dimensions(self):
        with self.assertRaises(ValidationError):
            geometry(input_width=0)
        with self.assertRaises(ValidationError):
            geometry(stream_height=-8)
        with self.assertRaises(ValidationError):
            geometry(roi=(0, 0, 0, 100))


class TestBboxOrders(unittest.TestCase):
    def assertBox(self, got, expected):
        for value, want in zip(got, expected):
            self.assertAlmostEqual(value, want, places=9)

    def test_all_three_conventions_agree(self):
        expected = (64.0, 128.0, 192.0, 256.0)      # 0.1,0.2,0.3,0.4 of 640x640
        common = dict(input_width=640, input_height=640, normalized=True)
        self.assertBox(parse_row_box([0.1, 0.2, 0.3, 0.4], "xy", **common), expected)
        self.assertBox(parse_row_box([0.2, 0.1, 0.4, 0.3], "yxyx", **common), expected)
        self.assertBox(parse_row_box([0.2, 0.3, 0.2, 0.2], "cxcywh", **common), expected)

    def test_pixel_boxes_are_not_scaled_again(self):
        self.assertEqual(parse_row_box([10.0, 20.0, 30.0, 40.0], "xy",
                                       input_width=640, input_height=640,
                                       normalized=False), (10.0, 20.0, 30.0, 40.0))

    def test_unknown_order_is_refused_not_guessed(self):
        with self.assertRaises(ValidationError):
            parse_row_box([0.1, 0.2, 0.3, 0.4], "xyxy_norm",
                          input_width=640, input_height=640, normalized=True)

    def test_clamp_keeps_an_edge_box_and_drops_an_off_frame_box(self):
        clipped = clamp_to_frame(BBox(0.9, 0.4, 1.6, 0.8))
        self.assertIsNotNone(clipped)
        self.assertAlmostEqual(clipped.x_max, 1.0, places=9)
        self.assertIsNone(clamp_to_frame(BBox(1.2, 0.4, 1.6, 0.8)))
        self.assertIsNone(clamp_to_frame(BBox(0.5, 0.5, 0.5, 0.6)))


class TestNormalizeRows(unittest.TestCase):
    def normalize(self, rows, **kwargs):
        base = dict(geometry=geometry(), model_id="m", model_generation=2,
                    frame_sequence=11, sensor_timestamp_ns=1_000_000_000,
                    publish_timestamp_ns=2_000_000_000, label_map=LABELS)
        base.update(kwargs)
        return normalize_rows(rows, **base)

    def test_a_valid_row_becomes_a_validated_normalized_detection(self):
        out = self.normalize([[0.87, 0, 0.1, 0.2, 0.3, 0.4]])
        detection = out.detections[0]
        self.assertEqual(detection.class_name, "person")
        # A stretched (non-aspect-preserving) mapping maps the model's fractions onto the
        # stream 1:1 by construction — asserting the identity here is what proves the pixel
        # round-trip in parse_row_box did not double-scale.
        self.assertAlmostEqual(detection.bbox.x_min, 0.1, places=9)
        self.assertAlmostEqual(detection.bbox.y_max, 0.4, places=9)
        self.assertEqual(detection.detection_id_in_frame, 0)
        self.assertEqual(out.counters.raw_outputs, 1)
        self.assertEqual(out.counters.malformed_rejected, 0)
        self.assertEqual(out.counters.post_model_nms, 1)
        self.assertEqual(out.model_generation, 2)
        out.validate()

    def test_malformed_rows_are_counted_not_clamped(self):
        rows = [
            [0.9, 0, 0.1, 0.2, 0.3, 0.4],         # good
            [float("nan"), 0, 0.1, 0.2, 0.3, 0.4],
            [1.4, 0, 0.1, 0.2, 0.3, 0.4],         # score out of range
            [0.9, 99, 0.1, 0.2, 0.3, 0.4],        # unknown class id
            [0.9, 0, 0.3, 0.4, 0.1, 0.2],         # inverted box
            [0.9, 0, 1.2, 0.2, 1.6, 0.4],         # wholly off-frame
            [0.9, 0, 0.1, 0.2],                   # too short
        ]
        out = self.normalize(rows)
        self.assertEqual(out.counters.raw_outputs, 7)
        self.assertEqual(out.counters.malformed_rejected, 6)
        self.assertEqual(len(out.detections), 1)
        self.assertEqual(out.counters.post_model_nms, 1)

    def test_an_empty_frame_is_still_a_valid_set(self):
        out = self.normalize([])
        self.assertEqual(out.detections, [])
        out.validate()

    def test_missing_sensor_timestamp_is_refused(self):
        with self.assertRaises(ValidationError):
            self.normalize([[0.9, 0, 0.1, 0.2, 0.3, 0.4]], sensor_timestamp_ns=0)

    def test_the_set_carries_its_source_geometry(self):
        out = self.normalize([[0.9, 0, 0.1, 0.2, 0.3, 0.4]],
                             geometry=geometry(preserve_aspect_ratio=True,
                                               roi=(0, 0, 1920, 1080)))
        self.assertTrue(out.preserve_aspect_ratio)
        self.assertEqual(out.roi, (0, 0, 1920, 1080))
        self.assertEqual((out.stream_width, out.stream_height), (1920, 1080))


class TestAnchorPolicy(unittest.TestCase):
    def setUp(self):
        self.cfg = AnchorConfig()
        self.box = BBox(0.40, 0.20, 0.60, 0.80)      # w 0.2, h 0.6

    def test_shoulders_win_when_confident(self):
        anchor, source = compute_anchor(self.box, keypoints(shoulder=0.9, hip=0.9),
                                        self.cfg)
        self.assertIs(source, AnchorSource.POSE_SHOULDERS)
        self.assertAlmostEqual(anchor.x, 0.5, places=9)
        self.assertAlmostEqual(anchor.y, 0.30, places=9)

    def test_weighting_follows_keypoint_confidence(self):
        points = [Keypoint(0.5, 0.5, 0.0) for _ in range(17)]
        points[COCO_SHOULDER_LEFT] = Keypoint(0.40, 0.30, 0.9)
        points[COCO_SHOULDER_RIGHT] = Keypoint(0.60, 0.30, 0.3)
        anchor, source = compute_anchor(self.box, points, self.cfg)
        self.assertIs(source, AnchorSource.POSE_SHOULDERS)
        self.assertLess(anchor.x, 0.5, "the more confident keypoint pulls the midpoint")

    def test_hips_are_used_when_the_shoulders_are_missing(self):
        anchor, source = compute_anchor(self.box, keypoints(shoulder=0.0, hip=0.8),
                                        self.cfg)
        self.assertIs(source, AnchorSource.POSE_TORSO)
        self.assertAlmostEqual(anchor.y, 0.55, places=9)

    def test_box_torso_fraction_when_pose_is_unavailable(self):
        anchor, source = compute_anchor(self.box, keypoints(shoulder=0.0, hip=0.0),
                                        self.cfg)
        self.assertIs(source, AnchorSource.BBOX_TORSO)
        self.assertAlmostEqual(anchor.y, 0.20 + 0.45 * 0.60, places=9)

    def test_pose_anchors_can_be_disabled_entirely(self):
        _anchor, source = compute_anchor(self.box, keypoints(),
                                         AnchorConfig(use_pose_anchors=False))
        self.assertIs(source, AnchorSource.BBOX_TORSO)

    def test_a_short_keypoint_vector_falls_back_instead_of_indexing_past_the_end(self):
        _anchor, source = compute_anchor(self.box, keypoints(count=10), self.cfg)
        self.assertIs(source, AnchorSource.BBOX_TORSO)

    def test_an_anchor_outside_its_own_box_is_clamped_into_it(self):
        anchor, source = compute_anchor(self.box, keypoints(sx=(0.05, 0.10), sy=0.95),
                                        self.cfg)
        self.assertIs(source, AnchorSource.POSE_SHOULDERS)
        self.assertTrue(self.box.x_min <= anchor.x <= self.box.x_max)
        self.assertTrue(self.box.y_min <= anchor.y <= self.box.y_max)

    def test_a_degenerate_box_falls_back_to_its_centre(self):
        anchor, source = compute_anchor(BBox(0.5, 0.5, 0.5, 0.5), (), self.cfg)
        self.assertIs(source, AnchorSource.BBOX_CENTER_FALLBACK)
        self.assertTrue(math.isfinite(anchor.x) and math.isfinite(anchor.y))


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
