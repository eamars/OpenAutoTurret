"""§52 — detector-level deduplication and class filtering.

Every test here asserts a COUNT as well as a survivor list. §16.4's "no silent duplicate
removal" is not satisfied by the right boxes coming out: the counters are the product, and
a stage that removed a box without saying so is the same defect as a tracker that lost an
identity without saying so.
"""
from __future__ import annotations

import unittest

from perception.config import DedupConfig
from perception.detection.class_filter import filter_permitted, is_permitted, permitted_name_set
from perception.detection.containment import containment_suppression, is_contained_duplicate
from perception.detection.dedup import DetectionDeduplicator
from perception.detection.nms import class_aware_nms
from perception.detection.pose_nms import object_keypoint_similarity, pose_nms
from perception.detection.types import (BBox, Detection, DetectionCounters, DetectionSet,
                                        Keypoint, PointNorm)
from perception.events import EventLog, EventType


def det(det_id, class_id=0, class_name="person", score=0.9,
        cx=0.5, cy=0.5, w=0.1, h=0.2, keypoints=()):
    return Detection(detection_id_in_frame=det_id, class_id=class_id,
                     class_name=class_name, detector_score=score,
                     bbox=BBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                     measured_anchor=PointNorm(cx, cy), keypoints=tuple(keypoints))


def dset(detections, *, counters=None):
    return DetectionSet(model_id="test-model", model_generation=1, frame_sequence=7,
                        sensor_timestamp_ns=1_000_000_000, publish_timestamp_ns=2,
                        stream_width=1920, stream_height=1080,
                        detections=list(detections),
                        counters=counters or DetectionCounters(
                            raw_outputs=len(detections), post_model_nms=len(detections)))


class TestClassAwareNms(unittest.TestCase):
    def test_lower_scoring_overlapping_box_is_suppressed_and_counted(self):
        kept, pairs = class_aware_nms([det(1, score=0.9), det(2, score=0.6)], 0.5)
        self.assertEqual([d.detection_id_in_frame for d in kept], [1])
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0].dropped.detection_id_in_frame, 2)
        self.assertEqual(pairs[0].method, "nms")
        self.assertGreaterEqual(pairs[0].measure, 0.5)

    def test_distant_boxes_both_survive(self):
        kept, pairs = class_aware_nms([det(1, cx=0.2), det(2, cx=0.8)], 0.5)
        self.assertEqual(len(kept), 2)
        self.assertEqual(pairs, [])

    def test_suppression_never_crosses_classes(self):
        # Same pixels, different classes: a disagreement about what is there, not a
        # duplicate of one thing (§16.1).
        kept, pairs = class_aware_nms([det(1, class_id=0, class_name="person", score=0.9),
                                       det(2, class_id=18, class_name="dog", score=0.8)], 0.5)
        self.assertEqual(len(kept), 2)
        self.assertEqual(pairs, [])

    def test_ties_break_on_detection_id_not_arrival_order(self):
        first = class_aware_nms([det(2, score=0.8), det(1, score=0.8)], 0.5)
        second = class_aware_nms([det(1, score=0.8), det(2, score=0.8)], 0.5)
        self.assertEqual([d.detection_id_in_frame for d in first[0]],
                         [d.detection_id_in_frame for d in second[0]])
        self.assertEqual(first[0][0].detection_id_in_frame, 1)

    def test_threshold_range_is_enforced(self):
        with self.assertRaises(ValueError):
            class_aware_nms([det(1)], 0.0)
        with self.assertRaises(ValueError):
            class_aware_nms([det(1)], 1.5)


class TestContainment(unittest.TestCase):
    def cfg(self, **overrides):
        base = dict(class_aware_nms=False, nms_iou=None, containment=True,
                    containment_ratio=0.9, center_distance_norm=0.08,
                    containment_max_scale_ratio=4.0)
        base.update(overrides)
        return DedupConfig(**base)

    def test_nested_pair_with_moderate_iou_is_suppressed(self):
        tight = det(1, cx=0.5, cy=0.5, w=0.08, h=0.16, score=0.9)
        loose = det(2, cx=0.5, cy=0.52, w=0.10, h=0.20, score=0.7)
        iou = tight.bbox.iou(loose.bbox)
        self.assertLess(iou, 0.85, "the pair must be one IoU would keep")
        kept, pairs = containment_suppression([loose, tight], self.cfg())
        self.assertEqual([d.detection_id_in_frame for d in kept], [1])
        self.assertEqual(pairs[0].method, "containment")

    def test_two_people_one_behind_the_other_are_not_merged(self):
        # Near-perfect containment because the boxes are concentric, but the boxes differ
        # in SIZE by more than one body can: two people at different depths (§16.2).
        near = det(1, cx=0.5, cy=0.5, w=0.20, h=0.40, score=0.9)
        far = det(2, cx=0.5, cy=0.5, w=0.05, h=0.10, score=0.7)
        self.assertGreater(near.bbox.containment(far.bbox), 0.95)
        kept, pairs = containment_suppression([near, far], self.cfg())
        self.assertEqual(len(kept), 2, "losing a person is worse than a duplicate")
        self.assertEqual(pairs, [])

    def test_centre_distance_alone_blocks_a_merge(self):
        cfg = self.cfg(containment_max_scale_ratio=100.0)
        big = det(1, cx=0.5, cy=0.30, w=0.30, h=0.60, score=0.9)
        small = det(2, cx=0.5, cy=0.70, w=0.28, h=0.56, score=0.6)
        duplicate, _measure = is_contained_duplicate(big, small, cfg)
        self.assertFalse(duplicate, "centres 0.4 apart are not the same body")
        kept, _pairs = containment_suppression([big, small], cfg)
        self.assertEqual(len(kept), 2)

    def test_disabled_stage_touches_nothing(self):
        boxes = [det(1), det(2)]
        kept, pairs = containment_suppression(boxes, self.cfg(containment=False))
        self.assertEqual(len(kept), 2)
        self.assertEqual(pairs, [])


class TestPoseNms(unittest.TestCase):
    def skeleton(self, dx=0.0, score=0.9):
        return [Keypoint(0.5 + dx + i * 0.01, 0.3 + i * 0.02, score) for i in range(17)]

    def test_identical_skeletons_score_one_and_the_weaker_is_dropped(self):
        oks = object_keypoint_similarity(self.skeleton(), self.skeleton(), 0.6, 0.1)
        self.assertAlmostEqual(oks, 1.0, places=9)
        kept, pairs = pose_nms([det(1, score=0.9, keypoints=self.skeleton()),
                                det(2, score=0.6, keypoints=self.skeleton(0.002))],
                               0.5, 0.1)
        self.assertEqual([d.detection_id_in_frame for d in kept], [1])
        self.assertEqual(pairs[0].method, "pose")

    def test_two_different_skeletons_survive(self):
        kept, pairs = pose_nms([det(1, score=0.9, keypoints=self.skeleton()),
                                det(2, score=0.8, keypoints=self.skeleton(0.4))], 0.5, 0.1)
        self.assertEqual(len(kept), 2)
        self.assertEqual(pairs, [])

    def test_low_confidence_keypoints_are_not_counted_as_mismatches(self):
        a = self.skeleton(score=0.9)
        b = self.skeleton(0.4, score=0.0)     # the model placed nothing on the second one
        self.assertEqual(object_keypoint_similarity(a, b, 0.6, 0.1), 0.0)

    def test_box_only_detections_pass_through(self):
        kept, _pairs = pose_nms([det(1, score=0.9, keypoints=self.skeleton()),
                                 det(2, score=0.8)], 0.5, 0.1)
        self.assertEqual(len(kept), 2)


class TestClassFilter(unittest.TestCase):
    def test_only_permitted_classes_survive_and_the_count_is_exact(self):
        boxes = dset([det(1, class_id=0, class_name="person"),
                      det(2, class_id=62, class_name="chair"),
                      det(3, class_id=18, class_name="dog"),
                      det(4, class_id=0, class_name="Person")])
        out = filter_permitted(boxes, ["person"])
        self.assertEqual([d.detection_id_in_frame for d in out.detections], [1, 4])
        self.assertEqual(out.counters.class_filtered, 2)

    def test_the_input_set_is_not_mutated(self):
        boxes = dset([det(1), det(2, class_name="chair")])
        filter_permitted(boxes, ["person"])
        self.assertEqual(len(boxes.detections), 2)
        self.assertEqual(boxes.counters.class_filtered, 0)

    def test_empty_permitted_list_permits_nothing(self):
        # The fail-safe direction: a blank class list must not turn every blob into a
        # candidate target (§15).
        out = filter_permitted(dset([det(1)]), [])
        self.assertEqual(out.detections, [])
        self.assertEqual(out.counters.class_filtered, 1)
        self.assertFalse(is_permitted(det(1), permitted_name_set([])))

    def test_pre_existing_counters_accumulate(self):
        boxes = dset([det(1, class_name="chair")],
                     counters=DetectionCounters(raw_outputs=5, class_filtered=3))
        out = filter_permitted(boxes, ["person"])
        self.assertEqual(out.counters.class_filtered, 4)
        self.assertEqual(boxes.counters.class_filtered, 3)


class TestDedupPipeline(unittest.TestCase):
    def config(self, **overrides):
        base = dict(class_aware_nms=True, nms_iou=0.5, containment=True,
                    containment_ratio=0.9, center_distance_norm=0.08,
                    containment_max_scale_ratio=4.0, pose_nms=False)
        base.update(overrides)
        return DedupConfig(**base)

    def test_one_person_does_not_leave_two_boxes_and_every_counter_is_exact(self):
        events = EventLog(capacity=64)
        dedup = DetectionDeduplicator(self.config(), event_log=events)
        result = dedup.run(dset([det(1, score=0.9), det(2, score=0.7), det(3, cx=0.9)]))
        self.assertEqual(len(result.detections.detections), 2)
        self.assertEqual(result.detections.counters.host_duplicates_suppressed, 1)
        self.assertEqual(result.detections.counters.containment_suppressed, 0)
        self.assertEqual(result.detections.counters.post_model_nms, 3,
                         "the input count must survive for §16.4's audit")
        self.assertEqual(events.count(EventType.DETECTION_DUPLICATE_SUPPRESSED), 1)
        event = events.recent(1)[0]
        self.assertEqual(event.fields["method"], "nms")

    def test_containment_and_nms_counters_are_separate(self):
        cfg = self.config(nms_iou=0.99)      # NMS effectively off; containment must act
        dedup = DetectionDeduplicator(cfg)
        result = dedup.run(dset([det(1, cx=0.5, cy=0.5, w=0.08, h=0.16, score=0.9),
                                 det(2, cx=0.5, cy=0.52, w=0.10, h=0.20, score=0.7)]))
        self.assertEqual(result.detections.counters.host_duplicates_suppressed, 0)
        self.assertEqual(result.detections.counters.containment_suppressed, 1)

    def test_uncommissioned_thresholds_skip_the_stage_and_say_so(self):
        cfg = DedupConfig(class_aware_nms=True, nms_iou=None, containment=True,
                          containment_ratio=None, center_distance_norm=None)
        result = DetectionDeduplicator(cfg).run(dset([det(1), det(2)]))
        self.assertEqual(len(result.detections.detections), 2)
        self.assertEqual(len(result.stages_skipped), 2, "both stages must report themselves")
        self.assertTrue(any("nms" in s for s in result.stages_skipped))

    def test_pose_stage_runs_only_when_the_frame_has_keypoints(self):
        # Box NMS is off here so the two identical boxes survive to the pose stage: with
        # NMS on, the pair is already one box and this test would be measuring nothing.
        cfg = self.config(pose_nms=True, class_aware_nms=False, containment=False)
        without = DetectionDeduplicator(cfg).run(dset([det(1), det(2)]))
        self.assertEqual(len(without.detections.detections), 2, "no keypoints, no pose stage")
        self.assertEqual(without.detections.counters.pose_duplicates_suppressed, 0)
        self.assertEqual(without.stages_skipped, [])

        with_pose = DetectionDeduplicator(cfg).run(dset([
            det(1, score=0.9, keypoints=[Keypoint(0.5, 0.3 + i * 0.02, 0.9) for i in range(17)]),
            det(2, score=0.6, keypoints=[Keypoint(0.5, 0.3 + i * 0.02, 0.9) for i in range(17)])]))
        self.assertEqual(with_pose.detections.counters.pose_duplicates_suppressed, 1)

    def test_input_set_is_never_mutated(self):
        boxes = dset([det(1, score=0.9), det(2, score=0.7)])
        before = len(boxes.detections)
        DetectionDeduplicator(self.config()).run(boxes)
        self.assertEqual(len(boxes.detections), before)
        self.assertEqual(boxes.counters.host_duplicates_suppressed, 0)

    def test_summary_reports_in_and_out(self):
        result = DetectionDeduplicator(self.config()).run(dset([det(1), det(2)]))
        summary = result.summary()
        self.assertEqual(summary["in"], 2)
        self.assertEqual(summary["out"], 1)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
