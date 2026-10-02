"""YOLOv8-pose on Hailo-8, offline: the decode, the letterbox round trip, and the head anchor.

Nothing touches /dev/hailo0. The decode is checked against values computed by hand from the Model
Zoo's own decoding rules (v2.18 ``_yolov8_decoding``); the adapter runs on a fake runtime that hands
back the nine head tensors; the anchor is checked through the real normalisation path.
"""

import json
import os
import unittest

import numpy as np

from perception.config import AnchorConfig
from perception.detection.anchor import compute_anchor
from perception.detection.types import AnchorSource, BBox, Keypoint
from perception.model.hailo_pose import HailoYoloPoseAdapter, decode_yolov8_pose
from perception.model.manifest import ModelManifest

MANIFEST = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "model", "manifests", "hailo_yolov8s_pose_hailo8_coco.json")


def peaked_dfl(bin_index):
    """One side's 16 DFL logits with all the mass on one bin: the expectation is that bin."""
    logits = np.full(16, -30.0)
    logits[bin_index] = 30.0
    return logits


def head_tensors(person=None):
    """The nine head outputs, empty except an optional person at (stride, row, col, score,
    (l, t, r, b) bins, 17x3 raw keypoints)."""
    out = {}
    for side, stride in ((20, 32), (40, 16), (80, 8)):
        box = np.zeros((side, side, 64))
        score = np.zeros((side, side, 1))
        kpt = np.zeros((side, side, 51))
        if person is not None and person[0] == stride:
            _, row, col, s, bins, raw = person
            box[row, col] = np.concatenate([peaked_dfl(b) for b in bins])
            score[row, col, 0] = s
            kpt[row, col] = raw.reshape(51)
        out[stride] = (box, score, kpt)
    return out


class Decode(unittest.TestCase):
    def test_box_and_keypoints_follow_the_model_zoo_rules(self):
        raw = np.zeros((17, 3))
        raw[0] = (0.25, 0.75, 4.0)     # nose: x = 16 * (2*0.25 + 20) = 328, y = 16 * (1.5 + 10) = 184
        raw[5] = (0.0, 1.0, -4.0)      # left shoulder, not visible
        found = decode_yolov8_pose(head_tensors((16, 10, 20, 0.9, (3, 2, 3, 6), raw)))
        self.assertEqual(len(found), 1)
        score, box, kp = found[0]
        self.assertAlmostEqual(score, 0.9)
        cx, cy = 20.5 * 16, 10.5 * 16
        np.testing.assert_allclose(box, [cx - 48, cy - 32, cx + 48, cy + 96], atol=1e-6)
        np.testing.assert_allclose(kp[0], [328.0, 184.0, 1 / (1 + np.exp(-4.0))], atol=1e-9)
        self.assertLess(kp[5, 2], 0.05)

    def test_below_the_floor_nothing_is_decoded_and_duplicates_collapse(self):
        self.assertEqual(decode_yolov8_pose(head_tensors((16, 5, 5, 0.1, (2, 2, 2, 2), np.zeros((17, 3))))), [])
        t = head_tensors((8, 40, 40, 0.8, (5, 5, 5, 5), np.zeros((17, 3))))
        box, score, kpt = t[8]
        box[40, 41], score[40, 41, 0] = box[40, 40], 0.7      # the neighbouring cell, same person
        found = decode_yolov8_pose(t)
        self.assertEqual(len(found), 1, "NMS keeps one box per person")
        self.assertAlmostEqual(found[0][0], 0.8)


class FakeRuntime:
    def __init__(self, tensors):
        self.tensors = tensors
        self.seen = None

    def infer(self, feed):
        self.seen = list(feed.values())[0]
        out = {}
        for stride, (box, score, kpt) in self.tensors.items():
            out[f"box{stride}"], out[f"score{stride}"], out[f"kpt{stride}"] = box[None], score[None], kpt[None]
        return out


def build_adapter(tensors):
    with open(MANIFEST, encoding="utf-8") as handle:
        manifest = ModelManifest.from_dict(json.load(handle))
    adapter = HailoYoloPoseAdapter(manifest)
    adapter.anchor_cfg = AnchorConfig(target="head")
    adapter.opened = True
    adapter._infer = FakeRuntime(tensors)
    adapter._input_name = "input_0"
    adapter._pose_outputs = {s: (f"box{s}", f"score{s}", f"kpt{s}") for s in (32, 16, 8)}
    return adapter


class Adapter(unittest.TestCase):
    def test_a_person_comes_back_leg_normalised_with_a_head_anchor(self):
        """640x360 leg (pad 140): the nose at tensor (328, 184) is leg (0.5125, 44/360)."""
        raw = np.zeros((17, 3))
        raw[:, 2] = -6.0
        raw[0] = (0.25, 0.75, 4.0)
        adapter = build_adapter(head_tensors((16, 10, 20, 0.9, (3, 2, 3, 6), raw)))
        adapter.configure_stream(640, 360)
        out = adapter.infer(np.full((360, 640, 3), 9, dtype=np.uint8), {}, frame_sequence=1,
                            sensor_timestamp_ns=1, publish_timestamp_ns=2)
        self.assertEqual(len(out.detections), 1)
        d = out.detections[0]
        self.assertEqual(d.class_name, "person")
        self.assertEqual(len(d.keypoints), 17)
        self.assertAlmostEqual(d.keypoints[0].x, 328 / 640, places=4)
        self.assertAlmostEqual(d.keypoints[0].y, (184 - 140) / 360, places=4)
        self.assertEqual(d.anchor_source, AnchorSource.POSE_HEAD)
        self.assertAlmostEqual(d.measured_anchor.x, 328 / 640, places=4)
        self.assertAlmostEqual(d.measured_anchor.y, (184 - 140) / 360, places=4)

    def test_wrong_outputs_are_refused_by_name(self):
        from perception.errors import ModelRejected

        class Info:
            def __init__(self, name, shape):
                self.name, self.shape = name, shape

        adapter = build_adapter({})
        with self.assertRaises(ModelRejected):
            adapter._check_outputs([Info("x", (20, 20, 64)), Info("y", (20, 20, 1))])


def keypoints(**seen):
    """COCO-17, all unseen except the named ones: name=(x, y, score)."""
    index = {"nose": 0, "eye_l": 1, "eye_r": 2, "ear_l": 3, "ear_r": 4, "sh_l": 5, "sh_r": 6}
    out = [Keypoint(0.0, 0.0, 0.0) for _ in range(17)]
    for name, (x, y, s) in seen.items():
        out[index[name]] = Keypoint(x, y, s)
    return out


class HeadAnchor(unittest.TestCase):
    box = BBox(0.30, 0.10, 0.60, 0.95)
    head = AnchorConfig(target="head")

    def test_from_behind_the_ears_place_the_head(self):
        a, src = compute_anchor(self.box, keypoints(ear_l=(0.42, 0.20, 0.9), ear_r=(0.48, 0.22, 0.9)), self.head)
        self.assertEqual(src, AnchorSource.POSE_HEAD)
        self.assertAlmostEqual(a.x, 0.45)
        self.assertAlmostEqual(a.y, 0.21)

    def test_without_head_keypoints_the_shoulders_place_it_above_them(self):
        # Shoulders 0.10 of the width apart on a 16:9 stream: 0.178 of the height; 0.75 of that up.
        a, src = compute_anchor(self.box, keypoints(sh_l=(0.40, 0.40, 0.8), sh_r=(0.50, 0.40, 0.8)),
                                self.head, aspect=16 / 9)
        self.assertEqual(src, AnchorSource.POSE_HEAD_FROM_SHOULDERS)
        self.assertAlmostEqual(a.x, 0.45)
        self.assertAlmostEqual(a.y, 0.40 - 0.75 * 0.10 * 16 / 9)

    def test_without_a_pose_it_falls_back_to_the_box_and_says_so(self):
        a, src = compute_anchor(self.box, (), self.head)
        self.assertEqual(src, AnchorSource.BBOX_HEAD)
        self.assertAlmostEqual(a.y, 0.10 + 0.12 * 0.85)

    def test_torso_profiles_are_unchanged(self):
        _, src = compute_anchor(self.box, keypoints(sh_l=(0.40, 0.40, 0.8), sh_r=(0.50, 0.40, 0.8)),
                                AnchorConfig())
        self.assertEqual(src, AnchorSource.POSE_SHOULDERS)


if __name__ == "__main__":
    unittest.main()
