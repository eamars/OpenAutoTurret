"""The Hailo input tensor, built offline: the leg's height is allowed to vary, the width is not.

Nothing here touches /dev/hailo0. The adapter is handed a fake runtime that records the tensor it was
given, which is the only way to check a letterbox on a machine without the accelerator -- and the only
way to have caught the constant that rejected every frame of the 640x360 leg on the first production
boot.
"""

import json
import os
import unittest

import numpy as np

from perception.model.hailo_yolo import HailoYoloAdapter
from perception.model.manifest import ModelManifest

MANIFEST = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "model", "manifests", "hailo_yolov8n_hailo8_coco.json")


class FakeRuntime:
    """Stands in for the HailoRT binding: records the tensor, returns an empty NMS batch."""

    def __init__(self, person_box=None):
        self.seen = None
        # Columns are whatever the caller hands back, so the decode under test sees a real box.
        self.person_box = person_box

    def infer(self, feed):
        self.seen = list(feed.values())[0]
        classes = [np.zeros((0, 5), dtype=np.float32) for _ in range(80)]
        if self.person_box is not None:
            classes[0] = np.array([self.person_box], dtype=np.float32)   # COCO person is class 0
        return {"output0": [classes]}


def build_adapter():
    with open(MANIFEST, encoding="utf-8") as handle:
        manifest = ModelManifest.from_dict(json.load(handle))
    adapter = HailoYoloAdapter(manifest)
    runtime = FakeRuntime()
    adapter.opened = True
    adapter._infer = runtime
    adapter._input_name = "input_0"
    adapter._output_name = "output0"
    return adapter, runtime


class Letterbox(unittest.TestCase):
    def _run(self, width, height):
        adapter, runtime = build_adapter()
        adapter.configure_stream(width, height)
        frame = np.full((height, width, 3), 7, dtype=np.uint8)
        adapter.infer(frame, {}, frame_sequence=1, sensor_timestamp_ns=1,
                      publish_timestamp_ns=2)
        self.assertEqual(runtime.seen.shape, (1, 640, 640, 3),
                         "the HEF is fed a 640x640 tensor whatever the leg's height is")
        return runtime.seen[0]

    def test_640x480_keeps_the_measured_geometry(self):
        """Regression: the probes measured this path, so 80 px of pad each side must not move."""
        tensor = self._run(640, 480)
        self.assertTrue(np.all(tensor[:80] == 114), "the top pad must stay padding")
        self.assertTrue(np.all(tensor[80:560] == 7), "the picture must land where the probes put it")
        self.assertTrue(np.all(tensor[560:] == 114))

    def test_640x360_is_centred_by_computation(self):
        """The production leg: 360 tall centred in 640 is 140 of pad, and nothing is hard-coded."""
        tensor = self._run(640, 360)
        self.assertTrue(np.all(tensor[:140] == 114))
        self.assertTrue(np.all(tensor[140:500] == 7))
        self.assertTrue(np.all(tensor[500:] == 114))

    def test_a_leg_that_is_not_the_configured_one_is_named_not_rescaled(self):
        """A frame that is not the configured leg is a config bug: say so, do not quietly resize."""
        from perception.errors import ModelRejected
        adapter, runtime = build_adapter()
        adapter.configure_stream(640, 360)
        wrong = np.zeros((480, 640, 3), dtype=np.uint8)
        with self.assertRaises(ModelRejected) as caught:
            adapter.infer(wrong, {}, frame_sequence=1, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        self.assertIn("640x360", str(caught.exception))
        self.assertIsNone(runtime.seen, "a rejected frame must never reach the accelerator")

    def test_a_wrong_width_is_named_too(self):
        from perception.errors import ModelRejected
        adapter, _runtime = build_adapter()
        adapter.configure_stream(1280, 720)
        wide = np.zeros((720, 1280, 3), dtype=np.uint8)
        with self.assertRaises(ModelRejected) as caught:
            adapter.infer(wide, {}, frame_sequence=1, sensor_timestamp_ns=1,
                          publish_timestamp_ns=2)
        self.assertIn("640", str(caught.exception))


class PadRoundTrip(unittest.TestCase):
    """The pad is created by the encoder, so the decoder has to undo it. No exceptions.

    Tensor rows arrive normalised to the 640x640 letterboxed tensor; rows going out are promised to be
    normalised to the frame we were fed. x is whole-width; y has the pad taken back out.
    """

    def _rows(self, leg_h, tensor_box):
        from perception.model.hailo_yolo import HailoYoloAdapter
        import json as _json
        import os as _os
        from perception.model.manifest import ModelManifest
        with open(MANIFEST, encoding="utf-8") as handle:
            manifest = ModelManifest.from_dict(_json.load(handle))
        adapter = HailoYoloAdapter(manifest)
        runtime = FakeRuntime(person_box=list(tensor_box))
        adapter.opened = True
        adapter._infer = runtime
        adapter._input_name = "input_0"
        adapter._output_name = "output0"
        adapter.configure_stream(640, leg_h)
        captured = {}

        class _Set:
            detections = ()

        def capture(rows, **_kw):
            captured["rows"] = [list(r) for r in rows]
            return _Set()

        adapter._rows_to_set = capture
        adapter.infer(np.full((leg_h, 640, 3), 9, dtype=np.uint8), {}, frame_sequence=1,
                      sensor_timestamp_ns=1, publish_timestamp_ns=2)
        return captured.get("rows", []), adapter

    def test_a_box_in_the_picture_comes_back_leg_normalised(self):
        """640x360 leg, pad 140: tensor y 0.359375..0.640625 is leg y 0.25..0.75."""
        rows, adapter = self._rows(360, [0.359375, 0.2, 0.640625, 0.6, 0.9])
        self.assertEqual(len(rows), 1, "a box inside the picture must survive the decode")
        score, klass, ymin, xmin, ymax, xmax = rows[0]
        self.assertAlmostEqual(ymin, 0.25, places=5)
        self.assertAlmostEqual(ymax, 0.75, places=5)
        self.assertAlmostEqual(xmin, 0.2, places=5)   # float32 round trip, so not exact
        self.assertAlmostEqual(xmax, 0.6, places=5)   # the leg is whole-width: x must not move
        self.assertAlmostEqual(score, 0.9, places=5)   # float32 again
        self.assertEqual(int(klass), 0)                # COCO person

    def test_a_box_only_in_the_padding_is_counted_not_smudged(self):
        """A sighting inside the letterbox is not a sighting: drop it, but say so in a counter."""
        rows, adapter = self._rows(360, [0.0, 0.2, 0.1, 0.6, 0.9])
        self.assertEqual(rows, [], "a box entirely inside the pad must not become an edge box")
        self.assertEqual(adapter.detections_pad_dropped, 1)

    def test_the_measured_480_leg_is_also_corrected(self):
        """The probes' own leg: at pad 80 the uncorrected y was 33% tall, and that was already wrong."""
        rows, _adapter = self._rows(480, [0.25, 0.1, 0.75, 0.4, 0.8])
        self.assertEqual(len(rows), 1)
        _score, _klass, ymin, _xmin, ymax, _xmax = rows[0]
        self.assertAlmostEqual(ymin, 80.0 / 480.0, places=5)   # (0.25*640 - 80) / 480
        self.assertAlmostEqual(ymax, 400.0 / 480.0, places=5)  # (0.75*640 - 80) / 480



if __name__ == "__main__":
    unittest.main(verbosity=2)
