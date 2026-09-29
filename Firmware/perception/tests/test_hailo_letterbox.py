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

    def __init__(self):
        self.seen = None

    def infer(self, feed):
        self.seen = list(feed.values())[0]
        return {"output0": [([np.zeros((0, 5), dtype=np.float32) for _ in range(80)])]}


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
