"""Mounting correction must preserve image/measurement alignment."""
import numpy as np
import pytest

from perception.config import AnchorConfig
from perception.detection.orientation import orient_detections
from perception.detection.types import BBox, Detection, Keypoint
from perception.errors import NoInferenceForFrame
from perception.pipeline import PerceptionPipeline, PreviewTap
from perception.tests.support import commissioned_config, dset


def test_rotated_box_uses_upright_torso_fraction_and_rotated_keypoints():
    raw = Detection(bbox=BBox(.1, .2, .3, .8), keypoints=(Keypoint(.2, .3, .9),))
    result = orient_detections(dset([raw]), 'rotate_180', AnchorConfig(torso_fraction=.45))
    d = result.detections[0]
    assert (d.bbox.x_min, d.bbox.y_min, d.bbox.x_max, d.bbox.y_max) == pytest.approx((.7, .2, .9, .8))
    assert (d.measured_anchor.x, d.measured_anchor.y) == pytest.approx((.8, .47))
    assert (d.keypoints[0].x, d.keypoints[0].y) == pytest.approx((.8, .7))
    assert raw.bbox.x_min == .1


def test_preview_keeps_orientation_when_sensor_has_no_new_inference():
    class Adapter:
        def infer(self, *args, **kwargs):
            raise NoInferenceForFrame('pending')
    preview = PreviewTap(fps=0)
    pipeline = PerceptionPipeline(commissioned_config(), adapter=Adapter(),
        preview=preview, orientation='rotate_180')
    raw = np.arange(18).reshape(2, 3, 3)
    outcome = pipeline.process_frame(raw, {}, frame_sequence=1, sensor_timestamp_ns=1)
    assert outcome.stage == 'inference_pending'
    np.testing.assert_array_equal(preview.take(), raw[::-1, ::-1])
