"""The contract tests for DetectionFrame -- no camera, no Hailo, no transport.

These exist because the dual-camera validation produced results that were correct only while the
camera identity and the exposure timestamp were carried by hand: the moment two feeds share one
accelerator, a dropped field silently turns "the wide saw a person" into "some camera saw
something".
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.detection_frame import (  # noqa: E402
    Detection,
    DetectionFrame,
    FrameShapeError,
    camera_id_from_path,
    empty_frame,
)

WIDE_PATH = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx500@1a"
HQ_PATH = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx477@1a"


def _frame(**overrides) -> DetectionFrame:
    payload = dict(camera_id=camera_id_from_path(HQ_PATH, "imx477"),
                   capture_timestamp_ns=40_759_048_983_000,
                   inference_timestamp_ns=40_759_048_983_000 + 29_000_000,
                   detections=(Detection(class_id=0, label="person", confidence=0.87,
                                         bbox=(0.1, 0.2, 0.3, 0.4), source="hailo"),),
                   source_width=640, source_height=360, inference_source="isp_lores",
                   frame_sequence=8047)
    payload.update(overrides)
    return DetectionFrame(**payload)


def test_identity_survives_the_enumeration_index_moving():
    """The station refuses to trust ``Num``; so does this id."""
    as_zero = camera_id_from_path(HQ_PATH, "imx477")
    assert as_zero == camera_id_from_path(HQ_PATH, "imx477")
    assert as_zero != camera_id_from_path(WIDE_PATH, "imx500")
    # A different i2c bus is a different socket, even with the same driver name.
    other_bus = HQ_PATH.replace("i2c@80000", "i2c@90000")
    assert camera_id_from_path(other_bus, "imx477") != as_zero
    assert "0" != as_zero and "1" != as_zero, "index-shaped identities are how mixups get in"


def test_a_missing_capture_stamp_stays_unknown_rather_than_becoming_zero():
    frame = _frame(capture_timestamp_ns=None)
    assert frame.latency_ms() is None, "0 ms 会被读成'流水线是瞬时的'，而未知不是零"
    payload = frame.to_dict()
    assert payload["capture_timestamp_ns"] is None
    assert payload["latency_ms"] is None
    assert DetectionFrame.from_dict(payload).capture_timestamp_ns is None


def test_latency_is_measured_between_exposure_start_and_a_completed_inference():
    assert _frame().latency_ms() == pytest.approx(29.0, abs=1e-6)
    backwards = _frame(inference_timestamp_ns=_frame().capture_timestamp_ns - 1_000)
    with pytest.raises(FrameShapeError, match="not comparable"):
        backwards.latency_ms()


def test_geometry_and_confidence_are_refused_rather_than_silently_wrong():
    with pytest.raises(FrameShapeError, match="outside the normalised frame"):
        Detection(0, "person", 0.5, (1.4, 0.0, 0.1, 0.1), "hailo")
    with pytest.raises(FrameShapeError, match="runs off the frame"):
        Detection(0, "person", 0.5, (0.9, 0.0, 0.5, 0.1), "hailo")
    with pytest.raises(FrameShapeError, match="confidence"):
        Detection(0, "person", 1.7, (0.1, 0.1, 0.2, 0.2), "hailo")
    with pytest.raises(FrameShapeError, match="which detector"):
        Detection(0, "person", 0.5, (0.1, 0.1, 0.2, 0.2), "")
    with pytest.raises(FrameShapeError, match="camera_id"):
        _frame(camera_id="")


def test_boxes_convert_to_source_pixels_because_the_gimbal_needs_pixels():
    frame = _frame(source_width=2028, source_height=1520)
    assert frame.pixel_boxes() == [(203, 304, 608, 608)]


def test_the_wire_form_carries_everything_the_turret_acts_on():
    payload = _frame().to_dict()
    for key in ("camera_id", "capture_timestamp_ns", "inference_timestamp_ns", "latency_ms",
                "source_size", "inference_source", "detections"):
        assert key in payload, f"{key} 掉了，归属或延迟就断了"
    back = DetectionFrame.from_dict(payload)
    assert back.camera_id == _frame().camera_id
    assert back.detections[0].bbox == pytest.approx((0.1, 0.2, 0.3, 0.4))
    assert back.latency_ms() == pytest.approx(29.0, abs=1e-3)


def test_empty_frame_helper_is_the_shortest_honest_way_to_report_a_miss():
    # 45 ms 在纳秒域里是 45_000_000 ns：这里手算错过一次，所以把两个戳都写成量级清楚的数。
    frame = empty_frame(camera_id_from_path(WIDE_PATH, "imx500"),
                        1_000_000_000, 1_045_000_000, (640, 360), "isp_lores")
    assert frame.detections == ()
    assert frame.latency_ms() == pytest.approx(45.0, abs=1e-6)
