"""The shared-device arbiter: boundedness, fairness, and where a device failure goes.

These are the three claims the B path rests on when a second camera joins one accelerator, and none
of them need a Hailo to prove — they are properties of the queueing, and the queueing is where a
"30 + 30 fps" claim quietly dies under load.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.inference_gate import (  # noqa: E402
    InferenceArbiter,
    InferenceJob,
    _fairness,
)


def _job(camera: str, n: int) -> InferenceJob:
    return InferenceJob(camera_id=camera, frame_sequence=n, sensor_timestamp_ns=n * 33_333_333,
                        image=None)


def test_a_job_without_an_identity_is_refused_at_the_door():
    with pytest.raises(ValueError, match="camera_id"):
        InferenceJob(camera_id="", frame_sequence=1, sensor_timestamp_ns=1, image=None)


def test_a_zero_depth_queue_is_refused_rather_than_becoming_a_passthrough():
    with pytest.raises(ValueError, match="queue_depth"):
        InferenceArbiter(infer=lambda j: "ok", queue_depth=0)


def test_the_buffer_stays_bounded_and_keeps_the_newest_frames():
    served = []
    arbiter = InferenceArbiter(infer=served.append, queue_depth=2)
    for n in range(50):
        arbiter.submit(_job("wide", n))
    assert arbiter.pending() == {"wide": 2}
    drained = []
    while True:
        taken = arbiter._take_next_round_robin()
        if taken is None:
            break
        drained.append(taken.frame_sequence)
    assert drained == [48, 49], "积压之后留下的必须是最后两帧"
    assert arbiter.stats()["dropped_full"]["wide"] == 48


def test_round_robin_does_not_let_the_faster_camera_take_the_device():
    seen = []
    arbiter = InferenceArbiter(infer=lambda j: seen.append(j.camera_id), queue_depth=2)
    for n in range(8):
        arbiter.submit(_job("wide", n))
        arbiter.submit(_job("detail", n))
        arbiter.submit(_job("detail", n + 100))          # detail floods its own slot
        arbiter.run_once()
        arbiter.run_once()
    ratio = arbiter.stats()["fairness"]
    assert ratio is not None and ratio > 0.5, f"一条路被饿着了：{seen} / fairness {ratio}"
    assert seen.count("wide") >= 3 and seen.count("detail") >= 3


def test_a_device_that_raises_counts_and_speaks_once_but_stays_alive():
    calls = {"n": 0}

    def explodes(job):
        calls["n"] += 1
        raise RuntimeError("hailo said no")

    arbiter = InferenceArbiter(infer=explodes, queue_depth=2)
    arbiter.submit(_job("wide", 1))
    assert arbiter.run_once() == ("wide", None)
    arbiter.submit(_job("wide", 2))
    assert arbiter.run_once() == ("wide", None)
    assert calls["n"] == 2
    assert arbiter.failures == 2 and arbiter.first_error.startswith("RuntimeError")
    assert arbiter.stats()["served"].get("wide", 0) == 0, "失败的一帧不能记成服务过"


def test_the_worker_thread_serves_both_streams_and_stops_cleanly():
    served = []
    done = threading.Event()

    def recorder(job):
        served.append(job.camera_id)
        if len(served) >= 6:
            done.set()
        return "ok"

    arbiter = InferenceArbiter(infer=recorder, queue_depth=2)
    arbiter.start()
    try:
        for n in range(6):
            arbiter.submit(_job("wide", n))
            arbiter.submit(_job("detail", n))
            time.sleep(0.002)
        assert done.wait(2.0), f"线程没把两路都喂完：{served}"
    finally:
        arbiter.stop()
    assert arbiter._thread is not None and not arbiter._thread.is_alive()
    assert set(arbiter.stats()["served"]) == {"wide", "detail"}


def test_fairness_is_unknown_rather_than_zero_when_nothing_ran():
    assert _fairness({}) is None
    assert _fairness({"wide": 0, "detail": 0}) is None
    assert _fairness({"wide": 10, "detail": 5}) == 0.5


def test_two_synthetic_camera_streams_end_to_end_without_any_hardware(tmp_path):
    """The objective's named evidence: a *pair* of fake sources through the real components.

    A `SecondaryCameraStream` per sensor (the same owner the station runs) feeding the real
    arbiter, results tagged with the sensor that produced them. No /dev, no picamera2, no Hailo:
    if the shape is wrong it has to show up here, not on the station at 23:00.
    """
    import types

    from perception.detail_stream import DetailFrame, SecondaryCameraStream

    ident_wide = types.SimpleNamespace(id="cam-wide0001", source="fwnode", durable=True)
    ident_detail = types.SimpleNamespace(id="cam-detail01", source="fwnode", durable=True)
    counter = {"n": 0}

    def make_source(role: str, ident, image_tag: str) -> SecondaryCameraStream:
        def poll():
            counter["n"] += 1
            return DetailFrame(camera_id=ident.id, frame_sequence=counter["n"],
                               sensor_timestamp_ns=counter["n"] * 33_333_333,
                               image=image_tag, metadata={})
        stream = SecondaryCameraStream(role=role, ident=ident, poll=poll, queue_depth=2)
        stream.start()
        return stream

    arbiter = InferenceArbiter(infer=lambda j: (j.camera_id, j.image), queue_depth=2)
    arbiter.start()
    wide = make_source("wide", ident_wide, "wide-pixels")
    detail = make_source("detail", ident_detail, "detail-pixels")
    seen: dict = {}
    deadline = time.monotonic() + 3.0
    try:
        while time.monotonic() < deadline and len(seen) < 2:
            for source in (wide, detail):
                frame = source.latest()
                if frame is not None:
                    served = arbiter.run_once()
                    if served is not None:
                        seen[served[0]] = served[1]
                    arbiter.submit(InferenceJob(camera_id=frame.camera_id,
                                               frame_sequence=frame.frame_sequence,
                                               sensor_timestamp_ns=frame.sensor_timestamp_ns,
                                               image=frame.image))
            time.sleep(0.002)
    finally:
        for source in (wide, detail):
            source.stop()
        arbiter.stop()

    assert ("cam-wide0001", "wide-pixels") in seen.values(), seen
    assert ("cam-detail01", "detail-pixels") in seen.values(), seen
    stats = arbiter.stats()
    assert set(stats["served"]) == {"cam-wide0001", "cam-detail01"}
    assert stats["failures"] == 0
    assert wide.delivered > 0 and detail.delivered > 0
