"""Contract tests for the REAL camera source — pinned against a FAKE picamera2.

These exist because of what the hardware taught us on 2026-09-03 (measured on the
station's IMX500, see docs/archive/post_homing_test_queue.md P7/P8):

  * ``request_callback`` is DEPRECATED on picamera2 0.3.37 and silently maps onto
    ``post_callback``, which only fires for capture-FILE jobs. A callback-based
    stream therefore receives NOTHING: the daemon looks alive and starves.
  * ``CompletedRequest.get_metadata()`` takes NO stream argument (passing "main"
    is a TypeError); ``make_array()`` DOES take the stream name.
  * ``SensorTimestamp`` is host CLOCK_MONOTONIC, ~1 frame behind the consumer's
    ``time.monotonic_ns()`` — the domain controld needs (§6.2/§11).

A fake cannot prove the camera works — ``tools/camera_bringup_probe.py`` does
that on real glass. What the fake *can* do is fail loudly if someone moves
`Picamera2FrameSource` back to the callback pattern or calls the request API the
wrong way, which no test would otherwise catch until it is on the station.
"""
from __future__ import annotations

import sys
import time
import types

import numpy as np
import pytest

from vision.frame_source import Picamera2FrameSource


class FakeRequest:
    def __init__(self, calls, meta, arr):
        self._calls = calls
        self._meta = meta
        self._arr = arr

    def get_metadata(self):                     # NO stream arg on 0.3.37
        self._calls.append(("get_metadata",))
        return self._meta

    def make_array(self, name):
        self._calls.append(("make_array", name))
        return self._arr

    def release(self):
        self._calls.append(("release",))


class FakePicamera2:
    """Stands in for the camera object; records every interaction."""

    last: "FakePicamera2 | None" = None

    def __init__(self):
        self.calls: list[tuple] = []
        self.requests: list[FakeRequest] = []
        self.stream_configuration = {"size": (8, 6)}   # w,h
        self.configured = []
        self.started = False
        self.closed = False
        FakePicamera2.last = self

    # deprecated-attribute trap: any assignment is recorded and asserted against
    def __setattr__(self, name, value):
        if name in ("request_callback", "post_callback", "pre_callback"):
            self.__dict__.setdefault("calls", []).append(("SET_CALLBACK", name))
        super().__setattr__(name, value)

    def create_video_configuration(self, main=None, buffer_count=None):
        self.calls.append(("create_video_configuration", main, buffer_count))
        return {"controls": {}, "main": main}

    def configure(self, cfg):
        self.calls.append(("configure",))
        self.configured.append(cfg)

    def start(self, config=None):
        self.calls.append(("start",))
        self.started = True

    def stop(self):
        self.calls.append(("stop",))

    def close(self):
        self.calls.append(("close",))
        self.closed = True

    def capture_request(self):
        self.calls.append(("capture_request",))
        if not self.requests:
            raise AssertionError("test forgot to queue a request")
        return self.requests.pop(0)


@pytest.fixture
def fake_picamera2(monkeypatch):
    mod = types.ModuleType("picamera2")
    mod.Picamera2 = FakePicamera2
    monkeypatch.setitem(sys.modules, "picamera2", mod)
    return FakePicamera2


def _source(fake_cls, orientation="none", rpk=""):
    src = Picamera2FrameSource("", rpk, image_size=(8, 6),
                              orientation=orientation, framerate_hz=30.0)
    return src


def _queue(cam, meta=None, arr=None):
    cam.requests.append(FakeRequest(cam.calls, meta if meta is not None else {},
                                    arr if arr is not None
                                    else np.zeros((6, 8, 4), dtype=np.uint8)))


def test_start_configures_a_video_stream_and_starts_once(fake_picamera2):
    src = _source(fake_picamera2)
    src.start()
    cam = fake_picamera2.last
    names = [c[0] for c in cam.calls]
    assert "create_video_configuration" in names and "start" in names
    # The main stream is requested as an 8-bit RGB stream: that is what
    # make_array("main") must be able to hand us as a numpy frame.
    cfg_call = next(c for c in cam.calls if c[0] == "create_video_configuration")
    assert cfg_call[1]["format"] == "XRGB8888"
    assert src.image_size() == (8, 6)      # adopted from stream_configuration


def test_capture_pulls_one_request_and_always_releases_it(fake_picamera2):
    src = _source(fake_picamera2)
    src.start()
    cam = fake_picamera2.last
    stamp = 1_000_000
    _queue(cam, {"SensorTimestamp": stamp})
    cap = src.capture()
    names = [c[0] for c in cam.calls]
    assert names.index("capture_request") < names.index("get_metadata")
    assert ("make_array", "main") in cam.calls
    assert "release" in names
    assert cap.sensor_timestamp_ns == stamp          # the sensor's own stamp wins
    assert cap.frame_sequence == 0
    _queue(cam, {"SensorTimestamp": stamp + 33_000_000})
    assert src.capture().frame_sequence == 1         # and increments


def test_deprecated_callback_api_is_never_touched(fake_picamera2):
    """The bug this pins: assigning request_callback looks right and yields 0 frames."""
    src = _source(fake_picamera2)
    src.start()
    cam = fake_picamera2.last
    _queue(cam, {"SensorTimestamp": 5})
    src.capture()
    src.stop()
    assert [c for c in cam.calls if c[0] == "SET_CALLBACK"] == []


def test_missing_sensor_timestamp_falls_back_to_monotonic_and_is_counted(
        fake_picamera2):
    src = _source(fake_picamera2)
    src.start()
    cam = fake_picamera2.last
    before = time.monotonic_ns()
    _queue(cam, {})                       # metadata WITHOUT SensorTimestamp
    cap = src.capture()
    assert cap.sensor_timestamp_ns >= before
    assert cap.sensor_timestamp_ns < time.monotonic_ns() + 10**9
    assert src.frames_without_sensor_timestamp == 1   # §11 degradation is visible
    _queue(cam, {"SensorTimestamp": 12345})
    src.capture()
    assert src.frames_without_sensor_timestamp == 1   # a real stamp is not counted


def test_orientation_is_applied_to_pixels_and_boxes_together(fake_picamera2):
    """29.1: pixels and geometry must live in the same corrected frame."""
    arr = np.zeros((6, 8, 4), dtype=np.uint8)
    arr[0, 0] = 255                       # top-left corner marker
    src = _source(fake_picamera2, orientation="rotate_180")
    src.start()
    cam = fake_picamera2.last
    # Shape A: (score, class_id, x, y, w, h) in PIXELS. A non-numeric label is a
    # ValueError inside the parser and drops that object (the safe direction), so
    # a real class id is used here.
    _queue(cam, {"SensorTimestamp": 7,
                 "Objects": [(0.9, 2, 1.0, 1.0, 2.0, 2.0)]}, arr)
    cap = src.capture()
    assert cap.image[5, 7, 0] == 255                  # corner moved to bottom-right
    assert cap.image[0, 0, 0] == 0
    assert cap.detections, "orientation must not drop the detection"
    d = cap.detections[0]
    assert d.class_id == 2 and d.confidence == pytest.approx(0.9)
    # px box (1,1)-(3,3) rotated 180 degrees over an 8x6 frame -> (5,3)-(7,5).
    assert (d.bbox_x_min_px, d.bbox_y_min_px) == pytest.approx((5.0, 3.0), abs=0.2)
    assert (d.bbox_x_max_px, d.bbox_y_max_px) == pytest.approx((7.0, 5.0), abs=0.2)


def test_stop_releases_the_camera_even_after_a_failed_capture(fake_picamera2):
    src = _source(fake_picamera2)
    src.start()
    cam = fake_picamera2.last
    with pytest.raises(RuntimeError):       # wrapped: no stack, one clear line
        src.capture()
    src.stop()
    names = [c[0] for c in cam.calls]
    assert "stop" in names and "close" in names
    with pytest.raises(RuntimeError):
        src.capture()                       # not restartable by accident


def test_rpk_configuration_failure_is_not_fatal(fake_picamera2, capsys):
    """No AI stack on this platform: a failed RPK config must stream anyway."""
    src = _source(fake_picamera2, rpk="/nonexistent/model.rpk")
    src.start()
    cam = fake_picamera2.last
    assert src.detection_backend == "none"
    assert src.detection_note
    _queue(cam, {"SensorTimestamp": 1})
    assert src.capture().sensor_timestamp_ns == 1      # frames keep flowing
    assert "WITHOUT detections" in capsys.readouterr().out
