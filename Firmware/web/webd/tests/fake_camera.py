"""A fake ``picamera2`` module for unit-testing the video path without a camera.

It mimics the picamera2 0.3.37 surface that :class:`web.webd.video.VideoSource`
actually uses now — ``Picamera2()``, ``create_video_configuration``,
``configure``/``start``/``stop``/``close``, ``camera_properties`` and
**``capture_request()``** (pull), where each request exposes
``make_array("main")`` + ``release()``. Installing it in
``sys.modules["picamera2"]`` makes ``VideoSource.start()`` (which does a lazy
``from picamera2 import ...``) use the fake.

Two deliberate modelling choices, both learned on the real IMX500 (see
docs/post_homing_test_queue.md, P12/P7):

  * Frames arrive at the CONSUMER's pace: ``capture_request()`` blocks about one
    frame period, like the real pipeline. A fake that returned instantly would
    model a spin loop, not a camera.
  * Assigning ``request_callback`` / ``post_callback`` RAISES. On the real
    station that assignment is deprecated, delivers ~0.8 frames/s and wedges
    ``stop()``; the fake refuses it so the design cannot silently slide back.

``VideoSource.start()`` constructs the camera with no arguments, so per-test
producer knobs are set through :func:`install`, which records them in the
module-level ``_PARAMS`` that ``FakePicamera2.__init__`` reads.
"""
from __future__ import annotations

import sys
import threading
import time
import types
from typing import Optional, Tuple

import numpy as np

# Per-install producer knobs (read by FakePicamera2.__init__).
_PARAMS: dict = {"frame_delay": 0.0}
_ABSENT = object()

DEPRECATED = ("request_callback", "post_callback", "pre_callback")


class _FakeRequest:
    """One completed request. `release()` is mandatory in the real API too."""

    def __init__(self, arr: np.ndarray, cam: "FakePicamera2") -> None:
        self._arr = arr
        self._cam = cam
        self.released = False

    def make_array(self, name: str) -> np.ndarray:
        assert name == "main"
        if self.released:
            raise AssertionError("make_array() after release()")
        return self._arr

    def get_metadata(self) -> dict:
        return {"SensorTimestamp": time.monotonic_ns()}

    def release(self) -> None:
        self.released = True
        self._cam.released += 1


class FakePicamera2:
    """Stands in for ``picamera2.Picamera2`` (see module docstring)."""

    def __init__(self) -> None:
        self.camera_properties = {"Model": "fake-imx500"}
        self._frame_delay: float = _PARAMS.get("frame_delay", 0.0)
        self._cfg_size: Tuple[int, int] = (160, 120)
        self._running = False
        self.requested = 0
        self.released = 0
        self.stopped = False

    def __setattr__(self, name, value):
        if name in DEPRECATED:
            raise AssertionError(
                f"{name} is deprecated on picamera2 0.3.37: it maps to "
                "post_callback, fires ~0.8x/s and wedges stop(). Pull "
                "capture_request() instead.")
        super().__setattr__(name, value)

    def create_video_configuration(self, main: dict,
                                   buffer_count: int = 6) -> dict:
        size = main.get("size", self._cfg_size)
        return {"main": {"size": tuple(size), "format": "XBGR8888"},
                "controls": {}, "lores": None, "raw": {}}

    def configure(self, cfg: dict) -> None:
        self._cfg = cfg
        self._cfg_size = tuple(cfg["main"]["size"])

    def start(self) -> None:
        self._running = True
        self.stopped = False

    def capture_request(self) -> _FakeRequest:
        """Block about one frame period, then hand out a completed request.

        After stop() this raises, exactly like the real camera stack — that is
        what ends `VideoSource._pull_loop()` when the camera dies underneath it.
        """
        if not self._running:
            raise RuntimeError("camera is stopped")
        # A real pipeline paces the consumer; 2 ms floor keeps the fake from
        # becoming a busy-spin while still being fast enough for test budgets.
        time.sleep(self._frame_delay if self._frame_delay > 0 else 0.002)
        w, h = self._cfg_size
        arr = np.zeros((h, w, 3), dtype=np.uint8)
        arr[..., 0] = 120  # B
        arr[..., 1] = 160  # G
        arr[..., 2] = 200  # R
        self.requested += 1
        return _FakeRequest(arr, self)

    def stop(self) -> None:
        self._running = False
        self.stopped = True

    def close(self) -> None:
        self.stop()


def make_module() -> types.ModuleType:
    mod = types.ModuleType("picamera2")
    mod.Picamera2 = FakePicamera2  # type: ignore[attr-defined]
    return mod


def install(frame_delay: float = 0.0) -> object:
    """Install the fake in ``sys.modules["picamera2"]``; return the prior value
    (or ``_ABSENT``) so the caller can restore it in tearDown."""
    _PARAMS["frame_delay"] = frame_delay
    prev = sys.modules.get("picamera2", _ABSENT)
    sys.modules["picamera2"] = make_module()
    return prev


def restore(prev) -> None:
    if prev is None or prev is _ABSENT:
        sys.modules.pop("picamera2", None)
    else:
        sys.modules["picamera2"] = prev
