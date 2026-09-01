"""A fake ``picamera2`` module for unit-testing the video path without a camera.

It mimics exactly the picamera2 API surface that :class:`web.webd.video.VideoSource`
uses (``Picamera2()``, ``create_video_configuration``, ``request_callback``
assignment, ``configure``/``start``/``stop``/``close``, ``camera_properties``).
``start()`` spawns a producer thread that fires the request callback as fast as
``frame_delay`` allows; each request's ``make_array("main")`` returns a BGR
uint8 array of the configured size. Installing it in ``sys.modules["picamera2"]``
makes ``VideoSource.start()`` (which does a lazy ``from picamera2 import ...``)
use the fake.

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


class _FakeRequest:
    def __init__(self, arr: np.ndarray) -> None:
        self._arr = arr

    def make_array(self, name: str) -> np.ndarray:
        assert name == "main"
        return self._arr


class FakePicamera2:
    """Stands in for ``picamera2.Picamera2`` (see module docstring)."""

    def __init__(self) -> None:
        self.camera_properties = {"Model": "fake-imx500"}
        self.request_callback = None  # assigned like picamera2's property
        self._frame_delay: float = _PARAMS.get("frame_delay", 0.0)
        self._cfg_size: Tuple[int, int] = (160, 120)
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def create_video_configuration(self, main: dict,
                                   buffer_count: int = 6) -> dict:
        size = main.get("size", self._cfg_size)
        return {"main": {"size": tuple(size), "format": "XBGR8888"},
                "lores": None, "raw": {}}

    def configure(self, cfg: dict) -> None:
        self._cfg = cfg
        self._cfg_size = tuple(cfg["main"]["size"])

    def start(self) -> None:
        w, h = self._cfg_size
        arr = np.zeros((h, w, 3), dtype=np.uint8)
        arr[..., 0] = 120  # B
        arr[..., 1] = 160  # G
        arr[..., 2] = 200  # R

        def _produce() -> None:
            while self._running:
                cb = self.request_callback
                if cb is not None:
                    cb(_FakeRequest(arr))
                if self._frame_delay > 0:
                    time.sleep(self._frame_delay)

        self._running = True
        self._thread = threading.Thread(target=_produce, daemon=True,
                                        name="fake-cam-producer")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

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
