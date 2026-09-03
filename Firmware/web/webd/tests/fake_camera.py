"""A fake ``picamera2`` module for unit-testing the video path without a camera.

It mimics the picamera2 0.3.37 surface that :class:`web.webd.video.VideoSource`
actually uses now — ``Picamera2()``, ``create_video_configuration``,
``configure``/``start``/``stop``/``close``, ``camera_properties`` and
**``capture_request()``** (pull), where each request exposes
``make_array("main")`` + ``release()``. Installing it in
``sys.modules["picamera2"]`` makes ``VideoSource.start()`` (which does a lazy
``from picamera2 import ...``) use the fake.

Two deliberate modelling choices, both learned on the real IMX500 (see
docs/archive/post_homing_test_queue.md, P12/P7):

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
# `pixel_format` is what the fake claims its main stream is, and `neutral_only` paints a
# scene with no colour in it (used to prove the colour-order check admits when it has seen
# nothing rather than reporting success).
_PARAMS: dict = {"frame_delay": 0.0, "pixel_format": "XBGR8888", "neutral_only": False}
_ABSENT = object()

DEPRECATED = ("request_callback", "post_callback", "pre_callback")


class _FakeRequest:
    """One completed request. `release()` is mandatory in the real API too."""

    def __init__(self, arr: np.ndarray, cam: "FakePicamera2",
                 truth_rgb: "Optional[np.ndarray]" = None) -> None:
        self._arr = arr
        self._cam = cam
        self._truth = truth_rgb
        self.released = False

    def make_array(self, name: str) -> np.ndarray:
        assert name == "main"
        if self.released:
            raise AssertionError("make_array() after release()")
        return self._arr

    def make_image(self, name: str = "main"):
        """The same frame as a PIL image, as the SCENE really looks.

        The real picamera2 builds this through a path it owns end to end, which is what makes it
        a useful reference when the question is whether our own channel decode is right. The
        fake therefore answers from the scene it painted -- not by running the same convention
        the code under test uses, which would make the comparison circular and the test worth
        nothing.
        """
        assert name == "main"
        from PIL import Image
        return Image.fromarray(self._truth.copy())

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
        return {"main": {"size": tuple(size),
                        "format": _PARAMS.get("pixel_format", "XBGR8888")},
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
        # THE SCENE, in RGB — the truth about what the camera is looking at, stated without
        # reference to any buffer convention. A red patch and a blue patch, because a channel
        # order that exchanges red and blue is invisible in a neutral scene: it survives every
        # test that only ever shows gray, which is precisely how the real bug survived.
        truth = np.zeros((h, w, 3), dtype=np.uint8)
        if _PARAMS.get("neutral_only"):
            # Genuinely neutral: three equal channels. Anything with a warm bias would let a
            # colour-order check claim to have distinguished something it could not see.
            truth[...] = 150
        else:
            truth[..., 0] = 200  # R
            truth[..., 1] = 160  # G
            truth[..., 2] = 120  # B
        if not _PARAMS.get("neutral_only"):
            y0, y1 = h // 4, h // 2
            truth[y0:y1, w // 8: 3 * w // 8] = (220, 30, 30)     # red
            truth[y0:y1, 5 * w // 8: 7 * w // 8] = (30, 30, 220)  # blue

        # THE BUFFER, laid out as the declared format says it must be. The correspondence comes
        # from picamera2's own FORMAT_TABLE ({"XBGR8888": "RGBX", "XRGB8888": "BGRX",
        # "BGR888": "RGB", "RGB888": "BGR"}), i.e. for XBGR8888 the bytes in memory are R,G,B,X.
        # The previous fake wrote a 3-channel array and labelled channel 0 as blue, which was
        # both the wrong shape and the wrong belief; it could not fail a wrong decode.
        fmt = _PARAMS.get("pixel_format", "XBGR8888")
        if fmt == "XBGR8888":
            arr = np.dstack([truth, np.full(truth.shape[:2], 255, dtype=np.uint8)])
        elif fmt == "XRGB8888":
            arr = np.dstack([truth[..., ::-1], np.full(truth.shape[:2], 255, dtype=np.uint8)])
        elif fmt == "RGB888":
            arr = truth.copy()
        elif fmt == "BGR888":
            arr = truth[..., ::-1].copy()
        else:
            arr = truth.copy()  # an unknown format: shape is nobody's business, the code must refuse
        self.requested += 1
        return _FakeRequest(arr, self, truth_rgb=truth)

    def stop(self) -> None:
        self._running = False
        self.stopped = True

    def close(self) -> None:
        self.stop()


def make_module() -> types.ModuleType:
    mod = types.ModuleType("picamera2")
    mod.Picamera2 = FakePicamera2  # type: ignore[attr-defined]
    return mod


def install(frame_delay: float = 0.0, pixel_format: str = "XBGR8888",
            neutral_only: bool = False) -> object:
    """Install the fake in ``sys.modules["picamera2"]``; return the prior value
    (or ``_ABSENT``) so the caller can restore it in tearDown."""
    _PARAMS["frame_delay"] = frame_delay
    _PARAMS["pixel_format"] = pixel_format
    _PARAMS["neutral_only"] = neutral_only
    prev = sys.modules.get("picamera2", _ABSENT)
    sys.modules["picamera2"] = make_module()
    return prev


def restore(prev) -> None:
    if prev is None or prev is _ABSENT:
        sys.modules.pop("picamera2", None)
    else:
        sys.modules["picamera2"] = prev
