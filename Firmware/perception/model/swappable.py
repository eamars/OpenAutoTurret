"""One adapter object for the life of visiond, whose model can change underneath it (2026-10-20).

The operator's target type (perception/target_type.py) can need the other network: HUMAN and HEAD run
the pose HEF, CAT and DOG the COCO-80 one. Everything that holds the adapter -- the pipeline, the
inference switch, the detail worker, the health publisher -- keeps holding this proxy; ``swap`` closes
the network in use and opens the other one inside it.

Closed first, then opened: one Hailo-8 allows one context (HAILO_DEVICE_IN_USE). Measured on the station
2026-10-20 (Hailo-8, yolov8s-pose <-> yolov8n): close 35-39 ms, open 44-85 ms, first frame 10-16 ms.

The caller holds the inference lock (InferenceSwitch.lock: the pipeline calls ``swap`` from inside a
frame), so no inference is in flight. The stream geometry and the camera binding the old adapter was
given are replayed onto the new one, so its first frame is mapped exactly as the last one was.
"""
from __future__ import annotations

import dataclasses
from typing import Any, Callable, Optional


class SwappableAdapter:
    _OWN = ("_inner", "_build", "_profile", "_stream_call", "_serve_call", "_bind_call", "_open_kwargs",
            "swaps")

    def __init__(self, inner: Any, *, profile: str, build: Callable[[str], Any]) -> None:
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "_build", build)
        object.__setattr__(self, "_profile", profile)
        object.__setattr__(self, "_stream_call", None)
        object.__setattr__(self, "_serve_call", None)
        object.__setattr__(self, "_bind_call", None)
        object.__setattr__(self, "_open_kwargs", None)
        object.__setattr__(self, "swaps", 0)

    # -- delegation -----------------------------------------------------------------------------
    def __getattr__(self, name: str) -> Any:
        return getattr(self._inner, name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name in self._OWN:
            object.__setattr__(self, name, value)
        else:
            setattr(self._inner, name, value)

    @property
    def profile(self) -> str:
        return self._profile

    @property
    def inner(self) -> Any:
        return self._inner

    # The calls a swap has to repeat on the new adapter, recorded as they pass through.
    def configure_stream(self, *args: Any, **kwargs: Any) -> None:
        object.__setattr__(self, "_stream_call", (args, kwargs))
        return self._inner.configure_stream(*args, **kwargs)

    def serve_camera(self, *args: Any, **kwargs: Any) -> None:
        object.__setattr__(self, "_serve_call", (args, kwargs))
        return self._inner.serve_camera(*args, **kwargs)

    def bind_camera(self, *args: Any, **kwargs: Any) -> None:
        object.__setattr__(self, "_bind_call", (args, kwargs))
        return self._inner.bind_camera(*args, **kwargs)

    def open(self, **kwargs: Any) -> None:
        object.__setattr__(self, "_open_kwargs", dict(kwargs))
        return self._inner.open(**kwargs)

    # -- the swap -------------------------------------------------------------------------------
    def set_anchor_target(self, target: str) -> None:
        cfg = getattr(self._inner, "anchor_cfg", None)
        if cfg is not None and getattr(cfg, "target", None) != target:
            self._inner.anchor_cfg = dataclasses.replace(cfg, target=target)

    def swap(self, profile: str, *, anchor_target: Optional[str] = None) -> bool:
        """Run ``profile``'s model from the next frame on. True when the network changed.

        A profile already in use changes nothing on the device (HUMAN <-> HEAD is only the aim). If
        the new network will not open, the old one is reopened and the error raised: the caller keeps
        its applied type, and the controller, which waits for frames of the type it asked for, holds.
        """
        if profile == self._profile:
            if anchor_target:
                self.set_anchor_target(anchor_target)
            return False
        new = self._build(profile)
        old = self._inner
        generation = int(getattr(old, "generation", 0))
        old.close()
        try:
            self._replay(new)
            new.open(**(self._open_kwargs or {}))
        except Exception:
            try:
                new.close()
            finally:
                self._replay(old)
                old.open(**(self._open_kwargs or {}))
            raise
        if hasattr(new, "note_model_generation"):
            new.note_model_generation(generation + 1)
        object.__setattr__(self, "_inner", new)
        object.__setattr__(self, "_profile", profile)
        object.__setattr__(self, "swaps", self.swaps + 1)
        if anchor_target:
            self.set_anchor_target(anchor_target)
        return True

    def _replay(self, adapter: Any) -> None:
        for call, method in ((self._stream_call, "configure_stream"), (self._bind_call, "bind_camera"),
                             (self._serve_call, "serve_camera")):
            if call is not None:
                args, kwargs = call
                getattr(adapter, method)(*args, **kwargs)
