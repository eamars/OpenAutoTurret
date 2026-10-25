"""On-demand full-resolution snapshots, for measurements such as a camera zero (ADR-006).

The wide camera's live preview is its inference leg, 640x360: 2.2 mrad a pixel, so a zero measured on
it is good to about 0.6 mrad. A snapshot is one frame of a camera's full picture -- wide 1920x1080,
detail 1280x720 -- taken on request. The next frame copies its full picture once (the wide capture
otherwise never copies it, camera.py), and the encode and the file happen on this slot's own thread,
never on the camera's. It is the same capture as the preview and the detector, so the same field of
view and the same normalised coordinates.

The file is written atomically, with the request id and the frame's identity in the JPEG itself (the
preview's OTA_FRAME comment), so a reader can tell its snapshot from an earlier one.
"""
from __future__ import annotations

import io
import json
import os
import tempfile
import threading
import time
import uuid
from pathlib import Path

MARKER = b"OTA_FRAME\x00"


def encode_jpeg(frame, *, quality: int, comment: dict | None = None) -> bytes:
    """Picamera2 RGB888 arrays hold B,G,R bytes on this platform (preview.py); the JPEG is RGB."""
    import numpy as np
    from PIL import Image

    rgb = np.ascontiguousarray(frame[..., :3][..., ::-1])
    output = io.BytesIO()
    Image.fromarray(rgb).save(output, "JPEG", quality=quality)
    jpeg = output.getvalue()
    if comment:
        payload = MARKER + json.dumps(comment, separators=(",", ":"), allow_nan=False).encode("utf-8")
        jpeg = jpeg[:2] + b"\xff\xfe" + (len(payload) + 2).to_bytes(2, "big") + payload + jpeg[2:]
    return jpeg


def read_comment(jpeg: bytes) -> dict | None:
    """The OTA_FRAME comment right after the JPEG's start, or None."""
    if len(jpeg) < 6 or jpeg[:2] != b"\xff\xd8" or jpeg[2:4] != b"\xff\xfe":
        return None
    length = int.from_bytes(jpeg[4:6], "big")
    payload = jpeg[6:4 + length]
    if not payload.startswith(MARKER):
        return None
    try:
        value = json.loads(payload[len(MARKER):].decode("utf-8"))
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


class SnapshotSlot:
    """One camera's snapshot: request() arms it, the camera thread offers the next full frame, the
    slot's thread encodes it to `path`."""

    def __init__(self, role: str, path: str, *, quality: int = 95) -> None:
        self.role = role
        self.path = Path(path)
        self.quality = int(quality)
        self.taken = 0
        self.failures = 0
        self.last_error = ""
        self._lock = threading.Lock()
        self._wanted: str | None = None
        self._frame = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def request(self) -> str:
        """Arm the slot; the id names the snapshot the next frame becomes. A second request before
        that frame replaces the first: both callers then wait for the same frame's file under the
        newer id, and the older one times out."""
        request_id = uuid.uuid4().hex[:16]
        with self._lock:
            self._wanted = request_id
        return request_id

    def wanted(self) -> bool:
        """The camera thread's per-frame question: one attribute read, no lock."""
        return self._wanted is not None

    def offer(self, image, metadata: dict | None = None) -> bool:
        """Hand over a full frame if one is wanted; the array is kept, not copied (make_array
        already copied it out of the camera buffer)."""
        with self._lock:
            request_id, self._wanted = self._wanted, None
            if request_id is None:
                return False
            self._frame = (request_id, image, dict(metadata or {}))
        self._ready.set()
        return True

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, name=f"snapshot-{self.role}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._ready.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            from common.thread_class import lower_this_thread
            # Measurement work, not motion: it yields to the inference and tracking thread.
            lower_this_thread()
        except ImportError:
            pass
        while not self._stop.is_set():
            if not self._ready.wait(0.2):
                continue
            self._ready.clear()
            with self._lock:
                frame, self._frame = self._frame, None
            if frame is not None:
                self.write(*frame)

    def write(self, request_id: str, image, metadata: dict) -> None:
        temporary = None
        try:
            height, width = int(image.shape[0]), int(image.shape[1])
            comment = dict(metadata, request=request_id, camera=self.role, width=width, height=height,
                           taken_ns=time.time_ns())
            jpeg = encode_jpeg(image, quality=self.quality, comment=comment)
            with tempfile.NamedTemporaryFile(dir=self.path.parent, suffix=".jpg.part", delete=False) as output:
                temporary = output.name
                output.write(jpeg)
            os.replace(temporary, self.path)
            self.taken += 1
            self.last_error = ""
        except Exception as exc:                                          # noqa: BLE001
            self.failures += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            if temporary:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass
