"""Latest-only JPEG worker. Encoding and file I/O never run on the camera thread."""
from __future__ import annotations

import os
import tempfile
import threading
from pathlib import Path


class JpegPreviewWorker:
    def __init__(self, tap, path: str, *, quality: int = 72) -> None:
        self.tap = tap
        self.path = Path(path)
        self.quality = quality
        self.published = 0
        self.failures = 0
        self.last_error = ""
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        # Load the JPEG plugin before capture starts: lazy imports in the worker
        # contend for the GIL and backed up real camera frames during the first probe.
        import numpy as np
        self._encode(np.zeros((8, 8, 3), dtype=np.uint8))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, name="perception-preview", daemon=True)
        self._thread.start()

    def _encode(self, frame) -> bytes:
        import io
        import numpy as np
        from PIL import Image

        # Picamera2 RGB888 arrays contain B,G,R bytes on this little-endian platform.
        rgb = np.ascontiguousarray(frame[..., :3][..., ::-1])
        output = io.BytesIO()
        Image.fromarray(rgb).save(output, "JPEG", quality=self.quality)
        return output.getvalue()

    def _run(self) -> None:
        while not self._stop.is_set():
            frame = self.tap.take()
            if frame is None:
                self._stop.wait(.01)
                continue
            temporary = None
            try:
                jpeg = self._encode(frame)
                with tempfile.NamedTemporaryFile(dir=self.path.parent, suffix=".jpg.part",
                                                 delete=False) as output:
                    temporary = output.name
                    output.write(jpeg)
                os.replace(temporary, self.path)
                self.published += 1
                self.last_error = ""
            except Exception as exc:
                self.failures += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
            finally:
                if temporary and os.path.exists(temporary):
                    os.unlink(temporary)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def stats(self) -> dict:
        return {"published": self.published, "failures": self.failures,
                "last_error": self.last_error,
                "running": bool(self._thread and self._thread.is_alive())}
