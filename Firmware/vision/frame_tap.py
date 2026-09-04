"""Frame tap: let the HUD preview share the IMX500 with the real vision pipeline.

WHY THIS EXISTS
The IMX500 has exactly one owner. Until now that meant an operator had to choose: run the real detector and lose the
video pane, or run the pane and starve vision (which failed with the genuinely unhelpful
`RuntimeError: Camera __init__ sequence did not complete`). The operator's requirement is that both work at once — you
cannot watch a tracker you cannot see.

So the frame that vision ALREADY captured is published a second time, as a JPEG, to a fixed path. One sensor owner,
two consumers. This is deliberately a tap and not a second stream: the same pixels the detector saw are the pixels the
operator sees, which is the only version of "preview" worth having when you are trying to judge whether a box is on
target.

The interface is an env var holding the path (`OTA_VISION_FRAME_TAP`), because the two processes that need to agree on
it (visiond and webd) already read their configuration from the environment, and a tap that has to be threaded through
three constructors is a tap that gets dropped on the floor.

Cost, stated rather than assumed: one PIL encode per tapped frame on the vision thread (~10-20 ms at 1080p, measured
for the pane in PROGRESS round 7). It is rate-capped below, and the cap is the point — the tap exists to be watched,
not to be recorded.
"""

from __future__ import annotations

import os
import time

# The tap is a preview, not evidence. 12 fps is above what a browser can show on this link and below the sensor rate,
# so the vision thread spends a third of its frame budget on it at most.
DEFAULT_MAX_FPS = 12.0
DEFAULT_QUALITY = 72


class FrameTap:
    """Atomically publish the newest JPEG of a frame array to one path.

    Atomic means write-to-temp-then-rename: a reader must never see half a JPEG, because a half JPEG decodes to a
    grey rectangle and the operator reads that as the camera having failed.
    """

    def __init__(self, path: str, max_fps: float = DEFAULT_MAX_FPS, quality: int = DEFAULT_QUALITY) -> None:
        self.path = str(path)
        self._min_interval = 1.0 / max(1.0, float(max_fps))
        self._quality = int(quality)
        self._last = 0.0
        self.published = 0
        self.last_error = ""

    def publish(self, arr) -> bool:
        """Publish `arr` if one is due. Returns True when the file was replaced."""
        now = time.monotonic()
        if now - self._last < self._min_interval:
            return False
        try:
            import io

            import numpy as np
            from PIL import Image

            a = arr
            # The real stream is negotiated as XRGB8888, which numpy hands over with the channels in BGRX order;
            # taking [2,1,0] is the same conversion the pane does, so both show the same colours.
            if a.ndim == 3 and a.shape[2] == 4:
                a = a[..., [2, 1, 0]]
            buf = io.BytesIO()
            Image.fromarray(np.ascontiguousarray(a)).save(buf, "JPEG", quality=self._quality)
            tmp = self.path + ".part"
            with open(tmp, "wb") as f:
                f.write(buf.getvalue())
            os.replace(tmp, self.path)  # atomic on POSIX: readers see old or new, never half
            self._last = now
            self.published += 1
            self.last_error = ""
            return True
        except Exception as e:  # noqa: BLE001 - a broken tap must never break detection
            # Stated, not swallowed: the tap failing is fine, the tap failing *silently* is the bug that wastes an
            # evening. One line per error, and only on the vision thread.
            msg = f"{type(e).__name__}: {e}"
            if msg != self.last_error:
                print(f"vision: frame tap could not publish to {self.path}: {msg}")
            self.last_error = msg
            return False


def from_env(env=None) -> "FrameTap | None":
    """Build a tap from OTA_VISION_FRAME_TAP, or None when it is unset.

    OTA_VISION_FRAME_TAP_FPS tunes the cost. Measured on this station, tapping at 12 fps took the detector from 15.13
    to 9.55 fps, because the JPEG encode happens on the vision thread — the same thread that has to hand frames to the
    detector. That trade is worth making when the operator is watching, and worth un-making when they are not, so it is
    a number rather than a decision I made for them. Lower it (6-8) if detection rate matters more than preview
    smoothness; the preview stays usable because a browser on this link cannot show 15 fps anyway.
    """
    e = os.environ if env is None else env
    path = (e.get("OTA_VISION_FRAME_TAP") or "").strip()
    if not path:
        return None
    fps = DEFAULT_MAX_FPS
    try:
        fps = float((e.get("OTA_VISION_FRAME_TAP_FPS") or "").strip() or DEFAULT_MAX_FPS)
    except ValueError:
        print("vision: OTA_VISION_FRAME_TAP_FPS is not a number; using the default")
    return FrameTap(path, max_fps=fps)


_TAP: "FrameTap | None" = None
_CHECKED = False


def publish_env(arr) -> bool:
    """Publish `arr` to the tap named by OTA_VISION_FRAME_TAP, if there is one.

    Checked once per process, because the capture loop runs 15 times a second and reading an environment variable to
    discover for the fifteenth time that nobody wants a tap is not free enough to be careless about.
    """
    global _TAP, _CHECKED
    if not _CHECKED:
        _TAP, _CHECKED = from_env(), True
    return _TAP.publish(arr) if _TAP is not None else False
