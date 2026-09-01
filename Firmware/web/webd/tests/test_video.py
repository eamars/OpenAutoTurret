"""VideoSource unit tests against the fake picamera2 (no real camera).

Covers the on/off lifecycle, the low-priority FPS cap (§42.3), the shared latest
frame slot, the multipart MJPEG framing, and the disabled/defensive paths.
"""
from __future__ import annotations

import time
import unittest

from ..video import VideoSource, mjpeg_frame
from . import fake_camera


class VideoSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        self._prev_cam = fake_camera.install(frame_delay=0.0)
        self.src = VideoSource(enabled=True)

    def tearDown(self) -> None:
        self.src.stop()
        fake_camera.restore(self._prev_cam)

    def test_off_by_default(self) -> None:
        st = self.src.state()
        self.assertFalse(st.running)
        self.assertEqual(self.src.latest()[0], b"")

    def test_start_stop_lifecycle(self) -> None:
        st = self.src.start(320, 240, fps=15, quality=80)
        self.assertTrue(st.running, msg=f"start failed: {st.error}")
        self.assertEqual(st.camera, "fake-imx500")
        self.assertEqual((st.width, st.height), (320, 240))

        # A real JPEG frame flows through the shared slot.
        deadline = time.time() + 3.0
        jpeg = b""
        while time.time() < deadline:
            jpeg, seq, _ = self.src.latest()
            if jpeg:
                break
            time.sleep(0.02)
        self.assertTrue(jpeg, "no frame produced")
        self.assertEqual(jpeg[:2], b"\xff\xd8", "frame is not a JPEG")
        self.assertGreater(seq, 0)

        st = self.src.stop()
        self.assertFalse(st.running)
        self.assertEqual(self.src.latest()[0], b"", "slot not cleared on stop")

    def test_start_is_idempotent(self) -> None:
        self.src.start(320, 240, fps=15, quality=80)
        st = self.src.start(640, 480, fps=5, quality=50)  # 2nd: no-op
        self.assertTrue(st.running)
        self.assertEqual(st.width, 320)  # original config kept

    def test_fps_cap_limits_publish_rate(self) -> None:
        # At a 10 fps cap the tight-loop producer must NOT publish per callback;
        # over a fixed window the published count stays near the cap (not in the
        # hundreds an uncapped loop would produce).
        self.src.start(64, 48, fps=10, quality=70)
        time.sleep(0.5)
        _jpeg, seq, _ = self.src.latest()
        self.assertGreaterEqual(seq, 1, "no frames at all")
        self.assertLess(seq, 25, f"cap not effective: {seq} frames in 0.5s")

    def test_latest_returns_monotonic_seq(self) -> None:
        self.src.start(64, 48, fps=30, quality=70)
        time.sleep(0.3)
        _j, s1, _ = self.src.latest()
        time.sleep(0.1)
        _j, s2, _ = self.src.latest()
        self.assertGreaterEqual(s2, s1)

    def test_disabled_source_never_opens(self) -> None:
        src = VideoSource(enabled=False)
        st = src.start(320, 240, fps=15, quality=80)
        self.assertFalse(st.running)
        self.assertIn("disabled", st.error)
        self.assertEqual(src.latest()[0], b"")


class MjpegFrameTest(unittest.TestCase):
    def test_framing(self) -> None:
        payload = b"\xff\xd8\xff\xe0hello\xff\xd9"
        out = mjpeg_frame(payload)
        self.assertTrue(out.startswith(b"--frame\r\n"))
        self.assertIn(b"Content-Type: image/jpeg\r\n", out)
        self.assertIn(b"Content-Length: " + str(len(payload)).encode(), out)
        self.assertIn(payload, out)
        self.assertTrue(out.endswith(b"\r\n"))

    def test_state_to_dict(self) -> None:
        st = VideoSource().state().to_dict()
        self.assertIn("running", st)
        self.assertIn("frames_published", st)
        self.assertIn("error", st)


if __name__ == "__main__":
    unittest.main()
