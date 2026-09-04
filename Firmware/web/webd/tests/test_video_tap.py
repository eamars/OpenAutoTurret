"""The frame tap shares one owner's sensor with the detector, so its failure mode is silent.

If visiond stops, the tap file stays behind, readable and complete. A consumer that only notices the file *vanishing*
serves the last frame forever and reports itself as running. This station did exactly that for eighty minutes: the pane
said `running True, error ''` over a JPEG nobody had written since before a controld restart. A pane that is stopped
and says so is worth more than a pane that is frozen and looks alive.
"""

import os
import tempfile
import time
import unittest

try:  # mirrors how the sibling tests import this module
    from web.webd.video import VideoSource
except ImportError:  # pragma: no cover
    from webd.video import VideoSource

STALE_JPEG = b"\xff\xd8\xff\xd9"  # two bytes are enough; nothing decodes it here


class TapStaleness(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "tap.jpg")
        with open(self.path, "wb") as f:
            f.write(STALE_JPEG)
        self._saved = os.environ.get("OTA_VISION_FRAME_TAP")
        os.environ["OTA_VISION_FRAME_TAP"] = self.path

    def tearDown(self):
        if self._saved is None:
            os.environ.pop("OTA_VISION_FRAME_TAP", None)
        else:
            os.environ["OTA_VISION_FRAME_TAP"] = self._saved

    def test_stale_tap_is_refused_and_says_why(self):
        old = time.time() - 120.0
        os.utime(self.path, (old, old))
        src = VideoSource(enabled=True)
        st = src.start(1280, 720, 12, 70)
        self.assertFalse(st.running)
        self.assertIn("stale", (st.error or "").lower())
        # It must not have served the stale frame first, just to be helpful.
        self.assertEqual(src.state().frames_published, 0)

    def test_fresh_tap_is_served_without_opening_the_camera(self):
        os.utime(self.path, None)  # mtime = now
        src = VideoSource(enabled=True)
        st = src.start(1280, 720, 12, 70)
        self.assertTrue(st.running, f"tap start failed: {st.error}")
        self.assertEqual(st.camera, "vision-tap")
        self.assertGreaterEqual(src.state().frames_published, 1)
        src.stop()

    def test_tap_dying_mid_stream_is_reported(self):
        os.utime(self.path, None)
        src = VideoSource(enabled=True)
        self.assertTrue(src.start(1280, 720, 12, 70).running, src.state().error)
        # Stop writing, let it go stale, and ask the source what it thinks.
        old = time.time() - 5.0
        os.utime(self.path, (old, old))
        src._tap_stale_s = 0.2
        deadline = time.time() + 3.0
        while time.time() < deadline and not src._open_error:
            time.sleep(0.05)
        self.assertIn("stale", (src._open_error or "").lower())


if __name__ == "__main__":
    unittest.main()
