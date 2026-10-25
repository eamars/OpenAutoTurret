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
        previous = src.state().frames_published
        os.utime(self.path, None)
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and src.state().frames_published == previous:
            time.sleep(.01)
        self.assertGreater(src.state().frames_published, previous)
        self.assertFalse(src._open_error)
        src.stop()


class TapSize(unittest.TestCase):
    """A tapped stream is whatever visiond wrote, so its state says the served picture's size: it used to
    echo the start request's 1920x1080 for both previews (wide 640x360, detail 1280x720)."""

    @staticmethod
    def jpeg(width, height, comment=b""):
        import io
        from PIL import Image
        out = io.BytesIO()
        Image.new("RGB", (width, height), (40, 80, 120)).save(out, "JPEG", quality=70)
        data = out.getvalue()
        if comment:   # a COM segment ahead of the frame header, as visiond's OTA_FRAME snapshots carry
            data = data[:2] + b"\xff\xfe" + (len(comment) + 2).to_bytes(2, "big") + comment + data[2:]
        return data

    def test_the_frame_header_gives_the_size(self):
        from web.webd.video import jpeg_size
        self.assertEqual(jpeg_size(self.jpeg(640, 360)), (640, 360))
        self.assertEqual(jpeg_size(self.jpeg(1280, 720, comment=b"OTA_FRAME\x00{}")), (1280, 720))
        self.assertIsNone(jpeg_size(STALE_JPEG))
        self.assertIsNone(jpeg_size(b"not a jpeg at all"))

    def test_the_state_reports_the_served_size_not_the_request(self):
        path = os.path.join(tempfile.mkdtemp(), "tap.jpg")
        with open(path, "wb") as f:
            f.write(self.jpeg(640, 360))
        saved = os.environ.get("OTA_VISION_FRAME_TAP")
        os.environ["OTA_VISION_FRAME_TAP"] = path
        try:
            src = VideoSource(enabled=True)
            st = src.start(1920, 1080, 10, 70)
            self.assertTrue(st.running, st.error)
            self.assertEqual((st.width, st.height), (640, 360))
            with open(path + ".tmp", "wb") as f:
                f.write(self.jpeg(1280, 720))
            os.replace(path + ".tmp", path)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and src.state().width != 1280:
                time.sleep(0.02)
            self.assertEqual((src.state().width, src.state().height), (1280, 720))
            src.stop()
        finally:
            if saved is None:
                os.environ.pop("OTA_VISION_FRAME_TAP", None)
            else:
                os.environ["OTA_VISION_FRAME_TAP"] = saved


if __name__ == "__main__":
    unittest.main()
