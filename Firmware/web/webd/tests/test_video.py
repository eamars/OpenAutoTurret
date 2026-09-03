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


class ColourOrderTest(unittest.TestCase):
    """The picture has to be the colour it is.

    These tests exist because the preview at this station rendered yellow as blue and pink as
    lavender for a long time, and no test noticed. The reason no test noticed is worth stating:
    the fake camera used to hand out a 3-channel buffer annotated `channel 0 = blue`, which both
    contradicted the real 4-channel XBGR8888 buffer and restated the very assumption the decode
    was built on. A test whose fixture is written from the same belief as the code under test
    cannot falsify it. The fake now paints a scene it describes in RGB and encodes the buffer
    per the format it declares, so a wrong channel order produces a measurably wrong frame.
    """

    def _frame(self, src):
        deadline = time.time() + 3.0
        while time.time() < deadline:
            jpeg, _seq, _ts = src.latest()
            if jpeg:
                from PIL import Image
                import io
                return np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
            time.sleep(0.02)
        self.fail("no frame was published")

    def _start(self, **install_kw):
        fake_camera.restore(self._prev_cam) if hasattr(self, "_prev_cam") else None
        self._prev_cam = fake_camera.install(frame_delay=0.0, **install_kw)
        src = VideoSource(enabled=True)
        self.addCleanup(src.stop)
        self.addCleanup(fake_camera.restore, self._prev_cam)
        st = src.start(320, 240, fps=15, quality=92)
        self.assertTrue(st.running, msg=f"start failed: {st.error}")
        return src

    def setUp(self) -> None:
        self._prev_cam = None

    def test_a_red_patch_arrives_as_red(self):
        """The whole colour bug in one assertion: paint red into the sensor, ask what came out."""
        src = self._start()
        arr = self._frame(src)
        h, w = arr.shape[:2]
        red = arr[h // 4 + 4, w // 4].astype(int)          # inside the red patch
        blue = arr[h // 4 + 4, 3 * w // 4].astype(int)     # inside the blue patch
        self.assertGreater(red[0] - red[2], 60,
                           "the red patch came out with blue over red — channels are permuted: %r"
                           % (red.tolist(),))
        self.assertGreater(blue[2] - blue[0], 60,
                           "the blue patch came out with red over blue: %r" % (blue.tolist(),))

    def test_the_channel_order_matches_the_installed_camera_library(self):
        """Our table must agree with the library that actually writes these buffers.

        Read from the installed picamera2's own source rather than from a copy of it here,
        because a hand-copied table keeps certifying the world as it used to be — which is how
        the `[..., ::-1]` in this file's history stayed plausible: it was justified by the
        V4L2 fourcc, not by the encoder in use.
        """
        import pathlib
        import re

        candidates = []
        try:
            import picamera2
            candidates.append(pathlib.Path(picamera2.__file__).parent / "request.py")
        except Exception:  # noqa: BLE001 - the venv has no camera library; the station does
            # The tests may run in a venv while the camera stack lives in the system
            # interpreter. Reading the copy the *station* would use is the whole point of the
            # guard, so look there before declaring the check impossible.
            candidates.append(pathlib.Path("/usr/lib/python3/dist-packages/picamera2/request.py"))
        src = next((c for c in candidates if c.exists()), None)
        if src is None:
            self.skipTest("no picamera2 source found on this machine")
        text = src.read_text(encoding="utf-8")
        m = re.search(r"FORMAT_TABLE\s*=\s*\{([^}]*)\}", text)
        self.assertIsNotNone(m, "picamera2's FORMAT_TABLE was not found where the decode assumes it is")
        table = dict(re.findall(r'"(\w+)":\s*"(\w+)"', m.group(1)))
        self.assertTrue(table, "FORMAT_TABLE parsed empty")
        for fmt, colourspace in table.items():
            order = colourspace.replace("X", "")          # "RGBX" -> "RGB"
            self.assertIn(fmt, ic._RGB_INDEX_FOR_FORMAT,
                          f"{fmt} is encodable by picamera2 but unknown to our decode")
            letters = {"R": 0, "G": 1, "B": 2}
            expected = tuple(letters[c] for c in order)
            self.assertEqual(ic._RGB_INDEX_FOR_FORMAT[fmt], expected,
                             f"{fmt}: picamera2 encodes it as {colourspace}, we decode it as "
                             f"{expected} — one of us is swapping red and blue")

    def test_an_unrecognised_format_is_refused_rather_than_guessed(self):
        # VideoSource.start() gives up when no frame ever arrives, which is the right
        # outcome — but the reason has to be the decode refusal, not the timeout.
        fake_camera.restore(self._prev_cam) if hasattr(self, "_prev_cam") else None
        self._prev_cam = fake_camera.install(frame_delay=0.0, pixel_format="YUYV422")
        src = VideoSource(enabled=True)
        self.addCleanup(src.stop)
        self.addCleanup(fake_camera.restore, self._prev_cam)
        src.start(320, 240, fps=15, quality=92)
        deadline = time.time() + 2.0
        while time.time() < deadline and not src.state().error:
            time.sleep(0.02)
        st = src.state()
        self.assertIn("cannot decode camera frame", st.error,
                      "an unknown format must stop the frame AND say why, rather than the "
                      "generic 'no frames' symptom: %r" % (st.error,))
        self.assertIn("YUYV422", st.error, "the refusal must name the format it refused")
        self.assertEqual(src.latest()[0], b"",
                         "a frame was published despite the refusal — colours would be permuted")

    def test_the_decode_is_checked_against_the_libraries_own_encoder(self):
        """Once per start, our decode is compared with a frame picamera2 built itself."""
        src = self._start()
        self._frame(src)
        deadline = time.time() + 2.0
        while time.time() < deadline and not src.state().colour_check:
            time.sleep(0.02)
        check = src.state().colour_check
        self.assertTrue(check.startswith("ok"),
                        f"the colour-order check did not pass: {check!r}")
        self.assertIn("picamera2", check)

    def test_the_check_notices_a_swapped_decode(self):
        """Deliberately break the decode; the check has to say so, or it is decoration."""
        from unittest import mock

        from .. import video as video_module
        # The defect this file used to have, injected verbatim: take three channels and
        # reverse them. That is a pure red/blue exchange, which is what the real bug was.
        with mock.patch.object(video_module.ic, "rgb_from_stream",
                              side_effect=lambda arr, fmt: arr[..., :3][..., ::-1]):
            src = self._start()
            self._frame(src)
            deadline = time.time() + 2.0
            while time.time() < deadline and not src.state().colour_check:
                time.sleep(0.02)
            check = src.state().colour_check
            self.assertTrue(check.startswith("FAILED"),
                            f"a red/blue exchange slipped past the check: {check!r}")

    def test_the_check_still_works_when_the_lens_is_mounted_upside_down(self):
        """The station mounts the IMX500 inverted, and that is the case the check first failed.

        Comparing our (rotated) decode against the library's unrotated reference frame makes
        every channel order score the same distance, so the check reports "ok" having learned
        nothing — and it did exactly that on the real station until the comparison was moved
        ahead of the orientation correction. The tests only ever ran with orientation "none",
        which is why they missed it.
        """
        fake_camera.restore(self._prev_cam) if hasattr(self, "_prev_cam") else None
        self._prev_cam = fake_camera.install(frame_delay=0.0)
        self.addCleanup(fake_camera.restore, self._prev_cam)
        src = VideoSource(enabled=True, orientation="rotate_180")
        self.addCleanup(src.stop)
        st = src.start(320, 240, fps=15, quality=92)
        self.assertTrue(st.running, msg=f"start failed: {st.error}")
        self.assertEqual(st.pixel_format, "XBGR8888")
        self._frame(src)
        deadline = time.time() + 2.0
        while time.time() < deadline and not src.state().colour_check:
            time.sleep(0.02)
        check = src.state().colour_check
        self.assertTrue(check.startswith("ok"),
                        f"with the lens inverted the check stopped discriminating: {check!r}")
        # "ok" that quotes identical numbers for every order is the failure above; make it
        # explicit so a future regression cannot hide inside a passing test.
        import re
        nums = [float(x) for x in re.findall(r"([0-9]+\.[0-9])", check)]
        self.assertLess(nums[0], nums[1],
                        f"the decoded order is not decisively the best of the six: {check!r}")

        # And the published picture is still the right way round AND the right colour.
        arr = np.asarray(Image.open(io.BytesIO(src.latest()[0])).convert("RGB"))
        h, w = arr.shape[:2]
        red = arr[3 * h // 4 - 4, 3 * w // 4].astype(int)   # rotate_180 moves red to bottom-right
        self.assertGreater(red[0] - red[2], 60,
                           "the inverted mount lost the colour: %r" % (red.tolist(),))

    def test_a_scene_with_no_colour_says_it_saw_nothing(self):
        """A neutral scene cannot tell one channel order from another; claiming otherwise would
        make the check a ceremony that always passes."""
        src = self._start(neutral_only=True)
        self._frame(src)
        deadline = time.time() + 2.0
        while time.time() < deadline and not src.state().colour_check:
            time.sleep(0.02)
        check = src.state().colour_check
        self.assertTrue(check.startswith("inconclusive"),
                        f"a colourless frame was reported as a passed check: {check!r}")


import io  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from common import image_corrections as ic  # noqa: E402
