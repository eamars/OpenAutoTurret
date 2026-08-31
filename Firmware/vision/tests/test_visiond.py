"""End-to-end vision daemon test.

Runs the FULL vision pipeline (synthetic camera -> target selector -> IPC ->
subscriber) with NO camera hardware and NO motor driver. The IpcSubscriber is a
stand-in for controld that only RECEIVES measurements — it never opens CAN or
drives the motor. This is the safe way to verify the camera system.
"""
import tempfile
import time
import unittest

from vision.frame_source import SyntheticFrameSource
from vision.ipc import IpcPublisher, IpcSubscriber
from vision.target_selector import TargetSelector, TargetSelectorConfig
from vision.visiond import VisionDaemon


class TestVisionDaemonEndToEnd(unittest.TestCase):
    def _run(self, num_frames: int = 30):
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(framerate_hz=30.0)
            ts = TargetSelector(TargetSelectorConfig())
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, ts, pub)
            published = daemon.run(num_frames=num_frames)
            time.sleep(0.05)  # let the subscriber thread pick up the last message
            latest = sub.latest
            sub.stop()
        return published, latest

    def test_publishes_valid_measurements(self):
        published, latest = self._run(num_frames=30)
        self.assertEqual(published, 30)
        self.assertIsNotNone(latest)
        self.assertTrue(latest.valid)

    def test_track_id_is_stable(self):
        # Run a sequence short enough that the synthetic target does NOT wrap
        # (the wrap is a real discontinuity that legitimately starts a new
        # track); the association must keep ONE stable track id.
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(velocity_px_per_frame=25.0)
            ts = TargetSelector(TargetSelectorConfig())
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, ts, pub)
            daemon.run(num_frames=30)  # 300 + 30*25 = 1050 px, well inside
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.visual_track_id, 1)  # single stable track

    def test_frame_sequence_and_timestamp_advance(self):
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(framerate_hz=30.0)
            ts = TargetSelector(TargetSelectorConfig())
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, ts, pub)
            daemon.run(num_frames=10)
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsNotNone(latest)
        self.assertGreaterEqual(latest.frame_sequence, 9)
        self.assertGreater(latest.sensor_timestamp_ns, 0)

    def test_no_target_published_as_invalid(self):
        # A synthetic source with no target (zero-size background only) should
        # produce an invalid measurement when the preferred class is absent.
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(
                target_class_id=2,  # 'car', not the preferred 'person'
                confidence=0.2,     # below threshold
                num_background_boxes=0,
            )
            ts = TargetSelector(TargetSelectorConfig(preferred_class_id=1, fallback_to_best=False, confidence_threshold=0.5))
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, ts, pub)
            daemon.run(num_frames=5)
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsNotNone(latest)
        self.assertFalse(latest.valid)


if __name__ == "__main__":
    unittest.main()
