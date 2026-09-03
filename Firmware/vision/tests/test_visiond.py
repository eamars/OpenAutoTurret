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
from vision.protocol import TrackSet
from vision.track_manager import TrackManager, TrackState
from vision import visiond
from vision.visiond import VisionDaemon


class TestVisionDaemonEndToEnd(unittest.TestCase):
    def _run(self, num_frames: int = 30):
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(framerate_hz=30.0)
            tm = TrackManager()
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, tm, pub)
            published = daemon.run(num_frames=num_frames)
            time.sleep(0.05)  # let the subscriber thread pick up the last message
            latest = sub.latest
            sub.stop()
        return published, latest

    def test_publishes_track_sets(self):
        """§59: the pipeline publishes a TrackSet per frame, not one chosen target.

        "valid" was v1's single-target claim. There is nothing to mark valid here:
        the set is what it is, empty when the scene is empty, and controld decides
        which (if any) candidate to follow.
        """
        published, latest = self._run(num_frames=30)
        self.assertEqual(published, 30)
        self.assertIsNotNone(latest)
        self.assertIsInstance(latest, TrackSet)
        self.assertGreater(latest.width, 0)
        self.assertGreater(latest.height, 0)
        self.assertGreater(latest.sensor_timestamp_ns, 0)  # §6.2: still mandatory
        self.assertGreaterEqual(len(latest.tracks), 1)
        self.assertTrue(all(t.state == int(TrackState.CONFIRMED)
                            for t in latest.tracks),
                        "30 frames in, the synthetic target should be confirmed")

    def test_track_id_is_stable(self):
        # Run a sequence short enough that the synthetic target does NOT wrap
        # (the wrap is a real discontinuity that legitimately starts a new
        # track); the association must keep ONE stable track id.
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(velocity_px_per_frame=25.0)
            tm = TrackManager()
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, tm, pub)
            daemon.run(num_frames=30)  # 300 + 30*25 = 1050 px, well inside
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsNotNone(latest)
        self.assertIsInstance(latest, TrackSet)
        self.assertEqual(len(latest.tracks), 1)
        # §10: the identity is a 128-bit uuid and the stable one on the wire; the
        # display label is separate, and both must be handed out exactly once.
        self.assertEqual(latest.tracks[0].track_uuid, (0, 1))
        self.assertEqual(latest.tracks[0].display_index, 1)

    def test_frame_sequence_and_timestamp_advance(self):
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(framerate_hz=30.0)
            tm = TrackManager()
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, tm, pub)
            daemon.run(num_frames=10)
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsNotNone(latest)
        self.assertGreaterEqual(latest.frame_sequence, 9)
        self.assertGreater(latest.sensor_timestamp_ns, 0)

    def test_a_candidate_nobody_wants_is_still_published(self):
        """v1 published "invalid" here, because its *selector* refused to look at a
        class it had not been told to prefer. v3 separates the two questions:
        visiond publishes what it sees (§59), and controld decides what is worth
        following. So the observable outcome flips — the car is in the set — and the
        thing that must not regress is the refusal, which now lives in controld and is
        asserted there (ControlFeedTrackSet.IgnoresClassesAndConfidencesV1WouldRefuse).
        """
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            fs = SyntheticFrameSource(
                target_class_id=2,  # 'car', not the preferred 'person'
                confidence=0.2,     # below v1's threshold
                num_background_boxes=0,
            )
            tm = TrackManager()
            pub = IpcPublisher(sock)
            daemon = VisionDaemon(fs, tm, pub)
            daemon.run(num_frames=10)
            time.sleep(0.05)
            latest = sub.latest
            sub.stop()
        self.assertIsInstance(latest, TrackSet)
        self.assertEqual(len(latest.tracks), 1)
        self.assertEqual(latest.tracks[0].class_id, 2)
        self.assertAlmostEqual(latest.tracks[0].detector_confidence, 0.2, places=2)
        # Publishing it is correct. Selecting it is controld's call, and it says no.


class TestVisiondCliDetector(unittest.TestCase):
    """The --detector switch (§10.1 backend selection) must never break a run,
    and it must be honest about what is actually producing detections."""

    def _cli(self, extra):
        import contextlib
        import io

        err = io.StringIO()
        with tempfile.TemporaryDirectory() as d:
            sock = d + "/vision.sock"
            sub = IpcSubscriber(sock)
            sub.start()
            try:
                with contextlib.redirect_stderr(err):
                    rc = visiond.main(["--synthetic", "--frames", "4",
                                       "--socket", sock] + extra)
                time.sleep(0.05)
                latest = sub.latest
            finally:
                sub.stop()
        return rc, latest, err.getvalue()

    def test_detector_none_runs_and_publishes(self):
        rc, latest, _ = self._cli(["--detector", "none"])
        self.assertEqual(rc, 0)
        # v3: "published something valid" becomes "published a TrackSet". The set is
        # not stamped valid by the publisher any more, because validity was the
        # selector's opinion and the selector is gone (§59).
        self.assertIsInstance(latest, TrackSet)
        self.assertGreater(latest.frame_sequence, 0)

    def test_explicit_detector_is_reported_as_ignored_in_synthetic(self):
        # The synthetic source publishes SCRIPTED detections: a detector choice
        # there must be announced as ignored, not silently swallowed (an
        # operator who believes --detector simple is running would draw the
        # wrong conclusion from a live P8 run).
        for extra in (["--detector", "simple"], ["--detector", "rpk"]):
            rc, latest, err = self._cli(extra)
            self.assertEqual(rc, 0)
            self.assertIsInstance(latest, TrackSet)
            self.assertIn("--detector", err)

    def test_auto_in_synthetic_is_quiet(self):
        rc, _, err = self._cli([])          # default is auto
        self.assertEqual(rc, 0)
        self.assertNotIn("--detector", err)


if __name__ == "__main__":
    unittest.main()
