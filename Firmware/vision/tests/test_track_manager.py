"""§82 — TrackManager unit tests, run where the TrackManager actually lives.

The eight required cases, named after §82's own list: single target; crossing
targets; temporary occlusion; detector dropout; two same-class targets; abrupt
bbox-size change; track creation/retirement; UUID stability.

These were first written in C++, against a C++ TrackManager, because §82 sits in a
chapter about controld. That was the wrong process: §9's TrackSet already carries
`state`, `age_frames`, `display_index` and the counters, so tracks are formed before
publication — and §58's "once per detector frame" is a camera-rate statement, which says
the same thing differently. Two implementations of §10's identity rules would disagree
first time two targets crossed, so there is one, here, and controld reads what it says.

No camera, no motor driver, no hardware (§82's closing line).
"""
import unittest

from vision.protocol import BBoxNorm, Track, TrackSet
from vision.track_manager import TrackManager, TrackManagerConfig, TrackState

FRAME_NS = 33_333_333  # 30 Hz camera


def det(class_id=1, name="person", ax=0.5, ay=0.5, w=0.10, h=0.20, conf=0.9):
    return Track(
        class_id=class_id,
        class_name=name,
        detector_confidence=conf,
        track_confidence=conf,
        bbox=BBoxNorm(ax - w / 2, ay - h / 2, ax + w / 2, ay + h / 2),
        anchor_x=ax,
        anchor_y=ay,
    )


class Harness:
    """Frame counter + clock, so the state ladder can be walked deterministically."""

    def __init__(self, manager=None):
        self.m = manager or TrackManager()
        self.seq = 0
        self.now = 0

    def feed(self, *dets):
        self.seq += 1
        self.now += FRAME_NS
        s = TrackSet(
            frame_sequence=self.seq,
            sensor_timestamp_ns=self.seq * FRAME_NS,
            publish_timestamp_ns=self.seq * FRAME_NS + 8_000_000,  # 8 ms later
            width=1280,
            height=720,
            tracks=list(dets),
        )
        return self.m.update(s, self.now)

    def feed_empty(self, n):
        for _ in range(n):
            self.feed()

    def only(self):
        ts = self.m.tracks()
        assert len(ts) == 1, f"expected exactly one track, got {len(ts)}"
        return ts[0]


class SingleTarget(unittest.TestCase):
    def test_becomes_selectable_after_consistent_frames(self):
        h = Harness()
        h.feed(det())
        uuid = h.only().track_uuid
        self.assertFalse(h.m.is_selectable(uuid),
                         "a first-frame detection is not something to point a "
                         "turret at (§8)")
        for _ in range(3):
            h.feed(det())
        self.assertTrue(h.m.is_selectable(uuid),
                        "consistent frames must confirm it (§8)")
        self.assertTrue(h.m.is_trackable(uuid))
        self.assertEqual(h.m.track_count(), 1, "one target must not become three")


class CrossingTargets(unittest.TestCase):
    def test_identities_survive_the_crossing(self):
        # The failure this pins: one frame of overlap makes the association swap, and
        # from then on the turret chases the wrong human being while the overlay labels
        # stay exactly as they were.
        h = Harness()
        a, b = det(ax=0.30), det(ax=0.40)
        for _ in range(4):
            a.anchor_x -= 0.005
            b.anchor_x += 0.005
            h.feed(a, b)
        tracks = h.m.tracks()
        self.assertEqual(len(tracks), 2)
        left = min(tracks, key=lambda t: t.anchor_x).track_uuid

        for _ in range(20):
            a.anchor_x -= 0.005
            b.anchor_x += 0.005
            h.feed(a, b)
        tracks = h.m.tracks()
        self.assertEqual(len(tracks), 2, "a crossing must not delete a track")
        now_left = min(tracks, key=lambda t: t.anchor_x).track_uuid
        self.assertEqual(now_left, left,
                         "identities swapped when the two crossed — the turret would "
                         "follow the wrong person while the screen said the same thing")


class TemporaryOcclusion(unittest.TestCase):
    def test_survives_and_stays_trackable_but_not_selectable(self):
        h = Harness()
        for _ in range(5):
            h.feed(det())
        uuid = h.only().track_uuid
        self.assertTrue(h.m.is_selectable(uuid))

        h.feed_empty(5)  # a pillar passes between camera and target
        self.assertIsNotNone(h.m.find(uuid), "an occluded track must still exist")
        self.assertEqual(h.m.find(uuid).state, TrackState.OCCLUDED)
        self.assertTrue(h.m.is_trackable(uuid),
                        "§20's coasting needs the selected target to still be a "
                        "subject")
        self.assertFalse(h.m.is_selectable(uuid),
                         "an operator must not be able to newly select something the "
                         "detector is not confirming right now")

        h.feed(det())
        self.assertEqual(h.m.find(uuid).state, TrackState.CONFIRMED)
        self.assertTrue(h.m.is_selectable(uuid))


class DetectorDropout(unittest.TestCase):
    def test_walks_the_ladder_then_retires(self):
        h = Harness(TrackManager(TrackManagerConfig(
            lost_frames=6, reacquire_frames=10)))
        for _ in range(5):
            h.feed(det())
        uuid = h.only().track_uuid

        h.feed_empty(6)
        self.assertIsNotNone(h.m.find(uuid))
        self.assertEqual(h.m.find(uuid).state, TrackState.LOST)
        self.assertFalse(h.m.is_trackable(uuid),
                         "a lost track is not something the controller keeps driving "
                         "toward")

        h.feed_empty(11)
        self.assertIsNone(h.m.find(uuid),
                          "§22: past the reacquisition window the track retires")
        self.assertGreater(h.m.stats.retired, 0)


class TwoSameClassTargets(unittest.TestCase):
    def test_get_different_labels(self):
        # §10: "Person #1 / Person #2", never raw UUIDs, and never one label shared by
        # two people.
        h = Harness()
        for _ in range(5):
            h.feed(det(ax=0.3), det(ax=0.7))
        a, b = sorted(h.m.tracks(), key=lambda t: t.display_index)
        self.assertNotEqual(a.track_uuid, b.track_uuid)
        self.assertNotEqual(a.display_index, b.display_index,
                            "two people cannot share one label on the selector")
        self.assertEqual((a.display_index, b.display_index), (1, 2))
        self.assertEqual(a.class_id, b.class_id)

    def test_label_is_reused_only_after_retirement(self):
        # §10 verbatim, and the reason selection carries the UUID: if a label could be
        # handed out while its previous owner still existed, the operator's "#1" would
        # silently mean a different human being.
        h = Harness(TrackManager(TrackManagerConfig(
            lost_frames=4, reacquire_frames=6)))
        for _ in range(5):
            h.feed(det(ax=0.3), det(ax=0.7))
        first = min(h.m.tracks(), key=lambda t: t.display_index)
        survivor = max(h.m.tracks(), key=lambda t: t.display_index)
        self.assertEqual(first.display_index, 1)

        for _ in range(12):
            h.feed(det(ax=0.7))  # only the survivor keeps being detected
        self.assertEqual(h.m.find(survivor.track_uuid).display_index, 2,
                         "the survivor keeps its label")
        self.assertIsNone(h.m.find(first.track_uuid), "the retired one is gone")

        for _ in range(5):
            h.feed(det(ax=0.7), det(ax=0.15, ay=0.8))
        newcomer = [t for t in h.m.tracks() if t.track_uuid != survivor.track_uuid]
        self.assertTrue(any(t.display_index == 1 for t in newcomer),
                        "a freed label goes to the next track, lowest first")


class AbruptBboxSizeChange(unittest.TestCase):
    def test_does_not_kill_the_track(self):
        # §82's quietest requirement, and the reason association is anchored on the
        # anchor point rather than on IoU: a detector that suddenly returns a fifth of
        # the box must not be read as "this object is gone and a new one appeared
        # here".
        h = Harness()
        for _ in range(5):
            h.feed(det(w=0.30, h=0.60))
        uuid = h.only().track_uuid
        self.assertTrue(h.m.is_selectable(uuid))

        for _ in range(3):
            h.feed(det(w=0.06, h=0.10))

        self.assertIsNotNone(h.m.find(uuid),
                             "the box shrank to a fifth and the identity was thrown "
                             "away")
        self.assertEqual(h.m.find(uuid).state, TrackState.CONFIRMED)
        self.assertEqual(h.m.track_count(), 1,
                         "and a second track was invented in its place")


class TrackCreationAndRetirement(unittest.TestCase):
    def test_counts_move_the_way_the_overlay_needs(self):
        h = Harness(TrackManager(TrackManagerConfig(
            confirm_frames=2, lost_frames=4, reacquire_frames=4)))
        self.assertEqual(h.m.stats.created, 0)
        h.feed(det())
        self.assertEqual(h.m.stats.created, 1)
        for _ in range(2):
            h.feed(det())
        self.assertEqual(h.m.find(h.only().track_uuid).state, TrackState.CONFIRMED)
        h.feed_empty(9)
        self.assertEqual(h.m.track_count(), 0, "retired tracks leave the table")
        self.assertEqual(h.m.stats.retired, 1)

    def test_capacity_never_sacrifices_a_confirmed_track(self):
        # At the cap something has to go. Dropping a confirmed track an operator may
        # have selected, to make room for a detection nobody asked for, would look like
        # the turret misbehaving with nothing in the log to explain it.
        cfg = TrackManagerConfig(max_tracks=4)
        h = Harness(TrackManager(cfg))
        dets = [det(ax=0.1 + 0.1 * i) for i in range(4)]
        for _ in range(5):
            h.feed(*dets)
        self.assertEqual(h.m.track_count(), 4)
        oldest = h.m.tracks()[0].track_uuid

        for _ in range(6):
            h.feed(*dets, det(ax=0.95, ay=0.95))
        self.assertLessEqual(h.m.track_count(), cfg.max_tracks,
                             "§60's cap is a contract, not a hint")
        self.assertIsNotNone(h.m.find(oldest),
                             "a confirmed track was dropped to make room for an "
                             "unproven detection")


class UuidStability(unittest.TestCase):
    def test_identity_survives_every_transition(self):
        # §10. The selection command carries the UUID, so an identity that changes
        # under the operator's finger either drops tracking or — worse, because it is
        # silent — retargets onto somebody else.
        h = Harness(TrackManager(TrackManagerConfig(
            lost_frames=6, reacquire_frames=20)))
        for _ in range(5):
            h.feed(det())
        t = h.only()
        uuid, label = t.track_uuid, t.display_index

        h.feed_empty(6)          # through OCCLUDED and toward LOST
        self.assertIsNotNone(h.m.find(uuid))
        h.feed(det(ax=0.52))     # back inside the reacquisition window
        slot = h.m.find(uuid)
        self.assertIsNotNone(
            slot, "reacquisition must reuse the identity, not mint a new one")
        self.assertEqual(slot.display_index, label)
        self.assertGreater(h.m.stats.reacquired, 0)


class LatencyStamps(unittest.TestCase):
    def test_unknown_is_not_zero_and_the_two_intervals_are_separate(self):
        # §61. "Never received" and "0 ms old" are different claims, and a dashboard
        # that shows the second when the first is true makes a dead detector look like
        # a healthy one with a slightly old frame.
        m = TrackManager()
        self.assertLess(m.stale_ms(1_000_000_000), 0.0)
        self.assertLess(m.sensor_age_ms(1_000_000_000), 0.0)
        self.assertLess(m.publish_to_receive_ms(), 0.0)

        capture = 300 * FRAME_NS
        now = capture + 12_000_000
        s = TrackSet(frame_sequence=300, sensor_timestamp_ns=capture,
                     publish_timestamp_ns=capture + 8_000_000,
                     width=1280, height=720, tracks=[det()])
        m.update(s, now)
        self.assertAlmostEqual(m.stale_ms(now), 0.0)
        self.assertAlmostEqual(m.sensor_age_ms(now), 12.0, places=3)
        self.assertAlmostEqual(
            m.publish_to_receive_ms(), 4.0, places=3,
            msg="the transport delay must be visible on its own, or a slow publisher "
                "and a slow camera are indistinguishable — and they need different "
                "fixes")


if __name__ == "__main__":
    unittest.main()
