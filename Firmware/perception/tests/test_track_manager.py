"""§52 — the identity lifecycle: confirmation, occlusion, loss, reacquisition, capacity.

The framing of every test here is "what would an operator notice". A state machine that is
merely internally consistent is not the requirement; the requirement is that a person walking
behind a pillar for 300 ms is still the same UUID when they emerge, that a person who has
been gone for four seconds is not, and that when the tracker is wrong the published counters
say so.

Every case feeds **explicit sensor timestamps**, because §19's whole argument is that the
timeouts mean milliseconds: a test that let the tracker fall back to frame counting would
still pass while the station's actual behaviour drifted with the inference rate.
"""
from __future__ import annotations

import unittest

from perception.events import EventLog, EventType
from perception.tests.support import (advance, commissioned_config, det, dset, ms, at)
from perception.tracking.track import Track, TrackState
from perception.tracking.track_manager import TrackManager


class TestConfirmation(unittest.TestCase):
    def setUp(self):
        self.config = commissioned_config()
        self.manager = TrackManager(self.config)

    def test_first_frame_creates_a_tentative_identity_not_a_confirmed_one(self):
        result = self.manager.update(dset([det(1, cx=0.5)]), at(0))
        self.assertEqual(len(result.tracks), 1)
        track = result.tracks[0]
        self.assertIs(track.state, TrackState.TENTATIVE)
        self.assertEqual(self.manager.counters.tracks_confirmed, 0)

    def test_identity_confirms_on_the_third_observation_past_120ms(self):
        sets = advance(self.manager, [[det(1)], [det(1)], [det(1)]])
        self.assertIs(sets[0].tracks[0].state, TrackState.TENTATIVE)
        self.assertIs(sets[1].tracks[0].state, TrackState.TENTATIVE,
                      "two observations at 120 ms visible must not confirm (§19)")
        self.assertIs(sets[2].tracks[0].state, TrackState.CONFIRMED_VISIBLE)
        self.assertEqual(self.manager.counters.tracks_confirmed, 1)
        self.assertGreaterEqual(sets[2].tracks[0].visible_ms(at(2)), 120.0)

    def test_a_tentative_identity_that_misses_the_gap_budget_retires(self):
        # §18.1: a candidate whose gap exceeds max_gap_ms is not believable, so it retires
        # rather than lingering until some future frame deigns to confirm it.
        self.manager.update(dset([det(1)]), at(0))
        result = self.manager.update(dset([], sensor_ns=at(0) + ms(0.3)),
                                     at(0) + ms(0.3))
        self.assertEqual(result.tracks, [])
        self.assertEqual(self.manager.counters.tracks_retired, 1)
        self.assertEqual(self.manager.counters.tracks_confirmed, 0)

    def test_confirmed_needs_contiguous_observations_not_merged_history(self):
        # Two observations, a 300 ms gap, then one more. The visible time totals over 120 ms
        # but the *contiguous* run does not, and §19's max_gap_ms is what encodes that.
        manager = TrackManager(self.config)
        advance(manager, [[det(1)], [det(1)]], start_frame=0)
        later = manager.update(dset([det(1)], frame_index=6), at(6))   # +360 ms
        self.assertTrue(later.tracks, "the identity should have been reacquirable")
        self.assertIsNot(later.tracks[0].state, TrackState.CONFIRMED_VISIBLE)


class TestOcclusionAndLoss(unittest.TestCase):
    def setUp(self):
        self.config = commissioned_config()
        self.manager = TrackManager(self.config)
        # Three frames at 60 ms: confirmed by frame 2 at 120 ms visible.
        advance(self.manager, [[det(1)], [det(1)], [det(1)]])
        self.confirmed_uuid = self.manager.tracks()[0].track_uuid

    def _feed_at(self, offset_ms: float, detections):
        sensor_ns = at(2) + int(round(offset_ms * 1e6))
        return self.manager.update(dset(detections, sensor_ns=sensor_ns),
                                   2_000_000_000 + int(round(offset_ms * 1e6)))

    def test_short_gap_goes_occluded_and_keeps_the_uuid(self):
        result = self._feed_at(200.0, [])
        self.assertEqual(len(result.tracks), 1)
        track = result.tracks[0]
        self.assertIs(track.state, TrackState.OCCLUDED)
        self.assertEqual(track.track_uuid, self.confirmed_uuid)
        self.assertFalse(track.measurement_valid, "no measurement, §34")

    def test_emerges_from_occlusion_as_the_same_identity(self):
        self._feed_at(200.0, [])
        result = self._feed_at(260.0, [det(1)])
        self.assertEqual([t.track_uuid for t in result.tracks], [self.confirmed_uuid])
        self.assertIs(result.tracks[0].state, TrackState.CONFIRMED_VISIBLE)

    def test_gap_past_the_occluded_window_makes_it_lost_reacquirable(self):
        result = self._feed_at(400.0, [])
        self.assertIs(result.tracks[0].state, TrackState.LOST_REACQUIRABLE)
        self.assertEqual(self.manager.counters.tracks_lost, 1)

    def test_reacquisition_reports_just_reacquired_and_counts(self):
        self._feed_at(400.0, [])
        result = self._feed_at(500.0, [det(1)])
        track = result.tracks[0]
        self.assertEqual(track.track_uuid, self.confirmed_uuid)
        self.assertTrue(track.just_reacquired, "§34: the controller must see this is a "
                                              "reacquisition, not a fresh measurement")
        self.assertGreaterEqual(self.manager.counters.tracks_reacquired, 1)

    def test_identity_retires_once_past_the_retain_window(self):
        events = EventLog()
        manager = TrackManager(commissioned_config(), event_log=events)
        advance(manager, [[det(1)], [det(1)], [det(1)]])
        uuid = manager.tracks()[0].track_uuid
        result = manager.update(dset([], sensor_ns=at(2) + ms(3.1)), 3_000_000_000)
        self.assertEqual(result.tracks, [])
        self.assertFalse(manager.exists(uuid))
        self.assertEqual(manager.counters.tracks_retired, 1)
        self.assertEqual(events.count(EventType.TRACK_RETIRED), 1)

    def test_selection_cannot_survive_its_identity_being_retired(self):
        manager = TrackManager(commissioned_config())
        advance(manager, [[det(1)], [det(1)], [det(1)]])
        uuid = manager.tracks()[0].track_uuid
        manager.set_selected_uuid(uuid)
        manager.update(dset([], sensor_ns=at(2) + ms(3.1)), 3_000_000_000)
        self.assertIsNone(manager.find(uuid))


class TestLowScoreRescue(unittest.TestCase):
    """§20's asymmetry: a low-score detection may rescue, never create, never measure."""

    def setUp(self):
        self.manager = TrackManager(commissioned_config())

    def test_low_score_alone_never_creates_an_identity(self):
        result = self.manager.update(dset([det(1, score=0.20)]), at(0))
        self.assertEqual(result.tracks, [], "a 0.20 detection must not mint a person")
        self.assertEqual(self.manager.counters.tracks_created, 0)

    def test_low_score_rescue_keeps_the_identity_but_is_not_a_measurement(self):
        advance(self.manager, [[det(1)], [det(1)], [det(1)]])
        track = self.manager.tracks()[0]
        self.assertIs(track.state, TrackState.CONFIRMED_VISIBLE)
        before = track.last_measurement_ns

        result = self.manager.update(dset([det(1, score=0.20)], frame_index=3,
                                          sensor_ns=at(3)), 2_000_000_000)
        rescued = result.tracks[0]
        self.assertEqual(rescued.track_uuid, track.track_uuid)
        self.assertIs(rescued.state, TrackState.OCCLUDED)
        self.assertFalse(rescued.measurement_valid)
        self.assertEqual(rescued.last_measurement_ns, before,
                         "a rescue must not refresh §19's measurement age")
        self.assertEqual(self.manager.counters.low_score_associations, 1)

    def test_rescued_only_identity_still_expires(self):
        # Bounded by design: "the detector half-saw something" cannot keep a person alive
        # forever, because the controller would then hold an aimable-looking identity with
        # no confident measurement behind it.
        advance(self.manager, [[det(1)], [det(1)], [det(1)]])
        for frame in range(3, 60):
            self.manager.update(dset([det(1, score=0.20)], frame_index=frame), at(frame))
        self.assertEqual(self.manager.tracks(), [])


class TestCapacity(unittest.TestCase):
    """§26: the ceiling is explicit, the eviction order is defined, and both are counted."""

    def test_overflow_evicts_the_least_confident_tentative_identity(self):
        manager = TrackManager(commissioned_config(max_tracks=2))
        result = manager.update(dset([det(1, cx=0.1, score=0.9),
                                      det(2, cx=0.5, score=0.8),
                                      det(3, cx=0.9, score=0.35)]), at(0))
        self.assertLessEqual(len(result.tracks), 2)
        self.assertEqual(manager.counters.tracks_evicted, 1)
        self.assertEqual(manager.counters.track_capacity, 2)
        self.assertEqual(result.counters.track_capacity_used, len(result.tracks))

    def test_the_selected_identity_is_never_evicted_while_its_ttl_is_active(self):
        manager = TrackManager(commissioned_config(max_tracks=2))
        first = manager.update(dset([det(1, cx=0.1, score=0.9)]), at(0))
        chosen = first.tracks[0].track_uuid
        manager.set_selected_uuid(chosen)
        # Second frame: two more people arrive; pressure must land on someone else.
        result = manager.update(dset([det(1, cx=0.1, score=0.9),
                                      det(2, cx=0.5, score=0.8),
                                      det(3, cx=0.9, score=0.95)], sensor_ns=at(1)),
                                at(1))
        self.assertIn(chosen, [t.track_uuid for t in result.tracks],
                      "§26: capacity must never steal the operator's target")

    def test_detections_refused_slots_are_counted_not_dropped_silently(self):
        config = commissioned_config(max_tracks=1)
        # protect_selected + a selected identity leaves nothing evictable, so the extra
        # people must be *reported* as dropped, which is §26's whole point.
        manager = TrackManager(config)
        first = manager.update(dset([det(1, cx=0.1, score=0.9)]), at(0))
        manager.set_selected_uuid(first.tracks[0].track_uuid)
        manager.update(dset([det(1, cx=0.1, score=0.9),
                             det(2, cx=0.5, score=0.95),
                             det(3, cx=0.9, score=0.98)], sensor_ns=at(1)), at(1))
        self.assertGreaterEqual(manager.counters.detections_dropped_capacity, 1)


class TestIdentityConfidence(unittest.TestCase):
    """§37: four quality fields that must not collapse into one number."""

    def test_a_high_detector_score_does_not_borrow_an_identitys_confidence(self):
        manager = TrackManager(commissioned_config())
        manager.update(dset([det(1, score=0.99)]), at(0))
        track = manager.tracks()[0]
        self.assertGreater(track.detector_score, 0.95)
        self.assertLess(track.identity_confidence, 0.5,
                        "one frame old is not a certain identity, however confident the "
                        "detector claims to be")

    def test_a_long_measured_identity_becomes_selectable(self):
        manager = TrackManager(commissioned_config())
        sets = advance(manager, [[det(1, score=0.9)] for _ in range(20)])
        track = sets[-1].tracks[0]
        self.assertTrue(track.selectable)
        self.assertGreater(track.identity_confidence, 0.5)
        self.assertTrue(manager.is_selectable(track.track_uuid)[0])

    def test_an_unconfirmed_identity_is_not_selectable(self):
        manager = TrackManager(commissioned_config())
        sets = advance(manager, [[det(1)], [det(1)]])
        track = sets[-1].tracks[0]
        self.assertFalse(track.selectable)
        allowed, reason = manager.is_selectable(track.track_uuid)
        self.assertFalse(allowed)
        self.assertIn("tentative", reason.lower())


class TestDisplayLabels(unittest.TestCase):
    """§27: labels are monotonic per class within a session and never reused."""

    def test_labels_increase_even_after_the_lower_one_retired(self):
        manager = TrackManager(commissioned_config())
        first = manager.update(dset([det(1, cx=0.1)]), at(0))
        second = manager.update(dset([det(1, cx=0.1), det(2, cx=0.9)],
                                     sensor_ns=at(1)), at(1))
        self.assertEqual(sorted(t.display_index for t in second.tracks), [1, 2])

        # Both leave the frame and expire; a newcomer must not inherit either label.
        manager.update(dset([], sensor_ns=at(2) + ms(3.2)), at(2) + ms(3.2))
        self.assertEqual(manager.tracks(), [])
        result = manager.update(dset([det(7, cx=0.5)], sensor_ns=at(3) + ms(3.2)),
                                at(3) + ms(3.2))
        labels = sorted(t.display_index for t in result.tracks)
        self.assertEqual(labels, [3],
                         "the retired path reused the lowest free label; §27 forbids it")

    def test_labels_are_per_class(self):
        manager = TrackManager(commissioned_config())
        result = manager.update(dset([det(1, cx=0.1, class_id=0, class_name="person"),
                                      det(2, cx=0.9, class_id=1, class_name="bicycle")]),
                                at(0))
        self.assertEqual(sorted(t.display_index for t in result.tracks), [1, 1])


class TestPublishedSet(unittest.TestCase):
    def test_track_set_validates_and_carries_stamps(self):
        manager = TrackManager(commissioned_config())
        result = advance(manager, [[det(1)]])[-1]
        result.validate()
        self.assertEqual(result.model_id, "unit-test-model")
        self.assertEqual(result.stream_width, 1920)
        self.assertEqual(result.session_uuid, manager.session_uuid)
        self.assertGreater(result.track_set_sequence, 0)

    def test_counter_snapshot_does_not_move_after_publication(self):
        # A published TrackSet is a record. If it shared the manager's live counters, the
        # recorder would write numbers that had already advanced past the frame it describes.
        manager = TrackManager(commissioned_config())
        published = manager.update(dset([det(1)]), at(0))
        before = published.counters.tracks_created
        manager.update(dset([det(1), det(2, cx=0.9)], sensor_ns=at(1)), at(1))
        self.assertEqual(published.counters.tracks_created, before)

    def test_out_of_order_timestamps_are_counted_not_silently_absorbed(self):
        manager = TrackManager(commissioned_config())
        advance(manager, [[det(1)], [det(1)]])
        manager.update(dset([det(1)], sensor_ns=at(0)), 3_000_000_000)   # time goes back
        self.assertEqual(manager.frames_out_of_order, 1)
        self.assertTrue(manager.tracks(), "a stale frame must not destroy the identity")

    def test_events_record_the_lifecycle_that_happened(self):
        events = EventLog()
        manager = TrackManager(commissioned_config(), event_log=events)
        advance(manager, [[det(1)], [det(1)], [det(1)]])
        # +250 ms of nothing: inside the 350 ms occluded window, so OCCLUDED and not yet
        # LOST — the two transitions are different events and must not be conflated.
        manager.update(dset([], sensor_ns=at(2) + ms(0.25)), at(3))
        manager.update(dset([det(1)], sensor_ns=at(2) + ms(0.32)), at(4))
        self.assertEqual(events.count(EventType.TRACK_CREATED), 1)
        self.assertEqual(events.count(EventType.TRACK_CONFIRMED), 1)
        self.assertEqual(events.count(EventType.TRACK_OCCLUDED), 1)
        # Coming back from a 250 ms occlusion is an ordinary update, not a reacquisition:
        # §34's flag exists so the controller can tell a first measurement after a *loss*
        # apart from a normal frame, and borrowing it here would train the controller to
        # distrust every measurement.
        self.assertEqual(events.count(EventType.TRACK_REACQUIRED), 0)

        # Now a real loss: past the occluded window, then a measurement.
        # Measured at +0.32 s, so the miss clock has to pass 350 ms from *there*.
        manager.update(dset([], sensor_ns=at(2) + ms(0.80)), at(5))
        manager.update(dset([det(1)], sensor_ns=at(2) + ms(0.87)), at(6))
        self.assertEqual(events.count(EventType.TRACK_LOST), 1)
        self.assertEqual(events.count(EventType.TRACK_REACQUIRED), 1)
        self.assertEqual([e.track_uuid for e in events.recent()
                          if e.event_type is EventType.TRACK_CREATED],
                         [manager.tracks()[0].track_uuid])


class TestAmbiguity(unittest.TestCase):
    """§32: an unresolvable reacquisition is published as ambiguity, not guessed."""

    def test_two_lost_identities_competing_for_one_detection_are_both_ambiguous(self):
        manager = TrackManager(commissioned_config())
        sensor = at(5)
        stale = sensor - ms(0.5)
        for det_id in (1, 2):
            track = Track.create(detection_id=det_id, class_id=0, class_name="person",
                                 bbox=det(det_id).bbox, anchor=det(det_id).measured_anchor,
                                 anchor_source=det(det_id).anchor_source,
                                 detector_score=0.9, sensor_timestamp_ns=stale,
                                 receive_timestamp_ns=stale, display_index=det_id)
            track.state = TrackState.LOST_REACQUIRABLE
            track.last_measurement_ns = stale
            track.observations = 5
            manager._tracks.append(track)

        result = manager.update(dset([det(9)], sensor_ns=sensor), sensor)
        ambiguous = [track for track in result.tracks if track.ambiguous]
        self.assertEqual(len(ambiguous), 2,
                         "§32: neither identity may be presented as the answer")
        for track in ambiguous:
            self.assertFalse(track.selectable)
            self.assertEqual(len(track.ambiguity_candidates), 1)
        self.assertEqual(result.counters.ambiguous_frames,
                         manager.counters.ambiguous_frames)

    def test_a_clear_measurement_resolves_ambiguity_on_the_next_frame(self):
        manager = TrackManager(commissioned_config())
        sensor = at(5)
        stale = sensor - ms(0.5)
        track = Track.create(detection_id=1, class_id=0, class_name="person",
                             bbox=det(1).bbox, anchor=det(1).measured_anchor,
                             anchor_source=det(1).anchor_source, detector_score=0.9,
                             sensor_timestamp_ns=stale, receive_timestamp_ns=stale,
                             display_index=1)
        track.state = TrackState.LOST_REACQUIRABLE
        track.last_measurement_ns = stale
        manager._tracks.append(track)
        manager.update(dset([det(9)], sensor_ns=sensor), sensor)
        result = manager.update(dset([det(9)], sensor_ns=sensor + ms(0.06)),
                                sensor + ms(0.06))
        self.assertFalse(result.tracks[0].ambiguous)


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
