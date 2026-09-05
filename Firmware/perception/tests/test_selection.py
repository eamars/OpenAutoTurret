"""§52 — selection: authority, idempotency, loss, ambiguity, and the TTL.

These cases read as command transcripts, because that is what they are. §3.4's evidence
against the retired system is a log line — ``select_target 2 -> "selected Person #1"`` — and
every assertion here is the kind of line that would have caught it: what was asked, what the
daemon believed, what the generation counter did, and what the controller was told.

The generation counter gets the most scrutiny of anything in this file. It is the only signal
the controller has for "should I reset acquisition", so a bump that should not have happened
is a visible aim glitch, and a missing bump means a genuine new target is treated as a
re-request. Both failure modes are silent in the image and obvious on the range.
"""
from __future__ import annotations

import unittest

from perception.config import SelectionConfig, SelectionPolicy
from perception.events import EventLog, EventType
from perception.protocol.selected_target import TargetState
from perception.selection.alias_map import AliasMap
from perception.selection.protocol import (ClearTargetRequest, SelectTargetRequest,
                                           SelectionReason)
from perception.selection.target_selection_manager import TargetSelectionManager
from perception.tests.support import (at, commissioned_config, commissioned_thresholds,
                                      det, dset, ms, track_at, track_set_of)
from perception.tracking.track import TrackState
from perception.tracking.track_manager import TrackManager

SENSOR = at(10)


def request(uuid: str, *, seen: int = 1, request_id: str = "r1") -> SelectTargetRequest:
    return SelectTargetRequest(request_id=request_id, track_uuid=uuid,
                               track_set_sequence_seen_by_ui=seen)


class TestExplicitSelection(unittest.TestCase):
    def setUp(self):
        self.events = EventLog()
        self.protected = []
        self.selector = TargetSelectionManager(
            commissioned_config(), event_log=self.events,
            on_selected=lambda uuid: self.protected.append(uuid))
        self.target = track_at(0.5, index=1)
        self.other = track_at(0.8, index=2)
        self.set = track_set_of([self.target, self.other], sequence=1)

    def test_selection_is_accepted_and_authoritative(self):
        ack = self.selector.select(request(self.target.track_uuid, seen=1), self.set,
                                   at(11))
        self.assertTrue(ack.accepted)
        self.assertIs(ack.reason, SelectionReason.ACCEPTED)
        self.assertEqual(ack.selected_track_uuid, self.target.track_uuid)
        self.assertEqual(ack.selected_display_label, "Person #1")
        self.assertEqual(ack.selection_generation, 1)
        self.assertFalse(ack.selection_unchanged)
        self.assertEqual(ack.authoritative_track_set_sequence, 1)
        self.assertEqual(self.protected, [self.target.track_uuid],
                         "§26: the tracker must learn whom to protect from eviction")

    def test_repeating_the_selection_does_not_bump_the_generation(self):
        first = self.selector.select(request(self.target.track_uuid), self.set, at(11))
        again = self.selector.select(request(self.target.track_uuid, request_id="r2"),
                                    self.set, at(12))
        self.assertTrue(again.accepted)
        self.assertIs(again.reason, SelectionReason.SELECTION_UNCHANGED)
        self.assertTrue(again.is_idempotent)
        self.assertEqual(again.selection_generation, first.selection_generation,
                         "§30: a duplicate must not reset the controller's acquisition")
        self.assertEqual(self.events.count(EventType.TARGET_SELECTION_IDEMPOTENT), 1)
        self.assertEqual(self.events.count(EventType.TARGET_SELECTED), 1)

    def test_switching_target_bumps_the_generation(self):
        self.selector.select(request(self.target.track_uuid), self.set, at(11))
        ack = self.selector.select(request(self.other.track_uuid, request_id="r2"),
                                   self.set, at(12))
        self.assertTrue(ack.accepted)
        self.assertEqual(ack.selection_generation, 2)
        self.assertFalse(ack.is_idempotent)

    def test_a_tentative_identity_is_not_confirmable(self):
        tentative = track_at(0.2, index=3, state=TrackState.TENTATIVE)
        subset = track_set_of([tentative], sequence=1)
        ack = self.selector.select(request(tentative.track_uuid), subset, at(11))
        self.assertTrue(ack.rejected)
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_CONFIRMED)
        # The refusal echoes what IS selected, so the client repaints truthfully (§29).
        self.assertEqual(ack.selected_track_uuid, "")

    def test_an_occluded_identity_is_not_currently_selectable(self):
        occluded = track_at(0.5, index=1, state=TrackState.OCCLUDED)
        subset = track_set_of([occluded], sequence=1)
        ack = self.selector.select(request(occluded.track_uuid), subset, at(11))
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE)
        self.assertIn("occluded", ack.detail)

    def test_an_unknown_uuid_is_not_found(self):
        ack = self.selector.select(request("ffffffff-0000"), self.set, at(11))
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_FOUND)
        self.assertEqual(self.selector.rejections["TRACK_NOT_FOUND"], 1)

    def test_an_empty_uuid_is_not_found(self):
        ack = self.selector.select(request(""), self.set, at(11))
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_FOUND)

    def test_a_merged_uuid_is_told_to_use_the_survivor(self):
        aliases = AliasMap()
        aliases.add(self.other.track_uuid, self.target.track_uuid,
                    reason="persistent_duplicate_overlap", sensor_timestamp_ns=SENSOR)
        selector = TargetSelectionManager(commissioned_config(), alias_map=aliases)
        ack = selector.select(request(self.other.track_uuid), self.set, at(11))
        self.assertIs(ack.reason, SelectionReason.TRACK_MERGED_USE_SURVIVOR)
        self.assertIn(self.target.track_uuid[:8], ack.detail)
        self.assertFalse(selector.state.has_selection,
                         "§25.1: a request is never silently re-pointed")

    def test_a_merged_uuid_already_selected_is_idempotent(self):
        aliases = AliasMap()
        aliases.add(self.other.track_uuid, self.target.track_uuid, reason="dupe",
                    sensor_timestamp_ns=SENSOR)
        selector = TargetSelectionManager(commissioned_config(), alias_map=aliases)
        selector.select(request(self.target.track_uuid), self.set, at(11))
        ack = selector.select(request(self.other.track_uuid, request_id="r2"), self.set,
                              at(12))
        self.assertIs(ack.reason, SelectionReason.SELECTION_UNCHANGED)

    def test_a_stale_ui_may_only_be_blamed_when_it_is_actually_stale(self):
        # The identity is gone AND the UI admits an older sequence: only then is
        # STALE_UI_TRACK_SET the truthful answer.
        selector = TargetSelectionManager(commissioned_config())
        selector.select(request(self.target.track_uuid), self.set, at(11))
        gone = track_set_of([self.other], sequence=9)
        ack = selector.select(request(self.target.track_uuid, seen=3), gone, at(12))
        self.assertIs(ack.reason, SelectionReason.SELECTION_UNCHANGED,
                      "the selection is unchanged before staleness is even considered")

        other_selection = TargetSelectionManager(commissioned_config())
        other_selection.select(request(self.target.track_uuid), self.set, at(11))
        missing = track_set_of([self.other], sequence=9)
        ack = other_selection.select(request("deadbeef-1234", seen=3), missing, at(12))
        self.assertIs(ack.reason, SelectionReason.STALE_UI_TRACK_SET)

        fresh = TargetSelectionManager(commissioned_config())
        ack = fresh.select(request("deadbeef-1234", seen=9), missing, at(12))
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_FOUND,
                      "a current view of a nonexistent identity is not the UI's fault")

    def test_an_uncommissioned_selectable_threshold_refuses(self):
        config = commissioned_config(thresholds=commissioned_thresholds(selectable=None))
        selector = TargetSelectionManager(config)
        subset = track_set_of([track_at(0.5)], sequence=1)
        ack = selector.select(request(subset.tracks[0].track_uuid), subset, at(11))
        self.assertIs(ack.reason, SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE)
        self.assertIn("COMMISSION", ack.detail)


class TestLossAndTtl(unittest.TestCase):
    def setUp(self):
        self.events = EventLog()
        self.selector = TargetSelectionManager(commissioned_config(),
                                              event_log=self.events)
        self.track = track_at(0.5, index=1)
        self.set = track_set_of([self.track], sequence=1)
        self.selector.select(request(self.track.track_uuid), self.set, at(11))

    def _update(self, state, *, sequence=2, sensor_ns=None, **mutations):
        """Advance the one selected identity into `state` and publish a frame.

        The identity is mutated rather than replaced: a test that built a fresh Track per
        case would be selecting — and then losing — a UUID the selection never held.
        """
        self.track.state = state
        self.track.measurement_valid = state is TrackState.CONFIRMED_VISIBLE
        for name, value in mutations.items():
            setattr(self.track, name, value)
        sensor = at(11) if sensor_ns is None else sensor_ns
        return self.selector.update(track_set_of([self.track], sequence=sequence,
                                                 sensor_ns=sensor), sensor)

    def test_occlusion_keeps_the_selection_and_withdraws_the_measurement(self):
        observation = self._update(TrackState.OCCLUDED, sensor_ns=SENSOR + ms(0.2))
        self.assertIs(observation.target_state, TargetState.OCCLUDED)
        self.assertFalse(observation.measurement_valid)
        self.assertEqual(observation.track_uuid, self.track.track_uuid)
        self.assertEqual(observation.selection_generation, 1,
                         "§31: losing a measurement is not a change of target")

    def test_lost_keeps_the_selection_within_the_ttl(self):
        observation = self._update(TrackState.LOST_REACQUIRABLE, sensor_ns=SENSOR,
                                   last_measurement_ns=SENSOR - ms(0.6))
        self.assertIs(observation.target_state, TargetState.LOST)
        self.assertFalse(observation.measurement_valid)
        self.assertEqual(observation.track_uuid, self.track.track_uuid)
        self.assertEqual(self.events.count(EventType.TARGET_STALE), 0)

    def test_expiry_publishes_one_stale_event_and_an_explicit_no_target(self):
        observation = self._update(TrackState.LOST_REACQUIRABLE, sensor_ns=SENSOR,
                                   last_measurement_ns=SENSOR - ms(3.1))
        self.assertIs(observation.target_state, TargetState.NO_TARGET)
        self.assertFalse(observation.measurement_valid)
        self.assertEqual(observation.track_uuid, "")
        self.assertEqual(self.events.count(EventType.TARGET_STALE), 1,
                         "§31: exactly one event for the moment the subject disappeared")
        self.assertEqual(observation.selection_generation, 2)

        again = self._update(TrackState.LOST_REACQUIRABLE, sequence=3,
                             sensor_ns=SENSOR + ms(0.06))
        self.assertEqual(self.events.count(EventType.TARGET_STALE), 1,
                         "the same expiry must not re-report every frame")
        self.assertEqual(again.selection_generation, 2)

    def test_an_unmeasurable_age_does_not_end_the_selection(self):
        # ``ms_from_ns`` answers -1.0 ("unknown") for a missing stamp rather than 0, and §31
        # must treat unknown as "do not expire". Guessing an age here would silently drop the
        # operator's target on a malformed batch.
        observation = self._update(TrackState.LOST_REACQUIRABLE, sensor_ns=SENSOR,
                                   last_measurement_ns=0, created_ns=0)
        self.assertIs(observation.target_state, TargetState.LOST)
        self.assertEqual(self.events.count(EventType.TARGET_STALE), 0)
        self.assertTrue(self.selector.state.has_selection)

    def test_a_vanished_identity_ends_the_selection_once(self):
        self.selector.update(track_set_of([], sequence=2, sensor_ns=SENSOR), SENSOR)
        self.assertEqual(self.events.count(EventType.TARGET_STALE), 1)
        self.assertFalse(self.selector.state.has_selection)

    def test_a_merge_moves_the_selection_without_a_generation_bump(self):
        survivor = track_at(0.52, index=1, display_index=1)
        self.selector.aliases.add(self.track.track_uuid, survivor.track_uuid,
                                  reason="dupe", sensor_timestamp_ns=SENSOR)
        observation = self.selector.update(track_set_of([survivor], sequence=2,
                                                        sensor_ns=SENSOR), SENSOR)
        self.assertEqual(observation.track_uuid, survivor.track_uuid)
        self.assertEqual(observation.selection_generation, 1,
                         "§25.1: the tracker tidied itself; the target did not change")
        self.assertEqual(self.events.count(EventType.TARGET_SELECTED), 1,
                         "no new selection was made, so no new selection event")

    def test_ambiguity_retains_the_selection_and_disables_the_measurement(self):
        observation = self._update(TrackState.LOST_REACQUIRABLE, sensor_ns=SENSOR,
                                   ambiguous=True,
                                   ambiguity_candidates=("rival-uuid-1",))
        self.assertIs(observation.target_state, TargetState.AMBIGUOUS)
        self.assertFalse(observation.measurement_valid,
                         "§32: the controller gets no valid new measurement")
        self.assertEqual(observation.track_uuid, self.track.track_uuid,
                         "§32: the selection is retained, not guessed at")
        self.assertEqual(observation.ambiguity, 1.0)
        self.assertEqual(self.events.count(EventType.TARGET_AMBIGUOUS), 1)

        # Still ambiguous three frames later: one episode, not one event per frame.
        for sequence in range(3, 6):
            self._update(TrackState.LOST_REACQUIRABLE, sequence=sequence,
                         sensor_ns=SENSOR + ms(0.06 * sequence), ambiguous=True,
                         ambiguity_candidates=("rival-uuid-1",))
        self.assertEqual(self.events.count(EventType.TARGET_AMBIGUOUS), 1)

    def test_a_clear_measurement_resumes_validity(self):
        self._update(TrackState.OCCLUDED, sensor_ns=SENSOR + ms(0.2))
        observation = self._update(TrackState.CONFIRMED_VISIBLE, sequence=3,
                                   sensor_ns=SENSOR + ms(0.26))
        self.assertIs(observation.target_state, TargetState.CONFIRMED_VISIBLE)
        self.assertTrue(observation.measurement_valid)


class TestClear(unittest.TestCase):
    def setUp(self):
        self.events = EventLog()
        self.selector = TargetSelectionManager(commissioned_config(),
                                              event_log=self.events)
        self.track = track_at(0.5, index=1)
        self.set = track_set_of([self.track], sequence=1)

    def test_clear_is_authoritative_and_publishes_no_target(self):
        self.selector.select(request(self.track.track_uuid), self.set, at(11))
        ack = self.selector.clear(ClearTargetRequest(request_id="c1"), self.set, at(12))
        self.assertTrue(ack.accepted)
        self.assertIs(ack.reason, SelectionReason.CLEARED)
        self.assertEqual(ack.selection_generation, 2)
        self.assertEqual(ack.selected_track_uuid, "")
        observation = self.selector.update(self.set, at(12))
        self.assertIs(observation.target_state, TargetState.NO_TARGET)
        self.assertEqual(self.events.count(EventType.TARGET_CLEARED), 1)

    def test_clearing_twice_is_a_successful_no_op(self):
        self.selector.select(request(self.track.track_uuid), self.set, at(11))
        self.selector.clear(ClearTargetRequest(), self.set, at(12))
        ack = self.selector.clear(ClearTargetRequest(request_id="c2"), self.set, at(13))
        self.assertTrue(ack.accepted, "pressing clear twice is not an error")
        self.assertIs(ack.reason, SelectionReason.NOTHING_TO_CLEAR)
        self.assertTrue(ack.selection_unchanged)
        self.assertEqual(ack.selection_generation, 2, "no further bump")


class TestAutoSelect(unittest.TestCase):
    def _selector(self, **selection_overrides):
        # §28.2: AUTO_SELECT_SINGLE is exactly-one-eligible, held for a dwell, and past separate
        # higher confidence thresholds than the manual gate. auto_select_min_* are load-bearing.
        values = {"policy": SelectionPolicy.AUTO_SELECT_SINGLE,
                  "auto_select_single_dwell_ms": 500.0,
                  "auto_select_min_detector_score": 0.70,
                  "auto_select_min_identity_confidence": 0.60}
        values.update(selection_overrides)
        config = commissioned_config(selection=SelectionConfig(**values))
        return TargetSelectionManager(config)

    def _drive(self, selector, tracks_by_frame):
        observations = []
        for index, tracks in enumerate(tracks_by_frame):
            subset = track_set_of(tracks, sequence=index + 1, sensor_ns=at(index))
            observations.append(selector.update(subset, at(index)))
        return observations

    def test_explicit_only_policy_never_selects_on_its_own(self):
        selector = TargetSelectionManager(commissioned_config())
        candidate = track_at(0.5, index=1)
        self._drive(selector, [[candidate]] * 12)
        self.assertFalse(selector.state.has_selection)

    def test_a_single_candidate_is_selected_only_after_the_dwell(self):
        selector = self._selector()
        candidate = track_at(0.5, index=1)
        observations = self._drive(selector, [[candidate]] * 12)
        self.assertEqual(observations[0].target_state, TargetState.NO_TARGET,
                         "one frame of one box must not take the turret")
        self.assertFalse(observations[5].measurement_valid, "8 frames = 480 ms < 500 ms")
        self.assertTrue(selector.state.has_selection)
        self.assertEqual(selector.state.selected_uuid, candidate.track_uuid)
        self.assertTrue(observations[-1].measurement_valid)

    def test_a_second_candidate_cancels_the_dwell(self):
        selector = self._selector()
        a = track_at(0.5, index=1)
        b = track_at(0.8, index=2)
        self._drive(selector, [[a]] * 6 + [[a, b]] * 6)
        self.assertFalse(selector.state.has_selection,
                         "§28.2: a second eligible person is a real ambiguity, not a "
                         "hardware cadence — exactly-one must not be traded for a best pick")

    def test_a_brief_gap_does_not_cancel_the_dwell(self):
        # The IMX500 on-sensor tensor lands on every other frame (camera ~30 fps, network
        # ~15 fps), so between measurements the sole candidate reads as OCCLUDED. §23 holds the
        # identity and retires it only after the window — so a skipped tensor is a hardware
        # cadence, not evidence, and must not reset the dwell or the turret never acquires.
        selector = self._selector()
        candidate = track_at(0.5, index=1)
        frames = []
        for index in range(15):
            candidate.state = (TrackState.OCCLUDED if 6 <= index <= 8
                               else TrackState.CONFIRMED_VISIBLE)
            candidate.measurement_valid = candidate.state is TrackState.CONFIRMED_VISIBLE
            frames.append([candidate])                   # the same identity every frame
        self._drive(selector, frames)
        self.assertTrue(selector.state.has_selection,
                        "an OCCLUDED sole candidate must keep its dwell across a skipped tensor")
        self.assertEqual(selector.state.selected_uuid, candidate.track_uuid)

    def test_a_lost_candidate_really_cancels_the_dwell(self):
        selector = self._selector()
        candidate = track_at(0.5, index=1)
        self._drive(selector, [[candidate]] * 6 + [[]] * 6)
        self.assertFalse(selector.state.has_selection,
                         "a genuinely absent candidate (not in the track set) must reset the dwell")

    def test_the_sole_candidate_must_be_the_same_identity_throughout(self):
        selector = self._selector()
        first = track_at(0.5, index=1)
        second = track_at(0.5, index=1)          # different uuid, same place, same label
        self._drive(selector, [[first]] * 5 + [[second]] * 5)
        self.assertFalse(selector.state.has_selection,
                         "a dwell started by one identity may not be cashed in by another")

    def test_an_operator_selection_is_never_stolen(self):
        selector = self._selector()
        chosen = track_at(0.8, index=2, score=0.9)
        sole = track_at(0.5, index=1, score=0.5)
        ack = selector.select(request(chosen.track_uuid),
                              track_set_of([sole, chosen], sequence=1), at(0))
        self.assertTrue(ack.accepted, ack.detail)
        # Both stay visible. The property is "auto-select never takes a held target away";
        # if the operator's identity left the frame the selection would legitimately end, and
        # then a sole candidate may be picked — which is a different, and allowed, story.
        self._drive(selector, [[sole, chosen]] * 12)
        self.assertEqual(selector.state.selected_uuid, chosen.track_uuid)

    def test_uncommissioned_auto_thresholds_disable_the_feature(self):
        selector = self._selector(auto_select_min_detector_score=None)
        candidate = track_at(0.5, index=1)
        self._drive(selector, [[candidate]] * 12)
        self.assertFalse(selector.state.has_selection)
        self.assertGreater(selector.auto.denied_missing_thresholds, 0)


class TestObservationContract(unittest.TestCase):
    def test_the_observation_carries_no_control_authority(self):
        # §36 and §54: prediction and actuation belong to the controller. A field that
        # smuggles a lead angle, a motor id or a drive mode back across this boundary
        # re-creates the double-filtered aim, so the absence is asserted structurally.
        selector = TargetSelectionManager(commissioned_config())
        track = track_at(0.5, index=1)
        subset = track_set_of([track], sequence=1)
        selector.select(request(track.track_uuid), subset, at(11))
        payload = selector.update(subset, at(11)).to_dict()
        forbidden = ("motor", "can_id", "servo", "drive_mode", "lead", "cybergear",
                     "acceleration", "torque", "aim_point_px")
        offenders = [key for key in payload if any(word in key.lower() for word in forbidden)]
        self.assertEqual(offenders, [])

    def test_the_four_quality_fields_stay_separate(self):
        selector = TargetSelectionManager(commissioned_config())
        track = track_at(0.5, index=1)
        track.detector_score = 0.55
        track.identity_confidence = 0.95
        track.association_quality = 0.80
        track.measurement_quality = 0.40
        subset = track_set_of([track], sequence=1)
        selector.select(request(track.track_uuid), subset, at(11))
        observation = selector.update(subset, at(11))
        self.assertAlmostEqual(observation.detector_score, 0.55, places=6)
        self.assertAlmostEqual(observation.identity_confidence, 0.95, places=6)
        self.assertAlmostEqual(observation.association_quality, 0.80, places=6)
        self.assertNotIn("measurement_quality", observation.to_dict(),
                         "the aim-suitability number is published in the TrackSet; the "
                         "observation carries the three the control loop reads")

    def test_a_failing_notification_is_counted_not_raised(self):
        def explode(uuid):
            raise RuntimeError("tracker went away")

        selector = TargetSelectionManager(commissioned_config(), on_selected=explode)
        track = track_at(0.5, index=1)
        ack = selector.select(request(track.track_uuid), track_set_of([track], sequence=1),
                              at(11))
        self.assertTrue(ack.accepted, "the selection did happen; the notice failed")
        self.assertEqual(selector.callback_failures, 1)

    def test_no_target_is_published_explicitly_every_frame(self):
        selector = TargetSelectionManager(commissioned_config())
        subset = track_set_of([], sequence=1)
        observation = selector.update(subset, at(11))
        self.assertIs(observation.target_state, TargetState.NO_TARGET)
        self.assertEqual(observation.track_uuid, "")
        self.assertEqual(observation.session_uuid, "session-test",
                         "§34: an explicit no-target is not the same as silence")


class TestWithLiveTracker(unittest.TestCase):
    """The two halves driven together, on real timestamps."""

    def test_supplied_collaborators_are_used_not_replaced(self):
        # AliasMap and EventLog both define __len__, so `alias_map or AliasMap()` — which
        # reads harmlessly — discards the empty-but-shared instance and writes aliases into
        # a private map nobody reads. Every §25.1 hand-off would then be invisible.
        events = EventLog()
        aliases = AliasMap()
        tracker = TrackManager(commissioned_config(), event_log=events,
                               alias_map=aliases)
        selector = TargetSelectionManager(tracker.config, event_log=events,
                                          alias_map=tracker.aliases)
        self.assertIs(tracker.events, events)
        self.assertIs(tracker.aliases, aliases)
        self.assertIs(selector.aliases, aliases)

    def _run(self):
        events = EventLog()
        manager_state = {}
        config = commissioned_config()
        tracker = TrackManager(config, event_log=events)
        selector = TargetSelectionManager(
            config, event_log=events, alias_map=tracker.aliases,
            on_selected=lambda uuid: manager_state.update(selected=uuid))
        published = []
        observations = []
        for index in range(12):
            detections = [det(1, cx=0.5, score=0.9)] if index < 6 else []
            track_set = tracker.update(dset(detections, frame_index=index), at(index))
            published.append(track_set)
            observations.append(selector.update(track_set, at(index)))
        return tracker, selector, events, published, observations, manager_state

    def test_selection_survives_a_short_absence_end_to_end(self):
        tracker, selector, events, published, observations, protected = self._run()
        chosen = published[3].tracks[0]
        ack = selector.select(SelectTargetRequest(request_id="r1",
                                                  track_uuid=chosen.track_uuid,
                                                  track_set_sequence_seen_by_ui=4),
                              published[3], at(3))
        self.assertTrue(ack.accepted)
        self.assertEqual(protected["selected"], chosen.track_uuid)

        # Feed the rest of the run with the person gone from frame 6 onwards.
        for index in range(4, 12):
            detections = [det(1, cx=0.5, score=0.9)] if index < 6 else []
            track_set = tracker.update(dset(detections, frame_index=index), at(index))
            observation = selector.update(track_set, at(index))
            if index >= 7:
                self.assertEqual(observation.track_uuid, chosen.track_uuid,
                                 "§31: the selection holds through occlusion and loss")
                self.assertFalse(observation.measurement_valid)

    def test_tracker_and_selector_share_one_alias_map(self):
        # The selector must see merges the resolver performed, or §25.1's atomic follow is
        # two dictionaries that disagree — which is how a selection becomes a dangling UUID.
        tracker = TrackManager(commissioned_config())
        selector = TargetSelectionManager(tracker.config, alias_map=tracker.aliases)
        self.assertIs(selector.aliases, tracker.aliases)


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
