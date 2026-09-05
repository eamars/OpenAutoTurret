"""§25 / §25.1 — two identities, one person: when a merge is justified, and who survives.

These tests feed **raw overlapping detections** and skip the detector-level deduplicator on
purpose. §16 tries to stop the second box before it becomes an identity; §25 exists because
it sometimes will not — different pose, different scale, a class-boundary disagreement — and
the resolver's correctness cannot be inherited from a stage upstream of it.

The dwell is the load-bearing rule. Every "held, not merged" test here is asserting that the
resolver *refused* to act, which is the behaviour most likely to be "improved" away by
someone who finds the waiting annoying: a single frame of overlap is also what two strangers
brushing past each other look like.
"""
from __future__ import annotations

import unittest

from perception.config import DedupConfig, TrackingConfig
from perception.events import EventLog, EventType
from perception.tests.support import (at, commissioned_config, det, dset, ms, track_at)
from perception.tracking.diagnostics import AssociationDiagnostics
from perception.tracking.duplicate_track_resolver import DuplicateTrackResolver
from perception.tracking.track_manager import TrackManager


class TestHeldPairs(unittest.TestCase):
    def setUp(self):
        self.resolver = DuplicateTrackResolver(TrackingConfig(), DedupConfig(
            nms_iou=0.45, containment_ratio=0.85, center_distance_norm=0.05))
        self.sensor = at(0)

    def _a_and_b(self, a_cx=0.50, b_cx=0.53, **kwargs):
        a = track_at(a_cx, index=1, **kwargs)
        b = track_at(b_cx, index=2, **kwargs)
        return a, b

    def test_overlap_on_one_frame_never_merges(self):
        a, b = self._a_and_b()
        decisions = self.resolver.observe([a, b], self.sensor)
        self.assertEqual(decisions, [])
        self.assertEqual(self.resolver.stats()["held"], 1)
        self.assertEqual(len(self.resolver.pending_pairs()), 1)

    def test_overlap_that_persists_past_the_dwell_merges(self):
        a, b = self._a_and_b()
        for frame in range(6):
            decisions = self.resolver.observe([a, b], at(frame))
        self.assertEqual(len(decisions), 1)
        self.assertEqual(decisions[0].reason, "persistent_duplicate_overlap")
        self.assertEqual(decisions[0].evidence.observations, 6)

    def test_overlap_that_stops_is_forgotten_not_aged(self):
        # Two people cross, separate, then cross again a minute later. The second crossing
        # must start a fresh dwell; inheriting the first would merge them on evidence that
        # expired a minute ago.
        a, b = self._a_and_b()
        for frame in range(3):
            self.resolver.observe([a, b], at(frame))
        self.assertEqual(len(self.resolver.pending_pairs()), 1)
        apart_b = track_at(0.9, index=2)
        self.resolver.observe([a, apart_b], at(4))
        self.assertEqual(self.resolver.pending_pairs(), [])
        merged_after = self.resolver.observe([a, b], at(5))
        self.assertEqual(merged_after, [], "the dwell restarted from frame 5")

    def test_different_classes_are_never_one_person(self):
        a = track_at(0.5, index=1)
        b = track_at(0.51, index=2)
        b.class_id, b.class_name = 1, "bicycle"
        for frame in range(6):
            decisions = self.resolver.observe([a, b], at(frame))
        self.assertEqual(decisions, [])

    def test_without_commissioned_thresholds_the_resolver_declines_loudly(self):
        resolver = DuplicateTrackResolver(TrackingConfig(), DedupConfig())
        a, b = self._a_and_b()
        self.assertEqual(resolver.observe([a, b], at(0)), [])
        self.assertTrue(resolver.disabled)
        self.assertIn("COMMISSION", resolver.disabled_reason)

    def test_boxes_far_apart_in_depth_are_not_duplicates(self):
        # One person directly behind another: containment is total, IoU is high, and they
        # are two people. §16.2's centre-distance term is the only thing separating these
        # cases, and it is also what keeps a merge from deleting a person.
        near = track_at(0.5, index=1, height=0.8)
        far = track_at(0.5, index=2, height=0.15)
        for frame in range(6):
            decisions = self.resolver.observe([near, far], at(frame))
        self.assertEqual(decisions, [])


class TestSurvivorChoice(unittest.TestCase):
    """§25.1's order, evaluated one rule at a time."""

    def setUp(self):
        self.resolver = DuplicateTrackResolver(TrackingConfig(), DedupConfig(
            nms_iou=0.45, containment_ratio=0.85, center_distance_norm=0.05))

    def _merge(self, a, b, frames=6, selected=None):
        if selected is not None:
            self.resolver.note_selected(selected)
        decisions = []
        for frame in range(frames):
            decisions.extend(self.resolver.observe([a, b], at(frame)))
        self.assertEqual(len(decisions), 1, "expected exactly one merge")
        return decisions[0]

    def test_the_selected_identity_survives_even_against_a_stronger_history(self):
        a = track_at(0.50, index=1)
        a.identity_confidence, a.visible_ms_total = 0.99, 9000.0
        b = track_at(0.53, index=2)
        b.identity_confidence, b.visible_ms_total = 0.10, 50.0
        decision = self._merge(a, b, selected=b.track_uuid)
        self.assertEqual(decision.survivor_uuid, b.track_uuid)
        self.assertEqual(decision.survivor_rule, "selected_identity")
        self.assertEqual(decision.merged_uuid, a.track_uuid)

    def test_higher_identity_confidence_wins_without_a_selection(self):
        a = track_at(0.50, index=1)
        a.identity_confidence = 0.20
        b = track_at(0.53, index=2)
        b.identity_confidence = 0.80
        decision = self._merge(a, b)
        self.assertEqual(decision.survivor_uuid, b.track_uuid)
        self.assertEqual(decision.survivor_rule, "higher_identity_confidence")

    def test_visible_time_then_creation_time_then_label(self):
        a = track_at(0.50, index=1)
        a.identity_confidence = b_conf = 0.5
        b = track_at(0.53, index=2)
        b.identity_confidence = b_conf
        b.visible_ms_total = 4000.0
        decision = self._merge(a, b)
        self.assertEqual(decision.survivor_rule, "more_visible_time")
        self.assertEqual(decision.survivor_uuid, b.track_uuid)

        b.visible_ms_total = a.visible_ms_total
        b.created_ns = a.created_ns + ms(1.0)
        decision = self._merge(a, b)
        self.assertEqual(decision.survivor_rule, "older_identity")

        b.created_ns = a.created_ns
        decision = self._merge(a, b)
        self.assertEqual(decision.survivor_rule, "lower_display_index")
        self.assertEqual(decision.survivor_uuid, a.track_uuid)

    def test_prefer_uses_the_same_rule_as_merge(self):
        # The suppression flag is set from `prefer`, so the identity held back from selection
        # while the dwell runs must be the one that would lose the merge. Anything else makes
        # the suppression flicker between two identities.
        a = track_at(0.50, index=1)
        b = track_at(0.53, index=2)
        b.identity_confidence = 0.9
        self.assertEqual(self.resolver.prefer(a, b), a.track_uuid)
        self.assertEqual(self.resolver.prefer(None, b), b.track_uuid)
        self.assertEqual(self.resolver.prefer(a, None), a.track_uuid)


class TestManagerIntegration(unittest.TestCase):
    """Two live identities for one person, inside a running ``TrackManager``.

    The sequence is scripted rather than declared as a helper, because the interesting part
    is *how* a duplicate comes to exist: two boxes start apart (so each legitimately becomes
    an identity), and one drifts onto the other. That is what a detector does when its second
    box settles onto a person it had been reporting beside — and it is the only way a
    duplicate survives the creation guard, which refuses to mint an identity for a box that
    already duplicates a live one.
    """

    #: (cx of the first box, cx of the second) per frame. 0.10 wide, so 0.53 separation is
    #: IoU ≈ 0.54 / containment 0.70, and each step is inside §21's gate for one frame.
    SCRIPT = ([(0.50, 0.75)] * 3
              + [(0.50, 0.65), (0.50, 0.59), (0.50, 0.55)]
              + [(0.50, 0.53)] * 12)

    def _drive(self, manager, frames=16, start=0):
        sets = []
        for offset in range(frames):
            index = start + offset
            cx_a, cx_b = self.SCRIPT[min(index, len(self.SCRIPT) - 1)]
            sets.append(manager.update(
                dset([det(1, cx=cx_a, score=0.9), det(2, cx=cx_b, score=0.8)],
                     frame_index=index), at(index)))
        return sets

    def test_two_identities_for_one_person_merge_after_the_dwell(self):
        manager = TrackManager(commissioned_config())
        sets = self._drive(manager, frames=3)
        self.assertEqual(len(sets[-1].tracks), 2, "two boxes apart are two identities")
        sets = self._drive(manager, frames=13, start=3)
        self.assertEqual(len(sets[-1].tracks), 1,
                         "§25: after the dwell, one person is one identity")
        self.assertEqual(manager.counters.tracks_merged, 1)

    def test_merged_identity_is_attributable_and_aliased(self):
        events = EventLog()
        manager = TrackManager(commissioned_config(), event_log=events)
        sets = self._drive(manager, frames=3)
        duplicate_uuid = by_label(sets[-1], 2).track_uuid
        self._drive(manager, frames=13, start=3)
        merges = events.recent(types=(EventType.TRACK_MERGED,))
        self.assertEqual(len(merges), 1)
        event = merges[0]
        self.assertEqual(event.track_uuid, duplicate_uuid)
        self.assertIn("evidence", event.fields)
        self.assertIn("rule", event.fields)
        self.assertEqual(manager.aliases.resolve(duplicate_uuid),
                         event.fields["survivor_uuid"])
        # Two different questions, two different methods (§Appendix A has to tell them
        # apart): `exists` answers "is this UUID still an identity?" — no, it is a name that
        # used to mean one — while `find` follows §25.1's alias so a caller holding the old
        # UUID can still reach the person. A selector that conflated them would answer
        # TRACK_NOT_FOUND where TRACK_MERGED_USE_SURVIVOR is the truthful reason.
        self.assertFalse(manager.exists(duplicate_uuid))
        resolved = manager.find(duplicate_uuid)
        self.assertIsNotNone(resolved)
        self.assertEqual(resolved.track_uuid, event.fields["survivor_uuid"])

    def test_selection_follows_the_survivor_atomically(self):
        manager = TrackManager(commissioned_config())
        sets = self._drive(manager, frames=3)
        duplicate = by_label(sets[-1], 2)
        # The moving duplicate has the weaker history, so without this the §25.1 confidence
        # rule would pick label 1 and the test would prove nothing about selection.
        self.assertNotEqual(duplicate.track_uuid, by_label(sets[-1], 1).track_uuid)
        manager.set_selected_uuid(duplicate.track_uuid)

        final = self._drive(manager, frames=13, start=3)[-1]
        self.assertEqual(len(final.tracks), 1)
        survivor = final.tracks[0]
        self.assertEqual(survivor.display_index, 2,
                         "§25.1: the selected identity is the survivor")
        self.assertEqual(manager._selected_uuid, survivor.track_uuid,
                         "§25.1: selection follows the alias; it does not clear")

    def test_pending_duplicate_is_not_selectable(self):
        # Before the dwell expires these are still two identities, and §25 permits holding the
        # weaker one back: an operator who clicks the duplicate gets a refusal now rather than
        # a target that dissolves into a merge a third of a second later.
        manager = TrackManager(commissioned_config())
        final = self._drive(manager, frames=12)[-1]
        self.assertEqual(len(final.tracks), 2, "the dwell has not expired yet")
        suppressed = [track for track in final.tracks if track.duplicate_resolving]
        self.assertEqual(len(suppressed), 1)
        self.assertFalse(suppressed[0].selectable)
        allowed, reason = manager.is_selectable(suppressed[0].track_uuid)
        self.assertFalse(allowed)
        self.assertIn("duplicate", reason)

    def test_a_detection_that_duplicates_a_live_identity_does_not_mint_another_one(self):
        # The anti-oscillation rule. Without it, merging away an identity leaves its detector
        # row unmatched, which creates a new identity next frame, which merges again 300 ms
        # later — a person whose label changes twice a second, the §27 symptom verbatim.
        manager = TrackManager(commissioned_config())
        final = self._drive(manager, frames=20)[-1]
        self.assertEqual(len(final.tracks), 1)
        self.assertGreaterEqual(manager.counters.detections_refused_duplicate, 1)
        self.assertEqual(len(manager._next_display_index["person"])
                         if isinstance(manager._next_display_index.get("person"), list)
                         else manager._next_display_index["person"], 3,
                         "only two labels were ever issued: one per real box")

    def test_merge_records_both_outcomes_in_the_diagnostics_ring(self):
        diagnostics = AssociationDiagnostics(capacity=64, enabled=True)
        manager = TrackManager(commissioned_config(), diagnostics=diagnostics)
        self._drive(manager, frames=20)
        merge_records = [record for frame in diagnostics.recent(limit=0)
                         for record in frame["tracks"] if record.get("kind") == "merge"]
        self.assertTrue(merge_records)
        decisions = {record["decision"] for record in merge_records}
        self.assertEqual(decisions, {"held", "merged"},
                         "§41: the merge that was refused is as reportable as the one taken")

    def test_two_people_one_behind_the_other_stay_two_identities(self):
        # The guard's cost has to be bounded: §16.2's centre-distance and scale terms are
        # what keep a queue of people from collapsing into one person.
        manager = TrackManager(commissioned_config())
        near = det(1, cx=0.50, height=0.8, score=0.9)
        far = det(2, cx=0.50, height=0.18, score=0.8)
        for index in range(8):
            result = manager.update(dset([near, far], frame_index=index), at(index))
        self.assertEqual(len(result.tracks), 2)
        self.assertEqual(manager.counters.tracks_merged, 0)


def by_label(track_set, label: int):
    return next(track for track in track_set.tracks if track.display_index == label)


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
