"""§41 — the association diagnostics ring: an identity decision must be readable later.

The requirement is not "have some logs". It is §53's question — *why did this identity stop
receiving measurements?* — answered from data that was already captured, in the order it
happened, with the gate names attached. An operator who has to re-run an experiment to find
out why the tracker lost a person has already lost the afternoon.

Two properties are tested hardest: the ring is **bounded** (a diagnostics buffer that grows
with uptime is a memory leak on a station that runs for days, and the interesting frames are
the ones from just before the fault, not from boot), and it is **optional at zero cost**
(when `enabled` is false, nothing is retained — a debug facility that keeps working in
production is how a Pi 5 starts dropping frames for no visible reason).
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest

from perception.config import TrackingConfig
from perception.tests.support import commissioned_config, det, dset
from perception.tracking.diagnostics import (AssociationDiagnostics, CandidateNote)
from perception.tracking.track_manager import TrackManager


class TestRingBounds(unittest.TestCase):
    def test_a_full_ring_drops_oldest_and_says_so(self):
        ring = AssociationDiagnostics(capacity=8, enabled=True)
        for frame in range(20):
            ring.begin_frame(frame + 1, 1_000_000_000 + frame * 60_000_000)
            ring.end_frame()
        self.assertEqual(ring.frames_recorded, 20)
        self.assertEqual(ring.frames_dropped, 12)
        self.assertEqual(len(ring.recent(limit=0)), 8)
        kept = [frame["frame_sequence"] for frame in ring.recent(limit=0)]
        self.assertEqual(kept, list(range(13, 21)),
                         "§41 keeps the newest frames: the fault is always recent")

    def test_capacity_below_usefulness_is_rejected(self):
        # A ring of two frames cannot answer "what led up to this" at all, and a typo'd
        # capacity of 1 would leave a facility that looks enabled and never helps.
        with self.assertRaises(ValueError):
            AssociationDiagnostics(capacity=2, enabled=True)

    def test_disabled_ring_retains_nothing(self):
        ring = AssociationDiagnostics(capacity=64, enabled=False)
        ring.begin_frame(1, 1_000_000_000)
        ring.record_detection(det(1))
        ring.record_track(_stub_track(), state_before="tentative", state_after="confirmed")
        ring.end_frame()
        self.assertEqual(ring.recent(limit=0), [])
        self.assertEqual(ring.frames_recorded, 0)
        self.assertEqual(ring.for_track("anything"), [])


class TestRecords(unittest.TestCase):
    def setUp(self):
        self.ring = AssociationDiagnostics(capacity=32, enabled=True)
        self.ring.begin_frame(7, 1_000_000_000)

    def test_detection_record_carries_candidates_and_the_outcome(self):
        notes = [CandidateNote(track_uuid="aaa", outcome="matched", cost=0.2, quality=0.9),
                 CandidateNote(track_uuid="bbb", outcome="gated", reason="class_mismatch")]
        self.ring.record_detection(det(1, score=0.8), candidates=notes, assigned_track="aaa")
        self.ring.end_frame()
        record = self.ring.recent(limit=1)[0]["detections"][0]
        self.assertEqual(record["detection_id"], 1)
        self.assertAlmostEqual(record["detector_score"], 0.8, places=6)
        self.assertEqual(record["assigned_track"], "aaa")
        self.assertEqual([c["outcome"] for c in record["candidates"]],
                         ["matched", "gated"])
        self.assertEqual(record["candidates"][1]["reason"], "class_mismatch")

    def test_suppressed_detection_keeps_its_reason(self):
        self.ring.record_detection(det(1), suppressed=True,
                                   dedup_reason="duplicate of live identity deadbeef")
        self.ring.end_frame()
        record = self.ring.recent(limit=1)[0]["detections"][0]
        self.assertTrue(record["suppressed"])
        self.assertIn("deadbeef", record["dedup_reason"])

    def test_merge_records_both_the_taken_and_the_refused(self):
        self.ring.record_merge("aaa", "bbb", reason="iou=0.62", persistent=False)
        self.ring.record_merge("aaa", "bbb", reason="iou=0.64 age_ms=310", persistent=True)
        self.ring.end_frame()
        records = self.ring.recent(limit=1)[0]["tracks"]
        self.assertEqual([r["decision"] for r in records], ["held", "merged"])
        self.assertTrue(all(r["kind"] == "merge" for r in records))

    def test_for_track_collects_every_mention_of_one_identity(self):
        self.ring.record_detection(det(1), candidates=[
            CandidateNote(track_uuid="aaa", outcome="gated", reason="impossible_scale")])
        self.ring.record_track(_stub_track("aaa"), state_before="confirmed",
                               state_after="occluded", miss_age_ms=210.0)
        self.ring.record_merge("aaa", "bbb", reason="overlap", persistent=True)
        self.ring.end_frame()
        hits = self.ring.for_track("aaa")
        kinds = ["merge" if "kind" in hit else
                 ("detection" if "detection_id" in hit else "track") for hit in hits]
        self.assertEqual(len(hits), 3,
                         "the gate, the transition and the merge are one story about aaa")
        self.assertIn("merge", kinds)
        self.assertEqual(self.ring.for_track("nope"), [])
        self.assertEqual(self.ring.for_track(""), [])


class TestDumpAndFault(unittest.TestCase):
    def test_dump_is_json_and_names_the_counts(self):
        ring = AssociationDiagnostics(capacity=16, enabled=True)
        for frame in range(4):
            ring.begin_frame(frame, 1_000_000_000 + frame)
            ring.record_detection(det(1))
            ring.end_frame()
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "association.json")
            written = ring.dump(path)
            with open(path, encoding="utf-8") as handle:
                payload = json.load(handle)
        self.assertEqual(written, 4)
        self.assertEqual(len(payload["frames"]), 4)
        self.assertEqual(payload["frames_recorded"], 4)
        self.assertEqual(payload["capacity"], 16)

    def test_persist_on_fault_writes_to_the_configured_path(self):
        ring = AssociationDiagnostics(capacity=16, enabled=True,
                                      persist_path=_tmp_path("assoc"))
        ring.begin_frame(1, 1_000_000_000)
        ring.end_frame()
        path = ring.persist_on_fault("track_set validation failed")
        try:
            self.assertIsNotNone(path)
            self.assertTrue(os.path.exists(path))
            self.assertEqual(ring.last_fault_reason, "track_set validation failed")
        finally:
            if path:
                os.remove(path)

    def test_a_ring_with_no_path_reports_nothing_rather_than_failing(self):
        ring = AssociationDiagnostics(capacity=16, enabled=True)
        self.assertIsNone(ring.persist_on_fault("no destination configured"))


class TestTrackerIntegration(unittest.TestCase):
    def test_the_tracker_fills_the_ring_while_it_works(self):
        tracking = TrackingConfig(diagnostics_enabled=True, diagnostics_capacity=32)
        manager = TrackManager(commissioned_config(tracking=tracking))
        manager.update(dset([det(1, cx=0.5, score=0.9)]), 1_000_000_000)
        manager.update(dset([det(1, cx=0.5, score=0.9)], frame_index=1),
                       1_060_000_000)
        frames = manager.diagnostics.recent(limit=0)
        self.assertEqual(len(frames), 2)
        created = [record for frame in frames for record in frame["detections"]
                   if record.get("created_track")]
        self.assertEqual(len(created), 1, "the frame that minted the identity says so")
        uuid = created[0]["created_track"]
        self.assertTrue(manager.diagnostics.for_track(uuid))

    def test_disabled_by_default_so_the_frame_path_pays_nothing(self):
        manager = TrackManager(commissioned_config())
        self.assertFalse(manager.diagnostics.enabled)
        manager.update(dset([det(1, cx=0.5, score=0.9)]), 1_000_000_000)
        self.assertEqual(manager.diagnostics.recent(limit=0), [])

    def test_refused_duplicate_creations_are_explained_not_silent(self):
        tracking = TrackingConfig(diagnostics_enabled=True, diagnostics_capacity=64)
        manager = TrackManager(commissioned_config(tracking=tracking))
        for index in range(6):
            manager.update(dset([det(1, cx=0.50, score=0.9), det(2, cx=0.51, score=0.85)],
                                frame_index=index), 1_000_000_000 + index * 60_000_000)
        suppressed = [record for frame in manager.diagnostics.recent(limit=0)
                      for record in frame["detections"] if record.get("suppressed")]
        self.assertTrue(suppressed, "§26's rule: refuse, and say why")
        self.assertIn("duplicate of live identity", suppressed[0]["dedup_reason"])


def _stub_track(uuid: str = "aaa"):
    class _Stub:
        track_uuid = uuid
    return _Stub()


def _tmp_path(prefix: str) -> str:
    handle, path = tempfile.mkstemp(prefix=prefix)
    os.close(handle)
    os.remove(path)                     # persist_on_fault appends a timestamp suffix
    return os.path.join(os.path.dirname(path), prefix)


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
