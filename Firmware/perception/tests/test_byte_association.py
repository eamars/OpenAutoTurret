"""§20 / Appendix C — who may create, who may rescue, and what §41 must be able to read.

The two-stage pass is where ByteTrack's asymmetry lives, so the tests are phrased as
permissions rather than as geometry: *which* detection is allowed to make *which* kind of
change to an identity. A test that only checked "the box moved" would pass with an
implementation that let a 0.2-confidence row mint a person.
"""
from __future__ import annotations

import unittest

from perception.config import AssociationGates, AssociationWeights, TrackingConfig
from perception.detection.types import PointNorm
from perception.tests.support import (at, commissioned_config, commissioned_thresholds,
                                      det, ms, track_at)
from perception.tracking.track import TrackState
from perception.tracking.byte_association import (candidate_notes, split_by_score,
                                                  two_stage_associate)


class TestScoreSplit(unittest.TestCase):
    def test_bands_partition_the_frame(self):
        thresholds = commissioned_thresholds()
        split = split_by_score([det(1, score=0.9), det(2, score=0.4), det(3, score=0.05)],
                               thresholds)
        self.assertEqual([d.detection_id_in_frame for d in split.high], [1])
        self.assertEqual([d.detection_id_in_frame for d in split.low], [2])
        self.assertEqual([d.detection_id_in_frame for d in split.ignored], [3])

    def test_uncommissioned_thresholds_ignore_everything(self):
        # §50. Treating a missing threshold as zero would make every detection "high" and
        # turn the tracker into an identity machine that believes it is configured.
        thresholds = commissioned_thresholds(low_association=None, confirmed_update=None)
        split = split_by_score([det(1, score=0.99)], thresholds)
        self.assertEqual(split.counts(), {"high": 0, "low": 0, "ignored": 1})


class TestPassPermissions(unittest.TestCase):
    def setUp(self):
        self.config = commissioned_config()
        self.thresholds = self.config.active_model.thresholds
        self.sensor = at(1)

    def _associate(self, tracks, detections, dt_s=0.06):
        return two_stage_associate(tracks, detections, dt_s=dt_s, cfg=self.config.tracking,
                                   thresholds=self.thresholds,
                                   sensor_timestamp_ns=self.sensor)

    def test_high_score_updates_a_confirmed_identity(self):
        tracks = [track_at(0.5)]
        outcome = self._associate(tracks, [det(1, cx=0.52, score=0.9)])
        self.assertEqual([(ti, did) for ti, did, _q in outcome.high_matches], [(0, 1)])
        self.assertEqual(outcome.reacquired, [])

    def test_low_score_never_creates(self):
        outcome = self._associate([], [det(1, score=0.20)])
        self.assertEqual(outcome.new_track_candidates, [])

    def test_a_mid_score_detection_may_start_a_tentative_identity(self):
        # 0.35 is above new_track (0.30) and below confirmed_update (0.50). If only the high
        # band could create, `new_track` would be dead configuration — §20's third stage is
        # what that threshold exists to gate.
        outcome = self._associate([], [det(1, score=0.35)])
        self.assertEqual(outcome.new_track_candidates, [1])

    def test_low_score_rescues_a_confirmed_identity_that_missed_the_high_pass(self):
        tracks = [track_at(0.5)]
        outcome = self._associate(tracks, [det(1, cx=0.5, score=0.20)])
        self.assertEqual([(ti, did) for ti, did, _q in outcome.low_matches], [(0, 1)])
        self.assertEqual(outcome.high_matches, [])

    def test_low_score_cannot_help_an_unconfirmed_candidate(self):
        # The asymmetry has a second half: rescuing is for identities that §18.1 already
        # vouched for. Otherwise a low-score row keeps a phantom alive indefinitely.
        tracks = [track_at(0.5, state=TrackState.TENTATIVE)]
        outcome = self._associate(tracks, [det(1, cx=0.5, score=0.20)])
        self.assertEqual(outcome.low_matches, [])
        self.assertEqual(outcome.unmatched_tracks, [0])

    def test_matching_a_lost_identity_is_reported_as_a_reacquisition(self):
        tracks = [track_at(0.5, state=TrackState.LOST_REACQUIRABLE,
                           sensor_ns=at(1) - ms(0.5))]
        outcome = self._associate(tracks, [det(1, cx=0.5, score=0.9)])
        self.assertEqual([(ti, did) for ti, did, _q in outcome.reacquired], [(0, 1)])
        self.assertEqual(outcome.high_matches, [])

    def test_a_track_measured_in_pass_one_cannot_be_measured_again_in_pass_two(self):
        tracks = [track_at(0.5, state=TrackState.OCCLUDED)]
        outcome = self._associate(tracks, [det(1, cx=0.5, score=0.9),
                                           det(2, cx=0.5, score=0.2)], dt_s=0.06)
        self.assertEqual([(ti, did) for ti, did, _q in outcome.high_matches], [(0, 1)])
        self.assertEqual(outcome.low_matches, [],
                         "one person may not receive two measurements in one frame")

    def test_gate_rejections_are_attributable_to_a_detection_id(self):
        # §41's contract: the reason must be attached to *which detection* was refused, by
        # its id — a score-split position means nothing once the bands are concatenated.
        tracks = [track_at(0.5)]
        outcome = self._associate(tracks, [det(1, cx=0.5, score=0.9),
                                           det(2, cx=0.5, score=0.9)], dt_s=0.06)
        self.assertEqual(len(outcome.high_matches), 1)
        self.assertEqual(outcome.unmatched_high, [2])
        for (_ti, detection_id, _reason) in outcome.rejected:
            self.assertIn(detection_id, (1, 2))


class TestAssociationCostReporting(unittest.TestCase):
    def test_candidate_notes_name_the_winner_and_the_gated(self):
        config = commissioned_config()
        tracks = [track_at(0.5), track_at(0.5, index=2)]      # identical rival identities
        outcome = two_stage_associate(tracks, [det(1, cx=0.5, score=0.9)], dt_s=0.06,
                                      cfg=config.tracking,
                                      thresholds=config.active_model.thresholds,
                                      sensor_timestamp_ns=at(1))
        notes = candidate_notes(tracks, outcome)
        outcomes = {note.outcome for note in notes[1]}
        self.assertIn("matched", outcomes)
        self.assertIn("considered", outcomes,
                      "§41: the rejected rival must be visible, not merely absent")

    def test_hard_gates_are_reported_by_name(self):
        config = commissioned_config()
        tracking = TrackingConfig(
            gates=AssociationGates(max_speed_norm_s=0.05, max_scale_ratio=1.5),
            weights=AssociationWeights())
        tracks = [track_at(0.5)]
        outcome = two_stage_associate(tracks, [det(1, cx=0.9, score=0.9)], dt_s=0.06,
                                      cfg=tracking,
                                      thresholds=config.active_model.thresholds,
                                      sensor_timestamp_ns=at(1))
        self.assertEqual(outcome.high_matches, [])
        self.assertIn("impossible_displacement", [r for _t, _d, r in outcome.rejected])

    def test_point_norm_prediction_uses_the_tracks_velocity(self):
        track = track_at(0.5, velocity_x=0.5)
        self.assertAlmostEqual(track.predicted_anchor(0.2).x, 0.6, places=9)
        self.assertEqual(track.predicted_anchor(0.0), PointNorm(0.5, 0.5))


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
