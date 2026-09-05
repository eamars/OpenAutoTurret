"""§21 — the assignment solver and the hard gates.

The solver is tested against a brute-force optimum rather than against expected output
of itself: "Hungarian/min-cost assignment" is a claim about optimality, and the only way
to check that claim is to enumerate the alternatives. ``random`` with a fixed seed
generates the matrices because a hand-picked 3x3 example is a demonstration, not a test.
"""
from __future__ import annotations

import itertools
import random
import unittest

from perception.config import TrackingConfig
from perception.detection.types import BBox, PointNorm
from perception.tracking.association import (association_cost, assign, gate_distance,
                                            hard_gate, pose_compatible,
                                            solve_assignment)
from perception.tracking.track import Track, TrackState


def brute_force(cost):
    """Optimal assignment over every injective partial matching.

    A tall matrix must be allowed to leave ROWS unmatched too, not just columns: the
    version of this helper that fixed the row set to the first ``min(rows, cols)`` rows
    reported the solver as wrong on a correct answer, so the reference implementation is
    the part that had to grow.
    """
    rows, cols = len(cost), len(cost[0])
    picks = min(rows, cols)
    best = None
    for row_set in itertools.combinations(range(rows), picks):
        for columns in itertools.permutations(range(cols), picks):
            total = sum(cost[r][c] for r, c in zip(row_set, columns))
            if best is None or total < best - 1e-12:
                best = total
    return best if best is not None else 0.0


def make_track(uuid="a", state=TrackState.CONFIRMED_VISIBLE, cx=0.5, cy=0.5,
               w=0.1, h=0.2, class_id=0, class_name="person", vx=0.0, vy=0.0):
    return Track(track_uuid=uuid, display_index=1, class_id=class_id,
                 class_name=class_name, state=state,
                 bbox=BBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                 anchor=PointNorm(cx, cy), velocity_x=vx, velocity_y=vy,
                 last_measurement_ns=1_000_000_000, measurement_valid=True)


def make_det(cx=0.5, cy=0.5, w=0.1, h=0.2, class_id=0, class_name="person",
             score=0.9, det_id=1):
    return _detection(det_id, class_id, class_name, score,
                      BBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                      PointNorm(cx, cy))


def _detection(det_id, class_id, class_name, score, bbox, anchor):
    from perception.detection.types import Detection
    return Detection(detection_id_in_frame=det_id, class_id=class_id,
                     class_name=class_name, detector_score=score, bbox=bbox,
                     measured_anchor=anchor)


class TestSolver(unittest.TestCase):
    def test_square_matches_brute_force(self):
        rng = random.Random(17)
        for _ in range(60):
            n = rng.randint(1, 5)
            matrix = [[round(rng.random(), 4) for _ in range(n)] for _ in range(n)]
            got = solve_assignment(matrix)
            total = sum(matrix[r][c] for r, c in got)
            self.assertAlmostEqual(total, brute_force(matrix), places=9,
                                   msg=f"matrix {matrix} -> {got}")

    def test_rectangular_matches_brute_force(self):
        # Both shapes, because the solver transposes one of them internally, and a
        # transposed matrix whose dimensions were carried over from the un-transposed one
        # is solved against a truncated view of the problem: optimal on every square and
        # every short-and-wide case, wrong exactly on the tall-and-narrow case a busy
        # scene produces (sixteen tracks, three detections).
        rng = random.Random(23)
        for _ in range(120):
            rows = rng.randint(1, 6)
            cols = rng.randint(1, 6)
            matrix = [[round(rng.random(), 4) for _ in range(cols)] for _ in range(rows)]
            got = solve_assignment(matrix)
            total = sum(matrix[r][c] for r, c in got)
            self.assertEqual(len({r for r, _c in got}), len(got))
            self.assertEqual(len({c for _r, c in got}), len(got))
            self.assertEqual(len(got), min(rows, cols))
            self.assertAlmostEqual(total, brute_force(matrix), places=9,
                                   msg=f"{rows}x{cols} {matrix} -> {got}")

    def test_more_tracks_than_detections_transposes(self):
        # Five tracks, two detections: every detection is claimed, by its best track.
        matrix = [[5.0, 5.0], [5.0, 5.0], [1.0, 5.0], [5.0, 5.0], [5.0, 1.0]]
        got = sorted(solve_assignment(matrix))
        self.assertEqual(got, [(2, 0), (4, 1)])

    def test_deterministic_for_identical_costs(self):
        matrix = [[1.0, 1.0], [1.0, 1.0]]
        first = solve_assignment(matrix)
        for _ in range(20):
            self.assertEqual(solve_assignment(matrix), first)

    def test_ragged_and_non_finite_are_refused(self):
        with self.assertRaises(ValueError):
            solve_assignment([[1.0, 2.0], [3.0]])
        with self.assertRaises(ValueError):
            solve_assignment([[float("nan"), 1.0], [1.0, 1.0]])
        self.assertEqual(solve_assignment([]), [])


class TestGates(unittest.TestCase):
    def setUp(self):
        self.cfg = TrackingConfig()

    def test_class_mismatch_is_gated(self):
        track = make_track(class_id=0, class_name="person")
        det = make_det(class_id=18, class_name="dog")
        self.assertEqual(hard_gate(track, det, 0.06, self.cfg,
                                   sensor_timestamp_ns=1_100_000_000),
                         "class_mismatch")

    def test_impossible_displacement_is_gated(self):
        track = make_track(cx=0.1, cy=0.5)
        det = make_det(cx=0.9, cy=0.5)
        self.assertEqual(hard_gate(track, det, 0.06, self.cfg,
                                   sensor_timestamp_ns=1_100_000_000),
                         "impossible_displacement")

    def test_gate_widens_with_elapsed_time_not_with_frames(self):
        # Same displacement: refused at one frame of elapsed time, allowed after several.
        track = make_track(cx=0.1, cy=0.5, vx=1.0)
        det = make_det(cx=0.35, cy=0.5)
        self.assertIsNotNone(hard_gate(track, det, 0.03, self.cfg,
                                       sensor_timestamp_ns=1_100_000_000))
        self.assertIsNone(hard_gate(track, det, 0.30, self.cfg,
                                    sensor_timestamp_ns=1_100_000_000))

    def test_reacquisition_gate_reaches_further(self):
        track = make_track(state=TrackState.LOST_REACQUIRABLE, cx=0.1, cy=0.5, vx=0.5)
        det = make_det(cx=0.30, cy=0.5)
        dt = 0.25
        tight = gate_distance(track, dt, self.cfg, reacquiring=False)
        wide = gate_distance(track, dt, self.cfg, reacquiring=True)
        self.assertGreater(wide, tight)
        distance = track.predicted_anchor(dt).distance_to(det.measured_anchor)
        self.assertLess(distance, wide)

    def test_impossible_scale_and_aspect(self):
        track = make_track(w=0.1, h=0.2)
        huge = make_det(w=0.6, h=0.9)
        self.assertEqual(hard_gate(track, huge, 0.06, self.cfg,
                                   sensor_timestamp_ns=1_100_000_000),
                         "impossible_scale")
        wide = make_det(w=0.28, h=0.10)
        self.assertEqual(hard_gate(track, wide, 0.06, self.cfg,
                                   sensor_timestamp_ns=1_100_000_000),
                         "impossible_aspect")

    def test_stale_beyond_ttl_is_gated(self):
        track = make_track(state=TrackState.LOST_REACQUIRABLE)
        # 4 s after the last measurement, past the 3000 ms retain_ms of §19.
        self.assertEqual(hard_gate(track, make_det(), 0.06, self.cfg,
                                   sensor_timestamp_ns=5_000_000_000),
                         "stale_beyond_ttl")

    def test_pose_gate_only_applies_when_both_sides_have_skeletons(self):
        self.assertTrue(pose_compatible((), ()))
        from perception.detection.types import Keypoint
        a = [Keypoint(0.1, 0.1, 0.9) for _ in range(17)]
        b = [Keypoint(0.9, 0.9, 0.9) for _ in range(17)]
        self.assertFalse(pose_compatible(a, b))


class TestCost(unittest.TestCase):
    def setUp(self):
        self.cfg = TrackingConfig()

    def test_closer_and_more_overlapping_costs_less(self):
        track = make_track(cx=0.5, cy=0.5)
        near = make_det(cx=0.51, cy=0.5)
        far = make_det(cx=0.62, cy=0.5)
        self.assertLess(association_cost(track, near, 0.06, self.cfg).cost,
                        association_cost(track, far, 0.06, self.cfg).cost)

    def test_quality_is_the_complement_of_the_normalized_cost(self):
        payload = association_cost(make_track(), make_det(), 0.06, self.cfg)
        self.assertAlmostEqual(payload.cost + payload.quality, 1.0, places=9)
        self.assertTrue(0.0 <= payload.quality <= 1.0)

    def test_appearance_term_is_dropped_when_no_descriptor_exists(self):
        cfg = TrackingConfig()
        cfg.weights.appearance = 0.5
        without = association_cost(make_track(), make_det(), 0.06, cfg,
                                   appearance_distance=None)
        scored = association_cost(make_track(), make_det(), 0.06, cfg,
                                  appearance_distance=0.8)
        self.assertNotIn("appearance", without.terms)
        self.assertIn("appearance", scored.terms)
        self.assertGreater(scored.cost, without.cost)


class TestAssign(unittest.TestCase):
    def setUp(self):
        self.cfg = TrackingConfig()
        self.now = 1_000_000_000

    def test_one_detection_between_two_tracks_goes_to_the_better_one(self):
        tracks = [make_track(uuid="far", cx=0.5), make_track(uuid="near", cx=0.31)]
        det = make_det(cx=0.30)
        result = assign(tracks, [det], 0.06, self.cfg, sensor_timestamp_ns=self.now,
                        include_states=(TrackState.CONFIRMED_VISIBLE,))
        self.assertEqual([t for t, _d, _q in result.matches], [1])
        self.assertEqual(result.unmatched_tracks, [0])
        self.assertEqual(result.unmatched_detections, [])

    def test_crossing_tracks_do_not_both_claim_one_detection(self):
        tracks = [make_track(uuid="l", cx=0.30), make_track(uuid="r", cx=0.32)]
        dets = [make_det(cx=0.31, det_id=1)]
        result = assign(tracks, dets, 0.06, self.cfg, sensor_timestamp_ns=self.now,
                        include_states=(TrackState.CONFIRMED_VISIBLE,))
        self.assertEqual(len(result.matches), 1, "one detection cannot be two identities")
        self.assertEqual(len({d for _t, d, _q in result.matches}), len(result.matches))

    def test_gated_cells_are_reported_not_silently_dropped(self):
        tracks = [make_track(uuid="p", class_id=0, cx=0.5)]
        dets = [make_det(class_id=18, class_name="dog", cx=0.5)]
        result = assign(tracks, dets, 0.06, self.cfg, sensor_timestamp_ns=self.now,
                        include_states=(TrackState.CONFIRMED_VISIBLE,))
        self.assertEqual(result.matches, [])
        self.assertIn((0, 0, "class_mismatch"), result.rejected)
        self.assertEqual(result.unmatched_tracks, [0])
        self.assertEqual(result.unmatched_detections, [0])

    def test_tentative_tracks_are_not_offered_when_excluded(self):
        tracks = [make_track(uuid="t", state=TrackState.TENTATIVE)]
        result = assign(tracks, [make_det()], 0.06, self.cfg,
                        sensor_timestamp_ns=self.now,
                        include_states=(TrackState.CONFIRMED_VISIBLE,))
        self.assertEqual(result.matches, [])
        self.assertEqual(result.unmatched_tracks, [])   # never a candidate at all

    def test_cells_carry_costs_for_the_diagnostics_trace(self):
        tracks = [make_track(uuid="a")]
        result = assign(tracks, [make_det()], 0.06, self.cfg,
                        sensor_timestamp_ns=self.now,
                        include_states=(TrackState.CONFIRMED_VISIBLE,))
        self.assertIn((0, 0), result.cells)
        self.assertTrue(result.cells[(0, 0)].terms)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
