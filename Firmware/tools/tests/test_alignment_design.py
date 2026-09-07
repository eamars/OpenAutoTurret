"""Contracts for the offline proposal model, not firmware integration tests."""
import copy
import json
import math
from pathlib import Path
import unittest

from tools import probe_alignment_design as model


FIRMWARE = Path(__file__).resolve().parents[2]


class AlignmentDesignTest(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((FIRMWARE / 'tools/fixtures/alignment_design.json').read_text())
        self.intrinsics = model.load_intrinsics(FIRMWARE / 'calibration/camera_intrinsics.yaml')

    def test_mount_sign_and_pixel_scale(self):
        origin, direction = model.laser_geometry(self.config)
        self.assertEqual(origin, (-0.075, 0.075, 0))
        u, v = model.project(model.beam_point(origin, direction, 10), self.intrinsics)
        self.assertAlmostEqual(u, 949.5825)
        self.assertAlmostEqual(v, 551.0025)

    def test_depth_mismatch_has_expected_physical_error(self):
        rows = model.distance_sweep(self.config, self.intrinsics)
        for row in rows:
            expected = math.hypot(75, 75) * abs(row['true_depth_m']/10 - 1)
            self.assertAlmostEqual(row['miss_at_configured_sight_mm'], expected)

    def test_disabled_mode_preserves_optical_axis(self):
        self.config['alignment']['mode'] = 'off'
        self.assertEqual(model.configured_sight(self.config), (0, 0, 1))
        report = model.run(self.config, FIRMWARE)
        self.assertIsNone(report['configured_example']['laser_reticle_px'])
        self.assertFalse(report['configured_example']['range_measured'])

    def test_fraction_changes_measurement_without_mutating_anchor(self):
        anchor = [0.5, 0.46]
        point, source = model.aim_point((0.4, 0.1, 0.6, 0.9), anchor,
                                        self.config['tracking']['aim_point'])
        self.assertAlmostEqual(point[1], 0.276)
        self.assertEqual(source, 'box_fraction')
        self.assertEqual(anchor, [0.5, 0.46])
        self.config['tracking']['aim_point']['mode'] = 'perception_anchor'
        point, source = model.aim_point((0.4, 0.1, 0.6, 0.9), anchor,
                                        self.config['tracking']['aim_point'])
        self.assertEqual(point, tuple(anchor))
        self.assertEqual(source, 'perception_anchor')

    def test_invalid_box_has_explicit_anchor_fallback(self):
        point, source = model.aim_point((0.8, 0.1, 0.2, 0.9), (0.5, 0.46),
                                        self.config['tracking']['aim_point'])
        self.assertEqual(point, (0.5, 0.46))
        self.assertEqual(source, 'invalid_box_anchor_fallback')
        with self.assertRaises(ValueError):
            model.aim_point((), (math.nan, 0.5), self.config['tracking']['aim_point'])

    def test_rejects_invalid_tuning_and_unknown_sensor_mode(self):
        mutations = [
            (('alignment', 'assumed_depth_m'), 0),
            (('alignment', 'assumed_depth_m'), math.nan),
            (('alignment', 'assumed_depth_m'), True),
            (('alignment', 'mode'), 'measured'),
            (('alignment', 'camera_from_laser_mm', 'right'), math.inf),
            (('alignment', 'laser_axis_deg', 'up'), 90),
            (('tracking', 'aim_point', 'y_fraction'), 1.1),
            (('tracking', 'aim_point', 'mode'), 'typo'),
        ]
        for path, value in mutations:
            config = copy.deepcopy(self.config)
            owner = config
            for key in path[:-1]:
                owner = owner[key]
            owner[path[-1]] = value
            with self.subTest(path=path, value=value), self.assertRaises(ValueError):
                model.validate_config(config)

    def test_no_projection_for_behind_camera_or_laser(self):
        with self.assertRaises(ValueError):
            model.project((0, 0, -1), self.intrinsics)
        with self.assertRaises(ValueError):
            model.beam_point((0, 0, 5), (0, 0, 1), 2)

    def test_positive_laser_angle_projects_right_and_up(self):
        self.config['alignment']['camera_from_laser_mm'] = dict(right=0, up=0, forward=0)
        self.config['alignment']['laser_axis_deg'] = dict(right=1, up=2)
        origin, direction = model.laser_geometry(self.config)
        near = model.project(model.beam_point(origin, direction, 2), self.intrinsics)
        far = model.project(model.beam_point(origin, direction, 100), self.intrinsics)
        self.assertGreater(near[0], self.intrinsics['cx'])
        self.assertLess(near[1], self.intrinsics['cy'])
        self.assertAlmostEqual(near[0], far[0])
        self.assertAlmostEqual(near[1], far[1])

    def test_reference_solver_rejects_unreachable_direction(self):
        extrinsics = model.load_rotation(FIRMWARE / 'calibration/camera_extrinsics.yaml')
        with self.assertRaises(ValueError):
            model.solve_direction((0, 0, -1), (0, 0, 1), extrinsics, (1, -0.5))


if __name__ == '__main__':
    unittest.main()
