"""§23 — camera-motion compensation: a hint is an offer, and most offers are refused.

The turret is a pan-tilt head, so "the person moved" and "the camera moved" are the same
pixel event. A tracker that cannot tell them apart loses identities on every pan — and worse,
its *prediction* says the whole scene is walking left, which drags the association gate and
the reacquisition margin with it.

Every "declines to compensate" case here is the point of the module. An incorrect shift is
worse than no shift: a wrong compensation displaces every predicted anchor in the same
direction, which splits identities systematically rather than randomly, and the operator sees
a crowd fragment into duplicates during a pan and reassemble when the pan stops.
"""
from __future__ import annotations

import math
import unittest

from perception.config import CameraMotionConfig, CameraMotionProviderName, TrackingConfig
from perception.errors import ConfigError
from perception.tests.support import (at, commissioned_config, det, dset)
from perception.tracking.camera_motion import (MAX_SHIFT_NORM, CameraMotionHint,
                                               ExternalPoseHintProvider, ImageGmcProvider,
                                               NoCameraMotion,
                                               build_camera_motion_provider)
from perception.tracking.track_manager import TrackManager

FOCAL_PX = 1000.0
WIDTH, HEIGHT = 1920, 1080


def hint(yaw=0.0, pitch=0.0, *, timestamp_ns=1_000_000_000, receive_ns=1_000_000_000):
    return CameraMotionHint(timestamp_ns=timestamp_ns, delta_yaw_rad=yaw,
                            delta_pitch_rad=pitch, receive_timestamp_ns=receive_ns)


def make_provider(**overrides) -> ExternalPoseHintProvider:
    cfg = CameraMotionConfig(provider=CameraMotionProviderName.EXTERNAL_POSE_HINT,
                             focal_px=FOCAL_PX)
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return ExternalPoseHintProvider(cfg, stream_width=WIDTH, stream_height=HEIGHT)


class TestNoProvider(unittest.TestCase):
    def test_absent_provider_compensates_nothing(self):
        self.assertIsNone(NoCameraMotion().shift_norm(at(1), at(1)))
        NoCameraMotion().reset()


class TestExternalPoseHint(unittest.TestCase):
    def test_yaw_becomes_a_normalised_image_shift(self):
        provider = make_provider()
        # A camera yaw of -0.0957 rad moves the scene right by 0.05 of the frame width.
        self.assertTrue(provider.push_hint(hint(yaw=-math.atan(0.05 * WIDTH / FOCAL_PX),
                                                 timestamp_ns=at(1), receive_ns=at(1))))
        shift = provider.shift_norm(at(1), at(1))
        self.assertIsNotNone(shift)
        self.assertAlmostEqual(shift.x, 0.05, places=6)
        self.assertAlmostEqual(shift.y, 0.0, places=9)

    def test_the_tangent_is_used_not_the_angle(self):
        provider = make_provider()
        yaw = 0.35                                   # ~20°: past where small-angle lies
        provider.push_hint(hint(yaw=yaw, timestamp_ns=at(1), receive_ns=at(1)))
        shift = provider.shift_norm(at(1), at(1))
        self.assertAlmostEqual(shift.x, -math.tan(yaw) * FOCAL_PX / WIDTH, places=9)

    def test_pending_deltas_accumulate_until_read(self):
        provider = make_provider()
        small = math.atan(0.025 * WIDTH / FOCAL_PX)
        provider.push_hint(hint(yaw=-small, timestamp_ns=at(1), receive_ns=at(1)))
        provider.push_hint(hint(yaw=-small, timestamp_ns=at(1), receive_ns=at(1)))
        self.assertEqual(provider.hints_received, 2)
        shift = provider.shift_norm(at(1), at(1))
        # Accumulated in *angle* space and converted once: two successive yaws are one pan
        # of the summed angle, and focal·tan(sum) is the image displacement of that pan.
        # Adding the two image displacements instead would be wrong by tan's curvature.
        self.assertAlmostEqual(shift.x, 0.05, delta=2e-4,
                               msg="two half-frame pans are one full frame of motion")
        self.assertIsNone(provider.shift_norm(at(1), at(1)),
                          "a consumed hint must not be spent twice")

    def test_a_stale_hint_is_declined_and_counted(self):
        provider = make_provider()
        provider.push_hint(hint(yaw=-0.05, timestamp_ns=at(0), receive_ns=at(0)))
        much_later = at(0) + 400_000_000             # older than hint_max_age_ms
        self.assertIsNone(provider.shift_norm(at(1), much_later))
        self.assertEqual(provider.hints_declined_stale, 1)
        self.assertEqual(provider.hints_received, 1,
                         "the hint was received; it was the compensation that was refused")

    def test_without_a_focal_length_the_hint_cannot_be_interpreted(self):
        # §23.2: a yaw in radians is not an image displacement until a focal length says how
        # many pixels it is worth. Assuming one would bake a calibration constant into code.
        provider = make_provider()
        provider.cfg.focal_px = None
        provider.push_hint(hint(yaw=-0.05, timestamp_ns=at(1), receive_ns=at(1)))
        self.assertIsNone(provider.shift_norm(at(1), at(1)))
        self.assertEqual(provider.hints_declined_no_focal, 1)

    def test_an_implausible_shift_is_refused_rather_than_clamped(self):
        provider = make_provider()
        provider.push_hint(hint(yaw=1.4, timestamp_ns=at(1), receive_ns=at(1)))
        self.assertGreater(math.tan(1.4) * FOCAL_PX / WIDTH, MAX_SHIFT_NORM)
        self.assertIsNone(provider.shift_norm(at(1), at(1)))

    def test_a_malformed_hint_is_not_queued(self):
        provider = make_provider()
        self.assertFalse(provider.push_hint(hint(yaw=float("nan"))))
        self.assertFalse(provider.push_hint(hint(yaw=0.1, timestamp_ns=0)))
        self.assertEqual(provider.hints_received, 0)


class TestFactory(unittest.TestCase):
    def test_none_provider_is_the_default(self):
        built = build_camera_motion_provider(CameraMotionConfig(),
                                             stream_width=WIDTH, stream_height=HEIGHT)
        self.assertIsInstance(built, NoCameraMotion)

    def test_external_pose_hint_builds(self):
        cfg = CameraMotionConfig(provider=CameraMotionProviderName.EXTERNAL_POSE_HINT,
                                 focal_px=FOCAL_PX)
        self.assertIsInstance(build_camera_motion_provider(cfg, stream_width=WIDTH,
                                                           stream_height=HEIGHT),
                              ExternalPoseHintProvider)

    def test_image_gmc_refuses_construction(self):
        # §23.1's optical-flow option is Vision-7 work. A provider that quietly returned
        # None would let the configuration claim a compensation it never applied, so the
        # refusal happens at construction, where the daemon can print it and stop.
        cfg = CameraMotionConfig(provider=CameraMotionProviderName.IMAGE_GMC)
        with self.assertRaises(ConfigError):
            build_camera_motion_provider(cfg, stream_width=WIDTH, stream_height=HEIGHT)
        with self.assertRaises(ConfigError):
            ImageGmcProvider(cfg)


class TestCompensationInsideTracker(unittest.TestCase):
    """The property §23 exists for: a panned camera must not cost an identity."""

    def _scenario(self, use_hint: bool):
        tracking = TrackingConfig()
        # A deliberately tight speed gate, so the difference between "compensated" and
        # "not compensated" is visible in one frame instead of being hidden by a permissive
        # gate that would forgive the pan anyway.
        tracking.gates.max_speed_norm_s = 0.1
        manager = TrackManager(commissioned_config(tracking=tracking))
        provider = make_provider()
        manager.camera_motion = provider

        manager.update(dset([det(1, cx=0.50)], frame_index=0), at(0))
        yaw = -math.atan(0.05 * WIDTH / FOCAL_PX)
        if use_hint:
            provider.push_hint(hint(yaw=yaw, timestamp_ns=at(1), receive_ns=at(1)))
        result = manager.update(dset([det(1, cx=0.55)], frame_index=1), at(1))
        return manager, result

    def test_a_pan_with_a_pose_hint_keeps_the_identity(self):
        manager, result = self._scenario(use_hint=True)
        self.assertEqual(len(result.tracks), 1)
        self.assertAlmostEqual(result.tracks[0].anchor.x, 0.55, places=6)
        self.assertEqual(manager.counters.tracks_created, 1)

    def test_the_same_pan_without_a_hint_splits_the_identity(self):
        # Recorded rather than fixed: this is the failure the module exists to prevent, and
        # a test that only asserted the good path would pass even if compensation were
        # deleted entirely.
        _manager, result = self._scenario(use_hint=False)
        self.assertEqual(len(result.tracks), 2,
                         "uncompensated, the scene-wide shift reads as a new person")

    def test_the_measured_anchor_is_never_modified_by_compensation(self):
        # §36: compensation belongs on the prediction. If it leaked into the anchor, the
        # controller would aim with a number that had been edited by a model of the camera.
        manager, result = self._scenario(use_hint=True)
        track = result.tracks[0]
        self.assertEqual(track.anchor.x, det(1, cx=0.55).measured_anchor.x,
                         "§36: the anchor the controller aims with is the measured one")
        self.assertIsNotNone(manager.last_camera_shift)
        self.assertAlmostEqual(manager.last_camera_shift.x, 0.05, places=6)


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()
