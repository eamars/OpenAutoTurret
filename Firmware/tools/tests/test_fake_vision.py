"""Tests for the synthetic-target publisher (tools/fake_vision.py).

A fake-vision tool is the kind of thing that quietly rots into a lie: if its
wire format drifts from §6.1, controld counts every frame as a drop and the
operator concludes "vision is broken"; if its anchor stops matching its bbox
centre, the LOS solver is fed a target that no real detector could produce and
the tracking test proves nothing. Both are pinned here, at the byte level.
"""
from __future__ import annotations

import math
import unittest

from tools.fake_vision import (hold_measurement, invalid_measurement, main,
                               sweep_measurement)
from vision.protocol import TargetMeasurement, _SIZE


def _sweep_at(t: float, **kw) -> TargetMeasurement:
    opts = dict(width=1280, height=720, amplitude=0.25, period_s=8.0,
                class_id=1, confidence=0.9, track_id=1)
    opts.update(kw)
    return sweep_measurement(0, t, **opts)


class WireFormatTest(unittest.TestCase):

    def test_every_mode_encodes_to_the_58_byte_measurement(self):
        for m in (_sweep_at(0.0), hold_measurement(0, anchor_u=640, anchor_v=360,
                                                   class_id=1, confidence=0.9,
                                                   track_id=1),
                  invalid_measurement(0)):
            self.assertEqual(len(m.encode()), _SIZE,
                             "controld counts any other length as a dropped "
                             "datagram, not as a bad target")

    def test_invalid_frame_is_well_formed_not_malformed(self):
        m = invalid_measurement(7)
        self.assertFalse(m.valid)
        self.assertEqual(len(m.encode()), _SIZE)
        back = TargetMeasurement.decode(m.encode())
        self.assertFalse(back.valid)
        self.assertEqual(back.frame_sequence, 7)


class SweepGeometryTest(unittest.TestCase):

    def test_anchor_is_the_bbox_centre(self):
        # §10.1: the anchor defaults to the bbox centre. If a future edit moves
        # one and not the other, the tracker solves LOS to a point no detector
        # would ever report, and the residual looks like a control bug.
        for t in (0.0, 1.0, 2.3, 4.0):
            m = _sweep_at(t)
            cu = (m.bbox_x_min_norm + m.bbox_x_max_norm) / 2.0 * 1280.0
            cv = (m.bbox_y_min_norm + m.bbox_y_max_norm) / 2.0 * 720.0
            self.assertAlmostEqual(cu, m.anchor_u_px, places=2)
            self.assertAlmostEqual(cv, m.anchor_v_px, places=2)

    def test_bbox_never_leaves_the_normalised_frame(self):
        # amplitude 0.5 puts the anchor on the edge; the box must clamp, not
        # report 1.04 of a frame.
        for phase in (0.0, 2.0, 4.0, 6.2):
            m = _sweep_at(phase * 8.0 / (2 * math.pi), amplitude=0.5)
            for v in (m.bbox_x_min_norm, m.bbox_y_min_norm,
                      m.bbox_x_max_norm, m.bbox_y_max_norm):
                self.assertGreaterEqual(v, 0.0)
                self.assertLessEqual(v, 1.0)

    def test_sweep_actually_moves_in_u(self):
        us = [_sweep_at(t * 8.0 / 30.0).anchor_u_px for t in range(30)]
        self.assertGreater(max(us) - min(us), 100.0,
                           "a sweep that does not sweep cannot make yaw move")


class CliTest(unittest.TestCase):

    def test_dry_run_produces_the_requested_number_of_frames(self):
        # 100 Hz for 0.2 s ~ 20 frames; the absolute schedule may deliver one
        # more or fewer, but a factor-of-two error means the pacer is broken.
        rc = main(["sweep", "--dry-run", "--hz", "100", "--seconds", "0.2"])
        self.assertEqual(rc, 0)

    def test_missing_socket_names_the_daemon_not_a_stack_trace(self):
        with self.assertRaises(SystemExit) as cm:
            main(["hold", "--socket", "/nonexistent/ota_vision.sock",
                  "--seconds", "0.01"])
        msg = str(cm.exception)
        self.assertIn("controld", msg)
        self.assertIn("vision ingest", msg)


if __name__ == "__main__":
    unittest.main()
