"""Protocol round-trip tests (camera-free)."""
import unittest

from vision.protocol import (
    CLASS_PERSON,
    PROTOCOL_SIZE,
    TargetMeasurement,
    empty_measurement,
)


class TestProtocol(unittest.TestCase):
    def test_round_trip_full(self):
        m = TargetMeasurement(
            frame_sequence=42,
            sensor_timestamp_ns=1_234_567_890,
            valid=True,
            class_id=CLASS_PERSON,
            confidence=0.87,
            bbox_x_min_norm=0.10,
            bbox_y_min_norm=0.20,
            bbox_x_max_norm=0.30,
            bbox_y_max_norm=0.40,
            anchor_u_px=480.0,
            anchor_v_px=270.0,
            visual_track_id=7,
        )
        decoded = TargetMeasurement.decode(m.encode())
        self.assertEqual(decoded.frame_sequence, 42)
        self.assertEqual(decoded.sensor_timestamp_ns, 1_234_567_890)
        self.assertTrue(decoded.valid)
        self.assertEqual(decoded.class_id, CLASS_PERSON)
        self.assertAlmostEqual(decoded.confidence, 0.87, places=5)
        self.assertAlmostEqual(decoded.bbox_x_min_norm, 0.10, places=5)
        self.assertAlmostEqual(decoded.bbox_y_min_norm, 0.20, places=5)
        self.assertAlmostEqual(decoded.bbox_x_max_norm, 0.30, places=5)
        self.assertAlmostEqual(decoded.bbox_y_max_norm, 0.40, places=5)
        self.assertAlmostEqual(decoded.anchor_u_px, 480.0, places=4)
        self.assertAlmostEqual(decoded.anchor_v_px, 270.0, places=4)
        self.assertEqual(decoded.visual_track_id, 7)

    def test_round_trip_no_track_id(self):
        m = TargetMeasurement(frame_sequence=1, sensor_timestamp_ns=1000, valid=True, visual_track_id=None)
        decoded = TargetMeasurement.decode(m.encode())
        self.assertIsNone(decoded.visual_track_id)

    def test_empty_measurement_is_invalid(self):
        m = empty_measurement()
        self.assertFalse(m.valid)
        decoded = TargetMeasurement.decode(m.encode())
        self.assertFalse(decoded.valid)

    def test_fixed_size(self):
        m = TargetMeasurement(frame_sequence=0, sensor_timestamp_ns=0)
        self.assertEqual(len(m.encode()), PROTOCOL_SIZE)

    def test_decode_rejects_wrong_size(self):
        with self.assertRaises(ValueError):
            TargetMeasurement.decode(b"\x00" * 10)


if __name__ == "__main__":
    unittest.main()
