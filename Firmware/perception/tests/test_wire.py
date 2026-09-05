"""The v1 wire message, pinned to controld's decoder (cross-language §6.2).

The format string and field order here are the source of truth for the Python side, mirrored
from ``control/src/tracking/target_measurement.hpp`` (same ``<QQbi`` + 7 floats + ``bQ``). The
single most important assertion is the **size**: 58 bytes. A 57- or 59-byte datagram is refused
by ``decode_track_set``/``TargetMeasurement::decode`` in C++, which means controld would count a
drop and have no target — a broken 60-byte message quietly becomes "turret not tracking" with no
log line explaining why. The size test is what keeps this file honest.
"""
from __future__ import annotations

import struct
import unittest

from perception.protocol.wire import (SocketPublisher, TargetMeasurement,
                                      encode_track_set, measurement_from_selection)


class TestWireSize(unittest.TestCase):
    def test_the_message_is_exactly_58_bytes(self):
        encoded = TargetMeasurement(frame_sequence=1, sensor_timestamp_ns=2, valid=True,
                                    bbox_x_min_norm=0.1, bbox_y_min_norm=0.2,
                                    bbox_x_max_norm=0.4, bbox_y_max_norm=0.8,
                                    anchor_u_px=320.0, anchor_v_px=540.0,
                                    visual_track_id=99).encode()
        self.assertEqual(len(encoded), 58, "controld refuses anything but exactly 58 bytes")

    def test_the_layout_matches_the_documented_format_string(self):
        # struct.calcsize("") must agree with the C++ static_assert(sizeof == 58).
        self.assertEqual(struct.calcsize("<QQbi" + "f" * 7 + "bQ"), 58)
        encoded = TargetMeasurement(frame_sequence=0x0102030405060708,
                                    sensor_timestamp_ns=0x1112131415161718, valid=True,
                                    class_id=0, confidence=0.9, bbox_x_min_norm=0.1,
                                    bbox_y_min_norm=0.2, bbox_x_max_norm=0.4,
                                    bbox_y_max_norm=0.8, anchor_u_px=320.0, anchor_v_px=540.0,
                                    visual_track_id=0xFFFFFFFFFFFFFFFF).encode()
        # First bytes: frame_sequence little-endian, then sensor_timestamp, then valid=1,
        # class_id, then the floats. Spot-check a couple of offsets.
        self.assertEqual(encoded[0:8], (0x0102030405060708).to_bytes(8, "little"))
        self.assertEqual(encoded[16], 1)                       # valid

    def test_decode_round_trips(self):
        message = TargetMeasurement(frame_sequence=7, sensor_timestamp_ns=3_000_000_000,
                                    valid=True, class_id=0, confidence=0.85,
                                    bbox_x_min_norm=0.1, bbox_y_min_norm=0.2,
                                    bbox_x_max_norm=0.4, bbox_y_max_norm=0.8,
                                    anchor_u_px=420.0, anchor_v_px=230.0,
                                    visual_track_id=1234)
        decoded = TargetMeasurement.decode(message.encode())
        self.assertEqual(decoded.frame_sequence, 7)
        self.assertEqual(decoded.sensor_timestamp_ns, 3_000_000_000)
        self.assertTrue(decoded.valid)
        self.assertAlmostEqual(decoded.confidence, 0.85)
        self.assertAlmostEqual(decoded.bbox_x_min_norm, 0.1, places=6)
        self.assertAlmostEqual(decoded.anchor_v_px, 230.0, places=6)
        self.assertEqual(decoded.visual_track_id, 1234)

    def test_a_missing_track_id_is_encoded_as_zero_and_decodes_to_none(self):
        message = TargetMeasurement(frame_sequence=1, sensor_timestamp_ns=2, valid=True,
                                    bbox_x_min_norm=0.1, bbox_y_min_norm=0.2,
                                    bbox_x_max_norm=0.4, bbox_y_max_norm=0.8,
                                    anchor_u_px=1.0, anchor_v_px=2.0, visual_track_id=None)
        self.assertEqual(message.encode()[49], 0, "has_track_id byte must be 0")
        self.assertIsNone(message.decode(message.encode()).visual_track_id)

    def test_a_wrong_size_is_refused_not_truncated(self):
        with self.assertRaises(ValueError):
            TargetMeasurement.decode(b"\x00" * 57)


class TestMeasurementFromSelection(unittest.TestCase):
    def test_a_no_target_selection_is_an_invalid_measurement(self):
        message = measurement_from_selection({"target_state_name": "NO_TARGET", "valid": False},
                                             frame_sequence=1, sensor_timestamp_ns=2,
                                             stream_size=(1280, 720))
        self.assertFalse(message.valid)
        self.assertEqual(message.encode()[16], 0, "valid byte must be 0 for NO_TARGET")

    def test_a_confirmed_selection_becomes_a_valid_measurement(self):
        message = measurement_from_selection(
            {"target_state_name": "CONFIRMED_TARGET", "measurement_valid": True,
             "bbox_norm": [0.1, 0.2, 0.4, 0.8], "anchor_px": [320.0, 540.0],
             "track_uuid": "0123456789abcdef", "confidence": 0.9},
            frame_sequence=3, sensor_timestamp_ns=4, stream_size=(1280, 720))
        self.assertTrue(message.valid)
        self.assertAlmostEqual(message.anchor_u_px, 320.0)
        self.assertEqual(message.visual_track_id, 0x0123456789abcdef)

    def test_an_absent_anchor_falls_back_to_the_bbox_centre_in_pixels(self):
        message = measurement_from_selection(
            {"target_state_name": "CONFIRMED_TARGET", "measurement_valid": True,
             "bbox_norm": [0.1, 0.2, 0.4, 0.8], "confidence": 0.8},
            frame_sequence=1, sensor_timestamp_ns=2, stream_size=(1280, 720))
        self.assertAlmostEqual(message.anchor_u_px, (0.1 + 0.4) / 2 * 1280)
        self.assertAlmostEqual(message.anchor_v_px, (0.2 + 0.8) / 2 * 720)

    def test_an_occluded_selected_target_is_still_a_valid_measurement(self):
        # §34: the selection survives loss; OCCLUDED is a held, present target (the tracker keeps
        # its last-known box). Publishing NO_TARGET on a skip-frame would make the controller forget
        # the person between every other measurement.
        message = measurement_from_selection(
            {"target_state_name": "OCCLUDED", "measurement_valid": False,
             "bbox_norm": [0.1, 0.2, 0.4, 0.8], "confidence": 0.8},
            frame_sequence=1, sensor_timestamp_ns=2, stream_size=(1280, 720))
        self.assertTrue(message.valid,
                        "an OCCLUDED selected target is still a target the controller should hold")
        self.assertAlmostEqual(message.bbox_x_min_norm, 0.1)

    def test_an_unresolved_identity_stops_the_turret(self):
        message = measurement_from_selection(
            {"target_state_name": "AMBIGUOUS", "measurement_valid": False},
            frame_sequence=1, sensor_timestamp_ns=2, stream_size=(1280, 720))
        self.assertFalse(message.valid)


class TestSocketPublisher(unittest.TestCase):
    def test_a_send_with_no_peer_is_reported_false_not_raised(self):
        with SocketPublisher("/tmp/does-not-exist-ota.sock") as publisher:
            self.assertFalse(publisher.send(TargetMeasurement(frame_sequence=1,
                                                              sensor_timestamp_ns=2).encode()))


if __name__ == "__main__":                                     # pragma: no cover
    unittest.main()


class TestTrackSetWire(unittest.TestCase):
    def _tracks(self, n=2, **kw):
        from perception.tests.support import track_at
        from perception.tracking.track import TrackState
        return [track_at(0.5 + 0.1 * i, index=i + 1, **kw) for i in range(n)]

    def test_the_track_set_is_exactly_2562_bytes(self):
        from perception.protocol.wire import encode_track_set, _TRACKSET_SIZE
        payload = encode_track_set(self._tracks(2), frame_sequence=1, sensor_timestamp_ns=2,
                                   publish_timestamp_ns=3, width=1920, height=1080)
        self.assertEqual(len(payload), 2562,
                         "controld refuses any TrackSet length other than 2562 bytes")
        self.assertEqual(_TRACKSET_SIZE, 2562)

    def test_header_carries_geometry_and_the_count(self):
        from perception.protocol.wire import encode_track_set
        import struct
        payload = encode_track_set(self._tracks(2), frame_sequence=11, sensor_timestamp_ns=22,
                                   publish_timestamp_ns=33, width=1920, height=1080)
        (frame_seq, sensor, publish, w, h, count) = struct.unpack("<QQQIIH", payload[:34])
        self.assertEqual((frame_seq, sensor, publish, w, h, count),
                         (11, 22, 33, 1920, 1080, 2))

    def test_a_retired_track_is_dropped_not_encoded(self):
        from perception.tracking.track import TrackState
        import struct
        tracks = [self._tracks(1)[0]]
        tracks[0].state = TrackState.RETIRED
        payload = encode_track_set(tracks, frame_sequence=1, sensor_timestamp_ns=2,
                                   publish_timestamp_ns=3, width=1920, height=1080)
        self.assertEqual(len(payload), 2562)
        count = struct.unpack("<H", payload[32:34])[0]
        self.assertEqual(count, 0, "a RETIRED identity is gone, so it must not be on the wire")
        self.assertNotIn(b"person", payload[34:])

    def test_a_person_track_is_encoded_with_controld_selectable_class_id_1(self):
        # The detector labels "person" as COCO index 0, but controld's SelectionManager only
        # selects class id 1 (allowed_classes{1,...}). On the un-mapped wire the box's class_id
        # byte was 0, so `select_target` was refused with "class id 0 is not selectable by
        # configuration (§14)" — the live blocker. The encoder must translate person -> 1.
        import struct
        tracks = self._tracks(1)
        track = tracks[0]
        track.class_name = "person"
        track.class_id = 0            # what the detector actually emits (COCO index 0)
        payload = encode_track_set(tracks, frame_sequence=1, sensor_timestamp_ns=2,
                                   publish_timestamp_ns=3, width=1920, height=1080)
        # TrackWire: uuid_hi(8) uuid_lo(8) display_index(2) class_id(2) name(12) ...
        class_id = struct.unpack("<H", payload[34 + 16:34 + 18])[0]
        self.assertEqual(class_id, 1,
                         "person must be published as controld's selectable class id (1), "
                         "not the raw COCO index (0)")

    def test_a_person_measurement_is_encoded_with_class_id_1(self):
        message = measurement_from_selection(
            {"target_state_name": "CONFIRMED_TARGET", "measurement_valid": True,
             "bbox_norm": [0.1, 0.2, 0.4, 0.8], "class_id": 0, "class_name": "person",
             "confidence": 0.9, "anchor_px": [320.0, 540.0],
             "track_uuid": "0123456789abcdef"},
            frame_sequence=3, sensor_timestamp_ns=4, stream_size=(1280, 720))
        self.assertEqual(message.class_id, 1,
                         "the v1 measurement must carry controld's person class id, not COCO's 0")
