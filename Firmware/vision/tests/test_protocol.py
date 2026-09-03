"""Protocol round-trip tests (camera-free)."""
import unittest

from vision.protocol import (
    BBoxNorm,
    TRACK_SET_SIZE,
    Track,
    TrackSet,
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


# --- v3 TrackSet wire (§9/§59/§60) ----------------------------------------

# The same bytes are asserted in control/tests/test_track_wire.cpp. Both languages may
# change the layout, but not silently: a swap of two adjacent same-width fields (say
# display_index and class_id) leaves every size assert green, compiles in both
# languages, and produces tracks whose label is a class number.
GOLDEN_PREFIX_HEX = (
    "2a00000000000000"
    "d202964900000000"
    "1222964900000000"
    "00050000"
    "d0020000"
    "0100"
    "0700000000000000"
    "0100000000000000"
    "0100"
    "0100"
    "706572736f6e000000000000"
    "01"
    "52b85e3f"
    "c3f5683f"
    "cdcccc3d"
    "cdcc4c3e"
    "9a99993e"
    "0000003f"
    "cdcc4c3e"
    "3333b33e"
    "0ad723bc"
    "00000000"
    "0900"
    "0800"
    "0000"
)


def _golden_track_set() -> TrackSet:
    return TrackSet(
        frame_sequence=42,
        sensor_timestamp_ns=1_234_567_890,
        publish_timestamp_ns=1_234_575_890,
        width=1280,
        height=720,
        tracks=[Track(
            track_uuid=(7, 1),
            display_index=1,
            class_id=1,
            class_name="person",
            state=1,  # CONFIRMED
            detector_confidence=0.87,
            track_confidence=0.91,
            bbox=BBoxNorm(0.1, 0.2, 0.3, 0.5),
            anchor_x=0.2,
            anchor_y=0.35,
            velocity_x_norm_s=-0.01,
            age_frames=9,
            visible_frames=8,
        )],
    )


def test_track_set_is_a_constant_length():
    """§60: fixed maximum track count, normalized coordinates. The receiver must never
    have to negotiate a length or allocate on a deadline."""
    assert TRACK_SET_SIZE == 34 + 79 * 32
    assert len(_golden_track_set().encode()) == TRACK_SET_SIZE
    empty = TrackSet().encode()
    assert len(empty) == TRACK_SET_SIZE, "an empty scene must not shrink the message"


def test_track_set_wire_matches_the_cpp_decoder():
    encoded = _golden_track_set().encode()
    assert encoded[:113].hex() == GOLDEN_PREFIX_HEX, (
        "34-byte header + one 79-byte record must agree with "
        "control/tests/test_track_wire.cpp byte for byte")


def test_track_set_round_trips_every_field():
    back = TrackSet.decode(_golden_track_set().encode())
    assert back.frame_sequence == 42
    assert back.sensor_timestamp_ns == 1_234_567_890
    assert back.publish_timestamp_ns == 1_234_575_890
    assert (back.width, back.height) == (1280, 720)
    assert len(back.tracks) == 1
    t = back.tracks[0]
    assert t.track_uuid == (7, 1)
    assert (t.display_index, t.class_id) == (1, 1)
    assert t.class_name == "person"
    assert t.state == 1
    assert abs(t.detector_confidence - 0.87) < 1e-6
    assert abs(t.bbox.x_max - 0.3) < 1e-6
    assert abs(t.anchor_y - 0.35) < 1e-6
    assert abs(t.velocity_x_norm_s + 0.01) < 1e-6
    assert (t.age_frames, t.visible_frames, t.missing_frames) == (9, 8, 0)


def test_two_messages_share_one_socket_and_are_told_apart_by_length():
    """§59's compatibility rule, which is what lets controld be upgraded before
    visiond. If the two ever came out the same length the rule would stop being
    decidable, and the fix would be a type byte — which v1 compatibility ruled out."""
    assert PROTOCOL_SIZE != TRACK_SET_SIZE
    assert len(TargetMeasurement().encode()) == PROTOCOL_SIZE


def test_tracks_past_the_cap_are_dropped_not_packed():
    """An oversized datagram is refused outright by both receivers, so growing here
    would turn a crowded scene into total blindness instead of a truncated list."""
    ts = _golden_track_set()
    ts.tracks = [Track(track_uuid=(0, i + 1), class_id=1) for i in range(64)]
    back = TrackSet.decode(ts.encode())
    assert len(back.tracks) == 32


def test_decode_refuses_any_other_length():
    good = _golden_track_set().encode()
    for bad in (good[:-1], good + b"\0", good[:113]):
        try:
            TrackSet.decode(bad)
        except ValueError:
            continue
        raise AssertionError("half a TrackSet must be refused, not parsed")
