// OpenAutoTurret v3 §59/§60 — the TrackSet wire format, pinned against the Python
// publisher.
//
// The first two tests carry a GOLDEN BYTE PREFIX: the same 113 bytes (the 34-byte
// header plus one 79-byte track record) are asserted in
// Firmware/vision/tests/test_protocol.py. Both sides may therefore be changed, but not
// silently — a field-order swap in either language breaks the other's test. That class
// of drift is the dangerous one: swapping two adjacent same-width fields (say
// `display_index` and `class_id`) keeps every size assert green, compiles in both
// languages, and produces tracks whose label is a class number.
#include "tracks/track_wire.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace ota::tracks;

namespace {

constexpr const char* kGoldenPrefixHex =
    "2a00000000000000"  // frame_sequence = 42
    "d202964900000000"  // sensor_timestamp_ns = 1234567890
    "1222964900000000"  // publish_timestamp_ns = 1234575890
    "00050000"          // width = 1280
    "d0020000"          // height = 720
    "0100"              // count = 1
    "0700000000000000"  // uuid.hi = 7
    "0100000000000000"  // uuid.lo = 1
    "0100"              // display_index = 1
    "0100"              // class_id = 1
    "706572736f6e000000000000"  // class_name = "person"
    "01"                // state = CONFIRMED
    "52b85e3f"          // detector_confidence = 0.87f
    "c3f5683f"          // track_confidence = 0.91f
    "cdcccc3d"          // bbox.x_min = 0.1f
    "cdcc4c3e"          // bbox.y_min = 0.2f
    "9a99993e"          // bbox.x_max = 0.3f
    "0000003f"          // bbox.y_max = 0.5f
    "cdcc4c3e"          // anchor.x = 0.2f
    "3333b33e"          // anchor.y = 0.35f
    "0ad723bc"          // velocity.x = -0.01f
    "00000000"          // velocity.y = 0.0f
    "0900"              // age_frames = 9
    "0800"              // visible_frames = 8
    "0000";             // missing_frames = 0

std::string to_hex(const uint8_t* p, std::size_t n) {
  static const char* kDigits = "0123456789abcdef";
  std::string out;
  out.reserve(n * 2);
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(kDigits[p[i] >> 4]);
    out.push_back(kDigits[p[i] & 0x0F]);
  }
  return out;
}

std::string from_hex(const std::string& hex) {
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  std::string out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2)
    out.push_back(static_cast<char>((nib(hex[i]) << 4) | nib(hex[i + 1])));
  return out;
}

// The same fixture both languages encode.
TrackSet fixture() {
  TrackSet s;
  s.frame_sequence = 42;
  s.sensor_timestamp_ns = 1234567890;
  s.publish_timestamp_ns = 1234575890;
  s.width = 1280;
  s.height = 720;
  Track t;
  t.uuid = TrackUuid{7, 1};
  t.display_index = 1;
  t.class_id = 1;
  std::memcpy(t.class_name, "person", 6);
  t.state = TrackState::Confirmed;
  t.detector_confidence = 0.87f;
  t.track_confidence = 0.91f;
  t.bbox.x_min = 0.1f; t.bbox.y_min = 0.2f; t.bbox.x_max = 0.3f; t.bbox.y_max = 0.5f;
  t.anchor_x = 0.2f; t.anchor_y = 0.35f;
  t.velocity_x_norm_s = -0.01f;
  t.age_frames = 9;
  t.visible_frames = 8;
  s.add(t);
  return s;
}

}  // namespace

TEST(TrackSetWire, SizesMatchTheSpec) {
  EXPECT_EQ(sizeof(TrackWire), 79u);
  EXPECT_EQ(kTrackSetWireSize, 2562u);
  // §59/§60: the two formats share one socket and are told apart by length. If a
  // TrackSet ever came out the same size as a v1 measurement, that dispatch would
  // stop being decidable and the fix would be a type byte — which v1's field
  // compatibility rules ruled out. This assert is the tripwire.
  EXPECT_NE(kTrackSetWireSize, kLegacyMeasurementSize);
}

TEST(TrackSetWire, EncodeMatchesThePythonPublishersGoldenPrefix) {
  uint8_t buf[kTrackSetWireSize];
  const std::size_t n = encode_track_set(fixture(), buf, sizeof(buf));
  ASSERT_EQ(n, kTrackSetWireSize);
  EXPECT_EQ(to_hex(buf, 113), std::string(kGoldenPrefixHex))
      << "byte-for-byte agreement with vision/protocol.py is the whole point of this "
         "test; if the Python side changed intentionally, change its fixture too";
}

TEST(TrackSetWire, DecodesWhatPythonEncoded) {
  const std::string bytes = from_hex(std::string(kGoldenPrefixHex));
  ASSERT_EQ(bytes.size(), 113u);
  // A prefix is not a message: the tail is zero padding, which is legal (§60 pads to a
  // constant length), so extend with zeros rather than pretend the fixture is whole.
  std::string full = bytes;
  full.resize(kTrackSetWireSize, '\0');
  TrackSet out;
  ASSERT_TRUE(decode_track_set(reinterpret_cast<const uint8_t*>(full.data()),
                               full.size(), out));
  EXPECT_EQ(out.frame_sequence, 42u);
  EXPECT_EQ(out.sensor_timestamp_ns, 1234567890);
  EXPECT_EQ(out.publish_timestamp_ns, 1234575890);
  EXPECT_EQ(out.width, 1280u);
  EXPECT_EQ(out.height, 720u);
  ASSERT_EQ(out.count, 1);
  const Track& t = out.tracks[0];
  EXPECT_EQ(t.uuid, (TrackUuid{7, 1}));
  EXPECT_EQ(t.display_index, 1);
  EXPECT_EQ(t.class_id, 1);
  EXPECT_STREQ(t.class_name, "person");
  EXPECT_EQ(t.state, TrackState::Confirmed);
  EXPECT_FLOAT_EQ(t.detector_confidence, 0.87f);
  EXPECT_FLOAT_EQ(t.track_confidence, 0.91f);
  EXPECT_FLOAT_EQ(t.bbox.y_max, 0.5f);
  EXPECT_FLOAT_EQ(t.anchor_x, 0.2f);
  EXPECT_FLOAT_EQ(t.velocity_x_norm_s, -0.01f);
  EXPECT_EQ(t.age_frames, 9);
  EXPECT_EQ(t.visible_frames, 8);
}

TEST(TrackSetWire, RefusesAnythingThatIsNotExactlyOneMessage) {
  uint8_t buf[kTrackSetWireSize];
  const std::size_t n = encode_track_set(fixture(), buf, sizeof(buf));
  ASSERT_EQ(n, kTrackSetWireSize);
  TrackSet out;
  EXPECT_FALSE(decode_track_set(nullptr, n, out));
  EXPECT_FALSE(decode_track_set(buf, n - 1, out));
  EXPECT_FALSE(decode_track_set(buf, n + 1, out));
  // Half a TrackSet is worse than none. A truncated read can drop exactly the record
  // somebody is pointing at, and the receiving side cannot tell that from "the scene
  // is empty". §8's LOST ladder then retires a live identity.
  EXPECT_FALSE(decode_track_set(buf, kLegacyMeasurementSize, out));
}

TEST(TrackSetWire, UnknownStateDecodesToTentativeNotToWhateverCameIn) {
  // A publisher from a newer build must not be able to hand the selection logic a
  // track it never confirmed. §8's rules are safety-relevant: CONFIRMED means
  // "you may point the turret at this".
  uint8_t buf[kTrackSetWireSize];
  TrackSet s = fixture();
  ASSERT_EQ(encode_track_set(s, buf, sizeof(buf)), kTrackSetWireSize);
  TrackSetWire& w = *reinterpret_cast<TrackSetWire*>(buf);
  w.tracks[0].state = 99;
  TrackSet out;
  ASSERT_TRUE(decode_track_set(buf, kTrackSetWireSize, out));
  ASSERT_EQ(out.count, 1);
  EXPECT_EQ(out.tracks[0].state, TrackState::Tentative);
}

TEST(TrackSetWire, ClassNameIsAlwaysTerminated) {
  uint8_t buf[kTrackSetWireSize];
  TrackSet s = fixture();
  std::memcpy(s.tracks[0].class_name, "personallerg", 12);  // fills all 12 bytes
  ASSERT_EQ(encode_track_set(s, buf, sizeof(buf)), kTrackSetWireSize);
  TrackSet out;
  ASSERT_TRUE(decode_track_set(buf, kTrackSetWireSize, out));
  ASSERT_EQ(out.count, 1);
  // 11 characters plus the terminator: the terminator is reserved, which is the
  // whole reason the name cannot consume all 12 wire bytes.
  EXPECT_STREQ(out.tracks[0].class_name, "personaller")
      << "a full-width name must truncate, not overrun into the next field";
}

TEST(TrackSetWire, EmptyUuidsArePaddingAndNotCandidates) {
  uint8_t buf[kTrackSetWireSize];
  TrackSet s = fixture();
  s.add(Track{});  // a zero-uuid record: the shape the zero tail decodes to
  ASSERT_EQ(encode_track_set(s, buf, sizeof(buf)), kTrackSetWireSize);
  TrackSet out;
  ASSERT_TRUE(decode_track_set(buf, kTrackSetWireSize, out));
  EXPECT_EQ(out.count, 1) << "padding must not become a selectable ghost";
}

TEST(TrackSetWire, RoundTripsAFullTable) {
  TrackSet in;
  in.frame_sequence = 1000;
  in.sensor_timestamp_ns = 5'000'000;
  for (int i = 0; i < kMaxTracks; ++i) {
    Track t;
    t.uuid = TrackUuid{0xabcdef00ULL, static_cast<uint64_t>(i) + 1};
    t.display_index = static_cast<uint16_t>(i + 1);
    t.class_id = 1;
    std::memcpy(t.class_name, "person", 6);
    t.state = TrackState::Confirmed;
    t.anchor_x = static_cast<float>(i) / 100.0f;
    in.add(t);
  }
  ASSERT_EQ(in.count, kMaxTracks);
  uint8_t buf[kTrackSetWireSize];
  ASSERT_EQ(encode_track_set(in, buf, sizeof(buf)), kTrackSetWireSize);
  TrackSet out;
  ASSERT_TRUE(decode_track_set(buf, kTrackSetWireSize, out));
  ASSERT_EQ(out.count, kMaxTracks);
  EXPECT_EQ(out.tracks[31].uuid, (TrackUuid{0xabcdef00ULL, 32}));
  EXPECT_FLOAT_EQ(out.tracks[31].anchor_x, 0.31f);
}

TEST(TrackSetWire, ArrivalDoesNotReassociateAnything) {
  // The identity in the message IS the identity. controld does not run a second
  // association over what visiond already formed: §10's reuse rules and §22's
  // retirement would then have two owners, and they disagree first time two targets
  // cross. This test exists so that adding a second association has to delete it on
  // purpose, with the tradeoff written down, rather than by accretion.
  uint8_t buf[kTrackSetWireSize];
  TrackSet sent = fixture();
  sent.tracks[0].uuid = TrackUuid{0x1234, 0x5678};
  sent.tracks[0].state = TrackState::Occluded;
  ASSERT_EQ(encode_track_set(sent, buf, sizeof(buf)), kTrackSetWireSize);
  TrackSet arrived;
  ASSERT_TRUE(decode_track_set(buf, kTrackSetWireSize, arrived));
  ASSERT_EQ(arrived.count, 1);
  EXPECT_EQ(arrived.tracks[0].uuid, (TrackUuid{0x1234, 0x5678}));
  EXPECT_EQ(arrived.tracks[0].state, TrackState::Occluded)
      << "state is visiond's judgement (§8) and arrives as data; controld must not "
         "silently promote or demote it on the way in";
}
