// OpenAutoTurret v3 — the TrackSet wire layout (§9, §59, §60).
//
// The byte-for-byte mirror of vision/protocol.py's TrackSet encoder. Two languages
// describe one message here, and nothing in either build fails if they drift — the
// Python side will happily encode a layout the C++ side reads as noise, and every
// field after the point of drift becomes a plausible-looking wrong number rather than
// an error. So this header is pinned three ways:
//
//   1. static_assert on every record size, which catches padding and width changes at
//      compile time on the side that changed them;
//   2. a golden byte prefix shared with vision/tests/test_protocol.py, which catches a
//      field-order change that leaves all the sizes intact (the dangerous class:
//      swapping two same-width fields keeps every assert green and swaps semantics);
//   3. a decode/encode round trip, which catches everything else.
//
// The receiver copies into the §9 POD (tracks/track_set.hpp) and never hands out
// pointers into the receive buffer, which lives on the ingest thread's stack.
#ifndef OTA_TRACKS_TRACK_WIRE_HPP
#define OTA_TRACKS_TRACK_WIRE_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "tracks/track_set.hpp"

namespace ota {
namespace tracks {

#pragma pack(push, 1)
struct TrackWire {
  uint64_t uuid_hi = 0;
  uint64_t uuid_lo = 0;
  uint16_t display_index = 0;
  uint16_t class_id = 0;
  char class_name[kClassNameLen] = {0};
  uint8_t state = 0;
  float detector_confidence = 0.0f;
  float track_confidence = 0.0f;
  float bbox[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // x_min y_min x_max y_max, normalized
  float anchor[2] = {0.0f, 0.0f};
  float velocity[2] = {0.0f, 0.0f};
  uint16_t age_frames = 0;
  uint16_t visible_frames = 0;
  uint16_t missing_frames = 0;
};

struct TrackSetWire {
  uint64_t frame_sequence = 0;
  int64_t sensor_timestamp_ns = 0;
  int64_t publish_timestamp_ns = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint16_t count = 0;
  TrackWire tracks[kMaxTracks];
};
#pragma pack(pop)

// 79 = 16 (uuid) + 4 (display/class) + 12 (name) + 1 (state) + 8 (confidences)
//      + 16 (bbox) + 8 (anchor) + 8 (velocity) + 6 (counters)
// 2562 = 34 (header) + 32 * 79
static_assert(sizeof(TrackWire) == 79, "TrackWire layout drifted from §9");
static_assert(sizeof(TrackSetWire) == 2562, "TrackSetWire size drifted from §60");

constexpr std::size_t kTrackSetWireSize = sizeof(TrackSetWire);
// v1's single-target message, kept readable so controld can be updated first (§59).
constexpr std::size_t kLegacyMeasurementSize = 58;

// §8's enum on the wire as an integer. Out-of-range values decode to Tentative rather
// than to whatever the adjacent bytes happen to say: a publisher from the future must
// not be able to hand the selection logic a "confirmed" track it never confirmed.
inline TrackState state_from_wire(uint8_t v) {
  switch (v) {
    case 0: return TrackState::Tentative;
    case 1: return TrackState::Confirmed;
    case 2: return TrackState::Occluded;
    case 3: return TrackState::Lost;
    default: return TrackState::Tentative;
  }
}

// Decode a TrackSet datagram. `size` must be exactly kTrackSetWireSize: SEQPACKET
// preserves boundaries, so a wrong length is a different protocol or a truncated read,
// and in both cases the right answer is to refuse the whole message. Half a TrackSet is
// worse than none — half a TrackSet can contain the track someone is pointing at.
inline bool decode_track_set(const uint8_t* data, std::size_t size, TrackSet& out) {
  if (data == nullptr || size != kTrackSetWireSize) return false;
  TrackSetWire w;
  std::memcpy(&w, data, sizeof(w));
  out = TrackSet{};
  out.frame_sequence = w.frame_sequence;
  out.sensor_timestamp_ns = w.sensor_timestamp_ns;
  out.publish_timestamp_ns = w.publish_timestamp_ns;
  out.width = w.width;
  out.height = w.height;
  const uint32_t n = w.count > kMaxTracks ? kMaxTracks : w.count;
  for (uint32_t i = 0; i < n; ++i) {
    const TrackWire& tw = w.tracks[i];
    Track t;
    t.uuid = TrackUuid{tw.uuid_hi, tw.uuid_lo};
    if (!t.uuid.valid()) continue;  // a zero uuid is padding, not a candidate
    t.display_index = tw.display_index;
    t.class_id = tw.class_id;
    // Always leave room for the terminator: a publisher that fills all 12 bytes is
    // saying "personallerg", not a name that overruns a buffer somewhere else.
    std::memcpy(t.class_name, tw.class_name, kClassNameLen - 1);
    t.class_name[kClassNameLen - 1] = '\0';
    t.state = state_from_wire(tw.state);
    t.detector_confidence = tw.detector_confidence;
    t.track_confidence = tw.track_confidence;
    t.bbox.x_min = tw.bbox[0];
    t.bbox.y_min = tw.bbox[1];
    t.bbox.x_max = tw.bbox[2];
    t.bbox.y_max = tw.bbox[3];
    t.anchor_x = tw.anchor[0];
    t.anchor_y = tw.anchor[1];
    t.velocity_x_norm_s = tw.velocity[0];
    t.velocity_y_norm_s = tw.velocity[1];
    t.age_frames = tw.age_frames;
    t.visible_frames = tw.visible_frames;
    t.missing_frames = tw.missing_frames;
    out.add(t);
  }
  return true;
}

// Encode, for tests and for any tool that has to speak this protocol from C++
// (§81 replay). Tracks past §60's cap are dropped for the same reason the publisher
// drops them: an oversized datagram is refused outright, so growing here would turn a
// crowded scene into blindness instead of a truncated candidate list.
inline std::size_t encode_track_set(const TrackSet& in, uint8_t* out,
                                    std::size_t capacity) {
  if (out == nullptr || capacity < kTrackSetWireSize) return 0;
  TrackSetWire w;
  std::memset(&w, 0, sizeof(w));
  w.frame_sequence = in.frame_sequence;
  w.sensor_timestamp_ns = in.sensor_timestamp_ns;
  w.publish_timestamp_ns = in.publish_timestamp_ns;
  w.width = in.width;
  w.height = in.height;
  const uint16_t n = in.count > kMaxTracks ? kMaxTracks : in.count;
  w.count = n;
  for (uint16_t i = 0; i < n; ++i) {
    const Track& t = in.tracks[i];
    TrackWire& tw = w.tracks[i];
    tw.uuid_hi = t.uuid.hi;
    tw.uuid_lo = t.uuid.lo;
    tw.display_index = t.display_index;
    tw.class_id = t.class_id;
    std::memcpy(tw.class_name, t.class_name, kClassNameLen);
    tw.state = static_cast<uint8_t>(t.state);
    tw.detector_confidence = t.detector_confidence;
    tw.track_confidence = t.track_confidence;
    tw.bbox[0] = t.bbox.x_min;
    tw.bbox[1] = t.bbox.y_min;
    tw.bbox[2] = t.bbox.x_max;
    tw.bbox[3] = t.bbox.y_max;
    tw.anchor[0] = t.anchor_x;
    tw.anchor[1] = t.anchor_y;
    tw.velocity[0] = t.velocity_x_norm_s;
    tw.velocity[1] = t.velocity_y_norm_s;
    tw.age_frames = t.age_frames;
    tw.visible_frames = t.visible_frames;
    tw.missing_frames = t.missing_frames;
  }
  std::memcpy(out, &w, sizeof(w));
  return sizeof(w);
}

}  // namespace tracks
}  // namespace ota

#endif  // OTA_TRACKS_TRACK_WIRE_HPP
