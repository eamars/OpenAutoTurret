#pragma once
// Native observation and display list travel in one SEQPACKET datagram.
#include "tracks/track_wire.hpp"
#include <cmath>

namespace ota::tracks {
#pragma pack(push, 1)
struct PerceptionHeader {
  char magic[4];
  uint16_t version;
  uint16_t header_size;
  uint64_t session_hi, session_lo, track_hi, track_lo, generation, track_set_sequence;
  uint8_t state, valid;
  uint16_t flags;
  float bbox[4], anchor[2], detector_score, association_quality, identity_confidence, ambiguity;
};
#pragma pack(pop)
static_assert(sizeof(PerceptionHeader) == 100, "native Python/C++ header mismatch");
constexpr size_t kPerceptionWireSize = sizeof(PerceptionHeader) + kTrackSetWireSize;

inline bool decode_perception_frame(const uint8_t* bytes, size_t size, TrackSet& out) {
  if (!bytes || size != kPerceptionWireSize) return false;
  PerceptionHeader h;
  std::memcpy(&h, bytes, sizeof(h));
  if (std::memcmp(h.magic, "OTP1", 4) || h.version != 1 || h.header_size != sizeof(h) ||
      h.state > 5 || h.valid > 1 || (h.flags & ~1u) || !(h.session_hi || h.session_lo)) return false;
  for (float v : {h.detector_score, h.association_quality, h.identity_confidence, h.ambiguity})
    if (!std::isfinite(v) || v < 0 || v > 1) return false;
  if (h.state == 0 && (h.track_hi || h.track_lo || h.valid)) return false;
  if (h.state != 0 && !(h.track_hi || h.track_lo)) return false;
  if (h.valid && (h.state != 1 || h.ambiguity > 0)) return false;
  if (!decode_track_set(bytes + sizeof(h), kTrackSetWireSize, out)) return false;
  if (out.sensor_timestamp_ns <= 0 || out.publish_timestamp_ns < out.sensor_timestamp_ns ||
      out.width == 0 || out.height == 0) return false;
  auto& obs = out.observation;
  obs.native = true;
  obs.session = {h.session_hi, h.session_lo};
  obs.selected = {h.track_hi, h.track_lo};
  obs.generation = h.generation;
  obs.track_set_sequence = h.track_set_sequence;
  obs.state = h.state;
  obs.valid = h.valid;
  obs.just_reacquired = h.flags & 1;
  obs.ambiguity = h.ambiguity;
  obs.association_quality = h.association_quality;
  obs.identity_confidence = h.identity_confidence;
  if (h.valid) {
    if (!(h.bbox[0] >= 0 && h.bbox[0] < h.bbox[2] && h.bbox[2] <= 1 &&
          h.bbox[1] >= 0 && h.bbox[1] < h.bbox[3] && h.bbox[3] <= 1 &&
          h.anchor[0] >= 0 && h.anchor[0] <= 1 && h.anchor[1] >= 0 && h.anchor[1] <= 1)) return false;
    Track* match = nullptr;
    for (unsigned i = 0; i < out.count; ++i)
      if (out.tracks[i].uuid == obs.selected) {
        if (match) return false;
        match = &out.tracks[i];
      }
    if (!match || match->state != TrackState::Confirmed) return false;
    // The observation is authoritative even if the display projection differs.
    match->bbox = {h.bbox[0], h.bbox[1], h.bbox[2], h.bbox[3]};
    match->anchor_x = h.anchor[0];
    match->anchor_y = h.anchor[1];
    match->detector_confidence = h.detector_score;
  }
  return true;
}
}  // namespace ota::tracks
