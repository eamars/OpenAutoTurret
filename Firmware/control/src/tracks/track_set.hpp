// OpenAutoTurret v3 — the TrackSet (§9, §59, §60).
//
// v1 sent controld a single "latest TargetMeasurement". v3 sends the whole set of
// candidate tracks, not because the controller needs all of them but because the
// browser, the target selector and the reacquisition logic all do (§59). controld
// stays authoritative for WHICH track is selected; visiond never tells anyone to
// move a motor.
//
// Fixed size and plain data, on purpose (§60): this crosses a process boundary onto
// a SEQPACKET socket and gets read by a real-time-ish thread. A std::vector here
// would mean the receiving side allocates on someone else's deadline, and the
// "compact binary or JSON" choice §60 allows stops being free the moment the
// message owns heap. The budget is kMaxTracks * sizeof(Track) + a small header —
// about 2.7 kB per detector frame, which at camera rate is tens of kB/s and nowhere
// near the bus the CAN link needs.
//
// Names follow §9 literally so the Python publisher, this header and the browser
// overlay all speak one schema; drift between them is the failure mode, and
// Firmware/vision/protocol.py's contract test is what watches it.
#ifndef OTA_TRACKS_TRACK_SET_HPP
#define OTA_TRACKS_TRACK_SET_HPP

#include <cstdint>
#include <cstring>

namespace ota {
namespace tracks {

// §60: "fixed maximum track count ... suggested max_visible_tracks = 32".
constexpr int kMaxTracks = 32;
// Class names are short labels for a human ("person", "vehicle"). Longer names are
// truncated at the boundary rather than growing the record.
constexpr int kClassNameLen = 12;

// §10: `track_uuid` is a 128-bit session-unique identity and the ONLY thing a
// selection command carries. `display_index` is the human label ("Person #2") and
// may be reused once a track is fully retired — which is exactly why the two are
// separate fields, and why a selection by display index would be a bug waiting for
// a retirement: the operator points at "Person #1", a track retires, the index is
// recycled onto a different human being, and the turret starts following someone
// else while the screen still says #1.
struct TrackUuid {
  uint64_t hi = 0;
  uint64_t lo = 0;

  bool valid() const { return (hi | lo) != 0; }
  bool operator==(const TrackUuid& o) const { return hi == o.hi && lo == o.lo; }
  bool operator!=(const TrackUuid& o) const { return !(*this == o); }
};

// §8. Note this is the *detection* state of one track, and a different thing from
// ota::tracking::TrackState (the v1 single-target tracking FSM: ReadyHold /
// Tracking / Coasting / ...), which AUTO_TRACK's controller replaces in V3-4. The
// two live in different namespaces and answer different questions: "how good is
// this detection?" versus "what is the turret doing about the selected one?".
enum class TrackState : uint8_t {
  Tentative,  // new detection, not yet consistent for `confirm_frames`
  Confirmed,  // stable candidate; may be selected
  Occluded,   // detection temporarily missing, prediction still valid
  Lost,       // past the allowed missing interval; retained for reacquisition
};

inline const char* track_state_name(TrackState s) {
  switch (s) {
    case TrackState::Tentative: return "tentative";
    case TrackState::Confirmed: return "confirmed";
    case TrackState::Occluded:  return "occluded";
    case TrackState::Lost:      return "lost";
  }
  return "unknown";
}

// §9 bbox_norm. Normalized (0..1) in image space, §60's bandwidth rule: it makes the
// message independent of whatever resolution the camera happens to be running at
// today, so a 1280x720 detector and a 1920x1080 overlay agree without a conversion
// step that someone has to remember.
struct BBoxNorm {
  float x_min = 0.0f;
  float y_min = 0.0f;
  float x_max = 0.0f;
  float y_max = 0.0f;
};

struct Track {
  TrackUuid uuid;
  uint16_t display_index = 0;  // 1-based, per class (§10)

  uint16_t class_id = 0;
  char class_name[kClassNameLen] = {0};

  TrackState state = TrackState::Tentative;

  float detector_confidence = 0.0f;  // this frame's detector score
  float track_confidence = 0.0f;     // filtered confidence across frames

  BBoxNorm bbox;
  float anchor_x = 0.0f;  // §9 anchor_norm: the point tracking follows
  float anchor_y = 0.0f;
  float velocity_x_norm_s = 0.0f;  // §9 image_velocity_norm_s
  float velocity_y_norm_s = 0.0f;

  uint16_t age_frames = 0;
  uint16_t visible_frames = 0;
  uint16_t missing_frames = 0;
};

// §9 TrackSet. `sensor_timestamp_ns` is the camera's own SensorTimestamp and stays
// mandatory: without it, "how old is this observation?" becomes a guess, and §61
// exists precisely so that question has a measured answer.
struct TrackSet {
  uint64_t frame_sequence = 0;
  int64_t sensor_timestamp_ns = 0;
  int64_t publish_timestamp_ns = 0;  // §61: when visiond let go of it
  uint32_t width = 0;                // the frame these were normalized against
  uint32_t height = 0;
  uint16_t count = 0;
  Track tracks[kMaxTracks];

  // Bounded insert used by both the publisher and the tests. Ignores overflow
  // rather than growing: §60's cap is a contract, not a hint.
  void add(const Track& t) {
    if (count < kMaxTracks) tracks[count++] = t;
  }
};

}  // namespace tracks
}  // namespace ota

#endif  // OTA_TRACKS_TRACK_SET_HPP
