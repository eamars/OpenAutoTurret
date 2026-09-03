// OpenAutoTurret v3 — TrackManager (§8, §10, §22, §58).
//
// Turns each detector frame's raw TrackSet into a small set of persistent tracks with
// stable identities. It runs ONCE PER DETECTOR FRAME (§58), on the vision ingest
// thread — never on the 200 Hz control thread. The control-rate side keeps predicting
// the selected target's pose; re-running visual association at 200 Hz would buy
// nothing and would cost the loop the frame budget the estimator actually needs.
//
// Two design decisions worth stating, because both are places a "cleaner"
// implementation goes wrong:
//
//   * Association is anchored on the ANCHOR point, with IoU only as a tiebreaker.
//      IoU-first association looks better on paper and fails the §82 case "abrupt
//      bbox-size change": a person crouches, a detector's box halves, IoU against the
//      remembered box collapses below the gate, the track dies, and a new one is born
//      with a fresh identity one frame later. The operator sees "Person #1" vanish and
//      "Person #3" appear in the same spot, and the turret drops a target it was
//      pointing at. Centre-of-mass motion is the trustworthy signal here.
//   * Matching is MUTUAL-best and greedy over ascending cost. Nearest-neighbour from
//      the detection side alone lets one detection be claimed by two tracks when they
//      cross, which reads as two selection rows pointing at the same human being.
//      Greedy-with-ties-broken-by-index is not optimal assignment; it is deterministic,
//      which is what the crossing test and any replay (§81) actually require.
//
// No allocation: fixed slots, fixed arrays. The ingest thread is not the control
// thread, but it feeds it, and a stall here becomes a stale TrackSet there.
#ifndef OTA_TRACKS_TRACK_MANAGER_HPP
#define OTA_TRACKS_TRACK_MANAGER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "tracks/track_set.hpp"

namespace ota {
namespace tracks {

// §8/§22 thresholds, all in frames (§58: TrackManager runs once per detector frame,
// so a frame is the natural unit — and a timeout stated in seconds would silently
// change meaning every time the inference rate changed).
struct TrackManagerConfig {
    int confirm_frames = 3;     // TENTATIVE -> CONFIRMED (§8: "a configurable number")
    int occlude_frames = 4;     // missing -> OCCLUDED
    int lost_frames = 12;       // missing -> LOST
    int reacquire_frames = 60;  // LOST is held this long, then retired (§22)
    // No association at all beyond this anchor jump in one frame (normalized).
    double max_jump_norm = 0.15;
    // A LOST track may be re-identified within this wider gate (§21's reacquisition
    // reuses the identity; V3-3/V3-4 decide what to do with it).
    double reacquire_gate_norm = 0.25;
    double iou_bonus = 0.35;  // weight of IoU as a tiebreaker, never as the gate
    int max_tracks = kMaxTracks;
};

class TrackManager {
 public:
  using Config = TrackManagerConfig;

  explicit TrackManager(Config cfg = {}, uint64_t session_nonce = 0)
      : cfg_(cfg), session_hi_(session_nonce) {}

  // A track slot. Public because tests and the overlay read fields directly rather
  // than through a dozen accessors; it is still not a handle — the manager owns it.
  struct Slot {
    TrackUuid uuid;
    uint16_t display_index = 0;  // 1-based, per class (§10)
    uint16_t class_id = 0;
    char class_name[kClassNameLen] = {0};
    TrackState state = TrackState::Tentative;
    float detector_confidence = 0.0f;
    float track_confidence = 0.0f;
    BBoxNorm bbox;
    float anchor_x = 0.0f, anchor_y = 0.0f;
    float velocity_x = 0.0f, velocity_y = 0.0f;
    float pred_x = 0.0f, pred_y = 0.0f;
    int age_frames = 0, visible_frames = 0, missing_frames = 0;
    int64_t last_seen_ns = 0;
    bool in_use = false;

    Track as_track() const {
      Track t;
      t.uuid = uuid;
      t.display_index = display_index;
      t.class_id = class_id;
      std::memcpy(t.class_name, class_name, kClassNameLen);
      t.state = state;
      t.detector_confidence = detector_confidence;
      t.track_confidence = track_confidence;
      t.bbox = bbox;
      t.anchor_x = anchor_x;
      t.anchor_y = anchor_y;
      t.velocity_x_norm_s = velocity_x;
      t.velocity_y_norm_s = velocity_y;
      t.age_frames = static_cast<uint16_t>(age_frames);
      t.visible_frames = static_cast<uint16_t>(visible_frames);
      t.missing_frames = static_cast<uint16_t>(missing_frames);
      return t;
    }
  };


  // One detector frame. `now_ns` is host-monotonic receive time (telemetry only,
  // §61); all STATE advances in frames, because §8's thresholds are stated in frames
  // and a state that advanced on wall-clock time would change meaning with the camera
  // rate. Silence is handled separately by stale_ms(): a detector that stops
  // publishing sends no frames, and nothing may silently age out tracks in a system
  // whose safety logic reads "no target".
  void update(const TrackSet& set, int64_t now_ns) {
    ++stats_.frames;
    last_receive_ns_ = now_ns;
    last_publish_ns_ = set.publish_timestamp_ns;
    const double dt = dt_seconds(set.sensor_timestamp_ns);
    last_sensor_ns_ = set.sensor_timestamp_ns;
    const uint16_t n_det =
        std::min<int>(set.count, kMaxTracks);  // clamp a bad publisher

    // 1. Predict every live track to this frame's capture time.
    for (int i = 0; i < kMaxTracks; ++i) {
      Slot& sl = slots_[i];
      if (!sl.in_use) continue;
      sl.pred_x = static_cast<float>(sl.anchor_x + sl.velocity_x * dt);
      sl.pred_y = static_cast<float>(sl.anchor_y + sl.velocity_y * dt);
    }

    // 2. Cost matrix, then greedy mutual-best assignment.
    int matched_det[kMaxTracks];
    int matched_slot[kMaxTracks];
    for (int i = 0; i < kMaxTracks; ++i) {
      matched_det[i] = -1;
      matched_slot[i] = -1;
    }
    Cost cells[kMaxTracks * kMaxTracks];
    int ncell = 0;
    for (int d = 0; d < n_det; ++d) {
      const Track& det = set.tracks[d];
      for (int i = 0; i < kMaxTracks; ++i) {
        const Slot& sl = slots_[i];
        if (!sl.in_use) continue;
        const double gate =
            (sl.state == TrackState::Lost) ? cfg_.reacquire_gate_norm
                                           : cfg_.max_jump_norm;
        const double dist = std::hypot(det.anchor_x - sl.pred_x,
                                       det.anchor_y - sl.pred_y);
        if (dist > gate) continue;  // never teleport a track across the frame
        if (det.class_id != sl.class_id) continue;  // identities do not cross classes
        // Anchor distance is the decision; IoU only breaks near-ties, scaled to the
        // same units so the sum stays meaningful.
        const double iou = overlap(sl.bbox, det.bbox);
        const double cost = dist - cfg_.iou_bonus * iou * gate;
        cells[ncell++] = Cost{static_cast<int>(cost * 1000.0), i, d};
      }
    }
    std::sort(cells, cells + ncell);
    for (int k = 0; k < ncell; ++k) {
      const int i = cells[k].slot, d = cells[k].det;
      if (matched_slot[i] >= 0 || matched_det[d] >= 0) continue;  // already claimed
      matched_slot[i] = d;
      matched_det[d] = i;
    }

    // 3. Matched slots: update in place, identity preserved (§10 UUID stability).
    for (int i = 0; i < kMaxTracks; ++i) {
      if (!slots_[i].in_use || matched_slot[i] < 0) continue;
      adopt(i, set.tracks[matched_slot[i]], now_ns, dt);
    }

    // 4. Unmatched slots: miss accounting and the §8 state ladder.
    for (int i = 0; i < kMaxTracks; ++i) {
      Slot& sl = slots_[i];
      if (!sl.in_use || matched_slot[i] >= 0) continue;
      ++sl.missing_frames;
      ++sl.age_frames;
      if (sl.state == TrackState::Lost) {
        if (sl.missing_frames >= cfg_.reacquire_frames + cfg_.lost_frames)
          retire(i);  // §22: past the reacquisition window, and only now is the
                      // display index free for reuse
        continue;
      }
      if (sl.missing_frames >= cfg_.lost_frames) {
        sl.state = TrackState::Lost;
        ++stats_.lost;
      } else if (sl.missing_frames >= cfg_.occlude_frames) {
        sl.state = TrackState::Occluded;
      } else if (sl.state == TrackState::Tentative &&
                 sl.missing_frames >= 2) {
        // A tentative track that is already blinking is not a target being born, it
        // is noise. Retire early: a TENTATIVE track is not selectable either way
        // (§8), so holding it costs a slot and confuses the overlay.
        retire(i);
      }
    }

    // 5. Unmatched detections: new tentative tracks.
    for (int d = 0; d < n_det; ++d) {
      if (matched_det[d] >= 0) continue;
      create(set.tracks[d], now_ns, dt);
    }
  }

  // --- views ---------------------------------------------------------------
  int track_count() const {
    int n = 0;
    for (int i = 0; i < kMaxTracks; ++i)
      if (slots_[i].in_use) ++n;
    return n;
  }
  // Dense read of the live tracks (index is stable only within one call).
  int copy_tracks(Track* out, int max_out) const {
    int n = 0;
    for (int i = 0; i < kMaxTracks && n < max_out; ++i)
      if (slots_[i].in_use) out[n++] = slots_[i].as_track();
    return n;
  }
  const Slot* find(TrackUuid uuid) const {
    for (int i = 0; i < kMaxTracks; ++i)
      if (slots_[i].in_use && slots_[i].uuid == uuid) return &slots_[i];
    return nullptr;
  }

  // §8: "CONFIRMED ... may be selected". Tentative tracks are not selectable, and an
  // occluded one must not become selectable *by selecting it* — the operator picking
  // a track that the detector is not currently confirming would hand AUTO_TRACK a
  // target it cannot see.
  bool is_selectable(TrackUuid uuid) const {
    const Slot* sl = find(uuid);
    return sl && sl->state == TrackState::Confirmed;
  }
  // The SELECTED track may ride out a short dropout (§20's coasting needs a subject
  // that still exists), which selection may not.
  bool is_trackable(TrackUuid uuid) const {
    const Slot* sl = find(uuid);
    return sl && (sl->state == TrackState::Confirmed ||
                  sl->state == TrackState::Occluded);
  }
  bool exists(TrackUuid uuid) const { return find(uuid) != nullptr; }

  // §61 latency telemetry, from the three stamps the set carries. Negative means
  // "never received", which is a different claim from "0 ms".
  double stale_ms(int64_t now_ns) const {
    if (last_receive_ns_ <= 0 || now_ns < last_receive_ns_) return -1.0;
    return (now_ns - last_receive_ns_) / 1e6;
  }
  double sensor_age_ms(int64_t now_ns) const {
    if (last_sensor_ns_ <= 0 || now_ns < last_sensor_ns_) return -1.0;
    return (now_ns - last_sensor_ns_) / 1e6;
  }
  double publish_to_receive_ms() const {
    if (last_publish_ns_ <= 0 || last_receive_ns_ < last_publish_ns_) return -1.0;
    return (last_receive_ns_ - last_publish_ns_) / 1e6;
  }

  struct Stats {
    uint64_t frames = 0;
    uint64_t created = 0;
    uint64_t retired = 0;
    uint64_t matched = 0;
    uint64_t lost = 0;
    uint64_t reacquired = 0;
  };
  const Stats& stats() const { return stats_; }
  const Config& config() const { return cfg_; }

 private:
  struct Cost {
    int scaled;  // fixed point so std::sort has no NaN ordering trap
    int slot;
    int det;
    bool operator<(const Cost& o) const {
      if (scaled != o.scaled) return scaled < o.scaled;
      if (slot != o.slot) return slot < o.slot;
      return det < o.det;
    }
  };

  static double overlap(const BBoxNorm& a, const BBoxNorm& b) {
    const double ix = std::min<double>(a.x_max, b.x_max) -
                      std::max<double>(a.x_min, b.x_min);
    const double iy = std::min<double>(a.y_max, b.y_max) -
                      std::max<double>(a.y_min, b.y_min);
    if (ix <= 0.0 || iy <= 0.0) return 0.0;
    const double inter = ix * iy;
    const double area_a =
        std::max(0.0, double(a.x_max - a.x_min)) * std::max(0.0, double(a.y_max - a.y_min));
    const double area_b =
        std::max(0.0, double(b.x_max - b.x_min)) * std::max(0.0, double(b.y_max - b.y_min));
    const double uni = area_a + area_b - inter;
    return uni > 0.0 ? inter / uni : 0.0;
  }

  double dt_seconds(int64_t sensor_ns) {
    if (last_sensor_ns_ <= 0 || sensor_ns <= last_sensor_ns_) return 1.0 / 30.0;
    const double dt = (sensor_ns - last_sensor_ns_) / 1e9;
    // A detector that stalls for 4 s must not extrapolate a track across the frame on
    // the next set; one with a jittery clock must not get a negative dt.
    return std::min(dt, 0.25);
  }

  void create(const Track& det, int64_t now_ns, double dt) {
    if (track_count() >= cfg_.max_tracks) {
      // At capacity the tentative with the least evidence gives way — never a
      // confirmed track, which an operator may already have selected and the turret
      // may be pointing at. Sacrificing a selected track to make room for a detection
      // that has not proved itself is the worst possible trade and it is invisible on
      // the screen, so the choice is made in the least-valuable direction explicitly.
      int victim = -1;
      for (int i = 0; i < kMaxTracks; ++i)
        if (slots_[i].in_use && slots_[i].state == TrackState::Tentative &&
            (victim < 0 || slots_[i].visible_frames > slots_[victim].visible_frames))
          victim = i;
      if (victim < 0) return;  // every slot is confirmed: drop this detection
      retire(victim);
    }
    for (int i = 0; i < kMaxTracks; ++i) {
      if (slots_[i].in_use) continue;
      Slot& sl = slots_[i];
      sl = Slot{};
      sl.in_use = true;
      sl.uuid = TrackUuid{session_hi_, ++uuid_counter_};
      sl.class_id = det.class_id;
      std::memcpy(sl.class_name, det.class_name, kClassNameLen);
      sl.display_index = alloc_display_index(det.class_id);
      sl.state = TrackState::Tentative;
      ++stats_.created;
      adopt(i, det, now_ns, dt);
      return;
    }
  }

  // Bring a slot up to date with a detection it has been associated with.
  void adopt(int i, const Track& det, int64_t now_ns, double dt) {
    Slot& sl = slots_[i];
    if (sl.visible_frames > 0 && sl.last_seen_ns > 0) {
      const double vx = (det.anchor_x - sl.anchor_x) / dt;
      const double vy = (det.anchor_y - sl.anchor_y) / dt;
      // A one-frame spike is a detection artefact, not motion. Limiting the update
      // rather than trusting the sample is what keeps a crossing target from being
      // read as a teleport and dropped.
      // Expressed as "no more than a max-jump per 50 ms", which is the same
      // physical statement as the association gate in units of normalized image per
      // second. Heuristic, and labelled as one: the point is that a single wild
      // sample cannot arm the predictor.
      const double cap = cfg_.max_jump_norm / 0.05;
      sl.velocity_x = static_cast<float>(
          std::max(-cap, std::min(cap, 0.5 * sl.velocity_x + 0.5 * vx)));
      sl.velocity_y = static_cast<float>(
          std::max(-cap, std::min(cap, 0.5 * sl.velocity_y + 0.5 * vy)));
    }
    sl.bbox = det.bbox;
    sl.anchor_x = det.anchor_x;
    sl.anchor_y = det.anchor_y;
    sl.detector_confidence = det.detector_confidence;
    // Filtered, not raw: §19 derates motion by confidence, so a single lucky frame
    // must not be able to raise the speed ceiling for one cycle.
    sl.track_confidence = sl.visible_frames == 0
                              ? det.track_confidence
                              : static_cast<float>(0.7 * sl.track_confidence +
                                                   0.3 * det.track_confidence);
    if (det.class_name[0] != '\0')
      std::memcpy(sl.class_name, det.class_name, kClassNameLen);
    ++sl.age_frames;
    ++sl.visible_frames;
    const bool was_missing = sl.missing_frames > 0;
    sl.missing_frames = 0;
    sl.last_seen_ns = now_ns;
    if (sl.state == TrackState::Lost && was_missing) ++stats_.reacquired;
    if (sl.state == TrackState::Occluded || sl.state == TrackState::Lost)
      sl.state = TrackState::Confirmed;  // §21: re-identification keeps the identity
    else if (sl.state == TrackState::Tentative &&
             sl.visible_frames >= cfg_.confirm_frames)
      sl.state = TrackState::Confirmed;  // §8: enough consistent frames to be picked
    ++stats_.matched;
  }

  void retire(int i) {
    if (!slots_[i].in_use) return;
    release_display_index(slots_[i].class_id, slots_[i].display_index);
    slots_[i] = Slot{};
    ++stats_.retired;
  }

  // §10: "display_index can be reused only after a track is fully retired". Lowest
  // free index per class, so the operator sees Person #1 and Person #2 before #3, and
  // a freed number comes back only once its owner is gone for good.
  uint16_t alloc_display_index(uint16_t class_id) {
    for (uint16_t cand = 1; cand <= kMaxTracks; ++cand) {
      bool taken = false;
      for (int i = 0; i < kMaxTracks; ++i)
        if (slots_[i].in_use && slots_[i].class_id == class_id &&
            slots_[i].display_index == cand)
          taken = true;
      if (!taken) return cand;
    }
    return kMaxTracks;
  }
  void release_display_index(uint16_t, uint16_t) {
    // Implicit: an index is "taken" while a live slot holds it, so retiring frees it.
    // Kept as a named operation because the alternative is a reader wondering whether
    // reuse is forgotten somewhere.
  }

  Config cfg_;
  uint64_t session_hi_ = 0;
  uint64_t uuid_counter_ = 0;
  Slot slots_[kMaxTracks];
  Stats stats_;
  int64_t last_sensor_ns_ = 0;
  int64_t last_publish_ns_ = 0;
  int64_t last_receive_ns_ = 0;
};

}  // namespace tracks
}  // namespace ota

#endif  // OTA_TRACKS_TRACK_MANAGER_HPP
