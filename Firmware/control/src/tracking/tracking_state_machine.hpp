// OpenAutoTurret — tracking state machine (architecture §34) with confidence
// decay (§35).
//
// Drives the target-tracking lifecycle from the freshness of valid camera
// measurements:
//
//   READY_HOLD -> (target acquired) -> TRACKING -> (measurement missing) ->
//   COASTING -> (reacquired) -> TRACKING
//   COASTING -> (timeout) -> BRAKE_TO_HOLD -> TARGET_LOST ->
//       +-- search enabled ----> SEARCH
//       +-- otherwise ----------> READY_HOLD
//
// All timing thresholds are configuration. "Do not continue extrapolating an
// old target indefinitely" (§34): as vision goes stale the confidence decays
// toward 0 (the reference manager scales speed by it, §35) and the state
// eventually forces a controlled stop.
//
// Pure logic — no CAN, no camera, no motor driver.
#pragma once

#include <algorithm>
#include <cstdint>

#include "common/types.hpp"

namespace ota {
namespace tracking {

enum class TrackState : uint8_t {
  ReadyHold,    // no target; holding the safe ready pose
  Tracking,     // target acquired; tracking (fresh or predicted to a small horizon)
  Coasting,     // measurement temporarily missing; continuing the prediction
  BrakeToHold,  // longer dropout; progressively braking toward a safe hold
  TargetLost,   // extended dropout; controlled stop (transient -> Search/ReadyHold)
  Search,       // target lost and search is enabled
};

inline const char* track_state_name(TrackState s) {
  switch (s) {
    case TrackState::ReadyHold:    return "ready_hold";
    case TrackState::Tracking:     return "tracking";
    case TrackState::Coasting:     return "coasting";
    case TrackState::BrakeToHold:  return "brake_to_hold";
    case TrackState::TargetLost:   return "target_lost";
    case TrackState::Search:       return "search";
  }
  return "?";
}

struct TrackingStateMachineConfig {
  // Time since the last VALID measurement that still counts as "coasting"
  // (prediction, full confidence-ish).
  int64_t coast_max_ns = 200 * 1000 * 1000;      // 200 ms
  // Time since the last VALID measurement before the target is declared lost.
  int64_t lost_ns = 1000 * 1000 * 1000;          // 1 s
  // When the target is lost, enter SEARCH (if true) else READY_HOLD.
  bool search_enabled = false;
};

class TrackingStateMachine {
 public:
  TrackingStateMachine() = default;
  explicit TrackingStateMachine(TrackingStateMachineConfig cfg)
      : cfg_(cfg) {}

  // Feed one control cycle. `now_ns` is the current monotonic time;
  // `has_valid` is true if a valid target measurement is available this cycle
  // (fresh or, while coasting, the last one is still within the coast window).
  // Returns the resulting state.
  TrackState update(TimeNs now_ns, bool has_valid) {
    now_cache_ns_ = now_ns;
    if (has_valid) {
      last_valid_ns_ = now_ns;
      has_seen_valid_ = true;
      state_ = TrackState::Tracking;
      return state_;
    }
    if (!has_seen_valid_) {
      state_ = TrackState::ReadyHold;  // never saw a target
      return state_;
    }
    if (state_ == TrackState::TargetLost) {
      // One-cycle transient: hand off the lost target to search or ready-hold.
      state_ = cfg_.search_enabled ? TrackState::Search
                                   : TrackState::ReadyHold;
      return state_;
    }
    if (state_ == TrackState::Search || state_ == TrackState::ReadyHold) {
      // Already handed off; stay until a target is reacquired.
      return state_;
    }
    const TimeNs t_since = now_ns - last_valid_ns_;
    if (t_since <= cfg_.coast_max_ns)
      state_ = TrackState::Coasting;
    else if (t_since <= cfg_.lost_ns)
      state_ = TrackState::BrakeToHold;
    else
      state_ = TrackState::TargetLost;
    return state_;
  }

  TrackState state() const { return state_; }

  // §35 confidence in the current target: 1.0 when fresh, decaying linearly to
  // 0.0 at the lost threshold. Used to scale tracking speed.
  double confidence() const {
    if (!has_seen_valid_) return 0.0;
    if (state_ == TrackState::ReadyHold || state_ == TrackState::Search)
      return 0.0;
    const TimeNs t_since = now_cache_ns_ - last_valid_ns_;
    const double frac = 1.0 - static_cast<double>(t_since) /
                                static_cast<double>(std::max<int64_t>(1, cfg_.lost_ns));
    return std::max(0.0, std::min(1.0, frac));
  }

  void reset() {
    state_ = TrackState::ReadyHold;
    last_valid_ns_ = 0;
    now_cache_ns_ = 0;
    has_seen_valid_ = false;
  }

  TimeNs last_valid_ns() const { return last_valid_ns_; }

 private:
  TrackingStateMachineConfig cfg_;
  TrackState state_ = TrackState::ReadyHold;
  TimeNs last_valid_ns_ = 0;
  TimeNs now_cache_ns_ = 0;  // set via update() (for confidence)
  bool has_seen_valid_ = false;
};

}  // namespace tracking
}  // namespace ota
