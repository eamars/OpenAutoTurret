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
//   READY_HOLD (never saw any target, search enabled, after lost_ns) -> SEARCH
//
// The last edge is the one that matters at a station: after boot there is no
// guarantee a target will ever appear, and SEARCH exists precisely to go looking
// for one. If it were reachable only through TARGET_LOST it would require an
// acquisition before it could be used to get an acquisition.
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
  Search,       // search enabled: target lost, OR none has ever appeared (§36)
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
    // The FSM is constructed when tracking is enabled, so the first cycle is the
    // station's "search clock zero". A flag rather than a 0 comparison: test
    // clocks legitimately start at 0, and a sentinel that collides with a valid
    // value is how a grace period silently becomes "expired immediately".
    if (!start_stamped_) {
      start_stamped_ = true;
      start_ns_ = now_ns;
    }
    if (has_valid) {
      last_valid_ns_ = now_ns;
      has_seen_valid_ = true;
      state_ = TrackState::Tracking;
      return state_;
    }
    if (!has_seen_valid_) {
      // No target has EVER been seen. §36: that is the normal way to boot, and
      // the sweep is how a target gets found — so waiting in READY_HOLD for a
      // target to happen to appear is not safety, it is a turret that refuses to
      // do the one job this state exists for. Give the detector the same grace a
      // lost target gets (lost_ns) — long enough that a slow first frame is not
      // answered with a sweep, short enough that "nothing yet" is answered by
      // looking — then roam. With search disabled the hold is still the answer.
      const bool grace_expired = now_ns - start_ns_ >= cfg_.lost_ns;
      state_ = (cfg_.search_enabled && grace_expired) ? TrackState::Search
                                                      : TrackState::ReadyHold;
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

  // §36 runtime opt-in, effective immediately.
  //
  // This is a flag, not a rebuild: it consults the next transition, so nothing
  // about the acquisition history is thrown away (that is what made the command
  // seem like a next-session-only thing). Turning it OFF while sweeping has to end
  // the sweep — an operator watching a turret roam wants the roaming to stop when
  // they say so, and `disable_search` answering "that applies to your next
  // session" would be a command you cannot use for its one real purpose.
  void set_search_enabled(bool enabled) {
    cfg_.search_enabled = enabled;
    if (!enabled && state_ == TrackState::Search) {
      // Hand back to the safe hold. The reference manager then drives to the
      // ready pose through the normal hold path, so there is no second stop
      // behaviour to keep in sync.
      state_ = TrackState::ReadyHold;
    }
  }
  bool search_enabled() const { return cfg_.search_enabled; }

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
    // A reset means "we are not enabled any more": the next enable must earn its
    // own grace window instead of inheriting an expired one and sweeping the
    // instant tracking comes back on.
    start_stamped_ = false;
    start_ns_ = 0;
  }

  TimeNs last_valid_ns() const { return last_valid_ns_; }

 private:
  TrackingStateMachineConfig cfg_;
  TrackState state_ = TrackState::ReadyHold;
  TimeNs last_valid_ns_ = 0;
  TimeNs now_cache_ns_ = 0;  // set via update() (for confidence)
  bool has_seen_valid_ = false;
  // Grace window origin for the never-seen-a-target case (§36 cold-start sweep).
  TimeNs start_ns_ = 0;
  bool start_stamped_ = false;
};

}  // namespace tracking
}  // namespace ota
