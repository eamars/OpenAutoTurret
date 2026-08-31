// OpenAutoTurret — reference manager (architecture §16).
//
// Arbitrates all reference sources by priority and produces the reference the
// safety envelope / trajectory generator act on. The priority order (§16):
//
//   FAULT / safety action  >  HOMING  >  SHUTDOWN  >  TRACKING  >  SEARCH  >
//   HOLD  >  developer request
//
// A lower-priority source can never override a higher-priority safety state.
// The FAULT / HOMING / SHUTDOWN sources are owned by the surrounding
// ControlLoop phases and the SafetySupervisor (they have higher priority and
// are resolved before the reference manager is consulted). This component
// arbitrates the remaining "ready/hold" sources: TRACKING > SEARCH > HOLD
// (developer requests would slot in just above HOLD).
//
// The tracking source is gated by the tracking state machine (§34) and scaled
// by target confidence (§35): a degrading target reduces the allowed speed; a
// lost target yields to hold/search. The target LOS is converted to joints by
// the LosJointSolver (§14).
//
// Pure arbitration — no CAN, no camera, no motor driver.
#pragma once

#include <algorithm>

#include "common/types.hpp"
#include "control/search_planner.hpp"
#include "geometry/los_joint_solver.hpp"
#include "tracking/tracking_state_machine.hpp"

namespace ota {

enum class ReferenceSource : uint8_t {
  None,
  Tracking,  // a target is being tracked (highest "ready" priority)
  Search,    // target lost + search enabled
  Hold,      // no target; hold the safe ready pose
  Developer, // manual/test command (sits just above hold in the full priority)
};

inline const char* reference_source_name(ReferenceSource s) {
  switch (s) {
    case ReferenceSource::None:      return "none";
    case ReferenceSource::Tracking:  return "tracking";
    case ReferenceSource::Search:    return "search";
    case ReferenceSource::Hold:      return "hold";
    case ReferenceSource::Developer: return "developer";
  }
  return "?";
}

// The reference the reference manager produces: desired joints + the speed
// limit to use for this reference (the safety envelope further constrains it).
struct ReferenceRequest {
  double q_yaw_rad = 0.0;
  double q_pitch_rad = 0.0;
  double v_max_rad_s = 0.0;
  ReferenceSource source = ReferenceSource::Hold;
  // True when this reference is a tracking reference (drives the §38.1
  // "no tracking while position validity unknown" gating in the supervisor).
  bool is_tracking_reference = false;
  // The target confidence that produced this reference (0..1; 1 for hold).
  double confidence = 0.0;
};

struct ReferenceManagerInput {
  // Tracking state machine state (from the §34 FSM) + confidence (§35).
  tracking::TrackState track_state = tracking::TrackState::ReadyHold;
  double target_confidence = 0.0;
  // Predicted target LOS (base-frame azimuth/elevation) at the actuation time.
  double predicted_az_rad = 0.0;
  double predicted_el_rad = 0.0;
  // Search planner output (valid when track_state == Search).
  bool in_search = false;
  double search_q_yaw_rad = 0.0;
  double search_q_pitch_rad = 0.0;
  double search_v_max_rad_s = 0.0;
  // Safe hold pose (raw rad) — where to hold when no target.
  double q_yaw_hold_rad = 0.0;
  double q_pitch_hold_rad = 0.0;
  // Configured speed limits.
  double track_v_max_rad_s = 30.0 * kDeg2Rad;
  double hold_v_max_rad_s = 10.0 * kDeg2Rad;
};

class ReferenceManager {
 public:
  explicit ReferenceManager(geo::LosJointSolver solver)
      : solver_(std::move(solver)) {}

  // Arbitrate by priority (TRACKING > SEARCH > HOLD) and produce the reference.
  ReferenceRequest compute(const ReferenceManagerInput& in) const {
    ReferenceRequest req;
    req.confidence = in.target_confidence;

    // 1) TRACKING: a fresh/coasting target. Solve the LOS to joints; scale the
    //    speed by confidence (§35). If the LOS is unreachable, fall through to
    //    hold (never command an impossible pose).
    const bool want_track =
        (in.track_state == tracking::TrackState::Tracking ||
         in.track_state == tracking::TrackState::Coasting);
    if (want_track) {
      double qy, qp;
      if (solver_.solve(in.predicted_az_rad, in.predicted_el_rad, qy, qp)) {
        req.q_yaw_rad = qy;
        req.q_pitch_rad = qp;
        req.source = ReferenceSource::Tracking;
        req.is_tracking_reference = true;
        const double c = std::max(0.0, std::min(1.0, in.target_confidence));
        req.v_max_rad_s = in.track_v_max_rad_s * c;
        return req;
      }
    }

    // 2) SEARCH: target lost and search enabled.
    if (in.in_search && in.track_state == tracking::TrackState::Search) {
      req.q_yaw_rad = in.search_q_yaw_rad;
      req.q_pitch_rad = in.search_q_pitch_rad;
      req.source = ReferenceSource::Search;
      req.v_max_rad_s = in.search_v_max_rad_s;
      req.confidence = 0.0;
      return req;
    }

    // 3) HOLD: no target (or unreachable). Move to / hold the safe pose.
    req.q_yaw_rad = in.q_yaw_hold_rad;
    req.q_pitch_rad = in.q_pitch_hold_rad;
    req.source = ReferenceSource::Hold;
    req.v_max_rad_s = in.hold_v_max_rad_s;
    req.confidence = 0.0;
    return req;
  }

 private:
  geo::LosJointSolver solver_;
};

}  // namespace ota
