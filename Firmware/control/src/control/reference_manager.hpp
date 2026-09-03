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
#include "control/motion_intent.hpp"
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
  // v3 (§53): the mode sources, named so telemetry can say WHICH mode is moving
  // the turret. v1 had no Manual/Roam distinction because it had no modes;
  // collapsing both into Developer would leave the operator's screen unable to
  // answer "why is it moving" — the first question during a live run.
  Manual,    // manual jog / step / goto
  Roam,      // AUTO_ROAM sweep waypoint
};

inline const char* reference_source_name(ReferenceSource s) {
  switch (s) {
    case ReferenceSource::None:      return "none";
    case ReferenceSource::Tracking:  return "tracking";
    case ReferenceSource::Search:    return "search";
    case ReferenceSource::Hold:      return "hold";
    case ReferenceSource::Developer: return "developer";
    case ReferenceSource::Manual:     return "manual";
    case ReferenceSource::Roam:       return "roam";
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
  // §67: the intent asked to point somewhere unreachable. The caller turns this
  // into AutoTrackState::TARGET_UNREACHABLE — the converter does not decide
  // policy, it reports what it could not satisfy. Without this field the only
  // signal would be "it holds, for no stated reason".
  bool target_unreachable = false;
  // Literal-only diagnostics (§52): static strings, so no allocation on the
  // control thread (§46).
  const char* reason = "";
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

  // --- v3: convert a MotionIntent (§53) ---------------------------------
  //
  // The shape §53 asks for. Note what is NOT here: no mode logic, no priority
  // chain. The caller has already applied the §26 ordering (safety override,
  // then homing/calibration/park, then the one authoritative mode controller)
  // and hands over the surviving intent; this function's whole job is turning
  // "point at this LOS" / "go to these joints" / "hold" into a joint reference
  // with a speed limit, and saying honestly when it cannot.
  //
  // The v1 compute() above stays for the existing paths and their tests; it
  // retires as each mode migrates onto intents, not before.
  struct IntentLimits {
    int64_t now_ns = 0;
    // Where to hold when the intent says hold.
    double q_yaw_hold_rad = 0.0;
    double q_pitch_hold_rad = 0.0;
    // Per-source configured ceilings; the intent's velocity_scale multiplies the
    // one belonging to its own source, and the envelope gets the final word.
    double track_v_max_rad_s = 30.0 * kDeg2Rad;
    double roam_v_max_rad_s = 10.0 * kDeg2Rad;
    double manual_v_max_rad_s = 30.0 * kDeg2Rad;
    double hold_v_max_rad_s = 10.0 * kDeg2Rad;
  };

  ReferenceRequest resolve(const MotionIntent& in, const IntentLimits& lim) const {
    ReferenceRequest req;
    req.confidence = in.confidence;
    req.reason = in.reason[0] != '\0' ? in.reason : intent_type_name(in.type);

    // An expired intent is a hold, whatever it asked for. §93 requires that a
    // stale intent cannot survive a transition; handling it here makes that a
    // property of the data instead of a rule every consumer must remember.
    if (in.source == MotionSource::None || !in.live_at(lim.now_ns)) {
      return hold_reference(lim, in.source == MotionSource::None
                                    ? "no intent"
                                    : "intent expired -> hold");
    }

    const double vs = clamp_scale(in.velocity_scale);
    switch (in.type) {
      case IntentType::LosDirection: {
        if (!in.has_los) return hold_reference(lim, "los intent without los");
        double qy, qp;
        if (!solver_.solve(in.los_az_rad, in.los_el_rad, qy, qp)) {
          // §67: an unreachable target is reported, not pressed into a hold.
          req = hold_reference(lim, "target outside travel");
          req.target_unreachable = true;
          return req;
        }
        req.q_yaw_rad = qy;
        req.q_pitch_rad = qp;
        req.source = ReferenceSource::Tracking;
        req.is_tracking_reference = true;
        req.v_max_rad_s = lim.track_v_max_rad_s * vs;
        return req;
      }
      case IntentType::JointPosition: {
        if (!in.has_joint_target)
          return hold_reference(lim, "joint intent without target");
        req.q_yaw_rad = in.q_yaw_rad;
        req.q_pitch_rad = in.q_pitch_rad;
        req.source = (in.source == MotionSource::AutoRoam) ? ReferenceSource::Roam
                                                          : ReferenceSource::Manual;
        req.v_max_rad_s = (in.source == MotionSource::AutoRoam
                               ? lim.roam_v_max_rad_s
                               : lim.manual_v_max_rad_s) * vs;
        return req;
      }
      case IntentType::WorldLevelYaw: {
        // The level constraint itself is resolved upstream: it needs gravity
        // from the IMU expansion, which §98 says is not a v3 dependency. What
        // arrives is the joint pose that constraint produced, plus the elevation
        // it was derived at — kept so telemetry can show what was asked for, not
        // only what was commanded.
        if (!in.has_joint_target)
          return hold_reference(lim, "level intent without joint target");
        req.q_yaw_rad = in.q_yaw_rad;
        req.q_pitch_rad = in.q_pitch_rad;
        req.source = ReferenceSource::Roam;
        req.v_max_rad_s = lim.roam_v_max_rad_s * vs;
        if (in.has_world_elevation) req.reason = "world-level sweep";
        return req;
      }
      case IntentType::JointVelocity: {
        // Deliberately not honoured, and deliberately not reinterpreted as a
        // position. Measured on this station, CyberGear speed mode does not move
        // a loaded axis at the commanded rate on default gains, so a jog has to
        // be integrated into a position reference at the control rate — which is
        // ManualController's job in V3-5, not the converter's. "Hold, with a
        // stated reason" is the honest interim: the operator reads why, instead
        // of watching a turret do something unrelated to the button pressed.
        (void)vs;
        return hold_reference(lim, "velocity intent not yet supported");
      }
      case IntentType::Hold:
      default:
        return hold_reference(lim, in.reason[0] != '\0' ? in.reason : "hold");
    }
  }

 private:
  static double clamp_scale(double s) {
    // Scales are derating factors (§19, §20.1). Capping at 1.5 lets a deliberate
    // modest override through while a misconfigured 10.0 cannot become a 10x
    // speed command; the envelope still clamps position and the supervisor still
    // owns the brake. The NaN case falls to 0.0 (a hold), which is the only
    // comparison-safe reading of an unrepresentable scale.
    if (!(s >= 0.0)) return 0.0;
    return s > 1.5 ? 1.5 : s;
  }

  static ReferenceRequest hold_reference(const IntentLimits& lim,
                                         const char* why) {
    ReferenceRequest req;
    req.q_yaw_rad = lim.q_yaw_hold_rad;
    req.q_pitch_rad = lim.q_pitch_hold_rad;
    req.source = ReferenceSource::Hold;
    req.v_max_rad_s = lim.hold_v_max_rad_s;
    req.confidence = 0.0;
    req.reason = why;
    return req;
  }

  geo::LosJointSolver solver_;
};

}  // namespace ota
