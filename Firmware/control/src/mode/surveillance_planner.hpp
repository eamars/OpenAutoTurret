#pragma once
// SURVEILLANCE (owner, 2026-10-05): the turret faces a saved watch point and holds it, like a fixed
// camera. A target in view is followed exactly as AUTO_TRACK follows one -- the automatic hand-off
// switches to AUTO_TRACK -- and when it is lost with nobody else to follow, the hand-off comes back
// here and the turret returns to the watch point. This planner owns only that return and the hold.
//
// Two decisions keep it small:
//
//   * The intent is the watch point on every cycle, moving or not. "Return" and "watch" are labels
//     for the operator, never a gate on the motion: the roam planner's TURNAROUND once waited forever
//     for an arrival the servo rests 0.24 deg short of (2026-10-02), and nothing here waits for one.
//     A turret pushed off its point goes back for the same reason it went there the first time.
//
//   * The targets arrive already resolved into this session's joint coordinates. Where the watch
//     point comes from (the saved file, the yaw's absolute encoder angle, the pitch's homed frame)
//     and whether it is inside the envelope are the loop's business, decided once at entry; the
//     planner never sees a stored number, so it cannot aim at one from another session.
//
// Control-thread only, no allocation, no I/O. Speed is not decided here: the intent carries the
// source, and the reference manager gives it the watch ceiling.
#include <cmath>
#include <cstdint>

#include "common/time.hpp"
#include "control/motion_intent.hpp"

namespace ota {

enum class SurveillanceState : uint8_t {
  Idle,    // not the mode's turn; nothing emitted
  Return,  // on the way to the watch point
  Watch,   // at it, holding
};

inline const char* surveillance_state_name(SurveillanceState s) {
  switch (s) {
    case SurveillanceState::Idle: return "IDLE";
    case SurveillanceState::Return: return "RETURN";
    case SurveillanceState::Watch: return "WATCH";
  }
  return "?";
}

struct SurveillanceConfig {
  // The WATCH label: both axes within `watch_band_rad` of the point. It drops back to RETURN only
  // beyond `return_band_rad`, so a servo resting near the band edge does not flicker the label. The
  // yaw servo rests within 0.44 deg (config/servo/yaw_accuracy.json) and the pitch servo within
  // ~0.2 deg, so 1 deg is reached by a turret that has stopped, and 2 deg is a real excursion.
  double watch_band_rad = 1.0 * 3.14159265358979323846 / 180.0;
  double return_band_rad = 2.0 * 3.14159265358979323846 / 180.0;
};

struct SurveillanceOutput {
  MotionIntent intent;
  SurveillanceState state = SurveillanceState::Idle;
  double target_yaw_rad = 0.0;
  double target_pitch_rad = 0.0;
  const char* reason = "surveillance idle";
};

class SurveillancePlanner {
 public:
  explicit SurveillancePlanner(SurveillanceConfig cfg = SurveillanceConfig()) : cfg_(cfg) {}

  bool active() const { return state_ != SurveillanceState::Idle; }
  SurveillanceState state() const { return state_; }
  double target_yaw_rad() const { return yaw_; }
  double target_pitch_rad() const { return pitch_; }

  // Aim at (yaw, pitch), joint radians of this session. Re-entering with a new point (the operator
  // saved another one, or a loss brought the hand-off back) simply re-aims.
  void enter(double yaw_rad, double pitch_rad) {
    yaw_ = yaw_rad;
    pitch_ = pitch_rad;
    state_ = SurveillanceState::Return;
  }

  void exit() { state_ = SurveillanceState::Idle; }

  SurveillanceOutput update(double q_yaw_rad, double q_pitch_rad, TimeNs now_ns) {
    SurveillanceOutput out;
    out.intent.source = MotionSource::Surveillance;
    out.intent.timestamp_ns = now_ns;
    if (state_ == SurveillanceState::Idle || !std::isfinite(yaw_) || !std::isfinite(pitch_)) {
      state_ = SurveillanceState::Idle;
      out.intent.set_reason(out.reason);
      return out;  // Hold. A planner with nothing to aim at does not invent a point.
    }
    const double off = std::fmax(std::fabs(q_yaw_rad - yaw_), std::fabs(q_pitch_rad - pitch_));
    if (!std::isfinite(off) || off > cfg_.return_band_rad) state_ = SurveillanceState::Return;
    else if (off <= cfg_.watch_band_rad) state_ = SurveillanceState::Watch;
    out.state = state_;
    out.target_yaw_rad = yaw_;
    out.target_pitch_rad = pitch_;
    out.intent.type = IntentType::JointPosition;
    out.intent.has_joint_target = true;
    out.intent.q_yaw_rad = yaw_;
    out.intent.q_pitch_rad = pitch_;
    out.intent.confidence = 1.0;  // no target involved, so nothing is uncertain
    out.intent.velocity_scale = 1.0;
    out.reason = state_ == SurveillanceState::Watch ? "watching" : "returning to the watch point";
    out.intent.set_reason(out.reason);
    return out;
  }

 private:
  SurveillanceConfig cfg_;
  SurveillanceState state_ = SurveillanceState::Idle;
  double yaw_ = 0.0;
  double pitch_ = 0.0;
};

}  // namespace ota
