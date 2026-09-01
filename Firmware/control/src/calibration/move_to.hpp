// OpenAutoTurret — "drive an axis to a target and hold" motion primitive.
//
// Shared by the homing plan (clearance poses between axes, §25) and the park
// controller (drive each axis to its park position, §33). Transport-agnostic:
// it takes HomingFeedback and returns a DesiredState, so it is unit-testable
// against a simulated plant with no CAN (§54).
//
// The DesiredState speed is a POSITIVE magnitude; the direction is implied by
// the target (the move executor drives toward target_rad_ at up to this speed).
// The move "arrives" when the position and velocity are both within tolerance
// of the target, then holds. It fails on a timeout.
#pragma once

#include <cmath>
#include <string>

#include "calibration/homing_controller.hpp"  // HomingFeedback, DesiredState
#include "common/types.hpp"

namespace ota {

class MoveTo {
 public:
  MoveTo(AxisId axis, double target_rad, double speed_rad_s, double pos_tol_rad,
         double vel_tol_rad_s, double timeout_s)
      : axis_(axis),
        target_rad_(target_rad),
        speed_rad_s_(speed_rad_s),
        pos_tol_rad_(pos_tol_rad),
        vel_tol_rad_s_(vel_tol_rad_s),
        timeout_ns_(static_cast<TimeNs>(timeout_s * 1e9)) {}

  DesiredState step(const HomingFeedback& fb) {
    if (t0_ns_ == 0) t0_ns_ = fb.t_ns;
    if (arrived_) return DesiredState{target_rad_, 0.0, 0.0, true, "holding at move target"};
    if (failed_) return DesiredState{fb.pos_rad, 0.0, 0.0, true, "move failed: " + reason_};

    const double d = target_rad_ - fb.pos_rad;
    if (std::fabs(d) < pos_tol_rad_ && std::fabs(fb.vel_rad_s) < vel_tol_rad_s_) {
      arrived_ = true;
      reason_ = "arrived at move target";
      return DesiredState{target_rad_, 0.0, 0.0, true, reason_};
    }
    if (fb.t_ns - t0_ns_ > timeout_ns_) {
      failed_ = true;
      reason_ = "move timeout";
      return DesiredState{fb.pos_rad, 0.0, 0.0, true, "move failed: " + reason_};
    }
    // Command a SIGNED velocity toward the target (the velocity-mode executor
    // — the control loop's homing phase, which drives via SpdRef — commands
    // ds.velocity_rad_s, so a magnitude-only state would not move the axis).
    // Cap the speed by the stop distance so it reaches 0 at the target
    // (v = sqrt(2 a d)): a constant-velocity command would never satisfy the
    // arrival velocity test and the move would time out. The position-mode
    // executor (park) ignores velocity_rad_s and uses target_rad/speed_rad_s.
    const double dist = std::fabs(d);
    const double v_stop = std::sqrt(2.0 * decel_rad_s2_ * dist);
    const double v_mag = std::min(speed_rad_s_, v_stop);
    const double v = (d >= 0.0) ? v_mag : -v_mag;
    return DesiredState{target_rad_, speed_rad_s_, v, false, "moving to target"};
  }

  bool terminal() const { return arrived_ || failed_; }
  bool ok() const { return arrived_; }
  const std::string& reason() const { return reason_; }

 private:
  AxisId axis_;
  double target_rad_;
  double speed_rad_s_;
  double pos_tol_rad_;
  double vel_tol_rad_s_;
  TimeNs timeout_ns_;
  // Deceleration for the velocity-mode stop-distance profile (rad/s^2). Chosen
  // conservative (slow) so the drive's own velocity loop can follow it without
  // overshoot; the position-mode executor (park) never uses it.
  double decel_rad_s2_ = 0.5;
  TimeNs t0_ns_ = 0;
  bool arrived_ = false;
  bool failed_ = false;
  std::string reason_;
};

}  // namespace ota
