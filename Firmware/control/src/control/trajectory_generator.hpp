// OpenAutoTurret — online jerk-limited trajectory generator (architecture §17).
//
// A receding-horizon, jerk-limited setpoint tracker for one axis. Each control
// cycle it emits a reference state (q_ref, v_ref, a_ref) that stays within the
// velocity, acceleration and jerk limits while driving toward the target. The
// position mode consumes q_ref; v_ref/a_ref are useful for safety and
// diagnostics. The generator continues from the current planned state when the
// target changes (it never restarts a discontinuous profile from zero).
#pragma once

#include "common/types.hpp"

namespace ota {

struct TrajectoryLimits {
  double v_max_rad_s = 30.0 * kDeg2Rad;    // |v| limit
  double a_max_rad_s2 = 60.0 * kDeg2Rad;   // |a| limit
  double j_max_rad_s3 = 300.0 * kDeg2Rad;  // |j| limit
};

struct TrajectoryState {
  double q_rad = 0.0;
  double v_rad_s = 0.0;
  double a_rad_s2 = 0.0;
};

class JerkLimitedTrajectory {
 public:
  JerkLimitedTrajectory(TrajectoryLimits lim, double dt_s);

  void set_target(double q_target_rad) { target_ = q_target_rad; }
  double target() const { return target_; }

  // Advance one control cycle (dt_ has elapsed) and return the reference state.
  TrajectoryState step();

  // Re-seed the generator at a known state (e.g. the measured q/v at start).
  void reset(double q0_rad, double v0_rad_s = 0.0, double a0_rad_s2 = 0.0);

  bool at_target(double pos_tol_rad, double vel_tol_rad_s) const;

  double q() const { return q_; }
  double v() const { return v_; }
  double a() const { return a_; }

 private:
  TrajectoryLimits lim_;
  double dt_;
  double q_ = 0.0;
  double v_ = 0.0;
  double a_ = 0.0;
  double target_ = 0.0;
};

}  // namespace ota
