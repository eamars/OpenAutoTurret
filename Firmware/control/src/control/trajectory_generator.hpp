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

// A plan to bring the axis to a full stop from a given state under the braking
// limits (architecture §17.2, §48). The stop distance is derived from the same
// jerk-limited model the planner uses (d_stop = v^2/(2a) + v*a/(2j)), so it is
// consistent with (and <=, without the extra safety margin) what the
// SafetyEnvelope's independent stop-feasibility check assumes.
struct StopPlan {
  double end_q_rad = 0.0;        // position where the axis comes to rest
  double time_to_stop_s = 0.0;   // time to come to rest
  double peak_decel_rad_s2 = 0.0;  // peak deceleration magnitude applied
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

  // §48: compute the stop plan from the given state (q, v, a) under the braking
  // limits (a_brake, j_brake). The deceleration is the full braking
  // acceleration ramped up at the braking jerk, so the stop distance is the
  // jerk-limited form (v^2/(2a) + v*a/(2j)). The state's acceleration is
  // accepted for API parity with §48 but the stop is derived from |v| (the
  // acceleration is a bounded second-order term the SafetyEnvelope's margin
  // covers); this keeps the model consistent with SafetyEnvelope::stop_distance.
  StopPlan plan_stop(const TrajectoryState& s, double a_brake, double j_brake) const;

  // §17.2/§48: from state s, can the axis come to a full stop (using the
  // braking limits) without crossing the active soft boundary q_boundary_rad
  // in the direction of motion? Returns true if the jerk-limited stop distance
  // fits before the boundary (stop reachable); false otherwise (stop infeasible
  // -> the supervisor must force a controlled max-safe stop). Consistent with
  // SafetyEnvelope::stop_feasible (which adds an extra safety margin).
  bool verify_stop_reachability(const TrajectoryState& s, double q_boundary_rad,
                                double a_brake, double j_brake) const;

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
