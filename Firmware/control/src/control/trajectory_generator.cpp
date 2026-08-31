// OpenAutoTurret — online jerk-limited trajectory generator (architecture §17).
#include "control/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace ota {
namespace {
// Velocity-tracking time constant for the proportional command. Small enough
// to settle on the target in a few seconds, large enough (several control
// cycles) not to excite the jerk-limited acceleration loop into oscillation.
constexpr double kVelTauS = 0.2;

// |v| below this is "at rest": a stationary axis cannot cross a boundary by
// stopping, so the stop-reachability check is satisfied regardless of position
// (mirrors SafetyEnvelopeParams::at_rest_vel_rad_s).
constexpr double kAtRestVelRadS = 1e-3;

double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}
}  // namespace

JerkLimitedTrajectory::JerkLimitedTrajectory(TrajectoryLimits lim, double dt_s)
    : lim_(lim), dt_(dt_s) {}

void JerkLimitedTrajectory::reset(double q0_rad, double v0_rad_s, double a0_rad_s2) {
  q_ = q0_rad;
  v_ = v0_rad_s;
  a_ = a0_rad_s2;
  target_ = q0_rad;
}

bool JerkLimitedTrajectory::at_target(double pos_tol_rad, double vel_tol_rad_s) const {
  // "Arrived and settled": within the position and velocity tolerances. (The
  // acceleration is left out because, under the jerk limit, it lags the velocity
  // and can be non-zero even while q and v are settled.)
  return std::fabs(target_ - q_) < pos_tol_rad && std::fabs(v_) < vel_tol_rad_s;
}

TrajectoryState JerkLimitedTrajectory::step() {
  const double d = target_ - q_;
  const double sign_d = (d >= 0.0) ? 1.0 : -1.0;
  const double ad = std::fabs(d);
  const double a_max = lim_.a_max_rad_s2;
  const double j_max = lim_.j_max_rad_s3;

  // Jerk-aware cruise cap. Reaching the cap leaves the acceleration at full
  // value, which coasts the velocity up by ~a_max^2/(2 j_max) before cruise
  // takes hold, so cruise a touch below v_max to keep the peak within the limit.
  const double v_cap = std::max(0.0, lim_.v_max_rad_s - (a_max * a_max) /
                                                     (2.0 * j_max));
  // Jerk-aware stop velocity: the largest speed from which we can still stop at
  // the target. Solving d_stop(v) = ad with d_stop(v) = v^2/(2 a_max) +
  // v a_max/(2 j_max) (the full-deceleration term plus the distance covered
  // while the jerk limit ramps the deceleration up).
  const double b = a_max / (2.0 * j_max);
  const double v_stop = a_max * (-b + std::sqrt(b * b + 2.0 * ad / a_max));
  // Velocity command: never faster than the cruise cap, and never faster than
  // the speed that still allows a stop at the target.
  const double v_cmd = sign_d * std::min(v_cap, v_stop);

  // Track the velocity command with a bounded acceleration, then apply the jerk
  // limit so the change in acceleration (and hence torque demand) is smooth.
  const double a_cmd_unclamped = (v_cmd - v_) / kVelTauS;
  double a_cmd = clamp(a_cmd_unclamped, -a_max, a_max);
  const double da_max = j_max * dt_;
  a_cmd = a_ + clamp(a_cmd - a_, -da_max, da_max);

  // Integrate.
  a_ = a_cmd;
  v_ += a_ * dt_;
  q_ += v_ * dt_;

  return TrajectoryState{q_, v_, a_};
}

StopPlan JerkLimitedTrajectory::plan_stop(const TrajectoryState& s, double a_brake,
                                          double j_brake) const {
  const double v = std::fabs(s.v_rad_s);
  // A (near) stationary axis is already stopped; the stop plan is "stay here"
  // (consistent with the SafetyEnvelope's at-rest short-circuit).
  if (v < kAtRestVelRadS) return StopPlan{s.q_rad, 0.0, 0.0};
  // Jerk-limited stop distance and time, matching the SafetyEnvelope model
  // (d_stop = v^2/(2a) + v*a/(2j); t_stop = v/a + a/(2j)).
  const double d_stop = v * v / (2.0 * a_brake) + v * a_brake / (2.0 * j_brake);
  const double t_stop = v / a_brake + a_brake / (2.0 * j_brake);
  const double dir = (s.v_rad_s >= 0.0) ? 1.0 : -1.0;
  return StopPlan{s.q_rad + dir * d_stop, t_stop, a_brake};
}

bool JerkLimitedTrajectory::verify_stop_reachability(const TrajectoryState& s,
                                                     double q_boundary_rad,
                                                     double a_brake, double j_brake) const {
  // A (near) stationary axis is already stopped; it cannot cross a boundary by
  // stopping, so the check is satisfied regardless of position.
  if (std::fabs(s.v_rad_s) < kAtRestVelRadS) return true;
  const StopPlan plan = plan_stop(s, a_brake, j_brake);
  if (s.v_rad_s > 0.0) return plan.end_q_rad <= q_boundary_rad + 1e-9;
  return plan.end_q_rad >= q_boundary_rad - 1e-9;
}

}  // namespace ota
