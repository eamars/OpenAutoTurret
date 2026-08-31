// Unit tests for the jerk-limited trajectory generator (architecture §17).
#include <gtest/gtest.h>

#include <cmath>

#include "control/trajectory_generator.hpp"

namespace {

using ota::JerkLimitedTrajectory;
using ota::TrajectoryLimits;

constexpr double kDt = 0.005;  // 200 Hz

struct LimitsSeen {
  double v_max = 0.0, a_max = 0.0, j_max = 0.0;
};

// Drive the generator to the target and track the observed limits.
LimitsSeen runToTarget(JerkLimitedTrajectory& traj, double target, int max_steps,
                       double pos_tol, double vel_tol) {
  traj.set_target(target);
  LimitsSeen seen;
  double prev_a = 0.0;
  for (int i = 0; i < max_steps; ++i) {
    auto s = traj.step();
    seen.v_max = std::max(seen.v_max, std::fabs(s.v_rad_s));
    seen.a_max = std::max(seen.a_max, std::fabs(s.a_rad_s2));
    seen.j_max = std::max(seen.j_max, std::fabs(s.a_rad_s2 - prev_a) / kDt);
    prev_a = s.a_rad_s2;
    if (traj.at_target(pos_tol, vel_tol)) break;
  }
  return seen;
}

TEST(Trajectory, ReachesTarget) {
  TrajectoryLimits lim;  // v=30deg/s, a=60deg/s2, j=300deg/s3
  JerkLimitedTrajectory traj(lim, kDt);
  traj.reset(0.0);
  const double target = 20.0 * ota::kDeg2Rad;
  auto seen = runToTarget(traj, target, 4000, 1e-4, 1e-3);
  EXPECT_NEAR(traj.q(), target, 1e-4);
  EXPECT_NEAR(traj.v(), 0.0, 1e-3);
  (void)seen;
}

TEST(Trajectory, RespectsLimits) {
  TrajectoryLimits lim;  // v=30deg/s, a=60deg/s2, j=300deg/s3
  JerkLimitedTrajectory traj(lim, kDt);
  traj.reset(0.0);
  const double target = 90.0 * ota::kDeg2Rad;  // a long move
  auto seen = runToTarget(traj, target, 6000, 1e-4, 1e-3);
  // Small tolerance for the jerk transition and the velocity safety cap.
  EXPECT_LE(seen.v_max, lim.v_max_rad_s + 1e-6);
  EXPECT_LE(seen.a_max, lim.a_max_rad_s2 + 1e-6);
  EXPECT_LE(seen.j_max, lim.j_max_rad_s3 + 1e-6);
}

TEST(Trajectory, NegativeDirection) {
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  traj.reset(0.0);
  const double target = -30.0 * ota::kDeg2Rad;
  auto seen = runToTarget(traj, target, 4000, 1e-4, 1e-3);
  EXPECT_NEAR(traj.q(), target, 1e-4);
  EXPECT_NEAR(traj.v(), 0.0, 1e-3);
  (void)seen;
}

TEST(Trajectory, ChangingTargetIsSmooth) {
  // Receding horizon: retarget mid-move; q/v/a must remain continuous (no
  // discontinuity is introduced by the retarget).
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  traj.reset(0.0);
  traj.set_target(40.0 * ota::kDeg2Rad);
  double prev_q = 0.0, prev_v = 0.0, prev_a = 0.0;
  bool ok = true;
  for (int i = 0; i < 4000; ++i) {
    if (i == 400) traj.set_target(-20.0 * ota::kDeg2Rad);  // reverse mid-move
    auto s = traj.step();
    // The reference position should advance smoothly (bounded per-step change).
    if (std::fabs(s.q_rad - prev_q) > lim.v_max_rad_s * kDt + 1e-6) ok = false;
    if (std::fabs(s.v_rad_s - prev_v) > lim.a_max_rad_s2 * kDt + 1e-6) ok = false;
    if (std::fabs(s.a_rad_s2 - prev_a) > lim.j_max_rad_s3 * kDt + 1e-6) ok = false;
    prev_q = s.q_rad;
    prev_v = s.v_rad_s;
    prev_a = s.a_rad_s2;
    if (traj.at_target(1e-4, 1e-3)) break;
  }
  EXPECT_TRUE(ok);
  EXPECT_NEAR(traj.q(), -20.0 * ota::kDeg2Rad, 1e-4);
}

}  // namespace
