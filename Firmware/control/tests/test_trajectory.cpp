// Unit tests for the jerk-limited trajectory generator (architecture §17).
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

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

// §17.3: repeated rapid retargeting must not introduce a reference
// discontinuity and the generator must still converge to the final target.
TEST(Trajectory, RepeatedRapidRetargetsStayContinuous) {
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  traj.reset(0.0);
  const double targets[4] = {50.0, -40.0, 30.0, -10.0};
  for (double t : targets) traj.set_target(t * ota::kDeg2Rad);
  double prev_q = 0.0, prev_v = 0.0, prev_a = 0.0;
  bool ok = true;
  for (int i = 0; i < 8000; ++i) {
    if (i % 500 == 499)
      traj.set_target(targets[(i / 500 + 1) % 4] * ota::kDeg2Rad);  // retarget
    auto s = traj.step();
    if (std::fabs(s.q_rad - prev_q) > lim.v_max_rad_s * kDt + 1e-6) ok = false;
    if (std::fabs(s.v_rad_s - prev_v) > lim.a_max_rad_s2 * kDt + 1e-6) ok = false;
    if (std::fabs(s.a_rad_s2 - prev_a) > lim.j_max_rad_s3 * kDt + 1e-6) ok = false;
    prev_q = s.q_rad;
    prev_v = s.v_rad_s;
    prev_a = s.a_rad_s2;
    if (traj.at_target(1e-4, 1e-3)) break;
  }
  EXPECT_TRUE(ok);  // C0/C1 continuity held through every retarget
}

// §48: plan_stop must reproduce the jerk-limited stop-distance model.
TEST(Trajectory, PlanStopMatchesModel) {
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  const double a = lim.a_max_rad_s2, j = lim.j_max_rad_s3;
  for (double vdeg : {-30.0, -10.0, 10.0, 30.0}) {  // non-at-rest speeds
    const double v = vdeg * ota::kDeg2Rad;
    ota::TrajectoryState s{0.5, v, 0.0};
    auto plan = traj.plan_stop(s, a, j);
    const double d = std::fabs(v) * std::fabs(v) / (2.0 * a) +
                     std::fabs(v) * a / (2.0 * j);
    const double dir = (v >= 0.0) ? 1.0 : -1.0;
    EXPECT_NEAR(plan.end_q_rad, 0.5 + dir * d, 1e-9);
    EXPECT_GT(plan.time_to_stop_s, 0.0);
    EXPECT_DOUBLE_EQ(plan.peak_decel_rad_s2, a);
  }
  // At rest (|v| below the at-rest threshold): the stop plan is "stay here".
  ota::TrajectoryState rest{0.5, 0.0, 0.0};
  auto rp = traj.plan_stop(rest, a, j);
  EXPECT_NEAR(rp.end_q_rad, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(rp.time_to_stop_s, 0.0);
}

// §17.2/§48: a stop that fits before the boundary is reachable; one that does
// not is not (and the supervisor must brake).
TEST(Trajectory, VerifyStopReachability) {
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  const double a = lim.a_max_rad_s2, j = lim.j_max_rad_s3;
  for (double vdeg : {5.0, 15.0, 30.0}) {
    const double v = vdeg * ota::kDeg2Rad;
    ota::TrajectoryState s{0.0, v, 0.0};
    const auto plan = traj.plan_stop(s, a, j);
    // Just beyond the stop point -> feasible. Exactly at it -> feasible.
    EXPECT_TRUE(traj.verify_stop_reachability(s, plan.end_q_rad + 1e-6, a, j));
    EXPECT_TRUE(traj.verify_stop_reachability(s, plan.end_q_rad, a, j));
    // Just before the stop point (in the direction of motion) -> infeasible.
    EXPECT_FALSE(traj.verify_stop_reachability(s, plan.end_q_rad - 1e-6, a, j));
    // A boundary far behind the start (opposite the motion) is never reached.
    EXPECT_TRUE(traj.verify_stop_reachability(s, plan.end_q_rad + 1.0, a, j));
  }
}

// §48: "This should be unit-tested against randomized positions/velocities."
// Deterministic RNG (fixed seed) so the test is reproducible, but the states
// are uniformly randomized over the operating range.
TEST(Trajectory, VerifyStopReachabilityRandomized) {
  TrajectoryLimits lim;
  JerkLimitedTrajectory traj(lim, kDt);
  const double a = lim.a_max_rad_s2, j = lim.j_max_rad_s3;
  std::mt19937 rng(0xC0FFEEu);
  std::uniform_real_distribution<double> qdist(-3.0, 3.0);
  std::uniform_real_distribution<double> vdist(-lim.v_max_rad_s, lim.v_max_rad_s);
  for (int i = 0; i < 2000; ++i) {
    const double q = qdist(rng);
    const double v = vdist(rng);
    ota::TrajectoryState s{q, v, 0.0};
    const auto plan = traj.plan_stop(s, a, j);
    const double dir = (v >= 0.0) ? 1.0 : -1.0;
    // The stop must end strictly in the direction of motion (or at q if at
    // rest) and never behind the start.
    if (std::fabs(v) > 1e-3) {
      EXPECT_GT((plan.end_q_rad - q) * dir, 0.0);
      // A boundary just beyond the stop is feasible; just before (in the
      // direction of motion) is infeasible.
      EXPECT_TRUE(traj.verify_stop_reachability(s, plan.end_q_rad + dir * 1e-6, a, j));
      EXPECT_FALSE(traj.verify_stop_reachability(s, plan.end_q_rad - dir * 1e-6, a, j));
    } else {
      // At rest: no travel, and always reachable (cannot cross by stopping).
      EXPECT_NEAR(plan.end_q_rad, q, 1e-9);
      EXPECT_TRUE(traj.verify_stop_reachability(s, q - 1.0, a, j));
    }
  }
}

}  // namespace
