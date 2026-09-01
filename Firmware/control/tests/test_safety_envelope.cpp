// Unit tests for the end-stop safety envelope (architecture §18.2/§18.3).
// The envelope is exercised INDEPENDENTLY of the trajectory generator (§18.3):
// these tests use only SafetyEnvelope/AxisLimits, no planner.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "control/safety_envelope.hpp"

namespace {

using ota::AxisLimits;
using ota::SafetyEnvelope;
using ota::SafetyEnvelopeParams;
constexpr double kDeg = ota::kDeg2Rad;

// A pitch-like axis: hard stops at -1.0 and +1.0 rad, soft margin 0.1 rad.
AxisLimits make_limits() {
  AxisLimits l;
  l.set_from_endpoints(-1.0, 1.0, 0.1);
  return l;
}

SafetyEnvelope make_env() {
  SafetyEnvelopeParams p;
  p.a_brake_rad_s2 = 60.0 * kDeg;
  p.j_brake_rad_s3 = 300.0 * kDeg;
  p.margin_rad = 0.01;  // small braking-distance buffer for clean test numbers
  return SafetyEnvelope(p);
}

}  // namespace

TEST(SafetyEnvelope, StopDistanceGrowsWithSpeed) {
  const auto env = make_env();
  EXPECT_GT(env.stop_distance(0.1), env.stop_distance(0.0));
  EXPECT_GT(env.stop_distance(1.0), env.stop_distance(0.1));
  EXPECT_GT(env.stop_distance(-1.0), env.stop_distance(0.5));  // symmetric in |v|
  // Jerk model is at least as conservative as pure constant decel.
  const double a = 60.0 * kDeg;
  EXPECT_GE(env.stop_distance(1.0) - 0.01, 1.0 * 1.0 / (2.0 * a));
}

TEST(SafetyEnvelope, StopFeasibleFarAndSlow) {
  const auto env = make_env();
  const auto lim = make_limits();
  // Mid-axis, slow: easily stops before either boundary.
  EXPECT_TRUE(env.stop_feasible(0.0, 0.1, lim));
  EXPECT_TRUE(env.stop_feasible(-0.8, -0.05, lim));  // inside soft limit, slow
  // At the soft boundary even a tiny speed is infeasible (the margin alone
  // exceeds the zero available distance).
  EXPECT_FALSE(env.stop_feasible(-0.9, -0.05, lim));
}

TEST(SafetyEnvelope, AtRestAtBoundaryIsFeasible) {
  // Regression (root cause #2): the supervisor must feed a position-derived
  // velocity (v_est_), NOT the drive's noisy self-reported v. At a mechanical
  // stop the axis sits OUTSIDE the inset soft limit, so the drive's at-rest
  // noise band (±0.05 rad/s) defeats the at-rest exemption and stop_feasible
  // fails -> spurious BRAKE flapping. A truly at-rest velocity (what v_est_
  // reports at a stop) must pass the at-rest exemption.
  const auto env = make_env();
  const auto lim = make_limits();
  // At the lower soft boundary (-0.9) with the drive's noisy at-rest sample
  // (0.05 rad/s): infeasible (this is the bug the v_est_ fix removes).
  EXPECT_FALSE(env.stop_feasible(-0.9, -0.05, lim));
  // A truly at-rest velocity (0.0, what v_est_ reports at a stop): feasible.
  EXPECT_TRUE(env.stop_feasible(-0.9, 0.0, lim));
  // Below the at_rest threshold (1e-3 rad/s): also feasible.
  EXPECT_TRUE(env.stop_feasible(-0.9, -0.0005, lim));
  SUCCEED();
}


TEST(SafetyEnvelope, StopInfeasibleNearBoundaryAndFast) {
  const auto env = make_env();
  const auto lim = make_limits();
  // Near the upper soft boundary (0.9) moving up fast: cannot stop in time.
  EXPECT_FALSE(env.stop_feasible(0.95, 2.0, lim));
  // Near the lower soft boundary moving down fast: cannot stop in time.
  EXPECT_FALSE(env.stop_feasible(-0.95, -2.0, lim));
}

TEST(SafetyEnvelope, MaxSpeedShrinksTowardBoundary) {
  const auto env = make_env();
  const auto lim = make_limits();
  const double v_mid = env.max_speed_at(0.0, lim);
  const double v_near = env.max_speed_at(0.85, lim);
  const double v_at = env.max_speed_at(0.9, lim);  // at the soft boundary
  EXPECT_GT(v_mid, v_near);
  EXPECT_GT(v_near, 0.0);
  EXPECT_DOUBLE_EQ(v_at, 0.0);
}

TEST(SafetyEnvelope, MaxSpeedConsistentWithStopDistance) {
  // max_speed_at is the speed whose full stop (incl. margin) exactly reaches
  // the nearer soft boundary: stop_distance(vmax) == distance_to_soft(q).
  const auto env = make_env();
  const auto lim = make_limits();
  for (double q : {-0.8, -0.4, 0.0, 0.4, 0.8}) {
    const double vmax = env.max_speed_at(q, lim);
    const double expect = lim.distance_to_soft(q);
    EXPECT_NEAR(env.stop_distance(vmax), expect, 1e-9) << "q=" << q;
  }
}

TEST(SafetyEnvelope, ConstrainReferenceClampsToSoft) {
  const auto env = make_env();
  const auto lim = make_limits();
  EXPECT_DOUBLE_EQ(env.constrain_reference(0.5, lim), 0.5);
  EXPECT_DOUBLE_EQ(env.constrain_reference(5.0, lim), lim.q_soft_max_rad);
  EXPECT_DOUBLE_EQ(env.constrain_reference(-5.0, lim), lim.q_soft_min_rad);
  // Invalid limits -> unchanged.
  AxisLimits none;
  EXPECT_DOUBLE_EQ(env.constrain_reference(3.0, none), 3.0);
}

TEST(SafetyEnvelope, EmergencyStopTargetClampsToBoundary) {
  const auto env = make_env();
  const auto lim = make_limits();
  // Fast, inside but near the upper boundary: the stop target must be clamped
  // to the upper soft limit (never past it), and stay ahead of the position.
  const double t = env.emergency_stop_target(0.8, 2.0, lim);
  EXPECT_LE(t, lim.q_soft_max_rad + 1e-12);
  EXPECT_GE(t, 0.8);  // does not reverse the commanded direction
  // Slow, mid-axis: stop target is just ahead, well inside the limits.
  const double t2 = env.emergency_stop_target(0.0, 0.2, lim);
  EXPECT_LT(t2, lim.q_soft_max_rad);
  EXPECT_GT(t2, 0.0);
}

// §48: randomized positions/velocities. Core invariant — whenever the envelope
// declares a stop feasible, the resulting stop position stays inside the HARD
// limits (it never permits reaching a boundary the brake cannot avoid).
TEST(SafetyEnvelope, RandomizedStopFeasibilityInvariant) {
  const auto env = make_env();
  const auto lim = make_limits();
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> dq(-0.99, 0.99);   // inside hard limits
  std::uniform_real_distribution<double> dv(-3.0, 3.0);
  int feasible = 0, infeasible = 0;
  for (int i = 0; i < 20000; ++i) {
    const double q = dq(rng), v = dv(rng);
    const double d_stop = env.stop_distance(v);
    const double q_stop = q + std::copysign(d_stop, v);
    if (env.stop_feasible(q, v, lim)) {
      ++feasible;
      EXPECT_GE(q_stop, lim.q_hard_min_rad - 1e-9) << "q=" << q << " v=" << v;
      EXPECT_LE(q_stop, lim.q_hard_max_rad + 1e-9) << "q=" << q << " v=" << v;
    } else {
      ++infeasible;
    }
  }
  // Both outcomes should occur across the random space (not degenerate).
  EXPECT_GT(feasible, 0);
  EXPECT_GT(infeasible, 0);
}

// max_speed_at is a safe cap: commanding it (or less) always leaves a feasible
// stop before the nearer soft boundary.
TEST(SafetyEnvelope, RandomizedMaxSpeedIsSafe) {
  const auto env = make_env();
  const auto lim = make_limits();
  std::mt19937 rng(999);
  // Strictly inside the soft limits (|q|<0.85 < 0.9) so a positive safe speed
  // exists; the "must be stopped" zone near the boundary is covered separately.
  std::uniform_real_distribution<double> dq(-0.85, 0.85);
  for (int i = 0; i < 20000; ++i) {
    const double q = dq(rng);
    const double vmax = env.max_speed_at(q, lim);
    // At exactly vmax the stop is feasible (distance_to_soft is the budget).
    EXPECT_TRUE(env.stop_feasible(q, vmax, lim)) << "q=" << q;
    EXPECT_TRUE(env.stop_feasible(q, -vmax, lim)) << "q=" << q;
  }
}
