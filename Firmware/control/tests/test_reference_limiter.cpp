// Tests for the reference limiter, i.e. for "smooth" as a property of what the controller asks for
// rather than of what the motor manages to survive. The numbers asserted here come from the station
// configuration (30 deg/s while tracking, 60 deg/s^2), not from whatever the implementation happens
// to do: a 25 deg move at those limits is a trapezoid that takes 0.5 s to reach 30 deg/s, 0.333 s
// to cross the middle, and 0.5 s to stop again - 1.333 s. If someone later tunes the limiter to be
// cleverer, these tests say which promise is being broken.
#include "control/reference_limiter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kVMax = 30.0 * kDeg;   // tracking ceiling, config/turret.yaml
constexpr double kAMax = 60.0 * kDeg;   // yaw axis, config/turret.yaml
constexpr double kDt = 0.005;           // the 200 Hz control period

// Run the limiter until it reports arrival, or until the budget runs out.
double run_to_arrival(ota::control::ReferenceLimiter& st, double target, double v_max, double a_max,
                      double* max_rate_out, double* max_accel_out, double* max_overshoot_out) {
  double t = 0.0, max_rate = 0.0, max_accel = 0.0, max_over = 0.0;
  double prev_q = st.q_rad, prev_v = st.v_rad_s;
  // Overshoot is measured as travel PAST the target, in the direction of the move, so a 25 deg move
  // that ends at 25.004 deg reads +0.004 deg and a move that falls short reads 0.
  const double dir = (target >= st.q_rad) ? 1.0 : -1.0;
  for (int i = 0; i < 40000; ++i) {
    const double q = ota::control::limit_reference(st, target, kDt, v_max, a_max);
    t += kDt;
    const double v = (q - prev_q) / kDt;
    const double a = (v - prev_v) / kDt;
    max_rate = std::max(max_rate, std::abs(v));
    max_accel = std::max(max_accel, std::abs(a));
    max_over = std::max(max_over, (q - target) * dir);
    prev_q = q;
    prev_v = v;
    if (q == target && st.v_rad_s == 0.0) break;
  }
  if (max_rate_out) *max_rate_out = max_rate;
  if (max_accel_out) *max_accel_out = max_accel;
  if (max_overshoot_out) *max_overshoot_out = max_over;
  return t;
}

TEST(ReferenceLimiter, RespectsBothConfiguredLimitsOnAStep) {
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  double rate = 0.0, accel = 0.0;
  const double t = run_to_arrival(st, 25.0 * kDeg, kVMax, kAMax, &rate, &accel, nullptr);
  EXPECT_LE(rate, kVMax * 1.001) << "reference out-ran its own speed ceiling";
  EXPECT_LE(accel, kAMax * 1.001) << "reference demanded more acceleration than the axis has";
  // The trapezoid the configuration predicts: 1.333 s. Tolerated to 1.6 s for discretisation, and
  // floored at 1.2 s so a "fix" that simply stops limiting cannot pass by being faster.
  EXPECT_GE(t, 1.20);
  EXPECT_LE(t, 1.60);
}

TEST(ReferenceLimiter, DoesNotOvershootTheTarget) {
  // Requirement (b) names oscillation explicitly. A discrete brake can cross the target by at most
  // roughly one period of carried speed (2*a*dt^2 = 0.003 deg here), which is far below anything an
  // operator could see, but it is not exactly zero - so the bar is 0.01 deg, not zero. Claiming an
  // exact crossing-free result from a discretised integrator would be the same kind of lie as
  // claiming smoothness from a stepped reference.
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  double over = 0.0;
  run_to_arrival(st, 25.0 * kDeg, kVMax, kAMax, nullptr, nullptr, &over);
  EXPECT_LE(over, 0.01 * kDeg);
}

TEST(ReferenceLimiter, LandsExactlyRatherThanAsymptotically) {
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  run_to_arrival(st, -8.0 * kDeg, kVMax, kAMax, nullptr, nullptr, nullptr);
  EXPECT_NEAR(st.q_rad, -8.0 * kDeg, 1e-12);
  EXPECT_NEAR(st.v_rad_s, 0.0, 1e-12);
}

TEST(ReferenceLimiter, WithoutAnAccelLimitItStillRateLimitsAndDoesNotCross) {
  // a_max = 0 means "no acceleration figure configured for this axis". The honest fallback is a
  // plain rate limit that still refuses to cross the target, not the old step behaviour.
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  double rate = 0.0, over = 0.0;
  const double t = run_to_arrival(st, 10.0 * kDeg, kVMax, 0.0, &rate, nullptr, &over);
  EXPECT_LE(rate, kVMax * 1.001);
  EXPECT_LE(over, 10.0 * kDeg * 1e-9);
  EXPECT_NEAR(t, 10.0 * kDeg / kVMax, 0.05);  // pure slew: distance over speed
}

TEST(ReferenceLimiter, LosesSpeedAuthorityByStoppingRatherThanJumping) {
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  for (int i = 0; i < 40; ++i) ota::control::limit_reference(st, 20.0 * kDeg, kDt, kVMax, kAMax);
  const double q_before = st.q_rad;
  ASSERT_GT(st.v_rad_s, 0.0);

  // Envelope says there is no room to move this cycle. The reference must shed its speed, not
  // vanish onto the target and not fight the safety action by continuing forward.
  for (int i = 0; i < 200; ++i) ota::control::limit_reference(st, 20.0 * kDeg, kDt, 0.0, kAMax);
  EXPECT_LT(st.q_rad - q_before, 2.0 * kDeg) << "reference travelled too far with no speed authority";
  EXPECT_NEAR(st.v_rad_s, 0.0, 1e-12);
}

TEST(ReferenceLimiter, IgnoresADuplicatedTimestampInsteadOfIntegratingIt) {
  ota::control::ReferenceLimiter st;
  st.reset_at(1.0);
  const double before = ota::control::limit_reference(st, 2.0, 0.0, kVMax, kAMax);
  EXPECT_DOUBLE_EQ(before, 1.0);
  EXPECT_DOUBLE_EQ(st.q_rad, 1.0);
}

TEST(ReferenceLimiter, TracksAMovingTargetWithBoundedError) {
  // A target moving at 20 deg/s is inside the ceiling, so the reference should end up following it
  // at that speed with a small, bounded lag - not chasing a step and not oscillating around it.
  ota::control::ReferenceLimiter st;
  st.reset_at(0.0);
  double err_worst = 0.0;
  for (int i = 0; i < 8000; ++i) {  // 40 s at 5 ms
    const double target = 20.0 * kDeg * (i * kDt);
    const double q = ota::control::limit_reference(st, target, kDt, kVMax, kAMax);
    if (i > 600) err_worst = std::max(err_worst, std::abs(target - q));  // past the ramp-up
  }
  // A bound on the TRANSIENT, not on the peak at all times. A target that starts moving instantly
  // cannot be followed with zero lag by anything that limits its own acceleration - the shortfall is
  // physics, not a defect, and a test demanding otherwise would be measuring the wrong thing. What
  // must hold is that the shortfall stays bounded (v^2/2a = 3.3 deg here) and that it CLOSES, which
  // is the part the first implementation got wrong: it settled at that shortfall forever.
  EXPECT_LE(err_worst, 5.0 * kDeg) << "lag while following a moving target grew without bound";
  double err_end = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double target = 20.0 * kDeg * (8000 * kDt + i * kDt);
    const double q = ota::control::limit_reference(st, target, kDt, kVMax, kAMax);
    err_end = std::abs(target - q);
  }
  // The bar is two control periods, and the reason is arithmetic rather than taste: at 20 deg/s one
  // 5 ms period of target travel is 0.100 deg, and the measured residual was 0.0975 deg - that is
  // one period of lag, which is what a discrete profile owes a continuous target. Demanding less
  // than a sampling period would not be a tighter requirement, it would be a demand for a different
  // sample rate, and it would fail for reasons that have nothing to do with the limiter.
  EXPECT_LE(err_end, 2.0 * 20.0 * kDeg * kDt) << "the reference never caught up with a target it could outrun";
}

}  // namespace
