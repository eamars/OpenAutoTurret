// Unit tests for the payload response metrics (architecture §44 "Measure").
// Pure time-series analysis — synthetic trajectories, no plant, no I/O.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "payload/response_metrics.hpp"

namespace {

using ota::payload::BrakeMetrics;
using ota::payload::ResponseSample;
using ota::payload::StepMetrics;
using ota::payload::analyze_brake;
using ota::payload::analyze_step;
using ota::payload::rms_tracking_error;

constexpr double kDt = 0.005;  // 200 Hz
constexpr double kAmp = 0.05;  // 3 deg step amplitude (rad)

// Sample a first-order position step q(t) = A(1 - e^(-t/tau)) at 200 Hz.
std::vector<ResponseSample> first_order_step(double tau, double t_end_s,
                                             double effort = 1.0) {
  std::vector<ResponseSample> s;
  const int n = static_cast<int>(t_end_s / kDt);
  for (int i = 0; i <= n; ++i) {
    const double t = i * kDt;
    const double q = kAmp * (1.0 - std::exp(-t / tau));
    s.push_back({
        static_cast<int64_t>(t * 1e9), q, kAmp / tau * std::exp(-t / tau),
        effort});
  }
  return s;
}

// Damped second-order step: overshoots the target.
std::vector<ResponseSample> damped_step(double zeta, double omega_n,
                                        double t_end_s) {
  std::vector<ResponseSample> s;
  const double wd = omega_n * std::sqrt(1.0 - zeta * zeta);
  const int n = static_cast<int>(t_end_s / kDt);
  for (int i = 0; i <= n; ++i) {
    const double t = i * kDt;
    const double q =
        kAmp * (1.0 - std::exp(-zeta * omega_n * t) *
                           std::cos(wd * t));
    s.push_back({static_cast<int64_t>(t * 1e9), q, 0.0, 0.5});
  }
  return s;
}

}  // namespace

TEST(ResponseMetrics, FirstOrderStepRiseAndSettle) {
  const double tau = 0.05;
  // Run well past settling (2 % band) so both times are interior crossings.
  const auto s = first_order_step(tau, 1.0);
  const StepMetrics m = analyze_step(s, 0.0, kAmp, 0.02);
  ASSERT_TRUE(m.valid);
  // 10-90 % rise of a first order: tau * ln(9) = 0.110 s.
  EXPECT_NEAR(m.rise_time_s, tau * std::log(9.0), 0.01);
  // Settling in a 2 % band: tau * ln(50) = 0.196 s.
  EXPECT_NEAR(m.settling_time_s, tau * std::log(50.0), 0.015);
  EXPECT_GE(m.overshoot, 0.0);
  EXPECT_LT(m.overshoot, 1e-6);  // no overshoot in a first-order response
  EXPECT_GT(m.peak_effort_nm, 0.0);
  // Settled tail tracks the target tightly.
  EXPECT_LT(m.tracking_rms_rad, 0.02 * kAmp);
}

TEST(ResponseMetrics, DampedStepShowsOvershoot) {
  const auto s = damped_step(0.5, 20.0, 1.0);
  const StepMetrics m = analyze_step(s, 0.0, kAmp, 0.02);
  ASSERT_TRUE(m.valid);
  // zeta=0.5 -> ~16 % overshoot; the analysis must catch it.
  EXPECT_GT(m.overshoot, 0.10);
  EXPECT_LT(m.overshoot, 0.25);
}

TEST(ResponseMetrics, NegativeStepNormalizesDirection) {
  const auto s = first_order_step(0.05, 1.0);
  // q0 -> q0 - A: the same trajectory sampled from the other side.
  std::vector<ResponseSample> neg;
  for (const auto& x : s)
    neg.push_back({x.t_ns, kAmp - x.q_rad, -x.v_rad_s, x.effort_nm});
  const StepMetrics m = analyze_step(neg, kAmp, 0.0, 0.02);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.rise_time_s, 0.05 * std::log(9.0), 0.01);
  EXPECT_LT(m.overshoot, 1e-6);
}

TEST(ResponseMetrics, NonConvergingMoveReportsWorstCase) {
  // A move that stalls below half the amplitude: invalid, but with a
  // defined (full-duration) rise/settling so a baseline comparison fails
  // safe instead of comparing zeros.
  std::vector<ResponseSample> s;
  for (int i = 0; i < 100; ++i) {
    const double t = i * kDt;
    s.push_back({static_cast<int64_t>(t * 1e9), 0.4 * kAmp, 0.0, 0.2});
  }
  const StepMetrics m = analyze_step(s, 0.0, kAmp, 0.02);
  EXPECT_FALSE(m.valid);
  // No 10 % crossing was ever made -> the full capture duration is the
  // reported (worst-case) rise and settling time.
  const double dur = 99 * kDt;
  EXPECT_NEAR(m.rise_time_s, dur, 1e-9);
  EXPECT_NEAR(m.settling_time_s, dur, 1e-9);
}

TEST(ResponseMetrics, ShortSampleTrainIsInvalid) {
  // Step analysis needs >= 5 samples.
  std::vector<ResponseSample> s4(4);
  for (int i = 0; i < 4; ++i) s4[i] = {i * 5'000'000, 0.0, 0.0, 0.0};
  const StepMetrics m = analyze_step(s4, 0.0, kAmp);
  EXPECT_FALSE(m.valid);
  // Brake analysis needs >= 3 samples.
  std::vector<ResponseSample> s2(2);
  for (int i = 0; i < 2; ++i) s2[i] = {i * 5'000'000, 0.0, 0.3, 0.0};
  const BrakeMetrics b = analyze_brake(s2, 0);
  EXPECT_FALSE(b.valid);
}

TEST(ResponseMetrics, BrakeStopDistanceIsV0Tau) {
  // Constant approach v0, first-order decay after the brake command:
  // stop distance -> v0 * tau as the at-rest threshold goes to zero.
  const double v0 = 0.3;
  const double tau = 0.05;
  std::vector<ResponseSample> s;
  for (int i = 0; i <= 400; ++i) {
    const double t = i * kDt;
    s.push_back({static_cast<int64_t>(t * 1e9),
                 v0 * tau * (1.0 - std::exp(-t / tau)),
                 v0 * std::exp(-t / tau), 0.0});
  }
  const BrakeMetrics m = analyze_brake(s, 0, 0.01);
  ASSERT_TRUE(m.valid);
  EXPECT_NEAR(m.v0_rad_s, v0, 0.01);
  EXPECT_GT(m.stop_distance_rad, 0.8 * v0 * tau);
  EXPECT_LT(m.stop_distance_rad, 1.2 * v0 * tau);
  EXPECT_GT(m.stop_time_s, 0.0);
}

TEST(ResponseMetrics, BrakeStillMovingIsInvalid) {
  // No at-rest sample after the command.
  std::vector<ResponseSample> s;
  for (int i = 0; i < 20; ++i)
    s.push_back({i * 5'000'000, i * 0.001, 0.3, 0.0});
  const BrakeMetrics m = analyze_brake(s, 0, 0.01);
  EXPECT_FALSE(m.valid);
}

TEST(ResponseMetrics, RmsTrackingError) {
  std::vector<ResponseSample> s;
  for (int i = 0; i < 10; ++i) s.push_back({i * 5'000'000, 0.001, 0.0, 0.0});
  EXPECT_NEAR(rms_tracking_error(s, 0.0), 0.001, 1e-12);
  EXPECT_NEAR(rms_tracking_error(s, 0.001), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(rms_tracking_error({}, 0.0), 0.0);
}
