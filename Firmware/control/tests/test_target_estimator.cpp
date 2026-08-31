// Unit tests for the target-state estimator (architecture §13): constant-
// velocity alpha-beta filter on base-frame LOS angles + forward prediction.
// Pure filtering — no camera, no CAN, no motor driver.
#include <gtest/gtest.h>

#include <cmath>

#include "tracking/target_estimator.hpp"

namespace {
using ota::tracking::TargetEstimator;
using ota::tracking::TargetEstimatorConfig;
using ota::tracking::wrap_angle;

constexpr double kPi = M_PI;

TEST(TargetEstimator, TracksConstantPosition) {
  TargetEstimator est;
  const double az = 0.3, el = 0.1;
  for (int i = 0; i < 100; ++i) est.update(az, el, static_cast<std::int64_t>(i) * 33333333);  // ~30 Hz
  EXPECT_NEAR(est.azimuth(), az, 1e-6);
  EXPECT_NEAR(est.elevation(), el, 1e-6);
  EXPECT_NEAR(est.azimuth_rate(), 0.0, 1e-6);
  EXPECT_NEAR(est.elevation_rate(), 0.0, 1e-6);
}

TEST(TargetEstimator, TracksConstantVelocityAndPredicts) {
  TargetEstimator est;
  const double az0 = 0.0, el0 = 0.0;
  const double rate = 0.05;  // rad/s
  // Feed 2 s of a target rotating at `rate` (30 Hz).
  for (int i = 0; i < 60; ++i) {
    const double t = i / 30.0;
    est.update(az0 + rate * t, el0, static_cast<std::int64_t>(t * 1e9));
  }
  // Velocity has converged.
  EXPECT_NEAR(est.azimuth_rate(), rate, 1e-4);
  EXPECT_NEAR(est.elevation_rate(), 0.0, 1e-6);

  // Predict 0.2 s ahead: the target has moved rate*0.2 further.
  const double t_last = 59.0 / 30.0;
  const int64_t t_predict = static_cast<int64_t>((t_last + 0.2) * 1e9);
  double az_pred, el_pred;
  est.predict(t_predict, az_pred, el_pred);
  const double expected = wrap_angle(az0 + rate * (t_last + 0.2));
  EXPECT_NEAR(az_pred, expected, 1e-4);
  EXPECT_NEAR(el_pred, el0, 1e-6);
}

TEST(TargetEstimator, PredictionAheadIsBetterThanHold) {
  // For a moving target, predicting forward should land closer to the target's
  // future position than holding the last measurement (the point of §13.3).
  TargetEstimator est;
  const double rate = 0.08;
  int64_t t_last = 0;
  for (int i = 0; i < 90; ++i) {
    const double t = i / 30.0;
    est.update(rate * t, 0.0, static_cast<std::int64_t>(t * 1e9));
    t_last = static_cast<int64_t>(t * 1e9);
  }
  const double t_future = 3.0;  // 0.2 s after the last sample at 2.93 s
  const double true_az = rate * t_future;

  double hold_az, hold_el;
  est.predict(t_last, hold_az, hold_el);  // "no prediction" = last state
  double pred_az, pred_el;
  est.predict(static_cast<int64_t>(t_future * 1e9), pred_az, pred_el);

  const double err_hold = std::fabs(wrap_angle(true_az - hold_az));
  const double err_pred = std::fabs(wrap_angle(true_az - pred_az));
  EXPECT_LT(err_pred, err_hold);  // prediction beats hold for a moving target
}

TEST(TargetEstimator, HandlesAngleWrap) {
  // A target that sweeps across the +-pi boundary must not cause a big jump in
  // the rate estimate (the innovation is wrapped).
  TargetEstimator est;
  const double rate = 0.5;  // rad/s, fast
  for (int i = 0; i < 120; ++i) {
    const double t = i / 30.0;
    est.update(wrap_angle(rate * t), 0.0, static_cast<int64_t>(t * 1e9));
  }
  // Rate estimate stays close to the true rate despite the wrap.
  EXPECT_NEAR(est.azimuth_rate(), rate, 5e-2);
}

TEST(TargetEstimator, ResetClearsState) {
  TargetEstimator est;
  est.update(0.5, 0.2, 1000);
  EXPECT_TRUE(est.initialized());
  est.reset();
  EXPECT_FALSE(est.initialized());
}

TEST(TargetEstimator, WrapAngle) {
  EXPECT_NEAR(wrap_angle(3.0 * kPi), kPi, 1e-12);       // -> +pi
  EXPECT_NEAR(wrap_angle(-3.0 * kPi), -kPi, 1e-12);     // -> -pi
  EXPECT_NEAR(wrap_angle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(wrap_angle(2.0 * kPi + 0.3), 0.3, 1e-12);
}

}  // namespace
