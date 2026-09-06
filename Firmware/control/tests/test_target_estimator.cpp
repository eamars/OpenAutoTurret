// Unit tests for the target-state estimator (architecture §13): constant-
// velocity Kalman filter on base-frame LOS angles + forward prediction.
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
    est.update(wrap_angle(3.0 + rate * t), 0.0, static_cast<int64_t>(t * 1e9));
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
  EXPECT_EQ(est.azimuth(), 0.0);
  EXPECT_EQ(est.elevation_rate(), 0.0);
}

TEST(TargetEstimator, RejectsOldDuplicateAndNonfiniteMeasurementsWithoutChangingState) {
  TargetEstimator est;
  ASSERT_TRUE(est.update(.1, .2, 1'000'000'000));
  ASSERT_TRUE(est.update(.12, .21, 1'060'000'000));
  const auto stamp = est.state_timestamp_ns();
  const double rate = est.azimuth_rate();
  EXPECT_FALSE(est.update(-.9, .3, 1'000'000'000));
  EXPECT_FALSE(est.update(.4, .3, stamp));
  EXPECT_FALSE(est.update(INFINITY, .3, stamp + 1));
  EXPECT_FALSE(est.update(.3, NAN, stamp + 1));
  EXPECT_EQ(est.state_timestamp_ns(), stamp);
  EXPECT_EQ(est.azimuth_rate(), rate);
}

TEST(TargetEstimator, QueriesDoNotAdvanceStateAndLongLossInvalidatesPrediction) {
  TargetEstimator est;
  est.update(0, 0, 1'000'000'000);
  est.update(.02, .01, 1'060'000'000);
  const auto stamp = est.state_timestamp_ns();
  double az, el;
  for (int i = 0; i < 40; ++i) est.predict(stamp + i * 5'000'000, az, el);
  EXPECT_EQ(est.state_timestamp_ns(), stamp);
  EXPECT_TRUE(est.prediction_valid(stamp + 100'000'000));
  EXPECT_FALSE(est.prediction_valid(stamp + 1'000'000'000));
  est.predict(stamp + 20'000'000'000'000LL, az, el);
  EXPECT_TRUE(std::isfinite(az));
  EXPECT_LE(std::fabs(el), M_PI / 2);
}

TEST(TargetEstimator, WrapAngle) {
  EXPECT_NEAR(wrap_angle(3.0 * kPi), kPi, 1e-12);       // -> +pi
  EXPECT_NEAR(wrap_angle(-3.0 * kPi), -kPi, 1e-12);     // -> -pi
  EXPECT_NEAR(wrap_angle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(wrap_angle(2.0 * kPi + 0.3), 0.3, 1e-12);
}

TEST(TargetEstimator, CovarianceGrowsDuringCoastAndOutlierDoesNotRefreshCapture) {
  TargetEstimator est;
  for (int i = 0; i < 90; ++i)
    ASSERT_TRUE(est.update(.1, .2, 1'000'000'000LL + i*40'000'000LL));
  const auto stamp = est.state_timestamp_ns();
  const auto variance = est.position_variance(0, stamp);
  EXPECT_GT(variance, 0);
  EXPECT_GT(est.position_variance(0, stamp+200'000'000), variance);
  EXPECT_FALSE(est.update(1.8, -.5, stamp+40'000'000));
  EXPECT_EQ(est.state_timestamp_ns(), stamp);
  EXPECT_GT(est.diagnostics().mahalanobis, 9.21);
  EXPECT_FALSE(est.diagnostics().last_accepted);
  EXPECT_TRUE(est.update(.1, .2, stamp+80'000'000));
  EXPECT_EQ(est.diagnostics().rejected, 1u);
}

TEST(TargetEstimator, UncertainObservationsProduceLessCorrectionAndMoreUncertainty) {
  TargetEstimator precise, uncertain;
  for (int i = 0; i < 90; ++i) {
    precise.update(0, 0, i*40'000'000LL);
    uncertain.update(0, 0, i*40'000'000LL);
  }
  ASSERT_TRUE(precise.update(.003, 0, 90*40'000'000LL));
  ASSERT_TRUE(uncertain.update(.003, 0, 90*40'000'000LL, .01, .01));
  EXPECT_LT(uncertain.azimuth(), precise.azimuth());
  EXPECT_GT(uncertain.position_variance(0, 90*40'000'000LL),
            precise.position_variance(0, 90*40'000'000LL));
}

TEST(TargetEstimator, LongGapReacquisitionDoesNotCarryOldVelocity) {
  TargetEstimator est;
  for (int i = 0; i < 90; ++i) est.update(.01*i, 0, 1'000'000'000LL+i*40'000'000LL);
  EXPECT_GT(est.azimuth_rate(), .2);
  ASSERT_TRUE(est.update(.1, .2, 8'000'000'000LL));
  EXPECT_EQ(est.azimuth_rate(), 0);
  EXPECT_EQ(est.diagnostics().gap_resets, 1u);
}

}  // namespace
