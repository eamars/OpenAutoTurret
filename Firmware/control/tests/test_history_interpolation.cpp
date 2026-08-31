// Unit tests for camera/motor timestamp alignment (architecture §11.1): linear
// interpolation of the motor history at the camera capture timestamp. This is a
// core requirement — a measurement arriving at t_now describes a frame captured
// at t_frame, so the pose at t_frame (not the current pose) must be used.
#include <gtest/gtest.h>

#include "common/motor_state_history.hpp"

namespace {
using ota::MotorSample;
using ota::MotorStateHistory;

TEST(HistoryInterpolation, LinearBetweenTwoSamples) {
  MotorStateHistory hist(16);
  hist.add(0, 0.0f, 1.0f);
  hist.add(1000000000, 1.0f, 1.0f);  // 1 s later, q = 1 rad
  MotorSample out;
  ASSERT_TRUE(hist.interpolate(500000000, out));  // midpoint
  EXPECT_NEAR(out.q, 0.5, 1e-6);
  EXPECT_NEAR(out.v, 1.0, 1e-6);
}

TEST(HistoryInterpolation, ExactAtSampleTime) {
  MotorStateHistory hist(16);
  hist.add(0, 0.1f, 0.5f);
  hist.add(1000000000, 0.9f, 1.5f);
  MotorSample out;
  ASSERT_TRUE(hist.interpolate(1000000000, out));
  EXPECT_NEAR(out.q, 0.9, 1e-6);
  EXPECT_NEAR(out.v, 1.5, 1e-6);
}

TEST(HistoryInterpolation, ReturnsFalseBeforeFirst) {
  MotorStateHistory hist(16);
  hist.add(1000, 0.0f, 0.0f);
  hist.add(2000, 1.0f, 0.0f);
  MotorSample out;
  EXPECT_FALSE(hist.interpolate(999, out));
}

TEST(HistoryInterpolation, ReturnsFalseAfterLast) {
  MotorStateHistory hist(16);
  hist.add(1000, 0.0f, 0.0f);
  hist.add(2000, 1.0f, 0.0f);
  MotorSample out;
  EXPECT_FALSE(hist.interpolate(2001, out));
}

TEST(HistoryInterpolation, ReturnsFalseWhenTooFewSamples) {
  MotorStateHistory hist(16);
  hist.add(1000, 0.0f, 0.0f);  // only one sample
  MotorSample out;
  EXPECT_FALSE(hist.interpolate(1000, out));
}

TEST(HistoryInterpolation, MultiSampleInterpolation) {
  MotorStateHistory hist(16);
  // q(t) = t (in seconds) -> q in rad, v = 1 rad/s.
  for (int i = 0; i <= 10; ++i) {
    hist.add(static_cast<int64_t>(i * 100000000LL), static_cast<float>(i * 0.1f), 1.0f);
  }
  MotorSample out;
  // Interpolate at t = 4.5 s -> q = 0.45 rad.
  ASSERT_TRUE(hist.interpolate(450000000, out));
  EXPECT_NEAR(out.q, 0.45, 1e-5);
  EXPECT_NEAR(out.v, 1.0, 1e-6);
}

}  // namespace
