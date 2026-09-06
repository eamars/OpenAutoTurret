#include <gtest/gtest.h>
#include "control/speed_servo.hpp"

using namespace ota;
TEST(SpeedServo, EncoderNoiseDoesNotChaseAStationaryReference) {
  control::SpeedServo servo;
  for (int i=0; i<200; ++i)
    EXPECT_DOUBLE_EQ(servo.step(0, 0, (i%2 ? .04 : -.04)*kDeg2Rad,
        3*kDeg2Rad, .005, kDeg2Rad*60, kDeg2Rad*300), 0);
}
TEST(SpeedServo, MovingReferenceReleasesQuietHoldAndObeysSpeedAndAccelerationBounds) {
  control::SpeedServo servo;
  servo.step(0, 0, 0, 1, .005, 1, 5);
  double prior=0;
  for (int i=0; i<200; ++i) {
    const double velocity=servo.step(.1, .05, 0, .05, .005, 1, 5);
    EXPECT_LE(std::abs(velocity), .05);
    EXPECT_LE(std::abs(velocity-prior), .005+1e-12);
    prior=velocity;
  }
  EXPECT_GT(prior, 0);
  EXPECT_FALSE(servo.quiet);
  EXPECT_DOUBLE_EQ(servo.step(.1, .05, 0, 0, .005, 1, 5), 0);
}
