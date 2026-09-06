#include <gtest/gtest.h>
#include "control/speed_servo.hpp"
#include "control/boundary_governor.hpp"

using namespace ota;
TEST(BoundaryGovernor, BothDirectionsStopInsideReserveDespiteMotorLag) {
  AxisLimits limits; limits.set_from_endpoints(-40*kDeg2Rad,40*kDeg2Rad,5*kDeg2Rad);
  control::BoundaryGovernor governor;
  for (double lag : {.05,.15}) for (double sign : {-1.,1.}) {
    control::SpeedServo servo;
    double q=0,v=0;
    for(int i=0;i<12000;++i) {
      const auto b=governor.at(q,limits,20*kDeg2Rad,servo.acceleration,v);
      double cmd=servo.step(sign*45*kDeg2Rad,sign*20*kDeg2Rad,q,20*kDeg2Rad,.005,
          governor.acceleration,governor.jerk,b.negative_acceleration_scale,b.positive_acceleration_scale);
      cmd=std::clamp(cmd,-b.negative_speed,b.positive_speed);
      servo.velocity=cmd;
      v+=(cmd-v)*(1-std::exp(-.005/lag)); q+=v*.005;
      ASSERT_GE(limits.distance_to_soft(q),governor.margin-1e-5);
    }
    EXPECT_NEAR(limits.distance_to_soft(q),governor.margin,1e-4);
    // An inward request must remain possible at the stopped endpoint.
    const auto b=governor.at(q,limits,20*kDeg2Rad);
    EXPECT_GT(sign>0 ? b.negative_speed : b.positive_speed,19*kDeg2Rad);
  }
}
TEST(SpeedServo, OutwardTaperDoesNotReduceBrakingAuthority) {
  control::SpeedServo servo;
  servo.velocity=10*kDeg2Rad;
  for(int i=0;i<80;++i)
    servo.step(-1,-.1,0,20*kDeg2Rad,.005,30*kDeg2Rad,120*kDeg2Rad,0,0);
  EXPECT_LT(servo.velocity,3*kDeg2Rad);
  EXPECT_LT(servo.acceleration,-29*kDeg2Rad);
}
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
