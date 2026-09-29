// The gate on §20's imu block. These four cases are the difference between "a sensor is reporting"
// and "the station knows its own attitude": the first is measured from the BNO085 trace, the second
// needs a mount calibration the station does not have. A change that lets the second follow the
// first -- a present flag reused as an attitude claim -- fails here rather than on the dashboard.
#include <gtest/gtest.h>

#include "telemetry/telemetry.hpp"

namespace {

using ota::telemetry::fill_imu_telemetry;
using ota::telemetry::TelemetrySnapshot;

class ImuTelemetryTest : public ::testing::Test {};

TEST_F(ImuTelemetryTest, NoTraceMeansEveryFlagIsFalseRatherThanTheFieldDefault) {
  TelemetrySnapshot snap;
  snap.imu_world_elevation_deg = 3.5;  // junk left in the field must not survive a closed gate
  fill_imu_telemetry(/*samples_fresh=*/false, /*gravity_vector_fresh=*/false, snap);
  EXPECT_FALSE(snap.imu_present);
  EXPECT_FALSE(snap.imu_gravity_valid);
  EXPECT_FALSE(snap.imu_world_elevation_valid);
}

TEST_F(ImuTelemetryTest, FreshSamplesMakePresentTrueButDoNotManufactureAnAttitude) {
  TelemetrySnapshot snap;
  fill_imu_telemetry(/*samples_fresh=*/true, /*gravity_vector_fresh=*/true, snap);
  EXPECT_TRUE(snap.imu_present);
  EXPECT_TRUE(snap.imu_gravity_valid);
  EXPECT_FALSE(snap.imu_world_elevation_valid)
      << "the sensor is on the moving pitch assembly; without a mount calibration its gravity "
         "vector is the gimbal's attitude, not the base's";
}

TEST_F(ImuTelemetryTest, AStaleTraceCannotValidateANumberItWouldPublish) {
  TelemetrySnapshot snap;
  fill_imu_telemetry(/*samples_fresh=*/false, /*gravity_vector_fresh=*/true, snap);
  EXPECT_FALSE(snap.imu_present);
  EXPECT_FALSE(snap.imu_gravity_valid)
      << "gravity_valid on a trace nobody is reading would let the page render a number that is "
         "not being updated";
}

TEST_F(ImuTelemetryTest, TheClosedGateLeavesTheValueAloneBecauseTheGateIsTheContract) {
  TelemetrySnapshot snap;
  snap.imu_world_elevation_deg = 0.0;
  fill_imu_telemetry(/*samples_fresh=*/true, /*gravity_vector_fresh=*/true, snap);
  EXPECT_FALSE(snap.imu_world_elevation_valid);
  EXPECT_DOUBLE_EQ(snap.imu_world_elevation_deg, 0.0)
      << "with the gate closed the emitter must send null; the value is irrelevant either way, so "
         "the filler must not quietly rewrite it into a claim";
}

}  // namespace
