// Unit tests for the world-frame telemetry bridge (Phase 7, §29/§30/§42.1):
// R_W_B_to_euler and fill_world_frame_telemetry. A tilted base must report the
// correct tilt and map a base-frame LOS to the correct world-frame LOS. Pure
// data — no CAN, no camera, no motor driver.
#include <gtest/gtest.h>

#include <cmath>

#include "calibration/installation_pose.hpp"
#include "calibration/world_frame_telemetry.hpp"

namespace {

using ota::BaseOrientation;
using ota::PoseSource;
using ota::R_W_B_to_euler;
using ota::fill_world_frame_telemetry;
using ota::geo::Mat3;
using ota::kDeg2Rad;
using ota::telemetry::TelemetrySnapshot;

constexpr double kTol = 1e-9;

class WorldFrameTelemetryTest : public ::testing::Test {};

TEST_F(WorldFrameTelemetryTest, EulerIdentityIsLevel) {
  Mat3 R;  // identity
  double roll, pitch, yaw;
  R_W_B_to_euler(R, roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, kTol);
  EXPECT_NEAR(pitch, 0.0, kTol);
  EXPECT_NEAR(yaw, 0.0, kTol);
}

TEST_F(WorldFrameTelemetryTest, EulerPitchTilt) {
  const double pitch = 20.0 * kDeg2Rad;
  const Mat3 R = Mat3::rot_y(pitch);
  double roll, p, yaw;
  R_W_B_to_euler(R, roll, p, yaw);
  EXPECT_NEAR(p, pitch, 1e-9);
  EXPECT_NEAR(roll, 0.0, 1e-9);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

TEST_F(WorldFrameTelemetryTest, EulerRollTilt) {
  const double roll = -15.0 * kDeg2Rad;
  const Mat3 R = Mat3::rot_x(roll);
  double r, pitch, yaw;
  R_W_B_to_euler(R, r, pitch, yaw);
  EXPECT_NEAR(r, roll, 1e-9);
  EXPECT_NEAR(pitch, 0.0, 1e-9);
  EXPECT_NEAR(yaw, 0.0, 1e-9);
}

TEST_F(WorldFrameTelemetryTest, EulerCombinedZYX) {
  const double roll = 8.0 * kDeg2Rad, pitch = -12.0 * kDeg2Rad,
               yaw = 30.0 * kDeg2Rad;
  const Mat3 R = Mat3::rot_z(yaw) * Mat3::rot_y(pitch) * Mat3::rot_x(roll);
  double r, p, y;
  R_W_B_to_euler(R, r, p, y);
  EXPECT_NEAR(r, roll, 1e-6);
  EXPECT_NEAR(p, pitch, 1e-6);
  EXPECT_NEAR(y, yaw, 1e-6);
}

TEST_F(WorldFrameTelemetryTest, SnapshotFillsTiltedBase) {
  // A base pitched down 10 deg (top tilts toward +X). The base-forward ray
  // (az=0, el=0 in the base frame) then points 10 deg below the world horizon.
  BaseOrientation o;
  o.R_W_B = Mat3::rot_y(10.0 * kDeg2Rad);
  o.source = PoseSource::VisualCalibration;
  o.valid = true;
  o.covariance = 1e-4;
  o.n_frames = 24;

  TelemetrySnapshot s;
  fill_world_frame_telemetry(o, /*az_base=*/0.0, /*el_base=*/0.0, s);

  EXPECT_NEAR(s.base_pitch_rad, 10.0 * kDeg2Rad, 1e-9);
  EXPECT_NEAR(s.base_roll_rad, 0.0, 1e-9);
  EXPECT_NEAR(s.base_yaw_rad, 0.0, 1e-9);
  EXPECT_TRUE(s.installation_calibrated);
  EXPECT_EQ(s.installation_source, static_cast<int8_t>(PoseSource::VisualCalibration));
  // Forward ray points 10 deg below the world horizon.
  EXPECT_NEAR(s.target_el_world_rad, -10.0 * kDeg2Rad, 1e-6);
  EXPECT_NEAR(s.target_az_world_rad, 0.0, 1e-6);
}

TEST_F(WorldFrameTelemetryTest, SnapshotIdentityIsUncalibrated) {
  BaseOrientation o;  // identity
  o.source = PoseSource::Identity;
  o.valid = true;
  o.covariance = 1e6;

  TelemetrySnapshot s;
  fill_world_frame_telemetry(o, 0.4, 0.2, s);

  EXPECT_FALSE(s.installation_calibrated);
  EXPECT_EQ(s.installation_source, static_cast<int8_t>(PoseSource::Identity));
  // Identity base: the world LOS equals the base LOS.
  EXPECT_NEAR(s.target_az_world_rad, 0.4, kTol);
  EXPECT_NEAR(s.target_el_world_rad, 0.2, kTol);
}

}  // namespace
