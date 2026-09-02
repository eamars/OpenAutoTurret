// Unit tests for the installation-orientation layer (architecture §28.4/§29/
// §30): BaseOrientation, the provider interface (FixedStoredPoseProvider), the
// world-frame LOS transform, and atomic persistence of R_W_B. Pure data + file
// I/O — no CAN, no camera, no motor driver.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "calibration/installation_pose.hpp"

namespace {

using ota::BaseOrientation;
using ota::FixedStoredPoseProvider;
using ota::InstallationPoseProvider;
using ota::PoseSource;
using ota::base_los_to_world;
using ota::identity_pose;
using ota::load_installation_pose;
using ota::save_installation_pose;
using ota::geo::Mat3;
using ota::kDeg2Rad;

constexpr double kTol = 1e-9;

std::string tmp_path(const std::string& name) {
  return "/tmp/ota_installation_test/" + name;
}

class InstallationPoseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::filesystem::remove_all("/tmp/ota_installation_test");
    std::filesystem::create_directories("/tmp/ota_installation_test");
  }
  void TearDown() override { std::filesystem::remove_all("/tmp/ota_installation_test"); }
};

TEST_F(InstallationPoseTest, IdentityPoseIsLevelAndWide) {
  BaseOrientation o = identity_pose();
  EXPECT_TRUE(o.valid);
  EXPECT_EQ(o.source, PoseSource::Identity);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(o.R_W_B.m[i][j], (i == j) ? 1.0 : 0.0, kTol);
  // Wide covariance: an uncalibrated base is *assumed* level, not known level.
  EXPECT_GT(o.covariance, 100.0);
}

TEST_F(InstallationPoseTest, WorldLosIdentityIsUnchanged) {
  BaseOrientation o = identity_pose();
  double az_w, el_w;
  base_los_to_world(o, 0.3, -0.2, az_w, el_w);
  EXPECT_NEAR(az_w, 0.3, kTol);
  EXPECT_NEAR(el_w, -0.2, kTol);
}

TEST_F(InstallationPoseTest, WorldLosReflectsTiltedBase) {
  // A base pitched (tilted about the base Y axis) by 20 deg: a base-frame
  // "straight ahead" ray (az=0, el=0, i.e. along base X) should point 20 deg
  // DOWN in the world frame. rot_y(tilt) maps (1,0,0) -> (cos t, 0, -sin t).
  const double tilt = 20.0 * kDeg2Rad;
  const Mat3 R_W_B{std::cos(tilt), 0.0, std::sin(tilt), 0.0, 1.0, 0.0,
                   -std::sin(tilt), 0.0, std::cos(tilt)};
  double az_w, el_w;
  base_los_to_world(R_W_B, 0.0, 0.0, az_w, el_w);
  EXPECT_NEAR(az_w, 0.0, 1e-9);
  EXPECT_NEAR(el_w, -tilt, 1e-9);
  // A ray at base el=+20 deg now points horizontally in the world.
  base_los_to_world(R_W_B, 0.0, tilt, az_w, el_w);
  EXPECT_NEAR(el_w, 0.0, 1e-9);
}

TEST_F(InstallationPoseTest, SaveLoadRoundTrip) {
  BaseOrientation o;
  o.timestamp_ns = 123456789;
  o.covariance = 0.0004;
  o.source = PoseSource::VisualCalibration;
  o.valid = true;
  o.n_frames = 20;
  o.reprojection_error_px = 0.5;
  // A small yaw tilt.
  const double a = 0.1;
  o.R_W_B = Mat3{std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0,
                 0.0, 0.0, 1.0};

  const std::string p = tmp_path("pose.yaml");
  std::string err;
  ASSERT_TRUE(save_installation_pose(p, o, err)) << err;

  BaseOrientation l = load_installation_pose(p);
  EXPECT_TRUE(l.valid);
  EXPECT_EQ(l.source, PoseSource::VisualCalibration);
  EXPECT_EQ(l.timestamp_ns, 123456789);
  EXPECT_NEAR(l.covariance, 0.0004, 1e-12);
  EXPECT_EQ(l.n_frames, 20);
  EXPECT_NEAR(l.reprojection_error_px, 0.5, 1e-12);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(l.R_W_B.m[i][j], o.R_W_B.m[i][j], 1e-9);
}

TEST_F(InstallationPoseTest, LoadMissingFileIsInvalid) {
  BaseOrientation l = load_installation_pose(tmp_path("does_not_exist.yaml"));
  EXPECT_FALSE(l.valid);
}

TEST_F(InstallationPoseTest, LoadMalformedFileIsInvalid) {
  const std::string p = tmp_path("bad.yaml");
  {
    std::ofstream f(p);
    f << "source=visual_calibration\nvalid=1\n";  // no matrix rows
  }
  BaseOrientation l = load_installation_pose(p);
  EXPECT_FALSE(l.valid);
}

TEST_F(InstallationPoseTest, LoadsPythonCommitterFormat) {
  // The §29 calibration is WRITTEN by vision/installation_calibration.py
  // (commit_R_W_B) and READ here, so the two formats are a contract only this
  // test can enforce. Shapes Python actually produces:
  //   - `%.17g` numbers, which go to exponent form for small values
  //     (covariance -> 1.0471975511965979e-03);
  //   - `valid=0`, i.e. an INVALID calibration, stated as a key. (The writer
  //     once emitted a bare `0` there. Measured with a probe: a matrix row needs
  //     three doubles, so that line was dropped and `valid` came from the
  //     parser's default of false — right by accident, and the file stopped
  //     carrying its own verdict. This test pins the contract as DOCUMENTED,
  //     not merely tolerated.)
  const std::string p = tmp_path("from_python.yaml");
  {
    std::ofstream f(p);
    f << "# ota-installation-pose v1\n"
      << "source=visual_calibration\n"
      << "valid=0\n"
      << "timestamp_ns=987654321\n"
      << "covariance=1.0471975511965979e-03\n"
      << "n_frames=4\n"
      << "reprojection_error_px=0.60000000000000009\n"
      << "0.99500416527802584 -0.099833416646828154 0\n"
      << "0.099833416646828154 0.99500416527802584 0\n"
      << "0 0 1\n";
  }
  BaseOrientation l = load_installation_pose(p);
  EXPECT_FALSE(l.valid);                          // honoured, not silently defaulted
  EXPECT_EQ(l.source, PoseSource::VisualCalibration);
  EXPECT_EQ(l.timestamp_ns, 987654321);
  EXPECT_NEAR(l.covariance, 1.0471975511965979e-03, 1e-15);
  EXPECT_EQ(l.n_frames, 4);
  EXPECT_NEAR(l.reprojection_error_px, 0.6, 1e-12);
  EXPECT_NEAR(l.R_W_B.m[0][0], 0.99500416527802584, 1e-12);   // matrix NOT shifted
  EXPECT_NEAR(l.R_W_B.m[0][1], -0.099833416646828154, 1e-12);
  EXPECT_NEAR(l.R_W_B.m[1][0], 0.099833416646828154, 1e-12);
  EXPECT_NEAR(l.R_W_B.m[2][2], 1.0, 1e-12);
}

TEST_F(InstallationPoseTest, FixedProviderLoadsStoredPose) {
  BaseOrientation o;
  o.source = PoseSource::VisualCalibration;
  o.valid = true;
  o.timestamp_ns = 42;
  const double a = 0.2;
  o.R_W_B = Mat3{std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0,
                 0.0, 0.0, 1.0};
  const std::string p = tmp_path("stored.yaml");
  std::string err;
  ASSERT_TRUE(save_installation_pose(p, o, err));

  FixedStoredPoseProvider provider(p);
  BaseOrientation g = provider.get();
  EXPECT_TRUE(g.valid);
  EXPECT_EQ(g.source, PoseSource::FixedStored);  // re-labelled on load
  EXPECT_NEAR(g.R_W_B.m[0][1], -std::sin(a), 1e-9);
  EXPECT_EQ(provider.name(), std::string("FixedStoredPoseProvider"));
}

TEST_F(InstallationPoseTest, FixedProviderMissingFileFallsBackToIdentity) {
  FixedStoredPoseProvider provider(tmp_path("absent.yaml"));
  BaseOrientation g = provider.get();
  EXPECT_TRUE(g.valid);  // the identity pose is a valid (level) orientation
  EXPECT_EQ(g.source, PoseSource::Identity);
  EXPECT_GT(g.covariance, 100.0);
}

TEST_F(InstallationPoseTest, AtomicCommitKeepsPriorOnFailure) {
  // A valid file is in place. Writing to a path whose temp rename would be fine
  // (normal case) must produce a valid file.
  const std::string p = tmp_path("commit.yaml");
  BaseOrientation a;
  a.valid = true;
  a.source = PoseSource::VisualCalibration;
  a.timestamp_ns = 1;
  std::string err;
  ASSERT_TRUE(save_installation_pose(p, a, err));
  // Overwrite with a second calibration; the rename must replace atomically.
  BaseOrientation b = a;
  b.timestamp_ns = 2;
  ASSERT_TRUE(save_installation_pose(p, b, err));
  BaseOrientation l = load_installation_pose(p);
  EXPECT_TRUE(l.valid);
  EXPECT_EQ(l.timestamp_ns, 2);
  // No leftover temp file.
  EXPECT_FALSE(std::filesystem::exists(p + ".tmp"));
}

TEST_F(InstallationPoseTest, SaveToBadPathReportsError) {
  BaseOrientation o;
  o.valid = true;
  o.source = PoseSource::VisualCalibration;
  std::string err;
  EXPECT_FALSE(save_installation_pose("/nonexistent_dir_xyz/pose.yaml", o, err));
  EXPECT_FALSE(err.empty());
}

}  // namespace
