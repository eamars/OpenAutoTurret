#include <gtest/gtest.h>
#include "geometry/los_uncertainty.hpp"

TEST(LosUncertainty, HorizontalAlignedCameraPreservesPixelScale) {
  ota::geo::CameraIntrinsics in;
  in.fx=1390; in.fy=1467;
  auto variance = ota::geo::pixel_los_variance(ota::geo::CameraModel(in),
      ota::geo::TurretKinematics::aligned(),0,0,in.cx,in.cy,14,20);
  EXPECT_NEAR(std::sqrt(variance[0]),14/in.fx,1e-7);
  EXPECT_NEAR(std::sqrt(variance[1]),20/in.fy,1e-7);
}

TEST(LosUncertainty, StationPitchAmplifiesYawUncertaintyAcrossAzimuthWrap) {
  ota::geo::CameraIntrinsics in;
  in.fx=1390; in.fy=1467;
  ota::geo::TurretKinematics kin;
  kin.R_PC={0,1,0,-1,0,0,0,0,1};
  // At this live pose a Monte Carlo pixel-noise probe measured about 2.03
  // degrees yaw sigma. The previous shared image-angle sigma was only .78.
  auto variance = ota::geo::pixel_los_variance(ota::geo::CameraModel(in),
      kin,0,-.29,in.cx,in.cy,14,20);
  EXPECT_NEAR(std::sqrt(variance[0]),14/(in.fx*std::sin(.29)),1e-6);
  EXPECT_NEAR(std::sqrt(variance[1]),20/in.fy,1e-7);
  EXPECT_GT(variance[0],6*variance[1]);
  auto rotated = ota::geo::pixel_los_variance(ota::geo::CameraModel(in),
      kin,2,-.29,in.cx,in.cy,14,20);
  EXPECT_NEAR(rotated[0],variance[0],1e-10);
}
