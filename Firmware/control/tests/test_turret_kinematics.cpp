// Unit tests for the turret (gimbal) kinematics (architecture §9.2, §10.3):
// camera-frame LOS -> base-frame LOS. Pure geometry — no camera, no CAN, no
// motor driver.
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/turret_kinematics.hpp"

namespace {
using ota::geo::TurretKinematics;
using ota::geo::Vec3;

constexpr double kPi = M_PI;

TEST(TurretKinematics, OpticalAxisAtZeroIsBaseForward) {
  auto kin = TurretKinematics::aligned();
  Vec3 r = kin.ray_to_base(Vec3{0.0, 0.0, 1.0}, 0.0, 0.0);  // camera +Z
  EXPECT_NEAR(r.x, 1.0, 1e-12);
  EXPECT_NEAR(r.y, 0.0, 1e-12);
  EXPECT_NEAR(r.z, 0.0, 1e-12);
}

TEST(TurretKinematics, CameraRightAtZeroIsBaseRight) {
  auto kin = TurretKinematics::aligned();
  // Camera +X (right) -> base -Y (base Y is left, so right is -Y).
  Vec3 r = kin.ray_to_base(Vec3{1.0, 0.0, 0.0}, 0.0, 0.0);
  EXPECT_NEAR(r.x, 0.0, 1e-12);
  EXPECT_NEAR(r.y, -1.0, 1e-12);
  EXPECT_NEAR(r.z, 0.0, 1e-12);
}

TEST(TurretKinematics, CameraDownAtZeroIsBaseDown) {
  auto kin = TurretKinematics::aligned();
  // Camera +Y (down) -> base -Z (base Z is up, so down is -Z).
  Vec3 r = kin.ray_to_base(Vec3{0.0, 1.0, 0.0}, 0.0, 0.0);
  EXPECT_NEAR(r.x, 0.0, 1e-12);
  EXPECT_NEAR(r.y, 0.0, 1e-12);
  EXPECT_NEAR(r.z, -1.0, 1e-12);
}

TEST(TurretKinematics, YawTurnsOpticalAxisSideways) {
  auto kin = TurretKinematics::aligned();
  // Yaw +90 deg: optical axis points to base +Y (left).
  Vec3 r = kin.ray_to_base(Vec3{0.0, 0.0, 1.0}, kPi / 2.0, 0.0);
  EXPECT_NEAR(r.x, 0.0, 1e-9);
  EXPECT_NEAR(r.y, 1.0, 1e-9);
  EXPECT_NEAR(r.z, 0.0, 1e-9);
}

TEST(TurretKinematics, PitchTiltsOpticalAxisVertical) {
  auto kin = TurretKinematics::aligned();
  // Pitch +90 deg tilts the optical axis off the horizontal (to -Z here).
  Vec3 r = kin.ray_to_base(Vec3{0.0, 0.0, 1.0}, 0.0, kPi / 2.0);
  EXPECT_NEAR(r.x, 0.0, 1e-9);
  EXPECT_NEAR(r.y, 0.0, 1e-9);
  EXPECT_NEAR(std::fabs(r.z), 1.0, 1e-9);
}

TEST(TurretKinematics, CombinedYawAndPitch) {
  auto kin = TurretKinematics::aligned();
  // Yaw +90 then pitch +45: optical axis is in the vertical-left plane.
  Vec3 r = kin.ray_to_base(Vec3{0.0, 0.0, 1.0}, kPi / 2.0, kPi / 4.0);
  EXPECT_NEAR(std::fabs(r.x), 0.0, 1e-9);  // no forward component
  EXPECT_NEAR(std::fabs(r.y), std::cos(kPi / 4.0), 1e-9);
  EXPECT_NEAR(std::fabs(r.z), std::sin(kPi / 4.0), 1e-9);
  EXPECT_NEAR(r.norm(), 1.0, 1e-12);
}

TEST(TurretKinematics, TransformPreservesLength) {
  auto kin = TurretKinematics::aligned();
  // A rotation must preserve the vector length for any input/joint angles.
  Vec3 in{0.3, -0.5, 1.0};
  for (double yaw : {-0.7, 0.0, 0.4, 1.2}) {
    for (double pitch : {-0.9, 0.0, 0.5}) {
      Vec3 r = kin.ray_to_base(in, yaw, pitch);
      EXPECT_NEAR(r.norm(), 1.0, 1e-12) << "yaw=" << yaw << " pitch=" << pitch;
    }
  }
}

TEST(TurretKinematics, LosAnglesFromBaseRay) {
  auto kin = TurretKinematics::aligned();
  double az, el;
  // Base forward (+X): azimuth 0, elevation 0.
  TurretKinematics::base_ray_to_los(Vec3{1.0, 0.0, 0.0}, az, el);
  EXPECT_NEAR(az, 0.0, 1e-12);
  EXPECT_NEAR(el, 0.0, 1e-12);
  // Base left (+Y): azimuth +90 deg, elevation 0.
  TurretKinematics::base_ray_to_los(Vec3{0.0, 1.0, 0.0}, az, el);
  EXPECT_NEAR(az, kPi / 2.0, 1e-12);
  EXPECT_NEAR(el, 0.0, 1e-12);
  // 45 deg up, forward: elevation +45 deg.
  const double s = std::sin(kPi / 4.0), c = std::cos(kPi / 4.0);
  TurretKinematics::base_ray_to_los(Vec3{c, 0.0, s}, az, el);
  EXPECT_NEAR(az, 0.0, 1e-9);
  EXPECT_NEAR(el, kPi / 4.0, 1e-9);
}

}  // namespace
