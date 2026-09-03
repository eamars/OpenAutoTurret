// Unit tests for the turret (gimbal) kinematics (architecture §9.2, §10.3):
// camera-frame LOS -> base-frame LOS, and — since §73's aim point — the same geometry run
// backwards, which is why the camera model is here too. Still pure geometry: no camera
// hardware, no CAN, no motor driver.
#include <gtest/gtest.h>

#include <cmath>

#include "geometry/camera_model.hpp"
#include "geometry/turret_kinematics.hpp"

namespace {
using ota::geo::CameraIntrinsics;
using ota::geo::CameraModel;
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

// §73's aim point needs the geometry run backwards: controld knows where it is pointing
// (a base-frame line of sight) and the page needs to know where that lands in the picture.
// `ray_to_base` has been in v1 since the beginning; the inverse is new, and an inverse that
// merely *looks* right is the kind of bug that shows up as a reticle a few degrees off
// target — small enough to look like tracking error, systematic enough to mislead an
// operator about what the turret is aiming at. So the check is a round trip, over poses and
// over pixels, against the transform that is already trusted.
TEST(TurretKinematics, TheInverseRoundTripsThroughTheGimbal) {
  const TurretKinematics k = TurretKinematics::aligned();
  const CameraModel cam(CameraIntrinsics{800.0, 800.0, 960.0, 540.0, 1920, 1080});
  const double poses[][2] = {{0.0, 0.0},
                             {kPi / 6.0, 0.1},
                             {-kPi / 4.0, -0.3},
                             {kPi * 0.9, 0.4},
                             {-kPi * 0.95, -0.05}};
  for (const auto& q : poses) {
    const double q_yaw = q[0], q_pitch = q[1];
    for (double u = 200.0; u <= 1720.0; u += 190.0) {
      for (double v = 120.0; v <= 960.0; v += 168.0) {
        const Vec3 r_cam = cam.pixel_to_ray(u, v);
        double az = 0.0, el = 0.0;
        TurretKinematics::base_ray_to_los(k.ray_to_base(r_cam, q_yaw, q_pitch), az, el);
        const Vec3 back = k.base_to_ray(TurretKinematics::los_to_base_ray(az, el), q_yaw,
                                        q_pitch);
        double u2 = 0.0, v2 = 0.0;
        cam.ray_to_pixel(back, u2, v2);
        EXPECT_NEAR(u2, u, 1e-6) << "yaw " << q_yaw << " pitch " << q_pitch << " at (" << u
                                 << "," << v << ")";
        EXPECT_NEAR(v2, v, 1e-6) << "yaw " << q_yaw << " pitch " << q_pitch << " at (" << u
                                 << "," << v << ")";
      }
    }
  }
}

TEST(TurretKinematics, StraightDownTheOpticalAxisIsThePrincipalPoint) {
  // The one case with an answer nobody has to compute: whatever the joints, a ray along the
  // optical axis projects to (cx, cy). If the inverse got the order of the rotations wrong,
  // this fails at every non-zero pose while the round trip above could still pass — the two
  // tests are not redundant.
  const TurretKinematics k = TurretKinematics::aligned();
  const CameraModel cam(CameraIntrinsics{800.0, 800.0, 640.0, 360.0, 1280, 720});
  for (double q_yaw = -2.0; q_yaw <= 2.0; q_yaw += 0.5) {
    for (double q_pitch = -0.5; q_pitch <= 0.5; q_pitch += 0.25) {
      double az = 0.0, el = 0.0;
      TurretKinematics::base_ray_to_los(k.ray_to_base(Vec3{0.0, 0.0, 1.0}, q_yaw, q_pitch),
                                        az, el);
      double u = 0.0, v = 0.0;
      cam.ray_to_pixel(k.base_to_ray(TurretKinematics::los_to_base_ray(az, el), q_yaw,
                                     q_pitch),
                       u, v);
      EXPECT_NEAR(u, 640.0, 1e-7) << "yaw " << q_yaw << " pitch " << q_pitch;
      EXPECT_NEAR(v, 360.0, 1e-7) << "yaw " << q_yaw << " pitch " << q_pitch;
    }
  }
}

TEST(TurretKinematics, ARayBehindTheCameraIsDetectableBeforeItIsProjected) {
  // `ray_to_pixel` handles a ray behind the camera by returning the principal point, which is
  // the right thing for a geometry helper and the wrong thing to publish: a target astern of
  // the camera would be drawn dead centre, over the reticle that says "this is where we are
  // aiming". So the caller has to be able to tell, and this documents the test that the sign
  // of z says so — including the case that matters on this station, a target behind the
  // turret while AUTO_ROAM sweeps past it.
  const TurretKinematics k = TurretKinematics::aligned();
  const Vec3 astern = TurretKinematics::los_to_base_ray(kPi, 0.0);  // 180 deg: straight back
  const Vec3 r_cam = k.base_to_ray(astern, 0.0, 0.0);
  EXPECT_LT(r_cam.z, 0.0) << "a ray astern came back with z = " << r_cam.z
                          << "; the invalid flag would have to come from somewhere else";
  const Vec3 ahead = TurretKinematics::los_to_base_ray(0.0, 0.0);
  EXPECT_GT(k.base_to_ray(ahead, 0.0, 0.0).z, 0.0);
}

}  // namespace
