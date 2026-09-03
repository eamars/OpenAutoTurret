// §20 asks for the prediction as camera-frame LOS angles. Those angles are the inverse of the pixel
// map, so the test that matters is the round trip: take pixels, make rays, make angles, and check the
// angles reproduce the pixels. Sign conventions in particular - the image's y grows downward while
// elevation grows upward - because the version that got that backwards would put the prediction cue
// on the far side of the target and still pass every magnitude check.

#include "geometry/camera_model.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

using ota::geo::CameraIntrinsics;
using ota::geo::CameraModel;
using ota::geo::Vec3;

// The station's own commissioned intrinsics: 1920x1080 with fx=1389, fy=1467, principal point at
// (960, 540). Validity is derived from these values by CameraIntrinsics::valid(); there is no flag
// to set, which is itself the point - an intrinsics set cannot claim to be valid while being empty.
CameraIntrinsics StationLike() {
  CameraIntrinsics in;
  in.fx = 1389.0;
  in.fy = 1467.0;
  in.cx = 960.0;
  in.cy = 540.0;
  in.width = 1920;
  in.height = 1080;
  return in;
}

TEST(CameraLosAngles, OpticalAxisIsZeroZero) {
  double yaw = 1.0, pitch = 1.0;
  CameraModel::ray_to_los_angles(Vec3{0.0, 0.0, 1.0}, yaw, pitch);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(0.0, pitch);
}

TEST(CameraLosAngles, SignsFollowTheImageAxes) {
  // A pixel right of the principal point must read positive yaw; a pixel ABOVE it (smaller v) must
  // read positive pitch, because "up in the picture" is up in the world and v runs the other way.
  const CameraModel cam(StationLike());
  double yaw = 0.0, pitch = 0.0;

  CameraModel::ray_to_los_angles(cam.pixel_to_ray(1200.0, 540.0), yaw, pitch);
  EXPECT_GT(yaw, 0.0) << "a point right of centre read as looking left";
  EXPECT_DOUBLE_EQ(0.0, pitch);

  CameraModel::ray_to_los_angles(cam.pixel_to_ray(960.0, 300.0), yaw, pitch);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_GT(pitch, 0.0) << "a point above the centre read as looking down";

  CameraModel::ray_to_los_angles(cam.pixel_to_ray(960.0, 800.0), yaw, pitch);
  EXPECT_LT(pitch, 0.0) << "a point below the centre read as looking up";
}

TEST(CameraLosAngles, RoundTripsThroughThePixelMap) {
  // tan(yaw) must equal the rectilinear relation the pixel map uses, at pixels across the frame and
  // out to the corners - where a constant pixel-per-degree shortcut would be visibly wrong, and
  // where the frame-exit margin that §20 exists to support is actually decided.
  const CameraModel cam(StationLike());
  const CameraIntrinsics& in = cam.intrinsics;
  const double us[5] = {120.0, 600.0, 960.0, 1400.0, 1800.0};
  const double vs[5] = {90.0, 300.0, 540.0, 780.0, 990.0};
  for (double u : us) {
    for (double v : vs) {
      double yaw = 0.0, pitch = 0.0;
      CameraModel::ray_to_los_angles(cam.pixel_to_ray(u, v), yaw, pitch);
      EXPECT_NEAR((u - in.cx) / in.fx, std::tan(yaw), 1e-9) << "u=" << u << " v=" << v;
      EXPECT_NEAR((in.cy - v) / in.fy, std::tan(pitch), 1e-9) << "u=" << u << " v=" << v;
    }
  }
}

TEST(CameraLosAngles, RayBehindTheCameraIsBeyondNinetyDegreesNotAnError) {
  // Documented rather than "fixed": the function answers a ray astern with an angle, and it is the
  // caller's guard on z that keeps such a ray out of the telemetry. A caller that forgot the guard
  // would find |yaw| > 90 deg here rather than a plausible-looking heading, which is why the guard
  // is worth its own test even though the behaviour is intended.
  double yaw = 0.0, pitch = 0.0;
  CameraModel::ray_to_los_angles(Vec3{0.3, 0.0, -1.0}, yaw, pitch);
  EXPECT_GT(std::abs(yaw), M_PI_2);
}

TEST(CameraLosAngles, CornersOfTheFrameLandOnTheCommissionedHalfField) {
  // Cross-check against the geometry this station measured with the encoder-as-theodolite probe:
  // 69.30 x 40.42 deg at 1920x1080 (the probe independently said 69.2 x 40.4). A corner ray must
  // sit just inside the half-field, which is what makes cue placement trustworthy near the edges -
  // the place where targets leave the frame and the lead rule is decided.
  //
  // Written on magnitudes plus signs, because v grows downward: the top corners look UP and the
  // bottom corners look DOWN. The first version of this test asserted a positive pitch for v=1079,
  // which is the bottom of the frame, and failed - contradicting the sign convention three tests
  // above had already established. A test that disagrees with its neighbours is usually the test
  // that is wrong.
  const CameraModel cam(StationLike());
  const double kDeg = M_PI / 180.0;

  double yaw = 0.0, pitch = 0.0;
  CameraModel::ray_to_los_angles(cam.pixel_to_ray(1919.0, 1.0), yaw, pitch);   // top right
  EXPECT_GT(yaw, 34.0 * kDeg) << "right edge should be near +half of 69.3 deg";
  EXPECT_LT(yaw, 34.65 * kDeg);
  EXPECT_GT(pitch, 20.0 * kDeg) << "top edge should be looking up, near +half of 40.4 deg";
  EXPECT_LT(pitch, 20.21 * kDeg);

  CameraModel::ray_to_los_angles(cam.pixel_to_ray(1.0, 1079.0), yaw, pitch);   // bottom left
  EXPECT_LT(yaw, -34.0 * kDeg) << "left edge must mirror the right edge";
  EXPECT_GT(yaw, -34.65 * kDeg);
  EXPECT_LT(pitch, -20.0 * kDeg) << "bottom edge should be looking down";
  EXPECT_GT(pitch, -20.21 * kDeg);
}

}  // namespace
