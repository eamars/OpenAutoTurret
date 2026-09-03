// Unit tests for the camera model (architecture §10.1–§10.2): intrinsics and
// pixel <-> ray conversion. Pure geometry — no camera, no CAN, no motor.
#include <gtest/gtest.h>

#include "geometry/camera_model.hpp"

namespace {
using ota::geo::CameraIntrinsics;
using ota::geo::CameraModel;
using ota::geo::Vec3;

CameraModel make_model() {
  CameraIntrinsics in;
  in.fx = 1000.0;
  in.fy = 1000.0;
  in.cx = 960.0;
  in.cy = 540.0;
  in.width = 1920;
  in.height = 1080;
  return CameraModel(in);
}

TEST(CameraModel, OpticalAxisMapsToCameraZ) {
  auto cam = make_model();
  // The principal point maps to the optical axis (camera +Z).
  Vec3 r = cam.pixel_to_ray(960.0, 540.0);
  EXPECT_NEAR(r.x, 0.0, 1e-12);
  EXPECT_NEAR(r.y, 0.0, 1e-12);
  EXPECT_NEAR(r.z, 1.0, 1e-12);
}

TEST(CameraModel, OffsetPixelGivesExpectedAngle) {
  auto cam = make_model();
  // One focal-length to the right: x = (u-cx)/fx = 1 -> 45 deg ray.
  Vec3 r = cam.pixel_to_ray(1960.0, 540.0);  // cx + fx
  EXPECT_NEAR(r.x, 1.0 / std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(r.z, 1.0 / std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(r.y, 0.0, 1e-12);
  // Unit length.
  EXPECT_NEAR(r.norm(), 1.0, 1e-12);
}

TEST(CameraModel, DownPixelGivesPositiveY) {
  auto cam = make_model();
  // Below the principal point: y = (v-cy)/fy > 0 (camera Y is down).
  Vec3 r = cam.pixel_to_ray(960.0, 1540.0);  // cy + fy
  EXPECT_NEAR(r.y, 1.0 / std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(r.x, 0.0, 1e-12);
}

TEST(CameraModel, RayToPixelRoundTrip) {
  auto cam = make_model();
  for (double u : {300.0, 960.0, 1600.0}) {
    for (double v : {200.0, 540.0, 900.0}) {
      Vec3 r = cam.pixel_to_ray(u, v);
      double u2, v2;
      cam.ray_to_pixel(r, u2, v2);
      EXPECT_NEAR(u2, u, 1e-6) << "u=" << u << " v=" << v;
      EXPECT_NEAR(v2, v, 1e-6) << "u=" << u << " v=" << v;
    }
  }
}

TEST(CameraModel, IntrinsicsValidity) {
  CameraIntrinsics good;
  EXPECT_TRUE(good.valid());
  CameraIntrinsics bad;
  bad.fx = 0.0;
  EXPECT_FALSE(bad.valid());
  CameraIntrinsics bad2;
  bad2.width = 0;
  EXPECT_FALSE(bad2.valid());
}

// The field of view the HUD's FOR inset and the frame-exit margin are computed from,
// cross-checked against an independent measurement rather than against the formula that
// produced it: tools/probe_theodolite.py walked the axis encoder against the image and
// measured 24.22 px/deg horizontally and 25.60 px/deg vertically in a 1920x1080 frame,
// which is 69.2 deg and 40.4 deg across. If this ever disagrees by more than half a
// degree, one of the two is wrong - and that is the point of writing it down.
TEST(CameraFieldOfView, AgreesWithTheTheodoliteWalkOnThisStation) {
  ota::geo::CameraIntrinsics in;
  in.fx = 1389.0; in.fy = 1467.0; in.cx = 960.0; in.cy = 540.0;
  in.width = 1920; in.height = 1080;
  double h = -1.0, v = -1.0;
  ota::geo::field_of_view_deg(in, h, v);
  EXPECT_NEAR(h, 69.2, 0.5);
  EXPECT_NEAR(v, 40.4, 0.5);

  // Uncalibrated must not become "zero degrees of view" dressed as a measurement... and must
  // not become a default either: the caller has to be able to tell "unknown" from "60 deg".
  ota::geo::CameraIntrinsics bad;
  bad.fx = 0.0;
  h = -1.0; v = -1.0;
  ota::geo::field_of_view_deg(bad, h, v);
  EXPECT_DOUBLE_EQ(h, 0.0);
  EXPECT_DOUBLE_EQ(v, 0.0);
}

}  // namespace
