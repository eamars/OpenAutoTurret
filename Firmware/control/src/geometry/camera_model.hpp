#pragma once
// Camera model (architecture §10.1–§10.2): intrinsics and the pixel -> ray
// conversion. The ray is expressed in the CAMERA frame, whose convention is
// OpenCV/IMX500: X right, Y down, Z forward (optical axis).
#include "geometry/vec3.hpp"

namespace ota {
namespace geo {

struct CameraIntrinsics {
  double fx = 1000.0;
  double fy = 1000.0;
  double cx = 960.0;
  double cy = 540.0;
  int width = 1920;
  int height = 1080;

  bool valid() const { return fx > 0.0 && fy > 0.0 && width > 0 && height > 0; }
};

struct CameraModel {
  CameraIntrinsics intrinsics;

  CameraModel() = default;
  explicit CameraModel(CameraIntrinsics in) : intrinsics(in) {}

  // Pixel (u, v) -> normalized ray in the camera frame (unit length, Z forward).
  // Uses full projective geometry (§10.2), not a constant pixel->degree factor.
  Vec3 pixel_to_ray(double u, double v) const {
    const double x = (u - intrinsics.cx) / intrinsics.fx;
    const double y = (v - intrinsics.cy) / intrinsics.fy;
    return Vec3{x, y, 1.0}.normalized();
  }

  // Camera-frame ray -> pixel (inverse of pixel_to_ray).
  void ray_to_pixel(const Vec3& r_cam, double& u, double& v) const {
    if (r_cam.z <= 0.0) {
      // Ray behind the camera; project at the optical axis (degenerate).
      u = intrinsics.cx;
      v = intrinsics.cy;
      return;
    }
    u = intrinsics.cx + (r_cam.x / r_cam.z) * intrinsics.fx;
    v = intrinsics.cy + (r_cam.y / r_cam.z) * intrinsics.fy;
  }
};

}  // namespace geo
}  // namespace ota
