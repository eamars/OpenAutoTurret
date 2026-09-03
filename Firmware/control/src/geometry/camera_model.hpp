#pragma once
// Camera model (architecture §10.1–§10.2): intrinsics and the pixel -> ray
// conversion. The ray is expressed in the CAMERA frame, whose convention is
// OpenCV/IMX500: X right, Y down, Z forward (optical axis).
#include <cmath>

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

// Field of view implied by an intrinsics set, in degrees.
//
// The HUD's Field-of-Regard inset and any "will the target leave the frame?" judgement need
// these, and they must come from the loaded calibration rather than from a number somebody
// typed into the page: the sensor's field changes with the requested stream size, so a
// hard-coded FOV is wrong the moment the preview size is. Published as camera.effective_hfov_deg
// / effective_vfov_deg (v3.2 s20) and cross-checked against the encoder-theodolite walk, which
// measured 69.2 deg and 40.4 deg on this station at 1920x1080.
inline void field_of_view_deg(const CameraIntrinsics& in, double& hfov_deg, double& vfov_deg) {
  if (!in.valid()) {
    hfov_deg = 0.0;
    vfov_deg = 0.0;
    return;
  }
  hfov_deg = 2.0 * std::atan(in.width / (2.0 * in.fx)) * (180.0 / M_PI);
  vfov_deg = 2.0 * std::atan(in.height / (2.0 * in.fy)) * (180.0 / M_PI);
}

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

  // Camera-frame line-of-sight angles of a camera-space ray: yaw positive to the right, pitch
  // positive upward, both zero along the optical axis. This is the exact inverse of the pixel map
  // above - tan(yaw) is (u-cx)/fx and tan(-pitch) is (v-cy)/fy, because the image's y axis grows
  // downward while elevation grows upward - and it is written next to that map on purpose: two
  // names for one angle, defined in two places, is a sign error that survives every review and
  // shows up on the operator's screen as a prediction cue on the wrong side of the target.
  //
  // A ray behind the camera (z <= 0) yields |yaw| > 90 deg rather than an error, which is the
  // honest answer for an angle function but useless as a heading. Callers that must not publish
  // astern rays guard on z, as the telemetry fill does.
  static void ray_to_los_angles(const Vec3& r_cam, double& yaw_rad, double& pitch_rad) {
    yaw_rad = std::atan2(r_cam.x, r_cam.z);
    pitch_rad = std::atan2(-r_cam.y, r_cam.z);
  }
};

}  // namespace geo
}  // namespace ota
