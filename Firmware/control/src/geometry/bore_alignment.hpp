#pragma once
// Virtual bore alignment at an explicitly assumed camera-axis depth. No sensors.
#include "geometry/camera_model.hpp"
#include <initializer_list>

namespace ota::geo {

struct BoreAlignmentConfig {
  bool enabled = false;
  double camera_right_mm = 0, camera_up_mm = 0, camera_forward_mm = 0;
  double bore_right_deg = 0, bore_up_deg = 0;
  double assumed_depth_m = 10;
  bool operator==(const BoreAlignmentConfig&) const = default;
};

struct BoreAlignment {
  bool enabled = false;
  bool valid = false;
  Vec3 origin_camera{}, direction_camera{0, 0, 1}, sight_camera{0, 0, 1};
  double u_norm = 0, v_norm = 0;
  const char* reason = "disabled";
};

inline BoreAlignment bore_alignment(const BoreAlignmentConfig& c,
                                      const CameraIntrinsics& in) {
  BoreAlignment out;
  out.enabled = c.enabled;
  if (!c.enabled) return out;
  out.reason = "invalid alignment configuration";
  for (double v : {c.camera_right_mm, c.camera_up_mm, c.camera_forward_mm,
                   c.bore_right_deg, c.bore_up_deg, c.assumed_depth_m})
    if (!std::isfinite(v)) return out;
  if (std::abs(c.bore_right_deg) >= 45 || std::abs(c.bore_up_deg) >= 45 ||
      c.assumed_depth_m <= 0) return out;
  out.reason = "invalid camera intrinsics";
  if (!in.valid() || !std::isfinite(in.fx) || !std::isfinite(in.fy) ||
      !std::isfinite(in.cx) || !std::isfinite(in.cy)) return out;
  // Corrected detector coordinates: right, down, forward. Orientation is already applied.
  out.origin_camera = Vec3{-c.camera_right_mm, c.camera_up_mm, -c.camera_forward_mm}/1000;
  out.direction_camera = Vec3{std::tan(c.bore_right_deg*M_PI/180),
                             -std::tan(c.bore_up_deg*M_PI/180), 1}.normalized();
  const double s = (c.assumed_depth_m-out.origin_camera.z)/out.direction_camera.z;
  out.reason = "reference plane behind bore";
  if (!(s > 0) || !std::isfinite(s)) return out;
  const Vec3 p = out.origin_camera + out.direction_camera*s;
  const double u = in.cx + in.fx*p.x/p.z, v = in.cy + in.fy*p.y/p.z;
  out.reason = "bore sight outside camera frame";
  if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || u > in.width || v < 0 || v > in.height)
    return out;
  out.sight_camera = Vec3{p.x/p.z, p.y/p.z, 1}.normalized();
  out.u_norm = u/in.width;
  out.v_norm = v/in.height;
  out.valid = true;
  out.reason = "assumed depth";
  return out;
}

}  // namespace ota::geo
