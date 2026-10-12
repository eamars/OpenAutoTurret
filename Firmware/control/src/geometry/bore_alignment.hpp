#pragma once
// Where the bore -- the payload's point of aim -- appears from each camera, at an explicitly assumed
// depth. No range sensor yet.
#include "geometry/camera_model.hpp"
#include <cmath>
#include <initializer_list>

namespace ota::geo {

// One camera's offset from the bore: where the camera sits and where it looks, both relative to the
// bore. Position in millimetres in the bore's frame (a camera 75 mm right of and 75 mm above the
// bore is right 75, up 75). Pointing in degrees: the optical axis turned right of the bore's axis,
// then up from it.
struct CameraOffset {
  double right_mm = 0, up_mm = 0, forward_mm = 0;
  double right_deg = 0, up_deg = 0;
  bool operator==(const CameraOffset&) const = default;
};

// The two cameras (perception's roles). Detail detections are published in the wide frame, so the
// tracker reasons with the wide camera's offset; each camera's own offset places the bore mark on
// that camera's picture.
enum class CameraRole { Wide, Detail };

struct BoreAlignmentConfig {
  bool enabled = false;
  double assumed_depth_m = 10;
  CameraOffset wide, detail;
  const CameraOffset& offset(CameraRole role) const { return role == CameraRole::Detail ? detail : wide; }
  bool operator==(const BoreAlignmentConfig&) const = default;
};

struct BoreAlignment {
  bool enabled = false;
  bool valid = false;
  Vec3 origin_camera{}, direction_camera{0, 0, 1}, sight_camera{0, 0, 1};
  double u_norm = 0, v_norm = 0;
  const char* reason = "disabled";
};

// `in` is the wide camera's model. For the detail camera that is still the right projection: its
// picture is published as a centred window of the wide frame scaled by the focal-length ratio, and
// the ratio cancels between the detail lens and that window.
inline BoreAlignment bore_alignment(const BoreAlignmentConfig& c, CameraRole role,
                                    const CameraIntrinsics& in) {
  BoreAlignment out;
  out.enabled = c.enabled;
  if (!c.enabled) return out;
  const CameraOffset& o = c.offset(role);
  out.reason = "invalid alignment configuration";
  for (double v : {o.right_mm, o.up_mm, o.forward_mm, o.right_deg, o.up_deg, c.assumed_depth_m})
    if (!std::isfinite(v)) return out;
  if (std::abs(o.right_deg) >= 45 || std::abs(o.up_deg) >= 45 || c.assumed_depth_m <= 0) return out;
  out.reason = "invalid camera intrinsics";
  if (!in.valid() || !std::isfinite(in.fx) || !std::isfinite(in.fy) ||
      !std::isfinite(in.cx) || !std::isfinite(in.cy)) return out;
  // Detector coordinates: x right, y down, z forward. The camera's axes in the bore's frame are
  // R = Ry(right) Rx(up) -- turned right about the vertical, then up about its own horizontal -- and
  // a bore-frame vector seen from the camera is R^T v. Exact at any angle: the two-tangent shortcut
  // the old model used is off by tan(right) (1/cos(up) - 1), 3 urad at 1 deg x 1 deg but 2.7 mrad
  // at 10 deg x 10 deg, which a camera mounted well off the bore can reach.
  const double cr = std::cos(o.right_deg*M_PI/180), sr = std::sin(o.right_deg*M_PI/180);
  const double cu = std::cos(o.up_deg*M_PI/180), su = std::sin(o.up_deg*M_PI/180);
  const auto to_camera = [&](const Vec3& v) {
    const Vec3 a{cr*v.x - sr*v.z, v.y, sr*v.x + cr*v.z};   // Ry(right)^T
    return Vec3{a.x, cu*a.y + su*a.z, -su*a.y + cu*a.z};    // Rx(up)^T
  };
  out.origin_camera = to_camera(Vec3{-o.right_mm, o.up_mm, -o.forward_mm}/1000);
  out.direction_camera = to_camera(Vec3{0, 0, 1});
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
