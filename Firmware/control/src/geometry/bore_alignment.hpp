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

// The bore seen from one camera: its origin and direction in the camera's coordinates, and where it
// crosses the plane `depth_m` ahead of the camera. False when it does not cross it in front.
//
// Detector coordinates: x right, y down, z forward. The camera's axes in the bore's frame are
// R = Ry(right) Rx(up) -- turned right about the vertical, then up about its own horizontal -- and a
// bore-frame vector seen from the camera is R^T v. Exact at any angle: the two-tangent shortcut the
// old model used is off by tan(right) (1/cos(up) - 1), 3 urad at 1 deg x 1 deg but 2.7 mrad at
// 10 deg x 10 deg, which a camera mounted well off the bore can reach.
inline bool bore_in_camera(const CameraOffset& o, double depth_m, Vec3& origin, Vec3& direction, Vec3& point) {
  const double cr = std::cos(o.right_deg*M_PI/180), sr = std::sin(o.right_deg*M_PI/180);
  const double cu = std::cos(o.up_deg*M_PI/180), su = std::sin(o.up_deg*M_PI/180);
  const auto to_camera = [&](const Vec3& v) {
    const Vec3 a{cr*v.x - sr*v.z, v.y, sr*v.x + cr*v.z};   // Ry(right)^T
    return Vec3{a.x, cu*a.y + su*a.z, -su*a.y + cu*a.z};    // Rx(up)^T
  };
  origin = to_camera(Vec3{-o.right_mm, o.up_mm, -o.forward_mm}/1000);
  direction = to_camera(Vec3{0, 0, 1});
  const double s = (depth_m-origin.z)/direction.z;
  if (!(s > 0) || !std::isfinite(s)) return false;
  point = origin + direction*s;
  return point.z > 0;
}

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
  Vec3 p;
  out.reason = "reference plane behind bore";
  if (!bore_in_camera(o, c.assumed_depth_m, out.origin_camera, out.direction_camera, p)) return out;
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

// ADR-006 D1, the zero: the inverse of bore_alignment. The payload points at a target `depth_m`
// ahead and the operator taps the target at (u_norm, v_norm) of the wide frame in this camera's
// picture. Returns the pointing (right_deg, up_deg) that puts this camera's bore mark exactly there,
// its installation position (mm) kept. Solved on the same projection bore_alignment draws with
// (Newton's method, numerical Jacobian), so the round trip is exact by construction.
struct CameraPointing {
  bool valid = false;
  double right_deg = 0, up_deg = 0;
  const char* reason = "not solved";
};

inline CameraPointing solve_camera_pointing(const CameraOffset& installed, double depth_m, double u_norm,
                                            double v_norm, const CameraIntrinsics& in) {
  CameraPointing out;
  out.reason = "invalid camera intrinsics";
  if (!in.valid() || !std::isfinite(in.fx) || !std::isfinite(in.fy)) return out;
  out.reason = "the target's distance must be positive";
  if (!(depth_m > 0) || !std::isfinite(depth_m)) return out;
  out.reason = "the tap is outside the picture";
  if (!(u_norm >= 0 && u_norm <= 1 && v_norm >= 0 && v_norm <= 1)) return out;
  out.reason = "invalid installation offset";
  for (double v : {installed.right_mm, installed.up_mm, installed.forward_mm})
    if (!std::isfinite(v)) return out;
  // Residual in pixels: where the mark falls with this pointing, less where it was tapped.
  const double tu = u_norm*in.width, tv = v_norm*in.height;
  CameraOffset o = installed;
  const auto residual = [&](double right, double up, double& du, double& dv) {
    o.right_deg = right; o.up_deg = up;
    Vec3 origin, direction, p;
    if (!bore_in_camera(o, depth_m, origin, direction, p)) return false;
    du = in.cx + in.fx*p.x/p.z - tu;
    dv = in.cy + in.fy*p.y/p.z - tv;
    return std::isfinite(du) && std::isfinite(dv);
  };
  double r = std::isfinite(installed.right_deg) ? installed.right_deg : 0;
  double u = std::isfinite(installed.up_deg) ? installed.up_deg : 0;
  constexpr double h = 1e-4;   // deg: 0.03 px on the wide camera, far above rounding
  out.reason = "the zero does not converge";
  for (int i = 0; i < 30; ++i) {
    double fu, fv;
    if (!residual(r, u, fu, fv)) return out;
    if (std::hypot(fu, fv) < 1e-6) {   // px
      out.reason = "the pointing is 45 deg or more off the bore";
      if (std::abs(r) >= 45 || std::abs(u) >= 45) return out;
      out.valid = true;
      out.right_deg = r;
      out.up_deg = u;
      out.reason = "zeroed";
      return out;
    }
    double a_u, a_v, b_u, b_v;
    if (!residual(r + h, u, a_u, a_v) || !residual(r, u + h, b_u, b_v)) return out;
    const double j11 = (a_u - fu)/h, j21 = (a_v - fv)/h, j12 = (b_u - fu)/h, j22 = (b_v - fv)/h;
    const double det = j11*j22 - j12*j21;
    if (!(std::abs(det) > 0) || !std::isfinite(det)) return out;
    r -= (j22*fu - j12*fv)/det;
    u -= (-j21*fu + j11*fv)/det;
    if (!std::isfinite(r) || !std::isfinite(u) || std::abs(r) > 80 || std::abs(u) > 80) return out;
  }
  return out;
}

}  // namespace ota::geo
