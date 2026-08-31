#pragma once
// Turret (gimbal) kinematics (architecture §9.2, §10.3): transform a
// camera-frame line of sight into the base frame, given the gimbal joint
// angles. The camera->pitch extrinsic (R_PC, §10.3) is configurable so a
// calibrated fiducial-board estimate can replace the ideal-aligned default
// without changing the transform math.
//
// Frame conventions (right-handed):
//   base (B):  X forward, Y left,  Z up
//   camera (C): X right,  Y down,  Z forward (optical axis)  [OpenCV/IMX500]
//
// Gimbal chain: base -> yaw (about base Z) -> pitch (about gimbal Y) -> camera.
// So a camera-frame vector is brought to the base frame by
//     r_B = R_z(q_yaw) * R_y(q_pitch) * R_PC * r_C.
#include <cmath>

#include "geometry/vec3.hpp"

namespace ota {
namespace geo {

struct TurretKinematics {
  // Camera -> pitch-frame rotation (fixed extrinsic, §10.3). Identity by
  // default; use aligned() for the ideal-aligned camera.
  Mat3 R_PC{};

  // The ideal-aligned camera: optical axis along the pitch frame's forward
  // axis, camera X right / Y down / Z forward -> base X forward / Y left / Z up.
  static TurretKinematics aligned() {
    TurretKinematics k;
    // Maps camera (x right, y down, z forward) to base (x fwd, y left, z up).
    k.R_PC = Mat3{0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
    return k;
  }

  // Camera-frame LOS -> base-frame LOS.
  Vec3 ray_to_base(const Vec3& r_cam, double q_yaw_rad, double q_pitch_rad) const {
    const Mat3 R = Mat3::rot_z(q_yaw_rad) * Mat3::rot_y(q_pitch_rad) * R_PC;
    return (R * r_cam).normalized();
  }

  // Base-frame LOS -> (azimuth, elevation) in radians.
  //   azimuth   = atan2(Y, X)   (0 = forward, + = left)
  //   elevation = atan2(Z, sqrt(X^2+Y^2)) (0 = level, + = up)
  static void base_ray_to_los(const Vec3& r_base, double& azimuth_rad,
                              double& elevation_rad) {
    azimuth_rad = std::atan2(r_base.y, r_base.x);
    elevation_rad = std::atan2(r_base.z, std::sqrt(r_base.x * r_base.x +
                                                   r_base.y * r_base.y));
  }
};

}  // namespace geo
}  // namespace ota
