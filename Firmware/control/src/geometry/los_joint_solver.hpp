#pragma once
// From target line of sight to desired joints (architecture §14).
//
// The target estimator produces a desired viewing ray in the base frame
// (azimuth, elevation). For a two-axis gimbal this solves for (q_yaw, q_pitch)
// such that the camera optical axis aligns with the desired ray, using the
// ACTUAL frame transforms (R_PC + yaw + pitch) so small build misalignments can
// be calibrated. For the aligned ideal mechanism this reduces to the angular
// decomposition  q_yaw = azimuth, q_pitch = -elevation  (exact, verified by
// round-trip against TurretKinematics).
//
// Pure geometry — no CAN, no camera, no motor driver.
#include <cmath>

#include "geometry/turret_kinematics.hpp"
#include "tracking/target_estimator.hpp"  // wrap_angle

namespace ota {
namespace geo {

class LosJointSolver {
 public:
  explicit LosJointSolver(TurretKinematics kin) : kin_(std::move(kin)) {}

  // Base-frame optical axis for the given joints (unit vector).
  Vec3 optical_axis(double q_yaw_rad, double q_pitch_rad) const {
    const Vec3 v = (Mat3::rot_z(q_yaw_rad) * Mat3::rot_y(q_pitch_rad) *
                    kin_.R_PC * Vec3(0.0, 0.0, 1.0));
    return v.normalized();
  }

  // Solve for the joints that point the optical axis at (azimuth, elevation).
  // Starts from the angular-decomposition guess and refines against the actual
  // R_PC (so a calibrated extrinsic with small misalignment is handled).
  // Returns false if the LOS is not reachable within `max_residual_rad`.
  bool solve(double az_rad, double el_rad, double& q_yaw_rad,
             double& q_pitch_rad, double max_residual_rad = 1e-3) const {
    const Vec3 target{std::cos(el_rad) * std::cos(az_rad),
                      std::cos(el_rad) * std::sin(az_rad), std::sin(el_rad)};
    angular_decomposition(az_rad, el_rad, q_yaw_rad, q_pitch_rad);
    // Gradient refinement against the actual transform (a few steps; the gimbal
    // is well-conditioned for small extrinsic misalignment).
    for (int i = 0; i < 12; ++i) {
      const Vec3 r = optical_axis(q_yaw_rad, q_pitch_rad);
      const Vec3 e = target - r;
      if (e.norm() < 1e-10) break;
      const double h = 1e-4;
      const Vec3 rp = optical_axis(q_yaw_rad, q_pitch_rad + h);
      const Vec3 rq = optical_axis(q_yaw_rad + h, q_pitch_rad);
      q_yaw_rad += e.dot(rq - r) / h;
      q_pitch_rad += e.dot(rp - r) / h;
    }
    const Vec3 r = optical_axis(q_yaw_rad, q_pitch_rad);
    double c = r.dot(target.normalized());
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    const double residual = std::acos(c);
    return residual < max_residual_rad;
  }

  // Angular-decomposition solution (exact for the aligned ideal gimbal, §14).
  static void angular_decomposition(double az_rad, double el_rad,
                                    double& q_yaw_rad, double& q_pitch_rad) {
    q_yaw_rad = tracking::wrap_angle(az_rad);
    q_pitch_rad = -el_rad;
  }

  const TurretKinematics& kinematics() const { return kin_; }

 private:
  TurretKinematics kin_;
};

}  // namespace geo
}  // namespace ota
