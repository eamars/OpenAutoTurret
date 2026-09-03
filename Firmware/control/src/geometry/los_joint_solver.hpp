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

// Pick the joint-space branch of an angle that lies nearest a reference angle.
//
// Joint angles repeat every 2*pi; a station's soft limits do not. This station's yaw
// travel is [-0.394, 5.588] rad, a range that does not straddle +/-pi, so a solved
// angle unwrapped into (-pi, pi] can name a perfectly reachable direction while
// sitting below the soft minimum - where the position envelope then clamps it to the
// limit. Seen on the station 2026-09-04: AUTO_TRACK held for 12 s with a target 182 px
// off the reticle, reason "tracking", q_ref_yaw pinned to exactly q_soft_min_yaw_rad
// (-0.3940), because the solver had answered -4.27 rad for what is +2.013 rad on the
// branch the limits use. Nothing was unreachable and nothing had expired; the turret
// was parked at a travel limit because of a choice of representation.
//
// Only the branch changes here, never the direction: the axis still points where the
// solver said it should. Pitch is deliberately left alone - its travel never wraps.
inline double wrap_near(double angle_rad, double reference_rad) {
  double a = tracking::wrap_angle(angle_rad);
  while (a - reference_rad > M_PI) a -= 2.0 * M_PI;   // a is the long way above
  while (reference_rad - a > M_PI) a += 2.0 * M_PI;   // a is the long way below
  return a;
}

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
    angular_decomposition(az_rad, el_rad, q_yaw_rad, q_pitch_rad);
    return refine_to_los(az_rad, el_rad, q_yaw_rad, q_pitch_rad, max_residual_rad);
  }

  // The same solve, refined from a pose we already believe: the one the turret is at.
  //
  // This exists because the analytic seed below is exact only for the ideal aligned
  // gimbal, and this station never operates in the half of travel where that ideal
  // holds. Its pitch soft limits are -74.7 to -4.9 deg, so sin(q_pitch) < 0 everywhere,
  // which makes the axis azimuth q_yaw + 180 deg and its elevation 90 deg + q_pitch.
  // Measured on the station 2026-09-04: asked to solve for the very direction the
  // camera was already looking at (az -77.30 deg, el +51.74 deg, from q_yaw 1.7924 /
  // q_pitch -0.6678), angular_decomposition seeded q_yaw -1.3492 / q_pitch -0.9030,
  // whose optical axis is 180 deg away in azimuth - and because the seeded elevation
  // comes out as 90 deg - el, exactly ORTHOGONAL to the direction it was given (the
  // measured separation is pi/2 to 1e-16). The twelve refinement steps cannot recover
  // from a start that far out, so AUTO_TRACK drove away from its own
  // target and the target left the frame. Seeding from the current pose is both
  // numerically sound and physically right: a tracker chases what it was just looking
  // at, and "the short way round" stops being something the caller has to repair.
  bool solve_from_pose(double az_rad, double el_rad, double seed_q_yaw_rad,
                       double seed_q_pitch_rad, double& q_yaw_rad, double& q_pitch_rad,
                       double max_residual_rad = 1e-3) const {
    q_yaw_rad = seed_q_yaw_rad;
    q_pitch_rad = seed_q_pitch_rad;
    return refine_to_los(az_rad, el_rad, q_yaw_rad, q_pitch_rad, max_residual_rad);
  }

  // Refine a joint pose until the optical axis matches the requested LOS. The seed
  // determines WHICH solution is found, so prefer solve_from_pose() over solve().
  bool refine_to_los(double az_rad, double el_rad, double& q_yaw_rad,
                     double& q_pitch_rad, double max_residual_rad) const {
    const Vec3 target{std::cos(el_rad) * std::cos(az_rad),
                      std::cos(el_rad) * std::sin(az_rad), std::sin(el_rad)};
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
