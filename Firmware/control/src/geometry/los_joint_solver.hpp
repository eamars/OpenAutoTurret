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
#include <algorithm>

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

// Choose an equivalent angle that physically exists in this joint's travel.
// The shortest angular representation may be across a mechanical end stop.
inline bool equivalent_in_range(double angle, double reference, double low, double high,
                                double& result) {
  if (!std::isfinite(angle) || !std::isfinite(reference) ||
      !std::isfinite(low) || !std::isfinite(high) || low>high) return false;
  const double period=2*M_PI;
  const double first=std::ceil((low-angle-1e-10)/period);
  const double last=std::floor((high-angle+1e-10)/period);
  if(first>last) return false;
  const double turn=std::clamp(std::round((reference-angle)/period),first,last);
  result=std::clamp(angle+period*turn,low,high);
  return true;
}

class LosJointSolver {
 public:
  explicit LosJointSolver(TurretKinematics kin, Vec3 sight_camera = Vec3{0, 0, 1})
      : kin_(std::move(kin)), sight_camera_(sight_camera.normalized()),
        sight_valid_(std::isfinite(sight_camera.x) && std::isfinite(sight_camera.y) &&
                     std::isfinite(sight_camera.z) && sight_camera.z > 0 &&
                     std::isfinite(sight_camera.norm()) && sight_camera.norm() > 0) {}

  // Base-frame configured sight axis (optical axis by default).
  Vec3 optical_axis(double q_yaw_rad, double q_pitch_rad) const {
    const Vec3 v = (Mat3::rot_z(q_yaw_rad) * Mat3::rot_y(q_pitch_rad) *
                    kin_.R_PC * sight_camera_);
    return v.normalized();
  }

  // Enumerate the two optical-axis pitch branches, then choose only joint
  // representations inside calibrated travel. This also works for a target
  // opposite the current view, where a local gradient can vanish.
  bool solve_within_limits(double az, double el, double seed_yaw, double seed_pitch,
      double yaw_low, double yaw_high, double pitch_low, double pitch_high,
      double& yaw, double& pitch) const {
    if(!sight_valid_ || !std::isfinite(az) || !std::isfinite(el) || std::abs(el)>M_PI/2) return false;
    const Vec3 body=(kin_.R_PC*sight_camera_).normalized();
    const double radius=std::hypot(body.x,body.z);
    if(radius<1e-9 || std::abs(std::sin(el))>radius+1e-9) return false;
    const double root=std::acos(std::clamp(std::sin(el)/radius,-1.0,1.0));
    const double offset=std::atan2(body.x,body.z);
    double best=1e100; bool found=false;
    for(double sign : {-1.,1.}) {
      double qp,qy;
      if(!equivalent_in_range(sign*root-offset,seed_pitch,pitch_low,pitch_high,qp)) continue;
      const double x=body.x*std::cos(qp)+body.z*std::sin(qp);
      const double raw_yaw=std::hypot(x,body.y)<1e-9 ? seed_yaw : az-std::atan2(body.y,x);
      if(!equivalent_in_range(raw_yaw,seed_yaw,yaw_low,yaw_high,qy)) continue;
      const double cost=(qy-seed_yaw)*(qy-seed_yaw)+(qp-seed_pitch)*(qp-seed_pitch);
      if(cost<best) { best=cost; yaw=qy; pitch=qp; found=true; }
    }
    return found;
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
    if (!sight_valid_ || !std::isfinite(az_rad) || !std::isfinite(el_rad) ||
        !std::isfinite(q_yaw_rad) || !std::isfinite(q_pitch_rad)) return false;
    const Vec3 target{std::cos(el_rad) * std::cos(az_rad),
                      std::cos(el_rad) * std::sin(az_rad), std::sin(el_rad)};
    // Damped Gauss-Newton accounts for the different joint sensitivities. The
    // previous unscaled gradient converged too slowly at this station's pitch:
    // a reachable live target failed after 12 iterations and the caller's
    // analytic fallback selected the opposite yaw / positive-pitch branch.
    for (int i = 0; i < 12; ++i) {
      const Vec3 r = optical_axis(q_yaw_rad, q_pitch_rad);
      const Vec3 e = target - r;
      if (e.norm() < 1e-10) break;
      const double h = 1e-4;
      const Vec3 jp = (optical_axis(q_yaw_rad, q_pitch_rad + h) - r) / h;
      const Vec3 jy = (optical_axis(q_yaw_rad + h, q_pitch_rad) - r) / h;
      const double a = jy.dot(jy) + 1e-8, b = jy.dot(jp), c = jp.dot(jp) + 1e-8;
      const double det = a*c - b*b;
      if (!(det > 1e-12)) return false;
      const double ey = e.dot(jy), ep = e.dot(jp);
      double dy = (c*ey-b*ep)/det, dp = (a*ep-b*ey)/det;
      // Bound each numerical step near a singular pose. Motion is separately
      // bounded by the trajectory generator and the calibrated envelope.
      const double step = std::hypot(dy, dp);
      if (step > .35) { dy *= .35/step; dp *= .35/step; }
      q_yaw_rad += dy;
      q_pitch_rad += dp;
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
  Vec3 sight_camera_;
  bool sight_valid_;
};

}  // namespace geo
}  // namespace ota
