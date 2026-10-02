#include "tracker.hpp"
#include "geometry/los_uncertainty.hpp"
#include <algorithm>
#include <cmath>

namespace ota::track {
bool valid(const TrackerParameters& p) {
  return valid(p.estimator) && valid(p.level1) && std::isfinite(p.timing.fixed_offset_s) &&
         std::abs(p.timing.fixed_offset_s)<0.5 && std::isfinite(p.timing.exposure_fraction) &&
         p.timing.exposure_fraction>=-1 && p.timing.exposure_fraction<=1 && std::isfinite(p.timing.row_time_s) &&
         std::isfinite(p.execution_horizon_s) && p.execution_horizon_s>=0 && p.execution_horizon_s<0.2 &&
         std::isfinite(p.pixel_sigma) && p.pixel_sigma>0 && std::isfinite(p.jacobian_det_min) && p.jacobian_det_min>0;
}

Tracker::Tracker(const TrackerParameters& p,const geo::TurretKinematics& kinematics,const geo::CameraIntrinsics& intrinsics,
                 const geo::Vec3& sight_camera,const Travel& travel)
    : p_(p),kinematics_(kinematics),camera_(intrinsics),solver_(kinematics,sight_camera),travel_(travel) {
  ok_=valid(p_) && intrinsics.valid() && estimator_.configure(p_.estimator) && level1_.configure(p_.level1) &&
      travel_.yaw_low<=travel_.yaw_high && travel_.pitch_low<travel_.pitch_high;
}

int64_t Tracker::observation_time(const PixelObservation& z) const {
  const double offset=p_.timing.fixed_offset_s+p_.timing.exposure_fraction*z.exposure_s+p_.timing.row_time_s*z.v;
  return z.sensor_ns+static_cast<int64_t>(std::llround(offset*1e9));
}

std::array<double,2> Tracker::los(double yaw,double pitch) const {
  std::array<double,2> a{};
  geo::TurretKinematics::base_ray_to_los(solver_.optical_axis(yaw,pitch),a[0],a[1]);
  return a;
}

bool Tracker::observe(const PixelObservation& z,double yaw,double pitch) {
  if (!ok_ || !std::isfinite(z.u) || !std::isfinite(z.v) || z.u<0 || z.v<0 || z.u>camera_.intrinsics.width ||
      z.v>camera_.intrinsics.height || !std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(z.exposure_s) ||
      z.exposure_s<0 || z.sensor_ns<=0) return false;
  // A different subject (or generation) never inherits the previous one's motion.
  if (have_identity_ && z.identity!=identity_) { estimator_.reset(); ++identity_changes_; }
  have_identity_=true; identity_=z.identity;
  const geo::Vec3 base=kinematics_.ray_to_base(camera_.pixel_to_ray(z.u,z.v),yaw,pitch);
  LosObservation o;
  o.t_ns=observation_time(z);
  geo::TurretKinematics::base_ray_to_los(base,o.az,o.el);
  const double su=z.sigma_u>0?z.sigma_u:p_.pixel_sigma, sv=z.sigma_v>0?z.sigma_v:p_.pixel_sigma;
  const auto variance=geo::pixel_los_variance(camera_,kinematics_,yaw,pitch,z.u,z.v,su,sv);
  o.var_az=variance[0]; o.var_el=variance[1];
  return estimator_.update(o);
}

JointGoal Tracker::joint_goal(const LosGoal& g,const std::array<double,2>& seed) const {
  JointGoal out;
  if (!g.position_valid) return out;
  const bool continuous=travel_.yaw_low==travel_.yaw_high;
  const double ylo=continuous?seed[0]-2*M_PI:travel_.yaw_low, yhi=continuous?seed[0]+2*M_PI:travel_.yaw_high;
  double yaw=seed[0],pitch=seed[1];
  if (!solver_.solve_within_limits(g.theta[0],g.theta[1],seed[0],seed[1],ylo,yhi,travel_.pitch_low,travel_.pitch_high,yaw,pitch))
    return out;  // unreachable: hold
  out.valid=true; out.q={yaw,pitch};
  // Joint rate from the LOS rate through the same geometry: J v = omega, J = d(LOS)/dq.
  const double h=1e-5;
  const auto yp=los(yaw+h,pitch), ym=los(yaw-h,pitch), pp=los(yaw,pitch+h), pm=los(yaw,pitch-h);
  const double j00=std::remainder(yp[0]-ym[0],2*M_PI)/(2*h), j01=std::remainder(pp[0]-pm[0],2*M_PI)/(2*h);
  const double j10=(yp[1]-ym[1])/(2*h), j11=(pp[1]-pm[1])/(2*h);
  const double det=j00*j11-j01*j10;
  if (!g.velocity_valid || !std::isfinite(det) || std::abs(det)<p_.jacobian_det_min) return out;
  const double vy=(j11*g.omega[0]-j01*g.omega[1])/det, vp=(-j10*g.omega[0]+j00*g.omega[1])/det;
  if (std::isfinite(vy) && std::isfinite(vp)) { out.v={vy,vp}; out.velocity_valid={true,true}; }
  return out;
}

void Tracker::engage(int64_t t_ns,const std::array<double,2>& q,const std::array<double,2>& v,const std::array<double,2>& a) {
  level1_.reset(t_ns,q,v,a);
}

TickRecord Tracker::tick(int64_t t_ns,const std::array<double,2>& q_measured) {
  TickRecord r;
  r.t_ns=t_ns; r.q_measured=q_measured;
  if (!ok_) return r;
  const int64_t horizon=static_cast<int64_t>(std::llround(p_.execution_horizon_s*1e9));
  r.los=estimator_.query(t_ns+horizon,p_.target_motion);
  const std::array<double,2> seed=level1_.initialized()?level1_.last().q:q_measured;
  r.joint=joint_goal(r.los,seed);
  r.reference=level1_.step(t_ns,r.joint,q_measured);
  for (int i=0;i<2;++i) {
    r.e_track[i]=r.joint.valid?r.joint.q[i]-r.reference.q[i]:0.;
    r.e_servo[i]=r.reference.q[i]-q_measured[i];
  }
  return r;
}

void Tracker::forget() {
  estimator_.reset();
  have_identity_=false;
}
}
