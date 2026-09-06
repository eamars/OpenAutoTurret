#include "tracking/target_estimator.hpp"

#include <algorithm>

namespace ota {
namespace tracking {

void TargetEstimator::init(double az, double el, std::int64_t t_ns) {
  azimuth_ = wrap_angle(az);
  elevation_ = el;
  azimuth_rate_ = 0.0;
  elevation_rate_ = 0.0;
  last_update_ns_ = t_ns;
  initialized_ = true;
  const double r = cfg_.measurement_sigma_rad * cfg_.measurement_sigma_rad;
  for (auto& p : covariance_)
    p = {r, 0.0, cfg_.initial_rate_sigma_rad_s * cfg_.initial_rate_sigma_rad_s};
}

bool TargetEstimator::update(double azimuth_rad, double elevation_rad,
                             std::int64_t t_ns, double az_variance, double el_variance) {
  diagnostics_.last_accepted = false;
  if (!std::isfinite(azimuth_rad) || !std::isfinite(elevation_rad) ||
      std::fabs(elevation_rad) > M_PI / 2.0 || t_ns < 0 ||
      !std::isfinite(az_variance) || !std::isfinite(el_variance) ||
      az_variance < 0 || el_variance < 0 ||
      (initialized_ && t_ns <= last_update_ns_)) {
    ++diagnostics_.rejected;
    return false;
  }
  const double base_r = cfg_.measurement_sigma_rad * cfg_.measurement_sigma_rad;
  const double r[2] = {std::max(base_r, az_variance), std::max(base_r, el_variance)};
  diagnostics_.measurement_variance_az = r[0];
  diagnostics_.measurement_variance_el = r[1];
  if (!initialized_ || (t_ns-last_update_ns_)*1e-9 > cfg_.max_prediction_s) {
    diagnostics_.gap_resets += initialized_ ? 1 : 0;
    init(azimuth_rad, elevation_rad, t_ns);
    covariance_[0].pp = r[0]; covariance_[1].pp = r[1];
    diagnostics_.last_accepted = true;
    ++diagnostics_.accepted;
    return true;
  }
  double dt = static_cast<double>(t_ns - last_update_ns_) * 1e-9;
  if (dt < cfg_.dt_min_s) dt = cfg_.dt_min_s;

  // Predict to the measurement time (constant velocity).
  const double az_pred = azimuth_ + azimuth_rate_ * dt;
  const double el_pred = elevation_ + elevation_rate_ * dt;

  // Innovation (azimuth wrapped to avoid the +-pi jump).
  const double d_az = wrap_angle(azimuth_rad - az_pred);
  const double d_el = elevation_rad - el_pred;
  diagnostics_.innovation_az = d_az;
  diagnostics_.innovation_el = d_el;

  if (cfg_.use_kalman) {
    Covariance p[2] = {propagated(covariance_[0], dt), propagated(covariance_[1], dt)};
    const double innovation[2] = {d_az, d_el};
    const double nis = d_az*d_az/(p[0].pp+r[0]) + d_el*d_el/(p[1].pp+r[1]);
    diagnostics_.mahalanobis = std::isfinite(nis) ? nis : 1e12;
    // A run of large innovations raises maneuver uncertainty without injecting
    // the rejected coordinates or advancing the accepted capture timestamp.
    if (nis > cfg_.soft_gate_chi2)
      diagnostics_.process_noise_scale = std::min(cfg_.max_process_noise_scale,
                                                diagnostics_.process_noise_scale * 2.0);
    else
      diagnostics_.process_noise_scale = 1.0 + (diagnostics_.process_noise_scale-1.0)*0.95;
    if (nis > cfg_.hard_gate_chi2 || !std::isfinite(nis)) {
      ++diagnostics_.rejected;
      return false;
    }
    double position[2] = {az_pred, el_pred};
    double rate[2] = {azimuth_rate_, elevation_rate_};
    for (int axis = 0; axis < 2; ++axis) {
      const double s = p[axis].pp + r[axis];
      const double kp = p[axis].pp/s, kv = p[axis].pv/s, a = 1-kp;
      position[axis] += kp*innovation[axis];
      rate[axis] += kv*innovation[axis];
      // Joseph form keeps covariance positive under repeated small updates.
      covariance_[axis] = {
        a*a*p[axis].pp + kp*kp*r[axis],
        a*(p[axis].pv-kv*p[axis].pp) + kp*kv*r[axis],
        p[axis].vv - 2*kv*p[axis].pv + kv*kv*(p[axis].pp+r[axis])};
    }
    azimuth_ = wrap_angle(position[0]);
    elevation_ = std::clamp(position[1], -M_PI/2.0, M_PI/2.0);
    azimuth_rate_ = rate[0]; elevation_rate_ = rate[1];
    last_update_ns_ = t_ns;
    diagnostics_.last_accepted = true;
    ++diagnostics_.accepted;
    return true;
  }

  // Explicit compatibility profile for recorded A/B comparison.
  azimuth_ = wrap_angle(az_pred + cfg_.alpha * d_az);
  elevation_ = el_pred + cfg_.alpha * d_el;
  azimuth_rate_ += (cfg_.beta / dt) * d_az;
  elevation_rate_ += (cfg_.beta / dt) * d_el;
  last_update_ns_ = t_ns;
  diagnostics_.last_accepted = true;
  ++diagnostics_.accepted;
  return true;
}

TargetEstimator::Covariance TargetEstimator::propagated(Covariance p, double dt) const {
  const double q = cfg_.angular_accel_sigma_rad_s2 * cfg_.angular_accel_sigma_rad_s2 *
                   diagnostics_.process_noise_scale;
  const double d2 = dt*dt, d3 = d2*dt, d4 = d2*d2;
  return {p.pp + 2*dt*p.pv + d2*p.vv + q*d4/4,
          p.pv + dt*p.vv + q*d3/2, p.vv + q*d2};
}

double TargetEstimator::position_variance(int axis, std::int64_t t_ns) const {
  if (!initialized_ || axis < 0 || axis > 1) return 0;
  const double dt = std::clamp((t_ns-last_update_ns_)*1e-9, 0.0, cfg_.max_prediction_s);
  return propagated(covariance_[axis], dt).pp;
}

void TargetEstimator::predict(std::int64_t t_ns, double& azimuth_rad,
                              double& elevation_rad) const {
  if (!initialized_) {
    azimuth_rad = 0.0;
    elevation_rad = 0.0;
    return;
  }
  const double dt = std::clamp(static_cast<double>(t_ns - last_update_ns_) * 1e-9,
                               0.0, cfg_.max_prediction_s);
  azimuth_rad = wrap_angle(azimuth_ + azimuth_rate_ * dt);
  elevation_rad = std::clamp(elevation_ + elevation_rate_ * dt, -M_PI / 2.0, M_PI / 2.0);
}

}  // namespace tracking
}  // namespace ota
