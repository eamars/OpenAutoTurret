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
}

void TargetEstimator::update(double azimuth_rad, double elevation_rad,
                             std::int64_t t_ns) {
  if (!initialized_) {
    init(azimuth_rad, elevation_rad, t_ns);
    return;
  }
  double dt = static_cast<double>(t_ns - last_update_ns_) * 1e-9;
  if (dt < cfg_.dt_min_s) dt = cfg_.dt_min_s;

  // Predict to the measurement time (constant velocity).
  const double az_pred = azimuth_ + azimuth_rate_ * dt;
  const double el_pred = elevation_ + elevation_rate_ * dt;

  // Innovation (azimuth wrapped to avoid the +-pi jump).
  const double d_az = wrap_angle(azimuth_rad - az_pred);
  const double d_el = elevation_rad - el_pred;

  // Alpha-beta update.
  azimuth_ = wrap_angle(az_pred + cfg_.alpha * d_az);
  elevation_ = el_pred + cfg_.alpha * d_el;
  azimuth_rate_ += (cfg_.beta / dt) * d_az;
  elevation_rate_ += (cfg_.beta / dt) * d_el;
  last_update_ns_ = t_ns;
}

void TargetEstimator::predict(std::int64_t t_ns, double& azimuth_rad,
                              double& elevation_rad) const {
  if (!initialized_) {
    azimuth_rad = 0.0;
    elevation_rad = 0.0;
    return;
  }
  const double dt = static_cast<double>(t_ns - last_update_ns_) * 1e-9;
  azimuth_rad = wrap_angle(azimuth_ + azimuth_rate_ * dt);
  elevation_rad = elevation_ + elevation_rate_ * dt;
}

}  // namespace tracking
}  // namespace ota
