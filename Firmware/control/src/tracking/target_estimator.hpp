#pragma once
// Target-state estimator (architecture §13): a constant-angular-velocity
// Kalman filter on the base-frame line-of-sight angles, with forward
// prediction to the intended actuation time (§13.3).
//
// State: [azimuth, elevation, azimuth_rate, elevation_rate] (radians / rad/s).
// Measurement: a valid base-frame LOS (azimuth, elevation) from the camera.
//
// This is pure geometry/filtering — it consumes LOS angles and does NOT touch
// CAN, the camera, or the motor driver.
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <array>

namespace ota {
namespace tracking {

inline double wrap_angle(double a) {
  // Wrap to [-pi, pi].
  a = std::fmod(a, 2.0 * M_PI);
  if (a > M_PI) a -= 2.0 * M_PI;
  if (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct TargetEstimatorConfig {
  // These are provisional noise priors, not commissioned station measurements.
  bool use_kalman = true;
  double measurement_sigma_rad = 0.004;
  double angular_accel_sigma_rad_s2 = 0.35;
  double initial_rate_sigma_rad_s = 0.5;
  double soft_gate_chi2 = 5.99;
  double hard_gate_chi2 = 9.21;
  double max_process_noise_scale = 100.0;
  double alpha = 0.8;   // position smoothing (0<alpha<1; higher = responsive)
  double beta = 0.3;    // velocity smoothing (0<beta<1; higher = responsive)
  double dt_min_s = 1e-3;  // guard against division by ~0 in the velocity update
  double max_prediction_s = 0.35;  // bounded coast, including image age and actuation lead
};

class TargetEstimator {
 public:
  explicit TargetEstimator(TargetEstimatorConfig cfg = {}) : cfg_(cfg) {}

  void reset() {
    initialized_ = false;
    azimuth_ = elevation_ = azimuth_rate_ = elevation_rate_ = 0.0;
    last_update_ns_ = 0;
    covariance_ = {};
    diagnostics_ = {};
  }
  bool initialized() const { return initialized_; }

  // Consume one valid base-frame LOS measurement at capture time t_ns.
  bool update(double azimuth_rad, double elevation_rad, std::int64_t t_ns,
              double az_variance = 0.0, double el_variance = 0.0);

  struct Diagnostics {
    double innovation_az = 0, innovation_el = 0, mahalanobis = 0;
    double process_noise_scale = 1.0;
    double measurement_variance_az = 0, measurement_variance_el = 0;
    uint64_t accepted = 0, rejected = 0, gap_resets = 0;
    bool last_accepted = false;
  };
  const Diagnostics& diagnostics() const { return diagnostics_; }
  double position_variance(int axis, std::int64_t t_ns) const;
  double rate_variance(int axis) const {
    return initialized_ && axis >= 0 && axis < 2 ? covariance_[axis].vv : 0.0;
  }

  // Predict the LOS at (future or current) time t_ns.
  void predict(std::int64_t t_ns, double& azimuth_rad, double& elevation_rad) const;

  // Current (last-updated) state.
  double azimuth() const { return azimuth_; }
  double elevation() const { return elevation_; }
  double azimuth_rate() const { return azimuth_rate_; }
  double elevation_rate() const { return elevation_rate_; }
  std::int64_t state_timestamp_ns() const { return last_update_ns_; }
  bool prediction_valid(std::int64_t t_ns) const {
    return initialized_ && t_ns >= last_update_ns_ &&
           static_cast<double>(t_ns - last_update_ns_) * 1e-9 <= cfg_.max_prediction_s;
  }

  const TargetEstimatorConfig& config() const { return cfg_; }

 private:
  void init(double az, double el, std::int64_t t_ns);

  TargetEstimatorConfig cfg_;
  bool initialized_ = false;
  double azimuth_ = 0.0;
  double elevation_ = 0.0;
  double azimuth_rate_ = 0.0;
  double elevation_rate_ = 0.0;
  std::int64_t last_update_ns_ = 0;
  struct Covariance { double pp = 0, pv = 0, vv = 0; };
  std::array<Covariance, 2> covariance_{};
  Diagnostics diagnostics_{};
  Covariance propagated(Covariance p, double dt) const;
};

}  // namespace tracking
}  // namespace ota
