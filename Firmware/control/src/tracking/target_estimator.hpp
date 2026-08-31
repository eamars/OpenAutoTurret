#pragma once
// Target-state estimator (architecture §13): a constant-angular-velocity
// alpha-beta filter on the base-frame line-of-sight angles, with forward
// prediction to the intended actuation time (§13.3).
//
// State: [azimuth, elevation, azimuth_rate, elevation_rate] (radians / rad/s).
// Measurement: a valid base-frame LOS (azimuth, elevation) from the camera.
//
// This is pure geometry/filtering — it consumes LOS angles and does NOT touch
// CAN, the camera, or the motor driver.
#include <cmath>
#include <cstdint>

namespace ota {
namespace tracking {

inline double wrap_angle(double a) {
  // Wrap to [-pi, pi].
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct TargetEstimatorConfig {
  double alpha = 0.8;   // position smoothing (0<alpha<1; higher = responsive)
  double beta = 0.3;    // velocity smoothing (0<beta<1; higher = responsive)
  double dt_min_s = 1e-3;  // guard against division by ~0 in the velocity update
};

class TargetEstimator {
 public:
  explicit TargetEstimator(TargetEstimatorConfig cfg = {}) : cfg_(cfg) {}

  void reset() { initialized_ = false; }
  bool initialized() const { return initialized_; }

  // Consume one valid base-frame LOS measurement at capture time t_ns.
  void update(double azimuth_rad, double elevation_rad, std::int64_t t_ns);

  // Predict the LOS at (future or current) time t_ns.
  void predict(std::int64_t t_ns, double& azimuth_rad, double& elevation_rad) const;

  // Current (last-updated) state.
  double azimuth() const { return azimuth_; }
  double elevation() const { return elevation_; }
  double azimuth_rate() const { return azimuth_rate_; }
  double elevation_rate() const { return elevation_rate_; }

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
};

}  // namespace tracking
}  // namespace ota
