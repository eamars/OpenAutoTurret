#pragma once
#include <algorithm>
#include <cmath>
#include "common/time.hpp"

namespace ota::gm6020 {
// Bounded commissioning PI loop, not payload-qualified production gains.
// Encoder-derived velocity avoids the drive's integer-rpm (6 deg/s) quantization.
class VelocityLoop {
 public:
  void reset(double position, TimeNs now) {
    previous_position_ = position; previous_time_ = now;
    velocity_ = integral_ = 0; valid_ = std::isfinite(position) && now > 0;
  }
  // Optional per-session limits let a bounded commissioning caller use a
  // separately approved envelope without changing legacy callers' defaults.
  int update(double reference_rad_s, double position, TimeNs now,
             double max_reference_rad_s = .14, double output_ceiling = 1500.0,
             double kp = 35000.0, double ki = 20000.0) {
    const double dt = (now - previous_time_) * 1e-9;
    if (!valid_ || now <= previous_time_ || !std::isfinite(reference_rad_s) ||
        !std::isfinite(position) || !std::isfinite(max_reference_rad_s) ||
        !std::isfinite(output_ceiling) || !std::isfinite(kp) || !std::isfinite(ki) ||
        max_reference_rad_s <= 0 || output_ceiling <= 0 || output_ceiling > 3000.0 ||
        kp <= 0 || ki < 0 || dt > .020 ||
        std::abs(reference_rad_s) > max_reference_rad_s) {
      valid_ = false; return 0;
    }
    const double measured = (position - previous_position_) / dt;
    velocity_ += dt / (.050 + dt) * (measured - velocity_);
    previous_position_ = position; previous_time_ = now;
    const double error = reference_rad_s - velocity_;
    const double candidate = std::clamp(integral_ + ki * error * dt, -output_ceiling, output_ceiling);
    const double output = kp * error + candidate;
    // Integrate only when unsaturated or moving the saturated output inward.
    if (std::abs(output) <= output_ceiling || (output > output_ceiling && error < 0) ||
        (output < -output_ceiling && error > 0)) integral_ = candidate;
    return static_cast<int>(std::lround(std::clamp(kp * error + integral_, -output_ceiling, output_ceiling)));
  }
  bool valid() const { return valid_; }
  double velocity_rad_s() const { return velocity_; }
 private:
  bool valid_{false};
  double previous_position_{}, velocity_{}, integral_{};
  TimeNs previous_time_{};
};
}  // namespace ota::gm6020
