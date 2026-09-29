#pragma once
#include <algorithm>
#include <cmath>
#include "common/time.hpp"

namespace ota::gm6020 {
// Bounded commissioning PI loop, not payload-qualified production gains.
// Encoder-derived velocity avoids the drive's integer-rpm (6 deg/s) quantization.
//
// The loop is unit-agnostic on purpose: it turns a speed error into an OUTPUT, and the output's
// unit is whatever the caller's gains and ceiling are expressed in. Two wrappers say which unit
// this cycle is in, because "15000" means something completely different on a voltage frame
// (60 % of ±25000 counts) than on a current frame (where 15000 would be absurd amperes):
//   update()       -> raw drive voltage counts, bounded by the voltage frame's own ±25000
//   update_amps()  -> torque current in amperes, bounded by the host's current limit
// The 25000 bound therefore lives in the voltage wrapper only. It was in the shared body, which is
// how a current ceiling could never be expressed here at all.
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
    // The bound belongs to the voltage frame (guide v1.4 accepts +-25000), not to this loop.
    if (!(output_ceiling > 0) || output_ceiling > 25000.0) { valid_ = false; return 0; }
    double out = 0;
    if (!step(reference_rad_s, position, now, max_reference_rad_s, output_ceiling, kp, ki, out))
      return 0;
    return static_cast<int>(std::lround(out));
  }
  // Torque-current output. `ceiling_a` is the host-side clamp in amperes, and kp/ki are in
  // amperes per (rad/s) and amperes per radian-second: they may NOT be the voltage gains divided
  // by a constant, because volts and torque-current do not divide by one back-EMF here -- the
  // drive closes its own current loop underneath us.
  double update_amps(double reference_rad_s, double position, TimeNs now,
                     double max_reference_rad_s, double ceiling_a,
                     double kp_a, double ki_a) {
    // Negated compares, so a non-finite ceiling or gain invalidates instead of slipping through.
    if (!(ceiling_a > 0) || !std::isfinite(ceiling_a) || !(kp_a > 0) || !(ki_a >= 0)) {
      valid_ = false; return 0;
    }
    double out = 0;
    if (!step(reference_rad_s, position, now, max_reference_rad_s, ceiling_a, kp_a, ki_a, out))
      return 0;
    return out;
  }
  bool valid() const { return valid_; }
  double velocity_rad_s() const { return velocity_; }

 private:
  // Shared PI: identical arithmetic for both units, so switching modes cannot quietly change the
  // loop's dynamics. Returns false (and latches `valid_` false) on any unusable input.
  bool step(double reference_rad_s, double position, TimeNs now, double max_reference_rad_s,
            double output_ceiling, double kp, double ki, double& out) {
    const double dt = (now - previous_time_) * 1e-9;
    if (!valid_ || now <= previous_time_ || !std::isfinite(reference_rad_s) ||
        !std::isfinite(position) || !std::isfinite(max_reference_rad_s) ||
        !std::isfinite(output_ceiling) || !std::isfinite(kp) || !std::isfinite(ki) ||
        max_reference_rad_s <= 0 || kp <= 0 || ki < 0 || dt > .020 ||
        std::abs(reference_rad_s) > max_reference_rad_s) {
      valid_ = false; out = 0; return false;
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
    out = std::clamp(kp * error + integral_, -output_ceiling, output_ceiling);
    return true;
  }

  bool valid_{false};
  double previous_position_{}, velocity_{}, integral_{};
  TimeNs previous_time_{};
};
}  // namespace ota::gm6020
