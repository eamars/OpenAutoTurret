#pragma once
#include <algorithm>
#include <cmath>
#include "common/time.hpp"
#include "can/gm6020_friction.hpp"

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
    late_cycle_ = false;
    friction_.reset(); friction_output_ = {}; previous_output_ = 0; calibrated_cap_ = 0;
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
  // A/(rad/s) and A/rad respectively: they may NOT be the voltage gains divided
  // by a constant, because volts and torque-current do not divide by one back-EMF here -- the
  // drive closes its own current loop underneath us.
  double update_amps(double reference_rad_s, double position, TimeNs now,
                     double max_reference_rad_s, double ceiling_a,
                     double kp_a, double ki_a, double rx_velocity_rad_s = NAN,
                     const FrictionConfig* friction = nullptr, bool moving_intent = false,
                     uint64_t rx_sequence = 0) {
    // Negated compares, so a non-finite ceiling or gain invalidates instead of slipping through.
    if (friction && friction->enabled && calibrated_cap_ == 0) calibrated_cap_ = ceiling_a;
    if (!(ceiling_a > 0) || !std::isfinite(ceiling_a) || !(kp_a > 0) || !(ki_a >= 0) ||
        (friction && friction->enabled && (!friction->valid(calibrated_cap_) || ceiling_a > calibrated_cap_))) {
      valid_ = false; return 0;
    }
    double out = 0;
    if (!step(reference_rad_s, position, now, max_reference_rad_s, ceiling_a, kp_a, ki_a, out,
              rx_velocity_rad_s, friction, moving_intent, rx_sequence))
      return 0;
    return out;
  }
  bool valid() const { return valid_; }
  bool late_cycle() const { return late_cycle_; }
  double integral() const { return integral_; }
  double velocity_rad_s() const { return velocity_; }
  const FrictionOutput& friction_output() const { return friction_output_; }

 private:
  // Shared PI: identical arithmetic for both units, so switching modes cannot quietly change the
  // loop's dynamics. Returns false (and latches `valid_` false) on any unusable input.
  bool step(double reference_rad_s, double position, TimeNs now, double max_reference_rad_s,
            double output_ceiling, double kp, double ki, double& out, double rx_velocity_rad_s = NAN,
            const FrictionConfig* friction = nullptr, bool moving_intent = false, uint64_t rx_sequence = 0) {
    const double dt = (now - previous_time_) * 1e-9;
    if (!valid_ || now <= previous_time_ || !std::isfinite(reference_rad_s) ||
        !std::isfinite(position) || !std::isfinite(max_reference_rad_s) ||
        !std::isfinite(output_ceiling) || !std::isfinite(kp) || !std::isfinite(ki) ||
        max_reference_rad_s <= 0 || kp <= 0 || ki < 0 || dt > .100 ||
        std::abs(reference_rad_s) > max_reference_rad_s) {
      valid_ = false; out = 0; return false;
    }
    const double measured = (position - previous_position_) / dt;
    if (std::isfinite(rx_velocity_rad_s)) velocity_ = rx_velocity_rad_s;
    else velocity_ += dt / (.050 + dt) * (measured - velocity_);
    previous_position_ = position; previous_time_ = now;
    const double error = reference_rad_s - velocity_;
    // A single late cycle is recoverable when the caller has independently
    // checked fresh feedback/heartbeat. Rebase timing, never integrate a long
    // scheduling gap. Invalid time and >100 ms loss remain latched failures.
    late_cycle_ = dt > .020;
    if (friction && friction->enabled) {
      constexpr double direction_threshold = .02 * 3.14159265358979323846 / 180.;
      const int direction = reference_rad_s > direction_threshold ? 1 : reference_rad_s < -direction_threshold ? -1 : 0;
      friction_output_ = friction_.update(*friction, moving_intent,
          direction, position, velocity_, rx_sequence, dt, calibrated_cap_);
      const double ff = std::clamp(friction_output_.feedforward_target_a,-output_ceiling,output_ceiling);
      friction_output_.feedforward_target_a = ff;
      // New directional intent must not spend seconds unwinding the previous
      // move's integral. The final slew bound preserves output continuity.
      if (friction_output_.new_attempt) integral_ = 0;
      else if (friction_output_.integral_handoff)
        integral_ = std::clamp(previous_output_ - kp*error - ff, -output_ceiling, output_ceiling);
      const double candidate = std::clamp(integral_ + (late_cycle_ ? 0 : ki*error*dt),
                                           -output_ceiling, output_ceiling);
      const double candidate_output = kp*error + candidate + ff;
      const double step = friction->output_slew_a_per_s * std::min(dt,.020);
      const auto limit = [&](double value) {
        return std::clamp(std::clamp(value,previous_output_-step,previous_output_+step),
                          -output_ceiling,output_ceiling);
      };
      const double delivered = limit(candidate_output);
      // Conditional integration uses the FINAL current and slew constraints.
      // A reducing thermal cap always outranks slew, leaving both current signs
      // available for braking. Quiet hold retains its supporting integral.
      if (!late_cycle_ && ((candidate_output-delivered)*error <= 0)) integral_ = candidate;
      out = limit(kp*error + integral_ + ff);
      previous_output_ = out;
      return true;
    }
    if (late_cycle_) {
      out = std::clamp(kp * error + integral_, -output_ceiling, output_ceiling);
      previous_output_ = out;
      return true;
    }
    const double candidate = std::clamp(integral_ + ki * error * dt, -output_ceiling, output_ceiling);
    const double output = kp * error + candidate;
    // Integrate only when unsaturated or moving the saturated output inward.
    if (std::abs(output) <= output_ceiling || (output > output_ceiling && error < 0) ||
        (output < -output_ceiling && error > 0)) integral_ = candidate;
    out = std::clamp(kp * error + integral_, -output_ceiling, output_ceiling);
    previous_output_ = out;
    return true;
  }

  bool valid_{false};
  bool late_cycle_{false};
  double previous_position_{}, velocity_{}, integral_{};
  TimeNs previous_time_{};
  YawFrictionCompensation friction_;
  FrictionOutput friction_output_{};
  double previous_output_ = 0;
  double calibrated_cap_ = 0;
};
}  // namespace ota::gm6020
