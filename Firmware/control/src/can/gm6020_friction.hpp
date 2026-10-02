#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ota::gm6020 {

// Disabled until both directions have been measured on the production mechanism.
struct FrictionConfig {
  bool enabled{false};
  double positive_breakaway_a{};
  double negative_breakaway_a{};
  double positive_run_a{};
  double negative_run_a{};
  double timeout_s{};
  double motion_displacement_rad{};
  double stationary_velocity_rad_s{};
  std::uint32_t fresh_samples{};
  double output_slew_a_per_s{};

  bool valid(double passed_current_cap_a) const {
    return std::isfinite(passed_current_cap_a) && passed_current_cap_a > 0 &&
           std::isfinite(positive_breakaway_a) && positive_breakaway_a >= 0 && positive_breakaway_a <= passed_current_cap_a &&
           std::isfinite(negative_breakaway_a) && negative_breakaway_a >= 0 && negative_breakaway_a <= passed_current_cap_a &&
           std::isfinite(positive_run_a) && positive_run_a >= 0 && positive_run_a <= passed_current_cap_a &&
           std::isfinite(negative_run_a) && negative_run_a >= 0 && negative_run_a <= passed_current_cap_a &&
           std::isfinite(timeout_s) && timeout_s > 0 && timeout_s <= 2.0 &&
           std::isfinite(motion_displacement_rad) && motion_displacement_rad > 0 &&
           std::isfinite(stationary_velocity_rad_s) && stationary_velocity_rad_s > 0 && fresh_samples > 0 &&
           std::isfinite(output_slew_a_per_s) && output_slew_a_per_s > 0;
  }
};

enum class FrictionState { Idle, Breakaway, Moving };

struct FrictionOutput {
  // Additive friction bias in amperes in every state. The root loop owns final slew/current
  // limits and anti-windup; new_attempt lets it clear inherited integral exactly once.
  double feedforward_target_a{};
  double previous_feedforward_a{};
  double next_feedforward_a{};
  FrictionState state{FrictionState::Idle};
  bool new_attempt{};  // set once at the beginning of an authorized direction lease
  bool attempt_active{};
  bool attempt_exhausted{};
  bool integral_handoff{};  // true at BREAKAWAY->MOVING or when intent is removed
  bool waiting_for_stationary{};
};

// Call once per control cycle with actual unwrapped position and RX sequence. Repeated RX
// snapshots do not count as evidence. An intent lease is (moving_intent, requested_direction);
// renewing the same pair never starts another pulse. Quiet hold therefore has exactly zero aid.
class YawFrictionCompensation {
 public:
  FrictionOutput update(const FrictionConfig& cfg, bool moving_intent, int requested_direction,
                        double position_rad, double measured_velocity_rad_s,
                        std::uint64_t rx_sequence, double dt_s,
                        double passed_current_cap_a) {
    FrictionOutput out;
    if (!std::isfinite(position_rad) ||
        !std::isfinite(measured_velocity_rad_s) || !std::isfinite(dt_s) || dt_s <= 0 ||
        (moving_intent && requested_direction != -1 && requested_direction != 0 && requested_direction != 1)) { reset(); return out; }
    if (!cfg.enabled || !cfg.valid(passed_current_cap_a)) {
      out.previous_feedforward_a = target_a_;
      out.integral_handoff = target_a_ != 0 || state_ != FrictionState::Idle;
      reset(); out.next_feedforward_a = 0;
      return out;
    }
    if (!moving_intent) {
      const bool was_active = state_ != FrictionState::Idle || target_a_ != 0;
      out.previous_feedforward_a = target_a_;
      state_ = FrictionState::Idle; target_a_ = 0; episode_active_ = false;
      waiting_stationary_ = exhausted_ = false; elapsed_s_ = 0;
      out.integral_handoff = was_active;
      out.feedforward_target_a = 0;
      out.next_feedforward_a = 0;
      out.state = state_;
      return out;
    }

    // A zero-direction sample within a continuing intent removes assist but does not
    // re-arm the attempt. The same lease resumes its existing state when direction returns.
    if (requested_direction == 0) {
      const bool was_active = target_a_ != 0;
      out.previous_feedforward_a = target_a_; target_a_ = 0;
      out.integral_handoff = was_active; out.next_feedforward_a = 0;
      out.feedforward_target_a = 0; out.state = state_;
      out.attempt_active = state_ == FrictionState::Breakaway;
      out.attempt_exhausted = exhausted_; out.waiting_for_stationary = waiting_stationary_;
      return out;
    }

    const bool new_lease = !episode_active_ || requested_direction != lease_direction_;
    if (new_lease) {
      const bool reversing = (last_direction_valid_ && requested_direction != lease_direction_) ||
                             measured_velocity_rad_s * requested_direction < -cfg.stationary_velocity_rad_s;
      episode_active_ = true; lease_direction_ = requested_direction; last_direction_valid_ = true;
      if (reversing) {
        out.previous_feedforward_a = target_a_;
        state_ = FrictionState::Idle; target_a_ = 0; waiting_stationary_ = true;
        station_samples_ = 0; exhausted_ = false; out.integral_handoff = true;
      } else {
        waiting_stationary_ = false;
        begin_attempt(position_rad);
        out.new_attempt = true;
      }
    }

    const bool fresh = !last_sequence_valid_ || rx_sequence != last_sequence_;
    if (fresh) {
      last_sequence_ = rx_sequence; last_sequence_valid_ = true;
      if (waiting_stationary_) {
        if (std::abs(measured_velocity_rad_s) <= cfg.stationary_velocity_rad_s) ++station_samples_;
        else station_samples_ = 0;
        if (station_samples_ >= cfg.fresh_samples) { waiting_stationary_ = false; begin_attempt(position_rad); out.new_attempt = true; }
      } else if (state_ == FrictionState::Breakaway &&
                 (position_rad - attempt_start_position_) * lease_direction_ >= cfg.motion_displacement_rad) {
        if (++motion_samples_ >= cfg.fresh_samples) {
          out.previous_feedforward_a = target_a_;
          state_ = FrictionState::Moving; out.integral_handoff = true;
        }
      } else if (state_ == FrictionState::Breakaway) {
        motion_samples_ = 0;
      }
    }

    if (state_ == FrictionState::Breakaway) {
      elapsed_s_ += dt_s;
      if (elapsed_s_ >= cfg.timeout_s) {
        out.previous_feedforward_a = target_a_;
        state_ = FrictionState::Idle; exhausted_ = true; target_a_ = 0; out.integral_handoff = true;
      } else {
        const double mag = requested_direction > 0 ? cfg.positive_breakaway_a : cfg.negative_breakaway_a;
        target_a_ = requested_direction * mag;
      }
    } else if (state_ == FrictionState::Moving) {
      const double mag = requested_direction > 0 ? cfg.positive_run_a : cfg.negative_run_a;
      target_a_ = requested_direction * mag;
    } else if (waiting_stationary_ || exhausted_) {
      target_a_ = 0;
    }
    out.feedforward_target_a = target_a_;
    if (out.integral_handoff) out.next_feedforward_a = target_a_;
    else out.next_feedforward_a = out.feedforward_target_a;
    out.state = state_; out.attempt_active = state_ == FrictionState::Breakaway;
    out.attempt_exhausted = exhausted_; out.waiting_for_stationary = waiting_stationary_;
    return out;
  }

  void reset() { episode_active_ = last_direction_valid_ = last_sequence_valid_ = waiting_stationary_ = exhausted_ = false; state_ = FrictionState::Idle; target_a_ = 0; }

 private:
  void begin_attempt(double position) { state_ = FrictionState::Breakaway; attempt_start_position_ = position; elapsed_s_ = 0; motion_samples_ = 0; exhausted_ = false; }

  FrictionState state_{FrictionState::Idle};
  bool episode_active_{}, last_direction_valid_{}, last_sequence_valid_{}, waiting_stationary_{}, exhausted_{};
  int lease_direction_{};
  std::uint64_t last_sequence_{};
  std::uint32_t station_samples_{}, motion_samples_{};
  double target_a_{}, attempt_start_position_{}, station_start_position_{}, elapsed_s_{};
};
}  // namespace ota::gm6020
