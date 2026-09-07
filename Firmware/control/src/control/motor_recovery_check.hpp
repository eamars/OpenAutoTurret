#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include "control/motor_backend.hpp"

namespace ota {

// Evidence gate shared by hardware and simulator. No motion is introduced to
// prove standstill: require a full second within a 0.25 degree position window.
// Drive velocity is deliberately not used: its idle noise exceeds this window.
class MotorRecoveryCheck {
 public:
  void begin(TimeNs now) { *this = {}; active_ = true; started_ = now; }
  void cancel() { active_ = false; }
  MotorBackend::Transition observe(TimeNs now,
      const std::array<AxisSnapshot, kAxisCount>& samples, double max_temp,
      std::string& detail) {
    using Result = MotorBackend::Transition;
    if (!active_) { detail = "no motor recovery is active"; return Result::Failed; }
    bool healthy = std::isfinite(max_temp) && max_temp > 0 && max_temp <= 75;
    detail = healthy ? "verifying stopped feedback on both axes" : "invalid temperature gate";
    for (int i = 0; i < kAxisCount; ++i) {
      const auto& s = samples[i];
      if (!s.has_feedback || s.rx_ns <= started_ || s.rx_ns > now ||
          now - s.rx_ns > 50'000'000 || !s.disabled || s.faults ||
          !std::isfinite(s.q_rad) || !std::isfinite(s.temp_c) ||
          s.temp_c > max_temp) {
        healthy = false;
        detail = std::string(axis_name(static_cast<AxisId>(i))) +
            ": waiting for fresh, disabled, fault-free feedback below temperature limit";
      }
    }
    if (!healthy) { stable_since_ = 0; counts_ = {}; }
    else {
      if (!stable_since_) {
        stable_since_ = now;
        for (int i = 0; i < kAxisCount; ++i) low_[i] = high_[i] = samples[i].q_rad;
        counts_ = {}; last_rx_ = {};
      }
      for (int i = 0; i < kAxisCount; ++i) {
        const auto& s = samples[i];
        if (s.rx_ns != last_rx_[i]) { ++counts_[i]; last_rx_[i] = s.rx_ns; }
        low_[i] = std::min(low_[i], s.q_rad); high_[i] = std::max(high_[i], s.q_rad);
        if (high_[i] - low_[i] > .25 * kDeg2Rad) {
          stable_since_ = 0;
          detail = std::string(axis_name(static_cast<AxisId>(i))) + ": motor has not settled";
        }
      }
      if (stable_since_ && now - stable_since_ >= 1'000'000'000 &&
          counts_[0] >= 10 && counts_[1] >= 10) {
        active_ = false; detail = "both motors verified disabled and healthy";
        return Result::Complete;
      }
    }
    if (now - started_ >= 5'000'000'000) {
      active_ = false; detail = "motor recovery timed out: " + detail;
      return Result::Failed;
    }
    return Result::Pending;
  }
 private:
  bool active_ = false;
  TimeNs started_ = 0, stable_since_ = 0;
  std::array<TimeNs, kAxisCount> last_rx_{};
  std::array<unsigned, kAxisCount> counts_{};
  std::array<double, kAxisCount> low_{}, high_{};
};
}  // namespace ota
