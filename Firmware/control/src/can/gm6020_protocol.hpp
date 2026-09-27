#pragma once

// GM6020 guide v1.4: standard CAN, signed big-endian voltage, modulo encoder.
// Current/temperature remain raw: the guide does not establish feedback scaling.
#include <cmath>
#include <cstdint>
#include <numbers>
#include <stdexcept>

#include "can/can_transport.hpp"

namespace ota::gm6020 {

struct Feedback {
  uint16_t angle_count{};
  int16_t speed_rpm{};
  int16_t current_raw{};
  uint8_t temperature_raw{};
  TimeNs rx_ns{};
  double speed_rad_s() const { return speed_rpm * (2.0 * std::numbers::pi / 60.0); }
};

inline uint16_t be16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
inline int16_t signed_be16(const uint8_t* p) {
  const auto n = be16(p);
  return static_cast<int16_t>(n < 32768 ? int(n) : int(n) - 65536);
}

inline bool decode(const can::RawFrame& frame, uint8_t id, Feedback& out) {
  if (id < 1 || id > 7 || frame.extended || frame.rtr || frame.error ||
      frame.dlc != 8 || frame.id != uint32_t(0x204 + id)) return false;
  const auto angle = be16(frame.data);
  if (angle > 8191) return false;
  out = {angle, signed_be16(frame.data + 2), signed_be16(frame.data + 4),
         frame.data[6], frame.rx_ns};
  return true;
}

// Own the entire group frame: only this motor's slot is populated.
// Multi-motor groups require a single group writer; never combine independent frames.
inline can::RawFrame voltage_frame(uint8_t motor_id, int voltage) {
  if (motor_id < 1 || motor_id > 7 || voltage < -25000 || voltage > 25000)
    throw std::invalid_argument("GM6020 motor ID or voltage outside v1.4 range");
  can::RawFrame frame;
  frame.extended = false;
  frame.id = motor_id <= 4 ? 0x1ff : 0x2ff;
  const auto slot = (motor_id - 1) % 4;
  const auto value = static_cast<uint16_t>(static_cast<int16_t>(voltage));
  frame.data[slot * 2] = static_cast<uint8_t>(value >> 8);
  frame.data[slot * 2 + 1] = static_cast<uint8_t>(value);
  return frame;
}

class UnwrappedEncoder {
 public:
  static constexpr double kRadiansPerCount = 2.0 * std::numbers::pi / 8192.0;
  // A session reference, never a claim of mechanical homing or absolute heading.
  void reset() { initialized_ = false; valid_ = true; counts_ = 0; stamp_ = 0; }
  bool update(uint16_t count, TimeNs stamp) {
    if (!valid_ || count > 8191 || stamp <= 0) return invalidate();
    if (!initialized_) {
      previous_ = count; stamp_ = stamp; initialized_ = true; return true;
    }
    const double dt = (stamp - stamp_) * 1e-9;
    // At the rated 320 rpm, an 80 ms observation gap spans under half a
    // revolution, so the modulo direction remains unambiguous. The Pi has
    // exhibited a 65.8 ms receive scheduling gap while the motor was still;
    // the old 50 ms ceiling latched a false encoder failure.
    if (dt <= 0 || dt > 0.08) return invalidate();
    int delta = int(count) - int(previous_);
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    // 40 rad/s bounds the rated 320 rpm with margin; gaps cannot hide half a turn.
    if (std::abs(delta) * kRadiansPerCount > 40.0 * dt + 2 * kRadiansPerCount)
      return invalidate();
    counts_ += delta; previous_ = count; stamp_ = stamp; return true;
  }
  bool valid() const { return initialized_ && valid_; }
  double relative_rad() const { return counts_ * kRadiansPerCount; }
 private:
  bool invalidate() { valid_ = false; return false; }
  bool initialized_{false}, valid_{true};
  uint16_t previous_{};
  int64_t counts_{};
  TimeNs stamp_{};
};
}  // namespace ota::gm6020
