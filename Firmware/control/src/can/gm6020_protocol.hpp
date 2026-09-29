#pragma once

// GM6020 guide v1.4: standard CAN, signed big-endian voltage, modulo encoder.
// Temperature stays raw: the guide establishes no °C scale. Current does not -- see
// current_a() below, whose scale is quoted from the same guide line the command side
// uses, and which was confirmed on this station's wire on 2026-09-29 (a 0.25 A command
// returned current_raw 1365; 0.25/3.0*16384 = 1365.3).
#include <cmath>
#include <cstdint>
#include <algorithm>
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
  // Amperes of torque current as reported by the drive, not an inferred N·m.
  // Defined below, next to the scale, so the two cannot drift apart.
  double current_a() const;
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

// Torque-current command (DJI guide v1.4). Motor ID 1 is the only mapping this station qualifies:
// the frame is 0x1FE, ID 1 lives in DATA[0:1] big-endian signed, and every remaining slot is zero
// -- a stale byte in another slot would command a motor that is not even fitted on this turret.
// Amperes are the unit everywhere above this boundary; raw int16 counts exist only here.
inline constexpr double kRawFullScale = 16384.0;      // documented numeric full scale
inline constexpr double kAmpsFullScale = 3.0;         // ... which is +-3.0 A of torque current
// Legacy software bound named after the 1.62 A rated condition. This is NOT
// qualification for sustained low-speed/stall use (the manual separately lists
// 0.90 A continuous stall). Keep the station's initial 0.8 A cap until measured
// duty/temperature qualification supports another value; protocol range is ±3 A.
inline constexpr double kMaxContinuousA = 1.62;
inline constexpr double kAmpsPerRaw = kAmpsFullScale / kRawFullScale;

// The drive reports its own torque current in the units it accepts on the command side, so
// the scale is shared by construction instead of repeated. Amperes, deliberately: turning
// this into N·m needs a torque constant the guide does not give, and an inferred N·m would
// be a different physical claim wearing a familiar label.
inline double Feedback::current_a() const { return current_raw * kAmpsPerRaw; }

// Pure encoding: no clamp, so the scale itself can be tested against the documented endpoints.
inline int current_raw_uncapped(double amps) {
  if (!std::isfinite(amps)) throw std::invalid_argument("GM6020 current command is not finite");
  const long raw = std::lround(amps / kAmpsPerRaw);
  if (raw < -16384 || raw > 16384)
    throw std::invalid_argument("GM6020 current command exceeds +-16384 raw counts");
  return static_cast<int>(raw);
}

// The host-side clamp, applied in amperes BEFORE encoding, and bounded by the motor's continuous
// rating: raising this number is an operator decision, and the software refuses to drift past
// 1.62 A on its own. limit_a must be finite and positive -- a missing limit is not "unlimited".
inline int current_raw_from_amps(double amps, double limit_a) {
  if (!std::isfinite(limit_a) || limit_a <= 0.0)
    throw std::invalid_argument("GM6020 host current limit must be finite and positive");
  if (limit_a > kMaxContinuousA)
    throw std::invalid_argument("GM6020 host current limit exceeds the 1.62 A continuous rating");
  return current_raw_uncapped(std::clamp(amps, -limit_a, limit_a));
}

// The zero request every startup, hold, fault and shutdown path must put on the wire. It carries
// no host clamp, because zero is inside every positive limit -- and it must not throw, because the
// fault path calls it with nothing above it to catch anything: a throwing zero frame turns a
// guard into std::terminate. A zeroed payload commands zero current to this motor and nothing at
// all to the other slots, which is exactly the claim we are willing to make about a motor that is
// not fitted on this turret.
inline can::RawFrame current_zero_frame(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 4)
    throw std::invalid_argument("GM6020 current frame 0x1FE covers IDs 1-4");
  can::RawFrame frame;
  frame.extended = false;
  frame.dlc = 8;
  frame.id = 0x1fe;
  return frame;
}

inline can::RawFrame current_frame(uint8_t motor_id, double amps, double limit_a) {
  if (motor_id != 1)
    throw std::invalid_argument("GM6020 current mode is qualified for motor ID 1 only");
  const int raw = current_raw_from_amps(amps, limit_a);
  can::RawFrame frame;   // RawFrame zero-initialises data, so unused slots cannot echo junk
  frame.extended = false;
  frame.dlc = 8;
  frame.id = 0x1fe;
  const auto value = static_cast<uint16_t>(static_cast<int16_t>(raw));
  frame.data[0] = static_cast<uint8_t>(value >> 8);
  frame.data[1] = static_cast<uint8_t>(value);
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
