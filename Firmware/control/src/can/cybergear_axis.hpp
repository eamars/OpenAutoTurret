#pragma once
// Per-axis runtime state (architecture §8.3, §8.4).
//
// The CAN RX thread is the sole writer (SeqLock latest snapshot + history
// append); control/CLI threads are sole readers.
#include <atomic>

#include "can/cybergear_protocol.hpp"
#include "common/motor_state_history.hpp"
#include "common/seqlock.hpp"
#include "common/time.hpp"

namespace ota::can {

// Latest decoded COMM_TYPE_2 feedback for one axis.
struct AxisLatest {
  TimeNs rx_ns{0};        // host monotonic time of the most recent frame
  bool has_feedback{false};
  float q_rad{0.0f};      // angle field, decoded (±4*pi nominal range)
  float v_rad_s{0.0f};
  float torque_nm{0.0f};
  float temp_c{0.0f};
  uint16_t faults{0};
  uint8_t mode{0};        // 0 reset, 1 cali, 2 motor/running
  uint64_t frames{0};     // count of feedback frames seen
};

class AxisRuntime {
 public:
  static constexpr std::size_t kDefaultHistoryCapacity = 1024;  // ~1 s @ ~100 Hz

  // Non-explicit on purpose: std::array<AxisRuntime, N>{} value-init.
  AxisRuntime(std::size_t history_capacity = kDefaultHistoryCapacity)
      : history_(history_capacity) {}

  // Rebuild the history ring with a new capacity. Must be called before the
  // RX thread starts (no writers active); the system does this in open() so
  // the configured history_capacity is honored.
  void reset_history(std::size_t capacity) { history_.reset(capacity); }

  // Writer (RX thread) only.
  void on_feedback(const cybergear::Feedback& fb, TimeNs rx_ns) {
    AxisLatest l{};
    l.rx_ns = rx_ns;
    l.has_feedback = true;
    l.q_rad = fb.angle_rad;
    l.v_rad_s = fb.vel_rad_s;
    l.torque_nm = fb.torque_nm;
    l.temp_c = fb.temp_c;
    l.faults = fb.faults;
    l.mode = static_cast<uint8_t>(fb.mode);
    l.frames = ++frame_count_;
    latest_.write(l);
    history_.add(rx_ns, fb.angle_rad, fb.vel_rad_s);
  }

  // One-shot register sample (diagnostics only).
  void on_register_sample(TimeNs t_ns, float q_rad, float v_rad_s) {
    history_.add(t_ns, q_rad, v_rad_s);
  }

  // Readers.
  bool latest(AxisLatest& out) const { return latest_.read(out); }
  MotorStateHistory& history() { return history_; }
  const MotorStateHistory& history() const { return history_; }

  bool feedback_fresh(TimeNs now_ns, TimeNs max_age_ns) const {
    AxisLatest l;
    if (!latest_.read(l)) return false;
    if (!l.has_feedback) return false;
    return (now_ns - l.rx_ns) <= max_age_ns;
  }

 private:
  std::atomic<uint64_t> frame_count_{0};  // sole writer: RX thread
  SeqLock<AxisLatest> latest_;
  MotorStateHistory history_;
};

}  // namespace ota::can
