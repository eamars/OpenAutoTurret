// OpenAutoTurret — telemetry and logging (architecture §6.3, §43).
//
// In-memory, allocation-bounded (ring buffers) telemetry that the control loop
// fills every cycle. No file I/O in the control loop (§46): the black-box ring
// and logs are in memory and can be persisted on fault or sampled to a log file
// at a low rate by a separate (non-real-time) thread.
//
//   §43.1  high-rate control log  — near control rate, both axes
//   §43.3  event log              — structured state/safety events
//   §43.4  black-box ring buffer  — rolling history for fault diagnosis
//   §6.3   telemetry snapshot     — the 10-20 Hz published snapshot
//
// Pure data structures — no CAN, no camera, no motor driver.
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "common/types.hpp"
#include "control/safety_supervisor.hpp"
#include "tracking/tracking_state_machine.hpp"

namespace ota {
namespace telemetry {

// §6.3 telemetry snapshot (published at 10-20 Hz).
struct TelemetrySnapshot {
  TimeNs timestamp_ns = 0;
  // Per-axis actuals.
  double q_yaw_rad = 0.0;
  double v_yaw_rad_s = 0.0;
  double effort_yaw = 0.0;
  double q_pitch_rad = 0.0;
  double v_pitch_rad_s = 0.0;
  double effort_pitch = 0.0;
  // Per-axis references.
  double q_ref_yaw_rad = 0.0;
  double q_ref_pitch_rad = 0.0;
  // Target / tracking.
  tracking::TrackState track_state = tracking::TrackState::ReadyHold;
  double target_confidence = 0.0;
  bool tracking_active = false;
  double target_az_world_rad = 0.0;   // world-frame LOS (valid when tracking)
  double target_el_world_rad = 0.0;
  // Installation orientation (§29/§30, Phase 7): base tilt relative to level,
  // from the active R_W_B. installation_source is the PoseSource enum value
  // (kept as an int to keep this header decoupled from the calibration module).
  double base_roll_rad = 0.0;
  double base_pitch_rad = 0.0;
  double base_yaw_rad = 0.0;
  bool installation_calibrated = false;  // true iff a real (non-identity) pose
  int8_t installation_source = 0;        // PoseSource::Unknown default
  // Safety / timing.
  SafetyAction safety_action = SafetyAction::Allow;
  int64_t feedback_age_ms = 0;
  int64_t control_cycle_us = 0;
};

// §43.1 high-rate control log record (one per cycle, both axes).
struct ControlLogRecord {
  TimeNs timestamp_ns = 0;
  double q_actual[kAxisCount] = {0.0, 0.0};
  double v_actual[kAxisCount] = {0.0, 0.0};
  double effort[kAxisCount] = {0.0, 0.0};
  double q_ref[kAxisCount] = {0.0, 0.0};
  double soft_limit_distance[kAxisCount] = {0.0, 0.0};
  SafetyAction safety_action = SafetyAction::Allow;
  int64_t feedback_age_ms = 0;
  int64_t cycle_duration_us = 0;
  tracking::TrackState track_state = tracking::TrackState::ReadyHold;
};

// §43.3 structured event.
enum class Event : uint8_t {
  TargetAcquired,
  TargetLost,
  CoastingStarted,
  BrakingStarted,
  SearchStarted,
  SearchStopped,
  HoldEntered,
  SafetyBrake,
  LimitProximity,
  CanFault,
  MotorFault,
  LoopOverrun,
  HomingStarted,
  HomingComplete,
  CalibrationCommit,
  Shutdown,
};

struct EventRecord {
  TimeNs timestamp_ns = 0;
  Event event = Event::TargetAcquired;
  std::string detail;
};

// Fixed-capacity ring buffer (allocation-bounded, §46 "no unbounded
// allocation").
template <typename T, std::size_t N>
class RingBuffer {
 public:
  void push(const T& item) {
    buf_[write_ % N] = item;
    ++write_;
    if (count_ < N) ++count_;
  }
  // Oldest -> newest.
  std::vector<T> all() const {
    std::vector<T> out;
    out.reserve(count_);
    if (count_ < N) {
      for (std::size_t i = 0; i < count_; ++i) out.push_back(buf_[i]);
    } else {
      const std::size_t start = write_ % N;
      for (std::size_t i = 0; i < N; ++i)
        out.push_back(buf_[(start + i) % N]);
    }
    return out;
  }
  const T& newest() const { return buf_[(write_ - 1) % N]; }
  std::size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  void clear() { write_ = 0; count_ = 0; }

 private:
  std::array<T, N> buf_{};
  std::size_t write_ = 0;
  std::size_t count_ = 0;
};

// The telemetry store, filled by the control loop.
class Telemetry {
 public:
  static constexpr std::size_t kControlLogCap = 4096;   // ~20 s at 200 Hz
  static constexpr std::size_t kEventCap = 512;
  static constexpr std::size_t kBlackBoxCap = 8192;     // ~40 s at 200 Hz

  // §6.3 the current-cycle snapshot (overwritten each cycle).
  void set_snapshot(const TelemetrySnapshot& s) { snapshot_ = s; }
  const TelemetrySnapshot& snapshot() const { return snapshot_; }

  // §43.1 high-rate control log.
  void push_control(const ControlLogRecord& r) { control_log_.push(r); }
  const RingBuffer<ControlLogRecord, kControlLogCap>& control_log() const {
    return control_log_;
  }

  // §43.3 event log.
  void push_event(TimeNs t, Event e, std::string detail = "") {
    event_log_.push(EventRecord{t, e, std::move(detail)});
  }
  const RingBuffer<EventRecord, kEventCap>& event_log() const {
    return event_log_;
  }

  // §43.4 black-box ring (rolling, for fault diagnosis).
  void push_blackbox(const ControlLogRecord& r) { blackbox_.push(r); }
  const RingBuffer<ControlLogRecord, kBlackBoxCap>& blackbox() const {
    return blackbox_;
  }

  // Clear everything (e.g. on reset).
  void clear() {
    control_log_.clear();
    event_log_.clear();
    blackbox_.clear();
    snapshot_ = TelemetrySnapshot{};
  }

 private:
  TelemetrySnapshot snapshot_;
  RingBuffer<ControlLogRecord, kControlLogCap> control_log_;
  RingBuffer<EventRecord, kEventCap> event_log_;
  RingBuffer<ControlLogRecord, kBlackBoxCap> blackbox_;
};

}  // namespace telemetry
}  // namespace ota
