#pragma once
// Independent of the diagnostic read/logging loop. Shares the one CAN owner.
// A trip latches command authority off and repeatedly requests both motors stop.
// This is a host watchdog, not a substitute for a drive-side communication timeout.
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>
#include "can/cybergear_system.hpp"

namespace ota::tools {
class CommissioningWatchdog {
 public:
  enum class Reason { None, Deadline, Heartbeat, Feedback, Fault, Excursion, Temperature };
  struct Limits {
    AxisId axis;
    double q_min, q_max, temperature_max;
    TimeNs duration_ns;
    TimeNs heartbeat_ns = 100'000'000;
    TimeNs feedback_ns = 100'000'000;
  };
  CommissioningWatchdog(can::CyberGearSystem& system, Limits limits)
      : system_(system), limits_(limits), started_(now_monotonic_ns()), heartbeat_(started_) {
    worker_ = std::thread([this] { monitor(); });
  }
  ~CommissioningWatchdog() { finish(); }
  CommissioningWatchdog(const CommissioningWatchdog&) = delete;
  CommissioningWatchdog& operator=(const CommissioningWatchdog&) = delete;

  void heartbeat() { heartbeat_.store(now_monotonic_ns()); }
  Reason reason() const { return reason_.load(); }
  TimeNs trip_ns() const { return trip_ns_.load(); }
  uint64_t stop_failures() const { return stop_failures_.load(); }
  static const char* name(Reason reason) {
    switch (reason) {
      case Reason::None: return "none";
      case Reason::Deadline: return "duration deadline";
      case Reason::Heartbeat: return "diagnostic loop deadline";
      case Reason::Feedback: return "feedback stale";
      case Reason::Fault: return "motor fault";
      case Reason::Excursion: return "position excursion";
      case Reason::Temperature: return "temperature limit";
    }
    return "unknown";
  }
  // Only bounded command sends belong here. Register waits and filesystem I/O
  // MUST stay outside: otherwise a stuck read would also block the watchdog.
  template<class Action> void command(Action action) {
    std::lock_guard lock(command_mutex_);
    if (finished_ || reason() != Reason::None)
      throw std::runtime_error(std::string("watchdog inhibited command: ") + name(reason()));
    const auto failure = check(now_monotonic_ns());
    if (failure != Reason::None) {
      trip(failure);
      throw std::runtime_error(std::string("watchdog inhibited command: ") + name(reason()));
    }
    action();
  }
  void finish() {
    {
      std::lock_guard lock(command_mutex_);
      if (!finished_) { finished_ = true; stop_both(); }
    }
    if (worker_.joinable()) worker_.join();
  }

 private:
  Reason check(TimeNs now) const {
    if (now - started_ >= limits_.duration_ns) return Reason::Deadline;
    if (now - heartbeat_.load() >= limits_.heartbeat_ns) return Reason::Heartbeat;
    can::AxisLatest latest{};
    if (!system_.axis(limits_.axis).latest(latest) || !latest.has_feedback ||
        latest.rx_ns > now || now - latest.rx_ns >= limits_.feedback_ns) return Reason::Feedback;
    if (latest.faults) return Reason::Fault;
    if (!std::isfinite(latest.q_rad) || latest.q_rad <= limits_.q_min || latest.q_rad >= limits_.q_max)
      return Reason::Excursion;
    if (!std::isfinite(latest.temp_c) || latest.temp_c >= limits_.temperature_max)
      return Reason::Temperature;
    return Reason::None;
  }
  void stop_both() {
    for (const auto axis : {AxisId::Pitch, AxisId::Yaw})
      if (!system_.send_stop(axis)) ++stop_failures_;
  }
  void trip(Reason reason) {
    if (reason_.load() == Reason::None) {
      reason_.store(reason);
      trip_ns_.store(now_monotonic_ns());
    }
    stop_both();
  }
  void monitor() {
    for (;;) {
      {
        std::lock_guard lock(command_mutex_);
        if (finished_) return;
        if (reason() != Reason::None) stop_both();
        else {
          const auto failure = check(now_monotonic_ns());
          if (failure != Reason::None) trip(failure);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(reason() == Reason::None ? 5 : 20));
    }
  }
  can::CyberGearSystem& system_;
  Limits limits_;
  TimeNs started_;
  std::atomic<TimeNs> heartbeat_, trip_ns_{0};
  std::atomic<Reason> reason_{Reason::None};
  std::atomic<uint64_t> stop_failures_{0};
  std::mutex command_mutex_;
  bool finished_ = false;
  std::thread worker_;
};
}  // namespace ota::tools
