#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "common/types.hpp"

namespace ota::control {

// Latest state decoded from the launcher-owned imu-bno085 NDJSON trace.
// Timestamps use CLOCK_MONOTONIC nanoseconds, matching AxisSnapshot::rx_ns.
// This is an observer API; it is deliberately separate from BaseOrientation.
struct ImuTraceSnapshot {
  bool trace_open = false;
  bool trace_ended = false;
  bool gap_seen = false;
  bool tare_valid = false;
  bool game_rv_present = false;
  bool gyro_present = false;
  bool game_rv_fresh = false;
  bool gyro_fresh = false;
  bool game_rv_tared = false;
  uint32_t generation = 0;
  uint32_t tare_generation = 0;
  uint32_t game_rv_generation = 0;
  uint32_t gyro_generation = 0;
  uint32_t game_rv_accuracy = 0;
  std::array<double, 4> tare_xyzw{};
  std::array<double, 4> game_rv_xyzw{};
  std::array<double, 4> relative_xyzw{};
  std::array<double, 3> gyro_rad_s{};
  TimeNs tare_rx_ns = 0;
  TimeNs game_rv_sample_ns = 0;
  TimeNs game_rv_rx_ns = 0;
  TimeNs gyro_sample_ns = 0;
  TimeNs gyro_rx_ns = 0;
  TimeNs last_trace_rx_ns = 0;
};

// Tails the acquisition process's line-buffered NDJSON output on a worker
// thread. `snapshot()` is lock-bounded memory access: it performs no file I/O.
class ImuTraceIngest {
 public:
  static constexpr TimeNs kDefaultFreshnessNs = 100'000'000;

  explicit ImuTraceIngest(TimeNs freshness_ns = kDefaultFreshnessNs)
      : freshness_ns_(freshness_ns > 0 ? freshness_ns : kDefaultFreshnessNs) {}
  ~ImuTraceIngest() { stop(); }

  ImuTraceIngest(const ImuTraceIngest&) = delete;
  ImuTraceIngest& operator=(const ImuTraceIngest&) = delete;

  // Start polling a path supplied by the launcher (typically OTA_IMU_TRACE).
  // A missing file is retried until stop(); errors are returned only for an
  // invalid lifecycle request or unusable path.
  bool start(std::string path, std::string& err);
  void stop();
  bool running() const { return running_.load(); }
  ImuTraceSnapshot snapshot(TimeNs now_ns) const;

 private:
  void reader_loop();
  void consume_line(const std::string& line, TimeNs read_ns);

  const TimeNs freshness_ns_;
  std::string path_;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex state_mu_;
  ImuTraceSnapshot state_;
};

}  // namespace ota::control
