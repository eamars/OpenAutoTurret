#pragma once

// Acquisition infrastructure, separate from the mathematical controller. Kernel
// RX timestamps are host receipt times, never claimed to be motor sample times.
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include "can/can_transport.hpp"

namespace ota::commission {
int64_t monotonic_ns();

struct ClockBracket {
  int64_t offset_ns{}, uncertainty_ns{}; // realtime minus monotonic
  static ClockBracket sample();
};

struct Receipt {
  can::RawFrame frame;
  int64_t kernel_realtime_ns{}, kernel_monotonic_ns{}, dequeue_ns{};
  int64_t clock_uncertainty_ns{};
  uint32_t socket_drops{}, drop_delta{};
};

class TimestampedReceiver {
 public:
  // Does not own fd. The same recvmsg/ancillary path serves SocketCAN and the
  // explicitly synthetic local datagram harness. Both require classic CAN MTU.
  explicit TimestampedReceiver(int fd);
  bool receive(Receipt& out); // false only for EAGAIN; other failures throw
  uint32_t kernel_drops() const; // includes loss not yet delivered in ancillary data
 private:
  int fd_;
  ClockBracket origin_;
  uint32_t drops_{};
  int64_t previous_ns_{};
};

// One acquisition/event-loop producer; one disk writer. A full queue fails the
// session instead of delaying acquisition or silently discarding evidence.
class Journal {
 public:
  Journal(const std::string& path, const std::string& header, size_t capacity = 4096);
  ~Journal();
  Journal(const Journal&) = delete;
  Journal& operator=(const Journal&) = delete;
  bool append(const std::string& json_line);
  bool finish(const std::string& footer);
  bool healthy() const { return !failed_.load(); }
  uint64_t written() const { return written_.load(); }
  size_t high_water() const { return high_water_.load(); }
  const char* failure_reason() const;
 private:
  struct Line { uint32_t size{}; char bytes[4096]{}; };
  bool write_line(const char*, size_t);
  bool write_bytes(const char*, size_t);
  void drain();
  int fd_{-1};
  std::vector<Line> queue_;
  std::atomic<size_t> head_{0}, tail_{0};
  std::atomic<bool> failed_{false}, done_{false};
  std::atomic<uint64_t> written_{0};
  std::atomic<size_t> high_water_{0};
  std::atomic<int> failure_code_{0};
  std::thread writer_;
  bool finished_{false};
};

std::string receipt_json(const Receipt&, const std::string& axis, uint64_t sequence);
} // namespace ota::commission
