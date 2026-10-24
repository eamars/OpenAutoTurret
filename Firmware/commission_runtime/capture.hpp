#pragma once

// Acquisition infrastructure, separate from the mathematical controller. Kernel
// RX timestamps are host receipt times, never claimed to be motor sample times.
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include <linux/can.h>
#include <sys/socket.h>
#include "can/can_transport.hpp"
#include "capture/journal.hpp"

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
  // Returns only once the kernel stamps at queue time; construct before bind,
  // since anything already queued is discarded unread (and counted).
  explicit TimestampedReceiver(int fd);
  bool receive(Receipt& out); // false only for EAGAIN; other failures throw
  uint32_t kernel_drops() const; // includes loss not yet delivered in ancillary data
  uint32_t startup_discarded() const { return discarded_; }
 private:
  bool dequeue(can_frame& wire, timespec& stamp, uint32_t& socket_drops); // false only for EAGAIN
  int fd_;
  ClockBracket origin_;
  uint32_t drops_{}, discarded_{};
  int64_t previous_ns_{};
};

// The journal writer is ota_core's (control/src/capture/journal.hpp): controld writes the same
// journals for its calibration sessions (ADR-005 D7).
using capture::Journal;

std::string receipt_json(const Receipt&, const std::string& axis, uint64_t sequence);
} // namespace ota::commission
