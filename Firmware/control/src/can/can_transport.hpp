#pragma once
// PHY-agnostic CAN transport boundary (architecture §8.1, incident
// 2026-09-02: the MCP2515 HAT is field-sick; development must continue on a
// different PHY).
//
// Everything ABOVE this interface is pure protocol (CyberGear framing,
// feedback tracking, register RPCs, homing, the control loop). Everything
// BELOW is pluggable:
//
//   SocketCanBus     — SocketCAN (MCP2515 Waveshare HAT, can0) [original]
//   YouseeTransport  — yousee USB-CAN AT-mode adapter (CH340 CDC-serial,
//                      'AT'+BE32(id<<3|ext)+dlc+data+CRLF framing)
//
// Selection is configuration only (can.backend + can.interface), so the PHY
// can be swapped at will without touching control code.
//
// Contract (identical for every implementation):
//   * frames carry 29-bit extended ids WITHOUT any flag bits in .id;
//   * the frame callback runs on an internal RX thread and must be fast;
//   * send() is thread-safe and never blocks the caller for long;
//   * stats() must count rx/tx/failures — supervision depends on it;
//   * can_state() may return Unknown when the PHY cannot report it
//     (feedback-staleness supervision covers the failure either way).
#include <cstdint>
#include <functional>
#include <string>

#include "can/can_netlink.hpp"  // CanIfState
#include "common/time.hpp"

namespace ota::can {

struct RawFrame {
  uint32_t id{0};
  uint8_t dlc{8};
  uint8_t data[8]{};
  ota::TimeNs rx_ns{0};  // monotonic timestamp at receipt
};

struct BusStats {
  uint64_t rx_frames{0};
  uint64_t rx_error_frames{0};  // decode/framing losses (PHY corruption hint)
  uint64_t tx_frames{0};
  uint64_t tx_failed{0};
  ota::TimeNs last_rx_ns{0};
};

class CanTransport {
 public:
  using FrameCallback = std::function<void(const RawFrame&)>;
  virtual ~CanTransport() = default;

  // Open the link and start the RX thread as one operation (idempotent
  // stop()/start() cycles must be possible for recovery).
  virtual bool start(std::string& err) = 0;
  virtual void stop() = 0;

  virtual void set_frame_callback(FrameCallback cb) = 0;

  virtual bool send(uint32_t ext_id, const uint8_t data[8],
                    std::string* err = nullptr) = 0;

  virtual BusStats stats() const = 0;
  virtual bool is_up() const = 0;
  virtual CanIfState can_state() const = 0;

  // Diagnostics / logging.
  virtual const char* kind() const = 0;    // "socketcan" | "yousee"
  virtual std::string device() const = 0;  // "can0" | "/dev/ttyUSB0"
};

}  // namespace ota::can
