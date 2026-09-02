#pragma once
// CyberGear system: one CAN transport multiplexing the two axes
// (architecture §8). The transport (PHY) is configuration-selected through
// the CanTransport interface: SocketCAN (MCP2515 HAT) or the yousee USB-CAN
// AT adapter — the protocol stack above never knows which is active.
//
// Responsibilities:
//  - RX: parse COMM_TYPE_2 feedback, update per-axis state + history;
//  - setup/diagnostics: synchronous discovery (COMM_TYPE_0) and register
//    reads (COMM_TYPE_17) with bounded timeouts — these are SLOW paths used
//    at boot/commissioning, never from the 200 Hz control loop (§46: no
//    synchronous register-query chain in the loop);
//  - fire-and-forget command TX for the control loop.
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "can/can_transport.hpp"
#include "can/cybergear_axis.hpp"
#include "can/cybergear_protocol.hpp"
#include "common/types.hpp"

namespace ota::can {

struct CyberGearSystemConfig {
  // Transport ("PHY") selection — swap freely (see can_transport.hpp).
  // "socketcan": iface = can0.   "yousee": iface = /dev/ttyUSB0 (serial).
  std::string transport = "socketcan";
  std::string iface = "can0";
  int uart_baud = 921600;      // yousee only (adapter UART side)
  uint32_t bitrate = 1000000;  // CAN-side bitrate both PHYs
  uint8_t host_can_id = 0;
  uint8_t pitch_motor_id = 100;  // 0x64
  uint8_t yaw_motor_id = 101;    // 0x65
  bool bring_up_if_down = false;
  std::size_t history_capacity = AxisRuntime::kDefaultHistoryCapacity;
};

class CyberGearSystem {
 public:
  bool open(const CyberGearSystemConfig& cfg, std::string& err);
  void close();

  // --- Setup / diagnostics (blocking, bounded) -----------------------------
  bool discover(AxisId axis, uint64_t& unique_id, int timeout_ms = 500,
                std::string* err = nullptr);
  bool read_register(AxisId axis, cybergear::Reg reg, double& value,
                     int timeout_ms = 500, std::string* err = nullptr);

  // --- Fire-and-forget command TX (control loop safe) ----------------------
  bool send(uint32_t ext_id, const uint8_t data[8], std::string* err = nullptr);
  bool send_position_ref(AxisId axis, float q_rad, std::string* err = nullptr) {
    cybergear::CanFrame f =
        cybergear::make_write_reg_float(cybergear::Reg::LocRef, q_rad,
                                        cfg_.host_can_id, motor_id(axis));
    return send(f.id, f.data, err);
  }
  bool send_enable(AxisId axis, std::string* err = nullptr);
  bool send_stop(AxisId axis, std::string* err = nullptr);
  bool send_set_zero(AxisId axis, std::string* err = nullptr);

  // --- State access ---------------------------------------------------------
  AxisRuntime& axis(AxisId a) { return axes_[static_cast<size_t>(a)]; }
  const AxisRuntime& axis(AxisId a) const { return axes_[static_cast<size_t>(a)]; }
  CanTransport& bus() { return *bus_; }
  // Read-only view for the CAN health report (§55): stats()/is_up()/can_state()
  // are const on the transport, so reporting bus health needs no mutable access
  // and must never be the reason the control thread touches the bus.
  CanTransport& bus() const { return *bus_; }
  uint8_t motor_id(AxisId a) const {
    return (a == AxisId::Pitch) ? cfg_.pitch_motor_id : cfg_.yaw_motor_id;
  }
  uint8_t host_id() const { return cfg_.host_can_id; }

 private:
  void on_frame(const can::RawFrame& f);
  // Single-flight synchronous wait. match_target == 0 means "don't care".
  bool wait_response(uint8_t comm_type, uint8_t match_target,
                     cybergear::CanFrame& out, int timeout_ms,
                     std::string* err);

  CyberGearSystemConfig cfg_{};
  std::unique_ptr<CanTransport> bus_;
  std::array<AxisRuntime, 2> axes_{};

  // Synchronous request/response (setup path only, never in control loop).
  std::mutex pend_mtx_;
  std::condition_variable pend_cv_;
  struct Pending {
    bool active{false};
    uint8_t motor{0};
    uint8_t comm{0};
    uint8_t match_target{0};
    cybergear::CanFrame frame;
  } pending_;
};

}  // namespace ota::can
