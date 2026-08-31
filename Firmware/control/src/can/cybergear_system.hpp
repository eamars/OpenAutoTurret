#pragma once
// CyberGear system: one SocketCAN bus multiplexing the two axes
// (architecture §8).
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
#include <mutex>
#include <string>

#include "can/cybergear_axis.hpp"
#include "can/cybergear_protocol.hpp"
#include "can/socketcan_bus.hpp"
#include "common/types.hpp"

namespace ota::can {

struct CyberGearSystemConfig {
  std::string iface = "can0";
  uint32_t bitrate = 1000000;
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
  SocketCanBus& bus() { return bus_; }
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
  SocketCanBus bus_;
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
