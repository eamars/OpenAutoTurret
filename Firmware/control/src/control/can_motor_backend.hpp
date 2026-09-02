// OpenAutoTurret — CAN motor backend (the real CyberGear transport).
//
// Adapts the CyberGearSystem (SocketCAN) to the MotorBackend interface so the
// ControlLoop and BootFsm run unchanged against real hardware. This is the
// only place the daemon touches the CAN bus.
//
// The SETUP methods are slow, blocking, bounded paths (§46: setup/diagnostics
// run only at boot and at one-time phase transitions, never in the control
// loop): discovery, register reads, and the enter-position-mode recipe (which
// includes the two 50 ms waits required by the CyberGear after a stop).
//
// The CONTROL-LOOP methods are fast and non-blocking: snapshot() only reads a
// seqlock-protected latest sample (no CAN query), and command() is pure
// fire-and-forget CAN TX (no synchronous register-query chain, no sleep).
#pragma once

#include <array>
#include <string>

#include "can/cybergear_system.hpp"
#include "control/motor_backend.hpp"

namespace ota {

class CanMotorBackend : public MotorBackend {
 public:
  explicit CanMotorBackend(can::CyberGearSystem& system,
                           int setup_timeout_ms = 500)
      : system_(system), timeout_ms_(setup_timeout_ms) {}

  // --- MotorBackend: setup (slow, boot/transition only) ---------------------
  bool discover(AxisId axis, uint64_t& unique_id, std::string& err) override;
  bool read_register(AxisId axis, cybergear::Reg reg, double& value,
                     int timeout_ms, std::string& err) override;
  bool enter_position_mode(AxisId axis, double limit_spd_rad_s,
                           std::string& err) override;
  bool enter_speed_mode(AxisId axis, double limit_cur_a,
                        std::string& err) override;
  void deenergize(AxisId axis) override;

  // --- MotorBackend: control loop (fast, non-blocking) ----------------------
  AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) override;
  void command(AxisId axis, double q_ref_rad, double limit_spd_rad_s) override;
  void command_velocity(AxisId axis, double velocity_rad_s) override;
  void keepalive(AxisId axis) override;
  void set_current_limit(AxisId axis, double limit_cur_a) override;
  void set_speed_loop_gains(AxisId axis, double spd_kp,
                            double spd_ki) override;

 private:
  // Fire-and-forget register writes (no response wait).
  bool write_reg_float(cybergear::Reg reg, float value, AxisId axis);
  bool write_reg_u8(cybergear::Reg reg, uint8_t value, AxisId axis);

  can::CyberGearSystem& system_;
  int timeout_ms_;
  // Position mode is tracked locally: the feedback "mode" field is the motor
  // state (reset/cali/running), not the RunMode register we set.
  std::array<bool, kAxisCount> in_position_mode_{};
  std::array<bool, kAxisCount> in_speed_mode_{};
  // The last current limit applied per axis (A); set_current_limit writes
  // LimitCur only on change (avoids a redundant CAN TX every cycle).
  std::array<double, kAxisCount> last_limit_cur_a_{};
  // The last SpdRef written per axis (rad/s); command_velocity writes only on
  // change (re-arming a speed reference every cycle would needlessly re-trigger
  // the drive, mirroring the position-mode write-on-change policy).
  std::array<double, kAxisCount> last_spd_ref_{{-1e30, -1e30}};
};

}  // namespace ota
