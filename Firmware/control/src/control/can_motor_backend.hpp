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
#include "control/motor_recovery_check.hpp"

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
  bool adopt_running_mode(AxisId axis, bool position, std::string& err, double speed_ki = -1, double speed_kp = 1) override;
  void heartbeat() override { system_.heartbeat(); }
  bool watchdog_fault() const override { return system_.motion_inhibited(); }
  bool recovery_before_homing() const override { return true; }
  bool begin_motor_recovery(std::string& err) override;
  Transition poll_motor_recovery(TimeNs now, double max_temp, std::string& err) override;
  void cancel_motor_recovery() override { recovery_.cancel(); system_.inhibit_motion(); }
  Transition transition_mode(AxisId axis, bool position, double limit,
                             TimeNs now_ns, std::string& err, double speed_ki = -1, double speed_kp = 1,
                             bool check_displacement = true) override;

  // --- MotorBackend: control loop (fast, non-blocking) ----------------------
  AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) override;
  void command(AxisId axis, double q_ref_rad, double limit_spd_rad_s) override;
  void command_velocity(AxisId axis, double velocity_rad_s) override;
  void keepalive(AxisId axis) override;
  void set_current_limit(AxisId axis, double limit_cur_a) override;
  void set_speed_loop_gains(AxisId axis, double spd_kp,
                            double spd_ki) override;

  // Bus health straight from the transport counters (§55).
  CanHealth can_health() const override;

 private:
  // Fire-and-forget register writes (no response wait).
  bool write_reg_float(cybergear::Reg reg, float value, AxisId axis);
  bool write_reg_u8(cybergear::Reg reg, uint8_t value, AxisId axis);

  can::CyberGearSystem& system_;
  int timeout_ms_;
  MotorRecoveryCheck recovery_;
  struct ModeTransition {
    int stage = 0;
    AxisId axis = AxisId::Pitch;
    bool position = false;
    bool check_displacement = true;
    double limit = 0, pin = 0, last_q = 0, speed_ki = -1, speed_kp = 1;
    double stopped_q = 0;  // position immediately before removing torque
    TimeNs started = 0, deadline = 0, still_since = 0, sampled = 0;
    int read_index = 0;
    bool waiting = false;
  } transition_;
  // Position mode is tracked locally: the feedback "mode" field is the motor
  // state (reset/cali/running), not the RunMode register we set.
  std::array<bool, kAxisCount> in_position_mode_{};
  std::array<bool, kAxisCount> in_speed_mode_{};
  // The last current limit applied per axis (A); set_current_limit writes
  // LimitCur only on change (avoids a redundant CAN TX every cycle).
  // Unknown after construction/disable; a keepalive must never invent a limit.
  std::array<double, kAxisCount> last_limit_cur_a_{{-1.0, -1.0}};
  // The last SpdRef written per axis (rad/s); command_velocity writes only on
  // change (re-arming a speed reference every cycle would needlessly re-trigger
  // the drive, mirroring the position-mode write-on-change policy).
  std::array<double, kAxisCount> last_spd_ref_{{-1e30, -1e30}};
  std::array<double, kAxisCount> last_limit_spd_{{-1e30, -1e30}};
  std::array<double, kAxisCount> last_loc_ref_{{-1e30, -1e30}};
  std::array<TimeNs, kAxisCount> last_ping_ns_{};
  void invalidate_commands(AxisId axis) {
    const auto a = static_cast<size_t>(axis);
    last_spd_ref_[a] = last_limit_spd_[a] = last_loc_ref_[a] = -1e30;
    last_limit_cur_a_[a] = -1.0;
    last_ping_ns_[a] = 0;
    in_position_mode_[a] = in_speed_mode_[a] = false;
  }
};

}  // namespace ota
