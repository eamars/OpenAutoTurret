// CanMotorBackend — see can_motor_backend.hpp.
#include "control/can_motor_backend.hpp"

#include <chrono>
#include <thread>

namespace ota {

namespace {
constexpr int kRecipeDelayMs = 50;  // CyberGear needs ~50 ms after a stop.
}  // namespace

bool CanMotorBackend::write_reg_float(cybergear::Reg reg, float value,
                                      AxisId axis) {
  cybergear::CanFrame f = cybergear::make_write_reg_float(
      reg, value, system_.host_id(), system_.motor_id(axis));
  std::string err;
  return system_.send(f.id, f.data, &err);
}

bool CanMotorBackend::write_reg_u8(cybergear::Reg reg, uint8_t value,
                                   AxisId axis) {
  cybergear::CanFrame f = cybergear::make_write_reg_u8(
      reg, value, system_.host_id(), system_.motor_id(axis));
  std::string err;
  return system_.send(f.id, f.data, &err);
}

// --- Setup (slow, boot/transition only) -------------------------------------

bool CanMotorBackend::discover(AxisId axis, uint64_t& unique_id,
                               std::string& err) {
  return system_.discover(axis, unique_id, timeout_ms_, &err);
}

bool CanMotorBackend::read_register(AxisId axis, cybergear::Reg reg,
                                    double& value, int timeout_ms,
                                    std::string& err) {
  return system_.read_register(axis, reg, value, timeout_ms, &err);
}

// The verified-live position-mode recipe:
//   stop (de-energizes) -> 50 ms -> RunMode=1 -> enable -> 50 ms ->
//   LimitSpd -> read MechPos (pin current) -> LocRef=current (hold in place).
bool CanMotorBackend::enter_position_mode(AxisId axis, double limit_spd_rad_s,
                                          std::string& err) {
  if (!system_.send_stop(axis, &err)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
  if (!write_reg_u8(cybergear::Reg::RunMode, 1, axis)) {
    err = "write RunMode=1 failed";
    return false;
  }
  if (!system_.send_enable(axis, &err)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
  if (!write_reg_float(cybergear::Reg::LimitSpd,
                       static_cast<float>(limit_spd_rad_s), axis)) {
    err = "write LimitSpd failed";
    return false;
  }
  double current = 0.0;
  if (!read_register(axis, cybergear::Reg::MechPos, current, timeout_ms_,
                     err)) {
    err = "read MechPos failed: " + err;
    return false;
  }
  if (!system_.send_position_ref(axis, static_cast<float>(current), &err))
    return false;
  in_position_mode_[static_cast<size_t>(axis)] = true;
  return true;
}

void CanMotorBackend::deenergize(AxisId axis) {
  std::string err;
  system_.send_stop(axis, &err);
  in_position_mode_[static_cast<size_t>(axis)] = false;
}

// --- Control loop (fast, non-blocking) --------------------------------------

AxisSnapshot CanMotorBackend::snapshot(AxisId axis, TimeNs now_ns) {
  AxisSnapshot s;
  can::AxisLatest l;
  if (system_.axis(axis).latest(l)) {
    s.has_feedback = l.has_feedback;
    s.rx_ns = l.rx_ns;
    s.q_rad = l.q_rad;
    s.v_rad_s = l.v_rad_s;
    s.torque_nm = l.torque_nm;
    s.temp_c = l.temp_c;
    s.faults = l.faults;
  }
  s.in_position_mode = in_position_mode_[static_cast<size_t>(axis)];
  return s;
}

void CanMotorBackend::command(AxisId axis, double q_ref_rad,
                              double limit_spd_rad_s) {
  // Fire-and-forget: write the speed limit then the position reference. No
  // response wait, no register query — safe in the 200 Hz control loop.
  write_reg_float(cybergear::Reg::LimitSpd,
                  static_cast<float>(limit_spd_rad_s), axis);
  system_.send_position_ref(axis, static_cast<float>(q_ref_rad));
}

}  // namespace ota
