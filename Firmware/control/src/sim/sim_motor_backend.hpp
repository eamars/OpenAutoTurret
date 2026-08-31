// OpenAutoTurret — simulated motor backend (test / HIL-planning support, §54).
//
// Implements MotorBackend against a small first-order plant with hard end
// stops, so the safety-critical ControlLoop can be unit-tested with no CAN. It
// also exposes test hooks (stale feedback, faults, temperature) to exercise the
// safety supervisor's Brake / Hold / Disable paths.
//
// Plant model: each axis has a first-order velocity response (tau ~ 50 ms)
// toward sign(target - q) * limit_spd while in position mode, and is clamped at
// its stops (pushing into a stop stalls it). De-energized axes do not move.
#pragma once

#include <cmath>

#include "control/motor_backend.hpp"

namespace ota::sim {

class SimMotorBackend : public MotorBackend {
 public:
  explicit SimMotorBackend(double dt_s = 0.005) : dt_(dt_s) {}

  // --- configuration / test hooks -----------------------------------------
  void set_stops(AxisId a, double low, double high) {
    axes_[ix(a)].stop_low = low;
    axes_[ix(a)].stop_high = high;
  }
  void set_position(AxisId a, double q) {
    axes_[ix(a)].q = q;
    axes_[ix(a)].v = 0.0;
  }
  void set_feedback_ok(AxisId a, bool ok) { axes_[ix(a)].feedback_ok = ok; }
  void set_faults(AxisId a, uint16_t f) { axes_[ix(a)].faults = f; }
  void set_temp(AxisId a, double t) { axes_[ix(a)].temp_c = t; }
  void set_efforts(AxisId a, double drive, double contact) {
    axes_[ix(a)].drive_effort = drive;
    axes_[ix(a)].contact_effort = contact;
  }
  double position(AxisId a) const { return axes_[ix(a)].q; }
  double velocity(AxisId a) const { return axes_[ix(a)].v; }
  bool in_position_mode(AxisId a) const { return axes_[ix(a)].in_position_mode; }

  // Setup-path fault injection (boot/self-test tests): force discovery or
  // register reads to fail.
  void set_discovery_ok(AxisId a, bool ok) { axes_[ix(a)].discovery_ok = ok; }
  void set_register_ok(bool ok) { register_ok_ = ok; }

  // --- MotorBackend: setup (slow) ------------------------------------------
  bool discover(AxisId a, uint64_t& unique_id, std::string& err) override {
    auto& ax = axes_[ix(a)];
    if (!ax.discovery_ok) {
      err = "sim: discovery disabled";
      return false;
    }
    unique_id = 0x1234 + ix(a);
    return true;
  }
  bool read_register(AxisId axis, cybergear::Reg reg, double& value, int,
                     std::string& err) override {
    if (!register_ok_) {
      err = "sim: register read disabled";
      return false;
    }
    const auto& ax = axes_[ix(axis)];
    if (reg == cybergear::Reg::MechPos) value = ax.q;
    else if (reg == cybergear::Reg::MechVel) value = ax.v;
    else value = 0.0;
    return true;
  }
  bool enter_position_mode(AxisId axis, double limit_spd_rad_s,
                           std::string&) override {
    auto& ax = axes_[ix(axis)];
    ax.deenergized = false;
    ax.in_position_mode = true;
    ax.limit_spd = limit_spd_rad_s;
    ax.target = ax.q;  // pin LocRef to the freshly-read position
    return true;
  }
  void deenergize(AxisId axis) override {
    auto& ax = axes_[ix(axis)];
    ax.deenergized = true;
    ax.in_position_mode = false;
    ax.v = 0.0;
  }

  // --- MotorBackend: control loop (fast) -----------------------------------
  AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) override {
    const auto& ax = axes_[ix(axis)];
    AxisSnapshot s;
    s.has_feedback = ax.feedback_ok;
    s.rx_ns = now_ns;
    s.q_rad = ax.q;
    s.v_rad_s = ax.v;
    s.torque_nm = ax.torque;
    s.temp_c = ax.temp_c;
    s.faults = ax.faults;
    s.in_position_mode = ax.in_position_mode;
    return s;
  }
  void command(AxisId axis, double q_ref_rad, double limit_spd_rad_s) override {
    auto& ax = axes_[ix(axis)];
    ax.target = q_ref_rad;
    ax.limit_spd = limit_spd_rad_s;
    advance(ix(axis));
  }

 private:
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  // Advance the plant by dt_. Models a position-mode CyberGear: the position
  // converges smoothly to the reference (first-order, tau ~ 50 ms) subject to
  // the speed limit, so moves settle (no sustained oscillation). Pushing into a
  // stop stalls it (v=0, clamped) and raises the contact effort; moving away
  // releases it. The stall/effort behavior matches the homing tests, so the
  // contact detector behaves identically.
  void advance(size_t i) {
    auto& ax = axes_[i];
    if (ax.deenergized || !ax.in_position_mode) {
      ax.v = 0.0;
      ax.torque = 0.0;
      ax.at_stop = false;
      ax.stop_side = 0;
      return;
    }
    constexpr double kTauPos = 0.05;  // position response time constant
    const double max_dq = ax.limit_spd * dt_;
    double dq = (ax.target - ax.q) * (dt_ / (kTauPos + dt_));
    if (dq > max_dq) dq = max_dq;
    else if (dq < -max_dq) dq = -max_dq;

    double new_q = ax.q + dq;
    ax.at_stop = false;
    ax.stop_side = 0;
    if (new_q >= ax.stop_high) {
      new_q = ax.stop_high;
      ax.at_stop = true;
      ax.stop_side = +1;
    } else if (new_q <= ax.stop_low) {
      new_q = ax.stop_low;
      ax.at_stop = true;
      ax.stop_side = -1;
    }
    ax.v = (new_q - ax.q) / dt_;
    ax.q = new_q;

    const bool pushing = ax.at_stop &&
                         ((ax.stop_side > 0 && ax.target > ax.q) ||
                          (ax.stop_side < 0 && ax.target < ax.q));
    const int dir = (dq >= 0) ? 1 : -1;
    if (pushing) ax.torque = ax.contact_effort * dir;
    else if (std::fabs(ax.v) > 1e-3) ax.torque = ax.drive_effort * dir;
    else ax.torque = 0.0;
  }

  struct Axis {
    double q = 0.0;
    double v = 0.0;
    double stop_low = -1.0;
    double stop_high = 1.0;
    double target = 0.0;
    double limit_spd = 1.0;
    bool at_stop = false;
    int stop_side = 0;
    double torque = 0.0;
    double drive_effort = 1.0;
    double contact_effort = 5.0;
    bool in_position_mode = false;
    bool deenergized = true;
    double temp_c = 25.0;
    uint16_t faults = 0;
    bool feedback_ok = true;
    bool discovery_ok = true;
  };

  double dt_;
  Axis axes_[2];
  bool register_ok_ = true;
};

}  // namespace ota::sim
