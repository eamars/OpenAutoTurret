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
#include <optional>

#include "calibration/calibration_session.hpp"
#include "control/motor_backend.hpp"
#include "can/gm6020_protocol.hpp"
#include "plant.hpp"
#include "control/motor_recovery_check.hpp"

namespace ota::sim {

class SimMotorBackend : public MotorBackend {
 public:
  explicit SimMotorBackend(double dt_s = 0.005) : dt_(dt_s) {}
  // The simulated plant runs no servos; a swap of the record in force (ADR-005) is counted so the
  // control loop's side of it can be tested, and can be refused to test that path.
  bool swap_servo_assets(const ServoAssets&, std::string& err) override {
    if (refuse_servo_swap_) { err = "simulated refusal"; return false; }
    ++servo_swaps_;
    return true;
  }
  int servo_swaps() const { return servo_swaps_; }
  // ADR-005 D7: the simulator runs a pitch session on its speed-mode plant, stepped once a control
  // cycle (it has no servo threads); the control loop's own pitch commands stand aside meanwhile.
  // A yaw session runs on the identified yaw plant (axis_control_core's model, as servo-sim): the test
  // hands it the plant, the session's current drives it at 1 kHz, and the simulated yaw follows it.
  void set_yaw_session_plant(const axis::YawPlantParameters& p) { yaw_plant_parameters_ = p; }
  bool begin_calibration_session(const std::shared_ptr<calibration::Session>& session,
                                 const calibration::PitchWindow& window, std::string& err) override {
    if (pitch_session_ || yaw_session_) { err = "a session is already running"; return false; }
    if (auto yaw = std::dynamic_pointer_cast<calibration::YawSession>(session)) {
      if (!yaw_plant_parameters_) { err = "the simulator has no yaw plant (set_yaw_session_plant)"; return false; }
      yaw_plant_.emplace(*yaw_plant_parameters_, kYawPlantOrigin);
      yaw_session_q0_ = axes_[ix(AxisId::Yaw)].q;
      yaw_session_t_ns_ = 0;
      yaw_session_ = std::move(yaw);
      ++sessions_;
      return true;
    }
    auto pitch = std::dynamic_pointer_cast<calibration::PitchSession>(session);
    if (!pitch) { err = "not a session the simulator runs"; return false; }
    auto& ax = axes_[ix(AxisId::Pitch)];
    session_was_position_ = ax.in_position_mode;
    ax.in_position_mode = false;
    ax.in_speed_mode = true;
    ax.spd_ref = 0.0;
    pitch_session_ = std::move(pitch);
    session_window_ = window;
    ++sessions_;
    return true;
  }
  void release_calibration_session() override {
    if (yaw_session_) {
      yaw_session_->abandon("released");
      yaw_session_.reset();
      auto& ax = axes_[ix(AxisId::Yaw)];
      ax.target = ax.q;
      ax.v = 0.0;
      return;
    }
    if (!pitch_session_) return;
    pitch_session_->abandon("released");
    pitch_session_.reset();
    auto& ax = axes_[ix(AxisId::Pitch)];
    ax.spd_ref = 0.0;
    if (session_was_position_) { ax.in_speed_mode = false; ax.in_position_mode = true; ax.target = ax.q; }
  }
  void advance_calibration_session(TimeNs now) override {
    if (yaw_session_) {
      // GM6020 frames at 1 kHz up to now, as the receive thread would deliver them.
      if (!yaw_session_t_ns_) yaw_session_t_ns_ = now - 5'000'000;
      const auto& p = *yaw_plant_parameters_;
      const double quantum = 2 * 3.14159265358979323846 / 8192;
      for (; yaw_session_t_ns_ + 1'000'000 <= now; yaw_session_t_ns_ += 1'000'000) {
        const double sampled_t = (yaw_session_t_ns_ + 1'000'000) * 1e-9;
        yaw_plant_->advance(sampled_t);
        const double sampled = yaw_plant_->position(), receipt = sampled_t + p.encoder_delay_s;
        yaw_plant_->advance(receipt);
        const double reading = yaw_plant_->reading(sampled, receipt);
        calibration::YawFrame f;
        f.rx_ns = static_cast<TimeNs>(receipt * 1e9);
        f.dequeue_ns = f.rx_ns;
        f.angle_count = static_cast<uint16_t>(((std::llround(reading / quantum) % 8192) + 8192) % 8192);
        f.temperature_raw = 35;
        const TimeNs at = f.rx_ns + 200'000;
        const double u = yaw_session_->control(at, f);
        if (!std::isfinite(u)) continue;
        const int raw = gm6020::current_raw_uncapped(u);
        yaw_plant_->command(at * 1e-9, raw * gm6020::kAmpsPerRaw);
        yaw_session_->sent(true, raw * gm6020::kAmpsPerRaw, raw, at, at + 30'000);
      }
      auto& ax = axes_[ix(AxisId::Yaw)];
      ax.q = yaw_session_q0_ + (yaw_plant_->position() - kYawPlantOrigin);
      ax.v = yaw_plant_->velocity();
      return;
    }
    if (!pitch_session_) return;
    auto& ax = axes_[ix(AxisId::Pitch)];
    ax.spd_ref = pitch_session_->step(now, measured(ax, now), now, ax.torque, session_window_);
  }
  int sessions() const { return sessions_; }
  bool session_owns_pitch() const { return pitch_session_ != nullptr; }
  bool session_owns_yaw() const { return yaw_session_ != nullptr; }
  void set_refuse_servo_swap(bool refuse) { refuse_servo_swap_ = refuse; }
  void set_recovery_before_homing(bool enabled) { pre_home_recovery_ = enabled; }
  bool recovery_before_homing() const override { return pre_home_recovery_; }
  bool begin_motor_recovery(std::string&) override {
    for (auto a : {AxisId::Pitch, AxisId::Yaw}) deenergize(a);
    recovery_begin_pending_ = true;
    return true;
  }
  Transition poll_motor_recovery(TimeNs now, double max_temp, std::string& err) override {
    if (recovery_begin_pending_) { recovery_.begin(now); recovery_begin_pending_ = false; }
    std::array<AxisSnapshot, kAxisCount> samples;
    for (int i = 0; i < kAxisCount; ++i) samples[i] = snapshot(static_cast<AxisId>(i), now);
    return recovery_.observe(now, samples, max_temp, err);
  }
  void cancel_motor_recovery() override { recovery_begin_pending_ = false; recovery_.cancel(); }

  // --- configuration / test hooks -----------------------------------------
  void set_stops(AxisId a, double low, double high) {
    axes_[ix(a)].stop_low = low;
    axes_[ix(a)].stop_high = high;
  }
  void set_position(AxisId a, double q) {
    axes_[ix(a)].q = q;
    axes_[ix(a)].v = 0.0;
  }
  // The encoder as the station reads it (2026-10-03): positions in whole counts, and an axis holding
  // still flickers between neighbouring counts. The pitch CyberGear reports 16 bits over +-4 pi
  // (0.38 mrad); held on its stop it flipped one count every few cycles. A plant that reports exact
  // positions hides every threshold that sits below that noise, which is how a 1e-3 rad/s at-rest
  // gate passed in simulation and chattered BRAKE on the station. `dither_counts` is the peak of a
  // deterministic noise added before rounding; 0.6 gives the station's occasional one-count flips.
  void set_encoder(AxisId a, double quantum_rad, double dither_counts) {
    axes_[ix(a)].quantum = quantum_rad;
    axes_[ix(a)].dither_counts = dither_counts;
  }
  // The speed reading as the station gives it: on 2026-10-24 the pitch CyberGear's speed spanned
  // -2.6..+8 deg/s while its position did not move by one count. A plant that reports its true speed
  // hides every at-rest test built on speed. `peak_rad_s` is the peak of a deterministic noise.
  void set_speed_noise(AxisId a, double peak_rad_s) { axes_[ix(a)].speed_noise = peak_rad_s; }
  void set_feedback_ok(AxisId a, bool ok) { axes_[ix(a)].feedback_ok = ok; }
  void set_faults(AxisId a, uint16_t f) { axes_[ix(a)].faults = f; }
  void set_temp(AxisId a, double t) { axes_[ix(a)].temp_c = t; }
  void set_efforts(AxisId a, double drive, double contact) {
    axes_[ix(a)].drive_effort = drive;
    axes_[ix(a)].contact_effort = contact;
  }
  // Payload-inertia emulation (Phase 9): a heavier/different payload slows
  // the closed-loop position response. tau is the first-order position
  // time constant (default 50 ms). Test hook — mirrors what a payload
  // change does to a real drive's response.
  void set_response_tau(AxisId a, double tau_s) {
    axes_[ix(a)].tau_pos = tau_s;
  }
  double position(AxisId a) const { return axes_[ix(a)].q; }
  ParkPositionEvidence park_position_evidence(AxisId a, TimeNs now) const override {
    return {true, true, now, axes_[ix(a)].q, 0};
  }
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
    ax.in_speed_mode = false;
    ax.limit_spd = limit_spd_rad_s;
    ax.target = ax.q;  // pin LocRef to the freshly-read position
    return true;
  }
  bool enter_speed_mode(AxisId axis, double limit_cur_a, std::string&) override {
    auto& ax = axes_[ix(axis)];
    ax.deenergized = false;
    ax.in_speed_mode = true;
    ax.in_position_mode = false;
    ax.limit_cur_a = limit_cur_a;
    ax.spd_ref = 0.0;  // hold in place (velocity loop holds q against load)
    return true;
  }
  void deenergize(AxisId axis) override {
    if (axis == AxisId::Pitch && pitch_session_) {
      pitch_session_->abandon("the pitch axis was de-energised");
      pitch_session_.reset();
    }
    if (axis == AxisId::Yaw && yaw_session_) {
      yaw_session_->abandon("the yaw axis was de-energised");
      yaw_session_.reset();
    }
    auto& ax = axes_[ix(axis)];
    ax.deenergized = true;
    ax.in_position_mode = false;
    ax.in_speed_mode = false;
    ax.v = 0.0;
  }

  // --- MotorBackend: control loop (fast) -----------------------------------
  AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) override {
    const auto& ax = axes_[ix(axis)];
    AxisSnapshot s;
    s.has_feedback = ax.feedback_ok;
    s.rx_ns = now_ns;
    s.q_rad = measured(ax, now_ns);
    s.v_rad_s = ax.v + (ax.speed_noise > 0 ? (2.0 * unit_hash(now_ns ^ 0x5DEECE66Dull) - 1.0) * ax.speed_noise : 0.0);
    s.torque_nm = ax.torque;
    s.temp_c = ax.temp_c;
    s.faults = ax.faults;
    s.disabled = ax.deenergized;
    s.in_position_mode = ax.in_position_mode;
    s.in_speed_mode = ax.in_speed_mode;
    return s;
  }
  void command(AxisId axis, double q_ref_rad, double limit_spd_rad_s) override {
    if (axis == AxisId::Pitch && pitch_session_) { advance(ix(axis)); return; }
    if (axis == AxisId::Yaw && yaw_session_) return;   // the plant moves it
    auto& ax = axes_[ix(axis)];
    ax.target = q_ref_rad;
    ax.limit_spd = limit_spd_rad_s;
    advance(ix(axis));
  }
  void command_velocity(AxisId axis, double velocity_rad_s) override {
    if (axis == AxisId::Pitch && pitch_session_) { advance(ix(axis)); return; }
    if (axis == AxisId::Yaw && yaw_session_) return;
    auto& ax = axes_[ix(axis)];
    ax.spd_ref = velocity_rad_s;
    advance(ix(axis));
  }
  void set_current_limit(AxisId axis, double limit_cur_a) override {
    axes_[ix(axis)].limit_cur_a = limit_cur_a;
  }
  double current_limit(AxisId axis) const {
    return axes_[ix(axis)].limit_cur_a;
  }
  bool in_speed_mode(AxisId a) const { return axes_[ix(a)].in_speed_mode; }

 private:
  MotorRecoveryCheck recovery_;
  bool recovery_begin_pending_ = false;
  bool pre_home_recovery_ = false;
  int servo_swaps_ = 0;
  bool refuse_servo_swap_ = false;
  std::shared_ptr<calibration::PitchSession> pitch_session_;
  calibration::PitchWindow session_window_{};
  bool session_was_position_ = false;
  int sessions_ = 0;
  static constexpr double kYawPlantOrigin = 1.0;   // the plant's absolute angle where a session begins
  std::shared_ptr<calibration::YawSession> yaw_session_;
  std::optional<axis::YawPlantParameters> yaw_plant_parameters_;
  std::optional<axis::YawPlant> yaw_plant_;
  double yaw_session_q0_ = 0.0;
  TimeNs yaw_session_t_ns_ = 0;
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  struct Axis;
  // The encoder reading of the true position (set_encoder). The noise is a hash of the sample time,
  // so one instant always reads the same and a test run is repeatable.
  static double unit_hash(uint64_t seed) {   // [0, 1), the same for the same seed
    uint64_t z = seed + 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
  }
  static double measured(const Axis& ax, TimeNs now_ns) {
    if (!(ax.quantum > 0)) return ax.q;
    const double u = unit_hash(static_cast<uint64_t>(now_ns));
    const double noise = (2.0 * u - 1.0) * ax.dither_counts * ax.quantum;
    return std::round((ax.q + noise) / ax.quantum) * ax.quantum;
  }
  // Advance the plant by dt_. Models a position-mode CyberGear: the position
  // converges smoothly to the reference (first-order, tau ~ 50 ms) subject to
  // the speed limit, so moves settle (no sustained oscillation). Pushing into a
  // stop stalls it (v=0, clamped) and raises the contact effort; moving away
  // releases it. The stall/effort behavior matches the homing tests, so the
  // contact detector behaves identically.
  void advance(size_t i) {
    auto& ax = axes_[i];
    if (ax.deenergized || (!ax.in_position_mode && !ax.in_speed_mode)) {
      ax.v = 0.0;
      ax.torque = 0.0;
      ax.at_stop = false;
      ax.stop_side = 0;
      return;
    }
    // The commanded motion: position mode chases the reference (first-order,
    // speed-limited); speed mode holds the commanded speed (SpdRef) — the
    // drive's own velocity loop is the smooth source of motion, so the plant
    // moves at that constant speed.
    double dq;
    if (ax.in_speed_mode) {
      dq = ax.spd_ref * dt_;
    } else {
      const double max_dq = ax.limit_spd * dt_;
      dq = (ax.target - ax.q) * (dt_ / (ax.tau_pos + dt_));
      if (dq > max_dq) dq = max_dq;
      else if (dq < -max_dq) dq = -max_dq;
    }

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

    // Pushing into a stop = the commanded motion points into the stop. Speed
    // mode: SpdRef points into it (SpdRef=0 at a stop is a HOLD, not a push).
    // Position mode: the reference lies beyond the stop.
    const bool pushing = ax.at_stop &&
                         (ax.in_speed_mode
                              ? ((ax.stop_side > 0 && ax.spd_ref > 0.0) ||
                                (ax.stop_side < 0 && ax.spd_ref < 0.0))
                              : ((ax.stop_side > 0 && ax.target > ax.q) ||
                                (ax.stop_side < 0 && ax.target < ax.q)));
    const int dir =
        (ax.in_speed_mode ? (ax.spd_ref >= 0.0) : (dq >= 0.0)) ? 1 : -1;
    if (pushing) ax.torque = ax.contact_effort * dir;
    else if (std::fabs(ax.v) > 1e-3) ax.torque = ax.drive_effort * dir;
    else ax.torque = 0.0;
  }

  struct Axis {
    double q = 0.0;
    double v = 0.0;
    double tau_pos = 0.05;  // position response time constant (50 ms default)
    double stop_low = -1.0;
    double stop_high = 1.0;
    double target = 0.0;
    double limit_spd = 1.0;
    double limit_cur_a = 0.0;
    bool at_stop = false;
    int stop_side = 0;
    double torque = 0.0;
    double drive_effort = 1.0;
    double contact_effort = 5.0;
    bool in_position_mode = false;
    bool in_speed_mode = false;
    double spd_ref = 0.0;  // commanded speed (SpdRef, rad/s) in speed mode
    bool deenergized = true;
    double temp_c = 25.0;
    uint16_t faults = 0;
    bool feedback_ok = true;
    bool discovery_ok = true;
    double quantum = 0.0;        // set_encoder: 0 reports the exact position
    double dither_counts = 0.0;
    double speed_noise = 0.0;    // set_speed_noise: 0 reports the true speed
  };

  double dt_;
  Axis axes_[2];
  bool register_ok_ = true;
};

}  // namespace ota::sim
