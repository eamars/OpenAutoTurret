// CanMotorBackend — see can_motor_backend.hpp.
#include "control/can_motor_backend.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <spdlog/spdlog.h>

#include "common/time.hpp"

namespace ota {

namespace {
constexpr int kRecipeDelayMs = 50;  // CyberGear needs ~50 ms after a stop.
// Every recipe frame is fire-and-forget: the CyberGear does not ACK register
// writes on the wire, so a frame can be silently dropped and the drive can
// end up in the wrong mode (or de-energized) while every write "succeeded".
// The recipes below therefore read back the mode-defining registers after
// each attempt and retry the whole recipe on any mismatch.
//
// Defense-in-depth, NOT the p3d/p3e root cause: the wire capture of p3e
// shows the recipe verified correctly (RunMode/LimitSpd/LocRef readback all
// correct) and yet the backoff still timed out. The real cause was upstream
// in the control loop — see control_loop.cpp: during the position-mode
// backoff the step-6 safety/command stage re-issued the DEFAULT hold
// reference (current position @ 0) on every Allow cycle, stomping the
// backoff target the Homing handler had just written, so the drive's
// position loop saw the target flip at 100 Hz and could never break static
// friction. (The recipe readback+retry is kept because a genuine drop would
// otherwise leave the drive in the wrong mode with no error.)
constexpr int kRecipeMaxAttempts = 3;
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
  for (int attempt = 1; attempt <= kRecipeMaxAttempts; ++attempt) {
    std::string rerr;
    if (!system_.send_stop(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (!write_reg_u8(cybergear::Reg::RunMode, 1, axis)) {
      err = "write RunMode=1 failed";
      continue;
    }
    if (!system_.send_enable(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (!write_reg_float(cybergear::Reg::LimitSpd,
                         static_cast<float>(limit_spd_rad_s), axis)) {
      err = "write LimitSpd failed";
      continue;
    }
    double current = 0.0;
    if (!read_register(axis, cybergear::Reg::MechPos, current, timeout_ms_,
                       rerr)) {
      err = "read MechPos failed: " + rerr;
      continue;
    }
    const bool pin_ok =
        system_.send_position_ref(axis, static_cast<float>(current), &rerr);
    if (!pin_ok) {
      err = "pin position ref failed: " + rerr;
      continue;
    }
    // Verify the recipe actually took: RunMode, LimitSpd and the LocRef pin
    // must all read back exactly what we wrote. A dropped frame (e.g. a lost
    // enable) leaves one of these wrong; retry the whole recipe from the
    // stop.
    double run_mode = 0.0, limit_spd = 0.0, loc_ref = 0.0;
    const bool verified =
        read_register(axis, cybergear::Reg::RunMode, run_mode, timeout_ms_,
                      rerr) &&
        read_register(axis, cybergear::Reg::LimitSpd, limit_spd,
                      timeout_ms_, rerr) &&
        read_register(axis, cybergear::Reg::LocRef, loc_ref, timeout_ms_,
                      rerr) &&
        run_mode == 1.0 &&
        std::fabs(limit_spd - limit_spd_rad_s) < 1e-6 &&
        std::fabs(loc_ref - current) < 1e-6;
    if (!verified) {
      err = "enter_position_mode: recipe verify failed (attempt " +
            std::to_string(attempt) + ")";
      spdlog::warn("enter_position_mode verify failed axis={} attempt={} "
                   "run_mode={} limit_spd={} loc_ref={} expect ls={} qr={}",
                   static_cast<int>(axis), attempt, run_mode, limit_spd,
                   loc_ref, limit_spd_rad_s, current);
      continue;
    }
    in_position_mode_[static_cast<size_t>(axis)] = true;
    // The mode flags are mutually exclusive. enter_speed_mode clears
    // in_position_mode_ (and this clears in_speed_mode_); leaving the other
    // flag set would keep the control loop's speed-mode branch
    // (command_velocity) active after a transition to position mode. That
    // branch only pings the drive when the safety supervisor brakes, so on
    // Allow cycles nothing is commanded, the feedback goes stale, and the
    // supervisor flaps ALLOW/BRAKE every feedback_max_age_ms (p0p hold
    // phase).
    in_speed_mode_[static_cast<size_t>(axis)] = false;
    return true;
  }
  return false;
}

// The verified-live speed-mode recipe (mirrors the position-mode recipe, which
// includes the two ~50 ms waits the CyberGear needs after a stop / after
// enable). Ends holding in place: SpdRef=0 so the drive's velocity loop holds
// the current position against any load up to LimitCur.
bool CanMotorBackend::enter_speed_mode(AxisId axis, double limit_cur_a,
                                       std::string& err) {
  for (int attempt = 1; attempt <= kRecipeMaxAttempts; ++attempt) {
    std::string rerr;
    if (!system_.send_stop(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (!write_reg_u8(cybergear::Reg::RunMode, 2, axis)) {
      err = "write RunMode=2 failed";
      continue;
    }
    if (!system_.send_enable(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (!write_reg_float(cybergear::Reg::LimitCur,
                         static_cast<float>(limit_cur_a), axis)) {
      err = "write LimitCur failed";
      continue;
    }
    if (!write_reg_float(cybergear::Reg::SpdRef, 0.0f, axis)) {
      err = "write SpdRef=0 failed";
      continue;
    }
    // Verify the recipe actually took (same fire-and-forget hazard as the
    // position-mode recipe; a dropped enable leaves the axis de-energized
    // and the following approach times out).
    double run_mode = 0.0, limit_cur = 0.0, spd_ref = 0.0;
    const bool verified =
        read_register(axis, cybergear::Reg::RunMode, run_mode, timeout_ms_,
                      rerr) &&
        read_register(axis, cybergear::Reg::LimitCur, limit_cur,
                      timeout_ms_, rerr) &&
        read_register(axis, cybergear::Reg::SpdRef, spd_ref, timeout_ms_,
                      rerr) &&
        run_mode == 2.0 && std::fabs(limit_cur - limit_cur_a) < 1e-6 &&
        std::fabs(spd_ref) < 1e-6;
    if (!verified) {
      err = "enter_speed_mode: recipe verify failed (attempt " +
            std::to_string(attempt) + ")";
      spdlog::warn("enter_speed_mode verify failed axis={} attempt={} "
                   "run_mode={} limit_cur={} spd_ref={} expect lc={}",
                   static_cast<int>(axis), attempt, run_mode, limit_cur,
                   spd_ref, limit_cur_a);
      continue;
    }
    last_limit_cur_a_[static_cast<size_t>(axis)] = limit_cur_a;
    last_spd_ref_[static_cast<size_t>(axis)] = 0.0;
    in_position_mode_[static_cast<size_t>(axis)] = false;
    in_speed_mode_[static_cast<size_t>(axis)] = true;
    return true;
  }
  return false;
}

void CanMotorBackend::deenergize(AxisId axis) {
  std::string err;
  system_.send_stop(axis, &err);
  in_position_mode_[static_cast<size_t>(axis)] = false;
  in_speed_mode_[static_cast<size_t>(axis)] = false;
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
  s.in_speed_mode = in_speed_mode_[static_cast<size_t>(axis)];
  return s;
}

void CanMotorBackend::command_velocity(AxisId axis, double velocity_rad_s) {
  // Write SpdRef only when it actually changes (mirrors the position-mode
  // write-on-change policy: a same-value rewrite is inert but we avoid the
  // needless CAN TX). The drive holds the written speed with its internal
  // velocity loop until it is changed.
  const int a = static_cast<int>(axis);
  if (std::fabs(velocity_rad_s - last_spd_ref_[a]) > 1e-6) {
    cybergear::CanFrame f = cybergear::make_write_reg_float(
        cybergear::Reg::SpdRef, static_cast<float>(velocity_rad_s),
        system_.host_id(), system_.motor_id(axis));
    std::string err;
    if (!system_.send(f.id, f.data, &err)) {
      spdlog::warn("send SpdRef FAIL axis={} v={:+.4f} err={}", a,
                   velocity_rad_s, err);
    }
    last_spd_ref_[a] = velocity_rad_s;
  }
  // Keepalive ping: see keepalive() below.
  keepalive(axis);
}

void CanMotorBackend::keepalive(AxisId axis) {
  // The CyberGear emits COMM_TYPE_2 feedback ONLY in response to a command
  // (no periodic telemetry, CyberGear_AI_Reference.md §21). On cycles where
  // no reference is commanded (speed-mode axis + supervisor Allow — the
  // control loop deliberately issues nothing so it does not disturb the
  // drive), a same-value LimitCur rewrite is inert (a limit, not a
  // reference: it does not re-arm the velocity loop) yet elicits a feedback
  // response, holding the feedback age below the safety supervisor's
  // feedback_max_age_ms. Without this the age crosses ~100 ms after ~5
  // cycles and the supervisor flaps BRAKE/ALLOW forever, each BRAKE stomping
  // the other axis's reference (p0p hold phase; p3e fault-phase flap,
  // wire-verified B/C 1:1 alternation).
  // Rate-limited: only when the age is already >30 ms and at most every
  // 50 ms (20 pings/s), so steady-state costs one CAN frame per ~50 ms.
  const int a = static_cast<int>(axis);
  static TimeNs last_ping_ns[2] = {0, 0};
  constexpr int64_t kPingWhenAgeNs = 30'000'000;   // ping at 30 ms age
  constexpr int64_t kPingIntervalNs = 50'000'000;  // at most 20 pings/s
  const TimeNs now_ns = now_monotonic_ns();
  can::AxisLatest fb{};
  bool stale = !system_.axis(axis).latest(fb) || !fb.has_feedback;
  if (!stale && (now_ns - fb.rx_ns) > kPingWhenAgeNs) stale = true;
  if (stale && (now_ns - last_ping_ns[a]) >= kPingIntervalNs) {
    write_reg_float(cybergear::Reg::LimitCur,
                    static_cast<float>(last_limit_cur_a_[a]), axis);
    last_ping_ns[a] = now_ns;
  }
}

void CanMotorBackend::command(AxisId axis, double q_ref_rad,
                              double limit_spd_rad_s) {
  // Write each register only when its value actually changes. Re-sending an
  // *unchanged* reference every control cycle re-arms the drive's motion
  // profile each time, which (observed on the CyberGear) prevents the
  // velocity loop from building speed: the axis hunts in place at a few
  // 0.1 N.m instead of moving at the commanded rate, and a hold never settles.
  // The drive holds a written reference until it is changed, so writing on
  // change is both safe and what the drive expects. The position reference is
  // compared with a small epsilon so sensor quantisation (25/65535 rad) does
  // not count as a change and re-arm a hold.
  constexpr double kQRefEpsilonRad = 1e-3;
  static double last_ls[2] = {-1e30, -1e30};
  static double last_qr[2] = {-1e30, -1e30};
  const int a = static_cast<int>(axis);
  const bool ls_changed = (limit_spd_rad_s != last_ls[a]);
  const bool qr_changed = std::fabs(q_ref_rad - last_qr[a]) > kQRefEpsilonRad;
  // Preserve ordering: set the speed limit before a new position reference so
  // the drive never chases a new target without its (possibly reduced) limit.
  if (ls_changed) {
    write_reg_float(cybergear::Reg::LimitSpd,
                    static_cast<float>(limit_spd_rad_s), axis);
    last_ls[a] = limit_spd_rad_s;
  }
  if (qr_changed) {
    std::string err;
    const bool pin_ok =
        system_.send_position_ref(axis, static_cast<float>(q_ref_rad), &err);
    last_qr[a] = q_ref_rad;
    if (!pin_ok) {
      spdlog::warn("send_position_ref FAIL axis={} q_ref={:+.6f} err={}",
                   a, q_ref_rad, err);
    }
  }
  // Keepalive ping: the CyberGear emits COMM_TYPE_2 feedback ONLY in
  // response to a command (CyberGear_AI_Reference.md §21: write-parameter
  // -> COMM_TYPE_2 response; there is no periodic telemetry mode). While
  // holding or creeping slowly, the write-on-change logic above transmits
  // nothing, the drive stays silent, the feedback age grows, and the
  // safety supervisor brakes at feedback_max_age_ms — which produced the
  // ALLOW/BRAKE flapping and the slow-creep stall (p0i). A same-value
  // LimitSpd rewrite is inert (a speed limit, not a position reference:
  // it does not re-arm the motion profile, unlike a LocRef rewrite — the
  // p0h hunting) yet still elicits a feedback response, holding the age
  // below ~40 ms so feedback_max_age_ms remains a genuine
  // loss-of-feedback detector.
  {
    static TimeNs last_ping_ns[2] = {0, 0};
    constexpr int64_t kPingWhenAgeNs = 30'000'000;   // ping at 30 ms age
    constexpr int64_t kPingIntervalNs = 50'000'000;  // at most 20 pings/s
    const TimeNs now_ns = now_monotonic_ns();
    can::AxisLatest fb{};
    bool stale = !system_.axis(axis).latest(fb) || !fb.has_feedback;
    if (!stale && (now_ns - fb.rx_ns) > kPingWhenAgeNs) stale = true;
    if (stale && (now_ns - last_ping_ns[a]) >= kPingIntervalNs) {
      write_reg_float(cybergear::Reg::LimitSpd,
                      static_cast<float>(limit_spd_rad_s), axis);
      last_ping_ns[a] = now_ns;
      // Do not touch last_ls[a]: the value written equals
      // limit_spd_rad_s, so the write-on-change bookkeeping stays exact.
    }
  }
}

void CanMotorBackend::set_current_limit(AxisId axis, double limit_cur_a) {
  // Write LimitCur (0x7018) only on change. The adaptive-current homing calls
  // this on the cycle it raises the drive current (§22); a same-value rewrite
  // is inert but would needlessly add a CAN TX, so track the last value.
  const int a = static_cast<int>(axis);
  if (std::fabs(limit_cur_a - last_limit_cur_a_[a]) < 1e-6) return;
  if (write_reg_float(cybergear::Reg::LimitCur, static_cast<float>(limit_cur_a),
                      axis)) {
    last_limit_cur_a_[a] = limit_cur_a;
  } else {
    spdlog::warn("write LimitCur FAIL axis={} cur={:.2f} A", a, limit_cur_a);
  }
}

}  // namespace ota
