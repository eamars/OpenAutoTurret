// CanMotorBackend — see can_motor_backend.hpp.
#include "control/can_motor_backend.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <spdlog/spdlog.h>

#include "can/pitch_current_policy.hpp"
#include "common/time.hpp"

namespace ota {

bool CanMotorBackend::begin_motor_recovery(std::string& err) {
  system_.inhibit_motion();
  system_.cancel_register_read();
  deenergize(AxisId::Pitch); deenergize(AxisId::Yaw);
  recovery_.begin(now_monotonic_ns());
  bool ok = true;
  for (auto a : {AxisId::Pitch, AxisId::Yaw}) {
    std::string e;
    if (!system_.send_clear_fault(a, &e)) {
      ok = false; err = std::string(axis_name(a)) + ": fault-clear write failed: " + e;
    }
  }
  if (!ok) recovery_.cancel();
  return ok;
}

MotorBackend::Transition CanMotorBackend::poll_motor_recovery(
    TimeNs now, double max_temp, std::string& err) {
  std::array<AxisSnapshot, kAxisCount> samples;
  for (int i = 0; i < kAxisCount; ++i) samples[i] = snapshot(static_cast<AxisId>(i), now);
  const auto result = recovery_.observe(now, samples, max_temp, err);
  if (result == Transition::Complete && !system_.finish_motor_recovery(max_temp, err))
    return Transition::Failed;
  return result;
}

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
  const bool sent = system_.send(f.id, f.data, &err);
  if (reg == cybergear::Reg::SpdRef || reg == cybergear::Reg::LocRef)
    record_output(axis, value, reg == cybergear::Reg::SpdRef ? 3 : 4, sent);
  return sent;
}

void CanMotorBackend::record_output(AxisId axis, double value, int kind, bool sent) {
  auto& evidence = output_evidence_[static_cast<int>(axis)];
  evidence.requested = value;
  evidence.command_kind = kind;
  evidence.reason = sent ? (kind == 5 ? 3 : 1) : 4;
  if (sent) {
    evidence.successful = value;
    evidence.tx_ns = now_monotonic_ns();
    ++evidence.tx_seq;
  }
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
//   stop (de-energizes) -> 50 ms -> RunMode=1 -> verified current cap and
//   pre-enable encoder pin at LimitSpd=0 -> enable -> pin fresh encoder ->
//   restore LimitSpd. Pitch never enables when its encoder read is invalid.
bool CanMotorBackend::enter_position_mode(AxisId axis, double limit_spd_rad_s,
                                          std::string& err) {
  invalidate_calibration();
  invalidate_commands(axis);
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
    if (axis == AxisId::Pitch) {
      constexpr double kPitchSetupCurrentA = can::kPitchCurrentCeilingA;
      double mode = 0.0, current = 0.0;
      if (!write_reg_float(cybergear::Reg::LimitCur,
                           static_cast<float>(kPitchSetupCurrentA), axis) ||
          !read_register(axis, cybergear::Reg::RunMode, mode, timeout_ms_, rerr) ||
          !read_register(axis, cybergear::Reg::LimitCur, current, timeout_ms_, rerr) ||
          mode != 1.0 || std::fabs(current - kPitchSetupCurrentA) > 1e-6 ||
          !system_.confirm_pitch_setup(current, 1)) {
        err = "pitch current limit / position mode readback failed before enable";
        continue;
      }
      // A supported encoder read is a prerequisite to applying position mode.
      // Pin it with zero speed authority before enabling so stale retained
      // LocRef state cannot move the suspended pitch axis.
      double pin = 0.0, zero_speed = -1.0, pinned = 0.0;
      if (!read_register(axis, cybergear::Reg::MechPos, pin, timeout_ms_, rerr) ||
          !std::isfinite(pin) ||
          !write_reg_float(cybergear::Reg::LimitSpd, 0.0f, axis) ||
          !system_.send_position_ref(axis, static_cast<float>(pin), &rerr) ||
          !read_register(axis, cybergear::Reg::LimitSpd, zero_speed, timeout_ms_, rerr) ||
          !read_register(axis, cybergear::Reg::LocRef, pinned, timeout_ms_, rerr) ||
          !std::isfinite(zero_speed) || !std::isfinite(pinned) ||
          std::abs(zero_speed) > 1e-6 || std::abs(pinned - pin) > 0.0004) {
        err = "pitch pre-enable encoder pin / zero-speed verification failed";
        continue;
      }
    }
    if (!system_.send_enable(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (axis != AxisId::Pitch &&
        !write_reg_float(cybergear::Reg::LimitSpd,
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
    if (axis == AxisId::Pitch &&
        !write_reg_float(cybergear::Reg::LimitSpd,
                         static_cast<float>(limit_spd_rad_s), axis)) {
      err = "write LimitSpd failed";
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
    last_loc_ref_[static_cast<size_t>(axis)] = current;
    last_limit_spd_[static_cast<size_t>(axis)] = limit_spd_rad_s;
    if (axis == AxisId::Pitch)
      last_limit_cur_a_[static_cast<size_t>(axis)] = can::kPitchCurrentCeilingA;
    return true;
  }
  deenergize(axis);
  return false;
}

// The verified-live speed-mode recipe (mirrors the position-mode recipe, which
// includes the two ~50 ms waits the CyberGear needs after a stop / after
// enable). Ends holding in place: SpdRef=0 so the drive's velocity loop holds
// the current position against any load up to LimitCur.
bool CanMotorBackend::enter_speed_mode(AxisId axis, double limit_cur_a,
                                       std::string& err) {
  invalidate_calibration();
  invalidate_commands(axis);
  if (axis == AxisId::Pitch && !can::valid_pitch_current_limit(limit_cur_a)) {
    err = "pitch speed mode current limit must be finite and in (0, 5 A]";
    deenergize(axis);
    return false;
  }
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
    if (axis == AxisId::Pitch) {
      double mode = 0.0, current = 0.0;
      if (!write_reg_float(cybergear::Reg::LimitCur,
                           static_cast<float>(limit_cur_a), axis) ||
          !read_register(axis, cybergear::Reg::RunMode, mode, timeout_ms_, rerr) ||
          !read_register(axis, cybergear::Reg::LimitCur, current, timeout_ms_, rerr) ||
          mode != 2.0 || std::fabs(current - limit_cur_a) > 1e-6 ||
          !can::valid_pitch_current_limit(current) ||
          !system_.confirm_pitch_setup(current, 2)) {
        err = "pitch current limit / speed mode readback failed before enable";
        continue;
      }
    }
    // Pitch must enter speed mode with a neutral reference already installed;
    // a retained SpdRef must never take effect at the enable edge.
    if (axis == AxisId::Pitch) {
      double reference = 0.0, position = 0.0;
      if (!write_reg_float(cybergear::Reg::SpdRef, 0.0f, axis) ||
          !read_register(axis, cybergear::Reg::SpdRef, reference, timeout_ms_, rerr) ||
          !read_register(axis, cybergear::Reg::MechPos, position, timeout_ms_, rerr) ||
          !std::isfinite(reference) || std::abs(reference) > 1e-6 ||
          !std::isfinite(position)) {
        err = "pitch neutral speed / encoder verification failed before enable";
        continue;
      }
    }
    if (!system_.send_enable(axis, &rerr)) {
      err = rerr;
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRecipeDelayMs));
    if (axis != AxisId::Pitch &&
        !write_reg_float(cybergear::Reg::LimitCur,
                         static_cast<float>(limit_cur_a), axis)) {
      err = "write LimitCur failed";
      continue;
    }
    if (axis != AxisId::Pitch &&
        !write_reg_float(cybergear::Reg::SpdRef, 0.0f, axis)) {
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
  deenergize(axis);
  return false;
}

MotorBackend::Transition CanMotorBackend::transition_mode(
    AxisId axis, bool position, double limit, TimeNs now, std::string& err, double speed_ki, double speed_kp,
    bool check_displacement) {
  auto& t = transition_;
  const auto i = static_cast<size_t>(axis);
  auto fail = [&](const char* why) {
    err = why;
    system_.cancel_register_read();
    t = ModeTransition{};
    deenergize(axis);
    return Transition::Failed;
  };
  auto complete = [&]() {
    in_position_mode_[i] = position; in_speed_mode_[i] = !position;
    if (position) {
      last_loc_ref_[i] = t.pin;
      last_limit_spd_[i] = limit;
      if (axis == AxisId::Pitch)
        last_limit_cur_a_[i] = t.pitch_limit_cur;
    }
    else { last_spd_ref_[i] = 0; last_limit_cur_a_[i] = limit; }
    t = ModeTransition{};
    return Transition::Complete;
  };
  can::AxisLatest s;
  system_.axis(axis).latest(s);
  if (!s.has_feedback || s.rx_ns<=0 || s.rx_ns>now+5000000LL || now-s.rx_ns>100000000LL ||
      !std::isfinite(s.q_rad) || s.faults)
    return fail("fresh finite fault-free feedback required for mode setup");
  if (t.stage == 0) {
    invalidate_calibration();
    if (!std::isfinite(limit) || limit <= 0) return fail("invalid mode limit");
    if (axis == AxisId::Pitch && !position &&
        !can::valid_pitch_current_limit(limit))
      return fail("pitch speed mode current limit must be finite and in (0, 5 A]");
    if (!std::isfinite(speed_kp) || speed_kp < 1 || speed_kp > 5)
      return fail("speed proportional gain outside commissioning range");
    if (!std::isfinite(speed_ki) || (speed_ki != -1 && (speed_ki < .002 || speed_ki > .05)))
      return fail("speed integral gain outside commissioned range");
    t.axis = axis; t.position = position; t.limit = limit; t.speed_ki = speed_ki; t.speed_kp = speed_kp;
    t.check_displacement = check_displacement;
    t.pitch_brake_with_stop = axis == AxisId::Pitch &&
                              (!system_.pitch_setup_verified() || s.mode != 2);
    t.started = t.sampled = t.still_since = now;
    t.last_q = t.pin = s.q_rad;
    t.stopped_q = s.q_rad;
    // Neutralize both reference registers before braking; only the active mode
    // consumes its register. An unknown or disabled drive is stopped directly.
    if (s.has_feedback && s.mode == 2 && now - s.rx_ns < 100000000LL &&
        !t.pitch_brake_with_stop) {
      if (!write_reg_float(cybergear::Reg::SpdRef, 0, axis) ||
          !write_reg_float(cybergear::Reg::LocRef, s.q_rad, axis)) return fail("neutral brake write failed");
    } else if (!system_.send_stop(axis, &err)) return fail("initial stop failed");
    t.stage = 1;
    return Transition::Pending;
  }
  if (t.axis != axis || t.position != position || t.limit != limit || t.speed_ki != speed_ki || t.speed_kp != speed_kp ||
      t.check_displacement != check_displacement)
    return fail("mode transition request changed while pending");
  if (now - t.started > 2500000000LL) return fail("mode transition timed out");
  if (s.has_feedback && s.faults) return fail("motor fault during mode transition");
  constexpr double kModeDriftLimit = .25 * kDeg2Rad;
  if (t.stage >= 2 || (t.stage == 1 && t.pitch_brake_with_stop)) {
    if (!s.has_feedback || now - s.rx_ns > 100000000LL)
      return fail("feedback lost during disabled mode setup");
    if (!std::isfinite(s.q_rad) || (check_displacement && std::abs(s.q_rad-t.stopped_q) > kModeDriftLimit))
      return fail("axis moved more than 0.25 degrees during mode setup; load holding unverified");
  }
  switch (t.stage) {
    case 1: // Stationary dwell measured from encoder position, not noisy velocity.
      if (!s.has_feedback || now - s.rx_ns > 100000000LL) return fail("feedback lost while braking");
      if (now - t.sampled >= 50000000LL) {
        if (std::abs(s.q_rad - t.last_q) > .04 * kDeg2Rad) {
          t.still_since = now; t.last_q = s.q_rad;
        }
        t.sampled = now;
        const bool ping_ok = axis == AxisId::Pitch && t.pitch_brake_with_stop
            ? system_.send_stop(axis, &err)
            : write_reg_float(cybergear::Reg::SpdRef, 0, axis);
        if (!ping_ok) return fail("brake keepalive failed");
      }
      if (now - t.still_since < 150000000LL) break;
      // An unverified pitch mode was stopped at stage 0. Preserve its
      // pre-stop pose so the dwell cannot hide movement of an unsupported load.
      if (!t.pitch_brake_with_stop) t.stopped_q = s.q_rad;
      if (!system_.send_stop(axis, &err)) return fail("mode stop failed");
      invalidate_commands(axis);
      t.deadline = now + 50000000LL; t.stage = 2;
      break;
    case 2:
      if (now < t.deadline) break;
      if (!s.has_feedback || now - s.rx_ns > 100000000LL) return fail("no stopped feedback");
      if (s.mode != 0 || s.rx_ns < t.deadline-50000000LL)
        return fail("drive did not confirm disabled before mode write");
      t.pin = s.q_rad;
      if (!write_reg_u8(cybergear::Reg::RunMode, position ? 1 : 2, axis)) return fail("mode write failed");
      t.stage = 3;
      break;
    case 3:
      if (axis == AxisId::Pitch) {
        t.pitch_limit_cur = position ? can::kPitchCurrentCeilingA : limit;
        if (!can::valid_pitch_current_limit(t.pitch_limit_cur) ||
            !write_reg_float(cybergear::Reg::LimitCur,
                             static_cast<float>(t.pitch_limit_cur), axis))
          return fail("pitch LimitCur write failed or exceeds 5 A");
        t.read_index = 0;
        t.waiting = false;
        t.stage = 10;
        break;
      }
      if (!write_reg_float(position ? cybergear::Reg::LimitSpd : cybergear::Reg::LimitCur,
                           position ? 0.0f : static_cast<float>(limit), axis)) return fail("limit write failed");
      t.stage = 4;
      break;
    case 4:
      if (axis == AxisId::Pitch && position &&
          !write_reg_float(cybergear::Reg::LimitSpd, 0.0f, axis))
        return fail("pitch zero LimitSpd write failed before position pin");
      if (!write_reg_float(position ? cybergear::Reg::LocRef : cybergear::Reg::SpdRef,
                           position ? t.pin : 0.0, axis)) return fail("neutral reference failed");
      if (speed_ki >= 0 &&
          (!write_reg_float(cybergear::Reg::SpdKp, speed_kp, axis) ||
           !write_reg_float(cybergear::Reg::SpdKi, speed_ki, axis))) return fail("speed gains write failed");
      t.stage = 5;
      break;
    case 5:
    case 9: {
      const cybergear::Reg regs[] = {cybergear::Reg::RunMode,
          position ? cybergear::Reg::LimitSpd : cybergear::Reg::LimitCur,
          position ? cybergear::Reg::LocRef : cybergear::Reg::SpdRef,
          cybergear::Reg::SpdKp, cybergear::Reg::SpdKi};
      const double expected[] = {position ? 1.0 : 2.0,
          position && t.stage == 5 ? 0.0 : limit, position ? t.pin : 0.0, speed_kp, speed_ki};
      if (!t.waiting) {
        if (!system_.begin_register_read(axis, regs[t.read_index], err)) return fail("register request failed");
        t.waiting = true; t.deadline = now + 100000000LL;
        break;
      }
      double value = 0;
      const int result = system_.poll_register_read(value, err);
      if (result < 0 || (result == 0 && now > t.deadline)) return fail("mode readback timed out");
      if (result == 0) break;
      // LocRef readback is quantized by this firmware (observed 29 urad
      // difference from the written float). Allow one feedback encoder count.
      const double tolerance = position && t.read_index == 2 ? 0.0004 : 1e-5;
      if (!std::isfinite(value) || std::abs(value - expected[t.read_index]) > tolerance)
        return fail(("mode readback mismatch at register " +
            std::to_string(static_cast<uint16_t>(regs[t.read_index])) + " got " +
            std::to_string(value) + " expected " + std::to_string(expected[t.read_index])).c_str());
      spdlog::info("drive readback axis={} register=0x{:04x} actual={} expected={}",
                   axis_name(axis), static_cast<uint16_t>(regs[t.read_index]), value, expected[t.read_index]);
      t.waiting = false;
      ++t.read_index;
      if (position && t.stage == 5 && t.read_index == 2) ++t.read_index;
      if (t.read_index == (speed_ki >= 0 ? 5 : 3)) {
        if (t.stage == 9) return complete();
        t.read_index = 0; t.stage = 6;
      }
      break;
    }
    case 6: {
      // The last COMM_TYPE_2 can describe the pose at disable, before the
      // suspended load moved. Read the actual encoder register after setup,
      // immediately before enable; register/gain readback is not position proof.
      if (!t.waiting) {
        if (!system_.begin_register_read(axis, cybergear::Reg::MechPos, err))
          return fail("pre-enable position request failed");
        t.waiting = true; t.deadline = now + 100000000LL;
        break;
      }
      double position_now = 0;
      const int result = system_.poll_register_read(position_now, err);
      if (result < 0 || (result == 0 && now > t.deadline))
        return fail("pre-enable position readback timed out");
      if (result == 0) break;
      t.waiting = false;
      if (!std::isfinite(position_now) || (check_displacement && std::abs(position_now-t.stopped_q) > kModeDriftLimit))
        return fail("pre-enable encoder moved more than 0.25 degrees; load holding unverified");
      if (!system_.send_enable(axis, &err)) return fail("enable failed");
      t.deadline = now + 50000000LL; t.stage = 7;
      break;
    }
    case 7:
      if (now < t.deadline) break;
      if (!s.has_feedback || s.mode != 2 || s.rx_ns < t.deadline - 50000000LL ||
          now - s.rx_ns > 100000000LL) return fail("enabled feedback missing");
      if (!position) return complete();
      // This firmware re-pins LocRef while disabled. Enable with LimitSpd=0,
      // then pin the fresh encoder position before restoring movement authority.
      t.pin = s.q_rad;
      if (!write_reg_float(cybergear::Reg::LocRef, t.pin, axis)) return fail("enabled pin failed");
      t.stage = 8;
      break;
    case 8:
      if (!write_reg_float(cybergear::Reg::LimitSpd, limit, axis)) return fail("restore speed limit failed");
      t.stage = 9;
      break;
    case 10: {
      // Pitch refs and enable are forbidden until both the installed current
      // ceiling and selected speed/position mode have been read back. This
      // poll state keeps the 200 Hz controller path non-blocking.
      const cybergear::Reg regs[] = {cybergear::Reg::LimitCur,
                                     cybergear::Reg::RunMode};
      const double expected[] = {t.pitch_limit_cur, position ? 1.0 : 2.0};
      if (!t.waiting) {
        if (!system_.begin_register_read(axis, regs[t.read_index], err))
          return fail("pitch safety setup read request failed");
        t.waiting = true;
        t.deadline = now + 100000000LL;
        break;
      }
      double value = 0.0;
      const int result = system_.poll_register_read(value, err);
      if (result < 0 || (result == 0 && now > t.deadline))
        return fail("pitch current/mode safety readback timed out");
      if (result == 0) break;
      if (!std::isfinite(value) || std::abs(value - expected[t.read_index]) > 1e-6)
        return fail("pitch current/mode readback mismatch before motion");
      spdlog::info("pitch safety readback register=0x{:04x} actual={} expected={}",
                   static_cast<uint16_t>(regs[t.read_index]), value, expected[t.read_index]);
      t.waiting = false;
      ++t.read_index;
      if (t.read_index == 2) {
        if (!system_.confirm_pitch_setup(t.pitch_limit_cur,
                                         position ? 1 : 2))
          return fail("pitch safety setup confirmation failed");
        t.read_index = 0;
        t.stage = 4;
      }
      break;
    }
  }
  return Transition::Pending;
}

void CanMotorBackend::deenergize(AxisId axis) {
  invalidate_calibration();
  if (transition_.stage && transition_.axis == axis) {
    system_.cancel_register_read();
    transition_ = ModeTransition{};
  }
  invalidate_commands(axis);
  std::string err;
  record_output(axis, NAN, 5, system_.send_stop(axis, &err));
  in_position_mode_[static_cast<size_t>(axis)] = false;
  in_speed_mode_[static_cast<size_t>(axis)] = false;
}

// --- Control loop (fast, non-blocking) --------------------------------------
bool CanMotorBackend::adopt_running_mode(AxisId axis, bool position, std::string& err, double speed_ki, double speed_kp) {
  const auto fail_pitch = [&](const std::string& why) {
    err = why;
    if (axis == AxisId::Pitch) deenergize(axis);
    return false;
  };
  double mode=0, q=0, current=0, speed=0;
  if (speed_ki >= 0) {
    double kp=0, ki=0;
    if (!read_register(axis, cybergear::Reg::SpdKp, kp, 100, err) ||
        !read_register(axis, cybergear::Reg::SpdKi, ki, 100, err) ||
        std::abs(kp-speed_kp)>1e-5 || std::abs(ki-speed_ki)>1e-5)
      return axis == AxisId::Pitch ? fail_pitch("pitch speed gains could not be verified for adoption") : false;
  }
  if (!read_register(axis, cybergear::Reg::RunMode, mode, 100, err) ||
      mode != (position ? 1.0 : 2.0) ||
      !read_register(axis, cybergear::Reg::MechPos, q, 100, err) || !std::isfinite(q))
    return axis == AxisId::Pitch ? fail_pitch("pitch RunMode / position feedback invalid for adoption") : false;
  if (!read_register(axis, cybergear::Reg::LimitCur, current, 100, err) ||
      !read_register(axis, cybergear::Reg::LimitSpd, speed, 100, err) ||
      !std::isfinite(speed))
    return axis == AxisId::Pitch ? fail_pitch("pitch current/speed limit readback failed during adoption") : false;

  const auto now = now_monotonic_ns();
  can::AxisLatest s;
  if (!system_.axis(axis).latest(s) || !s.has_feedback || s.mode != 2 || s.faults ||
      now - s.rx_ns > 100000000LL || !(current > 0 && current <= 23) ||
      !std::isfinite(current))
    return axis == AxisId::Pitch ? fail_pitch("pitch running feedback/limit invalid for adoption") : false;

  if (axis == AxisId::Pitch) {
    if (current > can::kPitchCurrentCeilingA) {
      // A retained run-mode with a higher stored cap is not adoptable. Stop
      // first, lower and read back the cap, then require a fresh controlled
      // mode setup before any new reference or enable.
      deenergize(axis);
      double lowered = 0.0;
      std::string lower_err;
      if (!write_reg_float(cybergear::Reg::LimitCur,
                           static_cast<float>(can::kPitchCurrentCeilingA), axis) ||
          !read_register(axis, cybergear::Reg::LimitCur, lowered, 100, lower_err) ||
          std::abs(lowered - can::kPitchCurrentCeilingA) > 1e-6 ||
          !system_.confirm_pitch_limit(lowered))
        return fail_pitch("pitch stored LimitCur exceeded 5 A and safe reduction could not be verified");
      return fail_pitch("pitch stored LimitCur exceeded 5 A; reduced and verified, controlled mode setup required");
    }
    if (!can::valid_pitch_current_limit(current) ||
        !system_.confirm_pitch_setup(current, position ? 1 : 2))
      return fail_pitch("pitch LimitCur / RunMode did not satisfy the 5 A adoption policy");
  }

  // Neutral references are written only after the pitch cap and supported
  // mode have been confirmed (the system TX guard enforces the same order).
  if (!write_reg_float(cybergear::Reg::SpdRef, 0, axis) ||
      !write_reg_float(cybergear::Reg::LocRef, q, axis))
    return axis == AxisId::Pitch ? fail_pitch("pitch neutral reference failed during adoption") : false;
  const auto i=static_cast<size_t>(axis);
  in_position_mode_[i]=position; in_speed_mode_[i]=!position;
  last_loc_ref_[i]=q; last_spd_ref_[i]=0; last_limit_cur_a_[i]=current; last_limit_spd_[i]=speed;
  return true;
}

AxisSnapshot CanMotorBackend::snapshot(AxisId axis, TimeNs now_ns) {
  AxisSnapshot s;
  can::AxisLatest l;
  if (system_.axis(axis).latest(l)) {
    s.has_feedback = l.has_feedback;
    s.rx_ns = l.rx_ns;
    s.raw_rx_ns = l.rx_ns;
    s.rx_seq = l.frames;
    s.q_rad = l.q_rad;
    s.v_rad_s = l.v_rad_s;
    s.torque_nm = l.torque_nm;
    s.temp_c = l.temp_c;
    s.faults = l.faults;
    s.disabled = l.mode == 0;
    s.enabled_state = l.mode;
  }
  s.in_position_mode = in_position_mode_[static_cast<size_t>(axis)];
  s.in_speed_mode = in_speed_mode_[static_cast<size_t>(axis)];
  return s;
}

void CanMotorBackend::command_velocity(AxisId axis, double velocity_rad_s) {
  if (transition_.stage && transition_.axis == axis) return;
  // Write SpdRef only when it actually changes (mirrors the position-mode
  // write-on-change policy: a same-value rewrite is inert but we avoid the
  // needless CAN TX). The drive holds the written speed with its internal
  // velocity loop until it is changed.
  const int a = static_cast<int>(axis);
  if (std::fabs(velocity_rad_s - last_spd_ref_[a]) > 1e-6) {
    auto& evidence = output_evidence_[a];
    evidence.requested = velocity_rad_s;
    evidence.command_kind = 3;
    cybergear::CanFrame f = cybergear::make_write_reg_float(
        cybergear::Reg::SpdRef, static_cast<float>(velocity_rad_s),
        system_.host_id(), system_.motor_id(axis));
    std::string err;
    if (!system_.send(f.id, f.data, &err)) {
      evidence.reason = 4;
      spdlog::warn("send SpdRef FAIL axis={} v={:+.4f} err={}", a,
                   velocity_rad_s, err);
    } else {
      last_spd_ref_[a] = velocity_rad_s;
      evidence.successful = velocity_rad_s;
      evidence.tx_ns = now_monotonic_ns();
      ++evidence.tx_seq;
      evidence.reason = 1;
    }
  }
  // Keepalive ping: see keepalive() below.
  keepalive(axis);
}

void CanMotorBackend::keepalive(AxisId axis) {
  if (transition_.stage && transition_.axis == axis) return;
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
  if (axis == AxisId::Pitch &&
      !can::valid_pitch_current_limit(last_limit_cur_a_[a])) {
    can::AxisLatest stopped;
    const auto now = now_monotonic_ns();
    if (system_.axis(axis).latest(stopped) && stopped.has_feedback &&
        stopped.mode == 0 && stopped.rx_ns > 0 && stopped.rx_ns <= now &&
        now - stopped.rx_ns < 100000000LL) return;
    spdlog::error("pitch keepalive blocked: cached LimitCur is not verified within 5 A");
    deenergize(axis);
    return;
  }
  // Keep feedback sampling separate from reference changes. The 20 Hz hold
  // cadence left almost no retry margin before the independent 100 ms watchdog
  // (station capture: healthy host, yaw feedback 100.031 ms old). Request at
  // 50 Hz when otherwise silent; the watchdog deadline remains unchanged.
  constexpr int64_t kPingWhenAgeNs = 15'000'000;
  constexpr int64_t kPingIntervalNs = 20'000'000;
  const TimeNs now_ns = now_monotonic_ns();
  can::AxisLatest fb{};
  bool stale = !system_.axis(axis).latest(fb) || !fb.has_feedback;
  if (!stale && (now_ns - fb.rx_ns) > kPingWhenAgeNs) stale = true;
  if (stale && last_limit_cur_a_[a] >= 0.0 &&
      (now_ns - last_ping_ns_[a]) >= kPingIntervalNs) {
    if (write_reg_float(cybergear::Reg::LimitCur,
                       static_cast<float>(last_limit_cur_a_[a]), axis))
      last_ping_ns_[a] = now_ns;
  }
}

void CanMotorBackend::command(AxisId axis, double q_ref_rad,
                              double limit_spd_rad_s) {
  if (transition_.stage && transition_.axis == axis) return;
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
  const int a = static_cast<int>(axis);
  const bool ls_changed = (limit_spd_rad_s != last_limit_spd_[a]);
  const bool qr_changed = std::fabs(q_ref_rad - last_loc_ref_[a]) > kQRefEpsilonRad;
  // Preserve ordering: set the speed limit before a new position reference so
  // the drive never chases a new target without its (possibly reduced) limit.
  if (ls_changed) {
    if (!write_reg_float(cybergear::Reg::LimitSpd,
                         static_cast<float>(limit_spd_rad_s), axis)) return;
    last_limit_spd_[a] = limit_spd_rad_s;
  }
  if (qr_changed) {
    std::string err;
    const bool pin_ok =
        system_.send_position_ref(axis, static_cast<float>(q_ref_rad), &err);
    record_output(axis, static_cast<float>(q_ref_rad), 4, pin_ok);
    if (!pin_ok) {
      spdlog::warn("send_position_ref FAIL axis={} q_ref={:+.6f} err={}",
                   a, q_ref_rad, err);
    } else {
      last_loc_ref_[a] = q_ref_rad;
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
    constexpr int64_t kPingWhenAgeNs = 15'000'000;
    constexpr int64_t kPingIntervalNs = 20'000'000;
    const TimeNs now_ns = now_monotonic_ns();
    can::AxisLatest fb{};
    bool stale = !system_.axis(axis).latest(fb) || !fb.has_feedback;
    if (!stale && (now_ns - fb.rx_ns) > kPingWhenAgeNs) stale = true;
    if (stale && (now_ns - last_ping_ns_[a]) >= kPingIntervalNs) {
      if (write_reg_float(cybergear::Reg::LimitSpd,
                         static_cast<float>(limit_spd_rad_s), axis))
        last_ping_ns_[a] = now_ns;
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
  if (axis == AxisId::Pitch &&
      !can::valid_pitch_current_limit(limit_cur_a)) {
    spdlog::error("pitch current-limit request rejected outside (0, 5 A]; stopping pitch");
    deenergize(axis);
    return;
  }
  if (std::fabs(limit_cur_a - last_limit_cur_a_[a]) < 1e-6) return;
  if (write_reg_float(cybergear::Reg::LimitCur, static_cast<float>(limit_cur_a),
                      axis)) {
    last_limit_cur_a_[a] = limit_cur_a;
  } else {
    spdlog::warn("write LimitCur FAIL axis={} cur={:.2f} A", a, limit_cur_a);
  }
}

void CanMotorBackend::set_speed_loop_gains(AxisId axis, double spd_kp,
                                           double spd_ki) {
  // Write the inner speed-loop gains (SpdKp 0x701F, SpdKi 0x7020). Called once
  // at payload-check start (not every cycle), so a plain fire-and-forget write
  // of both registers is fine — no write-on-change bookkeeping needed. The
  // values persist in the drive across daemon restarts (until the drive is
  // power-cycled or rewritten), which is desirable: the stronger speed loop
  // also makes the post-check position holds more authoritative.
  const int a = static_cast<int>(axis);
  if (!write_reg_float(cybergear::Reg::SpdKp, static_cast<float>(spd_kp),
                       axis))
    spdlog::warn("write SpdKp FAIL axis={} kp={:.4f}", a, spd_kp);
  if (!write_reg_float(cybergear::Reg::SpdKi, static_cast<float>(spd_ki),
                       axis))
    spdlog::warn("write SpdKi FAIL axis={} ki={:.6f}", a, spd_ki);
}

bool CanMotorBackend::restore_stopped_pitch_gains(double kp, double ki, std::string& err) {
  constexpr auto axis=AxisId::Pitch;
  auto fail=[&](const char* why) { deenergize(axis); err=why; return false; };
  if (!std::isfinite(kp) || kp<1 || kp>5 || !std::isfinite(ki) || ki<.002 || ki>.05)
    return fail("restored pitch gains outside supported commissioning bounds");
  deenergize(axis);
  double cap=0, mode=0;
  if (!read_register(axis,cybergear::Reg::LimitCur,cap,100,err) ||
      !read_register(axis,cybergear::Reg::RunMode,mode,100,err)) return fail("pitch setup readback failed during gain restore");
  can::AxisLatest s{};
  const auto now=now_monotonic_ns();
  if (!system_.axis(axis).latest(s) || !s.has_feedback || s.mode!=0 || s.faults ||
      s.rx_ns>now || now-s.rx_ns>100000000LL || !can::valid_pitch_current_limit(cap) ||
      (mode!=1 && mode!=2)) return fail("fresh disabled feedback and safe limit/mode required for gain restore");
  if (!system_.confirm_pitch_setup(cap,static_cast<int>(mode)) ||
      !write_reg_float(cybergear::Reg::SpdKp,kp,axis) ||
      !write_reg_float(cybergear::Reg::SpdKi,ki,axis)) return fail("disabled gain restore write failed");
  for (auto reg:{cybergear::Reg::SpdKp,cybergear::Reg::SpdKi}) {
    double actual=0, expected=reg==cybergear::Reg::SpdKp ? kp:ki;
    if (!read_register(axis,reg,actual,100,err) || !std::isfinite(actual) || std::abs(actual-expected)>1e-6)
      return fail("disabled gain restore readback mismatch");
  }
  deenergize(axis);
  return true;
}


CanHealth CanMotorBackend::can_health() const {
  // Pure counter read: the transport keeps these under its own lock and the
  // control loop calls this only when it builds a report, never to decide
  // anything (§55). An empty answer here would be a lie in the other direction,
  // so `available` is set only after the bus object is actually reached.
  CanHealth h;
  const can::CanTransport& bus = system_.bus();
  const can::BusStats s = bus.stats();
  h.available = true;
  h.kind = bus.kind();
  h.device = bus.device();
  h.up = bus.is_up();
  h.state = static_cast<int>(bus.can_state());
  h.rx_frames = s.rx_frames;
  h.rx_error_frames = s.rx_error_frames;
  h.tx_frames = s.tx_frames;
  h.tx_failed = s.tx_failed;
  h.last_rx_ns = s.last_rx_ns;
  return h;
}

}  // namespace ota
