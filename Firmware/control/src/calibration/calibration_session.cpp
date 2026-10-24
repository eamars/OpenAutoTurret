#include "calibration/calibration_session.hpp"
#include "calibration/calibration_store.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "servo_config.hpp"

namespace ota::calibration {
namespace {

std::string json_quote(const std::string& s) {
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') o += "\\r";
    else if (c == '\t') o += "\\t";
    else if (static_cast<unsigned char>(c) < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
    else o += c;
  }
  return o + "\"";
}

double number(const YAML::Node& n, const char* key, bool optional = false) {
  if (!n[key]) {
    if (optional) return 0.0;
    throw std::runtime_error(std::string("DATA_INVALID: ") + key + " is missing");
  }
  const double v = n[key].as<double>();
  if (!std::isfinite(v) || v < 0) throw std::runtime_error(std::string("DATA_INVALID: ") + key + " must be finite and not negative");
  return v;
}

void require(bool ok, const char* what) {
  if (!ok) throw std::runtime_error(what);
}

}  // namespace

void Session::record(const char* row, int length) {
  if (length > 0 && journal_) journal_->append(std::string_view(row, static_cast<std::size_t>(length)));
}

void Session::end(bool complete, std::string detail) {
  complete_ = complete && journal_ && journal_->healthy();
  detail_ = complete && journal_ && !journal_->healthy() ? std::string("DATA_INVALID: ") + journal_->failure_reason()
                                                          : std::move(detail);
  state_.store(SessionState::Holding, std::memory_order_release);
}

bool Session::finish(const std::string& idle_axis_row) {
  if (!journal_) return false;
  if (!idle_axis_row.empty()) journal_->append(idle_axis_row);
  const std::string footer = "{\"kind\":\"footer\",\"status\":\"" + std::string(complete_ ? "COMPLETE" : "INVALID") +
                             "\",\"detail\":" + json_quote(detail_) + ",\"executor\":\"controld\",\"steps\":" +
                             std::to_string(steps_) + ",\"motion_qualified\":false,\"parameter_qualified\":false}";
  return journal_->finish(footer);
}

std::shared_ptr<PitchSession> PitchSession::make(const std::string& manifest_json, const std::string& journal_path,
                                                 std::string label, std::string& err) {
  std::shared_ptr<PitchSession> s(new PitchSession(std::move(label), journal_path));
  try {
    const YAML::Node m = YAML::Load(manifest_json);
    require(m["schema"] && m["schema"].as<std::string>() == "adr0022.sensorless-homing/1",
            "INTEGRATION_MISMATCH: not a pitch session manifest");
    const YAML::Node t = m["servo_trial"];
    require(t && t.IsMap(), "DATA_INVALID: the manifest has no servo_trial");
    // controld's own homing gives the window (D7.5); a session that homes again is commissiond's.
    require(!(t["home_first"] && t["home_first"].as<bool>()), "INTEGRATION_MISMATCH: home_first: controld has homed already");
    const axis::PositionLoopParameters loop{number(t, "kp_per_s"), number(t, "ki_per_s2", true),
                                            number(t, "integral_clamp_rad_s", true), number(t, "speed_limit_rad_s")};
    require(loop.speed_limit > 0 && loop.speed_limit <= 1.0 && s->loop_.configure(loop), "DATA_INVALID: pitch servo trial loop");
    const double period = number(t, "command_period_s");
    require(period >= 0.0005 && period <= 0.02, "DATA_INVALID: pitch servo trial timing");
    s->period_ns_ = static_cast<TimeNs>(period * 1e9);
    s->hold_after_ns_ = static_cast<TimeNs>(number(t, "hold_after_s", true) * 1e9);
    s->following_error_ = number(t, "following_error_rad");
    s->approach_speed_ = number(t, "approach_speed_rad_s");
    s->margin_ = number(t, "window_margin_rad");
    require(s->following_error_ > 0 && s->approach_speed_ > 0 && s->approach_speed_ <= 0.2,
            "DATA_INVALID: pitch servo trial guards");
    if (const auto x = t["excitation"]) {
      s->excitation_ = {x["amplitude_rad_s"].as<double>(), x["f0_hz"].as<double>(), x["f1_hz"].as<double>(),
                        x["begin_s"].as<double>(), x["duration_s"].as<double>()};
      require(s->excitation_.valid(0.2), "DATA_INVALID: pitch servo trial excitation");
    }
    if (const auto schedule = t["gain_schedule"]) {
      for (const auto& g : schedule) s->schedule_.push_back({g["begin_s"].as<double>(), g["kp"].as<double>(), g["ki"].as<double>()});
      require(!s->schedule_.empty() && s->schedule_.size() <= 64, "DATA_INVALID: pitch servo trial gain schedule");
      for (const auto& g : s->schedule_)
        require(std::isfinite(g[1]) && g[1] > 0 && std::isfinite(g[2]) && g[2] >= 0,
                "DATA_INVALID: pitch servo trial gain schedule entry");
    }
    const auto samples = t["reference_samples"];
    require(samples && samples.IsSequence() && samples.size() >= 2, "DATA_INVALID: pitch trial reference table");
    try {
      for (const auto& item : samples)
        s->reference_.add({item["time_s"].as<double>(), item["position_rad"].as<double>(),
                           item["velocity_rad_s"].as<double>(), item["acceleration_rad_s2"].as<double>()});
    } catch (const std::runtime_error&) {
      throw std::runtime_error("DATA_INVALID: pitch trial reference");
    }
    s->expected_s_ = s->reference_.duration() + s->hold_after_ns_ * 1e-9;
    s->journal_ = std::make_unique<capture::Journal>(
        journal_path,
        "{\"kind\":\"header\",\"schema\":\"adr0022.sensorless-homing/1\",\"purpose\":\"pitch_sensorless_homing\","
        "\"provenance\":\"MEASURED\",\"executor\":\"controld\",\"session_label\":" + json_quote(s->label_) +
            ",\"manifest_yaml\":" + json_quote(to_json(m)) + "}",
        8192);
  } catch (const std::exception& e) {
    err = e.what();
    return nullptr;
  }
  return s;
}

void PitchSession::begin(TimeNs now, double pose, const PitchWindow& window) {
  begun_ = true;
  min_ = window.min_rad;
  max_ = window.max_rad;
  centre_ = 0.5 * (min_ + max_);
  slack_ = margin_ + 0.02;
  origin_ = pose;
  approach_s_ = std::max(0.5, std::abs(centre_ - pose) / approach_speed_ * 1.5);
  begin_ns_ = now + static_cast<TimeNs>(approach_s_ * 1e9);   // script time zero, after the approach
  next_ns_ = now;
  expected_s_.store(expected_s_.load(std::memory_order_relaxed) + approach_s_, std::memory_order_relaxed);
  char row[384];
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"pitch_trial_window\",\"endpoint_low_rad\":%.9g,\"endpoint_high_rad\":%.9g,\"window_min_rad\":%.9g,"
      "\"window_max_rad\":%.9g,\"center_rad\":%.9g,\"source\":\"controld homing\"}",
      min_ - margin_, max_ + margin_, min_, max_, centre_));
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"pitch_trial_approach\",\"from_rad\":%.9g,\"center_rad\":%.9g,\"duration_s\":%.9g}",
      pose, centre_, approach_s_));
}

double PitchSession::hold(TimeNs now, double pose) {
  const double dt = last_ns_ ? (now - last_ns_) * 1e-9 : 0.0;
  last_ns_ = now;
  const double limit = loop_.parameters().speed_limit;
  return std::clamp(loop_.step(dt, hold_q_, 0.0, pose), -limit, limit);
}

double PitchSession::step(TimeNs now, double pose, TimeNs status_ns, double torque_nm, const PitchWindow& window) {
  if (state() != SessionState::Running) return hold(now, pose);
  if (const char* why = stop_requested()) {
    hold_q_ = pose;
    end(false, std::string("stopped: ") + why);
    return hold(now, pose);
  }
  if (!begun_) {
    if (!window.valid()) {
      hold_q_ = pose;
      end(false, "MEASUREMENT_LIMITED: no pitch window from homing");
      return hold(now, pose);
    }
    begin(now, pose, window);
  }
  const double elapsed = (now - begin_ns_) * 1e-9;
  const double slack = elapsed < 0 ? slack_ : 0.0;
  if (pose < min_ - slack || pose > max_ + slack) {
    hold_q_ = pose;
    end(false, "HARD_ABORT: pitch outside servo trial window");
    return hold(now, pose);
  }
  if (now - begin_ns_ >= static_cast<TimeNs>(reference_.duration() * 1e9) + hold_after_ns_) {
    char row[160];
    record(row, std::snprintf(row, sizeof row, "{\"kind\":\"pitch_trial_end\",\"time_ns\":%lld,\"pose_rad\":%.9g}",
                              static_cast<long long>(now), pose));
    hold_q_ = centre_ + reference_.at(reference_.duration(), true).position;
    end(true, "");
    return hold(now, pose);
  }
  if (now < next_ns_) return command_;
  next_ns_ += period_ns_;
  if (next_ns_ < now) next_ns_ = now + period_ns_;
  while (next_gain_ < schedule_.size() && elapsed >= schedule_[next_gain_][0]) {
    const auto& g = schedule_[next_gain_++];
    loop_.set_gains(g[1], g[2]);
    char row[160];
    record(row, std::snprintf(row, sizeof row, "{\"kind\":\"pitch_trial_gains\",\"time_ns\":%lld,\"kp\":%.9g,\"ki\":%.9g}",
                              static_cast<long long>(now), g[1], g[2]));
  }
  const auto r = reference_.at(std::max(0.0, elapsed), true);
  double q_ref = centre_ + r.position, v_ref = r.velocity;
  if (elapsed < 0) {
    // Smoothstep approach from the starting pose to the centre.
    const double x = std::clamp(1 + elapsed / approach_s_, 0.0, 1.0), d = centre_ - origin_;
    q_ref = origin_ + d * (3 * x * x - 2 * x * x * x);
    v_ref = d * (6 * x - 6 * x * x) / approach_s_;
  }
  if (std::abs(q_ref - pose) > following_error_) {
    hold_q_ = pose;
    end(false, "HARD_ABORT: pitch servo following error");
    return hold(now, pose);
  }
  const double dt = last_ns_ ? (now - last_ns_) * 1e-9 : 0.0;
  last_ns_ = now;
  const double x = excitation_.at(elapsed);
  const double limit = loop_.parameters().speed_limit;
  command_ = std::clamp(loop_.step(dt, q_ref, v_ref, pose) + x, -limit, limit);
  ++steps_;
  char row[384];
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"pitch_trial\",\"time_ns\":%lld,\"qr\":%.9g,\"vr\":%.9g,\"q\":%.9g,\"status_ns\":%lld,\"torque\":%.6g,"
      "\"cmd\":%.9g,\"x\":%.9g,\"i\":%.9g}",
      static_cast<long long>(now), q_ref, v_ref, pose, static_cast<long long>(status_ns), torque_nm, command_, x,
      loop_.integral()));
  if (journal_ && !journal_->healthy()) {
    hold_q_ = pose;
    end(false, std::string("DATA_INVALID: ") + journal_->failure_reason());
    return hold(now, pose);
  }
  return command_;
}

std::shared_ptr<YawSession> YawSession::make(const std::string& manifest_json, const std::string& journal_path,
                                             std::string label, std::string& err) {
  std::shared_ptr<YawSession> s(new YawSession(std::move(label), journal_path));
  try {
    const YAML::Node m = YAML::Load(manifest_json);
    require(m["schema"] && m["schema"].as<std::string>() == "adr0022.yaw-control/1" && m["purpose"] &&
                m["purpose"].as<std::string>() == "yaw_shared_core_3a",
            "INTEGRATION_MISMATCH: not a yaw session manifest");
    s->current_bound_ = number(m, "yaw_current_bound_A");
    // Owner ruling 2026-10-02: GM6020 peaks up to the 3 A protocol full scale, the servo's RMS budget up
    // to the 1.62 A continuous rating.
    require(s->current_bound_ > 0 && s->current_bound_ <= 3.0, "HARD_ABORT: yaw current authority");
    try {
      s->parameters_ = axis::servo_from_yaml(m["servo_parameters"]);
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string("DATA_INVALID: servo parameters: ") + e.what());
    }
    require(s->parameters_.current_cap == s->current_bound_, "INTEGRATION_MISMATCH: current authority differs from servo");
    require(s->parameters_.rms_limit <= 1.62 && s->parameters_.rms_tau_s <= 5.0,
            "HARD_ABORT: servo RMS budget above the 1.62 A continuous rating");
    require(s->servo_.configure(s->parameters_), "DATA_INVALID: servo parameter apply failed");
    s->temperature_limit_ = number(m, "servo_motor_temperature_limit_C");
    require(s->temperature_limit_ > 0 && s->temperature_limit_ <= 70.0, "DATA_INVALID: servo motor temperature limit");
    // Owner rule 2026-10-02: stay below 100 RPM (10.47 rad/s).
    s->speed_limit_ = number(m, "servo_speed_limit_rad_s");
    require(s->speed_limit_ > 0 && s->speed_limit_ <= 10.0, "DATA_INVALID: servo speed limit");
    s->hold_after_s_ = number(m, "servo_hold_after_s");
    require(s->hold_after_s_ <= 10.0, "DATA_INVALID: servo hold window");
    s->oscillation_ = axis::OscillationMonitor(number(m, "servo_oscillation_limit_A", true));
    if (const auto schedule = m["servo_gain_schedule"]) {
      for (const auto& g : schedule)
        s->schedule_.push_back({g["begin_s"].as<double>(), g["kq"].as<double>(), g["kv"].as<double>(), g["ki"].as<double>()});
      require(!s->schedule_.empty() && s->schedule_.size() <= 64, "DATA_INVALID: servo gain schedule");
    }
    if (const auto x = m["servo_excitation"]) {
      // Identification only: a log-swept sine added to the servo output.
      s->excitation_ = {x["amplitude_A"].as<double>(), x["f0_hz"].as<double>(), x["f1_hz"].as<double>(),
                        x["begin_s"].as<double>(), x["duration_s"].as<double>()};
      require(s->excitation_.valid(0.4), "DATA_INVALID: servo excitation");
    }
    const auto samples = m["reference_samples"];
    require(samples && samples.IsSequence() && samples.size() >= 2, "DATA_INVALID: finite reference sample table required");
    try {
      for (const auto& item : samples)
        s->reference_.add({item["time_s"].as<double>(), item["position_rad"].as<double>(),
                           item["velocity_rad_s"].as<double>(), item["acceleration_rad_s2"].as<double>()});
    } catch (const std::runtime_error&) {
      throw std::runtime_error("DATA_INVALID: reference samples must be finite, start at zero and increase in time");
    }
    s->expected_s_ = s->reference_.duration() + s->hold_after_s_;
    // 1 kHz sessions log about three rows a millisecond; the ring absorbs SD-card write stalls.
    s->journal_ = std::make_unique<capture::Journal>(
        journal_path,
        "{\"kind\":\"header\",\"schema\":\"adr0022.yaw-control/1\",\"purpose\":\"yaw_shared_core_3a\",\"provenance\":"
        "\"MEASURED\",\"executor\":\"controld\",\"candidate_label\":" + json_quote(s->label_) +
            ",\"parameter_qualified\":false,\"manifest_yaml\":" + json_quote(to_json(m)) + "}",
        16384);
  } catch (const std::exception& e) {
    err = e.what();
    return nullptr;
  }
  return s;
}

double YawSession::position() const { return offset_rad_ + counts_ * (2.0 * M_PI / 8192.0); }

void YawSession::stop_here(const std::string& detail) {
  const std::string learned = "{\"kind\":\"servo_learned\",\"parameters\":" + axis::servo_to_json(servo_.learned()) + "}";
  if (journal_) journal_->append(learned);
  end(false, detail);
}

double YawSession::control(TimeNs now, const YawFrame& f) {
  if (state() != SessionState::Running) return 0.0;   // until production takes the axis back
  if (have_encoder_) {
    int delta = int(f.angle_count) - int(previous_count_);
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    counts_ += delta;
  } else {
    offset_rad_ = f.angle_count * (2.0 * M_PI / 8192.0);
  }
  have_encoder_ = true;
  previous_count_ = f.angle_count;
  char row[640];
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"yaw_feedback\",\"encoder_raw\":%u,\"encoder_unwrapped_counts\":%lld,\"q_relative_rad\":%.17g,"
      "\"speed_rpm\":%d,\"current_raw\":%d,\"current_A\":%.17g,\"temperature_raw\":%u,\"kernel_realtime_ns\":%lld,"
      "\"kernel_monotonic_ns\":%lld,\"dequeue_ns\":%lld}",
      unsigned(f.angle_count), static_cast<long long>(counts_), position(), int(f.speed_rpm), int(f.current_raw),
      f.current_a, unsigned(f.temperature_raw), static_cast<long long>(f.rx_realtime_ns),
      static_cast<long long>(f.rx_ns), static_cast<long long>(f.dequeue_ns)));
  if (const char* why = stop_requested()) {
    stop_here(std::string("stopped: ") + why);
    return 0.0;
  }
  if (!begin_ns_) {
    // Control begins on this frame, at the axis, from the current production was applying.
    epoch_ns_ = now - 1'000'000;
    begin_ns_ = now;
    initial_position_ = position();
    if (!servo_.reset((now - epoch_ns_) * 1e-9, initial_position_, 0.0, entry_current_)) {
      stop_here("DATA_INVALID: servo reset failed");
      return 0.0;
    }
    record(row, std::snprintf(row, sizeof row, "{\"kind\":\"yaw_control_begin\",\"time_ns\":%lld}",
                              static_cast<long long>(now)));
    const std::string readback = "{\"kind\":\"controller_parameters_readback\",\"candidate_label\":" + json_quote(label_) +
                                 ",\"source\":\"configured_servo\",\"parameters\":" +
                                 axis::servo_to_json(servo_.parameters()) + "}";
    if (journal_) journal_->append(readback);
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double since = (now - begin_ns_) * 1e-9;
  if (since >= reference_.duration() + hold_after_s_) {
    // The servo held the final reference sample; the session is done.
    const std::string learned = "{\"kind\":\"servo_learned\",\"parameters\":" + axis::servo_to_json(servo_.learned()) + "}";
    if (journal_) journal_->append(learned);
    end(true, "");
    return 0.0;
  }
  if (!servo_.observe_encoder((f.rx_ns - epoch_ns_) * 1e-9, position())) {
    stop_here("MEASUREMENT_LIMITED: servo encoder update rejected");
    return 0.0;
  }
  // A backlog read in one burst feeds the observer but does not step (as production's servo).
  if (last_step_ns_ && now - last_step_ns_ < static_cast<TimeNs>(parameters_.dt_min * 1e9))
    return std::numeric_limits<double>::quiet_NaN();
  while (next_gain_ < schedule_.size() && since >= schedule_[next_gain_][0]) {
    const auto& g = schedule_[next_gain_++];
    if (!servo_.set_gains(g[1], g[2], g[3])) {
      stop_here("DATA_INVALID: servo gain schedule entry");
      return 0.0;
    }
    record(row, std::snprintf(row, sizeof row, "{\"kind\":\"servo_gains\",\"time_ns\":%lld,\"kq\":%.9g,\"kv\":%.9g,\"ki\":%.9g}",
                              static_cast<long long>(now), g[1], g[2], g[3]));
  }
  const auto r = reference_.at(since);
  // References are relative to the position where control began.
  const auto out = servo_.step((now - epoch_ns_) * 1e-9, initial_position_ + r.position, r.velocity, r.acceleration);
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"servo_cycle\",\"time_ns\":%lld,\"qr\":%.9g,\"vr\":%.9g,\"ar\":%.9g,\"q\":%.9g,\"v\":%.9g,\"gyro\":0,"
      "\"req\":%.9g,\"u\":%.9g,\"ff\":%.9g,\"fr\":%.9g,\"p\":%.9g,\"d\":%.9g,\"i\":%.9g,\"rms\":%.9g,\"cap\":%.9g,"
      "\"sat\":%d,\"rock\":%d,\"stalls\":%ld,\"stale\":%ld,\"status\":%d}",
      static_cast<long long>(now), initial_position_ + r.position, r.velocity, r.acceleration, out.position,
      out.velocity, out.requested, out.limited, out.feedforward, out.friction, out.proportional, out.derivative,
      out.integral, out.rms, out.cap, out.saturated, out.rocking, out.stall_events, servo_.stale_samples(), out.status));
  if (f.temperature_raw >= temperature_limit_) {
    stop_here("HARD_ABORT: yaw motor temperature limit");
    return 0.0;
  }
  if (out.status == static_cast<int>(axis::ServoStatus::FollowingError)) {
    stop_here("HARD_ABORT: servo following error limit");
    return 0.0;
  }
  if (out.status != static_cast<int>(axis::ServoStatus::Ok)) {
    stop_here("MEASUREMENT_LIMITED: servo sensor data stale or invalid");
    return 0.0;
  }
  // Physical speed: encoder displacement over at least 20 ms. The observer's instantaneous estimate
  // overshoots for a few ms after a breakaway and is not, by itself, evidence of real overspeed.
  speed_ring_[speed_head_] = {now, position()};
  speed_head_ = (speed_head_ + 1) % speed_ring_.size();
  speed_count_ = std::min(speed_count_ + 1, speed_ring_.size());
  for (std::size_t k = 1; k < speed_count_; ++k) {
    const auto& [then, where] = speed_ring_[(speed_head_ + speed_ring_.size() - 1 - k) % speed_ring_.size()];
    if (now - then < 20'000'000) continue;
    const double speed = (position() - where) / ((now - then) * 1e-9);
    if (std::abs(speed) > speed_limit_) {
      char why[96];
      std::snprintf(why, sizeof why, "HARD_ABORT: yaw speed limit (encoder %.3f rad/s)", speed);
      stop_here(why);
      return 0.0;
    }
    break;
  }
  if (out.rocking) last_rock_ns_ = now;
  const bool rock_settling = last_rock_ns_ && now - last_rock_ns_ < 200'000'000;
  if (oscillation_.update(last_step_ns_ ? (now - last_step_ns_) * 1e-9 : 0.0, out.limited, rock_settling)) {
    char why[96];
    std::snprintf(why, sizeof why, "HARD_ABORT: servo oscillation (fast current RMS %.3f A)", oscillation_.rms());
    stop_here(why);
    return 0.0;
  }
  last_step_ns_ = now;
  const double x = excitation_.at(since);
  requested_ = out.limited + x;
  limited_ = std::clamp(requested_, -current_bound_, current_bound_);
  phase_ = x != 0.0 ? "excitation" : "control";
  pending_ = true;
  ++steps_;
  return limited_;
}

void YawSession::sent(bool ok, double actual_a, int raw, TimeNs begin_ns, TimeNs accepted_ns) {
  if (!pending_ || state() != SessionState::Running) return;
  pending_ = false;
  char row[384];
  char tx[32] = "null";
  if (ok) std::snprintf(tx, sizeof tx, "%.17g", actual_a);
  record(row, std::snprintf(row, sizeof row,
      "{\"kind\":\"yaw_current_tx\",\"phase\":\"%s\",\"requested_A\":%.17g,\"limited_A\":%.17g,\"successful_tx_A\":%s,"
      "\"current_raw\":%d,\"id\":510,\"unused_slots_zero\":true,\"begin_ns\":%lld,\"kernel_accepted_ns\":%lld,\"success\":%s}",
      phase_, requested_, limited_, tx, raw, static_cast<long long>(begin_ns), static_cast<long long>(accepted_ns),
      ok ? "true" : "false"));
  servo_.acknowledge(ok, ok ? actual_a : 0.0);
  if (!ok) stop_here("HARD_ABORT: yaw control TX failed");
  else if (journal_ && !journal_->healthy()) stop_here(std::string("DATA_INVALID: ") + journal_->failure_reason());
}

}  // namespace ota::calibration
