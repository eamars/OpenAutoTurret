#include "calibration/calibration_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace ota::calibration {
namespace {

std::string quoted(const std::string& s) {
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
                             "\",\"detail\":" + quoted(detail_) + ",\"executor\":\"controld\",\"steps\":" +
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
        "\"provenance\":\"MEASURED\",\"executor\":\"controld\",\"session_label\":" + quoted(s->label_) +
            ",\"manifest_yaml\":" + quoted(manifest_json) + "}",
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

}  // namespace ota::calibration
