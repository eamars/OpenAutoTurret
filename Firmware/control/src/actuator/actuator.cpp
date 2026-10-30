#include "actuator/actuator.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "common/thread_class.hpp"

namespace ota {
namespace {

constexpr const char* kPwm0Address = "1f00098000";

std::string escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

std::string local_iso_now() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  ::localtime_r(&t, &tm);
  char buf[40];
  std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%S%z", &tm);
  return buf;
}

// sysfs reports a refused value at write(), not at a buffered stream's close: plain POSIX calls.
bool write_file(const std::string& path, const std::string& value, std::string& err) {
  const int fd = ::open(path.c_str(), O_WRONLY);
  if (fd < 0) { err = path + ": " + std::strerror(errno); return false; }
  const ssize_t n = ::write(fd, value.data(), value.size());
  const int e = errno;
  ::close(fd);
  if (n != static_cast<ssize_t>(value.size())) { err = path + " <- " + value + ": " + std::strerror(e); return false; }
  return true;
}

std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::string s;
  std::getline(f, s);
  return s;
}

}  // namespace

std::string actuator_values_refusal(int rest_us, int active_us, double dwell_ms) {
  for (int us : {rest_us, active_us})
    if (us < kActuatorMinPulseUs || us > kActuatorMaxPulseUs)
      return "pulse " + std::to_string(us) + " us is outside " + std::to_string(kActuatorMinPulseUs) + ".." +
             std::to_string(kActuatorMaxPulseUs) + " us";
  if (!std::isfinite(dwell_ms) || dwell_ms < 0 || dwell_ms > kActuatorMaxDwellMs)
    return "dwell must be 0..5000 ms";
  return "";
}

std::string actuator_config_to_json(const ActuatorConfig& c) {
  std::ostringstream o;
  o << "{\n  \"schema\": 1,\n  \"gpio\": " << c.gpio << ",\n  \"period_us\": " << c.period_us
    << ",\n  \"rest_us\": " << c.rest_us << ",\n  \"active_us\": " << c.active_us
    << ",\n  \"dwell_ms\": " << c.dwell_ms << ",\n  \"hold_at_rest\": " << (c.hold_at_rest ? "true" : "false")
    << ",\n  \"saved\": \"" << escape(c.saved) << "\"\n}\n";
  return o.str();
}

bool actuator_config_from_json(const std::string& text, ActuatorConfig& out, std::string& err) {
  out = ActuatorConfig{};
  try {
    const YAML::Node n = YAML::Load(text);
    if (!n.IsMap()) { err = "not a JSON object"; return false; }
    if (!n["schema"] || n["schema"].as<int>() != 1) { err = "schema is not 1"; return false; }
    for (const char* key : {"gpio", "rest_us", "active_us", "dwell_ms"})
      if (!n[key]) { err = std::string(key) + " is required"; return false; }
    ActuatorConfig c;
    c.gpio = n["gpio"].as<int>();
    c.period_us = n["period_us"] ? n["period_us"].as<int>() : 20000;
    // The tool writes whole microseconds; a fraction is read and rounded, never refused for it.
    c.rest_us = static_cast<int>(std::lround(n["rest_us"].as<double>()));
    c.active_us = static_cast<int>(std::lround(n["active_us"].as<double>()));
    c.dwell_ms = n["dwell_ms"].as<double>();
    c.hold_at_rest = n["hold_at_rest"] ? n["hold_at_rest"].as<bool>() : false;
    c.saved = n["saved"] ? n["saved"].as<std::string>() : "";
    if (pwm0_channel_for_gpio(c.gpio) < 0) { err = "GPIO" + std::to_string(c.gpio) + " has no PWM0 channel"; return false; }
    if (c.period_us < 2500 || c.period_us > 50000) { err = "period_us must be 2500..50000"; return false; }
    if (const std::string why = actuator_values_refusal(c.rest_us, c.active_us, c.dwell_ms); !why.empty()) {
      err = why;
      return false;
    }
    if (c.rest_us >= c.period_us || c.active_us >= c.period_us) { err = "a pulse is longer than the period"; return false; }
    c.valid = true;
    out = c;
    err.clear();
    return true;
  } catch (const std::exception& e) {
    err = std::string("unparseable: ") + e.what();
    return false;
  }
}

bool load_actuator_config(const std::string& path, ActuatorConfig& out, std::string& err) {
  out = ActuatorConfig{};
  err.clear();
  std::ifstream f(path);
  if (!f) return false;
  std::stringstream ss;
  ss << f.rdbuf();
  return actuator_config_from_json(ss.str(), out, err);
}

bool save_actuator_config(const std::string& path, const ActuatorConfig& c, std::string& err) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) { err = "cannot open " + tmp + " for writing"; return false; }
    f << actuator_config_to_json(c);
    f.flush();
    if (!f.good()) { err = "write failed for " + tmp; return false; }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    err = "rename " + tmp + " -> " + path + " failed";
    return false;
  }
  err.clear();
  return true;
}

int pwm0_channel_for_gpio(int gpio) {
  switch (gpio) {
    case 12: return 0;
    case 13: return 1;
    case 14: case 18: return 2;
    case 15: case 19: return 3;
    default: return -1;
  }
}

std::unique_ptr<SysfsPwm> SysfsPwm::open(const std::string& class_root, int gpio, int period_us, std::string& err) {
  namespace fs = std::filesystem;
  const int ch = pwm0_channel_for_gpio(gpio);
  if (ch < 0) { err = "GPIO" + std::to_string(gpio) + " has no PWM0 channel"; return nullptr; }
  std::error_code ec;
  fs::path chip;
  for (const auto& e : fs::directory_iterator(class_root, ec)) {
    if (e.path().filename().string().rfind("pwmchip", 0) != 0) continue;
    const fs::path real = fs::canonical(e.path(), ec);
    if (!ec && real.string().find(kPwm0Address) != std::string::npos) { chip = e.path(); break; }
  }
  if (chip.empty()) {
    err = std::string("no PWM chip at ") + kPwm0Address + ": the pwm overlay is not loaded (dtoverlay=pwm,pin=" +
          std::to_string(gpio) + ",func=4)";
    return nullptr;
  }
  const fs::path dir = chip / ("pwm" + std::to_string(ch));
  if (!fs::exists(dir) && !write_file((chip / "export").string(), std::to_string(ch), err)) return nullptr;
  // udev makes the new channel's files writable a moment after the export.
  bool writable = false;
  for (int i = 0; i < 50 && !writable; ++i) {
    writable = ::access((dir / "period").c_str(), W_OK) == 0;
    if (!writable) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!writable) { err = (dir / "period").string() + " is not writable (the gpio group?)"; return nullptr; }
  const std::string period = std::to_string(static_cast<long long>(period_us) * 1000);
  if (read_file((dir / "period").string()) != period) {
    // A longer pulse than the new period is refused by the driver: clear it first.
    std::string ignored;
    write_file((dir / "duty_cycle").string(), "0", ignored);
    if (!write_file((dir / "period").string(), period, err)) return nullptr;
  }
  const bool enabled = read_file((dir / "enable").string()) == "1";
  return std::unique_ptr<SysfsPwm>(new SysfsPwm(dir.string(), enabled));
}

bool SysfsPwm::set(int pulse_us, bool enabled, std::string& err) {
  if (enabled) {
    if (!write_file(dir_ + "/duty_cycle", std::to_string(static_cast<long long>(pulse_us) * 1000), err)) return false;
    if (!enabled_ && !write_file(dir_ + "/enable", "1", err)) return false;
  } else if (enabled_ && !write_file(dir_ + "/enable", "0", err)) {
    return false;
  }
  enabled_ = enabled;
  return true;
}

// ---------------------------------------------------------------------------------------------------

Actuator::Actuator(std::unique_ptr<PwmOutput> pwm, std::string unavailable_reason, std::string path, int gpio,
                   ActuatorConfig config, std::string config_problem, Options opt)
    : pwm_(std::move(pwm)), path_(std::move(path)), gpio_(gpio), opt_(opt) {
  std::lock_guard lk(mutex_);
  if (!pwm_) {
    mode_ = Mode::Unavailable;
    reason_ = unavailable_reason.empty() ? "no PWM channel" : unavailable_reason;
  } else if (!config.valid) {
    mode_ = Mode::Unconfigured;
    reason_ = config_problem.empty() ? "no actuator.json yet: MENU > CALIBRATION > ACTUATOR finds A and B"
                                     : "actuator.json refused: " + config_problem;
  } else if (config.gpio != gpio_) {
    mode_ = Mode::Unconfigured;
    reason_ = "actuator.json is for GPIO" + std::to_string(config.gpio) + ", the launcher's is GPIO" +
              std::to_string(gpio_);
  } else {
    cfg_ = config;
    // Once to A, from wherever the servo is: a crash may have left it at B.
    mode_ = Mode::Settling;
    pulse_us_ = shot_rest_us_ = cfg_.rest_us;
    powered_ = true;
    until_ = Clock::now() + std::chrono::milliseconds(opt_.settle_ms);
  }
  if (mode_ == Mode::Unavailable) spdlog::warn("actuator unavailable: {}", reason_);
  else if (mode_ == Mode::Unconfigured) spdlog::warn("actuator GPIO{} unconfigured: {}", gpio_, reason_);
  else spdlog::info("actuator GPIO{}: rest {} us, active {} us, dwell {} ms, {} at rest", gpio_, cfg_.rest_us,
                    cfg_.active_us, cfg_.dwell_ms, cfg_.hold_at_rest ? "powered" : "limp");
  publish_locked();
  if (pwm_) thread_ = std::thread([this] { run(); });
}

Actuator::~Actuator() { shutdown(); }

const char* Actuator::mode_name_locked() const {
  switch (mode_) {
    case Mode::Unavailable: return "unavailable";
    case Mode::Unconfigured: return "unconfigured";
    case Mode::Settling: return stopping_ ? "stopping" : "returning";
    case Mode::Idle: return powered_ ? "rest" : "off";
    case Mode::Active: return "active";
    case Mode::Calibrating: return "calibrating";
    case Mode::Stopped: return "stopped";
  }
  return "";
}

std::string Actuator::state_name() const {
  std::lock_guard lk(mutex_);
  return mode_name_locked();
}

void Actuator::publish_locked() {
  std::ostringstream o;
  o << "{\"available\":" << (mode_ != Mode::Unavailable ? "true" : "false")
    << ",\"configured\":" << (cfg_.valid ? "true" : "false")
    << ",\"state\":\"" << mode_name_locked() << "\""
    << ",\"reason\":\"" << escape(reason_) << "\""
    << ",\"gpio\":" << gpio_
    << ",\"powered\":" << (powered_ ? "true" : "false")
    << ",\"pulse_us\":";
  if (powered_) o << pulse_us_; else o << "null";
  if (cfg_.valid)
    o << ",\"rest_us\":" << cfg_.rest_us << ",\"active_us\":" << cfg_.active_us << ",\"dwell_ms\":" << cfg_.dwell_ms;
  else
    o << ",\"rest_us\":null,\"active_us\":null,\"dwell_ms\":null";
  o << ",\"hold_at_rest\":" << (cfg_.hold_at_rest ? "true" : "false")
    << ",\"shots\":" << shots_
    << ",\"persistent\":" << (path_.empty() ? "false" : "true")
    << ",\"saved\":" << saved_ << "}";
  status_ = std::make_shared<const std::string>(o.str());
}

std::shared_ptr<const std::string> Actuator::status_json() const {
  std::lock_guard lk(mutex_);
  return status_;
}

std::string Actuator::fire() {
  std::lock_guard lk(mutex_);
  if (stopping_) return "the actuator is stopping";
  switch (mode_) {
    case Mode::Unavailable: return "actuator unavailable: " + reason_;
    case Mode::Unconfigured: return "actuator not calibrated: " + reason_;
    case Mode::Calibrating: return "the actuator's calibration is open";
    case Mode::Active: return "shot in progress";
    case Mode::Settling: return "the servo is still returning to rest";
    case Mode::Stopped: return "the actuator is stopped";
    case Mode::Idle: break;
  }
  mode_ = Mode::Active;
  pulse_us_ = cfg_.active_us;
  shot_rest_us_ = cfg_.rest_us;
  powered_ = true;
  back_to_calibration_ = false;
  until_ = Clock::now() + std::chrono::microseconds(static_cast<int64_t>(cfg_.dwell_ms * 1000.0));
  ++shots_;
  publish_locked();
  cv_.notify_all();
  return "";
}

void Actuator::abort() {
  std::lock_guard lk(mutex_);
  if (mode_ != Mode::Active) return;
  mode_ = Mode::Settling;
  pulse_us_ = shot_rest_us_;
  until_ = Clock::now() + std::chrono::milliseconds(opt_.settle_ms);
  publish_locked();
  cv_.notify_all();
}

std::string Actuator::begin_calibration(std::optional<int> start_us) {
  std::lock_guard lk(mutex_);
  if (stopping_) return "the actuator is stopping";
  if (mode_ == Mode::Unavailable) return "actuator unavailable: " + reason_;
  if (mode_ == Mode::Calibrating) return "";  // already open: the page may ask again
  if (mode_ == Mode::Active || mode_ == Mode::Settling) return "wait: the servo is moving";
  if (mode_ == Mode::Stopped) return "the actuator is stopped";
  const int us = start_us ? *start_us : cfg_.valid ? cfg_.rest_us : 1500;
  if (us < kActuatorMinPulseUs || us > kActuatorMaxPulseUs) return actuator_values_refusal(us, us, 0);
  mode_ = Mode::Calibrating;
  pulse_us_ = us;
  powered_ = true;
  publish_locked();
  cv_.notify_all();
  return "";
}

std::string Actuator::set_pulse(int us) {
  std::lock_guard lk(mutex_);
  if (mode_ != Mode::Calibrating) return "the actuator's calibration is not open";
  if (us < kActuatorMinPulseUs || us > kActuatorMaxPulseUs) return actuator_values_refusal(us, us, 0);
  pulse_us_ = us;
  publish_locked();
  cv_.notify_all();
  return "";
}

std::string Actuator::test(int rest_us, int active_us, double dwell_ms) {
  std::lock_guard lk(mutex_);
  if (mode_ != Mode::Calibrating)
    return mode_ == Mode::Active || mode_ == Mode::Settling ? "test in progress" : "the actuator's calibration is not open";
  if (const std::string why = actuator_values_refusal(rest_us, active_us, dwell_ms); !why.empty()) return why;
  mode_ = Mode::Active;
  pulse_us_ = active_us;
  shot_rest_us_ = rest_us;
  back_to_calibration_ = true;
  until_ = Clock::now() + std::chrono::microseconds(static_cast<int64_t>(dwell_ms * 1000.0));
  publish_locked();
  cv_.notify_all();
  return "";
}

std::string Actuator::save(int rest_us, int active_us, double dwell_ms) {
  std::lock_guard lk(mutex_);
  if (mode_ != Mode::Calibrating) return "the actuator's calibration is not open";
  if (const std::string why = actuator_values_refusal(rest_us, active_us, dwell_ms); !why.empty()) return why;
  ActuatorConfig c = cfg_;  // keeps hold_at_rest and the period
  if (!c.valid) c.period_us = 20000;
  c.valid = true;
  c.gpio = gpio_;
  c.rest_us = rest_us;
  c.active_us = active_us;
  c.dwell_ms = dwell_ms;
  c.saved = local_iso_now();
  cfg_ = c;
  reason_.clear();
  if (path_.empty()) saved_ = 0;
  else pending_save_ = c;
  publish_locked();
  cv_.notify_all();
  return "";
}

std::string Actuator::end_calibration() {
  std::lock_guard lk(mutex_);
  const bool testing = (mode_ == Mode::Active || mode_ == Mode::Settling) && back_to_calibration_;
  if (mode_ != Mode::Calibrating && !testing) return "";  // nothing open: closing is harmless
  back_to_calibration_ = false;
  if (cfg_.valid) {
    mode_ = Mode::Settling;
    pulse_us_ = shot_rest_us_ = cfg_.rest_us;
    powered_ = true;
    until_ = Clock::now() + std::chrono::milliseconds(opt_.settle_ms);
  } else {
    mode_ = Mode::Unconfigured;
    powered_ = false;
  }
  publish_locked();
  cv_.notify_all();
  return "";
}

void Actuator::request_shutdown() {
  std::lock_guard lk(mutex_);
  if (stopping_) return;
  stopping_ = true;
  back_to_calibration_ = false;
  if (cfg_.valid && (mode_ == Mode::Idle || mode_ == Mode::Active || mode_ == Mode::Settling ||
                     mode_ == Mode::Calibrating)) {
    mode_ = Mode::Settling;
    pulse_us_ = shot_rest_us_ = cfg_.rest_us;
    powered_ = true;
    until_ = Clock::now() + std::chrono::milliseconds(opt_.settle_ms);
  } else if (mode_ != Mode::Unavailable) {
    mode_ = Mode::Stopped;
    powered_ = false;
  }
  publish_locked();
  cv_.notify_all();
}

void Actuator::shutdown() {
  request_shutdown();
  if (thread_.joinable()) thread_.join();
}

void Actuator::run() {
  apply_thread_class("actuator", ThreadClass::Background);
  int applied_pulse = -1;
  bool applied_on = false;
  bool first = true;
  std::unique_lock lk(mutex_);
  for (;;) {
    const auto now = Clock::now();
    if (mode_ == Mode::Active && now >= until_) {
      mode_ = Mode::Settling;
      pulse_us_ = shot_rest_us_;
      until_ = now + std::chrono::milliseconds(opt_.settle_ms);
      publish_locked();
    } else if (mode_ == Mode::Settling && now >= until_) {
      if (stopping_) {
        mode_ = Mode::Stopped;
        powered_ = false;
      } else if (back_to_calibration_) {
        back_to_calibration_ = false;
        mode_ = Mode::Calibrating;
      } else {
        mode_ = Mode::Idle;
        powered_ = cfg_.hold_at_rest;
      }
      publish_locked();
    }
    std::optional<ActuatorConfig> save;
    save.swap(pending_save_);
    const int want_pulse = pulse_us_;
    const bool want_on = powered_;
    const bool write = mode_ != Mode::Unavailable &&
                       (first || want_on != applied_on || (want_on && want_pulse != applied_pulse));
    lk.unlock();
    std::string save_err;
    const bool save_ok = save && save_actuator_config(path_, *save, save_err);
    if (save) {
      if (save_ok) spdlog::info("actuator calibration saved to {}", path_);
      else spdlog::error("actuator calibration NOT saved ({}); it holds for this session only", save_err);
    }
    std::string err;
    bool ok = true;
    if (write) {
      ok = pwm_->set(want_pulse, want_on, err);
      if (!ok) ok = pwm_->set(want_pulse, want_on, err);  // once more before giving up
    }
    lk.lock();
    if (save) {
      saved_ = save_ok ? 1 : 0;
      publish_locked();
    }
    if (write) {
      if (ok) {
        applied_pulse = want_pulse;
        applied_on = want_on;
        first = false;
      } else {
        spdlog::error("actuator PWM write failed twice ({}); the actuator is unavailable", err);
        mode_ = Mode::Unavailable;
        reason_ = "PWM write failed: " + err;
        powered_ = false;
        publish_locked();
      }
    }
    if (stopping_ && (mode_ == Mode::Unavailable || (mode_ == Mode::Stopped && applied_on == powered_)) &&
        !pending_save_)
      return;
    // Wake for a request, or at the end of the dwell or the travel.
    const bool timed = mode_ == Mode::Active || mode_ == Mode::Settling;
    const bool pending = pending_save_.has_value() || (mode_ != Mode::Unavailable &&
        (powered_ != applied_on || (powered_ && pulse_us_ != applied_pulse)));
    if (pending) continue;
    if (timed) cv_.wait_until(lk, until_);
    else cv_.wait(lk);
  }
}

}  // namespace ota
