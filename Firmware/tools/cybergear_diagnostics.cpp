// Dedicated commissioning owner. Every invocation stops both motors, including
// snapshot. Only an explicit trial enables an axis; no homing or persistent writes.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <yaml-cpp/yaml.h>
#include "can/cybergear_system.hpp"
#include "control/can_motor_backend.hpp"
#include "control/reference_limiter.hpp"
#include "control/speed_servo.hpp"
#include "config/turret_config.hpp"
#include "commissioning_watchdog.hpp"

namespace fs = std::filesystem;
using namespace ota;
using namespace ota::can;
namespace cg = ota::cybergear;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void pause_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void save(const fs::path& path, const YAML::Node& node) {
  std::ofstream file(path);
  file << YAML::Dump(node) << '\n';
  file.flush();
  require(bool(file), "cannot save " + path.string());
}
void ensure_sole_owner() {
  for (const auto& path : fs::directory_iterator("/proc")) {
    std::ifstream file(path.path() / "comm");
    std::string name;
    if (std::getline(file, name)) require(name != "controld", "stop controld before commissioning");
  }
}
double read(CyberGearSystem& system, AxisId axis, cg::Reg reg) {
  double value = 0;
  std::string error;
  require(system.read_register(axis, reg, value, 250, &error),
          std::string(axis_name(axis)) + " " + cg::reg_name(reg) + ": " + error);
  require(std::isfinite(value), "nonfinite register response");
  return value;
}
void stop(CyberGearSystem& system, AxisId axis) {
  std::string error;
  require(system.send_stop(axis, &error), "stop failed: " + error);
}
AxisLatest feedback(CyberGearSystem& system, AxisId axis) {
  AxisLatest latest{};
  require(system.axis(axis).latest(latest) && latest.has_feedback &&
          now_monotonic_ns() - latest.rx_ns < 500'000'000, "feedback absent or stale");
  return latest;
}
YAML::Node snapshot(CyberGearSystem& system) {
  YAML::Node out;
  out["schema_version"] = 1;
  out["timestamp_ns"] = now_monotonic_ns();
  out["persistent_parameters_written"] = false;
  out["optional_raw_status"] = "unvalidated; some firmware echoes the previous runtime value for unsupported addresses";
  for (AxisId axis : {AxisId::Pitch, AxisId::Yaw}) {
    auto node = out[axis_name(axis)];
    node["motor_id"] = unsigned(system.motor_id(axis));
    uint64_t uid = 0;
    std::string error;
    require(system.discover(axis, uid, 500, &error), "discovery failed: " + error);
    node["unique_id"] = std::to_string(uid);
    for (auto reg : {cg::Reg::RunMode, cg::Reg::LocRef, cg::Reg::SpdRef, cg::Reg::IqRef,
                    cg::Reg::LimitSpd, cg::Reg::LimitCur, cg::Reg::LimitTorque,
                    cg::Reg::LocKp, cg::Reg::SpdKp, cg::Reg::SpdKi,
                    cg::Reg::CurKp, cg::Reg::CurKi, cg::Reg::CurFiltGain,
                    cg::Reg::MechPos, cg::Reg::MechVel, cg::Reg::Iqf, cg::Reg::VBus}) {
      node["runtime"][cg::reg_name(reg)] = read(system, axis, reg);
    }
    // These tables are firmware-dependent. Preserve raw replies; do not interpret
    // unsupported addresses as zero or use debugger addresses for writes.
    for (uint16_t address : {0x1003, 0x1004, 0x200c, 0x2014, 0x2015, 0x2016,
                             0x2017, 0x3004, 0x3022, 0x3023}) {
      std::array<uint8_t, 4> bytes{};
      std::ostringstream label, raw;
      label << "0x" << std::hex << address;
      if (system.read_parameter_raw(axis, address, bytes, 120, &error)) {
        for (auto byte : bytes) raw << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
        node["optional_raw"][label.str()] = raw.str();
      } else {
        node["optional_raw"][label.str()] = "unavailable: " + error;
      }
    }
    stop(system, axis);  // a stop reply reports mode and encoder fault flags
    pause_ms(50);
    auto latest = feedback(system, axis);
    node["feedback_mode"] = unsigned(latest.mode);
    node["fault_bits"] = latest.faults;
    node["encoder_uncalibrated_flag"] = bool(latest.faults & 32);
    node["temperature_c"] = latest.temp_c;
  }
  return out;
}

void verified_write(CyberGearSystem& system, AxisId axis, cg::Reg reg, double value,
                    double tolerance = 1e-6) {
  require(std::isfinite(value), "nonfinite write refused");
  const auto frame = reg == cg::Reg::RunMode
      ? cg::make_write_reg_u8(reg, static_cast<uint8_t>(value), system.host_id(), system.motor_id(axis))
      : cg::make_write_reg_float(reg, static_cast<float>(value), system.host_id(), system.motor_id(axis));
  std::string error;
  require(system.send(frame.id, frame.data, &error), "write failed: " + error);
  const double actual = read(system, axis, reg);
  require(std::abs(actual-value) <= tolerance * std::max(1.0, std::abs(value)),
          std::string("readback mismatch: ") + cg::reg_name(reg) +
          " expected=" + std::to_string(value) + " actual=" + std::to_string(actual));
}

double settled_position(CyberGearSystem& system, AxisId axis) {
  stop(system, axis);
  pause_ms(20);
  double position = read(system, axis, cg::Reg::MechPos);
  int stable = 0;
  for (int attempt=0; attempt<15; ++attempt) {
    pause_ms(100);
    const double next = read(system, axis, cg::Reg::MechPos);
    stable = std::abs(next-position) < 0.001 ? stable+1 : 0;
    position = next;
    if (stable >= 3) {
      stop(system, axis);
      pause_ms(20);
      require(feedback(system, axis).mode == 0, "standstill requires disabled feedback");
      return position;
    }
  }
  throw std::runtime_error("axis did not reach position-derived standstill");
}

// A bounded, one-axis experiment. The other axis remains disabled throughout.
// All gain changes are runtime-only, restored and verified on success or failure.
void trial(CyberGearSystem& system, const config::TurretConfig& cfg,
           const fs::path& output, const YAML::Node& before, const fs::path& profile_path) {
  const auto profile = YAML::LoadFile(profile_path.string());
  fs::copy_file(profile_path, output / "trial.yaml");
  const auto envelope = YAML::LoadFile(profile["envelope_state_path"].as<std::string>());
  require(envelope["soft_limits_valid"].as<bool>(), "homed envelope is required");
  const auto axis_name_text = profile["axis"].as<std::string>();
  require(axis_name_text == "pitch" || axis_name_text == "yaw", "invalid trial axis");
  const auto axis = axis_name_text == "pitch" ? AxisId::Pitch : AxisId::Yaw;
  const auto& limits = cfg.axes[static_cast<int>(axis)];
  const auto kind = profile["kind"].as<std::string>();
  const bool servo_step = kind == "speed-servo-step";
  const bool speed_mode = kind == "speed-sine" || servo_step;
  const bool current_mode = kind == "zero-current";
  const bool position_step = kind == "position-step" || servo_step;
  require(kind == "fixed-hold" || kind == "chasing-hold" || kind == "position-sine" ||
          position_step || speed_mode || current_mode, "unknown trial kind");
  const double seconds = profile["duration_s"].as<double>();
  const double speed_limit = profile["speed_limit_rad_s"].as<double>();
  const double current_limit = profile["current_limit_a"].as<double>();
  const double amplitude = profile["amplitude_rad"] ? profile["amplitude_rad"].as<double>() : 0;
  const double frequency = profile["frequency_hz"] ? profile["frequency_hz"].as<double>() : 0;
  const double omega = 2*M_PI*frequency;
  for (double value : {seconds, speed_limit, current_limit, amplitude, frequency})
    require(std::isfinite(value), "nonfinite trial setting");
  const double speed_ceiling = envelope["effective_speed_ceiling_deg_s"].as<double>()*kDeg2Rad;
  require(seconds > 0 && seconds <= 60 && speed_limit >= 0 &&
          (speed_limit > 0 || kind == "chasing-hold") &&
          speed_limit <= std::min(limits.max_velocity_deg_s*kDeg2Rad, speed_ceiling),
          "trial exceeds duration or commissioned speed ceiling");
  const auto original = before[axis_name_text]["runtime"];
  require(current_limit > 0 && current_limit <= original["limit_cur"].as<double>() &&
          current_limit <= limits.limit_cur_a, "trial cannot raise current limit");
  require(amplitude >= 0 && amplitude <= 2*kDeg2Rad && frequency >= 0 &&
          amplitude*omega <= speed_limit &&
          amplitude*omega*omega <= limits.max_acceleration_deg_s2*kDeg2Rad &&
          amplitude*omega*omega*omega <= limits.max_jerk_deg_s3*kDeg2Rad,
          "sine exceeds amplitude, velocity, acceleration or jerk envelope");
  require(!position_step || (seconds >= 10 && amplitude > 0 && frequency == 0),
          "position-step requires >=10 seconds, positive amplitude and zero frequency");
  const double lo = envelope["q_soft_min_" + axis_name_text + "_rad"].as<double>();
  const double hi = envelope["q_soft_max_" + axis_name_text + "_rad"].as<double>();
  require(hi > lo, "invalid homed interval");
  // Require 300 ms of position-derived standstill; drive MechVel is noisy at rest.
  const double q0 = settled_position(system, axis);
  require(q0 > lo + 0.25*(hi-lo) && q0 < hi - 0.25*(hi-lo), "trial needs central safe pose");
  const double excursion_limit = std::max(2*kDeg2Rad, amplitude*2);
  require(q0-excursion_limit > lo && q0+excursion_limit < hi, "trial excursion exceeds envelope");
  auto latest = feedback(system, axis);
  require(latest.mode == 0 && latest.faults == 0, "standby with calibrated encoder is required");
  const double initial_temp = latest.temp_c;
  std::vector<cg::Reg> changed = {cg::Reg::RunMode, cg::Reg::LimitCur, cg::Reg::LimitSpd};
  cg::Reg gain = cg::Reg::SpdKi;
  double gain_value = 0;
  const bool change_gain = bool(profile["gain"]);
  if (change_gain) {
    const auto name = profile["gain"].as<std::string>();
    require(name == "spd_ki" || name == "loc_kp" || name == "spd_kp", "unsupported gain trial");
    gain = name == "spd_ki" ? cg::Reg::SpdKi : (name == "spd_kp" ? cg::Reg::SpdKp : cg::Reg::LocKp);
    gain_value = profile["gain_value"].as<double>();
    require(std::isfinite(gain_value) && gain_value >= 0 &&
            gain_value <= (name == "spd_kp" && servo_step ? 5.0 :
                           name == "spd_ki" && servo_step ? 0.05 : original[name].as<double>()),
            "gain exceeds bounded commissioning range");
    changed.push_back(gain);
  }
  YAML::Node result;
  result["axis"] = axis_name_text;
  result["kind"] = kind;
  result["q_hold_rad"] = q0;
  result["envelope"] = envelope;
  result["restoration"] = original;
  result["references_on_restore"] = "zero speed/current and pin current pose; never reinstate stale destinations";
  save(output / "restoration.yaml", result);  // before the first trial write
  bool restored = false;
  std::unique_ptr<ota::tools::CommissioningWatchdog> watchdog;
  auto finish_watchdog = [&] {
    if (!watchdog) return;
    watchdog->finish();
    result["watchdog_reason"] = ota::tools::CommissioningWatchdog::name(watchdog->reason());
    result["watchdog_trip_ns"] = watchdog->trip_ns();
    result["watchdog_stop_failures"] = watchdog->stop_failures();
    result["watchdog_heartbeat_ms"] = 100;
    result["watchdog_feedback_ms"] = 100;
  };
  auto restore = [&] {
    finish_watchdog();  // revoke command authority before any restore read can wait
    std::string failures;
    auto attempt = [&](const auto& action) {
      try { action(); }
      catch (const std::exception& error) { failures += std::string(error.what()) + "; "; }
    };
    // A failed readback must not prevent restoring the remaining gains/limits.
    double settled = q0;
    bool standstill = false;
    attempt([&] { settled = settled_position(system, axis); standstill = true; });
    // Neutralize references before restoring the original runtime mode/limits.
    attempt([&] { verified_write(system, axis, cg::Reg::SpdRef, 0); });
    attempt([&] { verified_write(system, axis, cg::Reg::IqRef, 0); });
    for (auto reg : changed)
      attempt([&] { verified_write(system, axis, reg, original[cg::reg_name(reg)].as<double>()); });
    // Standby firmware continuously pins LocRef to measured position. Its readback
    // therefore has encoder noise; gain/limit/mode readbacks remain strict.
    if (standstill)
      attempt([&] { verified_write(system, axis, cg::Reg::LocRef, settled, 0.001); });
    attempt([&] {
      stop(system, axis);
      pause_ms(20);
      require(feedback(system, axis).mode == 0, "restoration did not confirm disabled mode");
      result["ended_disabled"] = true;
    });
    require(failures.empty(), failures);
    result["restoration_verified"] = true;
    restored = true;
  };
  try {
    if (change_gain) verified_write(system, axis, gain, gain_value);
    verified_write(system, axis, cg::Reg::RunMode, current_mode ? 3 : (speed_mode ? 2 : 1));
    verified_write(system, axis, cg::Reg::LimitCur, current_limit);
    verified_write(system, axis, cg::Reg::LimitSpd, speed_limit);
    verified_write(system, axis, cg::Reg::SpdRef, 0);
    verified_write(system, axis, cg::Reg::IqRef, 0);
    verified_write(system, axis, cg::Reg::LocRef, q0, 0.001);
    // Start only after setup is verified, while still disabled. All subsequent
    // motion writes go through its latched gate; register reads never hold it.
    stop(system, axis);
    pause_ms(20);
    watchdog = std::make_unique<ota::tools::CommissioningWatchdog>(system,
        ota::tools::CommissioningWatchdog::Limits{axis,
            std::max(lo, q0-excursion_limit), std::min(hi, q0+excursion_limit),
            std::min(cfg.safety.motor_overtemp_c, initial_temp+5),
            static_cast<TimeNs>((seconds+0.5)*1e9)});
    std::string error;
    watchdog->command([&] { require(system.send_enable(axis, &error), "enable failed: " + error); });
    pause_ms(20);
    require(feedback(system, axis).mode == 2, "drive did not enter enabled mode");
    if (!speed_mode && !current_mode) verified_write(system, axis, cg::Reg::LocRef, q0);
    require(read(system, axis, cg::Reg::RunMode) == (current_mode ? 3 : (speed_mode ? 2 : 1)),
            "run mode changed across enable");
    CanMotorBackend backend(system);
    backend.set_current_limit(axis, current_limit); // seed keepalive's verified limit cache
    require(read(system, axis, cg::Reg::LimitCur) == static_cast<float>(current_limit),
            "backend current-limit cache disagrees with trial");
    control::ReferenceLimiter reference;
    control::SpeedServo servo;
    reference.reset_at(q0);
    std::ofstream csv(output / "samples.csv");
    csv << "t_ns,q_rad,v_rad_s,iq_a,temperature_c,feedback_mode,fault_bits,q_ref_rad,loc_ref_rad,velocity_command_rad_s\n";
    std::ofstream commands(output / "commands.csv");
    commands << "t_ns,q_ref_rad,velocity_command_rad_s\n";
    const auto start = now_monotonic_ns();
    TimeNs next_sample = start;
    TimeNs previous_cycle = start;
    double q = q0, v = 0, iq = 0;
    int samples = 0;
    while (!interrupted && (now_monotonic_ns()-start)*1e-9 < seconds) {
      const auto t = now_monotonic_ns();
      const double elapsed = (t-start)*1e-9;
      const bool sine = speed_mode || kind == "position-sine";
      const double target = q0 + (position_step
          ? (elapsed >= 2 && elapsed < seconds-5 ? amplitude : 0)
          : (sine && elapsed < seconds-1 ? amplitude*std::sin(omega*elapsed) : 0));
      const double dt = std::max(1e-6, (t-previous_cycle)*1e-9);
      previous_cycle = t;
      const double shaped = control::limit_reference(reference, target, dt, speed_limit,
          limits.max_acceleration_deg_s2*kDeg2Rad, limits.max_jerk_deg_s3*kDeg2Rad);
      const double q_ref = kind == "chasing-hold" ? q : ((sine || position_step) ? shaped : q0);
      latest = feedback(system, axis);
      const double velocity = servo_step ? servo.step(q_ref, reference.v_rad_s, latest.q_rad,
          speed_limit, dt, limits.max_acceleration_deg_s2*kDeg2Rad,
          limits.max_jerk_deg_s3*kDeg2Rad) : (speed_mode ? reference.v_rad_s : 0);
      require(latest.faults == 0 && latest.mode == 2, "drive fault or unexpected mode during trial");
      require(latest.temp_c < cfg.safety.motor_overtemp_c && latest.temp_c < initial_temp+5,
              "temperature trial limit reached");
      require(std::abs(latest.q_rad-q0) < excursion_limit && latest.q_rad > lo && latest.q_rad < hi,
              "position excursion trial limit reached");
      watchdog->command([&] {
        if (current_mode) {
          if (t >= next_sample) {
            const auto frame = cg::make_write_reg_float(cg::Reg::IqRef, 0, system.host_id(), system.motor_id(axis));
            require(system.send(frame.id, frame.data, &error), "zero current write failed: " + error);
          }
        } else if (speed_mode) backend.command_velocity(axis, velocity);
        else backend.command(axis, q_ref, speed_limit);
      });
      commands << t << ',' << std::setprecision(12) << q_ref << ',' << velocity << '\n';
      if (t >= next_sample) {
        q = read(system, axis, cg::Reg::MechPos);
        v = read(system, axis, cg::Reg::MechVel);
        iq = read(system, axis, cg::Reg::Iqf);
        const double loc_ref = read(system, axis, cg::Reg::LocRef);
        require(std::abs(iq) <= current_limit+0.05, "current trial limit reached");
        csv << std::setprecision(12) << t << ',' << q << ',' << v << ',' << iq << ','
            << latest.temp_c << ',' << unsigned(latest.mode) << ',' << latest.faults << ','
            << q_ref << ',' << loc_ref << ',' << velocity << '\n';
        next_sample = t + 20'000'000;
        ++samples;
      }
      watchdog->heartbeat();
      std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
          std::chrono::nanoseconds(t + 5'000'000)));
    }
    require(bool(csv) && bool(commands), "trial output failed");
    result["samples"] = samples;
    result["interrupted"] = bool(interrupted);
    finish_watchdog();
    require(watchdog->reason() == ota::tools::CommissioningWatchdog::Reason::None,
            std::string("watchdog stopped trial: ") + ota::tools::CommissioningWatchdog::name(watchdog->reason()));
    restore();
    result["ok"] = !interrupted;
    save(output / "result.yaml", result);
  } catch (const std::exception& error) {
    result["ok"] = false;
    result["error"] = error.what();
    try { if (!restored) restore(); }
    catch (const std::exception& restore_error) {
      result["restoration_verified"] = false;
      result["restoration_error"] = restore_error.what();
      system.send_stop(axis);
    }
    save(output / "result.yaml", result);
    throw;
  }
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: cybergear-diagnostics CONFIG OUTPUT_DIRECTORY snapshot|disabled [pitch|yaw] [SECONDS]\n"
                 "       cybergear-diagnostics CONFIG OUTPUT_DIRECTORY trial PROFILE.yaml\n"
                 "Every invocation stops both motors. Only trial enables one axis.\n";
    return 2;
  }
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  CyberGearSystem system;
  try {
    ensure_sole_owner();
    const auto loaded = config::load_turret_config(argv[1]);
    require(loaded.ok, "invalid station config");
    const auto& cfg = loaded.config;
    CyberGearSystemConfig can;
    can.transport = cfg.can.backend;
    can.iface = cfg.can.interface;
    can.uart_baud = cfg.can.uart_baud;
    can.bitrate = cfg.can.bitrate;
    can.host_can_id = cfg.can.host_can_id;
    can.pitch_motor_id = cfg.motors[0].can_id;
    can.yaw_motor_id = cfg.motors[1].can_id;
    const fs::path output(argv[2]);
    require(!fs::exists(output), "output already exists; choose a fresh trial directory");
    fs::create_directories(output);
    fs::copy_file(argv[1], output / "station-config.yaml");
    std::string error;
    require(system.open(can, error), "CAN open failed: " + error);
    stop(system, AxisId::Pitch);
    stop(system, AxisId::Yaw);
    pause_ms(100);
    auto before = snapshot(system);
    save(output / "before.yaml", before);
    const std::string mode(argv[3]);
    require(mode == "snapshot" || mode == "disabled" || mode == "trial", "unknown diagnostic mode");
    if (mode == "trial") {
      require(argc >= 5, "trial needs a profile path");
      trial(system, cfg, output, before, argv[4]);
    }
    if (mode == "disabled") {
      require(argc >= 5, "disabled sample needs pitch or yaw");
      const std::string name(argv[4]);
      require(name == "pitch" || name == "yaw", "invalid axis");
      const AxisId axis = name == "pitch" ? AxisId::Pitch : AxisId::Yaw;
      const double seconds = argc > 5 ? std::stod(argv[5]) : 5;
      require(std::isfinite(seconds) && seconds > 0 && seconds <= 60, "duration must be (0,60] seconds");
      std::ofstream csv(output / "samples.csv");
      csv << "t_ns,q_rad,v_rad_s,iq_a,temperature_c,feedback_mode,fault_bits\n";
      const auto start = now_monotonic_ns();
      int samples = 0;
      while (!interrupted && (now_monotonic_ns()-start)*1e-9 < seconds) {
        const auto t = now_monotonic_ns();
        const auto q = read(system, axis, cg::Reg::MechPos);
        const auto v = read(system, axis, cg::Reg::MechVel);
        const auto iq = read(system, axis, cg::Reg::Iqf);
        stop(system, axis);
        pause_ms(2);
        const auto latest = feedback(system, axis);
        require(latest.mode == 0 && latest.faults == 0, "disabled baseline mode or fault mismatch");
        csv << std::setprecision(12) << t << ',' << q << ',' << v << ',' << iq << ','
            << latest.temp_c << ',' << unsigned(latest.mode) << ',' << latest.faults << '\n';
        ++samples;
        std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(t + 20'000'000)));
      }
      require(bool(csv), "sample output failed");
      std::cout << "disabled samples=" << samples << '\n';
    }
    stop(system, AxisId::Pitch);
    stop(system, AxisId::Yaw);
    system.close();
    std::cout << "saved " << output << "; both motors disabled\n";
    return interrupted ? 130 : 0;
  } catch (const std::exception& error) {
    system.send_stop(AxisId::Pitch);
    system.send_stop(AxisId::Yaw);
    system.close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
