// No-motion runtime check for the production split-CAN motor adapter.
// Startup requests zero yaw output -- zero torque current on a current-mode profile, zero voltage
// on a voltage one; the backend decides which -- and CyberGear STOP. No motion targets.
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include "common/time.hpp"
#include "config/mixed_hardware_profile.hpp"
#include "control/boot_fsm.hpp"
#include "control/mixed_can_motor_backend.hpp"

using namespace std::chrono_literals;

namespace {
int parse_seconds(const std::string& value) {
  int seconds = 3;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), seconds);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || seconds < 1 || seconds > 10)
    throw std::runtime_error("--observe-seconds must be from 1 through 10");
  return seconds;
}
}

int main(int argc, char** argv) {
  try {
    std::string config_path = "config/mixed_hardware.yaml";
    int observe_seconds = 3;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
      else if (arg == "--observe-seconds" && i + 1 < argc) observe_seconds = parse_seconds(argv[++i]);
      else throw std::runtime_error("Usage: probe-mixed-backend [--config PATH] [--observe-seconds 1..10]");
    }

    const auto loaded = ota::config::mixed::load_mixed_hardware_profile(config_path);
    if (!loaded.ok) {
      std::string errors;
      for (const auto& error : loaded.errors) errors += (errors.empty() ? "" : "; ") + error;
      throw std::runtime_error("mixed profile rejected: " + errors);
    }

    ota::MixedCanMotorBackend backend;
    std::string error;
    if (!backend.open(loaded.profile, error))
      throw std::runtime_error("mixed backend open failed: " + error);

    ota::BootFsm boot(backend, ota::BootConfig{});
    for (int step = 0; step < 12 && !boot.ready_to_home() && !boot.faulted(); ++step)
      boot.step();
    if (boot.faulted()) throw std::runtime_error("production boot probe failed: " + boot.error());
    if (!boot.ready_to_home()) throw std::runtime_error("production boot probe did not reach UNHOMED");
    if (!backend.yaw_reference_valid()) throw std::runtime_error("yaw session reference was not established");
    if (!backend.buses_healthy()) throw std::runtime_error("one or both CAN buses are unhealthy");
    // Discovery/register replies do not carry CyberGear's enabled/disabled
    // feedback. A STOP request is idempotent on this stopped station and asks
    // the drive for an explicit status frame before declaring it safe.
    backend.deenergize(ota::AxisId::Pitch);
    bool pitch_stop_confirmed = false;
    const auto pitch_deadline = ota::now_monotonic_ns() + 2'000'000'000LL;
    while (ota::now_monotonic_ns() < pitch_deadline) {
      const auto now = ota::now_monotonic_ns();
      const auto pitch = backend.snapshot(ota::AxisId::Pitch, now);
      if (pitch.has_feedback && pitch.rx_ns > 0 && pitch.rx_ns <= now &&
          now - pitch.rx_ns < 100'000'000LL && pitch.disabled_known && pitch.disabled) {
        pitch_stop_confirmed = true;
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    if (!pitch_stop_confirmed)
      throw std::runtime_error("CyberGear STOP did not yield fresh disabled feedback");

    std::cout << "MIXED_BACKEND_READY yaw_continuous=" << backend.supports_continuous_yaw()
              << " yaw_registerless=" << backend.yaw_feedback_registerless()
              << " pitch_uid=0x" << std::hex << boot.unique_ids()[static_cast<size_t>(ota::AxisId::Pitch)]
              << std::dec << " yaw_uid=" << boot.unique_ids()[static_cast<size_t>(ota::AxisId::Yaw)]
              << " yaw_reference_valid=" << backend.yaw_reference_valid()
              << " yaw_disable_confirmation_required=" << backend.requires_disable_confirmation(ota::AxisId::Yaw)
              << "\n";

    for (int tick = 0; tick < observe_seconds * 50; ++tick) {
      backend.heartbeat();
      if (tick % 5 == 0) {
        const auto now = ota::now_monotonic_ns();
        const auto yaw = backend.snapshot(ota::AxisId::Yaw, now);
        const auto pitch = backend.snapshot(ota::AxisId::Pitch, now);
        const auto health = backend.can_health_all();
        std::cout << "OBS t_ms=" << (tick * 20)
                  << " yaw_q_rad=" << yaw.q_rad << " yaw_v_rad_s=" << yaw.v_rad_s
                  << " yaw_age_ms=" << (yaw.has_feedback ? (now - yaw.rx_ns) / 1e6 : -1)
                  << " yaw_temp_raw=" << (yaw.temperature_raw_valid ? int(yaw.temperature_raw) : -1)
                  << " yaw_faults_known=" << yaw.faults_known
                  << " yaw_disabled_known=" << yaw.disabled_known
                  << " pitch_feedback=" << pitch.has_feedback;
        for (size_t i = 0; i < health.size(); ++i) {
          const auto& bus = health[i];
          std::cout << " bus" << i << "_device=" << bus.device
                    << "_up=" << bus.up << "_state=" << bus.state
                    << "_rxerr=" << bus.rx_error_frames << "_txfail=" << bus.tx_failed;
        }
        std::cout << '\n';
        if (!yaw.has_feedback || !backend.buses_healthy() || backend.watchdog_fault())
          throw std::runtime_error("observation gate changed: yaw_feedback=" +
              std::to_string(yaw.has_feedback) + " buses_healthy=" +
              std::to_string(backend.buses_healthy()) + " watchdog_fault=" +
              std::to_string(backend.watchdog_fault()));
      }
      std::this_thread::sleep_for(20ms);
    }

    std::cout << "MIXED_BACKEND_PROBE_PASS no_motion_targets=1 startup_yaw_zero=1 pitch_stop_confirmed=1"
              << " observation_seconds=" << observe_seconds
              << " pitch_disabled_confirmation_required="
              << backend.requires_disable_confirmation(ota::AxisId::Pitch) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "MIXED_BACKEND_PROBE_FAIL " << error.what() << '\n';
    return 1;
  }
}
