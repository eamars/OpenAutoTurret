// Bounded actual-transport commissioning; launcher owns this process.
// Default: GM receive only plus CyberGear discovery and read-only mechPos query.
// Explicit --yaw-voltage enables a short voltage pulse, never pitch actuation.
#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numbers>
#include <thread>
#include <string_view>
#include <yaml-cpp/yaml.h>

#include "can/cybergear_protocol.hpp"
#include "can/gm6020_protocol.hpp"
#include "can/socketcan_bus.hpp"

using namespace std::chrono_literals;
static volatile std::sig_atomic_t interrupted = 0;
static void stop_signal(int) { interrupted = 1; }
static constexpr double degrees = 180.0 / std::numbers::pi;

struct Sample {
  ota::gm6020::Feedback feedback;
  double position{};
  uint64_t count{};
  bool valid{};
};

int main(int argc, char** argv) {
 try {
  std::string config = "config/hardware_probe.yaml", trace_path;
  int voltage = 0, pulse_ms = 100, observe_ms = 2000;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (i + 1 >= argc) throw std::runtime_error("option requires a value: " + arg);
    const std::string value = argv[++i];
    if (arg == "--config") config = value;
    else if (arg == "--yaw-voltage") voltage = std::stoi(value);
    else if (arg == "--pulse-ms") pulse_ms = std::stoi(value);
    else if (arg == "--observe-ms") observe_ms = std::stoi(value);
    else if (arg == "--trace") trace_path = value;
    else throw std::runtime_error("unknown option: " + arg);
  }
  const auto cfg = YAML::LoadFile(config);
  if (cfg["schema_version"].as<int>() != 1) throw std::runtime_error("unknown probe schema");
  const auto limit = cfg["limits"];
  const int voltage_limit = limit["voltage_raw"].as<int>();
  const int duration_limit = limit["pulse_ms"].as<int>();
  const double travel_limit = limit["yaw_travel_deg"].as<double>();
  const double speed_limit = limit["yaw_speed_deg_s"].as<double>();
  const double age_limit = limit["feedback_age_ms"].as<double>();
  const double heartbeat_limit = limit["heartbeat_age_ms"].as<double>();
  if (voltage_limit < 1 || voltage_limit > 3000 || std::abs(voltage) > voltage_limit ||
      duration_limit < 1 || duration_limit > 500 || pulse_ms < 1 || pulse_ms > duration_limit ||
      !std::isfinite(travel_limit) || travel_limit <= 0 || travel_limit > 5 ||
      !std::isfinite(speed_limit) || speed_limit <= 0 || speed_limit > 20 ||
      !std::isfinite(age_limit) || age_limit <= 0 || age_limit > 20 ||
      !std::isfinite(heartbeat_limit) || heartbeat_limit <= 0 || heartbeat_limit > 40 ||
      observe_ms < 1000 || observe_ms > 10000)
    throw std::runtime_error("probe limits outside fixed commissioning envelope");
  const auto yaw_id = cfg["yaw"]["motor_id"].as<int>();
  const auto pitch_id = cfg["pitch"]["motor_id"].as<int>();
  if (yaw_id < 1 || yaw_id > 7 || pitch_id < 1 || pitch_id > 255)
    throw std::runtime_error("invalid motor ID");
  const auto expected_uid = std::stoull(cfg["pitch"]["unique_id_hex"].as<std::string>(), nullptr, 16);
  const auto bitrate = cfg["bitrate"].as<uint32_t>();
  if (bitrate != 1000000) throw std::runtime_error("expected classical CAN 1 Mbps");
  std::ofstream trace;
  if (!trace_path.empty()) {
    trace.open(trace_path);
    if (!trace) throw std::runtime_error("cannot open trace");
    trace << "time_ns,phase,voltage_raw,angle_count,yaw_relative_deg,speed_deg_s,current_raw,temperature_raw,feedback_age_ms\n";
  }
  std::signal(SIGTERM, stop_signal); std::signal(SIGINT, stop_signal);
  std::mutex sample_mutex;
  Sample sample;
  ota::gm6020::UnwrappedEncoder encoder;
  std::atomic<uint64_t> uid{0}, pitch_frames{0};
  std::atomic<bool> pitch_position_valid{false};
  std::atomic<double> pitch_position{0};
  ota::can::SocketCanBus yaw, pitch;
  yaw.set_frame_callback([&](const ota::can::RawFrame& f) {
    ota::gm6020::Feedback feedback;
    if (!ota::gm6020::decode(f, static_cast<uint8_t>(yaw_id), feedback)) return;
    std::lock_guard lock(sample_mutex);
    sample.feedback = feedback;
    sample.valid = encoder.update(feedback.angle_count, feedback.rx_ns);
    sample.position = encoder.relative_rad(); ++sample.count;
  });
  pitch.set_frame_callback([&](const ota::can::RawFrame& f) {
    if (!f.extended || f.rtr || f.error || f.dlc != 8) return;
    ota::cybergear::CanFrame frame{f.id, f.dlc, {}};
    std::copy(std::begin(f.data), std::end(f.data), frame.data);
    ota::cybergear::DiscoveryResponse response;
    if (ota::cybergear::parse_discovery_response(frame, response) && response.motor_id == pitch_id) {
      uid.store(response.unique_id); ++pitch_frames;
    }
    const auto ident = ota::cybergear::unpack_ext_id(f.id);
    ota::cybergear::Reg reg; double value;
    if ((ident.data2 & 0xff) == pitch_id && ident.target == 0 &&
        ota::cybergear::parse_reg_response(frame, reg, value) &&
        reg == ota::cybergear::Reg::MechPos && std::isfinite(value)) {
      pitch_position.store(value); pitch_position_valid.store(true); ++pitch_frames;
    }
  });
  auto open = [&](ota::can::SocketCanBus& bus, const char* axis) {
    ota::can::SocketCanBus::Options options;
    options.iface = cfg[axis]["interface"].as<std::string>();
    options.bitrate = bitrate; options.install_filters = false;
    const auto parent = std::filesystem::canonical("/sys/class/net/" + options.iface + "/device").filename().string();
    if (parent != cfg[axis]["spi_parent"].as<std::string>()) throw std::runtime_error("wrong SPI parent for " + options.iface);
    std::string error;
    if (!bus.open(options, error) || !bus.is_up() || bus.bitrate() != bitrate || !bus.start_rx(error))
      throw std::runtime_error("open " + options.iface + ": " + error);
  };
  open(yaw, "yaw"); open(pitch, "pitch");
  auto request = ota::cybergear::make_discovery_request(0, static_cast<uint8_t>(pitch_id));
  std::string error;
  if (!pitch.send(request.id, request.data, &error)) throw std::runtime_error(error);
  request = ota::cybergear::make_read_reg(ota::cybergear::Reg::MechPos, 0, static_cast<uint8_t>(pitch_id));
  // Allow the single-response device to answer discovery first.
  std::this_thread::sleep_for(50ms);
  if (!pitch.send(request.id, request.data, &error)) throw std::runtime_error(error);
  std::this_thread::sleep_for(450ms);
  auto read = [&] { std::lock_guard lock(sample_mutex); return sample; };
  const auto baseline = read();
  if (uid.load() != expected_uid || !pitch_position_valid.load() || !baseline.valid || baseline.count < 100 ||
      (ota::now_monotonic_ns() - baseline.feedback.rx_ns) * 1e-6 > age_limit ||
      std::abs(baseline.feedback.speed_rad_s() * degrees) > 1.0)
    throw std::runtime_error("identity/freshness/stationary baseline failed before any yaw output");
  std::cout << "IDENTIFIED yaw_standard_id=0x" << std::hex << 0x204 + yaw_id
            << " pitch_uid=0x" << uid.load() << std::dec
            << " pitch_mech_pos_rad=" << pitch_position.load() << " yaw_samples=" << baseline.count << std::endl;
  const auto zero = ota::gm6020::voltage_frame(static_cast<uint8_t>(yaw_id), 0);
  const auto command = ota::gm6020::voltage_frame(static_cast<uint8_t>(yaw_id), voltage);
  std::atomic<ota::TimeNs> heartbeat{ota::now_monotonic_ns()};
  std::atomic<bool> trip{false}, zero_failed{false};
  std::mutex command_mutex;
  // Independent of the probe loop, but not independent of this process/OS.
  std::jthread guard;
  if (voltage != 0) guard = std::jthread([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      {
        std::lock_guard lock(command_mutex);
        if (interrupted || (ota::now_monotonic_ns() - heartbeat.load()) * 1e-6 > heartbeat_limit) trip.store(true);
        if (trip.load() && !yaw.send_frame(zero)) zero_failed.store(true);
      }
      std::this_thread::sleep_for(5ms);
    }
    // Also run on exceptional scope exit, while the transport is still alive.
    for (int i = 0; i < 20; ++i) {
      if (!yaw.send_frame(zero)) zero_failed.store(true);
      std::this_thread::sleep_for(5ms);
    }
  });
  // After this point, no throwing work until the zero-output cleanup completes.
  const auto start = ota::now_monotonic_ns();
  auto next = std::chrono::steady_clock::now();
  double peak_speed = 0, peak_travel = 0;
  bool stopped = false;
  ota::TimeNs still_since = 0;
  const char* reason = "completed";
  for (;;) {
    const auto now = ota::now_monotonic_ns();
    const auto elapsed_ms = (now - start) * 1e-6;
    const auto current = read();
    const double age_ms = (now - current.feedback.rx_ns) * 1e-6;
    const double speed = std::abs(current.feedback.speed_rad_s() * degrees);
    const double travel = std::abs(current.position - baseline.position) * degrees;
    peak_speed = std::max(peak_speed, speed); peak_travel = std::max(peak_travel, travel);
    if (interrupted) { trip.store(true); reason = "interrupted"; }
    if (!current.valid || age_ms > age_limit) { trip.store(true); reason = "feedback_invalid_or_stale"; }
    if (speed > speed_limit || travel > travel_limit) { trip.store(true); reason = "speed_or_travel_guard"; }
    if (trip.load() && std::string_view(reason) == "completed") reason = "heartbeat_guard";
    bool active = false;
    heartbeat.store(now);
    {
      std::lock_guard lock(command_mutex);
      active = voltage != 0 && elapsed_ms < pulse_ms && !trip.load();
      if (voltage != 0 && !yaw.send_frame(active ? command : zero)) { trip.store(true); reason = "tx_failed"; }
    }
    if (trace) trace << now << ',' << (active ? "pulse" : "observe") << ',' << (active ? voltage : 0)
                     << ',' << current.feedback.angle_count << ',' << current.position * degrees << ','
                     << current.feedback.speed_rad_s() * degrees << ',' << current.feedback.current_raw << ','
                     << int(current.feedback.temperature_raw) << ',' << age_ms << '\n';
    if (!active && current.valid && age_ms <= age_limit && speed <= 1.0) {
      if (!still_since) still_since = now;
      stopped = now - still_since >= 250000000;
    } else { still_since = 0; stopped = false; }
    if (elapsed_ms >= pulse_ms + observe_ms || (trip.load() && stopped)) break;
    next += 5ms; std::this_thread::sleep_until(next);
  }
  if (voltage != 0) {
    for (int i = 0; i < 20; ++i) {
      if (!yaw.send_frame(zero)) zero_failed.store(true);
      std::this_thread::sleep_for(5ms);
    }
  }
  guard.request_stop(); if (guard.joinable()) guard.join();
  const auto final = read();
  const auto yaw_stats = yaw.stats(); const auto pitch_stats = pitch.stats();
  yaw.close(); pitch.close();
  std::cout << "RESULT reason=" << reason << " yaw_voltage=" << voltage
            << " peak_speed_deg_s=" << peak_speed << " peak_travel_deg=" << peak_travel
            << " final_displacement_deg=" << (final.position - baseline.position) * degrees
            << " yaw_frames=" << final.count << " yaw_tx=" << yaw_stats.tx_frames
            << " pitch_replies=" << pitch_frames.load() << " pitch_tx=" << pitch_stats.tx_frames
            << " stationary_observed=" << stopped << " zero_tx_failed=" << zero_failed.load()
            << " yaw_errors=" << yaw_stats.rx_error_frames << " pitch_errors=" << pitch_stats.rx_error_frames
            << std::endl;
  // This is deliberately not the production PARKED/de-energized marker.
  std::cout << "COMMISSIONING FINISHED; " << (voltage ? "zero output requested; disabled state unavailable" : "receive/discovery only") << std::endl;
  return trip.load() || !stopped || zero_failed.load() ? 2 : 0;
 } catch (const std::exception& error) {
   std::cerr << "Probe refused: " << error.what() << std::endl; return 1;
 }
}
