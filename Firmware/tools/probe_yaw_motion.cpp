// Continuous GM6020 yaw round-trip commissioning session.
// Session-relative motion only: no absolute heading claim, zero, homing, or persistence.
#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "can/gm6020_protocol.hpp"
#include "can/gm6020_velocity.hpp"
#include "can/socketcan_bus.hpp"

using namespace std::chrono_literals;
namespace {
constexpr double kRad = std::numbers::pi / 180.0;
constexpr double kDeg = 1.0 / kRad;
constexpr int kMotorId = 1;
constexpr ota::TimeNs kFeedbackAgeLimit = 100'000'000;
constexpr ota::TimeNs kHeartbeatLimit = 100'000'000;
constexpr double kSpeedLimitDegS = 25.0;
constexpr double kMaxOutputRaw = 3000.0;
constexpr double kMaxReferenceDegS = 15.0;
constexpr double kVelocityKp = 4500.0;
constexpr double kVelocityKi = 250.0;
constexpr double kMaxReferenceAccelerationDegS2 = 30.0;
constexpr uint8_t kTemperatureLimitC = 70;
constexpr ota::TimeNs kMotionLimit = 15'000'000'000;
constexpr double kTargetToleranceDeg = 0.7;
volatile std::sig_atomic_t interrupted = 0;
void signal_stop(int) { interrupted = 1; }

struct Ownership {
  int fd{-1};
  Ownership() {
    const auto path = "/tmp/ota-mixed-can-" + std::to_string(getuid()) + ".lock";
    fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("cannot open mixed-CAN ownership lock");
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("another station process owns the CAN buses");
    }
  }
  ~Ownership() { if (fd >= 0) ::close(fd); }
};

struct Sample {
  ota::gm6020::Feedback feedback{};
  double position{};
  uint64_t count{};
  bool valid{};
};

int parse_target(const char* text) {
  int target{};
  const std::string value(text);
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), target);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || target < 15 || target > 45)
    throw std::runtime_error("TARGET_DEG must be an integer from 15 through 45");
  return target;
}
}  // namespace

int main(int argc, char** argv) {
  ota::can::SocketCanBus yaw;
  bool opened = false;
  try {
    if (argc != 3) {
      std::cerr << "Usage: probe-yaw-motion TARGET_DEG TRACE.csv (15..45)\n";
      return 2;
    }
    const int target_deg = parse_target(argv[1]);
    std::signal(SIGINT, signal_stop);
    std::signal(SIGTERM, signal_stop);
    Ownership ownership;
    if (std::filesystem::canonical("/sys/class/net/can0/device").filename() != "spi0.0")
      throw std::runtime_error("can0 SPI parent mismatch (expected spi0.0)");
    std::ofstream trace(argv[2]);
    if (!trace) throw std::runtime_error("cannot open trace file");
    trace << "time_ns,phase,voltage_raw,angle_count,yaw_relative_deg,target_relative_deg,speed_deg_s,current_raw,temperature_raw,feedback_age_ms,estimated_speed_deg_s\n";

    std::mutex sample_mutex;
    Sample sample;
    ota::gm6020::UnwrappedEncoder encoder;
    yaw.set_frame_callback([&](const ota::can::RawFrame& frame) {
      ota::gm6020::Feedback feedback;
      if (!ota::gm6020::decode(frame, kMotorId, feedback)) return;
      std::lock_guard lock(sample_mutex);
      sample.feedback = feedback;
      sample.valid = encoder.update(feedback.angle_count, feedback.rx_ns);
      sample.position = encoder.relative_rad();
      ++sample.count;
    });

    ota::can::SocketCanBus::Options options;
    options.iface = "can0";
    options.bitrate = 1'000'000;
    options.bring_up_if_down = false;
    options.install_filters = false;
    options.receive_error_frames = true;
    std::string error;
    if (!yaw.open(options, error) || !yaw.is_up() || yaw.bitrate() != 1'000'000 ||
        yaw.can_state() != ota::can::CanIfState::ErrorActive || !yaw.start_rx(error))
      throw std::runtime_error("can0 must already be UP at 1 Mbps and ERROR-ACTIVE: " + error);
    opened = true;
    if (!yaw.refresh_health(&error) || !yaw.is_up() || yaw.bitrate() != 1'000'000 ||
        yaw.can_state() != ota::can::CanIfState::ErrorActive)
      throw std::runtime_error("can0 health check failed: " + error);

    const auto zero = ota::gm6020::voltage_frame(kMotorId, 0);
    // Establish a bounded startup stop request before accepting a baseline.
    // GM6020 voltage mode has no verified disable-state feedback.
    for (int i = 0; i < 20; ++i) {
      if (!yaw.send_frame(zero, &error)) throw std::runtime_error("startup zero command failed: " + error);
      std::this_thread::sleep_for(5ms);
    }
    auto read = [&] { std::lock_guard lock(sample_mutex); return sample; };
    auto next = std::chrono::steady_clock::now() + 500ms;
    while (std::chrono::steady_clock::now() < next && !interrupted) std::this_thread::sleep_for(2ms);
    const auto baseline = read();
    const auto baseline_age = ota::now_monotonic_ns() - baseline.feedback.rx_ns;
    if (interrupted || !baseline.valid || baseline.count < 50 || std::abs(baseline.position) > 0.5 * kRad || baseline_age < 0 ||
        baseline_age > kFeedbackAgeLimit || baseline.feedback.speed_rpm != 0 ||
        baseline.feedback.temperature_raw >= kTemperatureLimitC ||
        yaw.stats().rx_error_frames != 0)
      throw std::runtime_error("fresh stationary yaw feedback and clean CAN are required before motion");

    ota::gm6020::VelocityLoop velocity;
    const auto armed_at = ota::now_monotonic_ns();
    velocity.reset(baseline.position, armed_at);
    std::mutex command_mutex;
    std::atomic<ota::TimeNs> heartbeat{armed_at};
    std::atomic<bool> trip{false}, zero_failed{false};
    std::atomic<int> trip_reason{0};
    std::atomic<double> target_position{baseline.position + target_deg * kRad};
    std::atomic<double> estimate_deg_s{0.0};
    std::atomic<double> peak_excursion_deg{0.0}, peak_speed_deg_s{0.0}, outbound_position_deg{0.0};
    const auto session_start = armed_at;

    // Process-local guard serializes stop frames with the control stream.
    // It is retained for the entire powered session and cannot survive Pi/process loss.
    std::jthread guard([&](std::stop_token stop) {
      auto last_position = baseline.position;
      auto last_position_ns = baseline.feedback.rx_ns;
      double measured_speed = 0;
      while (!stop.stop_requested()) {
        const auto now = ota::now_monotonic_ns();
        const auto current = read();
        int reason = 0;
        const auto age = now - current.feedback.rx_ns;
        if (interrupted) reason = 1;
        else if (now - session_start > kMotionLimit) reason = 2;
        else if (now - heartbeat.load() > kHeartbeatLimit) reason = 3;
        else if (!current.valid || age < 0 || age > kFeedbackAgeLimit) reason = 4;
        else if (std::abs(current.position - baseline.position) * kDeg > target_deg + 8.0) reason = 5;
        else if (yaw.stats().rx_error_frames != 0 || yaw.stats().tx_failed != 0) reason = 6;
        else if (current.feedback.temperature_raw >= kTemperatureLimitC) reason = 10;
        if (current.valid && current.feedback.rx_ns - last_position_ns >= 50'000'000) {
          measured_speed = (current.position - last_position) /
              ((current.feedback.rx_ns - last_position_ns) * 1e-9);
          last_position = current.position;
          last_position_ns = current.feedback.rx_ns;
        }
        estimate_deg_s.store(measured_speed * kDeg);
        peak_speed_deg_s.store(std::max(peak_speed_deg_s.load(), std::abs(measured_speed * kDeg)));
        peak_excursion_deg.store(std::max(peak_excursion_deg.load(),
                                          std::abs(current.position - baseline.position) * kDeg));
        if (!std::isfinite(measured_speed) || std::abs(measured_speed * kDeg) > kSpeedLimitDegS) reason = 7;
        if (reason && !trip.load()) { trip_reason.store(reason); trip.store(true); }
        if (trip.load()) {
          std::lock_guard lock(command_mutex);
          if (!yaw.send_frame(zero)) zero_failed.store(true);
        }
        std::this_thread::sleep_for(5ms);
      }
      for (int i = 0; i < 20; ++i) {
        std::lock_guard lock(command_mutex);
        if (!yaw.send_frame(zero)) zero_failed.store(true);
        std::this_thread::sleep_for(5ms);
      }
    });

    std::cout << "YAW_SESSION id=" << kMotorId << " target_deg=" << target_deg
              << " speed_limit_deg_s=" << kMaxReferenceDegS
              << " output_ceiling_raw=" << kMaxOutputRaw
              << " excursion_guard_deg=" << target_deg + 8
              << " continuous_session=1\n" << std::flush;

    int voltage = 0;
    bool returning = false;
    ota::TimeNs settled_since = 0;
    ota::TimeNs outward_settled_since = 0;
    bool stationary = false;
    const char* phase = "outbound";
    double shaped_reference_rad_s = 0;
    auto reference_update_ns = armed_at;
    auto tick = std::chrono::steady_clock::now();
    while (!trip.load() && !stationary) {
      const auto now = ota::now_monotonic_ns();
      const auto current = read();
      const auto age = now - current.feedback.rx_ns;
      if (interrupted || !current.valid || age < 0 || age > kFeedbackAgeLimit ||
          yaw.stats().rx_error_frames != 0 || yaw.stats().tx_failed != 0) {
        trip_reason.store(interrupted ? 1 : 4); trip.store(true); break;
      }
      const double position = current.position;
      double requested_velocity = 0;
      double target = target_position.load();
      const double position_error = target - position;
      const double speed = velocity.velocity_rad_s() * kDeg;
      if (!returning) {
        if (std::abs(position_error) <= kTargetToleranceDeg * kRad && std::abs(speed) <= 1.5) {
          if (!outward_settled_since) outward_settled_since = now;
          if (now - outward_settled_since >= 250'000'000) {
            returning = true;
            phase = "return";
            outbound_position_deg.store((position - baseline.position) * kDeg);
            target_position.store(baseline.position);
            settled_since = 0;
            std::cout << "YAW_TURNAROUND position_deg=" << (position - baseline.position) * kDeg
                      << " energized_continuously=1\n" << std::flush;
            target = baseline.position;
          }
        } else outward_settled_since = 0;
      }
      const double error_deg = (target - position) * kDeg;
      if (std::abs(error_deg) > kTargetToleranceDeg)
        requested_velocity = std::clamp(error_deg * 2.0, -kMaxReferenceDegS, kMaxReferenceDegS) * kRad;
      else requested_velocity = 0;
      const double reference_dt = std::clamp((now - reference_update_ns) * 1e-9, 0.0, .020);
      const double max_reference_step = kMaxReferenceAccelerationDegS2 * kRad * reference_dt;
      shaped_reference_rad_s += std::clamp(requested_velocity - shaped_reference_rad_s,
                                            -max_reference_step, max_reference_step);
      reference_update_ns = now;
      heartbeat.store(now);
      {
        std::lock_guard lock(command_mutex);
        voltage = velocity.update(shaped_reference_rad_s, position, now,
                                  kMaxReferenceDegS * kRad, kMaxOutputRaw,
                                  kVelocityKp, kVelocityKi);
        if (!velocity.valid()) {
          trip_reason.store(8); trip.store(true); voltage = 0;
        }
        const auto command = ota::gm6020::voltage_frame(kMotorId, voltage);
        if (!trip.load() && !yaw.send_frame(command)) {
          trip_reason.store(9); trip.store(true);
        }
      }
      const double from_origin_deg = (position - baseline.position) * kDeg;
      trace << now << ',' << phase << ',' << voltage << ',' << current.feedback.angle_count << ','
            << from_origin_deg << ',' << (target - baseline.position) * kDeg << ','
            << current.feedback.speed_rad_s() * kDeg << ',' << current.feedback.current_raw << ','
            << int(current.feedback.temperature_raw) << ',' << age * 1e-6 << ','
            << estimate_deg_s.load() << '\n';
      if (returning && std::abs(from_origin_deg) <= kTargetToleranceDeg &&
          std::abs(estimate_deg_s.load()) <= 1.0 && current.feedback.speed_rpm == 0) {
        if (!settled_since) settled_since = now;
        stationary = now - settled_since >= 250'000'000;
      } else if (returning) settled_since = 0;
      tick += 5ms;
      std::this_thread::sleep_until(tick);
    }

    if (trip.load()) phase = "guard_stop";
    // Finish the session with repeated explicit zero voltage and fresh feedback.
    const auto zero_until = std::chrono::steady_clock::now() + 2s;
    auto zero_tick = std::chrono::steady_clock::now();
    ota::TimeNs final_still_since = 0;
    bool final_stationary = false;
    do {
      const auto now = ota::now_monotonic_ns();
      const auto current = read();
      const auto age = now - current.feedback.rx_ns;
      heartbeat.store(now);
      {
        std::lock_guard lock(command_mutex);
        if (!yaw.send_frame(zero)) zero_failed.store(true);
      }
      trace << now << ',' << (trip.load() ? "guard_stop" : "zero_observe") << ",0,"
            << current.feedback.angle_count << ',' << (current.position - baseline.position) * kDeg
            << ',' << (returning ? 0.0 : target_deg) << ',' << current.feedback.speed_rad_s() * kDeg
            << ',' << current.feedback.current_raw << ',' << int(current.feedback.temperature_raw) << ','
            << age * 1e-6 << ',' << estimate_deg_s.load() << '\n';
      if (current.valid && age >= 0 && age <= kFeedbackAgeLimit && current.feedback.speed_rpm == 0 &&
          std::abs(estimate_deg_s.load()) <= 1.0) {
        if (!final_still_since) final_still_since = now;
        final_stationary = now - final_still_since >= 250'000'000;
      } else { final_still_since = 0; final_stationary = false; }
      zero_tick += 5ms;
      std::this_thread::sleep_until(zero_tick);
    } while (!final_stationary && std::chrono::steady_clock::now() < zero_until);
    guard.request_stop();
    if (guard.joinable()) guard.join();
    const auto final = read();
    const auto stats = yaw.stats();
    yaw.close(); opened = false;
    std::cout << "YAW_RESULT trip=" << trip.load() << " reason=" << trip_reason.load()
              << " outbound_return=" << (returning && stationary)
              << " outbound_position_deg=" << outbound_position_deg.load()
              << " peak_excursion_deg=" << peak_excursion_deg.load()
              << " peak_measured_speed_deg_s=" << peak_speed_deg_s.load()
              << " final_position_deg=" << (final.position - baseline.position) * kDeg
              << " final_stationary=" << final_stationary
              << " zero_tx_failed=" << zero_failed.load()
              << " rx_errors=" << stats.rx_error_frames << " tx_failed=" << stats.tx_failed
              << " max_speed_guard_deg_s=" << kSpeedLimitDegS << '\n'
              << "COMMISSIONING FINISHED; zero voltage requested; motor disable state unavailable\n";
    return trip.load() || !returning || !final_stationary || zero_failed.load() ||
           stats.rx_error_frames || stats.tx_failed ? 2 : 0;
  } catch (const std::exception& error) {
    if (opened) {
      const auto zero = ota::gm6020::voltage_frame(kMotorId, 0);
      for (int i = 0; i < 20; ++i) { yaw.send_frame(zero); std::this_thread::sleep_for(5ms); }
      yaw.close();
    }
    std::cerr << "YAW_PROBE_REFUSED " << error.what() << '\n';
    return 1;
  }
}
