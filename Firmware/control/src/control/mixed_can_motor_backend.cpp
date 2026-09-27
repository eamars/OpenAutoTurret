#include "control/mixed_can_motor_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <numbers>
#include <thread>

#include <spdlog/spdlog.h>

#include "common/time.hpp"

namespace ota {
namespace {
using namespace std::chrono_literals;
constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;
constexpr double kDegreesPerRadian = 180.0 / std::numbers::pi;
constexpr double kYawMaxSpeedRadS = 15.0 * kRadiansPerDegree;
constexpr double kYawMaxAccelerationRadS2 = 20.0 * kRadiansPerDegree;
constexpr double kYawPositionGain = 2.0;
// The first automatic sweep requested 10 deg/s but reached 26.34 deg/s and
// correctly tripped the independent 25 deg/s guard. The earlier 30-degree
// motor probe needed at most 5,643 raw voltage, so retain voltage headroom
// while reducing the service-loop drive and acceleration for this payload.
constexpr double kYawOutputCeiling = 9000.0;
constexpr double kYawVelocityKp = 20000.0;
constexpr double kYawVelocityKi = 10000.0;
constexpr TimeNs kFreshnessLimitNs = 100'000'000;
constexpr TimeNs kHeartbeatLimitNs = 100'000'000;
constexpr TimeNs kStationaryWindowNs = 500'000'000;
constexpr TimeNs kNoProgressLimitNs = 1'500'000'000;
constexpr double kStationaryToleranceRad = 0.5 * kRadiansPerDegree;
constexpr double kNoProgressCommandRadS = 5.0 * kRadiansPerDegree;
// The temperature gate has no constant: the guide gives the feedback byte no
// scale, so the profile decides (0 = no gate). See axes.yaw.guard_temp_raw_ceiling.

CanHealth socketcan_health(const can::SocketCanBus& bus) {
  const auto stats = bus.stats();
  CanHealth health;
  health.available = true;
  health.kind = bus.kind();
  health.device = bus.device();
  health.up = bus.is_up();
  health.state = static_cast<int>(bus.can_state());
  health.rx_frames = stats.rx_frames;
  health.rx_error_frames = stats.rx_error_frames;
  health.tx_frames = stats.tx_frames;
  health.tx_failed = stats.tx_failed;
  health.last_rx_ns = stats.last_rx_ns;
  return health;
}
}  // namespace

MixedCanMotorBackend::MixedCanMotorBackend() : pitch_backend_(pitch_system_) {}

MixedCanMotorBackend::~MixedCanMotorBackend() { close(); }

bool MixedCanMotorBackend::validate_profile(const config::mixed::Profile& p,
                                            std::string& err) const {
  if (p.schema_version != 1 || p.yaw_bus.interface != "can0" ||
      p.yaw_bus.spi_parent != "spi0.0" || p.yaw_bus.bitrate != 1'000'000 ||
      p.pitch_bus.interface != "can1" || p.pitch_bus.spi_parent != "spi1.0" ||
      p.pitch_bus.bitrate != 1'000'000 || p.yaw_bus.interface == p.pitch_bus.interface) {
    err = "mixed profile must describe can0/spi0.0 and can1/spi1.0 at 1 Mbps";
    return false;
  }
  if (p.yaw.protocol != config::mixed::Protocol::Gm6020 ||
      p.yaw.bus_name != "yaw" || p.yaw.motor_id != 1 ||
      p.yaw.topology != config::mixed::Topology::Continuous ||
      p.yaw.control_mode != config::mixed::ControlMode::Voltage ||
      p.yaw.feedback_frame_id != std::optional<uint32_t>{0x205} ||
      p.yaw.command_frame_id != std::optional<uint32_t>{0x1ff}) {
    err = "mixed profile yaw must be GM6020 ID 1 with continuous voltage control";
    return false;
  }
  if (p.pitch.protocol != config::mixed::Protocol::CyberGear ||
      p.pitch.bus_name != "pitch" || p.pitch.motor_id != 127 ||
      p.pitch.topology != config::mixed::Topology::Bounded ||
      (p.pitch.control_mode != config::mixed::ControlMode::Position &&
       p.pitch.control_mode != config::mixed::ControlMode::Speed) ||
      !p.pitch.expected_unique_id || *p.pitch.expected_unique_id != 0x7216313130333105ULL ||
      !p.pitch.current_limit_a || !std::isfinite(*p.pitch.current_limit_a) ||
      *p.pitch.current_limit_a <= 0 || *p.pitch.current_limit_a > 5.0) {
    err = "mixed profile pitch must be bounded CyberGear ID 127 with expected UID and <=5 A limit";
    return false;
  }
  return true;
}

bool MixedCanMotorBackend::validate_can0(std::string& err) {
  const auto parent = std::filesystem::canonical("/sys/class/net/can0/device").filename().string();
  if (parent != profile_.yaw_bus.spi_parent) {
    err = "can0 SPI parent mismatch: expected " + profile_.yaw_bus.spi_parent + ", found " + parent;
    return false;
  }
  if (!yaw_bus_.refresh_health(&err) || !yaw_bus_.is_up() ||
      yaw_bus_.bitrate() != profile_.yaw_bus.bitrate ||
      yaw_bus_.can_state() != can::CanIfState::ErrorActive) {
    err = "can0 must remain UP at 1 Mbps and ERROR-ACTIVE: " + err;
    return false;
  }
  return true;
}

bool MixedCanMotorBackend::validate_can1(std::string& err) {
  const auto parent = std::filesystem::canonical("/sys/class/net/can1/device").filename().string();
  if (parent != profile_.pitch_bus.spi_parent) {
    err = "can1 SPI parent mismatch: expected " + profile_.pitch_bus.spi_parent + ", found " + parent;
    return false;
  }
  const auto* bus = dynamic_cast<const can::SocketCanBus*>(&pitch_system_.bus());
  if (!bus || !const_cast<can::SocketCanBus*>(bus)->refresh_health(&err) || !bus->is_up() ||
      bus->bitrate() != profile_.pitch_bus.bitrate ||
      bus->can_state() != can::CanIfState::ErrorActive) {
    err = "can1 must remain UP at 1 Mbps and ERROR-ACTIVE: " + err;
    return false;
  }
  return true;
}

bool MixedCanMotorBackend::open(const config::mixed::Profile& profile,
                                std::string& err) {
  close();
  if (!validate_profile(profile, err)) return false;
  try {
    profile_ = profile;
    {
      std::lock_guard lock(yaw_mutex_);
      yaw_encoder_.reset();
      yaw_state_ = YawState{};
      yaw_origin_rad_ = 0;
      yaw_position_target_rad_ = yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
      yaw_position_mode_ = yaw_speed_mode_ = false;
    }
    yaw_reference_valid_.store(false);
    yaw_trip_.store(false);
    pitch_enabled_owned_.store(false);
    pitch_transition_active_.store(false);
    pitch_stop_ping_ns_.store(0);
    const auto yaw_parent = std::filesystem::canonical("/sys/class/net/can0/device").filename().string();
    const auto pitch_parent = std::filesystem::canonical("/sys/class/net/can1/device").filename().string();
    if (yaw_parent != profile_.yaw_bus.spi_parent || pitch_parent != profile_.pitch_bus.spi_parent) {
      err = "split-CAN SPI parent identity mismatch";
      return false;
    }
    yaw_opened_.store(true);

    yaw_bus_.set_frame_callback([this](const can::RawFrame& frame) { on_yaw_frame(frame); });
    can::SocketCanBus::Options yaw_options;
    yaw_options.iface = profile_.yaw_bus.interface;
    yaw_options.bitrate = profile_.yaw_bus.bitrate;
    yaw_options.bring_up_if_down = false;
    yaw_options.install_filters = false;
    yaw_options.receive_error_frames = true;
    if (!yaw_bus_.open(yaw_options, err) || !yaw_bus_.is_up() ||
        yaw_bus_.bitrate() != profile_.yaw_bus.bitrate ||
        yaw_bus_.can_state() != can::CanIfState::ErrorActive || !yaw_bus_.start_rx(err)) {
      if (err.empty()) err = "can0 is not ready at the required health state";
      close();
      return false;
    }

    // CyberGearSystem has two logical axes. Alias its unused internal yaw ID
    // to the real pitch ID so its legacy watchdog cannot address a fictitious
    // CyberGear on can1. This backend never issues a yaw register/UID request.
    can::CyberGearSystemConfig pitch_config;
    pitch_config.transport = "socketcan";
    pitch_config.iface = profile_.pitch_bus.interface;
    pitch_config.bitrate = profile_.pitch_bus.bitrate;
    pitch_config.bring_up_if_down = false;
    pitch_config.pitch_motor_id = profile_.pitch.motor_id;
    pitch_config.yaw_motor_id = profile_.pitch.motor_id;
    if (!pitch_system_.open(pitch_config, err)) {
      close();
      return false;
    }
    pitch_opened_.store(true);
    if (!validate_can0(err) || !validate_can1(err)) {
      close();
      return false;
    }
    bus_health_ok_.store(true);

    const auto startup_zero = gm6020::voltage_frame(profile_.yaw.motor_id, 0);
    for (int i = 0; i < 20; ++i) {
      if (!yaw_bus_.send_frame(startup_zero, &err)) {
        err = "GM6020 startup zero request failed: " + err;
        close();
        return false;
      }
      std::this_thread::sleep_for(5ms);
    }

    uint64_t pitch_uid = 0;
    if (!pitch_backend_.discover(AxisId::Pitch, pitch_uid, err) ||
        pitch_uid != *profile_.pitch.expected_unique_id) {
      if (err.empty()) err = "pitch CyberGear UID mismatch";
      else err = "pitch CyberGear discovery/UID validation failed: " + err;
      close();
      return false;
    }
    if (!validate_can0(err) || !validate_can1(err)) {
      close();
      return false;
    }
    if (yaw_bus_.stats().rx_error_frames || yaw_bus_.stats().tx_failed) {
      err = "GM6020 bus reported CAN errors during startup";
      close();
      return false;
    }
    if (!establish_yaw_reference(err)) {
      close();
      return false;
    }

    heartbeat_ns_.store(0);
    heartbeat_seen_.store(false);
    yaw_trip_.store(false);
    yaw_motion_allowed_.store(false);
    opened_.store(true);
    yaw_guard_ = std::jthread([this](std::stop_token stop) { yaw_guard_loop(stop); });
    return true;
  } catch (const std::exception& error) {
    err = error.what();
    close();
    return false;
  }
}

void MixedCanMotorBackend::close() {
  opened_.store(false);
  if (yaw_guard_.joinable()) {
    yaw_guard_.request_stop();
    yaw_guard_.join();
  }
  yaw_motion_allowed_.store(false);
  yaw_reference_valid_.store(false);
  if (yaw_opened_.load()) {
    const auto zero = gm6020::voltage_frame(profile_.yaw.motor_id ? profile_.yaw.motor_id : 1, 0);
    for (int i = 0; i < 20; ++i) {
      yaw_bus_.send_frame(zero);
      std::this_thread::sleep_for(5ms);
    }
  }
  if (pitch_opened_.load()) {
    if (pitch_enabled_owned_.exchange(false))
      pitch_backend_.deenergize(AxisId::Pitch);
    pitch_system_.close();
  pitch_opened_.store(false);
  pitch_transition_active_.store(false);
  pitch_stop_ping_ns_.store(0);
  }
  yaw_bus_.close();
  yaw_opened_.store(false);
  {
    // The guard is joined by now, but the control loop may still be reading a trip it
    // observed, so the rewrite happens under the reader's mutex.
    const std::lock_guard detail_lock(yaw_trip_detail_mutex_);
    yaw_trip_.store(false);
    yaw_trip_detail_ = {};
  }
  yaw_requested_velocity_rad_s_.store(0);
  heartbeat_seen_.store(false);
  heartbeat_ns_.store(0);
  bus_health_ok_.store(false);
}

void MixedCanMotorBackend::on_yaw_frame(const can::RawFrame& frame) {
  gm6020::Feedback decoded;
  if (!gm6020::decode(frame, profile_.yaw.motor_id ? profile_.yaw.motor_id : 1, decoded)) return;
  std::lock_guard lock(yaw_mutex_);
  const auto previous_rx_ns = yaw_state_.feedback.rx_ns;
  const auto previous_count = yaw_state_.feedback.angle_count;
  const bool encoder_was_valid = yaw_state_.encoder_valid;
  yaw_state_.feedback = decoded;
  yaw_state_.received = true;
  yaw_state_.encoder_valid = yaw_encoder_.update(decoded.angle_count, decoded.rx_ns);
  if (encoder_was_valid && !yaw_state_.encoder_valid)
    spdlog::error("GM6020 encoder invalidated: dt_ms={:.3f} previous_count={} count={} speed_rpm={}",
                  (decoded.rx_ns - previous_rx_ns) / 1e6,
                  previous_count, decoded.angle_count, decoded.speed_rpm);
  yaw_state_.position_rad = yaw_encoder_.relative_rad() - yaw_origin_rad_;
  ++yaw_state_.count;
  if (!yaw_reference_valid_.load() && yaw_encoder_.valid()) {
    // Provisional origin permits stationary assessment. The committed origin
    // is refreshed after the bus/pitch UID checks complete.
    yaw_origin_rad_ = 0;
  }
}

bool MixedCanMotorBackend::establish_yaw_reference(std::string& err) {
  const auto started = now_monotonic_ns();
  TimeNs still_since = 0;
  double first_position = 0;
  bool have_first = false;
  const auto deadline = started + 2'000'000'000LL;
  while (now_monotonic_ns() < deadline) {
    const auto now = now_monotonic_ns();
    bool stationary = false;
    {
      std::lock_guard lock(yaw_mutex_);
      if (yaw_state_.received && yaw_state_.encoder_valid) {
        if (!have_first) { first_position = yaw_encoder_.relative_rad(); have_first = true; }
        const auto age = now - yaw_state_.feedback.rx_ns;
        stationary = age >= 0 && age <= kFreshnessLimitNs && yaw_state_.count >= 50 &&
            yaw_state_.feedback.speed_rpm == 0 &&
            std::abs(yaw_encoder_.relative_rad() - first_position) <= kStationaryToleranceRad;
      }
    }
    if (stationary) {
      if (!still_since) still_since = now;
      if (now - still_since >= kStationaryWindowNs) {
        std::lock_guard lock(yaw_mutex_);
        yaw_origin_rad_ = yaw_encoder_.relative_rad();
        yaw_state_.position_rad = 0;
        yaw_velocity_loop_.reset(0, now);
        yaw_velocity_loop_previous_command_ns_ = now;
        yaw_requested_velocity_rad_s_.store(0);
        yaw_position_target_rad_ = 0;
        yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
        yaw_reference_valid_.store(true);
        return true;
      }
    } else {
      still_since = 0;
    }
    const auto yaw_health = socketcan_health(yaw_bus_);
    if (!yaw_health.up || yaw_health.state != static_cast<int>(can::CanIfState::ErrorActive) ||
        yaw_health.rx_error_frames || yaw_health.tx_failed) {
      err = "GM6020 bus lost healthy state while establishing stationary session reference";
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  err = "GM6020 did not provide a fresh stationary baseline for the session reference";
  return false;
}

bool MixedCanMotorBackend::yaw_feedback_safe_locked(TimeNs now) const {
  if (!yaw_reference_valid_.load() || !yaw_state_.received || !yaw_state_.encoder_valid) return false;
  const auto age = now - yaw_state_.feedback.rx_ns;
  // A CAN receive callback can publish a frame after the caller sampled its
  // cycle timestamp but before it acquired yaw_mutex_. Allow only that small
  // clock-order race; the exported snapshot timestamp is clamped below.
  return age >= -5'000'000LL && age <= kFreshnessLimitNs &&
      std::isfinite(yaw_state_.position_rad) &&
      std::isfinite(yaw_state_.feedback.speed_rad_s());
}

AxisSnapshot MixedCanMotorBackend::yaw_snapshot_locked(TimeNs now) const {
  AxisSnapshot snapshot;
  snapshot.temperature_known = false;
  snapshot.temperature_raw_valid = yaw_state_.received;
  snapshot.temperature_raw = yaw_state_.feedback.temperature_raw;
  snapshot.temp_c = std::numeric_limits<double>::quiet_NaN();
  snapshot.faults_known = false;
  snapshot.disabled_known = false;
  snapshot.disabled = false;
  snapshot.in_position_mode = yaw_position_mode_;
  snapshot.in_speed_mode = yaw_speed_mode_;
  if (yaw_feedback_safe_locked(now)) {
    snapshot.has_feedback = true;
    snapshot.rx_ns = std::min(yaw_state_.feedback.rx_ns, now);
    snapshot.q_rad = yaw_state_.position_rad;
    snapshot.v_rad_s = yaw_state_.feedback.speed_rad_s();
    snapshot.torque_nm = std::numeric_limits<double>::quiet_NaN();
  } else {
    snapshot.rx_ns = yaw_state_.received ? std::min(yaw_state_.feedback.rx_ns, now) : 0;
    snapshot.q_rad = std::numeric_limits<double>::quiet_NaN();
    snapshot.v_rad_s = std::numeric_limits<double>::quiet_NaN();
    snapshot.torque_nm = std::numeric_limits<double>::quiet_NaN();
  }
  return snapshot;
}

void MixedCanMotorBackend::trip_yaw_locked() {
  yaw_trip_.store(true);
  yaw_motion_allowed_.store(false);
  yaw_position_mode_ = yaw_speed_mode_ = false;
  yaw_position_target_rad_ = yaw_state_.position_rad;
  yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
  yaw_requested_velocity_rad_s_.store(0);
  yaw_velocity_loop_.reset(yaw_state_.position_rad, now_monotonic_ns());
  send_yaw_zero_locked();
}

bool MixedCanMotorBackend::send_yaw_zero_locked() {
  const auto id = profile_.yaw.motor_id ? profile_.yaw.motor_id : 1;
  const auto zero = gm6020::voltage_frame(id, 0);
  return yaw_bus_.send_frame(zero);
}

void MixedCanMotorBackend::yaw_guard_loop(std::stop_token stop) {
  double prior_position = 0;
  TimeNs prior_sample_ns = 0;
  double measured_speed = 0;
  double progress_position = 0;
  TimeNs progress_at = now_monotonic_ns();
  TimeNs last_bus_health_check = 0;
  while (!stop.stop_requested()) {
    const auto poll_now = now_monotonic_ns();
    if (poll_now - last_bus_health_check >= 500'000'000LL) {
      std::string health_error;
      bool healthy = yaw_bus_.refresh_health(&health_error) && yaw_bus_.is_up() &&
          yaw_bus_.bitrate() == profile_.yaw_bus.bitrate &&
          yaw_bus_.can_state() == can::CanIfState::ErrorActive;
      if (healthy && pitch_opened_.load()) {
        const auto* bus = dynamic_cast<const can::SocketCanBus*>(&pitch_system_.bus());
        healthy = bus && const_cast<can::SocketCanBus*>(bus)->refresh_health(&health_error) && bus->is_up() &&
            bus->bitrate() == profile_.pitch_bus.bitrate &&
            bus->can_state() == can::CanIfState::ErrorActive;
      } else healthy = false;
      bus_health_ok_.store(healthy);
      last_bus_health_check = poll_now;
    }
    bool should_stop = false;
    {
      std::lock_guard lock(yaw_mutex_);
      const auto now = now_monotonic_ns();
      const auto health = socketcan_health(yaw_bus_);
      if (yaw_state_.received && yaw_state_.encoder_valid &&
          yaw_state_.feedback.rx_ns - prior_sample_ns >= 50'000'000) {
        if (prior_sample_ns > 0) {
          measured_speed = (yaw_state_.position_rad - prior_position) /
              ((yaw_state_.feedback.rx_ns - prior_sample_ns) * 1e-9);
        }
        prior_position = yaw_state_.position_rad;
        prior_sample_ns = yaw_state_.feedback.rx_ns;
        if (std::abs(yaw_state_.position_rad - progress_position) >=
            3.0 * gm6020::UnwrappedEncoder::kRadiansPerCount) {
          progress_position = yaw_state_.position_rad;
          progress_at = now;
        }
      }
      // A sector turnaround intentionally dwells at zero speed. Start the
      // no-progress window with the next substantial request, rather than
      // charging that stationary dwell against the new reverse command.
      const double requested_speed = yaw_requested_velocity_rad_s_.load();
      if (std::abs(requested_speed) < kNoProgressCommandRadS) {
        progress_position = yaw_state_.position_rad;
        progress_at = now;
      }
      const int yaw_temp_guard = profile_.yaw.yaw_guard_temp_raw_ceiling;
      should_stop = !yaw_feedback_safe_locked(now) || !health.up ||
          health.state != static_cast<int>(can::CanIfState::ErrorActive) ||
          health.rx_error_frames != 0 || health.tx_failed != 0 ||
          !bus_health_ok_.load() ||
          !std::isfinite(measured_speed) || std::abs(measured_speed) > 25.0 * kRadiansPerDegree ||
          (yaw_temp_guard > 0 &&
           yaw_state_.feedback.temperature_raw >= yaw_temp_guard) ||
          (std::abs(requested_speed) >= kNoProgressCommandRadS &&
           now - progress_at > kNoProgressLimitNs) ||
          (heartbeat_seen_.load() && now - heartbeat_ns_.load() > kHeartbeatLimitNs);
      if (should_stop && !yaw_trip_.load()) {
        // Capture *why* the guard latched, as a machine-readable token plus the
        // field matrix, before the trip is published. Without this the operator
        // sees one string shared by ten causes — the 2026-09-27 yaw trip looked
        // identical whether the cause was CAN, the encoder, or the raw temp byte.
        {
          MotorBackend::TripInputs in;
          in.feedback_unsafe = !yaw_feedback_safe_locked(now);
          in.can_down = !health.up;
          in.can_state_wrong = health.state != static_cast<int>(can::CanIfState::ErrorActive);
          in.can_counters_bad = health.rx_error_frames != 0 || health.tx_failed != 0;
          in.bus_unhealthy = !bus_health_ok_.load();
          in.speed_not_finite = !std::isfinite(measured_speed);
          in.speed_over_ceiling = std::abs(measured_speed) > 25.0 * kRadiansPerDegree;
          in.temp_raw_over = yaw_temp_guard > 0 &&
                             yaw_state_.feedback.temperature_raw >= yaw_temp_guard;
          in.no_progress = std::abs(requested_speed) >= kNoProgressCommandRadS &&
                           now - progress_at > kNoProgressLimitNs;
          in.heartbeat_stale = heartbeat_seen_.load() &&
                               now - heartbeat_ns_.load() > kHeartbeatLimitNs;
          in.reference_valid = yaw_reference_valid_.load();
          in.feedback_age_ms = (now - yaw_state_.feedback.rx_ns) / 1e6;
          in.temp_raw = yaw_state_.feedback.temperature_raw;
          in.speed_deg_s = measured_speed * kDegreesPerRadian;
          MotorBackend::TripDetail td{};
          MotorBackend::format_trip_detail(in, MotorBackend::select_trip_condition(in), td);
          const std::lock_guard detail_lock(yaw_trip_detail_mutex_);
          yaw_trip_detail_ = td;
        }
        spdlog::error("GM6020 guard trip: feedback_safe={} reference_valid={} received={} encoder_valid={} feedback_age_ms={:.3f} can_up={} can_state={} rxerr={} txfail={} both_buses_healthy={} measured_speed_deg_s={:.3f} temp_raw={} requested_speed_deg_s={:.3f} no_progress_ms={} heartbeat_seen={} heartbeat_age_ms={}",
                      yaw_feedback_safe_locked(now), yaw_reference_valid_.load(),
                      yaw_state_.received, yaw_state_.encoder_valid,
                      (now - yaw_state_.feedback.rx_ns) / 1e6, health.up, health.state,
                      health.rx_error_frames, health.tx_failed,
                      bus_health_ok_.load(), measured_speed * kDegreesPerRadian,
                      yaw_state_.feedback.temperature_raw,
                      yaw_requested_velocity_rad_s_.load() * kDegreesPerRadian,
                      (now - progress_at) / 1'000'000,
                      heartbeat_seen_.load(),
                      heartbeat_seen_.load() ? (now - heartbeat_ns_.load()) / 1'000'000 : 0);
        trip_yaw_locked();
      }
      if (yaw_trip_.load()) send_yaw_zero_locked();
    }
    std::this_thread::sleep_for(5ms);
  }
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard lock(yaw_mutex_);
      send_yaw_zero_locked();
    }
    std::this_thread::sleep_for(5ms);
  }
}

bool MixedCanMotorBackend::buses_healthy() const {
  if (!bus_health_ok_.load()) return false;
  const auto health = can_health_all();
  if (health.size() != 2) return false;
  for (const auto& bus : health) {
    if (!bus.available || !bus.up || bus.state != static_cast<int>(can::CanIfState::ErrorActive) ||
        bus.rx_error_frames != 0 || bus.tx_failed != 0) return false;
  }
  return true;
}

std::vector<CanHealth> MixedCanMotorBackend::can_health_all() const {
  return {yaw_opened_.load() ? socketcan_health(yaw_bus_) : CanHealth{},
          pitch_opened_.load() ? pitch_backend_.can_health() : CanHealth{}};
}

CanHealth MixedCanMotorBackend::can_health() const {
  return yaw_opened_.load() ? socketcan_health(yaw_bus_) : CanHealth{};
}

void MixedCanMotorBackend::start_watchdog() {
  if (pitch_opened_.load()) pitch_system_.start_watchdog();
}

void MixedCanMotorBackend::heartbeat() {
  const auto now = now_monotonic_ns();
  heartbeat_ns_.store(now);
  heartbeat_seen_.store(true);
  if (pitch_opened_.load()) {
    pitch_backend_.heartbeat();
    // A disabled CyberGear sends no periodic status. Until a mode transition
    // starts, an idempotent STOP at 50 Hz supplies fresh disabled feedback for
    // the homing supervisor without enabling or moving the pitch axis.
    if (!pitch_enabled_owned_.load() && !pitch_transition_active_.load() &&
        now - pitch_stop_ping_ns_.load() >= 20'000'000LL) {
      pitch_stop_ping_ns_.store(now);
      std::string error;
      if (!pitch_system_.send_stop(AxisId::Pitch, &error))
        spdlog::error("CyberGear disabled-status STOP request failed: {}", error);
    }
  }
}

bool MixedCanMotorBackend::watchdog_fault() const {
  return yaw_trip_.load() || (pitch_opened_.load() && pitch_backend_.watchdog_fault());
}

MotorBackend::TripDetail MixedCanMotorBackend::watchdog_trip_detail() const {
  // The guard fills the detail under yaw_trip_detail_mutex_ and only then publishes
  // yaw_trip_, so there is nothing to read until the flag says there is. Reading it
  // still takes that mutex: close() and the next trip both rewrite the detail, and a
  // flag check alone does not make a struct copy atomic. Untripped callers — every
  // healthy 200 Hz cycle — pay one atomic load and no lock.
  if (!yaw_trip_.load()) return TripDetail{};
  const std::lock_guard detail_lock(yaw_trip_detail_mutex_);
  return yaw_trip_.load() ? yaw_trip_detail_ : TripDetail{};
}

bool MixedCanMotorBackend::discover(AxisId axis, uint64_t& unique_id,
                                    std::string& err) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) return pitch_backend_.discover(axis, unique_id, err);
    unique_id = 0;
    err = "mixed backend is not open";
    return false;
  }
  unique_id = 0;
  err = "GM6020 yaw has no CyberGear UID discovery protocol";
  return false;
}

bool MixedCanMotorBackend::read_register(AxisId axis, cybergear::Reg reg,
                                         double& value, int timeout_ms,
                                         std::string& err) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) return pitch_backend_.read_register(axis, reg, value, timeout_ms, err);
    err = "mixed backend is not open";
    return false;
  }
  (void)reg; (void)value; (void)timeout_ms;
  err = "GM6020 yaw has no CyberGear register protocol";
  return false;
}

bool MixedCanMotorBackend::enter_position_mode(AxisId axis, double limit_spd_rad_s,
                                               std::string& err) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) {
      pitch_transition_active_.store(true);
      const bool ok = pitch_backend_.enter_position_mode(axis, limit_spd_rad_s, err);
      if (ok) pitch_enabled_owned_.store(true);
      pitch_transition_active_.store(false);
      return ok;
    }
    err = "mixed backend is not open";
    return false;
  }
  return transition_mode(axis, true, limit_spd_rad_s, now_monotonic_ns(), err) == Transition::Complete;
}

bool MixedCanMotorBackend::enter_speed_mode(AxisId axis, double limit_cur_a,
                                            std::string& err) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) {
      pitch_transition_active_.store(true);
      const bool ok = pitch_backend_.enter_speed_mode(axis, limit_cur_a, err);
      if (ok) pitch_enabled_owned_.store(true);
      pitch_transition_active_.store(false);
      return ok;
    }
    err = "mixed backend is not open";
    return false;
  }
  // This is a host-side velocity controller, not a GM current-limit setting.
  (void)limit_cur_a;
  return transition_mode(axis, false, 0, now_monotonic_ns(), err) == Transition::Complete;
}

MotorBackend::Transition MixedCanMotorBackend::transition_mode(
    AxisId axis, bool position, double limit, TimeNs now, std::string& err,
    double speed_ki, double speed_kp, bool check_displacement) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) {
      pitch_transition_active_.store(true);
      const auto result = pitch_backend_.transition_mode(axis, position, limit, now, err,
                                                         speed_ki, speed_kp, check_displacement);
      if (result == Transition::Complete) pitch_enabled_owned_.store(true);
      if (result != Transition::Pending) pitch_transition_active_.store(false);
      return result;
    }
    err = "mixed backend is not open";
    return Transition::Failed;
  }
  (void)speed_ki; (void)speed_kp; (void)check_displacement;
  std::lock_guard lock(yaw_mutex_);
  if (!std::isfinite(limit) || limit < 0 || !yaw_feedback_safe_locked(now) ||
      !buses_healthy() || yaw_trip_.load()) {
    err = "GM6020 yaw position control requires a fresh session reference and healthy buses";
    return Transition::Failed;
  }
  yaw_velocity_loop_.reset(yaw_state_.position_rad, now);
  yaw_velocity_loop_previous_command_ns_ = now;
  yaw_position_target_rad_ = yaw_state_.position_rad;
  yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
  yaw_requested_velocity_rad_s_.store(0);
  yaw_position_mode_ = position;
  yaw_speed_mode_ = !position;
  yaw_motion_allowed_.store(true);
  return Transition::Complete;
}

void MixedCanMotorBackend::deenergize(AxisId axis) {
  if (axis == AxisId::Pitch) {
    pitch_transition_active_.store(false);
    if (pitch_opened_.load()) pitch_backend_.deenergize(axis);
    pitch_enabled_owned_.store(false);
    return;
  }
  std::lock_guard lock(yaw_mutex_);
  yaw_motion_allowed_.store(false);
  yaw_position_mode_ = yaw_speed_mode_ = false;
  yaw_position_target_rad_ = yaw_state_.position_rad;
  yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
  yaw_requested_velocity_rad_s_.store(0);
  send_yaw_zero_locked();
}

AxisSnapshot MixedCanMotorBackend::snapshot(AxisId axis, TimeNs now) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) return pitch_backend_.snapshot(axis, now);
    return {};
  }
  std::lock_guard lock(yaw_mutex_);
  return yaw_snapshot_locked(now);
}

void MixedCanMotorBackend::command_yaw_velocity_locked(double desired, TimeNs now) {
  if (!std::isfinite(desired) || !yaw_feedback_safe_locked(now) || yaw_trip_.load() ||
      !yaw_motion_allowed_.load() || !heartbeat_seen_.load() ||
      now - heartbeat_ns_.load() > kHeartbeatLimitNs || !buses_healthy()) {
    if (!yaw_trip_.load() && (!yaw_feedback_safe_locked(now) || !buses_healthy())) trip_yaw_locked();
    else send_yaw_zero_locked();
    return;
  }
  const double dt = std::clamp((now - yaw_velocity_loop_previous_command_ns_) * 1e-9, 0.0, .020);
  const double step = kYawMaxAccelerationRadS2 * dt;
  yaw_shaped_speed_rad_s_ += std::clamp(desired - yaw_shaped_speed_rad_s_, -step, step);
  yaw_requested_velocity_rad_s_.store(desired);
  yaw_velocity_loop_previous_command_ns_ = now;
  const int voltage = yaw_velocity_loop_.update(yaw_shaped_speed_rad_s_, yaw_state_.position_rad, now,
      kYawMaxSpeedRadS, kYawOutputCeiling, kYawVelocityKp, kYawVelocityKi);
  if (!yaw_velocity_loop_.valid()) {
    trip_yaw_locked();
    return;
  }
  const auto command = gm6020::voltage_frame(profile_.yaw.motor_id, voltage);
  if (!yaw_bus_.send_frame(command)) trip_yaw_locked();
}

void MixedCanMotorBackend::command(AxisId axis, double q_ref_rad,
                                   double limit_spd_rad_s) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) pitch_backend_.command(axis, q_ref_rad, limit_spd_rad_s);
    return;
  }
  const auto now = now_monotonic_ns();
  std::lock_guard lock(yaw_mutex_);
  if (!std::isfinite(q_ref_rad) || !std::isfinite(limit_spd_rad_s)) {
    trip_yaw_locked();
    return;
  }
  yaw_position_target_rad_ = q_ref_rad;
  const double speed_cap = std::clamp(std::abs(limit_spd_rad_s), 0.0, kYawMaxSpeedRadS);
  const double error = yaw_position_target_rad_ - yaw_state_.position_rad;
  const double desired = std::clamp(error * kYawPositionGain, -speed_cap, speed_cap);
  command_yaw_velocity_locked(desired, now);
}

void MixedCanMotorBackend::command_velocity(AxisId axis, double velocity_rad_s) {
  if (axis == AxisId::Pitch) {
    if (pitch_opened_.load()) pitch_backend_.command_velocity(axis, velocity_rad_s);
    return;
  }
  const auto now = now_monotonic_ns();
  std::lock_guard lock(yaw_mutex_);
  if (!std::isfinite(velocity_rad_s)) {
    trip_yaw_locked();
    return;
  }
  yaw_speed_target_rad_s_ = std::clamp(velocity_rad_s, -kYawMaxSpeedRadS, kYawMaxSpeedRadS);
  command_yaw_velocity_locked(yaw_speed_target_rad_s_, now);
}

void MixedCanMotorBackend::keepalive(AxisId axis) {
  if (axis == AxisId::Pitch && pitch_opened_.load()) pitch_backend_.keepalive(axis);
}

void MixedCanMotorBackend::set_current_limit(AxisId axis, double limit_cur_a) {
  if (axis == AxisId::Pitch && pitch_opened_.load()) pitch_backend_.set_current_limit(axis, limit_cur_a);
  // GM6020's documented feedback/command protocol has no current-limit register.
}

void MixedCanMotorBackend::set_speed_loop_gains(AxisId axis, double spd_kp,
                                                double spd_ki) {
  if (axis == AxisId::Pitch && pitch_opened_.load()) pitch_backend_.set_speed_loop_gains(axis, spd_kp, spd_ki);
  // GM6020 velocity PI gains are fixed in this host backend.
}

}  // namespace ota
