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
#include <yaml-cpp/yaml.h>

#include "servo_config.hpp"

#include "common/time.hpp"

namespace ota {
namespace {
using namespace std::chrono_literals;
constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;
constexpr double kDegreesPerRadian = 180.0 / std::numbers::pi;
// The yaw speed ceiling lives in the header as apply_yaw_speed_ceiling(): a CLAMP on
// the ask, not a verdict on a reading -- see the ruling recorded there.
// kYawMaxAccelerationRadS2 lives in the header, next to the ceiling, where the station
// config test can pin it to axes.yaw.max_acceleration_deg_s2.
constexpr double kYawPositionGain = 2.0;
// The first automatic sweep requested 10 deg/s but reached 26.34 deg/s and
// correctly tripped the independent 25 deg/s guard. The earlier 30-degree
// motor probe needed at most 5,643 raw voltage, so retain voltage headroom
// while reducing the service-loop drive and acceleration for this payload.
// The yaw drive ceiling is now `yaw_voltage_ceiling_`, set from turret_mixed.yaml. Its history, kept
// because it is the reason: it was 9000 because a 30-degree probe moved the bare axis with at most
// 5,643 counts, and tonight the axis would not move at all with the output pinned there (the rig has
// a slip ring, so the resistance varies with angle and the old number was headroom measured at one
// lucky angle). WP6 replaces the single ceiling with a measured vout-vs-angle profile.
// THESE TWO ARE VOLTAGE-MODE GAINS. The current-mode gains are a separate pair of numbers
// (axes.yaw.current_kp_a_per_rad_s / _ki_...) in the profile, because counts and torque amperes do
// not convert through a constant here; see gm6020::VelocityLoop.
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

// How to SAY what we are pushing at the motor. The unit travels with the configured mode, and a
// log line that says `vout=0` after the station has switched to torque current is the kind of
// sentence that gets a still axis diagnosed as a dead backend (2026-09-28 did exactly that, twice).
std::string yaw_output_text(bool amperes, double value) {
  char buf[32];
  if (amperes) std::snprintf(buf, sizeof buf, "%+.3f A", value);
  else std::snprintf(buf, sizeof buf, "%.0f counts", value);
  return std::string(buf);
}

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
  // Current mode is admitted here rather than assumed downstream: the frame ids differ, and the
  // two preconditions belong to the operator's record, not to our optimism. The error is split so
  // "Current Ring is not enabled" cannot arrive dressed up as "your profile is the wrong shape".
  const bool yaw_current = p.yaw.control_mode == config::mixed::ControlMode::Current;
  if (p.yaw.protocol != config::mixed::Protocol::Gm6020 ||
      p.yaw.bus_name != "yaw" || p.yaw.motor_id != 1 ||
      p.yaw.topology != config::mixed::Topology::Continuous ||
      (!yaw_current && p.yaw.control_mode != config::mixed::ControlMode::Voltage) ||
      p.yaw.feedback_frame_id != std::optional<uint32_t>{0x205} ||
      (!yaw_current && p.yaw.command_frame_id != std::optional<uint32_t>{0x1ff}) ||
      (yaw_current && p.yaw.command_frame_id != std::optional<uint32_t>{0x1fe})) {
    err = yaw_current
        ? "mixed profile yaw must be GM6020 ID 1 continuous current control with feedback 0x205 "
          "and command 0x1FE"
        : "mixed profile yaw must be GM6020 ID 1 with continuous voltage control";
    return false;
  }
  // The acknowledgement and the host clamp are validated by the profile parser, which owns them
  // and is testable without a CAN socket; the backend keeps the frame-identity rule above.
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
    spdlog::info("mixed effective config: yaw mode={} frame=0x{:x} Kp={} A/(rad/s) Ki={} A/rad cap={} A; pitch configured_mode={} cap={} A; dynamics unqualified",
                 yaw_output_is_amperes() ? "current" : "voltage",
                 *profile_.yaw.command_frame_id, profile_.yaw.current_kp_a_per_rad_s,
                 profile_.yaw.current_ki_a_per_rad_s, profile_.yaw.host_current_limit_a,
                 static_cast<int>(profile_.pitch.control_mode), *profile_.pitch.current_limit_a);
    spdlog::info("yaw estimator rx_window_ms={} (0=legacy 50 ms); friction enabled={} break_pos={} break_neg={} run_pos={} run_neg={} A timeout={} s slew={} A/s; source=mixed_hardware",
        profile_.yaw.velocity_rx_window_ms, profile_.yaw.friction.enabled,
        profile_.yaw.friction.positive_breakaway_a, profile_.yaw.friction.negative_breakaway_a,
        profile_.yaw.friction.positive_run_a, profile_.yaw.friction.negative_run_a,
        profile_.yaw.friction.timeout_s, profile_.yaw.friction.output_slew_a_per_s);
    {
      std::lock_guard lock(yaw_mutex_);
      yaw_encoder_.reset();
      yaw_rx_velocity_.reset();
      yaw_state_ = YawState{};
      yaw_origin_rad_ = 0;
      yaw_position_target_rad_ = yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
      yaw_position_mode_ = yaw_speed_mode_ = false;
      yaw_tx_failure_since_ns_ = yaw_last_successful_tx_ns_ = 0;
      yaw_tx_seq_ = 0;
      yaw_stall_episodes_ = {};
      yaw_output_reason_ = 0;
      yaw_last_output_.store(NAN);
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

    // Startup stop: zero in whatever unit this profile commands. Sending a zero the drive ignores
    // is worse than sending nothing, because it looks like a stopped motor in every later log.
    const auto startup_zero = yaw_zero_frame(profile_.yaw);
    for (int i = 0; i < 20; ++i) {
      if (!yaw_bus_.send_frame(startup_zero, &err)) {
        err = std::string("GM6020 startup ") +
              (profile_.yaw.control_mode == config::mixed::ControlMode::Current
                   ? "zero-current"
                   : "zero-voltage") +
              " request failed: " + err;
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
    if (profile_.servo && !load_servos(err)) {
      close();
      return false;
    }

    heartbeat_ns_.store(0);
    heartbeat_seen_.store(false);
    yaw_trip_.store(false);
    yaw_motion_allowed_.store(false);
    opened_.store(true);
    yaw_guard_ = std::jthread([this](std::stop_token stop) { yaw_guard_loop(stop); });
    if (pitch_servo_configured_)
      pitch_servo_ = std::jthread([this](std::stop_token stop) { pitch_servo_loop(stop); });
    return true;
  } catch (const std::exception& error) {
    err = error.what();
    close();
    return false;
  }
}

void MixedCanMotorBackend::close() {
  opened_.store(false);
  if (pitch_servo_.joinable()) {
    pitch_servo_.request_stop();
    pitch_servo_.join();
  }
  pitch_servo_active_.store(false);
  {
    std::lock_guard lock(yaw_mutex_);
    yaw_servo_active_ = false;
  }
  if (yaw_guard_.joinable()) {
    yaw_guard_.request_stop();
    yaw_guard_.join();
  }
  yaw_motion_allowed_.store(false);
  yaw_reference_valid_.store(false);
  if (yaw_opened_.load()) {
    // Shutdown zero, same single decision as startup and fault: zero current where the profile
    // commands current. This is a request, not a de-energised claim -- the drive reports no
    // enable bit, so `disable state unavailable` is the honest words for it downstream.
    const auto zero = yaw_zero_frame(profile_.yaw);
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
  const bool accepted = yaw_encoder_.update(decoded.angle_count, decoded.rx_ns);
  yaw_state_.encoder_valid = yaw_encoder_.valid();
  if (encoder_was_valid && !yaw_state_.encoder_valid)
    spdlog::error("GM6020 encoder invalidated: dt_ms={:.3f} previous_count={} count={} speed_rpm={}",
                  (decoded.rx_ns - previous_rx_ns) / 1e6,
                  previous_count, decoded.angle_count, decoded.speed_rpm);
  if (!accepted) {
    // A transient (owner ruling 2026-10-02): skipped, counted, said at most once a second. The
    // frame is still fresh feedback for everything except the angle.
    if (decoded.rx_ns - yaw_encoder_log_ns_ > 1'000'000'000) {
      spdlog::warn("GM6020 reading skipped as implausible: dt_ms={:.3f} previous_count={} count={} speed_rpm={} "
                   "(skipped {}, believed after a run {}, long gaps {})", (decoded.rx_ns - previous_rx_ns) / 1e6,
                   previous_count, decoded.angle_count, decoded.speed_rpm, yaw_encoder_.rejected(),
                   yaw_encoder_.believed(), yaw_encoder_.long_gaps());
      yaw_encoder_log_ns_ = decoded.rx_ns;
    }
    ++yaw_state_.count;
    return;
  }
  yaw_state_.position_rad = yaw_encoder_.relative_rad() - yaw_origin_rad_;
  if (yaw_state_.encoder_valid)
    yaw_rx_velocity_.observe(yaw_encoder_.relative_rad(), decoded.rx_ns);
  ++yaw_state_.count;
  if (!yaw_reference_valid_.load() && yaw_encoder_.valid()) {
    // Provisional origin permits stationary assessment. The committed origin
    // is refreshed after the bus/pitch UID checks complete.
    yaw_origin_rad_ = 0;
  }
  if (yaw_servo_active_) step_yaw_servo_locked(decoded.rx_ns);
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
  snapshot.raw_rx_ns = yaw_state_.feedback.rx_ns;
  snapshot.rx_seq = yaw_state_.count;
  snapshot.encoder_raw = yaw_state_.received ? yaw_state_.feedback.angle_count : -1;
  snapshot.current_raw = yaw_state_.feedback.current_raw;
  snapshot.current_raw_valid = yaw_state_.received;
  if (yaw_feedback_safe_locked(now)) {
    snapshot.has_feedback = true;
    // Legacy supervisor consumes a cycle-bounded timestamp; raw_rx_ns retains
    // the unmodified RX timestamp, including frames arriving during this cycle.
    snapshot.rx_ns = std::min(yaw_state_.feedback.rx_ns, now);
    snapshot.q_rad = yaw_state_.position_rad;
    snapshot.v_rad_s = yaw_state_.feedback.speed_rad_s();
    // The status frame does carry a figure -- it is the drive's own torque current, and
    // this is where it used to be dropped on the floor. Amperes, not N·m: the guide gives
    // no torque constant, so reporting 0.0 here would be a lie and reporting newtons would
    // be a bigger one.
    snapshot.current_a = yaw_state_.feedback.current_a();
    snapshot.current_a_known = true;
    snapshot.torque_nm = std::numeric_limits<double>::quiet_NaN();
  } else {
    snapshot.rx_ns = yaw_state_.received ? std::min(yaw_state_.feedback.rx_ns, now) : 0;
    snapshot.q_rad = std::numeric_limits<double>::quiet_NaN();
    snapshot.v_rad_s = std::numeric_limits<double>::quiet_NaN();
    snapshot.torque_nm = std::numeric_limits<double>::quiet_NaN();
  }
  return snapshot;
}

void MixedCanMotorBackend::trip_yaw_locked(const char* condition) {
  if (condition && !yaw_trip_.load()) {
    TripInputs input;
    input.feedback_age_ms = (now_monotonic_ns() - yaw_state_.feedback.rx_ns) / 1e6;
    input.temp_raw = yaw_state_.feedback.temperature_raw;
    input.speed_deg_s = yaw_velocity_loop_.velocity_rad_s() * kDegreesPerRadian;
    input.reference_valid = yaw_reference_valid_.load();
    const std::lock_guard detail_lock(yaw_trip_detail_mutex_);
    format_trip_detail(input, condition, yaw_trip_detail_);
  }
  yaw_trip_.store(true);
  yaw_motion_allowed_.store(false);
  yaw_servo_active_ = false;
  yaw_position_mode_ = yaw_speed_mode_ = false;
  yaw_position_target_rad_ = yaw_state_.position_rad;
  yaw_speed_target_rad_s_ = yaw_shaped_speed_rad_s_ = 0;
  yaw_requested_velocity_rad_s_.store(0);
  yaw_velocity_loop_.reset(yaw_state_.position_rad, now_monotonic_ns());
  send_yaw_zero_locked();
}

// Every "stop asking" path funnels here: hold, trip, deenergize, and the guard thread's final
// flush. One decision, one frame builder (yaw_zero_frame), so a fault zero cannot quietly still be
// a voltage frame after the profile moved to current.
bool MixedCanMotorBackend::send_yaw_zero_locked() {
  yaw_output_reason_ = yaw_trip_.load() ? 3 : 2;
  return send_yaw_output_locked(yaw_zero_frame(profile_.yaw), 0);
}

bool MixedCanMotorBackend::send_yaw_output_locked(const can::RawFrame& frame, double output) {
  const auto now = now_monotonic_ns();
  yaw_requested_output_ = output;
  if (!(yaw_test_send_ ? yaw_test_send_(frame) : yaw_bus_.send_frame(frame))) {
    yaw_output_reason_ = 4;
    if (!yaw_tx_failure_since_ns_) yaw_tx_failure_since_ns_ = now;
    yaw_command_not_sent_.store(true);
    return false;
  }
  yaw_tx_failure_since_ns_ = 0;
  yaw_last_successful_tx_ns_ = now;
  ++yaw_tx_seq_;
  // Report the encoded slot value after quantization, not the pre-encoding
  // floating-point request. Successful write still is not a drive ACK.
  const auto slot = 2 * ((profile_.yaw.motor_id ? profile_.yaw.motor_id : 1) - 1);
  const auto raw = static_cast<int16_t>((frame.data[slot] << 8) | frame.data[slot + 1]);
  yaw_last_output_.store(yaw_output_is_amperes() ? raw * gm6020::kAmpsPerRaw : raw);
  return true;
}

MotorBackend::OutputEvidence MixedCanMotorBackend::output_evidence(AxisId axis) const {
  if (axis == AxisId::Pitch) return pitch_backend_.output_evidence(axis);
  std::lock_guard lock(yaw_mutex_);
  OutputEvidence result;
  result.tx_ns = yaw_last_successful_tx_ns_;
  result.tx_seq = yaw_tx_seq_;
  result.requested = yaw_requested_output_;
  result.successful = yaw_last_output_.load();
  result.integral = yaw_velocity_loop_.integral();
  result.velocity_estimate = yaw_velocity_loop_.velocity_rad_s();
  result.kp = yaw_output_is_amperes() ? profile_.yaw.current_kp_a_per_rad_s : kYawVelocityKp;
  result.ki = yaw_output_is_amperes() ? profile_.yaw.current_ki_a_per_rad_s : kYawVelocityKi;
  result.current_cap = yaw_output_is_amperes() ? profile_.yaw.host_current_limit_a : NAN;
  result.rx_velocity_20 = yaw_rx_velocity_.estimate(20);
  result.rx_velocity_30 = yaw_rx_velocity_.estimate(30);
  result.rx_velocity_40 = yaw_rx_velocity_.estimate(40);
  result.velocity_window_ms = profile_.yaw.velocity_rx_window_ms;
  result.friction_a = yaw_velocity_loop_.friction_output().feedforward_target_a;
  result.friction_state = static_cast<int>(yaw_velocity_loop_.friction_output().state);
  result.friction_exhausted = yaw_velocity_loop_.friction_output().attempt_exhausted;
  result.command_kind = yaw_output_is_amperes() ? 1 : 2;
  result.reason = yaw_output_reason_;
  if (yaw_servo_active_) {
    const auto& p = yaw_servo_.parameters();
    result.requested = yaw_servo_out_.requested;
    result.integral = yaw_servo_out_.integral;
    result.velocity_estimate = yaw_servo_out_.velocity;
    result.kp = p.kq;
    result.ki = p.ki;
    result.current_cap = yaw_servo_out_.cap;
    result.friction_a = yaw_servo_out_.friction;
  }
  return result;
}

void MixedCanMotorBackend::yaw_guard_loop(std::stop_token stop) {
  double prior_position = 0;
  TimeNs prior_sample_ns = 0;
  double measured_speed = 0;
  double progress_position = 0;
  TimeNs progress_at = now_monotonic_ns();
  TimeNs last_bus_health_check = 0;
  uint64_t previous_rx_errors = 0, previous_tx_failures = 0;
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
      // The verdict and its record are built from ONE object, EVERY cycle. The first version
      // assembled the record only after deciding to stop, so a condition that never faults
      // left no trace at all -- "it limped for an hour" was unauditable -- and the fields
      // describing a fault were assembled after the fault instead of of it.
      MotorBackend::TripInputs in;
      in.feedback_unsafe = !yaw_feedback_safe_locked(now);
      in.can_down = !health.up;
      in.can_state_wrong = health.state < 0 || health.state >= static_cast<int>(can::CanIfState::BusOff);
      in.can_counters_bad = health.rx_error_frames != previous_rx_errors || health.tx_failed != previous_tx_failures;
      previous_rx_errors = health.rx_error_frames;
      previous_tx_failures = health.tx_failed;
      in.bus_unhealthy = !yaw_bus_healthy();
      in.speed_not_finite = !std::isfinite(measured_speed);
      in.temp_raw_over = yaw_temp_guard > 0 &&
                         yaw_state_.feedback.temperature_raw >= yaw_temp_guard;
      // A demand left standing in the field from the last accepted cycle is not this
      // cycle's demand. Without this, "the caller stopped asking" and "the axis will not
      // move" are the same log line, and 2026-09-28 spent its afternoon between those two.
      const bool command_stale = yaw_command_is_stale(now, yaw_last_command_ns_);
      in.command_not_sent = yaw_command_not_sent_.load() || command_stale;
      in.no_progress = std::abs(requested_speed) >= kNoProgressCommandRadS &&
                       !command_stale && now - progress_at > kNoProgressLimitNs;
      in.heartbeat_stale = heartbeat_seen_.load() &&
                           now - heartbeat_ns_.load() > kHeartbeatLimitNs;
      // One failed TX is retried by the next normal cycle. Sustained failure
      // cancels motion even when unsolicited encoder feedback remains fresh.
      if (yaw_tx_failure_since_ns_ && now - yaw_tx_failure_since_ns_ >= 20'000'000)
        in.can_down = true;
      in.reference_valid = yaw_reference_valid_.load();
      in.feedback_age_ms = (now - yaw_state_.feedback.rx_ns) / 1e6;
      in.temp_raw = yaw_state_.feedback.temperature_raw;
      in.speed_deg_s = measured_speed * kDegreesPerRadian;
      yaw_stall_episodes_.observe(in.no_progress);

      // Owner's ordering, 2026-09-28: running beats holding, holding beats faulting, and a
      // fault is reserved for a motor we cannot control, a motor reporting its own heat, or
      // something equally dangerous. Everything else is driven through and said out loud.
      const GuardResponse verdict = yaw_guard_response(in, yaw_stall_episodes_.count);
      // Owner ruling 2026-10-02 ("Fault, hold, degrade"): a fault condition has to persist
      // kGuardFaultPersistNs before it is one. Until then it is an episode: blind (no fresh feedback
      // or no bus) the axis coasts on zero current, which for this balanced axis is safe; otherwise
      // (a stale control heartbeat, motor heat) the servo keeps holding what it was last given.
      const bool was_pending = yaw_guard_persistence_.started;
      const GuardResponse response = yaw_guard_persistence_.decide(verdict, now, kGuardFaultPersistNs);
      if (verdict == GuardResponse::Fault && !was_pending)
        spdlog::warn("GM6020 guard: {} (feedback_age_ms={:.1f}); fault if it lasts {} ms",
                     MotorBackend::select_trip_condition(in), (now - yaw_state_.feedback.rx_ns) / 1e6,
                     kGuardFaultPersistNs / 1'000'000);
      if (verdict != GuardResponse::Fault && was_pending && !yaw_trip_.load())
        spdlog::info("GM6020 guard: cleared after {:.1f} ms", (now - yaw_guard_persistence_.since_ns) / 1e6);
      if (response == GuardResponse::Hold && (in.feedback_unsafe || in.can_down || in.can_state_wrong))
        send_yaw_zero_locked();
      if (response == GuardResponse::Hold) {
        if (!yaw_degraded_.exchange(true)) ++yaw_guard_events_;  // one episode, one count
      } else if (response == GuardResponse::Fault && !yaw_trip_.load()) {
        MotorBackend::TripDetail td{};
        MotorBackend::format_trip_detail(in, MotorBackend::select_trip_condition(in), td);
        const std::lock_guard detail_lock(yaw_trip_detail_mutex_);
        yaw_trip_detail_ = td;
        spdlog::error("GM6020 guard fault: feedback_safe={} reference_valid={} received={} encoder_valid={} feedback_age_ms={:.3f} can_up={} can_state={} rxerr={} txfail={} both_buses_healthy={} measured_speed_deg_s={:.3f} temp_raw={} requested_speed_deg_s={:.3f} no_progress_ms={} ms_since_command={:.1f} heartbeat_seen={} heartbeat_age_ms={}",
                      yaw_feedback_safe_locked(now), yaw_reference_valid_.load(),
                      yaw_state_.received, yaw_state_.encoder_valid,
                      (now - yaw_state_.feedback.rx_ns) / 1e6, health.up, health.state,
                      health.rx_error_frames, health.tx_failed, bus_health_ok_.load(),
                      measured_speed * kDegreesPerRadian, yaw_state_.feedback.temperature_raw,
                      requested_speed * kDegreesPerRadian, (now - progress_at) / 1'000'000,
                      (now - yaw_last_command_ns_) * 1e-6, heartbeat_seen_.load(),
                      heartbeat_seen_.load() ? (now - heartbeat_ns_.load()) / 1'000'000 : 0);
        trip_yaw_locked();
      } else {
        const bool wants_motion = std::abs(requested_speed) >= kNoProgressCommandRadS;
        const bool any_doubt = yaw_guard_doubt(in, wants_motion);
        if (any_doubt) {
          if (!yaw_degraded_.exchange(true)) ++yaw_guard_events_;  // one episode, one count
          if (now - last_degrade_log_ns_ > 1'000'000'000) {  // at most one line a second
            spdlog::warn("GM6020 degraded, still driving: cond={} rxerr={} txfail={} cmd_stale={} "
                         "requested={:.3f} measured={:.3f} deg/s ms_since_command={:.1f} {} q={:.3f}rad",
                         MotorBackend::select_trip_condition(in), health.rx_error_frames,
                         health.tx_failed, command_stale ? 1 : 0, requested_speed * kDegreesPerRadian,
                         measured_speed * kDegreesPerRadian, (now - yaw_last_command_ns_) * 1e-6,
                         yaw_output_text(yaw_output_is_amperes(), yaw_last_output_.load()),
                         yaw_state_.position_rad);
            last_degrade_log_ns_ = now;
          }
        } else {
          yaw_degraded_.store(false);
        }
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
    if (!bus.available || !bus.up || bus.state < 0 ||
        bus.state >= static_cast<int>(can::CanIfState::BusOff)) return false;
  }
  return true;
}

bool MixedCanMotorBackend::yaw_bus_healthy() const {
  const auto bus = can_health();
  return bus.available && bus.up && bus.state >= 0 &&
      bus.state < static_cast<int>(can::CanIfState::BusOff);
}

std::vector<CanHealth> MixedCanMotorBackend::can_health_all() const {
  return {yaw_opened_.load() ? socketcan_health(yaw_bus_) : CanHealth{},
          pitch_opened_.load() ? pitch_backend_.can_health() : CanHealth{}};
}

CanHealth MixedCanMotorBackend::can_health() const {
  if (yaw_test_health_) return yaw_test_health_();
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
  return yaw_trip_.load() || pitch_servo_fault_.load() || (pitch_opened_.load() && pitch_backend_.watchdog_fault());
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
    release_pitch_servo();
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
  release_yaw_servo_locked();
  if (!std::isfinite(limit) || limit < 0 || !yaw_feedback_safe_locked(now) ||
      !yaw_bus_healthy() || yaw_trip_.load()) {
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
    release_pitch_servo();
    pitch_transition_active_.store(false);
    if (pitch_opened_.load()) pitch_backend_.deenergize(axis);
    pitch_enabled_owned_.store(false);
    return;
  }
  std::lock_guard lock(yaw_mutex_);
  release_yaw_servo_locked();
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
  // Stamped before every gate, including the refusals: "the loop put a demand in front of
  // me this cycle" and "that frame went out" are two facts, and reading them as one is
  // what made five trips today unreadable. Nothing sets this field but a call.
  yaw_last_command_ns_ = now;
  if (!std::isfinite(desired) || !yaw_feedback_safe_locked(now) || yaw_trip_.load() ||
      !yaw_motion_allowed_.load() || !heartbeat_seen_.load() ||
      now - heartbeat_ns_.load() > kHeartbeatLimitNs || !yaw_bus_healthy()) {
    // Refused, not tripped: unsafe feedback or a sick bus becomes a fault only in the guard, after
    // it has persisted (owner ruling 2026-10-02).
    {
      // Nothing went out but a zero. Say so, and stop quoting the last accepted
      // velocity as if it described this cycle -- that stale 10 deg/s is what made
      // three no_progress trips read as "commanded and blocked".
      yaw_requested_velocity_rad_s_.store(0);
      yaw_command_not_sent_.store(true);
      send_yaw_zero_locked();
    }
    return;
  }
  const double dt = std::clamp((now - yaw_velocity_loop_previous_command_ns_) * 1e-9, 0.0, .020);
  const double step = kYawMaxAccelerationRadS2 * dt;
  // The ceiling bites the REQUEST here. An axis reading faster than the ceiling is a
  // question about the reading or the load, and the answer to that is not to drop the
  // payload (see apply_yaw_speed_ceiling).
  const double ceiling_applied = apply_yaw_speed_ceiling(desired);
  yaw_shaped_speed_rad_s_ += std::clamp(ceiling_applied - yaw_shaped_speed_rad_s_, -step, step);
  yaw_requested_velocity_rad_s_.store(desired);
  yaw_command_not_sent_.store(false);  // a real frame follows below
  yaw_velocity_loop_previous_command_ns_ = now;
  // The velocity loop produces an EFFORT, and only this last step knows which unit this station
  // asked for. Everything above it -- position target, the acceleration ramp, encoder unwrap, the
  // speed ceiling, every guard -- is shared, so switching the axis from volts to amperes changes
  // the frame and the gains, not the control architecture.
  can::RawFrame command{};
  const auto prior_loop = yaw_velocity_loop_;
  double requested_output = 0;
  if (yaw_output_is_amperes()) {
    // Amperes in, amperes out, clamped twice over: the PI ceiling IS the host limit, and
    // current_frame clamps again against the same number before it encodes.
    const double amps = yaw_velocity_loop_.update_amps(
        yaw_shaped_speed_rad_s_, yaw_state_.position_rad, now, kYawSpeedCeilingRadS,
        profile_.yaw.host_current_limit_a, profile_.yaw.current_kp_a_per_rad_s,
        profile_.yaw.current_ki_a_per_rad_s,
        profile_.yaw.velocity_rx_window_ms ? yaw_rx_velocity_.estimate(profile_.yaw.velocity_rx_window_ms) : NAN,
        &profile_.yaw.friction, yaw_moving_intent_, yaw_state_.count);
    if (!yaw_velocity_loop_.valid()) {
      trip_yaw_locked("velocity_loop_invalid");
      return;
    }
    yaw_last_shaped_rad_s_.store(yaw_shaped_speed_rad_s_);  // what the loop was told to track
    requested_output = amps;
    command = gm6020::current_frame(profile_.yaw.motor_id, amps, profile_.yaw.host_current_limit_a);
  } else {
    const int voltage = yaw_velocity_loop_.update(
        yaw_shaped_speed_rad_s_, yaw_state_.position_rad, now, kYawSpeedCeilingRadS,
        yaw_voltage_ceiling_, kYawVelocityKp, kYawVelocityKi);
    if (!yaw_velocity_loop_.valid()) {
      trip_yaw_locked("velocity_loop_invalid");
      return;
    }
    yaw_last_shaped_rad_s_.store(yaw_shaped_speed_rad_s_);
    requested_output = static_cast<double>(voltage);
    command = gm6020::voltage_frame(profile_.yaw.motor_id, voltage);
  }
  yaw_output_reason_ = yaw_velocity_loop_.late_cycle() ? 5 : 1;
  if (!send_yaw_output_locked(command, requested_output)) {
    yaw_velocity_loop_ = prior_loop; // do not integrate an output that was never sent
    // Sustained failure is the guard's: can_down after 20 ms, a fault only if it persists.
  }
}

void MixedCanMotorBackend::command(AxisId axis, double q_ref_rad,
                                   double limit_spd_rad_s) {
  if (axis == AxisId::Pitch) {
    release_pitch_servo();
    if (pitch_opened_.load()) pitch_backend_.command(axis, q_ref_rad, limit_spd_rad_s);
    return;
  }
  const auto now = now_monotonic_ns();
  std::lock_guard lock(yaw_mutex_);
  release_yaw_servo_locked();
  if (!std::isfinite(q_ref_rad) || !std::isfinite(limit_spd_rad_s)) {
    trip_yaw_locked("nonfinite_reference");
    return;
  }
  yaw_position_target_rad_ = q_ref_rad;
  const double speed_cap = std::clamp(std::abs(limit_spd_rad_s), 0.0, kYawSpeedCeilingRadS);
  const double error = yaw_position_target_rad_ - yaw_state_.position_rad;
  const double desired = std::clamp(error * kYawPositionGain, -speed_cap, speed_cap);
  command_yaw_velocity_locked(desired, now);
}

void MixedCanMotorBackend::command_velocity(AxisId axis, double velocity_rad_s) {
  if (axis == AxisId::Pitch) {
    release_pitch_servo();
    if (pitch_opened_.load()) pitch_backend_.command_velocity(axis, velocity_rad_s);
    return;
  }
  const auto now = now_monotonic_ns();
  std::lock_guard lock(yaw_mutex_);
  release_yaw_servo_locked();
  if (!std::isfinite(velocity_rad_s)) {
    trip_yaw_locked("nonfinite_reference");
    return;
  }
  yaw_speed_target_rad_s_ = std::clamp(velocity_rad_s, -kYawSpeedCeilingRadS, kYawSpeedCeilingRadS);
  command_yaw_velocity_locked(yaw_speed_target_rad_s_, now);
}

void MixedCanMotorBackend::set_motion_intent(AxisId axis, bool moving) {
  if (axis != AxisId::Yaw) return;
  std::lock_guard lock(yaw_mutex_);
  yaw_moving_intent_ = moving;
}

bool MixedCanMotorBackend::apply_yaw_trial(const YawTrialSettings& s, std::string& error) {
  std::lock_guard lock(yaw_mutex_);
  if (!yaw_output_is_amperes() || yaw_trip_.load() || !yaw_feedback_safe_locked(now_monotonic_ns()) ||
      !std::isfinite(s.kp_a_per_rad_s) || s.kp_a_per_rad_s <= 0 || s.kp_a_per_rad_s > 10 ||
      !std::isfinite(s.ki_a_per_rad) || s.ki_a_per_rad < 0 || s.ki_a_per_rad > 20 ||
      (s.rx_window_ms != 0 && s.rx_window_ms != 20 && s.rx_window_ms != 30 && s.rx_window_ms != 40) ||
      (s.friction.enabled && !s.friction.valid(profile_.yaw.host_current_limit_a))) {
    error = "yaw trial requires fresh current-mode feedback and bounded finite settings"; return false;
  }
  yaw_velocity_loop_.prepare_current_tuning(yaw_shaped_speed_rad_s_,s.kp_a_per_rad_s,profile_.yaw.host_current_limit_a);
  profile_.yaw.current_kp_a_per_rad_s = s.kp_a_per_rad_s;
  profile_.yaw.current_ki_a_per_rad_s = s.ki_a_per_rad;
  profile_.yaw.velocity_rx_window_ms = s.rx_window_ms;
  profile_.yaw.friction = s.friction;
  spdlog::info("yaw session trial APPLIED: Kp={} A/(rad/s) Ki={} A/rad rx_window_ms={} friction={} break=+{}/-{} A run=+{}/-{} A slew={} A/s cap={} A (unchanged); not persisted or qualified",
      s.kp_a_per_rad_s,s.ki_a_per_rad,s.rx_window_ms,s.friction.enabled,
      s.friction.positive_breakaway_a,s.friction.negative_breakaway_a,s.friction.positive_run_a,
      s.friction.negative_run_a,s.friction.output_slew_a_per_s,profile_.yaw.host_current_limit_a);
  return true;
}

MotorBackend::YawTrialSettings MixedCanMotorBackend::yaw_trial_settings() const {
  std::lock_guard lock(yaw_mutex_);
  YawTrialSettings s;
  s.kp_a_per_rad_s = profile_.yaw.current_kp_a_per_rad_s;
  s.ki_a_per_rad = profile_.yaw.current_ki_a_per_rad_s;
  s.rx_window_ms = profile_.yaw.velocity_rx_window_ms;
  s.friction = profile_.yaw.friction;
  return s;
}

void MixedCanMotorBackend::keepalive(AxisId axis) {
  if (axis == AxisId::Pitch && pitch_opened_.load() && !pitch_servo_active_.load()) pitch_backend_.keepalive(axis);
}

// ---------------------------------------------------------------- ADR-002.2 servos (ADR-003 3b)

bool MixedCanMotorBackend::load_servos(std::string& err) {
  const auto& c = *profile_.servo;
  try {
    auto yaw = axis::servo_from_yaml(YAML::LoadFile(c.yaw_asset)["servo_parameters"]);
    // The profile's authority is the ceiling; an asset can ask for less, never more.
    yaw.current_cap = std::min(yaw.current_cap, c.yaw_current_limit_a);
    yaw.rms_limit = std::min(yaw.rms_limit, c.yaw_rms_limit_a);
    if (!yaw_servo_.configure(yaw)) {
      err = "yaw servo asset rejected: " + c.yaw_asset;
      return false;
    }
    const auto trial = YAML::LoadFile(c.pitch_asset)["servo_trial"];
    // The asset's speed clamp was a commissioning choice; the owner's cap is 100 RPM (2026-10-02).
    const axis::PositionLoopParameters loop{trial["kp_per_s"].as<double>(), trial["ki_per_s2"].as<double>(),
                                            trial["integral_clamp_rad_s"].as<double>(), c.speed_limit_rad_s};
    pitch_following_error_rad_ = trial["following_error_rad"].as<double>();
    if (!pitch_loop_.configure(loop) || !(pitch_following_error_rad_ > 0)) {
      err = "pitch servo asset rejected: " + c.pitch_asset;
      return false;
    }
    yaw_oscillation_ = axis::OscillationMonitor(c.oscillation_limit_a);
    yaw_servo_epoch_ns_ = now_monotonic_ns();
    yaw_servo_configured_ = pitch_servo_configured_ = true;
    spdlog::info("ADR-002.2 servos loaded: yaw {} (kq {} A/rad, kv {} A s/rad, ki {} A/(rad s), peak {} A, rms {} A, "
                 "inertia {} A s^2/rad); pitch {} (kp {}/s, ki {}/s^2, speed {} rad/s, following {} rad)",
                 c.yaw_asset, yaw.kq, yaw.kv, yaw.ki, yaw.current_cap, yaw.rms_limit, yaw.inertia,
                 c.pitch_asset, loop.kp, loop.ki, loop.speed_limit, pitch_following_error_rad_);
    return true;
  } catch (const std::exception& error) {
    err = std::string("servo asset: ") + error.what();
    return false;
  }
}

bool MixedCanMotorBackend::servo_available(AxisId axis) const {
  return axis == AxisId::Yaw ? yaw_servo_configured_ : pitch_servo_configured_ && !pitch_servo_fault_.load();
}

void MixedCanMotorBackend::release_yaw_servo_locked() {
  if (!yaw_servo_active_) return;
  yaw_servo_active_ = false;
  // The legacy loop takes over from where the axis is, at rest in its own terms.
  yaw_velocity_loop_.reset(yaw_state_.position_rad, now_monotonic_ns());
  yaw_shaped_speed_rad_s_ = 0;
  // Its episodes end with it: nothing would update them, and a latched HOLD would never clear.
  yaw_osc_episode_ = {}; yaw_stall_episode_ = {};
  yaw_osc_hold_.store(nullptr); yaw_stall_hold_.store(nullptr);
  spdlog::info("yaw servo released at q={:+.5f} rad (stale segments while engaged: {})",
               yaw_state_.position_rad, yaw_servo_stale_);
}

void MixedCanMotorBackend::release_pitch_servo() {
  std::lock_guard lock(pitch_servo_mutex_);
  pitch_servo_active_.store(false);
}

bool MixedCanMotorBackend::command_reference(AxisId axis, const ServoReference& r) {
  if (!std::isfinite(r.q) || !std::isfinite(r.v) || !std::isfinite(r.a) || !std::isfinite(r.j) ||
      !(r.valid_s > 0)) {
    if (axis == AxisId::Yaw) {
      std::lock_guard lock(yaw_mutex_);
      trip_yaw_locked("nonfinite_reference");
    }
    return false;
  }
  if (axis == AxisId::Pitch) {
    std::lock_guard lock(pitch_servo_mutex_);
    if (!pitch_servo_configured_ || pitch_servo_fault_.load() || !pitch_opened_.load()) return false;
    // Refused, not engaged (see pitch_servo_may_engage); refusing while engaged hands the axis to
    // the legacy path, whose command releases the servo.
    if (!pitch_reference_has_envelope(r)) return false;
    if (!pitch_servo_active_.load()) {
      can::AxisLatest l;
      if (!pitch_system_.axis(AxisId::Pitch).latest(l) || !l.has_feedback || l.mode != 2 || l.faults ||
          now_monotonic_ns() - l.rx_ns > kPitchFreshNs) return false;
      if (pitch_follow_hold_.load() && now_monotonic_ns() - pitch_follow_since_ns_ >= kServoQuietNs &&
          std::abs(r.q - l.q_rad) < kFollowClearRad) {
        spdlog::info("pitch servo following error cleared: the reference is back at the axis");
        pitch_follow_hold_.store(nullptr);
      }
      if (pitch_follow_hold_.load()) return false;
      if (!pitch_servo_may_engage(r, l.q_rad, profile_.servo->pitch_guard_rad)) return false;
      pitch_loop_.reset();
      pitch_servo_last_step_ns_ = 0;
      pitch_hold_q_ = l.q_rad;
      pitch_servo_active_.store(true);
      spdlog::info("pitch servo engaged at q={:+.5f} rad", l.q_rad);
    }
    pitch_reference_ = r;
    return true;
  }
  std::lock_guard lock(yaw_mutex_);
  if (!yaw_servo_configured_) return false;
  const auto now = now_monotonic_ns();
  yaw_last_command_ns_ = now;
  if (yaw_follow_hold_.load() && now - yaw_follow_since_ns_ >= kServoQuietNs &&
      std::abs(r.q - yaw_state_.position_rad) < kFollowClearRad) {
    spdlog::info("yaw servo following error cleared: the reference is back at the axis");
    yaw_follow_hold_.store(nullptr);
  }
  if (!yaw_servo_active_) {
    if (yaw_trip_.load() || !yaw_motion_allowed_.load() || !yaw_feedback_safe_locked(now)) return false;
    if (yaw_follow_hold_.load()) return false;   // not onto a reference still away from the axis
    // Take over at the measured state; the current on the wire becomes the integral's start.
    yaw_servo_offset_rad_ = yaw_encoder_.first_rad() + yaw_origin_rad_;
    const double applied = yaw_last_output_.load();
    if (!yaw_servo_.reset((yaw_state_.feedback.rx_ns - yaw_servo_epoch_ns_) * 1e-9,
                          yaw_state_.position_rad + yaw_servo_offset_rad_, 0.0,
                          std::isfinite(applied) ? applied : 0.0)) return false;
    yaw_oscillation_ = axis::OscillationMonitor(profile_.servo->oscillation_limit_a);
    yaw_servo_last_step_ns_ = yaw_servo_last_rock_ns_ = 0;
    yaw_servo_hold_q_ = yaw_state_.position_rad;
    yaw_servo_stale_ = 0;
    yaw_servo_active_ = true;
    spdlog::info("yaw servo engaged at q={:+.5f} rad (absolute {:+.5f})", yaw_state_.position_rad,
                 yaw_state_.position_rad + yaw_servo_offset_rad_);
  }
  yaw_reference_ = r;
  yaw_requested_velocity_rad_s_.store(r.v);
  yaw_command_not_sent_.store(false);
  return true;
}

void MixedCanMotorBackend::track_yaw_servo_episodes_locked(TimeNs now, bool rock_settling) {
  using E = control::EpisodeLatch::Event;
  const double limit = profile_.servo ? profile_.servo->oscillation_limit_a : 0.0;
  const double rms = yaw_oscillation_.rms();
  const unsigned persist_s = kServoFailurePersistNs / 1'000'000'000;
  switch (yaw_osc_episode_.update(now, limit > 0 && rms > limit, rms < 0.7 * limit, kServoFailurePersistNs, kServoQuietNs)) {
    case E::Started:
      yaw_osc_peak_a_ = rms;
      spdlog::warn("yaw servo oscillating: fast current RMS {:.3f} A > {:.3f} A (tolerated; HOLD if it lasts {} s)",
                   rms, limit, persist_s);
      break;
    case E::Persisted:
      spdlog::error("yaw servo oscillating for {} s (peak fast current RMS {:.3f} A): HOLD until it clears",
                    persist_s, yaw_osc_peak_a_);
      yaw_osc_hold_.store("yaw servo oscillating for 5 s");
      break;
    case E::Cleared:
      spdlog::info("yaw servo oscillation cleared after {:.2f} s (peak fast current RMS {:.3f} A)",
                   yaw_osc_episode_.last_duration_ns() * 1e-9, yaw_osc_peak_a_);
      yaw_osc_hold_.store(nullptr);
      break;
    case E::None: break;
  }
  if (yaw_osc_episode_.active()) yaw_osc_peak_a_ = std::max(yaw_osc_peak_a_, rms);
  // Stuck: the stall recovery keeps rocking without freeing the axis.
  switch (yaw_stall_episode_.update(now, rock_settling, !rock_settling, kServoFailurePersistNs, kServoQuietNs)) {
    case E::Persisted:
      spdlog::error("yaw servo stalled for {} s (stall recovery has not freed it): HOLD until it clears", persist_s);
      yaw_stall_hold_.store("yaw servo stalled for 5 s");
      break;
    case E::Cleared:
      if (yaw_stall_hold_.load()) spdlog::info("yaw servo stall cleared");
      yaw_stall_hold_.store(nullptr);
      break;
    default: break;
  }
}

// Pitch is unbalanced and the CyberGear holds on its own encoder: only a drive that has faulted
// (and so already stopped holding) is released. Yaw's GM6020 runs on host current, so a tripped
// yaw has no holding loop left and its zero current is the release.
bool MixedCanMotorBackend::fault_releases_axis(AxisId axis) const {
  if (axis == AxisId::Yaw) return true;
  if (!pitch_opened_.load()) return true;
  can::AxisLatest l;
  return pitch_system_.axis(AxisId::Pitch).latest(l) && l.has_feedback && l.faults != 0;
}

// The CyberGear holds speed zero on its own encoder; yaw (host current) has no such hold.
void MixedCanMotorBackend::hold_axis(AxisId axis) {
  if (axis != AxisId::Pitch || !pitch_opened_.load()) { deenergize(axis); return; }
  release_pitch_servo();
  pitch_transition_active_.store(false);
  if (!pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0))
    spdlog::warn("pitch hold: speed-zero write refused (mode transition in progress); the drive keeps its last target");
}

// Called with yaw_mutex_ held, on every GM6020 frame while the servo is engaged: the
// commissiond session's order (observe the encoder, step, transmit, acknowledge) with the
// production guards in front of it and the commissioning trips behind it.
void MixedCanMotorBackend::step_yaw_servo_locked(TimeNs rx_ns) {
  const auto now = now_monotonic_ns();
  // Unsafe feedback or an unhealthy bus: no step on this frame. Whether it is a fault is the guard's
  // call, after it has persisted (owner ruling 2026-10-02).
  if (!yaw_feedback_safe_locked(now) || !yaw_bus_healthy()) return;
  if (yaw_trip_.load() || !yaw_motion_allowed_.load() || !heartbeat_seen_.load() ||
      now - heartbeat_ns_.load() > kHeartbeatLimitNs) {
    yaw_servo_active_ = false;
    send_yaw_zero_locked();
    return;
  }
  const double offset = yaw_servo_offset_rad_;
  if (!yaw_servo_.observe_encoder((rx_ns - yaw_servo_epoch_ns_) * 1e-9, yaw_state_.position_rad + offset)) {
    // The observer could not take the reading: let go (zero current) and let the next reference
    // re-engage it from the measured state. A transient, not a fault (owner ruling 2026-10-02).
    spdlog::warn("yaw servo: encoder reading not accepted by the observer; released, re-engages on the next reference");
    release_yaw_servo_locked();
    send_yaw_zero_locked();
    return;
  }
  double q, v, a;
  if (yaw_reference_.at(now, q, v, a)) yaw_servo_hold_q_ = q;
  else { q = yaw_servo_hold_q_; v = a = 0; ++yaw_servo_stale_; }
  const auto out = yaw_servo_.step((now - yaw_servo_epoch_ns_) * 1e-9, q + offset, v, a);
  yaw_servo_out_ = out;
  if (out.status == static_cast<int>(axis::ServoStatus::FollowingError)) {
    // Usually an obstruction: stop pushing (let go), and HOLD until a reference near the axis comes
    // back -- the control loop re-anchors its stop at the measured axis. Not a fault.
    spdlog::error("yaw servo following error: reference {:+.4f} rad, axis {:+.4f} rad; released, HOLD",
                  q, yaw_state_.position_rad);
    yaw_follow_since_ns_ = now;
    yaw_follow_hold_.store("yaw servo following error");
    release_yaw_servo_locked();
    send_yaw_zero_locked();
    return;
  }
  if (out.status != static_cast<int>(axis::ServoStatus::Ok)) { trip_yaw_locked("servo_data_invalid"); return; }
  if (yaw_state_.feedback.temperature_raw >= profile_.servo->yaw_temperature_limit_raw) {
    trip_yaw_locked("servo_temperature");
    return;
  }
  // The owner's safety cap, on the drive's own speed report (independent of the observer).
  if (std::abs(yaw_state_.feedback.speed_rad_s()) > profile_.servo->speed_limit_rad_s) {
    spdlog::error("yaw servo speed trip: {:.3f} rad/s > {:.3f}", yaw_state_.feedback.speed_rad_s(),
                  profile_.servo->speed_limit_rad_s);
    trip_yaw_locked("servo_speed_limit");
    return;
  }
  if (out.rocking) yaw_servo_last_rock_ns_ = now;
  const bool rock_settling = yaw_servo_last_rock_ns_ && now - yaw_servo_last_rock_ns_ < 200'000'000;
  // Station, 2026-10-02 18:33 and 19:10: this guard tripped the station (fault, yaw de-energised)
  // 50 ms into a limit cycle the owner could not even see. Not a hazard: tolerated, reported, and a
  // HOLD only if it persists (track_yaw_servo_episodes_locked).
  // It watches the current the servo did not plan: output minus feedforward (inertia x a_ref +
  // friction). At 21:14 the same day a hard stop's own deceleration current (0.3 A fast RMS)
  // counted as oscillation and held tracking for 10 s; the planned share is not an oscillation.
  yaw_oscillation_.update(yaw_servo_last_step_ns_ ? (now - yaw_servo_last_step_ns_) * 1e-9 : 0.0,
                          out.limited - out.feedforward, rock_settling);
  track_yaw_servo_episodes_locked(now, rock_settling);
  yaw_servo_last_step_ns_ = now;
  yaw_output_reason_ = 1;
  const bool sent = send_yaw_output_locked(
      gm6020::servo_current_frame(profile_.yaw.motor_id, out.limited, profile_.servo->yaw_current_limit_a), out.limited);
  yaw_servo_.acknowledge(sent, sent ? yaw_last_output_.load() : 0.0);
  // Sustained transmit failure is the guard's (can_down after 20 ms, a fault only if it persists).
}

// The pitch servo: commissioning's host position loop at 1 kHz on the drive's own speed loop.
// Every SpdRef write is answered by a type-2 frame, so the loop steps on 1 kHz feedback.
void MixedCanMotorBackend::pitch_servo_loop(std::stop_token stop) {
  auto next = std::chrono::steady_clock::now();
  while (!stop.stop_requested()) {
    next += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next);
    if (pitch_stale_since_ns_) {
      // Late pitch feedback (above): cleared when it is fresh again, a fault if it lasts.
      can::AxisLatest s;
      const auto t = now_monotonic_ns();
      if (pitch_system_.axis(AxisId::Pitch).latest(s) && s.has_feedback && t - s.rx_ns <= kPitchFreshNs) {
        spdlog::info("pitch servo: feedback fresh again after {:.1f} ms", (t - pitch_stale_since_ns_) / 1e6);
        pitch_stale_since_ns_ = 0;
      } else if (t - pitch_stale_since_ns_ >= kPitchStaleFaultNs) {
        spdlog::error("pitch servo: no fresh feedback for {:.0f} ms; fault (the drive keeps holding speed zero)",
                      (t - pitch_stale_since_ns_) / 1e6);
        pitch_servo_fault_.store(true);
        pitch_stale_since_ns_ = 0;
        pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0);
      }
    }
    if (!pitch_servo_active_.load()) continue;
    std::lock_guard lock(pitch_servo_mutex_);
    if (!pitch_servo_active_.load()) continue;
    const auto now = now_monotonic_ns();
    can::AxisLatest l;
    const bool have = pitch_system_.axis(AxisId::Pitch).latest(l) && l.has_feedback;
    if (!have || l.mode != 2 || l.faults) {
      // The drive is not running or reports its own fault: it is not holding. A hazard -> fault.
      spdlog::error("pitch servo: drive disabled or faulted (feedback={} mode={} faults={})", have, l.mode, l.faults);
      pitch_servo_fault_.store(true);
      pitch_servo_active_.store(false);
      pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0);
      continue;
    }
    if (now - l.rx_ns > kPitchFreshNs) {
      // Late feedback is a transient (owner ruling 2026-10-02): the drive holds speed zero on its own
      // encoder while the servo lets go; it re-engages through the next reference once feedback is
      // fresh. A fault only if it lasts kPitchStaleFaultNs (pitch_stale_watch below).
      spdlog::warn("pitch servo: feedback {:.1f} ms old; released to the drive's speed-zero hold", (now - l.rx_ns) / 1e6);
      pitch_stale_since_ns_ = l.rx_ns;
      pitch_servo_active_.store(false);
      pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0);
      continue;
    }
    double q, v, a;
    if (pitch_reference_.at(now, q, v, a)) pitch_hold_q_ = q;
    else { q = pitch_hold_q_; v = a = 0; }
    // End-stop protection, independent of the reference (owner: never into the end stop).
    const bool bounded = pitch_reference_.q_max > pitch_reference_.q_min;
    const double guard = profile_.servo->pitch_guard_rad;
    const double low = pitch_reference_.q_min - guard, high = pitch_reference_.q_max + guard;
    if (!bounded || l.q_rad < low || l.q_rad > high) {
      spdlog::error("pitch servo end-stop guard: axis {:+.4f} rad outside [{:+.4f}, {:+.4f}]{}", l.q_rad, low, high,
                    bounded ? "" : " (no envelope given)");
      pitch_servo_fault_.store(true);
      pitch_servo_active_.store(false);
      pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0);
      continue;
    }
    if (std::abs(q - l.q_rad) > pitch_following_error_rad_) {
      // Usually an obstruction: stop pushing (the drive holds speed zero) and HOLD until a reference
      // near the axis comes back. Not a fault (owner ruling 2026-10-02).
      spdlog::error("pitch servo following error: reference {:+.4f} rad, axis {:+.4f} rad; released, HOLD", q, l.q_rad);
      pitch_follow_since_ns_ = now;
      pitch_follow_hold_.store("pitch servo following error");
      pitch_servo_active_.store(false);
      pitch_backend_.command_velocity_always(AxisId::Pitch, 0.0);
      continue;
    }
    const double dt = pitch_servo_last_step_ns_ ? (now - pitch_servo_last_step_ns_) * 1e-9 : 0.0;
    const double limit = pitch_loop_.parameters().speed_limit;
    const double speed = axis::travel_governor(std::clamp(pitch_loop_.step(dt, q, v, l.q_rad), -limit, limit), l.q_rad,
                                               low, high, profile_.servo->pitch_stop_acceleration_rad_s2);
    pitch_servo_last_step_ns_ = now;
    pitch_backend_.command_velocity_always(AxisId::Pitch, speed);
  }
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
