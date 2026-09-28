// Bounded actual-transport commissioning; launcher owns this process.
// Default: GM receive only plus CyberGear discovery and read-only mechPos query.
// Explicit --yaw-current-a (or --yaw-voltage, on a voltage-mode profile) enables a short pulse.
// The mode is declared in the probe config, never guessed: a drive whose current ring is on
// ignores a voltage frame, and a probe that reported "no motion" from that would be a lie about
// the mechanic (2026-09-28 spent an evening exactly there).
#include <atomic>
#include <algorithm>
#include <chrono>
#include <charconv>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numbers>
#include <thread>
#include <string_view>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "can/cybergear_protocol.hpp"
#include "can/gm6020_protocol.hpp"
#include "can/gm6020_velocity.hpp"
#include "can/socketcan_bus.hpp"
#include "can/pitch_current_policy.hpp"

using namespace std::chrono_literals;
static volatile std::sig_atomic_t interrupted = 0;
static void stop_signal(int) { interrupted = 1; }
static constexpr double degrees = 180.0 / std::numbers::pi;

static int integer(const std::string& value) {
  int result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    throw std::runtime_error("invalid integer: " + value);
  return result;
}

// Amperes arrive as text like "0.25" or "-.3". Strict: trailing junk is a mistake, not a rounding.
static double number(const std::string& value) {
  double result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    throw std::runtime_error("invalid number: " + value);
  if (!std::isfinite(result)) throw std::runtime_error("non-finite number: " + value);
  return result;
}

struct ProbeOwnership {
  int fd{-1};
  ProbeOwnership() {
    const auto path = "/tmp/ota-mixed-can-" + std::to_string(getuid()) + ".lock";
    fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("cannot open commissioning ownership lock");
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd); fd = -1;
      throw std::runtime_error("another mixed-CAN probe owns the station");
    }
  }
  ~ProbeOwnership() { if (fd >= 0) ::close(fd); }
};

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
  int speed_reference_deg_s = 0;
  double current_a = 0;
  // Drag sweep: drag the axis a signed number of degrees under speed regulation, with an optional
  // constant torque-current feedforward. The trace is the deliverable; the map is computed offline.
  int sweep_deg = 0;
  double sweep_ff_a = 0;
  bool apply_pitch_limit = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--apply-pitch-limit") { apply_pitch_limit = true; continue; }
    if (i + 1 >= argc) throw std::runtime_error("option requires a value: " + arg);
    const std::string value = argv[++i];
    if (arg == "--config") config = value;
    else if (arg == "--yaw-voltage") voltage = integer(value);
    else if (arg == "--yaw-current-a") current_a = number(value);
    else if (arg == "--yaw-speed-deg-s") speed_reference_deg_s = integer(value);
    else if (arg == "--yaw-sweep-deg") sweep_deg = integer(value);
    else if (arg == "--yaw-sweep-ff-a") sweep_ff_a = number(value);
    else if (arg == "--pulse-ms") pulse_ms = integer(value);
    else if (arg == "--observe-ms") observe_ms = integer(value);
    else if (arg == "--trace") trace_path = value;
    else throw std::runtime_error("unknown option: " + arg);
  }
  const auto cfg = YAML::LoadFile(config);
  if (cfg["schema_version"].as<int>() != 1) throw std::runtime_error("unknown probe schema");
  // Declared, not assumed. The probe used to have one kind of output and no way to say so; after
  // the drive was switched to the current ring, an undeclared default would have quietly turned
  // every pulse into a no-op and every result into "the axis does not move".
  const std::string yaw_mode_text = cfg["yaw"]["control_mode"].as<std::string>("");
  const bool yaw_current = yaw_mode_text == "current";
  if (!yaw_current && yaw_mode_text != "voltage")
    throw std::runtime_error("probe config must declare yaw.control_mode as 'current' or 'voltage'");
  const auto limit = cfg["limits"];
  const int voltage_limit = limit["voltage_raw"].as<int>();
  const double current_limit = limit["yaw_current_a"].as<double>();
  const double loop_kp_a = limit["current_kp_a_per_rad_s"].as<double>();
  const double loop_ki_a = limit["current_ki_a_per_rad_s"].as<double>();
  const int duration_limit = limit["pulse_ms"].as<int>();
  const double travel_limit = limit["yaw_travel_deg"].as<double>();
  const double speed_limit = limit["yaw_speed_deg_s"].as<double>();
  const double age_limit = limit["feedback_age_ms"].as<double>();
  const double heartbeat_limit = limit["heartbeat_age_ms"].as<double>();
  // The drag sweep gets its own envelope instead of borrowing the pulse-shaped one: it runs for
  // tens of seconds and hundreds of degrees, and "how hard may a 500 ms pulse push" is a different
  // approval from "how long may we take a drag map". One key answering both would be a lie in one
  // of the two directions. Absence is a named refusal, not a default that nobody chose.
  auto need = [&](const char* key) {
    const auto node = limit[key];
    if (!node) throw std::runtime_error(std::string("probe config is missing limits.") + key);
    return node;
  };
  const double sweep_travel_cap = need("sweep_travel_deg").as<double>();
  const int sweep_duration_cap = need("sweep_duration_ms").as<int>();
  const double sweep_speed_cap = need("sweep_speed_deg_s").as<double>();
  const double sweep_ff_cap = need("sweep_ff_a_max").as<double>();
  const bool sweeping = sweep_deg != 0;
  const int pulse_cap = sweeping ? sweep_duration_cap : duration_limit;
  const double speed_cap = sweeping ? sweep_speed_cap : 5.0;
  if (voltage_limit < 1 || voltage_limit > 3000 || voltage < -voltage_limit || voltage > voltage_limit ||
      // A first probe asks for much less than the station's own envelope, and the probe's ceiling
      // is the smaller of the two: this tool must not be the place where 0.8 A becomes 3 A.
      !std::isfinite(current_limit) || current_limit <= 0 || current_limit > ota::gm6020::kMaxContinuousA ||
      !std::isfinite(current_a) || current_a < -current_limit || current_a > current_limit ||
      !std::isfinite(loop_kp_a) || loop_kp_a <= 0 || loop_kp_a > 10.0 ||
      !std::isfinite(loop_ki_a) || loop_ki_a < 0 || loop_ki_a > 20.0 ||
      // One kind of push per run: --yaw-voltage is not a spelling of --yaw-current-a, and letting
      // both through would let a stale launcher flag decide which frame the motor never saw.
      (yaw_current && voltage != 0) || (!yaw_current && current_a != 0) ||
      duration_limit < 1 || duration_limit > 500 || pulse_ms < 1 || pulse_ms > pulse_cap ||
      !std::isfinite(travel_limit) || travel_limit <= 0 || travel_limit > 5 ||
      !std::isfinite(speed_limit) || speed_limit <= 0 || speed_limit > 20 ||
      !std::isfinite(age_limit) || age_limit <= 0 || age_limit > 20 ||
      !std::isfinite(heartbeat_limit) || heartbeat_limit <= 0 || heartbeat_limit > 40 ||
      // Sweep envelope: 400 degrees so a requested 360 plus settle is allowed, two minutes so a
      // stalled sweep still ends by itself, and the feedforward may not out-push the ceiling.
      !std::isfinite(sweep_travel_cap) || sweep_travel_cap <= 0 || sweep_travel_cap > 400 ||
      sweep_duration_cap < 1000 || sweep_duration_cap > 120000 ||
      !std::isfinite(sweep_speed_cap) || sweep_speed_cap <= 0 || sweep_speed_cap > 40 ||
      !std::isfinite(sweep_ff_cap) || sweep_ff_cap <= 0 || sweep_ff_cap > ota::gm6020::kMaxContinuousA ||
      (sweeping && (!yaw_current || speed_reference_deg_s == 0 || sweep_ff_a < 0 ||
                    std::abs(sweep_deg) > sweep_travel_cap || sweep_ff_a > sweep_ff_cap)) ||
      observe_ms < 1000 || observe_ms > 10000) {
    std::string why = "probe limits outside fixed commissioning envelope";
    if (yaw_current && voltage != 0)
      why = "--yaw-voltage is a voltage-mode option; this probe profile commands torque current, "
            "use --yaw-current-a (max " + std::to_string(current_limit) + " A)";
    if (!yaw_current && current_a != 0)
      why = "--yaw-current-a requires yaw.control_mode: current in the probe config";
    if (sweeping && !yaw_current)
      why = "a drag sweep is a torque-current measurement; this probe profile is not in current mode";
    if (sweeping && speed_reference_deg_s == 0)
      why = "--yaw-sweep-deg drags the axis under speed regulation; add --yaw-speed-deg-s";
    if (sweeping && sweep_ff_a < 0)
      why = "--yaw-sweep-ff-a is a magnitude; the direction comes from the sign of --yaw-sweep-deg";
    if (sweeping && (std::abs(sweep_deg) > sweep_travel_cap || sweep_ff_a > sweep_ff_cap))
      why = "sweep outside the configured envelope (max " + std::to_string(sweep_travel_cap) +
            " deg, feedforward up to " + std::to_string(sweep_ff_cap) + " A)";
    throw std::runtime_error(why);
  }
  const auto yaw_id = cfg["yaw"]["motor_id"].as<int>();
  const auto pitch_id = cfg["pitch"]["motor_id"].as<int>();
  if (yaw_id < 1 || yaw_id > 7 || pitch_id < 1 || pitch_id > 255)
    throw std::runtime_error("invalid motor ID");
  // 0x1FE carries IDs 1-4 in fixed slots and this station has exactly one GM6020. A probe that
  // threw mid-run because somebody moved a DIP switch would leave the guard thread to clean up --
  // survivable, but refusing before arming is the difference between a message and a shrug.
  if (yaw_current && yaw_id != 1)
    throw std::runtime_error("current mode is qualified for GM6020 ID 1 only, not ID " +
                             std::to_string(yaw_id));
  const auto expected_uid = std::stoull(cfg["pitch"]["unique_id_hex"].as<std::string>(), nullptr, 16);
  const auto pitch_limit = cfg["pitch"]["current_limit_a"].as<double>();
  if (!ota::can::valid_pitch_current_limit(pitch_limit))
    throw std::runtime_error("pitch current limit must be positive and at most 5 A");
  if (speed_reference_deg_s < -speed_cap || speed_reference_deg_s > speed_cap ||
      (speed_reference_deg_s != 0 && (voltage != 0 || current_a != 0)))
    throw std::runtime_error("velocity probe requires integer target within +/-" +
                             std::to_string(static_cast<int>(speed_cap)) +
                             " deg/s and no fixed open-loop push");
  const bool yaw_motion = voltage != 0 || current_a != 0 || speed_reference_deg_s != 0;
  if (apply_pitch_limit && yaw_motion)
    throw std::runtime_error("pitch limit setup must run without yaw actuation");
  const auto bitrate = cfg["bitrate"].as<uint32_t>();
  if (bitrate != 1000000) throw std::runtime_error("expected classical CAN 1 Mbps");
  if (cfg["yaw"]["interface"].as<std::string>() == cfg["pitch"]["interface"].as<std::string>())
    throw std::runtime_error("yaw and pitch require separate interfaces");
  ProbeOwnership ownership;
  std::ofstream trace;
  if (!trace_path.empty()) {
    trace.open(trace_path);
    if (!trace) throw std::runtime_error("cannot open trace");
    trace << "time_ns,phase,drive_output,angle_count,yaw_relative_deg,speed_deg_s,current_raw,temperature_raw,feedback_age_ms,estimated_speed_deg_s\n";
  }
  std::signal(SIGTERM, stop_signal); std::signal(SIGINT, stop_signal);
  std::mutex sample_mutex;
  Sample sample;
  ota::gm6020::UnwrappedEncoder encoder;
  std::atomic<uint64_t> uid{0}, pitch_frames{0};
  std::atomic<bool> pitch_position_valid{false};
  std::atomic<unsigned> pitch_register_status{255};
  std::atomic<double> pitch_position{0};
  std::mutex pitch_mutex;
  uint64_t limit_replies = 0, feedback_replies = 0;
  double limit_readback = 0;
  bool limit_readback_valid = false;
  ota::cybergear::Feedback pitch_feedback{};
  ota::TimeNs pitch_feedback_time = 0;
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
    if ((ident.data2 & 0xff) == pitch_id && ident.target == 0) {
      std::lock_guard lock(pitch_mutex);
      ota::cybergear::Feedback decoded;
      if (ota::cybergear::parse_feedback(frame, decoded)) {
        pitch_feedback = decoded; pitch_feedback_time = f.rx_ns; ++feedback_replies;
      }
      if (ident.comm_type == static_cast<uint8_t>(ota::cybergear::CommType::ReadReg) &&
          f.data[0] == 0x18 && f.data[1] == 0x70) {
        ota::cybergear::Reg returned;
        limit_readback_valid = ota::cybergear::parse_reg_response(frame, returned, limit_readback) &&
                              returned == ota::cybergear::Reg::LimitCur && std::isfinite(limit_readback);
        ++limit_replies;
      }
    }
    if (ident.comm_type == static_cast<uint8_t>(ota::cybergear::CommType::ReadReg) &&
        (ident.data2 & 0xff) == pitch_id && ident.target == 0 &&
        f.data[0] == 0x19 && f.data[1] == 0x70)
      pitch_register_status.store(ident.data2 >> 8);
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
    if (!bus.open(options, error) || !bus.is_up() || bus.bitrate() != bitrate ||
        bus.can_state() != ota::can::CanIfState::ErrorActive || !bus.start_rx(error))
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
  // Potentially blocking diagnostics happen before arming, never in the pulse loop.
  for (auto* bus : {&yaw, &pitch}) {
    if (!bus->refresh_health(&error) || !bus->is_up() || bus->bitrate() != bitrate ||
        bus->can_state() != ota::can::CanIfState::ErrorActive)
      throw std::runtime_error("CAN health invalid before arming: " + bus->iface() + " " + error);
  }
  auto read = [&] { std::lock_guard lock(sample_mutex); return sample; };
  const auto baseline = read();
  if (uid.load() != expected_uid || !baseline.valid || baseline.count < 100 ||
      (ota::now_monotonic_ns() - baseline.feedback.rx_ns) * 1e-6 > age_limit ||
      std::abs(baseline.feedback.speed_rad_s() * degrees) > 1.0)
    throw std::runtime_error("identity/freshness/stationary baseline failed before any yaw output");
  std::cout << "IDENTIFIED yaw_standard_id=0x" << std::hex << 0x204 + yaw_id
            << " pitch_uid=0x" << uid.load() << std::dec
            << " pitch_position_valid=" << pitch_position_valid.load()
            << " pitch_register_status=" << pitch_register_status.load()
            << " yaw_samples=" << baseline.count << std::endl;
  if (pitch_position_valid.load()) std::cout << "PITCH mech_pos_rad=" << pitch_position.load() << std::endl;
  else std::cout << "PITCH position unavailable; pitch actuation is not supported by this probe" << std::endl;
  if (apply_pitch_limit) {
    // Only lower/set the volatile position/speed current limit; never enable,
    // change run mode, command a target or restore an old (>5 A) limit.
    const auto setting = ota::cybergear::make_write_reg_float(ota::cybergear::Reg::LimitCur,
        static_cast<float>(pitch_limit), 0, static_cast<uint8_t>(pitch_id));
    const auto query = ota::cybergear::make_read_reg(ota::cybergear::Reg::LimitCur, 0,
        static_cast<uint8_t>(pitch_id));
    for (int check = 0; check < 3; ++check) {
      if (interrupted) throw std::runtime_error("pitch setup interrupted; motion remains disabled");
      if (check == 0 && !pitch.send(setting.id, setting.data, &error)) throw std::runtime_error(error);
      std::this_thread::sleep_for(50ms);
      uint64_t before;
      { std::lock_guard lock(pitch_mutex); before = limit_replies; }
      if (!pitch.send(query.id, query.data, &error)) throw std::runtime_error(error);
      const auto deadline = std::chrono::steady_clock::now() + 200ms;
      bool matched = false;
      while (!interrupted && std::chrono::steady_clock::now() < deadline) {
        {
          std::lock_guard lock(pitch_mutex);
          if (limit_replies != before) {
            matched = limit_readback_valid && ota::can::valid_pitch_current_limit(limit_readback) &&
                      std::abs(limit_readback - pitch_limit) < 1e-6;
            break;
          }
        }
        std::this_thread::sleep_for(2ms);
      }
      if (!matched) throw std::runtime_error("pitch 5 A ceiling not verified; enabling is prohibited");
    }
    std::lock_guard lock(pitch_mutex);
    std::cout << "PITCH_LIMIT verified_a=" << limit_readback << " readbacks=3 volatile=1" << std::endl;
    if (feedback_replies && ota::now_monotonic_ns() - pitch_feedback_time < 500000000) {
      std::cout << "PITCH_FEEDBACK mode=" << int(pitch_feedback.mode)
                << " faults=" << pitch_feedback.faults << " angle_rad=" << pitch_feedback.angle_rad
                << " raw_angle=" << pitch_feedback.raw_angle << " velocity_rad_s=" << pitch_feedback.vel_rad_s
                << " temperature_c=" << pitch_feedback.temp_c << std::endl;
    } else std::cout << "PITCH_FEEDBACK unavailable" << std::endl;
    std::cout << "PITCH_SETUP no_enable_no_motion_command=1; limit applies only to position/speed modes" << std::endl;
  }
  // The tool's own cleanup frame: the same "zero output, not de-energised" claim the station makes,
  // and it must be a zero the drive actually listens to -- a 0x1FF zero sent to a drive on the
  // current ring stops nothing.
  const auto zero = yaw_current
      ? ota::gm6020::current_zero_frame(static_cast<uint8_t>(yaw_id))
      : ota::gm6020::voltage_frame(static_cast<uint8_t>(yaw_id), 0);
  ota::gm6020::VelocityLoop velocity_loop;
  velocity_loop.reset(baseline.position, ota::now_monotonic_ns());
  std::atomic<ota::TimeNs> heartbeat{ota::now_monotonic_ns()};
  std::atomic<bool> trip{false}, zero_failed{false};
  std::mutex command_mutex;
  const auto start = ota::now_monotonic_ns();
  const auto pulse_deadline = start + uint64_t(pulse_ms) * 1000000;
  // Independent of the probe loop, but not independent of this process/OS.
  std::jthread guard;
  if (yaw_motion) guard = std::jthread([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      {
        std::lock_guard lock(command_mutex);
        if (interrupted || (ota::now_monotonic_ns() - heartbeat.load()) * 1e-6 > heartbeat_limit) trip.store(true);
        if ((trip.load() || ota::now_monotonic_ns() >= pulse_deadline) && !yaw.send_frame(zero)) zero_failed.store(true);
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
  auto next = std::chrono::steady_clock::now();
  double peak_speed = 0, peak_travel = 0;
  bool stopped = false;
  ota::TimeNs still_since = 0;
  const char* reason = "completed";
  // Breakaway assist: static drag on a crossed-roller bearing is larger than the drag once the
  // axis is turning, so a sweep that only asks the integrator for current spends seconds winding
  // up before it can measure anything. The feedforward is a magnitude whose sign follows the drag
  // direction, so a negative sweep cannot quietly fight its own PI.
  const double sweep_ff_signed =
      sweeping ? (speed_reference_deg_s < 0 ? -sweep_ff_a : sweep_ff_a) : 0.0;
  for (;;) {
    const auto current = read();
    const auto now = ota::now_monotonic_ns();
    const auto elapsed_ms = (now - start) * 1e-6;
    const double age_ms = (now - current.feedback.rx_ns) * 1e-6;
    const double speed = std::abs(current.feedback.speed_rad_s() * degrees);
    const double travel = std::abs(current.position - baseline.position) * degrees;
    peak_speed = std::max(peak_speed, speed); peak_travel = std::max(peak_travel, travel);
    if (interrupted) { trip.store(true); reason = "interrupted"; }
    if (!current.valid || age_ms > age_limit) { trip.store(true); reason = "feedback_invalid_or_stale"; }
    if (yaw.stats().rx_error_frames || pitch.stats().rx_error_frames) { trip.store(true); reason = "CAN_error_frame"; }
    // A drag sweep measures torque, not speed, and the owner's ruling on 09-29 is that nothing
    // shaped like a motion limit may end one: "this is a torque test, not a speed test", and
    // anything that can fail the test has to come out. So while sweeping there is no speed cap and
    // no travel guard — the requested angle is the whole point of the run, and a breakaway is
    // allowed to be violent. What is still able to stop a sweep is not a safety margin but the
    // measurement's own validity, three lines above: if the drive stopped answering, the map we
    // are drawing is fiction and the PI is integrating a position we no longer have.
    if (!sweeping && (speed > speed_limit || travel > travel_limit)) {
      trip.store(true); reason = "speed_or_travel_guard";
    }
    if (trip.load() && std::string_view(reason) == "completed") reason = "heartbeat_guard";
    bool active = false;
    // One number, two units: raw counts in voltage mode, amperes in current mode. The frame
    // builder below is the only thing that decides which one it encodes as.
    double applied_output = 0;
    heartbeat.store(now);
    {
      std::lock_guard lock(command_mutex);
      const auto send_time = ota::now_monotonic_ns();
      if ((send_time - current.feedback.rx_ns) * 1e-6 > age_limit) { trip.store(true); reason = "feedback_invalid_or_stale"; }
      active = yaw_motion && send_time < pulse_deadline && !trip.load();
      // The loop keeps running even while inactive, exactly as the voltage path did: its velocity
      // estimate must not go stale just because this cycle asked for nothing.
      const double reference =
          (speed_reference_deg_s && active) ? speed_reference_deg_s / degrees : 0.0;
      const int regulated_counts = (speed_reference_deg_s && !yaw_current)
          ? velocity_loop.update(reference, current.position, send_time) : 0;
      const double regulated_amps = (speed_reference_deg_s && yaw_current)
          ? velocity_loop.update_amps(reference, current.position, send_time,
                                      speed_cap / degrees, current_limit, loop_kp_a, loop_ki_a)
          : 0.0;
      if (active) {
        applied_output = yaw_current
            ? std::clamp(speed_reference_deg_s ? regulated_amps + sweep_ff_signed : current_a,
                         -current_limit, current_limit)
            : std::clamp(static_cast<double>(speed_reference_deg_s ? regulated_counts : voltage),
                         static_cast<double>(-voltage_limit), static_cast<double>(voltage_limit));
        if (speed_reference_deg_s && !velocity_loop.valid()) {
          trip.store(true); active = false; applied_output = 0; reason = "velocity_loop_invalid";
        }
      }
      const auto command = yaw_current
          ? ota::gm6020::current_frame(static_cast<uint8_t>(yaw_id), applied_output, current_limit)
          : ota::gm6020::voltage_frame(static_cast<uint8_t>(yaw_id),
                                       static_cast<int>(applied_output));
      if (yaw_motion && !yaw.send_frame(active ? command : zero)) { trip.store(true); reason = "tx_failed"; }
    }
    if (trace) trace << now << ',' << (active ? "pulse" : "observe") << ',' << applied_output
                     << ',' << current.feedback.angle_count << ',' << current.position * degrees << ','
                     << current.feedback.speed_rad_s() * degrees << ',' << current.feedback.current_raw << ','
                     << int(current.feedback.temperature_raw) << ',' << age_ms << ',' << velocity_loop.velocity_rad_s() * degrees << '\n';
    if (!active && current.valid && age_ms <= age_limit && speed <= 1.0) {
      if (!still_since) still_since = now;
      stopped = now - still_since >= 250000000;
    } else { still_since = 0; stopped = false; }
    // A sweep ends when it has dragged the requested angle, or the moment anything trips: the
    // guard is already forcing zeros, and sitting out the rest of the window watching zeros cannot
    // teach us anything the trace has not already written down.
    if (sweeping && (trip.load() || travel >= std::abs(static_cast<double>(sweep_deg)))) {
      if (!trip.load()) reason = "sweep_target_reached";
      break;
    }
    if (elapsed_ms >= pulse_ms + observe_ms || (trip.load() && stopped)) break;
    next += 5ms; std::this_thread::sleep_until(next);
  }
  if (yaw_motion) {
    // Retire the guard before the trailing burst, not after it. The guard's trip condition is a
    // stale feedback heartbeat, and only the observation loop refreshes that heartbeat: while this
    // 100 ms burst ran with the loop already finished, every yaw_motion run tripped on its own
    // cleanup and exited 2 no matter what the drive did. A gate that cannot go green is not a
    // gate, and the reason field could no longer be renamed, so the failure was also nameless.
    guard.request_stop();
    if (guard.joinable()) guard.join();
    for (int i = 0; i < 20; ++i) {
      if (!yaw.send_frame(zero)) zero_failed.store(true);
      std::this_thread::sleep_for(5ms);
    }
  }
  guard.request_stop(); if (guard.joinable()) guard.join();
  const auto final = read();
  const auto yaw_stats = yaw.stats(); const auto pitch_stats = pitch.stats();
  yaw.close(); pitch.close();
  const std::string sweep_text = sweeping
      ? " sweep_target_deg=" + std::to_string(sweep_deg) +
        " sweep_ff_a=" + std::to_string(sweep_ff_a)
      : "";
  std::cout << "RESULT reason=" << reason << sweep_text
            << (yaw_current ? " yaw_mode=current yaw_current_a=" : " yaw_mode=voltage yaw_voltage=")
            << (yaw_current ? current_a : voltage)
            << " current_ceiling_a=" << current_limit
            << " yaw_speed_target_deg_s=" << speed_reference_deg_s
            << " peak_speed_deg_s=" << peak_speed << " peak_travel_deg=" << peak_travel
            << " final_displacement_deg=" << (final.position - baseline.position) * degrees
            << " yaw_frames=" << final.count << " yaw_tx=" << yaw_stats.tx_frames
            << " pitch_replies=" << pitch_frames.load() << " pitch_tx=" << pitch_stats.tx_frames
            << " stationary_observed=" << stopped << " zero_tx_failed=" << zero_failed.load()
            << " yaw_errors=" << yaw_stats.rx_error_frames << " pitch_errors=" << pitch_stats.rx_error_frames
            << std::endl;
  // This is deliberately not the production PARKED/de-energized marker.
  std::cout << "COMMISSIONING FINISHED; " << (yaw_motion ? "zero output requested; disabled state unavailable" :
      apply_pitch_limit ? "pitch current limit verified; no motion command" : "receive/discovery only") << std::endl;
  return trip.load() || !stopped || zero_failed.load() ? 2 : 0;
 } catch (const std::exception& error) {
   std::cerr << "Probe refused: " << error.what() << std::endl; return 1;
 }
}
