// controld — sole owner of the station motor CAN links (architecture §4.1).
//
// The Phase-2 daemon. Sequence (§27):
//   load+validate config -> open CAN -> boot FSM (discover + self-test)
//   -> execute homing plan -> VALIDATE_TRAVEL -> [camera/installation/payload
//      are Phase-2 stubs] -> move to the safe ready pose and HOLD
//   -> on SIGINT/SIGTERM: safe park (§33) then de-energize.
//
// Safety model: every command passes the SafetySupervisor (the per-cycle
// safety gate, §38). The daemon NEVER tracks in Phase 2 (tracking_enabled is
// false); it homes, holds the safe ready pose, and parks on shutdown. Stale
// feedback brakes (recoverable); a motor fault / over-temp disables (sticky).
// There is no open-loop path.
//
// §46 loop discipline: the 200 Hz control loop does no file I/O, no HTTP, no
// blocking video, no synchronous register-query chain, and no unbounded
// allocation. The only slow (blocking) paths are boot-only (discovery,
// register reads) and the one-time enter-position-mode transition.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "calibration/camera_calibration.hpp"
#include "config/tracking_setup.hpp"
#include "calibration/homing_plan.hpp"
#include "calibration/retained_homing.hpp"
#include "calibration/park_controller.hpp"
#include "can/cybergear_system.hpp"
#include "common/thread_class.hpp"
#include "common/time.hpp"
#include "common/timing_stats.hpp"
#include "common/types.hpp"
#include "config/station_wiring.hpp"
#include "config/mixed_hardware_profile.hpp"
#include "config/turret_config.hpp"
#include "control/runtime_parameter_registry.hpp"
#include "control/boot_fsm.hpp"
#include "control/can_motor_backend.hpp"
#include "control/mixed_can_motor_backend.hpp"
#include "control/control_loop.hpp"
#include "control/imu_trace_ingest.hpp"
#include "payload/payload_profile.hpp"
#include "sim/sim_motor_backend.hpp"
#include "vision/vision_ingest.hpp"
#include "web/web_server.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <filesystem>

using namespace ota;

namespace {

// §58/§72: the mapping lives in config/station_wiring.cpp now, shared with
// tools/replay_session.cpp. Unqualified calls below still read the way they did.
using namespace ota::wire;

std::atomic<bool> g_shutdown{false};
void on_signal(int) { g_shutdown.store(true); }

// Cycle longer than this is logged as a stall (the 200 Hz period is 5 ms; the
// supervisor's own overrun grace is deadline_max_us = 2 ms).
constexpr TimeNs kSlowCycleNs = 8'000'000;

// §46 loop discipline: "the 200 Hz control loop does no file I/O". The control
// thread's log calls (the 100 Hz homing motion log + the supervisor/phase
// events) must therefore never touch the storage stack. The default spdlog
// logger is SYNCHRONOUS: the writing thread blocks in write(2), and on this
// station's storage a flush was measured to stall the control thread for
// ~0.8-1.1 s (sim run 2026-09-03, DERATE 'control-loop cycle overrun'
// overrun_us=1074018 — on the real bus the same stall ages the feedback past
// feedback_max_age_ms and the supervisor Brakes: the recurring ~98 ms
// Brake/Allow flap noted in P3/P4). An ASYNC logger with a BOUNDED queue and
// the NON-BLOCKING (overrun-oldest) policy moves every byte off the control
// thread and makes dropping a log line, rather than stalling a cycle, the
// failure mode.
void init_async_logging() {
  try {
    // Bounded queue (8192 messages) + ONE background writer, and the
    // non-blocking factory: when the queue is full the PRODUCER drops the
    // oldest message instead of waiting for storage.
    spdlog::init_thread_pool(8192, 1, [] { apply_thread_class("log-writer", ThreadClass::Background); });
    auto lg = spdlog::create_async_nb<spdlog::sinks::stdout_color_sink_mt>(
        "controld");
    lg->set_level(spdlog::level::info);
    lg->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(std::move(lg));
  } catch (const std::exception& e) {
    // Never let the logging setup decide whether the station runs: fall back
    // to the default (synchronous) logger and say so.
    spdlog::warn("async logger unavailable ({}); using the default logger",
                 e.what());
  }
}

// Offline/HIL bring-up mode (§54): the plant is SimMotorBackend, the process
// never opens a CAN transport at all, and no real motor can move. Used for the
// P8/P12 bring-up probes and for CI. Loudly announced, and it must be the
// first argument so it can never be smuggled in by a config file.
bool has_flag(int argc, char** argv, const char* name) {
  for (int i = 2; i < argc; ++i)
    if (std::strcmp(argv[i], name) == 0) return true;
  return false;
}

// `--flag <value>`, or "" when the flag is absent. The dump paths use it so that a machine reading
// the registry gets a file it can open, instead of having to pick the document out of whatever the
// logger happened to print on the same stream.
std::string flag_value(int argc, char** argv, const char* name) {
  for (int i = 2; i + 1 < argc; ++i)
    if (std::strcmp(argv[i], name) == 0) return argv[i + 1];
  return "";
}

// The sim plant sized to THIS station's measured geometry (P0/P3: pitch travel
// 79.5 deg at raw -2.1994..-0.8112; yaw 352.7 deg at raw -5.3458..+0.8104), so
// homing, the logical frame, and the soft limits behave like the real one.
std::unique_ptr<sim::SimMotorBackend> make_sim_backend() {
  auto sb = std::make_unique<sim::SimMotorBackend>();
  sb->set_stops(AxisId::Pitch, -2.1994, -0.8112);
  sb->set_stops(AxisId::Yaw, -5.3458, 0.8104);
  sb->set_position(AxisId::Pitch, -1.50);
  sb->set_position(AxisId::Yaw, -2.30);
  return sb;
}


// turret.yaml `tracking:` block + the §28.2/§28.3 calibration files -> the
// TrackingController configuration (Part 2, S1). Missing calibration files are
// NON-fatal: the aligned defaults stand and the boot log says UNCALIBRATED, so
// the geometry is never silently pretended to be known (Part 3, items 14/15).
TrackingController::Config make_tracking_cfg(const config::TurretConfig& cfg) {
  config::TrackingSetupDiagnostics diagnostics;
  auto t = config::make_tracking_config(cfg, &diagnostics);
  if (diagnostics.intrinsics.found)
    spdlog::info("camera intrinsics: {} ({})", diagnostics.intrinsics.detail, cfg.camera.intrinsics_file);
  else
    spdlog::warn("camera intrinsics: {} ({}) — UNCALIBRATED", diagnostics.intrinsics.detail,
                 cfg.camera.intrinsics_file);
  spdlog::info("camera extrinsics: {} ({})", diagnostics.extrinsics, cfg.camera.extrinsics_file);
  spdlog::info("aim point policy: {}", tracking::aim_mode_name(t.aim.mode));
  if (t.aim.mode == tracking::AimMode::BoxFraction)
    spdlog::info("measurement point: {:.0f}% right, {:.0f}% down inside box",
                 t.aim.x_fraction*100, t.aim.y_fraction*100);
  else if (t.aim.mode == tracking::AimMode::Legacy)
    spdlog::info("legacy aim: native anchor; legacy head override={} at {:.0f}% from top",
                 t.aim.aim_at_head, t.aim.head_fraction_from_top*100);
  const auto alignment = geo::bore_alignment(t.alignment, t.intrinsics);
  spdlog::info("bore alignment: {}, assumed depth={} m (no range measurement)",
      alignment.reason, t.alignment.assumed_depth_m);
  return t;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  const std::string config_path = (argc > 1) ? argv[1] : "config/turret.yaml";
  const bool sim_mode = has_flag(argc, argv, "--sim");
  init_async_logging();
  spdlog::set_level(spdlog::level::info);
  spdlog::info("controld starting (config: {})", config_path);
  if (sim_mode)
    spdlog::warn(
        "*** SIM MODE: SimMotorBackend — this process will NOT open any CAN "
        "transport and no real motor can move. No real-station test (P#) is "
        "satisfied by a sim run. ***");

  // 1. Load + validate config (a hard error blocks boot).
  config::LoadResult lr = config::load_turret_config(config_path);
  if (!lr.ok) {
    for (const auto& e : lr.errors) spdlog::error("config: {}", e);
    return 1;
  }
  for (const auto& w : lr.warnings) spdlog::warn("config: {}", w);
  const config::TurretConfig& cfg = lr.config;
  const bool mixed_mode = !sim_mode && !cfg.hardware_profile.empty();
  const char* mixed_commission_env = std::getenv("OTA_MIXED_COMMISSION_MANUAL");
  const bool mixed_commission_manual = mixed_mode && mixed_commission_env &&
      std::strcmp(mixed_commission_env, "1") == 0;
  if (!sim_mode && !mixed_mode &&
      std::filesystem::exists("/sys/class/net/can1/device")) {
    spdlog::error("split-bus station detected; explicit mixed hardware profile required before motor startup");
    return 1;
  }
  config::mixed::Profile mixed_profile;
  if (mixed_mode) {
    const auto result = config::mixed::load_mixed_hardware_profile(cfg.hardware_profile);
    if (!result.ok) {
      for (const auto& e : result.errors) spdlog::error("mixed hardware profile: {}", e);
      return 1;
    }
    mixed_profile = result.profile;
    spdlog::info("mixed hardware: continuous GM6020 yaw on {}, bounded CyberGear pitch on {}",
                 mixed_profile.yaw_bus.interface, mixed_profile.pitch_bus.interface);
  }
  // Reject invalid explicit alignment before opening a motor transport.
  TrackingController::Config tracking_cfg;
  try { tracking_cfg = make_tracking_cfg(cfg); }
  catch (const std::exception& e) {
    spdlog::error("tracking configuration: {}", e.what());
    return 1;
  }

  // `--dump-parameter-registry <path|->`: what this binary can change while it runs, as JSON.
  // Placed deliberately here — after the config and hardware profile are loaded, so every reported
  // value is the one this boot would run with, and before any CAN transport is opened, so asking a
  // station what it can tune has no way to move a motor. A file path is preferred over stdout
  // because the boot log shares stdout; "-" asks for stdout in a context where that is safe.
  const std::string registry_dump = flag_value(argc, argv, "--dump-parameter-registry");
  if (!registry_dump.empty()) {
    const std::string json = control::parameter_registry_json(
        cfg, mixed_mode ? &mixed_profile : nullptr, config_path, cfg.hardware_profile);
    if (registry_dump == "-") {
      std::fputs(json.c_str(), stdout);
      std::fflush(stdout);
      return 0;
    }
    std::FILE* out = std::fopen(registry_dump.c_str(), "wb");
    if (out == nullptr) {
      spdlog::error("--dump-parameter-registry: cannot write {}: {}", registry_dump,
                    std::strerror(errno));
      return 1;
    }
    std::fputs(json.c_str(), out);
    std::fclose(out);
    spdlog::info("parameter registry written to {}", registry_dump);
    return 0;
  }

  // 2. The motor backend. Real: open the CAN bus (this process is the sole
  //    owner). Sim: a first-order plant, no transport object at all.
  std::unique_ptr<can::CyberGearSystem> system;
  std::unique_ptr<MotorBackend> backend;
  MixedCanMotorBackend* mixed_backend = nullptr;
  if (sim_mode) {
    backend = make_sim_backend();
  } else if (mixed_mode) {
    auto mixed = std::make_unique<MixedCanMotorBackend>();
    // The voltage ceiling travels from turret_mixed.yaml (max_output_counts) and applies only in
    // voltage mode; in current mode the envelope is axes.yaw.host_current_limit_a, which the
    // profile already carries. Both are handed over, and one line says which one this station is
    // driving with -- after 2026-09-28 I do not want a unit change discovered from a still axis.
    mixed->set_yaw_voltage_ceiling(
        cfg.axes[static_cast<int>(AxisId::Yaw)].max_output_counts);
    if (mixed_profile.yaw.control_mode == config::mixed::ControlMode::Current) {
      spdlog::info(
          "yaw commanded in torque current: host limit {} A, gains kp {} / ki {} A per rad/s "
          "(max_output_counts {} is a voltage-mode number and does not apply here)",
          mixed_profile.yaw.host_current_limit_a, mixed_profile.yaw.current_kp_a_per_rad_s,
          mixed_profile.yaw.current_ki_a_per_rad_s,
          cfg.axes[static_cast<int>(AxisId::Yaw)].max_output_counts);
    } else {
      spdlog::info("yaw commanded in voltage: ceiling {} raw counts",
                   cfg.axes[static_cast<int>(AxisId::Yaw)].max_output_counts);
    }
    std::string cerr;
    if (!mixed->open(mixed_profile, cerr)) {
      spdlog::error("mixed CAN open failed: {}", cerr);
      return 1;
    }
    mixed_backend = mixed.get();
    backend = std::move(mixed);
  } else {
    system = std::make_unique<can::CyberGearSystem>();
    can::CyberGearSystemConfig scfg;
    scfg.transport = cfg.can.backend;
    scfg.iface = cfg.can.interface;
    scfg.uart_baud = cfg.can.uart_baud;
    scfg.bitrate = static_cast<uint32_t>(cfg.can.bitrate);
    scfg.host_can_id = static_cast<uint8_t>(cfg.can.host_can_id);
    scfg.pitch_motor_id = static_cast<uint8_t>(cfg.motors[0].can_id);
    scfg.yaw_motor_id = static_cast<uint8_t>(cfg.motors[1].can_id);
    std::string cerr;
    if (!system->open(scfg, cerr)) {
      spdlog::error("CAN open failed: {}", cerr);
      return 1;
    }
    spdlog::info("CAN transport: {} device={} (CAN {} bit/s)", cfg.can.backend,
                 cfg.can.interface, cfg.can.bitrate);
    backend = std::make_unique<CanMotorBackend>(*system);
  }

  // 3. Boot FSM: discover + self-test (slow, blocking, boot-only, §27).
  std::array<uint64_t, 2> motor_ids{};
  {
    BootFsm boot(*backend, BootConfig{});
    while (!g_shutdown.load() && !boot.ready_to_home() && !boot.faulted()) boot.step();
    if (boot.faulted() || g_shutdown.load()) {
      spdlog::error("boot fault: {} — station will NOT home or move", boot.error());
      backend->deenergize(AxisId::Pitch);
      backend->deenergize(AxisId::Yaw);
      return 1;
    }
    spdlog::info("boot OK: pitch uid=0x{:016x} yaw uid=0x{:016x}",
                 boot.unique_ids()[0], boot.unique_ids()[1]);
    motor_ids = boot.unique_ids();
  }

  // 5. Control loop: homing -> safe hold -> park on shutdown.
  std::unique_ptr<RetainedHoming> retained;
  if (!sim_mode && !mixed_mode) {
    retained = std::make_unique<RetainedHoming>(config_path, motor_ids);
    backend->set_calibration_invalidator([&retained]() {
      const auto begin = now_monotonic_ns();
      retained->invalidate();
      const auto elapsed = now_monotonic_ns() - begin;
      if (elapsed > 1000000)
        spdlog::warn("calibration invalidation stalled for {:.3f} ms", elapsed/1e6);
    });
  }
  auto control_cfg = make_control_cfg(cfg);
  if (mixed_mode) {
    const auto& yaw_axis = cfg.axes[static_cast<int>(AxisId::Yaw)];
    control_cfg.continuous_yaw_sector_half_span_rad =
        std::min(-yaw_axis.expected_travel_deg.min,
                 yaw_axis.expected_travel_deg.max) * kDeg2Rad;
    control_cfg.continuous_yaw_sector_inset_rad =
        yaw_axis.soft_margin_deg * kDeg2Rad;
    // The declared band survives the envelope being removed, for exactly one purpose:
    // the yaw tape on the HUD. It is the same number the named roam region is validated
    // inside, so showing it is not inventing a limit -- and it is centred on the homing
    // origin, which is what "0" has always meant for a session-relative yaw.
    control_cfg.continuous_yaw_band_half_span_rad =
        control_cfg.continuous_yaw_sector_half_span_rad;
    if (yaw_axis.position_envelope_none)
      control_cfg.continuous_yaw_sector_half_span_rad = 0.0;
    if (control_cfg.continuous_yaw_sector_half_span_rad < 0.0 ||
        (control_cfg.continuous_yaw_sector_half_span_rad > 0.0 &&
         control_cfg.continuous_yaw_sector_half_span_rad <=
             control_cfg.continuous_yaw_sector_inset_rad)) {
      spdlog::error("mixed continuous-yaw software sector is invalid");
      mixed_backend->close();
      return 1;
    }
    if (control_cfg.continuous_yaw_sector_half_span_rad == 0.0) {
      // `position_envelope: none`. That is a declaration that continuous yaw runs
      // without a position envelope, not a missing number -- said out loud here so a
      // log reader never has to infer it from the absence of a limit.
      spdlog::warn("continuous yaw declared WITHOUT a position envelope; the "
                   "sector is gone, not merely unmeasured (AUTO_ROAM patrols the whole "
                   "circle in one direction, and every other guard is unchanged)");
    }
    // GM6020 has no reported fault/disable status and its temperature byte
    // has no documented unit. The mixed backend instead independently bounds
    // raw temperature, encoder-derived speed, feedback age and CAN health;
    // do not report those missing fields as known values.
    control_cfg.allow_unknown_motor_health = true;
    spdlog::warn("mixed motor health: GM6020 fault/temperature units unavailable; independent raw-temperature/speed/freshness guard active");
  }
  if (mixed_commission_manual) {
    control_cfg.manual_commissioning = true;
    control_cfg.start_in_auto_roam = false;
    spdlog::warn("mixed commissioning: manual/hold only; AUTO_ROAM startup and tracking auto-enable suppressed");
  }
  ControlLoop loop(std::move(control_cfg), std::move(backend));

  // The BNO085 is mounted on the moving pitch assembly. It observes gimbal
  // motion independently of motor feedback; it is not the fixed base pose.
  // The launcher owns its sole I2C process and publishes a tare-scoped trace.
  std::unique_ptr<control::ImuTraceIngest> imu_observer;
  const char* const imu_trace = std::getenv("OTA_IMU_TRACE");
  const bool imu_trace_configured = imu_trace && *imu_trace;
  // The BNO085 is an instrument, not a precondition: no IMU condition may fault the station. What
  // it can do is report itself absent, which §20's fields are for -- `present:false` on the wire is
  // the honest answer to "is there usable inertial data", and a page that renders that as "no
  // sensor" is telling the operator the truth. A daemon that exited because the sensor was quiet
  // would replace a true statement with a station that has no telemetry at all.
  //
  // The one exception is an explicit commissioning session (OTA_MIXED_COMMISSION_MANUAL=1), where
  // the tare-scoped trace is the thing under qualification: there an unmet gate refuses to *start*
  // the session, before any motion, in the same class as a config that fails to load. It is not a
  // runtime fault, and nothing has moved when it happens.
  if (mixed_mode) {
    if (!imu_trace_configured) {
      spdlog::warn("no launcher BNO085 trace (OTA_IMU_TRACE unset or the capture did not start); "
                   "the imu block will report absent");
    } else {
      imu_observer = std::make_unique<control::ImuTraceIngest>();
      std::string imu_error;
      if (!imu_observer->start(imu_trace, imu_error)) {
        spdlog::warn("BNO085 trace not ingestable ({}): continuing without an observer", imu_error);
        imu_observer.reset();
      } else if (mixed_commission_manual) {
        const auto deadline = now_monotonic_ns() + 2'000'000'000LL;
        bool ready = false;
        while (!g_shutdown.load() && now_monotonic_ns() < deadline) {
          const auto state = imu_observer->snapshot(now_monotonic_ns());
          if (state.game_rv_fresh && state.game_rv_tared &&
              state.gyro_fresh && state.game_rv_accuracy >= 2) {
            ready = true;
            spdlog::info("BNO085 observer ready: generation={} game-RV status={} tare_rx_ns={}",
                         state.generation, state.game_rv_accuracy, state.tare_rx_ns);
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!ready) {
          spdlog::error("BNO085 has no fresh same-generation host tare, game-RV and gyro; "
                        "commissioning session refused before any motion");
          loop.deenergize_all();
          return 1;
        }
      }
      // A normal mixed start does not wait for the sensor at all: the 1 Hz publisher below reports
      // freshness as it finds it, so a slow-starting IMU shows up as `present:false` for a second
      // rather than as a station that refused to boot.
    }
  }

  // §20: tell the loop when the geometry it is using was measured, read from the file the
  // intrinsics were actually loaded from, so the number describes the values in force.
  loop.set_camera_calibration_mtime_ns(file_mtime_ns(cfg.camera.intrinsics_file));

  // Boot record of what the §20 geometry age is derived from. Worth logging permanently because the
  // two numbers live on different clocks: the mtime below is CLOCK_REALTIME (ns since the epoch) while
  // everything inside the control loop is monotonic (ns since boot). Mixing them is what made this field
  // publish null for a whole round even though the file plainly loaded, so the line that shows the input
  // is the line that would explain a surprising age.
  spdlog::info("geometry age source: path={} mtime_ns={}", cfg.camera.intrinsics_file,
               file_mtime_ns(cfg.camera.intrinsics_file));

  // 5a. Phase 6 (Part 2, S1): the vision ingest. controld BINDS the
  //     SOCK_SEQPACKET socket (§6.1) and visiond connects to it; every decoded
  //     TargetMeasurement is handed to the control loop (thread-safe, §6.2).
  //     It is observe-only for safety: a measurement can only ever move the
  //     turret through the tracking reference, which the §18 envelope and the
  //     §38 supervisor still bound — and tracking itself stays off until the
  //     homing gate passes (§38.1).
  vision::VisionLink vision_link;
  std::unique_ptr<vision::VisionIngest> vision;
  {
    vision::VisionIngest::Config vc;
    vc.socket_path = cfg.vision.socket_path;
    if (const char* sp = std::getenv("OTA_VISION_SOCKET")) vc.socket_path = sp;
    loop.set_vision_link(&vision_link);
    vision = std::make_unique<vision::VisionIngest>(
        vc, &vision_link, [&loop](const vision::TargetMeasurement& m) {
          loop.feed_measurement(m);
      },
      [&loop](const ota::tracks::TrackSet& set, ota::TimeNs arrival_ns) {
        // v3 §59: the multi-candidate path. Everything downstream of the selection
        // inside feed_track_set is v1's (§17), so the estimator, the timestamp
        // alignment and the safety chain are untouched by which message arrived.
        loop.feed_track_set(set, arrival_ns);
        });
    std::string verr;
    if (!vision->start(verr)) {
      spdlog::warn("vision ingest did not start: {} (tracking will have no "
                   "measurements; continuing without vision)", verr);
      vision.reset();
    } else {
      spdlog::info("vision ingest listening: UDS {} (58-byte TargetMeasurement, "
                   "§6.1)", vc.socket_path);
    }
  }

  // 5b. Phase 6: the commissioned tracking configuration. Auto-enable is a
  //     config decision (default FALSE); the enable itself is gated on homing
  //     inside the loop (§38.1), and the `start_tracking` command (§42.2) uses
  //     the same commissioned values.
  loop.set_tracking_config(tracking_cfg,
                           cfg.tracking.enabled && !mixed_commission_manual);
  spdlog::info("tracking: auto_enable={} (§38.1 gate: homing), speeds "
               "track={:.1f} search={:.1f} deg/s, lost_behavior={}",
               cfg.tracking.enabled && !mixed_commission_manual ? "yes" : "NO",
               tracking_cfg.track_v_max_rad_s*kRad2Deg, tracking_cfg.search_v_max_rad_s*kRad2Deg,
               cfg.tracking.target_lost_behavior);
  if (cfg.motion.configured) {
    const char* names[] = {"MANUAL","AUTO_TRACK","AUTO_ROAM"};
    for (int m=0; m<3; ++m) for (int i=0; i<kAxisCount; ++i) {
      const auto& p=cfg.motion.modes[m][i];
      spdlog::info("motion {} {}: target {:.1f} deg/s {:.1f} deg/s2; maximum {:.1f} deg/s {:.1f} deg/s2 (before axis/payload/intent/boundary caps)",
          names[m],axis_name(static_cast<AxisId>(i)),p.target.speed*kRad2Deg,
          p.target.acceleration*kRad2Deg,p.maximum.speed*kRad2Deg,p.maximum.acceleration*kRad2Deg);
    }
  }

  // Phase 7: load the stored installation orientation (base -> world, §29/§30).
  // No calibration file => identity pose (assumed-level base); the telemetry
  // reports it as uncalibrated so the web UI can prompt for a calibration.
  FixedStoredPoseProvider pose_provider(cfg.installation.pose_file);
  loop.set_base_orientation(pose_provider.get());

  // Phase 9: load the active payload profile (§28.5, §41). Missing or
  // invalid file is NOT fatal: the station runs with no_profile status and
  // conservative defaults, and telemetry says a payload tuning is required
  // (§31.3). The commissioning tool (turret-payload) writes these files.
  {
    payload::PayloadProfileStore store(cfg.payload.profile_dir);
    payload::PayloadProfile prof;
    std::string perr;
    loop.set_payload_profile_dir(cfg.payload.profile_dir);  // runtime selection
    if (store.load(cfg.payload.active_profile, prof, perr)) {
      const double vp = prof.pitch.v_max_rad_s / kDeg2Rad;
      const double vy = prof.yaw.v_max_rad_s / kDeg2Rad;
      loop.set_payload_profile(std::move(prof));
      spdlog::info("payload source={}/{} startup_qualification={} auto_verify={} (auto_verify does not disable loading)",
                   cfg.payload.profile_dir, cfg.payload.active_profile,
                   mixed_mode ? "unqualified: legacy hardware binding" : "legacy startup trust",
                   cfg.payload.auto_verify);
      spdlog::info("payload profile: loaded '{}' (v_max pitch={:.1f} deg/s, yaw={:.1f} deg/s)",
                   cfg.payload.active_profile, vp, vy);
    } else {
      spdlog::warn("payload profile: '{}' not loaded ({}); running with "
                   "no_profile status (commission with turret-payload, §44)",
                   cfg.payload.active_profile, perr);
    }
  }
  spdlog::info(
      "installation pose: source={} calibrated={} (file={})",
      pose_source_name(pose_provider.get().source),
      (pose_provider.get().source != PoseSource::Identity) ? "yes" : "no",
      cfg.installation.pose_file);

  std::string err;
  HomingPlan plan = make_homing_plan(cfg, err);
  if (!err.empty()) {
    spdlog::error("homing plan invalid: {}", err);
    loop.deenergize_all();
    return 1;
  }
  std::array<AxisLogicalModel, 2> saved_models;
  std::array<AxisLimits, 2> saved_limits;
  if (!retained) {
    // Say out loud why every boot on this profile homes: the retained-homing store is not built
    // for the mixed backend, so `reused` below is false by construction -- not because a saved
    // record failed validation. The owner's ruling of 2026-09-28 ("no re-home while the drives
    // demonstrably kept power and position") cannot be honoured until retention exists per axis
    // here: pitch CyberGear is multi-turn absolute, and yaw GM6020 needs the persisted count to
    // agree with the live encoder before we may skip. Logging the reason is the precondition for
    // flipping that switch with evidence, instead of discovering it during an incident.
    spdlog::info("homing retention: unavailable on this profile (mixed/sim); homing at boot is "
                 "therefore mandatory, zero_source=homing");
  }
  const bool reused = retained && retained->load(saved_models, saved_limits) &&
      loop.restore_retained_homing(saved_models, saved_limits, err);
  // Owner ruling 2026-10-03: the station is in one of two states, Homed or Shutdown. The launcher
  // says which one to start in (OTA_START_STATE): a boot is always Shutdown -- up and reachable,
  // both motors off, nothing moves until the web's HOME -- and a deploy restores the state it found.
  // Unset means homed, which is what every caller did before the ruling.
  const char* start_state_env = std::getenv("OTA_START_STATE");
  const bool start_shut_down = start_state_env && std::strcmp(start_state_env, "shutdown") == 0;
  if (start_shut_down) {
    loop.deenergize_all();
    spdlog::info("start state: SHUTDOWN (both motors off, not homed); MENU > HOME starts the station");
  } else if (!reused && !loop.start_homing(std::move(plan), err)) {
    spdlog::error("start homing failed: {}", err);
    loop.deenergize_all();
    return 1;
  }
  loop.set_homing_factory([cfg]() { std::string e; return make_homing_plan(cfg, e); });
  spdlog::info("calibration: {}", start_shut_down ? "not homed (start state shutdown)"
                                  : reused ? "retained calibration validated; homing skipped" : "homing required");
  spdlog::info("service startup: {} after calibration and ready gates",
               mixed_commission_manual ? "manual commissioning hold" :
               cfg.v3.default_mode == "AUTO_ROAM" ? "automatic roam" : "manual hold");

  // 5c. Phase 8: web server (webd-facing, §5.3/§42.2). Publishes the §6.3
  //     snapshot at 10-20 Hz and relays developer commands through the
  //     validation gate. It never opens can0; commands are queued and executed
  //     on the control thread next cycle. Socket path + rate are overridable
  //     via env (§53: nothing hard-coded).
  web::WebServer::Config web_cfg;
  if (const char* sp = std::getenv("OTA_WEB_SOCKET")) web_cfg.socket_path = sp;
  if (const char* hz = std::getenv("OTA_WEB_HZ")) {
    try { web_cfg.telemetry_hz = std::stoi(hz); } catch (...) {}
  }
  // Trip traces land beside the socket, i.e. inside the launcher's run dir, where
  // the next start archives them together with the logs instead of truncating them.
  {
    std::filesystem::path tr{web_cfg.socket_path};
    if (tr.has_parent_path()) {
      tr = tr.parent_path();
      tr /= "traces";
      loop.set_trace_archive_dir(tr.string());
    }
  }
  web::WebServer web(web_cfg,
                     [&loop]() { return loop.telemetry().snapshot(); },
                     [&loop](const std::string& n, const std::string& a) {
                       return loop.submit_command(n, a);
                     }, [&loop]() { return loop.telemetry().control_window(); });
  // The socket's parent directory does not survive a reboot by itself: systemd's RuntimeDirectory
  // makes it for the units, but a hand-run controld has nothing, and the bind then fails with
  // "No such file or directory" — which leaves a station that is running, homed and tracking with
  // **no operator interface at all**. That is what happened on 2026-09-04 after this station
  // rebooted: one line at `warning`, /api/state answering 503 for twenty minutes, and a console
  // log that looked perfectly healthy. Making the parent is cheap; a dark station is not.
  std::error_code mk_ec;
  const std::filesystem::path sock{web_cfg.socket_path};
  if (sock.has_parent_path() && !std::filesystem::exists(sock.parent_path())) {
    std::filesystem::create_directories(sock.parent_path(), mk_ec);
    if (mk_ec)
      spdlog::warn("could not create {} for the web socket: {}",
                   sock.parent_path().string(), mk_ec.message());
    else
      spdlog::info("created {} for the web socket", sock.parent_path().string());
  }
  if (!web.start(err)) {
    // Still a warning rather than a fatal exit — refusing to run the control loop because the
    // UI cannot bind would trade a visible fault for a hidden one — but stated at a level that
    // matches the consequence: nobody can see or command this station.
    spdlog::warn("web server did not start: {} (continuing without web UI)", err);

  } else {
    spdlog::info("web server listening: UDS {} @ {} Hz", web_cfg.socket_path,
                 web_cfg.telemetry_hz);
  }

  if (mixed_backend) {
    // Register discovery and IMU startup can leave the disabled CyberGear's
    // last status frame stale. A STOP request elicits a fresh, explicit
    // disabled frame before the homing loop begins; it sends no motion target.
    mixed_backend->deenergize(AxisId::Pitch);
    const TimeNs deadline = now_monotonic_ns() + 2'000'000'000LL;
    bool pitch_stopped = false;
    while (!g_shutdown.load() && now_monotonic_ns() < deadline) {
      const TimeNs now = now_monotonic_ns();
      const auto pitch = mixed_backend->snapshot(AxisId::Pitch, now);
      if (pitch.has_feedback && pitch.disabled_known && pitch.disabled &&
          pitch.rx_ns > 0 && pitch.rx_ns <= now &&
          now - pitch.rx_ns < 100'000'000LL) {
        pitch_stopped = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!pitch_stopped) {
      spdlog::error("mixed startup blocked: no fresh CyberGear disabled feedback after STOP");
      loop.deenergize_all();
      return 1;
    }
    spdlog::info("mixed startup: fresh pitch STOP feedback confirmed immediately before homing");
  }

  const TimeNs period_ns = static_cast<TimeNs>(1e9) / cfg.control_loop_hz;
  TimingStats stats;
  TimingStats work_stats;
  // From here on this thread is the 200 Hz control loop (docs/operations/os-setup.md).
  lock_process_memory();
  // The main thread keeps the process name: the launcher and tools find controld by it.
  apply_thread_class("controld", ThreadClass::Motor, rt_priority::kControl);
  TimeNs t_prev = now_monotonic_ns();
  // Wake-ups on a fixed grid: each cycle is due one period after the previous one was due, not one
  // period after it woke, so a late wake-up is not carried into every later cycle.
  TimeNs due = t_prev + period_ns;
  bool logged_fault = false;
  bool logged_ready = false;
  int cycles = 0;

  // Steady-state 200 Hz loop (no slow work inside).
  if (system) system->start_watchdog();
  if (mixed_backend) mixed_backend->start_watchdog();
  while (!g_shutdown.load()) {
    const TimeNs t0 = now_monotonic_ns();
    const TimeNs period = t0 - t_prev;
    t_prev = t0;
    const Phase ph = loop.step(t0, period);
    work_stats.record_period(now_monotonic_ns() - t0);
    if (retained && ph == Phase::Hold && loop.homed() && !retained->valid())
      retained->save(loop.models(), loop.limits());
    stats.record_period(period);
    // Stall attribution for the supervisor's Brake/Derate (§39.2/§39.3): a
    // cycle longer than the 5 ms period by more than 3 ms is logged with the
    // phase and the safety action it produced, so a live flap reads as
    // "slow cycle in phase=hold" rather than as an unexplained Brake.
    if (period > kSlowCycleNs) {
      spdlog::warn("SLOW CYCLE {:.3f} ms (phase={}, action={})",
                   ns_to_ms(period), phase_name(loop.phase()),
                   safety_action_name(loop.last_decision().action));
    }
    if (ph == Phase::Fault && !logged_fault) {
      spdlog::error("control fault: {}", loop.fault_reason());
      logged_fault = true;
    }
    if (ph == Phase::Hold && loop.position_ready() && loop.at_ready() && !logged_ready) {
      spdlog::info("position reference ready + at ready pose; holding (Ctrl-C to stop)");
      logged_ready = true;
    }
    ++cycles;
    if (cycles % cfg.control_loop_hz == 0) {
      // §55 acceptance metrics, continuously available in the log: the loop
      // timing distribution over the last second (§7.2/§43.1). `worst_us` is
      // the number that explains a supervisor Brake/DERATE ("was it the loop,
      // or the bus?").
      const TimingReport tr = stats.report();
      const TimingReport wr = work_stats.report();
      spdlog::info(
          "t={:.2f}s phase={} q_pitch={:+.4f} q_yaw={:+.4f} rad "
          "temp_pitch={:.1f} temp_yaw={:.1f} C temp_raw_pitch={} temp_raw_yaw={} "
          "a_pitch={:+.2f} a_yaw={:+.2f}",
          ns_to_ms(t0) / 1e3, phase_name(ph),
          loop.last_positions()[0], loop.last_positions()[1],
          loop.last_temps()[0], loop.last_temps()[1],
          loop.last_temp_raw()[0], loop.last_temp_raw()[1],
          loop.last_accels()[0], loop.last_accels()[1]);
      spdlog::info(
          "loop: target={} Hz p50={:.3f} p95={:.3f} p99={:.3f} worst={:.3f} ms "
          "(n={}) | step work p50={:.3f} p99={:.3f} worst={:.3f} ms",
          cfg.control_loop_hz, tr.p50_ns / 1e6, tr.p95_ns / 1e6,
          tr.p99_ns / 1e6, tr.worst_ns / 1e6, tr.samples,
          wr.p50_ns / 1e6, wr.p99_ns / 1e6, wr.worst_ns / 1e6);
      if (mixed_backend) {
        for (const auto& bus : mixed_backend->can_health_all()) {
          spdlog::info("CAN {}: up={} state={} rx={} rx_errors={} tx={} tx_failed={}",
                       bus.device, bus.up, bus.state, bus.rx_frames,
                       bus.rx_error_frames, bus.tx_frames, bus.tx_failed);
        }
      }
      if (!imu_observer && imu_trace_configured) {
        // Reached by a start whose profile carries no hardware profile, so no strict ingest was
        // built above: the trace is then observed rather than gated -- a sensor that has not
        // produced a line yet must not hold the station down, which is what the mixed path
        // deliberately does. On this station's mixed profile imu_observer already exists and this
        // block stays dormant; what runs there is the publish below. Measured 09-29: the mixed
        // profile has been ingesting and logging this trace at 1 Hz all along; what was missing was
        // the §20 fields, not the reader.
        imu_observer = std::make_unique<control::ImuTraceIngest>();
        std::string observe_error;
        if (imu_observer->start(imu_trace, observe_error)) {
          spdlog::info("BNO085 observer attached to {} (observe-only: no control input, no gate)",
                       imu_trace);
        } else {
          spdlog::warn("BNO085 trace present but not ingestable yet: {}", observe_error);
          imu_observer.reset();
        }
      }
      if (imu_observer) {
        const auto imu_state = imu_observer->snapshot(t0);
        loop.observe_imu(/*present=*/imu_state.trace_open &&
                                   (imu_state.game_rv_fresh || imu_state.gyro_fresh),
                         /*gravity_valid=*/imu_state.game_rv_fresh && !imu_state.gap_seen);
        spdlog::info("BNO085 observer: generation={} tare={} game_rv_fresh={} gyro_fresh={} status={} gap={} trace_ended={}",
                     imu_state.generation, imu_state.game_rv_tared,
                     imu_state.game_rv_fresh, imu_state.gyro_fresh,
                     imu_state.game_rv_accuracy, imu_state.gap_seen,
                     imu_state.trace_ended);
      }
      // 1 Hz vision/tracking status line (§6.1/§6.3): makes "visiond is not
      // publishing", "measurements are stale" and "the tracker is not
      // acquiring" distinguishable from the log alone.
      const telemetry::TelemetrySnapshot snap = loop.telemetry().snapshot();
      if (snap.vision_connected || loop.tracking_mode_enabled()) {
        spdlog::info(
            "vision: {} frames ({} dropped, seq {}, age {} ms) | tracking={} "
            "state={} conf={:.2f}",
            snap.vision_frames, snap.vision_dropped,
            snap.vision_last_frame_sequence, snap.vision_measurement_age_ms,
            loop.tracking_mode_enabled() ? "on" : "off",
            tracking::track_state_name(snap.track_state),
            snap.target_confidence);
      }
    }
    // A cycle that overran by a whole period starts a new grid instead of running the missed
    // cycles back to back.
    const TimeNs tnow = now_monotonic_ns();
    if (tnow - due >= period_ns) due = tnow;
    const timespec wake{static_cast<time_t>(due / 1'000'000'000LL),
                        static_cast<long>(due % 1'000'000'000LL)};
    while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR) {
    }
    due += period_ns;
  }

  // Park before joining I/O workers: their shutdown can exceed the independent
  // watchdog's heartbeat deadline. Commands remain gated by parking/shutdown.
  loop.set_vision_link(nullptr);
  spdlog::info("shutdown requested; {}", loop.position_ready() ? "controlled stop" : "zero/STOP requests");
  bool parking_started = false;
  if (loop.position_ready() && loop.phase() != Phase::Fault &&
      loop.phase() != Phase::Parked) {
    parking_started = loop.start_parking(err);
    if (!parking_started) spdlog::error("shutdown park rejected: {}", err);
  }
  if (parking_started) {
    t_prev = now_monotonic_ns();
    double budget_s = 20.0;
    for (int i = 0; i < kAxisCount; ++i) {
      if (mixed_mode && i == static_cast<int>(AxisId::Yaw)) continue;
      const auto& limit = loop.limits()[i];
      budget_s += 1.5 * (limit.q_soft_max_rad - limit.q_soft_min_rad) /
          (cfg.shutdown.speed_deg_s * kDeg2Rad);
    }
    const TimeNs park_deadline = t_prev + static_cast<TimeNs>(budget_s * 1e9);
    // An accepted unverified stop is not an active parking state machine.
    // Do not resume ordinary Hold/mode output for the whole parking budget.
    while (now_monotonic_ns() < park_deadline && loop.phase() == Phase::Parking) {
      const TimeNs t0 = now_monotonic_ns();
      loop.step(t0, t0 - t_prev);
      t_prev = t0;
      std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns));
    }
  }
  const bool shutdown_failed = loop.phase() != Phase::Parked;
  if (!shutdown_failed) {
    if (mixed_mode)
      spdlog::info("STOPPED (pitch disable confirmed; GM6020 yaw zero requested, disable state unavailable)");
    else
      spdlog::info("PARKED (motors de-energized at the park pose)");
  } else {
    loop.deenergize_all();
    if (mixed_mode)
      spdlog::error("STOP FAILED: pitch STOP and yaw zero requested (phase={}, fault='{}')",
                    phase_name(loop.phase()),
                    loop.fault_reason().empty() ? "stop unavailable or shutdown deadline exceeded" : loop.fault_reason());
    else
      spdlog::error("PARK FAILED: de-energized (phase={}, fault='{}')", phase_name(loop.phase()),
                   loop.fault_reason().empty() ? "park unavailable or shutdown deadline exceeded" : loop.fault_reason());
  }
  // One closing line in the stop-evidence file, carrying the stop_id the earlier records used:
  // the log says what we printed to a terminal, this says how the process ended.
  loop.note_shutdown(!shutdown_failed,
                     loop.fault_reason().empty()
                         ? (mixed_mode ? "stop requested" : "park requested")
                         : loop.fault_reason());
  if (system) system->close();
  if (mixed_backend) mixed_backend->close();
  if (imu_observer) imu_observer->stop();
  web.stop();
  if (vision) vision->stop();
  spdlog::info("controld stopped cleanly");
  spdlog::shutdown();  // drain + drop the async log queue (no lost tail)
  return shutdown_failed ? 2 : 0;
}
