// controld — sole owner of can0 and the motors (architecture §4.1).
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
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "calibration/camera_calibration.hpp"
#include "calibration/homing_plan.hpp"
#include "calibration/park_controller.hpp"
#include "can/cybergear_system.hpp"
#include "common/time.hpp"
#include "common/timing_stats.hpp"
#include "common/types.hpp"
#include "config/station_wiring.hpp"
#include "config/turret_config.hpp"
#include "control/boot_fsm.hpp"
#include "control/can_motor_backend.hpp"
#include "control/control_loop.hpp"
#include "payload/payload_profile.hpp"
#include "sim/sim_motor_backend.hpp"
#include "vision/vision_ingest.hpp"
#include "web/web_server.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>

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
    spdlog::init_thread_pool(8192, 1);
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
  TrackingController::Config t;
  t.estimator.alpha = cfg.tracking.estimator_alpha;
  t.estimator.beta = cfg.tracking.estimator_beta;
  t.fsm.coast_max_ns = static_cast<int64_t>(cfg.tracking.coast_timeout_ms) * 1000000;
  t.fsm.lost_ns = static_cast<int64_t>(cfg.tracking.lost_timeout_ms) * 1000000;
  t.fsm.search_enabled =
      cfg.tracking.search_enabled_by_default ||
      cfg.tracking.target_lost_behavior == "search";
  t.search.v_max_rad_s = cfg.tracking.search_speed_deg_s * kDeg2Rad;
  t.track_v_max_rad_s = cfg.tracking.track_speed_deg_s * kDeg2Rad;
  t.search_v_max_rad_s = cfg.tracking.search_speed_deg_s * kDeg2Rad;
  // Was hard-coded at 10.0 deg/s. The default in the config loader is the same number, so nothing
  // moves until an operator writes the key - which is the point: round 40 showed this constant is
  // what the acceptance criteria collide with, and a safety ceiling should not require a rebuild.
  t.hold_v_max_rad_s = cfg.tracking.hold_speed_deg_s * kDeg2Rad;
  t.control_delay_ns = static_cast<int64_t>(cfg.tracking.control_delay_ms) * 1000000;
  t.motor_response_ns =
      static_cast<int64_t>(cfg.tracking.motor_response_ms) * 1000000;
  t.fresh_threshold_ns =
      static_cast<int64_t>(cfg.tracking.fresh_threshold_ms) * 1000000;

  // §28.2 intrinsics (pixel -> camera ray).
  const IntrinsicsLoad il = load_camera_intrinsics(cfg.camera.intrinsics_file);
  if (il.found) {
    t.intrinsics = il.intrinsics;
    // Where to aim inside the target. Handed over here rather than defaulted inside the
    // controller, so the station file remains the single place the operator's rule is set.
    t.aim.aim_at_head = cfg.tracking.aim_at_head;
    t.aim.head_fraction_from_top = cfg.tracking.head_fraction_from_top;
    if (cfg.tracking.aim_at_head) {
      spdlog::info("tracking aim point: head, {:.0f}% below the top of the target box",
                   cfg.tracking.head_fraction_from_top * 100.0);
    } else {
      spdlog::info("tracking aim point: anchor (the detector's box centroid)");
    }
    spdlog::info("camera intrinsics: {} ({})", il.detail,
                 cfg.camera.intrinsics_file);
  } else {
    spdlog::warn("camera intrinsics: {} ({}) — tracking geometry is "
                 "UNCALIBRATED until the §28.2 commissioning pass (P9)",
                 il.detail, cfg.camera.intrinsics_file);
  }
  // §28.3 extrinsics (camera -> pitch frame, R_P_C).
  std::string xdetail;
  t.kinematics = load_camera_extrinsics(cfg.camera.extrinsics_file, xdetail);
  spdlog::info("camera extrinsics: {} ({})", xdetail,
               cfg.camera.extrinsics_file);
  return t;
}

}  // namespace

int main(int argc, char** argv) {
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

  // 2. The motor backend. Real: open the CAN bus (this process is the sole
  //    owner). Sim: a first-order plant, no transport object at all.
  std::unique_ptr<can::CyberGearSystem> system;
  std::unique_ptr<MotorBackend> backend;
  if (sim_mode) {
    backend = make_sim_backend();
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
  {
    BootFsm boot(*backend, BootConfig{});
    while (!boot.ready_to_home() && !boot.faulted()) boot.step();
    if (boot.faulted()) {
      spdlog::error("boot fault: {} — station will NOT home or move", boot.error());
      backend->deenergize(AxisId::Pitch);
      backend->deenergize(AxisId::Yaw);
      return 1;
    }
    spdlog::info("boot OK: pitch uid=0x{:016x} yaw uid=0x{:016x}",
                 boot.unique_ids()[0], boot.unique_ids()[1]);
  }

  // 4. SIGINT/SIGTERM -> safe park (§33), not a hard stop.
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // 5. Control loop: homing -> safe hold -> park on shutdown.
  ControlLoop loop(make_control_cfg(cfg), std::move(backend));

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
  loop.set_tracking_config(make_tracking_cfg(cfg), cfg.tracking.enabled);
  spdlog::info("tracking: auto_enable={} (§38.1 gate: homing), speeds "
               "track={:.1f} search={:.1f} deg/s, lost_behavior={}",
               cfg.tracking.enabled ? "yes" : "NO",
               cfg.tracking.track_speed_deg_s, cfg.tracking.search_speed_deg_s,
               cfg.tracking.target_lost_behavior);

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
  if (!loop.start_homing(std::move(plan), err)) {
    spdlog::error("start homing failed: {}", err);
    loop.deenergize_all();
    return 1;
  }
  spdlog::info("homing started; tracking {} (§38.1: enabled only after the "
               "homing gates pass)",
               cfg.tracking.enabled ? "auto-enables after homing"
                                    : "stays OFF until start_tracking");

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
  web::WebServer web(web_cfg,
                     [&loop]() { return loop.telemetry().snapshot(); },
                     [&loop](const std::string& n, const std::string& a) {
                       return loop.submit_command(n, a);
                     });
  if (!web.start(err)) {
    spdlog::warn("web server did not start: {} (continuing without web UI)",
                 err);
  } else {
    spdlog::info("web server listening: UDS {} @ {} Hz", web_cfg.socket_path,
                 web_cfg.telemetry_hz);
  }

  const TimeNs period_ns = static_cast<TimeNs>(1e9) / cfg.control_loop_hz;
  TimingStats stats;
  TimeNs t_prev = now_monotonic_ns();
  bool logged_fault = false;
  bool logged_ready = false;
  int cycles = 0;

  // Steady-state 200 Hz loop (no slow work inside).
  while (!g_shutdown.load()) {
    if (loop.shutdown_requested()) {
      spdlog::info("safe shutdown requested via web UI");
      g_shutdown.store(true);
      break;
    }
    const TimeNs t0 = now_monotonic_ns();
    const TimeNs period = t0 - t_prev;
    t_prev = t0;
    const Phase ph = loop.step(t0, period);
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
    if (ph == Phase::Hold && loop.homed() && loop.at_ready() && !logged_ready) {
      spdlog::info("homed + at ready pose; holding (Ctrl-C to park)");
      logged_ready = true;
    }
    ++cycles;
    if (cycles % cfg.control_loop_hz == 0) {
      // §55 acceptance metrics, continuously available in the log: the loop
      // timing distribution over the last second (§7.2/§43.1). `worst_us` is
      // the number that explains a supervisor Brake/DERATE ("was it the loop,
      // or the bus?").
      const TimingReport tr = stats.report();
      spdlog::info(
          "t={:.2f}s phase={} q_pitch={:+.4f} q_yaw={:+.4f} rad "
          "temp_pitch={:.1f} temp_yaw={:.1f} C a_pitch={:+.2f} a_yaw={:+.2f}",
          ns_to_ms(t0) / 1e3, phase_name(ph),
          loop.last_positions()[0], loop.last_positions()[1],
          loop.last_temps()[0], loop.last_temps()[1],
          loop.last_accels()[0], loop.last_accels()[1]);
      spdlog::info(
          "loop: target={} Hz p50={:.3f} p95={:.3f} p99={:.3f} worst={:.3f} ms "
          "(n={})",
          cfg.control_loop_hz, tr.p50_ns / 1e6, tr.p95_ns / 1e6,
          tr.p99_ns / 1e6, tr.worst_ns / 1e6, tr.samples);
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
    const TimeNs next = t0 + period_ns;
    const TimeNs tnow = now_monotonic_ns();
    if (tnow < next) std::this_thread::sleep_for(std::chrono::nanoseconds(next - tnow));
  }

  // 6. Shutdown: stop the web server (no more clients) and the vision ingest
  //    (no more measurements), then safe park (if homed and not faulted), else
  //    de-energize. The park move runs with tracking stopped by the loop.
  web.stop();
  if (vision) {
    vision->stop();
    vision.reset();
  }
  loop.set_vision_link(nullptr);
  spdlog::info("shutdown requested; {}", loop.homed() ? "parking" : "de-energizing");
  if (loop.homed() && loop.phase() != Phase::Fault &&
      loop.phase() != Phase::Parked && loop.start_parking(err)) {
    t_prev = now_monotonic_ns();
    for (int i = 0; i < 8000 && loop.phase() != Phase::Parked; ++i) {
      const TimeNs t0 = now_monotonic_ns();
      loop.step(t0, t0 - t_prev);
      t_prev = t0;
      std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns));
    }
  }
  if (loop.phase() == Phase::Parked) {
    spdlog::info("PARKED (motors de-energized at the park pose)");
  } else {
    loop.deenergize_all();
    spdlog::info("de-energized (phase={}, fault='{}')", phase_name(loop.phase()),
                 loop.fault_reason());
  }
  if (system) system->close();
  spdlog::info("controld stopped cleanly");
  spdlog::shutdown();  // drain + drop the async log queue (no lost tail)
  return 0;
}
