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
#include <string>
#include <thread>

#include "calibration/homing_plan.hpp"
#include "calibration/park_controller.hpp"
#include "can/cybergear_system.hpp"
#include "common/time.hpp"
#include "common/timing_stats.hpp"
#include "common/types.hpp"
#include "config/turret_config.hpp"
#include "control/boot_fsm.hpp"
#include "control/can_motor_backend.hpp"
#include "control/control_loop.hpp"

#include <spdlog/spdlog.h>

using namespace ota;

namespace {

std::atomic<bool> g_shutdown{false};
void on_signal(int) { g_shutdown.store(true); }

// --- config -> component mappings (units: YAML is degrees; control is rad) --

HomingAction parse_action(const config::HomingPlanActionConfig& c,
                          std::string& err) {
  HomingAction a;
  if (c.axis == "pitch") a.axis = AxisId::Pitch;
  else if (c.axis == "yaw") a.axis = AxisId::Yaw;
  else {
    err = "homing_plan: unknown axis '" + c.axis + "'";
    return a;
  }
  if (c.action == "home_full_range") {
    a.type = HomingActionType::HomeFullRange;
  } else if (c.action == "home_endpoint") {
    a.type = HomingActionType::HomeEndpoint;
    a.endpoint = (c.endpoint == "upper") ? Endpoint::Upper : Endpoint::Lower;
  } else if (c.action == "move") {
    a.type = HomingActionType::Move;
  } else {
    err = "homing_plan: unknown action '" + c.action + "'";
    return a;
  }
  a.precision = (c.precision == "coarse") ? Precision::Coarse : Precision::Fine;
  a.position_deg = c.position_deg;
  return a;
}

HomingPlan make_homing_plan(const config::TurretConfig& cfg, std::string& err) {
  HomingParams hp;  // start from the safe defaults...
  const config::ContactConfig& cc = cfg.homing.contact;  // ...override from YAML
  if (cc.coarse_speed_deg_s > 0)
    hp.coarse_speed_rad_s = cc.coarse_speed_deg_s * kDeg2Rad;
  if (cc.fine_speed_deg_s > 0)
    hp.fine_speed_rad_s = cc.fine_speed_deg_s * kDeg2Rad;
  if (cc.stall_velocity_threshold > 0)
    hp.contact.v_stall_threshold_rad_s = cc.stall_velocity_threshold;
  if (cc.current_or_effort_limit > 0)
    hp.contact.effort_contact_threshold_nm = cc.current_or_effort_limit;
  if (cc.contact_dwell_ms > 0) hp.contact.contact_dwell_ms = cc.contact_dwell_ms;
  if (cc.backoff_deg > 0) hp.backoff_rad = cc.backoff_deg * kDeg2Rad;
  if (cc.repeatability_deg > 0)
    hp.repeatability_rad = cc.repeatability_deg * kDeg2Rad;

  HomingPlanConfig hpc;
  hpc.homing = hp;
  for (int i = 0; i < kAxisCount; ++i) {
    hpc.travel_bands[i].min_deg = cfg.axes[i].expected_travel_deg.min;
    hpc.travel_bands[i].max_deg = cfg.axes[i].expected_travel_deg.max;
  }

  std::vector<HomingAction> actions;
  for (const auto& c : cfg.homing_plan.actions) {
    std::string e;
    actions.push_back(parse_action(c, e));
    if (!e.empty()) {
      err = e;
      return HomingPlan({}, hpc);
    }
  }
  if (actions.empty()) {
    // No explicit plan: full-range home both axes (pitch, then yaw).
    actions.push_back({HomingActionType::HomeFullRange, AxisId::Pitch,
                       Endpoint::Lower, Precision::Fine, 0.0});
    actions.push_back({HomingActionType::HomeFullRange, AxisId::Yaw,
                       Endpoint::Lower, Precision::Fine, 0.0});
  }
  return HomingPlan(std::move(actions), hpc);
}

ControlLoop::Config make_control_cfg(const config::TurretConfig& cfg) {
  ControlLoop::Config c;
  c.control_hz = cfg.control_loop_hz;
  c.feedback_max_age_ms = cfg.safety.feedback_max_age_ms;
  c.deadline_max_us = cfg.safety.deadline_max_us;
  c.deadline_miss_threshold = cfg.safety.deadline_miss_threshold;
  c.motor_overtemp_c = cfg.safety.motor_overtemp_c;
  c.soft_margin_rad = cfg.axes[0].soft_margin_deg * kDeg2Rad;
  c.park.park_logical_deg[0] = cfg.shutdown.pitch_park_deg;
  c.park.park_logical_deg[1] = cfg.shutdown.yaw_park_deg;
  c.park.pos_tol_deg = cfg.shutdown.pos_tolerance_deg;
  c.park.vel_tol_deg_s = cfg.shutdown.vel_tolerance_deg_s;
  c.park.dwell_ms = cfg.shutdown.dwell_ms;
  c.park.speed_deg_s = cfg.shutdown.speed_deg_s;
  return c;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string config_path = (argc > 1) ? argv[1] : "config/turret.yaml";
  spdlog::set_level(spdlog::level::info);
  spdlog::info("controld starting (config: {})", config_path);

  // 1. Load + validate config (a hard error blocks boot).
  config::LoadResult lr = config::load_turret_config(config_path);
  if (!lr.ok) {
    for (const auto& e : lr.errors) spdlog::error("config: {}", e);
    return 1;
  }
  for (const auto& w : lr.warnings) spdlog::warn("config: {}", w);
  const config::TurretConfig& cfg = lr.config;

  // 2. Open the CAN bus (this process is the sole owner of can0).
  can::CyberGearSystem system;
  can::CyberGearSystemConfig scfg;
  scfg.iface = cfg.can.interface;
  scfg.bitrate = static_cast<uint32_t>(cfg.can.bitrate);
  scfg.host_can_id = static_cast<uint8_t>(cfg.can.host_can_id);
  scfg.pitch_motor_id = static_cast<uint8_t>(cfg.motors[0].can_id);
  scfg.yaw_motor_id = static_cast<uint8_t>(cfg.motors[1].can_id);
  std::string err;
  if (!system.open(scfg, err)) {
    spdlog::error("CAN open failed: {}", err);
    return 1;
  }

  // 3. Boot FSM: discover + self-test (slow, blocking, boot-only, §27).
  auto backend = std::make_unique<CanMotorBackend>(system);
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
  spdlog::info("homing started; tracking is DISABLED in Phase 2");

  const TimeNs period_ns = static_cast<TimeNs>(1e9) / cfg.control_loop_hz;
  TimingStats stats;
  TimeNs t_prev = now_monotonic_ns();
  bool logged_fault = false;
  bool logged_ready = false;
  int cycles = 0;

  // Steady-state 200 Hz loop (no slow work inside).
  while (!g_shutdown.load()) {
    const TimeNs t0 = now_monotonic_ns();
    const TimeNs period = t0 - t_prev;
    t_prev = t0;
    const Phase ph = loop.step(t0, period);
    stats.record_period(period);
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
      spdlog::info("t={:.2f}s phase={} q_pitch={:+.4f} q_yaw={:+.4f} rad",
                   ns_to_ms(t0) / 1e3, phase_name(ph),
                   loop.last_positions()[0], loop.last_positions()[1]);
    }
    const TimeNs next = t0 + period_ns;
    const TimeNs tnow = now_monotonic_ns();
    if (tnow < next) std::this_thread::sleep_for(std::chrono::nanoseconds(next - tnow));
  }

  // 6. Shutdown: safe park (if homed and not faulted), else de-energize.
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
  system.close();
  spdlog::info("controld stopped cleanly");
  return 0;
}
