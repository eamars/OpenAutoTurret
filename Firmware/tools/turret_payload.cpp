// turret-payload — payload profiling / verification commissioning CLI
// (architecture §28.5, §31.3, §41, §44).
//
// Produces and checks the payload profiles under config/payload_profiles/:
//   * `profile` runs the §44 response-test battery (low-amplitude steps,
//     triangle, braking from increasing speeds, holding effort) on both axes
//     and stores the measured baseline + safe v/a/j limits as a profile;
//   * `verify` runs the §31.3 small-move check against the stored baseline
//     and reports ok/mismatch (what controld does at the optional §27 stage);
//   * `list` shows the stored profiles.
//
// DEVELOPER INSTRUMENT, NOT the control path: like turret-can it may only be
// used when controld is not running (single-owner-of-motors rule, §4.1).
//
// `--sim` is a rehearsal mode against the simulated plant: it NEVER opens a
// CAN socket and is safe to run anywhere (used by the test battery).
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "can/cybergear_system.hpp"
#include "control/boot_fsm.hpp"
#include "control/can_motor_backend.hpp"
#include "config/turret_config.hpp"
#include "control/motor_backend.hpp"
#include "common/time.hpp"
#include "payload/payload_check.hpp"
#include "payload/payload_profile.hpp"
#include "payload/payload_profiler.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;
using namespace ota::payload;

namespace {

volatile std::sig_atomic_t g_stop_flag = 0;
void on_signal(int) { g_stop_flag = 1; }

void print_usage(const char* prog) {
  std::printf(
      "usage: %s [options] <command>\n"
      "\n"
      "options:\n"
      "  --sim              rehearsal on the simulated plant (no CAN, safe)\n"
      "  --config PATH      station config (real mode; default config/turret.yaml)\n"
      "  --dir DIR          profile directory (default config/payload_profiles)\n"
      "  --name NAME        profile name (default: active profile / 'manual')\n"
      "  --notes TEXT       operator notes stored in the profile\n"
      "\n"
      "commands:\n"
      "  list                     list stored payload profiles\n"
      "  profile                  run the §44 battery on both axes from the\n"
      "                           current pose and store the profile\n"
      "  verify                   run the §31.3 check vs the stored baseline\n"
      "\n"
      "REAL MODE (no --sim) opens the CAN bus and moves the motors. Only run\n"
      "it when controld is NOT running and the payload state is known.",
      prog);
}

double deg2rad(double d) { return d * M_PI / 180.0; }

std::string fmt_metrics(const AxisProfileMetrics& m) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "step+  rise=%.3fs overshoot=%.1f%% settle=%.3fs peak_effort=%.2fNm rms=%.1fmm%s\n"
                "step-  rise=%.3fs overshoot=%.1f%% settle=%.3fs peak_effort=%.2fNm rms=%.1fmm%s\n"
                "tri    rms=%.1fmm   brake  v0=%.2fdeg/s stop=%.2fmm in %.3fs   hold=%.2fNm\n",
                m.step_pos.rise_time_s, 100.0 * m.step_pos.overshoot,
                m.step_pos.settling_time_s, m.step_pos.peak_effort_nm,
                1000.0 * m.step_pos.tracking_rms_rad,
                m.step_pos.valid ? "" : "  [invalid]",
                m.step_neg.rise_time_s, 100.0 * m.step_neg.overshoot,
                m.step_neg.settling_time_s, m.step_neg.peak_effort_nm,
                1000.0 * m.step_neg.tracking_rms_rad,
                m.step_neg.valid ? "" : "  [invalid]",
                1000.0 * m.triangle_rms_rad,
                180.0 / M_PI * m.brake.v0_rad_s,
                1000.0 * m.brake.stop_distance_rad, m.brake.stop_time_s,
                m.hold_effort_nm);
  return buf;
}

bool cmd_list(const std::string& dir) {
  PayloadProfileStore store(dir);
  const auto names = store.list();
  if (names.empty()) {
    std::printf("(no profiles in %s)\n", dir.c_str());
    return true;
  }
  std::printf("profiles in %s:\n", dir.c_str());
  for (const auto& n : names) {
    PayloadProfile p;
    std::string err;
    if (store.load(n, p, err)) {
      std::printf("  %-20s v_max pitch=%.1f deg/s  yaw=%.1f deg/s  created=%lld\n",
                  n.c_str(), 180.0 / M_PI * p.pitch.v_max_rad_s,
                  180.0 / M_PI * p.yaw.v_max_rad_s,
                  (long long)(p.created_ns / 1000000000LL));
    } else {
      std::printf("  %-20s [unreadable: %s]\n", n.c_str(), err.c_str());
    }
  }
  return true;
}

// Drive one axis's §31.3 check at 200 Hz against the backend. In --sim the
// clock is synthetic (the plant advances per command, not per wall time).
bool run_check(MotorBackend& b, AxisId a, PayloadCheck& check,
               bool sim, std::string& err) {
  TimeNs t0 = sim ? 0 : now_monotonic_ns();
  AxisSnapshot sp = b.snapshot(a, t0);
  if (!check.begin(t0, sp.q_rad, sp.has_feedback)) {
    err = check.fail_reason();
    return false;
  }
  const auto tick = std::chrono::microseconds(5000);
  for (int i = 0; i < 8000 && check.active(); ++i) {
    if (g_stop_flag) { err = "interrupted"; return false; }
    t0 += sim ? 5000000 : (now_monotonic_ns() - t0 > 0 ? now_monotonic_ns() - t0 : 5000000);
    sp = b.snapshot(a, t0);
    const auto out = check.step(t0, sp);
    b.command(a, out.q_ref_rad, out.limit_spd_rad_s);
    if (!sim) std::this_thread::sleep_for(tick);
  }
  if (check.failed()) { err = check.fail_reason(); return false; }
  if (check.active()) { err = "check did not complete in time"; return false; }
  return true;
}

bool cmd_verify(std::unique_ptr<MotorBackend> backend, const std::string& dir,
                const std::string& name, bool sim) {
  PayloadProfileStore store(dir);
  PayloadProfile prof;
  std::string err;
  if (!store.load(name, prof, err)) {
    std::fprintf(stderr, "cannot load profile '%s': %s\n", name.c_str(),
                 err.c_str());
    return false;
  }
  PayloadCheckConfig ccfg;
  const PayloadProfile* pptr = &prof;
  VerifyResult vr;  // folded with the same rule the daemon uses (§31.3)
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    PayloadCheck check(ccfg, pptr, a);
    if (!run_check(*backend, a, check, sim, err)) {
      std::fprintf(stderr, "%s: %s\n", axis_name(a), err.c_str());
      vr.axes[i].measured = false;  // unmeasured -> Error overall
      vr.detail = err;
      continue;
    }
    vr.axes[i] = check.axis_result();
    std::printf("%s: %s\n", axis_name(a),
                vr.axes[i].ok ? "OK (matches stored baseline)" : "MISMATCH");
    for (const auto& v : vr.axes[i].violations)
      std::printf("    %s\n", v.c_str());
  }
  vr.status = overall_status(vr.axes, true);
  std::printf("overall: %s\n", payload_status_name(vr.status));
  return vr.status != PayloadStatus::Error;
}

bool cmd_profile(std::unique_ptr<MotorBackend> backend,
                 const config::TurretConfig& cfg, const std::string& dir,
                 const std::string& name, const std::string& notes, bool sim) {
  ProfilerConfig pcfg;
  // The configured envelope caps the derived limits; fall back to a generous
  // default when the config is empty (e.g. --sim without a config file).
  const double env_v = cfg.axes[0].max_velocity_deg_s > 0.0
                           ? cfg.axes[0].max_velocity_deg_s
                           : 90.0;
  const double env_a = cfg.axes[0].max_acceleration_deg_s2 > 0.0
                           ? cfg.axes[0].max_acceleration_deg_s2
                           : 180.0;
  pcfg.env_v_max_rad_s = deg2rad(env_v);
  pcfg.env_a_max_rad_s2 = deg2rad(env_a);
  TimeNs sim_t = 0;
  PayloadProfiler profiler(
      *backend, pcfg,
      sim ? PayloadProfiler::TimeSource(
                [&sim_t]() { sim_t += 5000000; return sim_t; })
          : PayloadProfiler::TimeSource(),
      sim ? PayloadProfiler::Pacer([](void) {}) : PayloadProfiler::Pacer());
  PayloadProfile prof;
  prof.name = name;
  prof.created_ns = now_monotonic_ns();
  prof.config_revision = cfg.schema_version ? "turret.yaml" : "";
  prof.hardware = cfg.can.interface;
  prof.notes = notes;
  bool ok = true;
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    TimeNs t0 = sim ? sim_t : now_monotonic_ns();
    const double q0 = backend->snapshot(a, t0).q_rad;
    std::string err;
    AxisLimits lim;  // tool relies on the safe central region
    const AxisProfileMetrics m = profiler.profile_axis(a, q0, lim, err);
    if (!m.valid) {
      std::fprintf(stderr, "%s: profiling failed (%s)\n", axis_name(a),
                   err.c_str());
      ok = false;
      continue;
    }
    AxisPayloadProfile axp;
    PayloadProfiler::derive_limits(m, pcfg, axp);
    prof.axis(a) = axp;
    std::printf("%s:\n%s", axis_name(a), fmt_metrics(m).c_str());
    std::printf("  limits: v_max=%.1f deg/s a_max=%.1f deg/s^2 j_max=%.1f deg/s^3\n",
                180.0 / M_PI * axp.v_max_rad_s,
                180.0 / M_PI * axp.a_max_rad_s2,
                180.0 / M_PI * axp.j_max_rad_s3);
  }
  if (!ok) return false;
  PayloadProfileStore store(dir);
  std::string serr;
  if (!store.save(prof, serr)) {
    std::fprintf(stderr, "save failed: %s\n", serr.c_str());
    return false;
  }
  std::printf("saved profile '%s' -> %s/%s.yaml\n", name.c_str(), dir.c_str(),
              name.c_str());
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  bool sim = false;
  std::string config_path = "config/turret.yaml";
  std::string dir = "config/payload_profiles";
  std::string name;
  std::string notes;
  std::string command;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", what);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--sim") sim = true;
    else if (a == "--config") config_path = next("--config");
    else if (a == "--dir") dir = next("--dir");
    else if (a == "--name") name = next("--name");
    else if (a == "--notes") notes = next("--notes");
    else if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
    else if (!a.empty() && a[0] == '-') {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      return 2;
    } else if (command.empty()) command = a;
    else {
      std::fprintf(stderr, "unexpected argument: %s\n", a.c_str());
      return 2;
    }
  }
  if (command.empty()) { print_usage(argv[0]); return 2; }

  config::TurretConfig cfg;
  if (!sim) {
    config::LoadResult lr = config::load_turret_config(config_path);
    if (!lr.ok) {
      for (const auto& e : lr.errors) std::fprintf(stderr, "config: %s\n", e.c_str());
      return 1;
    }
    cfg = lr.config;
    if (name.empty()) name = cfg.payload.active_profile;
    if (dir.empty()) dir = cfg.payload.profile_dir;
  } else {
    if (name.empty()) name = "sim";
  }

  if (command == "list") return cmd_list(dir) ? 0 : 1;

  std::unique_ptr<MotorBackend> backend;
  if (sim) {
    std::printf("[sim] rehearsal on the simulated plant — no CAN, no motors\n");
    backend = std::make_unique<ota::sim::SimMotorBackend>();
    std::string perr;
    for (int i = 0; i < kAxisCount; ++i)
      backend->enter_position_mode(static_cast<AxisId>(i), deg2rad(10.0), perr);
  } else {
    // Real mode: own the CAN bus like controld does (§4.1).
    can::CyberGearSystem system;
    can::CyberGearSystemConfig scfg;
    scfg.iface = cfg.can.interface;
    scfg.bitrate = static_cast<uint32_t>(cfg.can.bitrate);
    scfg.host_can_id = static_cast<uint8_t>(cfg.can.host_can_id);
    scfg.pitch_motor_id = static_cast<uint8_t>(cfg.motors[0].can_id);
    scfg.yaw_motor_id = static_cast<uint8_t>(cfg.motors[1].can_id);
    std::string err;
    if (!system.open(scfg, err)) {
      std::fprintf(stderr, "CAN open failed: %s\n", err.c_str());
      return 1;
    }
    backend = std::make_unique<CanMotorBackend>(system);
    BootFsm boot(*backend, BootConfig{});
    while (!boot.ready_to_home() && !boot.faulted()) boot.step();
    if (boot.faulted()) {
      std::fprintf(stderr, "boot fault: %s\n", boot.error());
      backend->deenergize(AxisId::Pitch);
      backend->deenergize(AxisId::Yaw);
      return 1;
    }
    std::printf("[real] CAN bus owned; single-owner rule — controld must NOT run\n");
  }

  if (command == "profile")
    return cmd_profile(std::move(backend), cfg, dir, name, notes, sim) ? 0 : 1;
  if (command == "verify")
    return cmd_verify(std::move(backend), dir, name, sim) ? 0 : 1;
  std::fprintf(stderr, "unknown command: %s\n", command.c_str());
  return 2;
}
