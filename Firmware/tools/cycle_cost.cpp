// §94 / §46: what a control cycle actually costs, per mode, measured rather than assumed.
//
//   cycle-cost [--cycles N] [--config PATH] [--assert-p99-us LIMIT]
//
// Why this exists as a program and not a unit test: the number that matters is the shape of
// a distribution under a named condition, and a gtest that asserts on wall-clock time is a
// test that fails on a loaded laptop and passes on an idle one. So the tool prints what it
// measured, and `--assert-p99-us` turns it into a gate with a ceiling loose enough to be
// about *orders of magnitude* — the failure it is meant to catch is a change that puts an
// allocation, a file write, or a socket call on the control path, which shows up as a factor
// of ten, not as 40 µs of host jitter.
//
// What these numbers are, precisely — because a number printed without its conditions is an
// invitation to misuse it:
//
//   * They are the cost of `ControlLoop::step()` on **this host, in simulation**, timed
//     around the call. The station's own 5.06 ms p50 / 5.09 ms p99 belongs to the pre-v3
//     binary on the Pi, and nothing here replaces measuring v3 there.
//   * What is simulated is the plant, not the bus: the sim backend answers instantly, so a
//     real cycle additionally pays whatever the MCP2515/yousee exchange costs. The v3 modes
//     are not the difference between this and that; the same modes on the same backend are
//     the comparison that matters, and that comparison needs the station.
//   * The value of doing this in simulation is that it isolates what v3 *added*: the mode
//     machine, the selection manager, the track-set ingest, the event tail, the black-box
//     copy. If one of those has become expensive, it is visible here, in the mode that uses
//     it, without a motor anywhere near it.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"
#include "common/timing_stats.hpp"
#include "config/station_wiring.hpp"
#include "config/turret_config.hpp"
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"
#include "tracks/track_set.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

using namespace ota;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz
constexpr int kMaxHomeSteps = 60'000;

std::chrono::steady_clock::time_point t0() {
  return std::chrono::steady_clock::now();
}

struct Condition {
  std::string name;
  TimingStats stats{8192};
  int64_t over_period = 0;   // slower than the 5 ms period
  int64_t over_grace = 0;    // slower than the period plus the 2 ms grace (§39.3)
  bool informational = false;  // printed, but not gated (§46's homing sleeps)
};

void sample(Condition& c, int64_t compute_ns) {
  c.stats.record_period(compute_ns);
  if (compute_ns > kDtNs) ++c.over_period;
  if (compute_ns > kDtNs + 2'000'000) ++c.over_grace;
}

void print_row(const Condition& c) {
  const TimingReport r = c.stats.report();
  std::printf("%-22s %6llu  %8.1f %8.1f %8.1f %9.1f %8lld %8lld%s\n", c.name.c_str(),
              static_cast<unsigned long long>(r.samples), r.p50_ns / 1000.0,
              r.p95_ns / 1000.0, r.p99_ns / 1000.0, r.worst_ns / 1000.0,
              static_cast<long long>(c.over_period), static_cast<long long>(c.over_grace),
              c.informational ? "   (informational: §46 recipe sleeps)" : "");
}

}  // namespace

int main(int argc, char** argv) {
  int cycles = 4000;  // 20 s of control at 200 Hz per condition
  std::string config_path;
  int64_t assert_p99_us = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* n) {
      if (i + 1 >= argc) {
        std::cerr << "cycle-cost: " << n << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--cycles") {
      cycles = std::atoi(next("--cycles"));
    } else if (a == "--config") {
      config_path = next("--config");
    } else if (a == "--assert-p99-us") {
      assert_p99_us = std::atoll(next("--assert-p99-us"));
    } else if (a == "--help" || a == "-h") {
      std::cout << "usage: cycle-cost [--cycles N] [--config PATH] "
                   "[--assert-p99-us LIMIT]\n";
      return 0;
    } else {
      std::cerr << "cycle-cost: unknown option '" << a << "'\n";
      return 2;
    }
  }

  // spdlog writes to stdout, the same stream as the table, and a preserved black-box scene
  // warns on the first watchdog trip — which is every run of this tool, because homing's
  // recipe sleep trips it. Errors still print; the events belong in the station's log, not
  // between the rows of a measurement.")

  spdlog::set_level(spdlog::level::warn);

  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  cfg.park.park_logical_deg = {30.0, 60.0};
  cfg.park.speed_deg_s = 50.0;
  std::unique_ptr<config::TurretConfig> tc;
  if (!config_path.empty()) {
    config::LoadResult lr = config::load_turret_config(config_path);
    for (const auto& e : lr.errors) std::cerr << "cycle-cost: " << e << "\n";
    if (!lr.ok) return 2;
    tc.reset(new config::TurretConfig(lr.config));
    cfg = wire::make_control_cfg(*tc);
  }

  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);
  ControlLoop loop(cfg, std::move(backend));

  HomingPlan plan({}, HomingPlanConfig{});
  if (tc) {
    std::string perr;
    plan = wire::make_homing_plan(*tc, perr);
    if (!perr.empty()) {
      std::cerr << "cycle-cost: " << perr << "\n";
      return 3;
    }
  } else {
    HomingPlanConfig hcfg;
    HomingParams hp;
    hp.coarse_speed_rad_s = 20.0 * kDeg2Rad;
    hp.fine_speed_rad_s = 2.0 * kDeg2Rad;
    hp.settle_time_s = 0.3;
    hcfg.homing = hp;
    hcfg.travel_bands[0] = TravelBand{0.0, 115.0};
    hcfg.travel_bands[1] = TravelBand{0.0, 115.0};
    std::vector<HomingAction> actions;
    actions.push_back(
        HomingAction{.type = HomingActionType::HomeFullRange, .axis = AxisId::Pitch});
    actions.push_back(
        HomingAction{.type = HomingActionType::HomeFullRange, .axis = AxisId::Yaw});
    plan = HomingPlan(std::move(actions), hcfg);
  }

  std::string herr;
  if (!loop.start_homing(plan, herr)) {
    std::cerr << "cycle-cost: homing could not start: " << herr << "\n";
    return 3;
  }

  std::vector<Condition> conds;
  conds.emplace_back();
  conds.back().name = "homing";
  conds.back().informational = true;
  Condition& homing = conds.back();

  int64_t t = 0;
  bool homed = false;
  for (int i = 0; i < kMaxHomeSteps; ++i) {
    const auto s = t0();
    loop.step(t, kDtNs);
    sample(homing, std::chrono::duration_cast<std::chrono::nanoseconds>(t0() - s).count());
    t += kDtNs;
    if (loop.homed() && loop.at_ready()) {
      homed = true;
      break;
    }
    if (loop.phase() == Phase::Fault) break;
  }
  if (!homed) {
    std::cerr << "cycle-cost: the station never reached ready (" << loop.fault_reason()
              << ")\n";
    return 3;
  }

  // A cycle at rest, which is the baseline every other row is read against: the same
  // telemetry publish, the same supervisor, the same snapshot — none of the modes.
  Condition hold{"quiet hold", TimingStats(8192), 0, 0, false};
  for (int i = 0; i < cycles; ++i) {
    const auto s = t0();
    loop.step(t, kDtNs);
    sample(hold, std::chrono::duration_cast<std::chrono::nanoseconds>(t0() - s).count());
    t += kDtNs;
  }

  // MANUAL with a live lease: the manual controller advancing and the reference moving.
  loop.submit_command("manual_jog_start", "yaw+");
  loop.step(t, kDtNs);
  t += kDtNs;
  Condition jog{"manual jog (leased)", TimingStats(8192), 0, 0, false};
  for (int i = 0; i < cycles; ++i) {
    const auto s = t0();
    loop.step(t, kDtNs);
    sample(jog, std::chrono::duration_cast<std::chrono::nanoseconds>(t0() - s).count());
    t += kDtNs;
    if (i % 40 == 0) {
      loop.submit_command("manual_jog_start", "yaw+");  // keepalive, as the page does
      loop.step(t, kDtNs);
      t += kDtNs;
    }
  }
  loop.submit_command("manual_jog_stop", "");
  loop.step(t, kDtNs);
  t += kDtNs;

  // AUTO_TRACK, following somebody, with 30 Hz detector traffic — the path that carries the
  // track-set ingest, the selection manager, the estimator and the confidence derating.
  loop.submit_command("set_mode", "AUTO_TRACK");
  loop.step(t, kDtNs);
  t += kDtNs;
  Condition track{"auto_track (30 Hz)", TimingStats(8192), 0, 0, false};
  bool selected = false;
  uint64_t seq = 1;
  for (int i = 0; i < cycles; ++i) {
    if (i % 6 == 0) {  // 33 ms of wall clock per frame at this rate
      tracks::TrackSet set;
      set.frame_sequence = seq++;
      set.sensor_timestamp_ns = t;
      set.publish_timestamp_ns = t;
      set.width = 1280;
      set.height = 720;
      set.count = 1;
      tracks::Track& tr = set.tracks[0];
      tr.uuid.lo = 11;
      tr.display_index = 1;
      tr.class_id = 1;
      std::snprintf(tr.class_name, sizeof tr.class_name, "person");
      tr.state = tracks::TrackState::Confirmed;
      tr.detector_confidence = 0.9f;
      tr.track_confidence = 0.9f;
      const double ax = 0.30 + 0.0002 * static_cast<double>(i);
      tr.anchor_x = static_cast<float>(ax);
      tr.anchor_y = 0.55f;
      tr.bbox.x_min = static_cast<float>(ax - 0.05);
      tr.bbox.x_max = static_cast<float>(ax + 0.05);
      tr.bbox.y_min = 0.45f;
      tr.bbox.y_max = 0.65f;
      tr.age_frames = static_cast<uint16_t>(20 + i);
      tr.visible_frames = static_cast<uint16_t>(20 + i);
      loop.feed_track_set(set, t);
      if (!selected) {
        loop.submit_command("select_target", "1");
        selected = true;
      }
    }
    const auto s = t0();
    loop.step(t, kDtNs);
    sample(track, std::chrono::duration_cast<std::chrono::nanoseconds>(t0() - s).count());
    t += kDtNs;
  }

  // AUTO_ROAM: the sweep, which is the mode that runs unattended for minutes.
  loop.submit_command("clear_target", "");
  loop.step(t, kDtNs);
  loop.submit_command("set_mode", "AUTO_ROAM");
  loop.step(t, kDtNs);
  t += kDtNs * 2;
  Condition roam{"auto_roam (sweeping)", TimingStats(8192), 0, 0, false};
  for (int i = 0; i < cycles; ++i) {
    const auto s = t0();
    loop.step(t, kDtNs);
    sample(roam, std::chrono::duration_cast<std::chrono::nanoseconds>(t0() - s).count());
    t += kDtNs;
  }

  conds.push_back(std::move(hold));
  conds.push_back(std::move(jog));
  conds.push_back(std::move(track));
  conds.push_back(std::move(roam));

  std::printf("# cycle-cost — ControlLoop::step() compute time, simulated plant, host clock\n");
  std::printf("# station: %s; %d cycles (20 s) per condition at 200 Hz\n",
              config_path.empty() ? "built-in defaults (not any real station)"
                                  : config_path.c_str(),
              cycles);
  std::printf("# NOT a measurement of the station: the sim backend answers instantly, so a "
              "real cycle also pays the CAN exchange. What this isolates is what the modes "
              "cost each other.\n\n");
  std::printf("%-22s %6s  %8s %8s %8s %9s %8s %8s\n", "condition", "samples", "p50 us",
              "p95 us", "p99 us", "worst us", ">period", ">grace");
  int rc = 0;
  for (const auto& c : conds) {
    print_row(c);
    if (assert_p99_us > 0 && !c.informational) {
      const TimingReport r = c.stats.report();
      if (r.p99_ns / 1000.0 > static_cast<double>(assert_p99_us)) {
        std::fprintf(stderr,
                     "cycle-cost: %s p99 is %.1f us, over the %lld us gate — something on "
                     "the control path got expensive (allocation, I/O, or a loop over "
                     "something that grows)\n",
                     c.name.c_str(), r.p99_ns / 1000.0, static_cast<long long>(assert_p99_us));
        rc = 4;
      }
    }
  }
  std::printf("\n# >period counts cycles slower than the 5 ms period; >grace counts those "
              "past the period plus the 2 ms the §39.3 watchdog allows, which is what the "
              "supervisor would see as a miss.\n");
  std::printf("# The homing row is informational: the recipe sleeps in can_motor_backend are "
              "sleep_for(50 ms) on the control thread (§46), inherited unchanged by v3 and "
              "still waiting on an operator's risk decision.\n");
  return rc;
}
