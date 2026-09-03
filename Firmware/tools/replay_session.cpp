// §81: the command line an operator stands at when they say "replay what it did".
//
//   replay_session [options] <session.txt>
//   replay_session --help
//
// The transcript goes to stdout; refusals and the reason the tool gave up go to stderr.
// Exit codes are for scripts: 0 ran, 2 the script is bad, 3 the station never became
// ready, 4 an --expect string was not in the transcript.
//
// What this is *for*: a session that surprised someone on the station gets written down —
// or, more often, exported from whatever was recorded — and replayed here where the
// controller runs at any speed and nothing moves. If the same transcript comes out, the
// question becomes a code question. If it does not, the recording has something the bench
// does not: a different configuration, a different pose, a camera that sees something else.
// Both answers are worth an afternoon.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "control/session_replay.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz, the period the loop expects
constexpr int kMaxHomeSteps = 60000;  // 300 s of simulated homing

void usage() {
  std::cout <<
      "usage: replay-session [options] <session.txt>\n"
      "\n"
      "  --verbose         the loop's per-cycle motion and supervisor log on stderr\n"
      "  --expect SUBSTR   fail (exit 4) unless SUBSTR appears in the transcript.\n"
      "                    Repeatable. This is how the tool is used from a test.\n"
      "  --coast-ms N      §20 coast window          --lost-hold-ms N   hold window\n"
      "  --reacquire-ms N  §20 reacquisition window  --high-min F       §19 HIGH floor\n"
      "  --threshold F     §21 reacquire threshold   --margin F         §21 ambiguity\n"
      "  --lease-ms N      §38 jog lease             --keepalive-ms N   its renewal rate\n"
      "  --steps S[,S]     §41 step sizes offered, from 0.5 / 1 / 5\n"
      "\n"
      "The station under replay is built from these options, NOT from the station's\n"
      "config/turret.yaml. That is a real limit and it is stated rather than hidden: a\n"
      "replay whose envelope and timings differ from the machine that produced the\n"
      "recording is comparing two different stations. The header this tool prints names\n"
      "every value it used, so a mismatch is visible in the same window as the transcript\n"
      "instead of becoming a mystery three days later. Reading turret.yaml here is the\n"
      "obvious next step; it is not done yet, and half-doing it silently would be worse.\n";
}

struct Options {
  std::string path;
  std::vector<std::string> expect;
  ControlLoop::Config cfg;
};

}  // namespace

int main(int argc, char** argv) {
  Options o;
  bool verbose = false;
  spdlog::set_level(spdlog::level::warn);  // see --verbose
  o.cfg.control_hz = 200;
  o.cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  o.cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  o.cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  o.cfg.park.park_logical_deg = {30.0, 60.0};
  o.cfg.park.speed_deg_s = 50.0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "replay-session: " << name << " needs a value\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--verbose" || a == "-v") {
      // The loop's per-cycle logging goes to stdout, and the transcript goes to stdout,
      // and the two are not the same thing. Off by default, one flag away: the motion log
      // is the first thing to reach for when a replay does *not* match the station, and
      // the last thing to want when you are reading 200 lines of phase changes.
      spdlog::set_level(spdlog::level::info);
      verbose = true;
    } else if (a == "--help" || a == "-h") {
      usage();
      return 0;
    } else if (a == "--expect") {
      o.expect.emplace_back(next("--expect"));
    } else if (a == "--coast-ms") {
      o.cfg.auto_track_coast_ms = atoll(next("--coast-ms"));
    } else if (a == "--lost-hold-ms") {
      o.cfg.auto_track_lost_hold_ms = atoll(next("--lost-hold-ms"));
    } else if (a == "--reacquire-ms") {
      o.cfg.auto_track_reacquire_window_ms = atoll(next("--reacquire-ms"));
    } else if (a == "--high-min") {
      o.cfg.auto_track_high_min = static_cast<float>(atof(next("--high-min")));
    } else if (a == "--threshold") {
      o.cfg.reacquire_threshold = static_cast<float>(atof(next("--threshold")));
    } else if (a == "--margin") {
      o.cfg.ambiguous_match_margin = static_cast<float>(atof(next("--margin")));
    } else if (a == "--lease-ms") {
      o.cfg.manual_lease_ms = atoi(next("--lease-ms"));
    } else if (a == "--keepalive-ms") {
      o.cfg.manual_keepalive_ms = atoi(next("--keepalive-ms"));
    } else if (a == "--steps") {
      std::stringstream ss(next("--steps"));
      std::string tok;
      while (std::getline(ss, tok, ',')) o.cfg.step_sizes_deg.push_back(atof(tok.c_str()));
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "replay-session: unknown option '" << a << "' (--help)\n";
      return 2;
    } else {
      o.path = a;
    }
  }
  if (o.path.empty()) {
    usage();
    return 2;
  }

  std::ifstream f(o.path);
  if (!f.is_open()) {
    std::cerr << "replay-session: cannot open " << o.path << "\n";
    return 2;
  }
  std::stringstream buf;
  buf << f.rdbuf();
  ReplayScript script;
  std::string err;
  if (!parse_replay_script(buf.str(), script, err)) {
    std::cerr << "replay-session: " << o.path << ": " << err << "\n";
    return 2;
  }

  // The station under replay: a simulated plant with end stops, homed by the ordinary
  // homing plan, so the replay inherits §22's endpoint discovery rather than being told
  // where the limits are. That matters more than it sounds — the roam envelope and the
  // refusals are computed from the limits the station *measured*, and a tool that assumed
  // them would be replaying sessions on a machine that cannot exist.
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);
  ControlLoop loop(o.cfg, std::move(backend));

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
  HomingPlan plan(std::move(actions), hcfg);

  std::string herr;
  if (!loop.start_homing(plan, herr)) {
    std::cerr << "replay-session: homing could not start: " << herr << "\n";
    return 3;
  }
  int64_t t = 0;
  int i = 0;
  for (; i < kMaxHomeSteps; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
    if (loop.phase() == Phase::Fault) break;
    if (loop.homed() && loop.at_ready()) break;
  }
  if (!(loop.homed() && loop.at_ready())) {
    std::cerr << "replay-session: the replay station never reached ready ("
              << loop.fault_reason() << ")\n";
    return 3;
  }

  std::printf("# replay-session %s: %zu frames, %zu operator actions\n", o.path.c_str(),
              script.frames.size(), script.events.size());
  std::printf("# station under replay: simulated plant, homed by %s; timings from options "
              "or built-in defaults, NOT from turret.yaml\n",
              "the ordinary homing plan");
  std::printf(
      "# coast=%lldms lost_hold=%lldms reacquire=%lldms high_min=%.2f threshold=%.2f "
      "margin=%.2f lease=%dms steps=%zu\n",
      static_cast<long long>(o.cfg.auto_track_coast_ms),
      static_cast<long long>(o.cfg.auto_track_lost_hold_ms),
      static_cast<long long>(o.cfg.auto_track_reacquire_window_ms),
      static_cast<double>(o.cfg.auto_track_high_min), static_cast<double>(o.cfg.reacquire_threshold),
      static_cast<double>(o.cfg.ambiguous_match_margin), o.cfg.manual_lease_ms,
      o.cfg.step_sizes_deg.size());

  const ReplayResult res = replay_session(loop, script, t);
  for (const auto& line : res.lines) std::printf("%s\n", line.c_str());
  if (!res.ok) {
    std::cerr << "replay-session: " << res.error << "\n";
    return 3;
  }

  std::string transcript;
  for (const auto& l : res.lines) transcript += l + "\n";
  if (verbose) std::fprintf(stderr, "replay-session: %zu transcript lines\n", res.lines.size());
  int rc = 0;
  for (const auto& e : o.expect) {
    if (transcript.find(e) == std::string::npos) {
      std::cerr << "replay-session: expected '" << e << "' in the transcript, and it is "
                "not there\n";
      rc = 4;
    }
  }
  return rc;
}
