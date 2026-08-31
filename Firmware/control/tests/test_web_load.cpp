// §54.5: multiple web clients must NOT degrade control timing or CAN-feedback
// staleness. The control loop and the controld web server coexist (the web
// server runs on a non-RT thread reading loop.telemetry()); N clients are
// connected while the loop runs. No CAN, no motor, no camera — the
// SimMotorBackend plant only.
#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"
#include "web/web_server.hpp"

#include <gtest/gtest.h>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace ota;
using namespace ota::web;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz

HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg2Rad;
  hp.fine_speed_rad_s = 2.0 * kDeg2Rad;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 115.0};
  hcfg.travel_bands[1] = TravelBand{0.0, 115.0};
  std::vector<HomingAction> actions;
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Pitch});
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Yaw});
  return HomingPlan(std::move(actions), hcfg);
}

ControlLoop::Config make_cfg() {
  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  cfg.park.park_logical_deg = {30.0, 60.0};
  cfg.park.speed_deg_s = 50.0;
  cfg.park.dwell_ms = 300;
  cfg.park.pos_tol_deg = 0.5;
  cfg.park.vel_tol_deg_s = 1.0;
  cfg.park.min_soft_margin_deg = 2.0;
  return cfg;
}

bool run_to_ready(ControlLoop& loop, int64_t& t) {
  constexpr int kMaxSteps = 60000;
  for (int i = 0; i < kMaxSteps; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
    if (loop.phase() == Phase::Fault) return false;
    if (loop.homed() && loop.at_ready()) return true;
  }
  return false;
}

int connect_client(const std::string& path) {
  int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  EXPECT_GE(fd, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  EXPECT_EQ(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  return fd;
}

// Drain + count the messages currently buffered on an fd (non-blocking).
int count_available(int fd) {
  int count = 0;
  char buf[8192];
  while (true) {
    pollfd pfd{fd, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 0);
    if (pr != 1) break;
    ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
    if (n <= 0) break;
    ++count;
  }
  return count;
}

}  // namespace

// The §54.5 guarantee: N browser clients connected to the web server must not
// slow the 200 Hz control loop, must not starve any client, and must not make
// CAN feedback stale (the control path never touches the web path).
TEST(WebLoad, MultiClientNoControlDegradation) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);
  ControlLoop loop(make_cfg(), std::move(backend));
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, t))
      << "did not reach ready; phase=" << phase_name(loop.phase())
      << " fault=" << loop.fault_reason();

  const std::string sock = "/tmp/ota_web_load.sock";
  WebServer::Config wc;
  wc.socket_path = sock;
  wc.telemetry_hz = 20;  // within the §6.3 10-20 Hz band
  WebServer web(wc,
                [&loop] { return loop.telemetry().snapshot(); },
                [&loop](const std::string& n, const std::string& a) {
                  return loop.submit_command(n, a);
                });
  ASSERT_TRUE(web.start(err)) << err;

  const int kClients = 8;
  const int kCycles = 20000;  // 100 s of sim; wall-clock is a few ms

  // Baseline: K control cycles with no clients connected.
  auto b0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kCycles; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  auto b1 = std::chrono::steady_clock::now();
  const double base_ms =
      std::chrono::duration<double, std::milli>(b1 - b0).count();

  // Load: connect N clients, then run the same K cycles.
  std::vector<int> fds;
  for (int i = 0; i < kClients; ++i) fds.push_back(connect_client(sock));
  for (int i = 0; i < 200 && web.connected_clients() < kClients; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_EQ(web.connected_clients(), kClients) << "clients did not all connect";

  auto l0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kCycles; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  auto l1 = std::chrono::steady_clock::now();
  const double load_ms =
      std::chrono::duration<double, std::milli>(l1 - l0).count();

  // The web server must not slow the control loop. Generous headroom for CI
  // noise, but a >=5x regression would indicate the web path is hogging CPU.
  EXPECT_LT(load_ms, 5.0 * base_ms + 20.0)
      << "control loop degraded with " << kClients << " clients: base="
      << base_ms << "ms load=" << load_ms << "ms";

  // The loop is still healthy: holding, homed, and feedback is fresh (a proxy
  // for CAN-feedback staleness — the web path never delays the control path).
  EXPECT_EQ(loop.phase(), Phase::Hold);
  EXPECT_TRUE(loop.homed());
  const auto snap = loop.telemetry().snapshot();
  EXPECT_LT(snap.feedback_age_ms, 100)
      << "feedback went stale under web load: " << snap.feedback_age_ms << "ms";

  // Let the web server publish to all clients, then verify none was dropped
  // (no client starvation) — every one still receives telemetry.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  for (int i = 0; i < kClients; ++i) {
    EXPECT_GT(count_available(fds[i]), 0)
        << "client " << i << " was starved (no telemetry under load)";
  }

  web.stop();
  for (int fd : fds) ::close(fd);
}
