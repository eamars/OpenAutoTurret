// Transport test for the controld-side web server (§6.1, §42.2).
// Uses raw POSIX sockets as the webd stand-in: NO CAN, NO motor, NO camera.
#include "web/web_server.hpp"

#include <gtest/gtest.h>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

using namespace ota;
using namespace ota::web;

namespace {

// Connect a client to the UDS server; returns an fd (caller closes).
int connect_client(const std::string& path) {
  int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  EXPECT_GE(fd, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  EXPECT_EQ(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  return fd;
}

// Read one full message (SOCK_SEQPACKET preserves boundaries).
bool read_message(int fd, std::string& out, int timeout_ms = 1000) {
  pollfd pfd{fd, POLLIN, 0};
  int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr != 1) return false;
  char buf[8192];
  ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
  if (n <= 0) return false;
  out.assign(buf, static_cast<size_t>(n));
  return true;
}

bool send_message(int fd, const std::string& msg) {
  return ::send(fd, msg.data(), msg.size(), MSG_NOSIGNAL) ==
         static_cast<ssize_t>(msg.size());
}

telemetry::TelemetrySnapshot sample_snapshot() {
  telemetry::TelemetrySnapshot s;
  s.timestamp_ns = 123456789;
  s.track_state = tracking::TrackState::Tracking;
  s.tracking_active = true;
  s.target_confidence = 0.91;
  s.q_yaw_rad = 0.1;
  s.q_pitch_rad = -0.2;
  s.base_pitch_rad = 0.1;
  s.installation_calibrated = true;
  s.installation_source = static_cast<int8_t>(PoseSource::VisualCalibration);
  s.safety_action = SafetyAction::Allow;
  s.control_cycle_us = 4990;
  // A live yousee link with a few corrupted frames, so the §55 CAN family is
  // exercised with values that are not all zero (zeros pass any test by luck).
  s.can_available = true;
  s.can_kind = "yousee";
  s.can_device = "/dev/ttyUSB0";
  s.can_up = true;
  s.can_state = 2;  // CanIfState::ErrorPassive
  s.can_rx_frames = 4021;
  s.can_rx_error_frames = 17;
  s.can_tx_frames = 8000;
  s.can_tx_failed = 1;
  s.can_last_rx_age_ms = 4;
  return s;
}

}  // namespace

TEST(WebServer, PublishesTelemetryJson) {
  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test1.sock";
  cfg.telemetry_hz = 50;
  WebServer server(cfg, [] { return sample_snapshot(); },
                   [](const std::string&, const std::string&) {
                     CommandResult r;
                     r.ok = true;
                     return r;
                   });
  std::string err;
  ASSERT_TRUE(server.start(err)) << err;

  int cfd = connect_client(cfg.socket_path);
  std::string msg;
  ASSERT_TRUE(read_message(cfd, msg));
  // Verify it parses as our telemetry object with the right fields.
  EXPECT_NE(msg.find("\"type\":\"telemetry\""), std::string::npos);
  EXPECT_NE(msg.find("\"ts_ns\":123456789"), std::string::npos);
  EXPECT_NE(msg.find("\"track_state\":\"tracking\""), std::string::npos);
  EXPECT_NE(msg.find("\"target_confidence\":0.91"), std::string::npos);
  EXPECT_NE(msg.find("\"base_pitch_rad\":0.1"), std::string::npos);
  EXPECT_NE(msg.find("\"installation_source\":\"visual_calibration\""),
            std::string::npos);
  EXPECT_NE(msg.find("\"safety_action\":\"ALLOW\""), std::string::npos);
  // §55 CAN family: the counters the transport keeps must reach the wire.
  EXPECT_NE(msg.find("\"can_available\":true"), std::string::npos);
  EXPECT_NE(msg.find("\"can_kind\":\"" + std::string("yousee") + "\""),
            std::string::npos);
  EXPECT_NE(msg.find("\"can_device\":\"" + std::string("/dev/ttyUSB0") + "\""),
            std::string::npos);
  EXPECT_NE(msg.find("\"can_state\":2"), std::string::npos);
  EXPECT_NE(msg.find("\"can_rx_frames\":4021"), std::string::npos);
  EXPECT_NE(msg.find("\"can_rx_error_frames\":17"), std::string::npos);
  EXPECT_NE(msg.find("\"can_tx_frames\":8000"), std::string::npos);
  EXPECT_NE(msg.find("\"can_tx_failed\":1"), std::string::npos);
  EXPECT_NE(msg.find("\"can_last_rx_age_ms\":4"), std::string::npos);
  ::close(cfd);
  server.stop();
}

TEST(WebServer, NoBusIsPublishedAsAbsenceNotAsZeroHealth) {
  // The simulated backend has no CAN link. Publishing rx=0/tx=0/state=0 with
  // can_available unset would read as "a quiet, error-active bus" to both the
  // dashboard and the acceptance extractor - so the snapshot must say so.
  telemetry::TelemetrySnapshot bare;   // exactly what a sim run yields
  EXPECT_FALSE(bare.can_available);
  EXPECT_EQ(bare.can_state, -1);
  EXPECT_EQ(bare.can_last_rx_age_ms, -1);

  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test_can.sock";
  cfg.telemetry_hz = 50;
  WebServer server(cfg, [] { return telemetry::TelemetrySnapshot{}; },
                   [](const std::string&, const std::string&) {
                     CommandResult r;
                     r.ok = true;
                     return r;
                   });
  std::string err;
  ASSERT_TRUE(server.start(err)) << err;
  int cfd = connect_client(cfg.socket_path);
  std::string msg;
  ASSERT_TRUE(read_message(cfd, msg));
  EXPECT_NE(msg.find("\"can_available\":false"), std::string::npos);
  EXPECT_NE(msg.find("\"can_state\":-1"), std::string::npos);
  EXPECT_NE(msg.find("\"can_last_rx_age_ms\":-1"), std::string::npos);
  ::close(cfd);
  server.stop();
}

TEST(WebServer, CommandRoundTripOk) {
  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test2.sock";
  std::string seen_command, seen_arg;
  WebServer server(
      cfg, [] { return telemetry::TelemetrySnapshot{}; },
      [&](const std::string& c, const std::string& a) {
        seen_command = c;
        seen_arg = a;
        CommandResult r;
        r.ok = (c == "start_tracking");
        if (!r.ok) r.error = "not homed (position validity unknown)";
        return r;
      });
  std::string err;
  ASSERT_TRUE(server.start(err)) << err;

  int cfd = connect_client(cfg.socket_path);
  // Drain one telemetry message.
  std::string msg;
  ASSERT_TRUE(read_message(cfd, msg));
  EXPECT_EQ(msg.rfind("{\"type\":\"telemetry\"", 0), 0);

  // Send a command that the mock accepts.
  ASSERT_TRUE(send_message(cfd,
                           R"({"type":"command","command":"start_tracking"})"));
  ASSERT_TRUE(read_message(cfd, msg));
  EXPECT_NE(msg.find("\"type\":\"response\""), std::string::npos);
  EXPECT_NE(msg.find("\"ok\":true"), std::string::npos);
  EXPECT_EQ(seen_command, "start_tracking");

  // Send a command that the mock rejects.
  ASSERT_TRUE(send_message(
      cfd, R"({"type":"command","command":"select_target","arg":"2"})"));
  // The next message may be a telemetry tick or the response; scan a few.
  bool got_response = false;
  for (int i = 0; i < 5 && !got_response; ++i) {
    if (!read_message(cfd, msg)) break;
    if (msg.find("\"type\":\"response\"") != std::string::npos) {
      EXPECT_NE(msg.find("\"ok\":false"), std::string::npos);
      EXPECT_NE(msg.find("not homed"), std::string::npos);
      EXPECT_EQ(seen_command, "select_target");
      EXPECT_EQ(seen_arg, "2");
      got_response = true;
    }
  }
  EXPECT_TRUE(got_response);
  ::close(cfd);
  server.stop();
}

TEST(WebServer, MultipleClientsAllReceiveTelemetry) {
  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test3.sock";
  cfg.telemetry_hz = 50;
  cfg.max_clients = 4;
  WebServer server(cfg, [] { return sample_snapshot(); },
                   [](const std::string&, const std::string&) {
                     CommandResult r;
                     r.ok = true;
                     return r;
                   });
  std::string err;
  ASSERT_TRUE(server.start(err)) << err;

  const int N = 4;
  std::vector<int> fds;
  for (int i = 0; i < N; ++i) fds.push_back(connect_client(cfg.socket_path));
  // The accept thread may still be registering clients; poll briefly.
  for (int i = 0; i < 100 && server.connected_clients() < N; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(server.connected_clients(), N);
  for (int i = 0; i < N; ++i) {
    std::string msg;
    ASSERT_TRUE(read_message(fds[i], msg));
    EXPECT_NE(msg.find("\"type\":\"telemetry\""), std::string::npos);
    ::close(fds[i]);
  }
  server.stop();
}

TEST(WebServer, CleanStop) {
  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test4.sock";
  WebServer server(cfg, [] { return telemetry::TelemetrySnapshot{}; },
                   [](const std::string&, const std::string&) {
                     CommandResult r;
                     r.ok = true;
                     return r;
                   });
  std::string err;
  ASSERT_TRUE(server.start(err)) << err;
  int cfd = connect_client(cfg.socket_path);
  std::string msg;
  ASSERT_TRUE(read_message(cfd, msg));
  server.stop();  // must not hang, must join threads
  // After stop, a new connection should fail (socket removed).
  int bad = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, cfg.socket_path.c_str(), sizeof(addr.sun_path) - 1);
  EXPECT_NE(::connect(bad, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  ::close(bad);
  ::close(cfd);
  // Re-starting on the same path must also work.
  WebServer server2(cfg, [] { return telemetry::TelemetrySnapshot{}; },
                    [](const std::string&, const std::string&) {
                      CommandResult r;
                      r.ok = true;
                      return r;
                    });
  ASSERT_TRUE(server2.start(err)) << err;
  server2.stop();
}
