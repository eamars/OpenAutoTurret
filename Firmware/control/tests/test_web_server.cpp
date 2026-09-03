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
// Five seconds, not one. This deadline asks "does a frame arrive at all" against a publisher sending
// one every 20 ms, so the original second proved nothing about the system and something about the
// machine: on a loaded station - homing, vision, other suites - the verdict started depending on
// scheduling rather than on the code. One flake appeared and would not reproduce in three runs after,
// which is the worst property a suite can have when its red is meant to mean something. Callers that
// pass their own timeout keep it; latency claims elsewhere keep their tighter tolerances.
bool read_message(int fd, std::string& out, int timeout_ms = 5000) {
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

TEST(WebServer, ATrackIdentifierCrossesTheWireAsTextNotAsARoundNumber) {
  // §50's `track_uuid` and §78's "selected UUID". Two separate hazards in one field: the
  // identifier is 128 bits with the high half drawn from the whole 64-bit range, and every
  // consumer on the other end of this socket parses JSON into a language whose number is a
  // double, exact to 2^53. Emitted as a number it survives only when the nonce happens to be
  // small — which is every bit as bad as it sounds, because it works in every test anyone thinks
  // to write and fails on a real session, silently, at the precision where two tracks differ.
  telemetry::TelemetrySnapshot bare;
  EXPECT_FALSE(bare.selected_uuid_valid);
  EXPECT_STREQ("", bare.selected_uuid_text);  // absence is empty text, not "0:0" (§72)

  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test_uuid.sock";
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
  EXPECT_NE(msg.find("\"selected_uuid_valid\":false"), std::string::npos);
  EXPECT_NE(msg.find("\"selected_uuid\":\"\""), std::string::npos)
      << "with nobody selected the page must be handed nothing, not an identifier that looks "
         "like a real track from a previous session";
  ::close(cfd);
  server.stop();

  telemetry::TelemetrySnapshot filled;
  filled.selected_uuid_valid = true;
  telemetry::format_uuid_text(filled.selected_uuid_text, 0x9E3779B97F4A7C15ull, 31);
  WebServer::Config cfg2;
  cfg2.socket_path = "/tmp/ota_web_test_uuid2.sock";
  cfg2.telemetry_hz = 50;
  WebServer server2(cfg2, [&filled] { return filled; },
                    [](const std::string&, const std::string&) {
                      CommandResult r;
                      r.ok = true;
                      return r;
                    });
  ASSERT_TRUE(server2.start(err)) << err;
  cfd = connect_client(cfg2.socket_path);
  ASSERT_TRUE(read_message(cfd, msg));
  EXPECT_NE(msg.find("\"selected_uuid\":\"11400714819323198485:31\""), std::string::npos)
      << "the identifier reached the wire as something other than the exact text of both halves";
  ::close(cfd);
  server2.stop();
}

TEST(WebServer, AFrameLongerThanOneQueueSlotArrivesWhole) {
  // The bug this exists for, stated so it is not re-introduced by someone tidying the send path:
  // this socket is SOCK_SEQPACKET, which means one send() is one datagram — and the kernel will
  // accept a *partial* send and label whatever remains as a new datagram. The send loop that used
  // to live here wrote a long frame out in pieces. Nothing errored. The reader received a JSON
  // document sawn in half at roughly one kernel queue slot (~2 KB) and threw it away, so the
  // station's dashboard showed nothing while controld published happily at 15 Hz and logged not a
  // word. It took the v3 fields pushing the frame past that slot to expose a loop that had been
  // wrong since the day it was written.
  //
  // Which is also why this test fills the snapshot in rather than using a bare one: every other
  // test here passes a default-constructed snapshot, whose JSON is short, and "the tests were
  // green" was precisely the thing that was misleading. The assertion that the frame is *long* is
  // part of the test — if somebody shrinks the snapshot below a queue slot, this test has to fail
  // and say so, rather than quietly reverting to the case that used to pass by accident.
  telemetry::TelemetrySnapshot s;
  s.phase = "hold";
  s.operating_mode = "AUTO_TRACK";
  s.mode_phase = "TRACKING";
  s.intent_source = "auto_track";
  s.intent_type = "los_direction";
  s.intent_reason = "following the person the operator selected at 21:56:11";
  s.can_device = "/dev/ttyUSB0";
  s.can_kind = "yousee";
  s.payload_profile_name = "conservative-with-a-longer-name-for-a-reason";
  s.selected_uuid_valid = true;
  telemetry::format_uuid_text(s.selected_uuid_text, 0x9E3779B97F4A7C15ull, 31);
  s.soft_limits_valid = true;
  s.q_soft_min_pitch_rad = -0.95; s.q_soft_max_pitch_rad = 0.95;
  s.q_soft_min_yaw_rad = 2.05;   s.q_soft_max_yaw_rad = 3.10;
  for (int i = 0; i < telemetry::TelemetrySnapshot::kMaxTrackList; ++i) {
    telemetry::TrackListing& t = s.tracks[i];
    t.uuid_lo = 20 + i;
    t.uuid_hi = 0x9E3779B97F4A7C15ull;
    telemetry::format_uuid_text(t.uuid_text, t.uuid_hi, t.uuid_lo);
    t.display_index = static_cast<uint16_t>(i + 1);
    std::snprintf(t.label, sizeof t.label, "Person #%d", i + 1);
    std::snprintf(t.class_name, sizeof t.class_name, "person");
    std::snprintf(t.state, sizeof t.state, "CONFIRMED");
    t.confidence = 0.9f;
    t.bbox[0] = 0.1f; t.bbox[1] = 0.2f; t.bbox[2] = 0.3f; t.bbox[3] = 0.6f;
    t.selectable = true;
    t.selected = (i == 0);
  }
  s.track_count = telemetry::TelemetrySnapshot::kMaxTrackList;
  for (int i = 0; i < telemetry::TelemetrySnapshot::kEventTail; ++i) {
    s.event_tail[i].t_ns = 1000 * (i + 1);
    std::snprintf(s.event_tail[i].name, sizeof s.event_tail[i].name, "TARGET_TRACKING");
    std::snprintf(s.event_tail[i].detail, sizeof s.event_tail[i].detail,
                  "reacquired with a score of 0.71 against a margin of 0.15 over 42 ms of gap");
    s.event_tail_count = i + 1;
  }
  s.event_generation = 6;

  std::string wire;
  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test_bigframe.sock";
  cfg.telemetry_hz = 50;
  WebServer server(cfg, [&s] { return s; },
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

  // The premise, asserted rather than assumed: this frame is bigger than the slot that used to
  // truncate it. If this ever fails, the snapshot shrank and the rest of the test proves nothing.
  EXPECT_GT(msg.size(), 2048u)
      << "this frame is only " << msg.size()
      << " bytes — small enough to have passed under the old truncation, so the test has stopped "
         "testing anything";
  // And the point: one datagram, whole, ending where a JSON document ends.
  EXPECT_EQ(msg.back(), '}') << "the frame stops at byte " << msg.size()
                             << " with no closing brace: it was torn, and the reader cannot tell";
  EXPECT_NE(msg.find("\"selected_uuid\":\"11400714819323198485:31\""), std::string::npos)
      << "the middle of the frame is missing, which is what a torn datagram looks like from here";
  EXPECT_NE(msg.find("operating_mode"), std::string::npos);
  ::close(cfd);
  server.stop();
}

// A quote-aware bracket balance check. Not a JSON parser — the point is narrower and meaner: a
// frame whose braces and brackets do not nest cannot be JSON, and every previous test in this
// file looked for substrings, which a torn or mis-closed document can satisfy perfectly while
// being unreadable by anything on the other end of the socket.
inline bool json_balanced(const std::string& m, std::string* where) {
  std::string stack;
  bool in_string = false, escaped = false;
  for (size_t i = 0; i < m.size(); ++i) {
    const char c = m[i];
    if (in_string) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') in_string = true;
    else if (c == '{' || c == '[') stack.push_back(c);
    else if (c == '}' || c == ']') {
      if (stack.empty()) { if (where) *where = "closing with nothing open at byte " + std::to_string(i); return false; }
      const char open = stack.back();
      stack.pop_back();
      if ((c == '}' && open != '{') || (c == ']' && open != '[')) {
        if (where) *where = std::string("'") + c + "' closes a '" + open + "' at byte " + std::to_string(i);
        return false;
      }
    }
  }
  if (!stack.empty()) {
    if (where)
      *where = "frame ends with " + std::to_string(stack.size()) + " scopes still open; frame is "
               + std::to_string(m.size()) + " bytes and stops at ..." + m.substr(m.size() > 70 ? m.size() - 70 : 0);
    return false;
  }
  if (in_string) { if (where) *where = "frame ends inside a string"; return false; }
  return true;
}

TEST(WebServer, APreservedSceneDoesNotBreakTheFrameItTravelsIn) {
  // §80's black box arrived with one bracket too many at the end of its object, so every frame
  // carrying a preserved scene was unparsable — and it stayed that way from the commit that added
  // the feature until a station happened to brake at startup and a human asked why the dashboard
  // was empty. Three things conspired: the block is skipped entirely when no scene is preserved
  // (so a quiet station never showed it), the tests searched for substrings instead of parsing,
  // and webd answered the resulting JSON error by dropping the frame in silence.
  telemetry::TelemetrySnapshot s;
  s.phase = "fault";
  s.blackbox_capture_id = 7;
  telemetry::BlackBoxCapture& b = s.blackbox;
  b.id = 7;
  b.t_ns = 123456789;
  std::snprintf(b.reason, sizeof b.reason, "BRAKE in hold — stale motor feedback");
  std::snprintf(b.operating_mode, sizeof b.operating_mode, "AUTO_TRACK");
  std::snprintf(b.mode_phase, sizeof b.mode_phase, "TRACKING");
  std::snprintf(b.phase, sizeof b.phase, "hold");
  std::snprintf(b.selected_label, sizeof b.selected_label, "Person #1");
  std::snprintf(b.selected_uuid_text, sizeof b.selected_uuid_text, "11400714819323198485:11");
  std::snprintf(b.selection_visibility, sizeof b.selection_visibility, "VISIBLE");
  b.candidate_count = 3;
  for (int i = 0; i < 3; ++i) {
    telemetry::TrackListing& t = b.candidates[i];
    t.uuid_lo = 11 + i;
    t.uuid_hi = 0x9E3779B97F4A7C15ull;
    telemetry::format_uuid_text(t.uuid_text, t.uuid_hi, t.uuid_lo);
    std::snprintf(t.label, sizeof t.label, "Person #%d", i + 1);
    std::snprintf(t.state, sizeof t.state, "CONFIRMED");
  }

  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test_capture.sock";
  cfg.telemetry_hz = 50;
  WebServer server(cfg, [&s] { return s; },
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

  std::string where;
  EXPECT_TRUE(json_balanced(msg, &where))
      << "the frame is not well-formed with a capture in it: " << where
      << " — this is the shape that left a real station's dashboard blank";
  EXPECT_NE(msg.find("\"blackbox\""), std::string::npos) << "no capture in the frame at all";
  EXPECT_NE(msg.find("Person #3"), std::string::npos) << "the candidates went missing";
  // The rest of the frame still has to survive the black box being appended to it: the whole
  // point of §80 is that the scene arrives *with* the state, not instead of it.
  EXPECT_NE(msg.find("\"phase\""), std::string::npos);
  ::close(cfd);
  server.stop();
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

TEST(WebServer, ModeIntentAndCommandAckReachTheWire) {
  // The v3 fields cross a process boundary into a browser. Asserting them here,
  // on the actual bytes, is what keeps the C++ struct and the Python dataclass
  // from drifting apart in silence — the failure mode that already bit this
  // project once, in the other direction, when a telemetry field the dashboard
  // asked for had never existed on the wire.
  telemetry::TelemetrySnapshot s = sample_snapshot();
  s.operating_mode = "AUTO_TRACK";
  s.supervisory_state = "READY";
  s.mode_phase = "COAST";
  s.intent_source = "auto_track";
  s.intent_type = "los_direction";
  s.intent_reason = "coasting";
  s.intent_velocity_scale = 0.42;
  {
    // One listed candidate with a box, so the key that the overlay draws from is asserted
    // rather than assumed: `bbox` is what turns a marker into a rectangle, and a page that
    // reads a key nobody emits draws nothing and looks like it is working.
    telemetry::TrackListing& tr = s.tracks[0];
    s.track_count = 1;
    tr.display_index = 2;
    std::snprintf(tr.label, sizeof tr.label, "Person #2");
    std::snprintf(tr.state, sizeof tr.state, "CONFIRMED");
    tr.anchor_x = 0.4f;
    tr.anchor_y = 0.5f;
    tr.bbox[0] = 0.30f; tr.bbox[1] = 0.40f; tr.bbox[2] = 0.50f; tr.bbox[3] = 0.70f;
    tr.selectable = true;
    tr.selected = true;
  }
  s.aim_point_valid = true;
  s.aim_point_x = 0.42;
  s.aim_point_y = 0.31;
  s.intent_has_joint_target = true;
  s.intent_q_yaw_rad = 0.25;
  s.intent_q_pitch_rad = -0.1;
  s.cmd_ack_command = "set_mode";
  s.cmd_ack_accepted = 0;  // refused
  s.cmd_ack_reason = "station is not homed";
  s.cmd_ack_controller_state = "hold/AUTO_TRACK";
  s.cmd_ack_safety_state = "ALLOW";
  s.cmd_ack_seq = 7;

  WebServer::Config cfg;
  cfg.socket_path = "/tmp/ota_web_test_ack.sock";
  cfg.telemetry_hz = 50;
  WebServer server(cfg, [&] { return s; },
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

  EXPECT_NE(msg.find("\"operating_mode\":\"AUTO_TRACK\""), std::string::npos);
  EXPECT_NE(msg.find("\"supervisory_state\":\"READY\""), std::string::npos);
  EXPECT_NE(msg.find("\"mode_phase\":\"COAST\""), std::string::npos);
  EXPECT_NE(msg.find("\"intent_source\":\"auto_track\""), std::string::npos);
  EXPECT_NE(msg.find("\"intent_type\":\"los_direction\""), std::string::npos);
  EXPECT_NE(msg.find("\"intent_reason\":\"coasting\""), std::string::npos);
  // The scale matters as much as the names: a reference that was derated for low
  // confidence has to look derated, or the operator reads a tracking problem as a
  // latency problem.
  EXPECT_NE(msg.find("\"intent_velocity_scale\":0.42"), std::string::npos);
  // §92's requested pose has to survive the trip, because the page cannot show what did not
  // arrive — and it has to arrive *with its flag*. When no pose was asked for, the pose is
  // sent as zero rather than as whatever the last mode left behind: a stale number with a
  // false flag is still a stale number, and the reader that trusts the flag should never
  // have to be cleverer than the writer.
  // §73's reticle: the key has to arrive, and it has to arrive gated by its flag for the
  // same reason as every other optional quantity here — an aim point from a session that
  // ended is not where the turret is pointing now.
  EXPECT_NE(msg.find("\"aim_point_valid\":true"), std::string::npos);
  EXPECT_NE(msg.find("\"aim_point_x\":0.42"), std::string::npos);
  EXPECT_NE(msg.find("\"aim_point_y\":0.31"), std::string::npos);
  {
    telemetry::TelemetrySnapshot blind = s;
    blind.aim_point_valid = false;  // the numbers are still in the fields
    EXPECT_NE(web::format_telemetry(blind).find("\"aim_point_x\":0"), std::string::npos)
        << "a stale aim point was published as if it were live";
  }
  EXPECT_NE(msg.find("\"intent_has_joint_target\":true"), std::string::npos);
  EXPECT_NE(msg.find("\"bbox\":[0.300000,0.400000,0.500000,0.700000]"), std::string::npos)
      << "the candidate box is not on the wire; §73's overlay would be drawing from "
         "nothing. Got: " + msg.substr(msg.find("\"tracks\""), 260);
  EXPECT_NE(msg.find("\"intent_q_yaw_rad\":0.25"), std::string::npos);
  {
    telemetry::TelemetrySnapshot bare = s;
    bare.intent_has_joint_target = false;
    bare.intent_q_yaw_rad = 0.25;  // what the previous mode asked for, still in the field
    const std::string bare_msg = web::format_telemetry(bare);
    EXPECT_NE(bare_msg.find("\"intent_has_joint_target\":false"), std::string::npos);
    EXPECT_NE(bare_msg.find("\"intent_q_yaw_rad\":0"), std::string::npos)
        << "a pose nobody asked for was published anyway; the flag is not enough if the "
           "number keeps travelling";
  }
  EXPECT_NE(msg.find("\"cmd_ack_command\":\"set_mode\""), std::string::npos);
  EXPECT_NE(msg.find("\"cmd_ack_accepted\":0"), std::string::npos)
      << "a refusal must stay distinguishable from a success and from 'no "
         "command yet' (-1), so it cannot be flattened to a bool";
  EXPECT_NE(msg.find("\"cmd_ack_reason\":\"station is not homed\""),
            std::string::npos);
  EXPECT_NE(msg.find("\"cmd_ack_controller_state\":\"hold/AUTO_TRACK\""),
            std::string::npos);
  EXPECT_NE(msg.find("\"cmd_ack_seq\":7"), std::string::npos);
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

// --- structural validity of the telemetry line -------------------------------
//
// json_balanced above proves nesting. It cannot see the two mistakes a hand-written emitter actually
// makes. A double comma is perfectly balanced. A key emitted twice is perfectly balanced. Both
// produce a document that substring assertions accept happily while the reader on the other end
// either rejects the line outright or keeps one of the two values in silence - and until this
// function, the telemetry line was tested ONLY by substring search, which is precisely the weakness
// the comment on json_balanced calls out while continuing to do it.
//
// Not hypothetical. A nested block added to this emitter left `},` followed by `,"next":` behind, so
// the line read `...},,"next":...`. The build was clean, every CTest in this file was green, and only
// a manual look at the live payload showed that the UI's entire input was not JSON.
namespace strict {

struct Tok {
  bool is_str;
  std::string s;
  char punct;
};

inline std::vector<Tok> Tokenize(const std::string& m) {
  std::vector<Tok> out;
  std::string cur;
  bool in = false, esc = false;
  auto flush = [&]() {
    if (!cur.empty()) {
      out.push_back(Tok{false, cur, '\0'});
      cur.clear();
    }
  };
  for (size_t i = 0; i < m.size(); ++i) {
    const char c = m[i];
    if (in) {
      if (esc) { cur += c; esc = false; }
      else if (c == '\\') { cur += c; esc = true; }
      else if (c == '"') {
        out.push_back(Tok{true, cur, '\0'});
        // Clear, or the next flush() leaks this word out a second time as a bare token. That is how
        // the duplicate-key check appeared to pass: the spurious token sat where the ':' was expected,
        // so no key was ever recorded and nothing could ever be found twice. The self-test below is
        // the only reason that is visible instead of shipped.
        cur.clear();
        in = false;
      }
      else cur += c;
      continue;
    }
    if (c == '"') { flush(); in = true; cur.clear(); continue; }
    if (c == '{' || c == '}' || c == '[' || c == ']' || c == ',' || c == ':') {
      flush();
      out.push_back(Tok{false, std::string(), c});
      continue;
    }
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
    cur += c;
  }
  flush();
  return out;
}

// Rejects what balance cannot: empty elements (`{,`, `,,`, `,}`), and the same key twice in the same
// object. Key identity is qualified by the path, because `prediction.valid` and
// `field_of_regard.valid` are two different fields that both must be called `valid`.
inline bool Check(const std::string& m, std::string* where) {
  if (!json_balanced(m, where)) return false;
  const std::vector<Tok> t = Tokenize(m);
  std::vector<std::string> scopes(1, "$");
  std::vector<std::string> seen;
  auto path = [&]() {
    std::string p;
    for (size_t i = 1; i < scopes.size(); ++i) { p += scopes[i]; p += "."; }
    return p;
  };
  for (size_t i = 0; i < t.size(); ++i) {
    const Tok& k = t[i];
    if (k.punct == '{' || k.punct == '[') {
      if (i + 1 < t.size() && t[i + 1].punct == ',') {
        if (where) *where = "empty element right after '" + std::string(1, k.punct) + "' at token " +
                            std::to_string(i);
        return false;
      }
      std::string name = (k.punct == '{') ? "{}" : "[]";
      if (i >= 2 && t[i - 1].punct == ':' && t[i - 2].is_str) name = t[i - 2].s;
      scopes.push_back(name);
      continue;
    }
    if (k.punct == '}' || k.punct == ']') {
      if (i >= 1 && t[i - 1].punct == ',') {
        if (where) *where = "trailing comma before '" + std::string(1, k.punct) + "' at token " +
                            std::to_string(i);
        return false;
      }
      if (scopes.size() > 1) scopes.pop_back();
      continue;
    }
    if (k.punct == ',') {
      if (i >= 1 && t[i - 1].punct == ',') {
        if (where) *where = "empty element between two commas at token " + std::to_string(i);
        return false;
      }
      if (i + 1 < t.size() && (t[i + 1].punct == '}' || t[i + 1].punct == ']')) {
        if (where) *where = "trailing comma at token " + std::to_string(i);
        return false;
      }
      continue;
    }
    if (k.is_str && i + 1 < t.size() && t[i + 1].punct == ':') {
      const std::string id = path() + k.s;
      for (size_t j = 0; j < seen.size(); ++j) {
        if (seen[j] == id) {
          if (where) *where = "key '" + id + "' emitted twice in the same object; a reader keeps one "
                              "and the other vanishes without a trace";
          return false;
        }
      }
      seen.push_back(id);
    }
  }
  return true;
}

}  // namespace strict

TEST(WebServer, TheTelemetryLineIsStructurallyValidJson) {
  // The same populated snapshot the other tests in this file send, so the checker runs against a line
  // with real strings, enums and nested values in it rather than a bare default.
  telemetry::TelemetrySnapshot s = sample_snapshot();

  auto expect_valid = [&](const char* what) {
    const std::string msg = ota::web::format_telemetry(s);
    std::string where;
    EXPECT_TRUE(strict::Check(msg, &where)) << what << ": " << where << "\n  line: "
                                            << msg;
  };

  expect_valid("an empty snapshot");

  // The nested blocks, filled. Each is a place where a bracket or separator can go wrong, and each is
  // emitted by a chain of << that the compiler cannot check for me.
  s.prediction_valid = true;
  s.prediction_anchor_in_frame = true;
  s.prediction_los_yaw_rad = 0.06;
  s.prediction_los_pitch_rad = -0.02;
  s.prediction_anchor_x_norm = 0.531;
  s.prediction_anchor_y_norm = 0.492;
  s.prediction_horizon_ms = 40;
  expect_valid("with the §20 prediction block filled");

  const double kRad2Deg = 57.29577951308232;
  s.for_envelope_valid = true;
  s.for_envelope_kind = 1;
  s.for_envelope_count = 4;
  const double corners[8] = {-22.573, -74.712, 320.144, -74.712, 320.144, -4.891, -22.573, -4.891};
  for (int k = 0; k < 8; ++k) s.for_envelope_deg[k] = corners[k];
  (void)kRad2Deg;
  expect_valid("with the field-of-regard polygon filled");

  const std::string msg = ota::web::format_telemetry(s);
  EXPECT_NE(msg.find("\"prediction\":{"), std::string::npos) << "§20's block must be an object";
  EXPECT_NE(msg.find("\"field_of_regard\":{"), std::string::npos);
  EXPECT_NE(msg.find("\"safe_envelope_points\":[["), std::string::npos)
      << "§20 names safe_envelope_points[]; a point is a pair, so the array opens with a pair";
  EXPECT_NE(msg.find("\"coordinate_frame\":\"joint_deg\""), std::string::npos)
      << "§11.3: FOR coordinates are yaw/pitch degrees, and the page refuses to draw without being told";

  // A count of zero must emit an empty array, not an empty element. Both look similar in a diff; only
  // one of them parses.
  s.for_envelope_count = 0;
  s.prediction_valid = false;
  expect_valid("with the polygon empty");
  EXPECT_NE(ota::web::format_telemetry(s).find("\"safe_envelope_points\":[]"), std::string::npos);
}

TEST(WebServer, TheStructuralCheckerRejectsTheMistakesItExistsFor) {
  // A checker nobody has seen fail is a decoration. Each of these is balanced, so the pre-existing
  // json_balanced accepts all four; this is the only thing standing between them and a green build.
  std::string where;
  EXPECT_FALSE(strict::Check("{\"a\":1},,{\"b\":2}", &where)) << "the double comma that reached the station";
  EXPECT_FALSE(strict::Check("{\"a\":1,}", &where)) << "trailing comma";
  EXPECT_FALSE(strict::Check("{\"a\":1,\"a\":2}", &where)) << "duplicate key at the top level";
  EXPECT_TRUE(strict::Check("{\"p\":{\"valid\":true},\"f\":{\"valid\":false}}", &where))
      << "same key name in two different objects is not a duplicate";
  EXPECT_TRUE(strict::Check("{\"a\":[[1,2],[3,4]],\"b\":[]}", &where)) << "nested pairs and an empty array";
}

TEST(WebServer, TheCommandResponseSaysWhichQuestionItAnswered) {
  // A response answers "did the validation gate take this", not "did the station do it". Those are
  // different questions with different latencies, and a bare `ok` let the HUD render a queued
  // select_target as an acquired target - the operator would have seen a nonexistent person tracked.
  // The scope is on the wire now, and ok and verdict may not disagree.
  ota::web::CommandResult queued;
  queued.ok = true;
  const std::string sub = ota::web::format_response("set_mode", queued);
  EXPECT_NE(sub.find("\"verdict\":\"submitted\""), std::string::npos) << sub;
  EXPECT_NE(sub.find("\"ok\":true,\"verdict\":\"submitted\""), std::string::npos)
      << "ok and verdict must agree, or a reader has to pick a favourite: " << sub;

  ota::web::CommandResult refused;
  refused.ok = false;
  refused.error = "select_target needs the number shown on screen";
  const std::string rej = ota::web::format_response("select_target", refused);
  EXPECT_NE(rej.find("\"verdict\":\"rejected\""), std::string::npos) << rej;
  EXPECT_NE(rej.find("\"ok\":false,\"verdict\":\"rejected\""), std::string::npos) << rej;
  EXPECT_NE(rej.find("needs the number shown on screen"), std::string::npos)
      << "the refusal reason is the operator's only feedback for a gate rejection";

  // Both shapes still have to be JSON that a strict parser accepts - the response goes through the same
  // hand-written emitter as telemetry, which has already produced invalid output once this week.
  std::string where;
  EXPECT_TRUE(strict::Check(sub, &where)) << "submitted response is malformed: " << where << " in " << sub;
  EXPECT_TRUE(strict::Check(rej, &where)) << "rejected response is malformed: " << where << " in " << rej;
}
