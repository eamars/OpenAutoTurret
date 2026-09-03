// Vision ingest tests (architecture §6.1, Part-2 task S1).
// Raw POSIX sockets stand in for visiond: NO CAN, NO motor, NO camera.
#include <vector>
#include "tracks/track_wire.hpp"
#include "vision/vision_ingest.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "common/time.hpp"
#include "gtest/gtest.h"

namespace ota {
namespace {

// Mirror of web_server's test helper: SEQPACKET client, like visiond's
// IpcPublisher.
int connect_client(const std::string& path) {
  int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (fd < 0) return -1;
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

vision::TargetMeasurement make_measurement(uint64_t seq) {
  vision::TargetMeasurement m;
  m.frame_sequence = seq;
  m.sensor_timestamp_ns = 1234567890ULL + seq * 33'000'000ULL;
  m.valid = true;
  m.class_id = 1;  // "person"
  m.confidence = 0.75f;
  m.bbox_x_min_norm = 0.40f;
  m.bbox_y_min_norm = 0.30f;
  m.bbox_x_max_norm = 0.50f;
  m.bbox_y_max_norm = 0.60f;
  m.anchor_u_px = 960.5f;
  m.anchor_v_px = 540.25f;
  m.has_track_id = true;
  m.visual_track_id = 42;
  return m;
}

// Wait for a predicate the way the control thread sees it: never sleep a fixed
// amount, always bound the wait.
template <typename F>
bool wait_for(F pred, int timeout_ms = 2000) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

class VisionIngestTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_.socket_path = "/tmp/ota_vision_unit_test.sock";
    ::unlink(cfg_.socket_path.c_str());
  }
  void TearDown() override { ::unlink(cfg_.socket_path.c_str()); }

  vision::VisionIngest::Config cfg_;
  vision::VisionLink link_;
};

TEST_F(VisionIngestTest, DeliversDecodedMeasurementToHandler) {
  std::atomic<int> received{0};
  vision::TargetMeasurement got{};
  vision::VisionIngest ingest(cfg_, &link_, [&](const vision::TargetMeasurement& m) {
    got = m;
    received.fetch_add(1);
  });
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  ASSERT_TRUE(ingest.running());

  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  const auto wire = make_measurement(7).encode();
  ASSERT_EQ(::send(cfd, wire.data(), wire.size(), 0),
            static_cast<ssize_t>(wire.size()));

  ASSERT_TRUE(wait_for([&] { return received.load() == 1; }));
  // Byte-exact round trip through the 58-byte wire format (§6.2).
  EXPECT_EQ(got.frame_sequence, 7u);
  EXPECT_EQ(got.sensor_timestamp_ns, 1234567890ULL + 7 * 33'000'000ULL);
  EXPECT_TRUE(got.valid);
  EXPECT_EQ(got.class_id, 1);
  EXPECT_FLOAT_EQ(got.confidence, 0.75f);
  EXPECT_FLOAT_EQ(got.anchor_u_px, 960.5f);
  EXPECT_TRUE(got.has_track_id);
  EXPECT_EQ(got.visual_track_id, 42u);

  // The lock-free link counters the §6.3 snapshot reads.
  auto s = link_.stats();
  EXPECT_TRUE(s.connected);
  EXPECT_EQ(s.clients, 1);
  EXPECT_EQ(s.frames, 1u);
  EXPECT_EQ(s.dropped, 0u);
  EXPECT_EQ(s.last_frame_sequence, 7u);
  EXPECT_GT(s.last_arrival_ns, 0);

  ::close(cfd);
  ingest.stop();
}

TEST_F(VisionIngestTest, ShortDatagramIsDroppedNotParsed) {
  std::atomic<int> received{0};
  vision::VisionIngest ingest(cfg_, &link_,
                              [&](const vision::TargetMeasurement&) {
                                received.fetch_add(1);
                              });
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);

  const uint8_t junk[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  ASSERT_EQ(::send(cfd, junk, sizeof(junk), 0), 10);
  ASSERT_TRUE(wait_for([&] { return link_.stats().dropped == 1u; }));
  EXPECT_EQ(received.load(), 0) << "a malformed frame must never reach the loop";

  // A good frame after the junk still works (no desync: SEQPACKET keeps
  // datagram boundaries, so a bad frame cannot swallow the next one).
  const auto wire = make_measurement(3).encode();
  ASSERT_EQ(::send(cfd, wire.data(), wire.size(), 0),
            static_cast<ssize_t>(wire.size()));
  ASSERT_TRUE(wait_for([&] { return received.load() == 1; }));
  auto s = link_.stats();
  EXPECT_EQ(s.frames, 1u);
  EXPECT_EQ(s.dropped, 1u);
  EXPECT_EQ(s.last_frame_sequence, 3u);

  ::close(cfd);
  ingest.stop();
}

TEST_F(VisionIngestTest, OversizedDatagramIsDropped) {
  vision::VisionIngest ingest(cfg_, &link_, [](const vision::TargetMeasurement&) {});
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  // The reader buffer is 128 B: a larger datagram truncates/mismatches and must
  // be counted, never half-parsed.
  uint8_t big[200];
  std::memset(big, 0xAB, sizeof(big));
  ASSERT_EQ(::send(cfd, big, sizeof(big), 0), static_cast<ssize_t>(sizeof(big)));
  ASSERT_TRUE(wait_for([&] { return link_.stats().dropped == 1u; }));
  EXPECT_EQ(link_.stats().frames, 0u);
  ::close(cfd);
  ingest.stop();
}

TEST_F(VisionIngestTest, SecondPublisherIsRejected) {
  // visiond is the only publisher (§5.1): a second one would interleave
  // measurements with no ordering, so it is accepted and immediately dropped.
  cfg_.max_clients = 1;
  vision::VisionIngest ingest(cfg_, &link_, [](const vision::TargetMeasurement&) {});
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;

  int a = connect_client(cfg_.socket_path);
  ASSERT_GE(a, 0);
  ASSERT_TRUE(wait_for([&] { return link_.stats().clients == 1; }));
  int b = connect_client(cfg_.socket_path);
  ASSERT_GE(b, 0);  // accepted at the listen backlog, then closed
  ASSERT_TRUE(wait_for([&] { return link_.stats().clients == 1; }, 500));
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_EQ(link_.stats().clients, 1) << "the extra publisher must not stay";

  // The first publisher still counts.
  const auto wire = make_measurement(1).encode();
  ASSERT_EQ(::send(a, wire.data(), wire.size(), 0),
            static_cast<ssize_t>(wire.size()));
  ASSERT_TRUE(wait_for([&] { return link_.stats().frames == 1u; }));

  ::close(a);
  ::close(b);
  ingest.stop();
}

TEST_F(VisionIngestTest, StopUnlinksTheSocketAndCountsDown) {
  vision::VisionIngest ingest(cfg_, &link_, [](const vision::TargetMeasurement&) {});
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  ASSERT_TRUE(wait_for([&] { return link_.stats().connected; }));

  ingest.stop();
  EXPECT_FALSE(ingest.running());
  // A fresh connection must now fail: the socket file is gone.
  EXPECT_LT(connect_client(cfg_.socket_path), 0);
  // stop() joins the reader threads, so the count has already settled.
  EXPECT_EQ(link_.stats().clients, 0);
  ::close(cfd);
  // stop() is idempotent.
  ingest.stop();
}

TEST_F(VisionIngestTest, RestartRebindsTheSamePath) {
  // A daemon restart must not need a manual `rm` of the socket file (start()
  // unlinks a stale path first).
  vision::VisionIngest ingest(cfg_, &link_, [](const vision::TargetMeasurement&) {});
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  ingest.stop();
  vision::VisionIngest again(cfg_, &link_, [](const vision::TargetMeasurement&) {});
  ASSERT_TRUE(again.start(err)) << err;
  EXPECT_GE(connect_client(cfg_.socket_path), 0);
  again.stop();
}

TEST(VisionLinkCounters, TracksFramesDropsAndClients) {
  vision::VisionLink link;
  EXPECT_FALSE(link.stats().connected);
  EXPECT_EQ(link.stats().clients, 0);

  link.note_frame(10, 1000);
  link.note_frame(11, 2000);
  link.note_dropped();
  auto s = link.stats();
  EXPECT_EQ(s.frames, 2u);
  EXPECT_EQ(s.dropped, 1u);
  EXPECT_EQ(s.last_frame_sequence, 11u);
  EXPECT_EQ(s.last_arrival_ns, 2000);

  link.note_client_added();
  EXPECT_TRUE(link.stats().connected);
  link.note_client_removed();
  EXPECT_FALSE(link.stats().connected);
  // Over-removal must not underflow into a huge "client count".
  link.note_client_removed();
  EXPECT_EQ(link.stats().clients, 0);
}

TEST(VisionLinkCounters, AgeIsMeasuredOnTheControlClock) {
  // The snapshot's age must come from the SAME clock the motor history uses
  // (CLOCK_MONOTONIC): a measurement stamped 50 ms ago has to age 50 ms.
  vision::VisionLink link;
  const TimeNs t0 = now_monotonic_ns();
  link.note_frame(5, t0);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const TimeNs age_ns = now_monotonic_ns() - link.stats().last_arrival_ns;
  EXPECT_GE(age_ns, 50'000'000);
  EXPECT_LT(age_ns, 500'000'000);
}


// --- v3 §59/§60: the TrackSet datagram on the same socket -----------------

TEST_F(VisionIngestTest, TrackSetDatagramReachesTheTrackSetHandler) {
  std::atomic<int> measurements{0};
  std::atomic<int> track_sets{0};
  tracks::TrackSet got{};
  vision::VisionIngest ingest(
      cfg_, &link_,
      [&](const vision::TargetMeasurement&) { measurements.fetch_add(1); },
      [&](const tracks::TrackSet& s, TimeNs) {
        got = s;
        track_sets.fetch_add(1);
      });
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;

  tracks::TrackSet in;
  in.frame_sequence = 42;
  in.sensor_timestamp_ns = 1234567890;
  in.publish_timestamp_ns = 1234567890 + 8'000'000;
  in.width = 1280;
  in.height = 720;
  tracks::Track t;
  t.uuid = tracks::TrackUuid{7, 1};
  t.display_index = 1;
  t.class_id = 1;
  std::memcpy(t.class_name, "person", 6);
  t.state = tracks::TrackState::Confirmed;
  t.anchor_x = 0.2f;
  t.anchor_y = 0.35f;
  in.add(t);

  uint8_t wire[tracks::kTrackSetWireSize];
  ASSERT_EQ(tracks::encode_track_set(in, wire, sizeof(wire)),
            tracks::kTrackSetWireSize);
  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  ASSERT_EQ(::send(cfd, wire, sizeof(wire), 0),
            static_cast<ssize_t>(sizeof(wire)));

  ASSERT_TRUE(wait_for([&] { return track_sets.load() == 1; }));
  EXPECT_EQ(measurements.load(), 0)
      << "a TrackSet was routed to the single-target handler";
  EXPECT_EQ(got.frame_sequence, 42u);
  EXPECT_EQ(got.width, 1280u);
  ASSERT_EQ(got.count, 1);
  EXPECT_EQ(got.tracks[0].uuid, (tracks::TrackUuid{7, 1}));
  EXPECT_STREQ(got.tracks[0].class_name, "person");

  // §61: the stamps the latency telemetry is built from, recorded on the transport
  // thread that actually saw them.
  auto s = link_.stats();
  EXPECT_EQ(s.track_sets, 1u);
  EXPECT_EQ(s.frames, 1u);
  EXPECT_EQ(s.last_sensor_ns, 1234567890);
  EXPECT_EQ(s.last_publish_ns, 1234567890 + 8'000'000);
  ::close(cfd);
  ingest.stop();
}

TEST_F(VisionIngestTest, BothGenerationsShareTheSocketAndAreToldApartByLength) {
  // The compatibility rule that lets controld be upgraded before visiond, exercised on
  // a real socket: interleaved v1 and v3 datagrams plus one piece of junk, and every
  // one goes to the right place or is counted as dropped.
  std::atomic<int> measurements{0};
  std::atomic<int> track_sets{0};
  vision::VisionIngest ingest(
      cfg_, &link_,
      [&](const vision::TargetMeasurement&) { measurements.fetch_add(1); },
      [&](const tracks::TrackSet&, TimeNs) { track_sets.fetch_add(1); });
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;

  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  const auto v1 = make_measurement(11).encode();
  ASSERT_EQ(::send(cfd, v1.data(), v1.size(), 0),
            static_cast<ssize_t>(v1.size()));

  tracks::TrackSet in;
  in.frame_sequence = 12;
  in.width = 640;
  in.height = 480;
  tracks::Track t;
  t.uuid = tracks::TrackUuid{1, 9};
  t.class_id = 1;
  t.state = tracks::TrackState::Confirmed;
  in.add(t);
  uint8_t wire[tracks::kTrackSetWireSize];
  ASSERT_EQ(tracks::encode_track_set(in, wire, sizeof(wire)),
            tracks::kTrackSetWireSize);
  ASSERT_EQ(::send(cfd, wire, sizeof(wire), 0),
            static_cast<ssize_t>(sizeof(wire)));

  const uint8_t junk[200] = {0};
  ASSERT_EQ(::send(cfd, junk, sizeof(junk), 0),
            static_cast<ssize_t>(sizeof(junk)));

  ASSERT_TRUE(wait_for([&] {
    return measurements.load() == 1 && track_sets.load() == 1;
  }));
  auto s = link_.stats();
  EXPECT_EQ(s.frames, 2u) << "both generations count as frames";
  EXPECT_EQ(s.track_sets, 1u);
  EXPECT_EQ(s.dropped, 1u);
  EXPECT_EQ(s.last_frame_sequence, 12u) << "the newest message wins, as in v1";
  ::close(cfd);
  ingest.stop();
}

TEST_F(VisionIngestTest, ADatagramLargerThanTheBufferIsNotTruncatedIntoData) {
  // The buffer is larger than the largest valid message, and a read that fills it
  // completely is treated as truncation rather than parsed. A truncated TrackSet that
  // still happened to parse would report a scene that lost the tracks it dropped —
  // including the one somebody is pointing at.
  std::atomic<int> track_sets{0};
  vision::VisionIngest ingest(cfg_, &link_,
                              [](const vision::TargetMeasurement&) {},
                              [&](const tracks::TrackSet&, TimeNs) {
                                track_sets.fetch_add(1);
                              });
  std::string err;
  ASSERT_TRUE(ingest.start(err)) << err;
  int cfd = connect_client(cfg_.socket_path);
  ASSERT_GE(cfd, 0);
  std::vector<uint8_t> huge(8192, 0xAB);
  tracks::TrackSet in;
  in.frame_sequence = 5;
  uint8_t wire[tracks::kTrackSetWireSize];
  tracks::encode_track_set(in, wire, sizeof(wire));
  std::memcpy(huge.data(), wire, sizeof(wire));
  ASSERT_EQ(::send(cfd, huge.data(), huge.size(), 0),
            static_cast<ssize_t>(huge.size()));
  ASSERT_TRUE(wait_for([&] { return link_.stats().dropped >= 1u; }));
  EXPECT_EQ(track_sets.load(), 0) << "an oversized datagram was delivered as a TrackSet";
  ::close(cfd);
  ingest.stop();
}
}  // namespace
}  // namespace ota
