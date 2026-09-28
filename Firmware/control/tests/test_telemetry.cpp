// Unit tests for the telemetry store (architecture §6.3, §43): the snapshot,
// the high-rate control log, the event log, and the black-box ring buffer
// (all allocation-bounded ring buffers).
#include <gtest/gtest.h>

#include "telemetry/telemetry.hpp"

#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

namespace {
using ota::telemetry::ControlLogRecord;
using ota::telemetry::Event;
using ota::telemetry::Telemetry;
using ota::telemetry::TelemetrySnapshot;

TEST(Telemetry, ControlTraceRowCarriesItsPhaseAndThermalByte) {
  // A row that cannot say which phase it came from is the reason the 2026-09-28
  // no-progress trip could not be classified: "held with a stale request" and
  // "commanded against the envelope" look identical without the phase, and the
  // fixes those two need are opposites. -1 must survive too: it means "no such
  // byte on this axis", which is information, not absence of information.
  Telemetry t;
  ControlLogRecord r;
  r.timestamp_ns = 12345;
  r.phase = ota::Phase::Hold;
  r.temp_raw[0] = -1;   // pitch: CyberGear has no raw thermal byte
  r.temp_raw[1] = 28;   // yaw: the GM6020 byte, unit-less by contract
  t.push_control(r);
  const auto rows = t.control_log().all();
  ASSERT_EQ(rows.size(), 1u);
  const ControlLogRecord& back = t.control_log().newest();
  EXPECT_EQ(rows[0].timestamp_ns, 12345);
  EXPECT_EQ(back.phase, ota::Phase::Hold);
  EXPECT_EQ(back.temp_raw[0], -1);
  EXPECT_EQ(back.temp_raw[1], 28);
}

TEST(Telemetry, AFreezeKeepsTheCyclesThatLedToTheTrip) {
  // The export ring is 256 rows (1.28 s) and the loop keeps publishing while the
  // station sits fault-locked, so a trip that nobody reads within a second and a
  // half loses the very cycles that explain it. The freeze takes its rows from the
  // deep ring, so it holds more than any live reader could have caught.
  Telemetry t;
  for (int i = 0; i < 900; ++i) {
    ControlLogRecord r;
    r.timestamp_ns = 1000 + i;
    t.push_control(r);
  }
  t.freeze_control_trace();
  ASSERT_TRUE(t.trace_frozen());
  const int64_t frozen_last = t.control_window().frozen_t_ns;
  EXPECT_EQ(frozen_last, 1000 + 899);
  EXPECT_TRUE(t.control_window().frozen);
  EXPECT_EQ(t.control_window().rows.size(), 900u);

  // Two full live windows of noise later, the answer must not have moved.
  for (int i = 0; i < 600; ++i) {
    ControlLogRecord r;
    r.timestamp_ns = 5000000 + i;
    t.push_control(r);
  }
  EXPECT_EQ(t.control_window().rows.size(), 900u);
  EXPECT_EQ(t.control_window().frozen_t_ns, frozen_last);
  // The live ring moved on, which is the whole reason the freeze exists.
  EXPECT_GT(t.control_trace().back().timestamp_ns, 1000 + 899);

  t.clear();
  EXPECT_FALSE(t.trace_frozen());
  EXPECT_TRUE(t.control_window().frozen == false);
}

TEST(Telemetry, AStopRecordCarriesWhatTheBackendWasToldAndHowHardItPushed) {
  // `cmd` is the loop's intent; these two are the last hops before the motor. Without them a
  // stop record cannot separate "we never asked for speed" from "we asked and the axis did
  // not answer" -- the question the 2026-09-28 manual-jog measurement hinged on, when the
  // station could show a slow yaw and no number anywhere said what had been requested.
  // An axis whose drive does not report answers null, which is the truth; a zero would
  // claim we asked for nothing.
  const std::string dir = "/tmp/ota_trace_be_test_" + std::to_string(::getpid());
  Telemetry t;
  t.set_trace_archive_dir(dir);
  ControlLogRecord r;
  r.timestamp_ns = 5550001112223334445LL % 555000111222333444LL;
  r.backend_cmd[0] = 0.5;                                        // exactly representable
  r.backend_cmd[1] = std::numeric_limits<double>::quiet_NaN();   // drive silent about it
  r.drive_out[0] = 4662.0;
  r.drive_out[1] = std::numeric_limits<double>::quiet_NaN();
  r.phase = ota::Phase::Fault;
  t.push_control(r);
  t.freeze_control_trace();
  std::string path;
  ASSERT_TRUE(t.trace_archive_path(path));
  std::ifstream in(path);
  ASSERT_TRUE(in.good()) << path;
  std::string header, row;
  std::getline(in, header);
  std::getline(in, row);
  EXPECT_NE(std::string::npos, row.find("\"be_cmd\":[0.5,null]")) << row;
  EXPECT_NE(std::string::npos, row.find("\"vout\":[4662,null]")) << row;
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

TEST(Telemetry, AFrozenWindowAlsoReachesDiskAndSaysWhere) {
  // The socket answer is only there for someone who asks, and asking is exactly
  // what nobody can promise at three in the morning. The freeze therefore also
  // lands a file — and if the disk refuses, that must be a shrug, not a new way
  // for a trip to fail.
  const std::string dir = "/tmp/ota_trace_archive_test_" + std::to_string(::getpid());
  Telemetry t;
  t.set_trace_archive_dir(dir);
  ControlLogRecord r;
  r.timestamp_ns = 12345678901234567LL;
  r.command_seq = 18446744073709551615ULL;
  r.phase = ota::Phase::Fault;
  r.mode = ota::OperatingMode::AutoRoam;
  r.effort[1] = std::numeric_limits<double>::quiet_NaN();
  t.push_control(r);
  t.freeze_control_trace();

  std::string path;
  ASSERT_TRUE(t.trace_archive_path(path));
  std::ifstream in(path);
  ASSERT_TRUE(in.good()) << path;
  std::string header, row;
  std::getline(in, header);
  std::getline(in, row);
  // The header has to carry its own translation to wall clock. Rows are CLOCK_MONOTONIC
  // nanoseconds and a monotonic clock restarts at every boot, so a file that outlives the
  // boot it describes is otherwise unplaceable in time. The first two on-station files had
  // none of this and no test noticed: the archive directory's name was quietly doing the
  // job, which is exactly the kind of dependency that breaks during an incident.
  EXPECT_NE(header.find("\"clock\":\"CLOCK_MONOTONIC\""), std::string::npos) << header;
  EXPECT_NE(header.find("\"boot_id\":\""), std::string::npos) << header;
  EXPECT_NE(header.find("\"mono_to_wall_ns\":\""), std::string::npos) << header;
  const std::string wall_key = "\"wall_t_ns\":\"";
  const auto wall_at = header.find(wall_key);
  ASSERT_NE(wall_at, std::string::npos) << header;
  // Quoted, because an ns-class wall clock exceeds 2^53; and a real wall clock, because
  // 0 -- the shape of "nobody set it" -- must fail here, not three weeks later in an
  // incident review that wonders what hour the turret stopped at.
  const long long wall = std::stoll(header.substr(wall_at + wall_key.size()));
  EXPECT_GT(wall, 1000000000000000000LL) << header;
  // ns and the 64-bit sequence are decimal strings here (§2): this file outlives
  // the process, and uptime-class timestamps must not lose their low digits.
  EXPECT_NE(row.find("\"t\":\"12345678901234567\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"ack\":\"18446744073709551615\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"phase\":\"fault\""), std::string::npos) << row;
  EXPECT_NE(row.find("\"mode\":\"AUTO_ROAM\""), std::string::npos) << row;
  // Adjacency, not mere presence: the first on-station trip file had
  // "track":"search,"phase" -- the key was there and every substring test passed
  // while the line was not parseable JSON. Pin the boundary, or the test is decoration.
  EXPECT_NE(row.find("\"track\":\""), std::string::npos) << row;
  EXPECT_NE(row.find("\",\"phase\":\""), std::string::npos) << row;
  {
    int quotes = 0;
    for (char c : row) if (c == '"') ++quotes;
    EXPECT_EQ(quotes % 2, 0) << "unbalanced quotes (a missing closing quote is how "
                                "the first field after track broke): " << row;
  }
  EXPECT_NE(row.find("[0,null]"), std::string::npos) << row;
  EXPECT_EQ(row.find("nan"), std::string::npos) << row;
  EXPECT_NE(header.find("\"frozen_t_ns\":\"12345678901234567\""), std::string::npos) << header;
  ::remove(path.c_str());

  Telemetry blocked;
  blocked.set_trace_archive_dir("/proc/definitely-not-writable/traces");
  blocked.push_control(r);
  blocked.freeze_control_trace();          // must not throw
  EXPECT_TRUE(blocked.trace_frozen());     // the in-memory answer still works
  std::string none;
  EXPECT_FALSE(blocked.trace_archive_path(none));
}

TEST(Telemetry, SnapshotIsOverwrittenEachCycle) {
  Telemetry t;
  TelemetrySnapshot s;
  s.timestamp_ns = 100;
  s.q_yaw_rad = 0.5;
  t.set_snapshot(s);
  EXPECT_EQ(t.snapshot().timestamp_ns, 100);
  EXPECT_NEAR(t.snapshot().q_yaw_rad, 0.5, 1e-9);
  s.timestamp_ns = 200;
  s.q_pitch_rad = -0.25;
  t.set_snapshot(s);
  EXPECT_EQ(t.snapshot().timestamp_ns, 200);
  EXPECT_NEAR(t.snapshot().q_pitch_rad, -0.25, 1e-9);
}

TEST(Telemetry, FullWidthUuidTextPreservesBothHalves) {
  char text[ota::telemetry::kUuidTextLen] = {};
  ota::telemetry::format_uuid_text(text, UINT64_MAX, UINT64_MAX);
  EXPECT_STREQ(text, "18446744073709551615:18446744073709551615");
}

TEST(Telemetry, ControlLogKeepsLastN) {
  Telemetry t;
  const auto cap = Telemetry::kControlLogCap;
  for (int64_t i = 0; i < cap + 100; ++i) {
    ControlLogRecord r;
    r.timestamp_ns = i;
    t.push_control(r);
  }
  auto all = t.control_log().all();
  EXPECT_EQ(all.size(), cap);
  // Oldest kept record is (cap+100) - cap = 100.
  EXPECT_EQ(all.front().timestamp_ns, 100);
  EXPECT_EQ(all.back().timestamp_ns, cap + 99);
  const auto trace = t.control_trace();
  ASSERT_EQ(trace.size(),256u);
  EXPECT_EQ(trace.back().timestamp_ns,cap+99);
  EXPECT_EQ(trace.front().timestamp_ns,cap+100-256);
}

TEST(Telemetry, EventLogRecordsEvents) {
  Telemetry t;
  t.push_event(1, Event::TargetAcquired, "track=3");
  t.push_event(2, Event::TargetLost, "coast timeout");
  auto all = t.event_log().all();
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].event, Event::TargetAcquired);
  EXPECT_EQ(all[0].detail, "track=3");
  EXPECT_EQ(all[1].event, Event::TargetLost);
}

TEST(Telemetry, BlackBoxRingBounded) {
  Telemetry t;
  const auto cap = Telemetry::kBlackBoxCap;
  for (int64_t i = 0; i < cap + 50; ++i) {
    ControlLogRecord r;
    r.timestamp_ns = i;
    t.push_blackbox(r);
  }
  EXPECT_EQ(t.blackbox().size(), cap);
  auto all = t.blackbox().all();
  EXPECT_EQ(all.back().timestamp_ns, cap + 49);
}

TEST(Telemetry, ClearResetsEverything) {
  Telemetry t;
  TelemetrySnapshot s;
  s.timestamp_ns = 5;
  t.set_snapshot(s);
  t.push_control(ControlLogRecord{});
  t.push_event(1, Event::Shutdown, "");
  t.clear();
  EXPECT_EQ(t.control_log().size(), 0u);
  EXPECT_EQ(t.event_log().size(), 0u);
  EXPECT_EQ(t.snapshot().timestamp_ns, 0);
  EXPECT_TRUE(t.control_trace().empty());
}

}  // namespace

// The clock-jump rule, tested as arithmetic rather than as a clock: this service does
// not call settimeofday (and under tonight's no-sudo rule could not), so the bump
// decision has to be reachable without moving the world clock. A missed bump is a
// silent re-basing of history; a false one is a statistic that stops being joinable.
TEST(ClockEpoch, OnlyAStepBumpsIt) {
  EXPECT_EQ(1u, ota::telemetry::Telemetry::clock_epoch_after(1, 100, 140, false));
  EXPECT_EQ(1u, ota::telemetry::Telemetry::clock_epoch_after(1, 100, 100, true));
  // Scheduler movement, not a clock step.
  EXPECT_EQ(1u, ota::telemetry::Telemetry::clock_epoch_after(
                   1, 0, 9 * ota::telemetry::Telemetry::kClockJumpResolutionNs / 10, true));
  // An NTP-class step bumps, and only once per observation.
  EXPECT_EQ(2u, ota::telemetry::Telemetry::clock_epoch_after(
                    1, 0, 11 * ota::telemetry::Telemetry::kClockJumpResolutionNs / 10, true));
  EXPECT_EQ(7u, ota::telemetry::Telemetry::clock_epoch_after(
                    6, 0, 50 * ota::telemetry::Telemetry::kClockJumpResolutionNs, true));
}

// The published header must carry the epoch and the error bound, not just the offset:
// an offset without a bound is a number without a warranty, and the contract says the
// mapping needs an identity so cross-epoch joins can be refused instead of noticed late.
TEST(TripFileHeader, DeclaresEpochAndErrorBound) {
  namespace fs = std::filesystem;
  const auto dir = fs::temp_directory_path() / "ota-clock-epoch";
  fs::remove_all(dir);
  ota::telemetry::Telemetry t;
  t.set_trace_archive_dir(dir.string());
  ota::telemetry::ControlLogRecord r;
  r.timestamp_ns = 1;
  t.push_control(r);
  t.freeze_control_trace();
  std::string path;
  ASSERT_TRUE(t.trace_archive_path(path));
  std::ifstream in(path);
  std::string header;
  std::getline(in, header);
  EXPECT_NE(header.find("\"clock_epoch\":1"), std::string::npos) << header;
  const std::string key = "\"mono_to_wall_err_ns\":\"";
  const auto at = header.find(key);
  ASSERT_NE(at, std::string::npos) << header;
  // A bound big enough to be useless (or zero, which would claim we know the wall
  // clock to the nanosecond) both fail here: the honest answer is tens of microseconds.
  const long long bound = std::stoll(header.substr(at + key.size()));
  EXPECT_GT(bound, 0);
  EXPECT_LT(bound, 100 * 1000 * 1000);
  fs::remove_all(dir);
}
