// Unit tests for the telemetry store (architecture §6.3, §43): the snapshot,
// the high-rate control log, the event log, and the black-box ring buffer
// (all allocation-bounded ring buffers).
#include <gtest/gtest.h>

#include "telemetry/telemetry.hpp"

namespace {
using ota::telemetry::ControlLogRecord;
using ota::telemetry::Event;
using ota::telemetry::Telemetry;
using ota::telemetry::TelemetrySnapshot;

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
}

}  // namespace
