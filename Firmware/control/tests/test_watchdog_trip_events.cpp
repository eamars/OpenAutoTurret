// WP1a: a watchdog trip must arrive with its cause. These tests hold the two
// invariants the 2026-09-27 yaw trip violated: every event kind renders as a
// word (never UNKNOWN), and the vocabulary is append-only so an older log line
// read by a newer build still means what it said when it was written.
#include <cstdio>
#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include "control/motor_backend.hpp"
#include "telemetry/telemetry.hpp"

namespace {

using namespace ota;
using telemetry::Event;

TEST(WatchdogTripEvents, TheNewKindRendersAsItsWireWord) {
  EXPECT_STREQ("MOTOR_WATCHDOG_TRIP", telemetry::event_name(Event::MotorWatchdogTrip));
}

TEST(WatchdogTripEvents, AppendedNotInterleaved) {
  // Every kind before the trip event keeps its number; the trip event is last.
  EXPECT_EQ(0, static_cast<int>(Event::TargetAcquired));
  EXPECT_EQ(static_cast<int>(Event::StopMotion) + 1,
            static_cast<int>(Event::MotorWatchdogTrip));
}

TEST(WatchdogTripEvents, NoKindRendersAsUnknown) {
  // The switch covers the enum: an event that shows UNKNOWN on the dashboard is
  // an event nobody can act on, which is the incident's whole complaint.
  for (int raw = 0; raw <= static_cast<int>(Event::MotorWatchdogTrip); ++raw) {
    const std::string name = telemetry::event_name(static_cast<Event>(raw));
    EXPECT_NE("UNKNOWN", name) << "enum value " << raw << " has no wire word";
    EXPECT_FALSE(name.empty()) << "enum value " << raw << " has an empty name";
  }
}

TEST(WatchdogTripEvents, ABackendWithoutAGuardReportsNoCause) {
  // The default must be "no claim", never a fabricated one: a backend that
  // never latched a trip must not talk the loop into blaming some condition.
  MotorBackend::TripDetail detail{};
  EXPECT_FALSE(detail.valid);
  EXPECT_EQ('\0', detail.condition[0]);
}

TEST(WatchdogTripEvents, DetailBuffersHoldTheFullestHonestLine) {
  // The guard writes with snprintf, which truncates rather than overruns; the
  // fullest field matrix must still fit its declared purpose of naming the
  // cause and the axis. This is the exact string the trip event will carry.
  MotorBackend::TripDetail td{};
  td.valid = true;
  std::snprintf(td.condition, sizeof(td.condition), "%s", "temp_raw_over");
  std::snprintf(td.detail, sizeof(td.detail),
                "cond=%s fb_age_ms=%.3f temp_raw=%u speed_deg_s=%.3f can_up=%d",
                "temp_raw_over", 4.863, 45, -3.217, 1);
  EXPECT_STREQ("temp_raw_over", td.condition);
  EXPECT_STREQ("cond=temp_raw_over fb_age_ms=4.863 temp_raw=45 speed_deg_s=-3.217 can_up=1",
               td.detail);
}

}  // namespace
