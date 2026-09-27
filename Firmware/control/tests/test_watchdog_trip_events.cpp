// WP1a: a watchdog trip must arrive with its cause. These tests hold the two
// invariants the 2026-09-27 yaw trip violated: every event kind renders as a
// word (never UNKNOWN), and the vocabulary is append-only so an older log line
// read by a newer build still means what it said when it was written.
#include <cstdio>
#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include "control/mixed_can_motor_backend.hpp"
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
  // The guard writes through format_trip_detail, which truncates rather than
  // overruns; the fullest field matrix must still fit its declared purpose of
  // naming the cause and the context. 88 characters into a 96-byte buffer is
  // honest but not roomy, so the exact line is pinned here.
  MotorBackend::TripInputs in;
  in.temp_raw_over = true;
  in.temp_raw = 45;
  in.feedback_age_ms = 4.863;
  in.speed_deg_s = -3.217;
  in.reference_valid = false;
  MotorBackend::TripDetail td{};
  MotorBackend::format_trip_detail(in, MotorBackend::select_trip_condition(in), td);
  EXPECT_TRUE(td.valid);
  EXPECT_STREQ("temp_raw_over", td.condition);
  EXPECT_STREQ("cond=temp_raw_over fb_age_ms=4.863 temp_raw=45 speed_deg_s=-3.217 "
               "can_down=0 ref_valid=0",
               td.detail);
}

TEST(WatchdogTripEvents, AStateThatCannotLatchIsNeverNamedAsTheCause) {
  // reference_valid is not one of the guard's should_stop disjuncts. Naming it as
  // a cause would blame a bystander, so with nothing latching the answer is
  // "unknown" — the selector must not invent a culprit from context fields.
  MotorBackend::TripInputs in;
  in.reference_valid = false;
  EXPECT_STREQ("unknown", MotorBackend::select_trip_condition(in));
}

TEST(WatchdogTripEvents, ABystanderDoesNotShadowTheConditionThatFired) {
  // The failure this replaces: an invalid reference used to be tested before the
  // CAN checks, so a bus that fell off the network was reported as a reference
  // problem. Order now follows the guard, and context cannot win.
  MotorBackend::TripInputs in;
  in.reference_valid = false;
  in.can_down = true;
  EXPECT_STREQ("can_down", MotorBackend::select_trip_condition(in));
}

TEST(WatchdogTripEvents, SeveralConditionsReportTheFirstInGuardOrder) {
  // The matrix carries the rest; the token is the first condition the guard would
  // have stopped on, so the order is part of the contract, not an accident.
  MotorBackend::TripInputs in;
  in.no_progress = true;
  in.heartbeat_stale = true;
  in.temp_raw_over = true;
  EXPECT_STREQ("temp_raw_over", MotorBackend::select_trip_condition(in));
  in.temp_raw_over = false;
  EXPECT_STREQ("no_progress", MotorBackend::select_trip_condition(in));
}

// Three trips this morning were read as "commanded 10 deg/s and the axis refused". The
// backend had in fact sent a zero -- the command was refused upstream -- while the
// requested-speed field still quoted the last accepted cycle. An inference must not
// outrank the more specific fact available, so a refused command names itself.
TEST(WatchdogTripEvents, ARefusedCommandOutranksTheParalysisInference) {
  MotorBackend::TripInputs in;
  in.command_not_sent = true;
  in.no_progress = true;          // both are literally true; the specific one speaks first
  EXPECT_STREQ("command_not_sent", MotorBackend::select_trip_condition(in));
  in.no_progress = false;
  EXPECT_STREQ("command_not_sent", MotorBackend::select_trip_condition(in));
  // A refusal is still not allowed to masquerade as a bus or thermal problem.
  in.can_down = true;
  EXPECT_STREQ("can_down", MotorBackend::select_trip_condition(in));
}


// A ceiling's job is to clamp the ask, not to punish a reading (owner ruling 2026-09-28:
// power removal is the last resort, and an unpowered unbalanced payload drops onto a
// hard stop). So the numbers that used to trip now pass through a clamp -- and a request
// inside the ceiling passes through untouched, which is the half that paralysis bugs hide.
TEST(WatchdogTripEvents, TheCeilingClampsTheAskAndPowersNothingOff) {
  const double d2r = 3.14159265358979323846 / 180.0;
  EXPECT_DOUBLE_EQ(30.0 * d2r, apply_yaw_speed_ceiling(45.0 * d2r));
  EXPECT_DOUBLE_EQ(-30.0 * d2r, apply_yaw_speed_ceiling(-90.0 * d2r));
  EXPECT_DOUBLE_EQ(10.0 * d2r, apply_yaw_speed_ceiling(10.0 * d2r));
  EXPECT_DOUBLE_EQ(0.0, apply_yaw_speed_ceiling(0.0));
  EXPECT_EQ(30.0, kYawSpeedCeilingDegS);  // matched to pitch's declared maximum
}


// The owner's ordering of 2026-09-28, as a table instead of a paragraph: running beats
// holding, holding beats faulting, and Fault is only ever "not under control", "the motor
// says it is hot", or something as dangerous. Every row below is a condition that used to
// latch -- and on a station whose payload drops onto a hard stop whenever power goes, the
// latch was the expensive part, not the diagnosis.
static ota::GuardResponse resp(std::function<void(MotorBackend::TripInputs&)> set, int streak = 0) {
  MotorBackend::TripInputs in; set(in); return yaw_guard_response(in, streak);
}
TEST(WatchdogTripEvents, OnlyLossOfControlOrHeatMayFault) {
  EXPECT_EQ(ota::GuardResponse::Fault, resp([](MotorBackend::TripInputs& i){ i.feedback_unsafe = true; }));
  EXPECT_EQ(ota::GuardResponse::Fault, resp([](MotorBackend::TripInputs& i){ i.can_down = true; }));
  EXPECT_EQ(ota::GuardResponse::Fault, resp([](MotorBackend::TripInputs& i){ i.heartbeat_stale = true; }));
  EXPECT_EQ(ota::GuardResponse::Fault, resp([](MotorBackend::TripInputs& i){ i.temp_raw_over = true; }));
  // Everything that used to cost a power cut now costs a log line.
  EXPECT_EQ(ota::GuardResponse::Run, resp([](MotorBackend::TripInputs& i){ i.can_counters_bad = true; }));
  EXPECT_EQ(ota::GuardResponse::Run, resp([](MotorBackend::TripInputs& i){ i.bus_unhealthy = true; }));
  EXPECT_EQ(ota::GuardResponse::Run, resp([](MotorBackend::TripInputs& i){ i.speed_not_finite = true; }));
  EXPECT_EQ(ota::GuardResponse::Run, resp([](MotorBackend::TripInputs& i){ i.command_not_sent = true; }));
  EXPECT_EQ(ota::GuardResponse::Run,
            resp([](MotorBackend::TripInputs& i){ i.no_progress = true; }, ota::kYawStallHoldStreak - 1));
  // A stall repeated becomes a Hold -- powered, not pushing -- and never a Fault: an axis
  // that is not moving is not on its way to an endstop.
  EXPECT_EQ(ota::GuardResponse::Hold,
            resp([](MotorBackend::TripInputs& i){ i.no_progress = true; }, ota::kYawStallHoldStreak));
  EXPECT_EQ(ota::GuardResponse::Run, resp([](MotorBackend::TripInputs&){}));
}

// Freshness is a number, so it gets numbers: ten cycles of a 200 Hz loop.
TEST(WatchdogTripEvents, AStaleDemandIsNotThisCyclesDemand) {
  EXPECT_TRUE(yaw_command_is_stale(1'000'000'000, 0));                    // never commanded
  EXPECT_FALSE(yaw_command_is_stale(1'000'000'000, 999'990'000));          // 10 ms ago
  EXPECT_FALSE(yaw_command_is_stale(1'000'000'000, 950'000'000));          // exactly at the limit
  EXPECT_TRUE(yaw_command_is_stale(1'000'000'000, 949'999'000));           // one ns past it
}


}  // namespace
