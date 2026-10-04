// SURVEILLANCE (owner, 2026-10-05): the planner that faces the watch point. The property under test
// is that it always aims at the point and never waits for an arrival: "RETURN" and "WATCH" are what
// the operator reads, not conditions the motion depends on.
#include <gtest/gtest.h>

#include <cmath>

#include "mode/surveillance_planner.hpp"

namespace ota {
namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

TEST(SurveillancePlanner, IdleInventsNothingToFace) {
  SurveillancePlanner p;
  const auto out = p.update(0.3, -0.1, 1);
  EXPECT_EQ(out.state, SurveillanceState::Idle);
  EXPECT_EQ(out.intent.type, IntentType::Hold);
  EXPECT_EQ(out.intent.source, MotionSource::Surveillance);
}

TEST(SurveillancePlanner, ItAimsAtThePointOnEveryCycleAndSaysReturnUntilThere) {
  SurveillancePlanner p;
  p.enter(40 * kDeg, 10 * kDeg);
  for (const double q : {0.0, 20 * kDeg, 38 * kDeg}) {
    const auto out = p.update(q, 0.0, 1);
    EXPECT_EQ(out.state, SurveillanceState::Return) << q;
    EXPECT_EQ(out.intent.type, IntentType::JointPosition);
    EXPECT_TRUE(out.intent.has_joint_target);
    EXPECT_DOUBLE_EQ(out.intent.q_yaw_rad, 40 * kDeg);
    EXPECT_DOUBLE_EQ(out.intent.q_pitch_rad, 10 * kDeg);
  }
}

TEST(SurveillancePlanner, AnAxisRestingShortOfThePointIsWatchingAndStillAimedAtIt) {
  // The roam planner's TURNAROUND once waited forever for a servo that rests 0.24 deg short
  // (2026-10-02). Here a rest 0.6 deg short is WATCH, and the intent is still the point itself.
  SurveillancePlanner p;
  p.enter(40 * kDeg, 10 * kDeg);
  const auto out = p.update(39.4 * kDeg, 10.3 * kDeg, 1);
  EXPECT_EQ(out.state, SurveillanceState::Watch);
  EXPECT_DOUBLE_EQ(out.intent.q_yaw_rad, 40 * kDeg);
  EXPECT_DOUBLE_EQ(out.intent.q_pitch_rad, 10 * kDeg);
}

TEST(SurveillancePlanner, TheLabelHasHysteresisSoARestingServoDoesNotFlickerIt) {
  SurveillancePlanner p;
  p.enter(0.0, 0.0);
  EXPECT_EQ(p.update(0.5 * kDeg, 0, 1).state, SurveillanceState::Watch);
  EXPECT_EQ(p.update(1.5 * kDeg, 0, 2).state, SurveillanceState::Watch) << "inside the 2 deg return band";
  EXPECT_EQ(p.update(2.5 * kDeg, 0, 3).state, SurveillanceState::Return) << "pushed off its point";
  EXPECT_EQ(p.update(1.5 * kDeg, 0, 4).state, SurveillanceState::Return) << "not back inside 1 deg yet";
  EXPECT_EQ(p.update(0.2 * kDeg, 0, 5).state, SurveillanceState::Watch);
}

TEST(SurveillancePlanner, ReenteringReaimsAndExitStopsAiming) {
  SurveillancePlanner p;
  p.enter(10 * kDeg, 0.0);
  p.update(10 * kDeg, 0.0, 1);
  p.enter(-30 * kDeg, 5 * kDeg);
  const auto out = p.update(10 * kDeg, 0.0, 2);
  EXPECT_EQ(out.state, SurveillanceState::Return);
  EXPECT_DOUBLE_EQ(out.intent.q_yaw_rad, -30 * kDeg);
  p.exit();
  EXPECT_FALSE(p.active());
  EXPECT_EQ(p.update(10 * kDeg, 0.0, 3).intent.type, IntentType::Hold);
}

TEST(SurveillancePlanner, ANonFinitePointIsRefusedRatherThanFollowed) {
  SurveillancePlanner p;
  p.enter(std::nan(""), 0.0);
  const auto out = p.update(0.0, 0.0, 1);
  EXPECT_EQ(out.intent.type, IntentType::Hold);
  EXPECT_FALSE(p.active());
}

}  // namespace
}  // namespace ota
