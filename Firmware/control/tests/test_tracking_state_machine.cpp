// Unit tests for the tracking state machine (architecture §34) and the
// confidence decay (architecture §35). Pure logic, driven by measurement
// freshness.
#include <gtest/gtest.h>

#include "tracking/tracking_state_machine.hpp"

namespace {
using ota::tracking::TrackState;
using ota::tracking::TrackingStateMachine;
using ota::tracking::TrackingStateMachineConfig;

constexpr int64_t kMs = 1000 * 1000;

TrackingStateMachineConfig test_cfg() {
  TrackingStateMachineConfig c;
  c.coast_max_ns = 200 * kMs;   // 200 ms
  c.lost_ns = 1000 * kMs;       // 1 s
  c.search_enabled = false;
  return c;
}

TEST(TrackingStateMachine, AcquireFromReadyHold) {
  TrackingStateMachine fsm(test_cfg());
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
  fsm.update(0, /*has_valid=*/true);
  EXPECT_EQ(fsm.state(), TrackState::Tracking);
}

TEST(TrackingStateMachine, ColdStartWithNoTargetEverStillSearches) {
  // The station boots with no target in view and there is no guarantee one ever
  // appears. Gating SEARCH on TARGET_LOST made it reachable only after an
  // acquisition — i.e. only in the case it exists to prevent — so a cold turret
  // sat in READY_HOLD forever while the sweep that would have found a target
  // never ran. After the same grace a lost target gets, it must go looking.
  TrackingStateMachineConfig c = test_cfg();
  c.search_enabled = true;
  TrackingStateMachine fsm(c);

  fsm.update(0, /*has_valid=*/false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold)
      << "the grace window exists so a detector that is merely slow to publish "
         "its first frame is not answered with a sweep";
  fsm.update(999 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
  fsm.update(1000 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::Search);
  fsm.update(60 * kMs * 1000, false);
  EXPECT_EQ(fsm.state(), TrackState::Search) << "it must keep searching";
}

TEST(TrackingStateMachine, ColdStartHoldsWhenSearchIsDisabled) {
  // §36: search is opt-in. Disabling it must keep the old, boring behaviour —
  // otherwise "search off" would secretly mean "sweep the workspace".
  TrackingStateMachineConfig c = test_cfg();
  c.search_enabled = false;
  TrackingStateMachine fsm(c);
  fsm.update(0, false);
  fsm.update(10 * 1000 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
}

TEST(TrackingStateMachine, ColdStartGraceIsReArmedByReset) {
  // reset() is what happens when tracking is stopped and started again. The new
  // enable must get its own window; inheriting an expired one would make the
  // turret sweep the instant tracking is re-enabled, before the detector has
  // said anything at all.
  TrackingStateMachineConfig c = test_cfg();
  c.search_enabled = true;
  TrackingStateMachine fsm(c);
  fsm.update(0, false);
  fsm.update(2000 * kMs, false);
  ASSERT_EQ(fsm.state(), TrackState::Search);
  fsm.reset();
  fsm.update(3000 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
  fsm.update(4000 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::Search);
}

TEST(TrackingStateMachine, ARealTargetEndsTheColdStartSweep) {
  // The sweep is a means: the moment a target exists, TRACKING wins. Otherwise
  // this fix would have traded "never searches" for "never stops searching".
  TrackingStateMachineConfig c = test_cfg();
  c.search_enabled = true;
  TrackingStateMachine fsm(c);
  fsm.update(0, false);
  fsm.update(2000 * kMs, false);
  ASSERT_EQ(fsm.state(), TrackState::Search);
  fsm.update(2033 * kMs, /*has_valid=*/true);
  EXPECT_EQ(fsm.state(), TrackState::Tracking);
}

TEST(TrackingStateMachine, CoastingOnShortDropout) {
  TrackingStateMachine fsm(test_cfg());
  fsm.update(0, true);
  EXPECT_EQ(fsm.state(), TrackState::Tracking);
  // 100 ms with no measurement (within the 200 ms coast window).
  fsm.update(100 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::Coasting);
}

TEST(TrackingStateMachine, BrakeToHoldOnLongerDropout) {
  TrackingStateMachine fsm(test_cfg());
  fsm.update(0, true);
  // 400 ms with no measurement (beyond the 200 ms coast, within the 1 s lost).
  fsm.update(400 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::BrakeToHold);
}

TEST(TrackingStateMachine, TargetLostThenReadyHoldWhenSearchDisabled) {
  TrackingStateMachine fsm(test_cfg());  // search disabled
  fsm.update(0, true);
  fsm.update(1200 * kMs, false);  // beyond the 1 s lost threshold
  EXPECT_EQ(fsm.state(), TrackState::TargetLost);
  // Next cycle with no target: hands off to ready-hold.
  fsm.update(1300 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
}

TEST(TrackingStateMachine, TargetLostThenSearchWhenEnabled) {
  TrackingStateMachineConfig c = test_cfg();
  c.search_enabled = true;
  TrackingStateMachine fsm(c);
  fsm.update(0, true);
  fsm.update(1200 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::TargetLost);
  fsm.update(1300 * kMs, false);
  EXPECT_EQ(fsm.state(), TrackState::Search);
}

TEST(TrackingStateMachine, ReacquireAfterCoasting) {
  TrackingStateMachine fsm(test_cfg());
  fsm.update(0, true);
  fsm.update(100 * kMs, false);  // coasting
  EXPECT_EQ(fsm.state(), TrackState::Coasting);
  fsm.update(150 * kMs, true);   // reacquired
  EXPECT_EQ(fsm.state(), TrackState::Tracking);
}

TEST(TrackingStateMachine, ConfidenceDecaysAsTargetGoesStale) {
  TrackingStateMachine fsm(test_cfg());
  fsm.update(0, true);
  EXPECT_NEAR(fsm.confidence(), 1.0, 1e-6);
  // 500 ms stale (halfway to the 1 s lost threshold) -> ~0.5 confidence.
  fsm.update(500 * kMs, false);
  EXPECT_LT(fsm.confidence(), 0.6);
  EXPECT_GT(fsm.confidence(), 0.4);
  // Lost -> 0 confidence.
  fsm.update(1200 * kMs, false);
  EXPECT_NEAR(fsm.confidence(), 0.0, 1e-6);
}

TEST(TrackingStateMachine, NoMeasurementEverMeansNoTarget) {
  TrackingStateMachine fsm(test_cfg());
  // Never saw a target: stays in ready-hold, no confidence.
  fsm.update(0, false);
  EXPECT_EQ(fsm.state(), TrackState::ReadyHold);
  EXPECT_NEAR(fsm.confidence(), 0.0, 1e-6);
}

}  // namespace
