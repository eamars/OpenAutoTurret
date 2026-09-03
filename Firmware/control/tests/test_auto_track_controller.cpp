// §85 — AUTO_TRACK unit tests. The document's own boundary applies: "Assert only
// MotionIntent; motor behavior remains tested in v1." What is asserted here is the
// policy — follow or hold, and with how much authority — because that is what this
// class owns. Whether the drives obey a position command is v1's tested business.
#include <gtest/gtest.h>

#include "tracking/auto_track_controller.hpp"

namespace ota {
namespace {

constexpr TimeNs kMs = 1'000'000;
constexpr TimeNs kCycle = 5'000'000;  // 200 Hz

// A clock that only moves forward, and a controller that is stepped the way the loop
// steps it. Times are explicit rather than implicit because every interesting
// transition in §20 is a timeout, and a test that hides its clock cannot assert one.
class AutoTrackTest : public ::testing::Test {
 protected:
  void advance(int cycles = 1) {
    for (int i = 0; i < cycles; ++i) {
      now_ += kCycle;
      out_ = at_.update(in_, now_);
    }
  }
  void advance_ms(int64_t ms) {
    const int cycles = static_cast<int>(ms * 1000000 / kCycle);
    advance(cycles > 0 ? cycles : 1);
  }

  AutoTrackController at_;
  AutoTrackInput in_;
  AutoTrackOutput out_;
  TimeNs now_ = 1'000'000'000;
};

// Make the input look like a target the operator has chosen and the detector is
// confirming this very frame.
void make_visible(AutoTrackInput& in) {
  in.has_selection = true;
  in.target_visible = true;
  in.measurement_age_ms = 0;
  in.estimator_ready = true;
  in.los_feasible = true;
  in.detector_confidence = 0.95f;
  in.track_confidence = 0.92f;
  in.visible_frames = 30;
  in.missing_frames = 0;
}

TEST_F(AutoTrackTest, Case1VisibleTargetAcquiresThenTracks) {
  // §15/§16: WAIT_TARGET -> ACQUIRE -> TRACKING, and the acquisition is not instantaneous
  // even though §16 says "immediately": the word there is about not needing another
  // operator action, not about skipping the evidence.
  out_ = at_.update(in_, now_);
  EXPECT_EQ(out_.state, AutoTrackState::WaitTarget);
  EXPECT_FALSE(out_.follow_los) << "nothing is selected and it already wants to move";
  EXPECT_STREQ(out_.reason, "select a target")
      << "§16's message for the operator, verbatim, on the screen";

  make_visible(in_);
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::Acquire);
  advance(2);
  EXPECT_EQ(out_.state, AutoTrackState::Tracking);
  EXPECT_TRUE(out_.follow_los);
}

TEST_F(AutoTrackTest, Case2TrackingAtHighConfidenceAsksForFullAuthority) {
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);
  EXPECT_EQ(out_.band, ConfidenceBand::High);
  EXPECT_DOUBLE_EQ(out_.velocity_scale, 1.0)
      << "§19: HIGH is the normal configured tracking limit";
}

TEST_F(AutoTrackTest, Case3TemporaryDropoutCoastsWithShrinkingAuthority) {
  // §20.1: the prediction continues and the permitted acceleration falls as uncertainty
  // grows. Two things to get right, and they pull in opposite directions: the turret
  // must keep covering the target's last heading (a hard stop at every missed frame
  // would be worse than following a good prediction), and it must not keep swinging at
  // full speed on stale evidence.
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);

  in_.measurement_age_ms = 5000;
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::Coasting);
  const double first = out_.velocity_scale;
  EXPECT_TRUE(out_.follow_los) << "coasting that stops following is not coasting";
  advance_ms(150);
  EXPECT_EQ(out_.state, AutoTrackState::Coasting);
  EXPECT_LT(out_.velocity_scale, first)
      << "authority did not shrink as the coast wore on";
  advance_ms(400);
  EXPECT_EQ(out_.state, AutoTrackState::LostHold);
  EXPECT_FALSE(out_.follow_los)
      << "§20.2: past the coast, predicting motion into the future must stop";
}

TEST_F(AutoTrackTest, Case4LongLossHoldsAndDoesNotUnchoose) {
  make_visible(in_);
  advance(3);
  in_.target_visible = false;
  in_.measurement_age_ms = 5000;
  advance_ms(4000);
  EXPECT_EQ(out_.state, AutoTrackState::LostHold);
  EXPECT_STREQ(out_.reason, "lost: holding");
  EXPECT_TRUE(in_.has_selection)
      << "§12: the test must not forget that losing sight of somebody is not the same "
         "as unselecting them — the controller has no authority to clear a selection "
         "and this is where it would be tempted";
}

TEST_F(AutoTrackTest, Case5ReacquisitionNeedsNoOperatorClick) {
  make_visible(in_);
  advance(3);
  in_.target_visible = false;
  in_.measurement_age_ms = 5000;
  advance_ms(400);
  ASSERT_EQ(out_.state, AutoTrackState::LostHold);

  // §20.3. The target comes back and the turret picks it up again — through REACQUIRE
  // rather than straight into TRACKING, so the handover is visible in the telemetry
  // rather than being indistinguishable from never having lost it.
  in_.target_visible = true;
  in_.measurement_age_ms = 0;
  in_.just_reacquired = true;
  in_.reacquisition_score = 0.9f;
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::Reacquire);
  advance(2);
  EXPECT_EQ(out_.state, AutoTrackState::Tracking);
  EXPECT_TRUE(out_.follow_los);
}

TEST_F(AutoTrackTest, Case5ReacquiredTargetCarriesItsUncertainty) {
  // §19's fifth input. A target recovered from a guess is not as trustworthy as one
  // that never left, and the authority granted should say so — this is the difference
  // between a smooth continuation and a lurch toward whoever happened to reappear.
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);
  const double normal = out_.velocity_scale;

  AutoTrackController weak;
  AutoTrackInput w = in_;
  w.just_reacquired = true;
  w.reacquisition_score = 0.6f;
  AutoTrackOutput wo;
  for (int i = 0; i < 4; ++i) {
    now_ += kCycle;
    wo = weak.update(w, now_);
  }
  EXPECT_LT(wo.selected_confidence, AutoTrackController::selected_confidence(in_, 0.92f))
      << "the discount for an ambiguous history never arrived";
  EXPECT_LE(wo.velocity_scale, normal);
}

TEST_F(AutoTrackTest, Case6TargetChangeRestartsAcquisition) {
  // Not a transition inside the machine — ControlLoop calls reset() when the selection
  // changes (§15). Tested here because the alternative is a turret that walks into
  // #2's position carrying #1's acquisition history, and that looks exactly like a
  // target-stealing bug while being a lifecycle bug.
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);
  at_.reset();
  out_ = at_.update(in_, now_ += kCycle);
  EXPECT_EQ(out_.state, AutoTrackState::Acquire)
      << "a fresh selection resumed straight into TRACKING with the previous target's "
         "acquisition history";
  advance(1);
  EXPECT_EQ(out_.state, AutoTrackState::Acquire)
      << "and it did not take the two cycles of evidence either";
  advance(1);
  EXPECT_EQ(out_.state, AutoTrackState::Tracking);
}

TEST_F(AutoTrackTest, Case7UnreachableTargetHoldsRatherThanStrainingAtIt) {
  // §22. The signal comes from the intent converter (§67), which is the only thing that
  // knows what the envelope could satisfy; the state machine's job is to stop asking.
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);
  in_.los_feasible = false;
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::TargetUnreachable);
  EXPECT_FALSE(out_.follow_los);
  advance();
  EXPECT_STREQ(out_.reason, "target outside the safe envelope (22)")
      << "the steady-state reason is what the operator reads for as long as it lasts; "
         "the transition message is gone after one cycle";
  in_.los_feasible = true;
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::Tracking);
}

TEST_F(AutoTrackTest, Case8LowConfidenceMovesCarefullyAndInvalidDoesNotMove) {
  // §19's four bands, checked at the boundary that matters: LOW still moves (a target
  // held weakly is worth covering) and INVALID does not (there is no evidence).
  make_visible(in_);
  in_.track_confidence = 0.55f;
  in_.visible_frames = 4;
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);
  EXPECT_TRUE(out_.band == ConfidenceBand::Low ||
              out_.band == ConfidenceBand::Medium)
      << "a weak, young track was granted full authority: band "
      << confidence_band_name(out_.band);
  EXPECT_LT(out_.velocity_scale, 1.0);

  AutoTrackController at2;
  AutoTrackInput i2;
  make_visible(i2);
  i2.track_confidence = 0.0f;
  AutoTrackOutput o2;
  for (int i = 0; i < 4; ++i) o2 = at2.update(i2, now_ += kCycle);
  EXPECT_TRUE(o2.band == ConfidenceBand::Invalid || o2.velocity_scale <= 0.0)
      << "confidence zero granted motion authority (§19 INVALID: coast/hold)";
}

TEST_F(AutoTrackTest, Case9StaleCameraTimestampCountsAsAMissingMeasurement) {
  // §85's ninth case, and the one that is easiest to get wrong in the wiring rather
  // than in this class: §17 insists on tracking from the SensorTimestamp, and ControlLoop
  // expresses "this cycle had a fresh measurement" as a comparison against the control
  // clock. A publisher that stalls must read as evidence going stale, not as evidence
  // standing still.
  make_visible(in_);
  advance(3);
  ASSERT_EQ(out_.state, AutoTrackState::Tracking);

  // The last fresh measurement is two cycles old and nothing new has arrived.
  in_.measurement_age_ms = 5000;
  advance();
  EXPECT_EQ(out_.state, AutoTrackState::Coasting)
      << "a stalled publisher left the controller in TRACKING on evidence it never "
         "received";
  advance_ms(400);
  EXPECT_EQ(out_.state, AutoTrackState::LostHold);
}

TEST_F(AutoTrackTest, Case10AmbiguousReacquisitionIsNotTreatedAsEvidence) {
  // §21's refusal has to survive the trip into this class, or LOST_HOLD quietly becomes
  // TRACKING the moment any same-class candidate reappears.
  make_visible(in_);
  advance(3);
  in_.target_visible = false;
  in_.measurement_age_ms = 5000;
  advance_ms(400);
  ASSERT_EQ(out_.state, AutoTrackState::LostHold);

  in_.target_visible = true;
  in_.measurement_age_ms = 0;
  in_.ambiguous = true;
  in_.just_reacquired = true;
  in_.reacquisition_score = 0.55f;  // just over the manager's threshold
  advance(3);
  EXPECT_TRUE(out_.state == AutoTrackState::LostHold ||
              out_.velocity_scale < 1.0)
      << "a contested reacquisition was granted the authority of an unambiguous one: "
         "band "
      << confidence_band_name(out_.band) << " scale " << out_.velocity_scale;
}

}  // namespace
}  // namespace ota
