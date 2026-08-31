// Unit tests for the reference manager (architecture §16). Verifies the
// priority arbitration (TRACKING > SEARCH > HOLD), the confidence-scaled speed
// (§35), and the safe fall-through when the target LOS is unreachable.
#include <gtest/gtest.h>

#include "control/reference_manager.hpp"

namespace {
using ota::geo::LosJointSolver;
using ota::geo::TurretKinematics;
using ota::ReferenceManager;
using ota::ReferenceManagerInput;
using ota::ReferenceSource;
using ota::tracking::TrackState;

constexpr double kDeg = 3.14159265358979323846 / 180.0;

ReferenceManager make_rm() {
  return ReferenceManager(LosJointSolver(TurretKinematics::aligned()));
}

TEST(ReferenceManager, TrackingHasPriorityWhenTargetPresent) {
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::Tracking;
  in.target_confidence = 1.0;
  in.predicted_az_rad = 0.3;
  in.predicted_el_rad = 0.1;
  in.q_yaw_hold_rad = 0.0;
  in.q_pitch_hold_rad = 0.0;
  in.track_v_max_rad_s = 30.0 * kDeg;
  auto req = rm.compute(in);
  EXPECT_EQ(req.source, ReferenceSource::Tracking);
  EXPECT_TRUE(req.is_tracking_reference);
  // The reference points at the target (q_yaw = az, q_pitch = -el).
  EXPECT_NEAR(req.q_yaw_rad, 0.3, 1e-6);
  EXPECT_NEAR(req.q_pitch_rad, -0.1, 1e-6);
  EXPECT_NEAR(req.v_max_rad_s, 30.0 * kDeg, 1e-6);
}

TEST(ReferenceManager, CoastingStillTracks) {
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::Coasting;
  in.target_confidence = 0.5;
  in.predicted_az_rad = -0.2;
  in.predicted_el_rad = 0.0;
  auto req = rm.compute(in);
  EXPECT_EQ(req.source, ReferenceSource::Tracking);
  EXPECT_NEAR(req.q_yaw_rad, -0.2, 1e-6);
}

TEST(ReferenceManager, ConfidenceScalesTrackingSpeed) {
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::Tracking;
  in.predicted_az_rad = 0.0;
  in.predicted_el_rad = 0.0;
  in.track_v_max_rad_s = 30.0 * kDeg;
  in.target_confidence = 1.0;
  EXPECT_NEAR(rm.compute(in).v_max_rad_s, 30.0 * kDeg, 1e-6);
  in.target_confidence = 0.5;
  EXPECT_NEAR(rm.compute(in).v_max_rad_s, 15.0 * kDeg, 1e-6);
  in.target_confidence = 0.0;
  EXPECT_NEAR(rm.compute(in).v_max_rad_s, 0.0, 1e-9);
}

TEST(ReferenceManager, SearchWhenLostAndSearchEnabled) {
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::Search;
  in.in_search = true;
  in.search_q_yaw_rad = 0.5;
  in.search_q_pitch_rad = -0.1;
  in.search_v_max_rad_s = 10.0 * kDeg;
  auto req = rm.compute(in);
  EXPECT_EQ(req.source, ReferenceSource::Search);
  EXPECT_FALSE(req.is_tracking_reference);
  EXPECT_NEAR(req.q_yaw_rad, 0.5, 1e-9);
  EXPECT_NEAR(req.v_max_rad_s, 10.0 * kDeg, 1e-9);
}

TEST(ReferenceManager, HoldWhenNoTarget) {
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::ReadyHold;
  in.q_yaw_hold_rad = 0.1;
  in.q_pitch_hold_rad = 0.2;
  in.hold_v_max_rad_s = 10.0 * kDeg;
  auto req = rm.compute(in);
  EXPECT_EQ(req.source, ReferenceSource::Hold);
  EXPECT_FALSE(req.is_tracking_reference);
  EXPECT_NEAR(req.q_yaw_rad, 0.1, 1e-9);
  EXPECT_NEAR(req.q_pitch_rad, 0.2, 1e-9);
}

TEST(ReferenceManager, BrakeToHoldFallsToHoldNotTracking) {
  // A braking-to-hold target must NOT keep tracking (the target is stale).
  auto rm = make_rm();
  ReferenceManagerInput in;
  in.track_state = TrackState::BrakeToHold;
  in.target_confidence = 0.3;
  in.predicted_az_rad = 0.4;
  in.predicted_el_rad = 0.0;
  in.q_yaw_hold_rad = 0.0;
  in.q_pitch_hold_rad = 0.0;
  auto req = rm.compute(in);
  EXPECT_EQ(req.source, ReferenceSource::Hold);
  EXPECT_FALSE(req.is_tracking_reference);
}

}  // namespace
