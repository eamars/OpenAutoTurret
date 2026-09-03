// Unit tests for the reference manager (architecture §16). Verifies the
// priority arbitration (TRACKING > SEARCH > HOLD), the confidence-scaled speed
// (§35), and the safe fall-through when the target LOS is unreachable.
#include <gtest/gtest.h>

#include <cmath>

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

// ---------------------------------------------------------------------------
// The joint-space branch, not the direction, is what the limits apply to.
//
// Observed on the station on 2026-09-04: AUTO_TRACK sat with a target 182 px off
// the reticle, reporting `tracking`, while q_ref_yaw was pinned to EXACTLY
// q_soft_min_yaw_rad (-0.3940). Nothing was unreachable and nothing had expired;
// LosJointSolver had correctly solved the direction and returned -4.27 rad, an
// angle equivalent to the reachable +2.013 rad. Angles repeat every 2*pi; this
// station's yaw travel [-0.394, 5.588] rad does not, so the equivalent-but-far
// branch fell below the soft minimum and the envelope clamped it to the limit.
// The turret was holding at a limit because of a choice of representation.
// ---------------------------------------------------------------------------
TEST(ReferenceManager, TrackingReferenceTakesTheShortWayRound) {
  auto rm = make_rm();
  const double hold_yaw = 1.7874;  // where the station actually was

  // Sweep the whole azimuth circle at a downward-looking elevation: whatever the
  // solver's internal seed does, a reference must never demand the long way round.
  for (int i = 0; i < 36; ++i) {
    const double az = -M_PI + i * (M_PI / 18.0);
    ReferenceManager::IntentLimits lim;
    lim.q_yaw_hold_rad = hold_yaw;
    lim.q_pitch_hold_rad = -0.675;

    ota::MotionIntent in;
    in.source = ota::MotionSource::AutoTrack;
    in.type = ota::IntentType::LosDirection;
    in.has_los = true;
    in.los_az_rad = az;
    in.los_el_rad = 0.9;
    in.timestamp_ns = 1000000000;
    in.valid_until_ns = 0;  // no expiry, exactly as AUTO_TRACK builds it

    const auto req = rm.resolve(in, lim);
    if (req.target_unreachable) continue;  // genuinely not pointable: fine, and said so
    const double delta = std::fabs(req.q_yaw_rad - hold_yaw);
    EXPECT_LE(delta, M_PI + 1e-9)
        << "azimuth " << az << " resolved to " << req.q_yaw_rad
        << ", demanding a " << delta << " rad slew for a direction that has a nearer branch";
  }
}
