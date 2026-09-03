// §86 — AUTO_ROAM unit tests. The property under test throughout is predictability: a
// person watching a turret move by itself is safe only while they can say where it goes
// next. Every case below is that same question asked from a different angle — where it
// starts, which way it turns, and what it refuses to do.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mode/roam_planner.hpp"

namespace ota {
namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr TimeNs kCycle = 5'000'000;  // 200 Hz

RoamConfig test_config() {
  RoamConfig c;
  c.envelope.yaw_min_rad = -60.0 * kDeg;
  c.envelope.yaw_max_rad = +60.0 * kDeg;
  c.envelope.pitch_min_rad = -20.0 * kDeg;
  c.envelope.pitch_max_rad = +10.0 * kDeg;
  c.pitch_ref_rad = -5.0 * kDeg;
  c.braking_margin_rad = 5.0 * kDeg;
  return c;
}

class RoamTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = test_config();
    r_.set_config(cfg_);
  }

  // Advance the planner, optionally moving the simulated yaw toward the waypoint the
  // way a servo would: the point is that reversals are decided by *arrival*, so the rig
  // has to arrive.
  void run(int cycles, double q_yaw) {
    for (int i = 0; i < cycles; ++i) {
      now_ += kCycle;
      out_ = r_.update(q_yaw, -5.0 * kDeg, now_, kCycle);
    }
  }

  // Drive the simulated yaw along the planner's own waypoint, which is the only way to
  // exercise a full reversal without simulating the whole trajectory generator.
  void follow_waypoint(int cycles, double speed_rad_per_cycle) {
    for (int i = 0; i < cycles; ++i) {
      now_ += kCycle;
      out_ = r_.update(q_yaw_, pitch_, now_, kCycle);
      const double step = speed_rad_per_cycle * out_.direction;
      if (std::fabs(out_.target_yaw_rad - q_yaw_) > std::fabs(step))
        q_yaw_ += step;
      else
        q_yaw_ = out_.target_yaw_rad;
    }
  }

  RoamConfig cfg_;
  RoamPlanner r_{cfg_};
  RoamOutput out_;
  TimeNs now_ = 1'000'000'000;
  double q_yaw_ = 0.0;
  double pitch_ = -5.0 * kDeg;
};

TEST_F(RoamTest, EnteringFromCenterSweepsTowardTheNearerEndAndStaysInside) {
  // §36.3 plus §30. Starting dead centre makes "nearer" a coin toss, so what is really
  // being asserted is that the choice is made at all, and that the first waypoint is a
  // sweep bound rather than the raw envelope edge.
  r_.enter(0.0, pitch_);
  run(1, 0.0);
  EXPECT_EQ(out_.state, RoamState::Sweep);
  EXPECT_NE(out_.direction, 0);
  EXPECT_EQ(out_.intent.type, IntentType::JointPosition);
  const double expected = out_.direction < 0 ? r_.sweep_lo_rad() : r_.sweep_hi_rad();
  EXPECT_NEAR(out_.target_yaw_rad, expected, 1e-12);
  // §32: braking reserve, on the sweep, not just on the envelope.
  EXPECT_GT(out_.target_yaw_rad, cfg_.envelope.yaw_min_rad);
  EXPECT_LT(out_.target_yaw_rad, cfg_.envelope.yaw_max_rad);
  EXPECT_NEAR(cfg_.envelope.yaw_max_rad - r_.sweep_hi_rad(), cfg_.braking_margin_rad,
              1e-12);
}

TEST_F(RoamTest, EnteringNearTheLeftBoundaryDoesNotCrossTheRoomFirst) {
  // §36.4, and the case an operator actually creates: jog to the left edge, click
  // AUTO_ROAM, and watch what it does. Driving to the far side first is a several-second
  // crossing nobody asked for, and it is the thing that makes a mode switch feel like a
  // lunge.
  // -45 deg, not -58: the sweep bounds are inset by the braking margin, so -58 is
  // already outside the region and belongs to the approach case below. Inside, near the
  // left edge, is the situation an operator creates by jogging left and then clicking
  // AUTO_ROAM.
  r_.enter(-45.0 * kDeg, pitch_);
  run(1, -45.0 * kDeg);
  EXPECT_LT(out_.direction, 0)
      << "it turned around and crossed the whole envelope to start at the other end";
  EXPECT_NEAR(out_.target_yaw_rad, r_.sweep_lo_rad(), 1e-12);
  EXPECT_LT(std::fabs(out_.target_yaw_rad - (-45.0 * kDeg)), 20.0 * kDeg)
      << "the first move after a mode click should be the short one";
}

TEST_F(RoamTest, EnteringNearTheRightBoundarySweepsLeft) {
  r_.enter(45.0 * kDeg, pitch_);
  run(1, 45.0 * kDeg);
  EXPECT_GT(out_.direction, 0);
  EXPECT_NEAR(out_.target_yaw_rad, r_.sweep_hi_rad(), 1e-12);
}

TEST_F(RoamTest, EnteringOutsideTheEnvelopeMakesOneBoundedApproach) {
  // Homing leaves the turret at the ready pose, which may sit outside a narrow roam
  // region. §36 allows exactly one move to the nearest edge — after which the sweep
  // begins — and §34 forbids the alternative interpretation, "jump to the scan start".
  r_.enter(-80.0 * kDeg, pitch_);
  run(1, -80.0 * kDeg);
  EXPECT_EQ(out_.state, RoamState::MoveToScanStart);
  EXPECT_GT(out_.direction, 0);
  EXPECT_NEAR(out_.target_yaw_rad, r_.sweep_lo_rad(), 1e-12);
  // Arriving there hands over to the sweep rather than looping back out.
  run(2, r_.sweep_lo_rad());
  EXPECT_NE(out_.state, RoamState::MoveToScanStart)
      << "it held at the edge forever instead of starting the pattern";
}

TEST_F(RoamTest, ReversalsAlternateAndNeverLeaveTheEnvelope) {
  // §30's pattern, run as an actual sweep. The yaw follows the planner's own waypoint;
  // what is checked is that the sequence of waypoints alternates, that a turnaround is
  // ever actually reached (a sweep that never turns around is a turret driving into a
  // limit), and that no waypoint is ever outside the inset bounds.
  r_.enter(0.0, pitch_);
  double lo_seen = 1e9, hi_seen = -1e9;
  int reversals = 0;
  int last_dir = out_.direction;
  for (int i = 0; i < 40000; ++i) {
    now_ += kCycle;
    out_ = r_.update(q_yaw_, pitch_, now_, kCycle);
    if (out_.direction != 0 && out_.direction != last_dir) ++reversals;
    if (out_.direction != 0) last_dir = out_.direction;
    lo_seen = std::min(lo_seen, out_.target_yaw_rad);
    hi_seen = std::max(hi_seen, out_.target_yaw_rad);
    EXPECT_GE(out_.target_yaw_rad, cfg_.envelope.yaw_min_rad - 1e-12)
        << "a waypoint outside the roam envelope at cycle " << i;
    EXPECT_LE(out_.target_yaw_rad, cfg_.envelope.yaw_max_rad + 1e-12);
    const double step = 0.2 * kDeg * out_.direction;
    if (std::fabs(out_.target_yaw_rad - q_yaw_) > std::fabs(step))
      q_yaw_ += step;
    else
      q_yaw_ = out_.target_yaw_rad;
  }
  EXPECT_GE(reversals, 3)
      << "over two thousand cycles the turret turned around fewer than three times";
  EXPECT_NEAR(lo_seen, r_.sweep_lo_rad(), 1e-9);
  EXPECT_NEAR(hi_seen, r_.sweep_hi_rad(), 1e-9);
  EXPECT_GT(hi_seen - lo_seen, 100.0 * kDeg) << "a sweep 5 degrees wide is a jitter";
}

TEST_F(RoamTest, TurnaroundIsADeliberateStateNotABounce) {
  // §29 "smoothly reverse direction". On arrival the planner enters TURNAROUND and holds
  // the waypoint; it flips the target only once the yaw has actually settled. A reversal
  // issued on the arrival cycle itself turns a sweep into an oscillation at the end of
  // the envelope, where the braking reserve is smallest.
  r_.enter(0.0, pitch_);
  const double lo = r_.sweep_lo_rad();
  q_yaw_ = lo + 2.0 * cfg_.reach_tol_rad;
  run(1, q_yaw_);
  const int dir_in = out_.direction;
  q_yaw_ = lo;
  run(1, lo);
  EXPECT_EQ(out_.state, RoamState::Turnaround);
  EXPECT_EQ(out_.direction, dir_in) << "it flipped the waypoint on the arrival cycle";
  run(4, lo);
  EXPECT_NE(out_.state, RoamState::Turnaround)
      << "it latched in TURNAROUND and will never sweep again";
  EXPECT_EQ(out_.direction, -dir_in);
}

TEST_F(RoamTest, EnvelopeOutsideTheSafeEnvelopeIsRefused) {
  // §32, and §86's "collision envelope" case. The numbers below are the plausible
  // mistake, not a caricature: someone widens the roam region in a config file to cover
  // more of the room, and the only thing standing between that edit and the mechanical
  // stops is this check.
  RoamEnvelope safe;
  safe.yaw_min_rad = -90.0 * kDeg;
  safe.yaw_max_rad = 90.0 * kDeg;
  safe.pitch_min_rad = -30.0 * kDeg;
  safe.pitch_max_rad = 20.0 * kDeg;
  char why[192] = {};

  RoamEnvelope good = cfg_.envelope;
  EXPECT_TRUE(RoamPlanner::validate_envelope(good, safe, cfg_.pitch_ref_rad,
                                             cfg_.min_inside_safe_rad, why, sizeof why))
      << why;

  RoamEnvelope wide = good;
  wide.yaw_max_rad = 89.0 * kDeg;  // inside the soft limit, but not by the margin
  EXPECT_FALSE(RoamPlanner::validate_envelope(wide, safe, cfg_.pitch_ref_rad,
                                              cfg_.min_inside_safe_rad, why, sizeof why));
  EXPECT_NE(std::string(why).find("yaw"), std::string::npos) << why;

  RoamEnvelope flipped = good;
  flipped.yaw_min_rad = 60.0 * kDeg;
  flipped.yaw_max_rad = -60.0 * kDeg;
  EXPECT_FALSE(RoamPlanner::validate_envelope(flipped, safe, cfg_.pitch_ref_rad,
                                              cfg_.min_inside_safe_rad, why, sizeof why));
  EXPECT_NE(std::string(why).find("empty"), std::string::npos) << why;
}

TEST_F(RoamTest, PitchRestrictionIsEnforcedOnTheReferenceNotOnlyTheBounds) {
  // §86's "pitch restriction". Comparing envelopes against each other would pass a plan
  // whose held elevation sits outside them, which is why the reference is checked too.
  RoamEnvelope safe;
  safe.yaw_min_rad = -90.0 * kDeg;
  safe.yaw_max_rad = 90.0 * kDeg;
  safe.pitch_min_rad = -30.0 * kDeg;
  safe.pitch_max_rad = 20.0 * kDeg;
  char why[192] = {};
  RoamEnvelope tight = cfg_.envelope;
  tight.pitch_min_rad = -10.0 * kDeg;  // the sweep's -5 deg reference is inside this
  EXPECT_TRUE(RoamPlanner::validate_envelope(tight, safe, -5.0 * kDeg,
                                             cfg_.min_inside_safe_rad, why, sizeof why))
      << why;
  EXPECT_FALSE(RoamPlanner::validate_envelope(tight, safe, +9.5 * kDeg,
                                              cfg_.min_inside_safe_rad, why, sizeof why))
      << "+9.5 deg is inside a -10..+10 region but not inside it by the 2 deg margin: "
      << why;

  // And a sweep already running clamps its elevation into the envelope rather than
  // continuing to hold a value that has just become illegal.
  cfg_.envelope.pitch_min_rad = -4.0 * kDeg;  // now -5 deg is outside
  r_.set_config(cfg_);
  r_.enter(0.0, pitch_);
  run(1, 0.0);
  EXPECT_GE(out_.intent.q_pitch_rad, cfg_.envelope.pitch_min_rad - 1e-12)
      << "the sweep kept holding an elevation the envelope no longer allows";
}

TEST_F(RoamTest, LevelScanIsAskedForOnlyWhenTheImuIsValid) {
  // §33: with gravity available the preferred scan is a world-level yaw and the pitch
  // axis compensates installation tilt. The elevation requested is published so the
  // screen can show what was asked for, not only what was commanded.
  cfg_.level_scan_available = true;
  cfg_.world_elevation_rad = -3.0 * kDeg;
  r_.set_config(cfg_);
  r_.enter(0.0, pitch_);
  run(1, 0.0);
  EXPECT_EQ(out_.intent.type, IntentType::WorldLevelYaw);
  EXPECT_TRUE(out_.intent.has_world_elevation);
  EXPECT_NEAR(out_.intent.world_elevation_rad, -3.0 * kDeg, 1e-12);
  EXPECT_STREQ(out_.reason, "SWEEP");
}

TEST_F(RoamTest, WithoutTheImuItFallsBackToJointSpaceAndSaysSo) {
  // §33's fallback, which is this station's everyday reality. "IMU unavailable" must not
  // degrade into "AUTO_ROAM unavailable": the document is explicit that the IMU is not
  // required, and a station that cannot roam without a second sensor has one more way to
  // end up doing nothing.
  cfg_.level_scan_available = false;
  r_.set_config(cfg_);
  r_.enter(0.0, pitch_);
  run(1, 0.0);
  EXPECT_EQ(out_.intent.type, IntentType::JointPosition);
  EXPECT_FALSE(out_.intent.has_world_elevation);
  EXPECT_STREQ(out_.intent.reason, "bounded sweep");
  EXPECT_NEAR(out_.intent.q_pitch_rad, cfg_.pitch_ref_rad, 1e-12)
      << "without the IMU the sweep holds the configured joint-space pitch";
}

TEST_F(RoamTest, EmptyEnvelopeStopsTheSweepMidFlight) {
  // The envelope is editable while a sweep is running. A planner that validated only at
  // entry would carry on toward a waypoint that has just become illegal.
  r_.enter(0.0, pitch_);
  run(2, 0.0);
  ASSERT_TRUE(r_.active());
  cfg_.envelope.yaw_min_rad = 10.0 * kDeg;
  cfg_.envelope.yaw_max_rad = 5.0 * kDeg;  // inverted, deliberately
  r_.set_config(cfg_);
  run(1, 0.0);
  EXPECT_FALSE(r_.active());
  EXPECT_EQ(out_.intent.type, IntentType::Hold);
  EXPECT_STREQ(out_.reason, "roam envelope empty");
}

TEST_F(RoamTest, IdlePlannerInventsNothing) {
  // §35's Init, and the general rule that a controller which is not the authority emits
  // a hold. An idle planner picking a waypoint is how a turret starts moving on its own
  // while the operator believes it is in MANUAL.
  run(3, 0.0);
  EXPECT_EQ(out_.state, RoamState::Idle);
  EXPECT_EQ(out_.intent.type, IntentType::Hold);
  EXPECT_EQ(out_.direction, 0);
  EXPECT_EQ(out_.intent.source, MotionSource::AutoRoam)
      << "even a hold is attributed, so the screen can say which mode is holding";
}

}  // namespace
}  // namespace ota
