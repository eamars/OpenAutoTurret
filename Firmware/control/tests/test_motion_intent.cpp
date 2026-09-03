// v3 §25/§53 — MotionIntent and its conversion into the v1 reference path.
//
// The point of these tests is the boundary: three modes, one structure, and the
// converter must not invent motion that no mode asked for. Every branch that
// cannot be honoured ends in a hold WITH A REASON, because "the turret held and
// nobody knows why" is the failure mode v1 telemetry could not answer.
#include <gtest/gtest.h>

#include "control/motion_intent.hpp"
#include "control/reference_manager.hpp"

namespace {
using ota::geo::LosJointSolver;
using ota::geo::TurretKinematics;
using ota::IntentType;
using ota::MotionIntent;
using ota::MotionSource;
using ota::ReferenceManager;
using ota::ReferenceSource;

constexpr double kDeg = 3.14159265358979323846 / 180.0;

ReferenceManager make_rm() {
  return ReferenceManager(LosJointSolver(TurretKinematics::aligned()));
}

ReferenceManager::IntentLimits limits() {
  ReferenceManager::IntentLimits lim;
  lim.now_ns = 1'000'000'000;
  lim.q_yaw_hold_rad = 1.0;      // deliberately non-zero: a hold that silently
  lim.q_pitch_hold_rad = -0.5;   // drifted to 0 would pass a sloppier test
  lim.track_v_max_rad_s = 30.0 * kDeg;
  lim.roam_v_max_rad_s = 10.0 * kDeg;
  lim.manual_v_max_rad_s = 25.0 * kDeg;
  lim.hold_v_max_rad_s = 8.0 * kDeg;
  return lim;
}

}  // namespace

TEST(MotionIntent, NoIntentHoldsAtTheSafePose) {
  auto rm = make_rm();
  auto req = rm.resolve(MotionIntent{}, limits());
  EXPECT_EQ(req.source, ReferenceSource::Hold);
  EXPECT_DOUBLE_EQ(req.q_yaw_rad, 1.0);
  EXPECT_DOUBLE_EQ(req.q_pitch_rad, -0.5);
  EXPECT_NEAR(req.v_max_rad_s, 8.0 * kDeg, 1e-12);
  EXPECT_STREQ(req.reason, "no intent");
}

TEST(MotionIntent, ExpiredIntentHoldsWhateverItWasAskingFor) {
  // §93: "no stale mode intent survives a transition". Handled where the
  // timestamp lives, so it cannot be forgotten by a later caller.
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::AutoRoam;
  in.type = IntentType::JointPosition;
  in.has_joint_target = true;
  in.q_yaw_rad = 2.0;
  in.valid_until_ns = 900'000'000;  // one second before "now"
  auto req = rm.resolve(in, limits());
  EXPECT_EQ(req.source, ReferenceSource::Hold);
  EXPECT_STREQ(req.reason, "intent expired -> hold");
}

TEST(MotionIntent, LiveIntentIsHonouredRightUpToTheBoundary) {
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::AutoRoam;
  in.type = IntentType::JointPosition;
  in.has_joint_target = true;
  in.q_yaw_rad = 2.0;
  in.valid_until_ns = 1'000'000'000;  // expires exactly at now
  // "now < valid_until" is the live condition, so exactly-at means expired. The
  // direction of an off-by-one here decides whether a lease grants one extra
  // cycle of motion or loses one; either is fine, but it must be a decision.
  EXPECT_EQ(rm.resolve(in, limits()).source, ReferenceSource::Hold);
  in.valid_until_ns = 1'000'000'001;
  EXPECT_EQ(rm.resolve(in, limits()).source, ReferenceSource::Roam);
}

TEST(MotionIntent, LosIntentSolvesThroughTheV1Geometry) {
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::AutoTrack;
  in.type = IntentType::LosDirection;
  in.has_los = true;
  in.los_az_rad = 0.3;
  in.los_el_rad = 0.1;
  in.confidence = 1.0;
  auto req = rm.resolve(in, limits());
  EXPECT_EQ(req.source, ReferenceSource::Tracking);
  EXPECT_TRUE(req.is_tracking_reference);
  // Aligned kinematics: q_yaw = az, q_pitch = -el — same answer the v1 path gave,
  // which is the whole point of §111.18 (the v1 geometry stays authoritative).
  EXPECT_NEAR(req.q_yaw_rad, 0.3, 1e-6);
  EXPECT_NEAR(req.q_pitch_rad, -0.1, 1e-6);
  EXPECT_NEAR(req.v_max_rad_s, 30.0 * kDeg, 1e-9);
}

TEST(MotionIntent, ConfidenceDeratingArrivesAsAScaleNotAsASecret) {
  // §19/§20.1: derating is visible on the intent, so telemetry can show what was
  // asked versus what the loop allowed, instead of a controller quietly moving
  // slower and nobody being able to prove it from the log.
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::AutoTrack;
  in.type = IntentType::LosDirection;
  in.has_los = true;
  in.velocity_scale = 0.5;
  auto half = rm.resolve(in, limits());
  EXPECT_NEAR(half.v_max_rad_s, 15.0 * kDeg, 1e-9);
  in.velocity_scale = 0.0;  // INVALID (§19): coast/hold speed
  EXPECT_NEAR(rm.resolve(in, limits()).v_max_rad_s, 0.0, 1e-12);
}

TEST(MotionIntent, AbsurdScaleIsCappedRatherThanObeyed) {
  // A config typo of 10.0 in a velocity_scale must not become ten times the
  // speed. The envelope still has the final word, but a converter that multiplies
  // without limit hands the next layer a number it has no reason to expect.
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::AutoRoam;
  in.type = IntentType::JointPosition;
  in.has_joint_target = true;
  in.velocity_scale = 10.0;
  EXPECT_NEAR(rm.resolve(in, limits()).v_max_rad_s, 15.0 * kDeg, 1e-9);  // 1.5x cap
  in.velocity_scale = -1.0;
  EXPECT_NEAR(rm.resolve(in, limits()).v_max_rad_s, 0.0, 1e-12);
}

TEST(MotionIntent, JointPositionKeepsItsSourceSoTheUICanSayWhoMovedIt) {
  auto rm = make_rm();
  MotionIntent manual;
  manual.source = MotionSource::Manual;
  manual.type = IntentType::JointPosition;
  manual.has_joint_target = true;
  manual.q_yaw_rad = 0.2;
  auto rm_manual = rm.resolve(manual, limits());
  EXPECT_EQ(rm_manual.source, ReferenceSource::Manual);
  EXPECT_STREQ(reference_source_name(rm_manual.source), "manual");
  EXPECT_NEAR(rm_manual.v_max_rad_s, 25.0 * kDeg, 1e-9);

  MotionIntent roam = manual;
  roam.source = MotionSource::AutoRoam;
  auto rm_roam = rm.resolve(roam, limits());
  EXPECT_EQ(rm_roam.source, ReferenceSource::Roam);
  EXPECT_NEAR(rm_roam.v_max_rad_s, 10.0 * kDeg, 1e-9)
      << "roam must not inherit the manual speed ceiling just because both emit "
         "a joint position";
}

TEST(MotionIntent, MissingPayloadHoldsInsteadOfUsingZeroAsUnset) {
  // A default-constructed azimuth of 0.0 is "straight ahead", not "no target".
  // Reading it as unset is how a turret ends up pointing somewhere nobody asked
  // for, so the has_* flags are the only thing the converter may trust.
  auto rm = make_rm();
  MotionIntent los;
  los.source = MotionSource::AutoTrack;
  los.type = IntentType::LosDirection;  // has_los == false
  auto r1 = rm.resolve(los, limits());
  EXPECT_EQ(r1.source, ReferenceSource::Hold);
  EXPECT_STREQ(r1.reason, "los intent without los");

  MotionIntent joint;
  joint.source = MotionSource::Manual;
  joint.type = IntentType::JointPosition;  // has_joint_target == false
  auto r2 = rm.resolve(joint, limits());
  EXPECT_EQ(r2.source, ReferenceSource::Hold);
  EXPECT_STREQ(r2.reason, "joint intent without target");
}

TEST(MotionIntent, VelocityIntentIsRefusedOutLoud) {
  // Interim until V3-5, stated rather than silently converted into a position
  // move: measured on this station, drive speed mode does not track a commanded
  // rate on a loaded axis, so a jog must be integrated to position at the control
  // rate by ManualController. A converter that faked it would make the button
  // look implemented.
  auto rm = make_rm();
  MotionIntent in;
  in.source = MotionSource::Manual;
  in.type = IntentType::JointVelocity;
  in.has_joint_velocity = true;
  in.v_yaw_rad_s = 0.1;
  auto req = rm.resolve(in, limits());
  EXPECT_EQ(req.source, ReferenceSource::Hold);
  EXPECT_STREQ(req.reason, "velocity intent not yet supported");
}

TEST(MotionIntent, ReasonIsBoundedAndNeverOverruns) {
  // §25 carries a human-readable reason in a fixed buffer, because the intent is
  // built on the 200 Hz thread (§46: no allocation there). Truncation is the
  // designed behaviour; a write past the end is not.
  MotionIntent in;
  in.set_reason("selected target left the field of regard mid-sweep");
  EXPECT_STREQ(in.reason, "selected target lef");  // 19 chars + NUL
  EXPECT_EQ(sizeof(in.reason), 20u);
  in.set_reason(nullptr);
  EXPECT_STREQ(in.reason, "");
  MotionIntent h = MotionIntent::hold(MotionSource::AutoTrack, "no target");
  EXPECT_EQ(h.source, MotionSource::AutoTrack);
  EXPECT_EQ(h.type, IntentType::Hold);
  EXPECT_STREQ(h.reason, "no target");
  EXPECT_TRUE(h.live_at(123)) << "a hold with no deadline must never age out";
}
