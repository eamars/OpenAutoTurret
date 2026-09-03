// v3 §84 — ModeManager transition tests, plus the two things the spec freezes
// about modes: exactly one mode owns motion at a time (§26), and a supervisory
// state outranks all of them (§2).
//
// These are written as "what the operator's button does", not as branch
// coverage: the interesting failure of a mode manager is not a wrong boolean, it
// is a turret that changed what it was doing without being told to.
#include <gtest/gtest.h>

#include "mode/mode_manager.hpp"

namespace {

using ota::ModeRequestContext;
using ota::OperatingMode;
using ota::SupervisoryState;

ModeRequestContext ok_ctx() {
  ModeRequestContext c;
  c.position_valid = true;
  c.safety_healthy = true;
  c.roam_envelope_valid = true;
  c.supervisory = SupervisoryState::Ready;
  return c;
}

}  // namespace

TEST(ModeManager, StartsInManualHold) {
  // §73 / Appendix E: power on, home, and the turret sits in MANUAL/HOLD. Not
  // AUTO_ROAM "because it was left that way".
  ota::ModeManager m;
  EXPECT_EQ(m.mode(), OperatingMode::Manual);
  EXPECT_STREQ(operating_mode_name(m.mode()), "MANUAL");
}

TEST(ModeManager, AllSixTransitionsAreAllowedWhenPreconditionsHold) {
  const OperatingMode all[] = {OperatingMode::Manual, OperatingMode::AutoTrack,
                               OperatingMode::AutoRoam};
  for (const auto from : all) {
    for (const auto to : all) {
      ota::ModeManager m;
      ASSERT_TRUE(m.request(from, ok_ctx()).ok) << "setup: " << from;
      auto r = m.request(to, ok_ctx());
      EXPECT_TRUE(r.ok) << operating_mode_name(from) << " -> "
                        << operating_mode_name(to) << ": " << r.reason;
      EXPECT_EQ(m.mode(), to);
      EXPECT_EQ(r.changed, from != to)
          << "a request for the mode already in is an accepted no-op, not a "
             "transition to log as if it moved something";
    }
  }
}

TEST(ModeManager, AutoTrackNeedsNoTargetToBeEntered) {
  // §44: "A selected visible target is not required." It then holds in
  // WAIT_TARGET (§16). A gate that demanded a target would make the button
  // un-pressable while the camera is blocked — and the operator would have no
  // way to tell "mode refused" from "no target".
  auto ctx = ok_ctx();
  ota::ModeManager m;
  auto r = m.request(OperatingMode::AutoTrack, ctx);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(m.mode(), OperatingMode::AutoTrack);
}

TEST(ModeManager, UnhomedRefusesBothAutonomousModes) {
  for (const auto to : {OperatingMode::AutoTrack, OperatingMode::AutoRoam}) {
    auto ctx = ok_ctx();
    ctx.position_valid = false;
    ota::ModeManager m;
    auto r = m.request(to, ctx);
    EXPECT_FALSE(r.ok) << operating_mode_name(to);
    EXPECT_NE(std::string(r.reason).find("not homed"), std::string::npos)
        << "the reason must name the cause, not just 'rejected'";
  }
}

TEST(ModeManager, ManualIsAlwaysReachableEvenWithNoPositionAndNoSafety) {
  // The asymmetry in request() is deliberate and this is the test that pins it.
  // If a fault arrives while AUTO_TRACK is running and the operator presses
  // MANUAL, the answer has to be yes: MANUAL is the mode where they can act.
  // Trapping them in the mode that may be causing the problem, because a check
  // said the state was too unsafe to change, is the worst possible answer.
  // (What MANUAL may then DO is separate: a jog still needs its lease, a valid
  // position, and envelope clearance.)
  for (bool pos : {true, false}) {
    for (bool safe : {true, false}) {
      auto ctx = ok_ctx();
      ctx.position_valid = pos;
      ctx.safety_healthy = safe;
      ota::ModeManager m;
      ASSERT_TRUE(m.request(OperatingMode::AutoTrack, ok_ctx()).ok);
      auto r = m.request(OperatingMode::Manual, ctx);
      EXPECT_TRUE(r.ok) << "position_valid=" << pos << " safety=" << safe;
      EXPECT_EQ(m.mode(), OperatingMode::Manual);
    }
  }
}

TEST(ModeManager, UnderSupervisoryControlEvenManualWaits) {
  // The one place MANUAL does wait: mid-homing the motors belong to homing. The
  // answer is "not now, homing" — and STOP MOTION below still works instantly,
  // because an operator reaching for the STOP button must never be told to wait.
  auto ctx = ok_ctx();
  ctx.supervisory = SupervisoryState::Homing;
  ota::ModeManager m;
  // Start from an autonomous mode: already-in-Manual is an accepted no-op that
  // never reaches the gate, and that is the right behaviour (nothing changes, so
  // there is nothing to refuse).
  ASSERT_TRUE(m.request(OperatingMode::AutoRoam, ok_ctx()).ok);
  auto r = m.request(OperatingMode::Manual, ctx);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(std::string(r.reason).find("homing"), std::string::npos);
  EXPECT_TRUE(m.stop_motion(ctx).ok);
}

TEST(ModeManager, SafetyFaultRefusesEntryIntoAutonomy) {
  for (const auto to : {OperatingMode::AutoTrack, OperatingMode::AutoRoam}) {
    auto ctx = ok_ctx();
    ctx.safety_healthy = false;
    ota::ModeManager m;
    EXPECT_FALSE(m.request(to, ctx).ok) << operating_mode_name(to);
  }
}

TEST(ModeManager, RoamIsRefusedWithoutAValidEnvelope) {
  // §32/§69: the roam envelope is a separate, inner region. If it is empty or
  // outside the safe envelope, AUTO_ROAM must not start — and must not quietly
  // fall back to sweeping the full soft-limit band, which is the tempting
  // implementation and the one that finds the mechanical stops.
  auto ctx = ok_ctx();
  ctx.roam_envelope_valid = false;
  ota::ModeManager m;
  auto r = m.request(OperatingMode::AutoRoam, ctx);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(std::string(r.reason).find("envelope invalid"), std::string::npos);
  EXPECT_EQ(m.mode(), OperatingMode::Manual);
  // AUTO_TRACK with the same context is fine: the envelope is roam's problem.
  EXPECT_TRUE(m.request(OperatingMode::AutoTrack, ctx).ok);
}

TEST(ModeManager, SupervisoryStateOutranksTheRequest) {
  // §26. A queued mode change taking effect halfway through a homing run would
  // be motion nobody asked for, at the least predictable moment.
  auto ctx = ok_ctx();
  ctx.supervisory = SupervisoryState::Homing;
  ota::ModeManager m;
  auto r = m.request(OperatingMode::AutoRoam, ctx);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(std::string(r.reason).find("homing"), std::string::npos);
  EXPECT_EQ(m.mode(), OperatingMode::Manual);
}

TEST(ModeManager, StopMotionAlwaysLandsInManualHold) {
  // §27. From every mode, and even while a supervisory state is active: the
  // operator pressing STOP MOTION must not have to know the state machine to
  // get a stop. It is the one request that never argues.
  for (const auto from : {OperatingMode::Manual, OperatingMode::AutoTrack,
                          OperatingMode::AutoRoam}) {
    ota::ModeManager m;
    ASSERT_TRUE(m.request(from, ok_ctx()).ok);
    auto r = m.stop_motion(ok_ctx());
    EXPECT_TRUE(r.ok);
    EXPECT_EQ(m.mode(), OperatingMode::Manual);
    EXPECT_NE(std::string(r.reason).find("manual hold"), std::string::npos);
  }
  // ...including under homing.
  ota::ModeManager m;
  ASSERT_TRUE(m.request(OperatingMode::AutoRoam, ok_ctx()).ok);
  m.notify_supervisory(SupervisoryState::Homing);
  EXPECT_TRUE(m.stop_motion(ModeRequestContext{}).ok);
  EXPECT_EQ(m.mode(), OperatingMode::Manual);
}

TEST(ModeManager, FaultDropsAnAutonomousModeImmediately) {
  ota::ModeManager m;
  ASSERT_TRUE(m.request(OperatingMode::AutoTrack, ok_ctx()).ok);
  m.notify_supervisory(SupervisoryState::Fault);
  EXPECT_EQ(m.mode(), OperatingMode::Manual)
      << "a fault can be the reason the running mode was unsafe; it does not "
         "get to resume";
}

TEST(ModeManager, ReturnFromHomingLandsInManual) {
  // §84: "return from homing to safe MANUAL/HOLD". Not back into the roam
  // pattern that was running before power was cycled — that is how a station
  // starts sweeping on its own after a reboot, with nobody watching.
  ota::ModeManager m;
  ASSERT_TRUE(m.request(OperatingMode::AutoRoam, ok_ctx()).ok);
  m.notify_supervisory(SupervisoryState::Homing);
  EXPECT_TRUE(m.mode_is_overridden());
  EXPECT_EQ(m.mode(), OperatingMode::AutoRoam)
      << "during homing the requested mode is still shown for the UI, but the "
         "override flag is what says it owns nothing";
  m.notify_supervisory(SupervisoryState::Ready);
  EXPECT_FALSE(m.mode_is_overridden());
  EXPECT_EQ(m.mode(), OperatingMode::Manual);
}

TEST(ModeManager, EpochIsTheStaleIntentTripwire) {
  // §43/§93: "no stale mode intent survives transition". Every transition bumps
  // the epoch; a controller stamps its intent with the epoch it saw, and the
  // loop drops anything not current. Test the tripwire itself: two transitions
  // must produce two distinguishable epochs.
  ota::ModeManager m;
  const uint64_t e0 = m.epoch();
  ASSERT_TRUE(m.request(OperatingMode::AutoTrack, ok_ctx()).ok);
  const uint64_t e1 = m.epoch();
  ASSERT_TRUE(m.request(OperatingMode::AutoRoam, ok_ctx()).ok);
  const uint64_t e2 = m.epoch();
  EXPECT_GT(e1, e0);
  EXPECT_GT(e2, e1);
  // A refused request must NOT bump: an epoch that moves on a refusal would
  // invalidate the perfectly good intent of the mode that stayed in charge.
  // (Asked of AUTONOMY: a refused step *into* MANUAL is no longer a thing —
  // escaping to MANUAL is unconditional, see ManualIsAlwaysReachable...)
  auto ctx = ok_ctx();
  ctx.position_valid = false;
  EXPECT_FALSE(m.request(OperatingMode::AutoTrack, ctx).ok);
  EXPECT_EQ(m.epoch(), e2);
  // A refused no-op must not bump either.
  ASSERT_TRUE(m.request(OperatingMode::AutoRoam, ctx).ok);
  EXPECT_EQ(m.epoch(), e2);
}

TEST(ModeManager, ModeNamesParseTheDocumentedSpellingsAndRejectTheRest) {
  OperatingMode m = OperatingMode::Manual;
  EXPECT_TRUE(operating_mode_from_name("AUTO_TRACK", m));
  EXPECT_EQ(m, OperatingMode::AutoTrack);
  EXPECT_TRUE(operating_mode_from_name("auto_roam", m));
  EXPECT_EQ(m, OperatingMode::AutoRoam);
  EXPECT_TRUE(operating_mode_from_name("Manual", m));
  EXPECT_EQ(m, OperatingMode::Manual);
  // §51 has no "SEARCH": v1's search becomes an AUTO_ROAM phase, and silently
  // accepting the old spelling would keep a stale UI half-working — worse than
  // failing it with a reason.
  EXPECT_FALSE(operating_mode_from_name("SEARCH", m));
  EXPECT_FALSE(operating_mode_from_name("", m));
  EXPECT_FALSE(operating_mode_from_name(nullptr, m));
}
