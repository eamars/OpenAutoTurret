// §87 — MANUAL unit tests. The lease is the interesting part: every other case is a
// question about a number, and the lease is a question about what happens when the
// operator's browser stops existing.
#include <gtest/gtest.h>

#include "mode/manual_controller.hpp"

namespace ota {
namespace {

constexpr TimeNs kMs = 1'000'000;
constexpr TimeNs kCycle = 5'000'000;  // 200 Hz
constexpr double kDeg = 3.14159265358979323846 / 180.0;

class ManualTest : public ::testing::Test {
 protected:
  void step(int n = 1) {
    for (int i = 0; i < n; ++i) {
      now_ += kCycle;
      out_ = m_.update(q_, v_max_, now_, kCycle);
    }
  }

  ManualController m_;
  double q_[2] = {0.0, 0.0};        // {pitch, yaw}, logical
  double v_max_[2] = {0.35, 0.35};  // the station's configured manual limit
  ManualOutput out_;
  TimeNs now_ = 1'000'000'000;
};

TEST_F(ManualTest, JogLeaseRenewsAndExpires) {
  // {yaw, pitch}. Written out because an accidental {0,+1} here is a pitch jog that
  // silently leaves every yaw assertion vacuous — which is what this line originally
  // did, and the yaw test passed while testing nothing about yaw.
  JogDirection d{+1, 0};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Normal, now_));
  step();
  EXPECT_TRUE(out_.lease_active);
  EXPECT_GT(out_.lease_remaining_ms, 0);
  EXPECT_STREQ(out_.reason, "jogging");

  // The browser's ~100 ms refresh (§38/§72), comfortably inside the 300 ms lease.
  for (int i = 0; i < 12; ++i) {
    now_ += 100 * kMs / 12;
    ASSERT_TRUE(m_.jog_keepalive(now_))
        << "a keepalive inside the lease was refused; the turret would stop under a "
           "held button";
    step();
    EXPECT_TRUE(out_.lease_active);
  }

  // Silence. Not a stop message — the absence of one, which is the only signal a
  // dropped wifi or a closed tab reliably produces.
  // Walked cycle by cycle rather than stepped to a computed instant: what is being
  // asserted is that the expiry is *announced*, and a hand-computed cycle count would
  // be off by one more often than a test like this deserves.
  bool announced = false;
  for (int i = 0; i < 120 && !announced; ++i) {
    step();
    announced = (std::string(out_.reason) == "jog lease expired");
  }
  EXPECT_TRUE(announced)
      << "the lease lapsed without saying so; out_.reason was " << out_.reason;
  EXPECT_FALSE(m_.lease_active());
  step();
  EXPECT_STREQ(out_.reason, "manual hold")
      << "and after announcing it, the turret sits in MANUAL/HOLD";
  EXPECT_EQ(out_.intent.type, IntentType::Hold)
      << "§38: an expired lease is a controlled stop to MANUAL/HOLD, not a request "
         "that keeps its old target";
}

TEST_F(ManualTest, KeepaliveWithNoLeaseIsAnsweredHonestly) {
  EXPECT_FALSE(m_.jog_keepalive(now_))
      << "renewing nothing was reported as a renewal";
  JogDirection d{0, +1};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Fine, now_));
  step(70);  // 350 ms: the lease lapsed on its own
  EXPECT_FALSE(m_.jog_keepalive(now_))
      << "a lapsed lease resurrected by a late keepalive. The operator was holding the "
         "button the whole time; the turret stopped anyway, and must say so rather "
         "than pretend the lease never ended";
}

TEST_F(ManualTest, StopCommandIsEquivalentToReleasingTheButton) {
  JogDirection d{+1, 0};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Fast, now_));
  step();
  ASSERT_TRUE(out_.lease_active);
  m_.jog_stop(now_);
  step();
  EXPECT_FALSE(out_.lease_active);
  EXPECT_EQ(out_.intent.type, IntentType::Hold);
  EXPECT_STREQ(out_.reason, "manual hold");
}

TEST_F(ManualTest, ProfileScalesTheStationLimitRatherThanReplacingIt) {
  // §39: the profiles map to validated limits. FAST is 1.0 of the configured manual
  // ceiling, so the ceiling stays the thing that was commissioned; FINE is a fraction
  // of it. A profile that carried its own absolute speed would be a second,
  // uncommissioned limit somewhere in the system, and the one on this station has been
  // measured.
  JogDirection d{+1, 0};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Fast, now_));
  step();
  const double fast_v = out_.intent.v_yaw_rad_s;
  EXPECT_DOUBLE_EQ(fast_v, v_max_[1] * m_.config().fast.velocity_scale);

  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Fine, now_));
  step();
  EXPECT_LT(out_.intent.v_yaw_rad_s, fast_v)
      << "FINE and FAST produced the same turret";
  EXPECT_GT(out_.intent.v_yaw_rad_s, 0.0);
}

TEST_F(ManualTest, JogTargetStaysAheadAndReturnsWhenBlocked) {
  // §70's "jog toward the limit" at this level: the reference must sit ahead of the
  // turret in the requested direction, and — the part that matters — must NOT run away
  // while the turret is not moving. The envelope is what reduces the allowed speed near
  // a limit (tested in v1's envelope suite); what is tested here is that Manual
  // integration does not bank up a debt to spend when the limit moves.
  JogDirection d{+1, 0};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Normal, now_));
  step(20);  // 100 ms, turret pinned at q_=0 by the test rig
  const double target_then = out_.intent.q_yaw_rad;
  EXPECT_GT(target_then, q_[1]) << "the reference is behind the motion requested";
  step(20);
  EXPECT_NEAR(out_.intent.q_yaw_rad, target_then, 1e-9)
      << "the reference advanced while the turret did not. When the way opens, that "
         "stored distance is spent in one go — a lurch, at the moment the operator was "
         "told to be careful";
  // A horizon-sized step ahead, never more, whatever the lease has been doing.
  EXPECT_LT(out_.intent.q_yaw_rad,
            q_[1] + v_max_[1] * m_.config().normal.velocity_scale *
                        (m_.config().jog_horizon_ms / 1000.0) * 1.001);
}

TEST_F(ManualTest, DiagonalJogIsOneRequestNotTwo) {
  // §39 allows combined jog when the reference interface supports it. It does: both
  // axes travel in one ReferenceRequest, which is also what makes the envelope's limits
  // apply to the pair. Two separate requests would each be limited as if the other axis
  // were still.
  JogDirection d{+1, -1};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Normal, now_));
  step();
  EXPECT_TRUE(out_.intent.has_joint_target);
  EXPECT_NE(out_.intent.v_yaw_rad_s, 0.0);
  EXPECT_NE(out_.intent.v_pitch_rad_s, 0.0);
  EXPECT_LT(out_.intent.v_pitch_rad_s, 0.0);
}

TEST_F(ManualTest, StepIsARelativeMoveThatCompletes) {
  // §41: current logical q + delta, through v1 safety and v1 trajectory. The controller
  // does not clamp it to the limits itself — that is the envelope's job, and a silent
  // clamp would move the turret somewhere slightly different from where it was told.
  q_[1] = 0.30;
  ASSERT_TRUE(m_.step_move(1, 1.0 * kDeg, q_[1], now_));
  step();
  EXPECT_EQ(out_.intent.type, IntentType::JointPosition);
  EXPECT_NEAR(out_.intent.q_yaw_rad, 0.30 + 1.0 * kDeg, 1e-12);
  EXPECT_TRUE(out_.step_in_progress);
  EXPECT_DOUBLE_EQ(out_.intent.q_pitch_rad, q_[0])
      << "a yaw step also moved pitch";

  // The rig "arrives": feedback reaches the target, so the intent becomes a hold.
  q_[1] = 0.30 + 1.0 * kDeg;
  step();
  EXPECT_FALSE(out_.step_in_progress);
  EXPECT_EQ(out_.intent.type, IntentType::Hold);
  EXPECT_STREQ(out_.reason, "step complete");
}

TEST_F(ManualTest, StepThatNeverArrivesIsDroppedAndSaidSo) {
  // A step refused or blocked must not leave a position target pushing against a limit
  // forever.
  q_[1] = 0.0;
  ASSERT_TRUE(m_.step_move(1, 5.0 * kDeg, q_[1], now_));
  step(800);  // exactly the 4000 ms deadline, and no further: the cycle after the
              // timeout is an ordinary hold, and that is a different assertion.
  EXPECT_FALSE(out_.step_in_progress);
  EXPECT_STREQ(out_.reason, "step timed out");
  EXPECT_EQ(out_.intent.type, IntentType::Hold);
}

TEST_F(ManualTest, RefusedStepIsReportedOnce) {
  q_[1] = 0.0;
  ASSERT_TRUE(m_.step_move(1, 5.0 * kDeg, q_[1], now_));
  step();
  m_.notify_step_refused();
  step();
  EXPECT_TRUE(out_.step_rejected);
  EXPECT_NE(std::string(out_.step_reject_reason).find("envelope"), std::string::npos);
  step();
  EXPECT_FALSE(out_.step_rejected)
      << "a one-shot refusal became a permanent state, which will read as the turret "
         "being broken long after it was answered";
}

TEST_F(ManualTest, JogArgGrammarIsSharedAndStrict) {
  JogDirection d{};
  ManualProfile p = ManualProfile::Normal;
  char why[64] = {};
  EXPECT_TRUE(ManualController::parse_jog_arg("yaw+:fine", d, p, why, sizeof why));
  EXPECT_EQ(d.yaw, 1);
  EXPECT_EQ(p, ManualProfile::Fine);
  EXPECT_TRUE(ManualController::parse_jog_arg("yaw+|pitch-", d, p, why, sizeof why));
  EXPECT_EQ(d.yaw, 1);
  EXPECT_EQ(d.pitch, -1);
  EXPECT_FALSE(ManualController::parse_jog_arg("yaw++", d, p, why, sizeof why));
  EXPECT_FALSE(ManualController::parse_jog_arg("", d, p, why, sizeof why));
  EXPECT_FALSE(ManualController::parse_jog_arg("left", d, p, why, sizeof why));
  EXPECT_FALSE(ManualController::parse_jog_arg("yaw:ultra", d, p, why, sizeof why))
      << "an unknown profile must be refused, not run at NORMAL silently: the operator "
         "chose a speed and got a different one";
  EXPECT_STREQ(why, "unknown speed profile (fine/normal/fast)");
}

TEST_F(ManualTest, LeavingManualCancelsEverything) {
  JogDirection d{+1, 0};
  ASSERT_TRUE(m_.jog_start(d, ManualProfile::Normal, now_));
  step();
  ASSERT_TRUE(m_.lease_active());
  m_.cancel(now_);
  step();
  EXPECT_FALSE(m_.lease_active());
  EXPECT_EQ(out_.intent.type, IntentType::Hold)
      << "§93: a jog that survived a mode change would resume motion under a mode that "
         "no longer owns the axes";
}

}  // namespace
}  // namespace ota
