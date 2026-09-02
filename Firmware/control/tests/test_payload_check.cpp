// Unit tests for the in-loop payload response check stepper's safe-region
// handling (§31.3, §44). Regression for the P6 pre-flight blocker: the
// check must run from a per-axis region centered on the start pose, NOT
// from the static default region (0 +/- 20 deg), which the live ready pose
// (pitch -1.4960 rad / yaw -2.2608 rad) does not lie in — begin() used to
// fail "start pose outside the safe central region".
#include <gtest/gtest.h>

#include <cmath>

#include "common/types.hpp"
#include "control/motor_backend.hpp"
#include "payload/payload_check.hpp"

using ota::AxisId;
using ota::AxisSnapshot;
using ota::TimeNs;
using ota::payload::PayloadCheck;
using ota::payload::PayloadCheckConfig;

namespace {

constexpr double kDeg = M_PI / 180.0;
// The live ready pose (2026-09-02 homing): far outside the default region.
constexpr double kLiveReadyPitchRad = -1.4960;
constexpr double kLiveReadyYawRad = -2.2608;

}  // namespace

TEST(PayloadCheckRegion, LiveReadyPoseIsOutsideDefaultRegion) {
  // The pre-P6 behavior: default-constructed check (default cfg -> default
  // region 0 +/- 20 deg) rejects the live ready pose.
  PayloadCheck check;
  EXPECT_FALSE(check.begin(0, kLiveReadyPitchRad, true));
  EXPECT_EQ(check.fail_reason(), "start pose outside the safe central region");
}

TEST(PayloadCheckRegion, PerAxisRegionCenteredOnPoseAcceptsLiveReadyPose) {
  PayloadCheck check;
  check.set_region(kLiveReadyPitchRad, 10.0 * kDeg);
  ASSERT_TRUE(check.begin(1'000'000'000, kLiveReadyPitchRad, true));
  // Full 2 deg amplitude: region half-span (10 deg) - 2 deg margin > 2 deg.
  EXPECT_NEAR(check.amplitude_rad(), 2.0 * kDeg, 1e-12);
}

TEST(PayloadCheckRegion, BothLiveReadyPosesAccepted) {
  for (int i = 0; i < 2; ++i) {
    const double q = (i == 0) ? kLiveReadyPitchRad : kLiveReadyYawRad;
    PayloadCheck check;
    check.set_region(q, 10.0 * kDeg);
    ASSERT_TRUE(check.begin(0, q, true)) << check.fail_reason();
    EXPECT_NEAR(check.amplitude_rad(), 2.0 * kDeg, 1e-12);
  }
}

TEST(PayloadCheckRegion, TinyRegionFailsMinAmplitudeGuard) {
  // Half-span 2 deg: the 2 deg edge margin eats the whole span.
  PayloadCheck check;
  check.set_region(kLiveReadyPitchRad, 2.0 * kDeg);
  EXPECT_FALSE(check.begin(0, kLiveReadyPitchRad, true));
  EXPECT_EQ(check.fail_reason(),
            "safe central region too small for a conservative check move");
}

TEST(PayloadCheckRegion, NarrowRegionClampsAmplitude) {
  PayloadCheck check;
  check.set_region(kLiveReadyPitchRad, 3.0 * kDeg);
  ASSERT_TRUE(check.begin(0, kLiveReadyPitchRad, true));
  // amp = min(2 deg step, 3 deg half-span - 2 deg margin) = 1 deg.
  EXPECT_NEAR(check.amplitude_rad(), 1.0 * kDeg, 1e-12);
}

TEST(PayloadCheckRegion, NoFeedbackAtStartFails) {
  PayloadCheck check;
  check.set_region(kLiveReadyPitchRad, 10.0 * kDeg);
  EXPECT_FALSE(check.begin(0, kLiveReadyPitchRad, false));
  EXPECT_EQ(check.fail_reason(), "no axis feedback at check start");
}

TEST(PayloadCheckRegion, ReferencesStayInsideRegionAndReturnToStart) {
  // Drive the full PosStep -> PosReturn -> NegStep -> NegReturn sequence with
  // an ideal-tracking plant: every reference must stay inside the region and
  // the check must finish back at the start pose with measured steps.
  const double q0 = kLiveReadyYawRad;
  const double half_span = 3.0 * kDeg;
  PayloadCheck check;
  check.set_region(q0, half_span);
  ASSERT_TRUE(check.begin(0, q0, true));  // amp clamps to 1 deg

  AxisSnapshot s;
  s.has_feedback = true;
  s.q_rad = q0;
  s.v_rad_s = 0.0;
  TimeNs t = 1'000'000'000;
  double last_ref = q0;
  for (int i = 0; i < 5000 && check.active(); ++i) {
    t += 5'000'000;
    const auto out = check.step(t, s);
    last_ref = out.q_ref_rad;
    EXPECT_GE(out.q_ref_rad, q0 - half_span - 1e-12);
    EXPECT_LE(out.q_ref_rad, q0 + half_span + 1e-12);
    // Ideal tracking: the plant reaches the reference this cycle.
    s.q_rad = out.q_ref_rad;
    s.v_rad_s = 0.0;
  }
  EXPECT_FALSE(check.active());
  EXPECT_FALSE(check.failed()) << check.fail_reason();
  EXPECT_NEAR(last_ref, q0, 1e-9);
  EXPECT_TRUE(check.axis_result().measured);
}

TEST(PayloadCheckRegion, OffTargetSteadyStateSettlesWithinBand) {
  // Regression for P6 live Run A: the CyberGear position loop settles
  // ~0.057 deg (2.9 % of the 2 deg step) PAST the LocRef — the friction
  // deadband holds the axis just off target, and the position-loop P torque
  // at that error balances the load (P-equilibrium). The old ±2 % settle
  // band (0.04 deg at the 2 deg step) was BELOW that steady-state offset, so
  // the +step never confirmed settled and the 8 s move timeout aborted the
  // check. The band is now ±5 % of amplitude (0.1 deg): an axis that settles
  // 0.0010 rad off target must settle and the full battery must complete.
  const double q0 = kLiveReadyPitchRad;
  PayloadCheck check;
  check.set_region(q0, 10.0 * kDeg);
  ASSERT_TRUE(check.begin(1'000'000'000, q0, true));
  const double amp = check.amplitude_rad();  // full 2 deg
  ASSERT_NEAR(amp, 2.0 * kDeg, 1e-12);
  const double offset = 0.0010;  // rad = 0.057 deg: the live drive's steady-state error

  AxisSnapshot s;
  s.has_feedback = true;
  s.q_rad = q0;
  s.v_rad_s = 0.0;
  TimeNs t = 1'000'000'000;
  const double tau = 0.05;    // s: position-loop time constant
  const double dt = 5e-3;     // s: control period
  const double v_max = 5.0 * kDeg;  // deg/s: the conservative check speed
  for (int i = 0; i < 5000 && check.active(); ++i) {
    t += 5'000'000;
    const auto out = check.step(t, s);
    // Plant: speed-limited first-order chase of (reference + steady-state
    // offset) — the offset is the friction deadband bias around the LocRef,
    // present at every commanded position (as on the live drive).
    const double target = out.q_ref_rad + offset;
    double dq = (target - s.q_rad) * (dt / (tau + dt));
    if (dq > v_max * dt) dq = v_max * dt;
    if (dq < -v_max * dt) dq = -v_max * dt;
    s.v_rad_s = dq / dt;
    s.q_rad += dq;
  }
  EXPECT_FALSE(check.active());
  EXPECT_FALSE(check.failed()) << check.fail_reason();
  EXPECT_TRUE(check.axis_result().measured);
  // Both steps measured with sane amplitudes (the offset is small vs amp).
  EXPECT_NEAR(check.axis_result().step_pos.amplitude_rad, amp, 0.05 * amp);
  EXPECT_NEAR(check.axis_result().step_neg.amplitude_rad, amp, 0.05 * amp);
}
