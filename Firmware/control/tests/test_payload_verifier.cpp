// Tests for the payload verification comparison (§31.3, §28.5): baseline
// comparison, overall status folding, and the derating decision. Pure logic.
#include <gtest/gtest.h>

#include <string>

#include "common/types.hpp"
#include "payload/payload_verifier.hpp"

namespace {

using ota::kAxisCount;
using ota::payload::AxisPayloadProfile;
using ota::payload::AxisVerifyResult;
using ota::payload::DerateDecision;
using ota::payload::PayloadStatus;
using ota::payload::StepMetrics;
using ota::payload::VerifyResult;
using ota::payload::VerifyTolerances;
using ota::payload::compare_axis;
using ota::payload::decide;
using ota::payload::overall_status;
using ota::payload::payload_status_name;

constexpr double kAmp = 0.0524;  // 3 deg

// A clean measured step.
StepMetrics good_step(double rise = 0.11, double settle = 0.20,
                      double effort = 0.5, double rms = 0.001,
                      double overshoot = 0.0) {
  StepMetrics s;
  s.valid = true;
  s.amplitude_rad = kAmp;
  s.rise_time_s = rise;
  s.settling_time_s = settle;
  s.peak_effort_nm = effort;
  s.tracking_rms_rad = rms;
  s.overshoot = overshoot;
  return s;
}

AxisPayloadProfile make_baseline() {
  AxisPayloadProfile b;
  b.v_max_rad_s = 0.35;
  b.a_max_rad_s2 = 1.0;
  b.j_max_rad_s3 = 3.0;
  b.baseline.step_pos = good_step();
  b.baseline.step_neg = good_step();
  b.baseline.valid = true;
  return b;
}

// Fold two axis results into the overall status.
PayloadStatus fold(const AxisVerifyResult& pitch, const AxisVerifyResult& yaw,
                   bool profile_loaded) {
  AxisVerifyResult axes[kAxisCount] = {pitch, yaw};
  return overall_status(axes, profile_loaded);
}

}  // namespace

TEST(PayloadVerifier, IdenticalBaselineIsOk) {
  const auto base = make_baseline();
  const AxisVerifyResult r =
      compare_axis(base, base.baseline.step_pos, base.baseline.step_neg,
                   VerifyTolerances{});
  EXPECT_TRUE(r.measured);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.violations.empty());
}

TEST(PayloadVerifier, SlowerRiseIsAMismatch) {
  const auto base = make_baseline();
  // 3x slower rise: outside the [0.4, 2.5] ratio window.
  const StepMetrics slow = good_step(/*rise=*/0.33);
  const AxisVerifyResult r = compare_axis(base, slow, base.baseline.step_neg,
                                          VerifyTolerances{});
  EXPECT_TRUE(r.measured);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const auto& v : r.violations)
    if (v.find("rise") != std::string::npos) found = true;
  EXPECT_TRUE(found) << "expected a rise violation, got: "
                     << r.violations.front();
}

TEST(PayloadVerifier, FasterRiseWithinWindowIsOk) {
  const auto base = make_baseline();
  // 2x faster rise: inside the ratio window -> no violation.
  const StepMetrics fast = good_step(/*rise=*/0.055);
  const AxisVerifyResult r = compare_axis(base, fast, base.baseline.step_neg,
                                          VerifyTolerances{});
  EXPECT_TRUE(r.ok) << r.violations.front();
}

TEST(PayloadVerifier, HigherEffortIsAMismatch) {
  const auto base = make_baseline();
  // 3x the peak effort: beyond ratio (2x) + absolute floor.
  const StepMetrics heavy = good_step(0.11, 0.20, /*effort=*/1.6);
  const AxisVerifyResult r = compare_axis(base, heavy, base.baseline.step_neg,
                                          VerifyTolerances{});
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const auto& v : r.violations)
    if (v.find("effort") != std::string::npos) found = true;
  EXPECT_TRUE(found);
}

TEST(PayloadVerifier, HigherOvershootIsAMismatch) {
  const auto base = make_baseline();
  const StepMetrics wobbly = good_step(0.11, 0.20, 0.5, 0.001, 0.03);  // +3 % abs overshoot
  const AxisVerifyResult r = compare_axis(base, wobbly, base.baseline.step_neg,
                                          VerifyTolerances{});
  EXPECT_FALSE(r.ok);
}

TEST(PayloadVerifier, WorseOfBothStepsWins) {
  const auto base = make_baseline();
  // step_pos is clean, step_neg is 3x slower -> the axis fails.
  const StepMetrics bad_neg = good_step(/*rise=*/0.4);
  const AxisVerifyResult r = compare_axis(base, base.baseline.step_pos, bad_neg,
                                          VerifyTolerances{});
  EXPECT_TRUE(r.measured);
  EXPECT_FALSE(r.ok);
}

TEST(PayloadVerifier, UnmeasuredAxisIsNotOk) {
  const auto base = make_baseline();
  const AxisVerifyResult r =
      compare_axis(base, StepMetrics{}, StepMetrics{}, VerifyTolerances{});
  EXPECT_FALSE(r.measured);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.violations.empty());
}

TEST(PayloadVerifier, OverallStatusFoldsPerAxisResults) {
  const auto base = make_baseline();
  const AxisVerifyResult ok = compare_axis(base, base.baseline.step_pos,
                                           base.baseline.step_neg,
                                           VerifyTolerances{});
  const AxisVerifyResult bad =
      compare_axis(base, good_step(0.5), base.baseline.step_neg,
                   VerifyTolerances{});
  const AxisVerifyResult unmeasured;  // measured=false, ok=false

  EXPECT_EQ(fold(ok, ok, true), PayloadStatus::Ok);
  EXPECT_EQ(fold(bad, ok, true), PayloadStatus::Mismatch);
  EXPECT_EQ(fold(ok, bad, true), PayloadStatus::Mismatch);
  EXPECT_EQ(fold(bad, bad, true), PayloadStatus::Mismatch);
  EXPECT_EQ(fold(unmeasured, ok, true), PayloadStatus::Error);
  // No profile loaded: always NoProfile, regardless of measurements.
  EXPECT_EQ(fold(ok, ok, false), PayloadStatus::NoProfile);
  EXPECT_EQ(fold(bad, bad, false), PayloadStatus::NoProfile);
}

TEST(PayloadVerifier, StatusNames) {
  EXPECT_STREQ(payload_status_name(PayloadStatus::NoProfile), "no_profile");
  EXPECT_STREQ(payload_status_name(PayloadStatus::Ok), "ok");
  EXPECT_STREQ(payload_status_name(PayloadStatus::Mismatch), "mismatch");
  EXPECT_STREQ(payload_status_name(PayloadStatus::Error), "error");
}

TEST(PayloadVerifier, DecideDeratesOnlyOnMismatch) {
  VerifyResult r;
  r.status = PayloadStatus::Mismatch;
  DerateDecision d = decide(r, 0.5);
  EXPECT_TRUE(d.derate);
  EXPECT_DOUBLE_EQ(d.factor, 0.5);
  EXPECT_FALSE(d.action.empty());

  r.status = PayloadStatus::Ok;
  d = decide(r, 0.5);
  EXPECT_FALSE(d.derate);

  r.status = PayloadStatus::NoProfile;
  d = decide(r, 0.5);
  EXPECT_FALSE(d.derate);

  r.status = PayloadStatus::Error;
  d = decide(r, 0.5);
  EXPECT_FALSE(d.derate);  // unknown cause: no automatic action
}
