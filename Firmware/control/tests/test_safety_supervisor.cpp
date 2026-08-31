// Unit tests for the per-cycle safety supervisor (architecture §38 / §39).
#include <gtest/gtest.h>

#include "control/safety_supervisor.hpp"

namespace {

using ota::AxisSafetyInput;
using ota::AxisLimits;
using ota::SafetyAction;
using ota::SafetySupervisor;
using ota::SupervisorInput;
using ota::SupervisorParams;
constexpr double kDeg = ota::kDeg2Rad;

// A nominal, fresh, homed, holding system.
SupervisorInput nominal() {
  SupervisorInput in;
  in.homing_valid = true;
  in.tracking_enabled = false;
  for (int i = 0; i < static_cast<int>(ota::AxisId::Count); ++i) {
    in.axes[i].has_feedback = true;
    in.axes[i].feedback_age_ms = 0;
    in.axes[i].temp_c = 30.0;
    in.axes[i].motor_faults = 0;
    in.axes[i].q_raw_rad = 0.0;
    in.axes[i].v_rad_s = 0.0;
    in.axes[i].limits.set_from_endpoints(-1.0, 1.0, 0.1);
  }
  return in;
}

SafetySupervisor make_supervisor() {
  SupervisorParams p;
  p.feedback_max_age_ms = 100;
  p.deadline_max_us = 2000;
  p.deadline_miss_threshold = 5;
  p.motor_overtemp_c = 75.0;
  return SafetySupervisor(p);
}

}  // namespace

TEST(SafetySupervisor, NominalAllows) {
  const auto sup = make_supervisor();
  EXPECT_EQ(sup.evaluate(nominal()).action, SafetyAction::Allow);
}

TEST(SafetySupervisor, MotorFaultDisables) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.axes[0].motor_faults = 0x0003;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Disable);
}

TEST(SafetySupervisor, OvertempFaultStops) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.axes[1].temp_c = 90.0;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::FaultStop);
}

TEST(SafetySupervisor, StaleFeedbackBrakes) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.axes[0].feedback_age_ms = 500;  // > 100 ms
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Brake);
}

TEST(SafetySupervisor, MissingFeedbackBrakes) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.axes[1].has_feedback = false;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Brake);
}

TEST(SafetySupervisor, StopInfeasibleBrakes) {
  const auto sup = make_supervisor();
  auto in = nominal();
  // Near the upper soft boundary (0.9) moving up fast with valid limits.
  in.axes[0].q_raw_rad = 0.95;
  in.axes[0].v_rad_s = 2.0;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Brake);
}

TEST(SafetySupervisor, TrackingBeforeHomedHolds) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.homing_valid = false;
  in.tracking_enabled = true;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Hold);
}

TEST(SafetySupervisor, TrackingAfterHomedAllows) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.homing_valid = true;
  in.tracking_enabled = true;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Allow);
}

TEST(SafetySupervisor, PreHomingNoTrackingAllows) {
  // Not homed, not tracking, invalid limits: the homing motion itself is
  // allowed (the homing FSM bounds travel); nothing here should block it.
  const auto sup = make_supervisor();
  auto in = nominal();
  in.homing_valid = false;
  in.tracking_enabled = false;
  in.axes[0].limits.valid = false;
  in.axes[1].limits.valid = false;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Allow);
}

TEST(SafetySupervisor, RepeatedDeadlineMissHolds) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.deadline_miss_count = 5;  // == threshold
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Hold);
}

TEST(SafetySupervisor, SingleOverrunDerates) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.cycle_overrun_us = 5000;  // > 2000, but only a single miss
  in.deadline_miss_count = 1;
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Derate);
}

TEST(SafetySupervisor, FaultTakesPriorityOverStale) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.axes[0].motor_faults = 0x0001;
  in.axes[1].feedback_age_ms = 999;
  // Disable (fault) must win over Brake (stale).
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Disable);
}

TEST(SafetySupervisor, StaleTakesPriorityOverTrackingGate) {
  const auto sup = make_supervisor();
  auto in = nominal();
  in.homing_valid = false;
  in.tracking_enabled = true;  // would be Hold
  in.axes[0].feedback_age_ms = 999;  // but stale -> Brake (more urgent)
  EXPECT_EQ(sup.evaluate(in).action, SafetyAction::Brake);
}
