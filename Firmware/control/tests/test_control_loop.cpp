// OpenAutoTurret — ControlLoop tests (§46 per-cycle engine + safety).
//
// Driven against the SimMotorBackend (a first-order plant with end stops and a
// stall effort), so the whole safety path runs with no CAN:
//   * the Phase-2 deliverable: boot -> homed -> safe hold -> park cycle;
//   * stale feedback -> Brake (a recoverable safe stop, NOT a fault);
//   * motor hard fault -> Disable (de-energize, fault-locked).
#include <memory>
#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

#include <gtest/gtest.h>

using namespace ota;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz
constexpr int kMaxSteps = 60000;      // 300 s of sim

// A homing plan that full-homes both axes (measures/validates both endpoints).
HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg2Rad;
  hp.fine_speed_rad_s = 2.0 * kDeg2Rad;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 115.0};  // pitch
  hcfg.travel_bands[1] = TravelBand{0.0, 115.0};  // yaw
  std::vector<HomingAction> actions;
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Pitch});
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Yaw});
  return HomingPlan(std::move(actions), hcfg);
}

ControlLoop::Config make_cfg() {
  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  cfg.park.park_logical_deg = {30.0, 60.0};  // pitch 30 deg, yaw 60 deg
  cfg.park.speed_deg_s = 50.0;
  cfg.park.dwell_ms = 300;
  cfg.park.pos_tol_deg = 0.5;
  cfg.park.vel_tol_deg_s = 1.0;
  cfg.park.min_soft_margin_deg = 2.0;
  return cfg;
}

// Home both axes, then move to the safe ready pose (the §27 MOVE_TO_READY /
// HOLD_POSE transition). Returns the sim time after the ready pose is reached.
bool run_to_ready(ControlLoop& loop, sim::SimMotorBackend& sim, int64_t& t_out) {
  int64_t t = 0;
  for (int i = 0; i < kMaxSteps; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
    if (loop.phase() == Phase::Fault) return false;
    if (loop.homed() && loop.at_ready()) {
      t_out = t;
      return true;
    }
  }
  t_out = t;
  return false;
}

}  // namespace

// The Phase-2 deliverable: reliable boot -> homed -> safe hold -> park cycle.
TEST(ControlLoop, FullCycle_HomeHoldPark) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));

  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;

  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t))
      << "did not reach ready; phase=" << phase_name(loop.phase())
      << " fault=" << loop.fault_reason();
  EXPECT_EQ(loop.phase(), Phase::Hold);
  EXPECT_TRUE(loop.homed());
  // Both axes have valid limits after homing.
  EXPECT_TRUE(loop.limits()[static_cast<int>(AxisId::Pitch)].valid);
  EXPECT_TRUE(loop.limits()[static_cast<int>(AxisId::Yaw)].valid);

  // Park: move to the park pose, verify, dwell, de-energize pitch then yaw.
  ASSERT_TRUE(loop.start_parking(err)) << err;
  for (int i = 0; i < kMaxSteps && loop.phase() != Phase::Parked; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  EXPECT_EQ(loop.phase(), Phase::Parked)
      << "park did not complete; fault=" << loop.fault_reason();
  // Both axes end de-energized (held by friction), at the park pose.
  EXPECT_FALSE(sim->in_position_mode(AxisId::Pitch));
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
}

// Stale feedback -> Brake: a recoverable safe stop, NOT a fault. The axis is
// commanded to stop (not to keep moving) and recovers when feedback returns.
TEST(ControlLoop, StaleFeedbackTriggersBrake) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t)) << loop.fault_reason();
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Allow);

  // Drop the pitch feedback: the supervisor must Brake (safe stop), and this is
  // recoverable (not a fault) once the feedback returns.
  sim->set_feedback_ok(AxisId::Pitch, false);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Brake);
  EXPECT_NE(loop.phase(), Phase::Fault) << "stale feedback is not a hard fault";

  // Feedback returns: back to Allow (recovered, no manual reset needed).
  sim->set_feedback_ok(AxisId::Pitch, true);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Allow);
  EXPECT_EQ(loop.phase(), Phase::Hold);
}

// Motor hard fault -> Disable: de-energize the axes and fault-lock (sticky).
TEST(ControlLoop, MotorFaultTriggersDisable) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t)) << loop.fault_reason();

  // A hard fault on yaw: the supervisor must Disable (de-energize) and the
  // loop fault-locks (sticky; it does not auto-recover).
  sim->set_faults(AxisId::Yaw, 0x1);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Disable);
  EXPECT_EQ(loop.phase(), Phase::Fault);
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
  EXPECT_FALSE(sim->in_position_mode(AxisId::Pitch));

  // Faults are sticky: even with the fault cleared, the loop stays in Fault.
  sim->set_faults(AxisId::Yaw, 0x0);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.phase(), Phase::Fault);
}
