// Unit tests for the sensorless precision homing state machine (architecture
// §22/§23/§26). The HomingController is driven against a simulated axis that
// responds to the DesiredState (target + speed limit) and stalls when it pushes
// into its end-stop — no CAN, no trajectory generator (§54 simulated plant).
#include <gtest/gtest.h>

#include <cmath>

#include "calibration/homing_controller.hpp"

namespace {

using ota::DesiredState;
using ota::HomingController;
using ota::HomingFeedback;
using ota::HomingParams;
using ota::kDeg2Rad;

constexpr double kDtS = 0.005;  // 200 Hz
constexpr int64_t kDtNs = 5'000'000;

// A simulated axis with a physical end-stop. It follows the DesiredState: it
// drives toward the target at the speed limit (first-order velocity response)
// and stalls — velocity to zero, position clamped, effort rising — when it
// pushes into the stop. Moving away from the stop releases it.
struct SimAxis {
  double stop_at = 1.0;        // end-stop position (rad)
  double drive_effort = 1.0;   // N·m while driving
  double contact_effort = 5.0; // N·m while pushing into the stop
  double noise = 0.05;         // rad/s velocity noise amplitude

  double q = 0.0;
  double v = 0.0;
  double torque = 0.0;
  bool at_stop = false;
  int stop_dir = 0;  // travel direction when the stop was reached

  void reset(double start_q, double stop) {
    q = start_q;
    v = 0.0;
    torque = 0.0;
    at_stop = false;
    stop_dir = 0;
    stop_at = stop;
  }

  void step(const DesiredState& ds, int64_t t_ns) {
    const double to_stop = stop_at - q;
    const double v_cmd =
        ds.hold ? 0.0 : ((ds.target_rad >= q ? 1.0 : -1.0) * ds.speed_rad_s);

    // First-order velocity response (~50 ms time constant).
    const double alpha = kDtS / (0.05 + kDtS);
    v += alpha * (v_cmd - v);

    if (at_stop) {
      // At the stop it can only leave by travelling away from it (-stop_dir);
      // pushing into it (or holding) keeps it clamped at the stop.
      const bool into_stop = (stop_dir > 0 && v > 0) || (stop_dir < 0 && v < 0);
      if (into_stop || std::fabs(v) < 1e-6) {
        q = stop_at;
        v = 0.0;
      } else {
        q += v * kDtS;
        if (std::fabs(q - stop_at) > 1e-6) at_stop = false;
      }
    } else {
      // Not at the stop: move, and clamp if this step would push into it.
      const bool toward_stop = (v > 0 && to_stop > 0) || (v < 0 && to_stop < 0);
      if (toward_stop && std::fabs(v * kDtS) >= std::fabs(to_stop)) {
        q = stop_at;
        at_stop = true;
        v = 0.0;
        stop_dir = (v_cmd > 0) ? 1 : -1;
      } else {
        q += v * kDtS;
      }
    }

    // Effort is reported in the direction of travel (the contact detector
    // signs it by the approach direction). High when pushing into the stop,
    // moderate while driving, ~0 while holding.
    const bool pushing = at_stop &&
                         ((stop_dir > 0 && v_cmd > 0) || (stop_dir < 0 && v_cmd < 0));
    const int dir_cmd = (v_cmd >= 0) ? 1 : -1;
    const int dir_v = (v >= 0) ? 1 : -1;
    if (pushing) {
      torque = contact_effort * dir_cmd;
    } else if (std::fabs(v) > 1e-3) {
      torque = drive_effort * dir_v;
    } else {
      torque = 0.0;
    }
  }

  HomingFeedback feedback(int64_t t_ns) const {
    HomingFeedback fb;
    fb.t_ns = t_ns;
    fb.pos_rad = q;
    fb.vel_rad_s = v + std::sin(t_ns / 1e5) * noise;
    fb.torque_nm = torque + std::sin(t_ns / 3e4) * 0.1;
    fb.motor_fault = false;
    return fb;
  }
};

// Run the homing FSM to a terminal state (or max_steps). Returns the step count.
int run_homing(HomingController& hc, SimAxis& axis, int max_steps) {
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    HomingFeedback fb = axis.feedback(t);
    DesiredState ds = hc.step(fb);
    axis.step(ds, t);
    t += kDtNs;
    if (hc.terminal()) return i + 1;
  }
  return max_steps;
}

HomingParams test_params() {
  HomingParams p;
  p.coarse_speed_rad_s = 10.0 * kDeg2Rad;
  p.fine_speed_rad_s = 1.0 * kDeg2Rad;
  p.backoff_speed_rad_s = 10.0 * kDeg2Rad;
  p.backoff_rad = 5.0 * kDeg2Rad;
  p.small_backoff_rad = 2.0 * kDeg2Rad;
  p.repeatability_rad = 0.5 * kDeg2Rad;
  p.settle_time_s = 0.5;
  p.approach_timeout_s = 30.0;
  p.max_travel_rad = 150.0 * kDeg2Rad;
  p.arrival_tol_rad = 0.01;
  return p;
}

}  // namespace

TEST(Homing, CoarseDirectionReachesEndpointAndPasses) {
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.reset(0.5, 1.0);  // start 0.5 rad before the stop, approach in +
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  int steps = run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.fine_samples, 2);
  // The validated fine contact should be at (or a hair into) the stop.
  EXPECT_NEAR(r.fine_contact_rad, 1.0, 0.01);
  // Repeatability within the limit.
  EXPECT_LE(r.repeatability_rad, test_params().repeatability_rad);
  SUCCEED();
}

TEST(Homing, NegativeDirectionReachesEndpointAndPasses) {
  SimAxis axis;
  axis.stop_at = -1.0;
  axis.reset(-0.5, -1.0);  // start 0.5 rad before the stop, approach in -
  HomingController hc(ota::AxisId::Yaw, -1, test_params());

  int steps = run_homing(hc, axis, 6000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Complete)
      << "failed: " << r.fail_reason;
  EXPECT_TRUE(r.valid);
  EXPECT_NEAR(r.fine_contact_rad, -1.0, 0.01);
  SUCCEED();
}

TEST(Homing, NoContactFailsOnTimeout) {
  // The stop is far beyond the travel limit, so the approach never finds
  // contact and must fail (travel limit / timeout), leaving the axis unhomed.
  SimAxis axis;
  axis.stop_at = 100.0;  // never reached
  axis.reset(0.0, 100.0);
  HomingParams p = test_params();
  p.approach_timeout_s = 3.0;  // short timeout so the test is fast
  p.max_travel_rad = 2.0;      // 2 rad travel limit, well short of the stop
  HomingController hc(ota::AxisId::Pitch, +1, p);

  int steps = run_homing(hc, axis, 2000);
  const ota::HomingResult& r = hc.result();
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed)
      << "expected failure, got complete";
  EXPECT_FALSE(r.valid);
  EXPECT_FALSE(r.fail_reason.empty());
  SUCCEED();
}

TEST(Homing, MotorFaultFailsImmediately) {
  SimAxis axis;
  axis.stop_at = 1.0;
  axis.reset(0.5, 1.0);
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  // Drive a few steps, then assert a motor fault on the next feedback.
  int64_t t = 0;
  for (int i = 0; i < 50; ++i) {
    HomingFeedback fb = axis.feedback(t);
    if (i == 49) fb.motor_fault = true;  // fault on the last step
    DesiredState ds = hc.step(fb);
    axis.step(ds, t);
    t += kDtNs;
  }
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed);
  EXPECT_FALSE(hc.result().valid);
  SUCCEED();
}

TEST(Homing, HardAbortOnLargeEffortFails) {
  // The axis reaches a stop that produces a very large effort (above the hard
  // abort level of 9 N·m). The contact detector's hard abort must fire
  // immediately (no dwell), failing the homing instead of recording a contact.
  SimAxis axis;
  axis.stop_at = 0.2;         // reached quickly
  axis.contact_effort = 20.0; // well above effort_hard_abort_nm (9)
  axis.reset(0.0, 0.2);
  HomingController hc(ota::AxisId::Pitch, +1, test_params());

  run_homing(hc, axis, 2000);
  EXPECT_TRUE(hc.state() == ota::AxisHomeState::Failed) << hc.result().fail_reason;
  EXPECT_FALSE(hc.result().valid);
  EXPECT_NE(hc.result().fail_reason.find("hard abort"), std::string::npos);
  SUCCEED();
}
