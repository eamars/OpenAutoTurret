// Unit tests for the full-axis homing sequence (architecture §23). The
// FullAxisHoming wrapper is driven against a simulated axis with an end-stop
// at EACH end: home endpoint A, traverse, home endpoint B, then validate the
// measured travel against the commissioning band and set up the logical model
// (§24). No CAN, no trajectory generator (§54 simulated plant).
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "calibration/full_axis_homing.hpp"

namespace {

using ota::DesiredState;
using ota::FullAxisHoming;
using ota::FullAxisHomingParams;
using ota::HomingFeedback;
using ota::kDeg2Rad;

constexpr double kDtS = 0.005;  // 200 Hz
constexpr int64_t kDtNs = 5'000'000;

// A simulated axis with physical end-stops at BOTH ends. It follows the
// DesiredState (first-order velocity response toward the target at the speed
// limit) and stalls — velocity to zero, position clamped, effort rising —
// when it pushes into either stop. Moving away from a stop releases it.
// Effort is reported in the direction of travel (the contact detector signs
// it by the approach direction).
struct TwoStopAxis {
  double stop_low = -1.0;
  double stop_high = +1.0;
  double drive_effort = 1.0;   // N·m while driving
  double contact_effort = 5.0; // N·m while pushing into a stop
  double noise = 0.05;         // rad/s velocity noise amplitude

  double q = 0.0;
  double v = 0.0;
  double torque = 0.0;
  bool at_stop = false;
  int stop_side = 0;  // +1 high stop, -1 low stop, 0 free

  void reset(double start_q, double low, double high) {
    q = start_q;
    v = 0.0;
    torque = 0.0;
    at_stop = false;
    stop_side = 0;
    stop_low = low;
    stop_high = high;
  }

  void step(const DesiredState& ds) {
    const double v_cmd =
        ds.hold ? 0.0 : ((ds.target_rad > q ? 1.0 : -1.0) * ds.speed_rad_s);

    // First-order velocity response (~50 ms time constant).
    const double alpha = kDtS / (0.05 + kDtS);
    v += alpha * (v_cmd - v);

    if (at_stop) {
      // At a stop it can only leave by travelling away from it; pushing into
      // it (or holding) keeps it clamped.
      const bool away = (stop_side < 0 && v_cmd > 0) || (stop_side > 0 && v_cmd < 0);
      if (away) {
        at_stop = false;
        stop_side = 0;
      } else {
        q = (stop_side > 0) ? stop_high : stop_low;
        v = 0.0;
      }
    }
    if (!at_stop) {
      const double dq = v * kDtS;
      if (dq >= 0 && q + dq >= stop_high) {
        q = stop_high;
        v = 0.0;
        at_stop = true;
        stop_side = +1;
      } else if (dq <= 0 && q + dq <= stop_low) {
        q = stop_low;
        v = 0.0;
        at_stop = true;
        stop_side = -1;
      } else {
        q += dq;
      }
    }

    const bool pushing = at_stop &&
                         ((stop_side > 0 && v_cmd > 0) || (stop_side < 0 && v_cmd < 0));
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

// Run the full-axis sequence to a terminal state (or max_steps). Returns the
// step count.
int run_full_homing(FullAxisHoming& full, TwoStopAxis& axis, int max_steps) {
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    HomingFeedback fb = axis.feedback(t);
    DesiredState ds = full.step(fb);
    axis.step(ds);
    t += kDtNs;
    if (full.terminal()) return i + 1;
  }
  return max_steps;
}

// Endpoint A is the negative stop, endpoint B the positive stop. The true
// span of the default stops (±1.0 rad) is 114.59°, inside the band.
FullAxisHomingParams test_params() {
  FullAxisHomingParams p;
  p.homing.coarse_speed_rad_s = 10.0 * kDeg2Rad;
  p.homing.fine_speed_rad_s = 1.0 * kDeg2Rad;
  p.homing.backoff_speed_rad_s = 10.0 * kDeg2Rad;
  p.homing.backoff_rad = 5.0 * kDeg2Rad;
  p.homing.small_backoff_rad = 2.0 * kDeg2Rad;
  p.homing.repeatability_rad = 0.5 * kDeg2Rad;
  p.homing.settle_time_s = 0.5;
  p.homing.approach_timeout_s = 30.0;
  p.homing.max_travel_rad = 150.0 * kDeg2Rad;
  p.homing.arrival_tol_rad = 0.01;
  p.dir_endpoint_a = -1;
  p.dir_endpoint_b = +1;
  p.expected_travel_min_deg = 100.0;
  p.expected_travel_max_deg = 130.0;
  return p;
}

}  // namespace

TEST(FullAxisHoming, BothEndpointsHomedAndTravelValid) {
  TwoStopAxis axis;
  axis.reset(0.0, -1.0, +1.0);  // start mid-travel, stops at ±1.0 rad
  FullAxisHoming full(ota::AxisId::Pitch, test_params());

  int steps = run_full_homing(full, axis, 20000);
  const ota::FullAxisHomingResult& r = full.result();
  EXPECT_EQ(full.phase(), ota::FullAxisPhase::Complete) << r.fail_reason;
  EXPECT_TRUE(r.complete);
  EXPECT_TRUE(r.valid) << r.fail_reason;

  // Both validated endpoints at their stops.
  EXPECT_NEAR(r.endpoint_a_rad, -1.0, 0.02);
  EXPECT_NEAR(r.endpoint_b_rad, +1.0, 0.02);

  // Measured travel = span of the stops, inside the commissioning band.
  const double expected_travel_deg = 2.0 * 180.0 / M_PI;  // 114.59°
  EXPECT_NEAR(r.measured_travel_deg, expected_travel_deg, 1.0);
  EXPECT_GE(r.measured_travel_deg, 100.0);
  EXPECT_LE(r.measured_travel_deg, 130.0);
  EXPECT_LE(r.repeatability_rad, test_params().homing.repeatability_rad);

  // Logical model (§24): low endpoint = logical 0, travel positive.
  const ota::AxisLogicalModel& m = r.model;
  EXPECT_TRUE(m.has_reference);
  EXPECT_EQ(m.direction_sign, +1);
  EXPECT_NEAR(m.q_raw_reference_rad, -1.0, 0.02);
  EXPECT_NEAR(m.q_reference_logical_deg, 0.0, 1e-9);
  EXPECT_NEAR(m.raw_to_logical_deg(r.endpoint_a_rad), 0.0, 0.5);
  EXPECT_NEAR(m.raw_to_logical_deg(r.endpoint_b_rad), expected_travel_deg, 0.5);
  SUCCEED();
}

TEST(FullAxisHoming, TravelTooShortFails) {
  // Both endpoints home fine, but the span (0.5 rad = 28.6°) is below the
  // commissioning band, so the axis must fail validation (position invalid).
  TwoStopAxis axis;
  axis.reset(0.0, -1.0, -0.5);
  FullAxisHoming full(ota::AxisId::Pitch, test_params());

  run_full_homing(full, axis, 20000);
  const ota::FullAxisHomingResult& r = full.result();
  EXPECT_EQ(full.phase(), ota::FullAxisPhase::Failed) << "expected travel failure";
  EXPECT_TRUE(r.complete);
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.fail_reason.find("outside expected"), std::string::npos)
      << r.fail_reason;

  // The endpoints themselves were homed successfully.
  EXPECT_TRUE(full.home_a().result().valid);
  EXPECT_TRUE(full.home_b().result().valid);
  EXPECT_NEAR(r.endpoint_a_rad, -1.0, 0.02);
  EXPECT_NEAR(r.endpoint_b_rad, -0.5, 0.02);
  EXPECT_NEAR(r.measured_travel_deg, 0.5 * 180.0 / M_PI, 1.0);
  SUCCEED();
}

TEST(FullAxisHoming, TravelTooLongFails) {
  // Both endpoints home fine, but the span (4 rad = 229°) is above the
  // commissioning band. The travel limit is widened so the traverse can
  // actually reach the far stop.
  TwoStopAxis axis;
  axis.reset(0.0, -2.0, +2.0);
  FullAxisHomingParams p = test_params();
  p.homing.max_travel_rad = 280.0 * kDeg2Rad;
  FullAxisHoming full(ota::AxisId::Pitch, p);

  run_full_homing(full, axis, 20000);
  const ota::FullAxisHomingResult& r = full.result();
  EXPECT_EQ(full.phase(), ota::FullAxisPhase::Failed) << "expected travel failure";
  EXPECT_TRUE(r.complete);
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.fail_reason.find("outside expected"), std::string::npos)
      << r.fail_reason;
  EXPECT_TRUE(full.home_a().result().valid);
  EXPECT_TRUE(full.home_b().result().valid);
  EXPECT_NEAR(r.measured_travel_deg, 4.0 * 180.0 / M_PI, 1.0);
  SUCCEED();
}

TEST(FullAxisHoming, EndpointAFaultFailsWholeAxis) {
  // A motor fault while homing endpoint A must fail the WHOLE axis, with the
  // reason attributed to endpoint A.
  TwoStopAxis axis;
  axis.reset(0.0, -1.0, +1.0);
  FullAxisHoming full(ota::AxisId::Pitch, test_params());

  int64_t t = 0;
  for (int i = 0; i < 100; ++i) {
    HomingFeedback fb = axis.feedback(t);
    if (i == 49) fb.motor_fault = true;  // fault during endpoint A's approach
    DesiredState ds = full.step(fb);
    axis.step(ds);
    t += kDtNs;
    if (full.terminal()) break;
  }
  const ota::FullAxisHomingResult& r = full.result();
  EXPECT_EQ(full.phase(), ota::FullAxisPhase::Failed);
  EXPECT_TRUE(r.complete);
  EXPECT_FALSE(r.valid);
  EXPECT_NE(r.fail_reason.find("endpoint A"), std::string::npos) << r.fail_reason;
  EXPECT_NE(r.fail_reason.find("hard abort or motor fault"), std::string::npos)
      << r.fail_reason;
  SUCCEED();
}

TEST(FullAxisHoming, EndpointBTimeoutFailsWholeAxis) {
  // Endpoint B's stop is unreachable, so its approach times out. Endpoint A
  // homed fine, but the whole axis must fail, with the reason attributed to
  // endpoint B.
  TwoStopAxis axis;
  axis.reset(0.0, -1.0, +10.0);  // high stop far away
  FullAxisHomingParams p = test_params();
  // 10 s: long enough for endpoint A's own approaches (coarse ~5.7 s, fine
  // ~5 s) to succeed, short enough to keep the test fast while endpoint B's
  // approach times out.
  p.homing.approach_timeout_s = 10.0;
  FullAxisHoming full(ota::AxisId::Yaw, p);

  run_full_homing(full, axis, 20000);
  const ota::FullAxisHomingResult& r = full.result();
  EXPECT_EQ(full.phase(), ota::FullAxisPhase::Failed) << "expected failure";
  EXPECT_TRUE(r.complete);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(full.home_a().result().valid) << r.fail_reason;
  EXPECT_NE(r.fail_reason.find("endpoint B"), std::string::npos) << r.fail_reason;
  EXPECT_NE(r.fail_reason.find("timeout"), std::string::npos) << r.fail_reason;
  SUCCEED();
}
