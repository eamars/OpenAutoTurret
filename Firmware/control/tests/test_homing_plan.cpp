// Unit tests for the executable multi-axis homing plan (architecture §25). The
// plan is a transport-agnostic sequencer; it is driven against a simulated
// two-stop axis plant (no CAN, no trajectory generator — §54). Covers:
//   - home_full_range happy path (both endpoints + logical model),
//   - a two-axis plan (yaw then pitch),
//   - a Move (clearance pose) after homing, and a Move with no reference (fails),
//   - a coarse endpoint establishing a reference,
//   - the §25.2 coarse → move → full-range pattern,
//   - endpoint-pair span validation (invalid span fails the plan),
//   - endpoint failure (no contact) propagating to a plan failure.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"

namespace {

using ota::AxisId;
using ota::DesiredState;
using ota::HomingAction;
using ota::HomingActionType;
using ota::HomingFeedback;
using ota::HomingPlan;
using ota::HomingPlanConfig;
using ota::kDeg2Rad;

constexpr double kDtS = 0.005;  // 200 Hz
constexpr int64_t kDtNs = 5'000'000;

// A simulated axis with physical end-stops at BOTH ends (same model as the
// full-axis homing tests). Pushing into a stop stalls it (v=0, clamped, effort
// rises); moving away releases it. Effort is reported in the travel direction.
struct TwoStopAxis {
  double stop_low = -1.0;
  double stop_high = +1.0;
  double drive_effort = 1.0;
  double contact_effort = 5.0;
  double noise = 0.05;

  double q = 0.0;
  double v = 0.0;
  double torque = 0.0;
  bool at_stop = false;
  int stop_side = 0;

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
    const double alpha = kDtS / (0.05 + kDtS);
    v += alpha * (v_cmd - v);
    if (at_stop) {
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

struct Plant {
  TwoStopAxis pitch;
  TwoStopAxis yaw;
  HomingFeedback feedback(AxisId a, int64_t t) const {
    return (a == AxisId::Pitch) ? pitch.feedback(t) : yaw.feedback(t);
  }
  void step(AxisId a, const DesiredState& ds) {
    if (a == AxisId::Pitch) pitch.step(ds);
    else yaw.step(ds);
  }
};

  // A velocity-mode (SpdRef) executor plant, matching the control loop's homing
  // phase: the axis follows the SIGNED ds.velocity_rad_s (not speed_rad_s toward
  // target_rad). This is the executor the homing plan actually drives, so it is
  // the model that exercises the MoveTo signed-velocity command. Stop handling
  // and the contact/effort model mirror TwoStopAxis so the contact detector
  // (used by full-range homing) still latches.
  struct VelAxis {
    double stop_low = -1.0;
    double stop_high = +1.0;
    double drive_effort = 1.0;
    double contact_effort = 5.0;
    double noise = 0.05;

    double q = 0.0;
    double v = 0.0;
    double torque = 0.0;
    bool at_stop = false;
    int stop_side = 0;

    void reset(double start_q, double low, double high) {
      q = start_q; v = 0.0; torque = 0.0;
      at_stop = false; stop_side = 0;
      stop_low = low; stop_high = high;
    }

    void step(const DesiredState& ds) {
      const double v_cmd = ds.hold ? 0.0 : ds.velocity_rad_s;
      const double alpha = kDtS / (0.05 + kDtS);
      v += alpha * (v_cmd - v);
      if (at_stop) {
        const bool away = (stop_side < 0 && v_cmd > 0) || (stop_side > 0 && v_cmd < 0);
        if (away) { at_stop = false; stop_side = 0; }
        else { q = (stop_side > 0) ? stop_high : stop_low; v = 0.0; }
      }
      if (!at_stop) {
        const double dq = v * kDtS;
        if (dq >= 0 && q + dq >= stop_high) {
          q = stop_high; v = 0.0; at_stop = true; stop_side = +1;
        } else if (dq <= 0 && q + dq <= stop_low) {
          q = stop_low; v = 0.0; at_stop = true; stop_side = -1;
        } else { q += dq; }
      }
      const bool pushing = at_stop &&
                           ((stop_side > 0 && v_cmd > 0) || (stop_side < 0 && v_cmd < 0));
      const int dir_cmd = (v_cmd >= 0) ? 1 : -1;
      const int dir_v = (v >= 0) ? 1 : -1;
      if (pushing) torque = contact_effort * dir_cmd;
      else if (std::fabs(v) > 1e-3) torque = drive_effort * dir_v;
      else torque = 0.0;
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

  struct VelPlant {
    VelAxis pitch;
    VelAxis yaw;
    HomingFeedback feedback(AxisId a, int64_t t) const {
      return (a == AxisId::Pitch) ? pitch.feedback(t) : yaw.feedback(t);
    }
    void step(AxisId a, const DesiredState& ds) {
      if (a == AxisId::Pitch) pitch.step(ds);
      else yaw.step(ds);
    }
  };

  int run_plan_vel(HomingPlan& plan, VelPlant& plant, int max_steps) {
    int64_t t = 0;
    for (int i = 0; i < max_steps; ++i) {
      const AxisId a = plan.active_axis();
      HomingFeedback fb = plant.feedback(a, t);
      DesiredState ds = plan.step(fb);
      plant.step(a, ds);
      t += kDtNs;
      if (plan.complete() || plan.failed()) return i + 1;
    }
    return max_steps;
  }


int run_plan(HomingPlan& plan, Plant& plant, int max_steps) {
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    const AxisId a = plan.active_axis();
    HomingFeedback fb = plant.feedback(a, t);
    DesiredState ds = plan.step(fb);
    plant.step(a, ds);
    t += kDtNs;
    if (plan.complete() || plan.failed()) return i + 1;
  }
  return max_steps;
}

HomingPlanConfig make_cfg() {
  HomingPlanConfig c;
  c.homing.coarse_speed_rad_s = 10.0 * kDeg2Rad;
  c.homing.fine_speed_rad_s = 1.0 * kDeg2Rad;
  c.homing.backoff_speed_rad_s = 10.0 * kDeg2Rad;
  c.homing.backoff_rad = 5.0 * kDeg2Rad;
  c.homing.small_backoff_rad = 2.0 * kDeg2Rad;
  c.homing.repeatability_rad = 0.5 * kDeg2Rad;
  c.homing.settle_time_s = 0.5;
  c.homing.approach_timeout_s = 30.0;
  c.homing.max_travel_rad = 150.0 * kDeg2Rad;
  c.homing.arrival_tol_rad = 0.01;
  // Default stops ±1.0 rad => span 114.59°; band {−60,60} => expected 120°,
  // span bounds [60, 180].
  c.travel_bands[static_cast<int>(AxisId::Pitch)] = {-60.0, 60.0};
  c.travel_bands[static_cast<int>(AxisId::Yaw)] = {-60.0, 60.0};
  c.move_speed_rad_s = 20.0 * kDeg2Rad;
  c.move_pos_tol_rad = 0.01;
  c.move_vel_tol_rad_s = 0.5 * kDeg2Rad;
  c.move_timeout_s = 30.0;
  return c;
}

HomingAction full_range(AxisId a) {
  HomingAction act;
  act.type = HomingActionType::HomeFullRange;
  act.axis = a;
  act.precision = ota::Precision::Fine;
  return act;
}
HomingAction move_to(AxisId a, double deg) {
  HomingAction act;
  act.type = HomingActionType::Move;
  act.axis = a;
  act.position_deg = deg;
  return act;
}
HomingAction endpoint(AxisId a, ota::Endpoint e, ota::Precision p) {
  HomingAction act;
  act.type = HomingActionType::HomeEndpoint;
  act.axis = a;
  act.endpoint = e;
  act.precision = p;
  return act;
}

}  // namespace

TEST(HomingPlan, FullRangeHappyPath) {
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  HomingPlan plan({full_range(AxisId::Pitch)}, make_cfg());

  run_plan(plan, plant, 20000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  EXPECT_FALSE(plan.failed());
  EXPECT_TRUE(plan.axis_homed(AxisId::Pitch));
  EXPECT_NEAR(plan.raw_low(AxisId::Pitch), -1.0, 0.02);
  EXPECT_NEAR(plan.raw_high(AxisId::Pitch), +1.0, 0.02);
  const ota::AxisLogicalModel& m = plan.model(AxisId::Pitch);
  EXPECT_TRUE(m.has_reference);
  EXPECT_EQ(m.direction_sign, +1);
  EXPECT_NEAR(m.q_raw_reference_rad, -1.0, 0.02);
  EXPECT_NEAR(m.q_reference_logical_deg, 0.0, 1e-9);
  SUCCEED();
}

TEST(HomingPlan, TwoAxisPlanYawThenPitch) {
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  HomingPlan plan({full_range(AxisId::Yaw), full_range(AxisId::Pitch)}, make_cfg());

  run_plan(plan, plant, 40000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  EXPECT_TRUE(plan.axis_homed(AxisId::Yaw));
  EXPECT_TRUE(plan.axis_homed(AxisId::Pitch));
  EXPECT_NEAR(plan.raw_low(AxisId::Yaw), -1.0, 0.02);
  EXPECT_NEAR(plan.raw_high(AxisId::Yaw), +1.0, 0.02);
  EXPECT_NEAR(plan.raw_low(AxisId::Pitch), -1.0, 0.02);
  EXPECT_NEAR(plan.raw_high(AxisId::Pitch), +1.0, 0.02);
  SUCCEED();
}

TEST(HomingPlan, MoveAfterHoming) {
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  // Home pitch (low=-1.0 => logical 0), then move to logical +60°.
  HomingPlan plan({full_range(AxisId::Pitch), move_to(AxisId::Pitch, 60.0)}, make_cfg());

  run_plan(plan, plant, 30000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  // logical 60° => raw = -1.0 + 60°(rad) = -1.0 + 1.0472 = 0.0472.
  EXPECT_NEAR(plant.pitch.q, -1.0 + 60.0 * kDeg2Rad, 0.02);
  SUCCEED();
}

TEST(HomingPlan, MoveWithoutHomingFails) {
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  // A Move before any reference cannot resolve a logical target: fails at
  // construction (start_action), before any motion.
  HomingPlan plan({move_to(AxisId::Pitch, 60.0)}, make_cfg());

  EXPECT_TRUE(plan.failed());
  EXPECT_NE(plan.fail_reason().find("not referenced"), std::string::npos)
      << plan.fail_reason();
  SUCCEED();
}

TEST(HomingPlan, CoarseEndpointEstablishesReference) {
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  HomingPlan plan({endpoint(AxisId::Pitch, ota::Endpoint::Lower, ota::Precision::Coarse)},
                  make_cfg());

  run_plan(plan, plant, 20000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  // Only one endpoint known: referenced, but not fully homed.
  EXPECT_FALSE(plan.axis_homed(AxisId::Pitch));
  EXPECT_NEAR(plan.raw_low(AxisId::Pitch), -1.0, 0.02);
  EXPECT_TRUE(plan.model(AxisId::Pitch).has_reference);
  SUCCEED();
}

TEST(HomingPlan, CoarseMoveFullRangePattern) {
  // The §25.2 pattern: coarse-home one stop (a reference), move to a clearance
  // pose, then full-home both endpoints.
  Plant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  HomingPlan plan(
      {endpoint(AxisId::Pitch, ota::Endpoint::Lower, ota::Precision::Coarse),
       move_to(AxisId::Pitch, 30.0),
       full_range(AxisId::Pitch)},
      make_cfg());

  run_plan(plan, plant, 40000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  EXPECT_TRUE(plan.axis_homed(AxisId::Pitch));
  EXPECT_NEAR(plan.raw_low(AxisId::Pitch), -1.0, 0.02);
  EXPECT_NEAR(plan.raw_high(AxisId::Pitch), +1.0, 0.02);
  SUCCEED();
}

TEST(HomingPlan, EndpointPairSpanInvalidFails) {
  // Both endpoints home fine, but they are only 0.2 rad (11.5°) apart, far
  // below the expected band, so the plan must fail on span validation.
  Plant plant;
  plant.pitch.reset(-0.4, -0.5, -0.3);  // start between the two close stops
  HomingPlan plan(
      {endpoint(AxisId::Pitch, ota::Endpoint::Lower, ota::Precision::Fine),
       endpoint(AxisId::Pitch, ota::Endpoint::Upper, ota::Precision::Fine)},
      make_cfg());

  run_plan(plan, plant, 40000);
  EXPECT_TRUE(plan.failed()) << "expected span validation failure";
  EXPECT_NE(plan.fail_reason().find("outside expected"), std::string::npos)
      << plan.fail_reason();
  SUCCEED();
}

TEST(HomingPlan, EndpointFailurePropagates) {
  // No reachable lower stop: the coarse approach runs to timeout, and the
  // failure propagates to the whole plan.
  Plant plant;
  plant.pitch.reset(0.0, -10.0, +1.0);  // lower stop far away -> no contact
  HomingPlanConfig c = make_cfg();
  c.homing.approach_timeout_s = 2.0;  // fail fast
  HomingPlan plan({endpoint(AxisId::Pitch, ota::Endpoint::Lower, ota::Precision::Coarse)},
                  c);

  run_plan(plan, plant, 5000);
  EXPECT_TRUE(plan.failed()) << "expected endpoint failure";
  EXPECT_NE(plan.fail_reason().find("timeout"), std::string::npos)
      << plan.fail_reason();
  SUCCEED();
}

TEST(HomingPlan, MoveToSignedVelocityVelMode) {
  // Regression: the velocity-mode executor (the homing phase drives via SpdRef)
  // commands ds.velocity_rad_s. MoveTo must emit a SIGNED velocity (a magnitude-
  // only / zero-velocity command would never move the axis and would time out).
  // Here a full-range home (velocity-mode) establishes the reference, then a
  // move to logical +60° must actually drive the axis to the target.
  VelPlant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  HomingPlan plan({full_range(AxisId::Pitch), move_to(AxisId::Pitch, 60.0)}, make_cfg());

  run_plan_vel(plan, plant, 30000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  // logical 60° => raw = -1.0 + 60°(rad) = -1.0 + 1.0472 = +0.0472.
  EXPECT_NEAR(plant.pitch.q, -1.0 + 60.0 * kDeg2Rad, 0.02);
  SUCCEED();
}

TEST(HomingPlan, SafeParkSequenceVelMode) {
  // Regression for the user's safe-park requirement: home pitch -> park pitch
  // mid-travel -> home yaw -> park yaw mid-travel -> re-home pitch. The pitch
  // re-home is LAST because the yaw pose sets the pitch's available range
  // ("nothing dangles causing reduced range"). The move actions run in
  // velocity mode, so this also exercises the MoveTo signed-velocity command
  // between each home and the next axis's home.
  VelPlant plant;
  plant.pitch.reset(0.0, -1.0, +1.0);
  plant.yaw.reset(0.0, -1.0, +1.0);
  HomingPlan plan(
      {full_range(AxisId::Pitch),
       move_to(AxisId::Pitch, 30.0),   // safe pose (~mid-travel of the sim span)
       full_range(AxisId::Yaw),
       move_to(AxisId::Yaw, 30.0),     // safe pose (~mid-travel of the sim span)
       full_range(AxisId::Pitch)},     // pitch zero is LAST
      make_cfg());

  run_plan_vel(plan, plant, 60000);
  EXPECT_TRUE(plan.complete()) << plan.fail_reason();
  EXPECT_FALSE(plan.failed());
  EXPECT_TRUE(plan.axis_homed(AxisId::Pitch));
  EXPECT_TRUE(plan.axis_homed(AxisId::Yaw));
  SUCCEED();
}

