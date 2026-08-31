// Unit tests for the safe park / shutdown controller (architecture §33). The
// controller is a transport-agnostic state machine; it is driven against a
// simulated two-stop axis plant (no CAN — §54). Covers:
//   - the full park happy path (both axes to park, verify, dwell, disable, parked)
//     and the exact FSM state order,
//   - the §33.1 rejection of a park pose at a mechanical stop (config default
//     0/0 in the logical frame sits ON the low endpoint),
//   - failure when an axis is not referenced (not homed),
//   - the fast path when the axes are already at their park poses.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "calibration/park_controller.hpp"

namespace {

using ota::AxisId;
using ota::AxisLimits;
using ota::AxisLogicalModel;
using ota::DesiredState;
using ota::HomingFeedback;
using ota::kDeg2Rad;
using ota::kAxisCount;
using ota::ParkController;
using ota::ParkOutput;
using ota::ParkParams;
using ota::ParkState;

constexpr double kDtS = 0.005;  // 200 Hz
constexpr int64_t kDtNs = 5'000'000;

// A simulated axis with physical end-stops at BOTH ends (same model as the
// homing tests). Pushing into a stop stalls it (v=0, clamped); moving away
// releases it.
struct TwoStopAxis {
  double stop_low = -1.0;
  double stop_high = +1.0;
  double drive_effort = 1.0;
  double contact_effort = 5.0;
  // Velocity feedback noise kept small (0.3°/s) and BELOW the park verify
  // tolerance (1°/s), so the §33.2 dwell can settle. (The homing tests use a
  // larger noise because they exercise effort-based contact detection.)
  double noise = 0.005;

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
};

// Stops at ±1.0 rad (raw). Reference: raw -1.0 == logical 0°, direction +1, so
// logical = (raw + 1.0) * kRad2Deg in [0, 114.59°].
AxisLogicalModel make_model() {
  AxisLogicalModel m;
  m.direction_sign = 1;
  m.set_reference(-1.0, 0.0);
  return m;
}

// Hard limits = the stops; soft = hard ± 0.1 rad => soft [-0.9, 0.9].
AxisLimits make_limits() {
  AxisLimits lim;
  lim.set_from_endpoints(-1.0, 1.0, 0.1);
  return lim;
}

ParkParams make_params() {
  ParkParams p;
  p.park_logical_deg[static_cast<int>(AxisId::Pitch)] = 30.0;  // raw -0.4764
  p.park_logical_deg[static_cast<int>(AxisId::Yaw)] = 60.0;    // raw +0.0472
  p.pos_tol_deg = 0.5;
  p.vel_tol_deg_s = 1.0;
  p.dwell_ms = 500;
  p.speed_deg_s = 50.0;
  p.min_soft_margin_deg = 2.0;
  p.move_pos_tol_rad = 0.005;  // tighter than the 0.5° verify tolerance
  p.move_vel_tol_rad_s = 0.5 * kDeg2Rad;
  p.move_timeout_s = 30.0;
  return p;
}

struct ParkRunResult {
  int steps = 0;
  bool complete = false;
  bool failed = false;
  bool saw_disable_pitch = false;
  bool saw_disable_yaw = false;
  std::vector<ParkState> states;
  double pitch_q = 0.0;
  double yaw_q = 0.0;
  std::string fail_reason;
};

ParkRunResult run_park(ParkController& park, Plant& plant, int max_steps) {
  ParkRunResult r;
  int64_t t = 0;
  for (int i = 0; i < max_steps; ++i) {
    r.states.push_back(park.state());  // the state processed this cycle
    HomingFeedback pfb = plant.pitch.feedback(t);
    HomingFeedback yfb = plant.yaw.feedback(t);
    ParkOutput out = park.step(pfb, yfb);
    plant.pitch.step(out.pitch);
    plant.yaw.step(out.yaw);
    t += kDtNs;
    if (out.disable_pitch) r.saw_disable_pitch = true;
    if (out.disable_yaw) r.saw_disable_yaw = true;
    if (out.complete || out.failed) {
      r.complete = out.complete;
      r.failed = out.failed;
      r.fail_reason = out.message;
      r.steps = i + 1;
      break;
    }
  }
  r.pitch_q = plant.pitch.q;
  r.yaw_q = plant.yaw.q;
  return r;
}

std::vector<ParkState> unique_states(const std::vector<ParkState>& s) {
  std::vector<ParkState> u;
  for (ParkState st : s) {
    if (u.empty() || u.back() != st) u.push_back(st);
  }
  return u;
}

}  // namespace

// Full park: both axes travel to their park poses, are verified settled, dwell,
// and are de-energized one at a time. Checks the final positions AND the exact
// FSM state order.
TEST(ParkController, HappyPath) {
  Plant plant;
  plant.pitch.reset(0.5, -1.0, 1.0);
  plant.yaw.reset(0.5, -1.0, 1.0);

  const ParkParams p = make_params();
  std::array<AxisLimits, kAxisCount> limits = {make_limits(), make_limits()};
  std::array<AxisLogicalModel, kAxisCount> models = {make_model(), make_model()};
  ParkController park(p, limits, models);
  EXPECT_FALSE(park.failed());

  ParkRunResult r = run_park(park, plant, 2000);
  EXPECT_TRUE(r.complete) << r.fail_reason;
  EXPECT_FALSE(r.failed);
  // Both axes at their raw park targets (within tolerance).
  EXPECT_NEAR(r.pitch_q, -0.4764, 0.01);
  EXPECT_NEAR(r.yaw_q, 0.0472, 0.01);
  // Both de-energize requests were issued.
  EXPECT_TRUE(r.saw_disable_pitch);
  EXPECT_TRUE(r.saw_disable_yaw);
  // The exact FSM order from §33.
  const std::vector<ParkState> expected = {
      ParkState::StopTracking, ParkState::MoveYaw, ParkState::MovePitch,
      ParkState::Verify,       ParkState::Dwell,   ParkState::DisablePitch,
      ParkState::DisableYaw,   ParkState::Parked};
  EXPECT_EQ(unique_states(r.states), expected);
}

// §33.1: the park pose must not sit against a mechanical stop. Logical 0° is the
// LOW endpoint (raw -1.0), i.e. the config default 0/0 — this must be rejected.
TEST(ParkController, ParkAtStopRejected) {
  ParkParams p = make_params();
  p.park_logical_deg[static_cast<int>(AxisId::Pitch)] = 0.0;  // raw -1.0 (low stop)
  std::array<AxisLimits, kAxisCount> limits = {make_limits(), make_limits()};
  std::array<AxisLogicalModel, kAxisCount> models = {make_model(), make_model()};
  ParkController park(p, limits, models);
  EXPECT_TRUE(park.failed());
  EXPECT_NE(park.fail_reason().find("soft limit"), std::string::npos)
      << park.fail_reason();
}

// The park pose near the HIGH stop is also rejected (symmetric §33.1 check).
TEST(ParkController, ParkNearHighStopRejected) {
  ParkParams p = make_params();
  p.park_logical_deg[static_cast<int>(AxisId::Yaw)] = 110.0;  // raw +0.9199 > soft+margin
  std::array<AxisLimits, kAxisCount> limits = {make_limits(), make_limits()};
  std::array<AxisLogicalModel, kAxisCount> models = {make_model(), make_model()};
  ParkController park(p, limits, models);
  EXPECT_TRUE(park.failed());
  EXPECT_NE(park.fail_reason().find("soft limit"), std::string::npos)
      << park.fail_reason();
}

// An axis that is not referenced (not homed) cannot be parked.
TEST(ParkController, NotHomedFails) {
  AxisLogicalModel no_ref;  // has_reference == false
  const ParkParams p = make_params();
  std::array<AxisLimits, kAxisCount> limits = {make_limits(), make_limits()};
  std::array<AxisLogicalModel, kAxisCount> models = {no_ref, make_model()};
  ParkController park(p, limits, models);
  EXPECT_TRUE(park.failed());
  EXPECT_NE(park.fail_reason().find("not referenced"), std::string::npos)
      << park.fail_reason();
}

// If the axes already sit at their park poses, the moves arrive immediately and
// the controller still verifies, dwells, and de-energizes.
TEST(ParkController, AlreadyAtPark) {
  Plant plant;
  plant.pitch.reset(-0.4764, -1.0, 1.0);  // pitch park
  plant.yaw.reset(0.0472, -1.0, 1.0);     // yaw park

  const ParkParams p = make_params();
  std::array<AxisLimits, kAxisCount> limits = {make_limits(), make_limits()};
  std::array<AxisLogicalModel, kAxisCount> models = {make_model(), make_model()};
  ParkController park(p, limits, models);
  EXPECT_FALSE(park.failed());

  ParkRunResult r = run_park(park, plant, 1000);
  EXPECT_TRUE(r.complete) << r.fail_reason;
  EXPECT_FALSE(r.failed);
  EXPECT_NEAR(r.pitch_q, -0.4764, 0.01);
  EXPECT_NEAR(r.yaw_q, 0.0472, 0.01);
  EXPECT_TRUE(r.saw_disable_pitch);
  EXPECT_TRUE(r.saw_disable_yaw);
  // Should be quick (no long moves) — dominated by the 500 ms dwell (~100 cycles).
  EXPECT_LT(r.steps, 400);
}
