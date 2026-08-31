// OpenAutoTurret — per-cycle safety supervisor (architecture §38 / §39).
//
// Runs EVERY control cycle with authority ABOVE the tracking/hold reference
// (§38). It is a pure decision component: it takes a snapshot of the system
// (per-axis state, motor feedback freshness, motor faults, temperature,
// homing validity, timing health) and returns the highest-priority action.
// It owns a SafetyEnvelope (§18) so the stop-feasibility check (Layer 3) is
// consistent with the planner.
//
// Watchdogs implemented here:
//   §39.2 per-axis motor feedback age (stale -> no open-loop, safe stop);
//   §39.3 control-loop deadline (missed deadlines -> derate/hold);
//   §38.1  homing validity (no tracking before homed).
//
// Priority (most severe wins):
//   Disable    — motor hard fault (Layer 4);
//   FaultStop  — motor over-temperature;
//   Brake      — stale/missing feedback, or stop-feasibility violation (L3);
//   Hold       — tracking requested while homing invalid; repeated deadline miss;
//   Derate     — a single cycle deadline overrun;
//   Allow      — otherwise.
#pragma once

#include <array>
#include <string>

#include "common/types.hpp"
#include "control/safety_envelope.hpp"

namespace ota {

// Per-axis snapshot fed to the supervisor each cycle.
struct AxisSafetyInput {
  double q_raw_rad = 0.0;
  double v_rad_s = 0.0;
  bool has_feedback = false;
  int64_t feedback_age_ms = 0;  // age of last motor feedback (0 = fresh)
  double temp_c = 25.0;
  uint16_t motor_faults = 0;    // non-zero = hard fault
  AxisLimits limits;            // valid only after this axis is homed
};

// Whole-system snapshot fed to the supervisor each cycle.
struct SupervisorInput {
  std::array<AxisSafetyInput, kAxisCount> axes{};
  bool homing_valid = false;     // whole system homed (§38.1)
  bool tracking_enabled = false; // reference is currently a tracking reference
  int64_t cycle_overrun_us = 0;  // this cycle's overrun past the deadline (§39.3)
  int deadline_miss_count = 0;   // consecutive/recent missed deadlines (§39.3)
};

struct SupervisorParams {
  int feedback_max_age_ms = 100;
  int deadline_max_us = 2000;
  int deadline_miss_threshold = 5;
  double motor_overtemp_c = 75.0;
  // Braking model for the internal SafetyEnvelope (must match the planner).
  double a_brake_rad_s2 = 60.0 * kDeg2Rad;
  double j_brake_rad_s3 = 300.0 * kDeg2Rad;
  double stop_margin_rad = 0.05;
};

struct SupervisorDecision {
  SafetyAction action = SafetyAction::Allow;
  std::string reason;
};

class SafetySupervisor {
 public:
  SafetySupervisor() = default;
  explicit SafetySupervisor(SupervisorParams p) : p_(p) {
    SafetyEnvelopeParams e;
    e.a_brake_rad_s2 = p_.a_brake_rad_s2;
    e.j_brake_rad_s3 = p_.j_brake_rad_s3;
    e.margin_rad = p_.stop_margin_rad;
    env_ = SafetyEnvelope(e);
  }

  SupervisorDecision evaluate(const SupervisorInput& in) const {
    // Layer 4: hard motor fault -> disable.
    for (const auto& ax : in.axes)
      if (ax.motor_faults != 0)
        return {SafetyAction::Disable, "motor fault (faults=0x" +
                                          std::to_string(ax.motor_faults) + ")"};
    // Over-temperature -> fault stop.
    for (const auto& ax : in.axes)
      if (ax.temp_c > p_.motor_overtemp_c)
        return {SafetyAction::FaultStop, "motor over-temperature"};
    // §39.2 stale/missing feedback -> safe stop, never open-loop.
    for (const auto& ax : in.axes)
      if (!ax.has_feedback || ax.feedback_age_ms > p_.feedback_max_age_ms)
        return {SafetyAction::Brake, "stale or missing motor feedback"};
    // Layer 3: stop-feasibility violation -> brake.
    for (int i = 0; i < kAxisCount; ++i)
      if (!env_.stop_feasible(in.axes[i].q_raw_rad, in.axes[i].v_rad_s,
                              in.axes[i].limits))
        return {SafetyAction::Brake, "stop infeasible before soft boundary"};
    // §38.1: no tracking while position validity is unknown.
    if (in.tracking_enabled && !in.homing_valid)
      return {SafetyAction::Hold, "tracking blocked: not homed"};
    // §39.3: repeated deadline misses -> hold.
    if (in.deadline_miss_count >= p_.deadline_miss_threshold)
      return {SafetyAction::Hold, "repeated control-loop deadline misses"};
    // §39.3: a single cycle overrun -> derate.
    if (in.cycle_overrun_us > p_.deadline_max_us)
      return {SafetyAction::Derate, "control-loop cycle overrun"};
    return {SafetyAction::Allow, "ok"};
  }

 private:
  SupervisorParams p_;
  SafetyEnvelope env_;
};

}  // namespace ota
