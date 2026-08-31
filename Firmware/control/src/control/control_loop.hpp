// OpenAutoTurret — the 200 Hz control engine (architecture §46, §27, §33, §37).
//
// This is the safety-critical per-cycle component. It:
//   * snapshots the axes (non-blocking),
//   * runs the SafetySupervisor EVERY cycle (§38) — its decision has authority
//     ABOVE the phase reference,
//   * computes the reference for the current phase (homing plan / ready-hold /
//     park controller),
//   * applies the safety action (allow / derate / hold / brake / fault-stop /
//     disable) to the motor commands,
//   * commands position mode (fire-and-forget LocRef + LimitSpd).
//
// It depends ONLY on the MotorBackend interface, so the whole safety path is
// unit-testable against a simulated plant with no CAN (§54).
//
// Phase 2 phases (tracking/vision arrive in Phase 3+):
//   Idle -> Homing -> Hold(-> Parked) with Fault reachable from any moving phase.
//
// Steady-state (Hold) does NO slow work. The one-time phase transitions
// (entering position mode) are the only slow paths, and they happen a handful of
// times per boot, never in the steady-state loop (§46).
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include <utility>

#include "calibration/homing_plan.hpp"
#include "calibration/park_controller.hpp"
#include "control/motor_backend.hpp"
#include "control/reference_manager.hpp"
#include "control/safety_envelope.hpp"
#include "control/safety_supervisor.hpp"
#include "control/tracking_controller.hpp"
#include "common/types.hpp"
#include "tracking/target_measurement.hpp"

namespace ota {

enum class Phase {
  Idle,     // no motion phase active (pre-homing or post-park)
  Homing,   // executing the multi-axis homing plan
  Hold,     // ready-hold: at (or moving to) the safe ready pose, position mode
  Parking,  // executing the safe park / shutdown sequence (§33)
  Parked,   // de-energized at the park pose (power-safe)
  Fault,    // fault-locked: controlled stop commanded, no further motion
};

inline const char* phase_name(Phase p) {
  switch (p) {
    case Phase::Idle:    return "idle";
    case Phase::Homing:  return "homing";
    case Phase::Hold:    return "hold";
    case Phase::Parking: return "parking";
    case Phase::Parked:  return "parked";
    case Phase::Fault:   return "fault";
  }
  return "?";
}

class ControlLoop {
 public:
  struct Config {
    int control_hz = 200;
    // Braking model (must match the SafetyEnvelope the supervisor uses).
    double a_brake_rad_s2 = 60.0 * kDeg2Rad;
    double j_brake_rad_s3 = 300.0 * kDeg2Rad;
    double stop_margin_rad = 0.05;
    // Safety supervisor / watchdogs (§38/§39).
    int feedback_max_age_ms = 100;
    int deadline_max_us = 2000;
    int deadline_miss_threshold = 5;
    double motor_overtemp_c = 75.0;
    // Speeds.
    double hold_speed_rad_s = 10.0 * kDeg2Rad;
    double emergency_speed_rad_s = 5.0 * kDeg2Rad;
    double derate_factor = 0.5;
    // Soft-limit margin (rad) used to build AxisLimits from the homed endpoints.
    double soft_margin_rad = 2.0 * kDeg2Rad;
    // §33 park sequence parameters.
    ParkParams park;
  };

  ControlLoop(Config cfg, std::unique_ptr<MotorBackend> backend);

  // --- phase setup (slow; called by the boot FSM / main, not per cycle) ---
  bool start_homing(HomingPlan plan, std::string& err);
  bool start_hold(std::string& err);
  bool start_parking(std::string& err);  // requires homed_ (valid limits/models)
  void deenergize_all();

  // --- tracking (Phase 6, §13-§16) --------------------------------------
  // Enable the tracking mode. Requires a valid homing (position validity
  // known, §38.1). The TrackingController is then owned by the loop and the
  // Hold phase delegates its reference to it (tracking > search > hold, §16).
  bool enable_tracking(const TrackingController::Config& cfg, std::string& err);
  // Feed a new target measurement published by visiond (no-op if tracking is
  // not enabled). Non-blocking; the control loop consumes it next cycle.
  void feed_measurement(const vision::TargetMeasurement& m);
  bool tracking_mode_enabled() const { return tracking_ != nullptr; }
  // Access to the tracking controller (telemetry, state). Only call when
  // tracking_mode_enabled().
  const TrackingController& tracking_controller() const { return *tracking_; }

  // One control cycle. `period_ns` is how long the previous cycle took (drives
  // the §39.3 deadline watchdog). Returns the (possibly updated) phase.
  Phase step(TimeNs now_ns, TimeNs period_ns);

  Phase phase() const { return phase_; }
  bool homed() const { return homed_; }
  bool at_ready() const { return at_ready_; }
  const std::array<AxisLimits, kAxisCount>& limits() const { return limits_; }
  const std::array<AxisLogicalModel, kAxisCount>& models() const { return models_; }
  const std::string& fault_reason() const { return fault_reason_; }
  const SupervisorDecision& last_decision() const { return last_decision_; }
  // Latest raw-axis positions (rad) seen by the loop (for telemetry).
  const std::array<double, kAxisCount>& last_positions() const {
    return last_q_;
  }

 private:
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  bool enter_position_mode_all(double limit_spd, std::string& err);
  bool finalize_homing();
  void fault(const std::string& reason) {
    if (phase_ != Phase::Fault) {
      phase_ = Phase::Fault;
      fault_reason_ = reason;
    }
  }
  static HomingFeedback to_feedback(const AxisSnapshot& s);

  Config cfg_;
  std::unique_ptr<MotorBackend> backend_;
  SafetySupervisor supervisor_;
  SafetyEnvelope env_;
  std::array<AxisLimits, kAxisCount> limits_{};    // valid only after homing
  std::array<AxisLogicalModel, kAxisCount> models_{};
  std::array<double, kAxisCount> ready_raw_{};     // safe ready pose (raw rad)
  Phase phase_ = Phase::Idle;
  bool homed_ = false;
  bool at_ready_ = false;
  std::string fault_reason_;
  SupervisorDecision last_decision_;
  std::unique_ptr<HomingPlan> homing_;
  std::unique_ptr<ParkController> park_;
  std::array<double, kAxisCount> last_q_{};
  TimeNs deadline_ns_ = 0;
  int deadline_miss_count_ = 0;
  // Phase 6 tracking subsystem (null unless tracking mode is enabled).
  std::unique_ptr<TrackingController> tracking_;
  ReferenceRequest tracking_ref_;  // produced each cycle while tracking is on
  bool has_pending_measurement_ = false;
  vision::TargetMeasurement pending_measurement_;
};

}  // namespace ota
