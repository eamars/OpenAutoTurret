// OpenAutoTurret — sensorless precision homing state machine (architecture
// §22 per-endpoint, §23 full-axis, §26 homing safety).
//
// Drives ONE axis toward ONE end-stop through the recommended sequence:
//
//   APPROACH_COARSE -> (contact) -> SETTLE -> BACK_OFF -> SETTLE ->
//   APPROACH_FINE   -> (contact) -> VERIFY_REPEATABILITY -> COMPLETE / FAILED
//
// The controller is transport-agnostic. Each step() takes the current axis
// feedback and returns a DesiredState (a target position + speed limit, or a
// hold). A move executor — the live controld position-mode recipe driving the
// trajectory generator, or a simulated plant in unit tests — applies the
// DesiredState and feeds the resulting state back on the next step. This keeps
// the FSM unit-testable against a simulated plant (§54) with no CAN involved.
//
// The controller runs the contact detector (§21) internally during approach
// moves and tracks wall-clock time for settles and approach timeouts. A failed
// stage leaves the axis unhomed (§26): the result is invalid and the state is
// Failed; the caller must not treat the axis as referenced.
#pragma once

#include <string>

#include "calibration/contact_detector.hpp"
#include "common/types.hpp"

namespace ota {

// Per-axis homing parameters (§58 params 8-13, all config-driven).
struct HomingParams {
  double coarse_speed_rad_s = 10.0 * kDeg2Rad;  // slow coarse approach
  double fine_speed_rad_s = 1.0 * kDeg2Rad;     // very slow fine approach
  double backoff_speed_rad_s = 10.0 * kDeg2Rad; // speed for the back-off moves
  double backoff_rad = 5.0 * kDeg2Rad;          // back-off after the coarse contact
  double small_backoff_rad = 2.0 * kDeg2Rad;    // back-off for the repeatability pass
  double repeatability_rad = 0.5 * kDeg2Rad;    // max allowed |fine_1 - fine_2|
  double settle_time_s = 0.5;                   // dwell after each stop
  double approach_timeout_s = 30.0;             // max time for one approach move
  double max_travel_rad = 150.0 * kDeg2Rad;     // safety: max travel from the start
  double arrival_tol_rad = 0.01;                // "arrived" position tolerance
  ContactDetectorParams contact;
};

// One sample of axis state, fed to step() each cycle.
struct HomingFeedback {
  TimeNs t_ns = 0;
  double pos_rad = 0.0;
  double vel_rad_s = 0.0;
  double torque_nm = 0.0;
  bool motor_fault = false;
};

// What the move executor should do this cycle.
struct DesiredState {
  double target_rad = 0.0;  // target position (ignored when hold)
  double speed_rad_s = 0.0; // speed limit for the move
  bool hold = false;        // true = hold position, do not move
  std::string message;      // human-readable stage name (for logging)
};

// The outcome of a homing run.
struct HomingResult {
  bool complete = false;             // the FSM ran to a terminal state
  bool valid = false;                // the endpoint is valid (repeatability OK)
  double coarse_contact_rad = 0.0;   // the coarse contact position
  double fine_contact_rad = 0.0;     // the validated fine contact (avg of samples)
  double repeatability_rad = 0.0;    // |fine_contact_1 - fine_contact_2|
  int fine_samples = 0;              // number of fine contact samples
  std::string fail_reason;           // non-empty if !valid
};

class HomingController {
 public:
  // approach_dir is +1 or -1: the direction the axis travels to reach the
  // end-stop (the stop is in this direction from the start).
  HomingController(AxisId axis, int approach_dir, HomingParams p);

  // Drive one FSM step. `fb` is the current axis state; returns the DesiredState
  // to execute this cycle. Safe to call repeatedly; idempotent per stage.
  DesiredState step(const HomingFeedback& fb);

  AxisHomeState state() const { return state_; }
  const HomingResult& result() const { return result_; }
  bool terminal() const {
    return state_ == AxisHomeState::Complete || state_ == AxisHomeState::Failed;
  }

 private:
  enum class PhaseKind { None, Move, Settle };
  struct Phase {
    PhaseKind kind = PhaseKind::None;
    double target_rad = 0.0;
    double speed_rad_s = 0.0;
    bool approach = false;  // move until contact, not to a fixed target
    TimeNs start_ns = 0;
  };

  DesiredState hold_state(const std::string& msg) const;
  DesiredState move_state(double target, double speed, const std::string& msg) const;
  bool arrived(const HomingFeedback& fb) const;
  bool timed_out(const HomingFeedback& fb) const;
  bool settled(const HomingFeedback& fb) const;
  void begin_approach(double speed_rad_s, TimeNs now, double current_pos_rad);
  void begin_backoff_to(TimeNs now, double target_rad);
  void begin_settle(TimeNs now);
  void begin_hold();
  void fail(const std::string& reason);

  AxisId axis_;
  int dir_;
  HomingParams p_;

  AxisHomeState state_ = AxisHomeState::Unknown;
  ContactDetector detector_;
  Phase phase_;

  // VerifyRepeatability sub-phase: 0 = back-off, 1 = second fine approach.
  int verify_phase_ = 0;

  double start_pos_rad_ = 0.0;
  bool has_start_ = false;

  // Recorded contact positions (rad).
  double coarse_contact_rad_ = 0.0;
  double fine_contact1_rad_ = 0.0;
  double fine_contact2_rad_ = 0.0;
  int fine_samples_ = 0;

  HomingResult result_;
};

}  // namespace ota
