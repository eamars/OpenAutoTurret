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
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <utility>

#include "calibration/homing_plan.hpp"
#include "calibration/installation_pose.hpp"
#include "calibration/park_controller.hpp"
#include "calibration/world_frame_telemetry.hpp"
#include "web/command_validation.hpp"
#include "control/motor_backend.hpp"
#include "control/reference_manager.hpp"
#include "control/safety_envelope.hpp"
#include "control/safety_supervisor.hpp"
#include "control/tracking_controller.hpp"
#include "common/types.hpp"
#include "payload/payload_check.hpp"
#include "payload/payload_profile.hpp"
#include "payload/payload_verifier.hpp"
#include "tracking/target_measurement.hpp"

namespace ota {

enum class Phase {
  Idle,     // no motion phase active (pre-homing or post-park)
  Homing,   // executing the multi-axis homing plan
  Hold,     // ready-hold: at (or moving to) the safe ready pose, position mode
  Parking,  // executing the safe park / shutdown sequence (§33)
  Parked,   // de-energized at the park pose (power-safe)
  Fault,    // fault-locked: controlled stop commanded, no further motion
  // Phase 9: payload response check (§27, §31.3) — small moves in the safe
  // central region, one axis at a time.
  PayloadCheck,
};

inline const char* phase_name(Phase p) {
  switch (p) {
    case Phase::Idle:    return "idle";
    case Phase::Homing:  return "homing";
    case Phase::Hold:    return "hold";
    case Phase::Parking: return "parking";
    case Phase::Parked:  return "parked";
    case Phase::Fault:   return "fault";
    case Phase::PayloadCheck: return "payload_check";
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
    // Phase 9: payload verification (§27, §31.3).
    bool payload_auto_verify = false;  // §27 OPTIONAL_PAYLOAD_RESPONSE_CHECK:
                                       // run once on first hold, at boot
    double payload_check_step_deg = 2.0;   // conservative check amplitude
    double payload_check_speed_deg_s = 5.0;  // conservative check speed
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

  // --- installation orientation (Phase 7, §29/§30) -------------------------
  // Set the active base->world orientation (loaded from the stored pose at
  // boot, or updated by a calibration). Defaults to identity (assumed-level
  // base). Consumed by the telemetry snapshot (world-frame LOS + base tilt).
  void set_base_orientation(const BaseOrientation& o) { base_orientation_ = o; }
  const BaseOrientation& base_orientation() const { return base_orientation_; }

  // --- payload profiling / verification (Phase 9, §28.5, §31) -----------
  // Load the active payload profile (loaded from config/payload_profiles/ at
  // boot, or after re-profiling). Without one, the loop runs with the
  // no_profile status and conservative defaults (§31.3: request a tuning).
  void set_payload_profile(payload::PayloadProfile p);
  payload::PayloadStatus payload_status() const { return payload_status_; }
  bool payload_derated() const { return payload_derated_; }
  bool payload_check_active() const { return phase_ == Phase::PayloadCheck; }
  const std::string& payload_detail() const { return payload_detail_; }
  // Effective speed limit for hold-phase motion (ready pose, test motion) and
  // the tracking v_max: the configured hold speed, capped by the profile
  // v_max when present, scaled by the derate factor while derated
  // (§28.5/§31.3).
  double hold_speed_effective() const;

  // --- telemetry (§6.3, §43) --------------------------------------------
  // The loop fills the §6.3 snapshot EVERY cycle (webd/logd read it at 10-20 Hz
  // from a non-RT thread; guarded by the store's mutex). Tracking fields are
  // populated only while tracking mode is enabled.
  const telemetry::Telemetry& telemetry() const { return telemetry_; }

  // --- developer commands (§42.2) ----------------------------------------
  // Submit a high-level developer command from the web UI. Validates against
  // the authoritative state (web::validate_command) and enqueues it; it is
  // executed on the control thread the next cycle (single-threaded mutation).
  // Returns the validation result (accepted, or the rejection reason). The web
  // server calls this from its non-RT thread.
  web::CommandResult submit_command(const std::string& name,
                                    const std::string& arg);
  // True once the web UI requested a safe shutdown (main() polls this).
  bool shutdown_requested() const { return shutdown_requested_.load(); }
  const web::SystemCommandState& command_state() const {
    std::lock_guard<std::mutex> lk(command_mutex_);
    return command_state_;
  }

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
  // Latest drive-reported motor temperature (degC) per axis (for telemetry;
  // the 1 Hz log and the web snapshot). The drive's NTC via the feedback.
  const std::array<double, kAxisCount>& last_temps() const {
    return last_temp_;
  }
  // Position-derived acceleration (rad/s^2) per axis — the filtered derivative
  // of v_est_ (see kATauS). Exposed so the 1 Hz log shows motion quality
  // (jitter / stuck-slip) at a glance.
  const std::array<double, kAxisCount>& last_accels() const {
    return a_est_;
  }

 private:
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  bool enter_position_mode_all(double limit_spd, std::string& err);
  // Speed mode (velocity) for homing: enter speed mode on every axis with its
  // per-axis drive current limit. The drive's own velocity loop holds the
  // constant approach speed (SpdRef) — the smooth source of motion.
  bool enter_speed_mode_all(const double limit_cur_a[kAxisCount],
                            std::string& err);
  bool finalize_homing();
  // Phase 8: command execution on the control thread (§42.2).
  void process_commands();
  void execute_command(const std::string& name, const std::string& arg);
  void disable_tracking();
  // Phase 9: payload verification (§27, §31.3).
  void start_payload_check(TimeNs now_ns, bool manual);
  void finish_payload_check(TimeNs now_ns);
  void abort_payload_check(TimeNs now_ns, const std::string& reason);
  void apply_payload_derate(bool derated);
  void fault(const std::string& reason) {
    if (phase_ != Phase::Fault) {
      phase_ = Phase::Fault;
      fault_reason_ = reason;
    }
  }
  // vel_rad_s is passed in (the position-derived v_est_) rather than read from
  // the snapshot: the drive's self-reported v is a +/-0.05 rad/s noise band at
  // rest (P0j) that makes the MoveTo arrival test and the contact detector's
  // motion/stall logic flaky.
  static HomingFeedback to_feedback(const AxisSnapshot& s, double vel_rad_s);

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
  // Drive-reported motor temperature (degC) per axis (for the 1 Hz log + web).
  std::array<double, kAxisCount> last_temp_{};
  // Homing high-rate motion-log cycle counter (gates the 100 Hz log; see the
  // Phase::Homing case). Reset in start_homing().
  int homing_log_cycle_ = 0;
  // Position-derived velocity for at-rest decisions (P0j): the drive's
  // self-reported v is a ±0.05 rad/s noise band at rest that chatters the
  // fault phase's |v|>kAtRestVelRadS gate, ping-ponging the emergency-stop
  // reference at 1 Hz. Delta-q estimation (updated only when fresh feedback
  // arrives; low-pass tau=0.1 s) is ~0 at rest and tracks real motion.
  static constexpr double kVestTauS = 0.1;
  // Acceleration / jerk low-pass constants for telemetry capture (C1, A.1).
  // Shorter than kVestTauS so the logged a/j resolve a stick-slip slip while
  // staying above single-frame noise. The signals are a filtered derivative of
  // v_est_ (position-derived), never the drive's self-reported v.
  static constexpr double kATauS = 0.02;
  static constexpr double kJTauS = 0.02;
  std::array<TimeNs, kAxisCount> v_est_t_prev_{};
  std::array<double, kAxisCount> v_est_q_prev_{};
  std::array<double, kAxisCount> v_est_{};
  std::array<double, kAxisCount> a_est_{};
  std::array<double, kAxisCount> jerk_est_{};
  TimeNs deadline_ns_ = 0;
  int deadline_miss_count_ = 0;
  // Phase 6 tracking subsystem (null unless tracking mode is enabled).
  std::unique_ptr<TrackingController> tracking_;
  // Phase 7 installation orientation (base -> world). Identity by default.
  BaseOrientation base_orientation_ = identity_pose();
  // §6.3/§43 top-level telemetry (always filled; webd reads the snapshot).
  telemetry::Telemetry telemetry_;
  // Phase 8: developer-command plumbing (§42.2). command_state_ is published
  // each cycle for web-thread validation; command_queue_ is drained on the
  // control thread. Both guarded by command_mutex_.
  web::SystemCommandState command_state_;
  mutable std::mutex command_mutex_;
  std::deque<std::pair<std::string, std::string>> command_queue_;
  std::atomic<bool> shutdown_requested_{false};
  // One-shot restricted test-motion target (rad), consumed next cycle.
  bool has_test_motion_ = false;
  double test_motion_target_rad_ = 0.0;
  // Phase 9: payload profiling state (§28.5, §31.3, §41).
  std::optional<payload::PayloadProfile> payload_profile_;
  payload::PayloadStatus payload_status_ = payload::PayloadStatus::NoProfile;
  bool payload_derated_ = false;
  std::string payload_detail_;
  payload::PayloadCheckConfig payload_check_cfg_;
  std::array<payload::PayloadCheck, kAxisCount> payload_checks_;
  int payload_axis_ix_ = 0;          // axis currently being checked
  bool payload_check_manual_ = false;  // web-commanded (vs §27 auto)
  bool payload_auto_done_ = false;    // §27: the auto check runs once per boot
  bool payload_check_requested_ = false;  // web command pending (executed at
                                          // the top of the next cycle)
  ReferenceRequest tracking_ref_;  // produced each cycle while tracking is on
  bool has_pending_measurement_ = false;
  vision::TargetMeasurement pending_measurement_;
};

}  // namespace ota
