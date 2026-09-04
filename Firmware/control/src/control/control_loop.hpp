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
#include "mode/mode_manager.hpp"
#include "mode/manual_controller.hpp"
#include "mode/roam_planner.hpp"
#include "tracking/auto_track_controller.hpp"
#include "tracks/target_selection_manager.hpp"
#include "tracks/track_set.hpp"
#include "web/command_validation.hpp"
#include "control/motor_backend.hpp"
#include "control/reference_manager.hpp"
#include "control/reference_limiter.hpp"
#include "control/safety_envelope.hpp"
#include "control/safety_supervisor.hpp"
#include "control/tracking_controller.hpp"
#include "common/types.hpp"
#include "payload/payload_check.hpp"
#include "payload/payload_profile.hpp"
#include "payload/payload_verifier.hpp"
#include "tracking/target_measurement.hpp"

#include "control/aim_deadband.hpp"
#include "control/position_lead.hpp"

namespace ota {

namespace vision {
class VisionLink;  // lock-free vision-link counters (vision_ingest.hpp)
}

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

  // §80. Copies what the loop currently believes into the capture and gives it a new id.
  // Public deliberately: this is the same call the safety edge makes, and a test seam that
  // reaches a *different* function than the one that runs on metal is not a seam, it is a
  // second implementation. Tests call this one.
  void preserve_scene(const telemetry::TelemetrySnapshot& live, const char* reason);
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
    // Drive current limit (A, both axes) applied for the check; see
    // TurretConfig::PayloadConfig::check_current_a.
    double payload_check_current_a = 5.0;
    // Drive inner speed-loop gains (both axes) applied for the check; see
    // TurretConfig::PayloadConfig::check_spd_kp / check_spd_ki. The stock
    // gains (1.0 / 0.002) are too weak to hold the position-mode speed limit
    // against the pitch's gravity load, so the check raises them.
    double payload_check_spd_kp = 5.0;
    double payload_check_spd_ki = 0.02;
    // Per-axis safe-region half-span (§44). The region is centered on each
    // axis's pose at check start and intersected with the homed soft limits,
    // so a check from any homed pose is valid; it must stay >= ~4 deg for the
    // full check amplitude (2 deg step + 2 deg edge margin).
    double payload_check_region_half_span_deg = 10.0;
    // §72: values an operator may name in the config file. Zero / empty keeps today's
    // behaviour exactly — derived region, built-in 300 ms lease, sanctioned step sizes.
    // The distinction is deliberate: an omitted value is not a value.
    bool roam_region_named = false;
    double roam_yaw_min_deg = 0.0;
    double roam_yaw_max_deg = 0.0;
    bool roam_pitch_named = false;
    double roam_pitch_deg = 0.0;
    double roam_velocity_deg_s = 0.0;  // 0 = derive
    int auto_track_coast_ms = 0;
    int auto_track_lost_hold_ms = 0;
    int auto_track_reacquire_window_ms = 0;
    float auto_track_medium_min = 0.0f;
    float auto_track_high_min = 0.0f;
    float auto_track_medium_scale = 0.0f;
    float auto_track_low_scale = 0.0f;
    float reacquire_threshold = 0.0f;
    float ambiguous_match_margin = 0.0f;
    int manual_lease_ms = 0;
    int manual_keepalive_ms = 0;
    std::vector<double> step_sizes_deg;  // empty = the sanctioned three
    // §49 search sweep half-span (rad) around the ready pose. Clamped strictly
    // inside the homed soft limits when tracking is enabled (the SearchPlanner
    // requires its bounds to leave braking margin, §36).
    double search_span_rad = 45.0 * kDeg2Rad;
    // Drive-mode item 3: hold the aim while the line-of-sight wobbles inside this band, so detector jitter does
    // not walk the pointing. ZERO (the default) means the aim passes straight through, exactly as before this
    // key existed; release must exceed enter or it is clamped (see aim_deadband.hpp). Measured floor on this
    // station is 0.02177 deg, so anything at or below that is indistinguishable from quantisation.
    // Drive-mode item 4: the AUTO_TRACK <-> AUTO_ROAM hand-off. BOTH default to 0, which means the station
    // switches modes only when the operator tells it to - exactly today's behaviour. A nonzero value is the
    // delay the mode must have been UNSATISFIED for before the hand-off happens, measured from the moment the
    // condition first became true, so a flickering detection cannot drive the switch. Anti-hunting is NOT a
    // new number: after any automatic switch no further one is allowed for one full reacquire window
    // (auto_track_reacquire_window_ms, 3000 by default), an already-tuned figure rather than a fresh guess.
    int64_t auto_roam_on_loss_ms = 0;        // AUTO_TRACK -> AUTO_ROAM after this long continuously Lost
    int64_t auto_track_on_acquire_ms = 0;    // AUTO_ROAM -> AUTO_TRACK after this long with a target held
    // Drive-mode item 2: how many seconds ahead of the reference to ask the drive for, while the reference is
    // moving. 0 = off (default): the commanded position is the reference, exactly as before this key. The
    // measured shortfall it exists to reduce is p50 3.628 deg at 10 deg/s (PROGRESS rounds 18-19); a lead of
    // 0.173-0.36 s is the range that measurement brackets. See position_lead.hpp for the two properties that
    // keep it from becoming a safety problem: no lead at zero rate, and never past a soft limit.
    double position_lead_s = 0.0;
    double auto_track_deadband_deg = 0.0;
    double auto_track_deadband_release_deg = 0.0;
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
  // Supply the tracking configuration BEFORE homing completes so the
  // `start_tracking` developer command (§42.2) and any auto-enable use the
  // commissioned values (intrinsics, kinematics, speeds, §58 params 19-20)
  // instead of the built-in defaults. `enabled` gates the auto-enable: the
  // loop turns tracking on by itself once the homing gate passes (§38.1).
  void set_tracking_config(const TrackingController::Config& cfg,
                           bool auto_enable) {
    tracking_cfg_ = cfg;
    tracking_auto_enable_ = auto_enable;
  }
  const TrackingController::Config& tracking_config() const {
    return tracking_cfg_;
  }
  // Feed a new target measurement published by visiond. THREAD-SAFE: called
  // from the vision-ingest thread, consumed by the control thread next cycle.
  // Non-blocking. It never dereferences `tracking_` (that pointer is owned by
  // the control thread); a measurement delivered while tracking is off is
  // simply discarded by the consumer.
  void feed_measurement(const vision::TargetMeasurement& m);
  // Control-thread half of the TrackSet path: observe (§8/§21), choose (§12/§13),
  // convert into the v1 measurement the §17 chain consumes. Returns the track that
  // was followed, or nullptr when there was nothing legitimate to follow.
  const tracks::Track* apply_track_set(const tracks::TrackSet& set, TimeNs now);
  // v3 §17/§59: a detector frame's TrackSet arrives instead of a single target. The
  // bottom half of §17's chain — pixel -> ray -> motor interpolation -> LOS -> v1
  // TargetEstimator — is unchanged v1, which is why the hand-off below is still a
  // TargetMeasurement and not a second path beside the one that works.
  void feed_track_set(const tracks::TrackSet& set, TimeNs receive_ns);

  // The selection authority (§13/§14). Control-thread only: the ingest thread hands a
  // received TrackSet across the same mutex the v1 measurement uses, and everything
  // that *decides* — observation, visibility, §21 reacquisition, the operator's
  // command — happens on this thread, against the set that is actually in hand.
  bool tracking_mode_enabled() const { return tracking_ != nullptr; }

  // §36 runtime search opt-in: nullopt = "whatever turret.yaml says", true/false
  // = the operator's explicit word, which outranks the config in both
  // directions. Exposed so a test can assert what a command ACTUALLY changed.
  std::optional<bool> search_override() const { return search_override_; }

  // --- v3: the operating mode (§43) --------------------------------------
  // The mode is the authority on WHO may ask for motion; the v1 layers below it
  // decide whether that motion is safe. Both are asked every cycle, and they are
  // not the same question: AUTO_TRACK may be in charge and still be refused a
  // reference by the envelope in the same cycle.
  OperatingMode operating_mode() const { return mode_mgr_.mode(); }
  SupervisoryState supervisory_state() const { return mode_mgr_.supervisory(); }
  const MotionIntent& last_intent() const { return last_intent_; }

  // Ask for a mode change (web command, or the boot sequence). Never partial:
  // either the mode changes and the controllers follow, or nothing moves and the
  // reason says why (§52).
  ModeResult request_mode(OperatingMode target);
  // §27: cancel the active intent, land in MANUAL/HOLD. Unconditional.
  ModeResult stop_motion();

  // The facts the mode gate is judged on, assembled from authoritative state.
  ModeRequestContext mode_context() const;
  // §32: the region AUTO_ROAM may sweep, derived from the station's own limits rather
  // than typed in, and the safe envelope it must sit inside. `roam_config()` is called
  // every cycle, so a live config edit cannot leave a sweep heading for a waypoint that
  // has just become illegal.
  RoamConfig roam_config() const;
  RoamEnvelope safe_envelope() const;

  // §52: answer every command, including the ones that do nothing. `accepted`
  // here means controld acted on it — not that the web layer delivered it. A
  // command with no honest answer is how v1 ended up with buttons that reported
  // success for work nobody did (enable_search, and still today select_target,
  // start_homing and start_installation_calibration, which are acked and dropped).
  struct CommandAck {
    std::string command;
    bool accepted = false;
    std::string reason;
    std::string controller_state;  // phase + mode at the decision
    std::string safety_state;
    uint64_t seq = 0;
  };
  const CommandAck& last_command_ack() const { return last_ack_; }
  // Access to the tracking controller (telemetry, state). Only call when
  // tracking_mode_enabled().
  const TrackingController& tracking_controller() const { return *tracking_; }

  // --- vision link (Part 2, S1) -------------------------------------------
  // Observe-only: the ingest thread writes the link counters, the control
  // thread reads them into the §6.3 snapshot. The loop never owns it.
  void set_vision_link(const vision::VisionLink* link) { vision_link_ = link; }

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
  // `commissioned` = true for a boot-time load (the operator commissioned it,
  // §28.5: trusted until a verification says otherwise). A RUNTIME selection
  // (`select_payload_profile`, §42.2) is NOT trusted: the status stays
  // no_profile until `start_payload_verification` confirms it (§31.3).
  void set_payload_profile(payload::PayloadProfile p, bool commissioned = true);
  // Where `select_payload_profile` looks profiles up (§41 store directory).
  void set_payload_profile_dir(std::string dir) {
    payload_profile_dir_ = std::move(dir);
  }
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
  // §20: how old the camera geometry is. main.cpp reads the calibration file's own modification
  // time once at boot and every snapshot then reports the elapsed time. The loop deliberately does
  // not re-stat the file: the geometry in force is the one that was loaded, and a file replaced on
  // disk only takes effect after a restart, so ageing the loaded value is the truth while polling
  // the path would report a measurement the station is not using.
  void set_camera_calibration_mtime_ns(int64_t ns) { camera_cal_mtime_ns_ = ns; }

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
  int64_t camera_cal_mtime_ns_ = 0;   // ns since epoch; 0 == geometry age unknown
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
  // v3: make the v1 controllers match a mode that has already been accepted, and
  // build the authoritative cycle intent from whichever mode owns motion (§53).
  void sync_controllers_to_mode(OperatingMode mode);
  void ack_command(const std::string& name, bool accepted,
                   const std::string& why);
  const char* mode_phase_label() const;
  MotionIntent build_mode_intent(TimeNs now_ns) const;
  ReferenceManager::IntentLimits intent_limits(TimeNs now_ns) const;
  // Phase 9: payload verification (§27, §31.3). `sp` holds the current
  // axis snapshots: the per-axis safe region is centered on each axis's
  // current pose (the check starts where the station holds).
  void start_payload_check(TimeNs now_ns, bool manual,
                           const AxisSnapshot sp[kAxisCount]);
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
  // Park: position mode is entered once, when the ParkController leaves the
  // speed-mode move states (MoveYaw/MovePitch) for the §33.2 target-hold
  // (Verify/Dwell/Disable). The blocking enter_position_mode recipe (~100-200
  // ms) would otherwise re-run every cycle. Reset in start_parking().
  bool park_pos_mode_entered_ = false;
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
  // A command that the web thread's gate refused still goes onto the queue,
  // carrying the refusal, so the control thread publishes it as the answer.
  // Otherwise a gate rejection leaves the previous command's ack on the
  // operator's screen — which reads as "your last command worked" when the last
  // one never reached controld at all. The ack has to be written by the control
  // thread: command_state_ and telemetry_ belong to it, and the web thread
  // writing them would be a data race dressed up as a convenience.
  struct PendingCommand {
    std::string name;
    std::string arg;
    std::string gate_reject;  // empty = run it; non-empty = answer with this
  };
  std::deque<PendingCommand> command_queue_;
  std::atomic<bool> shutdown_requested_{false};
  // One-shot restricted test-motion target (rad), consumed next cycle.
  bool has_test_motion_ = false;
  double test_motion_target_rad_ = 0.0;
  // Phase 9: payload profiling state (§28.5, §31.3, §41).
  std::optional<payload::PayloadProfile> payload_profile_;
  // §41: where `select_payload_profile` loads from (config payload.profile_dir).
  std::string payload_profile_dir_;
  payload::PayloadStatus payload_status_ = payload::PayloadStatus::NoProfile;
  bool payload_derated_ = false;
  std::string payload_detail_;
  payload::PayloadCheckConfig payload_check_cfg_;
  std::array<payload::PayloadCheck, kAxisCount> payload_checks_;
  int payload_axis_ix_ = 0;          // axis currently being checked
  // FIXED hold targets for the payload check: every axis is held AT its
  // check-start position (a fixed target, NOT a re-pin to the live position)
  // with a NONZERO speed limit, so the CyberGear position loop actively holds
  // it. At LimitSpd=0 the position loop is pinned and the inactive axis
  // free-drifts under gravity/friction stick-slip (P6 live Run A: yaw +3.6
  // deg, incl. a 2.7 deg/s burst, while the pitch axis was checked). Holding
  // a fixed target with a nonzero limit is the proven park Verify pattern
  // (§33.2, control_loop.cpp).
  std::array<double, kAxisCount> payload_hold_target_{};
  bool payload_check_manual_ = false;  // web-commanded (vs §27 auto)
  bool payload_auto_done_ = false;    // §27: the auto check runs once per boot
  bool payload_check_requested_ = false;  // web command pending (executed at
                                          // the top of the next cycle)
  ReferenceRequest tracking_ref_;  // produced each cycle while tracking is on
  // Latest visiond measurement, handed over from the ingest thread. Guarded by
  // measurement_mutex_ (held for a 58-byte copy only; never contended long).
  std::mutex measurement_mutex_;
  bool has_pending_measurement_ = false;
  vision::TargetMeasurement pending_measurement_;
  tracks::TrackSet pending_set_{};
  bool has_pending_set_ = false;
  // Commissioned tracking configuration (from turret.yaml + the calibration
  // files). Used by the `start_tracking` command and the auto-enable path.
  TrackingController::Config tracking_cfg_;
  // Last published reference, to derive its rate and acceleration (see telemetry fields).
  double ref_prev_q_yaw_ = 0.0;
  double ref_prev_q_pitch_ = 0.0;
  double ref_rate_yaw_ = 0.0;
  double ref_rate_pitch_ = 0.0;
  double ref_prev_rate_yaw_ = 0.0;
  double ref_prev_rate_pitch_ = 0.0;
  double ref_accel_yaw_ = 0.0;
  double ref_accel_pitch_ = 0.0;
  int64_t ref_prev_ns_ = 0;
  bool ref_rate_init_ = false;
  // One profile per axis, so yaw and pitch are shaped independently (a target moving diagonally
  // gives them different amounts to do, and a shared profile would make the slower axis dictate).
  ota::control::ReferenceLimiter ref_lim_[kAxisCount];
  // True on a cycle that published a shaped reference, so the first cycle of an engagement re-seats
  // the profile at the pose the hardware is actually in instead of at wherever the last one ended.
  bool ref_lim_engaged_ = false;
  unsigned tracking_log_cycle_ = 0;
  std::optional<bool> search_override_;  // enable_search / disable_search
  // v3 §53: converts the authoritative mode's intent into a joint reference.
  // Built from the commissioned kinematics when a tracking session is configured
  // (that is where the calibrated kinematics live today). When V3-5 gives MANUAL
  // its own controller this must move onto the loop's own config: a mode that can
  // only convert intents while a tracking session happens to exist is a
  // dependency that will bite the first time someone jogs with tracking off.
  std::optional<ReferenceManager> ref_mgr_;
  ModeManager mode_mgr_;             // v3 §43: which mode owns motion
  MotionIntent last_intent_;         // what it asked for this cycle
  CommandAck last_ack_;              // §52: the answer to the last command
  uint64_t ack_seq_ = 0;
  TimeNs now_ns_ = 0;              // this cycle's clock, for command timestamps
  uint64_t selected_track_id_ = 0;
  TimeNs last_set_receive_ns_ = 0;   // §78: the age of the published candidate list   // low half of the UUID being followed (§78)
  tracks::TargetSelectionManager selection_;
  // §15-§20: AUTO_TRACK's own state machine, and the facts it was last fed. The input
  // is refreshed whenever a TrackSet arrives (camera rate) and read every control cycle,
  // which is the honest shape of it: the state machine runs at 200 Hz because the
  // coast timer does, but nothing about the target is *new* between frames.
  // §29-§36. AUTO_ROAM's own planner: it owns the sweep, the envelope it is bounded by,
  // and the direction it is going. It is consulted only while the mode says so (§53).
  // ModeResult::reason is a const char*, so a refusal built at runtime needs a place to
  // live longer than the statement that formed it. Fixed buffer, no allocation, stable
  // address: every caller copies it immediately (ack_command does), and the only writer
  // is the control thread inside request_mode.
  // Where a mode's "hold" means *here*, latched once on the transition into holding.
  // Latched rather than tracked: if the hold target followed the measured position every
  // cycle, the turret would creep with encoder noise, which is a slow version of the same
  // mistake. Latched rather than read from the last reference, because disable_tracking()
  // zeroes that — and a hold pose of (0,0) is a drive to joint zero, which is what this
  // was briefly doing before a STOP MOTION test caught it.
  // mutable: this is a cache of a decision, and the function that reads it
  // (intent_limits, const, called from the reference conversion) is the only writer.
  // Set once the operating mode has actually asked for motion. Until then "hold" keeps
  // its v1 meaning, because the post-homing sequence — reach the ready pose, run §27's
  // payload verification at rest, then declare READY_HOLD — is defined *at that pose*.
  // Applying "hold where you are" before the station has ever been where it stopped was
  // enough to make verification fail and the safety layer derate a freshly homed station,
  // which is the sort of collateral a safety-relevant default must not have.
  bool mode_has_moved_ = false;
  // §36/§44: authority is ramped in over this many cycles after a handover (300 ms).
  static constexpr int kModeRampCycles = 60;
  int mode_ramp_cycles_ = 0;
  mutable bool mode_hold_in_place_ = false;
  mutable AimDeadband aim_hold_;   // AUTO_TRACK aim hysteresis; reset when the session or mode changes
  mutable bool snap_deadband_clamped_ = false;  // operator gave release < enter; stated, not hidden
  // Drive-mode item 4. `loss_since_ns_` is when the condition FIRST became true, not the last time it was
  // seen true - a timer that refreshes on every sighting can never expire, which is the classic way this
  // feature silently stops working.
  int64_t loss_since_ns_ = 0;
  int64_t acquire_since_ns_ = 0;
  int64_t last_auto_switch_ns_ = 0;
  // Reference-rate estimate for the position lead (drive-mode item 2). One filter state per axis.
  double lead_rate_[2] = {0.0, 0.0};
  double lead_prev_q_[2] = {0.0, 0.0};
  int64_t lead_prev_ns_[2] = {0, 0};
  void evaluate_auto_switch(TimeNs now_ns);
  mutable bool mode_hold_latched_ = false;
  mutable double mode_hold_yaw_rad_ = 0.0;
  mutable double mode_hold_pitch_rad_ = 0.0;
  char mode_refusal_reason_[224] = {};
  RoamPlanner roam_;
  RoamOutput roam_out_;
  // §80: the preserved scene, held so it can be published until someone takes it. Kept
  // here rather than only in the snapshot because each cycle fills a fresh snapshot.
  telemetry::BlackBoxCapture blackbox_{};
  bool was_unsafe_ = false;
  bool manual_cfg_applied_ = false;
  bool v3_cfg_applied_ = false;

  // §79's transition memory: events fire on changes, not on every cycle.
  AutoTrackState last_at_state_ = AutoTrackState::WaitTarget;
  int last_roam_dir_ = 0;
  bool last_roam_turnaround_ = false;
  bool last_roam_active_ = false;
  bool last_at_ambiguous_ = false;
  uint64_t last_event_push_count_ = 0;
  int last_event_tail_count_ = 0;
  // The window itself, kept here rather than only in the snapshot. Each cycle fills a
  // fresh snapshot, so a count without a copy behind it publishes "there are 8 events"
  // next to eight blank rows — which a reader cannot tell apart from an empty station.
  // 128 bytes times 8 copied per cycle is the price of a feed that does not lie; the
  // alternative (rebuilding from the ring every cycle) costs the same and allocates.
  std::array<telemetry::TelemetrySnapshot::EventTail,
             telemetry::TelemetrySnapshot::kEventTail>
      last_event_tail_{};

  // One call per decision, at the place the decision was made. §43.3's ring is the
  // history and the log line is a rendering of the same thing, so an event cannot be
  // "logged" in one place and "structured" in another and drift from it. The subject is
  // folded into the detail because that is what the record can carry; §80's replay gets
  // the full picture from the black box beside it.
  // §72: the configured lease, applied the first time manual motion is asked for rather
  // than in a constructor that may run before the configuration does. Idempotent, so it
  // can sit on the path without becoming a second source of truth.
  void ensure_manual_cfg();

  // §72: the auto-track and reacquisition numbers, applied on the first cycle.
  void apply_v3_config_once();

  void emit(telemetry::Event e, TimeNs now_ns, uint64_t subject_id = 0,
            const char* subject = nullptr, const char* detail = nullptr) {
    std::string d;
    if (subject != nullptr && subject[0] != '\0') d += subject;
    if (subject_id != 0) {
      char id[24];
      std::snprintf(id, sizeof id, "%s#%llu", d.empty() ? "track " : " / track ",
                    static_cast<unsigned long long>(subject_id));
      d += id;
    }
    if (detail != nullptr && detail[0] != '\0') {
      if (!d.empty()) d += " | ";
      d += detail;
    }
    telemetry_.push_event(now_ns, e, std::move(d));
  }

  ManualController manual_;
  ManualOutput manual_out_;
  AutoTrackController autotrack_;
  AutoTrackInput at_input_;
  AutoTrackOutput at_out_;
  TimeNs last_measurement_ns_ = 0;
  bool at_was_visible_ = false;      // for §21's just-reacquired edge

  // Handed across measurement_mutex_ by the ingest thread (§61). Copied rather than
  // referenced: 2.6 KB at camera rate is nothing, and the alternative is the control
  // thread reading a buffer a socket thread may still be writing to while it decides
  // which human being the turret is allowed to follow.
  std::string ack_in_flight_;        // command being executed right now
  ReferenceRequest mode_proposal_;   // the controller's proposal, before the mode
  bool tracking_auto_enable_ = false;
  // Observe-only view of the vision transport (owned by main / VisionIngest).
  const vision::VisionLink* vision_link_ = nullptr;
};

}  // namespace ota
