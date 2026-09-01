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
  double coarse_speed_rad_s = 10.0 * kDeg2Rad;  // coarse approach
  // The CyberGear position loop crawls/stick-slips below ~10 deg/s (tiny spd_ki
  // can't beat static friction at low speed-ref; characterized 2026-09-01:
  // 1 deg/s -> 0.003 deg/s creep, 5 deg/s -> stick-slip bursts). Contact
  // precision comes from the mechanical stop, not the approach speed.
  //
  // 20 deg/s (raised from 10 on 2026-09-01 after p0l): the drive's breakaway
  // is position-dependent — at some yaw positions it stalls within ~1 deg of
  // a 10 deg/s fine approach (false contact, e.g. at raw -1.13) even though
  // the same position is passed freely at 20 deg/s (verified by a manual
  // move -1.14 -> -0.25 that crossed -1.13 smoothly). 20 deg/s gives enough
  // breakaway authority to beat the static-friction stall while still hitting
  // the mechanical stop cleanly (precision comes from the stop, not speed).
  double fine_speed_rad_s = 20.0 * kDeg2Rad;
  double backoff_speed_rad_s = 10.0 * kDeg2Rad; // speed for the back-off moves
  double backoff_rad = 5.0 * kDeg2Rad;          // back-off after the coarse contact
  double small_backoff_rad = 2.0 * kDeg2Rad;    // back-off for the repeatability pass
  double repeatability_rad = 0.5 * kDeg2Rad;    // max allowed |fine_1 - fine_2|
  double settle_time_s = 0.5;                   // dwell after each stop
  double approach_timeout_s = 30.0;             // max time for one approach move
  double max_travel_rad = 150.0 * kDeg2Rad;     // safety: max travel from the start
  double arrival_tol_rad = 0.01;                // "arrived" position tolerance
  // --- Backoff drive: closed-loop target tracking with stall-breaking ---
  // Root cause of the 2026-09-02 yaw homing hang (rehome2): the backoff was a
  // fixed-sign velocity command (10 deg/s). The yaw "end-stop" is a wide
  // (~7 deg) friction/detent zone; the 10 deg/s backoff stick-slipped in it
  // for ~10 s (q crawling at ~1 deg/s while pushing 0.3-0.6 N.m), then burst
  // free with momentum (measured v=+0.56 rad/s vs the +0.1745 command),
  // overshot the backoff target, and the fixed-sign command kept driving AWAY
  // from it — the axis lapped to the far edge of the same zone and the
  // timeout-less Backoff state hung forever. The backoff is now:
  //   (a) target-seeking: v = speed * sign(target - q) (0 near the target);
  //       an overshoot reverses the command instead of driving away;
  //   (b) stall-breaking: when the move stalls in friction for
  //       backoff_stall_detect_s, a short burst at backoff_burst_factor *
  //       speed breaks the static friction (30 deg/s is known to clear the
  //       zone — the 30 deg/s coarse approach does), then it resumes;
  //   (c) timeout: a backoff that cannot reach its target fails cleanly
  //       instead of hanging.
  double backoff_timeout_s = 10.0;               // hard timeout for a backoff move
  double backoff_stall_vel_rad_s = 0.02;         // |v| below this = "not moving"
  double backoff_stall_detect_s = 0.3;           // stalled duration that triggers a burst
  double backoff_burst_factor = 3.0;             // burst speed = factor * backoff speed
  double backoff_burst_s = 0.25;                 // burst duration (s)
  double backoff_arrive_vel_rad_s = 0.1;         // |v| below this to accept arrival
  // Re-arm the drive (de-energize/re-energize, the verified speed-mode
  // recipe) at the start of the coarse approach, before any motion.
  // FullAxisHoming enables this for endpoint B: when endpoint A completes
  // its second fine contact the drive's velocity-loop integral is still
  // wound up pushing the stop, and endpoint B's opposite-direction coarse
  // approach would start by fighting that residual (it crawled through in
  // rehome2, but only because the old code had no timeouts).
  bool rearm_before_start = false;
  ContactDetectorParams contact;
  // --- Adaptive-current homing (push-through, §22) ---
  // The yaw is a ~360 deg axis whose mid-travel friction stalls (false
  // contacts) beat a low LimitCur. The coarse approach does NOT stop at the
  // first contact: on each contact latch it raises the drive current by
  // `limit_cur_step_a` and keeps driving, until either a real (consistent)
  // end-stop is reached or the current hits `limit_cur_max_a`. It also caps
  // the cumulative rotation at `max_rotation_rad` (a full-rotation axis has
  // no end-stop, so >360 deg means "no stop found"). `torque_safety_nm` is
  // a hard |tau| abort so the push cannot overload the mechanical stop.
  // The executor applies `limit_cur_a` via DesiredState when it changes.
  double limit_cur_initial_a = 6.5;             // starting drive current (A)
  double limit_cur_step_a = 1.0;                // raise per contact latch (A)
  double limit_cur_max_a = 10.0;                // safe upper limit (A)
  double max_rotation_rad = 360.0 * kDeg2Rad;   // cumulative-rotation cap
  double torque_safety_nm = 10.0;               // |tau| abort threshold (N.m)
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
  double target_rad = 0.0;  // target position (position-mode executor)
  double speed_rad_s = 0.0; // speed limit for the move (position-mode)
  // Speed-mode (velocity) command: the constant speed (SpdRef) the drive's
  // velocity loop should hold, rad/s, signed (sign = direction). The
  // speed-mode homing executor uses this (not target_rad/speed_rad_s); the
  // position-mode executor ignores it. 0.0 = hold in place.
  double velocity_rad_s = 0.0;
  bool hold = false;        // true = hold position, do not move
  std::string message;      // human-readable stage name (for logging)
  // Drive current limit to apply this cycle (A). 0.0 = leave it unchanged.
  // The executor writes LimitCur on the cycle this is non-zero.
  double limit_cur_a = 0.0;
  // De-energize/re-energize the axis (the verified speed-mode recipe)
  // before executing this cycle's velocity command. Purpose: reset the
  // drive's velocity-loop integral, which winds up while pushing the
  // contact stop (1.5 s dwell + 0.5 s settle) and can then hold ~1+ N.m
  // of torque that NO SpdRef can overcome (rehome3 root cause: the backoff
  // command was on the bus and ignored — the axis stayed pinned to the
  // stop, tq +0.8..+1.4, v jitter only, until the 10 s backoff timeout).
  // The axis bounces slightly off the stop during the de-energize; the
  // target-seeking backoff and the fine re-approach are unaffected.
  bool rearm_speed_mode = false;
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
  // --- Adaptive-current diagnostics (§22) ---
  double peak_torque_nm = 0.0;       // max |tau| observed during the run
  double contact_torque_nm = 0.0;    // |tau| at the validated fine contact
  double final_limit_cur_a = 0.0;    // the current limit the run ended at (A)
  int current_raises = 0;            // number of adaptive current raises
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
    // Speed-mode (velocity) command for this phase (SpdRef, signed rad/s).
    // The speed-mode executor drives at this constant speed; the drive's own
    // velocity loop is the smooth source of motion (as opposed to a
    // host-generated moving position target, which stick-slips).
    double velocity_rad_s = 0.0;
    bool approach = false;  // move until contact, not to a fixed target
    TimeNs start_ns = 0;
  };

  DesiredState hold_state(const std::string& msg) const;
  DesiredState move_state(double target, double speed, double velocity,
                          const std::string& msg) const;
  bool arrived(const HomingFeedback& fb) const;
  bool timed_out(const HomingFeedback& fb) const;
  bool settled(const HomingFeedback& fb) const;
  // target_distance_rad < 0 = use p_.max_travel_rad (the fine approaches).
  void begin_approach(double speed_rad_s, TimeNs now, double current_pos_rad,
                      double target_distance_rad = -1.0);
  void begin_backoff_to(TimeNs now, double target_rad);
  // Backoff move drive: closed-loop position tracking in velocity mode
  // (target-seeking + stall-breaking burst; see the HomingParams backoff
  // comment). Updates phase_.velocity_rad_s and returns the commanded
  // velocity. Called every cycle while a backoff move is active.
  double backoff_velocity(const HomingFeedback& fb);
  // Backoff arrival: at the target AND slow enough to hold there (a
  // momentum-coasting axis must not be handed to the settle while it is
  // still crossing the target at speed).
  bool backoff_arrived(const HomingFeedback& fb) const;
  void begin_settle(TimeNs now);
  void begin_hold();
  void fail(const std::string& reason);
  // Jitter (stick-slip) diagnostic suffix for timeout/repeatability fails
  // (C1, A.4): empty unless the approach showed stick-slip.
  std::string jitter_suffix() const;

  AxisId axis_;
  int dir_;
  HomingParams p_;

  AxisHomeState state_ = AxisHomeState::Unknown;
  ContactDetector detector_;
  ContactResult last_cr_{};  // latest detector result during the approach
  Phase phase_;

  // Stall-breaking state for the backoff move (backoff_velocity).
  TimeNs stall_since_ns_ = 0;  // 0 = not currently stalled
  TimeNs burst_until_ns_ = 0;  // 0 = no burst in flight

  // VerifyRepeatability sub-phase: 0 = back-off, 1 = second fine approach.
  int verify_phase_ = 0;

  double start_pos_rad_ = 0.0;
  bool has_start_ = false;

  // Recorded contact positions (rad).
  double coarse_contact_rad_ = 0.0;
  double fine_contact1_rad_ = 0.0;
  double fine_contact2_rad_ = 0.0;
  int fine_samples_ = 0;

  // --- Adaptive-current state (§22) ---
  double limit_cur_a_ = 0.0;        // current drive limit (A); 0 until first use
  bool current_dirty_ = false;      // DesiredState must carry limit_cur_a_
  bool rearm_pending_ = false;      // next DesiredState must carry rearm_speed_mode
  double coarse_start_pos_rad_ = 0.0;  // position when the coarse approach began
  bool has_coarse_start_ = false;
  int current_raises_ = 0;
  double peak_torque_nm_ = 0.0;     // max |tau| over the whole run

  HomingResult result_;
};

}  // namespace ota
