// OpenAutoTurret — sensorless end-stop contact detector (architecture §21).
//
// End-stop contact is declared only when multiple filtered signals agree for a
// configurable dwell (not one sample):
//
//   |filtered_velocity| < v_stall_threshold
//   AND |delta_position_window| < q_stall_threshold
//   AND signed_effort_in_stop_direction > effort_contact_threshold
//   AND the approach command has been active long enough
//   AND (the axis reached motion_history_velocity since the approach began,
//        OR signed_effort > effort_hard_contact)
//
// persisted for contact_dwell_ms. A separate hard abort fires immediately on
// a large effort or a motor fault, independent of the dwell (§21.2), protecting
// against a failed stall detector.
//
// Notes (P0j, 2026-09-01):
//  - Velocity is derived from POSITION (delta-q over fresh feedback), not the
//    drive's self-reported velocity: the drive's v readout is a ±0.05 rad/s
//    noise band at rest and does not track actual motion.
//  - The (motion history OR hard effort) gate distinguishes "arrived fast and
//    stopped against a stop" from the drive's slow-start ramp: during the ramp
//    the axis creeps (below the stall gates) while pushing 0.2-0.45 N.m
//    against local loads, which the raw stall+effort signature alone reads as
//    contact. A genuine stop is reached after real motion, or shows the
//    0.43-0.73 N.m push plateau (effort_hard_contact) even on a slow approach.
#pragma once

#include <cstdint>

#include "common/types.hpp"

namespace ota {

struct ContactDetectorParams {
  double v_stall_threshold_rad_s = 0.2;      // |v| (position-derived) below this = stalled
  double q_stall_threshold_rad = 0.001;      // |dq| over window below this = not advancing
  double progress_window_s = 0.2;            // window for position-progress check
  double effort_contact_threshold_nm = 3.0;  // effort in stop direction for a candidate
  double effort_hard_contact_nm = 0.40;      // sustained effort this high = contact even
                                             // without prior motion (measured stop
                                             // plateau 0.43-0.73 N.m, P0e)
  double motion_history_vel_rad_s = 0.05;    // |v| (position-derived) must have reached
                                             // this since reset for a motion-based contact
  double effort_hard_abort_nm = 9.0;         // immediate safe-stop (magnitude)
  int contact_dwell_ms = 200;                // candidate persistence for contact
  int min_command_active_ms = 100;           // command must be active this long
  // --- jitter (stick-slip) detection (drive_current_friction_tuning.md C1) ---
  // A true mechanical stop is a PERMANENT stall (v stays ~0, no recovery). A
  // mid-approach breakaway stall (insufficient torque to beat friction) is
  // INTERMITTENT: the drive stalls, then slips free and moves again. That
  // stall->recovery transition is the signature used to reject a false contact
  // and keep approaching; the acceleration peak (the slip) and jerk spike
  // corroborate it. Thresholds are tuned on real clean-vs-jittery moves
  // (Phase A.3), not a priori.
  int jitter_window_ms = 250;                // trailing window for the metrics
  // Threshold for the JITTER/recovery detection (distinct from
  // v_stall_threshold_rad_s, which the contact gate uses and which sits ABOVE
  // the approach speed). A breakaway stall is "the drive was moving at the
  // approach speed, then stalled, then moved again", so this must be BELOW the
  // approach speed (10 deg/s = 0.175, 20 deg/s = 0.349 rad/s) and above the
  // filtered rest noise. A "recovery" is a single crossing of this threshold
  // from below, so each stall->motion episode is counted exactly once (a
  // separate lower threshold would let the filtered velocity re-trigger on
  // every sample while it climbs back through the band).
  double v_move_threshold_rad_s = 0.10;      // |v| above this = "moving" (jitter)
  double a_peak_rad_s2 = 8.0;                // |a| peak above this = a slip (not smooth)
};

struct ContactResult {
  bool candidate = false;      // all candidate conditions met this sample
  bool contact = false;        // contact declared (dwell satisfied; latches until reset)
  bool hard_abort = false;     // immediate safe-stop (large effort or fault)
  double signed_effort_nm = 0.0;   // dir * filtered_effort (for diagnostics)
  double delta_q_window_rad = 0.0; // position change over the progress window
  // --- jitter (stick-slip) diagnostics (C1) ---
  bool recovered_in_window = false;  // a stall->recovery in the trailing window
                                      // (=> NOT a permanent stop; reject contact)
  bool jitter = false;               // motion not smooth (recovery or a-peak)
  int stall_recoveries = 0;          // stall->recovery count in the trailing window
  int total_stall_recoveries = 0;    // stall->recovery count since reset()
  double max_accel_rad_s2 = 0.0;     // max a in the trailing window
  double min_accel_rad_s2 = 0.0;     // min a in the trailing window
  double max_jerk_rad_s3 = 0.0;      // max |j| in the trailing window
  double max_accel_since_reset = 0.0;  // max |a| since reset() (whole approach)
  double max_jerk_since_reset = 0.0;   // max |j| since reset() (whole approach)
  double effort_std_nm = 0.0;         // std of (signed) effort in the window
};

class ContactDetector {
 public:
  explicit ContactDetector(ContactDetectorParams p);

  // The commanded approach direction (+1 or -1); sets the stop-direction sign.
  void set_approach_direction(int dir);
  // Clear filters/dwell; call when starting a new approach.
  void reset();

  // Feed one sample. t_ns is monotonic ns; q_rad/torque_nm are the feedback
  // readout; motor_fault is the axis fault flag. v_rad_s (the drive's
  // self-reported velocity) is accepted for interface stability but is NOT
  // used for gating: it is noise-dominated (±0.05 rad/s at rest) and does not
  // track actual motion. Stalling is judged from position-derived velocity.
  ContactResult update(TimeNs t_ns, double q_rad, double v_rad_s, double torque_nm,
                       bool motor_fault);

  double filtered_velocity() const { return v_f_; }
  double filtered_effort() const { return effort_f_; }
  double delta_q_window() const { return delta_q_window_; }
  double max_velocity_since_reset() const { return v_max_since_reset_; }

 private:
  // One trailing-window sample for the jitter metrics.
  struct JitSample {
    TimeNs t_ns = 0;
    double a = 0.0;      // filtered acceleration (rad/s^2)
    double j = 0.0;      // filtered jerk (rad/s^3)
    double effort = 0.0; // signed effort (N.m)
    double v = 0.0;      // position-derived velocity (rad/s)
    // Had the axis already been clearly moving (|v| > v_stall) BEFORE this
    // sample? Used to tell a real breakaway STALL (was moving, then stalled,
    // then recovered) apart from the initial start-up breakaway (rest ->
    // motion), which must NOT count as a recovery.
    bool had_moved_before = false;
  };
  static constexpr int kJitCap = 128;  // ~640 ms at 200 Hz (window is 250 ms)

  ContactDetectorParams p_;
  int dir_ = 1;
  double v_f_ = 0.0;  // position-derived (delta-q), filtered
  double effort_f_ = 0.0;
  double delta_q_window_ = 0.0;
  double v_max_since_reset_ = 0.0;  // max |position-derived v| since reset()
  // Derived motion signals (C1): acceleration and jerk of the position-derived
  // velocity (NOT the drive's noisy self-reported v; see .cpp for the method).
  double a_f_ = 0.0;
  double j_f_ = 0.0;
  double v_f_prev_ = 0.0;
  double a_f_prev_ = 0.0;
  bool prev_stalled_ = true;
  bool ever_moved_ = false;        // has the axis clearly moved since reset()?
  int total_recoveries_ = 0;       // stall->recovery count since reset()
  double max_a_since_reset_ = 0.0;
  double max_j_since_reset_ = 0.0;
  JitSample jit_[kJitCap];
  int jit_head_ = 0;
  int jit_count_ = 0;
  bool has_prev_ = false;
  TimeNs t_prev_ns_ = 0;
  double q_prev_rad_ = 0.0;
  double q_ref_rad_ = 0.0;
  TimeNs t_ref_ns_ = 0;
  TimeNs reset_ns_ = 0;
  TimeNs dwell_start_ns_ = 0;
  bool candidate_active_ = false;
  bool contact_declared_ = false;
};

}  // namespace ota
