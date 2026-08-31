// OpenAutoTurret — sensorless end-stop contact detector (architecture §21).
//
// End-stop contact is declared only when multiple filtered signals agree for a
// configurable dwell (not one sample):
//
//   |filtered_velocity| < v_stall_threshold
//   AND |delta_position_window| < q_stall_threshold
//   AND signed_effort_in_stop_direction > effort_contact_threshold
//   AND the approach command has been active long enough
//
// persisted for contact_dwell_ms. A separate hard abort fires immediately on
// a large effort or a motor fault, independent of the dwell (§21.2), protecting
// against a failed stall detector.
#pragma once

#include <cstdint>

#include "common/types.hpp"

namespace ota {

struct ContactDetectorParams {
  double v_stall_threshold_rad_s = 0.2;      // |v| below this = stalled
  double q_stall_threshold_rad = 0.002;      // |dq| over window below this = not advancing
  double progress_window_s = 0.2;            // window for position-progress check
  double effort_contact_threshold_nm = 3.0;  // effort in stop direction for a candidate
  double effort_hard_abort_nm = 9.0;         // immediate safe-stop (magnitude)
  int contact_dwell_ms = 200;                // candidate persistence for contact
  int min_command_active_ms = 100;           // command must be active this long
};

struct ContactResult {
  bool candidate = false;      // all candidate conditions met this sample
  bool contact = false;        // contact declared (dwell satisfied; latches until reset)
  bool hard_abort = false;     // immediate safe-stop (large effort or fault)
  double signed_effort_nm = 0.0;   // dir * filtered_effort (for diagnostics)
  double delta_q_window_rad = 0.0; // position change over the progress window
};

class ContactDetector {
 public:
  explicit ContactDetector(ContactDetectorParams p);

  // The commanded approach direction (+1 or -1); sets the stop-direction sign.
  void set_approach_direction(int dir);
  // Clear filters/dwell; call when starting a new approach.
  void reset();

  // Feed one sample. t_ns is monotonic ns; q_rad/v_rad_s/torque_nm are the
  // feedback readout; motor_fault is the axis fault flag.
  ContactResult update(TimeNs t_ns, double q_rad, double v_rad_s, double torque_nm,
                       bool motor_fault);

  double filtered_velocity() const { return v_f_; }
  double filtered_effort() const { return effort_f_; }
  double delta_q_window() const { return delta_q_window_; }

 private:
  ContactDetectorParams p_;
  int dir_ = 1;
  double v_f_ = 0.0;
  double effort_f_ = 0.0;
  double delta_q_window_ = 0.0;
  bool has_prev_ = false;
  TimeNs t_prev_ns_ = 0;
  double q_ref_rad_ = 0.0;
  TimeNs t_ref_ns_ = 0;
  TimeNs reset_ns_ = 0;
  TimeNs dwell_start_ns_ = 0;
  bool candidate_active_ = false;
  bool contact_declared_ = false;
};

}  // namespace ota
