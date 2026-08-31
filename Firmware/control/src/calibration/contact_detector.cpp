// OpenAutoTurret — sensorless end-stop contact detector (architecture §21).
#include "calibration/contact_detector.hpp"

#include <algorithm>
#include <cmath>

namespace ota {
namespace {
// Short exponential low-pass time constant: rejects single-frame noise without
// adding hundreds of ms of lag (§21.1).
constexpr double kFilterTauS = 0.030;
}  // namespace

ContactDetector::ContactDetector(ContactDetectorParams p) : p_(p) {}

void ContactDetector::set_approach_direction(int dir) { dir_ = (dir < 0) ? -1 : 1; }

void ContactDetector::reset() {
  v_f_ = 0.0;
  effort_f_ = 0.0;
  delta_q_window_ = 0.0;
  has_prev_ = false;
  candidate_active_ = false;
  contact_declared_ = false;
}

ContactResult ContactDetector::update(TimeNs t_ns, double q_rad, double v_rad_s,
                                      double torque_nm, bool motor_fault) {
  ContactResult r;

  if (!has_prev_) {
    v_f_ = v_rad_s;
    effort_f_ = torque_nm;
    has_prev_ = true;
    t_prev_ns_ = t_ns;
    q_ref_rad_ = q_rad;
    t_ref_ns_ = t_ns;
    reset_ns_ = t_ns;
    dwell_start_ns_ = t_ns;
  } else {
    const double dt_s = std::max(1e-6, (t_ns - t_prev_ns_) / 1e9);
    const double alpha = dt_s / (kFilterTauS + dt_s);
    v_f_ += alpha * (v_rad_s - v_f_);
    effort_f_ += alpha * (torque_nm - effort_f_);
    // Advance the position-progress reference once the window has elapsed.
    if (t_ns - t_ref_ns_ >= static_cast<TimeNs>(p_.progress_window_s * 1e9)) {
      q_ref_rad_ = q_rad;
      t_ref_ns_ = t_ns;
    }
  }
  t_prev_ns_ = t_ns;

  delta_q_window_ = std::fabs(q_rad - q_ref_rad_);
  const double signed_effort = dir_ * effort_f_;
  r.signed_effort_nm = signed_effort;
  r.delta_q_window_rad = delta_q_window_;

  const bool stalled_v = std::fabs(v_f_) < p_.v_stall_threshold_rad_s;
  const bool stalled_q = delta_q_window_ < p_.q_stall_threshold_rad;
  const bool high_effort = signed_effort > p_.effort_contact_threshold_nm;
  const bool active_long_enough =
      (t_ns - reset_ns_) >= static_cast<TimeNs>(p_.min_command_active_ms * 1e6);
  r.candidate = stalled_v && stalled_q && high_effort && active_long_enough;

  // Hard abort is immediate and independent of the dwell (§21.2): a large
  // effort (either sense) or a reported fault.
  r.hard_abort = motor_fault || (std::fabs(effort_f_) > p_.effort_hard_abort_nm);

  if (r.candidate) {
    if (!candidate_active_) {
      candidate_active_ = true;
      dwell_start_ns_ = t_ns;
    }
    if ((t_ns - dwell_start_ns_) >= static_cast<TimeNs>(p_.contact_dwell_ms * 1e6)) {
      contact_declared_ = true;
    }
  } else {
    candidate_active_ = false;
  }
  r.contact = contact_declared_;
  return r;
}

}  // namespace ota
