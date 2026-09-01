// OpenAutoTurret — sensorless end-stop contact detector (architecture §21).
#include "calibration/contact_detector.hpp"

#include <algorithm>
#include <cmath>

namespace ota {
namespace {
// Short exponential low-pass time constant: rejects single-frame noise without
// adding hundreds of ms of lag (§21.1).
constexpr double kFilterTauS = 0.030;
// Acceleration / jerk low-pass constants (C1). Short enough to resolve a
// stick-slip slip (~10-50 ms) yet long enough to reject single-frame noise.
// The velocity they differentiate is already the position-derived v_f_ above,
// which is itself filtered — never the drive's self-reported v (±0.05 rad/s
// noise band at rest, P0j) and never a raw d^2q/dt^2.
constexpr double kATauS = 0.010;
constexpr double kJTauS = 0.010;
// Ignore integration steps shorter than this (stale/duplicate feedback) so a
// tiny dt does not turn filter ripple into a bogus acceleration spike.
constexpr double kMinDtS = 1e-3;
}  // namespace

ContactDetector::ContactDetector(ContactDetectorParams p) : p_(p) {}

void ContactDetector::set_approach_direction(int dir) { dir_ = (dir < 0) ? -1 : 1; }

void ContactDetector::reset() {
  v_f_ = 0.0;
  effort_f_ = 0.0;
  delta_q_window_ = 0.0;
  v_max_since_reset_ = 0.0;
  a_f_ = 0.0;
  j_f_ = 0.0;
  v_f_prev_ = 0.0;
  a_f_prev_ = 0.0;
  prev_stalled_ = true;
  ever_moved_ = false;
  total_recoveries_ = 0;
  max_a_since_reset_ = 0.0;
  max_j_since_reset_ = 0.0;
  jit_head_ = 0;
  jit_count_ = 0;
  has_prev_ = false;
  candidate_active_ = false;
  contact_declared_ = false;
}

ContactResult ContactDetector::update(TimeNs t_ns, double q_rad, double v_rad_s,
                                      double torque_nm, bool motor_fault) {
  ContactResult r;
  (void)v_rad_s;  // intentionally unused: the drive's self-reported velocity is
                  // noise-dominated (±0.05 rad/s at rest) and does not track
                  // motion; stalling is judged from position-derived velocity.

  if (!has_prev_) {
    effort_f_ = torque_nm;
    has_prev_ = true;
    t_prev_ns_ = t_ns;
    q_prev_rad_ = q_rad;
    q_ref_rad_ = q_rad;
    t_ref_ns_ = t_ns;
    reset_ns_ = t_ns;
    dwell_start_ns_ = t_ns;
  } else {
    const double dt_s = (t_ns - t_prev_ns_) / 1e9;
    if (dt_s >= kMinDtS) {
      const double alpha = dt_s / (kFilterTauS + dt_s);
      // Position-derived velocity (the trustworthy motion signal).
      const double v_inst = (q_rad - q_prev_rad_) / dt_s;
      v_f_ += alpha * (v_inst - v_f_);
      if (std::fabs(v_f_) > v_max_since_reset_) v_max_since_reset_ = std::fabs(v_f_);
      effort_f_ += alpha * (torque_nm - effort_f_);
      // Acceleration and jerk of the (filtered) position-derived velocity.
      const double a_inst = (v_f_ - v_f_prev_) / dt_s;
      const double alpha_a = dt_s / (kATauS + dt_s);
      a_f_ += alpha_a * (a_inst - a_f_);
      const double j_inst = (a_f_ - a_f_prev_) / dt_s;
      const double alpha_j = dt_s / (kJTauS + dt_s);
      j_f_ += alpha_j * (j_inst - j_f_);
      v_f_prev_ = v_f_;
      a_f_prev_ = a_f_;
      // Whole-approach extremes (for the insufficient-torque fault, A.4).
      if (std::fabs(a_f_) > max_a_since_reset_) max_a_since_reset_ = std::fabs(a_f_);
      if (std::fabs(j_f_) > max_j_since_reset_) max_j_since_reset_ = std::fabs(j_f_);
      // Recovery (jitter): the axis was moving at the approach speed, then
      // stalled, and has now clearly moved again — the signature of a
      // breakaway stall (NOT a permanent stop). Uses v_move_threshold_rad_s
      // (below the approach speed), NOT the contact gate's
      // v_stall_threshold_rad_s (above it). ever_moved_ (the value BEFORE this
      // sample) excludes the initial start-up breakaway (rest -> motion).
      const bool now_stalled = std::fabs(v_f_) < p_.v_move_threshold_rad_s;
      if (prev_stalled_ && std::fabs(v_f_) >= p_.v_move_threshold_rad_s &&
          ever_moved_) {
        ++total_recoveries_;
      }
      // Record a trailing-window sample (had_moved_before = state before this
      // sample's motion is counted).
      jit_[jit_head_] =
          JitSample{t_ns, a_f_, j_f_, dir_ * effort_f_, v_f_, ever_moved_};
      jit_head_ = (jit_head_ + 1) % kJitCap;
      if (jit_count_ < kJitCap) ++jit_count_;
      if (std::fabs(v_f_) > p_.v_move_threshold_rad_s) ever_moved_ = true;
      prev_stalled_ = now_stalled;
      q_prev_rad_ = q_rad;
    }
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

  // --- Trailing-window jitter metrics (C1) ---
  {
    const TimeNs window_ns = static_cast<TimeNs>(p_.jitter_window_ms * 1e6);
    double max_a = 0.0, min_a = 0.0, max_j = 0.0;
    double sum_e = 0.0, sum_e2 = 0.0;
    int n = 0, recoveries = 0;
    double prev_v = 0.0;
    bool prev_had_moved = false;
    bool have_prev = false;
    const int start = (jit_head_ - jit_count_ + kJitCap) % kJitCap;
    for (int k = 0; k < jit_count_; ++k) {
      const JitSample& s = jit_[(start + k) % kJitCap];
      if (t_ns - s.t_ns > window_ns) continue;  // outside the trailing window
      if (have_prev) {
        // A recovery: the previous sample was stalled (below the move
        // threshold), this one is moving, and the axis had ALREADY been moving
        // before the stall (excludes the initial start-up breakaway, which is
        // not a stall).
        const bool prev_stalled = std::fabs(prev_v) < p_.v_move_threshold_rad_s;
        const bool this_moved = std::fabs(s.v) >= p_.v_move_threshold_rad_s;
        if (prev_stalled && this_moved && prev_had_moved) ++recoveries;
      }
      prev_v = s.v;
      prev_had_moved = s.had_moved_before;
      have_prev = true;
      if (s.a > max_a) max_a = s.a;
      if (s.a < min_a) min_a = s.a;
      const double ja = std::fabs(s.j);
      if (ja > max_j) max_j = ja;
      sum_e += s.effort;
      sum_e2 += s.effort * s.effort;
      ++n;
    }
    double effort_std = 0.0;
    if (n > 1) {
      const double mean = sum_e / n;
      effort_std = std::sqrt(std::max(0.0, sum_e2 / n - mean * mean));
    }
    r.max_accel_rad_s2 = max_a;
    r.min_accel_rad_s2 = min_a;
    r.max_jerk_rad_s3 = max_j;
    r.effort_std_nm = effort_std;
    r.stall_recoveries = recoveries;
    r.total_stall_recoveries = total_recoveries_;
    r.max_accel_since_reset = max_a_since_reset_;
    r.max_jerk_since_reset = max_j_since_reset_;
    r.recovered_in_window = recoveries >= 1;
    // "Not smooth" = a slip (recovery) or an acceleration peak. Used for the
    // insufficient-torque fault/sizing; the CONTACT gate below uses only
    // recovered_in_window (a true stop never recovers; the a-peak is kept out
    // of the gate because a clean short approach's breakaway is a legitimate
    // peak).
    r.jitter = r.recovered_in_window || (max_a > p_.a_peak_rad_s2);
  }

  const bool stalled_v = std::fabs(v_f_) < p_.v_stall_threshold_rad_s;
  const bool stalled_q = delta_q_window_ < p_.q_stall_threshold_rad;
  const bool high_effort = signed_effort > p_.effort_contact_threshold_nm;
  const bool active_long_enough =
      (t_ns - reset_ns_) >= static_cast<TimeNs>(p_.min_command_active_ms * 1e6);
  // Discriminator (P0j): a genuine stop is reached AFTER the axis has moved
  // (v peaked above motion_history_velocity) or shows the high push plateau
  // (effort > effort_hard_contact). This rejects the drive's slow-start ramp,
  // where the axis creeps (below the stall gates) while pushing 0.2-0.45 N.m
  // against local loads, which the raw stall+effort signature misreads as
  // contact 37 degrees short of the true end-stop.
  const bool had_motion = v_max_since_reset_ > p_.motion_history_vel_rad_s;
  const bool very_high_effort = signed_effort > p_.effort_hard_contact_nm;
  // Jitter gate (C1, P0l/P0m): a mid-approach breakaway stall recovers (the
  // drive slips free), a true mechanical stop does not. If the axis recovered
  // in the trailing window this is NOT a permanent stop — reject the candidate
  // and keep approaching.
  const bool not_jitter = !r.recovered_in_window;
  r.candidate = stalled_v && stalled_q && high_effort && active_long_enough &&
                (had_motion || very_high_effort) && not_jitter;

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
