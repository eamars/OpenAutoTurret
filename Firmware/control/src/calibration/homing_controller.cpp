// OpenAutoTurret — sensorless precision homing state machine (architecture
// §22/§23/§26). See the header for the sequence and the transport-agnostic
// contract (step() takes feedback, returns a DesiredState for a move executor).
#include "calibration/homing_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ota {

HomingController::HomingController(AxisId axis, int approach_dir, HomingParams p)
    : axis_(axis), dir_(approach_dir < 0 ? -1 : 1), p_(p), detector_(p.contact),
      limit_cur_a_(p.limit_cur_initial_a), current_dirty_(true),
      rearm_pending_(p.rearm_before_start) {}

DesiredState HomingController::hold_state(const std::string& msg) const {
  return DesiredState{0.0, 0.0, 0.0, true, msg};
}

DesiredState HomingController::move_state(double target, double speed,
                                          double velocity,
                                          const std::string& msg) const {
  return DesiredState{target, speed, velocity, false, msg};
}

bool HomingController::arrived(const HomingFeedback& fb) const {
  return std::fabs(fb.pos_rad - phase_.target_rad) < p_.arrival_tol_rad;
}

bool HomingController::timed_out(const HomingFeedback& fb) const {
  return (fb.t_ns - phase_.start_ns) >=
         static_cast<TimeNs>(p_.approach_timeout_s * 1e9);
}

bool HomingController::settled(const HomingFeedback& fb) const {
  return (fb.t_ns - phase_.start_ns) >= static_cast<TimeNs>(p_.settle_time_s * 1e9);
}

void HomingController::begin_approach(double speed_rad_s, TimeNs now,
                                      double current_pos_rad,
                                      double target_distance_rad) {
  phase_.kind = PhaseKind::Move;
  phase_.approach = true;
  // Approaches are always speed mode (the backoff is the only position-mode
  // phase); clear the flag a previous backoff move left behind.
  phase_.position_mode = false;
  phase_.speed_rad_s = speed_rad_s;
  // Speed mode: command the constant approach speed (signed by the approach
  // direction). The drive's velocity loop holds it smoothly; the axis stops on
  // contact (or the timeout / rotation cap).
  phase_.velocity_rad_s = speed_rad_s * dir_;
  // Drive toward a far point in the approach direction; the axis actually stops
  // on contact (or the timeout), well short of this target. The coarse
  // approach passes the rotation cap (max_rotation_rad + margin) so the
  // `arrived` travel-limit check never trips before the rotation cap does.
  const double dist =
      (target_distance_rad < 0.0) ? p_.max_travel_rad : target_distance_rad;
  phase_.target_rad = current_pos_rad + dist * dir_;
  phase_.start_ns = now;
  detector_.reset();
  detector_.set_approach_direction(dir_);
  last_cr_ = ContactResult{};
}

void HomingController::begin_backoff_to(TimeNs now, double target_rad,
                                        double start_rad) {
  phase_.kind = PhaseKind::Move;
  phase_.approach = false;
  phase_.position_mode = true;
  // Position mode (p3c fix, 2026-09-02): the drive's own position loop
  // (loc_kp ~30 N.m/rad) drives target_rad with a speed cap of
  // backoff_speed (LimitSpd). At a 5 deg error its P-term wants 2.6 N.m
  // (clamped at LimitCur) from the FIRST cycle — the full current-limit
  // torque — so the yaw detent zone's static friction (~0.4-0.55 N.m) cannot
  // hold, and there is no integral to wind up (the velocity-mode backoff's
  // 0.066 N.m P-term + slow I build-up + momentum-burst overshoot is the
  // p3c failure; see the HomingParams backoff comment).
  phase_.speed_rad_s = p_.backoff_speed_rad_s;
  phase_.velocity_rad_s = 0.0;
  phase_.target_rad = target_rad;
  phase_.start_ns = now;
  // Wide arrival window (see HomingParams::backoff_arrive_frac): the
  // position loop can stall 0.5-1.5 deg short of the exact target inside
  // the detent zone (P falls below F_s as the error closes); anywhere inside
  // the window is a valid backoff. The window is always well below the
  // backoff distance, so the axis has actually left the stop.
  const double dist = std::fabs(target_rad - start_rad);
  arrive_tol_rad_ =
      std::max(0.5 * kDeg2Rad, p_.backoff_arrive_frac * dist);
  // One-shot position-mode entry (the executor runs the blocking
  // de-energize/RunMode=1/re-energize/LimitSpd/pin-LocRef recipe before
  // this cycle's command). The de-energize also resets the drive's
  // velocity-loop integral, which winds up to ~1.3-1.8 N.m while the
  // contact dwell pushes the stop (rehome3 root cause, wire-measured) —
  // in position mode that residual is inert (the velocity loop is not in
  // the control path), but the reset keeps the controller clean. The brief
  // de-energize lets the axis bounce off the stop toward the gravity
  // balance; the position move simply starts from wherever it lands.
  pos_enter_pending_ = true;
}

bool HomingController::backoff_arrived(const HomingFeedback& fb) const {
  return std::fabs(fb.pos_rad - phase_.target_rad) < arrive_tol_rad_ &&
         std::fabs(fb.vel_rad_s) < p_.backoff_arrive_vel_rad_s;
}

void HomingController::begin_settle(TimeNs now) {
  phase_.kind = PhaseKind::Settle;
  phase_.start_ns = now;
}

void HomingController::begin_hold() {
  phase_.kind = PhaseKind::None;
  phase_.start_ns = 0;
}

void HomingController::fail(const std::string& reason) {
  result_.complete = true;
  result_.valid = false;
  result_.fail_reason = reason;
  result_.peak_torque_nm = peak_torque_nm_;
  result_.final_limit_cur_a = limit_cur_a_;
  result_.current_raises = current_raises_;
  begin_hold();
  state_ = AxisHomeState::Failed;
}

std::string HomingController::jitter_suffix() const {
  // Only annotate when the approach actually showed stick-slip (a stall that
  // recovered, or an acceleration peak) — a clean approach that simply times
  // out (e.g. no stop found) is not a current problem.
  if (!last_cr_.jitter && last_cr_.total_stall_recoveries == 0) return "";
  char buf[192];
  std::snprintf(buf, sizeof(buf),
                " [jitter: stall_recoveries=%d, max_a=%.1f rad/s^2, "
                "max_j=%.1f rad/s^3, effort_std=%.2f N.m — insufficient "
                "torque authority, raise limit_cur]",
                last_cr_.total_stall_recoveries, last_cr_.max_accel_since_reset,
                last_cr_.max_jerk_since_reset, last_cr_.effort_std_nm);
  return buf;
}

DesiredState HomingController::step(const HomingFeedback& fb) {
  // 1. Run the contact detector if the current phase is an approach move.
  bool contact = false;
  bool hard_abort = false;
  if (phase_.kind == PhaseKind::Move && phase_.approach) {
    last_cr_ = detector_.update(fb.t_ns, fb.pos_rad, fb.vel_rad_s,
                                fb.torque_nm, fb.motor_fault);
    contact = last_cr_.contact;
    hard_abort = last_cr_.hard_abort;
  }

  // 2. Advance the FSM based on the feedback.
  switch (state_) {
    case AxisHomeState::Unknown: {
      start_pos_rad_ = fb.pos_rad;
      has_start_ = true;
      // The coarse approach may push up to the rotation cap (a full-rotation
      // axis has no stop until the mechanical end-stop), so the travel target
      // passes the cap by a margin and the cap itself is the safety bound.
      const double coarse_target_dist = p_.max_rotation_rad + 10.0 * kDeg2Rad;
      coarse_start_pos_rad_ = fb.pos_rad;
      has_coarse_start_ = true;
      begin_approach(p_.coarse_speed_rad_s, fb.t_ns, fb.pos_rad,
                     coarse_target_dist);
      state_ = AxisHomeState::ApproachCoarse;
      break;
    }

    case AxisHomeState::ApproachCoarse: {
      // Track peak torque over the run (for the §22 torque report). Use the
      // raw feedback torque (the actual drive torque) for the peak; the
      // torque-safety below uses the detector's filtered effort so it is
      // consistent with the hard-abort (same value, same threshold ordering).
      const double tau_abs = std::fabs(fb.torque_nm);
      if (tau_abs > peak_torque_nm_) peak_torque_nm_ = tau_abs;
      const double effort_abs = std::fabs(last_cr_.signed_effort_nm);
      // Hard abort / motor fault (contact-detector immediate safe-stop, the
      // lowest threshold — fires before the torque safety).
      if (hard_abort || fb.motor_fault) {
        fail("coarse approach: hard abort or motor fault (|tau|=" +
             std::to_string(tau_abs) + " N.m, peak " +
             std::to_string(peak_torque_nm_) + " N.m)");
        break;
      }
      // Torque safety: abort the push before the mechanical stop is overloaded.
      if (effort_abs > p_.torque_safety_nm) {
        fail("coarse approach: torque safety abort |tau|=" +
             std::to_string(effort_abs) + " N.m > " + std::to_string(p_.torque_safety_nm) +
             " N.m (peak " + std::to_string(peak_torque_nm_) + " N.m)");
        break;
      }
      // Rotation cap: if the axis has turned more than the cap without a
      // consistent (max-current) end-stop, there is no stop in this direction.
      if (has_coarse_start_) {
        const double rotated = std::fabs(fb.pos_rad - coarse_start_pos_rad_);
        if (rotated > p_.max_rotation_rad) {
          fail("coarse approach: rotation cap " +
               std::to_string(p_.max_rotation_rad / kDeg2Rad) +
               " deg exceeded (rotated " + std::to_string(rotated / kDeg2Rad) +
               " deg) with no consistent end-stop at " +
               std::to_string(limit_cur_a_) + " A");
          break;
        }
      }
      if (contact) {
        // In speed mode at the (low) homing current, a latched contact is a
        // stable mechanical stop: friction notches slip (the position never
        // stays steady) and are rejected by the contact detector's
        // stability/jitter gate, so a latched contact is the end-stop.
        // (No adaptive current raise — that was a position-mode hack that
        // made friction notches *hold* instead of slipping.)
        coarse_contact_rad_ = fb.pos_rad;
        begin_settle(fb.t_ns);
        state_ = AxisHomeState::ContactCoarse;
      } else if (timed_out(fb)) {
        fail("coarse approach: timeout with no contact" + jitter_suffix());
      } else if (arrived(fb)) {
        fail("coarse approach: reached travel limit with no contact" +
             jitter_suffix());
      }
      break;
    }

    case AxisHomeState::ContactCoarse:
      if (settled(fb)) {
        begin_backoff_to(fb.t_ns, coarse_contact_rad_ - p_.backoff_rad * dir_,
                         coarse_contact_rad_);
        state_ = AxisHomeState::Backoff;
      }
      break;

    case AxisHomeState::Backoff:
      // Position mode: the drive's own position loop drives the move
      // (LocRef/LimitSpd are pinned by the executor every cycle); there is
      // no host-side velocity to compute here.
      if (fb.motor_fault) {
        fail("backoff: motor fault");
      } else if (backoff_arrived(fb)) {
        begin_settle(fb.t_ns);
        state_ = AxisHomeState::Settle;
      } else if (fb.t_ns - phase_.start_ns >=
                 static_cast<TimeNs>(p_.backoff_timeout_s * 1e9)) {
        // The previous code had no timeout here: an axis that cannot reach
        // the backoff target (stuck in friction) hung the homing forever
        // (2026-09-02 yaw rehome2); p3c timed out inside the yaw detent
        // zone on the velocity-mode backoff that this replaced.
        fail("backoff: timeout (position mode, stuck in friction? q=" +
             std::to_string(fb.pos_rad) + " target=" +
             std::to_string(phase_.target_rad) + " dq=" +
             std::to_string(fb.pos_rad - phase_.target_rad) +
             " v=" + std::to_string(fb.vel_rad_s) + " rad)");
      }
      break;

    case AxisHomeState::Settle:
      if (settled(fb)) {
        begin_approach(p_.fine_speed_rad_s, fb.t_ns, fb.pos_rad);
        // The backoff left the axis in position mode; the fine approach is
        // speed mode. Re-arm (de-energize/re-energize, the speed-mode
        // recipe) so the approach starts from a clean velocity controller
        // — the same re-arm endpoint B uses before its coarse approach.
        // The de-energize lets the axis slide slightly off its backoff
        // settle position; the approach is "move until contact", so the
        // start position doesn't matter (the end-stop re-measures).
        rearm_pending_ = true;
        state_ = AxisHomeState::ApproachFine;
      }
      break;

    case AxisHomeState::ApproachFine:
      if (hard_abort || fb.motor_fault) {
        fail("fine approach: hard abort or motor fault");
      } else if (contact) {
        fine_contact1_rad_ = fb.pos_rad;
        fine_samples_ = 1;
        begin_settle(fb.t_ns);
        state_ = AxisHomeState::ContactFine;
      } else if (timed_out(fb)) {
        fail("fine approach: timeout with no contact" + jitter_suffix());
      } else if (arrived(fb)) {
        fail("fine approach: reached travel limit with no contact" +
             jitter_suffix());
      }
      break;

    case AxisHomeState::ContactFine:
      if (settled(fb)) {
        verify_phase_ = 0;
        begin_backoff_to(fb.t_ns, fine_contact1_rad_ - p_.small_backoff_rad * dir_,
                         fine_contact1_rad_);
        state_ = AxisHomeState::VerifyRepeatability;
      }
      break;

    case AxisHomeState::VerifyRepeatability:
      if (hard_abort || fb.motor_fault) {
        fail("repeatability: hard abort or motor fault");
      } else if (verify_phase_ == 0) {
        // Small backoff: position mode, same as the coarse backoff (it
        // starts at the stop, in the detent zone).
        if (fb.motor_fault) {
          fail("repeatability: small backoff motor fault");
        } else if (backoff_arrived(fb)) {
          verify_phase_ = 1;
          begin_approach(p_.fine_speed_rad_s, fb.t_ns, fb.pos_rad);
          // The small backoff left the axis in position mode; the second
          // fine approach is speed mode — re-arm (de-energize/re-energize)
          // so the SpdRef commands take effect (see the Settle ->
          // ApproachFine transition).
          rearm_pending_ = true;
        } else if (fb.t_ns - phase_.start_ns >=
                   static_cast<TimeNs>(p_.backoff_timeout_s * 1e9)) {
          fail("repeatability: small backoff timeout (position mode, stuck "
               "in friction? q=" +
               std::to_string(fb.pos_rad) + " target=" +
               std::to_string(phase_.target_rad) + " dq=" +
               std::to_string(fb.pos_rad - phase_.target_rad) +
               " v=" + std::to_string(fb.vel_rad_s) + " rad)");
        }
      } else {  // second fine approach
        if (contact) {
          fine_contact2_rad_ = fb.pos_rad;
          fine_samples_ = 2;
          const double rep = std::fabs(fine_contact2_rad_ - fine_contact1_rad_);
          result_.repeatability_rad = rep;
          if (rep <= p_.repeatability_rad) {
            result_.complete = true;
            result_.valid = true;
            result_.coarse_contact_rad = coarse_contact_rad_;
            result_.fine_contact_rad =
                0.5 * (fine_contact1_rad_ + fine_contact2_rad_);
            result_.fine_samples = fine_samples_;
            result_.peak_torque_nm = peak_torque_nm_;
            result_.contact_torque_nm = std::fabs(fb.torque_nm);
            result_.final_limit_cur_a = limit_cur_a_;
            result_.current_raises = current_raises_;
            begin_hold();
            state_ = AxisHomeState::Complete;
          } else {
            fail("repeatability exceeded: |q1 - q2| over the limit" +
                 jitter_suffix());
          }
        } else if (timed_out(fb)) {
          fail("repeatability: second approach timeout with no contact" +
               jitter_suffix());
        } else if (arrived(fb)) {
          fail("repeatability: second approach reached travel limit, no contact" +
               jitter_suffix());
        }
      }
      break;

    case AxisHomeState::Complete:
    case AxisHomeState::Failed:
      // Terminal: the phase is already a hold; nothing to do.
      break;
  }

  // 3. Report the desired state for the (possibly new) phase.
  DesiredState ds;
  switch (phase_.kind) {
    case PhaseKind::None:
      ds = hold_state(terminal() ? (state_ == AxisHomeState::Complete
                                       ? "homing complete"
                                       : "homing failed")
                                  : "idle");
      break;
    case PhaseKind::Settle:
      ds = hold_state("settle");
      break;
    case PhaseKind::Move:
      if (phase_.position_mode) {
        // Position mode (the backoff moves): the executor pins LocRef =
        // target_rad, LimitSpd = speed_rad_s (both write-on-change; a fixed
        // target costs one CAN write). velocity_rad_s stays 0.
        ds = move_state(phase_.target_rad, phase_.speed_rad_s, 0.0, "move:pos");
        ds.position_move = true;
      } else {
        ds = move_state(phase_.target_rad, phase_.speed_rad_s,
                        phase_.velocity_rad_s, phase_.approach ? "approach" : "move");
      }
      break;
  }
  // Apply the drive current limit on the cycle it changed (adaptive current,
  // §22). The executor writes LimitCur only when this is non-zero.
  if (current_dirty_) {
    ds.limit_cur_a = limit_cur_a_;
    current_dirty_ = false;
  }
  // One-shot velocity-controller re-arm (see begin_backoff_to /
  // rearm_before_start). The re-arm cycle must also carry the active axis'
  // current limit: enter_speed_mode re-writes LimitCur as part of the
  // recipe, and the executor takes it from this DesiredState. (The
  // executor's set_current_limit de-duplicates, so carrying the unchanged
  // value costs no extra CAN traffic.)
  if (rearm_pending_) {
    ds.rearm_speed_mode = true;
    ds.limit_cur_a = limit_cur_a_;
    rearm_pending_ = false;
  }
  // One-shot position-mode entry (the backoff moves). Carries the current
  // limit for the same reason as the re-arm cycle: the executor applies it
  // right after the mode-entry recipe (enter_position_mode's recipe does not
  // rewrite LimitCur; the backend de-duplicates the write).
  if (pos_enter_pending_) {
    ds.enter_pos_mode = true;
    ds.limit_cur_a = limit_cur_a_;
    pos_enter_pending_ = false;
  }
  return ds;
}

}  // namespace ota
