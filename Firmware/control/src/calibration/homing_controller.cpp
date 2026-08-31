// OpenAutoTurret — sensorless precision homing state machine (architecture
// §22/§23/§26). See the header for the sequence and the transport-agnostic
// contract (step() takes feedback, returns a DesiredState for a move executor).
#include "calibration/homing_controller.hpp"

#include <cmath>

namespace ota {

HomingController::HomingController(AxisId axis, int approach_dir, HomingParams p)
    : axis_(axis), dir_(approach_dir < 0 ? -1 : 1), p_(p), detector_(p.contact) {}

DesiredState HomingController::hold_state(const std::string& msg) const {
  return DesiredState{0.0, 0.0, true, msg};
}

DesiredState HomingController::move_state(double target, double speed,
                                          const std::string& msg) const {
  return DesiredState{target, speed, false, msg};
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
                                      double current_pos_rad) {
  phase_.kind = PhaseKind::Move;
  phase_.approach = true;
  phase_.speed_rad_s = speed_rad_s;
  // Drive toward a far point in the approach direction; the axis actually stops
  // on contact (or the timeout), well short of this target.
  phase_.target_rad = current_pos_rad + p_.max_travel_rad * dir_;
  phase_.start_ns = now;
  detector_.reset();
  detector_.set_approach_direction(dir_);
}

void HomingController::begin_backoff_to(TimeNs now, double target_rad) {
  phase_.kind = PhaseKind::Move;
  phase_.approach = false;
  phase_.speed_rad_s = p_.backoff_speed_rad_s;
  phase_.target_rad = target_rad;
  phase_.start_ns = now;
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
  begin_hold();
  state_ = AxisHomeState::Failed;
}

DesiredState HomingController::step(const HomingFeedback& fb) {
  // 1. Run the contact detector if the current phase is an approach move.
  bool contact = false;
  bool hard_abort = false;
  if (phase_.kind == PhaseKind::Move && phase_.approach) {
    const ContactResult cr =
        detector_.update(fb.t_ns, fb.pos_rad, fb.vel_rad_s, fb.torque_nm,
                         fb.motor_fault);
    contact = cr.contact;
    hard_abort = cr.hard_abort;
  }

  // 2. Advance the FSM based on the feedback.
  switch (state_) {
    case AxisHomeState::Unknown:
      start_pos_rad_ = fb.pos_rad;
      has_start_ = true;
      begin_approach(p_.coarse_speed_rad_s, fb.t_ns, fb.pos_rad);
      state_ = AxisHomeState::ApproachCoarse;
      break;

    case AxisHomeState::ApproachCoarse:
      if (hard_abort || fb.motor_fault) {
        fail("coarse approach: hard abort or motor fault");
      } else if (contact) {
        coarse_contact_rad_ = fb.pos_rad;
        begin_settle(fb.t_ns);
        state_ = AxisHomeState::ContactCoarse;
      } else if (timed_out(fb)) {
        fail("coarse approach: timeout with no contact");
      } else if (arrived(fb)) {
        fail("coarse approach: reached travel limit with no contact");
      }
      break;

    case AxisHomeState::ContactCoarse:
      if (settled(fb)) {
        begin_backoff_to(fb.t_ns, coarse_contact_rad_ - p_.backoff_rad * dir_);
        state_ = AxisHomeState::Backoff;
      }
      break;

    case AxisHomeState::Backoff:
      if (arrived(fb)) {
        begin_settle(fb.t_ns);
        state_ = AxisHomeState::Settle;
      }
      break;

    case AxisHomeState::Settle:
      if (settled(fb)) {
        begin_approach(p_.fine_speed_rad_s, fb.t_ns, fb.pos_rad);
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
        fail("fine approach: timeout with no contact");
      } else if (arrived(fb)) {
        fail("fine approach: reached travel limit with no contact");
      }
      break;

    case AxisHomeState::ContactFine:
      if (settled(fb)) {
        verify_phase_ = 0;
        begin_backoff_to(fb.t_ns, fine_contact1_rad_ - p_.small_backoff_rad * dir_);
        state_ = AxisHomeState::VerifyRepeatability;
      }
      break;

    case AxisHomeState::VerifyRepeatability:
      if (hard_abort || fb.motor_fault) {
        fail("repeatability: hard abort or motor fault");
      } else if (verify_phase_ == 0) {
        if (arrived(fb)) {
          verify_phase_ = 1;
          begin_approach(p_.fine_speed_rad_s, fb.t_ns, fb.pos_rad);
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
            begin_hold();
            state_ = AxisHomeState::Complete;
          } else {
            fail("repeatability exceeded: |q1 - q2| over the limit");
          }
        } else if (timed_out(fb)) {
          fail("repeatability: second approach timeout with no contact");
        } else if (arrived(fb)) {
          fail("repeatability: second approach reached travel limit, no contact");
        }
      }
      break;

    case AxisHomeState::Complete:
    case AxisHomeState::Failed:
      // Terminal: the phase is already a hold; nothing to do.
      break;
  }

  // 3. Report the desired state for the (possibly new) phase.
  switch (phase_.kind) {
    case PhaseKind::None:
      return hold_state(terminal() ? (state_ == AxisHomeState::Complete
                                         ? "homing complete"
                                         : "homing failed")
                                   : "idle");
    case PhaseKind::Settle:
      return hold_state("settle");
    case PhaseKind::Move:
      return move_state(phase_.target_rad, phase_.speed_rad_s,
                        phase_.approach ? "approach" : "move");
  }
  return hold_state("idle");
}

}  // namespace ota
