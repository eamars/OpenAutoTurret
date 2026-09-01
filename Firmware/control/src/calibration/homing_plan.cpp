// OpenAutoTurret — executable multi-axis homing plan (architecture §25).
#include "calibration/homing_plan.hpp"

namespace ota {

// ---------------------------------------------------------------------------
// CoarseEndpointHome
// ---------------------------------------------------------------------------
CoarseEndpointHome::CoarseEndpointHome(AxisId axis, int approach_dir, HomingParams p)
    : axis_(axis), dir_(approach_dir < 0 ? -1 : 1), p_(p), detector_(p.contact) {}

void CoarseEndpointHome::fail(const std::string& reason) {
  failed_ = true;
  fail_reason_ = reason;
  st_ = St::Failed;
}

DesiredState CoarseEndpointHome::step(const HomingFeedback& fb) {
  bool contact = false;
  bool hard_abort = false;
  if (st_ == St::Approach) {
    const ContactResult cr = detector_.update(fb.t_ns, fb.pos_rad, fb.vel_rad_s,
                                              fb.torque_nm, fb.motor_fault);
    contact = cr.contact;
    hard_abort = cr.hard_abort;
  }
  switch (st_) {
    case St::Unknown:
      start_pos_rad_ = fb.pos_rad;
      start_ns_ = fb.t_ns;
      detector_.reset();
      detector_.set_approach_direction(dir_);
      st_ = St::Approach;
      break;
    case St::Approach: {
      const bool timeout =
          (fb.t_ns - start_ns_) >= static_cast<TimeNs>(p_.approach_timeout_s * 1e9);
      const bool at_limit =
          std::fabs(fb.pos_rad - start_pos_rad_) >= p_.max_travel_rad;
      if (hard_abort || fb.motor_fault) {
        fail("coarse approach: hard abort or motor fault");
      } else if (contact) {
        contact_rad_ = fb.pos_rad;
        start_ns_ = fb.t_ns;
        st_ = St::Settling;
      } else if (timeout) {
        fail("coarse approach: timeout with no contact");
      } else if (at_limit) {
        fail("coarse approach: reached travel limit with no contact");
      }
      break;
    }
    case St::Settling:
      if ((fb.t_ns - start_ns_) >= static_cast<TimeNs>(p_.settle_time_s * 1e9)) {
        complete_ = true;
        st_ = St::Complete;
      }
      break;
    case St::Complete:
    case St::Failed:
      break;
  }

  if (st_ == St::Approach) {
    // Drive toward a far point in the approach direction; the axis actually stops
    // on contact (or the timeout/travel limit), well short of this target. The
    // speed is a positive magnitude; the direction is implied by the target.
    const double target = fb.pos_rad + p_.max_travel_rad * dir_;
    // The velocity-mode executor commands ds.velocity_rad_s, so it must carry
    // the SIGNED approach speed (magnitude * direction), not just the magnitude.
    const double v = p_.coarse_speed_rad_s * static_cast<double>(dir_);
    return DesiredState{target, p_.coarse_speed_rad_s, v, false, "coarse approach to stop"};
  }
  if (st_ == St::Settling) {
    return DesiredState{fb.pos_rad, 0.0, 0.0, true, "settling after coarse contact"};
  }
  if (st_ == St::Complete) {
    return DesiredState{contact_rad_, 0.0, 0.0, true, "coarse endpoint homed"};
  }
  return DesiredState{fb.pos_rad, 0.0, 0.0, true, "coarse home failed: " + fail_reason_};
}

// ---------------------------------------------------------------------------
// HomingPlan
// ---------------------------------------------------------------------------
HomingPlan::HomingPlan(std::vector<HomingAction> actions, HomingPlanConfig cfg)
    : actions_(std::move(actions)), cfg_(std::move(cfg)) {
  if (!actions_.empty()) start_action();
}

void HomingPlan::fail(const std::string& reason) {
  failed_ = true;
  fail_reason_ = reason;
}

std::string HomingPlan::describe_action() const {
  if (idx_ < 0 || idx_ >= static_cast<int>(actions_.size())) return "unknown action";
  const HomingAction& a = actions_[idx_];
  std::string s = axis_name(a.axis);
  s += " ";
  switch (a.type) {
    case HomingActionType::HomeEndpoint:
      s += (a.endpoint == Endpoint::Lower ? "home lower endpoint " : "home upper endpoint ");
      s += (a.precision == Precision::Coarse ? "(coarse)" : "(fine)");
      break;
    case HomingActionType::HomeFullRange:
      s += "home full range";
      break;
    case HomingActionType::Move:
      s += "move to " + std::to_string(a.position_deg) + " deg";
      break;
  }
  return s;
}

void HomingPlan::start_action() {
  const HomingAction& a = actions_[idx_];
  active_type_ = a.type;
  active_precision_ = a.precision;
  coarse_.reset();
  home_.reset();
  full_.reset();
  move_.reset();

  const size_t i = ix(a.axis);
  // Per-axis initial drive current (adaptive-current homing, §22): apply it to
  // the shared params for this action's axis (0 = keep the HomingParams default).
  HomingParams axis_homing = cfg_.homing;
  if (cfg_.limit_cur_initial_a[i] > 0.0) {
    axis_homing.limit_cur_initial_a = cfg_.limit_cur_initial_a[i];
  }
  switch (a.type) {
    case HomingActionType::HomeEndpoint: {
      const int dir = (a.endpoint == Endpoint::Lower) ? -1 : +1;
      if (a.precision == Precision::Coarse) {
        coarse_.emplace(a.axis, dir, axis_homing);
      } else {
        home_.emplace(a.axis, dir, axis_homing);
      }
      break;
    }
    case HomingActionType::HomeFullRange: {
      FullAxisHomingParams fp;
      fp.homing = axis_homing;
      fp.dir_endpoint_a = +1;
      fp.dir_endpoint_b = -1;
      const TravelBand& band = cfg_.travel_bands[i];
      const double expected = band.max_deg - band.min_deg;
      fp.expected_travel_min_deg = 0.5 * expected;
      fp.expected_travel_max_deg = 1.5 * expected;
      full_.emplace(a.axis, fp);
      break;
    }
    case HomingActionType::Move: {
      const AxisPlanState& st = states_[i];
      if (!st.model.has_reference) {
        fail(std::string(axis_name(a.axis)) +
             " move: axis not referenced (no endpoint homed), cannot resolve logical target");
        return;
      }
      const double raw = st.model.logical_to_raw_rad(a.position_deg);
      move_.emplace(a.axis, raw, cfg_.move_speed_rad_s, cfg_.move_pos_tol_rad,
                    cfg_.move_vel_tol_rad_s, cfg_.move_timeout_s);
      break;
    }
  }
}

void HomingPlan::advance() {
  ++idx_;
  if (idx_ >= static_cast<int>(actions_.size())) {
    complete_ = true;
  } else {
    start_action();
  }
}

void HomingPlan::record_endpoint(AxisId a, Endpoint e, double raw_rad, double rep_rad) {
  AxisPlanState& st = states_[ix(a)];
  if (rep_rad > st.repeatability_rad) st.repeatability_rad = rep_rad;
  if (e == Endpoint::Lower) {
    st.have_low = true;
    st.raw_low_rad = raw_rad;
  } else {
    st.have_high = true;
    st.raw_high_rad = raw_rad;
  }
  // Establish a reference as soon as any endpoint is known so a subsequent
  // clearance Move can resolve a logical target. A later full home (both
  // endpoints) refines this to the canonical model (low endpoint = logical 0).
  if (!st.model.has_reference) {
    st.model.direction_sign = 1;
    st.model.set_reference(raw_rad, 0.0);
  }
}

void HomingPlan::finalize_axis(AxisId a) {
  AxisPlanState& st = states_[ix(a)];
  if (st.finalized || !st.have_low || !st.have_high) return;
  st.finalized = true;
  const double span =
      setup_model_from_endpoints(st.model, st.raw_low_rad, st.raw_high_rad);
  const TravelBand& band = cfg_.travel_bands[ix(a)];
  const double expected = band.max_deg - band.min_deg;
  if (expected > 0.0) {
    const double lo = 0.5 * expected;
    const double hi = 1.5 * expected;
    if (span < lo || span > hi) {
      st.homed = false;
      st.span_issue = "measured travel " + std::to_string(span) +
                      " deg outside expected [" + std::to_string(lo) + ", " +
                      std::to_string(hi) + "]";
      return;
    }
  }
  st.homed = true;
}

void HomingPlan::record_action_result() {
  const HomingAction& a = actions_[idx_];
  switch (a.type) {
    case HomingActionType::HomeEndpoint: {
      double raw = 0.0;
      double rep = 0.0;
      if (coarse_) {
        raw = coarse_->contact_rad();
      } else {
        raw = home_->result().fine_contact_rad;
        rep = home_->result().repeatability_rad;
      }
      record_endpoint(a.axis, a.endpoint, raw, rep);
      finalize_axis(a.axis);
      AxisPlanState& st = states_[ix(a.axis)];
      if (st.finalized && !st.homed) {
        fail(describe_action() + " failed: " + st.span_issue);
      }
      break;
    }
    case HomingActionType::HomeFullRange: {
      const FullAxisHomingResult& r = full_->result();
      AxisPlanState& st = states_[ix(a.axis)];
      st.have_low = true;
      st.have_high = true;
      st.finalized = true;
      st.raw_low_rad = r.endpoint_b_rad;   // approached in the - direction
      st.raw_high_rad = r.endpoint_a_rad;  // approached in the + direction
      st.repeatability_rad = r.repeatability_rad;
      st.model = r.model;
      st.homed = r.valid;
      if (!r.valid) {
        st.span_issue = r.fail_reason;
        fail(describe_action() + " failed: " + r.fail_reason);
      }
      break;
    }
    case HomingActionType::Move:
      break;  // clearance move: nothing to record
  }
}

DesiredState HomingPlan::step(const HomingFeedback& fb) {
  if (complete_) return DesiredState{0.0, 0.0, 0.0, true, "homing plan complete"};
  if (failed_)
    return DesiredState{0.0, 0.0, 0.0, true, "homing plan failed: " + fail_reason_};

  DesiredState ds;
  bool sub_terminal = false;
  bool sub_ok = false;
  std::string sub_reason;
  switch (active_type_) {
    case HomingActionType::HomeEndpoint:
      if (coarse_) {
        ds = coarse_->step(fb);
        sub_terminal = coarse_->terminal();
        sub_ok = coarse_->valid();
        sub_reason = sub_ok ? "endpoint homed (coarse)" : coarse_->fail_reason();
      } else {
        ds = home_->step(fb);
        sub_terminal = home_->terminal();
        sub_ok = home_->result().valid;
        sub_reason = sub_ok ? "endpoint homed (fine)" : home_->result().fail_reason;
      }
      break;
    case HomingActionType::HomeFullRange:
      ds = full_->step(fb);
      sub_terminal = full_->terminal();
      sub_ok = full_->result().valid;
      sub_reason = sub_ok ? "full range homed" : full_->result().fail_reason;
      break;
    case HomingActionType::Move:
      ds = move_->step(fb);
      sub_terminal = move_->terminal();
      sub_ok = move_->ok();
      sub_reason = move_->reason();
      break;
  }

  if (sub_terminal) {
    if (sub_ok) {
      record_action_result();
      if (!failed_) advance();
    } else {
      fail(describe_action() + " failed: " + sub_reason);
    }
  }
  return ds;
}

AxisId HomingPlan::active_axis() const {
  if (idx_ < 0 || idx_ >= static_cast<int>(actions_.size())) return AxisId::Pitch;
  return actions_[idx_].axis;
}

}  // namespace ota
