// OpenAutoTurret — safe park / shutdown controller (architecture §33).
#include "calibration/park_controller.hpp"
#include <algorithm>

namespace ota {

ParkController::ParkController(ParkParams p,
                               const std::array<AxisLimits, kAxisCount>& limits,
                               const std::array<AxisLogicalModel, kAxisCount>& models)
    : p_(std::move(p)), dwell_ns_(static_cast<TimeNs>(p_.dwell_ms * 1e6)) {
  if (!std::isfinite(p_.pos_tol_deg) || p_.pos_tol_deg <= 0 ||
      !std::isfinite(p_.vel_tol_deg_s) || p_.vel_tol_deg_s <= 0 ||
      !std::isfinite(p_.min_observed_travel_deg) || p_.min_observed_travel_deg <= 0 ||
      !std::isfinite(p_.speed_deg_s) || p_.speed_deg_s <= 0 ||
      !std::isfinite(p_.verify_speed_deg_s) || p_.verify_speed_deg_s <= 0 ||
      !std::isfinite(p_.min_soft_margin_deg) || p_.min_soft_margin_deg <= 0 ||
      !std::isfinite(p_.end_clearance_deg) || p_.end_clearance_deg <= 0 ||
      p_.dwell_ms <= 0 || p_.evidence_max_age_ms <= 0) {
    fail("invalid parking verification parameters");
    return;
  }
  // Validate every axis BEFORE committing to the sequence (§33.1). The park pose
  // must be strictly inside the calibrated soft limit with a margin — never
  // directly against a mechanical stop.
  const double margin_rad = p_.min_soft_margin_deg * kDeg2Rad;
  for (int i = 0; i < kAxisCount; ++i) {
    const AxisId a = static_cast<AxisId>(i);
    if (!models[i].has_reference) {
      fail(std::string(axis_name(a)) + " park: axis not referenced (not homed)");
      return;
    }
    if (!limits[i].valid || !std::isfinite(limits[i].q_soft_min_rad) ||
        !std::isfinite(limits[i].q_soft_max_rad) ||
        limits[i].q_soft_min_rad >= limits[i].q_soft_max_rad) {
      fail(std::string(axis_name(a)) + " park: axis limits not valid (not homed)");
      return;
    }
    double raw = models[i].logical_to_raw_rad(p_.park_logical_deg[i]);
    const auto& mode = p_.target_mode[i];
    if (mode == "soft_center")
      raw = .5 * (limits[i].q_soft_min_rad + limits[i].q_soft_max_rad);
    else if (mode == "soft_min")
      raw = limits[i].q_soft_min_rad + p_.end_clearance_deg * kDeg2Rad;
    else if (mode == "soft_max")
      raw = limits[i].q_soft_max_rad - p_.end_clearance_deg * kDeg2Rad;
    else if (mode != "logical_degrees") {
      fail(std::string(axis_name(a)) + " park: unknown target mode '" + mode + "'");
      return;
    }
    park_raw_[i] = raw;
    const double lo = limits[i].q_soft_min_rad + margin_rad;
    const double hi = limits[i].q_soft_max_rad - margin_rad;
    if (!std::isfinite(raw) || raw <= lo || raw >= hi) {
      fail(std::string(axis_name(a)) +
           " park: park position not strictly inside the soft limit with margin "
           "(§33.1)");
      return;
    }
  }
  // At 3 deg/s a full-travel return can exceed the old fixed 30 s limit.
  // Budget each move for the calibrated travel plus settling, still bounded.
  for (int i = 0; i < kAxisCount; ++i)
    p_.move_timeout_s = std::max(p_.move_timeout_s, 10.0 +
        1.5 * (limits[i].q_soft_max_rad - limits[i].q_soft_min_rad) /
        (p_.speed_deg_s * kDeg2Rad));
  // Prepare the first park move (yaw, per the §33 sequence).
  // Move arrival must fit inside the stricter release window as well.
  p_.move_pos_tol_rad = std::min(p_.move_pos_tol_rad, .25*p_.pos_tol_deg*kDeg2Rad);
  yaw_move_.emplace(AxisId::Yaw, park_raw_[ix(AxisId::Yaw)], p_.speed_deg_s * kDeg2Rad,
                    p_.move_pos_tol_rad, p_.move_vel_tol_rad_s, p_.move_timeout_s);
}

bool ParkController::release_gate(const HomingFeedback& pitch_fb,
    const HomingFeedback& yaw_fb,
    const std::array<ParkPositionEvidence, kAxisCount>& evidence, TimeNs now_ns) {
  const HomingFeedback feedback[] = {pitch_fb, yaw_fb};
  for (int i = 0; i < kAxisCount; ++i) {
    const auto& fb = feedback[i];
    const auto& ev = evidence[i];
    std::string reason;
    const auto age = now_ns - fb.t_ns;
    const auto independent_age = now_ns - ev.sampled_ns;
    const double tolerance = .5 * p_.pos_tol_deg * kDeg2Rad;
    if (fb.motor_fault || !std::isfinite(fb.pos_rad) || !std::isfinite(fb.vel_rad_s) ||
        fb.t_ns <= 0 || age < 0 || age > p_.evidence_max_age_ms * 1'000'000LL)
      reason = "stale or untrusted motor feedback";
    else if (observed_travel_[i] < p_.min_observed_travel_deg * kDeg2Rad)
      reason = "no parking motion observed; no automatic release";
    else if (!at_park(fb, static_cast<AxisId>(i)))
      reason = "position/velocity outside guarded park tolerance";
    else if (!ev.trusted || ev.sampled_ns <= 0 || independent_age < 0 ||
             independent_age > p_.evidence_max_age_ms * 1'000'000LL ||
             !std::isfinite(ev.q_raw_rad) || !std::isfinite(ev.uncertainty_rad) ||
             ev.uncertainty_rad < 0)
      reason = "independent physical position confirmation unavailable or stale";
    else if (std::abs(ev.q_raw_rad - park_raw_[i]) + ev.uncertainty_rad >= tolerance)
      reason = "independent position outside guarded park tolerance";
    else if (independent_travel_[i] < p_.min_observed_travel_deg*kDeg2Rad)
      reason = "no independent parking motion observed; no automatic release";
    if (!reason.empty()) {
      fail(std::string(axis_name(static_cast<AxisId>(i))) + " park: " + reason);
      return false;
    }
  }
  return true;
}

ParkOutput ParkController::step(const HomingFeedback& pitch_fb,
    const HomingFeedback& yaw_fb,
    const std::array<ParkPositionEvidence, kAxisCount>& evidence, TimeNs now_ns) {
  if (now_ns == 0) now_ns = std::max(pitch_fb.t_ns, yaw_fb.t_ns);
  ParkOutput out;
  out.pitch = hold(pitch_fb);
  out.yaw = hold(yaw_fb);
  const double q[] = {pitch_fb.pos_rad, yaw_fb.pos_rad};
  const int moving_axis = state_ == ParkState::MoveYaw ? ix(AxisId::Yaw) :
                          state_ == ParkState::MovePitch ? ix(AxisId::Pitch) : -1;
  if (moving_axis >= 0) {
    const auto i = static_cast<size_t>(moving_axis);
    if (!observed_initial_[i]) {
      initial_q_[i] = q[i];
      observed_initial_[i] = true;
    }
    const double direction = park_raw_[i] >= initial_q_[i] ? 1 : -1;
    observed_travel_[i] = std::max(observed_travel_[i], direction * (q[i]-initial_q_[i]));
    const auto& ev = evidence[i];
    if (ev.trusted && ev.sampled_ns > 0 && ev.sampled_ns <= now_ns &&
        now_ns-ev.sampled_ns <= p_.evidence_max_age_ms*1'000'000LL &&
        std::isfinite(ev.q_raw_rad) && std::isfinite(ev.uncertainty_rad) && ev.uncertainty_rad >= 0) {
      if (!independent_initial_valid_[i]) {
        independent_initial_valid_[i] = true;
        independent_initial_q_[i] = ev.q_raw_rad;
        independent_initial_uncertainty_[i] = ev.uncertainty_rad;
      }
      independent_travel_[i] = std::max(independent_travel_[i],
          direction*(ev.q_raw_rad-independent_initial_q_[i]) - ev.uncertainty_rad -
          independent_initial_uncertainty_[i]);
    }
  }

  if (state_ >= ParkState::Verify && state_ <= ParkState::VerifyDisabled)
    release_gate(pitch_fb, yaw_fb, evidence, now_ns);

  switch (state_) {
    case ParkState::StopTracking:
      // Phase 2: there is no tracking/search to stop (the caller has already
      // disabled it). Hold for one cycle, then begin the park moves. Still in
      // speed mode (both axes at SpdRef=0): position mode is entered only at
      // Verify, when the §33.2 target-hold begins — the executor enters it on
      // the first !speed_mode cycle, so a premature position-mode entry here
      // would swallow the SpdRef park moves.
      out.speed_mode = true;
      out.message = "stop tracking (no-op in phase 2)";
      state_ = ParkState::MoveYaw;
      break;

    case ParkState::MoveYaw:
      out.yaw = yaw_move_->step(yaw_fb);
      out.yaw.velocity_rad_s = std::clamp(out.yaw.velocity_rad_s,
          -2.0 * std::abs(park_raw_[ix(AxisId::Yaw)] - yaw_fb.pos_rad),
           2.0 * std::abs(park_raw_[ix(AxisId::Yaw)] - yaw_fb.pos_rad));
      out.speed_mode = true;  // SpdRef-driven move (velocity_rad_s, signed)
      out.message = "move yaw to park";
      if (yaw_move_->terminal()) {
        if (yaw_move_->ok()) {
          state_ = ParkState::MovePitch;
        } else {
          fail("yaw park move failed: " + yaw_move_->reason());
        }
      }
      break;

    case ParkState::MovePitch:
      if (!pitch_move_) {
        pitch_move_.emplace(AxisId::Pitch, park_raw_[ix(AxisId::Pitch)],
                            p_.speed_deg_s * kDeg2Rad, p_.move_pos_tol_rad,
                            p_.move_vel_tol_rad_s, p_.move_timeout_s);
      }
      out.pitch = pitch_move_->step(pitch_fb);
      out.pitch.velocity_rad_s = std::clamp(out.pitch.velocity_rad_s,
          -2.0 * std::abs(park_raw_[ix(AxisId::Pitch)] - pitch_fb.pos_rad),
           2.0 * std::abs(park_raw_[ix(AxisId::Pitch)] - pitch_fb.pos_rad));
      out.speed_mode = true;  // SpdRef-driven move (velocity_rad_s, signed)
      out.message = "move pitch to park";
      if (pitch_move_->terminal()) {
        if (pitch_move_->ok()) {
          state_ = ParkState::Verify;
        } else {
          fail("pitch park move failed: " + pitch_move_->reason());
        }
      }
      break;

    case ParkState::Verify: {
      // POSITION-MODE HOLD AT THE PARK TARGET (not at the current position):
      // the drive's position loop pulls the axis back to the target while the
      // §33.2 check runs. Re-pinning to the current position (the old
      // behavior, shared with the ready-hold) has NO outer correction — at
      // the real station the yaw gravity balance sits ~4 deg off the 180 deg
      // park pose, so the axis drifted back out of the 0.5 deg window and the
      // park timed out at the 40 s shutdown window (rehome4; the yaw also
      // de-energized 1.4 deg short of target, rehome1 3.96 deg short).
      // The hold carries a NON-ZERO speed limit: the position loop needs a
      // non-zero LimitSpd to be able to pull an axis back to the target
      // (p3: a 0-limit hold pinned the overshoot in place for 40 s).
      out.pitch = DesiredState{park_raw_[ix(AxisId::Pitch)],
                               p_.verify_speed_deg_s * kDeg2Rad, 0.0, true,
                               "hold at park target"};
      out.yaw = DesiredState{park_raw_[ix(AxisId::Yaw)],
                             p_.verify_speed_deg_s * kDeg2Rad, 0.0, true,
                             "hold at park target"};
      out.message = "verify park pose";
      if (at_park(pitch_fb, AxisId::Pitch) && at_park(yaw_fb, AxisId::Yaw)) {
        dwell_start_ns_ = now_ns;
        state_ = ParkState::Dwell;
      }
      break;
    }

    case ParkState::Dwell: {
      // Same target hold as Verify: keep pulling at the park pose for the
      // dwell duration (drift back to Verify if it leaves the window).
      out.pitch = DesiredState{park_raw_[ix(AxisId::Pitch)],
                               p_.verify_speed_deg_s * kDeg2Rad, 0.0, true,
                               "hold at park target"};
      out.yaw = DesiredState{park_raw_[ix(AxisId::Yaw)],
                             p_.verify_speed_deg_s * kDeg2Rad, 0.0, true,
                             "hold at park target"};
      out.message = "park dwell";
      const bool still = at_park(pitch_fb, AxisId::Pitch) && at_park(yaw_fb, AxisId::Yaw);
      if (!still) {
        state_ = ParkState::Verify;  // drifted — re-verify before de-energizing
        break;
      }
      if (now_ns - dwell_start_ns_ >= dwell_ns_) {
        state_ = ParkState::DisablePitch;
      }
      break;
    }

    case ParkState::DisablePitch:
      out.disable_pitch = true;
      out.message = "disable pitch";
      state_ = ParkState::DisableYaw;
      break;

    case ParkState::DisableYaw:
      out.disable_yaw = true;
      out.message = "disable yaw";
      dwell_start_ns_ = now_ns;
      state_ = ParkState::VerifyDisabled;
      break;

    case ParkState::VerifyDisabled:
      out.message = "verify independent position after disable";
      if (now_ns - dwell_start_ns_ >= dwell_ns_) state_ = ParkState::Parked;
      break;

    case ParkState::Parked:
      out.complete = true;
      out.message = "parked (power-safe)";
      break;

    case ParkState::Failed:
      out.failed = true;
      out.message = "park failed: " + fail_reason_;
      break;
  }
  if (state_ == ParkState::Failed) {
    out.failed = true;
    out.message = "park failed: " + fail_reason_;
    out.disable_pitch = out.disable_yaw = out.complete = false;
  }
  return out;
}

}  // namespace ota
