// OpenAutoTurret — safe park / shutdown controller (architecture §33).
#include "calibration/park_controller.hpp"

namespace ota {

ParkController::ParkController(ParkParams p,
                               const std::array<AxisLimits, kAxisCount>& limits,
                               const std::array<AxisLogicalModel, kAxisCount>& models)
    : p_(std::move(p)), dwell_ns_(static_cast<TimeNs>(p_.dwell_ms * 1e6)) {
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
    if (!limits[i].valid) {
      fail(std::string(axis_name(a)) + " park: axis limits not valid (not homed)");
      return;
    }
    const double raw = models[i].logical_to_raw_rad(p_.park_logical_deg[i]);
    park_raw_[i] = raw;
    const double lo = limits[i].q_soft_min_rad + margin_rad;
    const double hi = limits[i].q_soft_max_rad - margin_rad;
    if (raw <= lo || raw >= hi) {
      fail(std::string(axis_name(a)) +
           " park: park position not strictly inside the soft limit with margin "
           "(§33.1)");
      return;
    }
  }
  // Prepare the first park move (yaw, per the §33 sequence).
  yaw_move_.emplace(AxisId::Yaw, park_raw_[ix(AxisId::Yaw)], p_.speed_deg_s * kDeg2Rad,
                    p_.move_pos_tol_rad, p_.move_vel_tol_rad_s, p_.move_timeout_s);
}

ParkOutput ParkController::step(const HomingFeedback& pitch_fb,
                                const HomingFeedback& yaw_fb) {
  ParkOutput out;
  out.pitch = hold(pitch_fb);
  out.yaw = hold(yaw_fb);

  switch (state_) {
    case ParkState::StopTracking:
      // Phase 2: there is no tracking/search to stop (the caller has already
      // disabled it). Hold for one cycle, then begin the park moves.
      out.message = "stop tracking (no-op in phase 2)";
      state_ = ParkState::MoveYaw;
      break;

    case ParkState::MoveYaw:
      out.yaw = yaw_move_->step(yaw_fb);
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
      out.message = "move pitch to park";
      if (pitch_move_->terminal()) {
        if (pitch_move_->ok()) {
          state_ = ParkState::Verify;
        } else {
          fail("pitch park move failed: " + pitch_move_->reason());
        }
      }
      break;

    case ParkState::Verify:
      out.message = "verify park pose";
      if (at_park(pitch_fb, AxisId::Pitch) && at_park(yaw_fb, AxisId::Yaw)) {
        dwell_start_ns_ = pitch_fb.t_ns;
        state_ = ParkState::Dwell;
      }
      break;

    case ParkState::Dwell: {
      out.message = "park dwell";
      const bool still = at_park(pitch_fb, AxisId::Pitch) && at_park(yaw_fb, AxisId::Yaw);
      if (!still) {
        state_ = ParkState::Verify;  // drifted — re-verify before de-energizing
        break;
      }
      if (pitch_fb.t_ns - dwell_start_ns_ >= dwell_ns_) {
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
      state_ = ParkState::Parked;
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
  return out;
}

}  // namespace ota
