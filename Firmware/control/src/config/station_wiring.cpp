#include "config/station_wiring.hpp"

#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"
#include "config/turret_config.hpp"
#include "control/control_loop.hpp"

namespace ota {
namespace wire {
namespace {

namespace detail {
// Moved out of main.cpp so that the daemon and the replay tool cannot be
// two different readings of the same file. Bodies are unchanged; the comment
// above came with them.

HomingAction parse_action(const config::HomingPlanActionConfig& c,
                          std::string& err) {
  HomingAction a;
  if (c.axis == "pitch") a.axis = AxisId::Pitch;
  else if (c.axis == "yaw") a.axis = AxisId::Yaw;
  else {
    err = "homing_plan: unknown axis '" + c.axis + "'";
    return a;
  }
  if (c.action == "home_full_range") {
    a.type = HomingActionType::HomeFullRange;
  } else if (c.action == "home_endpoint") {
    a.type = HomingActionType::HomeEndpoint;
    a.endpoint = (c.endpoint == "upper") ? Endpoint::Upper : Endpoint::Lower;
  } else if (c.action == "move") {
    a.type = HomingActionType::Move;
  } else {
    err = "homing_plan: unknown action '" + c.action + "'";
    return a;
  }
  a.precision = (c.precision == "coarse") ? Precision::Coarse : Precision::Fine;
  a.position_deg = c.position_deg;
  return a;
}

HomingPlan make_homing_plan(const config::TurretConfig& cfg, std::string& err) {
  HomingParams hp;  // start from the safe defaults...
  const config::ContactConfig& cc = cfg.homing.contact;  // ...override from YAML
  if (cc.coarse_speed_deg_s > 0)
    hp.coarse_speed_rad_s = cc.coarse_speed_deg_s * kDeg2Rad;
  if (cc.fine_speed_deg_s > 0)
    hp.fine_speed_rad_s = cc.fine_speed_deg_s * kDeg2Rad;
  if (cc.stall_velocity_threshold > 0)
    hp.contact.v_stall_threshold_rad_s = cc.stall_velocity_threshold;
  if (cc.v_move_threshold > 0)
    hp.contact.v_move_threshold_rad_s = cc.v_move_threshold;
  if (cc.current_or_effort_limit > 0)
    hp.contact.effort_contact_threshold_nm = cc.current_or_effort_limit;
  if (cc.effort_hard_contact_nm > 0)
    hp.contact.effort_hard_contact_nm = cc.effort_hard_contact_nm;
  if (cc.motion_history_velocity > 0)
    hp.contact.motion_history_vel_rad_s = cc.motion_history_velocity;
  if (cc.contact_dwell_ms > 0) hp.contact.contact_dwell_ms = cc.contact_dwell_ms;
  if (cc.backoff_deg > 0) hp.backoff_rad = cc.backoff_deg * kDeg2Rad;
  if (cc.repeatability_deg > 0)
    hp.repeatability_rad = cc.repeatability_deg * kDeg2Rad;
  if (cc.repeatability_retries > 0)
    hp.repeatability_retries = cc.repeatability_retries;
  // Adaptive-current homing (push-through, §22). Shared across axes.
  if (cc.limit_cur_step_a > 0) hp.limit_cur_step_a = cc.limit_cur_step_a;
  if (cc.limit_cur_max_a > 0) hp.limit_cur_max_a = cc.limit_cur_max_a;
  if (cc.max_rotation_deg > 0) hp.max_rotation_rad = cc.max_rotation_deg * kDeg2Rad;
  if (cc.torque_safety_nm > 0) hp.torque_safety_nm = cc.torque_safety_nm;

  HomingPlanConfig hpc;
  hpc.homing = hp;
  for (int i = 0; i < kAxisCount; ++i) {
    hpc.travel_bands[i].min_deg = cfg.axes[i].expected_travel_deg.min;
    hpc.travel_bands[i].max_deg = cfg.axes[i].expected_travel_deg.max;
    hpc.limit_cur_initial_a[i] = cfg.axes[i].limit_cur_a;
  }

  std::vector<HomingAction> actions;
  for (const auto& c : cfg.homing_plan.actions) {
    std::string e;
    actions.push_back(parse_action(c, e));
    if (!e.empty()) {
      err = e;
      return HomingPlan({}, hpc);
    }
  }
  if (actions.empty()) {
    // No explicit plan: full-range home both axes (pitch, then yaw).
    actions.push_back({HomingActionType::HomeFullRange, AxisId::Pitch,
                       Endpoint::Lower, Precision::Fine, 0.0});
    actions.push_back({HomingActionType::HomeFullRange, AxisId::Yaw,
                       Endpoint::Lower, Precision::Fine, 0.0});
  }
  return HomingPlan(std::move(actions), hpc);
}

ControlLoop::Config make_control_cfg(const config::TurretConfig& cfg) {
  ControlLoop::Config c;
  c.control_hz = cfg.control_loop_hz;
  // §72: the named values. Each is passed through only when the file named it; an
  // omitted value stays zero/false so the loop keeps deriving it from what the station
  // measured. Degrees here, radians in the loop — the same convention the rest of this
  // function obeys, and the reason the conversion happens here rather than in the file.
  c.roam_region_named = cfg.v3.has_roam_region;
  c.roam_yaw_min_deg = cfg.v3.roam_yaw_min_deg;
  c.roam_yaw_max_deg = cfg.v3.roam_yaw_max_deg;
  c.roam_pitch_named = cfg.v3.has_roam_pitch;
  c.roam_pitch_deg = cfg.v3.roam_pitch_deg;
  c.roam_velocity_deg_s = cfg.v3.roam_velocity_deg_s;
  c.auto_track_coast_ms = cfg.v3.auto_track_coast_ms;
  c.auto_track_lost_hold_ms = cfg.v3.auto_track_lost_hold_ms;
  c.auto_track_reacquire_window_ms = cfg.v3.auto_track_reacquire_window_ms;
  c.auto_track_medium_min = cfg.v3.auto_track_medium_min;
  c.auto_track_high_min = cfg.v3.auto_track_high_min;
  c.auto_track_medium_scale = cfg.v3.auto_track_medium_scale;
  c.auto_track_low_scale = cfg.v3.auto_track_low_scale;
  c.reacquire_threshold = cfg.v3.reacquire_threshold;
  c.ambiguous_match_margin = cfg.v3.ambiguous_match_margin;
  c.manual_lease_ms = cfg.v3.jog_lease_ms;
  c.manual_keepalive_ms = cfg.v3.jog_keepalive_ms;
  c.step_sizes_deg = cfg.v3.step_sizes_deg;
  c.feedback_max_age_ms = cfg.safety.feedback_max_age_ms;
  c.deadline_max_us = cfg.safety.deadline_max_us;
  c.deadline_miss_threshold = cfg.safety.deadline_miss_threshold;
  c.motor_overtemp_c = cfg.safety.motor_overtemp_c;
  c.soft_margin_rad = cfg.axes[0].soft_margin_deg * kDeg2Rad;
  c.park.park_logical_deg[0] = cfg.shutdown.pitch_park_deg;
  c.park.park_logical_deg[1] = cfg.shutdown.yaw_park_deg;
  c.park.pos_tol_deg = cfg.shutdown.pos_tolerance_deg;
  c.park.vel_tol_deg_s = cfg.shutdown.vel_tolerance_deg_s;
  c.park.dwell_ms = cfg.shutdown.dwell_ms;
  c.park.speed_deg_s = cfg.shutdown.speed_deg_s;
  // Verify/Dwell position-mode hold speed limit (deg/s). Non-zero: LimitSpd=0
  // pins the drive's position loop (p3 2026-09-02: 40 s stall at the
  // overshoot point, the 0-limit hold could not pull the axis back).
  c.park.verify_speed_deg_s = cfg.shutdown.verify_speed_deg_s;
  // Park moves run in speed mode: reuse the per-axis homing current limits
  // (pitch 3 A / yaw 1 A — under the 10 A safe cap) as the drive LimitCur for
  // the park moves.
  c.park.limit_cur_a[0] = cfg.axes[0].limit_cur_a > 0.0 ? cfg.axes[0].limit_cur_a
                                                        : c.park.limit_cur_a[0];
  c.park.limit_cur_a[1] = cfg.axes[1].limit_cur_a > 0.0 ? cfg.axes[1].limit_cur_a
                                                        : c.park.limit_cur_a[1];
  // Phase 9: payload verification (§27, §31.3).
  c.payload_auto_verify = cfg.payload.auto_verify;
  c.payload_check_region_half_span_deg =
      cfg.payload.check_region_half_span_deg;
  c.payload_check_current_a = cfg.payload.check_current_a;
  c.payload_check_spd_kp = cfg.payload.check_spd_kp;
  c.payload_check_spd_ki = cfg.payload.check_spd_ki;
  // §49 search sweep half-span (the loop clamps it inside the homed soft
  // limits when tracking is enabled).
  c.search_span_rad = cfg.tracking.search_span_deg * kDeg2Rad;
  return c;
}
}

}  // namespace

HomingPlan make_homing_plan(const config::TurretConfig& cfg, std::string& err) {
  return detail::make_homing_plan(cfg, err);
}

ControlLoop::Config make_control_cfg(const config::TurretConfig& cfg) {
  return detail::make_control_cfg(cfg);
}

}  // namespace wire
}  // namespace ota
