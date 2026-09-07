#pragma once
// Shared boot/configuration boundary, used by controld and offline integration tests.
#include "config/turret_config.hpp"
#include "calibration/camera_calibration.hpp"
#include "control/tracking_controller.hpp"

namespace ota::config {

struct TrackingSetupDiagnostics {
  IntrinsicsLoad intrinsics;
  std::string extrinsics;
};

inline TrackingController::Config make_tracking_config(const TurretConfig& cfg,
                                                       TrackingSetupDiagnostics* diagnostics = nullptr) {
  TrackingController::Config t;
  t.estimator.alpha = cfg.tracking.estimator_alpha;
  t.estimator.beta = cfg.tracking.estimator_beta;
  t.estimator.use_kalman = cfg.tracking.estimator_model == "constant_velocity";
  t.estimator.measurement_sigma_rad = cfg.tracking.estimator_measurement_sigma_rad;
  t.estimator.angular_accel_sigma_rad_s2 = cfg.tracking.estimator_accel_sigma_rad_s2;
  t.fsm.coast_max_ns = static_cast<int64_t>(cfg.tracking.coast_timeout_ms)*1000000;
  t.fsm.lost_ns = static_cast<int64_t>(cfg.tracking.lost_timeout_ms)*1000000;
  t.fsm.search_enabled = cfg.tracking.search_enabled_by_default || cfg.tracking.target_lost_behavior == "search";
  t.search.v_max_rad_s = cfg.tracking.search_speed_deg_s*kDeg2Rad;
  t.track_v_max_rad_s = cfg.tracking.track_speed_deg_s*kDeg2Rad;
  t.search_v_max_rad_s = cfg.tracking.search_speed_deg_s*kDeg2Rad;
  t.hold_v_max_rad_s = cfg.tracking.hold_speed_deg_s*kDeg2Rad;
  t.control_delay_ns = static_cast<int64_t>(cfg.tracking.control_delay_ms)*1000000;
  t.motor_response_ns = static_cast<int64_t>(cfg.tracking.motor_response_ms)*1000000;
  t.fresh_threshold_ns = static_cast<int64_t>(cfg.tracking.fresh_threshold_ms)*1000000;
  const auto intrinsics = load_camera_intrinsics(cfg.camera.intrinsics_file);
  if (intrinsics.found) {
    t.intrinsics = intrinsics.intrinsics;
    t.aim = cfg.tracking.aim_point;
    t.aim.aim_at_head = cfg.tracking.aim_at_head;
    t.aim.head_fraction_from_top = cfg.tracking.head_fraction_from_top;
  } else if (cfg.alignment.enabled || cfg.tracking.aim_point.mode != tracking::AimMode::Legacy) {
    throw std::invalid_argument("explicit aim/alignment requires valid camera intrinsics: " + intrinsics.detail);
  }
  std::string detail;
  t.kinematics = load_camera_extrinsics(cfg.camera.extrinsics_file, detail);
  if (diagnostics) *diagnostics = {intrinsics, detail};
  if (cfg.alignment.enabled && detail != "R_P_C loaded from file")
    throw std::invalid_argument("alignment requires valid camera extrinsics: " + detail);
  t.alignment = cfg.alignment;
  const auto alignment = geo::laser_alignment(t.alignment, t.intrinsics);
  if (alignment.enabled && !alignment.valid) throw std::invalid_argument(alignment.reason);
  return t;
}

}  // namespace ota::config
