// OpenAutoTurret — tracking controller (architecture §13-§16, §34, §36, §46).
//
// Integrates the tracking subsystem that sits ABOVE the safety envelope and
// trajectory generator in the §15 hierarchy:
//
//   target measurement -> (timestamp-aligned) base-frame LOS -> estimator ->
//   predicted LOS -> LOS->joint solver -> reference manager -> reference
//
// It owns:
//   * the TargetEstimator (§13) fed by timestamp-aligned LOS measurements
//     (motor pose interpolated at the capture time, §11);
//   * the TrackingStateMachine (§34) + confidence decay (§35);
//   * the SearchPlanner (§36, §49);
//   * the ReferenceManager (§16) + LosJointSolver (§14);
//   * the Telemetry store (§6.3, §43).
//
// The ControlLoop calls update_snapshots() each cycle (feeding the motor
// history), set_measurement() when visiond publishes a new frame, and
// compute_reference() to get the reference for the cycle.
//
// Pure computation — no CAN, no camera, no motor driver.
#pragma once

#include <cstdint>

#include "control/reference_manager.hpp"
#include "control/search_planner.hpp"
#include "common/motor_state_history.hpp"
#include "geometry/camera_model.hpp"
#include "geometry/los_joint_solver.hpp"
#include "geometry/los_uncertainty.hpp"
#include "telemetry/telemetry.hpp"
#include "tracking/target_estimator.hpp"
#include "tracking/aim_point.hpp"
#include "tracking/target_measurement.hpp"
#include "tracking/tracking_state_machine.hpp"

namespace ota {

class TrackingController {
 public:
  struct Config {
    tracking::TargetEstimatorConfig estimator;
    tracking::TrackingStateMachineConfig fsm;
    SearchPlannerConfig search;
    geo::TurretKinematics kinematics = geo::TurretKinematics::aligned();
    geo::CameraIntrinsics intrinsics;
    // Which point inside the target the axis is aimed at (head vs anchor). See aim_point.hpp.
    tracking::AimOptions aim;
    // §13.3 actuation horizon: how far ahead to predict so the setpoint
    // matters when it reaches the motor.
    int64_t control_delay_ns = 20 * 1000 * 1000;      // 20 ms
    int64_t motor_response_ns = 20 * 1000 * 1000;     // 20 ms
    // Time since the last valid measurement that still counts as "detected"
    // (TRACKING); beyond this the target is COASTING (prediction).
    int64_t fresh_threshold_ns = 100 * 1000 * 1000;   // 100 ms
    // Speed limits (rad/s) handed to the reference manager.
    double track_v_max_rad_s = 30.0 * kDeg2Rad;
    double search_v_max_rad_s = 10.0 * kDeg2Rad;
    double hold_v_max_rad_s = 10.0 * kDeg2Rad;
    // Motor history capacity (~1 s at feedback rate).
    size_t history_capacity = 512;
    // Subtract the two-sigma rate uncertainty before prediction/feed-forward.
    // Enabled by speed-mode service; configurable here for offline ablation.
    bool uncertainty_gated_motion = false;
  };

  explicit TrackingController(Config cfg)
      : cfg_(std::move(cfg)),
        camera_(cfg_.intrinsics),
        solver_(cfg_.kinematics),
        estimator_(cfg_.estimator),
        history_pitch_(cfg_.history_capacity),
        history_yaw_(cfg_.history_capacity),
        fsm_(cfg_.fsm),
        search_(cfg_.search),
        refman_(solver_) {}

  // Feed the latest pose (rad) each cycle (maintains the motor history for §11
  // and the current pose).
  void update_snapshots(TimeNs now_ns, double q_pitch_rad, double q_yaw_rad,
                        TimeNs pitch_sample_ns = -1, TimeNs yaw_sample_ns = -1) {
    // A 200 Hz read of a 50 Hz feedback sample is not a new measurement.
    // Use each axis's receive timestamp and append it once. Stamping repeated
    // poses with now created a delayed staircase in capture-time interpolation.
    if (pitch_sample_ns < 0) pitch_sample_ns = now_ns;
    if (yaw_sample_ns < 0) yaw_sample_ns = now_ns;
    if (pitch_sample_ns > last_pitch_sample_ns_ && pitch_sample_ns <= now_ns) {
      history_pitch_.add(pitch_sample_ns, static_cast<float>(q_pitch_rad), 0.0f);
      last_pitch_sample_ns_ = pitch_sample_ns;
    }
    if (yaw_sample_ns > last_yaw_sample_ns_ && yaw_sample_ns <= now_ns) {
      history_yaw_.add(yaw_sample_ns, static_cast<float>(q_yaw_rad), 0.0f);
      last_yaw_sample_ns_ = yaw_sample_ns;
    }
    now_ns_ = now_ns;
  }

  // Consume a target measurement published by visiond. Returns true if it was
  // a NEW valid measurement that advanced the estimator.
  bool set_measurement(const vision::TargetMeasurement& m) {
    if (!m.valid) return false;
    if (!std::isfinite(m.confidence) || !std::isfinite(m.association_quality) ||
        !std::isfinite(m.identity_confidence)) return false;
    // Capture time is authoritative across visiond restarts (sequence restarts at zero).
    if (has_measurement_ && m.sensor_timestamp_ns <= last_capture_ns_) return false;
    if (!std::isfinite(m.anchor_u_px) || !std::isfinite(m.anchor_v_px) ||
        m.anchor_u_px < 0 || m.anchor_v_px < 0 ||
        m.anchor_u_px > cfg_.intrinsics.width || m.anchor_v_px > cfg_.intrinsics.height)
      return false;
    // §11: interpolate the motor pose at the CAPTURE time. If the history
    // cannot cover it, the measurement is timing-invalid — do NOT update with
    // a newer pose.
    MotorSample sp, sy;
    if (!history_pitch_.interpolate(m.sensor_timestamp_ns, sp)) return false;
    if (!history_yaw_.interpolate(m.sensor_timestamp_ns, sy)) return false;
    // Camera pixel -> camera ray -> base-frame LOS.
    //
    // The pixel is the AIM point, not necessarily the anchor. They differ as soon as the station
    // asks for head aiming, and the difference is the operator's acceptance rule: a box centroid
    // on a standing person is a torso. With no usable box the two are the same point and
    // last_aim_point_.head_applied says which case the operator is looking at.
    const tracking::AimPoint ap =
        m.authoritative_anchor ? tracking::AimPoint{m.anchor_u_px, m.anchor_v_px, false} :
        tracking::aim_point_px(m.anchor_u_px, m.anchor_v_px, m.bbox_x_min_norm, m.bbox_y_min_norm,
                              m.bbox_x_max_norm, m.bbox_y_max_norm, cfg_.intrinsics, cfg_.aim);
    const geo::Vec3 r_cam = camera_.pixel_to_ray(ap.u_px, ap.v_px);
    const geo::Vec3 r_base =
        cfg_.kinematics.ray_to_base(r_cam, sy.q, sp.q);
    double az, el;
    geo::TurretKinematics::base_ray_to_los(r_base, az, el);
    // An operator selection change must not carry the previous subject's velocity.
    if (m.has_track_id && (!has_identity_ || last_identity_ != m.visual_track_id)) {
      estimator_.reset();
      fsm_.reset();
    }
    // A conservative diagonal angular covariance from anchor/box scale and
    // independent detector/association/identity qualities. The 2 px floor and
    // 2% box jitter are provisional priors to fit from stationary recordings.
    const double quality = std::clamp(static_cast<double>(m.confidence) *
        m.association_quality * m.identity_confidence, 0.05, 1.0);
    const double sigma_x = std::max({2.0, cfg_.estimator.measurement_sigma_rad*cfg_.intrinsics.fx,
        0.02*(m.bbox_x_max_norm-m.bbox_x_min_norm)*cfg_.intrinsics.width});
    const double sigma_y = std::max({2.0, cfg_.estimator.measurement_sigma_rad*cfg_.intrinsics.fy,
        0.02*(m.bbox_y_max_norm-m.bbox_y_min_norm)*cfg_.intrinsics.height});
    const auto variance = geo::pixel_los_variance(camera_, cfg_.kinematics,
        sy.q, sp.q, ap.u_px, ap.v_px, sigma_x, sigma_y);
    if (!estimator_.update(az, el, m.sensor_timestamp_ns,
                           variance[0]/quality, variance[1]/quality)) return false;
    last_aim_point_ = ap;
    aim_valid_ = true;
    has_identity_ = m.has_track_id;
    last_identity_ = m.visual_track_id;
    last_capture_ns_ = m.sensor_timestamp_ns;
    last_frame_sequence_ = m.frame_sequence;
    last_valid_arrival_ns_ = now_ns_;
    has_measurement_ = true;
    return true;
  }

  // Produce the reference request for this cycle.
  ReferenceRequest compute_reference(TimeNs now_ns, double hold_pitch_rad,
                                     double hold_yaw_rad) {
    now_ns_ = now_ns;
    // §35 confidence-aware: is the target still "detected"?
    const bool detected =
        has_measurement_ && (now_ns - last_capture_ns_) <
                                cfg_.fresh_threshold_ns;
    const tracking::TrackState st = fsm_.update(now_ns, detected);

    // §13.3 predict to the actuation time.
    double az, el;
    if (estimator_.initialized()) {
      estimator_.predict(now_ns + cfg_.control_delay_ns + cfg_.motor_response_ns,
                         az, el);
      if (cfg_.uncertainty_gated_motion && cfg_.estimator.use_kalman) {
        const double horizon = prediction_horizon_ns() * 1e-9;
        az = tracking::wrap_angle(estimator_.azimuth() + target_motion_rate(0)*horizon);
        el = std::clamp(estimator_.elevation() + target_motion_rate(1)*horizon, -M_PI/2, M_PI/2);
      }
    } else {
      az = el = 0.0;
    }

    // What was predicted *to the actuation time* is what got commanded, so that
    // is what gets remembered: predicted_los() above returns the estimator's
    // current state, and the two differ by exactly the lookahead. Telemetry that
    // shows the wrong one (§78) makes a latency problem look like a geometry
    // problem, which is a bad afternoon either way.
    predicted_az_act_rad_ = az;
    predicted_el_act_rad_ = el;

    ReferenceManagerInput in;
    in.track_state = st;
    in.target_confidence = fsm_.confidence();
    in.predicted_az_rad = az;
    in.predicted_el_rad = el;
    in.q_yaw_hold_rad = hold_yaw_rad;
    in.q_pitch_hold_rad = hold_pitch_rad;
    in.track_v_max_rad_s = cfg_.track_v_max_rad_s;
    in.search_v_max_rad_s = cfg_.search_v_max_rad_s;
    in.hold_v_max_rad_s = cfg_.hold_v_max_rad_s;
    if (st == tracking::TrackState::Search) {
      const SearchPlanner::Output so =
          search_.step(now_ns, last_q_yaw_);
      in.in_search = true;
      in.search_q_yaw_rad = so.q_yaw_rad;
      in.search_q_pitch_rad = so.q_pitch_rad;
      in.search_v_max_rad_s = so.v_max_rad_s;
    }
    last_ref_ = refman_.compute(in);
    return last_ref_;
  }

  // --- accessors ----------------------------------------------------------
  tracking::TrackState track_state() const { return fsm_.state(); }

  // enable_search / disable_search (§36), live. Bounds stay a start_tracking
  // property — they are derived from the homed soft limits at enable time — but
  // on/off is the operator's, any time.
  void set_search_enabled(bool enabled) { fsm_.set_search_enabled(enabled); }
  double confidence() const { return fsm_.confidence(); }
  bool has_measurement() const { return has_measurement_; }
  bool estimator_initialized() const { return estimator_.initialized(); }
  bool prediction_valid() const {
    return estimator_.prediction_valid(now_ns_ + cfg_.control_delay_ns + cfg_.motor_response_ns);
  }
  void predicted_los(double& az, double& el) const {
    az = estimator_.azimuth();
    el = estimator_.elevation();
  }
  // The LOS actually commanded this cycle: the estimator state predicted forward
  // by control delay + motor response (§13.3). This is the one an AUTO_TRACK
  // intent is built from, and the one §78 asks telemetry to show.
  void predicted_los_at_actuation(double& az, double& el) const {
    az = predicted_az_act_rad_;
    el = predicted_el_act_rad_;
  }

  // The estimator's own rate estimates, and the horizon the prediction above is worth. Published
  // because a lead claim is otherwise unmeasurable: `q_ref` is the OUTPUT of the slew limiter, so a
  // lead measured on it conflates "no lead was asked for" with "lead was asked for and the reference
  // could not slew that fast". These three numbers say what was actually requested.
  double target_az_rate_rad_s() const { return estimator_.azimuth_rate(); }
  double target_el_rate_rad_s() const { return estimator_.elevation_rate(); }
  double target_rate_variance(int axis) const { return estimator_.rate_variance(axis); }
  double target_motion_rate(int axis) const {
    const double rate=axis==0?estimator_.azimuth_rate():estimator_.elevation_rate();
    if (!cfg_.uncertainty_gated_motion || !cfg_.estimator.use_kalman) return rate;
    const double uncertainty=2*std::sqrt(std::max(0.0,estimator_.rate_variance(axis)));
    return std::copysign(std::max(0.0,std::abs(rate)-uncertainty),rate);
  }
  // Transform credible world angular velocity through the same solver as the
  // pointing request. Joint pitch is not generally negative world elevation.
  std::array<double,2> joint_motion_rates(double yaw, double pitch) const {
    constexpr double h=.001;
    if (target_motion_rate(0)==0 && target_motion_rate(1)==0) return {0,0};
    double next_yaw=yaw,next_pitch=pitch;
    if (!solver_.solve_from_pose(predicted_az_act_rad_+target_motion_rate(0)*h,
        predicted_el_act_rad_+target_motion_rate(1)*h,yaw,pitch,next_yaw,next_pitch)) return {0,0};
    return {(geo::wrap_near(next_yaw,yaw)-yaw)/h,(next_pitch-pitch)/h};
  }
  const tracking::TargetEstimator::Diagnostics& estimator_diagnostics() const {
    return estimator_.diagnostics();
  }
  double target_position_variance(int axis) const {
    return estimator_.position_variance(axis, now_ns_ + cfg_.control_delay_ns + cfg_.motor_response_ns);
  }
  int64_t prediction_horizon_ns() const {
    if (!estimator_.initialized()) return 0;
    return std::clamp<int64_t>(now_ns_ + cfg_.control_delay_ns + cfg_.motor_response_ns
                             - estimator_.state_timestamp_ns(), 0,
                             static_cast<int64_t>(cfg_.estimator.max_prediction_s * 1e9));
  }
  telemetry::Telemetry& telemetry() { return telemetry_; }
  const telemetry::Telemetry& telemetry() const { return telemetry_; }
  const ReferenceRequest& last_reference() const { return last_ref_; }

  // Record the current yaw (for the search planner's relative motion) and the
  // produced reference (for telemetry / tests). Called by the ControlLoop.
  void record_pose(double q_yaw_rad) { last_q_yaw_ = q_yaw_rad; }
  void record_reference(const ReferenceRequest& r) { last_ref_ = r; }

  void reset() {
    estimator_.reset();
    fsm_.reset();
    has_measurement_ = false;
    last_frame_sequence_ = 0;
    last_valid_arrival_ns_ = 0;
    last_capture_ns_ = 0;
    has_identity_ = false;
    aim_valid_ = false;
    predicted_az_act_rad_ = predicted_el_act_rad_ = 0.0;
    telemetry_.clear();
  }

  // What the axis is aimed at, for the HUD's aim cue. Kept in the controller because that is the
  // only place that knows both the pixel the reference was built from and the intrinsics that
  // turned it into a ray; a HUD that recomputed it would be a second implementation of the rule.
  struct AimStatus {
    double u_norm = 0.0;
    double v_norm = 0.0;
    bool valid = false;
    bool head = false;
  };
  AimStatus aim_status() const {
    AimStatus a;
    a.u_norm = last_aim_point_.u_px / static_cast<double>(cfg_.intrinsics.width);
    a.v_norm = last_aim_point_.v_px / static_cast<double>(cfg_.intrinsics.height);
    a.valid = aim_valid_ && cfg_.intrinsics.width > 0 && cfg_.intrinsics.height > 0;
    a.head = last_aim_point_.head_applied;
    return a;
  }


 private:
  Config cfg_;
  geo::CameraModel camera_;
  tracking::AimPoint last_aim_point_;
  bool aim_valid_ = false;
  geo::LosJointSolver solver_;
  tracking::TargetEstimator estimator_;
  MotorStateHistory history_pitch_;
  MotorStateHistory history_yaw_;
  TimeNs last_pitch_sample_ns_ = -1;
  TimeNs last_yaw_sample_ns_ = -1;
  tracking::TrackingStateMachine fsm_;
  SearchPlanner search_;
  ReferenceManager refman_;
  telemetry::Telemetry telemetry_;

  bool has_measurement_ = false;
  uint64_t last_frame_sequence_ = 0;
  TimeNs last_valid_arrival_ns_ = 0;
  uint64_t last_capture_ns_ = 0;
  bool has_identity_ = false;
  uint64_t last_identity_ = 0;
  TimeNs now_ns_ = 0;
  double last_q_yaw_ = 0.0;
  double predicted_az_act_rad_ = 0.0;
  double predicted_el_act_rad_ = 0.0;
  ReferenceRequest last_ref_;
};

}  // namespace ota
