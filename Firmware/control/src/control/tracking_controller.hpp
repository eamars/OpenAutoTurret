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
#include "telemetry/telemetry.hpp"
#include "tracking/target_estimator.hpp"
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
  void update_snapshots(TimeNs now_ns, double q_pitch_rad, double q_yaw_rad) {
    history_pitch_.add(now_ns, static_cast<float>(q_pitch_rad), 0.0f);
    history_yaw_.add(now_ns, static_cast<float>(q_yaw_rad), 0.0f);
    now_ns_ = now_ns;
  }

  // Consume a target measurement published by visiond. Returns true if it was
  // a NEW valid measurement that advanced the estimator.
  bool set_measurement(const vision::TargetMeasurement& m) {
    if (!m.valid) return false;
    if (m.frame_sequence <= last_frame_sequence_) return false;  // stale/repeat
    // §11: interpolate the motor pose at the CAPTURE time. If the history
    // cannot cover it, the measurement is timing-invalid — do NOT update with
    // a newer pose.
    MotorSample sp, sy;
    if (!history_pitch_.interpolate(m.sensor_timestamp_ns, sp)) return false;
    if (!history_yaw_.interpolate(m.sensor_timestamp_ns, sy)) return false;
    // Camera pixel -> camera ray -> base-frame LOS.
    const geo::Vec3 r_cam = camera_.pixel_to_ray(m.anchor_u_px, m.anchor_v_px);
    const geo::Vec3 r_base =
        cfg_.kinematics.ray_to_base(r_cam, sy.q, sp.q);
    double az, el;
    geo::TurretKinematics::base_ray_to_los(r_base, az, el);
    estimator_.update(az, el, m.sensor_timestamp_ns);
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
        has_measurement_ && (now_ns - last_valid_arrival_ns_) <
                                cfg_.fresh_threshold_ns;
    const tracking::TrackState st = fsm_.update(now_ns, detected);

    // §13.3 predict to the actuation time.
    double az, el;
    if (estimator_.initialized()) {
      estimator_.predict(now_ns + cfg_.control_delay_ns + cfg_.motor_response_ns,
                         az, el);
    } else {
      az = el = 0.0;
    }

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
  void predicted_los(double& az, double& el) const {
    az = estimator_.azimuth();
    el = estimator_.elevation();
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
    telemetry_.clear();
  }

 private:
  Config cfg_;
  geo::CameraModel camera_;
  geo::LosJointSolver solver_;
  tracking::TargetEstimator estimator_;
  MotorStateHistory history_pitch_;
  MotorStateHistory history_yaw_;
  tracking::TrackingStateMachine fsm_;
  SearchPlanner search_;
  ReferenceManager refman_;
  telemetry::Telemetry telemetry_;

  bool has_measurement_ = false;
  uint64_t last_frame_sequence_ = 0;
  TimeNs last_valid_arrival_ns_ = 0;
  TimeNs now_ns_ = 0;
  double last_q_yaw_ = 0.0;
  ReferenceRequest last_ref_;
};

}  // namespace ota
