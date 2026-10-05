#pragma once
#include "estimator.hpp"
#include "level1.hpp"
#include "geometry/camera_model.hpp"
#include "geometry/los_joint_solver.hpp"
#include "geometry/turret_kinematics.hpp"
#include <array>
#include <cstdint>

namespace ota::track {
// ADR-003's tracking core: one selected subject, camera pixel to joint reference.
//
//   observe():  pixel at its optical time t_o + the pose interpolated at t_o (the caller's
//               history) -> base-frame LOS (camera rotation removed) -> estimator update,
//               once per frame.
//   tick():     every control period: estimator query at t_control + execution_horizon
//               (age is the only prediction; nothing is added for servo dynamics) -> joint
//               goal through the one LOS->joint solver and its Jacobian -> Level 1 ->
//               the reference sample the ADR-002.2 servos execute.
// Shared by the independent 3a host, production controld and the simulator.

struct Travel { double yaw_low=0, yaw_high=0, pitch_low=0, pitch_high=0; };  // yaw_low==yaw_high: continuous

// t_o = sensor timestamp + fixed_offset + exposure_fraction * exposure + row_time * anchor row:
// the instant the anchor pixel was actually imaged. Measured, not assumed (tools/tracking/timing.py):
// on this station the stamp behaves like start-of-frame, not libcamera's documented start of
// exposure, so the exposure term is identified from sessions at two exposures.
struct TimingParameters { double fixed_offset_s=0, exposure_fraction=0.5, row_time_s=0; };

struct TrackerParameters {
  EstimatorParameters estimator{};
  Level1Parameters level1{};
  TimingParameters timing{};
  double execution_horizon_s=0;   // identified pure execution delay not already in the state (0: none known)
  double pixel_sigma=0;           // px, the anchor's measured noise (stage 2)
  bool target_motion=true;        // false: Level-1 FB only (ADR-003 sec. 7A comparison)
  double jacobian_det_min=1e-3;   // below it the joint rate is unreliable and the FF is zero
};
bool valid(const TrackerParameters& p);

struct PixelObservation {
  int64_t sensor_ns=0;           // SensorTimestamp on the control clock
  double exposure_s=0;
  double u=0, v=0;               // aim pixel, tracker frame
  double sigma_u=0, sigma_v=0;   // px; 0 uses pixel_sigma
  uint64_t identity=0;           // selected subject and generation; a change starts a new subject
};

// The last pixel observation as the estimator received it: enough to rebuild the world LOS offline
// and to see whether turret motion leaks into it (station, 2026-10-05: a 2.2 Hz yaw limit cycle).
struct ObservationRecord {
  int64_t sensor_ns=0, t_ns=0;   // SensorTimestamp and the optical time t_o it was stamped with
  double u=0, v=0;               // aim pixel, tracker frame
  double yaw=0, pitch=0;         // joint angles interpolated at t_o
  double az=0, el=0;             // the base-frame LOS handed to the estimator
  bool accepted=false;
};

struct TickRecord {
  int64_t t_ns=0;
  LosGoal los{};
  JointGoal joint{};
  ReferenceSample reference{};
  std::array<double,2> q_measured{}, e_track{}, e_servo{};
};

class Tracker {
 public:
  Tracker(const TrackerParameters& p,const geo::TurretKinematics& kinematics,const geo::CameraIntrinsics& intrinsics,
          const geo::Vec3& sight_camera,const Travel& travel);
  bool ok() const { return ok_; }
  int64_t observation_time(const PixelObservation& z) const;
  bool observe(const PixelObservation& z,double yaw_at_t_o,double pitch_at_t_o);
  // Take over the reference at the actual axis state (engagement, source change).
  void engage(int64_t t_ns,const std::array<double,2>& q,const std::array<double,2>& v,const std::array<double,2>& a);
  TickRecord tick(int64_t t_ns,const std::array<double,2>& q_measured);
  // Joint goal of a LOS goal near the seed branch (exposed for tests and replay).
  JointGoal joint_goal(const LosGoal& goal,const std::array<double,2>& seed) const;
  // LOS of the optical/sight axis at a joint pose.
  std::array<double,2> los(double yaw,double pitch) const;
  void forget();  // the subject is gone (deselection): estimator cleared, reference holds
  void set_travel(const Travel& travel) { travel_ = travel; }
  const TargetEstimator& estimator() const { return estimator_; }
  const Level1Generator& level1() const { return level1_; }
  Level1Generator& level1() { return level1_; }
  TrackerParameters& parameters() { return p_; }
  const TrackerParameters& parameters() const { return p_; }
  uint64_t identity_changes() const { return identity_changes_; }
  const ObservationRecord& last_observation() const { return last_observation_; }
 private:
  TrackerParameters p_;
  geo::TurretKinematics kinematics_;
  geo::CameraModel camera_;
  geo::LosJointSolver solver_;
  Travel travel_;
  TargetEstimator estimator_;
  Level1Generator level1_;
  bool ok_=false, have_identity_=false;
  uint64_t identity_=0, identity_changes_=0;
  ObservationRecord last_observation_{};
};
}
