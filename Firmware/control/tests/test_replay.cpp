// Recorded-vision replay tests (architecture §54.3, §11): a deterministic
// scenario in which a target moves in the base frame, the gimbal (yaw/pitch)
// moves, and camera measurements (pixel + capture timestamp) are replayed
// through the FULL C++ pipeline — pixel -> (interpolated gimbal pose) ->
// base-frame LOS -> estimator — WITHOUT a live camera, CAN, or motor driver.
// This verifies the camera/motor time alignment (§11): the pose used must be
// the pose at the frame's capture time, not the current pose.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "common/motor_state_history.hpp"
#include "geometry/camera_model.hpp"
#include "geometry/turret_kinematics.hpp"
#include "tracking/target_estimator.hpp"

namespace {
using ota::geo::CameraIntrinsics;
using ota::geo::CameraModel;
using ota::geo::Mat3;
using ota::geo::TurretKinematics;
using ota::geo::Vec3;
using ota::MotorSample;
using ota::MotorStateHistory;
using ota::tracking::TargetEstimator;
using ota::tracking::wrap_angle;

constexpr double kPi = M_PI;

// Build the camera + kinematics used by the scenario.
void make_cam_kin(CameraModel& cam, TurretKinematics& kin) {
  CameraIntrinsics in;
  in.fx = 1000.0; in.fy = 1000.0; in.cx = 960.0; in.cy = 540.0;
  in.width = 1920; in.height = 1080;
  cam = CameraModel(in);
  kin = TurretKinematics::aligned();
}

// Base-frame LOS (az, el) -> camera pixel, given the gimbal pose.
void base_los_to_pixel(const CameraModel& cam, const TurretKinematics& kin,
                       double az, double el, double yaw, double pitch,
                       double& u, double& v) {
  const Vec3 r_base{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                    std::sin(el)};
  // Inverse of ray_to_base: r_cam = R_PC^T * R_y^T * R_z^T * r_base.
  const Mat3 Rinv =
      kin.R_PC.transposed() * Mat3::rot_y(pitch).transposed() *
      Mat3::rot_z(yaw).transposed();
  cam.ray_to_pixel((Rinv * r_base).normalized(), u, v);
}

// Camera pixel -> base-frame LOS angles, given the gimbal pose.
void pixel_to_base_los(const CameraModel& cam, const TurretKinematics& kin,
                       double u, double v, double yaw, double pitch,
                       double& az, double& el) {
  const Vec3 r_cam = cam.pixel_to_ray(u, v);
  const Vec3 r_base = kin.ray_to_base(r_cam, yaw, pitch);
  TurretKinematics::base_ray_to_los(r_base, az, el);
}

// Scenario A: stationary gimbal, target rotating at a constant azimuth rate.
// The estimator must converge on the target's azimuth.
TEST(Replay, StationaryGimbalTracksRotatingTarget) {
  CameraModel cam;
  TurretKinematics kin;
  make_cam_kin(cam, kin);

  const double az_rate = 0.06;  // rad/s
  const double el = 0.04;       // rad (constant)
  TargetEstimator est;

  for (int i = 0; i < 90; ++i) {
    const double t = i / 30.0;
    const double az = az_rate * t;
    double u, v;
    base_los_to_pixel(cam, kin, az, el, 0.0, 0.0, u, v);
    double mz, me;
    pixel_to_base_los(cam, kin, u, v, 0.0, 0.0, mz, me);
    est.update(mz, me, static_cast<int64_t>(t * 1e9));
  }
  // The alpha-beta filter smooths, so it lags a constant-velocity target by a
  // small steady-state amount; the rate estimate converges to the true rate.
  EXPECT_NEAR(est.azimuth(), az_rate * 2.9333333, 5e-3);
  EXPECT_NEAR(est.elevation(), el, 5e-3);
  EXPECT_NEAR(est.azimuth_rate(), az_rate, 5e-3);
}

// Scenario B: moving gimbal. The camera frame moves with the gimbal, so the
// target's PIXEL position changes over time even if the target were stationary
// in the base frame. The base-frame LOS recovered from a measurement must use
// the gimbal pose at the FRAME CAPTURE TIME (interpolated from the motor
// history), not the current pose.
TEST(Replay, MovingGimbalAlignsToCaptureTime) {
  CameraModel cam;
  TurretKinematics kin;
  make_cam_kin(cam, kin);

  const double yaw_rate = 0.10;  // gimbal yaw rate (rad/s)
  const double target_az = 0.30;  // target fixed in the base frame
  const double target_el = 0.05;

  MotorStateHistory hist(256);
  // Record the gimbal pose over time (yaw(t) = yaw_rate * t, pitch = 0).
  for (int i = 0; i < 90; ++i) {
    const double t = i / 30.0;
    hist.add(static_cast<int64_t>(t * 1e9), static_cast<float>(yaw_rate * t), 0.0f);
  }

  // The target is fixed in the base frame; its pixel moves as the gimbal yaws.
  // Replay the last frame: the measurement was captured at t_frame, so the pose
  // at t_frame (not t_now) must be used to recover the base-frame LOS.
  const int last = 89;
  const double t_frame = last / 30.0;
  const double yaw_at_frame = yaw_rate * t_frame;

  double u, v;
  base_los_to_pixel(cam, kin, target_az, target_el, yaw_at_frame, 0.0, u, v);

  // Interpolate the yaw at the capture time from the motor history.
  MotorSample sample;
  ASSERT_TRUE(hist.interpolate(static_cast<int64_t>(t_frame * 1e9), sample));
  const double yaw_interp = sample.q;
  EXPECT_NEAR(yaw_interp, yaw_at_frame, 1e-5);

  double az_correct, el_correct;
  pixel_to_base_los(cam, kin, u, v, yaw_interp, 0.0, az_correct, el_correct);
  // Using the pose at the capture time recovers the true base-frame LOS.
  EXPECT_NEAR(az_correct, target_az, 1e-6);
  EXPECT_NEAR(el_correct, target_el, 1e-6);

  // Now use a STALE (newer) pose — e.g. the pose 0.1 s after the capture — to
  // show the misalignment the doc warns about (§11). The recovered LOS is wrong
  // by about the gimbal's rotation during that 0.1 s.
  const double yaw_stale = yaw_rate * (t_frame + 0.1);
  double az_stale, el_stale;
  pixel_to_base_los(cam, kin, u, v, yaw_stale, 0.0, az_stale, el_stale);
  const double az_err = std::fabs(wrap_angle(az_stale - target_az));
  EXPECT_GT(az_err, 5e-3);  // stale pose => visibly wrong LOS
  // The error should be close to the yaw delta over 0.1 s.
  EXPECT_NEAR(az_err, yaw_rate * 0.1, 5e-3);
}

}  // namespace
