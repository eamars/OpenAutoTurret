// Offline production-code probe. No camera, sockets, CAN or station launcher.
#include <iostream>
#include <stdexcept>
#include "calibration/camera_calibration.hpp"
#include "control/tracking_controller.hpp"

using namespace ota;
using namespace ota::geo;

void require(bool value, const char* why) {
  if (!value) throw std::runtime_error(why);
}

int main() {
  try {
    const auto intr = load_camera_intrinsics("calibration/camera_intrinsics.yaml");
    require(intr.found, "stored camera intrinsics not found");
    std::string detail;
    const auto kin = load_camera_extrinsics("calibration/camera_extrinsics.yaml", detail);
    require(detail == "R_P_C loaded from file", "stored extrinsics not found");
    double worst_miss = 0, worst_pixel = 0;
    int count = 0;
    for (double depth : {2., 5., 10., 30., 100.}) {
      TrackingController::Config cfg;
      cfg.intrinsics = intr.intrinsics;
      cfg.kinematics = kin;
      cfg.aim.mode = tracking::AimMode::BoxFraction;
      cfg.aim.y_fraction = .22;
      cfg.alignment.enabled = true;
      cfg.alignment.camera_right_mm = cfg.alignment.camera_up_mm = 75;
      cfg.alignment.assumed_depth_m = depth;
      const auto laser = laser_alignment(cfg.alignment, cfg.intrinsics);
      require(laser.valid, "laser geometry invalid");
      const Vec3 point = laser.sight_camera*(depth/laser.sight_camera.z);
      for (double yaw_deg : {20., 160., 300.}) for (double pitch_deg : {-10., -35., -65.}) {
        const double yaw = yaw_deg*M_PI/180, pitch = pitch_deg*M_PI/180;
        const Mat3 rotation = Mat3::rot_z(yaw)*Mat3::rot_y(pitch)*kin.R_PC;
        const Vec3 target = rotation*point;
        TrackingController tracker(cfg);
        tracker.update_snapshots(1000000000, pitch, yaw);
        tracker.update_snapshots(1005000000, pitch, yaw);
        vision::TargetMeasurement m;
        m.valid = m.authoritative_anchor = true;
        m.confidence = 1;
        m.sensor_timestamp_ns = 1005000000;
        m.anchor_u_px = laser.u_norm*cfg.intrinsics.width;
        m.anchor_v_px = laser.v_norm*cfg.intrinsics.height + (.45-.22)*100;
        m.bbox_x_min_norm = (m.anchor_u_px-40)/cfg.intrinsics.width;
        m.bbox_x_max_norm = (m.anchor_u_px+40)/cfg.intrinsics.width;
        m.bbox_y_min_norm = (laser.v_norm*cfg.intrinsics.height-.22*100)/cfg.intrinsics.height;
        m.bbox_y_max_norm = m.bbox_y_min_norm + 100./cfg.intrinsics.height;
        require(tracker.set_measurement(m), "native-anchor measurement rejected");
        tracker.compute_reference(1005000000, pitch, yaw);
        MotionIntent intent;
        intent.source = MotionSource::AutoTrack;
        intent.type = IntentType::LosDirection;
        intent.has_los = true;
        tracker.predicted_los_at_actuation(intent.los_az_rad, intent.los_el_rad);
        intent.sight_camera = tracker.alignment().sight_camera;
        ReferenceManager manager{LosJointSolver(kin)};
        ReferenceManager::IntentLimits limits;
        limits.q_yaw_hold_rad = yaw+.03;
        limits.q_pitch_hold_rad = pitch-.03;
        const auto result = manager.resolve(intent, limits);
        require(result.is_tracking_reference, "reference did not resolve");
        const Mat3 solved = Mat3::rot_z(result.q_yaw_rad)*Mat3::rot_y(result.q_pitch_rad)*kin.R_PC;
        const Vec3 delta = target - solved*laser.origin_camera;
        const Vec3 direction = solved*laser.direction_camera;
        const double miss = (delta-direction*delta.dot(direction)).norm()*1000;
        double u, v;
        CameraModel(cfg.intrinsics).ray_to_pixel(solved.transposed()*target, u, v);
        worst_miss = std::max(worst_miss, miss);
        worst_pixel = std::max(worst_pixel, std::hypot(u-laser.u_norm*cfg.intrinsics.width,
                                                    v-laser.v_norm*cfg.intrinsics.height));
        ++count;
      }
    }
    std::cout << count << " production tracking/reference scenes: max miss " << worst_miss
              << " mm; crosshair error " << worst_pixel << " px\n";
    // Wire/measurement boxes are float32, unlike the double-precision reference model.
    require(worst_miss < .1 && worst_pixel < .002, "pointing/crosshair mismatch");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
