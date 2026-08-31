#pragma once
// OpenAutoTurret — installation orientation (architecture §28.4, §29, §30).
//
// The station base does not have to be level. A visually calibrated
// base->world rotation R_W_B maps the (possibly tilted) base frame into the
// gravity-aligned world frame. The tracking/control pipeline consumes a
// standard BaseOrientation so a future IMU can be added without rewriting the
// tracker (§30).
//
// This module is pure data + boot-time file I/O (load the stored pose, atomic
// commit of a new one). It has NO CAN, NO camera, and NO motor driver.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "common/time.hpp"
#include "common/types.hpp"
#include "geometry/vec3.hpp"

namespace ota {

// §30: where the base->world orientation came from.
enum class PoseSource : uint8_t {
  Unknown = 0,
  Identity,           // default before calibration (assumes a level base)
  FixedStored,        // loaded from a persisted file (FixedStoredPoseProvider)
  VisualCalibration,  // from a §29 fiducial-board calibration
  Imu,                // future (§30)
  FusedVisualImu,     // future (§30)
};

inline constexpr const char* pose_source_name(PoseSource s) {
  switch (s) {
    case PoseSource::Unknown: return "unknown";
    case PoseSource::Identity: return "identity";
    case PoseSource::FixedStored: return "fixed_stored";
    case PoseSource::VisualCalibration: return "visual_calibration";
    case PoseSource::Imu: return "imu";
    case PoseSource::FusedVisualImu: return "fused_visual_imu";
  }
  return "unknown";
}

inline PoseSource pose_source_from_name(const std::string& s) {
  if (s == "identity") return PoseSource::Identity;
  if (s == "fixed_stored") return PoseSource::FixedStored;
  if (s == "visual_calibration") return PoseSource::VisualCalibration;
  if (s == "imu") return PoseSource::Imu;
  if (s == "fused_visual_imu") return PoseSource::FusedVisualImu;
  return PoseSource::Unknown;
}

// §30: the standard orientation object consumed by the tracker. R_W_B rotates
// a base-frame vector into the world (gravity-aligned) frame. Identity (level
// base) when no calibration exists.
struct BaseOrientation {
  TimeNs timestamp_ns = 0;
  geo::Mat3 R_W_B;  // base -> world rotation
  double covariance = 1.0;  // scalar attitude covariance (rad^2); lower = tighter
  PoseSource source = PoseSource::Identity;
  bool valid = false;
  int n_frames = 0;                       // calibration frames that went in
  double reprojection_error_px = 0.0;     // final reprojection error (visual)
};

// The default "no calibration" orientation: an assumed-level base.
inline BaseOrientation identity_pose() {
  BaseOrientation o;
  o.R_W_B = geo::Mat3{};  // identity
  o.source = PoseSource::Identity;
  o.valid = true;  // the identity pose is a valid (if uncalibrated) orientation
  o.covariance = 1e6;  // wide: we do not actually know the base is level
  return o;
}

// World-frame LOS from a base-frame LOS (azimuth, elevation) given R_W_B.
// Base-frame optical-axis direction is (cos el cos az, cos el sin az, sin el);
// rotate into the world frame and re-extract azimuth/elevation (the same
// convention as TurretKinematics::base_ray_to_los).
inline void base_los_to_world(const geo::Mat3& R_W_B, double az_base_rad,
                              double el_base_rad, double& az_world_rad,
                              double& el_world_rad) {
  const geo::Vec3 v_base{std::cos(el_base_rad) * std::cos(az_base_rad),
                         std::cos(el_base_rad) * std::sin(az_base_rad),
                         std::sin(el_base_rad)};
  const geo::Vec3 w = R_W_B * v_base;
  const double horiz = std::sqrt(w.x * w.x + w.y * w.y);
  az_world_rad = std::atan2(w.y, w.x);
  el_world_rad = std::atan2(w.z, horiz);
}

inline void base_los_to_world(const BaseOrientation& o, double az_base_rad,
                              double el_base_rad, double& az_world_rad,
                              double& el_world_rad) {
  base_los_to_world(o.R_W_B, az_base_rad, el_base_rad, az_world_rad,
                    el_world_rad);
}

// Decompose R_W_B (base -> world) into yaw-pitch-roll (ZYX) angles. For a
// level base these are zero; for a tilted base they describe the tilt.
// R_W_B = rot_z(yaw) * rot_y(pitch) * rot_x(roll).
inline void R_W_B_to_euler(const geo::Mat3& R_W_B, double& roll_rad,
                           double& pitch_rad, double& yaw_rad) {
  const double sp = std::clamp(-R_W_B.m[2][0], -1.0, 1.0);
  pitch_rad = std::asin(sp);
  roll_rad = std::atan2(R_W_B.m[2][1], R_W_B.m[2][2]);
  yaw_rad = std::atan2(R_W_B.m[1][0], R_W_B.m[0][0]);
}

inline void R_W_B_to_euler(const BaseOrientation& o, double& roll_rad,
                           double& pitch_rad, double& yaw_rad) {
  R_W_B_to_euler(o.R_W_B, roll_rad, pitch_rad, yaw_rad);
}

// Declared before FixedStoredPoseProvider, which calls it at construction.
BaseOrientation load_installation_pose(const std::string& path);

// A provider of the current base->world orientation. The tracker consumes this
// interface, so a future IMU needs no tracker changes (§30).
class InstallationPoseProvider {
 public:
  virtual ~InstallationPoseProvider() = default;
  virtual BaseOrientation get() const = 0;
  virtual std::string name() const = 0;
};

// Loads a persisted R_W_B once at construction (boot) and serves it. If the
// file is missing or invalid it serves the identity pose (a level base) and
// marks the stored pose as uncalibrated.
class FixedStoredPoseProvider : public InstallationPoseProvider {
 public:
  explicit FixedStoredPoseProvider(std::string path)
      : path_(std::move(path)) {
    BaseOrientation o = load_installation_pose(path_);
    if (o.valid && o.source != PoseSource::Identity) {
      o.source = PoseSource::FixedStored;
      stored_ = o;
    } else {
      stored_ = identity_pose();
    }
  }
  BaseOrientation get() const override { return stored_; }
  std::string name() const override { return "FixedStoredPoseProvider"; }

 private:
  std::string path_;
  BaseOrientation stored_ = identity_pose();
};

// --- persistence (atomic commit, §29.3 / §41) -----------------------------
//
// The on-disk format is a small, human-readable text file:
//   # ota-installation-pose v1
//   source=visual_calibration
//   valid=1
//   timestamp_ns=123456789
//   covariance=0.0001
//   n_frames=20
//   reprojection_error_px=0.5
//   1.000000 0.000000 0.000000
//   0.000000 1.000000 0.000000
//   0.000000 0.000000 1.000000
//
// save_installation_pose() writes to a temp file in the SAME directory and then
// renames it over the target, so a crash mid-write can never leave a torn
// calibration file (the previous valid file stays in place).

inline bool save_installation_pose(const std::string& path,
                                   const BaseOrientation& o,
                                   std::string& err) {
  const std::string dir = path.substr(0, path.find_last_of('/'));
  const std::string tmp = (dir.empty() ? path : dir + "/") +
                          (path.substr(path.find_last_of('/') + 1)) + ".tmp";
  std::ofstream f(tmp, std::ios::trunc);
  if (!f) {
    err = "cannot open " + tmp + " for writing";
    return false;
  }
  f << std::setprecision(17);  // full double round-trip for the rotation values
  f << "# ota-installation-pose v1\n";
  f << "source=" << pose_source_name(o.source) << "\n";
  f << "valid=" << (o.valid ? 1 : 0) << "\n";
  f << "timestamp_ns=" << o.timestamp_ns << "\n";
  f << "covariance=" << o.covariance << "\n";
  f << "n_frames=" << o.n_frames << "\n";
  f << "reprojection_error_px=" << o.reprojection_error_px << "\n";
  for (int i = 0; i < 3; ++i) {
    f << o.R_W_B.m[i][0] << " " << o.R_W_B.m[i][1] << " "
      << o.R_W_B.m[i][2] << "\n";
  }
  f.flush();
  f.close();
  if (!f.good()) {
    err = "write failed for " + tmp;
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    err = "rename " + tmp + " -> " + path + " failed";
    return false;
  }
  err.clear();
  return true;
}

// Load a stored pose. Returns an invalid pose (valid=false) on any problem so
// the caller can fall back to the identity pose.
inline BaseOrientation load_installation_pose(const std::string& path) {
  BaseOrientation o = identity_pose();
  o.valid = false;
  std::ifstream f(path);
  if (!f) return o;
  std::string line;
  std::vector<double> mat;
  while (std::getline(f, line)) {
    // Trim.
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                             line.back() == ' '))
      line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const size_t eq = line.find('=');
    if (eq != std::string::npos) {
      const std::string key = line.substr(0, eq);
      const std::string val = line.substr(eq + 1);
      try {
        if (key == "source")
          o.source = pose_source_from_name(val);
        else if (key == "valid")
          o.valid = (val == "1");
        else if (key == "timestamp_ns")
          o.timestamp_ns = std::stoll(val);
        else if (key == "covariance")
          o.covariance = std::stod(val);
        else if (key == "n_frames")
          o.n_frames = std::stoi(val);
        else if (key == "reprojection_error_px")
          o.reprojection_error_px = std::stod(val);
      } catch (...) {
        return BaseOrientation{};  // malformed -> invalid
      }
      continue;
    }
    // Otherwise: a matrix row of 3 doubles.
    std::istringstream ss(line);
    double a, b, c;
    if (ss >> a >> b >> c) {
      mat.push_back(a);
      mat.push_back(b);
      mat.push_back(c);
    }
  }
  if (mat.size() != 9) return BaseOrientation{};  // malformed -> invalid
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) o.R_W_B.m[i][j] = mat[i * 3 + j];
  // A valid file must have a finite, near-orthonormal rotation.
  for (double v : mat)
    if (!std::isfinite(v)) return BaseOrientation{};
  return o;
}

}  // namespace ota
