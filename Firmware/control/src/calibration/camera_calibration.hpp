#pragma once
// OpenAutoTurret — camera calibration files (architecture §28.2 / §28.3).
//
// §28.2 `camera_intrinsics.yaml`: the pinhole intrinsics (+ optional radial /
// tangential distortion, §10.2) that turn a detector pixel into a camera-frame
// ray. §28.3 `camera_extrinsics.yaml`: the camera->pitch rotation R_P_C.
//
// Until a commissioning run produces them (P9, the fiducial-board pass) the
// files are absent and the caller falls back to the aligned-identity defaults,
// reporting the calibration as UNCALIBRATED — never silently pretending the
// geometry is known (the same contract as the installation pose, §29).
//
// On-disk format matches the installation pose (human-readable key=value, one
// per line; `#` comments ignored):
//   # ota-camera-intrinsics v1
//   fx=1420.5
//   fy=1421.1
//   cx=960.0
//   cy=540.0
//   width=1920
//   height=1080
//   dist_model=plumb_bob        # optional; k1..k3/p1..p2 are stored but the v1
//   k1=-0.05                    # camera model applies none (Part 3 item 14)
//
// The extrinsic file carries a 3x3 rotation after the header line:
//   # ota-camera-extrinsics v1
//   0.0 0.0 1.0
//   -1.0 0.0 0.0
//   0.0 -1.0 0.0
//
// Pure data + boot-time file I/O: NO CAN, NO camera, NO motor driver.
#include <sys/stat.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "geometry/camera_model.hpp"
#include "geometry/turret_kinematics.hpp"

namespace ota {

// Modification time of a file, in ns since the epoch, or 0 when it cannot be read. Kept here rather
  // than at the call site because the answer belongs to whoever knows the path.
inline int64_t file_mtime_ns(const std::string& path) {
  struct stat sb {};
  if (::stat(path.c_str(), &sb) != 0) return 0;
  return static_cast<int64_t>(sb.st_mtim.tv_sec) * 1000000000LL +
         static_cast<int64_t>(sb.st_mtim.tv_nsec);
}

struct IntrinsicsLoad {
  bool found = false;            // the file existed and parsed
  int64_t source_mtime_ns = 0;   // when the geometry being reported was measured (0 = unknown)
  geo::CameraIntrinsics intrinsics;
  bool has_distortion = false;   // stored for later (unused by the v1 model)
  std::string detail;            // human-readable status for the boot log
};

// Parse a key=value calibration file into a map of raw string values.
inline void parse_key_values(const std::string& text,
                             std::string keys[64], double vals[64],
                             std::string& extra, int& n) {
  n = 0;
  std::istringstream is(text);
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') continue;
    const std::size_t eq = line.find('=');
    if (eq == std::string::npos) {  // matrix / other raw section
      extra += line + "\n";
      continue;
    }
    const std::string k = line.substr(0, eq);
    const std::string v = line.substr(eq + 1);
    if (k.empty() || n >= 64) continue;
    keys[n] = k;
    std::istringstream vs(v);
    vs >> vals[n];
    ++n;
  }
}

// Load §28.2 intrinsics. Missing file / invalid values => found=false and the
// caller keeps the aligned defaults (and says so in the log).
inline IntrinsicsLoad load_camera_intrinsics(const std::string& path) {
  IntrinsicsLoad out;
  std::ifstream f(path);
  if (!f) {
    out.detail = "no file (using aligned-default intrinsics, UNCALIBRATED)";
    return out;   // source_mtime_ns stays 0, and telemetry will say the age is unknown
  }
  out.source_mtime_ns = file_mtime_ns(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  std::string keys[64];
  double vals[64];
  std::string extra;
  int n = 0;
  parse_key_values(ss.str(), keys, vals, extra, n);

  geo::CameraIntrinsics in;  // defaults
  bool have[6] = {false, false, false, false, false, false};
  for (int i = 0; i < n; ++i) {
    const std::string& k = keys[i];
    if (k == "fx") { in.fx = vals[i]; have[0] = true; }
    else if (k == "fy") { in.fy = vals[i]; have[1] = true; }
    else if (k == "cx") { in.cx = vals[i]; have[2] = true; }
    else if (k == "cy") { in.cy = vals[i]; have[3] = true; }
    else if (k == "width") { in.width = static_cast<int>(vals[i]); have[4] = true; }
    else if (k == "height") { in.height = static_cast<int>(vals[i]); have[5] = true; }
    else if (k == "k1" || k == "k2" || k == "p1" || k == "p2" || k == "k3") {
      out.has_distortion = true;
    }
  }
  for (int i = 0; i < 6; ++i) {
    if (!have[i]) {
      out.detail = "missing key(s); intrinsics NOT applied";
      return out;
    }
  }
  if (!in.valid()) {
    out.detail = "invalid intrinsics (fx/fy/width/height must be positive)";
    return out;
  }
  out.found = true;
  out.intrinsics = in;
  std::ostringstream d;
  d << "fx=" << in.fx << " fy=" << in.fy << " cx=" << in.cx
    << " cy=" << in.cy << " " << in.width << "x" << in.height
    << (out.has_distortion ? " (distortion stored, not applied in v1)" : "");
  out.detail = d.str();
  return out;
}

// Load the §28.3 camera->pitch rotation. Missing/invalid => the aligned default.
inline geo::TurretKinematics load_camera_extrinsics(const std::string& path,
                                                    std::string& detail) {
  geo::TurretKinematics kin = geo::TurretKinematics::aligned();
  std::ifstream f(path);
  if (!f) {
    detail = "no file (using aligned camera mount, UNCALIBRATED)";
    return kin;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  std::string keys[64];
  double vals[64];
  std::string extra;
  int n = 0;
  parse_key_values(ss.str(), keys, vals, extra, n);
  std::istringstream ms(extra);
  double m[9];
  int got = 0;
  while (got < 9 && (ms >> m[got])) ++got;
  if (got != 9) {
    detail = "no 3x3 rotation found (using aligned camera mount)";
    return kin;
  }
  // Orthogonality check: R * R^T ~ I (a fiducial estimate must satisfy it).
  double err = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double dot = 0.0;
      for (int k = 0; k < 3; ++k) dot += m[i * 3 + k] * m[j * 3 + k];
      err += std::fabs(dot - (i == j ? 1.0 : 0.0));
    }
  }
  if (err > 1e-3) {
    detail = "rotation is not orthonormal (sum err " + std::to_string(err) +
             "); using aligned camera mount";
    return kin;
  }
  geo::Mat3 R;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) R.m[i][j] = m[i * 3 + j];
  kin.R_PC = R;
  detail = "R_P_C loaded from file";
  return kin;
}

}  // namespace ota
