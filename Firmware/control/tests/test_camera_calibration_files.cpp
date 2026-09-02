// Cross-language contract tests for the §28.2/§28.3 calibration FILES.
//
// These loaders had no test at all, and that silence is how a real defect
// survived: vision/installation_calibration.py used to write YAML mapping text
// ("fx: 1420.5", "R_P_C:\n  - [1.0, ...]"), which parse_key_values cannot see —
// a line without '=' is raw matrix text, and a raw line of "fx: 1420.5" yields
// no doubles. The result was NOT a crash: load_camera_intrinsics answered
// "missing key(s); intrinsics NOT applied" and the station kept running
// UNCALIBRATED with default intrinsics, which is exactly the outcome §28.2 says
// must never be mistaken for a known geometry.
//
// The literals below are written to match the PYTHON writer's output byte for
// byte, and vision/tests/test_calibration_file_format.py asserts the writer
// still produces exactly this shape. That pair is the contract.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "calibration/camera_calibration.hpp"

namespace {

namespace fs = std::filesystem;

constexpr const char* kDir = "/tmp/ota_camera_files_test";

std::string write_file(const std::string& name, const std::string& text) {
  fs::create_directories(kDir);
  const std::string p = std::string(kDir) + "/" + name;
  std::ofstream f(p, std::ios::trunc);
  f << text;
  f.close();
  return p;
}

class CameraCalibrationFiles : public ::testing::Test {
 protected:
  void SetUp() override { fs::remove_all(kDir); fs::create_directories(kDir); }
  void TearDown() override { fs::remove_all(kDir); }
};

// Exactly what CameraIntrinsics.to_yaml() emits for a fitted camera.
TEST_F(CameraCalibrationFiles, LoadsIntrinsicsWrittenByPython) {
  const std::string p = write_file(
      "camera_intrinsics.yaml",
      "# ota-camera-intrinsics v1\n"
      "# produced by tools/calibrate_camera_intrinsics.py\n"
      "fx=1417.6171875000001\n"
      "fy=1418.2999999999997\n"
      "cx=959.6875\n"
      "cy=540.125\n"
      "width=1920\n"
      "height=1080\n"
      "dist_model=plumb_bob\n"
      "k1=-0.052000000000000005\n"
      "k2=0.011000000000000001\n"
      "p1=0\n"
      "p2=0\n"
      "k3=0\n");

  ota::IntrinsicsLoad l = ota::load_camera_intrinsics(p);
  EXPECT_TRUE(l.found) << l.detail;
  EXPECT_NEAR(l.intrinsics.fx, 1417.6171875, 1e-6);
  EXPECT_NEAR(l.intrinsics.fy, 1418.3, 1e-6);
  EXPECT_NEAR(l.intrinsics.cx, 959.6875, 1e-6);
  EXPECT_NEAR(l.intrinsics.width, 1920, 0);
  EXPECT_NEAR(l.intrinsics.height, 1080, 0);
  EXPECT_TRUE(l.has_distortion);            // stored; v1 model applies none
  EXPECT_NE(l.detail.find("distortion stored"), std::string::npos) << l.detail;
}

// The shape the Python writer produced BEFORE the contract was fixed. Pinned so
// nobody "fixes" the writer back: the daemon cannot use it, and says so only in
// a log line most people never read.
TEST_F(CameraCalibrationFiles, LegacyYamlMappingIsNotApplied) {
  const std::string p = write_file(
      "camera_intrinsics.yaml",
      "fx: 1417.6171875000001\nfy: 1418.2999999999997\n"
      "cx: 959.6875\ncy: 540.125\n"
      "distortion: [-0.052, 0.011, 0.0, 0.0, 0.0]\n"
      "width: 1920\nheight: 1080\n");

  ota::IntrinsicsLoad l = ota::load_camera_intrinsics(p);
  EXPECT_FALSE(l.found);
  EXPECT_NE(l.detail.find("missing key"), std::string::npos) << l.detail;
}

TEST_F(CameraCalibrationFiles, LoadsExtrinsicsWrittenByPython) {
  // A 90 deg mount: camera z along base x. Raw rows, not a YAML mapping.
  const std::string p = write_file(
      "camera_extrinsics.yaml",
      "# ota-camera-extrinsics v1\n"
      "t_P_C=0 0 0.035000000000000003\n"
      "0 0 1\n"
      "-1 0 0\n"
      "0 -1 0\n");

  std::string detail;
  ota::geo::TurretKinematics kin = ota::load_camera_extrinsics(p, detail);
  EXPECT_NE(detail.find("loaded"), std::string::npos) << detail;
  EXPECT_NEAR(kin.R_PC.m[0][2], 1.0, 1e-12);
  EXPECT_NEAR(kin.R_PC.m[1][0], -1.0, 1e-12);
  EXPECT_NEAR(kin.R_PC.m[2][1], -1.0, 1e-12);
}

TEST_F(CameraCalibrationFiles, LegacyYamlExtrinsicsFallsBackToAligned) {
  // A yawed mount expressed in the OLD format. The point is not the numbers,
  // it is that the file's rotation never reaches the kinematics: "aligned" in
  // this codebase is the nominal mount convention (NOT identity), so compare
  // against aligned() rather than assuming I.
  const std::string p = write_file(
      "camera_extrinsics.yaml",
      "R_P_C:\n  - [0.0, -1.0, 0.0]\n  - [1.0, 0.0, 0.0]\n  - [0.0, 0.0, 1.0]\n"
      "t_P_C: [0.0, 0.0, 0.0]\n");
  std::string detail;
  ota::geo::TurretKinematics kin = ota::load_camera_extrinsics(p, detail);
  EXPECT_NE(detail.find("no 3x3 rotation found"), std::string::npos) << detail;
  const ota::geo::TurretKinematics def = ota::geo::TurretKinematics::aligned();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(kin.R_PC.m[i][j], def.R_PC.m[i][j], 1e-12);   // the DEFAULT
}

TEST_F(CameraCalibrationFiles, NonOrthonormalRotationIsRefused) {
  const std::string p = write_file(
      "camera_extrinsics.yaml",
      "# ota-camera-extrinsics v1\n1 1 0\n0 1 0\n0 0 1\n");
  std::string detail;
  ota::load_camera_extrinsics(p, detail);
  EXPECT_NE(detail.find("not orthonormal"), std::string::npos) << detail;
}

TEST_F(CameraCalibrationFiles, MissingFilesReportUncalibrated) {
  ota::IntrinsicsLoad l =
      ota::load_camera_intrinsics(std::string(kDir) + "/nothing.yaml");
  EXPECT_FALSE(l.found);
  EXPECT_NE(l.detail.find("UNCALIBRATED"), std::string::npos) << l.detail;

  std::string detail;
  ota::load_camera_extrinsics(std::string(kDir) + "/nothing.yaml", detail);
  EXPECT_NE(detail.find("UNCALIBRATED"), std::string::npos) << detail;
}

TEST_F(CameraCalibrationFiles, PartialFileIsNotApplied) {
  // width/height are not optional: the camera model needs them to turn a pixel
  // into a ray, so a file missing them must NOT be half-applied.
  const std::string p = write_file("camera_intrinsics.yaml",
                                   "# ota-camera-intrinsics v1\n"
                                   "fx=1400\nfy=1400\ncx=960\ncy=540\n");
  ota::IntrinsicsLoad l = ota::load_camera_intrinsics(p);
  EXPECT_FALSE(l.found);
  EXPECT_NE(l.detail.find("missing key"), std::string::npos) << l.detail;
}

}  // namespace
