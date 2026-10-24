#pragma once
// ADR-005 test fixture: a state directory with calibration records made the way
// tools/calibration_records.py makes them, from this checkout's own files.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "ota_test_paths.hpp"

namespace ota_cal_fixture {
namespace fs = std::filesystem;

inline fs::path firmware() {
  return ota_test_firmware_dir(fs::path(__FILE__).parent_path().parent_path().parent_path());
}

struct State {
  fs::path dir;
  State() {
    char name[] = "/tmp/ota-calibration-XXXXXX";
    if (!mkdtemp(name)) throw std::runtime_error("mkdtemp");
    dir = name;
  }
  ~State() { std::error_code ec; fs::remove_all(dir, ec); }
  fs::path cal() const { return dir / "calibration"; }
};

inline void write(const fs::path& p, const std::string& text) {
  fs::create_directories(p.parent_path());
  std::ofstream(p) << text;
}

inline std::string bore_yaml(const std::string& mode = "off", const std::string& zero = "10",
                             double trim_right = 0, double trim_up = 0) {
  const std::string trims = ", trim_right_mrad: " + std::to_string(trim_right) +
                            ", trim_up_mrad: " + std::to_string(trim_up) + "}\n";
  return "schema: 1\nmode: " + mode + "\nzero_m: " + zero + "\n" +
         "wide: {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 0, up_deg: 0" + trims +
         "detail: {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 0, up_deg: 0" + trims;
}

inline fs::path record_dir(const State& s, int id) {
  char d[8];
  std::snprintf(d, sizeof d, "%04d", id);
  return s.cal() / "records" / d;
}

inline void make_record(const State& s, int id, const std::string& name, const std::string& bore = bore_yaml()) {
  const fs::path dir = record_dir(s, id);
  const fs::path fw = firmware();
  fs::create_directories(dir);
  fs::copy_file(fw / "config/servo/yaw_servo.json", dir / "yaw_servo.json");
  fs::copy_file(fw / "config/servo/pitch_servo.json", dir / "pitch_servo.json");
  fs::copy_file(fw / "config/servo/yaw_accuracy.json", dir / "yaw_accuracy.json");
  fs::copy_file(fw / "calibration/camera_extrinsics.yaml", dir / "camera_extrinsics.yaml");
  write(dir / "bore.yaml", bore);
  write(dir / "record.json",
        "{\"schema\": \"ota.calibration-record/1\", \"name\": \"" + name + "\", \"parts\": {"
        "\"yaw\": {\"file\": \"yaw_servo.json\", \"origin\": \"shipped\", \"date\": \"2026-10-02\", \"rev\": \"abc1234\"},"
        "\"pitch\": {\"file\": \"pitch_servo.json\", \"origin\": \"shipped\"},"
        "\"accuracy\": {\"file\": \"yaw_accuracy.json\", \"origin\": \"shipped\"},"
        "\"extrinsic\": {\"file\": \"camera_extrinsics.yaml\", \"origin\": \"inherited\", \"from\": 1},"
        "\"bore\": {\"file\": \"bore.yaml\", \"origin\": \"shipped\"}}, \"last_run\": null}\n");
}

inline void choose(const State& s, int id) {
  write(s.cal() / "active.json", "{\"schema\": 1, \"active\": " + std::to_string(id) + "}\n");
}

// What tools/servo_commission/manifests.pitch() writes, cut to what a session reads: a 2 s script with
// a step of `step_rad` at 0.5 s, about the window centre (ADR-005 D7).
inline std::string pitch_manifest(const std::string& extra_trial = "", double step_rad = 0.02) {
  std::string samples;
  for (int k = 0; k <= 100; ++k) {
    const double t = k * 0.02;
    samples += std::string(k ? "," : "") + "{\"time_s\":" + std::to_string(t) + ",\"position_rad\":" +
               std::to_string(t >= 0.5 ? step_rad : 0.0) + ",\"velocity_rad_s\":0,\"acceleration_rad_s2\":0}";
  }
  return "{\"schema\":\"adr0022.sensorless-homing/1\",\"purpose\":\"pitch_sensorless_homing\",\"session_label\":\"t-identify\","
         "\"servo_trial\":{\"kp_per_s\":20,\"ki_per_s2\":0,\"integral_clamp_rad_s\":0,\"speed_limit_rad_s\":0.5,"
         "\"command_period_s\":0.001,\"hold_after_s\":0.5,\"following_error_rad\":0.1,\"approach_speed_rad_s\":0.08,"
         "\"window_margin_rad\":0.14" + extra_trial + ",\"reference_samples\":[" + samples + "]}}";
}

inline std::string active_json(const State& s) {
  std::ifstream in(s.cal() / "active.json");
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace ota_cal_fixture
