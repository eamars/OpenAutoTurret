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

inline std::string active_json(const State& s) {
  std::ifstream in(s.cal() / "active.json");
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace ota_cal_fixture
