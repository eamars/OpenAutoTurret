#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "config/mixed_hardware_profile.hpp"
#include "config/turret_config.hpp"
#include "control/motion_profile.hpp"

// WP0: the production split-bus station files must resolve to the documented
// layers. Every expectation here is cross-checked against the 2026-09-27 boot
// log and /api/state telemetry in ADR-001/reports/
// WP0_CONFIG_RESOLUTION_2026-09-27.md. This test proves units and override
// order; it authorizes no motion.
using namespace ota;
namespace {
const std::filesystem::path& firmware_root() {
  static const std::filesystem::path root =
      std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
  return root;
}
config::LoadResult load_from(YAML::Node root) {
  char path[] = "/tmp/ota_mixed_cfg_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) throw std::runtime_error("mkstemp");
  close(fd);
  { std::ofstream f(path); f << root; }
  auto result = config::load_turret_config(path);
  std::remove(path);
  return result;
}
const config::LoadResult& station() {
  static const config::LoadResult loaded = config::load_turret_config(
      (firmware_root()/"config/turret_mixed.yaml").string());
  return loaded;
}
}

TEST(MixedStationConfig, StationFileLoadsAndNamesTheSplitBusProfile) {
  ASSERT_TRUE(station().ok);
  for (const auto& e : station().errors) RecordProperty("errors", e);
  EXPECT_EQ(station().config.hardware_profile, "config/mixed_hardware.yaml");
  EXPECT_DOUBLE_EQ(station().config.axes[1].max_velocity_deg_s, 10);  // yaw
  EXPECT_DOUBLE_EQ(station().config.axes[0].max_velocity_deg_s, 30);  // pitch
}

TEST(MixedStationConfig, HardwareProfilePinsTheCommissionedTopology) {
  const auto loaded = config::mixed::load_mixed_hardware_profile(
      (firmware_root()/"config/mixed_hardware.yaml").string());
  ASSERT_TRUE(loaded.ok);
  EXPECT_EQ(loaded.profile.yaw.protocol, config::mixed::Protocol::Gm6020);
  EXPECT_EQ(loaded.profile.yaw.topology, config::mixed::Topology::Continuous);
  EXPECT_EQ(loaded.profile.yaw.control_mode, config::mixed::ControlMode::Voltage);
  ASSERT_TRUE(loaded.profile.yaw.feedback_frame_id.has_value());
  EXPECT_EQ(*loaded.profile.yaw.feedback_frame_id, 0x205);
  ASSERT_TRUE(loaded.profile.yaw.command_frame_id.has_value());
  EXPECT_EQ(*loaded.profile.yaw.command_frame_id, 0x1FF);
  EXPECT_EQ(loaded.profile.pitch.protocol, config::mixed::Protocol::CyberGear);
  EXPECT_EQ(loaded.profile.pitch.topology, config::mixed::Topology::Bounded);
  ASSERT_TRUE(loaded.profile.pitch.expected_unique_id.has_value());
  EXPECT_EQ(*loaded.profile.pitch.expected_unique_id, 0x7216313130333105ULL);
  // 5 A is the ceiling, not a target; the loader must refuse a larger pin.
  ASSERT_TRUE(loaded.profile.pitch.current_limit_a.has_value());
  EXPECT_DOUBLE_EQ(*loaded.profile.pitch.current_limit_a, 5.0);
  // The temperature gate is an owner's operating decision, pinned in the file
  // and visible here: the station runs a deliberate value, not a code constant.
  EXPECT_EQ(loaded.profile.yaw.yaw_guard_temp_raw_ceiling, 0);
}

TEST(MixedStationConfig, TemperatureGateIsSpelledOutAndRangeChecked) {
  // The parser's style is a fully-spelled station file: a missing key is an
  // error, not a silent default, so an operator reads the gate's state instead
  // of guessing it. Out of one byte's range: refused, not clamped.
  const auto text = [&]() {
    std::ifstream f(firmware_root() / "config/mixed_hardware.yaml");
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  }();
  const auto load = [&](const std::string& s) {
    const auto path = std::filesystem::temp_directory_path() / "ota_guard_probe.yaml";
    std::ofstream(path) << s;
    return config::mixed::load_mixed_hardware_profile(path.string());
  };
  const auto missing = load([&] {
    auto s = text;
    const auto pos = s.find("guard_temp_raw_ceiling: 0");
    s.erase(pos, std::string("guard_temp_raw_ceiling: 0").size());
    return s;
  }());
  EXPECT_FALSE(missing.ok);
  EXPECT_FALSE(missing.errors.empty());
  if (!missing.errors.empty())
    EXPECT_NE(std::string::npos, missing.errors[0].find("guard_temp_raw_ceiling"));
  auto too_big = text;
  const auto pos = too_big.find("guard_temp_raw_ceiling: 0");
  too_big.replace(pos, std::string("guard_temp_raw_ceiling: 0").size(),
                  "guard_temp_raw_ceiling: 256");
  EXPECT_FALSE(load(too_big).ok);
}

TEST(MixedStationConfig, AxisMaximumsCapEveryServiceModePerAxis) {
  ASSERT_TRUE(station().ok);
  const auto& m = station().config.motion;
  ASSERT_TRUE(m.configured);
  // No file uses motion.modes.<mode>.axes today: both axes share the mode pair,
  // and the axis layer then trims yaw. AUTO_TRACK must resolve to
  // pitch 20/30/100 and yaw 10/15/60 deg/s (boot log, 2026-09-27).
  for (int mode=0; mode<3; ++mode) {
    const auto yaw = control::resolve_motion(m.modes[mode][1], m.axis_maximum[1], {});
    EXPECT_DOUBLE_EQ(yaw.target.speed, 10*kDeg2Rad);
    EXPECT_DOUBLE_EQ(yaw.maximum.speed, 10*kDeg2Rad);
  }
  const auto track_pitch = control::resolve_motion(m.modes[1][0], m.axis_maximum[0], {});
  EXPECT_DOUBLE_EQ(track_pitch.target.speed, 20*kDeg2Rad);
  EXPECT_DOUBLE_EQ(track_pitch.target.acceleration, 30*kDeg2Rad);
  EXPECT_DOUBLE_EQ(track_pitch.target.jerk, 100*kDeg2Rad);
}

TEST(MixedStationConfig, ConservativePayloadDoesNotTrimTheDeclaredEnvelope) {
  // conservative.yaml pins v_max 0.35 rad/s = 20.1 deg/s per axis: above the
  // 20 deg/s service maximum, so the payload layer must not bite today. If a
  // future edit lowers it below the mode maximum, motion_limit_reason gains
  // ",payload" and this test must be re-read, not deleted.
  const auto doc = YAML::LoadFile(
      (firmware_root()/"config/payload_profiles/conservative.yaml").string());
  EXPECT_NEAR(doc["pitch"]["v_max_rad_s"].as<double>(), 0.35, 1e-9);
  EXPECT_NEAR(doc["yaw"]["v_max_rad_s"].as<double>(), 0.35, 1e-9);
}

TEST(MixedStationConfig, DirectionSignIsStrictlyTypedButNotConsumedByMixed) {
  auto root = YAML::LoadFile((firmware_root()/"config/turret_mixed.yaml").string());
  root["motors"]["yaw"]["direction_sign"] = 0;
  EXPECT_FALSE(load_from(root).ok);   // ±1 only; 0 is a type error, not "unused"
  root["motors"]["yaw"]["direction_sign"] = -1;  // live release pins -1
  EXPECT_TRUE(load_from(root).ok);    // loads either sense: the mixed backend
  root["motors"]["yaw"]["direction_sign"] = 1;   // HEAD pins +1 and both run
  EXPECT_TRUE(load_from(root).ok);    // on the same physical wiring (WP0 §4.2)
}
