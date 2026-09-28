#include "ota_test_paths.hpp"
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "config/mixed_hardware_profile.hpp"
#include "control/mixed_can_motor_backend.hpp"  // kYawMaxAccelerationRadS2, pinned below
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
      ota_test_firmware_dir(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path());
  return root;
}
const char* kNope = nullptr;  // 只为下面那个 helper 有个锚
template <class R>
std::string why(const R& r) {
  // 失败必须吵、而且带原因：以前这里只说 ok==false，加载器怎么想的要人手跑一遍才知道。
  std::string out;
  for (const auto& e : r.errors) { out += out.empty() ? "" : " | "; out += e; }
  return out.empty() ? std::string("(no reason given)") : out;
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
  ASSERT_TRUE(station().ok) << why(station());
  for (const auto& e : station().errors) RecordProperty("errors", e);
  EXPECT_EQ(station().config.hardware_profile, "config/mixed_hardware.yaml");
  // The two axes declare the same top speed (owner ruling 2026-09-28). MANUAL is one
  // gesture across both axes, and yaw felt broken because this file declared it three
  // times slower than pitch while motion.modes asked both for the same number.
  EXPECT_DOUBLE_EQ(station().config.axes[1].max_velocity_deg_s,
                   station().config.axes[0].max_velocity_deg_s);
  EXPECT_DOUBLE_EQ(station().config.axes[1].max_velocity_deg_s, 30);  // yaw
  EXPECT_DOUBLE_EQ(station().config.axes[0].max_velocity_deg_s, 30);  // pitch

  // Owner, 2026-09-28: "yaw和pitch在manual模式下速度应该保持匹配…yaw的速度明显偏慢了", and
  // on the second pass about the acceleration specifically. Velocity parity was pinned
  // above; the ramp behind it was the actual culprit, because a hard-coded 20 deg/s^2 sat
  // in the yaw backend while the pitch drive accelerated under its own profile. Read from
  // the YAML rather than the parsed struct so the gate is on the number he edits.
  const YAML::Node axes = YAML::LoadFile((firmware_root() / "config/turret_mixed.yaml").string())["axes"];
  for (const char* key : {"max_acceleration_deg_s2", "max_jerk_deg_s3"}) {
    EXPECT_DOUBLE_EQ(axes["yaw"][key].as<double>(), axes["pitch"][key].as<double>()) << key;
  }
  // And the constant that actually shapes the ramp is the declared one, not a fourth opinion.
  EXPECT_DOUBLE_EQ(kYawMaxAccelerationRadS2 * 180.0 / 3.14159265358979323846,
                   axes["yaw"]["max_acceleration_deg_s2"].as<double>());
}

TEST(MixedStationConfig, EveryAxisDeclaresItsOwnRatesInEveryMode) {
  // Owner's ruling of 2026-09-28: "我建议还是两轴单独设置。我不能确保yaw和pitch真的能做到
  // 等同的加速度。所以分开设置（但是值可以设置成一样）". A shared declaration makes the two
  // axes' equality a side effect of the file's shape; per-axis blocks make it a value someone
  // wrote down, which is the only version that can disagree with the measured plant later.
  const YAML::Node modes = YAML::LoadFile((firmware_root() / "config/turret_mixed.yaml").string())
                              ["motion"]["modes"];
  for (const char* mode : {"manual", "auto_track", "auto_roam"}) {
    const auto axes = modes[mode]["axes"];
    ASSERT_TRUE(axes.IsDefined()) << mode << " declares rates at mode level instead of per axis";
    for (const char* axis : {"yaw", "pitch"}) {
      ASSERT_TRUE(axes[axis].IsDefined()) << mode << "." << axis << " is missing its own block";
      for (const char* which : {"maximum", "target"})
        for (const char* key : {"speed_deg_s", "acceleration_deg_s2", "jerk_deg_s3"})
          EXPECT_TRUE(axes[axis][which][key].IsDefined()) << mode << "." << axis << "." << which
                                                          << "." << key << " not spelled out";
    }
  }
}

TEST(MixedStationConfig, HardwareProfilePinsTheCommissionedTopology) {
  const auto loaded = config::mixed::load_mixed_hardware_profile(
      (firmware_root()/"config/mixed_hardware.yaml").string());
  ASSERT_TRUE(loaded.ok) << why(loaded);
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
  ASSERT_TRUE(station().ok) << why(station());
  const auto& m = station().config.motion;
  ASSERT_TRUE(m.configured);
  // No file uses motion.modes.<mode>.axes today: both axes share the mode pair. Until
  // 2026-09-28 the axis layer then trimmed yaw back to 10 deg/s in every mode, which is
  // why manual felt like two different sticks. The trim is gone, so what is asserted is
  // the invariant the owner asked for rather than a pair of numbers: in every mode the
  // two axes resolve to the SAME speeds. A future per-axis override must fail here and
  // be argued about, not arrive quietly through the axis maximum.
  for (int mode=0; mode<3; ++mode) {
    const auto yaw = control::resolve_motion(m.modes[mode][1], m.axis_maximum[1], {});
    const auto pitch = control::resolve_motion(m.modes[mode][0], m.axis_maximum[0], {});
    EXPECT_DOUBLE_EQ(yaw.target.speed, pitch.target.speed) << "mode " << mode;
    EXPECT_DOUBLE_EQ(yaw.maximum.speed, pitch.maximum.speed) << "mode " << mode;
    // 20 deg/s: what motion.modes declares for every mode. The axis maximum (30) only
    // trims above the ask; asserting 30 here would confuse the cap with the request.
    EXPECT_DOUBLE_EQ(yaw.maximum.speed, 20*kDeg2Rad) << "mode " << mode;
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

// Current mode is refused unless the operator has recorded the drive's preconditions. The rule
// lives in the parser precisely so this test can exist without a CAN socket; if someone moves the
// checks behind the backend again, these three cases are the tripwire that says so.
namespace {
std::string write_variant(const char* tag, const std::string& yaml) {
  const auto path = (std::filesystem::temp_directory_path() /
                     ("ota_mixed_" + std::string(tag) + ".yaml")).string();
  std::ofstream out(path);
  out << yaml;
  out.close();
  return path;
}

std::string current_mode_yaml(const std::string& extra) {
  std::ifstream in((firmware_root() / "config/mixed_hardware.yaml").string());
  std::stringstream buf;
  buf << in.rdbuf();
  auto text = buf.str();
  const auto at = text.find("control_mode: voltage");
  EXPECT_NE(at, std::string::npos) << "the commissioned profile no longer pins yaw to voltage";
  text.replace(at, std::string("control_mode: voltage").size(),
               "control_mode: current" + extra);
  return text;
}
}  // namespace

TEST(MixedCurrentMode, RefusedWithoutRecordedAcknowledgement) {
  const auto path = write_variant("no_ack", current_mode_yaml(""));
  const auto loaded = config::mixed::load_mixed_hardware_profile(path);
  EXPECT_FALSE(loaded.ok);
  const bool named = std::any_of(loaded.errors.begin(), loaded.errors.end(),
                                 [](const std::string& e) {
                                   return e.find("current_ring_verified") != std::string::npos;
                                 });
  EXPECT_TRUE(named) << "refusal must name the missing acknowledgement, not just say 'invalid'";
}

TEST(MixedCurrentMode, RefusedWhenTheHostClampExceedsTheMotorRating) {
  const auto path = write_variant(
      "hot_limit", current_mode_yaml(
          "\n    current_ring_verified: true\n    host_current_limit_a: 1.63"));
  const auto loaded = config::mixed::load_mixed_hardware_profile(path);
  EXPECT_FALSE(loaded.ok);
  const bool named = std::any_of(loaded.errors.begin(), loaded.errors.end(),
                                 [](const std::string& e) {
                                   return e.find("host_current_limit_a") != std::string::npos &&
                                          e.find("1.62") != std::string::npos;
                                 });
  EXPECT_TRUE(named) << "the refusal must say which bound was broken and what the rating is";
}

TEST(MixedCurrentMode, AcceptedWhenBothPreconditionsAreRecorded) {
  const auto path = write_variant(
      "ok", current_mode_yaml(
          "\n    current_ring_verified: true\n    host_current_limit_a: 0.8"));
  const auto loaded = config::mixed::load_mixed_hardware_profile(path);
  EXPECT_TRUE(loaded.ok) << why(loaded);
  EXPECT_EQ(loaded.profile.yaw.control_mode, config::mixed::ControlMode::Current);
  EXPECT_TRUE(loaded.profile.yaw.current_ring_verified);
  EXPECT_DOUBLE_EQ(loaded.profile.yaw.host_current_limit_a, 0.8);
}
