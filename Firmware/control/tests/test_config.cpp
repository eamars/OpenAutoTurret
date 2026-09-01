// Unit tests for the versioned YAML config loader (architecture §40 / §58).
#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "config/turret_config.hpp"

namespace {

const char* kDir = "/tmp/ota_config_test";

std::string write_file(const std::string& name, const std::string& body) {
  std::string p = std::string(kDir) + "/" + name;
  std::ofstream f(p);
  f << body;
  f.close();
  return p;
}

// A minimal, fully-specified config (all required + commissioning values).
const std::string kFullConfig = R"(
schema_version: 1
can:
  interface: can0
  bitrate: 1000000
  host_can_id: 0
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control:
  loop_hz: 200
axes:
  pitch:
    expected_travel_deg: { min: -45, max: 45 }
    soft_margin_deg: 5
    max_velocity_deg_s: 40
    max_acceleration_deg_s2: 80
    max_jerk_deg_s3: 400
  yaw:
    expected_travel_deg: { min: -170, max: 170 }
    soft_margin_deg: 5
    max_velocity_deg_s: 60
    max_acceleration_deg_s2: 120
    max_jerk_deg_s3: 600
homing:
  contact:
    coarse_speed_deg_s: 12
    fine_speed_deg_s: 1.5
    current_or_effort_limit: 4.0
    stall_velocity_threshold: 0.6
    contact_dwell_ms: 250
    backoff_deg: 6
    repeatability_deg: 0.4
homing_plan:
  - action: home_full_range
    axis: yaw
    precision: fine
  - action: home_full_range
    axis: pitch
    precision: fine
tracking:
  search_enabled_by_default: false
  target_lost_behavior: hold
shutdown:
  yaw_park_deg: 10
  pitch_park_deg: -5
  pos_tolerance_deg: 0.5
  vel_tolerance_deg_s: 1.0
  dwell_ms: 500
  speed_deg_s: 10
safety:
  feedback_max_age_ms: 100
  deadline_max_us: 2000
  deadline_miss_threshold: 5
  motor_overtemp_c: 75.0
)";

}  // namespace

TEST(Config, ValidFullConfigLoads) {
  const std::string p = write_file("full.yaml", kFullConfig);
  auto r = ota::config::load_turret_config(p);
  EXPECT_TRUE(r.ok) << "errors: " << r.errors.size();
  for (auto& e : r.errors) std::cerr << "  err: " << e << "\n";
  EXPECT_EQ(r.config.schema_version, 1);
  EXPECT_EQ(r.config.can.interface, "can0");
  EXPECT_EQ(r.config.can.bitrate, 1000000);
  EXPECT_EQ(r.config.can.host_can_id, 0);
  EXPECT_EQ(r.config.motors[0].can_id, 100);
  EXPECT_EQ(r.config.motors[0].direction_sign, 1);
  EXPECT_EQ(r.config.motors[1].can_id, 101);
  EXPECT_EQ(r.config.motors[1].direction_sign, -1);
  EXPECT_EQ(r.config.control_loop_hz, 200);
  EXPECT_DOUBLE_EQ(r.config.axes[0].expected_travel_deg.min, -45.0);
  EXPECT_DOUBLE_EQ(r.config.axes[0].expected_travel_deg.max, 45.0);
  EXPECT_DOUBLE_EQ(r.config.axes[0].max_velocity_deg_s, 40.0);
  EXPECT_DOUBLE_EQ(r.config.axes[1].expected_travel_deg.min, -170.0);
  EXPECT_DOUBLE_EQ(r.config.homing.contact.coarse_speed_deg_s, 12.0);
  EXPECT_DOUBLE_EQ(r.config.homing.contact.fine_speed_deg_s, 1.5);
  EXPECT_EQ(r.config.homing.contact.contact_dwell_ms, 250);
  EXPECT_DOUBLE_EQ(r.config.shutdown.yaw_park_deg, 10.0);
  EXPECT_DOUBLE_EQ(r.config.shutdown.pitch_park_deg, -5.0);
  EXPECT_DOUBLE_EQ(r.config.shutdown.pos_tolerance_deg, 0.5);
  EXPECT_DOUBLE_EQ(r.config.shutdown.vel_tolerance_deg_s, 1.0);
  EXPECT_EQ(r.config.shutdown.dwell_ms, 500);
  EXPECT_DOUBLE_EQ(r.config.shutdown.speed_deg_s, 10.0);
  EXPECT_EQ(r.config.safety.feedback_max_age_ms, 100);
  EXPECT_EQ(r.config.safety.deadline_max_us, 2000);
  EXPECT_EQ(r.config.safety.deadline_miss_threshold, 5);
  EXPECT_DOUBLE_EQ(r.config.safety.motor_overtemp_c, 75.0);
  EXPECT_EQ(r.config.tracking.target_lost_behavior, "hold");
  // Homing plan parsed.
  ASSERT_EQ(r.config.homing_plan.actions.size(), 2u);
  EXPECT_EQ(r.config.homing_plan.actions[0].action, "home_full_range");
  EXPECT_EQ(r.config.homing_plan.actions[0].axis, "yaw");
  EXPECT_EQ(r.config.homing_plan.actions[0].precision, "fine");
  EXPECT_EQ(r.config.homing_plan.actions[1].axis, "pitch");
}

TEST(Config, HomingPlanWithAllActionTypesLoads) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
homing_plan:
  - action: home_endpoint
    axis: pitch
    endpoint: lower
    precision: coarse
  - action: move
    axis: pitch
    position_deg: 20
  - action: home_full_range
    axis: yaw
    precision: fine
)";
  const std::string p = write_file("plan_all.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_TRUE(r.ok) << "errors: " << r.errors.size();
  for (auto& e : r.errors) std::cerr << "  err: " << e << "\n";
  ASSERT_EQ(r.config.homing_plan.actions.size(), 3u);
  EXPECT_EQ(r.config.homing_plan.actions[0].action, "home_endpoint");
  EXPECT_EQ(r.config.homing_plan.actions[0].endpoint, "lower");
  EXPECT_EQ(r.config.homing_plan.actions[0].precision, "coarse");
  EXPECT_EQ(r.config.homing_plan.actions[1].action, "move");
  EXPECT_DOUBLE_EQ(r.config.homing_plan.actions[1].position_deg, 20.0);
  EXPECT_EQ(r.config.homing_plan.actions[2].action, "home_full_range");
}

TEST(Config, MissingHomingPlanFails) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
)";
  const std::string p = write_file("noplan.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (auto& e : r.errors)
    if (e.find("homing_plan") != std::string::npos) found = true;
  EXPECT_TRUE(found) << "expected a homing_plan error";
}

TEST(Config, BadHomingPlanActionFails) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
homing_plan:
  - action: teleport
    axis: pitch
)";
  const std::string p = write_file("badplan.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (auto& e : r.errors)
    if (e.find("action must be") != std::string::npos) found = true;
  EXPECT_TRUE(found) << "expected an action error";
}

TEST(Config, TbdValuesFallBackToConservativeDefaults) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
axes:
  pitch:
    expected_travel_deg: { min: TBD, max: TBD }
    soft_margin_deg: TBD
    max_velocity_deg_s: TBD
    max_acceleration_deg_s2: TBD
    max_jerk_deg_s3: TBD
  yaw:
    expected_travel_deg: { min: TBD, max: TBD }
    soft_margin_deg: TBD
    max_velocity_deg_s: TBD
    max_acceleration_deg_s2: TBD
    max_jerk_deg_s3: TBD
homing:
  contact:
    coarse_speed_deg_s: TBD
    fine_speed_deg_s: TBD
    current_or_effort_limit: TBD
    stall_velocity_threshold: TBD
    contact_dwell_ms: TBD
    backoff_deg: TBD
    repeatability_deg: TBD
homing_plan:
  - { action: home_full_range, axis: yaw }
  - { action: home_full_range, axis: pitch }
)";
  const std::string p = write_file("tbd.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_TRUE(r.ok) << "errors: " << r.errors.size();
  for (auto& e : r.errors) std::cerr << "  err: " << e << "\n";
  // Conservative defaults applied.
  EXPECT_DOUBLE_EQ(r.config.axes[0].expected_travel_deg.min, -120.0);
  EXPECT_DOUBLE_EQ(r.config.axes[0].expected_travel_deg.max, 120.0);
  EXPECT_DOUBLE_EQ(r.config.axes[0].soft_margin_deg, 5.0);
  EXPECT_DOUBLE_EQ(r.config.axes[0].max_velocity_deg_s, 30.0);
  EXPECT_DOUBLE_EQ(r.config.axes[1].expected_travel_deg.min, -180.0);
  EXPECT_DOUBLE_EQ(r.config.axes[1].expected_travel_deg.max, 180.0);
  EXPECT_DOUBLE_EQ(r.config.homing.contact.coarse_speed_deg_s, 10.0);
  // Fine approach runs at the characterized smooth speed (20 deg/s); 10 deg/s
  // has position-dependent breakaway stalls (drive_current_friction_tuning.md).
  EXPECT_DOUBLE_EQ(r.config.homing.contact.fine_speed_deg_s, 20.0);
  EXPECT_EQ(r.config.homing.contact.contact_dwell_ms, 200);
  // Warnings recorded for each TBD fallback.
  EXPECT_FALSE(r.warnings.empty());
}

TEST(Config, MissingRequiredKeyFails) {
  const std::string body = R"(
schema_version: 1
can:
  bitrate: 1000000
  host_can_id: 0
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
)";
  const std::string p = write_file("missing.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (auto& e : r.errors)
    if (e.find("can.interface") != std::string::npos) found = true;
  EXPECT_TRUE(found) << "expected a can.interface error";
}

TEST(Config, BadSchemaVersionFails) {
  std::string body = kFullConfig;
  body.replace(body.find("schema_version: 1"), 19, "schema_version: 2");
  const std::string p = write_file("badver.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
}

TEST(Config, BadDirectionSignFails) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 2 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
)";
  const std::string p = write_file("badsign.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
}

TEST(Config, SameCanIdFails) {
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 100, direction_sign: -1 }
control: { loop_hz: 200 }
)";
  const std::string p = write_file("sameid.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
}

TEST(Config, MissingFileFails) {
  auto r = ota::config::load_turret_config("/tmp/ota_config_test/does_not_exist.yaml");
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.errors.empty());
}

TEST(Config, MalformedYamlFails) {
  const std::string body = "schema_version: 1\n  bad_indent: [unclosed\n";
  const std::string p = write_file("malformed.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_FALSE(r.ok);
}

TEST(Config, DefaultsAreSafeAndConservative) {
  // The built-in defaults (used when the file omits the commissioning section)
  // must be slow and wide enough to be safe on first boot.
  const std::string body = R"(
schema_version: 1
can: { interface: can0, bitrate: 1000000, host_can_id: 0 }
motors:
  pitch: { can_id: 100, direction_sign: 1 }
  yaw:   { can_id: 101, direction_sign: -1 }
control: { loop_hz: 200 }
homing_plan:
  - { action: home_full_range, axis: yaw }
  - { action: home_full_range, axis: pitch }
)";
  const std::string p = write_file("defaults.yaml", body);
  auto r = ota::config::load_turret_config(p);
  EXPECT_TRUE(r.ok) << "errors: " << r.errors.size();
  for (auto& e : r.errors) std::cerr << "  err: " << e << "\n";
  // Speeds/accels/jerks are finite and positive.
  EXPECT_GT(r.config.axes[0].max_velocity_deg_s, 0.0);
  EXPECT_GT(r.config.axes[0].max_acceleration_deg_s2, 0.0);
  EXPECT_GT(r.config.axes[0].max_jerk_deg_s3, 0.0);
  EXPECT_GT(r.config.homing.contact.coarse_speed_deg_s, 0.0);
  EXPECT_GT(r.config.homing.contact.fine_speed_deg_s, 0.0);
  // NOTE: fine_speed is intentionally NOT required to be <= coarse_speed.
  // Contact precision comes from the mechanical stop + the repeatability check,
  // not the approach speed; and at the current friction/load the fine approach
  // must run at the characterized SMOOTH speed (20 deg/s) to avoid
  // position-dependent breakaway stalls (drive_current_friction_tuning.md §1),
  // which can be faster than the coarse speed.
  // The pitch band contains the observed ~-86 deg position.
  EXPECT_LE(r.config.axes[0].expected_travel_deg.min, -86.0);
  EXPECT_GE(r.config.axes[0].expected_travel_deg.max, 86.0);
  // Travel band is valid.
  EXPECT_LT(r.config.axes[0].expected_travel_deg.min,
            r.config.axes[0].expected_travel_deg.max);
}
