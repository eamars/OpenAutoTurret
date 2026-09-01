#pragma once
// OpenAutoTurret — versioned, validated YAML configuration (architecture §40).
//
// Every commissioning value from §58 lives here, never as a compile-time
// assumption. Unknown or "TBD" values fall back to conservative defaults and
// produce a (non-fatal) warning so the daemon can run at safe, slow speeds
// before the real numbers are measured; a missing *required* key or a bad
// schema version is a hard error.
#include <cstdint>
#include <string>
#include <vector>

namespace ota::config {

// §40 `can:` block.
struct CanConfig {
  std::string interface = "can0";
  int bitrate = 1000000;      // bits/s; CyberGear is 1 Mbit/s.
  int host_can_id = 0;        // the host's data2 field; must be in [0, 255].
};

// §40 `motors.<axis>:` block.
struct MotorConfig {
  int can_id = 0;             // target field, [0, 255].
  int direction_sign = 1;     // +1 or -1 (logical -> mechanical sense).
};

// §40 `axes.<axis>:` block. Units are degrees / deg/s / deg/s^2 / deg/s^3,
// exactly as written in the YAML; the trajectory layer converts to radians.
struct AxisLimitsConfig {
  struct TravelDeg {
    double min = 0.0;
    double max = 0.0;
  };
  TravelDeg expected_travel_deg;
  double soft_margin_deg = 0.0;
  double max_velocity_deg_s = 0.0;
  double max_acceleration_deg_s2 = 0.0;
  double max_jerk_deg_s3 = 0.0;
};

// §40 `homing.contact:` block (§58 params 8-13).
struct ContactConfig {
  double coarse_speed_deg_s = 0.0;
  double fine_speed_deg_s = 0.0;
  double current_or_effort_limit = 0.0;  // A (or N.m; validated positive).
  double stall_velocity_threshold = 0.0;  // rad/s; below this the axis is stalled.
  int contact_dwell_ms = 0;
  double backoff_deg = 0.0;
  double repeatability_deg = 0.0;
};

struct HomingConfig {
  ContactConfig contact;
};

// One step of the multi-axis homing plan (§25.2). Units are degrees for
// positions. `action` is "home_endpoint" | "move" | "home_full_range";
// `axis` is "pitch" | "yaw"; `endpoint` ("lower" | "upper") and `precision`
// ("coarse" | "fine") apply to the homing actions; `position_deg` applies to
// `move`.
struct HomingPlanActionConfig {
  std::string action;
  std::string axis;
  std::string endpoint = "lower";
  std::string precision = "fine";
  double position_deg = 0.0;
};

// §25.2 `homing_plan:` block — an ordered, executable list of homing steps.
struct HomingPlanConfig {
  std::vector<HomingPlanActionConfig> actions;
};

// §40 `tracking:` block (§58 params 19-20).
struct TrackingConfig {
  bool search_enabled_by_default = false;
  std::string target_lost_behavior = "hold";  // "hold" | "search"
};

// §40 `shutdown:` block (§58 param 4) + §33 park sequence parameters.
// The park positions (deg, in the homed logical frame) must lie inside the
// calibrated soft limits, not against the mechanical stop (§33.1).
struct ShutdownConfig {
  double yaw_park_deg = 0.0;
  double pitch_park_deg = 0.0;
  // §33.2 verification before the motors are de-energized.
  double pos_tolerance_deg = 0.5;    // |q - q_park| below this counts as "at park"
  double vel_tolerance_deg_s = 1.0;  // |v| below this counts as "settled"
  int dwell_ms = 500;                // verification must hold for this long
  double speed_deg_s = 10.0;         // speed limit for the park moves
};

// §38/§39 safety supervisor + watchdog tuning.
struct SafetyConfig {
  int feedback_max_age_ms = 100;     // §39.2: stale motor feedback -> no open-loop
  int deadline_max_us = 2000;        // §39.3: cycle longer than this = a miss
  int deadline_miss_threshold = 5;   // consecutive/short-window misses -> derate
  double motor_overtemp_c = 75.0;    // §38: over-temperature fault threshold
};

struct CameraConfig {
  std::string intrinsics_file;
  std::string extrinsics_file;
};

struct InstallationConfig {
  std::string pose_file;
};

struct PayloadConfig {
  // Phase 9 (§28.5, §41): the active payload profile name + its directory.
  std::string active_profile = "conservative";
  std::string profile_dir = "config/payload_profiles";
  // §27 OPTIONAL_PAYLOAD_RESPONSE_CHECK: run the response check once on
  // first hold, at boot (post-homing, pre-READY_HOLD).
  bool auto_verify = false;
};

struct TurretConfig {
  int schema_version = 1;
  CanConfig can;
  // Indexed by AxisId: [0] = pitch, [1] = yaw.
  MotorConfig motors[2];
  int control_loop_hz = 200;
  AxisLimitsConfig axes[2];
  HomingConfig homing;
  HomingPlanConfig homing_plan;
  TrackingConfig tracking;
  ShutdownConfig shutdown;
  SafetyConfig safety;
  CameraConfig camera;
  InstallationConfig installation;
  PayloadConfig payload;
};

struct LoadResult {
  bool ok = false;
  std::vector<std::string> errors;   // hard errors (block load).
  std::vector<std::string> warnings;  // conservative-default fallbacks.
  TurretConfig config;
};

// Load and validate `path`. On any hard error `ok` is false and `errors` is
// non-empty. "TBD"/missing commissioning values do NOT fail the load; they are
// replaced by conservative defaults and recorded in `warnings`.
LoadResult load_turret_config(const std::string& path);

}  // namespace ota::config
