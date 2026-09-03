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
  // CAN PHY selection (can_transport.hpp): "socketcan" (MCP2515 HAT,
  // interface = can0) or "yousee" (USB AT adapter, interface = /dev/ttyUSB0).
  std::string backend = "socketcan";
  std::string interface = "can0";
  int bitrate = 1000000;      // CAN-side bits/s; CyberGear is 1 Mbit/s.
  int uart_baud = 921600;     // yousee only: the adapter UART side.
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
  // Adaptive-current homing (push-through, §22): the INITIAL drive current
  // limit (A) for this axis. The homing raises it by limit_cur_step_a on each
  // false-contact latch, up to limit_cur_max_a. Written to LimitCur (0x7018)
  // on boot and as the homing pushes it up. 0 = use the HomingParams default.
  double limit_cur_a = 0.0;
};

// §40 `homing.contact:` block (§58 params 8-13).
struct ContactConfig {
  double coarse_speed_deg_s = 0.0;
  double fine_speed_deg_s = 0.0;
  double current_or_effort_limit = 0.0;  // A (or N.m; validated positive).
  double stall_velocity_threshold = 0.0;  // rad/s; below this the axis is stalled.
  double v_move_threshold = 0.0;  // rad/s; jitter/recovery "moving" threshold,
                                  // below the approach speed (distinct from
                                  // stall_velocity_threshold). 0 = use the
                                  // ContactDetectorParams default (0.10).
  double effort_hard_contact_nm = 0.0;  // N.m; sustained effort this high declares
                                        // contact even without prior motion (measured
                                        // end-stop push plateau 0.43-0.73 N.m, P0e).
  double motion_history_velocity = 0.0;  // rad/s; the axis must have reached this
                                         // (position-derived) speed since the approach
                                         // began for a motion-based contact.
  int contact_dwell_ms = 0;
  double backoff_deg = 0.0;
  double repeatability_deg = 0.0;
  int repeatability_retries = 0;  // second-approach re-runs on a non-repeatable
                                  // q2 (p3f friction stall); 0 = HomingParams
                                  // default (2)
  // --- Adaptive-current homing (push-through, §22) ---
  // The yaw is a ~360 deg axis whose mid-travel friction stalls (false
  // contacts) beat a low LimitCur. On each coarse-contact latch the homing
  // raises the drive current by this step (A) and keeps driving, until a
  // consistent end-stop or the safe upper limit. 0 = use the HomingParams
  // default.
  double limit_cur_step_a = 0.0;
  double limit_cur_max_a = 0.0;    // safe upper limit (A); 0 = default
  double max_rotation_deg = 0.0;   // coarse-approach rotation cap (deg); 0 = default
  double torque_safety_nm = 0.0;   // |tau| above this aborts the push (N.m); 0 = default
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

// §40 `tracking:` block (§58 params 19-20) + the Part-2 S1 ingest wiring.
struct TrackingConfig {
  // Auto-enable tracking once the homing gates pass (§38.1). FALSE by default:
  // a station must never start following pixels because a config file said so.
  // The `start_tracking` developer command (§42.2) is the operator path.
  bool enabled = false;
  bool search_enabled_by_default = false;
  std::string target_lost_behavior = "hold";  // "hold" | "search"
  // §16 tracking speed limit (deg/s). The architecture caps tracking at 30 deg/s.
  double track_speed_deg_s = 30.0;
  // §36/§49 search sweep speed (deg/s) — reduced relative to tracking.
  double search_speed_deg_s = 10.0;
  // §49 search sweep half-span (deg) around the ready pose. The loop clamps it
  // strictly inside the homed soft limits (minus the safety margin).
  double search_span_deg = 45.0;
  // §13.3 actuation horizon: control + motor-response latency the estimator
  // predicts across (ms each).
  int control_delay_ms = 20;
  int motor_response_ms = 20;
  // §34: how stale a valid measurement may be and still count as "detected".
  int fresh_threshold_ms = 100;
  // §34 coast window: keep extrapolating for this long after measurements stop.
  int coast_timeout_ms = 200;
  // §34: declare the target lost after this long without a valid measurement.
  int lost_timeout_ms = 1000;
  // §13 alpha-beta gains.
  double estimator_alpha = 0.8;
  double estimator_beta = 0.3;
};

// §5.1/§6.1 vision ingest (Part 2, S1): what controld binds so visiond can
// publish TargetMeasurements to it.
struct VisionConfig {
  std::string socket_path = "/tmp/ota_vision.sock";
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
  // Speed limit (deg/s) of the Verify/Dwell position-mode hold. Must be
  // non-zero (LimitSpd=0 pins the drive's position loop — it could never
  // pull an axis back into the §33.2 window; p3 2026-09-02) and low (the
  // position loop overshoots a fast correction by ~speed x 0.13 s of
  // velocity-loop lag; 2 deg/s lands a ~0.8 deg residual inside the 0.5 deg
  // window in one pass).
  double verify_speed_deg_s = 2.0;
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
  // §44 "safe central region": per-axis half-span (deg) of the check region,
  // centered on each axis's pose at check start and intersected with the
  // homed soft limits.
  double check_region_half_span_deg = 10.0;
  // Drive current limit (A) applied to BOTH axes for the duration of the
  // check. The post-homing LimitCur (3 A pitch / 1 A yaw) is marginal for a
  // 2 deg position-mode step (the yaw creeps at 1 A), so the check raises
  // both to this value (5 A, well under the 10 A station cap) and leaves it
  // there — the §33.2/hold position holds are MORE authoritative at 5 A, and
  // the boot speed-mode hold already uses this same 5 A default.
  double check_current_a = 5.0;
  // Drive inner speed-loop gains (both axes) applied at check start. The stock
  // CyberGear gains (SpdKp=1.0, SpdKi=0.002) are too weak to hold the
  // position-mode speed limit against the pitch's gravity load: the
  // "against-gravity" half of the 2 deg step creeps at a fraction of the
  // commanded rate and times out, while the "with-gravity" half is assisted and
  // is fast. Raising them lets the inner loop hold the commanded rate against
  // gravity so the step response is the drive's controlled (mass-sensitive)
  // response. Still bounded by the current/torque limits, so safe. 5x the
  // stock values; tune here if the step still creeps or (over)shoots.
  double check_spd_kp = 5.0;
  double check_spd_ki = 0.02;
};

// §72: the v3 block. Parsed and validated at load, and *nothing here is optional at
// runtime* — an absent block keeps today's behaviour exactly (the roam region derived from
// the station's own limits, the built-in 300 ms lease, the sanctioned step sizes). Naming
// a value is how an operator takes responsibility for it; not naming one is not the same
// as naming a default, and the two must not look alike in the file.
struct V3Config {
  // Boot mode. Only MANUAL is accepted (§16/§52): a station that starts sweeping or
  // tracking by itself on power-up has made a decision the operator never authorised,
  // and no config file in this project gets to make one.
  std::string default_mode = "MANUAL";

  // §33: a named sweep region, in the same degrees the rest of the file uses. Yaw only —
  // the pitch reference is a single pose, not a range, because the sweep holds pitch and
  // walks yaw. Unset means "derive it from what the station proved during homing".
  bool has_roam_region = false;
  double roam_yaw_min_deg = 0.0;
  double roam_yaw_max_deg = 0.0;
  bool has_roam_pitch = false;
  double roam_pitch_deg = 0.0;
  double roam_velocity_deg_s = 0.0;  // 0 = derive from the tracking search speed

  // §19/§20/§21: when a value is absent the controller's own default stands, and the
  // distinction is kept per key rather than per block — naming only coast_ms must not
  // silently re-pick defaults for the reacquisition numbers beside it.
  int auto_track_coast_ms = 0;
  int auto_track_lost_hold_ms = 0;
  int auto_track_reacquire_window_ms = 0;
  float auto_track_medium_min = 0.0f;
  float auto_track_high_min = 0.0f;
  float auto_track_medium_scale = 0.0f;
  float auto_track_low_scale = 0.0f;
  float reacquire_threshold = 0.0f;
  float ambiguous_match_margin = 0.0f;

  // §38/§41: the dead-man timings and the step sizes offered.
  int jog_keepalive_ms = 0;   // 0 = the controller's default
  int jog_lease_ms = 0;
  std::vector<double> step_sizes_deg;  // empty = the sanctioned three
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
  VisionConfig vision;
  ShutdownConfig shutdown;
  SafetyConfig safety;
  CameraConfig camera;
  InstallationConfig installation;
  PayloadConfig payload;
  V3Config v3;   // §72
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
