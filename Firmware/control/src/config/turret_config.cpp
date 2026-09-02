// OpenAutoTurret — YAML config loader (architecture §40 / §58).
#include "config/turret_config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>

#include <yaml-cpp/yaml.h>

namespace ota::config {
namespace {

// Trim leading/trailing ASCII whitespace.
std::string trim(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

// A value is "unspecified" if the key is absent, null, empty, or the literal
// string "TBD" (any case). Commissioning params fall back to defaults for these.
bool unspecified(const YAML::Node& v) {
  if (!v.IsDefined() || v.IsNull()) return true;
  if (v.IsScalar()) {
    const std::string s = trim(v.Scalar());
    if (s.empty()) return true;
    std::string up = s;
    std::transform(up.begin(), up.end(), up.begin(),
                   [](unsigned char c) { return std::toupper(c); });
    return up == "TBD";
  }
  return false;
}

// Fetch node[key] safely (an undefined/mapping-less node yields an undefined
// node, which the callers treat as "unspecified").
YAML::Node fetch(const YAML::Node& node, const std::string& key) {
  if (node.IsDefined() && node.IsMap()) return node[key];
  return YAML::Node();
}

// Conservative defaults for the §58 commissioning parameters. These keep the
// daemon safe (slow, wide bands) until the real values are measured; the pitch
// band is wide enough to contain the observed ~-86 deg position so nothing is
// treated as out-of-band on first boot.
struct Defaults {
  double pitch_travel_min = -120.0;
  double pitch_travel_max = 120.0;
  double yaw_travel_min = -180.0;
  double yaw_travel_max = 180.0;
  double soft_margin_deg = 5.0;
  double max_velocity_deg_s = 30.0;
  double max_acceleration_deg_s2 = 60.0;
  double max_jerk_deg_s3 = 300.0;
  double coarse_speed_deg_s = 10.0;
  // Fine approach speed (deg/s). Characterized (drive_current_friction_tuning.md
  // §1, P0j-P0m): at the current friction/load the yaw stick-slips with
  // position-dependent breakaway stalls at <=10 deg/s but runs smooth at
  // 20 deg/s. The fine approach must be REPEATABLE, so it uses the smooth
  // speed to avoid latching a false contact on a breakaway stall; the
  // ContactDetector jitter gate is a safety net, and the limit_cur sizing
  // (Phase A.5) is the root-cause fix. Matches HomingParams::fine_speed_rad_s.
  double fine_speed_deg_s = 20.0;
  double current_or_effort_limit = 3.0;
  double stall_velocity_threshold = 0.5;
  // Jitter/recovery "moving" threshold: below the approach speed (10/20 deg/s
  // = 0.175/0.349 rad/s) and above filtered rest noise. Tuned in A.3.
  double v_move_threshold = 0.1;
  double effort_hard_contact_nm = 0.40;
  double motion_history_velocity = 0.05;
  int contact_dwell_ms = 200;
  double backoff_deg = 5.0;
  double repeatability_deg = 0.5;
  int repeatability_retries = 2;  // p3f: second-approach re-runs on a
                                  // non-repeatable q2 (friction-stall false
                                  // contact); 0 in YAML = this default
  // Adaptive-current homing (push-through, §22). Defaults match
  // HomingParams; YAML overrides are authoritative.
  double limit_cur_step_a = 1.0;   // +1 A per false-contact latch
  double limit_cur_max_a = 10.0;   // safe upper limit (A); under the 23 A drive max
  double max_rotation_deg = 360.0; // coarse-approach rotation cap (deg)
  double torque_safety_nm = 10.0;  // |tau| above this aborts the push (N.m)
  double yaw_park_deg = 0.0;
  double pitch_park_deg = 0.0;
  double park_pos_tolerance_deg = 0.5;
  double park_vel_tolerance_deg_s = 1.0;
  int park_dwell_ms = 500;
  double park_speed_deg_s = 10.0;
  double park_verify_speed_deg_s = 2.0;
  int feedback_max_age_ms = 100;
  int deadline_max_us = 2000;
  int deadline_miss_threshold = 5;
  double motor_overtemp_c = 75.0;
};

double opt_double(const YAML::Node& node, const std::string& key,
                  const std::string& path, double def,
                  std::vector<std::string>& warn) {
  const YAML::Node v = fetch(node, key);
  if (unspecified(v)) {
    warn.push_back(path + ": not specified; using conservative default " +
                  std::to_string(def));
    return def;
  }
  try {
    return v.as<double>();
  } catch (const YAML::Exception&) {
    warn.push_back(path + ": could not parse as number; using default " +
                  std::to_string(def));
    return def;
  }
}

int opt_int(const YAML::Node& node, const std::string& key,
            const std::string& path, int def, std::vector<std::string>& warn) {
  return static_cast<int>(opt_double(node, key, path, static_cast<double>(def), warn));
}

bool opt_bool(const YAML::Node& node, const std::string& key,
              const std::string& path, bool def,
              std::vector<std::string>& warn) {
  const YAML::Node v = fetch(node, key);
  if (unspecified(v)) return def;
  try {
    return v.as<bool>();
  } catch (const YAML::Exception&) {
    warn.push_back(path + ": could not parse as bool; using default " +
                  std::string(def ? "true" : "false"));
    return def;
  }
}

std::string opt_string(const YAML::Node& node, const std::string& key,
                       const std::string& path, const std::string& def,
                       std::vector<std::string>& warn) {
  const YAML::Node v = fetch(node, key);
  if (unspecified(v)) {
    warn.push_back(path + ": not specified; using default '" + def + "'");
    return def;
  }
  try {
    return v.as<std::string>();
  } catch (const YAML::Exception&) {
    warn.push_back(path + ": could not parse as string; using default '" + def + "'");
    return def;
  }
}

// Required scalar: missing/unspecified or unparsable is a hard error.
double req_double(const YAML::Node& node, const std::string& key,
                  const std::string& path, std::vector<std::string>& err) {
  const YAML::Node v = fetch(node, key);
  if (unspecified(v)) {
    err.push_back("missing required value: " + path);
    return 0.0;
  }
  try {
    return v.as<double>();
  } catch (const YAML::Exception&) {
    err.push_back(path + ": could not parse as number");
    return 0.0;
  }
}

int req_int(const YAML::Node& node, const std::string& key,
            const std::string& path, std::vector<std::string>& err) {
  return static_cast<int>(req_double(node, key, path, err));
}

std::string req_string(const YAML::Node& node, const std::string& key,
                       const std::string& path, std::vector<std::string>& err) {
  const YAML::Node v = fetch(node, key);
  if (unspecified(v)) {
    err.push_back("missing required value: " + path);
    return "";
  }
  try {
    return v.as<std::string>();
  } catch (const YAML::Exception&) {
    err.push_back(path + ": could not parse as string");
    return "";
  }
}

void load_motor(const YAML::Node& mnode, const std::string& path,
                MotorConfig& out, std::vector<std::string>& err) {
  out.can_id = req_int(mnode, "can_id", path + ".can_id", err);
  if (out.can_id < 0 || out.can_id > 255)
    err.push_back(path + ".can_id out of range [0,255]: " + std::to_string(out.can_id));
  out.direction_sign = req_int(mnode, "direction_sign", path + ".direction_sign", err);
  if (out.direction_sign != 1 && out.direction_sign != -1)
    err.push_back(path + ".direction_sign must be +1 or -1, got " +
                  std::to_string(out.direction_sign));
}

void load_axis(const YAML::Node& anode, const std::string& name, AxisLimitsConfig& out,
               std::vector<std::string>& warn, std::vector<std::string>& err) {
  const std::string p = "axes." + name + ".";
  const double tmin = (name == "pitch") ? Defaults().pitch_travel_min : Defaults().yaw_travel_min;
  const double tmax = (name == "pitch") ? Defaults().pitch_travel_max : Defaults().yaw_travel_max;
  const YAML::Node tr = fetch(anode, "expected_travel_deg");
  out.expected_travel_deg.min = opt_double(tr, "min", p + "expected_travel_deg.min", tmin, warn);
  out.expected_travel_deg.max = opt_double(tr, "max", p + "expected_travel_deg.max", tmax, warn);
  out.soft_margin_deg = opt_double(anode, "soft_margin_deg", p + "soft_margin_deg",
                                   Defaults().soft_margin_deg, warn);
  out.max_velocity_deg_s = opt_double(anode, "max_velocity_deg_s", p + "max_velocity_deg_s",
                                      Defaults().max_velocity_deg_s, warn);
  out.max_acceleration_deg_s2 =
      opt_double(anode, "max_acceleration_deg_s2", p + "max_acceleration_deg_s2",
                 Defaults().max_acceleration_deg_s2, warn);
  out.max_jerk_deg_s3 = opt_double(anode, "max_jerk_deg_s3", p + "max_jerk_deg_s3",
                                   Defaults().max_jerk_deg_s3, warn);
  out.limit_cur_a = opt_double(anode, "limit_cur_a", p + "limit_cur_a", 0.0, warn);
  if (out.limit_cur_a < 0.0) err.push_back(p + "limit_cur_a must be >= 0");
  if (out.expected_travel_deg.min >= out.expected_travel_deg.max)
    err.push_back(p + "expected_travel_deg.min must be < max");
  if (out.soft_margin_deg < 0.0) err.push_back(p + "soft_margin_deg must be >= 0");
  if (out.max_velocity_deg_s <= 0.0) err.push_back(p + "max_velocity_deg_s must be > 0");
  if (out.max_acceleration_deg_s2 <= 0.0)
    err.push_back(p + "max_acceleration_deg_s2 must be > 0");
  if (out.max_jerk_deg_s3 <= 0.0) err.push_back(p + "max_jerk_deg_s3 must be > 0");
}

void load_contact(const YAML::Node& hc, HomingConfig& out, std::vector<std::string>& warn,
                  std::vector<std::string>& err) {
  const std::string p = "homing.contact.";
  const YAML::Node ct = fetch(hc, "contact");
  auto& cc = out.contact;
  cc.coarse_speed_deg_s = opt_double(ct, "coarse_speed_deg_s", p + "coarse_speed_deg_s",
                                     Defaults().coarse_speed_deg_s, warn);
  cc.fine_speed_deg_s = opt_double(ct, "fine_speed_deg_s", p + "fine_speed_deg_s",
                                   Defaults().fine_speed_deg_s, warn);
  cc.current_or_effort_limit =
      opt_double(ct, "current_or_effort_limit", p + "current_or_effort_limit",
                 Defaults().current_or_effort_limit, warn);
  cc.stall_velocity_threshold =
      opt_double(ct, "stall_velocity_threshold", p + "stall_velocity_threshold",
                 Defaults().stall_velocity_threshold, warn);
  cc.v_move_threshold =
      opt_double(ct, "v_move_threshold", p + "v_move_threshold",
                 Defaults().v_move_threshold, warn);
  cc.effort_hard_contact_nm =
      opt_double(ct, "effort_hard_contact_nm", p + "effort_hard_contact_nm",
                 Defaults().effort_hard_contact_nm, warn);
  cc.motion_history_velocity =
      opt_double(ct, "motion_history_velocity", p + "motion_history_velocity",
                 Defaults().motion_history_velocity, warn);
  cc.contact_dwell_ms = opt_int(ct, "contact_dwell_ms", p + "contact_dwell_ms",
                                Defaults().contact_dwell_ms, warn);
  cc.backoff_deg = opt_double(ct, "backoff_deg", p + "backoff_deg", Defaults().backoff_deg, warn);
  cc.repeatability_deg = opt_double(ct, "repeatability_deg", p + "repeatability_deg",
                                    Defaults().repeatability_deg, warn);
  cc.repeatability_retries =
      opt_int(ct, "repeatability_retries", p + "repeatability_retries",
              Defaults().repeatability_retries, warn);
  cc.limit_cur_step_a =
      opt_double(ct, "limit_cur_step_a", p + "limit_cur_step_a",
                 Defaults().limit_cur_step_a, warn);
  cc.limit_cur_max_a =
      opt_double(ct, "limit_cur_max_a", p + "limit_cur_max_a",
                 Defaults().limit_cur_max_a, warn);
  cc.max_rotation_deg =
      opt_double(ct, "max_rotation_deg", p + "max_rotation_deg",
                 Defaults().max_rotation_deg, warn);
  cc.torque_safety_nm =
      opt_double(ct, "torque_safety_nm", p + "torque_safety_nm",
                 Defaults().torque_safety_nm, warn);
  // limit_cur_step_a / limit_cur_max_a: legacy adaptive-raise knobs, now
  // optional (0 disables; speed-mode homing runs at a fixed current). No >0
  // requirement.
  if (cc.limit_cur_step_a < 0.0) err.push_back(p + "limit_cur_step_a must be >= 0");
  if (cc.limit_cur_max_a < 0.0) err.push_back(p + "limit_cur_max_a must be >= 0");
  if (cc.max_rotation_deg <= 0.0) err.push_back(p + "max_rotation_deg must be > 0");
  if (cc.torque_safety_nm <= 0.0) err.push_back(p + "torque_safety_nm must be > 0");
  if (cc.coarse_speed_deg_s <= 0.0) err.push_back(p + "coarse_speed_deg_s must be > 0");
  if (cc.fine_speed_deg_s <= 0.0) err.push_back(p + "fine_speed_deg_s must be > 0");
  if (cc.current_or_effort_limit <= 0.0)
    err.push_back(p + "current_or_effort_limit must be > 0");
  if (cc.stall_velocity_threshold <= 0.0)
    err.push_back(p + "stall_velocity_threshold must be > 0");
  if (cc.v_move_threshold <= 0.0)
    err.push_back(p + "v_move_threshold must be > 0");
  if (cc.effort_hard_contact_nm <= 0.0)
    err.push_back(p + "effort_hard_contact_nm must be > 0");
  if (cc.motion_history_velocity <= 0.0)
    err.push_back(p + "motion_history_velocity must be > 0");
  if (cc.contact_dwell_ms <= 0) err.push_back(p + "contact_dwell_ms must be > 0");
  if (cc.backoff_deg <= 0.0) err.push_back(p + "backoff_deg must be > 0");
  if (cc.repeatability_deg <= 0.0) err.push_back(p + "repeatability_deg must be > 0");
  if (cc.repeatability_retries < 0)
    err.push_back(p + "repeatability_retries must be >= 0");
}

// §25.2 homing_plan: an ordered list of homing steps. A missing/empty plan is
// a hard error (the daemon needs a plan to home); each action is validated for
// a known action/axis/endpoint/precision and, for `move`, a position.
void load_homing_plan(const YAML::Node& root, HomingPlanConfig& out,
                      std::vector<std::string>& warn, std::vector<std::string>& err) {
  const YAML::Node plan = fetch(root, "homing_plan");
  if (!plan.IsDefined() || !plan.IsSequence() || plan.size() == 0) {
    err.push_back("missing or empty required section: homing_plan");
    return;
  }
  int idx = 0;
  for (const YAML::Node& a : plan) {
    if (!a.IsDefined() || !a.IsMap()) {
      err.push_back("homing_plan[" + std::to_string(idx) + "] must be a mapping");
      ++idx;
      continue;
    }
    const std::string p = "homing_plan[" + std::to_string(idx) + "].";
    HomingPlanActionConfig ac;
    ac.action = req_string(a, "action", p + "action", err);
    ac.axis = req_string(a, "axis", p + "axis", err);
    if (ac.action != "home_endpoint" && ac.action != "move" &&
        ac.action != "home_full_range")
      err.push_back(p + "action must be home_endpoint|move|home_full_range, got '" +
                    ac.action + "'");
    if (ac.axis != "pitch" && ac.axis != "yaw")
      err.push_back(p + "axis must be pitch|yaw, got '" + ac.axis + "'");
    if (ac.action == "home_endpoint") {
      ac.endpoint = opt_string(a, "endpoint", p + "endpoint", "lower", warn);
      ac.precision = opt_string(a, "precision", p + "precision", "fine", warn);
      if (ac.endpoint != "lower" && ac.endpoint != "upper")
        err.push_back(p + "endpoint must be lower|upper, got '" + ac.endpoint + "'");
      if (ac.precision != "coarse" && ac.precision != "fine")
        err.push_back(p + "precision must be coarse|fine, got '" + ac.precision + "'");
    } else if (ac.action == "home_full_range") {
      ac.precision = opt_string(a, "precision", p + "precision", "fine", warn);
      if (ac.precision != "coarse" && ac.precision != "fine")
        err.push_back(p + "precision must be coarse|fine, got '" + ac.precision + "'");
    } else {  // move
      ac.position_deg = req_double(a, "position_deg", p + "position_deg", err);
    }
    out.actions.push_back(ac);
    ++idx;
  }
}

}  // namespace

LoadResult load_turret_config(const std::string& path) {
  LoadResult r;
  auto& c = r.config;
  auto& err = r.errors;
  auto& warn = r.warnings;

  YAML::Node root;
  try {
    std::ifstream f(path);
    if (!f.is_open()) {
      err.push_back("cannot open config file: " + path);
      return r;
    }
    root = YAML::Load(f);
  } catch (const YAML::Exception& e) {
    err.push_back(std::string("YAML parse error in ") + path + ": " + e.msg);
    return r;
  }
  if (!root.IsMap()) {
    err.push_back("config root must be a mapping");
    return r;
  }

  // schema_version (required, must be 1).
  const YAML::Node sv = fetch(root, "schema_version");
  if (unspecified(sv)) {
    err.push_back("missing required value: schema_version");
  } else {
    c.schema_version = sv.as<int>();
    if (c.schema_version != 1) {
      err.push_back("unsupported schema_version " + std::to_string(c.schema_version) +
                    " (this build supports 1)");
    }
  }

  // can (required).
  const YAML::Node can = fetch(root, "can");
  if (!can.IsDefined() || !can.IsMap()) {
    err.push_back("missing required section: can");
  } else {
    c.can.backend = opt_string(can, "backend", "can.backend", "socketcan", warn);
    c.can.interface = req_string(can, "interface", "can.interface", err);
    c.can.bitrate = static_cast<int>(req_double(can, "bitrate", "can.bitrate", err));
    c.can.uart_baud = opt_int(can, "uart_baud", "can.uart_baud", 921600, warn);
    c.can.host_can_id = req_int(can, "host_can_id", "can.host_can_id", err);
    if (c.can.backend != "socketcan" && c.can.backend != "yousee")
      err.push_back("can.backend must be 'socketcan' or 'yousee'");
    if (c.can.interface.empty()) err.push_back("can.interface must be non-empty");
    if (c.can.bitrate <= 0) err.push_back("can.bitrate must be > 0");
    if (c.can.uart_baud <= 0) err.push_back("can.uart_baud must be > 0");
    if (c.can.host_can_id < 0 || c.can.host_can_id > 255)
      err.push_back("can.host_can_id out of range [0,255]");
  }

  // motors (required; pitch and yaw can_id must differ).
  const YAML::Node motors = fetch(root, "motors");
  if (!motors.IsDefined() || !motors.IsMap()) {
    err.push_back("missing required section: motors");
  } else {
    const YAML::Node p = fetch(motors, "pitch");
    const YAML::Node y = fetch(motors, "yaw");
    if (!p.IsDefined() || !p.IsMap()) err.push_back("missing required section: motors.pitch");
    else load_motor(p, "motors.pitch", c.motors[0], err);
    if (!y.IsDefined() || !y.IsMap()) err.push_back("missing required section: motors.yaw");
    else load_motor(y, "motors.yaw", c.motors[1], err);
    if (p.IsDefined() && y.IsDefined() && c.motors[0].can_id == c.motors[1].can_id)
      err.push_back("motors.pitch.can_id and motors.yaw.can_id must differ");
  }

  // control (required loop_hz).
  const YAML::Node ctrl = fetch(root, "control");
  if (!ctrl.IsDefined() || !ctrl.IsMap()) {
    err.push_back("missing required section: control");
  } else {
    c.control_loop_hz = req_int(ctrl, "loop_hz", "control.loop_hz", err);
    if (c.control_loop_hz <= 0 || c.control_loop_hz > 1000)
      err.push_back("control.loop_hz must be in (0,1000]");
  }

  // axes, homing, shutdown (commissioning; always apply conservative defaults so
  // an omitted section still yields a safe config).
  load_axis(fetch(root["axes"], "pitch"), "pitch", c.axes[0], warn, err);
  load_axis(fetch(root["axes"], "yaw"), "yaw", c.axes[1], warn, err);
  load_contact(fetch(root, "homing"), c.homing, warn, err);
  load_homing_plan(root, c.homing_plan, warn, err);
  {
    const std::string p = "shutdown.";
    const YAML::Node sh = fetch(root, "shutdown");
    c.shutdown.yaw_park_deg = opt_double(sh, "yaw_park_deg", p + "yaw_park_deg",
                                         Defaults().yaw_park_deg, warn);
    c.shutdown.pitch_park_deg = opt_double(sh, "pitch_park_deg", p + "pitch_park_deg",
                                           Defaults().pitch_park_deg, warn);
    c.shutdown.pos_tolerance_deg =
        opt_double(sh, "pos_tolerance_deg", p + "pos_tolerance_deg",
                   Defaults().park_pos_tolerance_deg, warn);
    c.shutdown.vel_tolerance_deg_s =
        opt_double(sh, "vel_tolerance_deg_s", p + "vel_tolerance_deg_s",
                   Defaults().park_vel_tolerance_deg_s, warn);
    c.shutdown.dwell_ms = opt_int(sh, "dwell_ms", p + "dwell_ms",
                                  Defaults().park_dwell_ms, warn);
    c.shutdown.speed_deg_s =
        opt_double(sh, "speed_deg_s", p + "speed_deg_s", Defaults().park_speed_deg_s, warn);
    c.shutdown.verify_speed_deg_s =
        opt_double(sh, "verify_speed_deg_s", p + "verify_speed_deg_s",
                   Defaults().park_verify_speed_deg_s, warn);
    if (c.shutdown.pos_tolerance_deg <= 0.0)
      err.push_back(p + "pos_tolerance_deg must be > 0");
    if (c.shutdown.vel_tolerance_deg_s <= 0.0)
      err.push_back(p + "vel_tolerance_deg_s must be > 0");
    if (c.shutdown.dwell_ms <= 0) err.push_back(p + "dwell_ms must be > 0");
    if (c.shutdown.speed_deg_s <= 0.0) err.push_back(p + "speed_deg_s must be > 0");
    // Must be > 0: LimitSpd=0 pins the drive's position loop, so a 0-limit
    // Verify hold could never pull an axis back into the §33.2 window (p3).
    if (c.shutdown.verify_speed_deg_s <= 0.0)
      err.push_back(p + "verify_speed_deg_s must be > 0");
  }
  {
    const std::string p = "safety.";
    const YAML::Node sf = fetch(root, "safety");
    c.safety.feedback_max_age_ms =
        opt_int(sf, "feedback_max_age_ms", p + "feedback_max_age_ms",
                Defaults().feedback_max_age_ms, warn);
    c.safety.deadline_max_us =
        opt_int(sf, "deadline_max_us", p + "deadline_max_us", Defaults().deadline_max_us, warn);
    c.safety.deadline_miss_threshold =
        opt_int(sf, "deadline_miss_threshold", p + "deadline_miss_threshold",
                Defaults().deadline_miss_threshold, warn);
    c.safety.motor_overtemp_c =
        opt_double(sf, "motor_overtemp_c", p + "motor_overtemp_c",
                   Defaults().motor_overtemp_c, warn);
    if (c.safety.feedback_max_age_ms <= 0)
      err.push_back(p + "feedback_max_age_ms must be > 0");
    if (c.safety.deadline_max_us <= 0) err.push_back(p + "deadline_max_us must be > 0");
    if (c.safety.deadline_miss_threshold <= 0)
      err.push_back(p + "deadline_miss_threshold must be > 0");
    if (c.safety.motor_overtemp_c <= 0.0)
      err.push_back(p + "motor_overtemp_c must be > 0");
  }

  // tracking (sensible defaults).
  const YAML::Node trk = fetch(root, "tracking");
  {
    const YAML::Node se = fetch(trk, "search_enabled_by_default");
    if (se.IsDefined() && !unspecified(se)) {
      try {
        c.tracking.search_enabled_by_default = se.as<bool>();
      } catch (const YAML::Exception&) {
        warn.push_back("tracking.search_enabled_by_default: not a bool; using false");
      }
    }
    c.tracking.target_lost_behavior =
        opt_string(trk, "target_lost_behavior", "tracking.target_lost_behavior", "hold", warn);
    if (c.tracking.target_lost_behavior != "hold" &&
        c.tracking.target_lost_behavior != "search")
      err.push_back("tracking.target_lost_behavior must be 'hold' or 'search'");
  }

  // camera (defaults; Phase 4 fills these in).
  const YAML::Node cam = fetch(root, "camera");
  c.camera.intrinsics_file = opt_string(cam, "intrinsics_file", "camera.intrinsics_file",
                                        "calibration/camera_intrinsics.yaml", warn);
  c.camera.extrinsics_file = opt_string(cam, "extrinsics_file", "camera.extrinsics_file",
                                        "calibration/camera_extrinsics.yaml", warn);

  // installation (default; Phase 7 fills this in).
  const YAML::Node inst = fetch(root, "installation");
  c.installation.pose_file = opt_string(inst, "pose_file", "installation.pose_file",
                                        "calibration/installation_pose.yaml", warn);

  // payload (default; Phase 9 profiling, §28.5/§27).
  const YAML::Node pay = fetch(root, "payload");
  c.payload.active_profile = opt_string(pay, "active_profile", "payload.active_profile",
                                        "conservative", warn);
  c.payload.profile_dir = opt_string(pay, "profile_dir", "payload.profile_dir",
                                     "config/payload_profiles", warn);
  c.payload.auto_verify = opt_bool(pay, "auto_verify", "payload.auto_verify",
                                   false, warn);
  c.payload.check_region_half_span_deg = opt_double(
      pay, "check_region_half_span_deg", "payload.check_region_half_span_deg",
      10.0, warn);
  if (c.payload.check_region_half_span_deg < 2.25) {
    warn.push_back("payload.check_region_half_span_deg < 2.25 deg: the check "
                   "amplitude would be clamped below its 0.25 deg minimum; "
                   "keeping the value (the check will fail its guard)");
  }
  c.payload.check_current_a =
      opt_double(pay, "check_current_a", "payload.check_current_a", 5.0, warn);
  // The station current cap is 10 A (safety boundary): clamp, don't trust.
  if (c.payload.check_current_a > 10.0) {
    warn.push_back("payload.check_current_a > 10 A: clamped to the 10 A "
                   "station cap");
    c.payload.check_current_a = 10.0;
  }
  if (c.payload.check_current_a <= 0.0) {
    warn.push_back("payload.check_current_a <= 0 A: using the 5 A default");
    c.payload.check_current_a = 5.0;
  }
  c.payload.check_spd_kp =
      opt_double(pay, "check_spd_kp", "payload.check_spd_kp", 5.0, warn);
  c.payload.check_spd_ki =
      opt_double(pay, "check_spd_ki", "payload.check_spd_ki", 0.02, warn);
  if (c.payload.check_spd_kp <= 0.0) {
    warn.push_back("payload.check_spd_kp <= 0: using the 5.0 default");
    c.payload.check_spd_kp = 5.0;
  }
  if (c.payload.check_spd_ki < 0.0) {
    warn.push_back("payload.check_spd_ki < 0: using the 0.02 default");
    c.payload.check_spd_ki = 0.02;
  }

  r.ok = err.empty();
  return r;
}

}  // namespace ota::config
