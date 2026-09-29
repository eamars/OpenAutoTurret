#include "runtime_parameter_registry.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

namespace ota::control {
namespace {

std::string quote(const std::string& s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else out += c;
  }
  return out + "\"";
}

std::string num(double v) {
  if (!std::isfinite(v)) return "null";
  char buf[40];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}
// One overload, deliberately: counts and enum-valued ints are all small enough to travel through
// the double formatter without losing a digit, and a second overload made every `num(int)` call
// site ambiguous against it.
std::string num(int v) { return num(static_cast<double>(v)); }
std::string num(long long v) { return num(static_cast<double>(v)); }

// The condition every session-writable exchange shares. It is the guard the trial commands already
// enforce (control_loop.cpp), written down once instead of being re-derived by each caller: a
// parameter may only be swapped when the axis is idle, healthy, in Manual commissioning, and no
// manual lease or probe owns the output.
const char* kIdleManual =
    "phase=Hold, mode=Manual commissioning, safety=Allow, no manual lease, no response probe, "
    "both axes stationary with fresh feedback";

std::string group_of(const std::vector<ParameterEntry>& all) { return all.size() ? "" : ""; }

void emit_entry(std::ostringstream& out, const ParameterEntry& e) {
  auto field = [&](const char* key, const std::string& value) {
    out << ",\n      " << quote(key) << ": " << value;
  };
  out << "    {\n      ";
  out << quote("name") << ": " << quote(e.name);
  field("group", quote(e.group));
  field("type", quote(e.type));
  field("unit", quote(e.unit));
  field("default", e.default_literal.empty() ? "null" : e.default_literal);
  field("actual_value", e.actual_literal.empty() ? "null" : e.actual_literal);
  field("supported_range", quote(e.supported_range));
  field("allowed_test_range", quote(e.allowed_test_range));
  field("mode", quote(e.mode));
  field("mutability", quote(mutability_name(e.mutability)));
  field("apply_condition", quote(e.apply_condition));
  field("source_binding", quote(e.source_binding));
  field("readback_source", quote(readback_source_name(e.readback)));
  field("encoding", quote(e.encoding));
  field("restart_required", e.restart_required ? "true" : "false");
  field("reason", quote(e.reason));
  out << "\n    }";
}

}  // namespace

const char* mutability_name(Mutability m) {
  switch (m) {
    case Mutability::ExperimentWritable: return "experiment_writable";
    case Mutability::FixedInCampaign: return "fixed_in_campaign";
    case Mutability::ProtectedReadOnly: return "protected_read_only";
    case Mutability::Unsupported: return "unsupported";
  }
  return "unsupported";
}

const char* readback_source_name(Readback r) {
  switch (r) {
    case Readback::DriveRegister: return "drive_register";
    case Readback::HostEcho: return "host_echo";
    case Readback::ConfigFile: return "config_file";
    case Readback::None: return "none";
  }
  return "none";
}

std::vector<ExclusionRule> build_exclusion_rules() {
  // A config field that is not a control-loop knob still has to be accounted for: the coverage test
  // fails on any field that is neither bound to an entry nor covered here. "Not listed" is only
  // allowed as a decision, never as an accident.
  return {
      {"CanConfig", "bus topology and bitrate: a wiring fact, changing it means rebuilding the "
                    "transport, so it is restart-required by construction and not a tunable"},
      {"MotorConfig", "CAN ids and direction sign: identity, not behaviour"},
      {"AxisLimitsConfig", "soft travel limits and the expected-travel envelope: a safety envelope "
                           "approved by measurement, widening it is an operator decision and not an "
                           "optimizer dimension (including its TravelDeg endpoint pair)"},
      {"Protocol", "CAN interface, SPI parent and bitrate on each bus (CanBus holds one): wiring "
                   "identity. A different bus is a different transport, which is a restart by "
                   "construction"},
      {"Axis", "motor id, topology, control mode, the current-ring precondition and the guard "
               "temperature ceiling: which device this is and what it may be asked to do. Not a "
               "tunable; the friction struct hanging off it is listed field by field as entries"},
      {"Profile", "schema_version of the topology file: boot identity, and the file the entries "
                  "below are bound into"},
      {"ContactConfig", "homing contact detection: qualified against this mechanism on 2026-09; "
                        "changing it re-qualifies homing, which is a different campaign"},
      {"HomingPlanConfig", "the zeroing plan is a sequence, not a scalar knob; ordering changes are "
                           "algorithm changes and end a campaign"},
      {"HomingPlanActionConfig", "one step of that sequence"},
      {"TrackingConfig", "search/hold behaviour and reference shaping: quality-affecting but "
                         "outside ADR-002.1's declared scope (velocity loop and friction "
                         "compensation), and the aim geometry depends on calibration files"},
      {"VisionConfig", "a socket path"},
      {"CameraConfig", "calibration file paths: inputs to geometry, not gains"},
      {"InstallationConfig", "mounting pose file"},
      {"ShutdownConfig", "the park policy is the stop-path safety argument; every field here is "
                         "protected_read_only for the campaign (see park_power_probe) and belongs "
                         "to a payload re-qualification, not to loop tuning"},
      {"PayloadConfig", "profile selection and its verification checks: promoted through the "
                        "existing atomic profile save, not through the hot registry"},
      {"MotionConfig", "per-mode speed/accel/jerk tables are structured arrays; the block is "
                       "restart-required and its intersection order is an algorithm, so a "
                       "campaign may not reorder it. cfg.motion.configured is listed as an entry"},
      {"TurretConfig", "schema_version and hardware_profile are boot identity. control_loop_hz is "
                       "listed as an entry because it is the one field here a reader may confuse "
                       "with a tunable"},
      {"campaign", "durations, repeats, thresholds and sample validity live in campaign.lock.json "
                   "by ADR-002.1 docs/02; they are deliberately not firmware fields so that a "
                   "re-run can change them without touching the binary"},
  };
}

std::vector<ParameterEntry> build_parameter_registry(const config::TurretConfig& cfg,
                                                     const config::mixed::Profile* profile) {
  std::vector<ParameterEntry> e;
  const bool have_profile = profile != nullptr;
  const double cap = have_profile ? profile->yaw.host_current_limit_a : 0.0;
  const std::string cap_text = have_profile ? num(cap) : std::string("null");
  const gm6020::FrictionConfig f = have_profile ? profile->yaw.friction : gm6020::FrictionConfig{};

  auto add = [&e](ParameterEntry row) { e.push_back(std::move(row)); };
  ParameterEntry base;
  base.mode = kIdleManual;
  base.apply_condition = kIdleManual;

  // --- yaw velocity loop, current mode (the group ADR-002 docs/02 puts first) ------------------
  {
    ParameterEntry r = base;
    r.group = "yaw_current_loop";
    r.type = "double";
    r.unit = "A/(rad/s)";
    r.supported_range = "0 < kp <= 10, enforced by MixedCanMotorBackend::apply_yaw_trial";
    r.allowed_test_range = "frozen by campaign.lock.json; the registry states the bound, not the domain";
    r.mutability = Mutability::ExperimentWritable;
    r.source_binding = "Profile::Yaw::current_kp_a_per_rad_s";
    r.readback = Readback::HostEcho;
    r.encoding = "host variable in the mixed backend's yaw velocity loop; the GM6020 has no gain register";
    r.name = "yaw.current_kp_a_per_rad_s";
    r.default_literal = have_profile ? num(profile->yaw.current_kp_a_per_rad_s) : "null";
    r.actual_literal = r.default_literal;
    r.reason = have_profile
        ? "unqualified session value: applied, echoed by the host, never persisted"
        : "no mixed hardware profile loaded in this boot, so there is no value to report";
    add(r);
  }
  {
    ParameterEntry r = e.back();
    r.name = "yaw.current_ki_a_per_rad_s";
    r.unit = "A/rad";
    r.supported_range = "0 <= ki <= 20, enforced by MixedCanMotorBackend::apply_yaw_trial";
    r.source_binding = "Profile::Yaw::current_ki_a_per_rad_s";
    r.reason = "legacy key spelling; the physical unit is A/rad";
    r.default_literal = have_profile ? num(profile->yaw.current_ki_a_per_rad_s) : "null";
    r.actual_literal = r.default_literal;
    add(r);
  }
  {
    ParameterEntry r = base;
    r.name = "yaw.host_current_limit_a";
    r.group = "limits_and_state";
    r.type = "double";
    r.unit = "A";
    r.default_literal = cap_text;
    r.actual_literal = cap_text;
    r.supported_range = "> 0 required by current mode; the profile file is the approval record";
    r.allowed_test_range = "";
    r.mutability = Mutability::ProtectedReadOnly;
    r.source_binding = "Profile::Yaw::host_current_limit_a";
    r.readback = Readback::ConfigFile;
    r.encoding = "host-side command clamp";
    r.reason = "the approved envelope for this mechanism: the search may use it, never widen it "
               "(ADR-002.1 D7)";
    add(r);
  }

  // --- yaw friction compensation (four amplitudes are searchable; the rest is not measured) ----
  auto friction_row = [&](const char* name, const char* unit, double value, const char* binding,
                          Mutability m, const char* why) {
    ParameterEntry r = base;
    r.name = name;
    r.group = "yaw_friction_compensation";
    r.type = "double";
    r.unit = unit;
    r.default_literal = have_profile ? num(value) : "null";
    r.actual_literal = r.default_literal;
    r.supported_range = have_profile
        ? std::string("0 <= value <= host_current_limit_a=") + cap_text +
              ", enforced by gm6020::FrictionConfig::valid"
              : "unavailable without the profile";
    r.allowed_test_range = (m == Mutability::ExperimentWritable)
        ? "frozen by campaign.lock.json" : "";
    r.mutability = m;
    r.source_binding = binding;
    r.readback = Readback::HostEcho;
    r.encoding = "additive feedforward bias in amperes, applied by YawFrictionCompensation";
    r.reason = why;
    add(r);
  };
  friction_row("yaw.friction.positive_breakaway_a", "A", f.positive_breakaway_a,
               "Profile::Yaw::friction.positive_breakaway_a", Mutability::ExperimentWritable,
               "a candidate amplitude, not a measured friction force");
  friction_row("yaw.friction.negative_breakaway_a", "A", f.negative_breakaway_a,
               "Profile::Yaw::friction.negative_breakaway_a", Mutability::ExperimentWritable,
               "direction-specific on purpose: the mechanism is not symmetric");
  friction_row("yaw.friction.positive_run_a", "A", f.positive_run_a,
               "Profile::Yaw::friction.positive_run_a", Mutability::ExperimentWritable,
               "running friction, applied once movement is confirmed");
  friction_row("yaw.friction.negative_run_a", "A", f.negative_run_a,
               "Profile::Yaw::friction.negative_run_a", Mutability::ExperimentWritable,
               "running friction, negative direction");
  friction_row("yaw.friction.output_slew_a_per_s", "A/s", f.output_slew_a_per_s,
               "Profile::Yaw::friction.output_slew_a_per_s", Mutability::ExperimentWritable,
               "0 < slew <= 10, checked by the trial command before the backend sees it");
  friction_row("yaw.friction.timeout_s", "s", f.timeout_s,
               "Profile::Yaw::friction.timeout_s", Mutability::FixedInCampaign,
               "0 < t <= 2.0 s by FrictionConfig::valid; the value the trial command passes today "
               "(1.0 s) is a literal at the call site, not a measured breakaway window, so it is "
               "not a search dimension until one is measured");
  friction_row("yaw.friction.motion_displacement_rad", "rad", f.motion_displacement_rad,
               "Profile::Yaw::friction.motion_displacement_rad", Mutability::FixedInCampaign,
               "the 'it moved' threshold. The trial command computes it as 3 encoder counts "
               "(3*2pi/8192 = 0.132 deg), which the ADR-002 run report itself identifies as too "
               "easy to satisfy: fixed at that value this round rather than quietly searched");
  friction_row("yaw.friction.stationary_velocity_rad_s", "rad/s", f.stationary_velocity_rad_s,
               "Profile::Yaw::friction.stationary_velocity_rad_s", Mutability::FixedInCampaign,
               "0.5 deg/s at the trial call site: defines 'at rest' for the state machine, so "
               "changing it changes what every other metric means");
  friction_row("yaw.friction.fresh_samples", "samples", double(f.fresh_samples),
               "Profile::Yaw::friction.fresh_samples", Mutability::FixedInCampaign,
               "5 at the trial call site: RX snapshots confirming motion before a start attempt is "
               "spent; a search dimension only together with the RX window it is compared against");
  {
    ParameterEntry r = base;
    r.name = "yaw.friction.enabled";
    r.group = "yaw_friction_compensation";
    r.type = "bool";
    r.unit = "dimensionless";
    r.default_literal = have_profile ? (f.enabled ? "true" : "false") : "null";
    r.actual_literal = r.default_literal;
    r.supported_range = "true | false";
    r.mutability = Mutability::FixedInCampaign;
    r.source_binding = "Profile::Yaw::friction.enabled";
    r.readback = Readback::HostEcho;
    r.encoding = "selects whether YawFrictionCompensation contributes any bias";
    r.reason = "not searchable this round: the trial command derives this flag from 'any amplitude "
               "> 0', so switching it also switches the four amplitudes. ADR-002.1 docs/02 section "
               "4 requires one control path where the only difference is the value; until the flag "
               "is an independent request field, an on/off comparison is not a single-variable test";
    add(r);
  }

  // --- speed estimation (exposed, deliberately not searched: it is the measuring instrument) ---
  {
    ParameterEntry r = base;
    r.name = "yaw.velocity_rx_window_ms";
    r.group = "velocity_estimation";
    r.type = "enum";
    r.unit = "ms";
    r.default_literal = have_profile ? num(profile->yaw.velocity_rx_window_ms) : "null";
    r.actual_literal = r.default_literal;
    r.supported_range = "one of 0 (legacy ~50 ms filter), 20, 30, 40";
    r.mutability = Mutability::FixedInCampaign;
    r.source_binding = "Profile::Yaw::velocity_rx_window_ms";
    r.readback = Readback::HostEcho;
    r.encoding = "selects the fresh-RX history the velocity estimate uses";
    r.reason = "ADR-002.1 D5: the offline velocity estimate used to score jitter is fixed. "
               "Searching the window would let a candidate win by filtering, so the window is set "
               "once per campaign, not per candidate";
    add(r);
  }

  // --- pitch speed loop (the only gains in the system with a real register readback) -----------
  auto pitch_row = [&](const char* name, const char* unit, double value, const char* range,
                       const char* binding) {
    ParameterEntry r = base;
    r.name = name;
    r.group = "pitch_speed_loop";
    r.type = "double";
    r.unit = unit;
    r.default_literal = num(value);
    r.actual_literal = num(value);
    r.supported_range = range;
    r.allowed_test_range = "frozen by campaign.lock.json";
    r.mutability = Mutability::ExperimentWritable;
    r.source_binding = binding;
    r.readback = Readback::DriveRegister;
    r.encoding = "CyberGear SpdKp/SpdKi registers, written through "
                 "begin_pitch_speed_loop_gain_update and confirmed by register readback";
    r.reason = "the write is asynchronous: the command acknowledges 'queued' and only the readback "
               "turns it into Complete; RUN must wait for that, not for the ack";
    add(r);
  };
  pitch_row("pitch.service_speed_kp", "drive units (CyberGear SpdKp)", cfg.v3.service_speed_kp,
            "1 <= kp <= 5, checked by the pitch_control_trial parser", "TurretConfig::V3::service_speed_kp");
  pitch_row("pitch.service_speed_ki", "drive units (CyberGear SpdKi)", cfg.v3.service_speed_ki,
            "0.002 <= ki <= 0.05, checked by the pitch_control_trial parser",
            "TurretConfig::V3::service_speed_ki");

  // --- outer loop and reference shaping: visible, fixed this round ----------------------------
  {
    ParameterEntry r = base;
    r.name = "outer.position_servo_kp";
    r.group = "position_outer_loop";
    r.type = "double";
    r.unit = "(deg/s)/deg";
    r.default_literal = num(cfg.v3.position_servo_kp);
    r.actual_literal = num(cfg.v3.position_servo_kp);
    r.supported_range = "2 <= gain <= 6 while a response probe overrides it per step";
    r.mutability = Mutability::FixedInCampaign;
    r.source_binding = "TurretConfig::V3::position_servo_kp";
    r.readback = Readback::HostEcho;
    r.encoding = "host position P producing the velocity reference";
    r.reason = "exposed and hot-appliable, fixed this round: the trial probe can already override "
               "it per step, so searching it alongside the inner loop would confound two rings";
    add(r);
  }
  {
    ParameterEntry r = base;
    r.name = "motion.configured";
    r.group = "output_limits";
    r.type = "bool";
    r.unit = "dimensionless";
    r.default_literal = cfg.motion.configured ? "true" : "false";
    r.actual_literal = r.default_literal;
    r.supported_range = "true | false";
    r.mutability = Mutability::ProtectedReadOnly;
    r.source_binding = "TurretConfig::motion.configured";
    r.readback = Readback::ConfigFile;
    r.encoding = "presence of the motion block";
    r.restart_required = true;
    r.reason = "an absent motion block keeps the legacy shaping path, which is a different "
               "algorithm; the per-mode speed/accel/jerk arrays are excluded as structured data";
    add(r);
  }

  // --- what the optimizer may never widen ------------------------------------------------------
  auto protected_row = [&](const char* name, const char* group, const char* type, const char* unit,
                           const std::string& value, const char* binding, const char* why) {
    ParameterEntry r = base;
    r.name = name;
    r.group = group;
    r.type = type;
    r.unit = unit;
    r.default_literal = value;
    r.actual_literal = value;
    r.supported_range = "not a search dimension";
    r.allowed_test_range = "";
    r.mutability = Mutability::ProtectedReadOnly;
    r.source_binding = binding;
    r.readback = Readback::ConfigFile;
    r.encoding = "read at boot";
    r.restart_required = true;
    r.reason = why;
    add(r);
  };
  protected_row("control.control_loop_hz", "limits_and_state", "int", "Hz", num(cfg.control_loop_hz),
                "TurretConfig::control_loop_hz",
                "ADR-002.1 D2: the control period is firmware structure, not a candidate dimension");
  protected_row("safety.feedback_max_age_ms", "limits_and_state", "int", "ms",
                num(cfg.safety.feedback_max_age_ms), "TurretConfig::Safety::feedback_max_age_ms",
                "stale feedback must stop closed-loop output; widening it during a campaign would "
                "silently raise the risk the campaign is measured under");
  protected_row("safety.deadline_max_us", "limits_and_state", "int", "us",
                num(cfg.safety.deadline_max_us), "TurretConfig::Safety::deadline_max_us",
                "schedule overrun budget; a derate is an observation, not a failure to tune away");
  protected_row("safety.deadline_miss_threshold", "limits_and_state", "int", "misses",
                num(cfg.safety.deadline_miss_threshold), "TurretConfig::Safety::deadline_miss_threshold",
                "how many misses before derating; part of the protection chain");
  protected_row("safety.motor_overtemp_c", "limits_and_state", "double", "degC",
                num(cfg.safety.motor_overtemp_c), "TurretConfig::Safety::motor_overtemp_c",
                "device protection. Yaw's raw temperature byte is not calibrated to degC, so the "
                "campaign records the byte and does not claim a thermal qualification");
  protected_row("session.jog_lease_ms", "limits_and_state", "int", "ms", num(cfg.v3.jog_lease_ms),
                "TurretConfig::V3::jog_lease_ms",
                "the lease is the fail-safe for a dead runner; a campaign may not run without it");
  protected_row("session.jog_keepalive_ms", "limits_and_state", "int", "ms",
                num(cfg.v3.jog_keepalive_ms), "TurretConfig::V3::jog_keepalive_ms",
                "renewal period of that lease");
  protected_row("homing.speed_kp", "homing", "double", "A/(rad/s)", num(cfg.homing.speed_kp),
                "TurretConfig::Homing::speed_kp",
                "homing is qualified against the current mechanism; retuning it is a separate "
                "campaign with its own repeatability evidence");
  protected_row("homing.speed_ki", "homing", "double", "A/rad", num(cfg.homing.speed_ki),
                "TurretConfig::Homing::speed_ki", "same qualification as homing.speed_kp");
  protected_row("payload.active_profile", "payload", "string", "name",
                quote(cfg.payload.active_profile), "TurretConfig::Payload::active_profile",
                "a payload change re-qualifies both axes; promoted through the existing atomic "
                "profile save, never written by the optimizer");

  protected_row("yaw.guard_temp_raw_ceiling", "limits_and_state", "int", "raw byte (not degC)",
                have_profile ? num(profile->yaw.yaw_guard_temp_raw_ceiling) : "null",
                "Profile::Axis::yaw_guard_temp_raw_ceiling",
                "the GM6020 reports temperature as an uncalibrated raw byte, so this ceiling is a "
                "comparison against that byte and not a temperature: the campaign records the byte "
                "and does not claim a thermal qualification from it (0 = guard off)");

  // --- declared, because pretending it exists would be the other way of lying -----------------
  {
    ParameterEntry r = base;
    r.name = "pitch.internal_current_pi";
    r.group = "drive_unsupported";
    r.type = "string";
    r.unit = "drive units";
    r.default_literal = "null";
    r.actual_literal = "null";
    r.supported_range = "not writable at runtime";
    r.mutability = Mutability::Unsupported;
    r.source_binding = "CyberGear internal velocity/current loop";
    r.readback = Readback::None;
    r.encoding = "no documented writable register in the protocol this station speaks";
    r.reason = "ADR-002 keeps the drive's own current loop; it is neither readable nor writable "
               "here, so it may not appear as a YAML key that looks applied";
    add(r);
  }
  {
    ParameterEntry r = base;
    r.name = "yaw.current_limit_register";
    r.group = "drive_unsupported";
    r.type = "string";
    r.unit = "A";
    r.default_literal = "null";
    r.actual_literal = "null";
    r.supported_range = "not writable at runtime";
    r.mutability = Mutability::Unsupported;
    r.source_binding = "MotorBackend::set_current_limit (yaw branch is a no-op by design)";
    r.readback = Readback::None;
    r.encoding = "the GM6020 protocol has no current-limit register; the host clamp is the envelope";
    r.reason = "declared unsupported so that a writer cannot report success for a write that "
               "goes nowhere";
    add(r);
  }
  (void)group_of(e);
  return e;
}

std::string parameter_registry_json(const config::TurretConfig& cfg,
                                    const config::mixed::Profile* profile,
                                    const std::string& config_path,
                                    const std::string& profile_path) {
  const auto entries = build_parameter_registry(cfg, profile);
  const auto rules = build_exclusion_rules();
  std::ostringstream out;
  out << "{\n  " << quote("schema") << ": 1,\n";
  out << "  " << quote("generated_from") << ": {\n";
  out << "    " << quote("config") << ": " << quote(config_path);
  out << ",\n    " << quote("hardware_profile") << ": "
      << (profile_path.empty() ? "null" : quote(profile_path));
  out << "\n  },\n";
  out << "  " << quote("entries") << ": [\n";
  for (size_t i = 0; i < entries.size(); ++i) {
    emit_entry(out, entries[i]);
    out << (i + 1 == entries.size() ? "\n" : ",\n");
  }
  out << "  ],\n";
  out << "  " << quote("exclusion_rules") << ": [\n";
  for (size_t i = 0; i < rules.size(); ++i) {
    out << "    {" << quote("config_struct") << ": " << quote(rules[i].config_struct) << ", "
        << quote("reason") << ": " << quote(rules[i].reason) << "}";
    out << (i + 1 == rules.size() ? "\n" : ",\n");
  }
  out << "  ]\n}\n";
  return out.str();
}

}  // namespace ota::control
