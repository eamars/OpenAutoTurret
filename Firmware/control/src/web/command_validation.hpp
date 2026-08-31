#pragma once
// OpenAutoTurret — developer-command validation (architecture §42.2, §42.1).
//
// "Every request goes through controld state validation." The web UI (webd)
// only ever SUBMITS high-level commands; controld decides, from its own
// authoritative state, whether each command is allowed. webd never opens can0
// and never decides safety — it relays the operator's intent.
//
// Pure data + logic: NO CAN, NO camera, NO motor driver, NO network. The
// network transport (WebServer) and the actual execution (ControlLoop) are
// separate; this module is the validation gate in between and is unit-testable
// in isolation.
#include <cmath>
#include <cstdint>
#include <string>

namespace ota {
namespace web {

// The authoritative system state controld exposes for command validation.
// (A projection of the ControlLoop / SystemState; kept minimal on purpose.)
struct SystemCommandState {
  bool homed = false;             // position validity known (§38.1)
  bool fault = false;             // fault-locked: no motion commands
  bool tracking_enabled = false;  // tracking mode is on
  bool tracking_active = false;   // a tracking reference is being produced
  bool search_enabled = false;
  bool moving = false;            // currently executing a motion phase
  // Soft-limit envelope (rad) for restricted test motion, valid after homing.
  bool limits_valid = false;
  double q_min_rad[2] = {-1e9, -1e9};  // [pitch, yaw]
  double q_max_rad[2] = {1e9, 1e9};    // [pitch, yaw]
};

struct CommandResult {
  bool ok = false;
  std::string error;  // empty when ok
};

// Axis index helper (matches AxisId order: pitch=0, yaw=1).
inline constexpr int kPitchIx = 0;
inline constexpr int kYawIx = 1;

// Validate a high-level developer command (§42.2) against the system state.
// `command` is the bare command name; `arg` is a single optional argument
// (empty for no-arg commands). Returns ok or the rejection reason.
//
// Allowed commands:
//   hold, start_tracking, stop_tracking, enable_search, disable_search,
//   select_target <id>, start_homing, start_installation_calibration,
//   start_payload_verification, request_park, request_shutdown,
//   run_test_motion <pos_rad>  (restricted test motion)
inline CommandResult validate_command(const SystemCommandState& s,
                                      const std::string& command,
                                      const std::string& arg = "") {
  CommandResult r;
  if (command == "hold") {
    // Always allowed: hold is the safe default.
    r.ok = true;
    return r;
  }
  if (command == "request_park" || command == "request_shutdown") {
    // Safe operations: always allowed (parking is the recovery path).
    r.ok = true;
    return r;
  }
  // Motion/calibration commands are locked out while faulted.
  if (s.fault) {
    r.error = "system faulted; motion/calibration commands are locked out";
    return r;
  }
  // Homing is the command that ESTABLISHES position validity, so it is the one
  // command allowed while NOT homed (it requires not-yet-homed).
  if (command == "start_homing") {
    if (s.homed) {
      r.error = "already homed; re-homing requires a manual reset";
      return r;
    }
    r.ok = true;
    return r;
  }
  // Everything below requires a homed system.
  if (!s.homed) {
    r.error = "not homed (position validity unknown)";
    return r;
  }
  if (command == "start_tracking") {
    if (s.tracking_active) {
      r.error = "tracking already active";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "stop_tracking") {
    if (!s.tracking_enabled) {
      r.error = "tracking is not enabled";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "enable_search") {
    if (s.search_enabled) {
      r.error = "search already enabled";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "disable_search") {
    if (!s.search_enabled) {
      r.error = "search is not enabled";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "select_target") {
    if (!s.tracking_enabled) {
      r.error = "tracking is not enabled";
      return r;
    }
    int id = 0;
    try {
      id = std::stoi(arg);
    } catch (...) {
      r.error = "invalid target id";
      return r;
    }
    if (id < 0 || id > 15) {
      r.error = "target id out of range (0..15)";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "start_installation_calibration") {
    if (s.tracking_active || s.moving) {
      r.error = "stop tracking / motion before calibration";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "start_payload_verification") {
    if (s.moving) {
      r.error = "system is moving; wait for hold";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "run_test_motion") {
    if (!s.limits_valid) {
      r.error = "soft limits not established (homing incomplete)";
      return r;
    }
    double pos = 0.0;
    try {
      pos = std::stod(arg);
    } catch (...) {
      r.error = "invalid position argument (radians)";
      return r;
    }
    if (!std::isfinite(pos)) {
      r.error = "position not finite";
      return r;
    }
    // Restricted: must sit within the soft-limit envelope (both axes share the
    // single requested position in this simplified command; checked against the
    // yaw envelope which is the wide axis in practice).
    if (pos < s.q_min_rad[kYawIx] || pos > s.q_max_rad[kYawIx]) {
      r.error = "position outside soft-limit envelope";
      return r;
    }
    r.ok = true;
    return r;
  }
  r.error = "unknown command '" + command + "'";
  return r;
}

}  // namespace web
}  // namespace ota
