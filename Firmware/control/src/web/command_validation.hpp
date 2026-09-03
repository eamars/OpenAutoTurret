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
#include "mode/mode_manager.hpp"

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
  // Holding the safe ready pose (homing sequence finished). Homing passes
  // through Hold between stages, so `moving` alone is not enough for checks
  // that start from wherever the station happens to be.
  bool at_ready = false;
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
//   hold, set_mode <MANUAL|AUTO_TRACK|AUTO_ROAM>, stop_motion,
//   start_tracking, stop_tracking, enable_search, disable_search,
//   manual_jog_start <dir[:profile]> / manual_jog_keepalive /
//   manual_jog_stop / manual_step <axis><sign><deg>,
//   select_target <label> / clear_target, start_homing,
//   start_installation_calibration,
//   start_payload_verification, select_payload_profile <name>,
//   request_park, request_shutdown,
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
  if (command == "stop_motion") {
    // §27. No gate at all, on either layer. STOP MOTION is the button an operator
    // reaches for when something is already wrong; a state check standing between
    // them and a stop is worse than no button. It changes no safety state,
    // disables nothing, and parks nothing — it cancels the active intent and
    // lands in MANUAL/HOLD, which is the only thing that reliably stays stopped.
    r.ok = true;
    return r;
  }
  if (command == "set_mode") {
    // §51. Shape-checked here, state-checked by the control thread — deliberately.
    // Whether the station may change mode depends on homing, feedback freshness,
    // the safety ladder and the roam envelope, all of which controld owns and the
    // web thread can only guess at. Two layers gating the same state is how you
    // end up with a UI that rejects a request the loop would honour (that is
    // exactly what v1's enable_search/disable_search pair grew into). controld
    // answers with the real reason, §52.
    ota::OperatingMode target;
    if (!ota::operating_mode_from_name(arg.c_str(), target)) {
      r.error = "set_mode needs MANUAL, AUTO_TRACK or AUTO_ROAM";
      return r;
    }
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
  if (command == "manual_jog_start" || command == "manual_jog_keepalive" ||
      command == "manual_jog_stop" || command == "manual_step") {
    // Shape only. Which mode is allowed, whether a lease exists to renew, and whether a
    // step size is one of the sanctioned choices are controld's decisions (§38-§41,
    // §52), made on the thread that owns the lease — a web-thread guess about a lease
    // that expires in 300 ms would be wrong about as often as it was right.
    if (command == "manual_jog_start" && arg.empty()) {
      r.error = "jog needs a direction (yaw+, yaw-, pitch+, pitch-)";
      return r;
    }
    if (command == "manual_step" && arg.empty()) {
      r.error = "step needs an axis and degrees (yaw+1)";
      return r;
    }
    if ((command == "manual_jog_keepalive" || command == "manual_jog_stop") &&
        !arg.empty()) {
      r.error = "this command takes no argument";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "select_target" || command == "clear_target") {
    // §12: selection is independent of operating mode. This gate therefore checks
    // shape and nothing else — no tracking_enabled, no mode, no safety state. An
    // operator must be able to point at somebody while standing in MANUAL and have the
    // choice waiting when AUTO_TRACK starts; refusing because of the mode is precisely
    // the coupling §12 forbids. Whether the target exists, is CONFIRMED and is an
    // allowed class is controld's decision (§14), made against the TrackSet actually in
    // hand rather than against a web-thread snapshot.
    if (command == "clear_target") {
      if (!arg.empty()) {
        r.error = "clear_target takes no argument";
        return r;
      }
      r.ok = true;
      return r;
    }
    if (arg.empty()) {
      r.error = "select_target needs the label number shown on screen";
      return r;
    }
    for (char c : arg) {
      if (c < '0' || c > '9') {
        r.error = "target label must be a number";
        return r;
      }
    }
    if (arg.size() > 5) {
      r.error = "target label out of range";
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
    if (!s.at_ready) {
      // The check steps from wherever the station is holding. Requested during
      // the post-homing ready move, that "wherever" is a travel stop and the
      // safe central region comes out empty — reject here so the operator gets
      // a reason instead of an async abort in the log.
      r.error = "not at the ready pose yet (homing or repositioning in progress)";
      return r;
    }
    r.ok = true;
    return r;
  }
  if (command == "select_payload_profile") {
    // Runtime profile switch (§42.2). Never while the station is moving — the
    // profile is a motion limit, and it must not change under a running
    // payload check (§44) or a tracking move.
    if (arg.empty()) {
      r.error = "select_payload_profile requires a profile name";
      return r;
    }
    if (arg.size() > 64 || arg.find_first_of("/\\\n\r") != std::string::npos) {
      r.error = "invalid profile name";
      return r;
    }
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
