// OpenAutoTurret — safe park / shutdown controller (architecture §33).
//
// An explicit state machine (NOT a generic disable()) that moves the axes to a
// safe park pose, verifies they are settled there, dwells, and then
// de-energizes the motors one at a time:
//
//   StopTracking -> MoveYaw -> MovePitch -> Verify -> Dwell
//                -> DisablePitch -> DisableYaw -> Parked
//
// The park positions are expressed in the LOGICAL frame and must lie strictly
// inside the calibrated soft limits with a margin (§33.1) — never directly
// against a mechanical stop. Before the motors are de-energized, BOTH axes must
// be within position + velocity tolerance of their park poses, held for
// park_dwell_ms (§33.2).
//
// Software can guarantee this only for a COMMANDED shutdown while motor power is
// available; it cannot move the axes to park after an abrupt power loss (§33.3)
// — that is a hardware concern (capacitor hold-up, end-stop geometry, friction).
//
// Transport-agnostic: step() takes HomingFeedback for both axes and returns a
// ParkOutput (a DesiredState per axis plus de-energize requests). It is
// unit-testable against a simulated plant with no CAN (§54).
#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "calibration/move_to.hpp"
#include "common/logical_coordinates.hpp"
#include "common/types.hpp"
#include "control/safety_envelope.hpp"

namespace ota {

enum class ParkState {
  StopTracking,
  MoveYaw,
  MovePitch,
  Verify,
  Dwell,
  DisablePitch,
  DisableYaw,
  Parked,
  Failed,
};

inline const char* park_state_name(ParkState s) {
  switch (s) {
    case ParkState::StopTracking: return "stop_tracking";
    case ParkState::MoveYaw:      return "move_yaw";
    case ParkState::MovePitch:    return "move_pitch";
    case ParkState::Verify:       return "verify";
    case ParkState::Dwell:        return "dwell";
    case ParkState::DisablePitch: return "disable_pitch";
    case ParkState::DisableYaw:   return "disable_yaw";
    case ParkState::Parked:       return "parked";
    case ParkState::Failed:       return "failed";
  }
  return "?";
}

struct ParkParams {
  // Park targets in the LOGICAL frame (deg), one per axis.
  std::array<double, kAxisCount> park_logical_deg{};
  // §33.2 verification required before de-energizing.
  double pos_tol_deg = 0.5;
  double vel_tol_deg_s = 1.0;
  int dwell_ms = 500;
  // Speed limit for the park moves.
  double speed_deg_s = 10.0;
  // §33.1: the park pose must sit inside the soft limit by at least this margin.
  double min_soft_margin_deg = 2.0;
  // Move arrival tolerances / timeout (delegated to MoveTo).
  double move_pos_tol_rad = 0.01;
  double move_vel_tol_rad_s = 0.1 * kDeg2Rad;
  double move_timeout_s = 30.0;
};

// What the executor should do this cycle.
struct ParkOutput {
  DesiredState pitch;  // desired state for pitch (hold if not the active move axis)
  DesiredState yaw;    // desired state for yaw
  bool disable_pitch = false;  // request to de-energize pitch now
  bool disable_yaw = false;    // request to de-energize yaw now
  bool complete = false;       // reached Parked (power-safe)
  bool failed = false;         // reached Failed
  std::string message;
};

class ParkController {
 public:
  // `limits` must be valid (post-homing) and `models` must carry a reference for
  // every axis; otherwise the controller starts in the Failed state. The park
  // pose must be strictly inside the soft limit with a margin (§33.1).
  ParkController(ParkParams p, const std::array<AxisLimits, kAxisCount>& limits,
                 const std::array<AxisLogicalModel, kAxisCount>& models);

  ParkOutput step(const HomingFeedback& pitch_fb, const HomingFeedback& yaw_fb);

  ParkState state() const { return state_; }
  bool complete() const { return state_ == ParkState::Parked; }
  bool failed() const { return state_ == ParkState::Failed; }
  const std::string& fail_reason() const { return fail_reason_; }
  // The raw park target for an axis (valid once construction succeeded).
  double park_raw_rad(AxisId a) const { return park_raw_[ix(a)]; }

 private:
  static size_t ix(AxisId a) { return static_cast<size_t>(a); }
  void fail(const std::string& reason) {
    state_ = ParkState::Failed;
    fail_reason_ = reason;
  }
  bool at_park(const HomingFeedback& fb, AxisId a) const {
    const double pos_tol_rad = p_.pos_tol_deg * kDeg2Rad;
    const double vel_tol_rad_s = p_.vel_tol_deg_s * kDeg2Rad;
    return std::fabs(fb.pos_rad - park_raw_[ix(a)]) < pos_tol_rad &&
           std::fabs(fb.vel_rad_s) < vel_tol_rad_s;
  }
  static DesiredState hold(const HomingFeedback& fb) {
    return DesiredState{fb.pos_rad, 0.0, true, "hold"};
  }

  ParkParams p_;
  std::array<double, kAxisCount> park_raw_{};
  ParkState state_ = ParkState::StopTracking;
  std::string fail_reason_;
  std::optional<MoveTo> yaw_move_;
  std::optional<MoveTo> pitch_move_;
  TimeNs dwell_ns_ = 0;
  TimeNs dwell_start_ns_ = 0;
};

}  // namespace ota
