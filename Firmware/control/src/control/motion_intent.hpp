// OpenAutoTurret v3 — MotionIntent (§25).
//
// The one structure all three operating modes emit. v1 had each source hand the
// ReferenceManager its own bespoke fields (predicted LOS here, search joints
// there, a developer request somewhere else), which meant arbitration was a
// per-source if-chain and adding a mode meant editing that chain. v3 inverts it:
// ManualController, AutoTrackController and RoamPlanner each produce a
// MotionIntent, and the ReferenceManager converts *one* intent into a safe joint
// reference. The v1 layers downstream — SafetyEnvelope, TrajectoryGenerator, the
// CyberGear adapter — are untouched and still authoritative (§111.17/§111.18).
//
// This header is pure data: no CAN, no camera, no allocation. Two rules follow
// from that and from §46 (nothing blocking or heap-allocating runs on the control
// thread), and both are enforced here rather than left to convention:
//
//   * `reason` is a fixed buffer, not a std::string. It is written once per cycle
//     on the control thread and read by telemetry at 10-20 Hz; a heap allocation
//     in it would be a 200 Hz malloc.
//   * an intent carries `valid_until_ns`. Stale mode intents surviving a
//     transition is one of the concrete failure modes §93 tests for, so
//     expiry is decided by the structure that carries the timestamp rather than
//     by every consumer remembering to compare clocks.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ota {

// Which controller produced the intent. Exactly one of the three mode sources
// may be authoritative at a time (§26); Safety/Supervisory outrank all of them
// and are produced by the loop's own phases, not by a mode controller.
enum class MotionSource : uint8_t {
  None,
  Manual,
  AutoTrack,
  AutoRoam,
  Supervisory,  // homing / calibration / park
  Safety,       // supervisor override: brake or hold
};

inline const char* motion_source_name(MotionSource s) {
  switch (s) {
    case MotionSource::None:        return "none";
    case MotionSource::Manual:      return "manual";
    case MotionSource::AutoTrack:   return "auto_track";
    case MotionSource::AutoRoam:    return "auto_roam";
    case MotionSource::Supervisory: return "supervisory";
    case MotionSource::Safety:      return "safety";
  }
  return "?";
}

// What the intent is asking for. Notation follows §25.
enum class IntentType : uint8_t {
  Hold,              // stop where we are / hold this pose
  JointPosition,     // move to (q_yaw, q_pitch) through the trajectory generator
  JointVelocity,     // bounded velocity command (manual jog)
  LosDirection,      // point the optical axis at a base-frame LOS
  WorldLevelYaw,     // yaw sweep at a world-level elevation (IMU present, §33)
};

inline const char* intent_type_name(IntentType t) {
  switch (t) {
    case IntentType::Hold:          return "hold";
    case IntentType::JointPosition: return "joint_position";
    case IntentType::JointVelocity: return "joint_velocity";
    case IntentType::LosDirection:  return "los_direction";
    case IntentType::WorldLevelYaw: return "world_level_yaw";
  }
  return "?";
}

// A request for motion, with its authority and its shelf life.
//
// Optionality is explicit per field rather than encoded in magic values: a
// zero-valued LOS azimuth is a real direction (straight ahead), and reading it
// as "unset" is the kind of bug that points the turret somewhere nobody asked
// for. The `has_*` flags are what the converter must consult.
struct MotionIntent {
  static constexpr int kReasonLen = 20;

  MotionSource source = MotionSource::None;
  IntentType type = IntentType::Hold;

  int64_t timestamp_ns = 0;     // when it was produced
  int64_t valid_until_ns = 0;   // after which it must not be honoured

  // LOS target (base frame), for LosDirection.
  bool has_los = false;
  double los_az_rad = 0.0;
  double los_el_rad = 0.0;

  // Joint target (raw rad), for JointPosition and as the fallback for
  // WorldLevelYaw once the level constraint has been converted.
  bool has_joint_target = false;
  double q_yaw_rad = 0.0;
  double q_pitch_rad = 0.0;

  // Bounded velocity command (raw rad/s, signed), for JointVelocity.
  bool has_joint_velocity = false;
  double v_yaw_rad_s = 0.0;
  double v_pitch_rad_s = 0.0;

  // World-level elevation request, for WorldLevelYaw (§33/§40).
  bool has_world_elevation = false;
  double world_elevation_rad = 0.0;

  // Scales applied to the configured v/a/j for this intent. §19 derates on
  // target confidence and §20.1 derates progressively while coasting; both are
  // expressed here so one mechanism covers both, and so the derating is visible
  // in telemetry rather than buried inside a controller.
  double velocity_scale = 1.0;
  double acceleration_scale = 1.0;
  double jerk_scale = 1.0;

  // Selected-target confidence this intent was formed at (0..1; 1 for "n/a").
  double confidence = 1.0;

  char reason[kReasonLen] = {};

  MotionIntent() { std::memset(reason, 0, sizeof(reason)); }

  // Bounded, non-allocating. Truncates rather than overrunning: the reason is a
  // diagnostic for a human, and a truncated diagnostic is worth less than a
  // corrupted intent.
  void set_reason(const char* text) {
    if (text == nullptr) { reason[0] = '\0'; return; }
    std::snprintf(reason, sizeof(reason), "%s", text);
  }

  // An intent is live at `now_ns` if it has not expired. An intent that never
  // set a deadline (valid_until_ns == 0) is treated as always live: that is the
  // hold case, and a hold that silently aged out would be worse than a hold.
  bool live_at(int64_t now_ns) const {
    return valid_until_ns == 0 || now_ns < valid_until_ns;
  }

  static MotionIntent hold(MotionSource source, const char* why) {
    MotionIntent in;
    in.source = source;
    in.type = IntentType::Hold;
    in.confidence = 1.0;
    in.set_reason(why);
    return in;
  }
};

}  // namespace ota
