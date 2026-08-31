#pragma once
// OpenAutoTurret — common core types.
//
// Conventions (architecture doc §9):
//  - All critical timing uses a monotonic clock, expressed as nanoseconds
//    since boot (TimeNs). The camera SensorTimestamp lives in the same
//    monotonic domain.
//  - Mechanical joint coordinates (q_yaw, q_pitch) are kept strictly
//    separate from world azimuth/elevation.
#include <cstdint>

namespace ota {

using TimeNs = int64_t;

// Unit conversions (mechanical coordinates are radians internally; the host
// logical frame and most configuration values are degrees).
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Logical axes. CAN IDs come from configuration, never hard-coded here.
enum class AxisId : uint8_t {
  Pitch = 0,
  Yaw = 1,
  Count = 2,
};

// Number of axes as a plain int (AxisId is an enum class and cannot index a
// built-in array / std::array or be used in integer comparisons directly).
inline constexpr int kAxisCount = static_cast<int>(AxisId::Count);

inline constexpr const char* axis_name(AxisId a) {
  switch (a) {
    case AxisId::Pitch: return "pitch";
    case AxisId::Yaw:   return "yaw";
    default:            return "?";
  }
}

// System states (architecture doc, Appendix A).
enum class SystemState : uint8_t {
  Boot,
  CanInit,
  SelfTest,
  Unhomed,
  Homing,
  Calibrating,
  ReadyHold,
  Tracking,
  Coasting,
  Search,
  Braking,
  Parking,
  Parked,
  Fault,
};

// Per-axis homing state machine (Appendix A).
enum class AxisHomeState : uint8_t {
  Unknown,
  ApproachCoarse,
  ContactCoarse,
  Backoff,
  Settle,
  ApproachFine,
  ContactFine,
  VerifyRepeatability,
  Complete,
  Failed,
};

enum class TargetState : uint8_t {
  None,
  Candidate,
  Tracked,
  Coasting,
  Lost,
};

// Safety supervisor outputs (Appendix A / §38).
enum class SafetyAction : uint8_t {
  Allow,
  Derate,
  Brake,
  Hold,
  FaultStop,
  Disable,
};

inline constexpr const char* system_state_name(SystemState s) {
  switch (s) {
    case SystemState::Boot:        return "BOOT";
    case SystemState::CanInit:     return "CAN_INIT";
    case SystemState::SelfTest:    return "SELF_TEST";
    case SystemState::Unhomed:     return "UNHOMED";
    case SystemState::Homing:      return "HOMING";
    case SystemState::Calibrating: return "CALIBRATING";
    case SystemState::ReadyHold:   return "READY_HOLD";
    case SystemState::Tracking:    return "TRACKING";
    case SystemState::Coasting:    return "COASTING";
    case SystemState::Search:      return "SEARCH";
    case SystemState::Braking:     return "BRAKING";
    case SystemState::Parking:     return "PARKING";
    case SystemState::Parked:      return "PARKED";
    case SystemState::Fault:       return "FAULT";
  }
  return "?";
}

inline constexpr const char* safety_action_name(SafetyAction a) {
  switch (a) {
    case SafetyAction::Allow:     return "ALLOW";
    case SafetyAction::Derate:    return "DERATE";
    case SafetyAction::Brake:     return "BRAKE";
    case SafetyAction::Hold:      return "HOLD";
    case SafetyAction::FaultStop: return "FAULT_STOP";
    case SafetyAction::Disable:   return "DISABLE";
  }
  return "?";
}

}  // namespace ota
