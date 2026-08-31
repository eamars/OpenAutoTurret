// OpenAutoTurret — boot state machine (architecture §27).
//
// The boot sequence that runs BEFORE the 200 Hz control loop and before any
// homing motion. It is a slow, blocking, bounded path (§46: setup/diagnostics
// are slow boot-only paths, never in the control loop):
//
//   POWER_ON -> PROCESS_INIT -> CAN_INIT -> DISCOVER (pitch, yaw)
//            -> MOTOR_SELF_TEST (pitch, yaw) -> UNHOMED
//
// Any failure goes to FAULT_LOCKED (the station never homes or moves with an
// unknown motor state). The camera/installation/payload states of §27 come
// AFTER homing and are STUBS in Phase 2 (the ControlLoop's hold phase passes
// straight through them); they are not part of this pre-homing FSM.
//
// Transport-agnostic: it depends only on the MotorBackend, so the whole boot
// sequence is unit-testable against the SimMotorBackend with no CAN.
#pragma once

#include <array>
#include <string>

#include "can/cybergear_protocol.hpp"
#include "control/motor_backend.hpp"

namespace ota {

enum class BootState {
  PowerOn,
  ProcessInit,
  CanInit,
  DiscoverPitch,
  DiscoverYaw,
  SelfTestPitch,
  SelfTestYaw,
  Unhomed,      // ready to start the homing plan
  FaultLocked,  // boot failure; no homing, no motion
};

inline const char* boot_state_name(BootState s) {
  switch (s) {
    case BootState::PowerOn:       return "power_on";
    case BootState::ProcessInit:   return "process_init";
    case BootState::CanInit:       return "can_init";
    case BootState::DiscoverPitch: return "discover_pitch";
    case BootState::DiscoverYaw:   return "discover_yaw";
    case BootState::SelfTestPitch: return "self_test_pitch";
    case BootState::SelfTestYaw:   return "self_test_yaw";
    case BootState::Unhomed:       return "unhomed";
    case BootState::FaultLocked:   return "fault_locked";
  }
  return "?";
}

struct BootConfig {
  int discovery_timeout_ms = 500;
  int register_timeout_ms = 500;
  // The self-test reads this register from each motor to confirm it responds.
  cybergear::Reg self_test_register = cybergear::Reg::MechPos;
};

class BootFsm {
 public:
  BootFsm(MotorBackend& backend, BootConfig cfg);

  // Advance the boot FSM by one operation (one discovery or one self-test).
  // Each step may do a bounded slow operation; none of this runs in the 200 Hz
  // control loop. Returns the current state after the step.
  BootState step();

  BootState state() const { return state_; }
  bool ready_to_home() const { return state_ == BootState::Unhomed; }
  bool faulted() const { return state_ == BootState::FaultLocked; }
  const std::string& error() const { return error_; }
  const std::array<uint64_t, kAxisCount>& unique_ids() const {
    return unique_ids_;
  }

 private:
  void fail(const std::string& reason);

  MotorBackend& backend_;
  BootConfig cfg_;
  BootState state_ = BootState::PowerOn;
  std::string error_;
  std::array<uint64_t, kAxisCount> unique_ids_{};
};

}  // namespace ota
