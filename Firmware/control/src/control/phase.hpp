// OpenAutoTurret — the control-loop phase vocabulary.
//
// It lives here, and not in control_loop.hpp, because the telemetry record has
// to name the phase it was captured in: a per-cycle row that cannot say whether
// the axis was held or commanded cannot separate a stale request from a stalled
// mechanism. telemetry.hpp cannot include control_loop.hpp (that include goes
// the other way), so the vocabulary sits between them.
#pragma once

namespace ota {

enum class Phase {
  Idle,     // no motion phase active (pre-homing or post-park)
  Homing,   // executing the multi-axis homing plan
  Hold,     // ready-hold: at (or moving to) the safe ready pose, position mode
  Parking,  // executing the safe park / shutdown sequence (§33)
  Parked,   // de-energized at the park pose (power-safe)
  Fault,    // fault-locked: controlled stop commanded, no further motion
  Recovering, // disabled motor fault clear and feedback verification
  // Phase 9: payload response check (§27, §31.3) — small moves in the safe
  // central region, one axis at a time.
  PayloadCheck,
};

inline const char* phase_name(Phase p) {
  switch (p) {
    case Phase::Idle:    return "idle";
    case Phase::Homing:  return "homing";
    case Phase::Hold:    return "hold";
    case Phase::Parking: return "parking";
    case Phase::Parked:  return "parked";
    case Phase::Fault:   return "fault";
    case Phase::Recovering: return "recovering";
    case Phase::PayloadCheck: return "payload_check";
  }
  return "?";
}

}  // namespace ota
