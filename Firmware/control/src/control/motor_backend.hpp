// OpenAutoTurret — motor actuator abstraction (architecture §8, §46).
//
// The control loop and boot state machine depend ONLY on this interface, never
// on the concrete SocketCAN transport. That keeps the safety-critical per-cycle
// logic unit-testable against a simulated plant with no CAN (§54), while the
// production daemon uses CanMotorBackend (wrapping CyberGearSystem).
//
// Two classes of operation, mirroring §46:
//   * setup (SLOW, blocking, bounded) — discovery, register reads, entering
//     position mode (the stop -> RunMode=1 -> enable -> LimitSpd -> pin-LocRef
//     recipe). Called at boot / phase transitions, NEVER from the 200 Hz loop.
//   * control loop (FAST, non-blocking, fire-and-forget) — snapshot the latest
//     feedback and write the position reference + speed limit. No register-query
//     chain, no sleeps.
#pragma once

#include <cstdint>
#include <string>

#include "can/cybergear_protocol.hpp"  // cybergear::Reg
#include "common/types.hpp"

namespace ota {

// Latest known state of one axis (a non-blocking read of the freshest feedback).
struct AxisSnapshot {
  bool has_feedback = false;
  TimeNs rx_ns = 0;        // host monotonic time of the freshest feedback
  double q_rad = 0.0;
  double v_rad_s = 0.0;
  double torque_nm = 0.0;
  double temp_c = 25.0;
  uint16_t faults = 0;     // non-zero = hard fault
  bool in_position_mode = false;  // energized in position mode right now
  bool in_speed_mode = false;     // energized in speed (velocity) mode right now
};

class MotorBackend {
 public:
  virtual ~MotorBackend() = default;

  // --- setup (slow; boot / phase transitions only) ------------------------
  // Discover the motor on the bus (returns its unique id). Boot only.
  virtual bool discover(AxisId axis, uint64_t& unique_id, std::string& err) = 0;
  // Read one register (diagnostics / the position-mode pin). Bounded wait.
  virtual bool read_register(AxisId axis, cybergear::Reg reg, double& value,
                             int timeout_ms, std::string& err) = 0;
  // Enter position mode on this axis: de-energize, set RunMode=1, re-energize,
  // set the speed limit, and pin LocRef to the freshly-read position (so the
  // motor does not drive to a stale target). SLOW — phase transitions only.
  virtual bool enter_position_mode(AxisId axis, double limit_spd_rad_s,
                                   std::string& err) = 0;
  // Enter speed (velocity) mode on this axis: de-energize, set RunMode=2,
  // re-energize, set the current limit, and command SpdRef=0 (hold in place).
  // The drive's own velocity loop then holds the commanded speed smoothly —
  // the correct mode for "drive at a constant speed until something stops us"
  // (homing / zeroing, free roam), as opposed to position mode's
  // "drive to a target and hold" (tracking, hold). SLOW — phase transitions
  // only.
  virtual bool enter_speed_mode(AxisId axis, double limit_cur_a,
                                std::string& err) = 0;
  // De-energize the motor (safe stop). Called on shutdown / disable / fault.
  virtual void deenergize(AxisId axis) = 0;

  // --- control loop (fast; non-blocking, fire-and-forget) -----------------
  // Snapshot the freshest feedback for one axis (no blocking).
  virtual AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) = 0;
  // Position-mode command: write LocRef = q_ref_rad and LimitSpd =
  // limit_spd_rad_s. Fire-and-forget; safe to call every cycle.
  virtual void command(AxisId axis, double q_ref_rad,
                       double limit_spd_rad_s) = 0;
  // Speed-mode command: write SpdRef = velocity_rad_s (the drive holds this
  // speed with its internal velocity loop; current rises as needed up to the
  // current limit). Fire-and-forget; safe to call every cycle.
  virtual void command_velocity(AxisId axis, double velocity_rad_s) = 0;
  // Feedback keepalive: elicit a fresh COMM_TYPE_2 response WITHOUT changing
  // any reference (the CyberGear has no periodic telemetry — it answers
  // commands only). Needed for speed-mode axes on Allow cycles, where no
  // reference command is issued: without a periodic ping the feedback age
  // crosses feedback_max_age_ms and the supervisor flaps BRAKE/ALLOW every
  // ~100 ms, and each BRAKE stomps the other axis's reference (p0p hold
  // phase; the p3e fault-phase flap). No-op where feedback is self-generated
  // (sim). Safe to call every cycle; the implementation rate-limits.
  virtual void keepalive(AxisId axis) {}
  // Set the drive current limit (A, 0..23) for this axis (LimitCur, 0x7018).
  // Fire-and-forget; safe from the control loop — the adaptive-current homing
  // raises it on each false-contact latch (§22).
  virtual void set_current_limit(AxisId axis, double limit_cur_a) = 0;
  // Set the drive's inner speed-loop gains (SpdKp 0x701F, SpdKi 0x7020).
  // The stock CyberGear gains (SpdKp=1.0, SpdKi=0.002) are too weak to hold
  // the position-mode speed limit against a gravity load: on the pitch axis
  // the "against-gravity" half of a 2 deg check step creeps at a fraction of
  // the commanded rate on a few hundred milliamps and never settles in the
  // move budget (the "with-gravity" half is assisted and is fast). Raising the
  // speed-loop gains lets the inner loop build the torque needed to hold the
  // commanded rate against gravity, so the step response is the drive's
  // controlled response (mass-sensitive) rather than a gravity-dominated
  // creep. Fire-and-forget; the values are still bounded by the current /
  // torque limits. No-op where the backend has no drive-internal loop gains
  // (sim: its plant is a fixed time constant, not a tuned velocity loop).
  virtual void set_speed_loop_gains(AxisId axis, double spd_kp,
                                    double spd_ki) {}
};

}  // namespace ota
