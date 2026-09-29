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
#include <limits>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "can/cybergear_protocol.hpp"  // cybergear::Reg
#include "common/types.hpp"
#include "control/park_position_evidence.hpp"

namespace ota {

// Latest known state of one axis (a non-blocking read of the freshest feedback).
struct AxisSnapshot {
  bool has_feedback = false;
  TimeNs rx_ns = 0;        // host monotonic time of the freshest feedback
  TimeNs raw_rx_ns = 0;    // unchanged transport timestamp; trace prefers this when supplied
  uint64_t rx_seq = 0;
  int encoder_raw = -1;
  int current_raw = 0;
  bool current_raw_valid = false;
  int enabled_state = -1; // CyberGear type2 state, NOT RunMode
  double q_rad = 0.0;
  double v_rad_s = 0.0;
  double torque_nm = 0.0;
  // Torque current as the drive itself reports it, in amperes. Kept apart from torque_nm on
  // purpose: the GM6020's status frame carries a current, not a torque, and the guide gives
  // no torque constant to divide by -- so an N·m figure from it would be an inference
  // wearing a familiar label. Drives reporting neither leave this NaN with
  // current_a_known=false; the rule is null, never a flattering zero.
  double current_a = std::numeric_limits<double>::quiet_NaN();
  bool current_a_known = false;
  double temp_c = 25.0;
  bool temperature_known = true;  // false when the protocol has no established °C scale
  bool temperature_raw_valid = false;
  uint8_t temperature_raw = 0;    // opaque wire value; never interpret as °C by itself
  uint16_t faults = 0;     // non-zero = hard fault
  bool faults_known = true;       // false when the feedback protocol has no fault field
  bool disabled = false;          // confirmed by feedback, not by a sent STOP command
  bool disabled_known = true;     // false when the protocol cannot confirm disable state
  bool in_position_mode = false;  // energized in position mode right now
  bool in_speed_mode = false;     // energized in speed (velocity) mode right now
};

// CAN link health (§55 CAN family, §54.4 error-state observation).
//
// The transports have counted all of this since the transport interface
// existed (can_transport.hpp BusStats: rx/tx/error/failure counters + the
// interface state), and NOTHING consumed it: the acceptance report could not ask
// for the numbers, and a link quietly rotting into error-passive was invisible
// on the dashboard until feedback went stale. Defaults matter here: the sim
// backend reports available=false, so a simulated run can never look like a
// healthy bus.
struct CanHealth {
  bool available = false;
  std::string kind;              // "socketcan" | "yousee"
  std::string device;            // "can0" | "/dev/ttyUSB0"
  bool up = false;
  int state = -1;                // CanIfState as int: -1 unknown, 0 error-active,
                                 // 1 error-warning, 2 error-passive, 3 bus-off,
                                 // 4 stopped, 5 sleeping
  uint64_t rx_frames = 0;
  uint64_t rx_error_frames = 0;  // decode/framing losses (PHY corruption hint)
  uint64_t tx_frames = 0;
  uint64_t tx_failed = 0;
  TimeNs last_rx_ns = 0;         // host monotonic; 0 = never received anything
};

class MotorBackend {
 public:
  struct OutputEvidence {
    TimeNs tx_ns = 0;
    uint64_t tx_seq = 0;
    double requested = std::numeric_limits<double>::quiet_NaN();
    double successful = std::numeric_limits<double>::quiet_NaN();
    double integral = std::numeric_limits<double>::quiet_NaN();
    double velocity_estimate = std::numeric_limits<double>::quiet_NaN();
    double kp = std::numeric_limits<double>::quiet_NaN();
    double ki = std::numeric_limits<double>::quiet_NaN();
    double current_cap = std::numeric_limits<double>::quiet_NaN();
    int reason = 0; // 0 unknown, 1 normal, 2 explicit zero, 3 inhibited, 4 TX failed, 5 late cycle
    int command_kind = 0; // 0 unknown, 1 current A, 2 voltage counts, 3 SpdRef rad/s, 4 LocRef rad, 5 STOP
  };
  virtual OutputEvidence output_evidence(AxisId) const { return {}; }
  virtual ~MotorBackend() = default;
  void set_calibration_invalidator(std::function<void()> callback) { invalidate_ = std::move(callback); }
  void invalidate_calibration() { if (invalidate_) invalidate_(); }
  virtual bool adopt_running_mode(AxisId, bool, std::string&, double = -1, double = 1) { return false; }
  virtual void heartbeat() {}
  virtual bool watchdog_fault() const { return false; }
  // Independent links can inhibit one axis without releasing a healthy load.
  virtual bool watchdog_fault_axis(AxisId) const { return watchdog_fault(); }
  // The reason a guard latched, captured by the guard itself at trip time. Fixed-size
  // POD because the thread describing a fault must not allocate to do it; detail is
  // truncated rather than grown. `condition` is the machine-readable token the fault
  // string and the MOTOR_WATCHDOG_TRIP event both carry.
  struct TripDetail {
    bool valid = false;
    char condition[24] = {};
    char detail[96] = {};
  };
  // The guard's condition inputs, flattened so naming a cause is testable without a
  // CAN bus. Every flag mirrors exactly one disjunct of the guard's own should_stop:
  // a state that is not in should_stop must not appear as a candidate cause, or it
  // shadows the condition that actually fired. `reference_valid` travels as context
  // for exactly that reason.
  // What the axis was actually ASKED to do last cycle, and what came out of the actuator
  // law. On a voltage-mode yaw these are visible nowhere else: the drive reports position
  // and temperature only. Measured today, manual yaw peaked at 5.3 deg/s while pitch reached
  // 17.6, and the station could not tell me whether we asked for 5 or asked for 20 and were
  // slow getting there -- which is the difference between a settings bug and an actuator
  // bug. Zero by default: an axis that does not track its own ask says so by being silent.
  // Per axis, because one backend may own axes it cannot speak for. Unknown is NaN, not 0:
  // zero reads as "asked for nothing", while the truth is often "this drive does not tell
  // us", and both the telemetry line and the trip trace already render non-finite as null.
  virtual double diag_commanded_speed_rad_s(AxisId axis) const {
    (void)axis; return std::numeric_limits<double>::quiet_NaN();
  }
  virtual double diag_output(AxisId axis) const {
    (void)axis; return std::numeric_limits<double>::quiet_NaN();
  }
  virtual bool diag_degraded() const { return false; }
  virtual int diag_guard_events() const { return 0; }

  struct TripInputs {
    bool feedback_unsafe = false;
    bool can_down = false;
    bool can_state_wrong = false;
    bool can_counters_bad = false;
    bool bus_unhealthy = false;
    bool speed_not_finite = false;
    bool temp_raw_over = false;
    bool no_progress = false;
    // The command reached the backend but was not allowed out on the wire (non-finite,
    // motion not permitted, heartbeat stale). Zero gets sent and the previous requested
    // speed is no longer a description of anything, so it must not be the field a
    // `no_progress` verdict rests on -- 2026-09-28 read three trips that way.
    bool command_not_sent = false;
    bool heartbeat_stale = false;
    bool reference_valid = true;
    double feedback_age_ms = 0.0;
    unsigned temp_raw = 0;
    double speed_deg_s = 0.0;
  };
  // The first condition, in the guard's evaluation order, that would have latched it.
  static const char* select_trip_condition(const TripInputs& in) {
    if (in.feedback_unsafe) return "feedback_unsafe";
    if (in.can_down) return "can_down";
    if (in.can_state_wrong) return "can_state";
    if (in.temp_raw_over) return "temp_raw_over";
    if (in.heartbeat_stale) return "heartbeat_stale";
    if (in.can_counters_bad) return "can_counters";
    if (in.bus_unhealthy) return "bus_unhealthy";
    if (in.speed_not_finite) return "speed_nan";
    // `speed_over_ceiling` was deleted from this vocabulary, not lowered: it fired on a
    // READING, so a momentary overshoot removed power from an unbalanced payload. The
    // ceiling now clamps the command instead. Non-finite stays -- a NaN feedback is not
    // a fast axis, it is an axis we cannot see.
    // Before `no_progress` and on purpose: "we asked for 10 deg/s and nothing happened"
    // is a different accusation when we know the frame was never sent. The specific
    // truth outranks the inference.
    if (in.command_not_sent) return "command_not_sent";
    if (in.no_progress) return "no_progress";
    return "unknown";
  }
  // The compact matrix that travels with the token. The full field dump stays in the
  // log; this is what the event and the fault string can carry. Truncated, never grown.
  static void format_trip_detail(const TripInputs& in, const char* condition, TripDetail& out) {
    out.valid = true;
    std::snprintf(out.condition, sizeof(out.condition), "%s", condition);
    std::snprintf(out.detail, sizeof(out.detail),
                  "cond=%s fb_age_ms=%.3f temp_raw=%u speed_deg_s=%.3f can_down=%d ref_valid=%d",
                  condition, in.feedback_age_ms, in.temp_raw, in.speed_deg_s,
                  in.can_down ? 1 : 0, in.reference_valid ? 1 : 0);
  }
  virtual TripDetail watchdog_trip_detail() const { return {}; }
  virtual ParkPositionEvidence park_position_evidence(AxisId, TimeNs) const { return {}; }
  enum class Transition { Pending, Complete, Failed };
  virtual bool recovery_before_homing() const { return false; }
  // Topology/protocol capabilities. Legacy CyberGear and simulation retain
  // the original finite-yaw, register-backed, feedback-confirmed defaults.
  // A mixed backend can opt into continuous yaw and session-relative yaw
  // feedback without inventing a CyberGear UID or register response.
  virtual bool supports_continuous_yaw() const { return false; }
  virtual bool yaw_feedback_registerless() const { return false; }
  virtual bool requires_disable_confirmation(AxisId) const { return true; }
  virtual bool begin_motor_recovery(std::string& err) {
    err = "motor recovery unsupported by this backend"; return false;
  }
  virtual Transition poll_motor_recovery(TimeNs, double, std::string& err) {
    err = "motor recovery unsupported by this backend"; return Transition::Failed;
  }
  virtual void cancel_motor_recovery() {}
  // Repeated from the control loop. Hardware implements a nonblocking recipe.
  virtual Transition transition_mode(AxisId axis, bool position, double limit,
                                     TimeNs now_ns, std::string& err, double speed_ki = -1, double speed_kp = 1,
                                     bool check_displacement = true) {
    return (position ? enter_position_mode(axis, limit, err) : enter_speed_mode(axis, limit, err))
        ? Transition::Complete : Transition::Failed;
  }

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

  // Observe-only bus health. Not part of the control path: the loop reads it at
  // report time, never to decide anything. Backends without a CAN link (the
  // simulated plant) keep the default, which says "nothing to report".
  virtual CanHealth can_health() const { return {}; }
  // A mixed topology may have independent CAN links. Existing single-bus
  // backends keep their legacy health result through this default adapter.
  virtual std::vector<CanHealth> can_health_all() const { return {can_health()}; }
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
 private:
  std::function<void()> invalidate_;
};

}  // namespace ota
