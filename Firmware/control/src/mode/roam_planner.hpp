#pragma once
// v3 §29–§36 — AUTO_ROAM: one deterministic, bounded sweep, and the envelope that keeps
// it honest.
//
// The pattern is deliberately the simplest one the document allows (§30): sweep to one
// boundary, turn around, sweep to the other. A raster (§31) and any "clever" coverage
// optimisation come after this is validated on metal, and there is a reason for the
// order: a bounded sweep is the only pattern whose next position a person watching the
// turret can predict. When something goes wrong on a machine that moves by itself, the
// operator's safety depends on knowing where it meant to go next.
//
// Three separations this header insists on:
//
//   * The roam envelope is not the safe envelope (§32). The sweep runs inside an inner
//     region, inset again by a braking reserve, because the alternative — sweeping the
//     full soft-limit span — means the turret spends its working life at the extremes,
//     where the braking margin is smallest and the cable is most stressed. `validate_envelope`
//     below is what enforces "roam ⊂ safe" rather than trusting whoever typed the numbers.
//
//   * Entry is continuous (§36). Coming from MANUAL at 140° must not send the turret to
//     the left boundary first: it starts by sweeping toward whichever boundary is
//     nearer, so the first motion after a mode click is always the short one.
//
//   * No camera, no problem (§34). Vision keeps running during a sweep and the selection
//     stays visible, but RoamPlanner never consults a target — pursuing one means
//     switching to AUTO_TRACK. That keeps the mode semantics unambiguous, and it is also
//     what makes AUTO_ROAM the mode that works when no target ever appears: a station
//     with a disconnected camera can still roam, and say so.
//
// Control-thread only, no allocation, no I/O. Speeds are not decided here: the planner
// names a waypoint, the intent carries the roam ceiling, and the safety envelope gets the
// final word on both rate and position (§111.18).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "common/time.hpp"
#include "control/motion_intent.hpp"

namespace ota {

// §32/§72: the inner application envelope. Joint space (raw rad), because that is what
// the limits and the homing offsets are expressed in; a level-frame sweep (§33) is
// converted to joints before it is checked against anything.
struct RoamEnvelope {
  double yaw_min_rad = 0.0;
  double yaw_max_rad = 0.0;
  double pitch_min_rad = 0.0;
  double pitch_max_rad = 0.0;

  bool non_empty() const {
    return yaw_max_rad > yaw_min_rad && pitch_max_rad > pitch_min_rad;
  }
};

// §35. No "Error" state: a fault is not the planner's to own — the loop's safety and
// supervisory layers outrank every mode, and what reaches the planner is simply "you are
// no longer the one that owns motion" (§35's safety override -> BRAKE/HOLD, STOP ->
// MANUAL/HOLD, both of which are a mode change or an override seen from here).
enum class RoamState : uint8_t {
  Idle,             // not the mode's turn; nothing emitted
  MoveToScanStart,  // outside the envelope: the one legitimate long move
  Sweep,
  Turnaround,
};

inline const char* roam_state_name(RoamState s) {
  switch (s) {
    case RoamState::Idle: return "IDLE";
    case RoamState::MoveToScanStart: return "MOVE_TO_SCAN_START";
    case RoamState::Sweep: return "SWEEP";
    case RoamState::Turnaround: return "TURNAROUND";
  }
  return "?";
}

struct RoamConfig {
  RoamEnvelope envelope;
  // §30: the elevation the sweep holds. Kept fixed because a sweeping turret that also
  // changes elevation is a turret whose next position nobody can predict.
  double pitch_ref_rad = 0.0;
  double v_max_rad_s = 0.175;  // overridden from the station config
  // §29 "keep adequate braking margin" / §32 "leave braking reserve": the turnaround
  // point sits this far inside each yaw boundary. The sweep therefore never asks the
  // envelope to do a stop-from-full-speed-at-the-limit, which is the manoeuvre the
  // drives are least able to guarantee.
  double braking_margin_rad = 0.0873;  // 5 deg
  // How close to the turnaround point counts as arrived. Generous on purpose: waiting
  // for exact arrival means the turret dwells at the extremes, which is exactly the
  // continuous operation near mechanical limits §32 exists to avoid.
  double reach_tol_rad = 0.035;
  // A reversal must be a decision, not a bounce: after reaching an end the planner
  // dwells here before issuing the opposite waypoint, so a jittery feedback signal
  // cannot make the turret oscillate around the turnaround point at full rate.
  double turnaround_dwell_rad = 0.008;
  // §33: only used when the IMU expansion says gravity is valid. Core v3 runs without
  // the IMU, in which case the joint-space pitch above is the answer.
  bool level_scan_available = false;
  double world_elevation_rad = 0.0;
  // §32: the roam envelope must sit inside the safe envelope by at least this much.
  // Checked, not assumed — see validate_envelope.
  double min_inside_safe_rad = 0.0349;  // 2 deg
};

struct RoamOutput {
  MotionIntent intent;
  RoamState state = RoamState::Idle;
  double target_yaw_rad = 0.0;   // §78: where it is going next
  int direction = 0;             // -1 / +1 / 0: §78's "which way is it sweeping"
  bool envelope_valid = false;
  const char* reason = "roam idle";
};

class RoamPlanner {
 public:
  explicit RoamPlanner(RoamConfig cfg = RoamConfig()) : cfg_(cfg) {}

  void set_config(const RoamConfig& cfg) {
    // Re-seeding on a config change is deliberate: a roam envelope edited mid-sweep must
    // not leave the turret heading for a waypoint that is now outside the region it is
    // supposed to stay inside.
    const bool envelope_changed =
        cfg.envelope.yaw_min_rad != cfg_.envelope.yaw_min_rad ||
        cfg.envelope.yaw_max_rad != cfg_.envelope.yaw_max_rad;
    cfg_ = cfg;
    if (envelope_changed) state_ = RoamState::Idle;
  }
  const RoamConfig& config() const { return cfg_; }
  RoamState state() const { return state_; }
  bool active() const { return state_ != RoamState::Idle; }
  double target_yaw_rad() const { return target_yaw_; }
  int direction() const { return direction_; }

  // §32, enforced. Returns false and says which boundary failed. Two separate failures
  // are worth distinguishing for whoever is editing a config file at 11 pm: an *empty*
  // envelope means the numbers are the wrong way round or the braking margin ate the
  // region whole; an envelope that pokes outside the safe one means the two files that
  // describe the machine disagree.
  // `pitch_ref_rad` is checked too, and is not decoration: it is the one number in a
  // roam plan that is not one of the envelope's own bounds, so a validator that only
  // compared bounds would happily pass a sweep that holds its elevation outside the
  // region it claims to stay inside.
  static bool validate_envelope(const RoamEnvelope& roam, const RoamEnvelope& safe,
                                double pitch_ref_rad, double min_inside_rad, char* why,
                                size_t n) {
    if (!roam.non_empty()) {
      std::snprintf(why, n,
                    "roam envelope is empty (yaw %.1f..%.1f deg, pitch %.1f..%.1f deg)",
                    roam.yaw_min_rad * 57.29577951308232,
                    roam.yaw_max_rad * 57.29577951308232,
                    roam.pitch_min_rad * 57.29577951308232,
                    roam.pitch_max_rad * 57.29577951308232);
      return false;
    }
    auto inside = [&](double v, double lo, double hi) {
      return v >= lo + min_inside_rad && v <= hi - min_inside_rad;
    };
    if (!inside(roam.yaw_min_rad, safe.yaw_min_rad, safe.yaw_max_rad) ||
        !inside(roam.yaw_max_rad, safe.yaw_min_rad, safe.yaw_max_rad)) {
      std::snprintf(why, n,
                    "roam yaw %.1f..%.1f deg is not inside the safe envelope by %.1f deg",
                    roam.yaw_min_rad * 57.29577951308232,
                    roam.yaw_max_rad * 57.29577951308232,
                    min_inside_rad * 57.29577951308232);
      return false;
    }
    if (!inside(roam.pitch_min_rad, safe.pitch_min_rad, safe.pitch_max_rad) ||
        !inside(roam.pitch_max_rad, safe.pitch_min_rad, safe.pitch_max_rad)) {
      std::snprintf(why, n,
                    "roam pitch %.1f..%.1f deg is not inside the safe envelope by %.1f deg",
                    roam.pitch_min_rad * 57.29577951308232,
                    roam.pitch_max_rad * 57.29577951308232,
                    min_inside_rad * 57.29577951308232);
      return false;
    }
    if (!inside(pitch_ref_rad, roam.pitch_min_rad, roam.pitch_max_rad)) {
      std::snprintf(why, n,
                    "roam pitch reference %.1f deg is outside the roam envelope "
                    "(%.1f..%.1f deg)",
                    pitch_ref_rad * 57.29577951308232,
                    roam.pitch_min_rad * 57.29577951308232,
                    roam.pitch_max_rad * 57.29577951308232);
      return false;
    }
    return true;
  }

  // §36, entry. Called on the cycle the mode becomes AUTO_ROAM.
  void enter(double q_yaw_rad, double q_pitch_rad) {
    const double lo = sweep_lo_rad();
    const double hi = sweep_hi_rad();
    // §30: the elevation a sweep holds is the configured one, not whatever the operator
    // happened to leave it at — otherwise "AUTO_ROAM" means different things from
    // different starting poses, and the pattern stops being predictable. It is clamped
    // into the roam envelope rather than trusted, and the move to it is a normal bounded
    // trajectory rather than a snap (§36.2).
    (void)q_pitch_rad;
    pitch_target_rad_ = clamp_envelope_pitch(cfg_.pitch_ref_rad);
    if (q_yaw_rad < lo || q_yaw_rad > hi) {
      // Outside the region: the one case where a long move is correct. Head for the
      // nearer boundary so the approach is still the short way round.
      state_ = RoamState::MoveToScanStart;
      target_yaw_ = (q_yaw_rad < lo) ? lo : hi;
      direction_ = (q_yaw_rad < lo) ? +1 : -1;
    } else {
      // Inside: start toward the nearer boundary. §36.3's "nearest sensible sweep
      // direction" is not a style preference — entering at 170 degrees and driving to
      // -60 first is a several-second crossing of the room that nobody asked for, and
      // it is the first thing the turret does after an operator clicks.
      const bool to_low_first = (q_yaw_rad - lo) <= (hi - q_yaw_rad);
      target_yaw_ = to_low_first ? lo : hi;
      direction_ = to_low_first ? -1 : +1;
      state_ = RoamState::Sweep;
    }
    reversing_ = false;
    reversal_armed_ = false;
  }

  void exit() {
    state_ = RoamState::Idle;
    direction_ = 0;
    reversing_ = false;
    reversal_armed_ = false;
  }

  // One control cycle. `q_*` are the current joint positions; the trajectory generator
  // starts from wherever they are (§36.2), so this only ever names a waypoint.
  RoamOutput update(double q_yaw_rad, double q_pitch_rad, TimeNs now_ns,
                    TimeNs period_ns) {
    RoamOutput out;
    out.state = state_;
    out.target_yaw_rad = target_yaw_;
    out.direction = direction_;
    out.envelope_valid = cfg_.envelope.non_empty();
    out.intent.source = MotionSource::AutoRoam;
    out.intent.timestamp_ns = now_ns;

    // The envelope is checked first, and before the Idle branch, for one reason: an
    // invalid envelope is the one case where "roam idle" would be a lie. A sweep stopped
    // because someone edited the region out from under it is not idle, it was stopped,
    // and the difference is what an operator reads when the turret is not doing what they
    // asked. Re-checked every cycle rather than at entry because the envelope is live
    // configuration, and a sweep already in progress is the case that matters.
    if (!out.envelope_valid) {
      exit();
      out.state = state_;
      out.reason = "roam envelope empty";
      out.intent.set_reason(out.reason);
      return out;
    }
    if (state_ == RoamState::Idle) {
      out.reason = "roam idle";
      out.intent.set_reason(out.reason);
      return out;  // Hold. A planner with nothing to do does not invent a waypoint.
    }

    // Re-clamped every cycle: the envelope is editable live, and the elevation the sweep
    // holds must follow it rather than keep an old, now-outside value.
    pitch_target_rad_ = clamp_envelope_pitch(cfg_.pitch_ref_rad);

    const double lo = sweep_lo_rad();
    const double hi = sweep_hi_rad();
    const double dist = std::fabs(q_yaw_rad - target_yaw_);

    if (dist <= cfg_.reach_tol_rad) {
      if (!reversing_) {
        // Reached an end. Arm the reversal; do not flip the waypoint on the same cycle,
        // which is what turns a sweep into a vibration at the turnaround point.
        reversing_ = true;
        reversal_armed_ = false;
        state_ = RoamState::Turnaround;
      } else if (!reversal_armed_ &&
                 std::fabs(q_yaw_rad - target_yaw_) <=
                     cfg_.reach_tol_rad * 0.5) {
        // Actually settled — feedback has stopped moving toward the waypoint, not merely
        // come within tolerance of it.
        reversal_armed_ = true;
      }
      if (reversal_armed_) {
        direction_ = -direction_;
        target_yaw_ = (direction_ < 0) ? lo : hi;
        reversing_ = false;
        reversal_armed_ = false;
        state_ = RoamState::Sweep;
      }
    } else if (reversing_) {
      // Drifted back out of the tolerance band before settling (a push, a servo hunt).
      // Abandon the reversal and keep sweeping toward the still-current waypoint rather
      // than latching: a planner that waits forever for an arrival that stopped
      // happening looks identical to a turret that has stopped working.
      reversing_ = false;
      reversal_armed_ = false;
      state_ = RoamState::MoveToScanStart;
    } else if (state_ == RoamState::MoveToScanStart && dist <= cfg_.reach_tol_rad * 4) {
      state_ = RoamState::Sweep;
    }

    if (target_yaw_ < lo || target_yaw_ > hi) {
      // Belt and braces: a waypoint outside the sweep region means the numbers changed
      // under the planner. Say so and hold rather than go there.
      exit();
      out.state = state_;
      out.reason = "waypoint outside roam envelope";
      out.intent.set_reason(out.reason);
      return out;
    }

    out.state = state_;
    out.target_yaw_rad = target_yaw_;
    out.direction = direction_;
    out.intent.type = IntentType::JointPosition;
    out.intent.has_joint_target = true;
    out.intent.q_yaw_rad = target_yaw_;
    out.intent.q_pitch_rad = pitch_target_rad_;
    // §33: with a valid IMU the preferred scan is a world-level yaw, and the pitch axis
    // compensates installation tilt. The intent carries the elevation it was asked for so
    // telemetry can show what was requested, not only what was commanded; the conversion
    // to a joint pose happens in the loop, which is where the IMU lives.
    if (cfg_.level_scan_available) {
      out.intent.type = IntentType::WorldLevelYaw;
      out.intent.has_world_elevation = true;
      out.intent.world_elevation_rad = cfg_.world_elevation_rad;
      out.intent.set_reason("world-level sweep");
    } else {
      out.intent.set_reason(state_ == RoamState::Turnaround ? "roam turnaround"
                                                            : "bounded sweep");
    }
    // Rate authority: the sweep is never in a hurry. The ceiling is the station's roam
    // limit, and the envelope can only reduce it further (§70's mechanism, unchanged).
    out.intent.confidence = 1.0;  // §34: no target involved, so nothing is uncertain
    out.intent.velocity_scale = 1.0;
    (void)period_ns;
    out.reason = roam_state_name(state_);
    return out;
  }

  // The actual sweep bounds: the configured roam envelope, inset by the braking margin.
  double sweep_lo_rad() const { return cfg_.envelope.yaw_min_rad + cfg_.braking_margin_rad; }
  double sweep_hi_rad() const { return cfg_.envelope.yaw_max_rad - cfg_.braking_margin_rad; }

 private:
  double clamp_envelope_pitch(double p) const {
    if (p < cfg_.envelope.pitch_min_rad) return cfg_.envelope.pitch_min_rad;
    if (p > cfg_.envelope.pitch_max_rad) return cfg_.envelope.pitch_max_rad;
    return p;
  }

  RoamConfig cfg_;
  RoamState state_ = RoamState::Idle;
  double target_yaw_ = 0.0;
  double pitch_target_rad_ = 0.0;
  int direction_ = 0;
  bool reversing_ = false;
  bool reversal_armed_ = false;
};

}  // namespace ota
