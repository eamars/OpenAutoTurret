// OpenAutoTurret — end-stop safety and dynamic braking (architecture §18).
//
// Two closely related pieces:
//
//   AxisLimits (§18.1)  — per-axis physical (hard) and logical (soft) limits,
//     in RAW mechanical radians. The hard limits are the estimated mechanical
//     stops; the soft limits sit inside them by a margin that covers homing
//     uncertainty, stop repeatability, control delay, braking margin and
//     mechanical tolerance. Before homing the limits are not `valid`; after
//     homing they are set from the measured endpoints + the configured margin.
//
//   SafetyEnvelope (§18.2/§18.3) — an INDEPENDENT checker (testable apart from
//     the normal trajectory generator, per §18.3) that each control cycle asks:
//     "can this axis still generate a valid full-stop trajectory before the
//     soft boundary?" The stop is derived from the SAME jerk-limited model the
//     planner uses (§18.2), not only the constant-deceleration equation, so the
//     checker agrees with what the planner can actually achieve.
//
// Safety layers (§18.3):
//   Layer 1 — the trajectory planner obeys the limits normally;
//   Layer 2 — this envelope checks predicted stop feasibility;
//   Layer 3 — an imminent boundary violation forces a controlled max-safe stop
//             (the caller commands emergency_stop_target() with a low speed);
//   Layer 4 — a motor/CAN hard fault may require disabling the motor.
#pragma once

#include <cmath>

#include "common/types.hpp"

namespace ota {

// §18.1 per-axis limits, raw mechanical radians.
struct AxisLimits {
  bool valid = false;  // false until homing measures the hard limits.
  double q_hard_min_rad = 0.0;
  double q_hard_max_rad = 0.0;
  double q_soft_min_rad = 0.0;
  double q_soft_max_rad = 0.0;

  // Set from the two measured raw endpoints (the homing stops) plus the soft
  // margin. `raw_low_rad` must be < `raw_high_rad`.
  void set_from_endpoints(double raw_low_rad, double raw_high_rad,
                          double soft_margin_rad) {
    q_hard_min_rad = raw_low_rad;
    q_hard_max_rad = raw_high_rad;
    q_soft_min_rad = raw_low_rad + soft_margin_rad;
    q_soft_max_rad = raw_high_rad - soft_margin_rad;
    valid = (q_soft_min_rad < q_soft_max_rad);
  }

  bool in_soft(double q_rad) const {
    return valid && q_rad >= q_soft_min_rad && q_rad <= q_soft_max_rad;
  }
  bool in_hard(double q_rad) const {
    return valid && q_rad >= q_hard_min_rad && q_rad <= q_hard_max_rad;
  }
  // Distance (>= 0 when inside) to the nearer soft boundary.
  double distance_to_soft(double q_rad) const {
    if (!valid) return 0.0;
    return std::min(q_rad - q_soft_min_rad, q_soft_max_rad - q_rad);
  }
};

// Braking model parameters (raw radians / s / s^2 / s^3).
struct SafetyEnvelopeParams {
  double a_brake_rad_s2 = 60.0 * kDeg2Rad;   // braking acceleration
  double j_brake_rad_s3 = 300.0 * kDeg2Rad;  // braking jerk
  double margin_rad = 0.05;                  // extra distance margin (safety)
  double v_max_rad_s = 30.0 * kDeg2Rad;      // fallback speed when no limits set
};

class SafetyEnvelope {
 public:
  SafetyEnvelope() = default;
  explicit SafetyEnvelope(SafetyEnvelopeParams p) : p_(p) {}

  // Distance needed to come to a full stop from speed |v| under the braking
  // model. Uses the same jerk-limited form the planner derives
  // (d_stop = v^2/(2a) + v a/(2j)) plus a small margin, so it is consistent
  // with (and conservatively >=) what the planner can actually achieve.
  double stop_distance(double v_rad_s) const {
    const double s = std::fabs(v_rad_s);
    return s * s / (2.0 * p_.a_brake_rad_s2) +
           s * p_.a_brake_rad_s2 / (2.0 * p_.j_brake_rad_s3) + p_.margin_rad;
  }

  // Layer 2: can the axis still stop before the nearer soft boundary?
  // When limits are not valid there is nothing to check (returns true).
  bool stop_feasible(double q_rad, double v_rad_s, const AxisLimits& lim) const {
    if (!lim.valid) return true;
    const double d_stop = stop_distance(v_rad_s);
    const double d_avail = (v_rad_s > 0.0) ? (lim.q_soft_max_rad - q_rad)
                        : (v_rad_s < 0.0) ? (q_rad - lim.q_soft_min_rad)
                        : lim.distance_to_soft(q_rad);
    return d_stop <= d_avail + 1e-9;
  }

  // Layer 1: the largest SAFE speed at position q — the largest v for which a
  // full stop (INCLUDING the safety margin) still fits before the nearer soft
  // boundary. So stop_distance(max_speed_at(q)) == distance_to_soft(q) and
  // stop_feasible(q, max_speed_at(q)) holds. 0 when within the margin of a
  // boundary (the axis must be stopped there); v_max when limits are not valid.
  double max_speed_at(double q_rad, const AxisLimits& lim) const {
    if (!lim.valid) return p_.v_max_rad_s;
    const double budget = lim.distance_to_soft(q_rad) - p_.margin_rad;
    if (budget <= 0.0) return 0.0;
    const double a = p_.a_brake_rad_s2, j = p_.j_brake_rad_s3;
    const double b = a / (2.0 * j);
    const double v = a * (-b + std::sqrt(b * b + 2.0 * budget / a));
    return std::max(0.0, v);
  }

  // Layer 1: clamp a requested reference position to the soft limits.
  double constrain_reference(double q_ref_rad, const AxisLimits& lim) const {
    if (!lim.valid) return q_ref_rad;
    return std::max(lim.q_soft_min_rad, std::min(lim.q_soft_max_rad, q_ref_rad));
  }

  // Layer 3: the position to command for a controlled maximum-safe stop from
  // (q, v): where the axis would land under the braking model, clamped to the
  // soft limits. The caller commands this as the reference with a low speed
  // limit to bring the axis to rest before the boundary.
  double emergency_stop_target(double q_rad, double v_rad_s,
                               const AxisLimits& lim) const {
    const double target = q_rad + std::copysign(stop_distance(v_rad_s), v_rad_s);
    return constrain_reference(target, lim);
  }

 private:
  SafetyEnvelopeParams p_;
};

}  // namespace ota
