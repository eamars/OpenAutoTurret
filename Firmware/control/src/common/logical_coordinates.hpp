// OpenAutoTurret — host logical joint coordinates (architecture §24).
//
// After endpoints are measured, logical coordinates are defined independently
// of the motor's volatile zero:
//
//   q_logical = direction_sign * (q_raw - q_raw_reference) + q_reference_logical
//
// `q_raw` is the motor's raw position readout (the feedback angle, ±12.5 rad);
// `q_logical` is the host-logical position in degrees. The reference (raw
// position + its logical value) is established during homing and persisted in
// the calibration file (§28/§41); reboot safety does not depend on the motor's
// own zero command (§24).
#pragma once

#include <cmath>

namespace ota {

constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Affine raw<->logical model for one axis. All logical values are degrees, all
// raw values are radians (feedback-angle space). The mapping is identity-safe:
// before a reference is established (has_reference == false) the model is a no-op
// pass-through (raw deg == logical deg) so callers can still reason about it.
struct AxisLogicalModel {
  int direction_sign = 1;              // logical -> mechanical sense (+1 or -1)
  double q_raw_reference_rad = 0.0;    // raw position at the home reference
  double q_reference_logical_deg = 0.0;// logical value at the home reference
  bool has_reference = false;          // true once homing has set the reference

  void set_reference(double raw_ref_rad, double logical_ref_deg) {
    q_raw_reference_rad = raw_ref_rad;
    q_reference_logical_deg = logical_ref_deg;
    has_reference = true;
  }

  // raw (rad, feedback angle) -> logical (deg).
  double raw_to_logical_deg(double q_raw_rad) const {
    if (!has_reference) return q_raw_rad * kRad2Deg;
    return direction_sign * (q_raw_rad - q_raw_reference_rad) * kRad2Deg +
           q_reference_logical_deg;
  }

  // logical (deg) -> raw (rad, feedback angle).
  double logical_to_raw_rad(double q_logical_deg) const {
    if (!has_reference) return q_logical_deg * kDeg2Rad;
    // direction_sign is ±1, so 1/sign == sign.
    return q_raw_reference_rad +
           direction_sign * (q_logical_deg - q_reference_logical_deg) * kDeg2Rad;
  }

  // Convenience: raw (rad) -> logical (rad).
  double raw_to_logical_rad(double q_raw_rad) const {
    return raw_to_logical_deg(q_raw_rad) * kDeg2Rad;
  }
};

// Set the model up from the two measured raw endpoints (rad). The low endpoint
// becomes logical 0 and the high endpoint becomes logical +travel, choosing the
// sense so that logical travel is positive. Returns the measured travel (deg).
inline double setup_model_from_endpoints(AxisLogicalModel& m, double raw_low_rad,
                                         double raw_high_rad) {
  const double span_rad = raw_high_rad - raw_low_rad;
  m.direction_sign = (span_rad >= 0.0) ? 1 : -1;
  m.set_reference(raw_low_rad, 0.0);
  return std::fabs(span_rad) * kRad2Deg;
}

}  // namespace ota
