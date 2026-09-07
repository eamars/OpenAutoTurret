#pragma once
#include <algorithm>
#include <array>
#include "common/types.hpp"

namespace ota::control {

// Operator configuration is in degrees; resolved control values are radians.
// These service ceilings preserve the existing commissioned command envelope.
inline constexpr double kServiceSpeed = 20 * kDeg2Rad;
inline constexpr double kServiceAcceleration = 30 * kDeg2Rad;
inline constexpr double kServiceJerk = 120 * kDeg2Rad;

struct MotionRates {
  double speed = kServiceSpeed;
  double acceleration = kServiceAcceleration;
  double jerk = kServiceJerk;
};
struct MotionProfile {
  MotionRates maximum;
  MotionRates target{10*kDeg2Rad, 15*kDeg2Rad, 60*kDeg2Rad};
};
struct MotionConfig {
  bool configured = false;  // absent block retains the legacy configuration path
  // Manual, AutoTrack, AutoRoam; each contains pitch then yaw.
  std::array<std::array<MotionProfile, kAxisCount>, 3> modes{};
  std::array<MotionRates, kAxisCount> axis_maximum{};
};

inline MotionRates intersect(MotionRates a, const MotionRates& b) {
  return {std::min(a.speed,b.speed), std::min(a.acceleration,b.acceleration),
          std::min(a.jerk,b.jerk)};
}

// Shared by reference generation and the final servo. Payload caps remain
// per axis; a slower pitch does not silently throttle yaw.
inline MotionProfile resolve_motion(MotionProfile p, const MotionRates& axis,
                                    const MotionRates& payload, double derate = 1) {
  p.maximum = intersect(intersect(p.maximum, axis), payload);
  p.maximum.speed *= std::clamp(derate,0.0,1.0);
  // Derating propulsion must not invent a weaker braking capability. The
  // boundary governor uses the unscaled, capped maximum acceleration/jerk.
  p.target = intersect(p.target, p.maximum);
  p.target.acceleration *= std::clamp(derate,0.0,1.0);
  return p;
}
}  // namespace ota::control
