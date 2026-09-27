#pragma once
#include <cmath>

namespace ota::can {
// Owner's installed-pitch ceiling. Configuration may lower, never raise it.
inline constexpr double kPitchCurrentCeilingA = 5.0;
inline bool valid_pitch_current_limit(double amps) {
  return std::isfinite(amps) && amps > 0 && amps <= kPitchCurrentCeilingA;
}
}  // namespace ota::can
