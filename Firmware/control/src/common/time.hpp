#pragma once
#include <ctime>

#include "types.hpp"

namespace ota {

// Monotonic nanoseconds (CLOCK_MONOTONIC). All control-path timestamps use
// this domain (architecture §7.3).
inline TimeNs now_monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<TimeNs>(ts.tv_sec) * 1000000000ll + ts.tv_nsec;
}

inline TimeNs ms_to_ns(double ms) { return static_cast<TimeNs>(ms * 1e6); }
inline double ns_to_ms(TimeNs ns) { return static_cast<double>(ns) / 1e6; }

}  // namespace ota
