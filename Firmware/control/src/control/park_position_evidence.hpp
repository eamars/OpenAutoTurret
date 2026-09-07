#pragma once
#include "common/types.hpp"

namespace ota {
// A separately acquired measurement of OUTPUT position, in calibrated raw
// coordinates. Re-reading the drive's encoder is not independent evidence.
// Production backends must leave this unavailable until a sensor is wired and
// its calibration, uncertainty and acquisition timestamp have been validated.
struct ParkPositionEvidence {
  bool trusted = false;
  bool simulated = false;
  TimeNs sampled_ns = 0;
  double q_raw_rad = 0;
  double uncertainty_rad = 0;
};
}  // namespace ota
