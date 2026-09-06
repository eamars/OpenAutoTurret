#pragma once
#include <algorithm>
#include <cmath>
#include "common/types.hpp"

namespace ota::control {
// Host position correction plus trajectory velocity feed-forward. There is no
// host integral. Quiet hold has hysteresis above the measured encoder noise.
struct SpeedServo {
  double velocity = 0, acceleration = 0;
  bool quiet = false;
  void reset() { velocity = acceleration = 0; quiet = false; }
  double step(double reference, double feed_forward, double measured,
              double cap, double dt, double a_max, double j_max,
              double negative_acceleration_scale=1, double positive_acceleration_scale=1) {
    if (!(dt > 0 && dt < .1) || cap <= 0) { reset(); return 0; }
    const double error = reference - measured;
    const bool still_reference = std::abs(feed_forward) < .02 * kDeg2Rad;
    if (!still_reference || std::abs(error) > .15*kDeg2Rad) quiet = false;
    else if (std::abs(error) < .08*kDeg2Rad) quiet = true;
    const double desired = quiet ? 0.0 : std::clamp(
        feed_forward + 3.0 * std::clamp(error, -2*kDeg2Rad, 2*kDeg2Rad), -cap, cap);
    // Reduce only acceleration that builds outward speed. Never reduce the
    // acceleration available to brake motion already heading toward a stop.
    const double lo=velocity<=0 ? a_max*std::clamp(negative_acceleration_scale,0.0,1.0) : a_max;
    const double hi=velocity>=0 ? a_max*std::clamp(positive_acceleration_scale,0.0,1.0) : a_max;
    const double desired_a = std::clamp((desired-velocity)/dt, -lo, hi);
    acceleration += std::clamp(desired_a-acceleration, -j_max*dt, j_max*dt);
    const double next = velocity + acceleration*dt;
    if ((desired-velocity)*(desired-next) <= 0) { velocity=desired; acceleration=0; }
    else velocity=std::clamp(next, -cap, cap);
    return velocity;
  }
};
}
