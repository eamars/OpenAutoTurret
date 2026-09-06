#pragma once

#include "control/reference_limiter.hpp"

namespace ota::control {

// A moving observation is not a sequence of destinations to reach at the
// maximum braking speed. Track its position and explicit estimated velocity
// with a critically damped reference. In particular, do not differentiate
// frame-to-frame position corrections into another target velocity estimate.
inline double track_reference(ReferenceLimiter& st, double target, double target_velocity,
                              double dt, double v_max, double a_max, double j_max) {
  if (!st.initialised) st.reset_at(target);
  if (!(dt > 0 && dt < .1)) return st.q_rad;
  if (!(v_max > 0 && a_max > 0 && j_max > 0))
    return limit_reference(st, target, dt, v_max, a_max, j_max, target_velocity);
  constexpr double omega = 2.5;  // s^-1; critically damped before limit saturation
  st.target_v_rad_s = std::clamp(target_velocity, -v_max, v_max);
  double wanted = std::clamp(omega*omega*(target-st.q_rad) +
      2*omega*(st.target_v_rad_s-st.v_rad_s), -a_max, a_max);
  // Leave enough velocity headroom to release acceleration under the jerk
  // bound before reaching either speed limit.
  wanted = std::clamp(wanted,
      -std::sqrt(2*j_max*std::max(0.0, v_max+st.v_rad_s)),
       std::sqrt(2*j_max*std::max(0.0, v_max-st.v_rad_s)));
  st.a_rad_s2 += std::clamp(wanted-st.a_rad_s2, -j_max*dt, j_max*dt);
  const double previous_v = st.v_rad_s;
  st.v_rad_s = std::clamp(st.v_rad_s + st.a_rad_s2*dt, -v_max, v_max);
  st.q_rad += .5*(previous_v+st.v_rad_s)*dt;
  st.prev_target_rad = target;
  st.have_prev_target = true;
  return st.q_rad;
}

}  // namespace ota::control
