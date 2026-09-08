#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include "calibration/homing_controller.hpp"
#include "control/motor_backend.hpp"

namespace ota {
// Bounds encoder evidence on BOTH axes, including asynchronous drive setup.
// These are abort thresholds, not certified braking distances or load ratings.
class HomingMotionGuard {
 public:
  void expect(AxisId axis, const DesiredState& ds, double q) {
    auto& s = axes_[static_cast<size_t>(axis)];
    const bool moving = !ds.hold && (ds.position_move || ds.velocity_rad_s != 0);
    const int dir = !moving ? 0 : ds.target_rad > q ? 1 : -1;
    // Retain the origin throughout a hold or unchanged move; refreshing it on
    // every tick would allow a slow load-dependent drift to accumulate forever.
    if (!s.expected || moving != s.moving || (moving &&
        (ds.target_rad != s.target || ds.position_move != s.position))) {
      s.origin = s.extreme = q;
      s.target = moving ? ds.target_rad : q;
      s.direction = dir;
      s.position = ds.position_move;
      s.moving = moving;
      s.expected = true;
    }
  }
  std::string observe(AxisId axis, const AxisSnapshot& fb, TimeNs now,
                      double speed_cap, TimeNs max_age, double max_temp) {
    auto& s = axes_[static_cast<size_t>(axis)];
    const auto fail = [axis](const char* why) { return std::string(axis_name(axis))+" homing: "+why; };
    // A concurrently received frame can be newer than the cycle start by a
    // fraction of one tick. Larger future timestamps remain invalid.
    if (!fb.has_feedback || fb.rx_ns < 0 || fb.rx_ns > now+5'000'000 ||
        now-fb.rx_ns > max_age || (s.sampled && fb.rx_ns < s.latest_ns))
      return fail("missing, stale or regressing feedback");
    if (!std::isfinite(fb.q_rad) || !std::isfinite(fb.torque_nm) ||
        !std::isfinite(fb.temp_c) || fb.faults || fb.temp_c > max_temp)
      return fail("invalid encoder or unhealthy drive");
    if (!s.expected) { DesiredState hold; hold.hold=true; expect(axis,hold,fb.q_rad); }
    if (s.sampled && fb.rx_ns > s.window_ns) {
      const double dt = (fb.rx_ns-s.window_ns)*1e-9;
      // Position quantization allowance; do not trust the noisy drive velocity.
      if (std::abs(fb.q_rad-s.window_q) > speed_cap*dt + .05*kDeg2Rad)
        return fail("encoder motion exceeds homing speed ceiling");
    }
    if (!s.sampled || fb.rx_ns-s.window_ns >= 50'000'000) {
      s.window_ns=fb.rx_ns; s.window_q=fb.q_rad; s.sampled=true;
    }
    s.latest_ns=fb.rx_ns;
    constexpr double tolerance = .5*kDeg2Rad;
    const double low=std::min(s.origin,s.target)-tolerance;
    const double high=std::max(s.origin,s.target)+tolerance;
    if (fb.q_rad < low || fb.q_rad > high)
      return fail("encoder left commanded corridor or stationary hold");
    if (s.moving && !s.position) {
      if ((fb.q_rad-s.extreme)*s.direction < -tolerance)
        return fail("encoder reversed against approach command");
      if ((fb.q_rad-s.extreme)*s.direction > 0) s.extreme=fb.q_rad;
    }
    return {};
  }
 private:
  struct Axis {
    bool expected=false, sampled=false, moving=false, position=false;
    int direction=0;
    double origin=0, target=0, extreme=0, window_q=0;
    TimeNs window_ns=0, latest_ns=0;
  };
  std::array<Axis,kAxisCount> axes_{};
};
} // namespace ota
