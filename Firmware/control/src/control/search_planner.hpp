// OpenAutoTurret — search / free-roaming planner (architecture §36, §49).
//
// Search is optional and DISABLED by default unless configured (§36). It uses
// the exact same trajectory/safety path as tracking: this planner only produces
// the desired joint target + a reduced speed limit, and the trajectory
// generator / safety envelope enforce the smooth S-curve turnarounds and the
// coupled collision envelope.
//
// Pattern (§49): sweep the yaw axis between two bounds that stay STRICTLY
// inside the tracking soft limits (so tracking still has braking margin if a
// target is acquired mid-search), dwelling briefly and reversing smoothly at
// each bound. Elevation is bounded at a single configured level for v1.
//
// Pure planning — no CAN, no camera, no motor driver.
#pragma once

#include <cmath>

#include "common/types.hpp"

namespace ota {

struct SearchPlannerConfig {
  // Yaw sweep bounds (raw rad). MUST be strictly inside the tracking soft
  // limits so tracking keeps braking margin (verified by the caller).
  double yaw_low_rad = -45.0 * kDeg2Rad;
  double yaw_high_rad = 45.0 * kDeg2Rad;
  // Bounded elevation (raw rad) held constant during the sweep.
  double pitch_rad = 0.0;
  // Reduced motion limits relative to tracking (§36).
  double v_max_rad_s = 10.0 * kDeg2Rad;
  // Dwell at each turnaround before reversing (seconds).
  double dwell_s = 0.25;
};

class SearchPlanner {
 public:
  explicit SearchPlanner(SearchPlannerConfig cfg) : cfg_(cfg) {}

  struct Output {
    double q_yaw_rad = 0.0;
    double q_pitch_rad = 0.0;
    double v_max_rad_s = 0.0;
    // True while dwelling at a turnaround (the caller can treat this as
    // "nearly at rest").
    bool dwelling = false;
  };

  // Begin a sweep from the current yaw (so the first move is toward the nearer
  // bound, not a jump).
  void start(double q_yaw_start_rad) {
    started_ = true;
    // Move toward whichever bound is farther (a full first sweep).
    const double d_low = std::fabs(q_yaw_start_rad - cfg_.yaw_low_rad);
    const double d_high = std::fabs(cfg_.yaw_high_rad - q_yaw_start_rad);
    moving_high_ = d_high >= d_low;
    dwell_until_ns_ = 0;
  }

  bool started() const { return started_; }

  // Step the planner at time now_ns given the current yaw. Returns the desired
  // joint target + reduced speed. At a bound, it dwells briefly, then reverses
  // (the trajectory generator produces the smooth S-curve turnaround).
  Output step(TimeNs now_ns, double q_yaw_now_rad) {
    Output out;
    out.q_pitch_rad = cfg_.pitch_rad;
    out.v_max_rad_s = cfg_.v_max_rad_s;
    if (!started_) start(q_yaw_now_rad);
    out.q_yaw_rad = moving_high_ ? cfg_.yaw_high_rad : cfg_.yaw_low_rad;

    if (now_ns < dwell_until_ns_) {
      out.dwelling = true;
      return out;
    }
    const double err = std::fabs(q_yaw_now_rad - out.q_yaw_rad);
    if (err < kSearchPosTolRad) {
      // Reached the bound: dwell, then reverse.
      dwell_until_ns_ = now_ns + static_cast<TimeNs>(cfg_.dwell_s * 1e9);
      moving_high_ = !moving_high_;
      out.dwelling = true;
    }
    return out;
  }

  const SearchPlannerConfig& config() const { return cfg_; }

 private:
  static constexpr double kSearchPosTolRad = 0.02;  // ~1.1 deg
  SearchPlannerConfig cfg_;
  bool started_ = false;
  bool moving_high_ = true;
  TimeNs dwell_until_ns_ = 0;
};

}  // namespace ota
