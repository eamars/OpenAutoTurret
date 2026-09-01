// OpenAutoTurret — full-axis homing sequence (architecture §23).
#include "calibration/full_axis_homing.hpp"

#include <algorithm>
#include <cmath>

namespace ota {

namespace {
// Endpoint B starts with its coarse approach pointing AWAY from endpoint
// A's stop, at the moment the drive's velocity-loop integral is still
// wound up pushing that stop (endpoint A's final fine contact + dwell).
// Reset the drive's velocity controller at endpoint B's start so the
// approach begins clean (rehome3 root cause, 2026-09-02).
HomingParams with_start_rearm(HomingParams p) {
  p.rearm_before_start = true;
  return p;
}
}  // namespace

FullAxisHoming::FullAxisHoming(AxisId axis, FullAxisHomingParams p)
    : axis_(axis),
      p_(p),
      home_a_(axis, p.dir_endpoint_a, p.homing),
      home_b_(axis, p.dir_endpoint_b, with_start_rearm(p.homing)) {}

DesiredState FullAxisHoming::step(const HomingFeedback& fb) {
  switch (phase_) {
    case FullAxisPhase::HomeA: {
      const DesiredState ds = home_a_.step(fb);
      if (home_a_.terminal()) {
        if (home_a_.result().valid) {
          // Endpoint A is validated. The axis now sits at A, which is on the
          // near side of endpoint B, so homing B naturally performs the
          // "traverse safely" step of §23 as it approaches across the axis.
          result_.endpoint_a_rad = home_a_.result().fine_contact_rad;
          result_.repeatability_rad = home_a_.result().repeatability_rad;
          phase_ = FullAxisPhase::HomeB;
          DesiredState hold;
          hold.hold = true;
          hold.message = "endpoint A homed; starting endpoint B";
          return hold;
        }
        fail("endpoint A homing failed: " + home_a_.result().fail_reason);
      }
      return ds;
    }
    case FullAxisPhase::HomeB: {
      const DesiredState ds = home_b_.step(fb);
      if (home_b_.terminal()) {
        if (home_b_.result().valid) {
          result_.endpoint_b_rad = home_b_.result().fine_contact_rad;
          result_.repeatability_rad =
              std::max(result_.repeatability_rad,
                       home_b_.result().repeatability_rad);
          validate();
        } else {
          fail("endpoint B homing failed: " + home_b_.result().fail_reason);
        }
      }
      return ds;
    }
    case FullAxisPhase::Complete: {
      DesiredState hold;
      hold.hold = true;
      hold.message = "full-axis homing complete";
      return hold;
    }
    case FullAxisPhase::Failed: {
      DesiredState hold;
      hold.hold = true;
      hold.message = "full-axis homing failed";
      return hold;
    }
  }
  DesiredState hold;
  hold.hold = true;
  hold.message = "idle";
  return hold;
}

void FullAxisHoming::validate() {
  const double raw_low =
      std::min(result_.endpoint_a_rad, result_.endpoint_b_rad);
  const double raw_high =
      std::max(result_.endpoint_a_rad, result_.endpoint_b_rad);
  // Set up the host logical model (§24): the low endpoint becomes logical 0
  // and the travel is positive. This also yields the measured span in degrees.
  result_.measured_travel_deg =
      setup_model_from_endpoints(result_.model, raw_low, raw_high);

  if (result_.measured_travel_deg >= p_.expected_travel_min_deg &&
      result_.measured_travel_deg <= p_.expected_travel_max_deg) {
    result_.complete = true;
    result_.valid = true;
    phase_ = FullAxisPhase::Complete;
  } else {
    fail("measured travel " + std::to_string(result_.measured_travel_deg) +
         " deg outside expected [" +
         std::to_string(p_.expected_travel_min_deg) + ", " +
         std::to_string(p_.expected_travel_max_deg) + "]");
  }
}

void FullAxisHoming::fail(const std::string& reason) {
  result_.complete = true;
  result_.valid = false;
  result_.fail_reason = reason;
  phase_ = FullAxisPhase::Failed;
}

}  // namespace ota
