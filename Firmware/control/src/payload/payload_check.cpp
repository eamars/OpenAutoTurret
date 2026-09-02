// OpenAutoTurret — in-loop payload response check implementation (§31.3).
#include "payload/payload_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>   // snprintf for the fail message


namespace ota {
namespace payload {

double PayloadCheck::clamp_to_region(double q) const {
  const double lo = region_center_ - region_half_span_;
  const double hi = region_center_ + region_half_span_;
  return std::max(lo, std::min(hi, q));
}

bool PayloadCheck::begin(TimeNs now_ns, double q_start_rad, bool has_feedback) {
  if (failed_) return false;
  if (!has_feedback) {
    fail("no axis feedback at check start");
    return false;
  }
  const double lo = region_center_ - region_half_span_;
  const double hi = region_center_ + region_half_span_;
  if (q_start_rad < lo || q_start_rad > hi) {
    // With numbers: this fires when the region was clamped to (or past) zero by
    // the soft limits, i.e. the station is holding too close to a travel stop.
    // Without them the operator sees a verdict and no way to act on it.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "start pose %.2f deg outside the safe central region "
                  "[%.2f, %.2f] deg (half span %.2f deg; a region clamped near "
                  "zero means the station is holding too close to a travel "
                  "stop - retry from the ready pose)",
                  q_start_rad * 180.0 / M_PI, lo * 180.0 / M_PI,
                  hi * 180.0 / M_PI, region_half_span_ * 180.0 / M_PI);
    fail(buf);
    return false;
  }
  q_start_ = q_start_rad;
  // Conservative: never use more than the requested amplitude, and keep a
  // margin from the region edge.
  const double margin = 2.0 * kDeg2Rad;
  amp_ = std::min(cfg_.step_amplitude_rad,
                  std::min(q_start_rad - lo, hi - q_start_rad) - margin);
  if (amp_ < 0.25 * kDeg2Rad) {
    fail("safe central region too small for a conservative check move");
    return false;
  }
  samples_.clear();
  step_pos_ = StepMetrics{};
  step_neg_ = StepMetrics{};
  axis_result_ = AxisVerifyResult{};
  move_ = CheckMove::PosStep;
  q_target_ = clamp_to_region(q_start_ + amp_);
  move_start_ns_ = now_ns;
  settled_since_ns_ = -1;
  active_ = true;
  return true;
}

bool PayloadCheck::move_settled(const AxisSnapshot& snap) const {
  const double band = cfg_.settle_band_frac * amp_;
  return std::fabs(snap.q_rad - q_target_) <= band &&
         std::fabs(snap.v_rad_s) <= cfg_.at_rest_vel_rad_s;
}

void PayloadCheck::advance_move(TimeNs now_ns) {
  switch (move_) {
    case CheckMove::PosStep:
      step_pos_ = analyze_step(samples_, q_start_, q_start_ + amp_,
                               cfg_.settle_band_frac);
      move_ = CheckMove::PosReturn;
      q_target_ = q_start_;
      break;
    case CheckMove::PosReturn:
      move_ = CheckMove::NegStep;
      q_target_ = clamp_to_region(q_start_ - amp_);
      break;
    case CheckMove::NegStep:
      step_neg_ = analyze_step(samples_, q_start_, q_start_ - amp_,
                               cfg_.settle_band_frac);
      move_ = CheckMove::NegReturn;
      q_target_ = q_start_;
      break;
    case CheckMove::NegReturn:
    case CheckMove::Done:
      move_ = CheckMove::Done;
      active_ = false;
      break;
  }
  samples_.clear();
  move_start_ns_ = now_ns;
  settled_since_ns_ = -1;
}

PayloadCheck::StepOut PayloadCheck::step(TimeNs now_ns,
                                         const AxisSnapshot& snap) {
  StepOut out;
  if (!active_) {
    out.hold = true;
    out.q_ref_rad = snap.q_rad;
    out.limit_spd_rad_s = 0.0;
    return out;
  }
  if (!snap.has_feedback) {
    fail("axis feedback lost during the payload check");
    return out;
  }
  out.q_ref_rad = q_target_;
  out.limit_spd_rad_s = cfg_.speed_rad_s;

  samples_.push_back({now_ns, snap.q_rad, snap.v_rad_s, snap.torque_nm});

  if (move_settled(snap)) {
    if (settled_since_ns_ < 0) settled_since_ns_ = now_ns;
    if ((now_ns - settled_since_ns_) / 1e9 >= cfg_.settle_time_s) {
      advance_move(now_ns);
      if (move_ == CheckMove::Done) {
        out.hold = true;
        out.q_ref_rad = snap.q_rad;
        out.limit_spd_rad_s = 0.0;
      } else {
        out.q_ref_rad = q_target_;
        out.limit_spd_rad_s = cfg_.speed_rad_s;
      }
    }
  } else {
    settled_since_ns_ = -1;
    if ((now_ns - move_start_ns_) / 1e9 > cfg_.max_move_s) {
      fail(std::string("move timeout in segment ") +
           (move_ == CheckMove::PosStep ? "pos_step"
            : move_ == CheckMove::PosReturn ? "pos_return"
            : move_ == CheckMove::NegStep ? "neg_step" : "neg_return"));
    }
  }
  if (!active_ && !failed_) {
    // Compare against the stored baseline (or flag NoProfile).
    if (profile_ != nullptr) {
      axis_result_ = compare_axis(profile_->axis(axis_), step_pos_, step_neg_,
                                  cfg_.tol);
    } else {
      axis_result_.measured = step_pos_.valid || step_neg_.valid;
      axis_result_.step_pos = step_pos_;
      axis_result_.step_neg = step_neg_;
      axis_result_.ok = true;  // nothing to compare against
    }
  }
  return out;
}

}  // namespace payload
}  // namespace ota
