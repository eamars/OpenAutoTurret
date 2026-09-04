#pragma once
//
// Rate-proportional position lead (drive-mode item 2: measured following error).
//
// WHY THIS EXISTS
// Normal motion commands the drive a POSITION and a speed limit (`control_loop.cpp:1085`,
// `backend_->command(a, qr, ls)`), so the CyberGear closes its own position loop and controld cannot reach that loop's
// gain — the only position-gain-looking register in this codebase is `Reg::MechPos`, which is a read-back, not a gain.
// What was measured instead (PROGRESS 2026-09-06 rounds 18-19, real station): while sweeping at 10.00 deg/s the axis
// trails its commanded position by **p50 3.628 deg, p95 4.274 deg**, the sign of the error follows the direction of
// travel (733 of 740 samples), it is the same size whether the reference stepped this cycle or not (3.631 vs 3.626),
// and it is zero at rest. That is a drive-side position-mode error under load, and controld's only honest lever is to
// ask for a position that already accounts for it.
//
// WHAT THIS IS: a feed-forward term on the commanded position. Ask for where the reference will be in
// `lead_s` seconds, not where it is now, so the drive's own lag stops being a constant shortfall.
//
// WHAT THIS MUST NEVER DO, and the two properties that enforce it:
//   1. **No lead at zero rate.** A stationary turret must be commanded exactly the reference. Lead is proportional to
//      the reference's own rate, so a hold, a park, and every idle cycle are bit-identical to today. This is also what
//      keeps it from creeping past a target it has stopped on.
//   2. **A lead can never command beyond a soft limit.** The result is clamped into [q_soft_min_rad, q_soft_max_rad].
//      Soft limits are a safety boundary and §111.18 gives the envelope the final word on position; a "smoothness"
//      feature must not be the reason that stops being true. Clamping here is belt-and-braces for the same reason the
//      envelope checks again — the outer check stays authoritative and unchanged.
//
// DEFAULT OFF: `lead_s == 0` returns the command untouched, so a station that does not ask for this behaves exactly as
// it did before this file existed. Sizing starts from the measurement: a 0.173-0.36 s lead at 10 deg/s is 1.7-3.6 deg.

#include <algorithm>

namespace ota {

// `rate_est_rad_s` is the caller's estimate of how fast the reference itself is moving, sign included, already capped
// by the commanded speed limit so a reference STEP (a mode change, a re-seed) cannot manufacture a huge lead.
inline double apply_position_lead(double cmd_rad, double rate_est_rad_s, double lead_s,
                                  double soft_min_rad, double soft_max_rad) {
  if (lead_s <= 0.0) return cmd_rad;  // disabled: exactly today's behaviour
  if (rate_est_rad_s == 0.0) return cmd_rad;  // property 1: nothing to lead at a standstill
  const double leaded = cmd_rad + lead_s * rate_est_rad_s;
  // property 2: forward is never beyond the soft limit. std::clamp needs min<=max, which the envelope guarantees;
  // guarding anyway because a mis-loaded limit pair must not become an exception on the control thread.
  if (soft_min_rad <= soft_max_rad)
    return std::clamp(leaded, soft_min_rad, soft_max_rad);
  return cmd_rad;
}

}  // namespace ota
