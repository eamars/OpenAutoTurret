// Rate/acceleration limiter for the reference position (requirement (b): "smooth").
//
// Why this file exists. The reference handed to the axes used to be the resolver's answer for this
// cycle's line of sight, position-clamped and nothing else: when the estimated target direction
// moved, the reference moved by the same amount in the same instant. A velocity ceiling was still
// handed to the drive, so the axis did obey it - which is exactly the problem. A reference that
// steps while the drive is capped is a step response, not a profile. Measured on this station at
// 99 Hz: the reference reached 105 deg/s against its own 30 deg/s limit, its median acceleration was
// 106 deg/s^2 against a configured 60, and the axis spent 5% of a dart run more than 15 deg behind
// its reference - 432 px of image travel on a 69 deg frame. Neither component was behaving
// incorrectly. The profile between them was missing.
//
// So the reference is now shaped by the limits the operator already configured, not by invented
// ones: the intent's own v_max (30 deg/s while tracking, less when derated, less again near a
// boundary) and the axis's configured maximum acceleration. Limiting velocity alone would keep
// demanding infinite acceleration at the start of every move, so velocity changes are ramped by
// a_max, which bounds jerk implicitly - the result is the near-trapezoid an operator expects to
// feel rather than a sequence of edges.
//
// Deliberately a free function over a small state struct: the properties worth trusting (what it
// does to a step, to a lost speed authority, whether it lands exactly) are arithmetic, and proving
// them should not require a motor, a bus, or a station.
#ifndef OTA_CONTROL_REFERENCE_LIMITER_HPP
#define OTA_CONTROL_REFERENCE_LIMITER_HPP

#include <algorithm>
#include <cmath>

namespace ota::control {

// The remembered half-cycle: where the reference stands and the speed it carries into the next
// period. `initialised` instead of a sentinel, because 0 rad is a legitimate reference.
struct ReferenceLimiter {
  double q_rad = 0.0;
  double v_rad_s = 0.0;
  bool initialised = false;
  // The target's own velocity, lightly smoothed. A re-resolved line of sight is noisy frame to
  // frame, and an unsmoothed derivative of that noise would be fed straight into the reference -
  // which is how a limiter meant to remove jitter would become its source.
  double target_v_rad_s = 0.0;
  double prev_target_rad = 0.0;
  bool have_prev_target = false;

  // Re-seat the profile at an actual pose - used while the axes stand still, so the first cycle of
  // a move starts from where the hardware is rather than where the last engagement ended.
  void reset_at(double q0_rad) {
    q_rad = q0_rad;
    v_rad_s = 0.0;
    target_v_rad_s = 0.0;
    have_prev_target = false;
    initialised = true;
  }
};

// Advance one control period toward `target_rad`, never exceeding `v_max_rad_s` or `a_max_rad_s2`,
// and return the reference to publish this cycle.
//
// The wanted velocity is not "close the error in one cycle" - that is what makes a step response
// step. It is the speed from which the profile can still brake to exactly the target at a_max,
// capped at v_max. That single choice is what removes the overshoot, and overshoot is what
// requirement (b) calls oscillation. Position is then integrated from the velocity that survived
// the a_max ramp; capping position and velocity independently instead would let the position run
// away from the speed the profile claims to have.
inline double limit_reference(ReferenceLimiter& st, double target_rad, double dt_s,
                              double v_max_rad_s, double a_max_rad_s2) {
  if (!st.initialised) {
    st.reset_at(target_rad);
    return st.q_rad;
  }
  if (!(dt_s > 0.0)) return st.q_rad;  // a repeated or stalled timestamp must not integrate

  // What the target is doing, measured over the same period the reference is advanced over.
  {
    const double k_target_v = 0.25;
    if (st.have_prev_target) {
      const double raw = (target_rad - st.prev_target_rad) / dt_s;
      st.target_v_rad_s += k_target_v * (raw - st.target_v_rad_s);
    }
    st.prev_target_rad = target_rad;
    st.have_prev_target = true;
  }

  if (!(v_max_rad_s > 0.0)) {
    // No speed authority this cycle (a safety action, or an envelope that leaves no room). Bleed the
    // carried speed off at a_max rather than teleporting onto the target or pressing on regardless.
    const double dv = (a_max_rad_s2 > 0.0) ? a_max_rad_s2 * dt_s : std::abs(st.v_rad_s);
    st.v_rad_s -= std::copysign(std::min(dv, std::abs(st.v_rad_s)), st.v_rad_s);
    st.q_rad += st.v_rad_s * dt_s;
    return st.q_rad;
  }

  const double err = target_rad - st.q_rad;
  const double dir = (err >= 0.0) ? 1.0 : -1.0;
  const double u_target = st.target_v_rad_s * dir;  // target's speed, along the error direction

  // The wanted speed is the target's own speed plus whatever can still be braked away before
  // arriving - NOT "close the error in one cycle", which is what made a step response step, and not
  // braking to a standstill either, which on a moving target leaves a permanent lag of v^2/2a (3.3
  // deg at 20 deg/s with these limits, about 80 px on this station's screen). That distinction was
  // found by a test, not by reading: the first version of this file passed everything except
  // following a target that was actually moving.
  //
  // Braking along the error direction is time-optimal and terminates: r' = -sqrt(2 a r) reaches
  // zero in finite time, so the reference arrives instead of creeping toward the target forever.
  double u_want = std::max(0.0, u_target);
  if (a_max_rad_s2 > 0.0) {
    // Aim the braking curve at where the reference will BE at the middle of this period, not where
    // it stood when the period began. Without that half-step the profile trails the continuous
    // decel curve by about a*dt*t_decel/2 - on this station 0.075 deg, which measured as 0.074 deg
    // of crossing past the target. Half a hundredth of a pixel would not matter; this is 1.8 px, and
    // it is the difference between "arrives" and "arrives and comes back".
    // Distance, so magnitude throughout. The first version of this line subtracted the half-step
    // from the SIGNED error and floored it at zero, which silently made the braking distance zero
    // for every move in the negative direction - the reference simply refused to go there. The test
    // that caught it was the one that happened to aim at a negative angle; a suite that only ever
    // moved one way would have shipped a turret that tracked left and not right.
    const double dist_ahead =
        std::max(0.0, std::abs(err) - std::abs(st.v_rad_s) * 0.5 * dt_s);
    u_want += std::sqrt(2.0 * a_max_rad_s2 * dist_ahead);
  } else {
    u_want = std::max(u_want, std::abs(err) / dt_s);
  }
  u_want = std::min(u_want, v_max_rad_s);

  const double dv_cap =
      (a_max_rad_s2 > 0.0) ? a_max_rad_s2 * dt_s : std::abs(u_want - st.v_rad_s * dir);
  const double v_want = u_want * dir;
  st.v_rad_s += std::max(-dv_cap, std::min(dv_cap, v_want - st.v_rad_s));
  st.q_rad += st.v_rad_s * dt_s;

  // Finish the move - but only when finishing is something this axis could actually do. Zeroing the
  // carried speed is itself an acceleration change, so it is allowed only once that speed is within
  // one period of a_max's authority; and the position is snapped only once the remaining error is
  // inside one period of travel, which at these limits is well under a hundredth of a pixel. The
  // first version of this rule landed eagerly and the acceleration test caught it: an exact arrival
  // bought with an infinite deceleration is not smoothness, it is the same defect in a nicer place.
  const double v_stop_authority = (a_max_rad_s2 > 0.0) ? a_max_rad_s2 * dt_s : 0.0;
  // The position may be snapped onto the target once the residual is inside one period of travel or
  // inside a*dt^2, whichever is larger. The second term is not a fudge: it is the distance a profile
  // that is already stopping as hard as it legally can still covers in one period - 0.0015 deg, or
  // four hundredths of a pixel, on this station. Without that floor the half-step prediction above
  // makes the last approach converge without ever being exactly equal, so the move never
  // terminates and a "settled" verdict never fires - which is the failure mode that turns a smooth
  // reference into a permanently unsettled one in the eyes of everything watching it.
  const double snap_floor = std::max(std::abs(st.v_rad_s) * dt_s, a_max_rad_s2 * dt_s * dt_s);
  if (std::abs(st.v_rad_s) <= v_stop_authority &&
      std::abs(target_rad - st.q_rad) <= snap_floor) {
    st.q_rad = target_rad;
    st.v_rad_s = 0.0;
  }
  return st.q_rad;
}

}  // namespace ota::control

#endif  // OTA_CONTROL_REFERENCE_LIMITER_HPP
