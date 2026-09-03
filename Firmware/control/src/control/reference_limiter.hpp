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
  // The profile's own acceleration. A state, not a per-cycle recomputation, because the whole point
  // of the third-order limiter is that acceleration cannot change instantaneously.
  double a_rad_s2 = 0.0;
  double prev_target_rad = 0.0;
  bool have_prev_target = false;

  // Re-seat the profile at an actual pose - used while the axes stand still, so the first cycle of
  // a move starts from where the hardware is rather than where the last engagement ended.
  void reset_at(double q0_rad) {
    q_rad = q0_rad;
    v_rad_s = 0.0;
    a_rad_s2 = 0.0;
    target_v_rad_s = 0.0;
    have_prev_target = false;
    initialised = true;
  }
};

// Distance needed to bring the profile to a stop from speed `v_rad_s` while its acceleration is
// currently `a_rad_s2`, given both ceilings. With bounded jerk this is strictly more than v^2/2a,
// because the profile first has to spend TIME turning its acceleration around, and it travels while
// doing so. The closed form is short: ramp a to -a_max over t1 = (a + a_max)/j, then coast down at
// -a_max. Where the ramp alone would already have stopped it, the ramp expression is used as-is,
// which overestimates - and overestimating here means braking earlier, which is the safe direction.
//
// This function exists because omitting it is not a small error: with the plain v^2/2a rule the
// profile entered every final approach with the turnaround still to pay, overshot by 0.16 deg, and
// never landed at all - it was still travelling at 21 deg/s two hundred simulated seconds in.
inline double stopping_distance_rad(double v_rad_s, double a_rad_s2, double a_max_rad_s2,
                                    double j_max_rad_s3) {
  const double av = std::abs(v_rad_s);
  if (av <= 0.0 || a_max_rad_s2 <= 0.0) return 0.0;
  if (j_max_rad_s3 <= 0.0) return (av * av) / (2.0 * a_max_rad_s2);

  const double sgn = (v_rad_s >= 0.0) ? 1.0 : -1.0;
  const double a_on = std::max(-a_max_rad_s2, std::min(a_max_rad_s2, a_rad_s2 * sgn));  // along motion
  const double t1 = (a_on + a_max_rad_s2) / j_max_rad_s3;
  const double d1 = av * t1 + 0.5 * a_on * t1 * t1 - j_max_rad_s3 * t1 * t1 * t1 / 6.0;
  const double v1 = av + a_on * t1 - 0.5 * j_max_rad_s3 * t1 * t1;
  if (v1 <= 0.0) return std::max(0.0, d1);
  return std::max(0.0, d1 + (v1 * v1) / (2.0 * a_max_rad_s2));
}

// Advance one control period toward `target_rad`, never exceeding `v_max_rad_s` or `a_max_rad_s2`,
// and return the reference to publish this cycle.
//
// The wanted velocity is not "close the error in one cycle" - that is what makes a step response
// step. It is the speed from which the profile can still brake to exactly the target at a_max,
// capped at v_max. That single choice is what removes the overshoot, and overshoot is what
// requirement (b) calls oscillation. Position is then integrated from the velocity that survived
// the a_max ramp; capping position and velocity independently instead would let the position run
// away from the speed the profile claims to have.
// j_max_rad_s3 is deliberately not defaulted: every caller must say what jerk the machine allows,
// because a call site that quietly omitted it would restore the exact defect this exists for (the
// station measured 793 deg/s^3 median against a configured 300). Passing 0 means "no jerk figure is
// configured for this axis", which falls back to ramping acceleration no further than a_max itself -
// the previous behaviour, not an unlimited jump.
inline double limit_reference(ReferenceLimiter& st, double target_rad, double dt_s,
                              double v_max_rad_s, double a_max_rad_s2, double j_max_rad_s3) {
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
    st.a_rad_s2 = 0.0;
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
    // Reserve what it will cost to turn the acceleration around before braking is even possible,
    // then brake against what is left. Omitting this is what made the profile arrive too fast to
    // stop and never land.
    const double reserve =
        std::max(0.0, stopping_distance_rad(st.v_rad_s, st.a_rad_s2, a_max_rad_s2, j_max_rad_s3) -
                          (st.v_rad_s * st.v_rad_s) / (2.0 * a_max_rad_s2));
    u_want += std::sqrt(2.0 * a_max_rad_s2 * std::max(0.0, dist_ahead - reserve));
  } else {
    u_want = std::max(u_want, std::abs(err) / dt_s);
  }
  u_want = std::min(u_want, v_max_rad_s);

  const double v_want = u_want * dir;
  if (a_max_rad_s2 > 0.0 && j_max_rad_s3 > 0.0) {
    // Third order, which is the whole reason this branch exists. Capping the MAGNITUDE of
    // acceleration while still changing it in steps bounds jerk only by a_max/dt - 12,000 deg/s^3
    // at 200 Hz against the 300 the station configures - and that is precisely the 793 deg/s^3
    // median that was measured on the real axis. Capping a quantity is not the same as ramping it,
    // and the mistake had simply moved one derivative down.
    double a_want = std::max(-a_max_rad_s2,
                             std::min(a_max_rad_s2, (v_want - st.v_rad_s) / dt_s));
    // Do not ask for acceleration that would push through the speed ceiling: cap it to what reaches
    // the ceiling exactly this cycle. The first version asked for it, integrated it, and then
    // clamped the velocity and ZEROED the acceleration in the same cycle - which bounded jerk at
    // a_max/dt, 12,000 deg/s^3, while the file was busy claiming to limit jerk to 300. Fixing one
    // step discontinuity by adding another is the same mistake with a different sign, and the test
    // that caught it measured 6,900 deg/s^3 in simulation before it ever reached hardware.
    if (a_want * st.v_rad_s > 0.0 || (st.v_rad_s == 0.0 && a_want != 0.0)) {
      // Reserve the speed that will still be gained while the acceleration is being ramped back to
      // zero - a^2/2j - before asking for any more. Without that term the profile only discovers the
      // ceiling when it is already against it, and then either cuts acceleration in one cycle (a
      // 6,900 deg/s^3 jolt, measured, which is the very defect this file was written to remove) or
      // overshoots the ceiling and has to be clipped, which is the same jolt in the velocity.
      // Anticipating it is what makes the arrival at full speed a curve instead of a corner.
      const double ramp_gain =
          (j_max_rad_s3 > 0.0) ? (st.a_rad_s2 * st.a_rad_s2) / (2.0 * j_max_rad_s3) : 0.0;
      const double head_room = v_max_rad_s - std::abs(st.v_rad_s) - ramp_gain;
      a_want = std::min(a_want * ((st.v_rad_s != 0.0) ? (st.v_rad_s > 0.0 ? 1.0 : -1.0)
                                                     : (a_want > 0.0 ? 1.0 : -1.0)),
                        head_room / dt_s) *
               ((st.v_rad_s != 0.0) ? (st.v_rad_s > 0.0 ? 1.0 : -1.0)
                                    : (a_want > 0.0 ? 1.0 : -1.0));
    }
    const double da_cap = j_max_rad_s3 * dt_s;
    st.a_rad_s2 += std::max(-da_cap, std::min(da_cap, a_want - st.a_rad_s2));
    // No clamp here any more, and that is deliberate. The head-room cap above reserves a^2/2j of
    // speed, so the ceiling is approached on a curve and never crossed; clamping the already-ramped
    // acceleration to "whatever reaches the ceiling this cycle" was an instantaneous cut, and it is
    // what put p95 jerk at 340 and the peak at 851 deg/s^3 on hardware while the median sat exactly
    // at the 300 the construction promises. Every change to acceleration now goes through the jerk
    // ramp, which is the only way the limit is true rather than approximate.
    st.v_rad_s += st.a_rad_s2 * dt_s;
    // Dust only: nothing should reach this, and if it does it is a rounding hair, not a clamp.
    if (std::abs(st.v_rad_s) > v_max_rad_s)
      st.v_rad_s = std::copysign(v_max_rad_s, st.v_rad_s);
  } else {
    const double dv_cap =
        (a_max_rad_s2 > 0.0) ? a_max_rad_s2 * dt_s : std::abs(v_want - st.v_rad_s);
    st.v_rad_s += std::max(-dv_cap, std::min(dv_cap, v_want - st.v_rad_s));
    st.a_rad_s2 = 0.0;
  }
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
    st.a_rad_s2 = 0.0;  // inside the same position resolution as the snap above, not a real jolt
  }
  return st.q_rad;
}

}  // namespace ota::control

#endif  // OTA_CONTROL_REFERENCE_LIMITER_HPP
