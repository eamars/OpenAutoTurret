#pragma once
//
// Aim deadband with hysteresis (§ drive-mode item 3, requested by the operator).
//
// WHY THIS EXISTS
// The AUTO_TRACK aim is the predicted line-of-sight, and that estimate moves a little even when the target does not:
// the detector reports a slightly different box each frame, and the estimator's velocity term shakes with it. Every
// millimetre of that wobble currently becomes a re-aim, so the turret keeps restarting small corrections — which is
// what an operator sees as fidget while the encoder itself is provably quiet (measured on this station at rest:
// 0.0877 deg peak-to-peak with ZERO reversals, PROGRESS 2026-09-06 round 2).
//
// WHAT IT IS NOT
// This does not touch the position servo. The axis keeps servoing to the aim with the same authority as before; only
// the *re-aiming* is damped. The supervisor, the envelope and the safety authority are untouched and stay armed. It is
// the "(1) reading" recorded in PROGRESS round 3 — not the reading that opens the position loop, which would stop
// correcting gravity sag and backlash creep on a station with a MEASURED friction deadband. That one is the
// operator's decision, not a side effect of a helper struct.
//
// WHY TWO THRESHOLDS
// One threshold chatters: a target sitting exactly at the boundary alternates between following and holding every
// frame, which is worse than either. So a small deviation ENTERS the hold, and only a larger one RELEASES it. The
// gap between them is the hysteresis band, and the numbers are deliberately the operator's to choose — the measured
// floor on this station is 0.02177 deg (the smallest non-zero step the reported angle can take, round 2), so a band
// at or below that is indistinguishable from quantisation and would be decoration.
//
// DISABLED BY DEFAULT: with enter_deg == 0 (the config default) apply() returns the candidate untouched and clears its
// state, so a station that never asks for this behaves exactly as it did before this file existed.

#include <cmath>

namespace ota {

struct AimDeadband {
  // Anchor: the aim we are pointing at while holding. Initialised on first use, because "the last aim" has no
  // meaning before the first measurement arrives, and starting from zero would swing the turret to boresight.
  double az_rad = 0.0;
  double el_rad = 0.0;
  bool armed = false;
  bool holding = false;
  // Set when the operator's pair was inverted or missing and had to be clamped, so the caller can say so once in
  // telemetry rather than the clamp being invisible. Silent correction of a safety-relevant number is the habit this
  // project has already been burned by.
  bool config_clamped = false;

  void reset() {
    armed = false;
    holding = false;
  }

  // Filter a candidate LOS (radians, world bearings) against the anchor. `az_out`/`el_out` are modified in place so
  // the caller cannot forget to use the result. enter/release in RADIANS; release < enter is a configuration error
  // and is clamped to 1.5 x enter rather than trusted, because an inverted pair would otherwise stop holding
  // entirely — a failure that looks like "it just ignores the deadband" and costs an afternoon to find.
  void apply(double* az_out, double* el_out, double enter_rad, double release_rad) {
    if (enter_rad <= 0.0) {  // disabled: pass straight through
      reset();
      config_clamped = false;
      return;
    }
    config_clamped = release_rad < enter_rad;
    if (config_clamped) release_rad = enter_rad * 1.5;

    if (!armed) {  // first measurement since enabling/re-arming: anchor where we are asked to point
      az_rad = *az_out;
      el_rad = *el_out;
      armed = true;
      holding = false;
      return;
    }

    // Elevation is small-angle here and the band is tiny (fractions of a degree), so planar distance is adequate and
    // avoids a trig call per cycle on the control thread. A wrap-around at +/-pi would matter for a coarse gate, not
    // for a sub-degree band; the difference is only correct if the target is not within a few degrees of the seam,
    // which is stated rather than assumed because the seam is real.
    const double d = std::hypot(*az_out - az_rad, *el_out - el_rad);

    if (!holding) {
      if (d >= enter_rad) {  // real movement: re-anchor and follow it
        az_rad = *az_out;
        el_rad = *el_out;
        return;
      }
      holding = true;  // small deviation: stop re-aiming, let the servo sit still
    } else if (d >= release_rad) {
      holding = false;  // confirmed departure: follow again, at full authority
      az_rad = *az_out;
      el_rad = *el_out;
      return;
    }

    *az_out = az_rad;  // inside the band: point where we already point
    *el_out = el_rad;
  }
};

}  // namespace ota
