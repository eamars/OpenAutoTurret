"""Controller design from identified plant parameters (deterministic rules + simulation).

Yaw: one loop shape scaled by inertia: kq = a*wn^2, kv = 2*zeta*a*wn (zeta 0.5), and an
integral at a fixed rate ki = integral_rate*kq (1.5/s): the integral works against friction,
which does not scale with inertia (station 2026-10-02: wn-scaled integral left steps 0.2 deg short).
The usable wn is bounded by the encoder current crosstalk left after compensation:
the reading becomes q + r*i, and a residual r <= 0 puts a right-half-plane zero at
1/sqrt(a*|r|); a small positive residual (the compensation bias b) keeps it away.
The boundary is found in simulation for residuals b +- (the table uncertainty + a drift
allowance), at every angle, and checked on the station by a gain ladder.

The drift allowance is crosstalk_drift_fraction x the table's largest |g| (rule 1.0: the loop
survives losing the compensation entirely). Station 2026-10-02: commissioned at 12:34 with
the boundary taken at the scan's own +-0.48 mrad/A, by 21:15 the reading at 326 deg moved
with the current at -6.7..-8.1 mrad/A against the table's -4.1, and the servo buzzed at
standstill after hard stops (a crosstalk loop held up by static friction) until the
supervisor held the station. The scan's scatter within one session is not the uncertainty
that matters in service.

Pitch: the drive's speed loop is a delay d and lag tau; the host P loop around it
gets the gain with the requested phase margin, ki from the same ratio rule.
"""
import math

import numpy as np

import manifests
import sim

DEG = math.pi / 180


def gains(inertia, wn, zeta=0.5, integral_rate=1.5):
    kq = inertia * wn * wn
    return {"kq": round(kq, 3), "kv": round(2 * zeta * inertia * wn, 4), "ki": round(kq * integral_rate, 2)}


def plant(identified):
    """The simulator's yaw plant from identified parameters (asset "plant" section)."""
    f = identified["friction"]
    p = {"inertia": identified["inertia"],
         "coulomb_positive": f["positive"]["coulomb"], "coulomb_negative": f["negative"]["coulomb"],
         "stribeck_positive": f["positive"]["stribeck"], "stribeck_negative": f["negative"]["stribeck"],
         "stribeck_speed": 0.5 * (f["positive"]["stribeck_speed"] + f["negative"]["stribeck_speed"]),
         "viscous": 0.5 * (f["positive"]["viscous"] + f["negative"]["viscous"]),
         "creep_drop": f.get("creep_drop", 0.0), "creep_speed": f.get("creep_speed", 0.01),
         "presliding_stiffness": 2000.0, "presliding_damping": 5.0,
         "friction_map_positive": f["map_positive"], "friction_map_negative": f["map_negative"],
         "actuation_delay_s": identified["actuation_delay_s"], "current_tau_s": 0.0005, "encoder_delay_s": 0.0003,
         "crosstalk_delay_s": identified["crosstalk"]["delay_s"], "crosstalk_map": identified["crosstalk"]["table"]}
    return p


def servo(base, identified, design):
    """Full servo_parameters: the prior's limits and observer, identified feedforward, designed gains."""
    s = dict(base)
    f = identified["friction"]
    s.update(inertia=round(identified["inertia"], 5),
             coulomb_positive=round(f["positive"]["coulomb"], 4), coulomb_negative=round(f["negative"]["coulomb"], 4),
             stribeck_positive=round(f["positive"]["stribeck"], 4), stribeck_negative=round(f["negative"]["stribeck"], 4),
             stribeck_speed=round(0.5 * (f["positive"]["stribeck_speed"] + f["negative"]["stribeck_speed"]), 4),
             viscous=round(0.5 * (f["positive"]["viscous"] + f["negative"]["viscous"]), 4),
             creep_drop=round(f.get("creep_drop", 0.0), 4), creep_speed=round(f.get("creep_speed", 0.01), 5),
             friction_map_positive=f["map_positive"], friction_map_negative=f["map_negative"],
             friction_learning_rate=0.5,
             crosstalk_delay_s=identified["crosstalk"]["delay_s"],
             crosstalk_map=[round(g - design["bias"], 7) for g in identified["crosstalk"]["table"]])
    s.update(gains(identified["inertia"], design["wn"], design["zeta"], design["integral_rate"]))
    # Stall recovery and the integral authority scale with the low-speed friction level.
    low = 0.5 * (f["positive"]["coulomb"] + f["positive"]["stribeck"] + f["negative"]["coulomb"] + f["negative"]["stribeck"])
    s["integral_cap"] = round(min(1.2, max(0.4, 1.0 * low)), 3)
    # Stall recovery is always on in a designed asset (the prior leaves it off: its
    # thresholds need the friction level): a 25 ms reverse rock after 0.3 s stuck more
    # than 0.14 deg behind while pushing (station 2026-10-02: 0.29 deg let small steps
    # sit 0.27-0.29 deg short at 1.3 A, just inside the threshold).
    # Only near a stopped reference (|v_ref| < 0.01 rad/s): a rock during the walking profile's
    # reversals cost 0.26 deg RMS there (station 2026-10-02), while the reversal frees it anyway.
    stall = dict(s["stall_recovery"], error_rad=0.0025, speed_rad_s=0.009, time_s=0.3, rock_s=0.025,
                 reference_speed_rad_s=0.01)
    stall["current_A"] = round(min(1.2, max(0.3, 0.9 * low)), 3)
    stall["rock_current_A"] = round(min(0.8, max(0.2, 0.6 * low)), 3)
    s["stall_recovery"] = stall
    return s


def _stability_script():
    """Eight test angles 45 degrees apart: at each a ladder step (hold, small steps, slow ramps)."""
    S = []
    for k in range(8):
        S.append((3.0, "step", 45 * DEG, "move"))
        S += manifests.ladder_step(str(k))
    return S


_SCRIPT = None


def stable(plant_parameters, servo_parameters, start, limit=0.3):
    """True when the simulated eight-angle ladder step stays below the oscillation limit.
    Returns (stable, angle_of_onset)."""
    global _SCRIPT
    if _SCRIPT is None:
        _SCRIPT = manifests.samples(_stability_script())[0]
    request = {"axis": "yaw", "servo": servo_parameters, "plant": plant_parameters,
               "reference": sim.reference_arrays(_SCRIPT), "start_position_rad": start,
               "hold_after_s": 0.5, "speed_limit_rad_s": 1.75, "oscillation_limit_A": limit}
    r, status = sim.run(request)
    # Too-weak gains end in a following-error stop: not unstable, just slow.
    if "oscillation" not in status and "speed limit" not in status:
        return True, None
    return False, float(np.mod(r["q_true"][-1], 2 * math.pi))


def boundary(identified, base, bias, residual_shift, zeta, integral_rate, start=0.0, lo=15.0, hi=200.0, steps=7):
    """Largest stable wn (bisection in log wn) for plant crosstalk = table + residual_shift,
    servo table = table - bias. Returns (wn, onset angle just above it)."""
    p = plant(identified)
    p["crosstalk_map"] = [g + residual_shift for g in identified["crosstalk"]["table"]]
    design = {"bias": bias, "zeta": zeta, "integral_rate": integral_rate}

    def ok(wn):
        design["wn"] = wn
        return stable(p, servo(base, identified, design), start)

    if not ok(lo)[0]:
        return lo, None
    good, bad, angle = lo, hi, None
    if ok(hi)[0]:
        return hi, None
    for _ in range(steps):
        mid = math.sqrt(good * bad)
        s, a = ok(mid)
        if s:
            good = mid
        else:
            bad, angle = mid, a
    return good, angle


def predicted_usecase(identified, servo_parameters, start=0.0):
    """Simulated use-case pass on the identified plant -> (score rows, extra, status)."""
    import score
    import usecase
    m = manifests.yaw("predict", servo_parameters, "usecase")
    r, status = sim.yaw(servo_parameters, plant(identified), m["reference_samples"], start=start, hold_after=m["servo_hold_after_s"])
    res = {"t": r["t"], "qr": r["qr"], "vr": r["vr"], "q": r["q_hat"], "v": r["v_hat"], "u": r["u"],
           "true_t": r["t"], "true_q": r["q_true"], "true_v": r["v_true"]}
    rows = usecase.metrics(res, score.segments(m))
    return rows, {"stalls": int(r["stalls"][-1]) if len(r["stalls"]) else 0}, status


def predicted_cost(rows, extra, status):
    """Acceptance failures first, then the use-case errors relative to their limits."""
    import score
    if status != "COMPLETE":
        return 1e6
    lim = score.YAW_LIMITS
    worst = lambda prefix, key: max([abs(r[key]) for r in rows if r["label"].startswith(prefix) and key in r] or [0.0])
    return (10.0 * len(score.accept_yaw(rows, extra)) + worst("ramp", "pos_p95_5") / lim["ramp_p95_5_fast"] +
            worst("walker", "err_rms") / lim["walker_rms"] + worst("step", "final_err") / lim["step_final"])


def crosstalk_error_bound(identified, rules):
    """The crosstalk error (rad/A) the loop must stay quiet with: the scan's uncertainty plus the
    drift allowance (module docstring)."""
    table = identified["crosstalk"]["table"]
    drift = rules.get("crosstalk_drift_fraction", 0.0) * max(abs(g) for g in table)
    return identified["crosstalk"]["uncertainty"] + drift


def design_yaw(identified, base, rules, start=0.0, log=print):
    """For each compensation bias: the robust simulated boundary (table +- the error bound),
    the design gain margin*boundary, and a simulated use-case pass. Keep the bias whose
    use case scores best (the bias trades stability margin for crosstalk error)."""
    delta = crosstalk_error_bound(identified, rules)
    candidates = []
    for bias in rules["crosstalk_bias_candidates_rad_per_A"]:
        results = [boundary(identified, base, bias, shift, rules["zeta"], rules["integral_rate_per_s"], start)
                   for shift in (-delta, +delta)]
        wn_b = min(r[0] for r in results)
        dsg = {"zeta": rules["zeta"], "integral_rate": rules["integral_rate_per_s"], "bias": bias, "wn": rules["design_margin"] * wn_b}
        rows, extra, status = predicted_usecase(identified, servo(base, identified, dsg), start)
        cost = predicted_cost(rows, extra, status)
        log(f"  bias {bias * 1e3:.1f} mrad/A: stable to wn {results[0][0]:.1f} / {results[1][0]:.1f} rad/s "
            f"(table -/+ {delta * 1e3:.2f} mrad/A); use case at wn {dsg['wn']:.1f}: cost {cost:.2f}")
        candidates.append((cost, bias, wn_b, [r[1] for r in results if r[1] is not None]))
    if all(c[2] <= 15.0 * 1.0001 for c in candidates):
        # Unstable even at the lowest gain: the identified friction (usually a survey whose slow
        # plateaus stick-slipped) makes the model unusable for design. The station decides alone.
        log("  MODEL INADEQUATE: no quiet gain even at wn 15 in simulation; the station ladder decides alone")
        bias = rules["crosstalk_bias_candidates_rad_per_A"][1]
        return {"zeta": rules["zeta"], "integral_rate": rules["integral_rate_per_s"], "bias": bias, "model_adequate": False,
                "boundary_wn_sim": None, "wn": None, "onset_angles_sim": []}
    cost, bias, wn_b, angles = min(candidates, key=lambda c: (round(c[0], 2), c[1]))
    return {"zeta": rules["zeta"], "integral_rate": rules["integral_rate_per_s"], "bias": bias, "model_adequate": True,
            "boundary_wn_sim": round(wn_b, 2), "wn": round(rules["design_margin"] * wn_b, 2), "predicted_cost": round(cost, 3),
            "onset_angles_sim": [round(a, 4) for a in angles], "crosstalk_error_bound": round(delta, 6)}


def ladder_angles(design, crosstalk):
    """Two station test angles, where the compensation is least certain: the angle where the
    two scan directions disagree most, then the steepest part of the table (an angle error
    there leaves the most crosstalk), at least 60 degrees apart; then the simulated onsets."""
    g = np.asarray(crosstalk["table"])
    n = len(g)
    candidates = []
    if crosstalk.get("disagreement_by_angle"):
        candidates.append(2 * math.pi * int(np.argmax(crosstalk["disagreement_by_angle"])) / n)
    slope = np.abs(np.roll(g, -1) - np.roll(g, 1))
    candidates += [2 * math.pi * int(k) / n for k in np.argsort(slope)[::-1]]
    candidates += list(design.get("onset_angles_sim") or [])
    chosen = []
    for a in candidates:
        if all(abs(math.remainder(a - c, 2 * math.pi)) > 60 * DEG for c in chosen):
            chosen.append(a)
        if len(chosen) == 2:
            break
    return chosen


def pitch_gains(delay, tau, rules, gain=1.0):
    """kp giving the phase margin of rules['phase_margin_deg'] around g*exp(-sd)/(s(tau s + 1))."""
    target = math.radians(rules["phase_margin_deg"])

    def margin(kp):  # crossover ~ g*kp for an integrator plant with a slow-enough lag
        kp = kp * gain
        w = kp
        for _ in range(50):  # |L(jw)| = 1 -> w = kp / sqrt(1 + (w tau)^2)
            w = kp / math.sqrt(1 + (w * tau) ** 2)
        return math.pi / 2 - math.atan(w * tau) - w * delay

    lo, hi = 1.0, 500.0
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if margin(mid) > target else (lo, mid)
    kp = min(lo, rules["maximum_kp_per_s"])  # lo already includes the speed loop's gain
    return {"kp_per_s": round(kp, 2), "ki_per_s2": round(kp * kp / rules["integral_ratio"], 2),
            "phase_margin_deg": round(math.degrees(margin(kp)), 1), "pm_limited_kp": round(lo, 2)}
