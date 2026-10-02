"""ADR-003 stage 1: the 14 scenarios in simulation, the Level-1 FF/FB comparison, and checks.

Each scenario runs three times with the same seed: FF+FB with the ADR-002.2 servos and plant
models ("ff"), the same with the target motion off ("fb": no velocity feedforward and no motion
extrapolation), and FF+FB on an ideal actuator ("ideal": Level 1 alone).

The checks test the property each scenario must show (ADR-003 03 sec. 6, "必须证明"), with
bounds that come from design quantities, never from a result:
  H      the estimator's coast horizon: a changed motion must be recognised before the old
         model's own validity runs out;
  4/lam  the critically damped Level-1 settling time (to about 9%);
  stop   the reference's own time to stop from v_max (v/a + a/j) at its limits;
  sigma  the anchor pixel noise: at rest the framing error must come down to the noise level;
  brake  the reference's worst braking distance at its own v/a/j limits.
Framing errors, lags and peaks are reported as performance, to be judged against the
photography budget (config/tracking/photography_spec.json), not gated here.
"""
import math

import numpy as np

import evaluate as ev
import scenarios
import sim

DEG = math.pi / 180


def run_all(params, ids=None):
    out = {}
    for sid in ids or scenarios.SCENARIOS:
        out[sid] = {}
        for key, motion, actuator in (("ff", True, "servo"), ("fb", False, "servo"), ("ideal", True, "ideal")):
            t, f, status, applied = sim.run(scenarios.build(sid, params, motion=motion, actuator=actuator))
            out[sid][key] = (t, f, status)
        out[sid]["applied"] = applied
    return out


def braking_distance(v, a_max, j_max):
    """Worst stopping distance from speed v with acceleration a_max still pushing outward."""
    t1 = 2 * a_max / j_max  # ramp the acceleration from +a_max to -a_max
    d1 = v * t1 + a_max * t1 ** 2 / 2 - j_max * t1 ** 3 / 6
    v1 = v + a_max * t1 - j_max * t1 ** 2 / 2
    return d1 + max(v1, 0.0) ** 2 / (2 * a_max)


def _c(name, value, limit, ok):
    v = None if value is None else float(value)
    return {"check": name, "value": None if v is None or not math.isfinite(v) else round(v, 4), "limit": limit, "pass": bool(ok)}


def recognised(t, start, horizon):
    """Seconds after `start` until the rate estimate is within 2 sigma of the truth at its own state time."""
    k = t["t"] >= start
    tt, est, sig = t["t"][k], t["est_waz"][k], t["sig_waz"][k]
    truth = np.interp(tt - t["age"][k], t["t"], t["truth_waz"])
    ok = np.abs(est - truth) <= 2 * sig
    bad = np.where(~ok)[0]
    if not len(bad):
        return 0.0
    if bad[-1] == len(ok) - 1:
        return float("inf")
    return float(tt[bad[-1] + 1] - start)


def speed_ratio(t, a, b, rate_dps):
    q, tt = ev.window(t["qtrue_y"], t["t"], a, b), ev.window(t["t"], t["t"], a, b)
    return float(np.polyfit(tt, q, 1)[0] / (rate_dps * DEG))


def checks(sid, r, params):
    t, f, status = r["ff"]
    tb, fb, _ = r["fb"]
    end = float(t["t"][-1])
    lim, est = params["level1"], params["estimator"]
    H, lam, sigma = est["horizon_s"], lim["yaw"]["lambda_rad_s"], params["pixel_sigma_px"]
    settle = H + 4 / lam
    c = lim["yaw"]
    stop = c["v_max_rad_s"] / c["a_max_rad_s2"] + c["a_max_rad_s2"] / c["j_max_rad_s3"]
    w = lambda x, a, b: ev.window(x, t["t"], a, b)
    out = [_c("session completes", None, "COMPLETE", status == "COMPLETE"),
           _c("reference is one integral (rad)", ev.consistency(t), 1e-6, ev.consistency(t) <= 1e-6),
           _c("reference within its v/a/j limits (fraction)", ev.bounds(t, lim), 1.0, ev.bounds(t, lim) <= 1 + 1e-9)]
    if sid == "T01":
        first = float(np.min(f["t_arrival"][f["delivered"] > 0]))
        moved = t["t"][np.abs(t["vr_y"]) > 0]
        start = float(moved[0]) if len(moved) else float("inf")
        slew = (t["t"] > 0.3) & (t["t"] < 2.0)
        excess = float(np.mean(np.abs(t["est_waz"][slew] - t["truth_waz"][slew]) > 3 * t["sig_waz"][slew]))
        final = ev.framing(f, end - 2, end)["framing_rms_px"]
        # The tick that consumes the frame chooses the first jerk; its motion shows from the next tick.
        out += [_c("motion starts within two ticks of the first observation (s)", start - first, 2 * lim["period_s"] + 1e-4,
                   start - first <= 2 * lim["period_s"] + 1e-4),
                _c("converges from angle error alone: final framing RMS (px)", final, f"<= sigma {sigma}", final <= sigma),
                _c("camera rotation is not target motion: rate beyond 3 sigma while slewing (fraction)", excess, 0.01, excess <= 0.01),
                _c("FF ~0 for a static subject: mean weight", float(np.mean(w(t["ffw_az"], 2, end))), 0.05,
                   np.mean(w(t["ffw_az"], 2, end)) <= 0.05)]
    elif sid == "T02":
        fr = ev.framing(f, 1, end)["framing_rms_px"]
        drift = abs(float(np.polyfit(w(t["t"], 1, end), w(t["qtrue_y"], 1, end), 1)[0])) / DEG
        out += [_c("hold jitter at or below the pixel noise: framing RMS (px)", fr, f"<= sigma {sigma}", fr <= sigma),
                _c("FF tends to zero: mean weight", float(np.mean(w(t["ffw_az"], 1, end))), 0.05, np.mean(w(t["ffw_az"], 1, end)) <= 0.05),
                _c("no drift (deg/s)", drift, 0.01, drift <= 0.01)]
    elif sid == "T03":
        ratio = speed_ratio(t, 8, end, 2.0)
        rec = recognised(t, 0.0, H)
        ff, fbr = ev.framing(f, 8, end)["framing_rms_px"], ev.framing(fb, 8, end)["framing_rms_px"]
        out += [_c("the axis really follows (speed ratio)", ratio, "1 +- 0.05", abs(ratio - 1) <= 0.05),
                _c("rate confidence converges: within 2 sigma (s)", rec, f"<= {end - 8}", rec <= end - 8),
                _c("FF+FB framing no worse than FB only (px)", ff, round(fbr, 2), ff <= fbr)]
    elif sid == "T04":
        ff, fbr = ev.framing(f, 3, end)["framing_rms_px"], ev.framing(fb, 3, end)["framing_rms_px"]
        lf, lb = ev.lag_s(t, 3, end, 20.0), ev.lag_s(tb, 3, end, 20.0)
        out += [_c("moving lag below FB only (s)", lf, round(lb, 4), abs(lf) < abs(lb)),
                _c("framing below FB only (px)", ff, round(fbr, 2), ff < fbr)]
    elif sid in ("T05", "T06"):
        change_end = 3.0
        rec = recognised(t, change_end, H)
        out += [_c("innovation updates the rate within the horizon (s)", rec, H, rec <= H)]
        if sid == "T06":
            st = ev.settle_s(f, change_end, 3 * sigma, end)
            out += [_c("no lasting lead after the stop: within 3 sigma (s)", st, round(settle + stop, 3), st <= settle + stop)]
    elif sid == "T07":
        rec = recognised(t, 3.0, H)
        st = ev.settle_s(f, 3.0, 3 * sigma, end)
        # Not knowable before it is seen: recognition (<= H), the reference's own stop, Level-1 settling.
        out += [_c("new observations correct the old model within the horizon (s)", rec, H, rec <= H),
                _c("stop settles within H + stop + 4/lambda (s)", st, round(settle + stop, 3), st <= settle + stop)]
    elif sid == "T08":
        ch_ff = ev.sign_changes(w(t["goal_waz"], 1.5, end), w(t["ffw_az"], 1.5, end))
        ch_v = ev.sign_changes(w(t["vr_y"], 1.5, end), np.ones_like(w(t["vr_y"], 1.5, end)), floor=0.5)
        rec = recognised(t, 3.0, H)
        tf, tb_ = ev.layers(t, 1, end)["e_total_yaw_rms_deg"], ev.layers(tb, 1, end)["e_total_yaw_rms_deg"]
        out += [_c("target FF changes sign once (no chatter)", ch_ff, 1, ch_ff <= 1),
                _c("reference reverses once (no chase in the old direction)", ch_v, 1, ch_v <= 1),
                _c("new direction recognised within the horizon (s)", rec, H, rec <= H),
                _c("FF+FB total error below FB only (deg)", tf, round(tb_, 4), tf < tb_)]
    elif sid == "T09":
        fr = ev.framing(f, 2, end)["framing_rms_px"]
        noise_px = scenarios.SCENARIOS["T09"]["camera"]["pixel_noise_px"]
        # The goal rate steps with each frame by design; the reference must not (only jerk-limited change).
        vref = ev.peak_step(w(t["vr_y"], 2, end))
        step = c["a_max_rad_s2"] * lim["period_s"]
        out += [_c("FF enters through the generator: reference speed step per tick (deg/s)", vref / DEG, round(step / DEG, 4), vref <= step + 1e-12),
                _c("position feedback holds the subject at the noise level: framing RMS (px)", fr, f"<= sigma {noise_px}", fr <= noise_px)]
    elif sid == "T10":
        off = w(t["goal_waz"], 4.01, 7.0)
        ratio = speed_ratio(t, 5.0, 7.0, 8.0)
        before = abs(ev.lag_s(t, 2.0, 4.0, 8.0)); during = abs(ev.lag_s(t, 5.0, 7.0, 8.0)); after = abs(ev.lag_s(t, 7.0 + settle, end, 8.0))
        out += [_c("FF exactly zero while unavailable", ev.peak(off), 0.0, ev.peak(off) == 0),
                _c("position feedback keeps following (speed ratio)", ratio, "1 +- 0.05", abs(ratio - 1) <= 0.05),
                _c("FF restored: lag back below the unavailable lag (s)", after, round(during, 4), after < during),
                _c("FF restored to the earlier lag (s)", after, round(before + 0.01, 4), after <= before + 0.01)]
    elif sid == "T11":
        ff, fbr = ev.framing(f, 5, 8)["framing_rms_px"], ev.framing(fb, 5, 8)["framing_rms_px"]
        ratio = speed_ratio(tb, 5.0, 8.0, 8.0)
        # A frame delivered after a newer one is an old packet: dropped and counted, never used.
        d = np.where(f["delivered"] > 0)[0]
        d = d[np.argsort(f["t_arrival"][d], kind="stable")]
        newest = np.maximum.accumulate(f["t_mid"][d])
        in_order = d[f["t_mid"][d] >= newest]
        k = in_order[(f["t_mid"][in_order] >= 4) & (f["t_mid"][in_order] < 8)]
        stale = int(len(d) - len(in_order))
        rejected = int(np.sum((f["delivered"] > 0) & (f["accepted"] == 0) & (f["out_of_view"] == 0)))
        out += [_c("delayed in-order frames used at their optical time (accepted fraction)", float(np.mean(f["accepted"][k] > 0)), 1.0,
                   bool(np.all(f["accepted"][k] > 0))),
                _c("only out-of-order frames rejected (count)", rejected, stale, rejected == stale),
                _c("prediction better than none under delay (px)", ff, round(fbr, 2), ff < fbr),
                _c("closed loop without prediction (FB-only speed ratio)", ratio, "1 +- 0.05", abs(ratio - 1) <= 0.05)]
    elif sid == "T12":
        gaps = float(np.max(np.diff(t["t"])))
        held = ev.window(t["pos_valid"], t["t"], 3.0, 3.6)
        acc = int(t["accepted"][-1]); delivered = int(np.sum(f["delivered"] > 0))
        out += [_c("control never waits for vision (max tick gap s)", gaps, lim["period_s"] + 1e-6, gaps <= lim["period_s"] + 1e-6),
                _c("bounded coast keeps the subject through a 0.4 s drop (valid fraction)", float(np.mean(held)), 1.0, np.mean(held) == 1),
                _c("one update per frame (accepted <= delivered)", acc, delivered, acc <= delivered)]
    elif sid == "T13":
        flagged = bool(np.any((w(t["flag_y"], 3, 5).astype(int) & 16) > 0))
        lead = ev.peak(w(t["qr_y"] - t["qm_y"], 3, 5)) / DEG
        bound = (c["lead_limit_rad"] + braking_distance(c["v_max_rad_s"], c["a_max_rad_s2"], c["j_max_rad_s3"])) / DEG
        # After release the reference first closes the gap the target opened while the axis was
        # stuck (2 s at 10 deg/s, closed at v_max - 10 deg/s), then stops its excess speed and settles.
        gap_s = 2.0 * 10.0 / (c["v_max_rad_s"] / DEG - 10.0)
        before = ev.framing(f, 1.5, 3.0)["framing_rms_px"]
        after = ev.framing(f, 5.0 + gap_s + stop + settle, end)["framing_rms_px"]
        out += [_c("saturation recorded (lead limit engaged)", None, True, flagged),
                _c("bounded: reference lead over the stuck axis (deg)", lead, round(bound, 3), lead <= bound),
                _c("after release no old waypoints: back to the pre-saturation framing (px)", after, round(before + sigma, 2), after <= before + sigma)]
    elif sid == "T14":
        lost = t["t"][(t["t"] > 3.0) & (t["pos_valid"] == 0)]
        when = float(lost[0] - 3.0) if len(lost) else float("inf")
        limit = H + scenarios.CAMERA["frame_period_s"] + scenarios.CAMERA["latency_s"] + 0.03
        held = ev.peak(ev.window(t["vr_y"], t["t"], 3.0 + limit + 4 / lam, 5.5)) / DEG
        k2 = np.where(t["identity_changes"] > 0)[0]
        reset = abs(float(t["est_waz"][k2[0]])) / DEG if len(k2) else float("inf")
        st = ev.settle_s(f, 8.0, 3 * sigma, end)
        out += [_c("finite coast: target invalid after the horizon (s)", when, round(limit, 3), when <= limit),
                _c("then the reference holds (deg/s)", held, 0.05, held <= 0.05),
                _c("new identity does not inherit velocity (deg/s)", reset, 0.0, reset == 0),
                _c("settles on the new subject within H + 4/lambda (s)", st, round(settle, 3), st <= settle)]
    return out


def performance(sid, r):
    """Reported (not gated): framing at exposure, blur, and the error layers, whole run."""
    t, f, _ = r["ff"]
    end = float(t["t"][-1])
    out = {"ff_fb": ev.framing(f, 0, end), "fb_only": ev.framing(r["fb"][1], 0, end), "ideal_actuator": ev.framing(r["ideal"][1], 0, end)}
    out["layers_ff_fb"] = {k: round(v, 4) for k, v in ev.layers(t, 0, end).items()}
    return out


def comparison(r):
    """ADR-003 sec. 7A: Level-1 FF+FB against FB only, same data and parameters, steady windows."""
    out = {}
    for sid, (a, b, rate) in {"T03": (8, 20, 2.0), "T04": (3, 8, 20.0), "T05": (4.5, 6, 20.0), "T08": (4.2, 6, -15.0),
                              "T10": (9, 12, 8.0), "T11": (5, 8, 8.0)}.items():
        if sid not in r:
            continue
        t, f, _ = r[sid]["ff"]; tb, fb, _ = r[sid]["fb"]; ti, fi, _ = r[sid]["ideal"]
        out[sid] = {"window_s": [a, b], "target_rate_dps": rate,
                    "framing_rms_px": {"ff_fb": round(ev.framing(f, a, b)["framing_rms_px"], 2),
                                       "fb_only": round(ev.framing(fb, a, b)["framing_rms_px"], 2),
                                       "ff_fb_ideal_actuator": round(ev.framing(fi, a, b)["framing_rms_px"], 2)},
                    "lag_s": {"ff_fb": round(ev.lag_s(t, a, b, rate), 4), "fb_only": round(ev.lag_s(tb, a, b, rate), 4)}}
    return out
