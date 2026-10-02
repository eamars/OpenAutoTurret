"""Automatic servo commissioning: identify the plant, design the controller, verify on the station.

    commission.py yaw   [--from-prior | --update] [--sim TRUTH.json] [--out DIR] [--no-write]
    commission.py pitch [--from-prior | --update] [--sim TRUTH.json] [--out DIR] [--no-write]
    commission.py check yaw|pitch            short probe: is the current asset still right?
    commission.py validate yaw|pitch         two use-case passes with the current asset
    commission.py session yaw|pitch SCRIPT   one session with the current asset (diagnosis)
    commission.py calibrate yaw              accuracy limits from measured FF+FB tracking, and the
                                             motor-FF comparison (ADR-003 sec. 7B); see accuracy.py
    commission.py redesign yaw               gains re-derived offline from the asset's identified plant
                                             under the current design rules (no station)

Yaw, --from-prior (nothing known; config/servo/yaw_prior.json):
  survey     friction against speed (0.25-40 deg/s both ways) and per-angle maps
  crosstalk  80 Hz probe tone over a full turn each way: the encoder's current sensitivity
  inertia    current sweep while sliding: inertia and loop delay
  design     simulation: largest quiet gain on the identified plant, crosstalk table +- its
             uncertainty and a drift allowance (design.py), every 45 degrees; the compensation
             bias that allows the most
  ladder     station: rising gains at the two weakest angles until the oscillation guard
             trips; final gain = onset / 1.2, never above the robust design
  validate   two use-case passes and a full circle, scored against fixed limits
--update starts from the current asset and skips the crosstalk scan when a short
check of the table agrees. (The sensitivity does drift in service -- 3 mrad/A within a day on
2026-10-02 -- which is what the design's drift allowance is for.)

Pitch: home against the endstops (every session), identify the drive's speed loop
from a speed-reference sweep, set kp for a 60 degree phase margin, ladder kp on the
station, validate twice.

Every decision is a fixed rule on measured numbers; the report (report.json,
report.md in --out) records each number and rule. --sim runs the identical pipeline
against servo-sim and a hidden true plant (the offline robustness proof).
"""
import argparse
import copy
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

import accuracy
import design
import identify
import journal
import manifests
import score
import sim
import station as stations

DEG = math.pi / 180
HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
CONFIG = REPO / "Firmware/config/servo"


DELAY_OFFSET_S = 0.0012


class Failed(RuntimeError):
    pass


class Run:
    """One commissioning run: its station, output directory, log and report."""

    def __init__(self, args, axis):
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.id = f"{axis}-{stamp}{'-sim' if args.sim else ''}"
        self.out = Path(args.out or REPO / "run/servo-commission" / self.id)
        self.out.mkdir(parents=True, exist_ok=True)
        self.logfile = open(self.out / "commission.log", "a", encoding="utf-8")
        self.axis = axis
        self.report = {"run": self.id, "axis": axis, "started": stamp, "station": "sim" if args.sim else stations.HOST,
                       "steps": []}
        if args.sim:
            truth = json.load(open(args.sim, encoding="utf-8"))
            self.station = stations.SimStation(truth, log=self.log)
            self.report["sim_truth"] = str(args.sim)
        else:
            self.station = stations.Station(self.out, log=self.log, build=not args.no_build)
        self.count = 0

    def log(self, text):
        print(text, flush=True)
        self.logfile.write(text + "\n"); self.logfile.flush()

    def session(self, name, manifest):
        self.count += 1
        label = f"{self.id}-{self.count:02d}-{name}"
        self.log(f"[{self.count}] {name}: {label}")
        j = self.station.run(self.axis, manifest, label)
        footer = j.get("footer") or {}
        self.log(f"    footer {footer.get('status')} {footer.get('detail', '')}")
        rec = idle_axis(self, self.axis, j)
        if rec:
            self.report.setdefault("idle_axis", []).append(dict(rec, session=name))
        return j

    def step(self, name, **values):
        self.report["steps"].append({"step": name, **values})
        (self.out / "report.json").write_text(json.dumps(self.report, indent=1, default=float), encoding="utf-8")


def complete(j, what):
    footer = j.get("footer") or {}
    if footer.get("status") != "COMPLETE":
        hint = ""
        c = j.get("cycles")
        if c is not None and len(c["t"]) and "following error" in footer.get("detail", ""):
            k = c["t"] > c["t"][-1] - 1.0
            caps = c["cap"][k]
            if np.max(caps) > 0 and (np.any(caps < np.max(c["cap"])) or np.mean(np.abs(c["u"][k])) > 0.9 * np.max(caps)):
                hint = (f" -- the servo ran out of authority (|u| {np.mean(np.abs(c['u'][k])):.2f} A against a cap of "
                        f"{np.min(c['cap'][k]):.2f} A, RMS {np.max(c['rms'][k]):.2f} A): the plant needs more than the motor's continuous budget")
        raise Failed(f"{what}: session ended {footer.get('status')}: {footer.get('detail')}{hint}")


def last_yaw_angle(j):
    return float(j["feedback"]["q"][-1])


IDLE_AXIS_LIMIT = math.radians(0.2)  # the idle axis is unpowered; its friction holds it (measured <= 0.044 deg)


def idle_axis(run, axis, j):
    """Decoupling record: where the other axis sat and how far it moved during this session."""
    if axis == "yaw":
        p = j.get("pitch_posture")
        if p is None or not len(p):
            return None
        rec = {"pitch_posture_rad": round(float(np.median(p)), 4), "pitch_motion_rad": round(float(np.ptp(p)), 5)}
        moved = rec["pitch_motion_rad"]
    else:
        if "yaw_motion_rad" not in j:
            return None
        rec = {"yaw_motion_rad": round(j["yaw_motion_rad"], 5)}
        moved = rec["yaw_motion_rad"]
    if moved > IDLE_AXIS_LIMIT:
        run.log(f"    DECOUPLING: the idle axis moved {math.degrees(moved):.2f} deg during this session (best effort, not a stop)")
    return rec


# ---------------------------------------------------------------------------- yaw
def yaw_survey(run, servo):
    j = run.session("survey", manifests.yaw(f"{run.id}-survey", servo, "survey", vmax=65.0))
    complete(j, "survey")
    samples = identify.friction_plateaus(j)
    rows = identify.friction_by_speed(samples)
    fit = identify.fit_friction(rows)
    maps = identify.friction_maps(samples, fit)
    fit["map_positive"], fit["map_negative"] = maps["positive"], maps["negative"]
    run.log("    friction: " + ", ".join(f"{math.degrees(r['speed']):g}{'+' if r['direction'] > 0 else '-'} {r['current']:.2f}A"
                                         for r in rows))
    run.log(f"    fit: + {fit['positive']['coulomb']:.2f}+{fit['positive']['stribeck']:.2f}A  - {fit['negative']['coulomb']:.2f}+"
            f"{fit['negative']['stribeck']:.2f}A  creep {fit['creep_drop']:.2f}A@{math.degrees(fit['creep_speed']):.2f}deg/s"
            f"  rms {fit['rms_residual']:.3f}A")
    guidance = json.load(open(CONFIG / "yaw_prior.json", encoding="utf-8"))["hardware_signature"]["thermal_guidance_A"]
    heavy = [r for r in rows if r["current"] > guidance]
    if heavy:
        run.log(f"    THERMAL GUIDANCE: sliding friction reaches {max(r['current'] for r in heavy):.2f} A, above the "
                f"{guidance} A guidance level (rating {servo['rms_limit']} A): watch motor temperature in sustained slow rotation")
    run.step("survey", plateaus=rows, friction={k: v for k, v in fit.items()}, temperature_raw=[float(j["feedback"]["temperature"][0]),
             float(j["feedback"]["temperature"][-1])], capability_warning=bool(heavy))
    return fit, j


def with_friction(servo, fit):
    s = dict(servo)
    s.update(coulomb_positive=round(fit["positive"]["coulomb"], 4), coulomb_negative=round(fit["negative"]["coulomb"], 4),
             stribeck_positive=round(fit["positive"]["stribeck"], 4), stribeck_negative=round(fit["negative"]["stribeck"], 4),
             stribeck_speed=round(0.5 * (fit["positive"]["stribeck_speed"] + fit["negative"]["stribeck_speed"]), 4),
             viscous=round(0.5 * (fit["positive"]["viscous"] + fit["negative"]["viscous"]), 4),
             creep_drop=round(fit["creep_drop"], 4), creep_speed=round(fit["creep_speed"], 5),
             friction_map_positive=fit["map_positive"], friction_map_negative=fit["map_negative"])
    return s


def yaw_crosstalk(run, servo):
    script = manifests.crosstalk_scan()
    table, _ = manifests.samples(script)
    probe = dict(manifests.CROSSTALK_PROBE, begin_s=2.0, duration_s=table[-1]["time_s"] - 4.0)
    j = run.session("crosstalk", manifests.yaw(f"{run.id}-crosstalk", servo, script, excitation=probe))
    complete(j, "crosstalk")
    rows = identify.crosstalk_windows(j, probe["begin_s"], probe["duration_s"], probe["f0_hz"], window_s=0.25)
    half = len(rows) // 2
    return rows, half, probe["f0_hz"], j


def crosstalk_from(rows, half, frequency, inertia, run):
    delay, table, rms, bins = identify.crosstalk_table(rows, frequency, inertia=inertia)
    _, t1, _, _ = identify.crosstalk_table(rows[:half], frequency, inertia=inertia)
    _, t2, _, _ = identify.crosstalk_table(rows[half:], frequency, inertia=inertia)
    difference = np.abs(np.asarray(t1) - np.asarray(t2))
    disagreement = float(np.sqrt(np.mean(difference ** 2)))
    by_angle = np.convolve(np.concatenate([difference[-5:], difference, difference[:5]]), np.ones(11) / 11, mode="same")[5:-5]
    uncertainty = max(0.0003, disagreement)
    run.log(f"    crosstalk: |g| max {np.max(np.abs(table)) * 1e3:.2f} mrad/A, delay {delay * 1e3:.2f} ms, "
            f"one-way scans differ by {disagreement * 1e3:.2f} mrad/A rms ({len(rows)} windows)")
    return {"delay_s": round(delay, 6), "table": [round(float(g), 7) for g in table], "uncertainty": round(uncertainty, 6),
            "fit_rms": round(rms, 6), "scan_disagreement": round(disagreement, 6), "windows": len(rows), "probe_hz": frequency,
            "disagreement_by_angle": [round(float(x), 7) for x in by_angle]}


def yaw_inertia(run, servo, crosstalk):
    script = manifests.inertia_sweep()
    sweep = dict(manifests.INERTIA_SWEEP, begin_s=3.0, duration_s=10.5)
    j = run.session("inertia", manifests.yaw(f"{run.id}-inertia", servo, script, excitation=sweep, vmax=35.0))
    complete(j, "inertia")
    points = identify.inertia_response(j, sweep, crosstalk["table"], crosstalk["delay_s"])
    a, d, r, rms = identify.inertia_from_frf(points, residual_bound=crosstalk["uncertainty"])
    coherent = sum(1 for p in points if p[2] >= 0.5)
    run.log(f"    inertia {a:.4f} A*s^2/rad, loop delay {d * 1e3:.2f} ms, residual crosstalk {r * 1e3:+.2f} mrad/A, "
            f"fit {rms:.2f} ({coherent}/{len(points)} coherent bands)")
    run.step("inertia", inertia=a, loop_delay_s=d, residual_crosstalk=r, fit_rms=rms,
             bands=[(p[0], abs(p[1]), math.degrees(np.angle(p[1])), p[2]) for p in points])
    return a, d, j


def yaw_ladder(run, servo_base, identified, dsg, rules, position):
    """Station gain ladder at the weakest angles; one session per angle (the guard ends a session)."""
    if dsg.get("model_adequate", True):
        wns = [round(dsg["boundary_wn_sim"] * s, 2) for s in (0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3)]
    else:  # no model guidance: a fixed geometric range covering every plant seen so far
        wns = [round(25.0 * 1.2 ** k, 2) for k in range(7)]
    angles = design.ladder_angles(dsg, identified["crosstalk"])
    onsets = []
    for n, angle in enumerate(angles):
        move = math.remainder(angle - position, 2 * math.pi)
        script, begins = manifests.ladder([move], len(wns))
        schedule = [{"begin_s": 0.0, **design.gains(identified["inertia"], wns[0], dsg["zeta"], dsg["integral_rate"])}]
        schedule += [{"begin_s": b, **design.gains(identified["inertia"], wn, dsg["zeta"], dsg["integral_rate"])} for b, wn in zip(begins, wns)]
        servo = design.servo(servo_base, identified, dict(dsg, wn=wns[0]))
        j = run.session(f"ladder{n}", manifests.yaw(f"{run.id}-ladder{n}", servo, script, gain_schedule=schedule,
                                                     oscillation_limit=rules["oscillation_limit_A"]))
        position = last_yaw_angle(j) if "feedback" in j else position
        detail = (j.get("footer") or {}).get("detail", "")
        end = j["cycles"]["t"][-1] if "cycles" in j else 0.0
        onset = None
        if "oscillation" in detail or "speed limit" in detail:  # both end a growing limit cycle
            k = max([i for i, b in enumerate(begins) if b <= end] or [0])
            onset = wns[k]
            onsets.append(onset)
            run.log(f"    angle {math.degrees(angle):.0f} deg: oscillation at wn {wns[k]:.1f} (step {k}), quiet below")
        else:
            complete(j, f"ladder at {math.degrees(angle):.0f} deg")
            run.log(f"    angle {math.degrees(angle):.0f} deg: quiet up to wn {wns[-1]:.1f}")
        run.step(f"ladder{n}", angle_deg=math.degrees(angle), wn_steps=wns, onset_wn=onset, footer=j.get("footer"))
    return (min(onsets) if onsets else None), wns[-1], position


def final_wn(dsg, onset, top, rules):
    """The commissioned gain: the station ladder's onset (or its top step) / station_onset_margin,
    never above the robust design wn. Returns (wn, rule text)."""
    cap = dsg["wn"] if dsg.get("model_adequate", True) and dsg.get("wn") else float("inf")
    edge, what = (onset, f"station onset {onset:.1f}") if onset else (top, f"no onset up to {top:.1f}: top")
    wn = min(edge / rules["station_onset_margin"], cap)
    return wn, f"min({what} / {rules['station_onset_margin']}, robust design {cap:.1f})"


CIRCLE_MARGIN = 1.1  # the full circle runs at this multiple of the chosen gain


def yaw_circle_check(run, base, identified, dsg, position, attempts=3):
    """The ladder tested two angles; the chosen gain must have margin at every angle. A full
    circle each way at CIRCLE_MARGIN x the gain must finish without the oscillation guard
    tripping; otherwise the gain drops by 10% and the circle repeats."""
    wn = dsg["wn"]
    for attempt in range(attempts):
        servo = design.servo(base, identified, dict(dsg, wn=wn * CIRCLE_MARGIN))
        j = run.session(f"circle{attempt}", manifests.yaw(f"{run.id}-circle{attempt}", servo, "circle"))
        position = last_yaw_angle(j) if "feedback" in j else position
        footer = j.get("footer") or {}
        quiet = footer.get("status") == "COMPLETE"
        unstable = "oscillation" in footer.get("detail", "") or "speed limit" in footer.get("detail", "")
        if not quiet and not unstable:
            raise Failed(f"circle check: session ended {footer.get('status')}: {footer.get('detail')}")
        fast = osc_fast_rms(j["cycles"]["t"], j["cycles"]["u"]) if "cycles" in j else float("nan")
        run.log(f"    circle at {CIRCLE_MARGIN} x wn {wn:.1f}: {'quiet' if quiet else 'guard tripped'} (fast current RMS peak {fast:.3f} A)")
        run.step(f"circle{attempt}", wn=wn, tested_wn=wn * CIRCLE_MARGIN, fast_rms_peak=fast, quiet=quiet, footer=footer)
        if quiet:
            return wn, position
        wn *= 0.9
    raise Failed(f"no gain with margin all the way round after {attempts} circle checks")


def yaw_validate(run, servo, identified, passes=2, circle=True, position=None, predict=True):
    results = []
    scripts = ["usecase"] * passes + (["circle"] if circle else [])
    for n, name in enumerate(scripts):
        m = manifests.yaw(f"{run.id}-{name}{n}", servo, name)
        j = run.session(f"{name}{n}", m)
        rows, extra = score.yaw(j, identified["crosstalk"]["table"], identified["crosstalk"]["delay_s"]) if "cycles" in j else ([], {})
        fails = score.gate_yaw(rows, extra) if name == "usecase" else []
        conformance = score.conformance_yaw(rows) if name == "usecase" else []
        status = (j.get("footer") or {}).get("status")
        if status != "COMPLETE":
            fails.append(f"session {status}: {(j.get('footer') or {}).get('detail')}")
        if name == "circle" and "cycles" in j:
            fast = osc_fast_rms(j["cycles"]["t"], j["cycles"]["u"])
            if fast > 0.2:
                fails.append(f"circle: fast current RMS {fast:.2f} A")
        summ = score.summary(rows)
        run.log(f"    {name}{n}: {json.dumps(summ)} stalls {extra.get('stalls')} -> {'ACCEPTED' if not fails else 'NOT ACCEPTED: ' + '; '.join(fails)}"
                + (f"; ACCURACY BELOW THE CALIBRATED LIMITS: {'; '.join(conformance)}" if conformance else ""))
        entry = {"script": name, "summary": summ, "extra": extra, "fails": fails, "conformance": conformance, "rows": rows}
        if predict and name == "usecase" and n == 0:
            entry["predicted"] = predicted_usecase(m, identified, position)
            run.log(f"    model predicted: {json.dumps(entry['predicted'])}")
        results.append(entry)
        if "feedback" in j:
            position = last_yaw_angle(j)
    run.step("validate", results=results)
    return results, position


def osc_fast_rms(t, u):
    ema, ms, peak = float(u[0]), 0.0, 0.0
    for k in range(1, len(t)):
        dt = max(t[k] - t[k - 1], 1e-4)
        ema += (u[k] - ema) * (1 - math.exp(-dt / 0.015))
        fast = u[k] - ema
        ms += (fast * fast - ms) * (1 - math.exp(-dt / 0.25))
        peak = max(peak, ms)
    return math.sqrt(peak)


def predicted_usecase(manifest, identified, position):
    """Model prediction (before looking at the measured run) of the same manifest."""
    r, status = sim.yaw(manifest["servo_parameters"], design.plant(identified), manifest["reference_samples"],
                        start=position or 0.0, hold_after=manifest["servo_hold_after_s"])
    res = {"t": r["t"], "qr": r["qr"], "vr": r["vr"], "q": r["q_hat"], "v": r["v_hat"], "u": r["u"],
           "true_t": r["t"], "true_q": r["q_true"], "true_v": r["v_true"]}
    rows = __import__("usecase").metrics(res, score.segments(manifest))
    return score.summary(rows) | {"status": status}


def commission_yaw(args):
    run = Run(args, "yaw")
    prior = json.load(open(CONFIG / "yaw_prior.json", encoding="utf-8"))
    rules = prior["design_rules"]
    current = json.load(open(CONFIG / "yaw_servo.json", encoding="utf-8")) if args.update else None
    run.log(f"yaw commissioning {run.id} ({'update from the current asset' if args.update else 'from the prior: nothing assumed'}); "
            "the pitch stays disabled and rests where the last pitch session parked it (commission pitch first)")
    try:
        base = prior["servo_parameters"]
        fit, j = yaw_survey(run, base)
        position = last_yaw_angle(j)
        servo = with_friction(base, fit)
        if args.update and current.get("identified", {}).get("crosstalk"):
            crosstalk = current["identified"]["crosstalk"]
            run.log("    crosstalk: reusing the asset's table (checked by the inertia fit's residual below)")
        else:
            rows, half, frequency, j = yaw_crosstalk(run, servo)
            position = last_yaw_angle(j)
            crosstalk = crosstalk_from(rows, half, frequency, None, run)
        servo = dict(servo, crosstalk_delay_s=crosstalk["delay_s"],
                     crosstalk_map=[g - rules["crosstalk_bias_candidates_rad_per_A"][1] for g in crosstalk["table"]])
        inertia, loop_delay, j = yaw_inertia(run, servo, crosstalk)
        position = last_yaw_angle(j)
        if not (args.update and current.get("identified", {}).get("crosstalk")):
            crosstalk = crosstalk_from(rows, half, frequency, inertia, run)  # remove the rigid-body share of the probe response
        run.step("crosstalk", **{k: v for k, v in crosstalk.items() if k != "table"})
        posture = [r["pitch_posture_rad"] for r in run.report.get("idle_axis", []) if "pitch_posture_rad" in r]
        identified = {"inertia": round(inertia, 5), "loop_delay_s": round(loop_delay, 6),
                      # operating point (ADR-002.2 D2): yaw inertia and friction depend on where the pitch sits
                      "pitch_posture_rad": posture[0] if posture else None,
                      # The sweep's delay = actuation + 1.2 ms (current lag, encoder, transmit timing; calibrated
                      # against known delays in simulation, see tests/test_pipeline.py).
                      "actuation_delay_s": round(min(0.003, max(0.0, loop_delay - DELAY_OFFSET_S)), 6),
                      "friction": fit, "crosstalk": crosstalk}
        run.log("design (simulation on the identified plant)")
        dsg = design.design_yaw(identified, base, rules, start=position, log=run.log)
        if dsg["model_adequate"]:
            run.log(f"    sim boundary wn {dsg['boundary_wn_sim']:.1f} rad/s with bias {dsg['bias'] * 1e3:.1f} mrad/A")
        run.step("design", **dsg)
        onset, top, position = yaw_ladder(run, base, identified, dsg, rules, position)
        # The station decides, capped by the robust design (the simulated boundary with the crosstalk
        # error bound, times the design margin): the station's onset is measured with today's crosstalk,
        # the cap keeps the margin for the crosstalk of another day (design.py).
        wn, rule = final_wn(dsg, onset, top, rules)
        dsg.update(wn=round(wn, 2), station_onset_wn=onset, final_rule=rule)
        run.log(f"    ladder wn {wn:.1f} rad/s ({rule}) -> {design.gains(identified['inertia'], wn, dsg['zeta'], dsg['integral_rate'])}")
        wn, position = yaw_circle_check(run, base, identified, dsg, position)
        dsg["wn"] = round(wn, 2)
        servo = design.servo(base, identified, dsg)
        results, position = yaw_validate(run, servo, identified, position=position, circle=False)
        accepted = all(not r["fails"] for r in results)
        asset = yaw_asset(run, prior, identified, dsg, servo, results, accepted)
        finish(run, args, asset, accepted, "yaw_servo.json")
    except Failed as e:
        run.log(f"STOPPED: {e}")
        run.step("stopped", reason=str(e))
        sys.exit(1)


def yaw_asset(run, prior, identified, dsg, servo, results, accepted):
    return {"schema": "ota.servo-asset/2", "axis": "yaw", "hardware_signature": prior["hardware_signature"],
            "provenance": {"commissioned": run.report["started"], "run": run.id, "station": run.report["station"],
                           "tool": "Firmware/tools/servo_commission/commission.py", "accepted": accepted},
            "identified": identified, "design": dsg,
            "plant": design.plant(identified),
            "validation": [{"script": r["script"], "summary": r["summary"], "extra": r["extra"], "fails": r["fails"],
                            "conformance": r.get("conformance", []), "predicted": r.get("predicted")} for r in results],
            "servo_parameters": servo}


def finish(run, args, asset, accepted, name):
    (run.out / name).write_text(json.dumps(asset, indent=1), encoding="utf-8")
    run.report["accepted"] = accepted
    run.step("finished", asset=str(run.out / name), accepted=accepted)
    if accepted and not args.no_write and not args.sim:
        (CONFIG / name).write_text(json.dumps(asset, indent=1) + "\n", encoding="utf-8")
        run.log(f"ACCEPTED: wrote {CONFIG / name}")
    else:
        run.log(f"{'ACCEPTED' if accepted else 'NOT ACCEPTED'}: asset left in {run.out / name}")
    write_markdown(run)


def write_markdown(run):
    lines = [f"# Servo commissioning {run.id}", "", f"Station: {run.report['station']}. Axis: {run.axis}.", ""]
    for s in run.report["steps"]:
        lines.append(f"## {s['step']}")
        for k, v in s.items():
            if k in ("step", "rows", "results", "plateaus", "bands"):
                continue
            lines.append(f"- {k}: {json.dumps(v, default=float)[:400]}")
        if s["step"] == "validate":
            for r in s["results"]:
                lines.append(f"- {r['script']}: {json.dumps(r['summary'])} {'ACCEPTED' if not r['fails'] else 'fails: ' + '; '.join(r['fails'])}")
                if r.get("predicted"):
                    lines.append(f"  - model predicted: {json.dumps(r['predicted'])}")
        lines.append("")
    (run.out / "report.md").write_text("\n".join(lines), encoding="utf-8")


# -------------------------------------------------------------------------- pitch
PITCH_FAST_LIMIT = 0.05  # rad/s: fast SpdRef RMS; last night's clean kp 15-50 runs peaked at 0.016-0.022


def pitch_session(run, name, settings, trial, script, **kw):
    return run.session(name, manifests.pitch(f"{run.id}-{name}", settings, trial, script, **kw))


def pitch_onset_rms(t, cmd):
    return osc_fast_rms(t, cmd)


def commission_pitch(args):
    run = Run(args, "pitch")
    prior = json.load(open(CONFIG / "pitch_prior.json", encoding="utf-8"))
    rules = prior["design_rules"]
    settings, trial = prior["native_settings"], dict(prior["servo_trial"])
    run.log(f"pitch commissioning {run.id} ({'homing first in every session' if trial.get('home_first') else 'window from production homing'})")
    try:
        j = pitch_session(run, "identify", settings, trial, "pitch_identify", excitation=manifests.PITCH_SWEEP)
        complete(j, "pitch identification")
        w = j["window"] or {"window_min_rad": trial["window_min_rad"], "window_max_rad": trial["window_max_rad"],
                            "center_rad": trial["center_rad"], "endpoint_low_rad": trial["window_min_rad"] - trial["window_margin_rad"],
                            "endpoint_high_rad": trial["window_max_rad"] + trial["window_margin_rad"]}
        run.log(f"    endstops {w['endpoint_low_rad']:.3f} .. {w['endpoint_high_rad']:.3f} rad -> window "
                f"{w['window_min_rad']:.3f} .. {w['window_max_rad']:.3f}, centre {w['center_rad']:.3f}")
        sweep = manifests.PITCH_SWEEP
        gain, delay, tau, rms, points = identify.pitch_speed_loop(j, sweep["begin_s"], sweep["duration_s"], sweep["f0_hz"], sweep["f1_hz"])
        run.log(f"    speed loop: gain {gain:.2f}, delay {delay * 1e3:.1f} ms, lag {tau * 1e3:.1f} ms (fit {rms:.2f}, {len(points)} bands)")
        run.step("identify", window=w, speed_gain=gain, speed_delay_s=delay, speed_tau_s=tau, fit_rms=rms)
        g = design.pitch_gains(delay, tau, rules, gain)
        run.log(f"    design: kp {g['kp_per_s']} /s, ki {g['ki_per_s2']} /s^2 (phase margin {g['phase_margin_deg']} deg)")
        run.step("design", **g)
        scales = [0.6, 0.8, 1.0, 1.25, 1.5]
        script, begins = manifests.pitch_ladder(len(scales))
        schedule = [{"begin_s": b, "kp": round(g["kp_per_s"] * s, 2), "ki": round(g["ki_per_s2"] * s * s, 2)} for b, s in zip(begins, scales)]
        lt = dict(trial, kp_per_s=schedule[0]["kp"], ki_per_s2=schedule[0]["ki"])
        j = pitch_session(run, "ladder", settings, lt, script, gain_schedule=schedule)
        fast = []
        for b, e in zip(begins, begins[1:] + [begins[-1] + manifests.PITCH_LADDER_STEP_S]):
            k = (j["trial"]["t"] >= b + 0.5) & (j["trial"]["t"] < e)
            fast.append(pitch_onset_rms(j["trial"]["t"][k], j["trial"]["cmd"][k]) if k.sum() > 100 else float("nan"))
        quiet = [s for s, f in zip(scales, fast) if f < PITCH_FAST_LIMIT]
        run.log("    ladder fast SpdRef RMS: " + ", ".join(f"kp {g['kp_per_s'] * s:.0f}: {f:.3f}" for s, f in zip(scales, fast)))
        onset = next((s for s, f in zip(scales, fast) if not f < PITCH_FAST_LIMIT), None)
        scale = 1.0 if onset is None else min(1.0, onset / rules["station_onset_margin"])
        final = {"kp_per_s": round(g["kp_per_s"] * scale, 2), "ki_per_s2": round(g["ki_per_s2"] * scale * scale, 2)}
        run.step("ladder", scales=scales, fast_rms=fast, onset_scale=onset, final=final, footer=j.get("footer"))
        run.log(f"    final kp {final['kp_per_s']} ki {final['ki_per_s2']}")
        trial.update(final)
        results = []
        for n in range(2):
            j = pitch_session(run, f"usecase{n}", settings, trial, "pitch_usecase")
            rows, extra = score.pitch(j) if "trial" in j else ([], {})
            fails = score.gate_pitch(rows, extra)
            advisory = score.accept_pitch(rows, extra)
            if (j.get("footer") or {}).get("status") != "COMPLETE":
                fails.append(f"session {(j.get('footer') or {}).get('detail')}")
            summ = score.summary(rows)
            run.log(f"    usecase{n}: {json.dumps(summ)} -> {'ACCEPTED' if not fails else 'NOT ACCEPTED: ' + '; '.join(fails)}")
            results.append({"script": "pitch_usecase", "summary": summ, "fails": fails, "advisory": advisory, "rows": rows})
        run.step("validate", results=results)
        accepted = all(not r["fails"] for r in results)
        asset = {"schema": "ota.servo-asset/2", "axis": "pitch", "hardware_signature": prior["hardware_signature"],
                 "provenance": {"commissioned": run.report["started"], "run": run.id, "station": run.report["station"],
                                "tool": "Firmware/tools/servo_commission/commission.py", "accepted": accepted},
                 "identified": {"window": w, "speed_gain": gain, "speed_delay_s": delay, "speed_tau_s": tau},
                 "design": dict(g, ladder_onset_scale=onset, final=final),
                 "plant": {"speed_gain": round(gain, 3), "speed_delay_s": round(delay, 5), "speed_tau_s": round(tau, 5), "reply_delay_s": 0.0005,
                           "position_quantum_rad": prior["hardware_signature"]["position_quantum_rad"],
                           "endpoint_low_rad": w["endpoint_low_rad"], "endpoint_high_rad": w["endpoint_high_rad"]},
                 "validation": [{"script": r["script"], "summary": r["summary"], "fails": r["fails"]} for r in results],
                 "native_settings": settings, "servo_trial": trial}
        finish(run, args, asset, accepted, "pitch_servo.json")
    except Failed as e:
        run.log(f"STOPPED: {e}")
        run.step("stopped", reason=str(e))
        sys.exit(1)


# ------------------------------------------------------------------ check/validate
def check(args):
    """Is the current asset still right? A short probe compared with the model's prediction."""
    axis = args.axis
    run = Run(args, axis)
    asset = load_asset(args)
    if axis == "yaw":
        m = manifests.yaw(f"{run.id}-probe", asset["servo_parameters"], "probe")
        j = run.session("probe", m)
        rows, extra = score.yaw(j, asset["identified"]["crosstalk"]["table"], asset["identified"]["crosstalk"]["delay_s"])
        samples = identify.friction_plateaus(j)
        friction = identify.friction_by_speed(samples)
        fit = asset["identified"]["friction"]

        def expected(r):
            p = fit["positive" if r["direction"] > 0 else "negative"]
            return identify.servo_friction_curve(r["speed"], p["coulomb"], p["stribeck"], p["stribeck_speed"], p["viscous"],
                                                 fit["creep_drop"], fit["creep_speed"])
        drift = [r["current"] / expected(r) - 1 for r in friction]
        fails = score.gate_yaw(rows, extra)
        worst = max((abs(d) for d in drift), default=0.0)
        verdict = "REUSE_EXACT" if not fails and worst < 0.25 else "UPDATE_PARAMETERS"
        run.log(f"    friction against the asset: {', '.join(f'{d:+.0%}' for d in drift)}; acceptance fails: {fails or 'none'}")
    else:
        j = pitch_session(run, "probe", asset["native_settings"], asset["servo_trial"], "pitch_usecase")
        rows, extra = score.pitch(j)
        fails = score.gate_pitch(rows, extra) + ([] if (j.get("footer") or {}).get("status") == "COMPLETE" else ["session failed"])
        verdict = "REUSE_EXACT" if not fails else "UPDATE_PARAMETERS"
    run.log(f"{verdict}" + ("" if verdict == "REUSE_EXACT" else f": run `commission.py {axis} --update`"))
    run.step("check", verdict=verdict, fails=fails, summary=score.summary(rows))
    write_markdown(run)


def load_asset(args):
    return json.load(open(args.asset or CONFIG / f"{args.axis}_servo.json", encoding="utf-8"))


def validate(args):
    run = Run(args, args.axis)
    asset = load_asset(args)
    if args.axis == "yaw":
        results, _ = yaw_validate(run, asset["servo_parameters"], asset["identified"], circle=False, predict=False)
    else:
        results = []
        for n in range(2):
            j = pitch_session(run, f"usecase{n}", asset["native_settings"], asset["servo_trial"], "pitch_usecase")
            rows, extra = score.pitch(j)
            fails = score.gate_pitch(rows, extra)
            advisory = score.accept_pitch(rows, extra)
            run.log(f"    usecase{n}: {json.dumps(score.summary(rows))} -> {'ACCEPTED' if not fails else '; '.join(fails)}"
                    + (f" (advisory: {'; '.join(advisory)})" if advisory else ""))
            results.append({"summary": score.summary(rows), "fails": fails, "advisory": advisory})
        run.step("validate", results=results)
    write_markdown(run)


def rescore(args):
    """Re-evaluate a recorded yaw run's validation journals with the current scorer (no station).
    Deterministic: the same journals always give the same verdict. Writes the asset when accepted."""
    out = Path(args.run_dir)
    asset = json.load(open(out / "yaw_servo.json", encoding="utf-8"))
    x = asset["identified"]["crosstalk"]
    results, accepted = [], True
    for path in sorted(out.glob("*-usecase*/yaw-control.jsonl")):
        j = journal.yaw(path)
        rows, extra = score.yaw(j, x["table"], x["delay_s"])
        fails = score.gate_yaw(rows, extra)
        if (j.get("footer") or {}).get("status") != "COMPLETE":
            fails.append(f"session {(j.get('footer') or {}).get('detail')}")
        conformance = score.conformance_yaw(rows)
        accepted &= not fails
        print(f"{path.parent.name}: {json.dumps(score.summary(rows))} stalls {extra.get('stalls')} -> "
              f"{'ACCEPTED' if not fails else 'NOT ACCEPTED: ' + '; '.join(fails)}"
              + (f"; accuracy below the calibrated limits: {'; '.join(conformance)}" if conformance else ""))
        results.append({"script": "usecase", "summary": score.summary(rows), "extra": extra, "fails": fails, "conformance": conformance})
    accepted &= len(results) >= 2
    old = {v["script"] + str(i): v.get("predicted") for i, v in enumerate(asset.get("validation", []))}
    asset["validation"] = [dict(r, predicted=old.get(f"usecase{i}")) for i, r in enumerate(results)]
    asset["provenance"]["accepted"] = accepted
    asset["provenance"]["rescored"] = "gates: tracking and stalls; conformance: config/servo/yaw_accuracy.json"
    (out / "yaw_servo.json").write_text(json.dumps(asset, indent=1), encoding="utf-8")
    if accepted and not args.no_write:
        (CONFIG / "yaw_servo.json").write_text(json.dumps(asset, indent=1) + "\n", encoding="utf-8")
        print(f"ACCEPTED: wrote {CONFIG / 'yaw_servo.json'}")
    else:
        print("ACCEPTED" if accepted else "NOT ACCEPTED")


# --------------------------------------------------------------- accuracy (ADR-003)
CALIBRATION_ANGLES = 4  # positions 90 deg apart, each with one FF+FB and one FB-only pass


def calibration_script(goto):
    return ([(7.0, "step", goto, "goto")] if goto else []) + manifests.usecase.script()


def asset_validation_journals(asset):
    """The use-case journals the asset was accepted on (its commissioning run's validation)."""
    for report in sorted((REPO / "run/servo-commission").glob("*/report.json")):
        if json.load(open(report, encoding="utf-8")).get("run") == asset["provenance"]["run"]:
            return sorted(report.parent.glob("*-usecase*/yaw-control.jsonl"))
    return []


def px_per_deg():
    """Tracker-frame scale from the measured intrinsics (calibration/camera_intrinsics.yaml)."""
    text = (REPO / "Firmware/calibration/camera_intrinsics.yaml").read_text(encoding="utf-8")
    fx = float(next(line.split("=")[1] for line in text.splitlines() if line.startswith("fx=")))
    return fx * DEG, fx


def calibrate(args):
    """Yaw accuracy limits from the measured performance of the FF+FB servo (owner ruling
    2026-10-02), and ADR-003 sec. 7B: the same references with motor feedforward removed.

    Fixed campaign: at 4 angles 90 deg apart, one use-case pass with the asset and one with
    accuracy.motor_feedback_only(asset), in alternating order (ABBA...). Every FF+FB pass must
    complete and meet the tracking gates; the limits are accuracy.limits_from() over these
    passes plus the passes the asset was accepted on. No pass is repeated or dropped: --resume
    DIR scores the passes already recorded there (same plan, same angles) and runs the rest."""
    if args.resume:
        args.out = args.resume
    run = Run(args, "yaw")
    asset = load_asset(args)
    x = asset["identified"]["crosstalk"]
    variants = {"ff_fb": asset["servo_parameters"], "fb_only": accuracy.motor_feedback_only(asset["servo_parameters"])}
    passes = {"ff_fb": [], "fb_only": []}
    for k in range(CALIBRATION_ANGLES):
        for n, name in enumerate(("ff_fb", "fb_only") if k % 2 == 0 else ("fb_only", "ff_fb")):
            goto = 90 * DEG if k and not n else 0.0
            done = [p for p in sorted(run.out.glob(f"*-{name}{k}/yaw-control.jsonl"))
                    if (journal.yaw(p).get("footer") or {}).get("status") == "COMPLETE"]
            if done:
                run.log(f"[recorded] {name}{k}: {done[-1].parent.name}")
                j = journal.yaw(done[-1])
            else:
                j = run.session(f"{name}{k}", manifests.yaw(f"{run.id}-{name}{k}", variants[name], calibration_script(goto)))
            status = (j.get("footer") or {}).get("status")
            rows, extra = score.yaw(j, x["table"], x["delay_s"]) if "cycles" in j else ([], {})
            fails = score.gate_yaw(rows, extra) + ([] if status == "COMPLETE" else [f"session {status}"])
            metrics = accuracy.pass_metrics(rows)
            run.log(f"    {name}{k}: {json.dumps({m: round(v, 3) for m, v in metrics.items()})} stalls {extra.get('stalls')}"
                    + (f" -- {'; '.join(fails)}" if fails else ""))
            # Valid for the comparison: the session completed and the axis really tracked. Stall
            # recoveries are an outcome to compare, not a reason to leave a pass out.
            valid = status == "COMPLETE" and not score.tracking_fails(rows)
            passes[name].append({"pass": f"{name}{k}", "angle_index": k, "metrics": metrics, "fails": fails, "valid": valid,
                                 "stalls": extra.get("stalls"), "summary": score.summary(rows)})
            run.step("calibration_pass", **passes[name][-1])
            if name == "ff_fb" and fails:
                write_markdown(run)
                raise Failed(f"calibration pass {name}{k} failed the tracking gates ({'; '.join(fails)}): no limits written; "
                             "the servo itself needs `commission.py check yaw` first")
    prior = []
    for path in asset_validation_journals(asset):
        rows, extra = score.yaw(journal.yaw(path), x["table"], x["delay_s"])
        prior.append({"pass": path.parent.name, "metrics": accuracy.pass_metrics(rows), "stalls": extra.get("stalls")})
    ff = passes["ff_fb"] + prior
    limits = accuracy.limits_from([p["metrics"] for p in ff])
    scale, fx = px_per_deg()
    ff_valid = [p for p in passes["ff_fb"] if p["valid"]]
    fb_valid = [p for p in passes["fb_only"] if p["valid"]]
    comparison = None
    if ff_valid and fb_valid:
        comparison = accuracy.compare([dict(p["metrics"], stalls=p["stalls"]) for p in ff_valid],
                                      [dict(p["metrics"], stalls=p["stalls"]) for p in fb_valid])
    result = {
        "schema": "ota.servo-accuracy/1", "axis": "yaw",
        "status": "OWNER_RULING_MEASURED_CAPABILITY",
        "ruling": "Owner 2026-10-02: calibrate the yaw accuracy limits from the real tracking performance of the "
                  "feedforward + feedback controller (the ADR-003 photography spec had no framing budget).",
        "asset_run": asset["provenance"]["run"], "calibration_run": run.id,
        "rule": f"limit = ceil({accuracy.MARGIN} x worst pass, grid) over {len(ff)} FF+FB passes "
                f"({CALIBRATION_ANGLES} angles 90 deg apart + the asset's validation passes)",
        "error": "servo error q_ref - q_true (encoder with the measured crosstalk removed), use-case script of usecase.py",
        "limits": limits,
        "units": {m: u for m, (_, u, _) in accuracy.METRICS.items()},
        "definitions": {m: d for m, (_, _, d) in accuracy.METRICS.items()},
        "tracker_frame": {"camera": "IMX500 wide, 1920x1080", "fx_px_per_rad": fx, "px_per_deg": round(scale, 2),
                          "limits_px": accuracy.pixels(limits, scale)},
        "passes": [{"pass": p["pass"], "metrics": {m: round(v, 4) for m, v in p["metrics"].items()}, "stalls": p["stalls"]}
                   for p in ff],
        "motor_feedforward_comparison": {
            "what": "ADR-003 sec. 7B: identical references and gains; fb_only has every plant feedforward term zero",
            "fb_only_passes": [{"pass": p["pass"], "metrics": {m: round(v, 4) for m, v in p["metrics"].items()},
                                "stalls": p["stalls"], "fails": p["fails"]} for p in passes["fb_only"]],
            "paired_means": comparison},
    }
    run.step("limits", limits=limits, limits_px=result["tracker_frame"]["limits_px"], comparison=comparison)
    (run.out / "yaw_accuracy.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    run.log(f"limits: {json.dumps(limits)}")
    run.log(f"limits in tracker pixels: {json.dumps(result['tracker_frame']['limits_px'])}")
    if comparison:
        run.log("motor FF comparison (FB-only / FF+FB): " + ", ".join(f"{m} {c['fb_over_ff']}" for m, c in comparison.items()))
    if not args.no_write:
        (CONFIG / "yaw_accuracy.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
        run.log(f"wrote {CONFIG / 'yaw_accuracy.json'}")
    write_markdown(run)


def redesign(args):
    """Yaw gains re-derived offline from the asset's identified plant under the current design
    rules (no station). The station onset recorded in the asset still bounds the result; the
    asset's validation entries stay, marked with the gain they were measured at."""
    prior = json.load(open(CONFIG / "yaw_prior.json", encoding="utf-8"))
    rules = prior["design_rules"]
    path = Path(args.asset) if args.asset else CONFIG / "yaw_servo.json"
    asset = json.load(open(path, encoding="utf-8"))
    identified, old = asset["identified"], asset["design"]
    print(f"redesign {path.name}: commissioned wn {old['wn']} rad/s ({old.get('final_rule', '')})")
    dsg = design.design_yaw(identified, prior["servo_parameters"], rules, log=print)
    if not dsg["model_adequate"]:
        print("MODEL INADEQUATE: no offline redesign; commission on the station")
        sys.exit(1)
    onset = old.get("station_onset_wn")
    wn, rule = final_wn(dsg, onset, None, rules) if onset else (dsg["wn"], f"robust design {dsg['wn']:.1f}")
    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    dsg.update(wn=round(wn, 2), station_onset_wn=onset, final_rule=f"{rule} (redesigned offline {stamp})")
    servo = design.servo(prior["servo_parameters"], identified, dsg)
    rows, extra, status = design.predicted_usecase(identified, servo)
    print(f"wn {old['wn']} -> {dsg['wn']} rad/s: {design.gains(identified['inertia'], dsg['wn'], dsg['zeta'], dsg['integral_rate'])}; "
          f"crosstalk error bound {dsg['crosstalk_error_bound'] * 1e3:.2f} mrad/A; predicted use case {status}, "
          f"cost {design.predicted_cost(rows, extra, status):.3f}")
    for v in asset.get("validation", []):
        v.setdefault("measured_at_wn", old["wn"])
    asset["design"] = dsg
    asset["servo_parameters"] = servo
    asset["provenance"]["redesigned"] = {"at": stamp, "previous_wn": old["wn"], "previous_bias": old.get("bias"),
                                         "rule": rule, "tool": "commission.py redesign"}
    out = Path(args.out) if args.out else path
    if args.no_write and not args.out:
        print("--no-write: nothing written")
        return
    out.write_text(json.dumps(asset, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {out}")


def one_session(args):
    run = Run(args, args.axis)
    asset = load_asset(args)
    opts = json.loads(args.opts)
    if args.axis == "yaw":
        j = run.session(args.script, manifests.yaw(f"{run.id}-{args.script}", asset["servo_parameters"], args.script, **opts))
        if "cycles" in j:
            rows, extra = score.yaw(j, asset["identified"]["crosstalk"]["table"], asset["identified"]["crosstalk"]["delay_s"])
            __import__("usecase").show(rows)
    else:
        j = pitch_session(run, args.script, asset["native_settings"], asset["servo_trial"], args.script, **opts)
        if "trial" in j:
            __import__("usecase").show(score.pitch(j)[0])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    for axis in ("yaw", "pitch"):
        c = sub.add_parser(axis)
        g = c.add_mutually_exclusive_group()
        g.add_argument("--from-prior", action="store_true", help="assume nothing (the default)")
        g.add_argument("--update", action="store_true", help="start from the current asset (reuse the crosstalk table)")
    for name in ("check", "validate"):
        c = sub.add_parser(name)
        c.add_argument("axis", choices=("yaw", "pitch"))
        c.add_argument("--asset", help="asset file (default config/servo/<axis>_servo.json)")
    c = sub.add_parser("calibrate", help="yaw accuracy limits from measured FF+FB tracking (accuracy.py)")
    c.add_argument("axis", choices=("yaw",))
    c.add_argument("--asset", help="asset file (default config/servo/yaw_servo.json)")
    c.add_argument("--resume", help="a calibration run directory: score its recorded passes, run the missing ones")
    c = sub.add_parser("rescore", help="re-evaluate a recorded yaw run directory with the current scorer")
    c.add_argument("run_dir")
    c = sub.add_parser("redesign", help="yaw gains re-derived offline from the asset's identified plant (current rules)")
    c.add_argument("axis", choices=("yaw",))
    c.add_argument("--asset", help="asset file (default config/servo/yaw_servo.json)")
    c = sub.add_parser("session")
    c.add_argument("axis", choices=("yaw", "pitch")); c.add_argument("script")
    c.add_argument("--asset", help="asset file (default config/servo/<axis>_servo.json)")
    c.add_argument("--opts", default="{}", help="extra manifest options as JSON")
    for c in sub.choices.values():
        c.add_argument("--sim", help="run against servo-sim with this true plant (JSON) instead of the station")
        c.add_argument("--out", help="output directory (default run/servo-commission/<run id>)")
        c.add_argument("--no-write", action="store_true", help="never write config/servo/*.json")
        c.add_argument("--no-build", action="store_true", help="reuse the current ARM64 build")
    args = p.parse_args()
    if args.command == "yaw":
        commission_yaw(args)
    elif args.command == "pitch":
        commission_pitch(args)
    elif args.command == "check":
        check(args)
    elif args.command == "validate":
        validate(args)
    elif args.command == "rescore":
        rescore(args)
    elif args.command == "calibrate":
        calibrate(args)
    elif args.command == "redesign":
        redesign(args)
    else:
        one_session(args)


if __name__ == "__main__":
    main()
