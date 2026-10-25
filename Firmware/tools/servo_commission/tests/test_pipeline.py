"""Offline proof of the commissioning pipeline: identification recovers known plants,
and the whole pipeline (prior -> asset) is accepted against simulated stations.

    ~/venvs/oattest/bin/python -m pytest Firmware/tools/servo_commission/tests

Needs the native servo-sim (run/adr0022-local/firmware, `make servo-sim`, or OTA_SERVO_SIM). The
simulated stations carry the encoder noise and friction roughness measured on rpi-turret
(sim_cases/*.json), so a sweep that only works on a quiet plant fails here. Takes several minutes.
"""
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import commission  # noqa: E402
import design  # noqa: E402
import identify  # noqa: E402
import manifests  # noqa: E402
import score  # noqa: E402
import sim  # noqa: E402
import station  # noqa: E402

CASES = HERE / "sim_cases"
def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


PRIOR = load(HERE.parents[1] / "config/servo/yaw_prior.json")["servo_parameters"]
RULES = load(HERE.parents[1] / "config/servo/yaw_prior.json")["design_rules"]


def truth(name, inertia=None):
    t = load(CASES / f"{name}.json")
    if inertia:
        t["yaw"]["inertia"] = inertia
    return t


def servo_for(t, table):
    return dict(PRIOR, coulomb_positive=0.35, coulomb_negative=0.33, stribeck_positive=0.35, stribeck_negative=0.35,
                stribeck_speed=0.3, crosstalk_map=[g - 0.002 for g in table], crosstalk_delay_s=0.001)


def drifted(table, rng):
    """The station's table three weeks on (2026-10-24): +1.1 mrad/A overall, 0.5 mrad/A rms by angle."""
    return list(np.asarray(table) + 0.0011 + rng.normal(0, 0.0005, len(table)))


def residual_bound(table):
    return RULES["crosstalk_drift_fraction"] * max(abs(g) for g in table)


def identified_friction(plateau_positive, plateau_negative, inertia=0.26):
    """An identified-plant record whose largest measured plateau per direction is the given level."""
    def side(b):
        return {"coulomb": 0.2 * b, "stribeck": 0.8 * b, "stribeck_speed": 0.2, "viscous": 0.05, "plateau_max": b}
    return {"inertia": inertia, "actuation_delay_s": 0.001,
            "friction": {"positive": side(plateau_positive), "negative": side(plateau_negative), "creep_drop": 0.0,
                         "creep_speed": 0.01, "map_positive": [0.0] * 24, "map_negative": [0.0] * 24},
            "crosstalk": {"delay_s": 0.001, "table": [0.0] * 120, "uncertainty": 0.0005}}


class Design(unittest.TestCase):
    def test_integral_cap_covers_breakaway_and_drift(self):
        """The integral is the servo's whole authority at a stopped reference (friction FF is zero
        there), so its cap is what the station showed: the larger direction's largest survey plateau,
        plus the static excess over the plateaus the station needed to break away
        (breakaway_over_plateau_A: 0.44-0.69 A over plateaus of 0.36 on 2026-10-25 00:38), plus the
        session-to-session drift; a stall is declared only once that much is applied (stall_current =
        cap). Not the fit's coulomb + stribeck: that is an extrapolation to zero speed, and the
        04:03 fit put it at 1.66 A for plateaus of 0.59 (cap 1.2 A, the ceiling)."""
        margin = RULES["breakaway_over_plateau_A"] + RULES["friction_drift_A"]
        dsg = {"wn": 7.0, "zeta": 0.5, "integral_rate": 3.0, "friction_drift": RULES["friction_drift_A"],
               "breakaway_over_plateau": RULES["breakaway_over_plateau_A"], "bias": 0.002}
        payload = design.servo(PRIOR, identified_friction(0.36, 0.34), dsg)
        self.assertAlmostEqual(payload["integral_cap"], 0.36 + margin, places=3)
        self.assertGreater(payload["integral_cap"], 0.69)  # what 00:38 needed
        self.assertEqual(payload["stall_recovery"]["current_A"], payload["integral_cap"])
        degenerate = identified_friction(0.59, 0.46)
        degenerate["friction"]["positive"].update(coulomb=0.13, stribeck=1.53)  # the 04:03 fit
        self.assertAlmostEqual(design.servo(PRIOR, degenerate, dsg)["integral_cap"], 0.59 + margin, places=3)
        bare = design.servo(PRIOR, identified_friction(0.75, 0.70, inertia=0.04), dsg)
        self.assertEqual(bare["integral_cap"], 1.2)  # the thermal ceiling
        self.assertEqual(bare["stall_recovery"]["current_A"], 1.2)
        self.assertEqual(RULES["friction_drift_A"], 0.15)
        self.assertEqual(RULES["breakaway_over_plateau_A"], 0.35)

    def test_friction_fit_is_identifiable(self):
        """The survey plateaus of 2026-10-25 04:03 (left) and 2026-10-24 23:43 (right), as measured.
        The fit's two exponentials (Stribeck, creep) used to trade off when their speeds could meet:
        04:03 came out as stribeck 1.53 A against creep 1.25 A, feedforward 0.08 A above the 1 deg/s
        plateau. With the creep speed held under 0.86 deg/s and each plateau weighted by its spread,
        the model never rises above the measured plateaus by more than their spread, and its static
        level stays near the plateaus."""
        spd = [0.25, 0.5, 1, 2, 5, 10, 20, 40, 65]
        surveys = {
            "04:03": {1: ([0.59, 0.42, 0.40, 0.41, 0.44, 0.28, 0.22, 0.17, 0.18], [0.11, 0.07, 0.06, 0.14, 0.28, 0.24, 0.09, 0.07, 0.29]),
                      -1: ([0.15, 0.31, 0.40, 0.45, 0.46, 0.33, 0.23, 0.18, 0.19], [0.09, 0.06, 0.10, 0.17, 0.24, 0.33, 0.09, 0.10, 0.24])},
            "23:43": {1: ([0.38, 0.37, 0.43, 0.42, 0.42, 0.28, 0.18, 0.19, 0.18], [0.04, 0.11, 0.09, 0.14, 0.24, 0.31, 0.07, 0.07, 0.16]),
                      -1: ([0.12, 0.24, 0.52, 0.61, 0.31, 0.28, 0.21, 0.22, 0.20], [0.04, 0.05, 0.14, 0.16, 0.19, 0.31, 0.09, 0.09, 0.29])}}
        for name, sides in surveys.items():
            rows = [{"direction": d, "speed": v * math.pi / 180, "current": c, "spread": sp, "n": 500}
                    for d, (cur, spr) in sides.items() for v, c, sp in zip(spd, cur, spr)]
            fit = identify.fit_friction(rows)
            self.assertLessEqual(fit["creep_speed"], identify.CREEP_SPEED_MAX + 1e-9, name)
            v = np.linspace(0.1, 70, 2000) * math.pi / 180
            for d, key in ((1, "positive"), (-1, "negative")):
                p = fit[key]
                g = identify.servo_friction_curve(v, p["coulomb"], p["stribeck"], p["stribeck_speed"], p["viscous"], fit["creep_drop"], fit["creep_speed"])
                cur, spr = sides[d]
                self.assertLessEqual(float(np.max(g)), max(cur) + max(spr), f"{name} {key}: model peak {np.max(g):.2f}")
                g1 = float(np.interp(math.pi / 180, v, g))
                self.assertLessEqual(g1, cur[2] + spr[2], f"{name} {key}: model at 1 deg/s {g1:.2f} vs plateau {cur[2]}")
                self.assertLess(p["coulomb"] + p["stribeck"], 1.0, f"{name} {key}: static level {p['coulomb'] + p['stribeck']:.2f}")
                self.assertEqual(p["plateau_max"], max(cur))

    def test_friction_maps_land_where_the_servo_reads_them(self):
        """Servo::friction_at reads map entry i at angle i/24 of a turn, linear in between. The survey's
        map counted each sample towards the entry below it, so it came out half an entry (7.5 deg) late:
        on the 5x case's sticky arc 0.04 A rms of shape error over the turn and 0.14-0.17 A at its edges.
        Counted towards the nearest entry: under 0.025 A rms (measured 0.012-0.014), surveys started off
        the sticky arc. (From on it, at 3.5 rad, the curve also takes that arc's friction: Pipeline.
        test_yaw_payload_5x_as_measured_with_friction_drift.)"""
        t = truth("payload_5x_knee")
        y = t["yaw"]
        q = np.linspace(0, 2 * math.pi, 720, endpoint=False)

        def at(table, angle):
            x = np.mod(angle, 2 * math.pi) / (2 * math.pi) * len(table)
            k0 = np.floor(x).astype(int) % len(table)
            w = x - np.floor(x)
            table = np.asarray(table, float)
            return (1 - w) * table[k0] + w * table[(k0 + 1) % len(table)]

        for start in (1.0, 5.2):
            j = station.SimStation(dict(t, yaw_start_rad=start), log=lambda *a: None).run(
                "yaw", manifests.yaw("t-survey", PRIOR, "survey", vmax=65.0), "survey")
            samples = identify.friction_plateaus(j)
            maps = identify.friction_maps(samples, identify.fit_friction(identify.friction_by_speed(samples)))
            for key in ("positive", "negative"):
                shape = at(maps[key], q) - at(y[f"friction_map_{key}"], q)
                rms = float(np.sqrt(np.mean((shape - np.mean(shape)) ** 2)))
                self.assertLess(rms, 0.025, f"start {start} {key}: map shape error {rms:.3f} A rms")

    def test_slow_ramps_on_the_free_play_mount(self):
        """MEASURE 2026-10-25 (PAYLOAD 10-24): kq 13 / ki 36 (the crosstalk allowance's gain for this
        payload), the fit's creep term, and the drive's free play (0.7 deg: the motor turns alone, against
        the gap's drag, at every force reversal before it takes the load). The 1 deg/s ramps hold the speed gate while the
        bearing sits between the feedforward and 0.1 A under it, and ramp-1 stands and catches up once
        the bearing is 0.25 A stickier than the feedforward: the accuracy campaign's first pass at 09:13
        (ramp-1 1.253 after the bearing had worked in by 0.27 A; a start transient of 1.7 deg, inside the
        engineering gates since). The integral keeps its sign through a reversal (servo.cpp): the rigid plant needed
        the old reversal flip to pass this, the station never did. Stalls stay at 0 (the cap rule)."""
        t = truth("payload_5x_knee")["yaw"]
        t.pop("friction_drift_A", None)
        f = {"positive": {"coulomb": 0.143, "stribeck": 0.4874, "stribeck_speed": 0.08, "viscous": 0.0157, "plateau_max": 0.37},
             "negative": {"coulomb": 0.1983, "stribeck": 0.2915, "stribeck_speed": 0.08, "viscous": 0.0157, "plateau_max": 0.36},
             "creep_drop": 0.4296, "creep_speed": 0.01989, "map_positive": [0.0] * 24, "map_negative": [0.0] * 24}
        identified = {"inertia": 0.27036, "actuation_delay_s": 0.001, "friction": f,
                      "crosstalk": {"delay_s": t["crosstalk_delay_s"], "table": t["crosstalk_map"], "uncertainty": 0.0005}}
        dsg = {"wn": 6.9, "zeta": 0.5, "integral_rate": 3.0, "friction_drift": RULES["friction_drift_A"],
               "breakaway_over_plateau": RULES["breakaway_over_plateau_A"], "bias": 0.002}
        servo = design.servo(PRIOR, identified, dsg)
        self.assertAlmostEqual(servo["kq"], 13.0, delta=0.2)
        m = manifests.yaw("t", servo, "usecase")
        import score, usecase  # noqa: E402
        for stickier, holds in ((-0.1, True), (0.0, True), (0.25, False)):
            plant = dict(t, coulomb_positive=t["coulomb_positive"] + stickier, coulomb_negative=t["coulomb_negative"] + stickier,
                         free_play_rad=math.radians(0.7))
            for start in (1.0, 5.2):
                r, status = sim.yaw(servo, plant, m["reference_samples"], start=start, hold_after=m["servo_hold_after_s"])
                self.assertEqual(status, "COMPLETE")
                res = {"t": r["t"], "qr": r["qr"], "vr": r["vr"], "q": r["q_hat"], "v": r["v_hat"], "u": r["u"],
                       "true_t": r["t"], "true_q": r["q_true"], "true_v": r["v_true"]}
                rows = usecase.metrics(res, score.segments(m))
                ratios = {x["label"]: x["v_ratio"] for x in rows if "v_ratio" in x}
                what = f"bearing {stickier:+.2f} A, start {start}: {ratios}"
                if holds:
                    self.assertTrue(all(0.9 <= v <= 1.1 for v in ratios.values()), what)
                else:
                    self.assertGreater(ratios["ramp-1"], 1.1, what)
                self.assertEqual(int(r["stalls"][-1]), 0, what)

    def test_plant_noise_seed(self):
        """Two seeds are two noise draws; the same seed repeats the run exactly (the proofs run several)."""
        t = truth("payload_5x_knee")["yaw"]
        t.pop("friction_drift_A", None)
        script = manifests.samples([(1.0, "hold", 0, "hold"), (1.5, "ramp", 0.2, "ramp")])[0]
        servo = servo_for({"yaw": t}, t["crosstalk_map"])
        runs = [sim.yaw(servo, dict(t, noise_seed=seed), script)[0]["q_hat"] for seed in (1, 2, 1)]
        self.assertTrue(np.array_equal(runs[0], runs[2]))
        self.assertFalse(np.array_equal(runs[0], runs[1]))


class Identification(unittest.TestCase):
    def inertia_session(self, t, table, guess=None):
        """One inertia session with the sweep chosen for `guess` (the survey's rough inertia); the
        compensation uses `table`. Returns the journal and the excitation."""
        script, sweep = manifests.inertia_sweep_for(guess, PRIOR["kq"])
        j = station.SimStation(t).run("yaw", manifests.yaw("t", servo_for(t, table), script, excitation=sweep, vmax=35.0), "t")
        self.assertEqual(j["footer"]["status"], "COMPLETE", f"inertia {t['yaw']['inertia']}: {j['footer']}")
        return j, sweep

    def identified(self, j, sweep, table):
        points = identify.inertia_response(j, sweep, table, 0.001)
        return identify.inertia_from_frf(points, residual_bound=residual_bound(table))

    def test_inertia_within_fifteen_percent(self):
        """Bare turret (half and whole), 2.5x, the 2026-10-24 payload (0.20) and 5x: the sweep
        chosen from a rough inertia 25% too high still identifies, with a freshly scanned table and
        with the drifted table a --update reuses. (10x: test_prior_gains_at_10x.)"""
        rng = np.random.default_rng(1)
        for name, inertia in (("station_estimate", 0.015), ("station_estimate", None), ("heavy_payload", None),
                              ("payload_5x", 0.2), ("payload_5x", None)):
            t = truth(name, inertia)
            J = t["yaw"]["inertia"]
            scanned = list(np.array(t["yaw"]["crosstalk_map"]) + rng.normal(0, 0.0003, 120))
            j, sweep = self.inertia_session(t, scanned, guess=1.25 * J)
            a = self.identified(j, sweep, scanned)[0]
            self.assertLess(abs(a / J - 1), 0.15, f"{name} J {J}: scanned table -> {a:.4f} (sweep from {sweep['f0_hz']} Hz)")
            old = drifted(t["yaw"]["crosstalk_map"], rng)
            j, sweep = self.inertia_session(t, old, guess=1.25 * J)
            a, _, r, _ = self.identified(j, sweep, old)
            self.assertLess(abs(a / J - 1), 0.15, f"{name} J {J}: drifted table -> {a:.4f} (sweep from {sweep['f0_hz']} Hz)")
            self.assertLess(abs(r + 0.0011), 0.0006, f"{name}: the fit should find the table's +1.1 mrad/A drift, found {r * 1e3:+.2f}")

    def test_sweep_follows_the_plant(self):
        """The bare turret (and no guess) keeps its proven 8-30 Hz, 10.5 s sweep; 2.5x starts at
        5.6 Hz; the 2026-10-24 payload and anything heavier at 3 Hz over 18.3 s."""
        for guess, f0, duration in ((0.04, 8.0, 10.5), (None, 8.0, 10.5), (0.075, 5.6, 13.3), (0.2, 3.0, 18.3), (0.45, 3.0, 18.3)):
            _, sweep = manifests.inertia_sweep_for(guess, PRIOR["kq"])
            self.assertEqual((sweep["f0_hz"], sweep["duration_s"]), (f0, duration), f"inertia {guess}")

    def test_sweep_through_the_drive_free_play(self):
        """The 5x payload with the drive's 0.7 deg of free play, as the station's sweep saw it
        (2026-10-25 10:49: -0.37 A on the motor for a quarter of every cycle, the rigid body read at
        3-9 Hz, J 0.212): the gap's drag keeps the motor on the face it pushes (YawPlantParameters::
        gap_damping). A free gap rattled the motor across it on every dip: 2 coherent bands of 16, no
        inertia. Within 20% (the rigid plant's sweep reads +10% here), from either side of the turn."""
        t = truth("payload_5x_knee")
        t["yaw"].pop("friction_drift_A")
        t["yaw"]["free_play_rad"] = math.radians(0.7)
        table = t["yaw"]["crosstalk_map"]
        for start in (1.0, 3.5):
            t["yaw_start_rad"] = start
            j, sweep = self.inertia_session(t, table, guess=0.18)
            points = identify.inertia_response(j, sweep, table, 0.001)
            coherent = sum(1 for p in points if p[2] >= 0.5)
            self.assertGreaterEqual(coherent, 8, f"start {start}: {coherent}/16 coherent bands")
            inertia = self.identified(j, sweep, table)[0]
            self.assertLess(abs(inertia / t["yaw"]["inertia"] - 1), 0.2, f"start {start}: J {inertia:.4f}")

    def test_the_gap_drag_is_stable_for_a_light_motor(self):
        """The drag is implicit (plant.cpp): with a motor side 20 times lighter, an explicit drag of 12
        diverged within the sweep (HARD_ABORT: yaw speed limit; review 2026-10-25). inertia_session
        asserts the session completes."""
        t = truth("payload_5x_knee")
        t["yaw"].pop("friction_drift_A")
        t["yaw"].update(free_play_rad=math.radians(0.7), motor_inertia=0.0005, gap_damping=12.0)
        self.inertia_session(t, t["yaw"]["crosstalk_map"], guess=0.18)

    @unittest.expectedFailure
    def test_prior_gains_at_10x(self):
        """Known limit (2026-10-24, simulator): at 10x the bare turret's inertia the prior's gains
        (kq 20, kv 0.5: zeta 0.08) limit-cycle through friction, 2.7 A peaks and +-0.7 rad/s about
        the sliding speed, with or without roughness; the survey trips the speed limit. No sweep can
        identify in that state. It is the prior that has to change, not the identification; when it
        does, this test turns into an unexpected success and 10x joins the cases above."""
        t = truth("payload_10x")
        rng = np.random.default_rng(4)
        scanned = list(np.array(t["yaw"]["crosstalk_map"]) + rng.normal(0, 0.0003, 120))
        j, sweep = self.inertia_session(t, scanned, guess=t["yaw"]["inertia"])
        a = self.identified(j, sweep, scanned)[0]
        self.assertLess(abs(a / t["yaw"]["inertia"] - 1), 0.15, f"10x: {a:.4f}")

    def test_inertia_from_speed_changes(self):
        """The survey's rough inertia, which chooses the sweep, is within 30% for a bare turret and a payload."""
        for name in ("station_estimate", "payload_5x"):
            t = truth(name)
            j = station.SimStation(t).run("yaw", manifests.yaw("t", servo_for(t, t["yaw"]["crosstalk_map"]), "survey", vmax=65.0), "t")
            self.assertEqual(j["footer"]["status"], "COMPLETE", j["footer"])
            guess, bins = identify.inertia_from_speed_changes(j)
            self.assertLess(abs(guess / t["yaw"]["inertia"] - 1), 0.3, f"{name}: {guess:.4f} from {bins} bins")

    def test_delay_calibration(self):
        """commission.DELAY_OFFSET_S maps the sweep's delay to the plant's actuation delay."""
        for act in (0.0005, 0.0015):
            t = truth("station_estimate")
            t["yaw"]["actuation_delay_s"] = act
            j, sweep = self.inertia_session(t, t["yaw"]["crosstalk_map"], guess=t["yaw"]["inertia"])
            d = self.identified(j, sweep, t["yaw"]["crosstalk_map"])[1]
            self.assertAlmostEqual(d - commission.DELAY_OFFSET_S, act, delta=0.0003)

    def test_crosstalk_table(self):
        t = truth("station_estimate")
        script = manifests.crosstalk_scan()
        table, _ = manifests.samples(script)
        probe = dict(manifests.CROSSTALK_PROBE, begin_s=2.0, duration_s=table[-1]["time_s"] - 4.0)
        j = station.SimStation(t).run("yaw", manifests.yaw("t", servo_for(t, t["yaw"]["crosstalk_map"]), script,
                                                            excitation=probe), "t")
        rows = identify.crosstalk_windows(j, probe["begin_s"], probe["duration_s"], 80.0, window_s=0.25)
        _, g, _, _ = identify.crosstalk_table(rows, 80.0, inertia=t["yaw"]["inertia"])
        error = np.sqrt(np.mean((np.array(g) - np.array(t["yaw"]["crosstalk_map"])) ** 2))
        self.assertLess(error, 0.0006)


class Pipeline(unittest.TestCase):
    def run_pipeline(self, axis, case, *options, accepted=True):
        """accepted: the verdict expected, or None for a case that sits on a gate (either is right)."""
        with tempfile.TemporaryDirectory() as out:
            done = subprocess.run([sys.executable, str(HERE / "commission.py"), axis, "--sim", str(CASES / f"{case}.json"),
                                   "--out", out, *options], capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stdout[-2000:] + done.stderr[-2000:])
            asset = load(Path(out) / f"{axis}_servo.json")
            report = load(Path(out) / "report.json")
        if accepted is not None:
            self.assertEqual(asset["provenance"]["accepted"], accepted, json.dumps(asset["validation"])[:2000])
        return asset, report

    def test_yaw_from_prior_station_estimate(self):
        asset, _ = self.run_pipeline("yaw", "station_estimate")
        self.assertLess(abs(asset["identified"]["inertia"] / truth("station_estimate")["yaw"]["inertia"] - 1), 0.15)

    def test_yaw_from_prior_heavy_payload(self):
        """A 2.5x payload: the same command finds the new inertia and scales the gains up.

        The gain is bounded by the crosstalk error the loop must survive (design.py): its
        right-half-plane zero sits at 1/sqrt(J |g|), so the usable wn falls as 1/sqrt(J) and
        kv = 2 zeta J wn grows as sqrt(J) -- not in proportion to J."""
        light, _ = self.run_pipeline("yaw", "station_estimate")
        heavy, _ = self.run_pipeline("yaw", "heavy_payload")
        ratio = heavy["identified"]["inertia"] / light["identified"]["inertia"]
        self.assertGreater(ratio, 2)
        self.assertGreater(heavy["servo_parameters"]["kv"], 0.8 * math.sqrt(ratio) * light["servo_parameters"]["kv"])

    def test_yaw_from_prior_payload_5x(self):
        """The 2026-10-24 payload class: the survey's rough inertia lowers the sweep to 3 Hz, the
        sweep identifies within 15%, and the pipeline accepts. (10x: Identification.test_prior_gains_at_10x.)"""
        asset, report = self.run_pipeline("yaw", "payload_5x")
        J = truth("payload_5x")["yaw"]["inertia"]
        self.assertLess(abs(asset["identified"]["inertia"] / J - 1), 0.15, f"identified {asset['identified']['inertia']}")
        sweep = next(s for s in report["steps"] if s["step"] == "inertia")["sweep"]
        self.assertEqual(sweep["f0_hz"], 3.0)

    def test_yaw_payload_5x_as_measured_with_friction_drift(self):
        """The 5x payload as the station measured it (payload_5x_knee: the friction knee the fit cannot
        follow, and the bearing's level moving +-0.15 A between sessions): accepted under the
        engineering gates (score.YAW_GATES) from two start angles, at most 4 stalls each (measured worst
        2 with the cap rule, 5 without). Stalls: Design.test_integral_cap_covers_breakaway_and_drift.

        From 3.5 rad it is refused, and the test says so (a known limit, closed 2026-10-25). The survey
        runs its 0.25-2 deg/s plateaus on the few degrees where it starts, and no other plateau crosses
        that arc at a steady speed (every one begins or ends there, settling), so nothing in the survey
        tells a sticky arc from a high slow friction: the curve takes the start's friction. This case's
        map has a +0.32 A spike at 3.5 rad, so the 1 deg/s feedforward reads 0.11-0.18 A high over the
        whole turn and ramp+1 slips ahead. Fitting the curve and the map as one model was tried: with
        the start arc unobserved it only traded this bias for others (the --update case then slipped
        16 deg). On the station it is not what limits slow tracking: its maps span -0.08..+0.14 A (#5,
        survey at 50 deg) and -0.04..+0.06 A (#6, 63 deg), which bounds the bias, while the bearing's
        own slow level rises 0.25 A within a run (the work-in, docs/ADR-005/STAGE2.md) against a servo
        tolerance at 1 deg/s of 0.1-0.15 A."""
        refused = 3.5
        for start in (1.0, refused, 5.2):
            t = truth("payload_5x_knee")
            t["yaw_start_rad"] = start
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as f:
                json.dump(t, f)
            try:
                with tempfile.TemporaryDirectory() as out:
                    done = subprocess.run([sys.executable, str(HERE / "commission.py"), "yaw", "--sim", f.name, "--out", out],
                                          capture_output=True, text=True)
                    self.assertEqual(done.returncode, 0, done.stdout[-2000:] + done.stderr[-2000:])
                    asset = load(Path(out) / "yaw_servo.json")
            finally:
                Path(f.name).unlink()
            fails = [x for v in asset["validation"] for x in v["fails"]]
            if start == refused:
                self.assertFalse(asset["provenance"]["accepted"], "start 3.5 accepted: the docstring's limit is gone")
                self.assertTrue(fails and all(x.startswith("ramp+") and "(pointing)" in x for x in fails), fails)
                continue
            self.assertTrue(asset["provenance"]["accepted"], f"start {start}: " + json.dumps(fails))
            stalls = [v["extra"]["stalls"] for v in asset["validation"]]
            self.assertTrue(all(n <= 4 for n in stalls), f"start {start}: stalls {stalls}")
            f = asset["identified"]["friction"]
            plateau = max(f["positive"]["plateau_max"], f["negative"]["plateau_max"])
            self.assertAlmostEqual(asset["servo_parameters"]["integral_cap"],
                                   min(1.2, plateau + RULES["breakaway_over_plateau_A"] + RULES["friction_drift_A"]), places=3)
            self.assertEqual(asset["servo_parameters"]["stall_recovery"]["current_A"], asset["servo_parameters"]["integral_cap"])

    def test_yaw_update_with_a_drifted_table(self):
        """`yaw --update` on the payload with the asset's three-week-old table (RE-RUN on the station,
        2026-10-24): the inertia is still identified and the fit reports the drift. This plant's
        friction is nearly twice the station's (0.58 A at 5 deg/s against 0.31-0.36), and with the
        drift allowance the robust design is wn 4.5 (kq 4.9), soft: it stick-slips on the slow ramps.
        Its verdict sits on the 3.4 deg pointing gate and turns on the drift drawn (review 2026-10-25,
        seeds 2-6): accepted for 4 of 5 with the friction map on its entries, 2 of 5 while it sat half
        an entry late (seed 2, this one, then slipped 4.6 deg on ramp-5; seed 5 now misses by 0.15 deg on
        rev-a). So the verdict is not what this test checks: the identification and the design are.
        The map itself: Design.test_friction_maps_land_where_the_servo_reads_them."""
        t = truth("payload_5x")
        old = drifted(t["yaw"]["crosstalk_map"], np.random.default_rng(2))
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as f:
            json.dump({"identified": {"crosstalk": {"delay_s": t["yaw"]["crosstalk_delay_s"], "table": old, "uncertainty": 0.000479}}}, f)
        asset, report = self.run_pipeline("yaw", "payload_5x", "--update", "--asset", f.name, accepted=None)
        Path(f.name).unlink()
        self.assertLess(asset["servo_parameters"]["kq"], 6.0)
        fails = [x for v in asset["validation"] for x in v["fails"]]
        self.assertTrue(all("(pointing)" in x for x in fails), f"only the pointing gate is in play: {fails}")
        self.assertLess(abs(asset["identified"]["inertia"] / t["yaw"]["inertia"] - 1), 0.15, asset["identified"]["inertia"])
        step = next(s for s in report["steps"] if s["step"] == "inertia")
        self.assertLess(abs(step["residual_crosstalk"] + 0.0011), 0.0006, f"drift found: {step['residual_crosstalk'] * 1e3:+.2f} mrad/A")

    def test_pitch_from_prior(self):
        for case in ("station_estimate", "heavy_payload"):
            asset, _ = self.run_pipeline("pitch", case)
            tau = truth(case)["pitch"]["speed_tau_s"]
            self.assertLess(abs(asset["identified"]["speed_tau_s"] / tau - 1), 0.2)


if __name__ == "__main__":
    unittest.main()
