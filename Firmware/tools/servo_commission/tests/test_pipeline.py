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
import identify  # noqa: E402
import manifests  # noqa: E402
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
    def run_pipeline(self, axis, case, *options):
        with tempfile.TemporaryDirectory() as out:
            done = subprocess.run([sys.executable, str(HERE / "commission.py"), axis, "--sim", str(CASES / f"{case}.json"),
                                   "--out", out, *options], capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stdout[-2000:] + done.stderr[-2000:])
            asset = load(Path(out) / f"{axis}_servo.json")
            report = load(Path(out) / "report.json")
        self.assertTrue(asset["provenance"]["accepted"], json.dumps(asset["validation"])[:2000])
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

    def test_yaw_update_with_a_drifted_table(self):
        """`yaw --update` on the payload with the asset's three-week-old table (RE-RUN on the station,
        2026-10-24): the inertia is still identified, the fit reports the drift, the run is accepted."""
        t = truth("payload_5x")
        old = drifted(t["yaw"]["crosstalk_map"], np.random.default_rng(2))
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as f:
            json.dump({"identified": {"crosstalk": {"delay_s": t["yaw"]["crosstalk_delay_s"], "table": old, "uncertainty": 0.000479}}}, f)
        asset, report = self.run_pipeline("yaw", "payload_5x", "--update", "--asset", f.name)
        Path(f.name).unlink()
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
