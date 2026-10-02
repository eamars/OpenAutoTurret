"""Offline proof of the commissioning pipeline: identification recovers known plants,
and the whole pipeline (prior -> asset) is accepted against simulated stations.

    run/servo-commission/venv/bin/python -m unittest discover -s Firmware/tools/servo_commission/tests

Needs the native servo-sim (run/adr0022-local/firmware, `make servo-sim`). Takes a few minutes.
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


def truth(name):
    return load(CASES / f"{name}.json")


def servo_for(t, table):
    return dict(PRIOR, coulomb_positive=0.35, coulomb_negative=0.33, stribeck_positive=0.35, stribeck_negative=0.35,
                stribeck_speed=0.3, crosstalk_map=[g - 0.002 for g in table], crosstalk_delay_s=0.001)


class Identification(unittest.TestCase):
    def inertia_session(self, t, table):
        sweep = dict(manifests.INERTIA_SWEEP, begin_s=3.0, duration_s=10.5)
        j = station.SimStation(t).run("yaw", manifests.yaw("t", servo_for(t, table), manifests.inertia_sweep(),
                                                            excitation=sweep, vmax=35.0), "t")
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        return identify.inertia_from_frf(identify.inertia_response(j, sweep, table, 0.001), residual_bound=0.0003)

    def test_inertia_within_ten_percent(self):
        rng = np.random.default_rng(1)
        for name, scale in (("station_estimate", 0.5), ("station_estimate", 1.0), ("heavy_payload", 1.0)):
            t = truth(name)
            t["yaw"]["inertia"] *= scale
            table = list(np.array(t["yaw"]["crosstalk_map"]) + rng.normal(0, 0.0003, 120))
            a = self.inertia_session(t, table)[0]
            self.assertLess(abs(a / t["yaw"]["inertia"] - 1), 0.12, f"{name} x{scale}: {a:.4f}")

    def test_delay_calibration(self):
        """commission.DELAY_OFFSET_S maps the sweep's delay to the plant's actuation delay."""
        for act in (0.0005, 0.0015):
            t = truth("station_estimate")
            t["yaw"]["actuation_delay_s"] = act
            d = self.inertia_session(t, t["yaw"]["crosstalk_map"])[1]
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
    def run_pipeline(self, axis, case):
        with tempfile.TemporaryDirectory() as out:
            done = subprocess.run([sys.executable, str(HERE / "commission.py"), axis, "--sim", str(CASES / f"{case}.json"),
                                   "--out", out], capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stdout[-2000:] + done.stderr[-2000:])
            asset = load(Path(out) / f"{axis}_servo.json")
        self.assertTrue(asset["provenance"]["accepted"], json.dumps(asset["validation"])[:2000])
        return asset

    def test_yaw_from_prior_station_estimate(self):
        asset = self.run_pipeline("yaw", "station_estimate")
        self.assertLess(abs(asset["identified"]["inertia"] / truth("station_estimate")["yaw"]["inertia"] - 1), 0.15)

    def test_yaw_from_prior_heavy_payload(self):
        """A 2.5x payload: the same command finds the new inertia and scales the gains up.

        The gain is bounded by the crosstalk error the loop must survive (design.py): its
        right-half-plane zero sits at 1/sqrt(J |g|), so the usable wn falls as 1/sqrt(J) and
        kv = 2 zeta J wn grows as sqrt(J) -- not in proportion to J."""
        light, heavy = self.run_pipeline("yaw", "station_estimate"), self.run_pipeline("yaw", "heavy_payload")
        ratio = heavy["identified"]["inertia"] / light["identified"]["inertia"]
        self.assertGreater(ratio, 2)
        self.assertGreater(heavy["servo_parameters"]["kv"], 0.8 * math.sqrt(ratio) * light["servo_parameters"]["kv"])

    def test_pitch_from_prior(self):
        for case in ("station_estimate", "heavy_payload"):
            asset = self.run_pipeline("pitch", case)
            tau = truth(case)["pitch"]["speed_tau_s"]
            self.assertLess(abs(asset["identified"]["speed_tau_s"] / tau - 1), 0.2)


if __name__ == "__main__":
    unittest.main()
