"""controld's calibration sessions write commissiond's journals (ADR-005 D7): journal.py, the parser the
whole pipeline reads them with, takes what the C++ executors actually wrote.

Runs control/tests/test_calibration_session from this checkout's build (OTA_SESSION_TEST names it;
by default run/adr0022-local/firmware) and parses the journals its sessions leave behind.
"""
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import journal  # noqa: E402

BINARY = Path(os.environ.get("OTA_SESSION_TEST", HERE.parents[2] / "run/adr0022-local/firmware/control/test_calibration_session"))


@unittest.skipUnless(BINARY.exists(), f"build test_calibration_session first ({BINARY})")
class ControldJournal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        done = subprocess.run([str(BINARY), "--gtest_filter=PitchSession.RunsThe*:YawSession.Follows*"],
                              env=dict(os.environ, OTA_SESSION_JOURNALS=cls.tmp.name), capture_output=True, text=True)
        assert done.returncode == 0, done.stdout[-2000:]
        cls.dir = Path(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_a_yaw_session_reads_as_commissiond_s(self):
        j = journal.yaw(self.dir / "yaw-control.jsonl")
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        self.assertEqual(j["footer"]["executor"], "controld")
        self.assertIn("current_cap", j["manifest"]["servo_parameters"])
        self.assertIsNotNone(j["learned"])
        f, c, tx = j["feedback"], j["cycles"], j["tx"]
        self.assertGreater(len(f["t"]), 3900)
        self.assertLess(abs(f["t"][0]), 0.002, "times count from yaw_control_begin")
        self.assertAlmostEqual(f["q"][0], c["q"][0], delta=0.01, msg="feedback and the servo share coordinates")
        self.assertAlmostEqual(float(c["qr"][-1] - c["qr"][0]), 0.2, delta=1e-6)
        self.assertEqual(len(tx["t"]), len(c["t"]))
        self.assertTrue((tx["t"] >= c["t"]).all(), "a current is sent after the step that chose it")
        self.assertEqual(len(j["pitch_posture"]), 2, "the held pitch's range, for the decoupling record")

    def test_a_pitch_session_reads_as_commissiond_s(self):
        j = journal.pitch(self.dir / "sensorless-homing.jsonl")
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        self.assertAlmostEqual(j["window"]["center_rad"], -0.1)
        self.assertAlmostEqual(j["centre"], -0.1)
        t = j["trial"]["t"]
        self.assertLess(t[0], -3.0, "the approach comes before script time zero")
        self.assertAlmostEqual(float(t[-1]), 2.5, delta=0.01, msg="the script and its hold")
        self.assertAlmostEqual(j["yaw_motion_rad"], 0.0003)
        self.assertTrue(math.isfinite(float(j["trial"]["cmd"].max())))


if __name__ == "__main__":
    unittest.main()
