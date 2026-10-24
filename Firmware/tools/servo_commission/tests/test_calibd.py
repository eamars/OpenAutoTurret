"""calibd (ADR-005 D7): the session protocol against a fake controld, and a VERIFY through the real
pipeline with servo-sim standing in for controld.

The fake answers a session with a journal controld's executors really wrote (the C++ test
test_calibration_session leaves them behind, OTA_SESSION_JOURNALS); the VERIFY needs the native
servo-sim, as tests/test_pipeline.py does.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import calibd  # noqa: E402
import sim  # noqa: E402
import station as stations  # noqa: E402

SESSION_TEST = Path(os.environ.get("OTA_SESSION_TEST", HERE.parents[2] / "run/adr0022-local/firmware/control/test_calibration_session"))


class FakeController:
    """controld's side of a run, as calibd sees it: telemetry and the gate."""

    def __init__(self, journals: Path, run_id=3, refuse=None, stop_during=False):
        self.journals, self.refuse, self.stop_during = journals, refuse, stop_during
        self.run = {"id": run_id, "kind": "VERIFY", "active": True, "state": "waiting", "session": None,
                    "last_session": "", "note": "", "verdict": ""}
        self.ack = ("", -1, "")
        self.sent = []
        self.lock = threading.Lock()

    def telemetry(self):
        with self.lock:
            return SimpleNamespace(calibrating=self.run["active"], calibration_run=json.loads(json.dumps(self.run)),
                                   calibration_record={"id": 2, "name": "FRICTION 10-20"}, cmd_ack_command=self.ack[0],
                                   cmd_ack_accepted=self.ack[1], cmd_ack_reason=self.ack[2])

    def send(self, name, arg=""):
        self.sent.append((name, arg))
        if name == "calibration_note":
            with self.lock:
                self.run["note"] = arg.split(" ", 1)[1]
            return True, ""
        if name == "calibration_finish":
            with self.lock:
                self.run.update(active=False, state="done", verdict=arg.split(" ", 2)[2])
            return True, ""
        assert name == "calibration_session", name
        run, axis, path = arg.split(" ", 2)
        label = Path(path).parent.name
        if self.refuse:
            with self.lock:
                self.ack = ("calibration_session", 0, self.refuse)
            return True, ""
        threading.Thread(target=self.session, args=(axis, Path(path), label), daemon=True).start()
        return True, ""

    def session(self, axis, manifest, label):
        with self.lock:
            self.run.update(state="session", session={"label": label, "axis": axis})
        time.sleep(0.05)
        if self.stop_during:
            with self.lock:
                self.run.update(active=False, state="stopped", session=None, verdict="stopped by the operator")
            return
        name = calibd.JOURNAL[axis]
        shutil.copy(self.journals / name, manifest.parent / name)
        with self.lock:
            self.run.update(state="fitting", session=None, last_session=f"{label}: COMPLETE")


@unittest.skipUnless(SESSION_TEST.exists(), f"build test_calibration_session first ({SESSION_TEST})")
class SessionProtocol(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.journals = Path(cls.tmp.name) / "journals"
        cls.journals.mkdir()
        done = subprocess.run([str(SESSION_TEST), "--gtest_filter=PitchSession.RunsThe*:YawSession.Follows*"],
                              env=dict(os.environ, OTA_SESSION_JOURNALS=str(cls.journals)), capture_output=True, text=True)
        assert done.returncode == 0, done.stdout[-2000:]

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def station(self, fake, name):
        return calibd.ControldStation(fake, 3, Path(self.tmp.name) / name, start_timeout_s=1.0, poll_s=0.01)

    def test_a_session_goes_to_controld_and_its_journal_comes_back_parsed(self):
        fake = FakeController(self.journals)
        j = self.station(fake, "a").run("yaw", {"schema": "adr0022.yaw-control/1", "output": "x"}, "0003-verify-probe")
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        name, arg = fake.sent[0]
        self.assertEqual(name, "calibration_session")
        run, axis, path = arg.split(" ", 2)
        self.assertEqual((run, axis), ("3", "yaw"))
        manifest = json.loads(Path(path).read_text())
        self.assertEqual(manifest["session_label"], "0003-verify-probe")
        self.assertNotIn("output", manifest, "controld puts the journal beside the manifest")
        j = self.station(fake, "b").run("pitch", {}, "0003-rerun-identify")
        self.assertAlmostEqual(j["centre"], -0.1)

    def test_a_refusal_says_why_and_a_stop_ends_the_pipeline(self):
        with self.assertRaisesRegex(RuntimeError, "no pitch window"):
            self.station(FakeController(self.journals, refuse="session x not started: no pitch window from homing"),
                         "c").run("pitch", {}, "x")
        with self.assertRaisesRegex(calibd.Stopped, "stopped by the operator"):
            self.station(FakeController(self.journals, stop_during=True), "d").run("yaw", {}, "y")


class SimLink(FakeController):
    """A fake controld whose sessions are servo-sim's answers (SimStation), written out as journals
    would be -- here simply handed to the pipeline through the station interface."""


@unittest.skipUnless(sim.SERVO_SIM.exists(), f"build servo-sim first ({sim.SERVO_SIM})")
class VerifyThroughThePipeline(unittest.TestCase):
    def test_verify_runs_check_yaw_and_reports_its_verdict(self):
        truth = json.loads((HERE / "sim_cases" / "station_estimate.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / "state"
            record = state / "calibration" / "records" / "0002"
            record.mkdir(parents=True)
            shutil.copy(HERE.parents[1] / "config/servo/yaw_servo.json", record / "yaw_servo.json")
            fake = FakeController(Path(tmp))
            runner = calibd.Runner(fake, state, fake.run, {"id": 2, "name": "FRICTION 10-20"}, poll_s=0.01)
            sim_station = stations.SimStation(truth, log=lambda *_: None)
            original = calibd.ControldStation
            calibd.ControldStation = lambda *a, **k: sim_station
            try:
                verdict = runner.execute()
            finally:
                calibd.ControldStation = original
            self.assertIn(verdict, ("REUSE_EXACT", "UPDATE_PARAMETERS"), verdict)
            notes = [a for n, a in fake.sent if n == "calibration_note"]
            self.assertTrue(any(a.endswith("check | starting") for a in notes), notes)
            self.assertTrue((runner.root / "check" / "report.json").exists(), "kept in the store's runs/")


if __name__ == "__main__":
    unittest.main()
