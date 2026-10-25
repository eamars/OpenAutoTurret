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


class AFailedPipelineIsAVerdict(unittest.TestCase):
    """A pipeline that cannot go on says why in its verdict, never as calibd crashing. check and
    calibrate let their Failed through: MEASURE 2026-10-25 00:38 and 09:13 ended "NOT ACCEPTED:
    calibd failed: calibration pass ff_fb0 failed the tracking gates"."""

    class Dead:
        """A station whose every session is a transient (a CAN hole, a missing footer)."""
        name = "controld"

        def run(self, axis, manifest, label):
            import commission
            raise commission.Transient(f"{label}: the journal has no footer")

    def test_check_and_calibrate_end_stopped_with_the_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / "state"
            record = state / "calibration" / "records" / "0002"
            record.mkdir(parents=True)
            shutil.copy(HERE.parents[1] / "config/servo/yaw_servo.json", record / "yaw_servo.json")
            fake = FakeController(Path(tmp))
            runner = calibd.Runner(fake, state, fake.run, {"id": 2, "name": "FRICTION 10-20"}, poll_s=0.01)
            original = calibd.ControldStation
            calibd.ControldStation = lambda *a, **k: self.Dead()
            try:
                verdict = runner.execute()   # VERIFY: check yaw
            finally:
                calibd.ControldStation = original
            self.assertRegex(verdict, r"^NOT ACCEPTED: check: probe: .*the journal has no footer \(\d attempts\)$")
            report = runner.pipeline("calibrate", {}, self.Dead())
            stopped = [s for s in report["steps"] if s["step"] == "stopped"]
            self.assertEqual(len(stopped), 1, report["steps"])
            self.assertIn("ff_fb0", stopped[0]["reason"])


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

    def test_check_pitch_runs_the_pitch_use_case_on_the_record_s_pitch_asset(self):
        truth = json.loads((HERE / "sim_cases" / "station_estimate.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            state = Path(tmp) / "state"
            record = state / "calibration" / "records" / "0001"
            record.mkdir(parents=True)
            shutil.copy(HERE.parents[1] / "config/servo/pitch_servo.json", record / "pitch_servo.json")
            fake = FakeController(Path(tmp))
            fake.run["kind"] = "CHECK-PITCH"
            runner = calibd.Runner(fake, state, fake.run, {"id": 1, "name": "AS SHIPPED"}, poll_s=0.01)
            sim_station = stations.SimStation(truth, log=lambda *_: None)
            original = calibd.ControldStation
            calibd.ControldStation = lambda *a, **k: sim_station
            try:
                verdict = runner.execute()
            finally:
                calibd.ControldStation = original
            self.assertIn(verdict, ("REUSE_EXACT", "UPDATE_PARAMETERS"), verdict)
            report = json.loads((runner.root / "check" / "report.json").read_text())
            self.assertEqual(report["axis"], "pitch")



class StepAssets(unittest.TestCase):
    """A later step measures what the run will install (MEASURE on #5, 2026-10-25 01:12: the accuracy
    campaign ran the record's bare-turret yaw asset on the 5x payload, after the run had accepted its own)."""

    def runner(self, tmp, kind="MEASURE"):
        state = Path(tmp) / "state"
        record = state / "calibration" / "records" / "0005"
        record.mkdir(parents=True)
        (record / "yaw_servo.json").write_text('{"from": "record"}')
        run = {"id": 1, "kind": kind, "active": True, "state": "waiting"}
        return calibd.Runner(FakeController(Path(tmp)), state, run, {"id": 5, "name": "PAYLOAD 10-24"}, poll_s=0.01)

    def asset_given_to(self, runner, which, options):
        import commission
        name = {"yaw": "commission_yaw", "calibrate": "calibrate"}[which]
        seen, original = {}, getattr(commission, name)
        setattr(commission, name, lambda args: seen.setdefault("asset", args.asset))
        try:
            runner.pipeline(which, options, station=None)
        finally:
            setattr(commission, name, original)
        return json.loads(Path(seen["asset"]).read_text())["from"]

    def test_the_accuracy_campaign_measures_the_yaw_servo_this_run_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            runner = self.runner(tmp)
            self.assertEqual(self.asset_given_to(runner, "yaw", {"update": True}), "record", "yaw --update starts from the record")
            (runner.root / "yaw").mkdir(parents=True)
            (runner.root / "yaw" / "yaw_servo.json").write_text('{"from": "this run"}')
            self.assertEqual(self.asset_given_to(runner, "calibrate", {}), "this run")

    def test_with_no_yaw_accepted_in_the_run_the_record_s_servo_is_measured(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.asset_given_to(self.runner(tmp, "RERUN+ACCURACY"), "calibrate", {}), "record")

    def test_the_asset_s_validation_passes_are_found_in_calibd_s_layout(self):
        import commission
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            yaw = runs / "0001-measure-20261025-003847" / "yaw"
            for label in ("yaw-20261025-004206-06-usecase0", "yaw-20261025-004206-07-usecase1",
                          "yaw-20261025-004206-07-usecase1-r1", "yaw-20261025-004206-05-circle0"):
                (yaw / "sessions" / label).mkdir(parents=True)
                (yaw / "sessions" / label / "yaw-control.jsonl").write_text("{}\n")
            (yaw / "report.json").write_text('{"run": "yaw-20261025-004206"}')
            original, commission.RUNS = commission.RUNS, runs
            try:
                found = commission.asset_validation_journals({"provenance": {"run": "yaw-20261025-004206"}})
            finally:
                commission.RUNS = original
            self.assertEqual([p.parent.name for p in found], ["yaw-20261025-004206-06-usecase0", "yaw-20261025-004206-07-usecase1",
                                                              "yaw-20261025-004206-07-usecase1-r1"],
                             "every use-case attempt; calibrate keeps the COMPLETE ones")


class Pruning(unittest.TestCase):
    """ADR-005 D3: reports stay; journals only for each record's latest installed run and the recent."""

    def run_dir(self, runs, name, record, installed):
        d = runs / name
        (d / "yaw" / "sessions" / "s1").mkdir(parents=True)
        (d / "yaw" / "sessions" / "s1" / "yaw-control.jsonl").write_text("{}\n")
        (d / "yaw" / "report.json").write_text("{}")
        (d / "run.json").write_text(json.dumps({"record": record, "installed": installed}))
        return d

    def test_only_the_latest_installed_run_per_record_and_the_recent_keep_journals(self):
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            old_install_2 = self.run_dir(runs, "0001-rerun", 2, True)
            new_install_2 = self.run_dir(runs, "0002-rerun", 2, True)
            install_3 = self.run_dir(runs, "0003-measure", 3, True)
            verify = self.run_dir(runs, "0004-verify", 2, False)
            recent = [self.run_dir(runs, f"000{k}-verify", 2, False) for k in (5, 6, 7)]
            (runs / "0008-verify").mkdir()   # in progress: no run.json yet
            (runs / "0008-verify" / "a.jsonl").write_text("{}\n")
            removed = calibd.prune(runs)
            journals = lambda d: list(d.rglob("*.jsonl"))
            self.assertEqual(journals(old_install_2), [], "superseded by #2's newer install")
            self.assertEqual(journals(verify), [], "neither installed nor recent")
            for d in [new_install_2, install_3] + recent:
                self.assertEqual(len(journals(d)), 1, d.name)
            self.assertTrue((runs / "0008-verify" / "a.jsonl").exists(), "a run in progress is not touched")
            self.assertTrue((old_install_2 / "yaw" / "report.json").exists(), "reports are kept")
            self.assertEqual(len(removed), 2)

    def test_recent_means_by_time_across_a_controld_restart(self):
        # Station 2026-10-24 23:18: run ids restart with every controld, so the run that had just
        # ended was 0001 beside older 0002s and 0003s -- and sorting by name pruned it on the spot.
        with tempfile.TemporaryDirectory() as tmp:
            runs = Path(tmp)
            names = ["0001-verify-20261024-182744", "0002-verify-20261024-183120", "0001-verify-20261024-190657",
                     "0002-verify-20261024-211446", "0003-rerun+accuracy-20261024-211618",
                     "0001-check-pitch-20261024-184122", "0001-rerun+accuracy-20261024-225809"]
            dirs = {n: self.run_dir(runs, n, 5, False) for n in names}
            calibd.prune(runs, keep_recent=3)
            kept = sorted(n for n, d in dirs.items() if list(d.rglob("*.jsonl")))
            self.assertEqual(kept, ["0001-rerun+accuracy-20261024-225809", "0002-verify-20261024-211446",
                                    "0003-rerun+accuracy-20261024-211618"])


if __name__ == "__main__":
    unittest.main()
