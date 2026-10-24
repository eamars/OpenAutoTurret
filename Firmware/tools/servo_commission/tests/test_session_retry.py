"""A transient repeats the session; a result does not (owner, 2026-10-24: the station is real, anything
can happen, and one transient must not end a run)."""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import commission  # noqa: E402


class ScriptedStation:
    """Answers each session with the next outcome: a footer (status, detail), or an exception."""
    name = "scripted"

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.labels = []

    def run(self, axis, manifest, label):
        self.labels.append(label)
        o = self.outcomes.pop(0)
        if isinstance(o, Exception):
            raise o
        return {"footer": {"status": o[0], "detail": o[1]}, "manifest": manifest}


class SessionRetry(unittest.TestCase):
    def run_session(self, outcomes):
        station = ScriptedStation(outcomes)
        with tempfile.TemporaryDirectory() as tmp:
            run = commission.Run(SimpleNamespace(sim=None, out=tmp, station=station), "yaw")
            try:
                j = run.session("inertia", {"m": 1})
            finally:
                run.logfile.close()
            return j, station, run

    def test_a_data_hole_runs_the_session_again(self):
        # Station 2026-10-24 23:51: a 30 ms yaw CAN dropout ended the inertia session this way.
        j, station, run = self.run_session([("INVALID", "MEASUREMENT_LIMITED: servo sensor data stale or invalid"),
                                            ("COMPLETE", "")])
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        self.assertEqual(len(station.labels), 2)
        self.assertTrue(station.labels[1].endswith("-01-inertia-r1"), station.labels)
        self.assertEqual(run.count, 1, "one session in the run's numbering, whatever it took")
        self.assertEqual(run.report["retries"][0]["reason"], "MEASUREMENT_LIMITED: servo sensor data stale or invalid")

    def test_a_late_start_or_footer_runs_it_again(self):
        j, station, _ = self.run_session([commission.Transient("controld did not start the session"),
                                          commission.Transient("the journal has no footer"), ("COMPLETE", "")])
        self.assertEqual(j["footer"]["status"], "COMPLETE")
        self.assertEqual(len(station.labels), 3)

    def test_results_are_never_repeated(self):
        # What a ladder or a circle measures, the plant's limits, and the operator's STOP.
        for detail in ("HARD_ABORT: servo oscillation (fast current RMS 0.412 A)", "HARD_ABORT: yaw speed limit",
                       "HARD_ABORT: servo following error limit", "HARD_ABORT: yaw current authority",
                       "HARD_ABORT: yaw motor temperature limit", "the operator pressed STOP"):
            j, station, _ = self.run_session([("INVALID", detail)])
            self.assertEqual(len(station.labels), 1, detail)
            self.assertEqual(j["footer"]["detail"], detail)

    def test_a_transient_that_keeps_coming_ends_as_before(self):
        hole = ("INVALID", "MEASUREMENT_LIMITED: servo sensor data stale or invalid")
        j, station, _ = self.run_session([hole, hole, hole])
        self.assertEqual(len(station.labels), 1 + commission.SESSION_RETRIES)
        self.assertEqual(j["footer"]["status"], "INVALID", "complete() then fails the step, as before")
        with self.assertRaises(commission.Failed):
            self.run_session([commission.Transient("controld did not start the session")] * (1 + commission.SESSION_RETRIES))


if __name__ == "__main__":
    unittest.main()
