"""The §55 extractor, against a fixture log whose answers are known by hand.

P13's deliverable is a number table, and a table produced by a parser that
quietly returns 0 for a line it did not understand is the worst possible
failure: the operator signs off on figures that were never in the log. So the
fixture spells every line format out (copied from the real `controld --sim` log
and from the format strings in main.cpp / control_loop.cpp), and the tests
assert both directions — the numbers that ARE there come out exact, and the
numbers that are NOT there come out as NOT MEASURED, never as zero.
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest

from tools.acceptance_metrics import analyse, render_md

# A 12-second run: home, hold, one payload check, one slow cycle. Timestamps are
# real spdlog format; the values below are the answers asserted in the tests.
FIXTURE = """\
[2026-09-03 10:00:00.100] [controld] [info] controld starting (config: config/turret.yaml)
[2026-09-03 10:00:01.000] [controld] [info] homing started; tracking auto-enables after homing
[2026-09-03 10:00:02.000] [controld] [info] motion t=1000.00ms ax=pitch q=-0.10000 v=+0.0100 a=+0.00 j=+0.0 tq=+1.200 cmd=+0.0100 msg=approach
[2026-09-03 10:00:03.000] [controld] [info] motion t=2000.00ms ax=pitch q=-0.80000 v=+0.0100 a=+0.00 j=+0.0 tq=+4.750 cmd=+0.0100 msg=approach
[2026-09-03 10:00:04.000] [controld] [info] motion t=3000.00ms ax=pitch q=-0.80000 v=+0.0000 a=+0.00 j=+0.0 tq=+0.100 cmd=+0.0000 msg=endpoint A homed; starting endpoint B
[2026-09-03 10:00:05.000] [controld] [info] motion t=4000.00ms ax=yaw q=+1.00000 v=+0.0100 a=+0.00 j=+0.0 tq=+2.500 cmd=+0.0100 msg=approach
[2026-09-03 10:00:06.000] [controld] [info] motion t=5000.00ms ax=yaw q=-1.50000 v=+0.0100 a=+0.00 j=+0.0 tq=+3.250 cmd=+0.0100 msg=settle
[2026-09-03 10:00:07.000] [controld] [info] t=100.00s phase=homing q_pitch=-0.8000 q_yaw=-1.5000 rad temp_pitch=41.0 temp_yaw=39.5 C a_pitch=+0.00 a_yaw=+0.00
[2026-09-03 10:00:08.000] [controld] [info] homed + at ready pose; holding (Ctrl-C to park)
[2026-09-03 10:00:09.000] [controld] [info] loop: target=200 Hz p50=5.050 p95=5.060 p99=5.070 worst=5.100 ms (n=1600)
[2026-09-03 10:00:10.000] [controld] [info] SLOW CYCLE 7.250 ms (phase=hold, action=HOLD)
[2026-09-03 10:00:11.000] [controld] [info] loop: target=200 Hz p50=5.060 p95=5.080 p99=5.090 worst=7.250 ms (n=2000)
[2026-09-03 10:00:12.000] [controld] [info] vision: 148 frames (2 dropped, seq 1234, age 41.5 ms) | tracking=true state=tracking conf=0.87
[2026-09-03 10:00:13.000] [controld] [info] vision: 150 frames (5 dropped, seq 1384, age 62.5 ms) | tracking=true state=search conf=0.41
[2026-09-03 10:00:14.000] [controld] [info] supervisor: HOLD reason='vision stale' overrun_us=1200 misses=3
[2026-09-03 10:00:15.000] [controld] [info] payload check complete: ok (derated=false)
[2026-09-03 10:00:16.000] [controld] [info] control fault: stall detected on pitch
"""

CAN_STATS = """\nturret-can stats
error frames 12
dropped 3
invalid 0
feedback age 4.2
"""


class MetricsExtractionTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="ota_metrics_")
        self.log = os.path.join(self.dir, "controld.log")
        with open(self.log, "w") as f:
            f.write(FIXTURE)

    def _rows(self, can: bool = False, telem: bool = False) -> dict:
        can_file = ""
        if can:
            can_file = os.path.join(self.dir, "can.txt")
            with open(can_file, "w") as f:
                f.write(CAN_STATS)
        telem_file = ""
        if telem:
            telem_file = os.path.join(self.dir, "telem.jsonl")
            with open(telem_file, "w") as f:
                f.write('{"track_state": "acquiring"}\n'
                        '{"track_state": "tracking", "los_error_px": 3.5}\n'
                        '{"track_state": "tracking", "los_error_px": 1.5}\n')
        result = analyse([self.log], [telem_file] if telem else [],
                         [can_file] if can else [])
        return {r.name: r for r in result["rows"]}

    def _value(self, name: str, **kw) -> str:
        return self._rows(**kw)[name].value

    # -- control timing ----------------------------------------------------
    def test_loop_timing_taken_from_the_last_report_and_the_max(self):
        rows = self._rows()
        self.assertEqual(rows["target loop rate"].value, "200 Hz")
        self.assertEqual(rows["latest p50/p95/p99"].value,
                         "5.060 / 5.080 / 5.090 ms")
        self.assertEqual(rows["worst cycle ever reported"].value, "7.250 ms")

    def test_slow_cycle_warnings_counted_with_phases(self):
        v = self._value("SLOW CYCLE warnings")
        self.assertTrue(v.startswith("1 ("), v)
        self.assertIn("7.250", v)
        self.assertIn("hold", v)

    # -- homing ------------------------------------------------------------
    def test_homing_duration_from_the_two_events(self):
        self.assertEqual(
            self._value("duration (homing started -> at ready)"), "7.00 s")

    def test_travel_is_the_swept_range_when_only_one_endpoint_is_logged(self):
        # pitch: -0.10000 .. -0.80000 -> 0.7 rad = 40.11 deg
        self.assertIn("0.7000 rad",
                      self._value("measured travel (pitch) swept range"))
        self.assertIn("40.11 deg",
                      self._value("measured travel (pitch) swept range"))

    def test_peak_effort_ignores_non_homing_motion(self):
        self.assertEqual(
            self._value("peak effort (abs) on homing moves (pitch)"),
            "4.750 Nm")

    def test_repeatability_refuses_to_claim_a_spread_from_one_run(self):
        self.assertTrue(
            self._value("endpoint repeatability").startswith("NOT MEASURED"))

    # -- tracking ----------------------------------------------------------
    def test_vision_block_parsed_with_dropped_delta(self):
        rows = self._rows()
        self.assertEqual(rows["vision frames / dropped"].value,
                         "150 / 5 (last report)")
        self.assertEqual(rows["max dropped between reports"].value, "3")
        self.assertIn("tracking=1", rows["tracker states observed"].value)
        self.assertIn("search=1", rows["tracker states observed"].value)

    def test_los_error_is_never_invented(self):
        """No los_error_px in the log, none in a capture: the cell must say so.
        With a capture that carries it, it must be reported - both halves."""
        self.assertTrue(self._value("LOS tracking error (mean/max)")
                        .startswith("NOT MEASURED"))
        v = self._value("LOS tracking error (mean/max)", telem=True)
        self.assertIn("mean 2.500", v)
        self.assertIn("max 3.500", v)

    # -- limits ------------------------------------------------------------
    def test_faults_and_supervisor_actions_are_listed_not_summed_away(self):
        rows = self._rows()
        self.assertEqual(rows["control faults"].value,
                         "stall detected on pitch x1")
        self.assertIn("HOLD", rows["supervisor transitions"].value)
        self.assertIn("derated=false",
                      rows["derate / payload-check outcomes"].value)

    # -- CAN ---------------------------------------------------------------
    def test_can_counters_require_the_stats_file(self):
        self.assertTrue(self._value(
            "feedback age / dropped / error frames").startswith("NOT MEASURED"))
        rows = self._rows(can=True)
        self.assertEqual(rows["error frames"].value, "12")
        self.assertEqual(rows["dropped"].value, "3")

    # -- report hygiene ----------------------------------------------------
    def test_no_metric_is_reported_as_zero_when_its_source_is_absent(self):
        rows = self._rows()
        for name, r in rows.items():
            if r.value.strip() in ("0", "0.0", "0.000"):
                self.assertNotIn(
                    name,
                    ("LOS tracking error (mean/max)", "endpoint repeatability"),
                    f"{name} was reported as zero with no source")

    def test_markdown_escapes_pipes(self):
        rows = analyse([self.log], [], [])
        # Force a pipe into a value: the table must survive it.
        rows["rows"][0].value = "a|b"
        md = render_md(rows)
        self.assertIn("a\\|b", md)

    def test_every_row_names_where_it_came_from(self):
        for r in analyse([self.log], [], [])["rows"]:
            self.assertTrue(r.source and r.source != "—",
                            f"{r.name} has no provenance")


if __name__ == "__main__":
    unittest.main()
