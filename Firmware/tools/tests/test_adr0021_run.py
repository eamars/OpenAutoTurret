"""The campaign runner: it obeys the lock, refuses on drift, and never invents an improvement.

These tests use a stub station rather than hardware, because what is being pinned down is the runner's
discipline — refuse before touching the machine when the bindings no longer hold; count what actually
happened; leave an unscored trial unscored. Whether the transaction verifies a register is the
station's business and is measured on the station.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import adr0021_plan as plan  # noqa: E402
import adr0021_acceptance  # noqa: E402
import adr0021_scorer as scoring  # noqa: E402
import adr0021_scorer as scoring  # noqa: E402
import adr0021_run as runner  # noqa: E402
Runner = runner.Runner


class StubStation:
    """A station that always accepts, and answers the way controld does."""

    trace_rows = 4
    scorable_rows = False
    trace_history_rows = 4          # a rolling window has history in its head; tests may clear it
    trace_loses_identity = False
    trace_truncated = False

    def __init__(self, payload_status="no_profile", phase="hold"):
        self.payload_status, self.phase = payload_status, phase
        self._seq, self._pending, self.received, self.calls = 0, [], 0, []
        self.revision = 0

    def command(self, name, arg=None):
        # Commands are recorded, not just counted: a test that only counts cannot tell a RUN from a
        # snapshot, which is exactly the distinction :46 turns on.
        self.received += 1
        self.calls.append((name, arg))
        self._pending = None            # an ack only exists once something has been answered
        if name == "param_prepare":
            self._pending = f"prepared request_id=prepp-{self._seq + 1} expected_hash=aa revision_after_apply=1"
        elif name == "param_apply":
            self._pending = ("applied; revision=" + str(self.revision + 1)) if arg else "no request id"
            if arg:
                self.revision += 1
        elif name == "param_snapshot":
            self._pending = f"state=idle revision={self.revision} applied_hash=aa expected_hash=aa"
        else:
            self._pending = "unhandled"
        self._seq += 1
        return {"ok": True, "verdict": "submitted"}

    def seq(self):
        return self._seq

    def ack(self, after_seq):
        """Honours after_seq, because a stub that answers regardless hides exactly one bug:
        the runner asking for an ack that already went by. The hardware caught this; the stub
        had been letting it through.
        """
        if self._pending is None or self._seq == after_seq:
            return {"accepted": False, "reason": "no cmd_ack after that sequence", "seq": self._seq}
        return {"accepted": "no request" not in self._pending, "reason": self._pending,
                "seq": self._seq}

    def trace_window(self, expect_context=""):
        """The stub answers with a window whose records all carry the tag the runner set — unless a
        test asks it to lose the identity, which is the failure the runner must treat as blocked.
        It goes through command() so the per-candidate command count stays honest: a window is asked
        for, and asking costs a round trip on the real station too.
        """
        self.command("read_control_trace")
        if getattr(self, "refuse_run", False) and not self.runs_accepted:
            pass
        # Head: rows written before this candidate announced itself. Tail: this candidate's rows. The
        # summary comes from the same function the station reader uses, so the stub cannot pass by
        # disagreeing with the real code about what a window means.
        rows = [{"param_context": "history"} for _ in range(self.trace_history_rows)] + \
               [{"param_context": expect_context or "none"} for _ in range(self.trace_rows)]
        if self.trace_loses_identity:
            rows[-1]["param_context"] = "other-campaign"
        summary = adr0021_acceptance.summarise_window(rows, expect_context)
        summary.update({"bytes": 340000, "truncated": self.trace_truncated, "reason": "stub window"})
        return summary

    def frame(self):
        return {"payload_profile_status": self.payload_status, "phase": self.phase,
                "cmd_ack_seq": self._seq}


def a_lock(inventory: dict, max_trials: int = 30) -> dict:
    """An eight-cell grid (4 x 2): deliberately not the frozen 16, with the reason stated, because an
    unstated resize is exactly what the ADR refuses — and a runner must refuse on a tampered lock."""
    spec = {
        "campaign_id": "runner-unittest", "objective": "x",
        "dimensions": [{"name": "yaw.current_kp_a_per_rad_s", "levels": [0.8, 1.0, 1.2, 1.6]},
                       {"name": "yaw.current_ki_a_per_rad_s", "levels": [0.4, 0.9]}],
        "refine": {"max_candidates": 8, "gate": "improvement > 0.15"},
        "confirm": {"repeats": 2}, "stop": {"max_trials": max_trials, "no_improvement_rounds": 2},
        "fixed": {"velocity_dps": 6.0},
        # The lock owes the reader four things (00_CODEX_START.md:36, :50); a runner fixture that skipped
        # them would be testing a planner that lets a real campaign skip them too.
        "scorer": {"metric": "jitter_rad_s_pp", "metrics_version": scoring.METRICS_VERSION,
                   "metrics_sha256": scoring.metrics_sha256()},
        "seed": 20260930,
        "geometry_calibration": "BLOCKED_geometry_identity_not_measured_this_session",
        "retry": {"allowed": 1, "same_parameters": True, "same_conditions": True},
        "payload_profile": "no_profile",
        "coarse_count_reason": "a four-cell grid tests the runner; it is not a claim about the machine",
    }
    return plan.freeze(spec, inventory)


BASELINE = [1.0, 0.6, 20.0, 0.0, 0.0, 0.0, 0.0, 2.0]


def bound_binary(lock: dict) -> str:
    """The digest the lock believes is running. Tests pass this so the bindings hold, and the drift
    test overrides it — the runner refusing an unrecognised binary is the behaviour, not a nuisance.
    """
    return lock["bound_to"]["binary_sha256"]


class BeforeTouchingTheMachine(unittest.TestCase):
    def setUp(self):
        self.inventory = plan.load_inventory(plan.INVENTORY)
        self.lock = a_lock(self.inventory)
        self.dir = tempfile.mkdtemp()

    def runner(self, station=None, lock=None, inventory=None, binary_sha=None):
        chosen = lock or self.lock
        return runner.Runner(station or StubStation(), chosen,
                             (inventory or self.inventory)["_sha256"],
                             binary_sha or bound_binary(chosen),
                             list(BASELINE), self.dir)

    def test_a_design_edited_after_freezing_never_touches_the_hardware(self):
        tampered = json.loads(json.dumps(self.lock))
        tampered["design"]["coarse"][0]["params"]["yaw.current_kp_a_per_rad_s"] = 9.0
        station = StubStation()
        manifest = self.runner(station, lock=tampered).run()
        self.assertTrue(any(row.startswith("BLOCKED_lock_tampered") for row in manifest["blocked"]))
        self.assertEqual(0, station.received, "a runner that touched the machine after refusing is "
                                              "not refusing, it is apologising afterwards")

    def test_a_different_parameter_set_blocks_the_campaign(self):
        drifted = json.loads(json.dumps(self.inventory))
        drifted["_sha256"] = "f" * 64
        manifest = self.runner(inventory=drifted).run()
        self.assertTrue(any(row.startswith("BLOCKED_inventory_drift") for row in manifest["blocked"]))

    def test_a_different_binary_blocks_the_campaign(self):
        lock = json.loads(json.dumps(self.lock))
        manifest = self.runner(lock=lock, binary_sha="0" * 64).run()   # a different controld is a
        # different machine: the evidence of this campaign would not be about the one that ran
        self.assertTrue(any(row.startswith("BLOCKED_binary_drift") for row in manifest["blocked"]))

    def test_a_station_carrying_another_payload_does_not_run_this_campaign(self):
        manifest = self.runner(StubStation(payload_status="heavy_lens")).run()
        self.assertTrue(any(row.startswith("BLOCKED_payload_profile_heavy_lens") for row in manifest["blocked"]),
                        "the payload binding is part of the design; a different load is a different "
                        "campaign, not a detail")

    def test_a_station_that_is_moving_is_blocked_not_waited_out_indefinitely(self):
        manifest = self.runner(StubStation(phase="run")).run()
        self.assertTrue(any(row.startswith("BLOCKED_phase_run") for row in manifest["blocked"]))


class ItObysWhatTheLockSays(unittest.TestCase):
    def setUp(self):
        self.inventory = plan.load_inventory(plan.INVENTORY)
        self.lock = a_lock(self.inventory)
        self.station = StubStation()
        self.dir = tempfile.mkdtemp()
        self.manifest = runner.Runner(self.station, self.lock, self.inventory["_sha256"],
                                      bound_binary(self.lock), list(BASELINE), self.dir).run()

    def test_every_candidate_in_the_lock_ran_and_the_runner_stopped_when_the_design_ran_out(self):
        self.assertEqual(8, len(self.lock["design"]["coarse"]), "4 levels x 2 levels, as the spec says")
        self.assertEqual(len(self.lock["design"]["coarse"]), len(self.manifest["trials"]))
        self.assertEqual("design_exhausted", self.manifest["stopped_by"])
        self.assertTrue(os.path.exists(os.path.join(self.dir, "manifest.json")))

    def test_no_candidate_is_left_in_the_machine(self):
        for trial in self.manifest["trials"]:
            self.assertIn("restore", trial, "a trial without a restore moved the machine and called "
                                           "it an experiment")
            self.assertTrue(trial["restore"].get("accepted"), trial["restore"])

    def test_each_trial_was_restored_so_the_next_one_starts_from_the_baseline(self):
        # prepare + apply + snapshot + prepare + apply per candidate: the restore is not optional.
        self.assertEqual(7 * 8, self.station.received,
                         "context, prepare, apply, snapshot, then prepare and apply again to return to "
                         "the baseline: six commands per candidate — the campaign says who it is before "
                         "it writes anything, and the restore is not optional")
        self.assertEqual(self.station.revision, 8 * 2,
                         "one revision per applied candidate and per applied restore")

    def test_an_unscored_trial_is_recorded_as_unscored_rather_than_as_an_improvement(self):
        for trial in self.manifest["trials"]:
            self.assertIsNone(trial["metrics"])
            self.assertIn("does not flatter", trial["unscored_reason"])


class TheTraceWindowCarriesTheCampaign(unittest.TestCase):
    """§5 is satisfied per record or it is not satisfied: an archived row that does not say whose trial
    it was can only be attributed by correlating timestamps with somebody's log.
    """

    def _run_with(self, station):
        inventory = plan.load_inventory(plan.INVENTORY)
        lock = a_lock(inventory)
        with tempfile.TemporaryDirectory() as directory:
            return Runner(station, lock, inventory["_sha256"], bound_binary(lock), list(BASELINE),
                          directory).run()

    def test_a_window_too_big_for_the_buffer_blocks_rather_than_reporting_a_short_run(self):
        station = StubStation()
        station.trace_truncated = True
        manifest = self._run_with(station)
        self.assertTrue(any(row.startswith("BLOCKED_trace_truncated") for row in manifest["blocked"]),
                        manifest["blocked"])

    def test_a_record_without_the_identity_blocks_the_campaign(self):
        station = StubStation()
        station.trace_loses_identity = True
        manifest = self._run_with(station)
        self.assertTrue(any(row.startswith(("BLOCKED_trace_identity_missing", "BLOCKED_trace_identity_absent"))
                            for row in manifest["blocked"]),
                        manifest["blocked"])

    def test_an_empty_window_is_not_a_passing_window(self):
        # The first hardware run passed this check with zero records counted, which is the check saying
        # nothing and being reported as agreement. An empty window now blocks.
        station = StubStation()
        station.trace_rows = 0
        station.trace_history_rows = 0
        manifest = self._run_with(station)
        self.assertTrue(any(row.startswith("BLOCKED_trace_window_empty") for row in manifest["blocked"]),
                        manifest["blocked"])

    def test_an_untagged_but_complete_window_is_not_called_a_failure(self):
        # A station with no campaign context set is a different question from a record losing its tag:
        # the runner reports the count and lets the first refusal (no context) speak, not a fake one.
        station = StubStation()
        manifest = self._run_with(station)
        self.assertEqual([], manifest["blocked"])


class ItStopsWhenTheDesignSaysSo(unittest.TestCase):
    def test_max_trials_is_a_limit_not_a_suggestion(self):
        inventory = plan.load_inventory(plan.INVENTORY)
        # The limit is stated in the spec before freezing: editing a frozen design would be a new
        # campaign, and the runner refuses it as tampering — which it should, including here.
        lock = a_lock(inventory, max_trials=1)
        self.assertEqual(8, len(lock["design"]["coarse"]), "the design asks for eight; the stop says one")
        with tempfile.TemporaryDirectory() as directory:
            manifest = runner.Runner(StubStation(), lock, inventory["_sha256"], bound_binary(lock),
                                     list(BASELINE), directory).run()
        self.assertEqual(1, len(manifest["trials"]))
        self.assertEqual("max_trials", manifest["stopped_by"])


if __name__ == "__main__":
    unittest.main(verbosity=2)


class TheCampaignCanRunAndScore(unittest.TestCase):
    """:46 — a trial that never ran is not a trial, and a score against moved thresholds is not a score."""

    def _run(self, station, mutate_lock=None, **keywords):
        inventory = plan.load_inventory(plan.INVENTORY)
        lock = a_lock(inventory)
        if mutate_lock:
            mutate_lock(lock)
        with tempfile.TemporaryDirectory() as directory:
            return Runner(station, lock, inventory["_sha256"], bound_binary(lock), list(BASELINE),
                          directory, **keywords).run()

    def test_running_trials_drives_the_firmware_guarded_trial_for_every_candidate(self):
        station = StubStation()
        manifest = self._run(station, run_trials=True)
        drove = [call for call in station.calls if call[0] == "yaw_control_trial"]
        self.assertEqual(8, len(drove), [call[0] for call in station.calls][:12])
        self.assertTrue(all((trial.get("restore") or {}).get("accepted") for trial in manifest["trials"]),
                        "whatever the RUN did, the machine has to be left at its baseline")

    def test_a_scored_campaign_records_the_classification_it_awarded(self):
        station = StubStation()
        station.scorable_rows = True
        manifest = self._run(station, run_trials=True, scorer=scoring)
        tally = manifest.get("classifications", {})
        self.assertTrue(tally, manifest["trials"][0].get("score"))
        self.assertEqual(8, sum(tally.values()))

    def test_thresholds_that_moved_under_the_lock_stop_the_campaign(self):
        station = StubStation()
        station.scorable_rows = True
        manifest = self._run(station, lambda lock: lock["bound_to"].update(metrics_sha256="f" * 64),
                             run_trials=True, scorer=scoring)
        self.assertTrue(any(row.startswith("BLOCKED_metrics_version_drift") for row in manifest["blocked"]),
                        manifest["blocked"])
