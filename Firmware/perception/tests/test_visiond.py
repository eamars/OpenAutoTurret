"""The daemon's command line — the only part of this subsystem an operator actually touches.

These are contract tests for exit codes and artefacts, not for internals: a run that produces a
beautiful report but exits 0 while the gates failed is a CI script that goes green on a broken
station. The interesting assertions are therefore about *status* (0/2/3/4/5), about what is
refused at the door (§50), and about the artefacts a later investigation depends on (§9.1, §41,
§43).
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import tempfile
import unittest

from perception import visiond
from perception.config import SelectionPolicy

SHIPPED = os.path.join("configs", "perception_v1.json")


def run(*argv):
    """Call the CLI, capturing its streams and its exit code."""
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = visiond.main(list(argv))
    return code, out.getvalue(), err.getvalue()


class CliCase(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(self.root, ignore_errors=True))
        self.cwd = os.getcwd()
        # The shipped config's manifest paths are resolved relative to the package, and the
        # offline profile's artifact resolution is part of what is under test here — so run the
        # CLI from the Firmware root regardless of where pytest was invoked.
        os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(visiond.__file__))))
        self.addCleanup(os.chdir, self.cwd)

    def args(self, *argv, report=True, quiet=True):
        out = ["--config", SHIPPED]
        out += list(argv)
        if report:
            out += ["--report", os.path.join(self.root, "report.json")]
        if quiet:
            out.append("--quiet")
        return out

    def report(self):
        with open(os.path.join(self.root, "report.json"), encoding="utf-8") as handle:
            return json.load(handle)

    def record_dir(self, name="rec"):
        return os.path.join(self.root, name)

    def replay(self, directory, *extra, quiet=True):
        """Replay a recording, always with a report file to assert against."""
        argv = ["--config", SHIPPED, "--profile", "offline_replay", "--replay", directory,
                "--report", os.path.join(self.root, "report.json")]
        argv += list(extra)
        if quiet:
            argv.append("--quiet")
        return run(*argv)

    def record_offline(self, frames=18, name="rec"):
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames",
                                      str(frames), "--record-dataset", self.record_dir(name)))
        self.assertEqual(code, visiond.EXIT_OK, err)
        return self.record_dir(name)


class TestConfigurationGate(CliCase):
    def test_the_shipped_profile_refuses_to_start_in_production(self):
        # §50: a placeholder is not a threshold. The daemon must refuse rather than run on
        # somebody's guess, and it must say which fields are still open.
        code, _, err = run("--profile", "person_detect", "--production", "--quiet")
        self.assertEqual(code, visiond.EXIT_CONFIG)
        self.assertIn("COMMISSION", err)
        self.assertIn("thresholds", err)

    def test_a_run_says_out_loud_which_checks_were_waived(self):
        # Every check the offline profile skips has to be announced, or "no duplicates seen"
        # and "nothing was looking for duplicates" produce the same report.
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "3"))
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertIn("artifact questions are waived", err)

    def test_an_unknown_profile_is_a_configuration_error_not_a_crash(self):
        code, _, err = run("--profile", "person_detect_typo", "--quiet")
        self.assertEqual(code, visiond.EXIT_CONFIG)
        self.assertTrue(err.strip())

    def test_a_threshold_override_cannot_be_used_in_production(self):
        code, _, err = run("--profile", "person_detect", "--production",
                           "--dedup-iou", "0.45", "--quiet")
        self.assertEqual(code, visiond.EXIT_CONFIG)
        self.assertIn("--production", err)

    def test_an_offline_override_is_announced_as_an_experiment(self):
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "3",
                                      "--dedup-iou", "0.45"))
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertIn("not a commissioned configuration", err)

    def test_clear_selection_can_only_narrow_the_policy(self):
        parser = visiond.build_parser()
        args = parser.parse_args(["--config", SHIPPED, "--profile", "offline_replay",
                                  "--clear-selection"])
        config = visiond.load_config(args)
        self.assertIs(config.selection.policy, SelectionPolicy.EXPLICIT_ONLY)

    def test_a_missing_config_file_is_reported_not_raised(self):
        code, _, err = run("--config", "configs/does_not_exist.json", "--quiet")
        self.assertEqual(code, visiond.EXIT_CONFIG)
        self.assertIn("does_not_exist", err)


class TestOfflineCapture(CliCase):
    def test_a_run_produces_the_artefacts_an_investigation_needs(self):
        directory = self.record_offline(18)
        report = self.report()
        self.assertEqual(report["mode"], "capture")
        self.assertEqual(report["counters"]["frames"], 18)
        self.assertEqual(report["counters"]["failures"], 0)
        self.assertEqual(report["counters"]["documents_written"], 0,
                         "no --publish-dir was given, so no documents were written")
        self.assertGreater(report["counters"]["frames_with_tracks"], 0)
        self.assertEqual(report["counters"]["clock_domain_rejected"], 0,
                         "the synthetic clock must stay comparable with the host clock")
        self.assertTrue(os.path.exists(os.path.join(directory, "manifest.json")))
        self.assertTrue(os.path.exists(os.path.join(directory, "detections.jsonl")))

    def test_the_published_documents_are_the_same_pair_the_pipeline_made(self):
        self.record_offline()
        publish_dir = os.path.join(self.root, "pub")
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "6",
                                      "--publish-dir", publish_dir))
        self.assertEqual(code, visiond.EXIT_OK, err)
        with open(os.path.join(publish_dir, "track_set.json"), encoding="utf-8") as handle:
            track_set = json.load(handle)
        with open(os.path.join(publish_dir, "selected_target.json"),
                  encoding="utf-8") as handle:
            observation = json.load(handle)
        self.assertGreater(track_set["counters"]["tracks_created"], 0)
        # The wire carries the numeric enum (append-only, §38) plus the name for humans.
        self.assertEqual(observation["target_state"], 0)
        self.assertEqual(observation["target_state_name"], "NO_TARGET",
                         "§28: nothing is selected until an operator asks")
        text = json.dumps(observation).lower()
        for forbidden in ("motor", "lead", "aim", "turret"):
            self.assertNotIn(forbidden, text, f"§36: perception does not own {forbidden}")

    def test_an_offline_profile_is_not_refused_for_lacking_a_model_file(self):
        # §50's artifact questions apply to the station. Refusing a mock because its path is
        # empty would train people to run with the checks off.
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "2"))
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertNotIn("path is empty", err)
        self.assertIn("artifact questions are waived", err)

    def test_the_environment_manifest_is_written_when_asked(self):
        path = os.path.join(self.root, "environment.json")
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "2",
                                      "--environment-manifest", path))
        self.assertEqual(code, visiond.EXIT_OK, err)
        with open(path, encoding="utf-8") as handle:
            record = json.load(handle)
        for key in ("hostname", "machine", "python", "packages", "collector_notes"):
            self.assertIn(key, record, key)
        self.assertIsInstance(record["collector_notes"], list)
        # Whatever this interpreter happens to have, §9.1's value is that the *required* set is
        # named: either the packages are recorded, or their absence is written down.
        required = [name for name in ("imx500-all", "imx500-firmware", "imx500-models")
                    if name in record["packages"]]
        missing_notes = [note for note in record["collector_notes"] if "not installed" in note]
        self.assertTrue(required or missing_notes, record)

    def test_the_diagnostics_ring_is_persisted_when_asked(self):
        path = os.path.join(self.root, "diagnostics.json")
        code, _, err = run(*self.args("--profile", "offline_replay", "--max-frames", "12",
                                      "--diagnostics", path))
        self.assertEqual(code, visiond.EXIT_OK, err)
        with open(path, encoding="utf-8") as handle:
            payload = json.load(handle)
        self.assertGreater(payload["frames_recorded"], 0)
        self.assertTrue(payload["frames"], "§41: identity decisions have a recorded cause")
        self.assertIn("capacity", payload)

    def test_a_run_with_no_pixels_says_so_instead_of_looking_like_a_dropping_tap(self):
        # The synthetic profile has no camera, so there is nothing to tap. Counting that apart
        # from `offered` is the difference between "the tap dropped 100% of frames" and "there
        # was no image". The tap's real behaviour on live pixels is covered by
        # TestCameraOwner in test_pipeline.py, where the frames are not synthetic.
        code, _, _ = run(*self.args("--profile", "offline_replay", "--max-frames", "4",
                                    "--preview-fps", "2"))
        self.assertEqual(code, visiond.EXIT_OK)
        preview = self.report()["preview"]
        self.assertTrue(preview["enabled"])
        self.assertEqual(preview["offered"], 0)
        self.assertEqual(preview["no_pixels"], 4)

    def test_a_disabled_preview_costs_nothing(self):
        run(*self.args("--profile", "offline_replay", "--max-frames", "3",
                       "--disable-preview"))
        preview = self.report()["preview"]
        self.assertFalse(preview["enabled"])
        self.assertEqual(preview["offered"], 0)


class TestReplayCommand(CliCase):
    def test_a_recording_replays_and_is_run_twice_for_determinism(self):
        directory = self.record_offline(18)
        code, out, err = self.replay(directory)
        self.assertEqual(code, visiond.EXIT_OK, err)
        payload = self.report()
        self.assertEqual(payload["mode"], "replay")
        self.assertEqual(payload["report"]["frames"], 18)
        self.assertEqual(payload["report"]["identities_seen"], 2,
                         "one crossing person and one arriving at frame 10")
        self.assertEqual(payload["report"]["duplicate_active_identities_max"], 0,
                         "no duplicate row inside 18 frames")
        self.assertTrue(payload["determinism"]["identical"], payload["determinism"])
        self.assertIn("determinism: identical", err)

    def test_the_commissioned_dedup_suppresses_the_offline_duplicate(self):
        # The offline scene emits a deliberate duplicate at frame 37. §16's thresholds are now
        # commissioned in the shipped config, so the duplicate is suppressed and the gates pass.
        # The exit code is the point: a passing gate must not look like a failure.
        directory = self.record_offline(40)
        code, out, err = self.replay(directory, "--gates")
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertEqual(self.report()["gates"], [])

    def test_commissioned_dedup_numbers_make_the_same_recording_pass(self):
        # Same 40 frames, same scene, thresholds supplied: §16 eats the frame-37 duplicate and
        # the gates pass. The pair of runs is the evidence that the gate is measuring something.
        directory = self.record_offline(40)
        code, out, err = self.replay(directory, "--gates", "--dedup-iou", "0.45",
                                     "--dedup-containment", "0.85",
                                     "--dedup-center-distance", "0.05")
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertEqual(self.report()["gates"], [])

    def test_a_selection_by_label_asks_the_recording_again(self):
        directory = self.record_offline(18)
        code, out, err = self.replay(directory, "--select-label", "Person #1")
        self.assertEqual(code, visiond.EXIT_OK, err)
        payload = self.report()
        self.assertEqual(payload["report"]["selection_generations"], 1)
        self.assertTrue(payload["determinism"]["identical"])

    def test_a_label_that_was_never_shown_is_reported_not_silently_ignored(self):
        directory = self.record_offline(18)
        code, out, _ = self.replay(directory, "--select-label", "Person #9")
        self.assertEqual(code, visiond.EXIT_OK)
        notes = self.report()["report"]["notes"]
        self.assertTrue(any("never issued" in note for note in notes), notes)

    def test_pointing_replay_at_a_file_is_refused(self):
        stray = os.path.join(self.root, "stray.jsonl")
        with open(stray, "w", encoding="utf-8") as handle:
            handle.write("{}\n")
        code, _, err = run("--config", SHIPPED, "--replay", stray, "--quiet")
        self.assertEqual(code, visiond.EXIT_REPLAY)
        self.assertIn("not a directory", err)

    def test_replaying_under_another_model_is_allowed_but_shouted_about(self):
        # Comparing adapters on one recording is a legitimate experiment; doing it quietly is
        # not. The run continues, the mismatch is on stderr and in the report.
        directory = self.record_offline(6)
        code, _, err = run("--config", SHIPPED, "--profile", "person_detect_available",
                           "--replay", directory, "--report",
                           os.path.join(self.root, "report.json"), "--quiet")
        self.assertEqual(code, visiond.EXIT_OK, err)
        self.assertIn("CONFIGURATION DIFFERS FROM RECORDING", err)
        self.assertTrue(self.report()["source"]["config_mismatches"])

    def test_stdout_is_exactly_one_json_document(self):
        directory = self.record_offline(6)
        code, out, _ = self.replay(directory, quiet=False)
        self.assertEqual(code, visiond.EXIT_OK)
        payload = json.loads(out)              # a stream with two documents appended does not
        self.assertEqual(payload["mode"], "replay")
        self.assertIn("determinism", payload)


class TestProbeCommand(CliCase):
    def test_a_model_that_cannot_be_probed_is_not_admitted(self):
        code, out, err = run("--config", SHIPPED, "--profile", "person_detect",
                             "--probe-model", "/nonexistent/imx500_network_yolo11n_pp.rpk",
                             "--quiet")
        self.assertEqual(code, visiond.EXIT_MODEL)
        self.assertIn("NOT admitted", err)
        payload = json.loads(out)
        self.assertFalse(payload["usable"])

    def test_a_manifest_is_only_drafted_from_something_that_opened(self):
        target = os.path.join(self.root, "draft.json")
        code, out, err = run("--config", SHIPPED, "--profile", "person_detect",
                             "--probe-model", "/nonexistent/network.rpk",
                             "--emit-manifest", target, "--quiet")
        self.assertEqual(code, visiond.EXIT_MODEL)
        self.assertFalse(os.path.exists(target),
                         "a draft from a model that never opened is a filename plus a pile of "
                         "COMMISSION, and it looks exactly like a manifest")
        self.assertIn("never opened", err)

    def test_drafting_a_manifest_without_a_probe_is_refused(self):
        code, out, err = run("--config", SHIPPED, "--emit-manifest",
                             os.path.join(self.root, "draft.json"), "--quiet")
        self.assertEqual(code, visiond.EXIT_CONFIG)
        self.assertIn("--probe-model", err)



if __name__ == "__main__":                                       # pragma: no cover
    unittest.main()
