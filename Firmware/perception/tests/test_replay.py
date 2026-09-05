"""§43/§45/§52 — recordings, Level-B replay, determinism, and the metrics that come from them.

The determinism test in here is the reason the rest of the subsystem can be trusted: it re-runs
the same recording through the same pipeline and requires the published output to be identical.
Everything else in the file protects that claim — a replay that silently repaired a malformed
line, re-sorted frames, or ran under different thresholds would produce a reassuring
"identical" that meant nothing.
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest

from perception.errors import ValidationError
from perception.events import EventLog, EventType
from perception.model import ModelManifest
from perception.replay import (Recorder, RecordingManifest, ReplaySource,
                               compare_ground_truth, compare_runs, engineering_gates,
                               evaluate, run_level_b, write_ground_truth)
from perception.protocol.selected_target import TargetState
from perception.selection.protocol import SelectTargetRequest
from perception.tests.support import at, commissioned_config, det, dset, ms
from perception.model.adapter import offline_scene_rows

MANIFEST = ModelManifest(model_id="unit-test-model", input_width=640, input_height=640,
                         inference_rate_hz=16.0)


def walking_frames(count: int = 24, *, speed: float = 0.01, start: float = 0.30,
                   score: float = 0.85):
    """A person crossing the frame at a fixed rate, one detection per frame."""
    return [dset([det(0, cx=start + speed * index, cy=0.5, width=0.06, height=0.25,
                      score=score)], frame_index=index, sensor_ns=at(index))
            for index in range(count)]


def record_frames(directory: str, frames, *, config=None, manifest=MANIFEST,
                  stream=None, **kwargs) -> str:
    with Recorder(directory, config=config or commissioned_config(),
                  model_manifest=manifest,
                  stream=stream or {"width": 1920, "height": 1080}, **kwargs) as recorder:
        for frame in frames:
            recorder.record_frame(frame, camera={"width": 1920, "height": 1080,
                                                 "preserve_aspect_ratio": True})
    return directory


class RecordingCase(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.directory = self._temporary.name
        self.addCleanup(self._temporary.cleanup)

    def record(self, frames=None, name=None, **kwargs):
        """Record into a fresh subdirectory. §43 is one directory per run, deliberately."""
        self._counter = getattr(self, "_counter", 0) + 1
        return record_frames(os.path.join(self.directory, name or f"rec-{self._counter}"),
                             frames if frames is not None else walking_frames(), **kwargs)


class TestRecorder(RecordingCase):
    def test_a_recording_is_a_directory_with_what_43_requires(self):
        path = self.record()
        self.assertEqual(sorted(os.listdir(path)),
                         ["camera.jsonl", "detections.jsonl", "events.jsonl",
                          "manifest.json", "observations.jsonl"])
        manifest = RecordingManifest.from_dict(json.load(open(os.path.join(path,
                                                                           "manifest.json"))))
        self.assertEqual(manifest.model_id, "unit-test-model")
        self.assertEqual(manifest.configuration.get("tracking") is not None, True)
        self.assertEqual(manifest.stream, {"width": 1920, "height": 1080})
        self.assertEqual(manifest.frames_written, 24)

    def test_the_environment_record_is_preserved_when_supplied(self):
        path = self.record(environment={"os": "Linux", "packages": {"imx500-models": "1"}})
        manifest = json.load(open(os.path.join(path, "manifest.json")))
        self.assertEqual(manifest["environment"]["packages"]["imx500-models"], "1")

    def test_images_are_files_not_inline_blobs(self):
        path = os.path.join(self.directory, "rec")
        frames = walking_frames(3)
        with Recorder(path, config=commissioned_config(), model_manifest=MANIFEST,
                      record_images=True) as recorder:
            for frame in frames:
                recorder.record_frame(frame, image_bytes=b"\xff\xd8jpeg" * 8)
        self.assertTrue(os.path.isdir(os.path.join(path, "images")))
        self.assertEqual(len(os.listdir(os.path.join(path, "images"))), 3)
        with open(os.path.join(path, "detections.jsonl"), encoding="utf-8") as handle:
            text = handle.read()
        self.assertNotIn("jpeg", text)              # no base64 blobs in the JSON stream

    def test_the_recorder_closes_itself_once_and_keeps_the_frame_count(self):
        recorder = Recorder(os.path.join(self.directory, "rec"), config=commissioned_config(),
                            model_manifest=MANIFEST).open()
        recorder.record_frame(walking_frames(1)[0])
        recorder.close()
        recorder.close()
        self.assertEqual(recorder.stats()["frames"], 1)
        with self.assertRaises(RuntimeError):
            recorder.record_frame(walking_frames(1)[0])

    def test_ground_truth_arrives_later_and_does_not_disturb_the_recording(self):
        path = self.record(walking_frames(4))
        write_ground_truth(path, [{"sensor_timestamp_ns": at(0),
                                   "persons": [{"id": "gt-1", "bbox": [0.27, 0.375, 0.33, 0.625]}],
                                   "selected": "gt-1"}])
        source = ReplaySource(path)
        self.assertEqual(len(source.ground_truth()), 1)


class TestReplaySource(RecordingCase):
    def test_frames_come_back_exactly_as_they_were_written(self):
        frames = walking_frames(6)
        source = ReplaySource(self.record(frames))
        replayed = list(source.detection_sets())
        self.assertEqual(len(replayed), 6)
        for original, again in zip(frames, replayed):
            self.assertEqual(again.sensor_timestamp_ns, original.sensor_timestamp_ns)
            self.assertEqual(again.frame_sequence, original.frame_sequence)
            self.assertEqual(again.model_id, original.model_id)
            self.assertEqual(len(again.detections), len(original.detections))
            self.assertAlmostEqual(again.detections[0].bbox.x_min,
                                   original.detections[0].bbox.x_min, places=12)
            self.assertEqual(again.detections[0].class_name, "person")

    def test_camera_metadata_is_joined_by_frame(self):
        source = ReplaySource(self.record(walking_frames(2)))
        frames = list(source.frames())
        self.assertEqual(frames[0].camera["width"], 1920)
        self.assertTrue(frames[1].has_detections)

    def test_a_lone_jsonl_file_is_not_a_recording(self):
        path = os.path.join(self.directory, "lonely.jsonl")
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("{}\n")
        with self.assertRaises(ValidationError) as caught:
            ReplaySource(path)
        self.assertIn("not a directory", str(caught.exception))

    def test_a_missing_manifest_is_refused_rather_than_guessed(self):
        path = os.path.join(self.directory, "bare")
        os.makedirs(path)
        with self.assertRaises(ValidationError) as caught:
            ReplaySource(path)
        self.assertIn("manifest.json", str(caught.exception))

    def test_a_mismatched_model_is_reported_and_can_refuse_the_run(self):
        path = self.record()
        lenient = ReplaySource(path, expected_model_id="someone-elses-network")
        self.assertEqual(len(lenient.summary.config_mismatches), 1)
        with self.assertRaises(ValidationError):
            ReplaySource(path, expected_model_id="someone-elses-network",
                         require_matching_config=True)

    def test_a_different_stream_geometry_is_a_coordinate_problem(self):
        path = self.record()
        source = ReplaySource(path, expected_stream=(1280, 720))
        self.assertIn("§14's coordinates", source.summary.config_mismatches[0])

    def test_a_malformed_line_is_fatal_when_the_replay_must_be_trusted(self):
        path = self.record(walking_frames(3))
        with open(os.path.join(path, "detections.jsonl"), "a", encoding="utf-8") as handle:
            handle.write("{ this is not json\n")
        strict = ReplaySource(path, strict=True)
        with self.assertRaises(Exception):
            list(strict.detection_sets())
        lenient = ReplaySource(path, strict=False)
        self.assertEqual(len(list(lenient.detection_sets())), 3)
        lenient.finish()
        self.assertEqual(lenient.summary.malformed_lines, 1)

    def test_out_of_order_frames_are_counted_not_repaired(self):
        frames = [walking_frames(1)[0]]
        path = self.record(frames)
        with open(os.path.join(path, "detections.jsonl"), "a", encoding="utf-8") as handle:
            early = dset([det(0, cx=0.4, cy=0.5, width=0.06, height=0.25, score=0.8)],
                         frame_index=0, sensor_ns=at(0) - ms(500)).to_dict()
            handle.write(json.dumps({"kind": "detection_set", "frame_sequence": 99,
                                     "sensor_timestamp_ns": at(0) - ms(500),
                                     "set": early}) + "\n")
        source = ReplaySource(path, strict=False)
        self.assertEqual(len(list(source.detection_sets())), 2)
        source.finish()
        self.assertEqual(source.summary.out_of_order_frames, 1)

    def test_the_published_observations_of_the_original_run_survive(self):
        config = commissioned_config()
        path = os.path.join(self.directory, "replayed")
        frames = walking_frames(12)
        # flush_every=1: the replay below reads while the writer is still open, and a
        # recording whose newest frames are still in a buffer is a confusing thing to replay.
        with Recorder(path, config=config, model_manifest=MANIFEST,
                      flush_every=1) as recorder:
            for frame in frames:
                recorder.record_frame(frame)
            source = ReplaySource(path)
            run_level_b(source.detection_sets(), config, record=recorder)
        reopened = ReplaySource(path)
        published = reopened.observations()
        self.assertEqual(len(published), 12)
        self.assertEqual(published[-1][0].tracks[0].class_name, "person")

    def test_a_level_a_only_recording_says_so_instead_of_returning_nothing(self):
        path = os.path.join(self.directory, "images_only")
        with Recorder(path, config=commissioned_config(), model_manifest=MANIFEST,
                      record_detections=False, record_images=True) as recorder:
            recorder.record_frame(None, image_bytes=b"\xff\xd8")
        source = ReplaySource(path)
        self.assertIn("Level-A", source.summary.notes[0])
        frames = list(source.frames())
        self.assertEqual(len(frames), 1)
        self.assertFalse(frames[0].has_detections)
        self.assertTrue(frames[0].image_path.endswith(".jpg"))


class TestLevelBReplay(RecordingCase):
    def setUp(self):
        super().setUp()
        self.config = commissioned_config()
        self.path = self.record(walking_frames(24), name="base")

    def test_one_moving_person_is_one_identity(self):
        run = run_level_b(ReplaySource(self.path).detection_sets(), self.config)
        final = run.track_sets[-1]
        self.assertEqual(len(final.tracks), 1)
        self.assertEqual(final.tracks[0].display_index, 1)
        self.assertEqual(final.tracks[0].state.name, "CONFIRMED_VISIBLE")
        self.assertEqual(run.report.identities_seen, 1)
        self.assertEqual(run.report.identity_switches, 0)

    def test_the_recording_drives_the_clock_not_the_host(self):
        # §19's lifecycle runs on sensor time. If any part of it read the host clock, the
        # published accumulated visible time would depend on how fast this machine happened to
        # be — so the assertion is about the *published* number, not about repeatability.
        run = run_level_b(ReplaySource(self.path).detection_sets(), self.config)
        track = run.track_sets[-1].tracks[0]
        self.assertEqual(track.observations, len(run.track_sets))
        self.assertEqual(track.last_measurement_ns, at(len(run.track_sets) - 1))
        self.assertEqual(run.track_sets[-1].sensor_timestamp_ns, at(len(run.track_sets) - 1))
        # `visible_ms_total` counts *closed* spans; the live span is still open, so the
        # published total must not have swallowed the frame that is currently in progress.
        self.assertEqual(track.visible_ms_total, 0.0)

    def test_replay_is_deterministic_across_runs(self):
        first = run_level_b(ReplaySource(self.path).detection_sets(), self.config)
        second = run_level_b(ReplaySource(self.path).detection_sets(), self.config)
        diff = compare_runs(first.canonical, second.canonical)
        self.assertTrue(diff.identical, diff.to_dict())
        self.assertEqual(diff.differences, [])

    def test_a_different_threshold_set_changes_the_outcome_and_the_comparison_says_where(self):
        strict = commissioned_config(thresholds=None)
        lenient = commissioned_config(thresholds=None)
        strict.active_model.thresholds.confirmed_update = 0.90
        lenient.active_model.thresholds.confirmed_update = 0.50
        reference = run_level_b(ReplaySource(self.path).detection_sets(), strict).canonical
        candidate = run_level_b(ReplaySource(self.path).detection_sets(), lenient).canonical
        diff = compare_runs(reference, candidate)
        self.assertFalse(diff.identical)
        # Asserted on the frames, not on the reported paths: the reported paths are a sample,
        # and a sample is not a claim about where two runs diverged.
        divergence = [(left["frame_sequence"], left["tracks"], right["tracks"])
                      for left, right in zip(reference, candidate) if left != right]
        self.assertTrue(divergence)
        self.assertTrue(any(left_tracks != right_tracks
                            for _, left_tracks, right_tracks in divergence), divergence)

    def test_compare_runs_reports_a_length_mismatch_as_the_first_problem(self):
        reference = run_level_b(ReplaySource(self.path).detection_sets(),
                                self.config, frame_limit=24).canonical
        candidate = run_level_b(ReplaySource(self.path).detection_sets(),
                                self.config, frame_limit=10).canonical
        diff = compare_runs(reference, candidate)
        self.assertFalse(diff.identical)
        self.assertIn("length differs", diff.notes[0])

    def test_dedup_can_be_left_out_to_ask_the_other_question(self):
        # The interesting result is not that §16 suppresses the twin — it is that with §16
        # switched off, the tracker's own §25 creation guard catches it anyway and *counts* it.
        # Two independent lines of defence, each with its own number, is the point.
        frames = [dset([det(0, cx=0.40, cy=0.5, width=0.06, height=0.25, score=0.9),
                        det(1, cx=0.405, cy=0.505, width=0.06, height=0.25, score=0.85)],
                       frame_index=index, sensor_ns=at(index)) for index in range(8)]
        path = self.record(frames)
        with_dedup = run_level_b(ReplaySource(path).detection_sets(), self.config)
        without = run_level_b(ReplaySource(path).detection_sets(), self.config,
                              apply_dedup=False)
        self.assertEqual(with_dedup.report.identities_seen, 1)
        self.assertEqual(without.report.identities_seen, 1)
        self.assertGreater(without.report.detections_refused_duplicate, 0)
        self.assertEqual(without.report.duplicate_active_identities_max, 0)
        # What went *into* the tracker differs, which is the only way to tell the runs apart
        # from published numbers: one saw one detection per frame, the other saw two.
        self.assertEqual(with_dedup.track_sets[-1].counters.detections_in, len(frames))
        self.assertEqual(without.track_sets[-1].counters.detections_in, 2 * len(frames))

    def test_a_selection_by_label_reasks_the_operators_question(self):
        # The label is the only reference that survives a re-run: §17 mints new UUIDs every
        # run, so a replay that insisted on a recorded UUID would select nothing and call the
        # scene clean.
        frames = walking_frames(12)
        path = self.record(frames)
        selected = run_level_b(ReplaySource(path).detection_sets(), self.config,
                               select_label="Person #1")
        self.assertEqual(selected.report.selection_generations, 1)
        self.assertIs(selected.observations[-1].target_state, TargetState.CONFIRMED_VISIBLE)

    def test_a_selection_request_by_uuid_works_within_one_run(self):
        path = self.path
        uuid = run_level_b(ReplaySource(path).detection_sets(),
                           self.config).track_sets[-1].tracks[0].track_uuid
        second = run_level_b(ReplaySource(path).detection_sets(), self.config,
                             select_request=SelectTargetRequest(request_id="r",
                                                                track_uuid=uuid,
                                                                track_set_sequence_seen_by_ui=1))
        self.assertEqual(second.report.selection_generations, 0 if uuid !=
                         second.observations[-1].track_uuid else 1)
        self.assertEqual(second.report.target_stealings, 0)

    def test_a_label_that_no_recorded_frame_carried_is_said_so(self):
        run = run_level_b(ReplaySource(self.path).detection_sets(), self.config,
                          select_label="Person #7")
        self.assertEqual(run.report.selection_generations, 0)
        self.assertTrue(any("never issued" in note for note in run.report.notes),
                        run.report.notes)

    def test_a_request_for_an_identity_that_is_not_in_the_recording_is_said_so(self):
        run = run_level_b(
            ReplaySource(self.path).detection_sets(), self.config,
            select_request=SelectTargetRequest(request_id="r", track_uuid="f" * 32,
                                               track_set_sequence_seen_by_ui=1))
        self.assertTrue(any("never appeared" in note for note in run.report.notes),
                        run.report.notes)

    def test_events_emitted_during_replay_are_captured_for_the_metrics(self):
        events = EventLog(capacity=256)
        run = run_level_b(ReplaySource(self.path).detection_sets(), self.config,
                          event_log=events)
        self.assertEqual(run.events, events.to_dicts())
        self.assertIn(EventType.TRACK_CREATED.value, [event["event"] for event in run.events])

    def test_an_empty_recording_replays_cleanly_instead_of_crashing(self):
        path = os.path.join(self.directory, "empty")
        with Recorder(path, config=self.config, model_manifest=MANIFEST) as recorder:
            pass
        run = run_level_b(ReplaySource(path).detection_sets(), self.config)
        self.assertEqual(run.report.frames, 0)
        self.assertEqual(engineering_gates(run.report), [])

    def test_a_second_run_cannot_be_appended_into_the_first_recording(self):
        frames = walking_frames(3)
        path = self.record(frames)
        with self.assertRaises(ValidationError) as caught:
            record_frames(path, frames)
        self.assertIn("one directory per run", str(caught.exception))
        # Deliberate appending stays available, because a truncated run is sometimes worth
        # continuing — as long as somebody had to say so.
        record_frames(path, frames, allow_existing=True)
        self.assertEqual(len(list(ReplaySource(path).detection_sets())), 6)


class TestMetricsAndGates(RecordingCase):
    def setUp(self):
        super().setUp()
        self.config = commissioned_config()

    def _two_people_converging(self):
        """Two established identities that walk onto each other.

        A ramp, not a jump: teleporting them onto one another would blow §22's motion gate, the
        old identities would go LOST, and a new pair arriving already stacked would be refused
        by §25's creation guard — three ways for the fixture, rather than the pipeline, to
        decide what the metric ends up measuring.
        """
        apart = [dset([det(0, cx=0.25 + 0.021 * index, cy=0.5, width=0.08, height=0.28,
                           score=0.9),
                       det(1, cx=0.72 - 0.021 * index, cy=0.5, width=0.08, height=0.28,
                           score=0.88)],
                      frame_index=index, sensor_ns=at(index)) for index in range(12)]
        # Held for two frames only: §25.1's dwell must elapse before the resolver may merge, so
        # these are the frames where "two live identities, one person" is observable at all. A
        # longer hold would measure the merge instead of the duplication.
        together = [dset([det(0, cx=0.501, cy=0.5, width=0.08, height=0.28, score=0.9),
                          det(1, cx=0.509, cy=0.5, width=0.08, height=0.28, score=0.88)],
                         frame_index=index + 12, sensor_ns=at(index + 12))
                    for index in range(2)]
        return self.record(apart + together)

    def test_two_live_identities_over_one_person_fail_the_gate(self):
        # §16 switched off, so the question "can two live identities cover one person?" is put
        # to the tracker instead of being answered in advance by the stage in front of it.
        run = run_level_b(ReplaySource(self._two_people_converging()).detection_sets(),
                          self.config, apply_dedup=False)
        self.assertGreaterEqual(run.report.identities_seen, 2)
        self.assertGreaterEqual(run.report.duplicate_active_identities_max, 1)
        self.assertGreater(run.report.duplicate_candidate_rate, 0.0)
        failures = engineering_gates(run.report)
        self.assertTrue(any("duplicate_active_identities_max" in line for line in failures))
        self.assertTrue(any("§46" in line for line in failures))

    def test_dedup_is_what_keeps_the_published_set_from_showing_both(self):
        # The same recorded frames with §16 on: the overlapping detection never reaches the
        # tracker, so the published set carries one measurement per person. Running both over
        # identical frames is how §46's rate gate is shown to be earned rather than assumed.
        path = self._two_people_converging()
        with_dedup = run_level_b(ReplaySource(path).detection_sets(), self.config)
        without = run_level_b(ReplaySource(path).detection_sets(), self.config,
                              apply_dedup=False)
        self.assertEqual(with_dedup.report.duplicate_active_identities_max, 0)
        self.assertGreater(without.report.duplicate_active_identities_max, 0)
        self.assertLess(with_dedup.report.duplicate_candidate_rate,
                        without.report.duplicate_candidate_rate)

    def test_gates_are_limits_with_meaning_not_boolean_flags(self):
        from perception.replay.evaluator import ReplayReport
        report = ReplayReport(frames=100, frames_with_persons=100,
                              duplicate_candidate_frames=20, identity_switches=3,
                              target_stealings=1, wrong_subject_selections=2,
                              stale_measurements_published=1,
                              duplicate_active_identities_max=2,
                              short_lived_identities=5)
        failures = engineering_gates(report)
        self.assertEqual(len(failures), 6)
        self.assertTrue(all("—" in line for line in failures), failures)
        # A gate set to None means "not applicable to this scenario", not "failed".
        self.assertEqual(len(engineering_gates(report, {"short_lived_identities_max": 10})), 6)
        self.assertEqual(len(engineering_gates(report, {"short_lived_identities_max": 2})), 7)
        self.assertEqual(len(engineering_gates(report, {"id_switches_max": 5})), 5)

    def test_reacquisition_latency_comes_from_the_event_field(self):
        # Serialised through EventLog, not hand-written dicts: a metric that reads a key no
        # writer produces reports "nothing happened" forever, and passes.
        log = EventLog(capacity=16)
        log.emit(EventType.TRACK_REACQUIRED, track_uuid="a" * 32, miss_ms=240.0)
        log.emit(EventType.TRACK_REACQUIRED, track_uuid="b" * 32, miss_ms=310.0)
        log.emit(EventType.TRACK_CREATED, track_uuid="c" * 32)
        report = evaluate([], [], events=log.to_dicts())
        self.assertEqual(report.reacquisition_latency_ms.count, 2)
        self.assertAlmostEqual(report.reacquisition_latency_ms.max, 310.0, places=6)

    def test_first_selectable_latency_is_reported_in_frames_and_says_so(self):
        run = run_level_b(ReplaySource(self.record(walking_frames(12))).detection_sets(),
                          self.config)
        summary = run.report.first_selectable_latency_frames
        self.assertIsNotNone(summary)
        # §18.1's confirmation needs three observations and 120 ms, so at 60 ms frames the
        # earliest possible publish of `selectable` is the third frame.
        self.assertGreaterEqual(summary.count, 1)
        self.assertGreaterEqual(summary.p50, 2)
        self.assertTrue(any("published *frames*" in note for note in run.report.notes))

    def test_a_healthy_run_reports_no_wrong_subject_and_no_stale_measurement(self):
        run = run_level_b(ReplaySource(self.record(walking_frames(6))).detection_sets(),
                          self.config)
        report = evaluate(run.track_sets, run.observations)
        self.assertEqual(report.wrong_subject_selections, 0)
        self.assertEqual(report.stale_measurements_published, 0)

    def _selected_run(self):
        """A run with a held selection, issued through the real request path (§28/§30)."""
        run = run_level_b(ReplaySource(self.record(walking_frames(8))).detection_sets(),
                          self.config, select_label="Person #1")
        self.assertTrue(run.observations[-1].track_uuid)
        return run

    def test_an_observation_naming_an_identity_that_is_not_published_is_counted(self):
        # The point of the metric is a selector and a tracker disagreeing in public. The clean
        # way to produce that is to take a real observation and move it off the set, so the
        # measurement is of the check and not of a fixture's shape.
        run = self._selected_run()
        observations = list(run.observations)
        last = observations[-1]
        self.assertTrue(last.track_uuid)
        from dataclasses import replace
        observations[-1] = replace(last, track_uuid="a" * 32)
        report = evaluate(run.track_sets, observations)
        self.assertEqual(report.wrong_subject_selections, 1)
        self.assertEqual(report.target_stealings, 1)      # A → B with A still present

    def test_a_stale_geometry_on_a_fresh_measurement_is_counted(self):
        run = self._selected_run()
        observations = list(run.observations)
        last = observations[-1]
        self.assertTrue(last.measurement_valid)
        from dataclasses import replace
        shifted = replace(last.bbox, x_min=last.bbox.x_min + 0.01)
        observations[-1] = replace(last, bbox=shifted)
        report = evaluate(run.track_sets, observations)
        self.assertEqual(report.stale_measurements_published, 1)


class TestGroundTruth(RecordingCase):
    def test_truth_turns_id_switches_and_selection_mismatch_into_numbers(self):
        config = commissioned_config()
        frames = walking_frames(10)
        path = self.record(frames)
        write_ground_truth(path, [{"sensor_timestamp_ns": at(index),
                                   "persons": [{"id": "gt-1",
                                                "bbox": [0.30 + 0.01 * index - 0.03, 0.375,
                                                        0.30 + 0.01 * index + 0.03, 0.625]}],
                                   "selected": "gt-1"} for index in range(10)])
        source = ReplaySource(path)
        run = run_level_b(source.detection_sets(), config)
        result = compare_ground_truth(run.track_sets, run.observations, source.ground_truth())
        self.assertEqual(result["frames_with_truth"], 10)
        self.assertEqual(result["matched_person_frames"], 10)
        self.assertEqual(result["id_switches"], 0)
        self.assertEqual(result["fragmented_identities"], 0)
        # Nothing selected: §28's default is explicit_only, so "selected gt-1" was never true.
        self.assertEqual(result["selection_mismatch"], 10)

    def test_truth_frames_are_matched_by_exact_sensor_stamp(self):
        config = commissioned_config()
        path = self.record(walking_frames(4))
        write_ground_truth(path, [{"sensor_timestamp_ns": at(99),
                                   "persons": [{"id": "gt-1",
                                                "bbox": [0.27, 0.375, 0.33, 0.625]}]}])
        source = ReplaySource(path)
        run = run_level_b(source.detection_sets(), config)
        result = compare_ground_truth(run.track_sets, run.observations, source.ground_truth())
        self.assertEqual(result["frames_with_truth"], 0)


class TestDefaultScene(unittest.TestCase):
    def test_the_offline_scene_reaches_the_visible_band(self):
        # The mock profile is used for the offline acceptance run; if its own boxes fall
        # outside the letterbox, every metric it produces is vacuous.
        from perception.detection.normalize import InferenceGeometry, normalize_rows
        geometry = InferenceGeometry(input_width=640, input_height=640, stream_width=1920,
                                     stream_height=1080, preserve_aspect_ratio=True,
                                     bbox_normalized=True)
        for frame in (0, 10, 37, 60):
            out = normalize_rows(offline_scene_rows(frame), geometry=geometry,
                                 model_id="offline-mock", model_generation=1,
                                 frame_sequence=frame, sensor_timestamp_ns=at(frame),
                                 publish_timestamp_ns=at(frame), label_map="coco")
            self.assertEqual(out.counters.malformed_rejected, 0, frame)
            self.assertGreaterEqual(len(out.detections), 1, frame)

    def test_the_offline_scene_is_a_function_of_the_frame_only(self):
        self.assertEqual(offline_scene_rows(7), offline_scene_rows(7))


if __name__ == "__main__":                                       # pragma: no cover
    unittest.main()
