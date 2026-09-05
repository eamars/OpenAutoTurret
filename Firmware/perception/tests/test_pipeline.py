"""§39/§40/§41 at the frame boundary: timings, preview isolation, publishing, and fault counting.

The pipeline is where the subsystem's promises become observable numbers, so these tests are
mostly about what a number is *allowed* to mean:

* a stage that was never instrumented must appear in ``stages_unmeasured``, not as ``0.0`` — a
  zero percentile reads as "fast", which is the opposite of "not measured" (§40);
* a span whose two stamps are in different clock domains is refused, not published (§40's
  "don't call it a latency you didn't measure");
* a frame that fails must fail as *one frame* — counted, evented, and followed by a good one;
* the preview must never be able to slow the pipeline down (§39), which a depth-one buffer makes
  a structural property rather than a tuning one.
"""
from __future__ import annotations

import json
import os
import tempfile
import unittest
from perception.camera import SENSOR_TIMESTAMP_KEY, CameraOwner
from perception.protocol.track_set import empty_track_set
from perception.config import DedupConfig
from perception.errors import ModelRejected
from perception.events import EventLog, EventType
from perception.model import ModelManifest, MockAdapter, rows_for_moving_target
from perception.pipeline import STAGES, JsonPublisher, PerceptionPipeline, PreviewTap
from perception.replay import ReplaySource, Recorder
from perception.tests.support import (at, commissioned_config, det, dset, ms)


class StubAdapter:
    """A model that does exactly what the test says, including failing on cue.

    It reports (or withholds) stage timings on purpose: §40's honesty about unmeasured stages
    can only be tested by an adapter that measures nothing.
    """

    name = "stub"

    def __init__(self, sets=None, *, fail_frames=(), raise_on=None, timings=None,
                 clock=None, work_ns=1_000_000) -> None:
        self.setes = list(sets or [])
        self.fail_frames = set(fail_frames)
        self.raise_on = raise_on                 # exception to raise, on cue
        self.timings = timings                  # None → report nothing at all
        self.clock = clock or (lambda: 0)
        self.work_ns = int(work_ns)
        self.inferences = 0
        self.requests = []
        self.opened = False
        self.closed = False
        self.last_timings_ms = {}

    def describe(self):
        return {"adapter": self.name, "inferences": self.inferences}

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def infer(self, image, metadata=None, *, frame_sequence, sensor_timestamp_ns,
              publish_timestamp_ns):
        self.inferences += 1
        self.requests.append(int(frame_sequence))
        self.clock_forward = self.clock() + self.work_ns
        if int(frame_sequence) in self.fail_frames and self.raise_on is None:
            return dset([], frame_index=int(frame_sequence),
                        sensor_ns=int(sensor_timestamp_ns),
                        publish_ns=int(publish_timestamp_ns))
        if self.raise_on is not None and int(frame_sequence) in self.fail_frames:
            raise self.raise_on
        if self.timings is not None:
            self.last_timings_ms = dict(self.timings)
        index = len(self.requests) - 1
        if index < len(self.setes):
            return self.setes[index]
        if self.setes:
            return self.setes[-1]
        return dset([], frame_index=int(frame_sequence), sensor_ns=int(sensor_timestamp_ns),
                    publish_ns=int(publish_timestamp_ns))


class FakeClock:
    """A host clock the test advances, so durations mean something."""

    def __init__(self, start: int = 10_000_000_000, step_ns: int = 500_000) -> None:
        self.now = int(start)
        self.step = int(step_ns)

    def __call__(self) -> int:
        self.now += self.step
        return self.now


class PipelineCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.mkdtemp()
        self.addCleanup(lambda: __import__("shutil").rmtree(self.directory, ignore_errors=True))

    def make(self, config=None, *, adapter=None, sets=None, **kwargs):
        self.config = config or commissioned_config()
        self.clock = kwargs.pop("clock", None) or FakeClock()
        self.adapter = adapter if adapter is not None else StubAdapter(
            sets or [dset([det(0, cx=0.4)], frame_index=0)], clock=self.clock)
        self.events = kwargs.pop("events", None) or EventLog(capacity=256)
        return PerceptionPipeline(self.config, adapter=self.adapter, event_log=self.events,
                                  clock=self.clock, session_uuid="session-test", **kwargs)


class TestPreviewTap(unittest.TestCase):
    def test_the_buffer_depth_is_not_configurable(self):
        # §39 says one. A knob here is how the retired tap ended up five frames deep.
        with self.assertRaises(ValueError) as caught:
            PreviewTap(latest_queue_depth=4)
        self.assertIn("depth one", str(caught.exception))

    def test_newest_frame_wins_and_the_old_one_is_counted_as_discarded(self):
        tap = PreviewTap(fps=0)
        self.assertTrue(tap.offer("a"))
        self.assertTrue(tap.offer("b"))
        stats = tap.stats()
        self.assertEqual((stats["offered"], stats["enqueued"], stats["overwritten"]),
                         (2, 2, 1))
        self.assertEqual(tap.take(), "b")
        self.assertIsNone(tap.take(), "the slot drains; nothing is queued behind it")

    def test_a_consumer_that_keeps_up_is_the_only_way_to_see_every_frame(self):
        tap = PreviewTap(fps=10, clock=lambda: 0)      # frozen clock: every offer is early
        for index in range(20):
            tap.offer(index)
        self.assertEqual(tap.stats()["rate_limited"], 19)
        self.assertEqual(tap.take(), 0)

    def test_rate_limiting_measures_the_configured_rate_not_the_frame_rate(self):
        now = {"ns": 0}
        tap = PreviewTap(fps=10, clock=lambda: now["ns"])
        self.assertTrue(tap.offer("first"))
        now["ns"] = ms(0.05)
        self.assertFalse(tap.offer("too soon"))         # 50 ms < 100 ms budget
        now["ns"] = ms(0.11)
        self.assertTrue(tap.offer("in time"))
        self.assertEqual(tap.stats()["rate_limited"], 1)

    def test_a_disabled_tap_accepts_nothing_and_costs_nothing(self):
        tap = PreviewTap(enabled=False, fps=0)
        self.assertFalse(tap.offer("frame"))
        self.assertEqual(tap.stats()["offered"], 0)

    def test_a_none_frame_is_not_an_offered_frame(self):
        tap = PreviewTap(fps=0)
        self.assertFalse(tap.offer(None))
        self.assertEqual(tap.stats()["offered"], 0)


class TestJsonPublisher(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.mkdtemp()
        self.addCleanup(lambda: __import__("shutil").rmtree(self.directory, ignore_errors=True))

    def test_both_documents_are_written_and_replaced_not_appended(self):
        from perception.protocol.track_set import empty_track_set
        from perception.selection.protocol import SelectTargetRequest
        from perception.selection.target_selection_manager import TargetSelectionManager
        from perception.tracking.track_manager import TrackManager
        config = commissioned_config()
        manager_sets = [dset([det(0, cx=0.4)], frame_index=index) for index in range(4)]
        manager = TrackManager(config, session_uuid="s", event_log=EventLog(capacity=64))
        published = [manager.update(one, at(index)) for index, one in enumerate(manager_sets)]
        selector = TargetSelectionManager(config, event_log=EventLog(capacity=64))
        last = published[-1]
        selector.select(SelectTargetRequest(request_id="r", track_uuid=last.tracks[0].track_uuid,
                                           track_set_sequence_seen_by_ui=1),
                        last, last.sensor_timestamp_ns)
        observation = selector.update(last, last.sensor_timestamp_ns)

        publisher = JsonPublisher(self.directory)
        publisher.publish(published[0], observation)
        publisher.publish(last, observation)
        on_disk = json.load(open(publisher.track_set_path, encoding="utf-8"))
        self.assertEqual(on_disk["track_set_sequence"], last.track_set_sequence)
        self.assertEqual(json.load(open(publisher.selected_path,
                                       encoding="utf-8"))["track_uuid"],
                         last.tracks[0].track_uuid)
        self.assertEqual([name for name in os.listdir(self.directory) if name.endswith(".tmp")],
                         [], "an atomic write leaves no partial file behind")

    def test_a_target_that_cannot_be_written_is_a_counted_failure(self):
        # Pointing the document name at a directory is a deterministic write failure that does
        # not depend on the user this test suite runs as.
        blocked = os.path.join(self.directory, "documents")
        os.makedirs(blocked)
        publisher = JsonPublisher(self.directory, track_set_name="documents")
        empty = empty_track_set()
        with self.assertRaises(OSError):
            publisher.publish(empty, None)
        self.assertEqual(publisher.failures, 1)
        self.assertTrue(publisher.last_error)


class TestPipelineFrame(PipelineCase):
    def frames(self, count=4, *, adapter=None, pipeline=None, start=1):
        pipeline = pipeline or self.make()
        outcomes = []
        for index in range(start, start + count):
            outcomes.append(pipeline.process_frame(
                b"image", {"SensorTimestamp": at(index - 1)},
                frame_sequence=index, sensor_timestamp_ns=at(index - 1),
                capture_started_ns=self.clock.now - ms(1.0)))
        return outcomes

    def test_a_frame_produces_the_published_pair_and_every_stage_timing(self):
        # Sensor stamps come from the same fake clock domain as the pipeline's clock, so the
        # end-to-end stage is legitimately measurable here.
        # The fake host clock sits one second *ahead* of the sensor stamps, so the two are in
        # the same domain and the end-to-end stage is legitimately measurable here. (ms() takes
        # seconds: ms(1000) would put the clock 16 minutes ahead, which the clock-domain guard
        # is right to refuse.)
        clock = FakeClock(start=at(0) + ms(1.0), step_ns=200_000)
        adapter = StubAdapter([dset([det(0, cx=0.4)], frame_index=0)],
                              timings={"model_output_parse_ms": 1.5,
                                       "coordinate_normalization_ms": 0.4}, clock=clock)
        pipeline = self.make(adapter=adapter, clock=clock)
        outcomes = self.frames(3, pipeline=pipeline)
        self.assertTrue(all(outcome.published for outcome in outcomes))
        self.assertEqual(outcomes[-1].track_set.tracks[0].class_name, "person")
        self.assertEqual(outcomes[-1].observation.target_state.name, "NO_TARGET")

        timing = pipeline.timing_report()
        for stage in STAGES:
            self.assertIn(stage, timing, stage)
            self.assertEqual(timing[stage]["count"], 3, stage)
            for key in ("p50", "p95", "p99", "max"):
                self.assertIn(key, timing[stage], stage)
        self.assertNotIn("stages_unmeasured", timing)
        # The adapter's own numbers are the recorded ones, not a re-derivation.
        self.assertAlmostEqual(timing["model_output_parse_ms"]["p50"], 1.5, places=6)
        self.assertAlmostEqual(timing["coordinate_normalization_ms"]["max"], 0.4, places=6)

    def test_a_stage_that_was_never_instrumented_is_named_rather_than_zeroed(self):
        pipeline = self.make()                          # stub reports no timings at all
        self.frames(2, pipeline=pipeline)
        timing = pipeline.timing_report()
        # The default fake clock is not in the stamps' domain, so §40's end-to-end stage is
        # legitimately refused here as well — see the cross-domain test for that on its own.
        self.assertEqual(sorted(set(timing["stages_unmeasured"]) - {"sensor_to_publish_ms"}),
                         ["coordinate_normalization_ms", "model_output_parse_ms"])
        self.assertNotIn("model_output_parse_ms", timing,
                         "an unmeasured stage must not appear with a percentile table")

    def test_the_real_adapter_measures_the_stages_it_claims_to(self):
        manifest = ModelManifest(model_id="unit-test-model", input_width=640,
                                 input_height=640, inference_rate_hz=16.0,
                                 bbox_normalized=True, bbox_order="xy")
        adapter = MockAdapter(manifest, rows_by_frame=lambda index: rows_for_moving_target(
            rows=1, normalized=True))
        adapter.configure_stream(1920, 1080)
        adapter.open()
        pipeline = self.make(adapter=adapter)
        self.frames(3, pipeline=pipeline)
        timing = pipeline.timing_report()
        unmeasured = timing.get("stages_unmeasured", [])
        self.assertNotIn("coordinate_normalization_ms", unmeasured)
        self.assertGreater(timing["coordinate_normalization_ms"]["max"], 0.0)
        # A mock has no tensor read to time, so the parse stage is *unmeasured* — not free.
        # Reporting 0.0 here is how a subsystem ends up with a latency table nobody can trust.
        self.assertIn("model_output_parse_ms", unmeasured)
        self.assertNotIn("model_output_parse_ms", timing)

    def test_a_cross_domain_end_to_end_span_is_refused_not_reported_as_latency(self):
        pipeline = self.make()
        self.frames(2, pipeline=pipeline)               # stamps near at(0), clock at 1e10
        self.assertEqual(pipeline.counters.clock_domain_rejected, 2)
        timing = pipeline.timing_report()
        self.assertIn("sensor_to_publish_ms", timing["stages_unmeasured"])
        self.assertNotIn("sensor_to_publish_ms", timing)

    def test_a_sensor_stamp_in_the_future_is_refused_too(self):
        pipeline = self.make()
        outcome = pipeline.process_frame(b"image", None, frame_sequence=1,
                                         sensor_timestamp_ns=self.clock.now + ms(5_000))
        self.assertTrue(outcome.published)
        self.assertEqual(pipeline.counters.clock_domain_rejected, 1)

    def test_the_recorder_receives_the_frame_before_dedup_ran(self):
        # §43: record what the model said, so a replay can re-ask §16's question. Recording the
        # post-dedup set would make every dedup experiment a tautology.
        recorder_path = os.path.join(self.directory, "rec")
        recorder = Recorder(recorder_path, config=commissioned_config(),
                            model_manifest=ModelManifest(model_id="unit-test-model",
                                                         input_width=640,
                                                         input_height=640)).open()
        twin = dset([det(0, cx=0.40, width=0.10, height=0.30, score=0.9),
                     det(1, cx=0.41, width=0.10, height=0.30, score=0.85)], frame_index=0)
        pipeline = self.make(adapter=StubAdapter([twin]), recorder=recorder)
        outcome = self.frames(1, pipeline=pipeline)[0]
        pipeline.stop()
        self.assertEqual(len(outcome.detection_set.detections), 2, "recorded pre-§16")
        self.assertEqual(len(outcome.track_set.tracks), 1, "one identity published")
        recorded = list(ReplaySource(recorder_path).detection_sets())
        self.assertEqual(len(recorded[0].detections), 2)

    def test_a_model_failure_fails_the_frame_and_is_attributable(self):
        adapter = StubAdapter([dset([det(0, cx=0.4)], frame_index=0)],
                              fail_frames={1}, raise_on=ModelRejected("rpk refused the probe"))
        pipeline = self.make(adapter=adapter)
        outcomes = self.frames(2, pipeline=pipeline)
        failed, good = outcomes
        self.assertFalse(failed.published)
        self.assertEqual(failed.stage, "inference")
        self.assertIn("probe", failed.failure)
        self.assertTrue(good.published, "one bad frame must not end the run")
        self.assertEqual(pipeline.counters.inference_failures, 1)
        self.assertEqual(pipeline.counters.documents_written, 0,
                         "no publisher is configured, so nothing was written")
        self.assertEqual(pipeline.events.count(EventType.MODEL_REJECTED_INCOMPATIBLE), 1)
        self.assertIn(EventType.MODEL_REJECTED_INCOMPATIBLE.value,
                      [event.name for event in pipeline.events.tail()])

    def test_an_unexpected_fault_is_counted_and_evented_not_swallowed(self):
        adapter = StubAdapter([dset([det(0, cx=0.4)], frame_index=0)],
                              fail_frames={1},
                              raise_on=RuntimeError("normalization index out of range"))
        pipeline = self.make(adapter=adapter)
        outcome = self.frames(1, pipeline=pipeline)[0]
        self.assertFalse(outcome.published)
        self.assertIn("RuntimeError", outcome.failure)
        self.assertEqual(pipeline.counters.failures, 1)
        self.assertNotEqual(pipeline.counters.inference_failures, 1,
                            "a bug is not a model rejection")
        self.assertEqual(pipeline.events.counts()[EventType.PIPELINE_FRAME_FAULT.value], 1)

    def test_a_selection_callback_takes_precedence_over_the_file_publisher(self):
        import tempfile as _tempfile
        directory = _tempfile.mkdtemp()
        self.addCleanup(lambda: __import__("shutil").rmtree(directory, ignore_errors=True))
        seen = []
        pipeline = self.make(publisher=JsonPublisher(directory))
        pipeline.process_frame(b"image", None, frame_sequence=1, sensor_timestamp_ns=at(0),
                               publish=lambda track_set, observation: seen.append(track_set))
        self.assertEqual(len(seen), 1)
        self.assertEqual(pipeline.counters.documents_written, 0,
                         "the caller owns publication when it takes the documents itself")
        self.assertFalse(os.path.exists(os.path.join(directory, "track_set.json")))

    def test_a_publisher_that_fails_is_reported_but_does_not_lose_the_frame(self):
        blocked = os.path.join(self.directory, "documents")
        os.makedirs(blocked)
        pipeline = self.make(publisher=JsonPublisher(self.directory,
                                                     track_set_name="documents"))
        outcome = self.frames(1, pipeline=pipeline)[0]
        self.assertTrue(outcome.published, "the frame's work is done; the write is what failed")
        self.assertEqual(pipeline.counters.publish_failures, 1)
        self.assertEqual(pipeline.events.counts()[EventType.PUBLISH_FAILED.value], 1)
        self.assertEqual(pipeline.counters.documents_written, 0,
                         "the counter counts successful writes, not attempts")

    def test_the_preview_sees_one_offer_per_frame_and_never_blocks_the_run(self):
        preview = PreviewTap(fps=0)                     # no rate limit: count the offers
        pipeline = self.make(preview=preview)
        self.frames(5, pipeline=pipeline)
        self.assertEqual(preview.stats()["offered"], 5)
        self.assertEqual(preview.stats()["enqueued"], 5, "a depth-one tap never blocks")
        self.assertEqual(preview.take(), b"image")

    def test_empty_frames_are_a_counted_condition_not_a_missing_frame(self):
        pipeline = self.make(adapter=StubAdapter(fail_frames=set()))   # returns empty sets
        self.frames(3, pipeline=pipeline)
        self.assertEqual(pipeline.counters.frames, 3)
        self.assertEqual(pipeline.counters.empty_frames, 3)
        self.assertEqual(pipeline.counters.frames_with_detections, 0)
        self.assertEqual(pipeline.counters.frames_with_tracks, 0)

    def test_a_dedup_stage_switched_off_by_commission_is_visible_in_the_counters(self):
        # The shipped configuration leaves §16's numbers as COMMISSION, so the stage declines.
        # That has to be observable, or "no duplicates seen" and "no dedup ran" look identical.
        config = commissioned_config(dedup=DedupConfig())
        pipeline = self.make(config=config)
        self.frames(2, pipeline=pipeline)
        self.assertTrue(pipeline.counters.stages_skipped, pipeline.counters.to_dict())
        self.assertTrue(any("nms" in reason for reason in pipeline.counters.stages_skipped),
                        pipeline.counters.stages_skipped)
        self.assertGreaterEqual(sum(pipeline.counters.stages_skipped.values()), 2)

    def test_the_frame_report_sums_up_the_run_without_motor_vocabulary(self):
        pipeline = self.make()
        self.frames(2, pipeline=pipeline)
        report = pipeline.report()
        for key in ("counters", "timing", "preview", "adapter", "selection", "events"):
            self.assertIn(key, report)
        text = json.dumps(report).lower()
        for forbidden in ("motor", "turret", "lead_angle", "lead_time", "aim_point"):
            self.assertNotIn(forbidden, text, f"§36: {forbidden} does not belong to perception")

    def test_stopping_closes_the_recorder_once(self):
        recorder = Recorder(os.path.join(self.directory, "rec"), config=commissioned_config(),
                            model_manifest=ModelManifest(model_id="unit-test-model",
                                                         input_width=640,
                                                         input_height=640)).open()
        pipeline = self.make(adapter=StubAdapter([dset([det(0, cx=0.4)], frame_index=0)]),
                             recorder=recorder)
        self.frames(2, pipeline=pipeline)
        pipeline.stop()
        pipeline.stop()                                 # idempotent: no double-close noise
        self.assertEqual(recorder.stats()["frames"], 2)
        self.assertTrue(recorder.stats()["closed"])
        source = ReplaySource(os.path.join(self.directory, "rec"))
        self.assertEqual(len(list(source.detection_sets())), 2)
        self.assertEqual(len(source.observations()), 2)

    def test_frame_outcome_serialises_the_way_the_log_reads_it(self):
        pipeline = self.make()
        outcome = self.frames(1, pipeline=pipeline)[0]
        payload = outcome.to_dict()
        self.assertEqual(payload["target_state"], "NO_TARGET")
        self.assertTrue(payload["published"])
        self.assertEqual(payload["tracks"], 1)
        self.assertIsInstance(payload["timings_ms"], dict)


class FakeRequest:
    def __init__(self, image, metadata, *, raise_on_array=False):
        self.image = image
        self.metadata = metadata
        self.released = 0
        self.raise_on_array = raise_on_array

    def make_array(self, stream):
        if self.raise_on_array:
            raise RuntimeError("buffer gone")
        return self.image

    def get_metadata(self):
        return self.metadata

    def release(self):
        self.released += 1


class FakeCamera:
    """A stand-in for Picamera2: hands out prepared requests, and never keeps one alive."""

    def __init__(self, requests):
        self.requests = list(requests)
        self.handed_out = []
        self.stopped = False
        self.closed = False

    def capture_request(self, show=False):
        if not self.requests:
            raise AssertionError("test camera ran dry")
        request = self.requests.pop(0)
        self.handed_out.append(request)
        return request

    def stop(self):
        self.stopped = True

    def close(self):
        self.closed = True


class TestCameraOwner(unittest.TestCase):
    def owner(self, requests, **kwargs):
        kwargs.setdefault("stream_size", (1920, 1080))
        kwargs.setdefault("clock", FakeClock(start=at(0), step_ns=ms(0.06)))
        self.camera = FakeCamera(requests)
        events = kwargs.pop("events", None)
        if events is None:                              # `or` would replace an empty log
            events = EventLog(capacity=64)
        return CameraOwner(self.camera, events=events, **kwargs)

    def stamped(self, sensor_ns, image=b"pixels"):
        return FakeRequest(image, {SENSOR_TIMESTAMP_KEY: sensor_ns})

    def test_a_stamped_frame_carries_the_sensor_time_and_a_sequence_number(self):
        owner = self.owner([self.stamped(at(0)), self.stamped(at(1))])
        first = owner.next_frame()
        second = owner.next_frame()
        self.assertTrue(first.usable)
        self.assertEqual(first.sensor_timestamp_ns, at(0))
        self.assertEqual((first.frame_sequence, second.frame_sequence), (1, 2))
        self.assertEqual(first.stream_size, (1920, 1080))
        self.assertEqual(owner.stats.delivered, 2)
        self.assertEqual([request.released for request in self.camera.handed_out], [1, 1],
                         "every request is released, in order, exactly once")

    def test_every_metadata_shape_libcamera_has_used_is_accepted(self):
        shapes = [
            at(0),                                        # plain int of ns
            float(at(0)),                                 # float of ns
            (at(0), True),                                # value + validity flag
            type("Stamp", (), {"timestamp": at(0)})(),     # wrapped timestamp object
            type("Stamp", (), {"iso8601": "2024-01-01T00:00:00.000000+00:00"})(),
        ]
        for shape in shapes:
            with self.subTest(shape=type(shape).__name__):
                owner = self.owner([FakeRequest(b"pixels", {SENSOR_TIMESTAMP_KEY: shape})])
                frame = owner.next_frame()
                self.assertTrue(frame.usable, shape)
                if not isinstance(shape, str):
                    self.assertGreater(frame.sensor_timestamp_ns, 0)

    def test_a_missing_stamp_makes_the_frame_unusable_rather_than_stamped_with_wall_clock(self):
        # Substituting now() here would silently feed §19's lifecycle a clock that is not the
        # sensor's, which is the bug this whole subsystem was written to remove.
        events = EventLog(capacity=16)
        owner = self.owner([FakeRequest(b"pixels", {"FrameTime": 123})], events=events)
        frame = owner.next_frame()
        self.assertFalse(frame.usable)
        self.assertIn("SensorTimestamp", frame.unusable_reason)
        self.assertEqual(owner.stats.missing_sensor_timestamp, 1)
        self.assertEqual(owner.frame_sequence, 0, "an unusable frame is not a frame")
        self.assertEqual(self.camera.handed_out[0].released, 1, "the request is still released")
        self.assertEqual(events.counts()[EventType.CAMERA_FRAME_STALLED.value], 1)

    def test_a_shape_that_is_not_a_timestamp_is_not_guessed_at(self):
        for value in (None, True, "not a time", {}, []):
            with self.subTest(value=value):
                owner = self.owner([FakeRequest(b"pixels", {SENSOR_TIMESTAMP_KEY: value})])
                self.assertFalse(owner.next_frame().usable)

    def test_the_request_is_released_even_when_the_array_call_explodes(self):
        owner = self.owner([FakeRequest(None, {SENSOR_TIMESTAMP_KEY: at(0)},
                                        raise_on_array=True)])
        with self.assertRaises(RuntimeError):
            owner.next_frame()
        self.assertEqual(self.camera.handed_out[0].released, 1,
                         "a leaked request pins a buffer and stops delivery")

    def test_a_gap_beyond_the_timeout_is_reported_once_per_arrival(self):
        clock = FakeClock(start=at(0), step_ns=ms(2.0))
        owner = self.owner([self.stamped(at(0)), self.stamped(at(1))], clock=clock,
                           stall_timeout_ms=1500.0)
        owner.next_frame()
        owner.next_frame()
        self.assertEqual(owner.stats.stalled, 1)
        self.assertGreater(owner.stats.max_gap_ms, 1500.0)

    def test_iteration_stops_at_the_frame_count_and_can_show_the_broken_ones(self):
        owner = self.owner([self.stamped(at(0)),
                            FakeRequest(b"pixels", {}),
                            self.stamped(at(1)),
                            self.stamped(at(2))])
        frames = list(owner.frames(max_frames=2))
        self.assertEqual([frame.frame_sequence for frame in frames], [1, 2])
        # With skip_unusable, an unstamped frame is yielded instead of vanishing: §41's rule is
        # that a loss is visible, and a silently-absent frame is indistinguishable from a scene
        # with nobody in it.
        owner_two = self.owner([FakeRequest(b"pixels", {}), self.stamped(at(0))])
        everything = list(owner_two.frames(max_frames=1, skip_unusable=False))
        self.assertEqual(len(everything), 2)
        self.assertFalse(everything[0].usable)
        self.assertTrue(everything[1].usable)

    def test_the_preview_tap_sees_the_frame_from_the_capture_side(self):
        preview = PreviewTap(fps=0)
        owner = self.owner([self.stamped(at(0))], preview=preview)
        owner.next_frame()
        self.assertEqual(preview.stats()["enqueued"], 1)
        self.assertEqual(preview.take(), b"pixels")

    def test_a_closed_camera_reports_itself_as_unusable_instead_of_raising(self):
        owner = self.owner([self.stamped(at(0))])
        owner.close()
        self.assertTrue(self.camera.stopped and self.camera.closed)
        frame = owner.next_frame()
        self.assertFalse(frame.usable)
        self.assertIn("closed", frame.unusable_reason)

    def test_close_survives_a_device_that_raises_on_the_way_out(self):
        class Angry(FakeCamera):
            def stop(self):
                raise RuntimeError("device gone")

        self.camera = Angry([self.stamped(at(0))])
        owner = CameraOwner(self.camera, stream_size=(1920, 1080), clock=FakeClock())
        owner.close()
        self.assertTrue(owner.stats.notes)

    def test_a_stream_size_cannot_be_guessed(self):
        with self.assertRaises(Exception) as caught:
            CameraOwner(FakeCamera([]), stream_size=(0, 0))
        self.assertIn("stream size", str(caught.exception))


if __name__ == "__main__":                                       # pragma: no cover
    unittest.main()
