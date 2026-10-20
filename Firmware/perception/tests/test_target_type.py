"""The operator's target type, HUMAN/HEAD/CAT/DOG (owner 2026-10-20; docs/DESIGN_TARGET_TYPE_2026-10-20.md)."""
import dataclasses
import unittest

from perception.config import AnchorConfig, ModelConfig, VisionConfig
from perception.events import EventLog
from perception.model.swappable import SwappableAdapter
from perception.pipeline import PerceptionPipeline
from perception.protocol.native_wire import HEADER, encode_perception_frame
from perception.target_type import BY_CODE, DEFAULT_TARGET_TYPE, TARGET_TYPES, target_type
from perception.tests.support import (commissioned_config, commissioned_dedup, commissioned_thresholds,
                                      det, dset)
from perception.tests.test_native_selection import fixture as native_fixture
from perception.tests.test_pipeline import FakeClock


class TestTheTable(unittest.TestCase):
    def test_four_types_two_models(self):
        self.assertEqual(set(TARGET_TYPES), {"HUMAN", "HEAD", "CAT", "DOG"})
        self.assertEqual(TARGET_TYPES["HUMAN"].profile, TARGET_TYPES["HEAD"].profile)
        self.assertEqual(TARGET_TYPES["CAT"].profile, TARGET_TYPES["DOG"].profile)
        self.assertNotEqual(TARGET_TYPES["HEAD"].profile, TARGET_TYPES["CAT"].profile)
        self.assertEqual((TARGET_TYPES["HUMAN"].anchor, TARGET_TYPES["HEAD"].anchor), ("torso", "head"))
        self.assertEqual(sorted(BY_CODE), [1, 2, 3, 4], "codes fit header flag bits 2-4 and 0 means none")

    def test_the_default_is_todays_behaviour_and_nothing_is_guessed(self):
        self.assertEqual(DEFAULT_TARGET_TYPE, "HEAD")
        self.assertIs(target_type("cat"), TARGET_TYPES["CAT"])
        self.assertIsNone(target_type("bird"))
        self.assertIsNone(target_type(""))


class FakeNetwork:
    """A stand-in adapter that logs device use to a shared list, like the one Hailo context does."""

    def __init__(self, profile, log, *, fail_open=False):
        self.profile, self.log, self.fail_open = profile, log, fail_open
        self.anchor_cfg = AnchorConfig()
        self.generation = 0
        self.stream = self.camera = None

    def configure_stream(self, width, height, roi=None, *, declared=None):
        self.stream = (width, height, declared)

    def serve_camera(self, camera_id, width, height, *, declared):
        self.camera = camera_id
        self.configure_stream(width, height, declared=declared)

    def open(self):
        if self.fail_open:
            raise RuntimeError("HAILO_OUT_OF_PHYSICAL_DEVICES")
        self.log.append(("open", self.profile))

    def close(self):
        self.log.append(("close", self.profile))

    def note_model_generation(self, generation):
        self.generation = generation


class TestTheSwap(unittest.TestCase):
    def make(self, **fail):
        log = []
        built = []

        def build(profile):
            built.append(profile)
            return FakeNetwork(profile, log, fail_open=fail.get(profile, False))
        adapter = SwappableAdapter(FakeNetwork("pose", log), profile="pose", build=build)
        adapter.configure_stream(640, 360, declared=(1920, 1080))
        adapter.serve_camera("cam-wide", 640, 360, declared=(1920, 1080))
        adapter.open()
        return adapter, log, built

    def test_the_old_network_closes_before_the_new_one_opens_with_the_same_geometry(self):
        adapter, log, _ = self.make()
        self.assertTrue(adapter.swap("objects", anchor_target="torso"))
        self.assertEqual(log, [("open", "pose"), ("close", "pose"), ("open", "objects")],
                         "one Hailo-8 allows one context: close, then open")
        self.assertEqual(adapter.inner.stream, (640, 360, (1920, 1080)))
        self.assertEqual(adapter.inner.camera, "cam-wide")
        self.assertEqual(adapter.generation, 1, "a reload bumps the model generation (§33)")
        self.assertEqual(adapter.profile, "objects")

    def test_the_same_model_only_moves_the_aim(self):
        adapter, log, built = self.make()
        self.assertFalse(adapter.swap("pose", anchor_target="head"))
        self.assertEqual(built, [])
        self.assertEqual(log, [("open", "pose")], "HUMAN <-> HEAD touches nothing on the device")
        self.assertEqual(adapter.anchor_cfg.target, "head")

    def test_a_network_that_will_not_open_leaves_the_old_one_running(self):
        adapter, log, _ = self.make(objects=True)
        with self.assertRaises(RuntimeError):
            adapter.swap("objects", anchor_target="torso")
        self.assertEqual(adapter.profile, "pose")
        self.assertEqual(log[-1], ("open", "pose"), "the old network is reopened")


class FakeContext:
    def __init__(self):
        self.requested = ""

    def requested_target_type(self, session_uuid, now_ns):
        return self.requested

    def selection_flags(self, session_uuid, now_ns):
        from perception.selection.control_context import SelectionFlags
        return SelectionFlags()


class SceneAdapter:
    """Every frame shows one person and one cat; a swap is recorded, not performed."""

    def __init__(self, clock):
        self.clock, self.swaps, self.frame = clock, [], 0
        self.anchor_cfg = AnchorConfig()

    def swap(self, profile, *, anchor_target=None):
        self.swaps.append((profile, anchor_target))
        return len(self.swaps) == 1 or self.swaps[-1][0] != self.swaps[-2][0]

    def describe(self):
        return {}

    def infer(self, image, metadata=None, *, frame_sequence, sensor_timestamp_ns, publish_timestamp_ns,
              camera_id=""):
        self.frame += 1
        return dset([det(0, cx=.3, class_name="person"), det(1, cx=.7, class_id=15, class_name="cat")],
                    frame_index=self.frame, sensor_ns=int(sensor_timestamp_ns))


class TestThePipelineFollowsTheController(unittest.TestCase):
    def make(self, *, pinned=False):
        base = commissioned_config()
        models = {name: ModelConfig(profile_name=name, model_id=name, adapter="mock", task="object_detection",
                                    permitted_classes=("person",), thresholds=commissioned_thresholds())
                  for name in ("hailo_yolov8s_pose", "hailo_yolov8n")}
        config = dataclasses.replace(base, profile="hailo_yolov8s_pose", models=models,
                                     dedup=commissioned_dedup())
        self.clock = FakeClock()
        self.adapter = SceneAdapter(self.clock)
        pipeline = PerceptionPipeline(config, adapter=self.adapter, event_log=EventLog(capacity=256),
                                      clock=self.clock, session_uuid="session-test")
        pipeline.controller_context = self.context = FakeContext()
        pipeline.pin_profile = pinned
        return pipeline

    def run_frames(self, pipeline, n):
        outcome = None
        for _ in range(n):
            outcome = pipeline.process_frame(None, frame_sequence=self.adapter.frame + 1,
                                             sensor_timestamp_ns=self.clock())
        return outcome

    def classes(self, outcome):
        return {t.class_name for t in outcome.track_set.tracks}

    def test_cat_swaps_the_model_and_only_cats_are_tracked(self):
        pipeline = self.make()
        self.context.requested = "HEAD"
        first = self.run_frames(pipeline, 6)
        self.assertEqual(self.classes(first), {"person"})
        self.assertEqual(first.track_set.target_type_code, 2)
        self.context.requested = "CAT"
        cat = self.run_frames(pipeline, 6)
        self.assertEqual(self.adapter.swaps[-1], ("hailo_yolov8n", "torso"))
        self.assertEqual(self.classes(cat), {"cat"}, "the person is neither listed nor selectable")
        self.assertEqual(cat.track_set.target_type_code, 3)

    def test_no_request_changes_nothing(self):
        pipeline = self.make()
        outcome = self.run_frames(pipeline, 3)
        self.assertEqual(self.adapter.swaps, [])
        self.assertEqual(outcome.track_set.target_type_code, 0)

    def test_a_pinned_profile_acknowledges_without_swapping(self):
        pipeline = self.make(pinned=True)
        self.context.requested = "CAT"
        outcome = self.run_frames(pipeline, 3)
        self.assertEqual(self.adapter.swaps, [])
        self.assertEqual(self.classes(outcome), {"person"})
        self.assertEqual(outcome.track_set.target_type_code, 3)


def test_the_applied_type_rides_flag_bits_2_to_4():
    selector, frame, track = native_fixture()
    observation = selector.update(frame, frame.sensor_timestamp_ns)
    frame.narrow_view = True
    frame.target_type_code = 4
    flags = HEADER.unpack_from(encode_perception_frame(frame, observation))[11]
    assert (flags >> 2) & 7 == 4
    assert flags & 2 == 2, "the narrow-view bit is untouched"
