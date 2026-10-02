"""Only the main display's camera is inferred (owner, 2026-10-02), and a swap keeps the subject.

The properties an operator would notice:
- the detail camera's boxes land in the wide frame where the subject is (one frame for everything);
- pressing swap while tracking keeps the same identity when the subject is at the aim point, even
  though the narrow optic sees only head and shoulders of the person the wide one saw whole;
- one adapter, never two at once: a frame from the camera that is not on the main display is not
  inferred, and the first frame of a swap re-binds the adapter before it is inferred;
- the swap is refused, with a reason, for a camera this boot does not have.
"""
from __future__ import annotations

import threading
import unittest
from dataclasses import replace

from perception.camera_id import CameraId
from perception.detail_stream import DetailFrame, SecondaryCameraStream
from perception.detection.types import Keypoint, PointNorm
from perception.detection.view import narrow_to_wide, to_wide_frame
from perception.inference_switch import InferenceSwitch, View
from perception.main_camera import MainCamera
from perception.selection.service import SelectionService
from perception.tests.support import advance, at, commissioned_config, det, dset
from perception.tracking.track_manager import TrackManager


class TestDetailBoxesInTheWideFrame(unittest.TestCase):
    def test_the_detail_picture_is_a_centred_window_one_scale_smaller(self):
        self.assertAlmostEqual(narrow_to_wide(0.5, 5.9), 0.5)
        self.assertAlmostEqual(narrow_to_wide(0.0, 5.9), 0.5 - 0.5 / 5.9)
        self.assertAlmostEqual(narrow_to_wide(1.0, 5.9), 0.5 + 0.5 / 5.9)

    def test_boxes_anchors_and_keypoints_move_together_and_the_set_declares_the_wide_picture(self):
        d = det(1, cx=0.6, cy=0.4, width=0.2, height=0.5, anchor=PointNorm(0.6, 0.2),
                keypoints=(Keypoint(0.55, 0.18, 0.9),))
        source = replace(dset([d]), stream_width=1280, stream_height=720)
        mapped = to_wide_frame(source, 5.9, (1920, 1080))
        out = mapped.detections[0]
        self.assertEqual((mapped.stream_width, mapped.stream_height), (1920, 1080))
        self.assertAlmostEqual(out.bbox.x_min, narrow_to_wide(0.5, 5.9))
        self.assertAlmostEqual(out.bbox.y_max, narrow_to_wide(0.65, 5.9))
        self.assertAlmostEqual(out.measured_anchor.x, narrow_to_wide(0.6, 5.9))
        self.assertAlmostEqual(out.keypoints[0].x, narrow_to_wide(0.55, 5.9))
        self.assertEqual(out.keypoints[0].score, 0.9)
        mapped.validate()

    def test_a_view_that_is_not_narrower_is_refused(self):
        with self.assertRaises(ValueError):
            to_wide_frame(dset([det(1)]), 1.0, (1920, 1080))


class TestSwapKeepsTheSubject(unittest.TestCase):
    # The wide camera sees a whole person; the head anchor is near the top of the box.
    WHOLE = dict(cx=0.5, cy=0.5, width=0.1, height=0.3, anchor=PointNorm(0.5, 0.38))
    # The detail camera, mapped into the wide frame, sees only head and shoulders of that person,
    # about a degree off (the optics' residual misalignment): almost no overlap with the old box.
    HEAD = dict(cx=0.505, cy=0.39, width=0.035, height=0.045, anchor=PointNorm(0.505, 0.385))

    def selected_manager(self):
        manager = TrackManager(commissioned_config())
        sets = advance(manager, [[det(1, **self.WHOLE)]] * 3)
        uuid = sets[-1].tracks[0].track_uuid
        manager.set_selected_uuid(uuid)
        return manager, uuid

    def test_without_the_handoff_the_narrow_view_would_be_a_stranger(self):
        manager, uuid = self.selected_manager()
        result = manager.update(dset([det(1, **self.HEAD)], frame_index=3), at(3))
        matched = [t for t in result.tracks if t.track_uuid == uuid and t.observations == 4]
        self.assertEqual(matched, [], "the control case must fail to associate, or the test proves nothing")

    def test_the_selected_subject_keeps_its_identity_across_a_swap(self):
        manager, uuid = self.selected_manager()
        manager.note_source_change(gate_norm=0.05, window_ms=1000.0)
        result = manager.update(dset([det(1, **self.HEAD)], frame_index=3), at(3))
        track = next(t for t in result.tracks if t.track_uuid == uuid)
        self.assertEqual(track.observations, 4)
        self.assertAlmostEqual(track.anchor.x, 0.505)
        # The new optic's box is taken as it is: blending it with the old one invents a shape.
        self.assertAlmostEqual(track.bbox.y_min, 0.39 - 0.045 / 2)
        self.assertEqual(len(result.tracks), 1, "no second identity for the same person")
        self.assertEqual(manager.handoffs["kept"], 1)
        # And the swap is not motion.
        self.assertEqual((track.velocity_x, track.velocity_y), (0.0, 0.0))

    def test_someone_else_away_from_the_aim_point_is_not_taken_for_the_subject(self):
        manager, uuid = self.selected_manager()
        manager.note_source_change(gate_norm=0.05, window_ms=1000.0)
        stranger = dict(self.HEAD, cx=0.58, anchor=PointNorm(0.58, 0.385))
        result = manager.update(dset([det(1, **stranger)], frame_index=3), at(3))
        old = next(t for t in result.tracks if t.track_uuid == uuid)
        self.assertEqual(old.observations, 3)
        self.assertEqual(manager.handoffs["kept"], 0)

    def test_past_the_window_the_ordinary_loss_rules_apply(self):
        manager, uuid = self.selected_manager()
        manager.note_source_change(gate_norm=0.05, window_ms=100.0)
        manager.update(dset([], frame_index=3), at(3))
        manager.update(dset([], frame_index=6), at(6))      # 180 ms later: window closed
        result = manager.update(dset([det(1, **self.HEAD)], frame_index=7), at(7))
        self.assertEqual(manager.handoffs["expired"], 1)
        self.assertFalse(any(t.track_uuid == uuid and t.observations == 4 for t in result.tracks))


class FakeAdapter:
    def __init__(self):
        self.served = []

    def serve_camera(self, camera_id, width, height, *, declared):
        self.served.append((camera_id, (width, height), tuple(declared)))


class FakeManager:
    def __init__(self):
        self.changes = []

    def note_source_change(self, **kw):
        self.changes.append(kw)


class FakePipeline:
    def __init__(self):
        self.manager = FakeManager()
        self.view_scale, self.view_canvas, self.preview = 1.0, None, "wide-tap"


class TestOneAdapterFollowsTheMainDisplay(unittest.TestCase):
    def setUp(self):
        self.main = MainCamera(available=("wide", "detail"))
        self.pipeline, self.adapter = FakePipeline(), FakeAdapter()
        self.switch = InferenceSwitch(self.main, self.pipeline, self.adapter, {
            "wide": View("cam-w", (640, 360), (1920, 1080)),
            "detail": View("cam-d", (640, 360), (1280, 720), scale=5.9, canvas=(1920, 1080))},
            wide_preview="wide-tap")

    def test_only_the_main_display_is_inferred(self):
        self.assertTrue(self.switch.acquire("wide"))
        self.assertFalse(self.switch.acquire("detail"))

    def test_a_swap_rebinds_maps_and_reroutes_the_preview_before_the_first_detail_frame(self):
        self.switch.acquire("wide")
        self.main.request("detail")
        self.assertFalse(self.switch.acquire("wide"))
        self.assertTrue(self.switch.acquire("detail"))
        self.assertEqual(self.adapter.served[-1], ("cam-d", (640, 360), (1280, 720)))
        self.assertEqual((self.pipeline.view_scale, self.pipeline.view_canvas), (5.9, (1920, 1080)))
        self.assertIsNone(self.pipeline.preview, "detail pixels must never reach the wide preview")
        self.assertEqual(len(self.pipeline.manager.changes), 1)
        self.main.request("wide")
        self.assertTrue(self.switch.acquire("wide"))
        self.assertEqual(self.adapter.served[-1][0], "cam-w")
        self.assertEqual((self.pipeline.view_scale, self.pipeline.preview), (1.0, "wide-tap"))
        self.assertEqual(self.switch.switches, 2)

    def test_a_frame_older_than_the_last_inferred_one_is_not_offered(self):
        self.assertTrue(self.switch.fresh(100))
        self.assertFalse(self.switch.fresh(90))
        self.assertTrue(self.switch.fresh(101))


class TestMainCameraRequests(unittest.TestCase):
    def test_every_boot_starts_on_wide(self):
        self.assertEqual(MainCamera(available=("wide", "detail")).role, "wide")

    def test_a_camera_this_boot_does_not_have_is_refused_by_name(self):
        main = MainCamera()
        ok, why = main.request("detail")
        self.assertFalse(ok)
        self.assertIn("not running", why)
        self.assertFalse(main.request("zoom")[0])
        self.assertEqual(main.role, "wide")

    def test_losing_the_detail_camera_returns_the_main_display_to_wide(self):
        main = MainCamera(available=("wide", "detail"))
        main.request("detail")
        main.make_unavailable("detail")
        self.assertEqual(main.role, "wide")

    def test_the_socket_answers_with_the_resulting_state(self):
        service = SelectionService("/unused")
        service.main_camera = MainCamera(available=("wide", "detail"))
        reply = service._main_camera({"type": "set_main_camera", "role": "detail"})
        self.assertTrue(reply["accepted"])
        self.assertEqual(reply["main_camera"]["role"], "detail")
        refused = SelectionService("/unused")._main_camera({"role": "detail"})
        self.assertFalse(refused["accepted"])


class TestDetailBufferIsLatestOnly(unittest.TestCase):
    def test_taking_the_newest_frame_discards_the_older_ones(self):
        ident = CameraId(id="cam-d", source="fwnode")
        stream = SecondaryCameraStream(role="detail", ident=ident, poll=lambda: None, queue_depth=2)
        for seq in (1, 2):
            stream._push(DetailFrame(camera_id="cam-d", frame_sequence=seq, sensor_timestamp_ns=seq,
                                     image=None, metadata={}))
        self.assertEqual(stream.latest().frame_sequence, 2)
        self.assertIsNone(stream.latest(), "an older frame must never come out after a newer one")


if __name__ == "__main__":
    unittest.main()
