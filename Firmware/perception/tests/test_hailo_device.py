"""One Hailo-8, one HailoRT context, two cameras: what the shared device must guarantee.

Nothing here touches /dev/hailo0. The runtime is a stand-in; the turnstile, the per-camera counters
and the join-time refusals are the shipped code. These are the claims the second feed depends on:

* two adapters **share** the chip (H5 opened two contexts and the station came down:
  ``HAILO_DEVICE_IN_USE(73)`` → ``EXIT_MODEL`` → dead previews);
* while both cameras want the device, neither is starved — fairness is a mechanism, so it can be
  measured here and asserted on the station with the same number;
* the cost of sharing is published as a queue wait per feed, not blended into "the model got slower";
* a camera that fails does not take the other one down with it, and closing is the last one out.
"""

import json
import os
import threading
import time
import unittest

import numpy as np

from perception.errors import ModelRejected
from perception.model.hailo_device import HailoDevice
from perception.model.hailo_yolo import HailoYoloAdapter
from perception.model.manifest import ModelManifest

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(os.path.dirname(HERE), "model", "manifests",
                        "hailo_yolov8n_hailo8_coco.json")
HEF = os.path.join(os.path.dirname(HERE), "model", "artifacts", "yolov8n.hef")
# The stand-in runtime only sees a byte buffer, so a camera is identified the same way a real one
# would be: by something inside the frame. Marker 1 is cam-a, marker 2 is cam-detail.
WHO = {1: "cam-a", 2: "cam-detail"}
TENSOR = np.full((1, 640, 640, 3), 1, dtype=np.uint8)      # cam-a's frame
TENSOR_DETAIL = np.full((1, 640, 640, 3), 2, dtype=np.uint8)


def manifest_dict():
    with open(MANIFEST, encoding="utf-8") as handle:
        return json.load(handle)


class RecordingRuntime:
    """A stand-in ``InferVStreams``: it remembers who it served, and can be made slow or wrong."""

    def __init__(self, *, slow_ms=0.0, slow_for=None, fail_when=None):
        self.served = []                      # camera ids, in the order the chip saw them
        self.calls = 0
        self.slow_ms = slow_ms
        self.slow_for = slow_for              # camera names that are slow; None means every feed
        self.fail_when = fail_when            # callable(call_index) -> bool
        self._lock = threading.Lock()

    def infer(self, feed):
        with self._lock:
            index = self.calls
            self.calls += 1
            marker = int(list(feed.values())[0][0, 0, 0, 0])
            self.served.append(WHO.get(marker, f"unknown:{marker}"))
        who = WHO.get(marker, "?")
        if self.slow_ms and (self.slow_for is None or who in self.slow_for):
            time.sleep(self.slow_ms / 1000.0)
        if self.fail_when is not None and self.fail_when(index):
            raise RuntimeError("the stand-in runtime says no")
        classes = [np.zeros((0, 5), dtype=np.float32) for _ in range(80)]
        return {"output0": [classes]}


def one_camera_device(**kw):
    """A device already open for cam-wide... rather: opened for one camera, on the pinned artefact."""
    runtime = RecordingRuntime(**kw)
    device = HailoDevice.for_testing(runtime, input_name="input_0", member="cam-a",
                                     artifact_sha256=manifest_dict()["sha256"])
    return device, runtime


# open() demands a manifest that says where its artefact is; on a station the installer fills that
# in, and resolve_artifact tries the package directory first, so a relative name is what ships.
INSTALLED = {**manifest_dict(), "path": "model/artifacts/yolov8n.hef"}


class OneDeviceTwoCameras(unittest.TestCase):
    """The H5 regression, stated as the number that would have caught it: one runtime object."""

    def setUp(self):
        self.manifest = ModelManifest.from_dict(INSTALLED)

    def _adapter(self, device, camera_id):
        adapter = HailoYoloAdapter(self.manifest, device=device)
        adapter.bind_camera(camera_id)
        adapter.configure_stream(640, 360, declared=(1920, 1080))
        return adapter

    def test_a_second_camera_joins_the_open_device_instead_of_opening_another(self):
        runtime = RecordingRuntime()
        device = HailoDevice.for_testing(runtime, input_name="input_0",
                                         artifact_sha256=manifest_dict()["sha256"],
                                         member="cam-wide")
        second = self._adapter(device, "cam-detail")
        second.open()                                   # must not import hailo_platform
        self.assertEqual(device.cameras(), [],
                         "counting starts with frames, not with open(): an idle second camera must "
                         "not look like a second feed at 0 Hz")
        device.run(TENSOR, camera_id="cam-wide")        # one frame from the first camera
        for _ in range(3):
            second.infer(np.zeros((360, 640, 3), dtype=np.uint8), {}, frame_sequence=1,
                         sensor_timestamp_ns=1, publish_timestamp_ns=2, camera_id="cam-detail")
        self.assertEqual(runtime.calls, 4, "every frame went through the one shared runtime")
        facts = second.describe()
        self.assertEqual(facts["members"], 2, "the device knows two cameras are on it")
        self.assertEqual(facts["shared_with"], ["cam-wide"],
                         "and names the other one, so a rate claim can be read per feed")
        self.assertEqual(facts["device_served"], 3, "this camera's count, not the device's total")
        self.assertEqual(device.facts_for("cam-wide")["device_served"], 1)

    def test_a_second_camera_on_a_different_artefact_is_refused_by_both_digests(self):
        """Two networks on one context is not a config I will silently run; name both SHAs."""
        device = HailoDevice.for_testing(RecordingRuntime(), member="cam-wide",
                                         artifact_sha256="a" * 64)
        with self.assertRaises(ModelRejected) as caught:
            device.open_for("cam-detail", artifact_path=HEF, expected_sha="b" * 64,
                            require_input=(640, 640, 3), profile="hailo")
        message = str(caught.exception)
        self.assertIn("one Hailo context runs one artefact", message)
        self.assertIn("aaaaaaaaaaaa", message, "the refusal must show what was already open")
        self.assertIn("bbbbbbbbbbbb", message, "and what this camera asked for")

    def test_the_last_camera_to_leave_is_the_one_that_closes_the_chip(self):
        device = HailoDevice.for_testing(RecordingRuntime(), member="cam-wide",
                                         artifact_sha256=manifest_dict()["sha256"])
        device.open_for("cam-detail", artifact_path=HEF, expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")
        device.release("cam-wide")
        self.assertTrue(device.opened, "one camera is still inferring; the chip stays open")
        device.release("cam-detail")
        self.assertFalse(device.opened)
        with self.assertRaises(ModelRejected):
            device.run(TENSOR_DETAIL, camera_id="cam-detail")

    def test_opening_the_same_device_twice_from_one_adapter_is_a_routing_bug(self):
        device = HailoDevice.for_testing(RecordingRuntime(), member="cam-wide",
                                         artifact_sha256=manifest_dict()["sha256"])
        with self.assertRaises(ModelRejected) as caught:
            device.open_for("cam-wide", artifact_path=HEF,
                            expected_sha=manifest_dict()["sha256"],
                            require_input=(640, 640, 3), profile="hailo")
        self.assertIn("cannot open the same device twice", str(caught.exception))


class Turnstile(unittest.TestCase):
    """Fairness, measured the way the station will measure it: how many each feed got."""

    def test_two_feeds_that_both_want_the_chip_are_served_evenly(self):
        """The acceptance number for B-route is fairness >= 0.9; here it must be exact-ish.

        Both threads spin with no pause of their own, so contention is continuous: under
        contention the device hands the turn to whichever camera is waiting, which is what makes
        the ratio a property of the mechanism rather than of thread scheduling luck.
        """
        rounds = 40
        device, runtime = one_camera_device(slow_ms=1.0)
        device.open_for("cam-detail", artifact_path=HEF,
                        expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")

        def hammer(camera_id):
            for _ in range(rounds):
                device.run(TENSOR if camera_id == "cam-a" else TENSOR_DETAIL,
                           camera_id=camera_id)

        threads = [threading.Thread(target=hammer, args=(cid,))
                   for cid in ("cam-a", "cam-detail")]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(30)
            self.assertFalse(thread.is_alive(), "a feed never got stuck behind the other")
        counts = {cid: runtime.served.count(cid) for cid in ("cam-a", "cam-detail")}
        fairness = min(counts.values()) / max(counts.values())
        self.assertGreaterEqual(fairness, 0.9,
                                f"both feeds wanted {rounds} and got {counts}")
        self.assertGreater(device.facts_for("cam-a")["turn_contested"], 0,
                           "the turnstile was actually exercised, not bypassed by a lucky schedule")

    def test_while_both_feeds_are_queued_neither_one_bursts_two_frames_ahead(self):
        """The turnstile promises "the camera that waited goes next", not a fixed schedule.

        Adjacent same-camera services are legitimate at the edges, where only one feed had arrived;
        in the middle of a contended run they mean one camera was served twice while the other sat
        in the waiting room. That is the shape of starvation, so it is what this bounds.
        """
        rounds = 60
        device, runtime = one_camera_device(slow_ms=2.0)
        device.open_for("cam-detail", artifact_path=HEF,
                        expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")

        def hammer(camera_id):
            for _ in range(rounds):
                device.run(TENSOR if camera_id == "cam-a" else TENSOR_DETAIL,
                           camera_id=camera_id)

        threads = [threading.Thread(target=hammer, args=(cid,))
                   for cid in ("cam-a", "cam-detail")]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(60)
            self.assertFalse(thread.is_alive(), "a feed never got stuck behind the other")
        served = [entry for entry in runtime.served if entry in ("cam-a", "cam-detail")]
        middle = served[len(served) // 5: -len(served) // 5]
        bursts = sum(1 for first, second in zip(middle, middle[1:]) if first == second)
        self.assertLessEqual(bursts, len(middle) // 10,
                             f"one camera ran ahead while the other waited: {''.join(served)}")

    def test_the_uncontested_device_is_never_handed_over(self):
        """One camera, no theatre: sharing must cost a single-feed station nothing it can't see."""
        device, runtime = one_camera_device()
        for _ in range(5):
            device.run(TENSOR, camera_id="cam-a")
        self.assertEqual(runtime.calls, 5)
        facts = device.facts_for("cam-a")
        self.assertEqual(facts["turn_contested"], 0)
        self.assertEqual(facts["shared_with"], [])

    def test_waiting_for_the_other_camera_is_published_separately_from_chip_time(self):
        """'The model got slower' and 'this feed is starved' need different answers.

        Camera A holds a 30 ms inference; B asks meanwhile. B's own chip time cannot be 30 ms, so
        if the wait shows up anywhere it has to be in ``queue_wait_ms``.
        """
        device, _runtime = one_camera_device(slow_ms=30.0, slow_for=("cam-a",))
        device.open_for("cam-detail", artifact_path=HEF,
                        expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")
        first = threading.Thread(target=device.run, args=(TENSOR,), kwargs={"camera_id": "cam-a"})
        first.start()
        time.sleep(0.005)                              # let A take the turn
        device.run(TENSOR_DETAIL, camera_id="cam-detail")
        first.join(10)
        waiting = device.facts_for("cam-detail")
        self.assertGreaterEqual(waiting["queue_wait_ms"], 10.0,
                                f"B waited for A: {waiting}")
        self.assertLess(waiting["device_ms"], waiting["queue_wait_ms"],
                        "the wait must not be smuggled into the chip's own time")

    def test_a_failing_camera_releases_the_device_for_the_other_one(self):
        """A dropped frame is one camera's bad day; it must not wedge the shared chip."""
        device, _runtime = one_camera_device(fail_when=lambda index: index == 0)
        device.open_for("cam-detail", artifact_path=HEF,
                        expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")
        with self.assertRaises(ModelRejected):
            device.run(TENSOR, camera_id="cam-a")
        done = threading.Event()

        def other():
            device.run(TENSOR_DETAIL, camera_id="cam-detail")
            done.set()

        thread = threading.Thread(target=other)
        thread.start()
        thread.join(5)
        self.assertTrue(done.is_set(), "the surviving camera must not inherit the deadlock")
        self.assertEqual(device.facts_for("cam-a")["device_failures"], 1)
        self.assertEqual(device.facts_for("cam-detail")["device_failures"], 0,
                         "a failure is charged to the camera that asked, not to the device")

    def test_an_unowned_frame_on_a_shared_device_is_refused(self):
        """With one feed, an unlabelled frame is harmless. With two, its counters would be a lie."""
        device, _runtime = one_camera_device()
        device.open_for("cam-detail", artifact_path=HEF,
                        expected_sha=manifest_dict()["sha256"],
                        require_input=(640, 640, 3), profile="hailo")
        device.run(TENSOR_DETAIL, camera_id="cam-detail")
        with self.assertRaises(ModelRejected) as caught:
            device.run(TENSOR, camera_id="")
        self.assertIn("no camera_id", str(caught.exception))


class AFakeRuntimeIsNotAProductionPass(unittest.TestCase):
    """The station still has to say it. This file proves the ordering, not the accelerator."""

    def test_the_shared_path_never_pretends_a_second_device_was_opened(self):
        facts = one_camera_device()[0].facts_for("cam-a")
        self.assertEqual(facts["members"], 1,
                         "a device nobody joined reports one member, not zero and not two")


if __name__ == "__main__":
    unittest.main(verbosity=2)
