"""ADR-006's full-resolution snapshot: one request, the next frame's full picture, written off the
camera thread with the request id inside the JPEG; the frames themselves are left as they were."""

import json
import socket
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from perception.camera import CameraOwner
from perception.selection.service import SelectionService
from perception.snapshot import SnapshotSlot, read_comment


class FakeRequest:
    def __init__(self, names, stamp_ns, sizes):
        self.names, self.stamp_ns, self.sizes = names, stamp_ns, sizes

    def make_array(self, stream):
        self.names.append(stream)
        h, w = self.sizes[stream]
        return np.zeros((h, w, 3), dtype=np.uint8)

    def get_metadata(self):
        return {"SensorTimestamp": self.stamp_ns}

    def release(self):
        self.names.append("released")


class FakeCamera:
    def __init__(self):
        self.names = []
        self.stamp_ns = 1_000_000_000

    def capture_request(self):
        self.stamp_ns += 33_000_000
        return FakeRequest(self.names, self.stamp_ns, {"main": (1080, 1920), "lores": (360, 640)})


def wait_for(path, request_id, timeout_s=3.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if path.exists():
            meta = read_comment(path.read_bytes())
            if meta and meta.get("request") == request_id:
                return meta
        time.sleep(0.02)
    return None


class Slot(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "snapshot_wide.jpg"
        self.slot = SnapshotSlot("wide", str(self.path))
        self.slot.start()

    def tearDown(self):
        self.slot.stop()
        self.dir.cleanup()

    def test_a_frame_is_kept_only_when_asked_for(self):
        self.assertFalse(self.slot.wanted())
        self.assertFalse(self.slot.offer(np.zeros((4, 4, 3), np.uint8)))
        request = self.slot.request()
        self.assertTrue(self.slot.wanted())
        self.assertTrue(self.slot.offer(np.zeros((1080, 1920, 3), np.uint8), {"frame_sequence": 7}))
        self.assertFalse(self.slot.wanted(), "one request, one frame")
        meta = wait_for(self.path, request)
        self.assertIsNotNone(meta, self.slot.last_error)
        self.assertEqual((meta["width"], meta["height"], meta["camera"], meta["frame_sequence"]), (1920, 1080, "wide", 7))

    def test_the_pixels_arrive_as_rgb(self):
        # Picamera2's RGB888 is B,G,R in memory; the snapshot, like the preview, is a true-colour JPEG.
        bgr = np.zeros((64, 64, 3), np.uint8)
        bgr[..., 0] = 255   # blue in memory
        request = self.slot.request()
        self.slot.offer(bgr)
        self.assertIsNotNone(wait_for(self.path, request))
        from PIL import Image
        r, g, b = Image.open(self.path).convert("RGB").getpixel((32, 32))
        self.assertGreater(b, 200)
        self.assertLess(r, 50)


class WideCapture(unittest.TestCase):
    """The wide leg never copies its 1080p picture (2026-10-03); a snapshot copies it once."""

    def test_the_full_picture_is_copied_for_one_frame_and_not_handed_on(self):
        device = FakeCamera()
        slot = SnapshotSlot("wide", "unused.jpg")
        owner = CameraOwner(device, stream_size=(1920, 1080), inference_stream="lores",
                            inference_size=(640, 360), copy_main=False, snapshot=slot)
        frame = owner.next_frame()
        self.assertEqual(device.names.count("main"), 0)
        slot.request()
        frame = owner.next_frame()
        self.assertEqual(device.names.count("main"), 1, "the snapshot's one copy")
        self.assertIsNone(frame.image, "downstream reads what it read before")
        self.assertEqual(frame.inference_image.shape, (360, 640, 3))
        request_id, image, metadata = slot._frame
        self.assertEqual(image.shape, (1080, 1920, 3))
        self.assertEqual(metadata["frame_sequence"], frame.frame_sequence)
        owner.next_frame()
        self.assertEqual(device.names.count("main"), 1, "and only that frame")


class Service(unittest.TestCase):
    def test_a_snapshot_request_names_the_file_and_the_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            service = SelectionService(str(Path(tmp) / "sel.sock"))
            slot = SnapshotSlot("detail", str(Path(tmp) / "snapshot_detail.jpg"))
            service.snapshots = {"detail": slot}
            service.start()
            try:
                def ask(role):
                    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as c:
                        c.settimeout(1)
                        c.connect(service.path)
                        c.sendall(json.dumps({"type": "snapshot", "role": role}).encode())
                        return json.loads(c.recv(16384))
                reply = ask("detail")
                self.assertTrue(reply["accepted"])
                self.assertEqual(reply["path"], str(slot.path))
                self.assertTrue(slot.wanted())
                self.assertEqual(ask("wide")["reason"], "NO_SNAPSHOT")
            finally:
                service.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
