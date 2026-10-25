"""ADR-006: GET /api/video/snapshot asks visiond for one full-resolution frame and returns that frame,
not an earlier snapshot still on disk; a refusal or a missing frame is said, not served."""
from __future__ import annotations

import json
import os
import socket
import tempfile
import threading
import time
import unittest

from fastapi.testclient import TestClient

from ..app import create_app
from ..config import WebConfig
from ..controld_client import ControldClient
from ..fake_controld import FakeControld
from ..selection_client import snapshot_comment


def jpeg_with(comment: dict) -> bytes:
    payload = b"OTA_FRAME\x00" + json.dumps(comment).encode()
    return b"\xff\xd8\xff\xfe" + (len(payload) + 2).to_bytes(2, "big") + payload + b"\xff\xd9"


class FakeVisiond:
    """The selection socket's snapshot answer, and the file the next frame becomes."""

    def __init__(self, path: str, image_dir: str, *, accept=True, deliver=True):
        self.path, self.image_dir, self.accept, self.deliver = path, image_dir, accept, deliver
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.listener.bind(path)
        self.listener.listen(4)
        self.listener.settimeout(0.1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        while not self.stop.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            with client:
                data = json.loads(client.recv(4096))
                role = data["role"]
                target = os.path.join(self.image_dir, f"snapshot_{role}.jpg")
                if not self.accept:
                    client.sendall(json.dumps({"accepted": False, "reason": "NO_SNAPSHOT"}).encode())
                    continue
                client.sendall(json.dumps({"accepted": True, "role": role, "request": "r2",
                                           "path": target}).encode())
                if self.deliver:
                    def later():
                        time.sleep(0.15)
                        with open(target, "wb") as f:
                            f.write(jpeg_with({"request": "r2", "camera": role, "width": 1920, "height": 1080}))
                    threading.Thread(target=later, daemon=True).start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.listener.close()


@unittest.skipUnless(hasattr(socket, "AF_UNIX") and hasattr(socket, "SOCK_SEQPACKET"), "needs Linux local sockets")
class SnapshotRoute(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_snap_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.selection = os.path.join(self._tmp.name, "selection.sock")
        self._env = os.environ.get("OTA_SELECTION_SOCKET")
        os.environ["OTA_SELECTION_SOCKET"] = self.selection
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.tc = TestClient(create_app(self.client, WebConfig(host="127.0.0.1", port=0, socket_path=self.sock)))
        self.tc.__enter__()
        self.visiond = None

    def tearDown(self):
        try:
            self.tc.__exit__(None, None, None)
        finally:
            if self.visiond:
                self.visiond.close()
            self.client.stop()
            self.fake.stop()
            if self._env is None:
                os.environ.pop("OTA_SELECTION_SOCKET", None)
            else:
                os.environ["OTA_SELECTION_SOCKET"] = self._env
            self._tmp.cleanup()

    def test_the_new_frame_is_served_not_the_old_file(self):
        self.visiond = FakeVisiond(self.selection, self._tmp.name)
        old = os.path.join(self._tmp.name, "snapshot_wide.jpg")
        with open(old, "wb") as f:
            f.write(jpeg_with({"request": "r1", "camera": "wide"}))
        r = self.tc.get("/api/video/snapshot", params={"camera": "wide"})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(r.headers["content-type"], "image/jpeg")
        self.assertEqual(snapshot_comment(r.content)["request"], "r2")
        self.assertEqual(json.loads(r.headers["x-ota-snapshot"])["width"], 1920)

    def test_a_refusal_is_a_conflict(self):
        self.visiond = FakeVisiond(self.selection, self._tmp.name, accept=False)
        r = self.tc.get("/api/video/snapshot", params={"camera": "detail"})
        self.assertEqual(r.status_code, 409)
        self.assertEqual(r.json()["reason"], "NO_SNAPSHOT")

    def test_no_frame_in_time_is_a_timeout(self):
        self.visiond = FakeVisiond(self.selection, self._tmp.name, deliver=False)
        r = self.tc.get("/api/video/snapshot", params={"camera": "wide", "timeout_s": 0.3})
        self.assertEqual(r.status_code, 504)

    def test_an_unknown_camera_is_refused(self):
        r = self.tc.get("/api/video/snapshot", params={"camera": "thermal"})
        self.assertEqual(r.status_code, 400)


if __name__ == "__main__":
    unittest.main(verbosity=2)
