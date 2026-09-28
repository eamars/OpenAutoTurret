"""The shared control-trace reader, against a scripted SEQPACKET server.

These are the four behaviours the reader exists for, and each one was written
because the naive version gets it wrong:

* the reply is not the next frame -- controld publishes telemetry at ~15 Hz on
  the same socket, so a reader that takes "whatever arrives" returns telemetry;
* a reply bigger than the receive buffer is **truncated**, and a truncated frame
  parses as a malformed one, which is how a megabyte of evidence turns into
  "the daemon is speaking broken JSON";
* a refusal must not look like an empty window;
* and every failure has to name the socket it tried, because "controld is not
  running" and "webd was pointed at another run directory" are different calls.
"""
from __future__ import annotations

import json
import os
import socket
import tempfile
import threading
import unittest

from common import control_trace
from common.control_trace import TraceUnavailable, request_trace


class SeqpacketServer(threading.Thread):
    """One accepted connection, one scripted answer, then it stops."""

    def __init__(self, path: str, answers: list[str], prefix_telemetry: bool = False) -> None:
        super().__init__(daemon=True)
        self._path = path
        self._answers = answers
        self._prefix = prefix_telemetry
        self._srv = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._srv.bind(path)
        self._srv.listen(1)
        self._srv.settimeout(5.0)
        self.error: str = ""

    def run(self) -> None:
        try:
            cfd, _ = self._srv.accept()
        except OSError as exc:
            self.error = f"no connection arrived: {exc}"
            return
        with cfd:
            try:
                cfd.recv(4096)  # the read_control_trace command
            except OSError:
                return
            if self._prefix:
                cfd.send(json.dumps({"type": "telemetry", "track_state": "hold"}).encode())
            for answer in self._answers:
                cfd.send(answer.encode())

    def join_quietly(self) -> None:
        self._srv.close()
        self.join(timeout=2.0)


class TraceReaderTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_trace_")
        self.path = os.path.join(self._tmp.name, "controld.sock")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _run(self, answers: list[str], prefix_telemetry: bool = False, **kw):
        server = SeqpacketServer(self.path, answers, prefix_telemetry)
        server.start()
        try:
            return request_trace(self.path, **kw)
        finally:
            server.join_quietly()

    def test_the_trace_frame_survives_telemetry_on_the_same_socket(self):
        frame = {"type": "control_trace", "axes": ["pitch", "yaw"], "frozen": False,
                 "rows": [{"t": "1000000000", "phase": "hold"}]}
        got = self._run([json.dumps(frame)], prefix_telemetry=True)
        self.assertEqual(got["rows"][0]["phase"], "hold")
        # The ns stamps ride through as strings: 2^64-1 does not fit a JS number.
        self.assertEqual(got["rows"][0]["t"], "1000000000")

    def test_a_truncated_reply_is_refused_not_returned(self):
        """The one claim here that is easy to fake: truncation must be detected."""
        big = {"type": "control_trace", "rows": [{"t": str(i), "phase": "hold"} for i in range(64)]}
        original = control_trace.MAX_FRAME
        control_trace.MAX_FRAME = 64  # force the buffer below the datagram
        try:
            with self.assertRaises(TraceUnavailable) as ctx:
                self._run([json.dumps(big)])
            self.assertIn("MAX_FRAME=64", str(ctx.exception))
        finally:
            control_trace.MAX_FRAME = original

    def test_a_refusal_is_not_reported_as_an_empty_window(self):
        refusal = {"type": "response", "command": "read_control_trace", "ok": False,
                   "error": "no trace provider wired"}
        with self.assertRaises(TraceUnavailable) as ctx:
            self._run([json.dumps(refusal)])
        self.assertIn("no trace provider wired", str(ctx.exception))

    def test_a_socket_with_nothing_behind_it_names_itself(self):
        with self.assertRaises(TraceUnavailable) as ctx:
            request_trace(os.path.join(self._tmp.name, "absent.sock"), timeout_s=0.5)
        self.assertIn("absent.sock", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
