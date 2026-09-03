"""Controld UDS client (architecture §5.3, §6.1).

The webd side of the ``SOCK_SEQPACKET`` Unix socket that controld exposes.
A background thread keeps the connection alive (reconnecting on drop), reads
telemetry + command responses, and dispatches them:

  * ``telemetry`` -> the latest snapshot + an optional callback;
  * ``response``  -> a FIFO queue consumed by :meth:`ControldClient.send_command`.

SAFETY: this client NEVER opens can0 and NEVER decides safety. It only submits
commands; controld's validation gate decides whether they are legal (§42.2).
"""
from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from typing import Callable, Optional

log = logging.getLogger(__name__)

from .protocol import (
    ResponseMessage,
    Telemetry,
    command_to_json,
    parse_message,
)


class ControldClient:
    """Client for the controld web socket (telemetry + command relay)."""

    def __init__(
        self,
        socket_path: str,
        on_telemetry: Optional[Callable[[Telemetry], None]] = None,
        reconnect_interval: float = 0.5,
    ) -> None:
        self._path = socket_path
        self._on_telemetry = on_telemetry
        self._reconnect_interval = reconnect_interval
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._connected_evt = threading.Event()
        # Frames received from controld that could not be parsed. This counter exists because the
        # first version of this file answered an unparseable frame with a bare `return`, and the
        # result on a real station was a dashboard showing nothing while the daemon logged a
        # healthy publish loop and nothing anywhere said the words "malformed frame". Silence is
        # the one failure mode an operator cannot debug.
        self.malformed_frames = 0
        self._latest: Optional[Telemetry] = None
        self._latest_mono: Optional[float] = None
        self._latest_lock = threading.Lock()
        self._resp_q: "queue.Queue[ResponseMessage]" = queue.Queue()
        self._cmd_lock = threading.Lock()  # serialize command sends

    @property
    def on_telemetry(self) -> Optional[Callable[[Telemetry], None]]:
        return self._on_telemetry

    @on_telemetry.setter
    def on_telemetry(self, fn: Optional[Callable[[Telemetry], None]]) -> None:
        self._on_telemetry = fn

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, name="controld-client", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._close_socket()
        self._connected_evt.clear()

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # -- read access --------------------------------------------------------
    def latest_telemetry(self) -> Optional[Telemetry]:
        with self._latest_lock:
            return self._latest

    def telemetry_age_s(self) -> Optional[float]:
        """Seconds since a telemetry frame actually arrived; None if none ever has.

        `connected()` cannot answer the question §25 asks. It reports a socket, and a daemon that
        hangs while holding that socket open stays "connected" indefinitely while every number on
        the page freezes - which is worse than a disconnection, because a disconnection announces
        itself. So age is taken from the arrival stamp and measured on the monotonic clock, growing
        whether or not anything is reading it: the fact being recorded is when the data stopped, not
        when it was looked at.
        """
        with self._latest_lock:
            mono = self._latest_mono
        if mono is None:
            return None
        return max(0.0, time.monotonic() - mono)

    def connected(self) -> bool:
        return self._connected_evt.is_set()

    def wait_connected(self, timeout: float = 2.0) -> bool:
        return self._connected_evt.wait(timeout)

    # -- commands -----------------------------------------------------------
    def send_command(
        self, command: str, arg: str = "", timeout: float = 2.0
    ) -> ResponseMessage:
        """Submit a command and wait for controld's response.

        Sends are serialized (one in flight at a time) so the next response on
        the socket is this command's. Stale responses from a previous, timed-out
        send are drained first. If the socket is down or the response does not
        arrive within ``timeout``, a non-ok response is returned (the caller
        surfaces it to the operator).
        """
        if not self.connected():
            # The gate never saw it, so nothing was queued. "rejected" answers the one question this
            # response answers, and the reason says why.
            return ResponseMessage(command, ok=False, error="controld not connected",
                                   verdict="rejected")
        with self._cmd_lock:
            # Drain any stale responses from a previous (timed-out) send.
            while True:
                try:
                    self._resp_q.get_nowait()
                except queue.Empty:
                    break
            self._sock_send(command_to_json(command, arg))
            try:
                return self._resp_q.get(timeout=timeout)
            except queue.Empty:
                # Sent, but the gate never answered. The verdict is unknown, NOT "rejected": asserting a
                # refusal for a command that may have executed is the same lie in the other direction, and
                # this field exists to stop exactly that. It stays None.
                return ResponseMessage(
                    command, ok=False, error="no response within timeout"
                )

    # -- internals ----------------------------------------------------------
    def _sock_send(self, msg: str) -> bool:
        if self._sock is None:
            return False
        try:
            self._sock.sendall(msg.encode("utf-8"))
            return True
        except OSError:
            return False

    def _run(self) -> None:
        backoff = self._reconnect_interval
        while not self._stop_evt.is_set():
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                sock.settimeout(0.25)
                sock.connect(self._path)
            except OSError:
                # controld not up yet (or socket gone) — close + retry.
                sock.close()
                if not self._stop_evt.wait(backoff):
                    backoff = min(5.0, backoff * 1.5)
                continue
            self._sock = sock
            self._connected_evt.set()
            backoff = self._reconnect_interval
            try:
                self._read_loop(sock)
            finally:
                self._close_socket()
                self._connected_evt.clear()

    def _read_loop(self, sock: socket.socket) -> None:
        while not self._stop_evt.is_set():
            try:
                raw = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return  # peer closed
            if not raw:
                return
            self._dispatch(raw.decode("utf-8", "replace"))

    def _dispatch(self, raw: str) -> None:
        try:
            mtype, payload = parse_message(raw)
        except (ValueError, KeyError) as exc:
            self.malformed_frames += 1
            # Rate-limited, not suppressed: at 15 Hz this would drown the journal, and once is
            # enough for the first occurrence and then again every five seconds of continuing
            # failure, which is what "it is broken and stays broken" is allowed to cost.
            if self.malformed_frames == 1 or self.malformed_frames % 75 == 0:
                log.error(
                    "rejected a frame from controld (%d so far): %s%s — the page has had no "
                    "telemetry since the last good frame; frame began %r",
                    self.malformed_frames, exc,
                    (" at byte %d" % exc.pos if hasattr(exc, "pos") else ""), raw[:72])
            return
        if mtype == "telemetry":
            from .protocol import telemetry_from_json

            t = telemetry_from_json(payload)
            with self._latest_lock:
                self._latest = t
                self._latest_mono = time.monotonic()
            if self._on_telemetry is not None:
                try:
                    self._on_telemetry(t)
                except Exception:  # never let a callback kill the reader
                    pass
        elif mtype == "response":
            self._resp_q.put(payload)
