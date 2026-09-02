"""Latest-value IPC over a Unix-domain socket (architecture §6.1).

``visiond -> controld`` uses a ``SOCK_SEQPACKET`` Unix socket so message
boundaries are preserved. The vision daemon is the client (publisher); controld
binds the socket and keeps the latest measurement. Video frames never traverse
this path (§6.1). The test subscriber here is a stand-in for controld that only
RECEIVES — it never opens CAN or drives the motor.
"""
from __future__ import annotations

import os
import socket
import sys
import threading
import time
from typing import Callable, Optional

from .protocol import TargetMeasurement


class IpcPublisher:
    """Client side: connect to the controld socket and publish measurements."""

    def __init__(self, socket_path: str) -> None:
        self._path = socket_path
        self._sock: Optional[socket.socket] = None

    def start(self, timeout_s: float = 0.0, retry_log_s: float = 5.0,
              should_stop: Optional[Callable[[], bool]] = None) -> None:
        """Connect to controld's socket, waiting for it to appear.

        controld BINDS this socket (§6.1), so a systemd start can legitimately
        put visiond ahead of the daemon (`Wants=` is soft, and controld may be
        restarting). Failing hard here makes the unit crash-loop every
        `RestartSec` with a connect traceback, which reads as "vision keeps
        restarting" rather than the truth: there is no daemon to talk to yet. So
        wait, and say so legibly once in a while.

        ``timeout_s <= 0`` (the service default) waits until connected or
        ``should_stop()``; a positive value re-raises the last OSError after the
        deadline. Either way the caller gets ONE clear message, not a stack.
        """
        deadline = None if timeout_s <= 0 else time.monotonic() + timeout_s
        last_log = 0.0
        attempt = 0
        while True:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                sock.connect(self._path)
                self._sock = sock
                if attempt:
                    print(f"connected to controld at {self._path} "
                          f"after {attempt} attempt(s)", flush=True)
                return
            except OSError as e:
                sock.close()
                attempt += 1
                now = time.monotonic()
                if attempt == 1 or now - last_log >= retry_log_s:
                    last_log = now
                    print(f"waiting for controld at {self._path}: {e}",
                          file=sys.stderr, flush=True)
                if should_stop is not None and should_stop():
                    raise
                if deadline is not None and now >= deadline:
                    raise
                time.sleep(0.2)

    def publish(self, m: TargetMeasurement) -> None:
        if self._sock is None:
            raise RuntimeError("IpcPublisher.start() was not called")
        self._sock.send(m.encode())

    def stop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class IpcSubscriber:
    """Server side (test stand-in for controld): bind + accept + keep latest.

    This is the ONLY role controld plays in the vision test path, and it
    explicitly does NOT touch CAN or the motor driver.
    """

    def __init__(self, socket_path: str) -> None:
        self._path = socket_path
        self._sock: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[TargetMeasurement] = None
        self._lock = threading.Lock()
        self._running = False

    @property
    def latest(self) -> Optional[TargetMeasurement]:
        with self._lock:
            return self._latest

    def start(self) -> None:
        # Clean up any stale socket file from a previous run.
        try:
            os.unlink(self._path)
        except FileNotFoundError:
            pass
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._sock.bind(self._path)
        self._sock.listen(1)
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        assert self._sock is not None
        self._conn, _ = self._sock.accept()
        while self._running:
            try:
                data = self._conn.recv(64)
            except OSError:
                break
            if not data:
                break
            with self._lock:
                self._latest = TargetMeasurement.decode(data)

    def stop(self) -> None:
        self._running = False
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            os.unlink(self._path)
        except FileNotFoundError:
            pass
