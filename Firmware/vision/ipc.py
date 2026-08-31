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
import threading
from typing import Optional

from .protocol import TargetMeasurement


class IpcPublisher:
    """Client side: connect to the controld socket and publish measurements."""

    def __init__(self, socket_path: str) -> None:
        self._path = socket_path
        self._sock: Optional[socket.socket] = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._sock.connect(self._path)

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
