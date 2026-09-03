"""Fake controld — server side of the web socket (tests only).

A stand-in for controld's web server that speaks the same JSON-over-UDS
protocol, so the webd (client) and its tests can be exercised WITHOUT a real
controld, CAN, or a camera. It:

  * binds the ``SOCK_SEQPACKET`` Unix socket;
  * accepts multiple clients;
  * publishes a (configurable) telemetry snapshot at a set rate;
  * answers commands through a pluggable handler (default: accept all).

It mirrors ``format_telemetry`` / ``format_response`` in
``control/src/web/web_server.hpp`` so the wire bytes are identical to what a
real controld would emit.
"""
from __future__ import annotations

import json
import os
import socket
import threading
from typing import Callable, Optional

from .protocol import Telemetry, ResponseMessage, telemetry_to_json


class FakeControld:
    """Server-side stand-in for controld's web socket (no CAN, no motor)."""

    def __init__(self, socket_path: str, telemetry_hz: float = 15.0) -> None:
        self._path = socket_path
        self._hz = max(1.0, telemetry_hz)
        self._sock: Optional[socket.socket] = None
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._telemetry = Telemetry(
            track_state="ready_hold",
            safety_action="ALLOW",
            q_yaw_rad=0.0,
            q_pitch_rad=0.0,
            installation_calibrated=False,
            installation_source="identity",
            # §20's prediction block, at rest. The fake daemon carries it because the fake is what
            # the page tests run against: a field the fake does not speak is a field the page tests
            # cannot notice going missing, which is how "webd dropped it" turns into a day of
            # suspecting the daemon. Idle means not predicting - hence valid False, not zeros dressed
            # up as a real prediction at the centre of the frame.
            # §20/§11: a stand-in field of regard in joint degrees, so the page's FOR inset has
            # something to draw in tests and so a regression that drops the block is visible to the
            # suite rather than only to the station. Values are a plausible rectangle for this turret,
            # not a claim about it: the real one is measured and published by controld.
            # §20's camera block, idle. fps 0.0 because nothing has arrived yet, and the FOV values
            # are the station's commissioned ones so the inset draws a real rectangle in tests.
            camera={"fps": 0.0, "effective_hfov_deg": 69.3002, "effective_vfov_deg": 40.4171,
                    "measurement_age_ms": 3600000},
            # §20's imu block: the stub that says there is nothing. world_elevation_deg is None rather
            # than 0.0, because 0.0 is the claim "this turret is level" and nothing on this station is
            # entitled to make it.
            imu={"present": False, "gravity_valid": False, "world_elevation_valid": False,
                 "world_elevation_deg": None, "basis": "no inertial sensor on this station"},
            field_of_regard={
                "valid": True,
                "kind": 1,
                "coordinate_frame": "joint_deg",
                "safe_envelope_points": [[-30.0, -60.0], [30.0, -60.0], [30.0, 20.0], [-30.0, 20.0]],
            },
            prediction={
                "valid": False,
                "predicted_los_yaw_deg": 0.0,
                "predicted_los_pitch_deg": 0.0,
                "predicted_anchor_norm": [0.0, 0.0],
                "anchor_in_frame": False,
                "horizon_ms": 40,
            },
        )
        # command handler: (command, arg) -> ResponseMessage
        self._handler: Callable[[str, str], ResponseMessage] = (
            self._default_handler
        )

    # -- configuration ------------------------------------------------------
    def set_telemetry(self, **fields) -> None:
        """Update fields of the published telemetry snapshot."""
        for k, v in fields.items():
            if hasattr(self._telemetry, k):
                setattr(self._telemetry, k, v)

    def set_command_handler(
        self, fn: Callable[[str, str], ResponseMessage]
    ) -> None:
        self._handler = fn

    @staticmethod
    def _default_handler(command: str, arg: str) -> ResponseMessage:
        # Mirrors controld's two-answer contract: this is the gate's answer, not the station's.
        return ResponseMessage(command=command, ok=True, error="", verdict="submitted")

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        if os.path.exists(self._path):
            os.unlink(self._path)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._sock.bind(self._path)
        self._sock.listen(8)
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._accept_loop, name="fake-controld", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
        with self._clients_lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if os.path.exists(self._path):
            os.unlink(self._path)

    # -- internals ----------------------------------------------------------
    def _accept_loop(self) -> None:
        assert self._sock is not None
        self._sock.settimeout(0.25)
        while not self._stop_evt.is_set():
            try:
                cfd, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._clients_lock:
                self._clients.append(cfd)
            threading.Thread(
                target=self._client_loop, args=(cfd,), daemon=True
            ).start()

    def _client_loop(self, cfd: socket.socket) -> None:
        interval = 1.0 / self._hz
        next_send = 0.0
        import time

        while not self._stop_evt.is_set():
            # Read commands (non-blocking-ish via timeout).
            cfd.settimeout(0.05)
            try:
                raw = cfd.recv(65536)
            except socket.timeout:
                raw = b""
            except OSError:
                break
            if raw:
                self._handle_command(cfd, raw.decode("utf-8", "replace"))
            # Publish telemetry at the configured rate.
            now = time.monotonic()
            if now >= next_send:
                self._send(cfd, telemetry_to_json(self._telemetry))
                next_send = now + interval
            # Detect peer closed.
            if not self._alive(cfd):
                break
        self._drop_client(cfd)

    @staticmethod
    def _alive(cfd: socket.socket) -> bool:
        try:
            # A zero-byte peek is not possible on SEQPACKET; rely on recv errors.
            return True
        except OSError:
            return False

    def _drop_client(self, cfd: socket.socket) -> None:
        try:
            cfd.close()
        except OSError:
            pass
        with self._clients_lock:
            if cfd in self._clients:
                self._clients.remove(cfd)

    def _handle_command(self, cfd: socket.socket, raw: str) -> None:
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            return
        if obj.get("type") != "command":
            return
        command = obj.get("command", "")
        arg = obj.get("arg", "")
        resp = self._handler(command, arg)
        msg = json.dumps(
            {
                "type": "response",
                "command": resp.command,
                "ok": resp.ok,
                "verdict": resp.verdict,
                **({"error": resp.error} if resp.error else {}),
            },
            separators=(",", ":"),
        )
        self._send(cfd, msg)

    def _send(self, cfd: socket.socket, msg: str) -> None:
        try:
            cfd.sendall(msg.encode("utf-8"))
        except OSError:
            pass
