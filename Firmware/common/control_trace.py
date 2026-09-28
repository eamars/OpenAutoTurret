"""The read side of controld's per-cycle control trace.

The ring and the ``read_control_trace`` command have existed for a long time, yet
nothing in the stack ever asked for them: the 2026-09-28 no-progress trip left no
per-cycle evidence behind, not because the rows were never written but because
nobody read them inside the ~20 s before the ring wrapped. Two callers want that
now -- ``tools/pull_control_trace.py`` on the station, and webd's
``/api/control_trace`` so an operator's browser can ask during a live fault --
so the reader lives here, once, rather than being pasted twice and drifting.

Two facts drive the shape of this code, both measured rather than assumed:

* controld serves a ``SOCK_SEQPACKET`` socket, and a datagram larger than the
  receive buffer is **truncated, not split**. The ring is 4096 rows at roughly
  250 B each, so a full reply is on the order of a megabyte: a small buffer does
  not return "part of the trace", it returns a prefix that fails to parse and
  reads like a broken protocol. ``MAX_FRAME`` therefore has to be comfortably
  above a full ring, and a reply that still fills it is reported as truncation
  instead of being handed back as if it were complete.
* the reply is ``{"type":"control_trace", ...}`` -- a third frame type beside
  telemetry and ``response`` -- and a telemetry frame is published at ~15 Hz on
  the same socket, so a reader must skip frames until it sees the one it asked
  for. Grabbing "the next frame" is a race that usually loses.

This module is a reader only: it sends one read command, never touches can0, and
cannot ask for anything safety-relevant.
"""
from __future__ import annotations

import json
import socket
from typing import Any, Dict

# One row is ~250 B of JSON and the ring is 4096 deep, so a full reply is on the
# order of a megabyte. Asking for less silently truncates a SEQPACKET, which is
# worse than a timeout: a truncated frame parses as a malformed one.
MAX_FRAME = 32 * 1024 * 1024

# How long to wait for the window. It matches the default the station tool has
# always shipped with: a SEQPACKET reply from a local daemon is sub-millisecond,
# so a longer wait only ever buys time on a machine that is already suffering --
# and a browser 503 that arrives after two seconds of a loaded station is a lie
# about the station, not a measurement of it.
DEFAULT_TIMEOUT_S = 5.0


class TraceUnavailable(RuntimeError):
    """The trace could not be read, with the reason in the message.

    Every failure path names the socket it tried, because the two common ones --
    controld not running, and webd pointing at another run directory -- look
    identical from the browser unless the message says which path was opened.
    """


def request_trace(socket_path: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> Dict[str, Any]:
    """Ask controld for the current control-trace window and return the frame.

    Raises :class:`TraceUnavailable` with a reason rather than returning a
    half-frame or an empty dict: an empty trace and an unreadable one have to be
    distinguishable, or "no anomalies in the ring" becomes a lie we tell ourselves
    during an incident.
    """
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    except OSError as exc:  # pragma: no cover - only without AF_UNIX
        raise TraceUnavailable(f"cannot build a SEQPACKET socket: {exc}") from exc
    sock.settimeout(timeout_s)
    try:
        sock.connect(socket_path)
    except OSError as exc:
        sock.close()
        raise TraceUnavailable(
            f"cannot reach controld at {socket_path}: {exc.strerror or exc}"
            " -- the trace lives inside controld, so a stopped stack has none to give"
        ) from exc
    try:
        sock.send(json.dumps({"type": "command", "command": "read_control_trace"}).encode())
        while True:
            try:
                # recvmsg, not recv: SEQPACKET reports truncation only through the
                # message flags, and a silently truncated megabyte looks exactly
                # like a daemon speaking broken JSON.
                data, _anc, flags, _addr = sock.recvmsg(MAX_FRAME)
            except socket.timeout as exc:
                raise TraceUnavailable(
                    f"no trace reply from {socket_path} within {timeout_s}s"
                    " -- controld answered the socket but not the command"
                ) from exc
            except OSError as exc:
                raise TraceUnavailable(f"read from {socket_path} failed: {exc.strerror or exc}") from exc
            if not data:
                raise TraceUnavailable(f"{socket_path} closed before answering read_control_trace")
            if flags & getattr(socket, "MSG_TRUNC", 0):
                raise TraceUnavailable(
                    f"trace reply from {socket_path} exceeds MAX_FRAME={MAX_FRAME} bytes;"
                    " returning it truncated would hide rows from a window that already"
                    " wrapped, so it is refused instead"
                )
            try:
                frame = json.loads(data.decode("utf-8", "replace"))
            except ValueError as exc:
                raise TraceUnavailable(
                    f"trace reply from {socket_path} is not JSON ({exc});"
                    f" it began {data[:72]!r}"
                ) from exc
            mtype = frame.get("type")
            if mtype == "control_trace":
                return frame
            if mtype == "response":
                # controld answered with a refusal instead of a window: the command
                # is unknown to this build, or the provider is not wired. Say which.
                raise TraceUnavailable(
                    f"controld refused read_control_trace: "
                    f"{frame.get('error') or frame.get('verdict') or 'no reason given'}"
                )
            # Anything else on this socket is telemetry; the reply we want is the
            # one that says so.
    finally:
        sock.close()
