"""IpcPublisher connect behaviour (visiond start-up under systemd).

controld BINDS the vision socket (§6.1), so "the socket is not there yet" is a
NORMAL state at service start — `Wants=` is a soft dependency and controld may be
restarting. What the publisher does about it decides whether the operator sees a
daemon waiting patiently or a unit crash-looping with connect tracebacks.
"""
from __future__ import annotations

import socket
import tempfile
import threading
import time

import pytest

from vision.ipc import IpcPublisher, IpcSubscriber
from vision.protocol import TargetMeasurement


def test_connects_immediately_when_the_daemon_is_there():
    with tempfile.TemporaryDirectory() as d:
        sock = d + "/vision.sock"
        sub = IpcSubscriber(sock)
        sub.start()
        pub = IpcPublisher(sock)
        pub.start(timeout_s=2.0)
        m = TargetMeasurement(frame_sequence=3, sensor_timestamp_ns=99, valid=False)
        pub.publish(m)
        time.sleep(0.1)
        assert sub.latest is not None and sub.latest.frame_sequence == 3
        pub.stop()
        sub.stop()


def test_waits_for_a_socket_that_appears_late():
    """The systemd case: controld binds a moment after visiond starts."""
    with tempfile.TemporaryDirectory() as d:
        sock = d + "/vision.sock"
        pub = IpcPublisher(sock)

        def late_bind():
            time.sleep(0.4)
            srv = IpcSubscriber(sock)
            srv.start()
            holder.append(srv)

        holder: list = []
        t = threading.Thread(target=late_bind, daemon=True)
        t.start()
        started = time.monotonic()
        pub.start(timeout_s=5.0)          # must NOT raise
        elapsed = time.monotonic() - started
        t.join(timeout=2.0)
        assert elapsed >= 0.3, "it should have actually waited"
        assert elapsed < 3.0
        pub.stop()
        for srv in holder:
            srv.stop()


def test_timeout_raises_oserror_not_a_hang():
    with tempfile.TemporaryDirectory() as d:
        pub = IpcPublisher(d + "/nobody-home.sock")
        started = time.monotonic()
        with pytest.raises(OSError):
            pub.start(timeout_s=0.5)
        assert time.monotonic() - started < 3.0
        assert pub._sock is None          # no half-open publisher left behind


def test_should_stop_aborts_the_wait_promptly():
    """Ctrl-C / SIGTERM during the wait must not have to wait out the timeout."""
    with tempfile.TemporaryDirectory() as d:
        pub = IpcPublisher(d + "/nobody-home.sock")
        flag = {"stop": False}

        def stopper():
            time.sleep(0.3)
            flag["stop"] = True

        threading.Thread(target=stopper, daemon=True).start()
        started = time.monotonic()
        with pytest.raises(OSError):
            pub.start(timeout_s=60.0, should_stop=lambda: flag["stop"])
        assert time.monotonic() - started < 3.0


def test_reconnect_after_the_daemon_dies_is_caller_policy():
    """Documented boundary: publish() on a dead connection raises, it does not
    silently queue. visiond exits (and systemd restarts it) rather than
    accumulating measurements no one is reading."""
    with tempfile.TemporaryDirectory() as d:
        sock = d + "/vision.sock"
        sub = IpcSubscriber(sock)
        sub.start()
        pub = IpcPublisher(sock)
        pub.start(timeout_s=2.0)
        pub.publish(TargetMeasurement(frame_sequence=1, sensor_timestamp_ns=1,
                                      valid=False))
        sub.stop()
        time.sleep(0.2)
        with pytest.raises(OSError):
            # A SEQPACKET peer that is gone shows up as EPIPE/ECONNRESET on a
            # send; give it a few attempts (the first may still be buffered).
            for i in range(20):
                pub.publish(TargetMeasurement(frame_sequence=2 + i,
                                              sensor_timestamp_ns=2,
                                              valid=False))
                time.sleep(0.02)
        pub.stop()


def test_sequence_packet_boundaries_survive():
    """One datagram == one measurement: two publishes must not coalesce (§6.2)."""
    with tempfile.TemporaryDirectory() as d:
        sock = d + "/vision.sock"
        sub = IpcSubscriber(sock)
        sub.start()
        pub = IpcPublisher(sock)
        pub.start(timeout_s=2.0)
        for seq in (11, 12, 13):
            pub.publish(TargetMeasurement(frame_sequence=seq,
                                          sensor_timestamp_ns=seq * 10,
                                          valid=True,
                                          anchor_u_px=float(seq),
                                          anchor_v_px=1.0))
            time.sleep(0.03)
        time.sleep(0.15)
        assert sub.latest is not None
        assert sub.latest.frame_sequence == 13
        assert sub.latest.anchor_u_px == pytest.approx(13.0)
        pub.stop()
        sub.stop()
