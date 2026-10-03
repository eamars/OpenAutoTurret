"""The controller context reads controld's own telemetry socket, not the web UI (2026-10-03)."""
import json
import os
import socket
import tempfile
import threading
import time
import uuid

from perception.selection.control_context import ControllerContext


def test_the_mode_comes_from_controlds_socket():
    session = uuid.uuid4()
    hi, lo = session.int >> 64, session.int & ((1 << 64) - 1)
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, 'control-web.sock')
        server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        server.bind(path)
        server.listen(1)
        stop = threading.Event()

        def serve():
            conn, _ = server.accept()
            with conn:
                while not stop.is_set():
                    frame = {'type': 'telemetry', 'ts_ns': time.monotonic_ns(),
                             'perception_session_uuid': f'{hi}:{lo}', 'perception_native': True,
                             'operating_mode': 'AUTO_ROAM',
                             'blackbox': {'operating_mode': 'MANUAL'}}
                    conn.send(json.dumps(frame).encode())
                    time.sleep(0.02)

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        context = ControllerContext('unix:' + path)
        context.start()
        try:
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                if context.allows_auto_select(session.hex, time.monotonic_ns()):
                    break
                time.sleep(0.02)
            assert context.operating_mode(session.hex, time.monotonic_ns()) == 'AUTO_ROAM'
        finally:
            stop.set()
            context.close()
            server.close()


def test_no_controller_means_no_mode():
    context = ControllerContext('unix:/tmp/definitely-not-a-controld.sock')
    context.start()
    try:
        time.sleep(0.1)
        assert context.operating_mode(uuid.uuid4().hex, time.monotonic_ns()) == ''
    finally:
        context.close()
