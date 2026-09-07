"""Signal the real webd server with synthetic MJPEG and WebSocket clients.

No camera, station video, browser, or motor connection is used.
"""
import argparse
import asyncio
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request


def server(port):
    from web.webd import app
    from web.webd.config import WebConfig

    class SyntheticVideo:
        def __init__(self, **_):
            self.running = True

        def is_running(self):
            return self.running

        def latest(self):
            return b"synthetic-stream-payload", time.monotonic_ns(), time.monotonic()

        def stop(self):
            self.running = False

    app.VideoSource = SyntheticVideo
    app.WebdApp(WebConfig(host="127.0.0.1", port=port,
                         socket_path="/tmp/ota-shutdown-probe-no-controller.sock")).run()


async def probe():
    import websockets
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    with tempfile.TemporaryFile(mode="w+") as log:
        child = subprocess.Popen([sys.executable, __file__, "--server", str(port)],
                                 stdout=log, stderr=log)
        try:
            for _ in range(100):
                try:
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/api/health", timeout=.2).close()
                    break
                except OSError:
                    await asyncio.sleep(.05)
            async with websockets.connect(f"ws://127.0.0.1:{port}/ws"):
                # Leave a stream open and unread: this also exercises blocked sends.
                stream = socket.create_connection(("127.0.0.1", port))
                stream.sendall(b"GET /api/video HTTP/1.1\r\nHost: localhost\r\n\r\n")
                await asyncio.sleep(.2)
                started = time.monotonic()
                child.send_signal(signal.SIGTERM)
                await asyncio.to_thread(child.wait, timeout=4.8)
                elapsed = time.monotonic() - started
                stream.close()
            log.seek(0)
            output = log.read()
            print(output)
            assert "Application shutdown complete" in output, output
            assert "timeout graceful shutdown exceeded" not in output.lower(), output
            # Modern Uvicorn re-raises SIGTERM after graceful teardown.
            assert child.returncode in (0, -signal.SIGTERM), child.returncode
            print(f"PASS: active synthetic MJPEG/WebSocket shutdown in {elapsed:.3f}s")
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", type=int)
    args = parser.parse_args()
    if args.server:
        server(args.server)
    else:
        asyncio.run(probe())
