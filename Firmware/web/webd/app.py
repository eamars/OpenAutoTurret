"""webd FastAPI application (architecture §5.3, §42, §54.5).

Wiring:
  * a :class:`ControldClient` keeps the UDS to controld alive and delivers
    telemetry + command responses;
  * a :class:`TelemetryHub` fans telemetry out to connected browser clients
    over WebSockets. Each client has its own bounded queue; a slow client is
    DROPPED rather than blocking anyone (§42.3 "control timing wins over
    browser video"). Because webd is a separate process from controld and
    controld's web server runs on a non-RT thread, browser load can never
    degrade the control loop or CAN feedback staleness (§54.5).
  * FastAPI routes: ``/`` (dashboard), ``/api/state``, ``/api/health``,
    ``/api/command`` (POST), ``/ws`` (telemetry stream).

SAFETY: webd never opens can0 and never decides safety. It relays commands to
controld, whose validation gate (§42.2) is the sole authority.
"""
from __future__ import annotations

import asyncio
import os
import json
import queue
import threading
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from .config import WebConfig, load_web_config
from .blackbox import BlackBoxWriter
from .controld_client import ControldClient
from .dashboard import dashboard_html
from .protocol import ResponseMessage, Telemetry, telemetry_to_json
from .video import VideoSource, mjpeg_frame


@dataclass
class ClientSession:
    """One connected browser client: its socket + a bounded telemetry queue."""

    ws: WebSocket
    q: "queue.Queue[str]" = field(default_factory=lambda: queue.Queue(maxsize=256))


class TelemetryHub:
    """Fans telemetry out to browser clients; drops slow clients (§42.3)."""

    def __init__(self) -> None:
        self._sessions: list[ClientSession] = []
        self._lock = threading.Lock()

    def add(self, ws: WebSocket) -> ClientSession:
        s = ClientSession(ws=ws)
        with self._lock:
            self._sessions.append(s)
        return s

    def remove(self, s: ClientSession) -> None:
        with self._lock:
            if s in self._sessions:
                self._sessions.remove(s)

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._sessions)

    def on_telemetry(self, t: Telemetry) -> None:
        """Called on the controld-client reader thread. Non-blocking: a slow
        client's full queue causes its next frame to be dropped, never a
        block of the reader or the control path."""
        data = telemetry_to_json(t)
        with self._lock:
            sessions = list(self._sessions)
        for s in sessions:
            try:
                s.q.put_nowait(data)
            except queue.Full:
                pass  # slow client: drop this frame (control wins, §42.3)

    async def serve(self, s: ClientSession, latest: Optional[Telemetry]) -> None:
        """Drain one client's queue until it disconnects."""
        try:
            if latest is not None:
                await s.ws.send_text(telemetry_to_json(latest))
            while True:
                try:
                    data = await asyncio.to_thread(s.q.get, True, 1.0)
                except queue.Empty:
                    continue
                await s.ws.send_text(data)
        except (WebSocketDisconnect, RuntimeError):
            pass
        except Exception:
            pass
        finally:
            self.remove(s)


class CommandRequest(BaseModel):
    command: str
    arg: str = ""


class VideoStartRequest(BaseModel):
    """Optional start parameters; defaults come from the WebConfig (§53)."""

    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[float] = None


def create_app(client: ControldClient, config: WebConfig) -> FastAPI:
    """Build the FastAPI app around a live ControldClient."""
    hub = TelemetryHub()

    # Re-point the client's telemetry callback at the hub (so the dashboard
    # gets live frames). Kept off the control path: this only feeds browsers.
    # §80: the hub keeps the page fed; this keeps the evidence. Composed here rather than
    # inside the hub, because the hub's contract is "what the browser sees" and this is a
    # side effect on disk — and because the writer is absent unless a directory is named.
    # Both run on the client's reader thread, which is not the control loop: a hung disk
    # can stall the dashboard, and must never be able to stall the turret.
    blackbox = BlackBoxWriter(config.blackbox_dir)

    def on_telemetry(t: Telemetry) -> None:
        hub.on_telemetry(t)
        blackbox.observe(t)

    client.on_telemetry = on_telemetry

    # Separate low-priority video source (§42.3): its own path from the IMX500,
    # never through the control socket. Off until a client turns it on.
    video = VideoSource(
        enabled=config.video_enabled,
        orientation=config.video_orientation,
        white_balance=config.video_white_balance,
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):  # noqa: ARG001
        client.start()
        yield
        # Release the camera on shutdown (blocking; keep it off the loop).
        await asyncio.to_thread(video.stop)
        client.stop()

    app = FastAPI(title=f"{config.title} webd", lifespan=lifespan)

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return dashboard_html(config.title)

    @app.get("/api/state")
    async def state() -> JSONResponse:
        t = client.latest_telemetry()
        if t is None:
            return JSONResponse(
                status_code=503, content={"error": "no telemetry yet"}
            )
        return JSONResponse({"type": "telemetry", **json.loads(telemetry_to_json(t))})

    @app.get("/api/health")
    async def health() -> dict:
        return {
            "ok": True,
            "controld_connected": client.connected(),
            # Non-zero here means controld is publishing and webd is refusing what it hears. It
            # belongs beside `controld_connected` rather than somewhere clever, because the pair
            # is the whole diagnosis: connected but refusing frames looks identical to disconnected
            # from the dashboard, and is a different emergency.
            "malformed_frames": getattr(client, "malformed_frames", 0),
            "browser_clients": hub.client_count,
        }

    @app.post("/api/command")
    async def command(req: CommandRequest) -> ResponseMessage:
        return client.send_command(req.command, req.arg)

    @app.get("/api/payload_profiles")
    async def payload_profiles() -> dict:
        """The stored profile NAMES, for the dashboard's picker (§28.5).

        Names only, and a courtesy: controld re-validates every
        `select_payload_profile` and answers with a reason when a name has no
        file (§31.3), so a stale or mis-pathed listing here is visible, not
        dangerous. `dir` is echoed because the commonest failure is webd and
        controld running from different working directories — the operator has
        to be able to compare it against turret.yaml's payload.profile_dir.
        """
        directory = config.payload_profile_dir
        try:
            names = sorted(
                os.path.splitext(f)[0] for f in os.listdir(directory)
                if f.endswith((".yaml", ".yml")) and not f.startswith("."))
            error = "" if names else (
                f"no profile files in {os.path.abspath(directory)} — check "
                "that this matches payload.profile_dir in config/turret.yaml")
        except OSError as e:
            names, error = [], f"cannot read {os.path.abspath(directory)}: {e.strerror}"
        return {"dir": os.path.abspath(directory), "profiles": names,
                "error": error}

    # -- video preview (separate low-priority path, §42.3) ------------------

    @app.get("/api/video/state")
    async def video_state() -> dict:
        return video.state().to_dict()

    @app.post("/api/video/start")
    async def video_start(req: Optional[VideoStartRequest] = None) -> JSONResponse:
        # Body is optional: a bare POST (no JSON body) starts with the defaults.
        req = req or VideoStartRequest()
        width = req.width or config.video_width
        height = req.height or config.video_height
        fps = req.fps if req.fps and req.fps > 0 else float(config.video_fps)
        # Camera open is blocking — keep it off the event loop.
        st = await asyncio.to_thread(video.start, width, height, fps,
                                     config.video_quality)
        return JSONResponse({"ok": st.running, **st.to_dict()})

    @app.post("/api/video/stop")
    async def video_stop() -> JSONResponse:
        st = await asyncio.to_thread(video.stop)
        return JSONResponse({"ok": True, **st.to_dict()})

    @app.get("/api/video")
    async def video_stream(limit: Optional[int] = None) -> StreamingResponse:
        """MJPEG stream (multipart/x-mixed-replace). Only while the video is on;
        every client re-sends a frame only when it changes, so N viewers share
        one capture. ``?limit=N`` caps the number of frames (production safety
        valve; also makes the stream bounded for tests)."""
        if not video.is_running():
            return JSONResponse(
                status_code=409, content={"error": "video not running"}
            )

        async def gen():
            last_seq = -1
            sent = 0
            stale_s = 0.0
            while limit is None or sent < limit:
                # End the stream when the source is switched off (or dies). Without
                # this an open <img> keeps the connection — and therefore uvicorn's
                # graceful shutdown — alive indefinitely: `systemctl stop turret-web`
                # would then sit in "Waiting for connections to close" and never run
                # the lifespan shutdown that releases the IMX500, so a browser left
                # open could hold the camera away from visiond until TimeoutStopSec
                # SIGKILLs us.
                if not video.is_running():
                    break
                jpeg, seq, _ts = video.latest()
                if seq != last_seq and jpeg:
                    last_seq = seq
                    sent += 1
                    stale_s = 0.0
                    yield mjpeg_frame(jpeg)
                else:
                    await asyncio.sleep(0.02)
                    stale_s += 0.02
                    if stale_s > 10.0:
                        break        # capture stalled: let the client reconnect

        return StreamingResponse(
            gen(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        session = hub.add(ws)
        await hub.serve(session, client.latest_telemetry())

    return app


class WebdApp:
    """Lifecycle wrapper: config + client + app (+ optional uvicorn server)."""

    def __init__(self, config: Optional[WebConfig] = None) -> None:
        self.config = config or load_web_config()
        self.client = ControldClient(self.config.socket_path)
        self.app = create_app(self.client, self.config)
        self._server = None

    def run(self) -> int:
        """Run uvicorn in-process (blocking). Used by the daemon entry point."""
        import uvicorn

        uvicorn.run(
            self.app,
            host=self.config.host,
            port=self.config.port,
            log_level="info",
            # Bounded graceful exit. A browser holding the MJPEG stream must not
            # decide when webd stops: the stream now ends when video stops, but a
            # stuck client would otherwise keep uvicorn in "Waiting for
            # connections to close" — past the lifespan shutdown that releases
            # the camera — until systemd's TimeoutStopSec (15 s) kills the unit.
            timeout_graceful_shutdown=5,
        )
        return 0


def create_webd_app() -> FastAPI:
    """Factory for the external uvicorn CLI:
    ``uvicorn web.webd.app:create_webd_app --factory``.

    Builds a fresh WebdApp (config from env, §53) and returns its FastAPI app.
    The ControldClient is started by the app's lifespan, so the same app object
    works for both the in-process and external-server paths.
    """
    return WebdApp().app


def main() -> int:
    """Entry point: ``python -m web.webd.app``."""
    WebdApp().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
