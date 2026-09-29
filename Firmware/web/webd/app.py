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
  * FastAPI routes: ``/`` (v3.2 HUD), ``/dashboard`` (legacy engineering page),
    ``/api/state``, ``/api/health``,
    ``/api/command`` (POST), ``/ws`` (telemetry stream).

SAFETY: webd never opens can0 and never decides safety. It relays commands to
controld, whose validation gate (§42.2) is the sole authority.
"""
from __future__ import annotations

import asyncio
import os
import json
import logging
import dataclasses
import queue
import threading
import uvicorn
from contextlib import asynccontextmanager

from common.control_trace import TraceUnavailable, request_trace
from dataclasses import dataclass, field
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from .config import WebConfig, load_web_config
from .imu_state import ImuTraceReader
from .blackbox import BlackBoxWriter
from .controld_client import ControldClient
from .protocol import TELEMETRY_STALE_AFTER_S
from .dashboard import dashboard_html
from web.webd.hud import HUD_HTML
from .protocol import ResponseMessage, Telemetry, telemetry_to_json
from .video import VideoSource, mjpeg_frame
from .selection_client import request_selection


log = logging.getLogger("webd")


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
        self.stopping = threading.Event()

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
            while not self.stopping.is_set():
                try:
                    data = await asyncio.to_thread(s.q.get, True, 0.1)
                except queue.Empty:
                    continue
                await s.ws.send_text(data)
        except (WebSocketDisconnect, RuntimeError):
            pass
        except Exception:
            pass
        finally:
            self.remove(s)
            try:
                await s.ws.close(code=1001)
            except (WebSocketDisconnect, RuntimeError):
                pass


class CommandRequest(BaseModel):
    command: str
    arg: str = ""


class VideoStartRequest(BaseModel):
    """Optional start parameters; defaults come from the WebConfig (§53)."""

    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[float] = None


#: Roles the /api/video family accepts. Kept here, not imported from perception: webd also runs
#: on a host with no camera package, and a web daemon that cannot start because a sensor module
#: failed to import is a worse outage than a duplicated tuple of two strings.
STREAM_ROLES = ("wide", "detail")


def _read_streams(manifest_path: str) -> tuple:
    """Read the named-stream manifest visiond publishes. Returns (streams, why_absent).

    ``({}, "…")`` separates three states the UI has to say apart: nothing configured, nothing
    published yet, and a file that exists but does not parse.
    """
    if not manifest_path:
        return {}, "no manifest configured (OTA_VISION_STREAM_MANIFEST is unset)"
    try:
        with open(manifest_path, "r", encoding="utf-8") as handle:
            payload = json.loads(handle.read())
    except FileNotFoundError:
        return {}, f"no manifest published yet at {manifest_path}"
    except (OSError, ValueError) as exc:
        return {}, f"manifest at {manifest_path} is unreadable: {type(exc).__name__}: {exc}"
    streams = payload.get("streams") or {}
    unknown = sorted(set(streams) - set(STREAM_ROLES))
    if unknown:
        log.warning("stream manifest carries roles webd does not know: %s", ", ".join(unknown))
    return streams, ""


def _role_or_error(camera: Optional[str]) -> tuple:
    """No `camera` parameter means `wide`, so the pre-dual-stream HUD keeps working."""
    role = (camera or "wide").strip().lower()
    if role not in STREAM_ROLES:
        return None, (f"unknown camera {camera!r}; this station publishes "
                      + ", ".join(STREAM_ROLES))
    return role, ""


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

    # One source per named stream. `wide` reuses the instance every existing route already talks
    # to, so a client that never mentions `camera` cannot tell the difference. A second source is
    # created the first time someone asks for that stream, and each keeps its own slot: two
    # viewers, two rates, one file read per stream.
    sources = {"wide": video}

    def source_for(role: str) -> VideoSource:
        if role not in sources:
            sources[role] = VideoSource(enabled=config.video_enabled,
                                        orientation=config.video_orientation,
                                        white_balance=config.video_white_balance)
        return sources[role]

    # One reader for the process: it keeps the byte offset it has already consumed, so a new
    # instance per request would re-read the whole trace every time the dashboard polls.
    imu_reader = ImuTraceReader(path=config.imu_trace or "/nonexistent/imu.ndjson",
                                fresh_ms=int(config.imu_fresh_ms))

    @asynccontextmanager
    async def lifespan(app: FastAPI):  # noqa: ARG001
        client.start()
        yield
        # Release the camera on shutdown (blocking; keep it off the loop).
        hub.stopping.set()
        await asyncio.gather(asyncio.to_thread(video.stop), asyncio.to_thread(client.stop))

    app = FastAPI(title=f"{config.title} webd", lifespan=lifespan)
    # Set BEFORE Uvicorn drains connections, not in the post-drain lifespan.
    app.state.begin_shutdown = hub.stopping.set

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        # v3.2 s3: the operator view is the camera-dominant HUD. The card dashboard is what
        # s3 rules out, but its engineering numbers are still needed until the DIAG drawer
        # exists, so it stays reachable rather than being deleted out from under anyone.
        return HUD_HTML

    @app.get("/dashboard", response_class=HTMLResponse)
    async def dashboard() -> str:
        """The pre-v3.2 engineering page. Kept until the HUD's DIAG drawer carries the same
        numbers; s23 means it is a tool, not the operator view."""
        return dashboard_html(config.title)

    def _stamped(t):
        """Attach webd's own view of how old this snapshot is, on a copy.

        The cached Telemetry object belongs to the client and is shared by every reader, so the age
        is written onto a replacement rather than onto the cached instance: mutating it would make
        the snapshot's age depend on who read it last, which is the class of bug that produces a page
        that looks fresh for exactly as long as nobody looks at it.
        """
        age_s = client.telemetry_age_s()
        age_ms = None if age_s is None else int(round(age_s * 1000.0))
        return dataclasses.replace(
            t,
            telemetry_age_ms=age_ms,
            telemetry_stale=bool(age_s is None or age_s > TELEMETRY_STALE_AFTER_S),
        )


    @app.get("/api/state")
    async def state() -> JSONResponse:
        t = client.latest_telemetry()
        if t is None:
            return JSONResponse(
                status_code=503, content={"error": "no telemetry yet"}
            )
        payload = {"type": "telemetry", "controld_connected": client.connected(),
                   **json.loads(telemetry_to_json(_stamped(t)))}
        # (b): the identity belongs to whoever holds the sensor, and that is visiond. Its manifest
        # is the only honest source for "which camera is this", so the empty strings controld
        # publishes get filled here rather than left to look like a camera with no name.
        streams, absent = _read_streams(config.stream_manifest)
        wide = streams.get("wide") or {}
        if wide.get("camera_id"):
            payload["camera_id"] = wide["camera_id"]
            payload["camera_identity_source"] = wide.get("identity_source") or ""
        payload["video_streams"] = [
            {"role": role, "camera_id": entry.get("camera_id"),
             "identity_source": entry.get("identity_source"),
             "durable": entry.get("durable"), "delivered_fps": entry.get("delivered_fps"),
             "dropped": entry.get("dropped"), "width": entry.get("width"),
             "height": entry.get("height"),
             "running": bool(sources.get(role) and sources[role].is_running())}
            for role, entry in sorted(streams.items())]
        if absent:
            payload["video_streams_error"] = absent
        # controld's §20 imu block stays the base (it is the control-side claim); what we can see
        # in the trace is layered on top, because controld's imu_present has never been assigned.
        imu = dict(payload.get("imu") or {})
        imu.update(imu_reader.read_once())
        payload["imu"] = imu
        return JSONResponse(payload)

    @app.get("/api/health")
    async def health() -> dict:
        return {
            "ok": True,
            "controld_connected": client.connected(),
            # Non-zero here means controld is publishing and webd is refusing what it hears. It
            # belongs beside `controld_connected` rather than somewhere clever, because the pair
            # is the whole diagnosis: connected but refusing frames looks identical to disconnected
            # from the dashboard, and is a different emergency.
            "telemetry_age_ms": (lambda a: None if a is None else int(round(a * 1000.0)))(
                client.telemetry_age_s()),
            "malformed_frames": getattr(client, "malformed_frames", 0),
            "browser_clients": hub.client_count,
        }

    @app.post("/api/command")
    async def command(req: CommandRequest) -> ResponseMessage:
        return client.send_command(req.command, req.arg)

    @app.post("/api/selection")
    async def selection(request: dict) -> dict:
        return await asyncio.to_thread(request_selection,
            os.environ.get('OTA_SELECTION_SOCKET', '/tmp/ota-selection.sock'), request)

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

    @app.get("/api/control_trace")
    async def control_trace() -> JSONResponse:
        """controld's per-cycle ring, handed to the browser as-is.

        The ring is the only witness the station has of a per-cycle fault, and it
        wraps in about twenty seconds -- which is exactly the window an operator
        is inside while deciding what to do. Until now the only way to read it was
        a shell on the station (``tools/pull_control_trace.py``); both readers now
        share ``common/control_trace.py`` so the frame-size floor and the
        "skip telemetry until the frame that says so" rule cannot drift apart.

        503 rather than an empty window when the trace cannot be read: "no
        anomalies in the ring" and "the ring was unreadable" have to stay two
        different sentences during an incident.
        """
        try:
            frame = await asyncio.to_thread(request_trace, config.socket_path)
        except TraceUnavailable as exc:
            return JSONResponse(status_code=503, content={"error": str(exc)})
        return JSONResponse(frame)

    # -- video preview (separate low-priority path, §42.3) ------------------

    @app.get("/api/video/state")
    async def video_state(camera: Optional[str] = None) -> dict:
        role, problem = _role_or_error(camera)
        if problem:
            return {"error": problem, "roles": list(STREAM_ROLES)}
        streams, absent = _read_streams(config.stream_manifest)
        entry = streams.get(role) or {}
        return {**source_for(role).state().to_dict(), "role": role,
                "published": bool(entry), "published_reason": absent,
                "delivered_fps": entry.get("delivered_fps"),
                "camera_id": entry.get("camera_id")}

    @app.post("/api/video/start")
    async def video_start(camera: Optional[str] = None,
                          req: Optional[VideoStartRequest] = None) -> JSONResponse:
        # Body is optional: a bare POST (no JSON body) starts with the defaults.
        req = req or VideoStartRequest()
        role, problem = _role_or_error(camera)
        if problem:
            return JSONResponse({"ok": False, "error": problem, "roles": list(STREAM_ROLES)})
        streams, absent = _read_streams(config.stream_manifest)
        entry = streams.get(role)
        if not entry and role != "wide":
            # "Not published" is not "start anyway": guessing a filename is how a UI ends up
            # showing one camera twice and calling it two streams.
            return JSONResponse({"ok": False, "role": role,
                                 "error": f"no {role} stream published ({absent or 'visiond '
                                          'published no entry for it'})"})
        # `wide` with no manifest keeps the pre-(b) behaviour (the OTA_VISION_FRAME_TAP file, or
        # webd's own camera): the old HUD and the old deployments must keep working, and the
        # response says which path it took rather than quietly looking like a named stream.
        legacy_fallback = "" if entry else f"no manifest entry ({absent}); served the legacy path"
        width = req.width or config.video_width
        height = req.height or config.video_height
        fps = req.fps if req.fps and req.fps > 0 else float(config.video_fps)
        # Camera open is blocking — keep it off the event loop.
        st = await asyncio.to_thread(
            source_for(role).start, width, height, fps, config.video_quality,
            str((entry or {}).get("path") or ""), role)
        return JSONResponse({"ok": st.running, "role": role,
                             "camera_id": (entry or {}).get("camera_id") or "",
                             "manifest_fallback": legacy_fallback, **st.to_dict()})

    @app.post("/api/video/stop")
    async def video_stop(camera: Optional[str] = None) -> JSONResponse:
        role, problem = _role_or_error(camera)
        if problem:
            return JSONResponse({"ok": False, "error": problem})
        st = await asyncio.to_thread(source_for(role).stop)
        return JSONResponse({"ok": True, **st.to_dict()})

    @app.get("/api/video")
    async def video_stream(limit: Optional[int] = None,
                           camera: Optional[str] = None) -> StreamingResponse:
        """MJPEG stream (multipart/x-mixed-replace). Only while the video is on;
        every client re-sends a frame only when it changes, so N viewers share
        one capture. ``?limit=N`` caps the number of frames (production safety
        valve; also makes the stream bounded for tests)."""
        role, problem = _role_or_error(camera)
        if problem:
            return JSONResponse(status_code=400,
                                content={"error": problem, "roles": list(STREAM_ROLES)})
        source = source_for(role)
        if not source.is_running():
            return JSONResponse(
                status_code=409, content={"error": f"{role} video not running"}
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
                if hub.stopping.is_set() or not source.is_running():
                    break
                jpeg, seq, _ts = source.latest()
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


class WebdServer(uvicorn.Server):
    async def shutdown(self, sockets=None):
        self.config.app.state.begin_shutdown()
        # A client that stopped reading can be blocked inside ASGI send(),
        # before the generator gets to observe the stop flag. Close only the
        # MJPEG transports so Uvicorn delivers disconnect and cancels the send.
        for connection in list(self.server_state.connections):
            scope = getattr(connection, "scope", {})
            if scope.get("type") == "http" and scope.get("path") == "/api/video":
                # close() itself waits for a blocked output buffer to drain.
                connection.transport.abort()
        await super().shutdown(sockets=sockets)


class WebdApp:
    """Lifecycle wrapper: config + client + app (+ optional uvicorn server)."""

    def __init__(self, config: Optional[WebConfig] = None) -> None:
        self.config = config or load_web_config()
        self.client = ControldClient(self.config.socket_path)
        self.app = create_app(self.client, self.config)
        self._server = None

    def run(self) -> int:
        """Run uvicorn in-process (blocking). Used by the daemon entry point."""
        config = uvicorn.Config(
            self.app,
            host=self.config.host,
            port=self.config.port,
            log_level="info",
            # Reserve up to 3 seconds for source cleanup inside the launcher's
            # 5-second window. WebdServer ends streams before connection drain.
            timeout_graceful_shutdown=0.75,
        )
        self._server = WebdServer(config)
        self._server.run()
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
