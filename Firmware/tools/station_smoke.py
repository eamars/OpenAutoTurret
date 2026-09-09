"""Read-only HTTP/WebSocket smoke test for an active station stack."""
from __future__ import annotations

import argparse
import json
import time
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, urlopen


def base_url(raw: str) -> str:
    parsed = urlsplit(raw.rstrip("/"))
    if parsed.scheme not in ("http", "https") or not parsed.netloc:
        raise ValueError("--url must include an http(s) scheme and host")
    return urlunsplit((parsed.scheme, parsed.netloc, "", "", "")).rstrip("/")


def get_bytes(url: str, timeout: float) -> bytes:
    request = Request(url, headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=timeout) as response:
        if response.status != 200:
            raise RuntimeError(f"{url} returned HTTP {response.status}")
        return response.read()


def get_json(url: str, timeout: float) -> dict:
    try:
        value = json.loads(get_bytes(url, timeout))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"{url} did not return JSON: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"{url} returned a non-object JSON value")
    return value


def ws_url(http_url: str) -> str:
    parsed = urlsplit(http_url)
    scheme = "wss" if parsed.scheme == "https" else "ws"
    return urlunsplit((scheme, parsed.netloc, "/ws", "", ""))


def receive_telemetry(url: str, timeout: float) -> dict:
    try:
        from websockets.sync.client import connect

        with connect(ws_url(url), open_timeout=timeout, close_timeout=timeout) as ws:
            raw = ws.recv(timeout=timeout)
    except Exception as error:
        raise RuntimeError(f"WebSocket connection failed: {error}") from error
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"WebSocket returned invalid JSON: {error}") from error
    if not isinstance(value, dict) or value.get("type") != "telemetry":
        raise RuntimeError("WebSocket did not return a telemetry message")
    return value


def readiness_gaps(state: dict) -> list[str]:
    gaps: list[str] = []
    if not state.get("controld_connected"):
        gaps.append("controld_connected")
    if not state.get("soft_limits_valid"):
        gaps.append("soft_limits_valid")
    if state.get("operating_mode") not in ("AUTO_ROAM", "AUTO_TRACK"):
        gaps.append(f"operating_mode={state.get('operating_mode')}")
    if state.get("supervisory_state") != "READY":
        gaps.append(f"supervisory_state={state.get('supervisory_state')}")
    if not state.get("vision_connected"):
        gaps.append("vision_connected")
    if not state.get("can_up"):
        gaps.append("can_up")
    if state.get("telemetry_stale"):
        gaps.append("telemetry_stale")
    try:
        if float(state.get("camera_fps", 0)) <= 0:
            gaps.append("camera_fps")
    except (TypeError, ValueError):
        gaps.append("camera_fps")
    return gaps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--wait-ready",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="wait for the normal automatic ready state after the smoke checks",
    )
    args = parser.parse_args()
    if args.timeout <= 0 or args.wait_ready < 0:
        parser.error("--timeout must be positive and --wait-ready cannot be negative")

    base = base_url(args.url)
    page = get_bytes(base + "/", args.timeout)
    if b"OpenAutoTurret" not in page:
        raise RuntimeError("station page did not contain the OpenAutoTurret dashboard")
    get_json(base + "/api/health", args.timeout)
    telemetry = receive_telemetry(base, args.timeout)
    print(
        "HTTP/WebSocket smoke passed: "
        f"phase={telemetry.get('phase')} fault={telemetry.get('fault', '')!r}"
    )

    if args.wait_ready == 0:
        return 0

    deadline = time.monotonic() + args.wait_ready
    next_report = 0.0
    while True:
        state = get_json(base + "/api/state", args.timeout)
        fault = state.get("fault")
        if fault:
            raise RuntimeError(f"station reported fault during activation: {fault}")
        gaps = readiness_gaps(state)
        if not gaps:
            print(
                "Station ready: "
                f"mode={state.get('operating_mode')} "
                f"phase={state.get('phase')} "
                f"camera_fps={state.get('camera_fps')}"
            )
            return 0
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(
                "station did not reach READY before timeout; " + ", ".join(gaps)
            )
        now = time.monotonic()
        if now >= next_report:
            print(f"Waiting for station readiness ({int(remaining)}s left): {', '.join(gaps)}")
            next_report = now + 10.0
        time.sleep(min(2.0, remaining))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        raise SystemExit(f"Station smoke failed: {error}") from error
