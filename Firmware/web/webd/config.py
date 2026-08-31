"""webd configuration (architecture §53: no hard-coded IP).

Everything is resolved from environment variables with sensible defaults, so
the same build runs on any host. The browser-facing bind host/port and the
controld socket path are all configurable at deploy time; nothing is baked in.

Env vars:
  OTA_WEB_HOST    bind host for the HTTP/WS server (default 0.0.0.0)
  OTA_WEB_PORT    bind port (default 8080)
  OTA_WEB_SOCKET  controld UDS path (default /run/ota/controld-web.sock)
  OTA_WEB_HZ      telemetry rate hint for the dashboard (default 15)
"""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass
class WebConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    socket_path: str = "/run/ota/controld-web.sock"
    telemetry_hz: int = 15
    title: str = "OpenAutoTurret"


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def load_web_config() -> WebConfig:
    """Build a WebConfig from the environment."""
    return WebConfig(
        host=os.environ.get("OTA_WEB_HOST", "0.0.0.0"),
        port=_env_int("OTA_WEB_PORT", 8080),
        socket_path=os.environ.get("OTA_WEB_SOCKET", "/run/ota/controld-web.sock"),
        telemetry_hz=_env_int("OTA_WEB_HZ", 15),
        title=os.environ.get("OTA_WEB_TITLE", "OpenAutoTurret"),
    )
