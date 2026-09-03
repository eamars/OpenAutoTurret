"""webd configuration (architecture §53: no hard-coded IP).

Everything is resolved from environment variables with sensible defaults, so
the same build runs on any host. The browser-facing bind host/port and the
controld socket path are all configurable at deploy time; nothing is baked in.

Env vars:
  OTA_WEB_HOST    bind host for the HTTP/WS server (default 0.0.0.0)
  OTA_WEB_PORT    bind port (default 8080)
  OTA_WEB_SOCKET  controld UDS path (default /run/ota/controld-web.sock)
  OTA_WEB_HZ      telemetry rate hint for the dashboard (default 15)

  # Video preview (separate low-priority path, §42.3). The stream is OFF until
  # a client turns it on; these set the resolution / capped FPS / JPEG quality
  # used when it is turned on.
  OTA_VIDEO_ENABLE    1 = feature available (default 1); 0 = never open camera
  OTA_VIDEO_WIDTH     default stream width (default 640)
  OTA_VIDEO_HEIGHT    default stream height (default 480)
  OTA_VIDEO_FPS       capped publish FPS (default 15, §42.3 "reduce FPS")
  OTA_VIDEO_QUALITY   JPEG quality 1..95 (default 80)
  OTA_VIDEO_ORIENTATION install orientation correction (default none):
                        none | rotate_180 | flip_horizontal | flip_vertical
  OTA_VIDEO_WB        white balance: off | auto (gray-world, default off).
                        off = trust the sensor (neutral once BGRX order is correct);
                        auto = software gray-world for a genuinely mis-balanced install
                        applied at capture, before processing / control
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from common import image_corrections as ic


@dataclass
class WebConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    socket_path: str = "/run/ota/controld-web.sock"
    # §80: where preserved scenes are written. Empty means the writer is not installed at
    # all. It is opt-in because writing files is a side effect, and a side effect that
    # appears because code was merged is a surprise to whoever has to explain the disk
    # filling up. Set OTA_BLACKBOX_DIR to turn it on.
    blackbox_dir: str = ""
    telemetry_hz: int = 15
    title: str = "OpenAutoTurret"
    video_enabled: bool = True
    video_width: int = 640
    video_height: int = 480
    video_fps: int = 15
    video_quality: int = 80
    video_orientation: str = "none"
    video_white_balance: str = "off"
    # Where the payload profiles live, ONLY so the dashboard can offer a list.
    # The daemon reads its own cfg.payload.profile_dir and is the authority: a
    # name webd lists but controld cannot find is REJECTED with a reason
    # (§31.3/§42.2), never applied. Keep this equal to turret.yaml's
    # payload.profile_dir or the dropdown just offers lies.
    payload_profile_dir: str = "config/payload_profiles"


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _env_choice(name: str, default: str, choices) -> str:
    """Read an env var, keeping it within ``choices`` (else the default)."""
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    raw = raw.strip().lower()
    return raw if raw in choices else default


def load_web_config() -> WebConfig:
    """Build a WebConfig from the environment."""
    return WebConfig(
        host=os.environ.get("OTA_WEB_HOST", "0.0.0.0"),
        port=_env_int("OTA_WEB_PORT", 8080),
        socket_path=os.environ.get("OTA_WEB_SOCKET", "/run/ota/controld-web.sock"),
        blackbox_dir=os.environ.get("OTA_BLACKBOX_DIR", ""),
        telemetry_hz=_env_int("OTA_WEB_HZ", 15),
        title=os.environ.get("OTA_WEB_TITLE", "OpenAutoTurret"),
        video_enabled=_env_flag("OTA_VIDEO_ENABLE", True),
        video_width=_env_int("OTA_VIDEO_WIDTH", 640),
        video_height=_env_int("OTA_VIDEO_HEIGHT", 480),
        video_fps=max(1, _env_int("OTA_VIDEO_FPS", 15)),
        video_quality=min(95, max(1, _env_int("OTA_VIDEO_QUALITY", 80))),
        video_orientation=_env_choice("OTA_VIDEO_ORIENTATION", "none",
                                      ic.ORIENTATIONS),
        video_white_balance=_env_choice("OTA_VIDEO_WB", "off",
                                        ic.WHITE_BALANCES),
        payload_profile_dir=os.environ.get("OTA_PAYLOAD_PROFILE_DIR",
                                           "config/payload_profiles"),
    )
