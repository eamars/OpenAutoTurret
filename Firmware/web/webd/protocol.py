"""webd <-> controld JSON wire protocol (architecture §5.3, §6.1).

Transport is a ``SOCK_SEQPACKET`` Unix-domain socket, so each message is a
self-contained JSON object and message boundaries are preserved. Three message
types flow over it:

  * ``telemetry`` (controld -> webd): the §6.3 snapshot, downsampled to
    10-20 Hz. Read-only state for the dashboard.
  * ``command``   (webd -> controld): a high-level developer command
    (§42.2). ``{"type":"command","command":"<name>","arg":"<arg>"}``.
  * ``response``  (controld -> webd): the result of a command
    (``{"type":"response","command":"<name>","ok":..,"error":".."}``).

webd NEVER opens can0 and NEVER decides safety: it only submits commands, and
controld's ``validate_command`` gate decides whether they are legal. Video
frames do NOT traverse this socket (§42.3) — they are a separate low-priority
path.

The telemetry field names mirror ``format_telemetry`` in
``control/src/web/web_server.hpp`` exactly (same keys, same casing):
``track_state`` lowercase, ``installation_source`` lowercase, ``safety_action``
UPPERCASE.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from typing import Any, Optional, Tuple


@dataclass
class Telemetry:
    """One §6.3 telemetry snapshot (controld -> webd)."""

    ts_ns: int = 0
    track_state: str = "ready_hold"
    tracking_active: bool = False
    target_confidence: float = 0.0
    q_yaw_rad: float = 0.0
    v_yaw_rad_s: float = 0.0
    q_ref_yaw_rad: float = 0.0
    q_pitch_rad: float = 0.0
    v_pitch_rad_s: float = 0.0
    q_ref_pitch_rad: float = 0.0
    effort_yaw: float = 0.0
    effort_pitch: float = 0.0
    target_az_world_rad: float = 0.0
    target_el_world_rad: float = 0.0
    base_roll_rad: float = 0.0
    base_pitch_rad: float = 0.0
    base_yaw_rad: float = 0.0
    installation_calibrated: bool = False
    installation_source: str = "identity"
    safety_action: str = "ALLOW"
    feedback_age_ms: int = 0
    control_cycle_us: int = 0


@dataclass
class CommandMessage:
    """A high-level developer command (webd -> controld, §42.2)."""

    command: str
    arg: str = ""

    def to_json(self) -> str:
        return command_to_json(self.command, self.arg)


@dataclass
class ResponseMessage:
    """The result of a command (controld -> webd)."""

    command: str
    ok: bool
    error: str = ""


def command_to_json(command: str, arg: str = "") -> str:
    """Serialize a command message to the wire JSON string."""
    return json.dumps(
        {"type": "command", "command": command, "arg": arg},
        separators=(",", ":"),
    )


def telemetry_to_json(t: Telemetry) -> str:
    """Serialize a telemetry snapshot to the wire JSON string."""
    d = asdict(t)
    d["type"] = "telemetry"
    return json.dumps(d, separators=(",", ":"))


def telemetry_from_json(obj: dict) -> Telemetry:
    """Build a Telemetry from a decoded telemetry JSON object.

    Unknown keys are ignored and missing keys fall back to the dataclass
    defaults, so the wire format can grow without breaking old clients.
    """
    fields = {f for f in Telemetry.__dataclass_fields__}  # type: ignore[attr-defined]
    kwargs = {k: v for k, v in obj.items() if k in fields and k != "type"}
    return Telemetry(**kwargs)


def parse_message(raw: str) -> Tuple[str, Any]:
    """Decode one wire message. Returns ``(type, payload)`` where payload is a
    dict for telemetry (the raw JSON object) and a dataclass for command /
    response."""
    obj = json.loads(raw)
    t = obj.get("type")
    if t == "telemetry":
        return "telemetry", obj
    if t == "command":
        return "command", CommandMessage(
            command=obj.get("command", ""), arg=obj.get("arg", "")
        )
    if t == "response":
        return "response", ResponseMessage(
            command=obj.get("command", ""),
            ok=bool(obj.get("ok", False)),
            error=obj.get("error", ""),
        )
    raise ValueError(f"unknown message type: {t!r}")
