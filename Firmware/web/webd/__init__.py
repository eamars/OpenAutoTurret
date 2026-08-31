"""webd — the operator web dashboard (architecture §5.3, §42).

Reads the controld §6.3 telemetry snapshot over a Unix-domain socket and
serves it to browser clients (HTTP + WebSocket), and relays high-level
developer commands back to controld. It NEVER opens can0 and never drives the
motor; every command passes controld's validation gate (§42.2).

SAFETY: webd is a pure read/relay layer. It has no CAN device, no motor
handle, and no authority to move the turret — controld enforces all state
transitions.
"""
from .protocol import (
    CommandMessage,
    ResponseMessage,
    Telemetry,
    command_to_json,
    parse_message,
    telemetry_from_json,
    telemetry_to_json,
)

__all__ = [
    "CommandMessage",
    "ResponseMessage",
    "Telemetry",
    "command_to_json",
    "parse_message",
    "telemetry_from_json",
    "telemetry_to_json",
]
