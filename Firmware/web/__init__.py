"""OpenAutoTurret web layer (Phase 8).

``webd`` is the operator-facing web service (architecture §5.3, §42): a FastAPI
HTTP + WebSocket dashboard that reads the controld telemetry snapshot and
submits high-level developer commands. It NEVER opens can0 and never decides
safety — every command is validated by controld's state machine before it is
executed (§42.2). Video frames do not traverse the control IPC (§42.3).
"""
