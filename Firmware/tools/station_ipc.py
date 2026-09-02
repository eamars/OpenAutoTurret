#!/usr/bin/env python3
"""controld UDS client — commands + telemetry one-liners (any python).

  python3 tools/station_ipc.py cmd start_payload_verification
  python3 tools/station_ipc.py cmd select_payload_profile conservative
  python3 tools/station_ipc.py telemetry
  python3 tools/station_ipc.py state        # phase + at_ready + payload/tracking fields
  python3 tools/station_ipc.py --socket /path/controld-web.sock state
"""
import json, os, re, socket, sys

# Same precedence as the daemon: explicit arg > $OTA_WEB_SOCKET > its default.
SOCK = os.environ.get("OTA_WEB_SOCKET", "/run/ota/controld-web.sock")
if len(sys.argv) > 2 and sys.argv[1] in ("--socket",):
    SOCK = sys.argv[2]
    sys.argv = [sys.argv[0]] + sys.argv[3:]

def _sock():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.connect(SOCK); s.settimeout(4); return s

def cmd(name, arg=None):
    s = _sock()
    msg = {"type": "command", "command": name}
    if arg is not None:
        msg["arg"] = arg
    s.send(json.dumps(msg).encode())
    buf = b""
    while True:
        try: buf += s.recv(65536)
        except socket.timeout: break
        if re.search(rb'\{"type":"response".*?\}', buf): break
    for m in re.findall(r'\{"type":"response".*?\}', buf.decode(errors="replace")):
        print(m)

def telemetry(n=1, keys=None):
    s = _sock(); got = 0
    while got < n:
        try: d = json.loads(s.recv(65536).decode())
        except socket.timeout: return
        if d.get("type") == "telemetry":
            got += 1
            if keys: print({k: d.get(k, "?") for k in keys})
            else: print(json.dumps(d)[:600])

if __name__ == "__main__":
    if not __import__("os").path.exists(SOCK):
        sys.exit(f"no controld socket at {SOCK} (daemon running?)")
    w = sys.argv[1] if len(sys.argv) > 1 else "telemetry"
    if w == "cmd":
        if len(sys.argv) < 3:
            sys.exit("usage: station_ipc.py cmd <command> [arg]")
        cmd(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
    elif w == "telemetry": telemetry()
    elif w == "state":
        # at_ready is here deliberately: it is the P0 "homed + at ready pose"
        # line, and it is the honest precondition for the developer commands
        # (a payload check started while still travelling aborts with a clamped
        # region). phase == hold is NOT enough: homing passes through hold
        # between stages.
        telemetry(1, ["phase", "at_ready", "payload_check_active",
                      "payload_profile_status",
                      "payload_derated", "track_state", "target_confidence",
                      "q_pitch_rad", "q_yaw_rad", "fault", "safety_action",
                      "vision_connected", "vision_frames",
                      "vision_measurement_age_ms"])
    else: print(__doc__)
