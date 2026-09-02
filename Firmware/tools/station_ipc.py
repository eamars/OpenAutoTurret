#!/usr/bin/env python3
"""controld UDS client — commands + telemetry one-liners (any python).

  python3 tools/station_ipc.py cmd start_payload_verification
  python3 tools/station_ipc.py telemetry
  python3 tools/station_ipc.py state        # phase + payload/tracking fields
"""
import json, re, socket, sys

SOCK = "/run/ota/controld-web.sock"

def _sock():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    s.connect(SOCK); s.settimeout(4); return s

def cmd(name):
    s = _sock()
    s.send(json.dumps({"type": "command", "command": name}).encode())
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
    if w == "cmd": cmd(sys.argv[2])
    elif w == "telemetry": telemetry()
    elif w == "state":
        telemetry(1, ["phase", "payload_check_active", "payload_profile_status",
                      "payload_derated", "track_state", "confidence",
                      "q_pitch", "q_yaw", "fault"])
    else: print(__doc__)
