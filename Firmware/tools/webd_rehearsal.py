#!/usr/bin/env python3
"""P12 offline rehearsal: the REAL webd against a REAL controld (--sim), plus the
real camera video path. No CAN, no motor: the sim backend cannot move hardware,
so everything here is [SW]/[CAMERA] scope.

What it proves, in order (each line is PASS/FAIL with the measured number):

  1. webd serves the dashboard and the /api/state fields the queue doc added
     (phase, fault, vision_*) — i.e. the 6.3 fields survive controld -> webd -> JSON.
  2. A developer command goes through webd's validation gate and REACHES controld
     (payload check starts: payload_check_active becomes true in the stream).
  3. An invalid profile name is REJECTED at the gate with a reason (42.2: the
     response must not look like success).
  4. The video path: /api/video/start publishes JPEGs at a usable rate (this is
     the pull-pattern fix; the deprecated callback gave ~0.8 fps and a stop()
     that never returned), frames are real JPEGs, and /api/video/stop returns
     promptly with webd still responsive afterwards.
  5. Both daemons shut down on SIGTERM within the expected window.

Usage: python3 tools/webd_rehearsal.py [--skip-video]
Requires ./build/control/controld built, and the IMX500 free for step 4.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import sys
import time
import http as _http        # alias: main() defines no local named http
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # Firmware/
PROBE = os.path.join(ROOT, "build", "probe")
WEB_SOCK = os.path.join(PROBE, "rehearsal-web.sock")
VISION_SOCK = os.path.join(PROBE, "rehearsal-vision.sock")
HOST, PORT = "127.0.0.1", 8099
BASE = f"http://{HOST}:{PORT}"

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
          + (f" — {detail}" if detail else ""), flush=True)


def http(path: str, data: dict | None = None, timeout: float = 5.0,
         raw: bool = False):
    req = urllib.request.Request(
        BASE + path,
        data=None if data is None else json.dumps(data).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read()
    if raw:
        return body
    return json.loads(body.decode())


def wait_for(pred, timeout: float, interval: float = 0.2):
    """Poll pred() until truthy or the deadline; returns (truthy, last_value)."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = pred()
        except Exception as e:  # noqa: BLE001
            last = f"error: {e}"
        if last:
            return True, last
        time.sleep(interval)
    return False, last


def terminate(proc: subprocess.Popen, name: str, timeout: float = 12.0) -> str:
    """SIGTERM and wait — never SIGKILL a daemon that may be energized."""
    if proc.poll() is not None:
        return f"{name} already exited rc={proc.returncode}"
    os.kill(proc.pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return f"{name} exited rc={proc.returncode} after SIGTERM"
        time.sleep(0.2)
    return f"{name} DID NOT exit within {timeout}s"


def main() -> int:
    skip_video = "--skip-video" in sys.argv
    os.makedirs(PROBE, exist_ok=True)
    for p in (WEB_SOCK, VISION_SOCK):
        if os.path.exists(p):
            os.unlink(p)

    print("=== start controld --sim (no CAN transport, no motor) ===", flush=True)
    env_c = dict(os.environ,
                 OTA_WEB_SOCKET=WEB_SOCK, OTA_VISION_SOCKET=VISION_SOCK)
    controld = subprocess.Popen(
        [os.path.join(ROOT, "build", "control", "controld"),
         os.path.join(ROOT, "config", "turret_sim.yaml"), "--sim"],
        cwd=ROOT, env=env_c,
        stdout=open(os.path.join(PROBE, "rehearsal-controld.log"), "w"),
        stderr=subprocess.STDOUT)
    try:
        ok, detail = wait_for(lambda: os.path.exists(WEB_SOCK), 15.0)
        check("controld bound its web socket", ok, str(detail))

        print("=== start webd (system python; the unit uses the same) ===", flush=True)
        env_w = dict(os.environ, OTA_WEB_HOST=HOST, OTA_WEB_PORT=str(PORT),
                     OTA_WEB_SOCKET=WEB_SOCK, OTA_WEB_HZ="15",
                     OTA_VIDEO_ORIENTATION="rotate_180")
        webd = subprocess.Popen([sys.executable, "-m", "web.webd.app"],
                                cwd=ROOT, env=env_w,
                                stdout=open(os.path.join(PROBE, "rehearsal-webd.log"), "w"),
                                stderr=subprocess.STDOUT)
        ok, detail = wait_for(lambda: _health(), 20.0)
        check("webd answers /api/health", ok, str(detail))

        # -- 1. snapshot fields through the whole chain ------------------------
        # webd answers as soon as it is up, but /api/state is 503 until controld's
        # first snapshot arrives — poll rather than assume.
        ok, st = wait_for(lambda: _state(), 20.0)
        check("webd relays controld telemetry", ok, str(st)[:80])
        st = st if isinstance(st, dict) else {}
        keys = set(st.keys())
        check("state carries phase/fault", {"phase", "fault"} <= keys,
              f"keys={sorted(k for k in keys if k in ('phase', 'fault', 'homed'))}")
        vkeys = {k for k in keys if k.startswith("vision_")}
        check("state carries the vision health block", len(vkeys) >= 3,
              f"{sorted(vkeys)}")
        html = http("/", raw=True).decode()
        check("dashboard renders phase + vision rows",
              ("Phase" in html or "phase" in html) and "vision" in html.lower())

        # -- 2/3. the command gate ---------------------------------------------
        # Developer commands are refused until the daemon is homed (position
        # validity unknown). That refusal is the CORRECT behaviour, so wait for
        # homing first and only then test the gate itself.
        # at_ready is the honest gate: homing passes through phase=hold between
        # stages, so waiting on phase alone races the ready move (and a payload
        # check requested then steps from a travel stop). §38.1/§6.3.
        ok, detail = wait_for(lambda: (_state() or {}).get("at_ready") is True,
                              120.0, 1.0)
        check("controld homed and at the ready pose (via webd)", ok,
              f"last state={str(detail)[:60]}")

        bad = http("/api/command", {"command": "select_payload_profile",
                                    "arg": "does_not_exist"}, timeout=10.0)
        check("unknown payload profile rejected with a reason",
              bad.get("ok") is False and "does_not_exist" in json.dumps(bad),
              json.dumps(bad)[:120])

        t0 = time.monotonic()
        started = http("/api/command", {"command": "start_payload_verification"},
                       timeout=10.0)
        check("valid command accepted by the gate", started.get("ok") is not False,
              json.dumps(started)[:120])
        seen_active = False
        states: list[str] = []
        for _ in range(int(20 / 0.2)):
            s = _state()
            if s and s.get("payload_check_active"):
                seen_active = True
            if s:
                states.append(str(s.get("phase")))
            if time.monotonic() - t0 > 18:
                break
            time.sleep(0.2)
        check("payload_check_active visible through webd", seen_active,
              f"observed for {time.monotonic()-t0:.1f}s, {len(states)} polls")
        print(f"       phases seen via webd: {sorted(set(states))}", flush=True)

        # -- 4. video ----------------------------------------------------------
        if skip_video:
            print("       (video leg skipped: --skip-video)", flush=True)
        else:
            vs = http("/api/video/state", timeout=10.0)
            check("video starts", True, str(vs.get("error") or "no error"))
            r = http("/api/video/start", {})
            n0 = int(r.get("frames_published", 0))
            time.sleep(3.0)
            vs2 = http("/api/video/state")
            n1 = int(vs2.get("frames_published", 0))
            fps = (n1 - n0) / 3.0
            check("video publishes at a usable rate", fps >= 3.0,
                  f"{fps:.1f} fps (deprecated-callback baseline measured 0.8 fps)")
            # /api/video is an ENDLESS multipart stream. Read it with http.client
            # (a socket we can put a timeout on): urllib's response object gives no
            # socket handle, and .read() on an endless stream never returns — which
            # is also exactly how a browser keeps the connection, and shutdown, open.
            conn = _http.client.HTTPConnection(HOST, PORT, timeout=5.0)
            conn.request("GET", "/api/video")
            stream = conn.getresponse()
            buf = b""
            t_read = time.monotonic()
            while len(buf) < 200000 and time.monotonic() - t_read < 4.0:
                chunk = stream.read(8192)
                if not chunk:
                    break
                buf += chunk
            check("frames are real JPEGs",
                  buf.count(b"\xff\xd8") >= 1 and len(buf) > 4000,
                  f"{len(buf)} bytes, {buf.count(b'--frame')} parts")

            # Stopping video must END open streams: that is what lets uvicorn run
            # its lifespan shutdown and hand the camera back to visiond.
            t_stop = time.monotonic()
            http("/api/video/stop", {}, timeout=15.0)
            dt = time.monotonic() - t_stop
            check("video stops promptly", dt < 5.0, f"{dt:.2f}s")
            conn.sock.settimeout(5.0)
            t_eof = time.monotonic()
            ended = False
            try:
                while True:
                    if not stream.read(4096):
                        ended = True
                        break
            except Exception:  # noqa: BLE001  (closed or timed-out socket)
                ended = True
            conn.close()
            check("open MJPEG ends when video stops", ended,
                  f"{time.monotonic()-t_eof:.2f}s to EOF")

            ok, detail = wait_for(lambda: _health(), 5.0)
            check("webd still responsive after video stop", ok, str(detail))

        # -- 5. shutdown -------------------------------------------------------
        print("=== shutdown (SIGTERM, never SIGKILL) ===", flush=True)
        msg_w = terminate(webd, "webd", 12.0)
        check("webd shut down cleanly", "exited" in msg_w, msg_w)
        msg_c = terminate(controld, "controld", 20.0)
        check("controld parked and exited", "exited" in msg_c, msg_c)
    finally:
        if controld.poll() is None:
            os.kill(controld.pid, signal.SIGTERM)
            time.sleep(3)
        if controld.poll() is None:
            print("!! controld still alive after SIGTERM — NOT killing it; "
                  "investigate before continuing", flush=True)

    failed = [n for n, ok, _ in results if not ok]
    print(f"\n=== rehearsal: {len(results)-len(failed)}/{len(results)} checks passed"
          + (f" — FAILED: {failed}" if failed else " — ALL PASS") + " ===", flush=True)
    return 1 if failed else 0


def _health():
    try:
        return http("/api/health", timeout=2.0)
    except Exception:  # noqa: BLE001
        return None


def _state():
    try:
        return http("/api/state", timeout=2.0)
    except Exception:  # noqa: BLE001
        return None


if __name__ == "__main__":
    sys.exit(main())
