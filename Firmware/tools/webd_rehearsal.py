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

  6. (§54.5, --load) The dashboard under load: N websocket clients at 15 Hz
     plus three MJPEG viewers. Measured from controld's own 1 Hz `loop:` line,
     not from the network side: the loop must keep its cadence, no client may be
     starved, /api/state must stay responsive, and the publish rate must not
     multiply with viewers (§42.3 shared capture). Then webd is stopped WITH
     clients still attached, because that is what can strand the camera.

Usage: python3 tools/webd_rehearsal.py [--skip-video] [--load N] [--load-seconds S]
       (--load 0 disables the load leg)
Requires ./build/control/controld built, and the IMX500 free for step 4/6.
"""
from __future__ import annotations

import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
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


# ------------------------------------------------------- §54.5 load leg (P12)
#
# The queue's P12 ask is "the dashboard under load must not disturb control".
# Doing that live costs a supervised window and a motor; the part that actually
# threatens the control loop is measurable offline: N browsers pulling 15 Hz
# telemetry, plus an MJPEG preview, all served by webd, while controld keeps its
# 200 Hz loop. controld reports its OWN timing (the 1 Hz `loop:` line, §6.3),
# which is the only instrument that answers the question — a slow HTTP response
# says nothing about the loop, and a fast one proves nothing either.

attached_note = ""          # set by run_load_leg; drives the shutdown banner

LOOP_RE = re.compile(r"loop: target=(\d+) Hz p50=([\d.]+) p95=([\d.]+) "
                     r"p99=([\d.]+) worst=([\d.]+) ms \(n=(\d+)\)")
SLOW_RE = re.compile(r"SLOW CYCLE ([\d.]+) ms")


def _arg_int(name: str, default: int) -> int:
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            try:
                return int(sys.argv[i + 1])
            except ValueError:
                return default
        if a.startswith(name + "="):
            try:
                return int(a.split("=", 1)[1])
            except ValueError:
                return default
    return default


def _daemon_stats() -> dict:
    """controld's own latest loop timing + cumulative SLOW CYCLE count."""
    path = os.path.join(PROBE, "rehearsal-controld.log")
    out = {"p50": None, "p99": None, "worst": None, "n": 0, "slow": 0,
           "reports": 0}
    try:
        with open(path, errors="replace") as f:
            txt = f.read()
    except OSError:
        return out
    hits = LOOP_RE.findall(txt)
    if hits:
        t = hits[-1]
        out.update(p50=float(t[1]), p99=float(t[3]), worst=float(t[4]),
                   n=int(t[5]))
    out["slow"] = len(SLOW_RE.findall(txt))
    # `n=` is the sample count INSIDE the timing window and it SATURATES (the
    # statistics are kept in a 4096-sample ring: measured n=4096 and
    # identical p50/worst across successive reads). Liveness therefore counts
    # REPORTS — one line per second — which cannot saturate.
    out["reports"] = len(hits)
    out["worst_seen"] = max((float(h[4]) for h in hits), default=0.0)
    return out


def _ws_leg(n_ws: int, seconds: float, n_sticky: int, out: list,
            lock: threading.Lock) -> None:
    """Run n_ws telemetry clients for `seconds`, and leave n_sticky attached.

    The sticky ones are the point of the second half of the leg: a browser that
    is still reading when webd is stopped is exactly what can keep uvicorn in
    "waiting for connections to close" past the lifespan shutdown that hands the
    camera back (§42.3).
    """
    import asyncio

    try:
        import websockets
    except ImportError:
        with lock:
            out.append({"frames": 0, "error": "websockets not installed"})
        return

    async def client(sticky: bool) -> None:
        rec = {"frames": 0, "max_gap": 0.0, "error": ""}
        url = f"ws://{HOST}:{PORT}/ws"
        try:
            async with websockets.connect(url, max_queue=256) as ws:
                prev = None
                deadline = time.monotonic() + seconds
                while sticky or time.monotonic() < deadline:
                    timeout = 2.0 if not sticky else 120.0
                    try:
                        await asyncio.wait_for(ws.recv(), timeout=timeout)
                    except asyncio.TimeoutError:
                        if not sticky:
                            rec["error"] = "recv timeout"
                        break
                    now = time.monotonic()
                    if prev is not None:
                        rec["max_gap"] = max(rec["max_gap"], now - prev)
                    prev = now
                    rec["frames"] += 1
        except Exception as e:  # noqa: BLE001
            rec["error"] = repr(e)[:80]     # a closed socket at shutdown is fine
        if not sticky:
            with lock:
                out.append(rec)

    async def main_coro() -> None:
        for _ in range(n_sticky):
            asyncio.get_running_loop().create_task(client(True))
        await asyncio.gather(*(client(False) for _ in range(n_ws)),
                             return_exceptions=True)

    try:
        asyncio.run(main_coro())
    except Exception as e:  # noqa: BLE001
        with lock:
            out.append({"frames": 0, "error": repr(e)[:80]})


def _video_reader(stop: threading.Event, out: list, lock: threading.Lock) -> None:
    conn = _http.client.HTTPConnection(HOST, PORT, timeout=6.0)
    rec = {"jpegs": 0, "bytes": 0, "error": ""}
    try:
        conn.request("GET", "/api/video")
        stream = conn.getresponse()
        while not stop.is_set():
            chunk = stream.read(8192)
            if not chunk:
                break
            rec["bytes"] += len(chunk)
            rec["jpegs"] += chunk.count(b"\xff\xd8")
    except Exception as e:  # noqa: BLE001
        rec["error"] = repr(e)[:60]
    finally:
        try:
            conn.close()
        except Exception:  # noqa: BLE001
            pass
        with lock:
            out.append(rec)


def run_load_leg(n_ws: int, seconds: float, video_ok: bool) -> None:
    global attached_note
    print(f"=== §54.5 load leg: {n_ws} dashboard clients"
          + (" + 3 MJPEG viewers" if video_ok else " (video skipped)")
          + f" for {seconds:.0f}s ===", flush=True)
    base = _daemon_stats()
    if base["p50"] is None:
        print("       (no baseline loop line yet; using in-leg numbers only)",
              flush=True)

    ws_out: list = []
    ws_lock = threading.Lock()
    threading.Thread(target=_ws_leg, args=(n_ws, seconds, 2, ws_out, ws_lock),
                     daemon=True).start()
    ok, detail = wait_for(
        lambda: int((http("/api/health", timeout=3.0) or {}).get(
            "browser_clients", 0)) >= n_ws, 10.0, 0.2)
    check("every dashboard client is attached", ok,
          f"browser_clients={str(detail)[:40]}")

    vid_out: list = []
    vid_lock = threading.Lock()
    stop_ev = threading.Event()
    fps_req, n0 = 10.0, 0
    if video_ok:
        r = http("/api/video/start", {"width": 640, "height": 480, "fps": 10},
                 timeout=20.0)
        fps_req = float(r.get("fps") or 10.0)
        n0 = int(r.get("frames_published", 0))
        for _ in range(3):
            threading.Thread(target=_video_reader,
                             args=(stop_ev, vid_out, vid_lock),
                             daemon=True).start()
        print(f"       video on for 3 viewers ({fps_req:g} fps requested)",
              flush=True)

    # Sample the operator-facing path under load: /api/state is what the
    # dashboard polls, so its latency is what the operator perceives as lag.
    lat: list[float] = []
    failures = 0
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        st = _state()
        if st is None:
            failures += 1
        else:
            lat.append(time.monotonic() - t0)
        time.sleep(0.1)

    if video_ok:
        stop_ev.set()
        http("/api/video/stop", {}, timeout=15.0)
        time.sleep(0.6)
        vs = http("/api/video/state", timeout=5.0)
        n1 = int(vs.get("frames_published", 0))

    during = _daemon_stats()
    with ws_lock:
        recs = list(ws_out)
    with vid_lock:
        vrecs = list(vid_out)

    frames = [r["frames"] for r in recs] or [0]
    check("every client received telemetry (none starved)",
          all(f > 0 for f in frames), f"frames/client={frames}")
    check("no client was starved by the others (fairness)",
          max(frames) == 0 or min(frames) >= 0.5 * max(frames),
          f"min={min(frames)} max={max(frames)} over ~{seconds:.0f}s")
    worst_gap = max((r.get("max_gap", 0.0) for r in recs), default=99.0)
    check("no telemetry stall longer than 1 s on any client", worst_gap < 1.0,
          f"worst gap {worst_gap:.2f}s")
    lat.sort()
    p95 = lat[int(len(lat) * 0.95) - 1] if lat else 99.0
    check("/api/state stays responsive under load",
          p95 < 0.25 and failures == 0,
          f"p95 {p95*1000:.0f} ms over {len(lat)} samples, {failures} failures")

    if during["p50"] is not None:
        starved = (during["worst"] is None or during["worst"] >= 10.0
                   or during["p50"] > (base["p50"] or 0.0) * 4.0 + 1.0
                   or during["reports"] <= base["reports"])
        check("control loop kept its cadence under web load", not starved,
              f"before p50={base['p50']} worst={base['worst']} "
              f"reports={base['reports']}; during p50={during['p50']} "
              f"worst={during['worst']} reports={during['reports']}; "
              f"slow_cycles={during['slow'] - base['slow']}")
        print(f"       loop under load: p50={during['p50']:.3f} "
              f"p99={during['p99']:.3f} worst={during['worst']:.3f} ms over "
              f"{during['n']} samples", flush=True)
    else:
        check("controld published loop statistics", False,
              "no 'loop:' line in the log — the leg proves nothing about timing")

    if video_ok:
        got = [r["jpegs"] for r in vrecs] or [0]
        check("every MJPEG viewer got frames", all(g >= 2 for g in got),
              f"jpegs/viewer={got} errors={[r['error'] for r in vrecs if r['error']]}")
        # The claim in §42.3 is that viewers SHARE one capture. If the publisher
        # rate scaled with clients, three viewers would triple the camera work.
        elapsed = max(1.0, seconds)
        pub_fps = (n1 - n0) / elapsed
        check("viewers share one capture (publish rate did not multiply)",
              pub_fps <= fps_req * 1.8 + 2.0,
              f"{pub_fps:.1f} fps published for 3 viewers at {fps_req:g} fps")

    # Leave the sticky clients (and, if we can, one live stream) attached so the
    # shutdown below is exercised under exactly the condition that bites.
    if video_ok:
        http("/api/video/start", {"width": 640, "height": 480, "fps": 10},
             timeout=20.0)
        threading.Thread(target=_video_reader,
                         args=(threading.Event(), [], threading.Lock()),
                         daemon=True).start()
        attached_note = "2 telemetry clients and 1 MJPEG stream still attached"
    else:
        attached_note = "2 telemetry clients still attached"


def main() -> int:
    skip_video = "--skip-video" in sys.argv
    load_ws = _arg_int("--load", 6)          # 0 disables the load leg
    load_seconds = _arg_int("--load-seconds", 12)
    print(f"=== P12 offline rehearsal (load leg: {load_ws} clients x "
          f"{load_seconds}s; video: {'on' if not skip_video else 'skipped'}) ===",
          flush=True)
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
            check("video state readable before start",
                  isinstance(vs, dict) and "running" in vs,
                  f"running={vs.get('running')} error={vs.get('error')!r}")
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

        # -- §54.5 load leg: N dashboards + video while the daemon keeps its loop
        if load_ws > 0:
            run_load_leg(load_ws, load_seconds, not skip_video)

        # -- 5. shutdown -------------------------------------------------------
        # With clients STILL attached on purpose: the P12 risk is not the steady
        # state, it is uvicorn's lifespan shutdown sitting in "waiting for
        # connections to close" while a browser holds the endless MJPEG stream —
        # which is the same mechanism that would keep the camera away from
        # visiond after a real restart.
        if attached_note:
            print(f"=== shutting down with {attached_note} ===", flush=True)
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
