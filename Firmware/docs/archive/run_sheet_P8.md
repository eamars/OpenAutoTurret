# Run sheet — P8 live tracking + P3/P4 stall re-check (supervised window)

Station `rpi-turret` (Pi 5) · **motors move** · one person at the E-stop, one at
the keyboard · expected wall time 35–45 min (cold homing alone is ~2.5 min).
Queue rows covered: **P8** (steps 1–6; step 7 needs a separate GO), and the
**P3/P4 stall re-check** using the loop-attribution logging added 2026-09-03.

Detector for this window is the classical bridge (`--detector simple`), by
decision 2026-09-03: the Pi 5 has no RPK/Hailo AI stack, so P7 stays open as
"no RPK model inference" and P8 is flown with a moving-blob target. That is
enough to prove the *control* path (acquisition, LOS follow, loss → brake →
ready-hold, envelope authority); it is not evidence about detector quality.

---

## 0. Hard rules for this window

* Currents stay **≤ 10 A** on both axes (the config is already inside that; the
  payload check uses 5 A). Do not raise anything to "make it track better".
* `tracking.enabled` stays **false** in `config/turret.yaml`. Auto-enable would
  make the station follow pixels the instant homing finished. The operator path
  is the `start_tracking` developer command (§42.2), gated on homing (§38.1).
* Stop with `request_park` and then `systemctl stop` (SIGTERM). **Never SIGKILL
  an energized daemon** — that leaves the drives holding without a supervisor.
* If anything looks wrong: E-stop first, questions after. Then
  `systemctl status turret-control turret-vision` + `journalctl -n 200`.

## 1. Pre-flight (before anything is energized)

| # | Check | Command / expected |
|---|---|---|
| 1 | Tree + build current, tests green | `git pull && ninja -C build && ctest --test-dir build` → **42/42**; `.venv/bin/python -m pytest -q --ignore=legacy --ignore=Firmware/legacy` → **139 passed, 0 skipped** |
| 2 | Units installed and readable | `systemd-analyze verify systemd/turret-*.service` (only acceptable complaint: `/opt/open_auto_turret` not installed → run from the repo path) |
| 3 | Interpreter | vision + web units use `/usr/bin/python3` — the venv has no `picamera2` and no `uvicorn` |
| 4 | Currents / envelope | `grep -nE "limit_cur_a|check_current_a|soft_margin_deg|deg_s" config/turret.yaml` — per-axis `limit_cur_a` 3.0 pitch / 1.0 yaw, `check_current_a: 5.0`, `soft_margin_deg: 5`, track 30 deg/s / search 10 deg/s. Nothing above 10 A, no margin widened |
| 5 | Search stays off | `grep -n -A3 "^tracking:" config/turret.yaml` → `search_enabled_by_default: false`, `target_lost_behavior: hold`, track 30 deg/s, search 10 deg/s |
| 6 | Camera is free (exclusive device) | `sudo fuser -v /dev/video1` → stop `turret-web` for this window: its live view and `visiond` cannot share the IMX500 |
| 7 | Scene is **lit** and the target is real | The last session measured **mean luma 1.5/255**. The bridge detector reports the largest *moving* blob: switch the room lights on and plan to move the target (a person walking slowly, or a cardboard panel) with visible contrast against the background |
| 8 | Witness supervisor running (for the P3/P4 leg) | `journalctl -u turret-can-supervisor -f` in a second terminal, per `docs/archive/HANDOFF_2026-09-03.md` (it watches `can0`, the MCP2515 witness HAT) |
| 9 | Dry run done (5 min, no motors) | see §2 |

## 2. Dry run — same commands, `--sim`, nothing armed

```bash
cd Firmware
OTA_VISION_SOCKET=$PWD/build/probe/ota_p8.sock \
OTA_WEB_SOCKET=$PWD/build/probe/controld-p8.sock \
./build/control/controld config/turret_sim.yaml --sim
# second terminal — send the real commands through the real gate:
python3 tools/station_ipc.py --socket "$PWD/build/probe/controld-p8.sock" state
python3 tools/station_ipc.py --socket "$PWD/build/probe/controld-p8.sock" cmd start_tracking
```
Measured on 2026-09-03, exactly this pair of commands:

```
state           → {'phase': 'homing', 'at_ready': False, ..., 'safety_action': 'ALLOW', 'vision_measurement_age_ms': -1}
cmd start_tracking → {"ok": false, "error": "not homed (position validity unknown)"}
```

That rejection **is** P8 step 1's evidence (the §42.2 gate answers synchronously,
before the daemon's §38.1 check is even reached, so the daemon logs nothing — the
client answer is the evidence). `state` also proves `at_ready` is visible from the
operator tool. Let homing finish in sim and `start_tracking` again to feel the
accepted path, then `Ctrl-C` the sim daemon.

## 3. Live sequence

Open two terminals: `journalctl -u turret-control -f` (watch) and the command
shell. Dashboard: `http://<pi>:8080` if `turret-web` is up (it is not, per §1.6 —
`tools/station_ipc.py telemetry` is the substitute).

| # | Action | What must happen (record it) |
|---|---|---|
| 3.1 | `sudo systemctl start turret-control` | homing stages log; then **`homed + at ready pose; holding (Ctrl-C to park)``. `station_ipc.py state` → `phase=hold  at_ready=true` |
| 3.2 | **Idle stall baseline (P3/P4 leg)** — do nothing for 60 s | the 1 Hz `loop: target=200 Hz p50=… p95=… p99=… worst=… ms` line. Record `worst`. `grep -c "SLOW CYCLE"` in the last minute = the idle slow-cycle count (want 0) |
| 3.3 | `sudo systemctl start turret-vision` (already `--detector simple --orientation rotate_180`) | controld log gains `vision: N frames (0 dropped, seq S, age A ms) \| tracking=off state=none conf=0.00`; `age` must stay **< 100 ms** (§34 staleness → Brake) |
| 3.4 | **P8 step 1** — on the next boot, send `cmd start_tracking` while `phase=homing` (or reuse §2's sim evidence) | rejected: `not homed (position validity unknown)`; nothing moves; `at_ready=false` |
| 3.5 | **P8 step 2** — `sudo python3 tools/station_ipc.py cmd start_tracking` with a person standing still in frame | `tracking_active: true`; `track_state` → … → `tracking`; the axes converge and settle; record the LOS error band (the `q_pitch/q_yaw` deltas in the 1 Hz line) for §55 |
| 3.6 | **P8 step 3** — target walks slowly across the field | the yaw follows at **≤ 30 deg/s** (§16, scaled by confidence). If it moves faster than that, or in the wrong direction: **abort** |
| 3.7 | **P8 step 4** — target leaves the frame | `tracking → coasting (≤200 ms) → brake_to_hold → target_lost → ready_hold`; confidence decays to 0; with `target_lost_behavior: hold` it stays at the ready pose |
| 3.8 | **P8 step 5** — `sudo python3 tools/station_ipc.py cmd enable_search` (no config edit), then have the person re-enter | sweep between the configured yaw limits with dwells; the log may say `search sweep clamped to [a, b] deg (logical) to stay inside the soft limits` — **record the clamp**, it is the envelope doing its job (§49); reacquisition when the person reappears |
| 3.9 | `cmd disable_search` then `cmd stop_tracking` | motion stops, `tracking_active: false`; `cmd hold` to sit on the ready pose |
| 3.10 | **Tracking-load stall re-check (P3/P4 leg)** — re-arm tracking for 60 s and watch the same `loop:` line | this is the flap question: if `worst=` stays ≈ 5 ms while the Brake/DERATE happens, the loop was **not** the cause (bus/supervisor side); if `SLOW CYCLE x.xx ms (phase=…, action=…)` lines appear with it, attribution is in the log with the phase *and* the safety action it produced. Save both lines |
| 3.11 | **P8 step 6** — envelope authority | hold the target near a soft limit: the reference must be clamped, the axis must stop short of the stop, no limit crossing in the log |
| 3.12 | `sudo python3 tools/station_ipc.py cmd request_park` → `sudo systemctl stop turret-vision turret-control` | park completes, both axes de-energize, daemons exit 0 (no SIGKILL) |

**Not in this sheet — P8 step 7 (fault mid-track) and P5:** fault injection needs
an explicit GO from you; it is deliberately not bundled into this window.

## 4. Abort criteria (any one)

* Motion in the unexpected direction, or faster than 30 deg/s.
* `phase=fault` or a `fault_reason` you did not cause.
* Repeated `SLOW CYCLE` above 20 ms, or a supervisor Brake you cannot explain
  from the `loop:` line.
* Any contact with a travel stop, unusual noise, or drive temperature climbing.
* `vision_measurement_age_ms` pinned near/above 100 ms while visiond is running.

Abort = E-stop → `request_park` if it still answers → `systemctl stop
turret-vision turret-control` → capture `journalctl -u turret-control -n 400`
and the supervisor log **before** touching anything else.

## 5. Paste-back block (fill in at the station, then into the queue rows)

```
P8 + stall re-check  date: ______  operator: ______  lights: on/off
build commit: ____________            homing time to at_ready: ____ s
3.2 idle loop worst: ____ ms   SLOW CYCLE count (60 s idle): ____
3.3 vision age: ____ ms   dropped frames: ____   detector blobs seen: ____
3.5 acquisition: pass/fail   LOS band: ±____ deg
3.6 follow: pass/fail   felt speed: ____ deg/s (cap 30)
3.7 loss path states seen: ____________________________________________
3.8 search clamp line: ________________________________________________
3.10 loop worst under tracking: ____ ms   SLOW CYCLE count: ____
     → attribution: loop / bus / unexplained
3.11 envelope clamp at limit: pass/fail
3.12 park + clean shutdown: pass/fail   controld exit: ____
notes: ________________________________________________________________
```

Paste into `docs/archive/post_homing_test_queue.md` → P8 steps 1–6 + pass criteria, and
into P3/P4 where the ~98 ms Brake flap was recorded (the attribution line is the
new evidence). If 3.6–3.11 all pass with no limit crossing, P8 flips to verified;
P7 remains open until there is an RPK/accelerator path.
