# Station runbook — one-off deployment of the OpenAutoTurret stack

**Audience:** an engineer or agent deploying this station from a cold start, with motor authority already granted.
Every command below was executed on this station and the expected output is what was actually observed, not what the
code suggests it should be. Where something is *not* verified, the document says so instead of implying otherwise.

Read §1 and §7 before typing anything. §7 is a list of ways this station has already cost someone an afternoon.

---

## 1. Ground rules that are not negotiable

| Rule | Why it exists |
|---|---|
| **Never put a `pkill` and a launch of the same component in one command line.** | `pkill -f "vision[.]visiond"` also matches the *launch* string later on the same line and SIGTERMs your own shell. Killed my shell twice. |
| **Use `pkill -x controld`** (exact name) for controld. | `-x` cannot match a script's own command line. |
| **A `ctest` pass proves nothing until the test binary is known-fresh.** | A green `57/57` was reported *in the same command* that reported a compile error, because the binary was never rebuilt. Touch the changed file if the build is suspiciously quiet. |
| **Never grep a call-site list with `head -N` and conclude.** | `head -12` on a list of 21 backend call sites made me attribute normal behaviour to calibration code and retract a conclusion a round later. |
| **`/api/command` returning `ok:true, verdict:"submitted"` is NOT acceptance.** | Validation rejects happen after the ack. The truth appears in `cmd_ack_reason`. See §7.4. |
| **§24 (16 items) and §110 (30 items) are operator-signed.** Never self-sign, never present a number as accepted. |
| **Retract anything reproduction contradicts**, in writing, in the commit. |

---

## 2. Station facts

| | |
|---|---|
| Host | `rpi-turret`, aarch64, root |
| Dev checkout | `/home/eamars/workspace/OpenAutoTurret/Firmware` |
| Installed prefix | `/opt/open_auto_turret` (see `systemd/turret-control.service` `ExecStart`) |
| controld | compiled C++ — `build/control/controld config/turret.yaml` |
| webd | **`/usr/bin/python3 -m web.webd.app`** |
| visiond | **`/usr/bin/python3 -m vision.visiond …`** — the venv has **no Picamera2 at all**, so visiond under the venv cannot see the camera |
| Tests | `.venv/bin/python -m pytest web vision -q` → **380 passed**; `ctest --test-dir build` → **57/57** |
| Camera | IMX500, **one owner only** — see §5 |
| IMU | **none installed** — no inertial data exists |

---

## 3. Build and verify

```bash
cd /home/eamars/workspace/OpenAutoTurret/Firmware
cmake --build build --target controld test_control_loop -j"$(nproc)"   # expect: Linking CXX executable …
ctest --test-dir build            # expect: 100% tests passed … out of 57
/home/eamars/workspace/OpenAutoTurret/.venv/bin/python -m pytest web vision -q   # expect: 380 passed
```

`py_compile` passing is **not** a check: it did not find a missing `import os`, which only appeared at runtime as 15
test failures.

---

## 4. Runtime start order

Dependencies: **controld binds the vision IPC socket**, so start it first; visiond and webd connect to it.

```bash
cd /home/eamars/workspace/OpenAutoTurret/Firmware

# 1. control daemon  (OTA_BLACKBOX_DIR is REQUIRED for the §80 safety artifacts to exist at all)
OTA_BLACKBOX_DIR=/var/lib/ota/blackbox setsid nohup ./build/control/controld config/turret.yaml \
  > /tmp/controld.log 2>&1 < /dev/null &

# 2. vision, real IMX500, with the frame tap (§5) so the preview can share the camera
OTA_VISION_FRAME_TAP=/tmp/ota_vision_frame.jpg setsid nohup /usr/bin/python3 -m vision.visiond \
  --real --detector simple --socket /tmp/ota_vision.sock > /tmp/visiond.log 2>&1 < /dev/null &

# 3. web daemon — the SAME tap path, or the pane will try to open the camera and fail (§5)
OTA_BLACKBOX_DIR=/var/lib/ota/blackbox OTA_VISION_FRAME_TAP=/tmp/ota_vision_frame.jpg \
  OTA_VIDEO_WIDTH=1920 OTA_VIDEO_HEIGHT=1080 OTA_VIDEO_FPS=15 \
  setsid nohup /usr/bin/python3 -m web.webd.app > /tmp/webd.log 2>&1 < /dev/null &
```

### 4.1 After ANY controld restart, home before you move

`at_ready` is **false** after a restart — homing state is not restored from the drives. It stays false indefinitely;
waiting does not help.

```bash
curl -s -m 10 -X POST -H "Content-Type: application/json" \
  -d '{"command":"start_homing","arg":""}' http://127.0.0.1:8080/api/command
```
Expect `"ok":true` and, when finished, `at_ready: true`. Homing is real motion to the limits — expect that, and do not
run it with people or objects in the sweep.

### 4.2 Verify the whole stack in one look

```bash
curl -s -m 5 http://127.0.0.1:8080/api/state | python3 -c "import json,sys; d=json.load(sys.stdin);
print(d.get('operating_mode'),d.get('mode_phase'),d.get('safety_action'),'ready',d.get('at_ready'),
      '| camera_fps',d.get('camera_fps'),'| track_sets',d.get('vision_track_sets'))"
```
Healthy = `… ALLOW ready True | camera_fps ~15 | track_sets` climbing. `camera_fps 0` means vision is not running.

---

## 5. Dual feed: preview **and** detector at the same time

The IMX500 has one owner. Before this existed, starting the pane while vision ran failed with the misleading
`RuntimeError: Camera __init__ sequence did not complete`.

Now: **visiond taps the array it already captured** to a JPEG (`vision/frame_tap.py`), and webd serves that instead of
opening the camera.

* Set `OTA_VISION_FRAME_TAP` to the **same path in both processes.** Different paths = the pane opens the camera =
  visiond cannot start.
* The file must be **fresher than 2 s** when the pane starts, or webd falls back to the camera.
* `OTA_VISION_FRAME_TAP_FPS` (default **12**) is the cost knob: measured on this station, tapping at 12 fps took the
  detector from **15.13 → 9.55 fps**, because the encode runs on the vision thread. Use 6–8 when detection rate matters
  more than preview smoothness.

```bash
curl -s -m 25 -X POST -H "Content-Type: application/json" \
  -d '{"width":1920,"height":1080,"fps":15,"quality":80}' http://127.0.0.1:8080/api/video/start
curl -s -m 5 http://127.0.0.1:8080/api/video/state | python3 -c "import json,sys; d=json.load(sys.stdin);
print('camera',d['camera'],'fps',round(d['fps_published'],1),'err',repr(d['error']))"
# expect: camera vision-tap fps ~9-10 err ''
```

The MJPEG endpoint is **`/api/video`** (`boundary=frame`), *not* `/api/video/stream`. Frame-count proof:

```bash
timeout 4 curl -s -N http://127.0.0.1:8080/api/video -o /tmp/raw.bin
python3 -c "d=open('/tmp/raw.bin','rb').read(); print('JPEG frames:', d.count(b'\xff\xd8'))"   # expect ~38 in 4 s
```

**Installed deployment:** neither `systemd/turret-vision.service` nor `systemd/turret-web.service` sets
`OTA_VISION_FRAME_TAP` yet. Add it to **both** (identical value) or the installed station keeps the old either/or
behaviour.

---

## 6. Config keys added this session — and where they do NOT go

`auto_track.deadband_deg` / `deadband_release_deg` (item 3), `auto_track.roam_on_loss_ms` / `track_on_acquire_ms`
(item 4), `auto_track.position_lead_s` (item 2). **All default to off/0** — a station that does not name them behaves as
before they existed.

⚠️ **There is no live `auto_track:` node in `config/turret.yaml`.** The block that documents these keys is
*commented documentation*. Parsed top-level nodes are: `axes, camera, can, control, homing, homing_plan, installation,
motors, payload, safety, schema_version, shutdown, tracking`. Uncommenting a key inside that block yields
`YAML parse error … end of map not found` and **controld then refuses to start** — which is what happened during the
first lead trial, after the working controld had already been killed.

To enable a key: add a real `auto_track:` mapping where `turret_config.cpp:423` (`fetch(n, "auto_track")`) will find
it, then **start controld and confirm it reaches supervisory Ready.** A file that parses in Python is not proof
controld accepts it. Because every `auto_track` key is currently a default, `coast_ms` / `lost_hold_ms` /
`reacquire_window_ms` are also defaults on this station — not tuned values.

---

## 7. Traps, each paid for in practice

1. **Self-`pkill`.** Kill and launch never share a command line; use `pkill -x controld`.
2. **Camera single-owner.** Preview vs detector. Solved by §5; the error message does not hint at the real cause.
3. **`at_ready` false after restart** until `start_homing` (§4.1).
4. **`/api/command` lies by omission.** `select_target 0` returned `ok:true / submitted`; `cmd_ack_reason` said
   `target id must be a positive label number`. With a valid label it said **`person #1 is LOST, not CONFIRMED`** —
   which is why AUTO_TRACK never moves on the `simple` detector: blob tracks never reach `CONFIRMED`, so selection is
   always refused, so the intent stays `hold / "select a target"`. Always read `cmd_ack_reason`.
5. **Stale test binaries.** Verify the rebuild, then trust `ctest`.
6. **Truncated greps.** See §1.
7. **`webd` serves cached state when controld's IPC is down** — plausible-looking `safety_action`/`at_ready` while
   commands answer `controld not connected`. Trust the command verdict, not the snapshot.
8. **`FrameRate` is not a valid control** for a bare picamera2 video configuration (advertised controls are
   `FrameDurationLimits`, `NoiseReductionMode`). Writing it fails silently — and a `try/except Exception: pass`
   around it once turned a dead change into a reported success. Sensor modes here: `(2028,1520) @ 30 fps`,
   `(4056,3040) @ 10 fps`.
9. **`install_station.py` does not set `OTA_BLACKBOX_DIR`**, and `systemd/turret-web.service` carries it commented
   (line 48). Without it the station writes **no §80 artifacts**, so the evidence the sign-off process depends on
   silently does not exist. Set it explicitly in both units and in any hand-run launch.
10. **`docs/` has no other deployment runbook.** `operator_status_v3_2_2026-09-05_r67.md` is a status report, not a
    procedure.

---

## 8. Known defects, open at the time of writing (not fixed, do not assume they are)

1. **HUD prediction is untethered from the box.** `control_loop.cpp:1649-1650` fills
   `predicted_target_{az,el}_world_rad` from `tracking_` — the **v1** estimator — not from the v3 selection. Observed in
   one sample: `predicted_target_los_valid: true` with `selected_uuid_valid: false`, `el = -27.3974` in a `_rad` field,
   and a separate `prediction.valid: false`. Fix direction: publish prediction only when a uuid is selected, and only
   from the estimator that produced it.
2. **`track_state` reports `tracking` / `coasting` while `confidence_band` is `INVALID`** and nothing is selected. The
   neighbouring fields are honest; the label is not.
3. **The `simple` detector stamps `class_name='person'`** on threshold blobs at conf ≈0.31–0.36. Non-production by
   design, but it is a class label on a threshold and must never be used as evidence.
4. **`/api/command` submit-ack masking** (§7.4).
5. **Item 4's watcher has no unit test** and no measured latency; `evaluate_auto_switch()` is implemented, default-off,
   unaccepted.
6. **`position_lead` has no after-measurement.** Before-baseline is fixed at **p50 3.628° / p95 4.274° at 10.00 °/s**
   (PROGRESS round 18); the trial script `scripts/lead_trial.sh` is self-reverting and refuses to measure unless
   AUTO_ROAM is actually entered.

---

## 9. How to leave the station

Say which mode it is in and prove it with the §4.2 one-liner. Do not leave a component running that you did not start,
and do not leave `OTA_VISION_FRAME_TAP` set in one process only. At the time of writing: controld `AUTO_TRACK /
WAIT_TARGET / ALLOW`, visiond running with the tap, webd running with the tap, **preview pane ON and serving the tap**,
all three default-off keys still unset.
