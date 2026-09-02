# P7 Vision Readiness — Investigation & Conclusions (2026-09-02/03)

**Status: P7 (live camera/vision verification) is BLOCKED on a platform gap,
not on our code.** All station-side facts below were verified empirically on
rpi-turret. This note gives the next engineer a decision-ready picture.

## 1. What was verified live

| Fact | Evidence |
|---|---|
| IMX500 sensor present and streaming | `Picamera2()` preview config 1920x1080 XRGB8888 starts cleanly; sensor picks 2028x1520 RAW (`/tmp/probe_cam.log`) |
| picamera2 importable on **system** python3 only | venv lacks `libcamera` (`ModuleNotFoundError`); webd runs on system python for the same reason |
| Camera is EXCLUSIVE | only one consumer at a time — webd must be stopped before any vision test (`pkill -f web.webd.app`), and restarted afterwards |
| Orientation correction path is known-good | webd uses `OTA_VIDEO_ORIENTATION=rotate_180` (camera mounted upside-down); `vision/frame_source.py` applies the same `image_corrections` path to detector boxes — geometry stays correct once detection exists |

## 2. What is missing for live detection (the blocker)

`vision/frame_source.py::Picamera2FrameSource` was written against an IMX500
AI API shape (`picam.configure(rpk_cfg, detect_objects=True)`,
`frame.metadata["Objects"]`) that **does not exist on this system**:

* `picamera2` **0.3.37** — no `detect_objects` attribute on `Picamera2`
  (AttributeError on every frame, `/tmp/probe_cam.log`); no
  `picamera2/postprocessing/` directory (`/tmp/probe_api.log`).
* No detector model assets: `find / -name "*.rpk"` → none.
* No Hailo/Dataforensics SDK: `dnf`, `hailo`, `hilortc` all MISSING.
  The IMX500 "compile RPK → run detection" pipeline needs this stack
  (usually a newer `python3-picamera2` + libcamera AI post-processor).
* No internet on the station to fetch it from my tools; `apt-cache policy`
  returned nothing on this mirror — package availability unknown (run
  `sudo apt update && apt-cache policy python3-picamera2 rpicam-apps`).

**Conclusion:** live NN detection cannot run until the Raspberry Pi AI-camera
stack is installed/upgraded (apt) or RPK tooling + model are brought in on
removable media. Until then `visiond --real` cannot start, and P8 live
tracking has no real detector.

## 3. Options (ranked for the next session)

1. **Platform upgrade (proper):** `apt update` → check `python3-picamera2`
   (needs ≥ 2.3-era with `postprocessing`/AI support) + `rpicam-apps` +
   IMX500 firmware packages; then `picam2.postprocessing`/"ai" example from
   the new docs becomes the reference and `Picamera2FrameSource` gets adapted
   to that API. CAUTION: webd currently works on the *installed* picamera2 —
   verify webd video after any upgrade; apt-managed, so `apt install` (NOT
   pip --force) and keep rollback notes.
2. **Bring an RPK + SDK on a USB stick** (needs a machine with internet:
   Dataforensics `dnf-torch`/`dnf` or the Raspberry Pi `rpi-camera-examples`
   YOLO11n RPK JSON pair — note YOLO11n RPK is AGPL, license review before
   distribution, per queue §P7).
3. **Bridge detector for P8 bring-up only:** implement a classical
   `FrameSource`-conforming detector (e.g. motion/blob centroid with
   `class_id=1` mapping) selectable via `--detector simple`. This exercises
   the ENTIRE downstream chain (IPC 58-byte protocol, selector, tracking FSM
   live, §34/§35 states) on real glass without the NN stack; swap the RPK
   detector in when (1)/(2) land. Clearly marked non-production.

## 4. Independent P8 prerequisite (can proceed TODAY, offline)

The `controld` **vision ingest wiring (Part 2, S1)** is unblocked work:
a `SOCK_SEQPACKET` client in controld decoding the 58-byte
`TargetMeasurement` (`control/src/tracking/target_measurement.hpp`,
encoder/decoder already exist per `vision/protocol.py`) →
`ControlLoop::feed_measurement`, `tracking:` config block + intrinsics file →
`TrackingController::Config`, `enable_tracking()` behind the homing gate.
Pure C++ + unit-testable with the synthetic source; needed regardless of
which detector option wins.

## 5. Repro commands

```bash
pkill -f web.webd.app                        # free the sensor
python3 /tmp/probe_cam.py                    # stream + detect_objects probe
python3 /tmp/probe_api.py                    # picamera2 API surface survey
# restart the web UI afterwards:
cd Firmware && env OTA_WEB_HOST=0.0.0.0 OTA_WEB_PORT=8080 \
  OTA_VIDEO_ORIENTATION=rotate_180 setsid nohup python3 -m web.webd.app \
  > /tmp/webd.log 2>&1 &
```
