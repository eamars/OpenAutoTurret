# OpenAutoTurret — Implementation Progress Tracker

**Tracks implementation of** [`Firmware/docs/open_auto_turret_software_control_architecture_v1.md`](Firmware/docs/open_auto_turret_software_control_architecture_v1.md) (v1.0 baseline).

**Location rule:** all production software lives under `Firmware/` (`control/`, `vision/`, `web/`, `config/`, `calibration/`, `tools/`). The architecture doc's §50 tree is applied relative to `Firmware/`.

## Status legend

`[ ]` not started · `[~]` in progress · `[x]` done · `[!]` blocked

## Phase overview (doc §56)

| Phase | Name | Status | Notes |
|---|---|:---:|---|
| 0 | Preserve/instrument POC | [x] | POC archived under `Firmware/legacy/` (commit d9e79cc); reference doc `CyberGear_AI_Reference.md` committed (b6e942b) |
| 1 | Production CAN/motor layer | [~] | See checklist below |
| 2 | Homing and safety foundation | [~] | Config, logical coords, contact detector, homing FSM, full-axis (§23), multi-axis plan (§25), soft-limit envelope (§18), safety supervisor (§38/§39), boot FSM (§27), park (§33), and the `controld` daemon all done+tested; calibration persistence (§28/§41) pending |
| 3 | Trajectory generator | [x] | All done+tested: online jerk-limited generator, receding-horizon retarget (§17.3), stopping trajectories + stop-reachability (§17.2/§48, randomized tests), coupled collision envelope interface (§19, path validation) |
| 4 | Vision daemon (Python) | [x] | visiond (synthetic-safe + real guarded), target association (§12), UDS `SOCK_SEQPACKET` publish; 17 camera-free tests, no CAN/motor. Live IMX500 deferred to test queue |
| 5 | Geometry and estimator (C++) | [x] | camera model (§10), gimbal kinematics + configurable R_P_C (§9.2/§10.3), §11 history-interpolation alignment, alpha-beta estimator + prediction (§13); 6 new C++ test binaries incl. replay (§54.3) |
| 6 | Closed-loop tracking | [~] | LOS→joint (§14), tracking FSM (§34)+confidence (§35), reference manager (§16), search planner (§36/§49), telemetry (§6.3/§43) all done+tested; closed-loop integration verified on SimMotorBackend + synthetic vision (no CAN/motor). systemd units (§52) deferred |
| 7 | Installation orientation calibration | [ ] | |
| 8 | Web UI and diagnostics | [ ] | |
| 9 | Payload profiling/tuning | [x] | §44 battery (`turret-payload`, `--sim` rehearsal), schema-v1 store, §31.3 in-loop check + auto-verify (§27) + manual command, mismatch→derate (§31.4), telemetry+dashboard mirror; 5 new SimMotorBackend-only C++ binaries (38/38 ctest) + 3 webd tests (95/95 Python). No CAN/motor in any test |

**Session target (Appendix D first milestone):** boot → discover both motors → robust sensorless multi-axis homing → logical coordinates → safe jerk-limited moves → stopping envelope → safe park → timing/feedback logs. Vision may only generate motion targets after this is reliable.

## Phase 1 — Production CAN/motor layer

- [x] SocketCAN raw transport (RX thread, RAW filters, non-blocking TX, self-pipe shutdown) — `control/src/can/socketcan_bus.{hpp,cpp}`
- [x] CyberGear protocol encode/decode (discovery, feedback, enable/stop/set-zero, regs 0x7005..0x7020, MIT) — `control/src/can/cybergear_protocol.{hpp,cpp}`
- [x] Per-axis runtime state: latest feedback (SeqLock) + ~1 s history ring with interpolation — `control/src/can/cybergear_axis.hpp`
- [x] Common: types/state enums (App. A), monotonic time, SeqLock, `MotorStateHistory`, `TimingStats` — `control/src/common/`
- [x] `CyberGearSystem` implementation (`cybergear_system.cpp`): open/close, per-axis discovery, bounded register read, fire-and-forget TX, feedback fan-out
- [x] CMake build (`Firmware/control/CMakeLists.txt`, C++20, deps: yaml-cpp, spdlog, GTest for tests)
- [x] Unit tests: protocol encode/decode + bit fields, history interpolation, timing stats, seqlock (32 tests green)
- [ ] Motor health tracking (feedback freshness + fault flags present in `AxisLatest`; bus error counters not yet surfaced)
- [x] Probe/commissioning CLI (`turret-can`): discover, read regs, enable, live feedback, speed/position jog — deliverable "two motors controllable independently with high-rate telemetry"
- [x] Build + tests green on this machine (Pi 5 IS the target)

**Verified on live hardware (2026-09-01):**
- **Feedback model is request/response, not free-running.** The CyberGear emits NO
  periodic COMM_TYPE_2 by default (20 s of silence after enable; the 0x2004 `echoFreHz`
  config register is in the 0x2000 series, which comm 17/18 cannot reach — that path is
  the undocumented comm-19 parameter-table command). Instead, every control command
  (COMM_TYPE_1 MIT, COMM_TYPE_3 enable, COMM_TYPE_4 stop, COMM_TYPE_18 reg write) is
  answered by one COMM_TYPE_2 feedback frame (§25.1). The 200 Hz loop therefore sends
  MIT references and consumes the resulting feedback asynchronously (fire-and-forget TX +
  RX thread). Confirmed 1:1 (20 MIT → 20 feedback) at 200 Hz on both motors.
- **Feedback angle uses the ±12.5 rad mapping** (same as the MIT position command), not
  the manual's "±4π": decoding with ±12.5 tracks the 0x7019 mechPos read to <0.001 rad.
  `Ranges::kFbAngle*` set to ±12.5; encoded in `test_protocol.cpp`.
- **Stationary velocity feedback is quantization-noise dominated** (±~0.15 rad/s ≈ ±9°/s
  at a held position, position stable to 1 u16 count = 0.022°). Velocity-based "stopped"
  logic must use a threshold above ~0.2 rad/s. Position is accurate and stable.
- **comm 17/18 address only the 0x7005..0x7020 runtime table**; reads of the 0x2000/0x3000/
  0x1000 series return stale/garbage (last-value buffer), confirming they need comm 19.
- **Speed mode (run_mode=2 + SpdRef 0x700A) does NOT drive the loaded axis at the
  commanded rate** — with default gains (spd_kp=1, spd_ki=0.002) and limit_cur up to
  5 A, a 0.2 rad/s command produced ~0.002 rad/s load motion (~1% of command). Position
  mode (run_mode=1 + loc_ref 0x7016) drives reliably (swept the full −37° to a stale
  loc_ref on first contact; a +0.6° jog tracked its target in ~2.4 s, with a ~1 s
  static-friction crawl at the start). The commissioning `jog` therefore uses a
  position-mode move to a target; the 200 Hz control loop uses MIT mode (run_mode=0),
  not speed mode.

## Phase 2 — Homing and safety foundation

- [x] Versioned YAML config loader + validation (`control/src/config/turret_config.{hpp,cpp}`, §40; all §58 commissioning params are config, not compile-time). Missing/TBD values → built-in conservative defaults + warning; required keys / bad schema_version / out-of-range / `min>=max` / `dir_sign∉{±1}` / duplicate can_id → hard error. 9 loader tests green.
- [ ] Calibration persistence with integrity (§28, §41): schema version, timestamps, HW IDs, atomic write (tmp+rename), last-known-good
- [x] Host logical joint coordinates (§24): raw↔logical mapping, direction sign, home reference (`common/logical_coordinates.hpp`; endpoint+0 convention via `setup_model_from_endpoints`)
- [x] Contact detector (§21): filtered velocity/progress/signed-effort, dwell, independent hard abort (5 tests)
- [x] Precision homing state machine per endpoint (§22): coarse approach → back-off → settle → fine approach → repeatability verify (5 tests, §26 failure semantics)
- [x] Full-axis homing + travel band validation (§23): wrapper `calibration/full_axis_homing.{hpp,cpp}` (home A → traverse → home B → validate span, build logical model); 5 unit tests on a two-stop simulated plant (success, travel too short/long → invalid, endpoint A fault / endpoint B timeout → whole axis fails)
- [x] Multi-axis homing plan interpreter (§25, §25.1, §25.2): `home_endpoint` / `move` / `home_full_range` steps — `calibration/homing_plan.{hpp,cpp}` (8 tests)
- [x] Soft limits + braking envelope (§18.1, §18.2): stop feasibility from the trajectory stop model — `control/safety_envelope.{hpp,cpp}` (9 tests); independent of trajectory gen (§18.3)
- [x] Safety supervisor (§38) with layered watchdogs (§39): pure const-evaluable decision gate (owns the envelope), stale-feedback/over-temp/deadline layers, homing gate per §38.1 — `control/safety_supervisor.{hpp}` (13 tests)
- [x] Boot state machine (§27) incl. FAULT_LOCKED path: POWER_ON→…→DISCOVER→MOTOR_SELF_TEST→UNHOMED (camera/installation/payload stubbed in Phase 2) — `control/boot_fsm.{hpp,cpp}` (3 tests)
- [x] Safe shutdown/park state machine (§33) incl. verification + dwell: move yaw→pitch, verify pos+vel, dwell, de-energize pitch→yaw; rejects a park pose outside the soft limits (§33.1) — `calibration/park_controller.{hpp,cpp}` (5 tests)
- [x] `controld` daemon: `MotorBackend` abstraction + `SimMotorBackend` (test plant) + `CanMotorBackend` (real CAN, §46 position-mode recipe) + `ControlLoop` (§46 200 Hz engine) + `main.cpp` (config→boot→home→hold→park, SIGINT/SIGTERM park) — `control/src/control/`, `control/src/main.cpp`
- [x] Unit tests: 15 ctest binaries green (config 9, contact 5, homing 5, full-axis 5, homing plan 8, trajectory 4, envelope 9, supervisor 13, park 5, control loop 3, boot fsm 3, protocol/history/seqlock/timing)

## Phase 3 — Trajectory generator

- [x] Online jerk-limited generator (§17): velocity-cap + stop-distance profile, proportional vel loop (τ=0.2 s) + jerk clamp; 4 tests (commit ce1d405). Bang-bang variants were dead-ends (limit-cycling) — see log.
- [x] Receding-horizon target updates without discontinuity (§17.3): `set_target` retargets continue from the current q/v/a state (no restart from zero); C0/C1 continuity verified through a single mid-move reversal and repeated rapid retargets (`ChangingTargetIsSmooth`, `RepeatedRapidRetargetsStayContinuous`)
- [x] Stopping trajectories + `verify_stop_reachability` (§17.2, §48): `StopPlan plan_stop(state, a_brake, j_brake)` (jerk-limited d_stop = v²/2a + va/2j, consistent with the SafetyEnvelope model) + `verify_stop_reachability(state, q_boundary, a_brake, j_brake)` (stop-before-boundary check); **2000-sample randomized unit test** over q∈[−3,3] rad, v∈[−v_max,v_max] (fixed seed, reproducible) per §48
- [x] Coupled collision envelope interface (§19): `CollisionEnvelope` abstract (controller/checker depend only on it) + `RectangularCollisionEnvelope` (constant pitch/yaw limits, the v1 form); `is_path_safe` validates the **whole path**, not just the endpoint (mid-path exit is caught); a piecewise table/polygon plugs in as a new `is_safe` impl later without touching the controller — `control/collision_envelope.hpp` (7 tests)

## Phase 4 — Vision daemon (Python)

- [x] `vision/visiond.py`: frame source (real Picamera2/IMX500 **guarded** + deterministic `SyntheticFrameSource`), detection parse, `SensorTimestamp` + frame sequence attach. Runs the full pipeline in `--synthetic` mode with NO camera and NO motor
- [x] Target association/selection (§12): class gate, confidence threshold, IoU + centroid-proximity association, track age/continuity, auto-reacquisition, configurable fallback → **one** selected target per frame (`vision/target_selector.py`)
- [x] `TargetMeasurement` publish over UDS `SOCK_SEQPACKET` (§6.1, §6.2) — `vision/ipc.py`; 58-byte wire format, cross-checked C++↔Python (both directions)
- [x] **No CAN access, no motion logic in visiond** (enforced by design; the process only publishes measurements — verified by the camera-free test suite, which never touches CAN or the motor driver)
- Tests: 17 `unittest` cases (protocol round-trip, association/continuity/reacquisition, end-to-end daemon→subscriber) + CLI smoke; all camera-free. **Live IMX500 run is deferred to the test queue** (needs the real camera; see `docs/post_homing_test_queue.md` §P7).

## Phase 5 — Geometry and estimator (C++)

- [x] `geometry/camera_model.*`: intrinsics + full projective pixel↔ray (§10.1/§10.2), OpenCV/IMX500 camera frame (X right, Y down, Z forward); distortion is a no-op placeholder for v1 (added as a hook later)
- [x] `geometry/turret_kinematics.*`: base frame (X fwd, Y left, Z up); `ray_to_base = R_z(q_yaw)·R_y(q_pitch)·R_P_C` with a **configurable** R_P_C extrinsic (aligned default; swappable for a fiducial-board estimate, §10.3) + LOS→(azimuth, elevation). The LOS→**joint** solve (§14) is the reference manager's job → Phase 6
- [x] Timestamp alignment (§11): `MotorStateHistory::interpolate` at the frame's `sensor_timestamp` (returns false → timing-invalid, never silently uses a newer pose) — 6 tests; the replay test proves a stale pose gives a visibly wrong LOS
- [x] `tracking/target_estimator.*`: const-angular-velocity alpha-beta on base-frame LOS angles (azimuth wrap-safe) + forward prediction to actuation time (§13.1/§13.3); a full covariance Kalman + confidence-aware measurement noise (§13.2) is the documented upgrade path
- [x] Replay tests with recorded measurements (§54.3): deterministic scenario (moving target + moving gimbal) replayed through pixel→interpolated-pose→base-frame-LOS→estimator, no camera/CAN/motor; 6 new C++ test binaries
- Note: `tracking/target_measurement.hpp` mirrors `vision/protocol.py` (58-byte LE); cross-language encode/decode verified both directions.

## Phase 6 — Closed-loop tracking

- [x] `control_loop.*` per §46 pseudocode (200 Hz, absolute-deadline sleep, no I/O/alloc in loop) — tracking path wired into the existing loop; 200 Hz (5 ms) step verified in the SimMotorBackend integration test
- [x] Reference manager arbitration with priority (§16) — `control/reference_manager.hpp`: TRACKING (Tracking/Coasting) > SEARCH > HOLD, v_max scaled by confidence
- [x] Tracking state machine (§34): READY_HOLD/TRACKING/COASTING/BRAKE_TO_HOLD/LOST, config thresholds — `tracking/tracking_state_machine.hpp` + §35 confidence decay
- [x] Search planner (§36, §49) and stationary hold (§37) — `control/search_planner.hpp` (soft-limit sweep, S-curve dwell, reduced speed); hold = ready-pose reference
- [x] Telemetry snapshot publisher 10–20 Hz (§6.3) + high-rate control log (§43.1) + event log (§43.3) + black-box ring (§43.4) — `telemetry/telemetry.hpp` (ring-buffered, filled in-loop)
- [x] LOS→joint solver (§14) — `geometry/los_joint_solver.hpp` (angular-decomposition seed + gradient refinement vs calibrated R_P_C)
- [x] Closed-loop integration test (mocks only, no CAN) — `tests/test_tracking_integration.cpp`: synthetic base-frame target → pixel → full stack on SimMotorBackend; tracking, loss→coast→brake→hold, search sweep, soft-limit containment, fault→safe stop
- [x] controld main(): signal handling, boot FSM wiring, UNHOMED on restart (§52) — from Phases 2/5 (daemon present; not run, no motor)
- [ ] systemd unit files (§52) — deployment only, deferred (no motor)

## Phase 7 — Installation orientation

- [x] `installation_pose.*`: `InstallationPoseProvider` interface + `FixedStoredPoseProvider` (§30) — `calibration/installation_pose.hpp` (+ `test_installation_pose`); `main.cpp` loads the pose file and feeds `loop.set_base_orientation(...)`
- [x] Fiducial (ChArUco) multi-frame visual calibration, outlier rejection, atomic commit (§29) — `vision/installation_calibration.py` (+ `test_installation_calibration`); R_W_B written as C++-compatible text, atomically committed; camera-only, no motor
- [x] World-frame telemetry (§29/§30) — `calibration/world_frame_telemetry.hpp` (+ `test_world_frame_telemetry`): base tilt + world-frame LOS from the active R_W_B, published in the §6.3 snapshot
- [x] controld loads the stored pose at boot and makes tracking world-correct for a tilted base

## Phase 8 — Web UI and diagnostics

- [x] `web/webd` FastAPI: dashboard (§42.1 panels: System/Vision/Target/Yaw-pitch/Calibration/Video/CAN), developer controls (§42.2), **real IMX500 MJPEG video feed** with an on/off switch (separate low-priority path, §42.3) — `web/webd/app.py` + `dashboard.py` + `video.py`
- [x] controld-side web server (§5.3/§6.1/§6.3): `control/src/web/web_server.hpp` — JSON over UDS `SOCK_SEQPACKET`, 10–20 Hz downsampled snapshot, command relay; never opens can0 (+ `test_web_server`)
- [x] Developer-command validation gate (§42.2): `control/src/web/command_validation.hpp` + `ControlLoop::submit_command`/`process_commands` (validated on the web thread, executed on the control thread) (+ `test_command_validation`)
- [x] Telemetry restructured: the loop ALWAYS fills the §6.3 snapshot (webd reads it from a non-RT thread); tracking fields conditional
- [x] Hostname-based access only, no hard-coded IP (§53) — `web/webd/config.py` (bind host/port + socket path via env)
- [x] webd tests (no CAN; camera via a fake `picamera2`): `web/webd/tests/` — protocol, controld client, FastAPI app, **video source + video API** (+ a `FakeControld` stand-in); 33 tests
- [x] Video feed (§42.3): `web/webd/video.py` `VideoSource` opens/releases the IMX500 on demand (off = zero CPU), capped-FPS MJPEG over `GET /api/video` (multipart/x-mixed-replace), `POST /api/video/start|stop` + `GET /api/video/state`; dashboard switch toggles it. Env knobs `OTA_VIDEO_{ENABLE,WIDTH,HEIGHT,FPS,QUALITY}` (§53)
  - [x] Install-level image corrections, applied **at the source** so preview and vision/control see the same image (§42.3): `common/image_corrections.py` — orientation (`rotate_180`/flip_h/flip_v on the frame **and** on visiond detector boxes) + white balance (no-clip gray-world). webd applies both in the capture callback; driven by `OTA_VIDEO_{ORIENTATION,WHITE_BALANCE}` (§53)
- [x] UI load test: control p99 unaffected, no client starvation, no CAN-feedback staleness (§54.5) — `tests/test_web_load.cpp`
- [x] systemd unit files (§52) — `Firmware/systemd/`: `turret-control` / `turret-vision` / `turret-web` (+ optional `turret-log`) + `can0` bring-up; `Restart=on-failure` is safe because boot always returns UNHOMED; `Wants=` (not `Requires=`) keeps the loop network-independent; templates only, not run (no CAN/motor)

## Phase 9 — Payload profiling/tuning

- [x] Profiling utility (§44): step/triangle/brake/holding tests → rise/settle/overshoot/stop distance
  - `control/src/payload/response_metrics.{hpp}` — `analyze_step`/`analyze_brake`/`rms_tracking_error` (first-order rise/settle, overshoot, stop distance, tracking RMS; validity guards).
  - `control/src/payload/payload_profiler.{hpp,cpp}` — the §44 battery over any `MotorBackend` (injectable clock/pacer ⇒ deterministic in tests): ±step, tracking triangle, multi-speed brake, holding effort; `clamp_target` keeps every target ≥2° inside the soft limits (fails safe on a degenerate band); `derive_limits` yields a conservative v/a/j envelope from the measured response.
  - `tools/turret_payload.cpp` + `turret-payload` — operator CLI: `profile` / `verify` / `list`. `--sim` runs the full battery on the `SimMotorBackend` (no CAN/motor); real mode is wired (CAN + `BootFsm`) but only run on the supervised station.
- [x] Payload profile storage + mismatch detection + derating (§31.3, §31.4, §41, §42.1/§42.2)
  - `control/src/payload/payload_profile.{hpp,cpp}` — schema-v1 store (atomic tmp+fsync+rename, `.prev` backup, safe-name validation, dir creation).
  - `control/src/payload/payload_verifier.{hpp,cpp}` — `VerifyTolerances` (rise/settle ratio, overshoot, peak-effort, tracking-RMS), per-axis `compare_axis`, `overall_status`, `decide` (only a Mismatch derates).
  - `control_loop.{hpp,cpp}`: `Phase::PayloadCheck` runs the §31.3 in-loop check one axis at a time (amplitude clamped by the margin to the region edges), auto-verifies once after reaching Hold (§27) and accepts a manual `start_payload_verification` command (§42.2). A Mismatch sets `payload_derated_` ⇒ `hold_speed_effective()` / ready-pose & test-motion limits / tracking `v_max` all drop by `derate_factor`; a re-verified `Ok` clears it. SafetySupervisor keeps authority (§38).
  - Web mirror: 4 telemetry fields (`payload_profile_name/status/derated/check_active`) over the wire + a Payload panel on the dashboard.
- [x] Tests on the **mocked device** (SimMotorBackend; no CAN/motor): response metrics, profile store, profiler, verifier, daemon integration
  - 5 new C++ binaries, all `SimMotorBackend`-only (no CAN, controld never started): `test_response_metrics` (8), `test_payload_profile` (10), `test_payload_profiler` (6), `test_payload_verifier` (10), `test_payload_daemon` (4: auto-verify matches baseline, mismatch derates + re-verify clears, no-profile behavior, store round-trip).
  - **Result: 38/38 ctest green** (33 prior + 5 new) **and** 95/95 Python green (92 prior + 3 new webd protocol tests).

## Cross-cutting

- [ ] Testing per §54 (unit, simulated plant, replay, HIL checklist, UI load)
- [ ] Acceptance-metric reporting per §55 (timing/CAN/homing/tracking/limits)
- [ ] Simulated axis plant for state-machine tests (§54.2)

## Open commissioning parameters (doc §58 — config, not code blockers)

`Firmware/config/turret.yaml` now carries **conservative placeholders** for the 23 §58 values (slow speeds, wide soft-limit bands — the pitch band is wide enough to contain the observed −86° position). The loader falls back to the same built-in conservative defaults for any key left `TBD` or missing (with a warning). Refine them as homing commissioning measures the real travel/speeds/stall thresholds; they are config, never compile-time assumptions.

## Session log

- **2026-09-01 (NZST)** — Moved `control/` from repo root to `Firmware/control/` (corrected placement). Created this tracker. Plan: finish Phase 1 (system impl, build, tests, probe CLI), then Phases 2+3 (Appendix D milestone: homing, logical coords, jerk-limited motion, braking, park).

- **2026-09-01 (NZST)** — Resolved the periodic-feedback blocker. Root-caused via raw
  sniffer + netlink repro: (1) fixed `can_netlink.cpp` single-interface RTM_GETLINK hang
  (kernel 6.18 sends NEWLINK but never NLMSG_DONE → now completes on match, all recvs
  poll-bounded, set path uses NLM_F_ACK); (2) fixed `socketcan_bus.cpp` CAN_RAW bind to use
  `struct sockaddr_can` (was `ifreq`). Then discovered the CyberGear does NOT free-run
  feedback: it is request/response (each MIT/enable/stop/reg-write → one COMM_TYPE_2).
  Verified 1:1 at 200 Hz on both motors. Set `Ranges::kFbAngle*` to ±12.5 (matches the MIT
  position mapping; tracks mechPos to <0.001 rad, manual's ±4π is off ~0.005 rad). Reworked
  `turret-can feedback` to drive §25.1 MIT active-hold and display live per-axis data —
  fan-out confirmed (pitch 0x64 / yaw 0x65). Added `FeedbackAngleRangeMatchesMitPosition`
  test. 32 tests green. Next: tiny pre-approved jog → stats → stop, then Phase 2/3.

- **2026-09-01 (NZST)** — Re-ran the pre-approved pitch jog per user request. The speed-mode
  jog (run_mode=2 + SpdRef) did not drive the loaded axis (measured ~1% of commanded
  speed with default gains, limit_cur up to 5 A), so I reworked `jog` into a safe
  position-mode move (stop → run_mode=1 → enable → limit_spd=2 → loc_ref = current +
  delta → wait for arrival). `jog pitch 2 0.3` moved +0.6° to its target in ~2.4 s (~1 s
  static-friction crawl, then accelerating) and now holds stably at −1.504 rad (−86.2°)
  in position mode. Confirmed the earlier −37° excursion was a stale loc_ref drive
  (fixed by pinning loc_ref to the current position before entering position mode).
  Speed-mode weakness noted for later; the 200 Hz control loop uses MIT mode (run_mode=0).
  Next: Phase 2/3 (YAML config, logical coords, homing FSM, stop envelope, park).

- **2026-09-01 (NZST)** — Built the Phase 2 config foundation. Added
  `control/src/config/turret_config.{hpp,cpp}`: a versioned YAML loader (§40) covering
  every block (can / motors / control / axes / homing.contact / tracking / shutdown /
  camera / installation / payload). Required keys, `schema_version==1`, `can_id`/
  `host_can_id` ∈ [0,255], `direction_sign∈{±1}`, `travel.min<max`, and positive
  speeds/accels/jerks/dwell are hard errors; any §58 commissioning value left `TBD` or
  missing falls back to a built-in conservative default (slow, wide bands) with a
  warning. Added `Firmware/config/turret.yaml` with conservative placeholders matching
  the discovery ground truth (pitch can_id 100 dir +1, yaw can_id 101 dir −1, host 0,
  can0 @ 1 Mbit, loop 200 Hz; pitch band [−120,120] contains the observed −86°). 9 new
  loader tests (full load, TBD fallback, missing-key, bad schema, bad dir_sign,
  duplicate can_id, missing file, malformed YAML, default-safety) — all 41 tests green.
  Next: wire config into `controld`, then logical coordinates + homing FSM.


- **2026-09-01 (NZST)** — Implemented the §23 full-axis homing wrapper and ran one
  full homing operation against a simulated plant (no live motor motion). New
  `control/src/calibration/full_axis_homing.{hpp,cpp}`: a transport-agnostic
  `FullAxisHoming` that sequences two `HomingController` (§22) instances —
  precision-home endpoint A → (the endpoint-B approach doubles as the §23
  "traverse safely") → precision-home endpoint B → compute measured travel —
  then validates `expected_travel_min_deg <= measured_travel_deg <=
  expected_travel_max_deg`. On success it sets up the host logical model (§24)
  via `setup_model_from_endpoints` (low endpoint = logical 0, travel positive)
  and reports the measured span; on any endpoint failure or out-of-band travel it
  fails with `valid=false` (the axis must not be treated as referenced, §23/§26).
  Build green (ninja, new files auto-globbed). Ran ONE operation end-to-end via a
  standalone harness on a two-stop plant (stops at ±1.0 rad, start mid-travel at
  0, like the real pitch position): endpoint A homed at −1.0000 rad, traversed,
  endpoint B homed at +1.0000 rad, measured travel 114.592° (within the [100,130]°
  band), worst repeatability 0.000° (limit 0.5°), logical model dir_sign=+1 with
  raw_ref=−1.0000 rad → logical 0 and endpoint B at +114.592° logical. Terminal
  state Complete, result valid, harness exit 0. Next: in-repo unit tests for the
  wrapper (success + travel-out-of-band + single-endpoint-fail), then §25 homing
  plan, §18 safety envelope, and wiring config+trajectory+homing into controld.

- **2026-09-01 (NZST)** — Added the in-repo unit tests for the §23 full-axis
  homing wrapper and committed it. New `control/tests/test_full_axis_homing.cpp`
  (5 tests) drives `FullAxisHoming` against a two-stop simulated plant (end-stops
  at each end, torque signed in the travel direction, per-end-side clamping):
  (1) both endpoints homed + travel 114.592° inside the [100,130]° band →
  Complete/valid, logical model dir_sign=+1 with the low endpoint at logical 0
  and the high endpoint at +travel; (2) travel too short (28.6°) → both endpoints
  valid but the axis fails validation ("outside expected"), position invalid;
  (3) travel too long (229.2°, widened travel limit) → same out-of-band failure;
  (4) motor fault during endpoint A → whole axis fails, reason attributed to
  "endpoint A … hard abort or motor fault"; (5) endpoint B approach timeout
  (unreachable stop) → whole axis fails, reason attributed to "endpoint B …
  timeout", endpoint A still reported valid. One test bug found and fixed: the
  first cut set the approach timeout to 5 s, which is shorter than endpoint A's
  own coarse (~5.7 s) and fine (~5 s) approaches, so endpoint A timed out first —
  raised to 10 s (the failure correctly proved the wrapper attributes per-endpoint
  faults). Full suite now 9/9 binaries green (60 tests: config 9, contact
  detector 5, full-axis homing 5, history 7, homing 5, protocol 17, seqlock 4,
  timing stats 4, trajectory 4). Committed wrapper + tests.

- **2026-09-01 (NZST)** — Completed the Phase 2 safety foundation and the
  `controld` daemon (all in simulation; **no live motor motion**). New transport
  abstraction `control/motor_backend.hpp` (SETUP: discover/read/enter-position-mode/
  de-energize; CONTROL LOOP: non-blocking snapshot + fire-and-forget command) so
  `ControlLoop`/`BootFsm` depend only on the interface (§54 testability). Two
  backends: `sim/sim_motor_backend.hpp` (first-order position plant with end stops
  + stall effort, so contact detection and park settle behave like the real motor —
  a bang-bang velocity model was rejected: it limit-cycled and never settled below
  the MoveTo velocity tolerance) and `control/can_motor_backend.{hpp,cpp}` (real
  CyberGear transport; the verified-live §46 position-mode recipe: stop→50 ms→
  RunMode=1→enable→50 ms→LimitSpd→read MechPos→LocRef=current). New `ControlLoop`
  (`control/control_loop.{hpp,cpp}`), the §46 200 Hz per-cycle engine: snapshot →
  deadline watchdog → supervisor decision → per-axis phase reference (homing/hold/
  park/fault) → safety action (Allow/Derate/Hold/Brake/Disable) → command; tracking
  is hard-disabled in Phase 2. `BootFsm` (§27, boot-only slow path): discover +
  self-test → UNHOMED, any failure → FAULT_LOCKED. `main.cpp` rewritten: config →
  open CAN → boot → start homing → 200 Hz hold loop → SIGINT/SIGTERM park. Fixed a
  post-homing-recovery bug: a freshly-homed axis sits at the raw stop (outside the
  inset soft limit) at rest, so `stop_feasible` now short-circuits `true` when
  |v| < at_rest (a stationary axis cannot cross a boundary by stopping). Added
  `Firmware/docs/post_homing_test_queue.md` (prioritized live/HIL queue: P0 live
  homing → P1 hold → P2 park → P3 re-home repeatability → P4 stale feedback → P5
  fault injection → P6 payload stub; flags the 0/0 park default that violates
  §33.1). Full suite now **15/15 ctest binaries green**; `controld` builds and
  exits cleanly on a bad config (no CAN). **Stopping here, before the live homing
  run** (user authorizes/runs it). Next: live homing → post-homing queue → (Phase 3)
  collision envelope + tracking.

- **2026-09-01 (NZST)** — Completed **Phase 3 (trajectory generator)**, all in
  simulation, verified with unit tests only (no live/real tests run during
  implementation). Three items:
  (1) **Receding-horizon retarget (§17.3)** — the generator already continues from
  the current q/v/a on retarget (no restart from zero); added `RepeatedRapidRetargetsStayContinuous`
  alongside the existing `ChangingTargetIsSmooth` to prove C0/C1 continuity through a
  mid-move reversal and repeated rapid retargets.
  (2) **Stopping trajectories + stop-reachability (§17.2, §48)** — added
  `StopPlan plan_stop(state, a_brake, j_brake)` and
  `verify_stop_reachability(state, q_boundary, a_brake, j_brake)` to
  `JerkLimitedTrajectory`. The stop model is the jerk-limited
  `d_stop = v²/2a + va/2j`, kept **consistent with `SafetyEnvelope::stop_distance`**
  (the envelope adds its extra safety margin on top). `plan_stop` treats
  |v| < at_rest (1e-3 rad/s) as "already stopped → stay here", mirroring the
  envelope's at-rest short-circuit. Added a **2000-sample randomized test** over
  q∈[−3,3] rad, v∈[−v_max,v_max] (fixed-seed, reproducible) per §48
  ("unit-tested against randomized positions/velocities"): it asserts the stop lands
  strictly in the direction of motion, that a boundary just beyond the stop is
  reachable and one just before (in the direction of motion) is not.
  (3) **Coupled collision envelope interface (§19)** — new
  `control/collision_envelope.hpp`: the `CollisionEnvelope` abstract type (the
  controller/checker depend only on it) + `RectangularCollisionEnvelope` (constant
  pitch/yaw limits, the v1 form) + `is_path_safe`, which validates the **whole
  path** (a mid-path exit is caught even when both endpoints are inside). A
  piecewise table/polygon plugs in later as a new `is_safe` impl without touching
  the controller. 7 collision tests (inside/outside, custom ranges, whole-path-vs-
  endpoint, empty path, polymorphic interface).
  One test bug found and fixed: the first cut of `plan_stop` had no at-rest branch,
  so the randomized test's at-rest case saw a tiny (~1e-5 rad) non-zero stop
  distance and failed the exact-zero check — added the at-rest short-circuit to
  `plan_stop` (and dropped v=0 from the model test, covering it via the at-rest
  sub-case instead). Full suite now **16/16 ctest binaries green**. No live motion;
  no `vcan9`/`can0` changes.

- **2026-09-01 (NZST)** — Implemented **Phase 4 (vision daemon, Python)** and
  **Phase 5 (geometry + estimator, C++)**. Per the user's instruction, all
  camera/estimator work is verified with **safe, camera-free / replay tests that
  never invoke the motor driver**, and the **live** camera + tracking tests are
  queued for later (post-homing queue §P7/§P8), not run now.
  - **Phase 4** (`Firmware/vision/`): `protocol.py` (the §6.2 `TargetMeasurement`
    + a fixed 58-byte little-endian wire format), `frame_source.py` (a
    `FrameSource` with a **deterministic `SyntheticFrameSource`** for
    camera-free runs and a **guarded** real `Picamera2FrameSource`),
    `target_selector.py` (§12 association: class gate + confidence + IoU +
    centroid proximity + track age/continuity + auto-reacquisition + fallback →
    one selected target), `ipc.py` (UDS `SOCK_SEQPACKET` latest-value publisher +
    a test subscriber), and `visiond.py` (the daemon; `--synthetic` is the SAFE
    default, `--real` needs the camera). **visiond never opens CAN or drives the
    motor** (enforced by design). 17 `unittest` cases + a CLI smoke, all
    camera-free (the end-to-end test runs the full pipeline into a subscriber
    stand-in for controld).
  - **Phase 5** (`Firmware/control/src/{geometry,tracking}/`): `vec3.hpp`
    (dependency-free Vec3/Mat3 — Eigen's CMake config isn't installed),
    `camera_model.hpp` (§10.1/§10.2 intrinsics + full-projective pixel↔ray,
    OpenCV camera frame), `turret_kinematics.hpp` (§9.2/§10.3 base frame X fwd /
    Y left / Z up; `ray_to_base = R_z(q_yaw)·R_y(q_pitch)·R_P_C` with a
    **configurable** R_P_C extrinsic + LOS→(az,elev)), `target_measurement.hpp`
    (mirrors `vision/protocol.py`), and `target_estimator.{hpp,cpp}` (§13
    const-angular-velocity alpha-beta on base-frame LOS, azimuth wrap-safe, +
    forward prediction to actuation time). Six new C++ test binaries:
    `test_camera_model`, `test_turret_kinematics`, `test_target_estimator`,
    `test_history_interpolation` (§11.1), `test_target_measurement`, and
    `test_replay` (§54.3 deterministic moving-target + moving-gimbal replay
    through pixel→interpolated-pose→base-frame-LOS→estimator, plus a check that a
    **stale** pose yields a visibly wrong LOS — the §11 warning made concrete).
  - **Cross-language protocol check:** C++ encode → Python decode and Python
    encode → C++ decode both verified (58-byte LE, all fields match).
  - Full suite now **22/22 ctest binaries green** (16 prior + 6) **and** 17
    Python vision tests green. Two compile fixes (missing `<cmath>` for `M_PI`;
    `CameraModel` default ctor) and two test-tolerance fixes (the synthetic target
    legitimately wraps → new track; the alpha-beta filter's steady-state lag on a
    moving target). No live camera, no `can0`/`vcan9` changes, no motor motion.
  - **Deferred (queued, not run):** §P7 live IMX500 vision verification
    (camera-only, no motor), §P8 live tracking (needs Phase 6). Phase 6 (LOS→joint
    solve §14, reference manager, tracking FSM, closed-loop) is next.

- **2026-09-01 (NZST)** — Implemented **Phase 6 (closed-loop tracking)** end-to-end
  in C++ and verified it with **mocks only** (SimMotorBackend + synthetic vision —
  **no CAN, no motor driver**, per the explicit "don't touch the actual motor"
  constraint). New sources (all header-only, auto-globbed):
  - `geometry/los_joint_solver.hpp` (§14): LOS→(q_yaw,q_pitch) via an
    angular-decomposition seed (aligned gimbal: q_yaw=azimuth, q_pitch=−elevation)
    refined by a 12-step gradient against the **calibrated** R_P_C; reports residual
    and returns false when the LOS is unreachable.
  - `tracking/tracking_state_machine.hpp` (§34/§35): READY_HOLD→TRACKING→(missing)
    COASTING→(timeout) BRAKE_TO_HOLD→TARGET_LOST→SEARCH/READY_HOLD, all thresholds
    config; confidence decays 1.0→0.0 as the target goes stale (0 before first
    acquisition, handled via `has_seen_valid_`).
  - `control/search_planner.hpp` (§36/§49): soft-limit yaw sweep with S-curve dwell
    turnarounds at reduced speed.
  - `control/reference_manager.hpp` (§16): TRACKING (Tracking/Coasting, v_max
    ×confidence) > SEARCH > HOLD arbitration.
  - `telemetry/telemetry.hpp` (§6.3/§43): snapshot + high-rate control log + event
    log + black-box ring (bounded ring buffers, filled in-loop, no alloc).
  - `control/tracking_controller.hpp`: the §11/§13/§14 glue — per-axis
    MotorStateHistory, pose interpolated at the capture timestamp (timing-invalid →
    not fed), pixel→ray→base→(az,el)→estimator, predict to now+control_delay+
    motor_response, then reference-manager output.
  - `control/control_loop.{hpp,cpp}`: `enable_tracking()` (gated on `homed_`, §38.1),
    `feed_measurement()`, and the Hold-phase tracking branch (tracking reference
    constrained by the §18 envelope) + in-loop control-log/black-box/snapshot
    publication.
  - `tests/`: 5 new component binaries (los_joint_solver, tracking_state_machine,
    search_planner, reference_manager, telemetry) + `tests/test_tracking_integration.cpp`
    (5 cases): a base-frame target projected to pixels through the real camera+kinematics
    drives the full stack on the simulated plant — **tracks a rotating target**
    (optical axis converges on it), **loss→coast→brake→ready-hold**, **search sweep**,
    **stays within soft limits**, and **fault→safe stop** (supervisor disables the
    motors, §16 top priority).
  - Coordinate-frame note: for the aligned 1:1 sim the LOS→joint output (joint
    angle) equals the raw position, so the tracking reference is used directly as the
    raw reference and clamped by the §18 envelope; the logical frame (deg from the
    homing endpoint) is used only for the ready pose/limits.
  - Result: **28/28 ctest green** (22 prior + 5 component + 1 integration) **and**
    17/17 Python vision tests green. Two test-setup fixes found while wiring the
    integration (the `TravelBand{min,max}` check is `span∈[0.5·(max−min),1.5·(max−min)]`;
    and the homing coarse approach can't close a 180° span, so the sim uses ±1 rad
    stops like the existing control-loop test). No live camera, no `can0`/`vcan9`
    changes, no motor motion.
  - **Next:** systemd unit files (§52, deployment), then Phase 7 (installation
    orientation). Live tests (§P7/§P8) remain queued — not run, no motor.

- **2026-09-01 (NZST)** — Implemented **Phase 8 (web UI + diagnostics)** end-to-end,
  computation-only (no CAN, no motor; controld not run).
  - **C++ (controld side, §5.3/§6.1/§6.3):** `control/src/web/web_server.hpp` — a
    non-RT web server that publishes the §6.3 telemetry snapshot (JSON over UDS
    `SOCK_SEQPACKET`, 10–20 Hz downsampled) and relays developer commands. It never
    opens `can0`; it only reads `loop.telemetry()` and calls `loop.submit_command`.
    Restructured the control loop so it **always** fills the top-level telemetry
    (so webd reads a fresh snapshot from another thread); tracking fields are
    conditional. Added `ControlLoop::submit_command`/`process_commands` + a
    `web::command_validation` gate: a developer command is **validated on the web
    thread** against `command_state_` and, only if valid, executed on the control
    thread next cycle (§42.2 "every request goes through the state validation").
    `main.cpp` wires the web server (env-overridable socket/rate, §53) and stops it
    on shutdown.
  - **Python (`web/webd`, §42/§53):** FastAPI/Uvicorn app + dashboard (System/
    Vision/Target/Yaw-pitch/Calibration/Video-placeholder panels + §42.2 developer
    controls), a `ControldClient` (UDS reader with reconnect, command send), a
    config loader (host/port/socket/hz/title via env — **no hard-coded IP**), and a
    `FakeControld` server stand-in for tests. `telemetry_from_json`/`to_json` match
    the C++ wire format exactly (lockstep).
  - **Tests:** C++ `test_web_server` (transport), `test_command_validation`
    (the §42.2 gate), and `test_web_load` (§54.5: 8 clients connected while the 200 Hz
    loop runs — control timing not degraded, **no client starvation**, CAN-feedback
    staleness proxy stays fresh). Python `web/webd/tests/`: protocol, controld client
    (incl. reconnect), and the FastAPI app — **20 tests**. Plus a live uvicorn smoke
    run (fake controld → `/api/health`, `/api/state`, `/api/command`, dashboard).
  - **§52 systemd:** `Firmware/systemd/` — `can0`, `turret-control`, `turret-vision`,
    `turret-web`, (+ optional `turret-log`) + a README. `turret-control` restarts
    `on-failure` but **always returns UNHOMED** (boot re-homes; never resumes stale
    coords); all deps are `Wants=`/`After=` so the deterministic loop never depends
    on network or the web UI (§53). Templates only — none enabled/run (no CAN/motor).
  - Result: **33/33 ctest green** (32 prior + 1 web-load) **and** 20/20 Python webd
    green; controld builds clean. No live camera, no `can0`/`vcan9` changes, no
    motor motion.
  - **Next:** Phase 9 (payload profiling/tuning, §44/§31) — or the queued live
    commissioning tests (§P7/§P8), which need the physical station + a safe,
    supervised run (not automated here).
- **2026-09-01 (NZST)** — Added the **real video feed** to the Phase 8 web UI (the §42.3 placeholder became a working IMX500 stream) with an **on/off switch**. Computation + camera only — no CAN, no motor, controld not run.
  - **Probed the camera first** (permitted HW): libcamera/picamera2 present; IMX500 registered to the PiSP pipeline (BCM2712_C0). Confirmed the safe capture path is the picamera2 **request callback** + software JPEG (`main` XBGR8888 -> RGB) — the hardware `MJPEG` main *format* aborts the process, so it is not used. Real rate: 640×480 at ~30 fps available; we cap the publish rate for §42.3 low-priority.
  - **`web/webd/video.py`** `VideoSource`: a single-owner, on/off camera. `start()` opens the IMX500 lazily/defensively (webd still runs with no camera; a busy/absent camera is reported as an endpoint error, never a crash) and waits for the first frame. A capture thread (the picamera2 callback) JPEG-encodes **only when a frame is due** (FPS cap = the main §42.3 CPU lever) and stores the latest frame in a lock-guarded slot; N browser clients share that one capture. `stop()` releases the camera. **Separate path:** it never touches the controld control socket (§6.1/doc §42.3); webd is already a separate process from controld, so browser video cannot degrade the control loop or CAN feedback (§54.5).
  - **`web/webd/app.py`**: `GET /api/video/state`, `POST /api/video/start` (optional width/height/fps), `POST /api/video/stop`, and `GET /api/video` (MJPEG, optional `?limit=N` safety valve). Start/stop run via `asyncio.to_thread` (camera open is blocking). Camera released on shutdown.
  - **`dashboard.py`**: the Video panel is now full-width with a **switch** that POSTs start/stop and points an `<img>` at `/api/video` (MJPEG-in-`<img>`). Off = camera released (no CPU); on = live feed. Reflects state on load.
  - **`config.py`** (§53): `OTA_VIDEO_{ENABLE,WIDTH,HEIGHT,FPS,QUALITY}` (defaults 1/640/480/15/80); `tools/run_webd_inspection.py` now exposes them.
  - **Tests:** `web/webd/tests/fake_camera.py` (a fake `picamera2` that mimics the exact API used), `test_video.py` (lifecycle, FPS cap, shared slot, MJPEG framing, disabled), `test_video_api.py` (state/start/stop + bounded MJPEG stream, custom size, disabled feature). **33/33 Python webd green** (20 prior + 8 + 5); C++ build unchanged (ninja no-op).
  - **Live-verified on the real camera:** `/api/video/start` opened the IMX500 in ~0.9s (`camera: imx500`, 640×480 @ 15fps cap -> ~11–15 fps effective); `/api/video?limit=4` returned 4 valid 640×480 JPEG frames; `frames_published` advanced while running (live); `stop` released the camera and the stream returns 409. The inspection web service was restarted and is serving the **real** video (telemetry remains the FakeControld sim).
  - **2026-09-01 (NZST)** — Fixed the two live-feed issues the user reported: the camera is **mounted upside-down** and the feed was **sunk in a red tone**. Both are corrected **at the source (capture), shared by the preview and the vision/control pipeline** — not a browser display hack. Computation + IMX500 only — no CAN, no motor, controld not run.
    - **`common/image_corrections.py`** (new, dependency-light package): `apply_orientation_image` (rotate_180 / flip_h / flip_v as strided views), `apply_orientation_bbox` (the same transform on detector boxes so visiond's HW-detector geometry stays correct), `gray_world_correction` (per-channel multipliers that equalize the means **and** add a common de-saturation scale so a saturated channel never clips the others), `apply_white_balance`, + `validate_orientation`/`validate_white_balance`.
    - **Orientation:** `web/webd/video.py` applies it to the raw frame *first* (before channel-extract/WB); `vision/frame_source.py` + `visiond.py` apply it to the detector boxes (`--orientation`). Driven by `OTA_VIDEO_ORIENTATION` (§53); this install is `rotate_180` (set in `tools/run_webd_inspection.py`). **Live-verified:** the served frame correlates 0.9996 with a 180° rotation of the raw sensor frame (−0.34 un-rotated), so the correction is genuinely in the path. If the physical mount is a mirror-flip instead, switch the env to `flip_horizontal`/`flip_vertical`.
    - **White balance (the red tone) — root cause was clipping, not a channel swap.** The IMX500's R channel is **100% saturated at 255** (measured) with G/B carrying a heavy bright tail (~25% of pixels above 186); the camera's AWB drifts and its AE under-exposes over time. A plain gray-world (per-channel gain = target/mean, EMA α=0.2) looked neutral on paper but the live feed stayed warm (R/G≈1.15): boosting G/B to match the low red gain pushed their bright pixels past 255, which **clipped** and dragged their means back down. Fix: `gray_world_correction` scales the gray-world gains by a common de-saturation factor (the largest `s≤1` such that no channel's max × gain clips at 255), so every channel lands on the same mean; also disabled the camera's **AWB** at capture (`AwbEnable=0`, webd owns the balance) so it can't drift underneath.
    - **Live result (IMX500, 640×480@15):** before `R=255 G≈96 B≈98` (R/G≈2.65, very red) → after **`R=97 G=96.6 B=97.2` (R/G=1.003, R/B=0.998), neutral and stable** across frames. The de-saturation lands the feed on a common (lower) mean — the correct neutral for a saturated-red scene. The converged multipliers are exposed as `wb_gains` in `/api/video/state` (and `orientation`/`white_balance`) for diagnosis/tuning.
    - **Tests:** 4 new `common` tests for `gray_world_correction` (neutral noop, black safe-noop, saturated-red neutralizes, no-clip beats plain gray-world). **Full suite 92 green (28 common + 34 webd + 30 vision, 2 skipped = the camera-only paths).**
    - **Next:** confirm the served image reads as right-side-up to the user (180° is the standard upside-down correction; switch `OTA_VIDEO_ORIENTATION` if it's a mirror). Phase 9 (payload profiling, §44/§31) or the queued live commissioning tests (§P7/§P8) remain the next functional steps.
- **2026-09-01 (NZST)** — Implemented **Phase 9 (payload profiling/tuning)** end-to-end on the **mocked device only** (`SimMotorBackend`; no CAN, no motor, controld never started). All production code under `Firmware/`.
  - **C++ (`control/src/payload/`):** `response_metrics.{hpp}` (step/brake/tracking metrics + validity guards), `payload_profiler.{hpp,cpp}` (§44 battery over any `MotorBackend` with an injectable clock/pacer for determinism; `clamp_target` keeps every target ≥2° inside the soft limits and fails safe on a degenerate band; `derive_limits`), `payload_profile.{hpp,cpp}` (schema-v1 atomic store, `.prev`, safe names), `payload_verifier.{hpp,cpp}` (tolerances, `compare_axis`, `overall_status`, `decide`). Fixed `payload_profile` to round-trip `max_verified_speed_rad_s`.
  - **Daemon (`control/src/control/control_loop.*`):** `Phase::PayloadCheck` (§31.3 in-loop check, one axis at a time, amplitude clamped by region margin), auto-verify once after Hold (§27), manual `start_payload_verification` command (§42.2), mismatch→`payload_derated_`→`hold_speed_effective()`/ready/test/tracking `v_max` derate (§31.4), re-verify clears it. `SafetySupervisor` keeps authority (§38).
  - **Tool (`tools/turret_payload.cpp` → `turret-payload`):** `profile`/`verify`/`list`; `--sim` rehearsal fully working (smoke: profile + verify OK on the sim plant). Real mode wired but never executed here.
  - **Web mirror (`web/webd/`):** 4 telemetry fields + a Payload panel on the dashboard; 3 new protocol tests.
  - **Tests — mocked device only:** 5 new C++ binaries, all `SimMotorBackend`: `test_response_metrics` (8), `test_payload_profile` (10), `test_payload_profiler` (6), `test_payload_verifier` (10), `test_payload_daemon` (4). Root-caused + fixed the daemon segfault (a `unique_ptr` moved into `ControlLoop` was later dereferenced; the reference member is the valid handle). **Result: 38/38 ctest green and 95/95 Python green.**
  - **Next:** live commissioning of the payload check on the physical station (supervised; not automated here) — or the remaining queued §P7/§P8 live tests.
- **2026-09-01 (NZST)** — Rewrote **`Firmware/docs/post_homing_test_queue.md`** (the Phase-2 edition was stale: P6 said the payload check was a stub, P8 said tracking was "not built yet", and Phases 6–9 work was absent). Now a three-part runbook for the live station:
  - **Part 1 — ordered live queue P0–P13** (each tagged `[MOTOR]`/`[CAMERA]`/`[SW]`): P0 homing (gate) → P1 hold → P2 park → P3 re-home repeatability → P4 stale feedback (Brake) → P5 fault injection (Disable; last of the motor group) → **P6 in-loop payload check (real now, §27/§31.3, incl. mismatch→derate→clear)** → P7 live vision (camera-only) → **P8 live tracking (Phase 6 is implemented+tested on SimMotorBackend; one code prerequisite remains: controld has no vision wiring — S1)** → P9 installation orientation calibration (Phase 7; fiducial run → R_W_B → world-frame aim check) → P10 real-plant payload profiling/verification (Phase 9; `turret-payload` without `--sim`) → P11 systemd deployment (§52) → P12 live web end-to-end (webd ↔ real controld, §54.5 load, video orientation confirm) → P13 §54.4 HIL checklist + §55 acceptance-metrics report.
  - **Part 2 — "good to go" swap-in map (S1–S12):** every place a mock/guard/placeholder stands in for the real thing and the exact replacement: S1 controld vision wiring (the only code change — UDS client + `enable_tracking`), S2 `turret-payload` drop `--sim`, S3 visiond `--synthetic`→`--real`, S4 webd `FakeControld`→real daemon socket, S5 enable the systemd units, S6 SimMotorBackend suites→HIL counterparts, S7–S9 the missing `calibration/*.yaml` files, S10 `active_profile: conservative`→real profile, S11 `turret.yaml` placeholders (23 §58 values, travel bands, 0/0 park pose)→measured values, S12 video (real already, confirm only).
  - **Part 3 — placeholder registry (27 items, development order):** Phase 1 (comm-19 table, speed mode, bus error counters) → Phase 2 (§58 params, travel bands, park 0/0, §27 stub states [payload now real], homing-result persistence [deferred by design], vcan9) → Phase 3 (piecewise envelope) → Phase 4 (RPK/config JSON + license note, 3-class set/person-only, `--synthetic` default) → Phase 5 (distortion no-op, R_P_C identity, alpha-beta→Kalman path, bbox-centre anchor) → Phase 6 (vision wiring, systemd not run) → Phase 7 (`calibration/*.yaml` missing) → Phase 8 (FakeControld, orientation confirm) → Phase 9 (conservative profile, real-plant commissioning) → cross-cutting (§54.4 HIL, §55 report, §54.2 plant fidelity). Plus a "Resolved since the Phase-2 edition" section for audit.
  - **Findings that drove the rewrite:** production `main.cpp` never opens `/tmp/ota_vision.sock` / calls `enable_tracking` (tracking is live-testable only after S1); `Firmware/calibration/` does not exist (the three `turret.yaml` paths are placeholders); `turret-payload` real mode wired but never run; P7 still needs the picamera2 config JSON + detector RPK (not in repo).
