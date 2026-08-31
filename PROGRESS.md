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
| 9 | Payload profiling/tuning | [ ] | |

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

- [ ] `installation_pose.*`: `InstallationPoseProvider` interface + `FixedStoredPoseProvider` (§30)
- [ ] Fiducial (ChArUco/AprilTag) multi-frame visual calibration, outlier rejection, atomic commit (§29)
- [ ] World-frame telemetry

## Phase 8 — Web UI and diagnostics

- [ ] `web/webd` FastAPI: dashboard (§42.1), developer controls (§42.2), optional preview (§42.3)
- [ ] Hostname-based access only, no hard-coded IP (§53)
- [ ] UI load test: control p99 unaffected (§54.5)

## Phase 9 — Payload profiling/tuning

- [ ] Profiling utility (§44): step/triangle/brake/holding tests → rise/settle/overshoot/stop distance
- [ ] Payload profile storage + mismatch detection + derating (§31.3, §31.4)

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
