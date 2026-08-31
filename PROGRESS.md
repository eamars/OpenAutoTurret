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
| 2 | Homing and safety foundation | [~] | Config, logical coords, contact detector, per-endpoint homing FSM, full-axis homing (§23) all done+tested; safety envelope/supervisor/boot/park pending |
| 3 | Trajectory generator | [~] | Online jerk-limited generator + 4 tests done; stopping/stop-reachability + collision envelope pending |
| 4 | Vision daemon (Python) | [ ] | |
| 5 | Geometry and estimator (C++) | [ ] | |
| 6 | Closed-loop tracking | [ ] | |
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
- [ ] Multi-axis homing plan interpreter (§25, §25.1, §25.2): `home_endpoint` / `move` / `home_full_range` steps, clearance poses
- [ ] Soft limits + braking envelope (§18.1, §18.2): stop feasibility from trajectory model
- [ ] Safety supervisor (§38) with layered watchdogs (§39)
- [ ] Boot state machine (§27) incl. FAULT_LOCKED path
- [ ] Safe shutdown/park state machine (§33) incl. verification + dwell
- [~] Unit tests: contact detector (5), homing FSM (5), full-axis homing (5), trajectory (4) done; stop feasibility tests pending

## Phase 3 — Trajectory generator

- [x] Online jerk-limited generator (§17): velocity-cap + stop-distance profile, proportional vel loop (τ=0.2 s) + jerk clamp; 4 tests (commit ce1d405). Bang-bang variants were dead-ends (limit-cycling) — see log.
- [ ] Receding-horizon target updates without discontinuity (§17.3)
- [ ] Stopping trajectories + `verify_stop_reachability` (§17.2, §48), randomized unit tests
- [ ] Coupled collision envelope interface (§19): constant limits now, piecewise table/polygon later; path (not just endpoint) validation

## Phase 4 — Vision daemon (Python)

- [ ] `vision/visiond.py`: Picamera2 + IMX500, detection parse, `SensorTimestamp` attach, frame sequence
- [ ] Target association/selection (§12): class gate, confidence, IoU, continuity; one selected target
- [ ] `TargetMeasurement` publish over UDS `SOCK_SEQPACKET` (§6.1, §6.2)
- [ ] No CAN access, no motion logic in visiond

## Phase 5 — Geometry and estimator (C++)

- [ ] `geometry/camera_model.*`: intrinsics, distortion, pixel→ray (§10)
- [ ] `geometry/turret_kinematics.*`: frame convention (R_A_B), R_B_Y(q_yaw)·R_Y_P(q_pitch)·R_P_C chain, LOS→joint solve (§9, §14)
- [ ] Timestamp alignment: history interpolation at `sensor_timestamp`, timing-invalid flag (§11)
- [ ] `tracking/target_estimator.*`: const-ang-velocity Kalman/alpha-beta, confidence-aware prediction horizon (§13, §35)
- [ ] Replay tests with recorded measurements

## Phase 6 — Closed-loop tracking

- [ ] `control_loop.*` per §46 pseudocode (200 Hz, absolute-deadline sleep, no I/O/alloc in loop)
- [ ] Reference manager arbitration with priority (§16)
- [ ] Tracking state machine (§34): READY_HOLD/TRACKING/COASTING/BRAKE_TO_HOLD/LOST, config thresholds
- [ ] Search planner (§36, §49) and stationary hold (§37)
- [ ] Telemetry snapshot publisher 10–20 Hz (§6.3) + high-rate control log (§43.1) + event log (§43.3) + black-box ring (§43.4)
- [ ] controld main(): signal handling, boot FSM wiring, UNHOMED on restart (§52)
- [ ] systemd unit files (§52)

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
