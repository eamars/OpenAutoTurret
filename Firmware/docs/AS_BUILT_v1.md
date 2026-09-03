# OpenAutoTurret — As-Built on the v1 Architecture

**Snapshot: 2026-09-03.** What is implemented, how it was verified, and what was
measured on the real station — the starting point for the next architecture revision.
It says what was measured, not what was intended, and where something is unproven it
says so in words.

* **Not a plan.** No future work is described here. `docs/README.md` says where the
  open items live and why they are not in this file.
* **Not a spec.** Requirements stay in
  [`open_auto_turret_software_control_architecture_v1.md`](open_auto_turret_software_control_architecture_v1.md).
  Every `§` below is a section of that document.
* Claims marked **live** were observed on the physical turret; claims marked **sim**
  are proven against the simulated plant only and have never moved real metal.

## Verification baseline at this snapshot

| Suite | Result | What it does *not* cover |
|---|---|---|
| C++ unit + integration (`ctest`) | **43 / 43** | No CAN, no motor driver, no camera. Every motion claim runs on `SimMotorBackend` — a first-order plant with end stops, no inertia or backlash. |
| Python (`pytest`, repo root) | **221 / 221** | Camera paths run against a fake `picamera2`; the tests that need a real IMX500 are guarded and skip. |
| Dashboard script (`node --check`) | in `web/webd/tests/test_dashboard_js_parses.py` | Proves the page parses and that `connect()` is still wired up. Not that the UI is *correct* — that gap is in the defects list, because it cost an operator an afternoon. |

## Implemented

### Phase 1 — CAN / motor layer
SocketCAN raw transport with RX thread, RAW filters, non-blocking TX, self-pipe
shutdown (`can/socketcan_bus.{hpp,cpp}`) · CyberGear protocol encode/decode —
discovery, feedback, enable/stop/set-zero, register table `0x7005..0x7020`, MIT
(`can/cybergear_protocol.{hpp,cpp}`, incl. the ±12.5 rad feedback mapping pinned in
`control/tests/test_protocol.cpp`) · per-axis latest feedback in a `SeqLock` plus a
~1 s history ring with interpolation (`can/cybergear_axis.hpp`) · common layer:
state enums (App. A), monotonic time, `MotorStateHistory`, `TimingStats`
(`common/`) · `CyberGearSystem` — open/close, per-axis discovery, bounded register
read, fire-and-forget TX, feedback fan-out (`can/cybergear_system.cpp`) · `turret-can` commissioning CLI
(discover / read regs / enable / live feedback / jog) · C++20 CMake build against
yaml-cpp + spdlog + GTest · CAN link health: adapter kind, up/down, rx/tx frames,
error frames, TX failures and last-feedback age, in the telemetry snapshot and on the
dashboard, and reported as *deltas* by `tools/acceptance_metrics.py`.

### Phase 2 — Homing and the safety foundation
Versioned YAML config loader + validation, §40 (`config/turret_config.{hpp,cpp}`) —
missing/`TBD` keys fall back to built-in conservative defaults with a warning; bad
schema version, out-of-range, `min >= max`, `dir_sign ∉ {±1}`, duplicate CAN ids are
hard errors · logical joint coordinates, §24 (`common/logical_coordinates.hpp`) ·
contact detector, §21 (`calibration/contact_detector.hpp`) — filtered
velocity/progress/signed effort, dwell, independent hard abort · precision homing FSM
per endpoint, §22 (`calibration/homing_controller.{hpp,cpp}`) — coarse → back-off →
settle → fine → repeatability verify, §26 failure semantics · full-axis homing and
travel-band validation, §23 (`calibration/full_axis_homing.{hpp,cpp}`) — home A,
traverse, home B, validate span, build the model · multi-axis plan interpreter, §25
(`calibration/homing_plan.{hpp,cpp}`) · soft limits and the braking envelope, §18
(`control/safety_envelope.{hpp,cpp}`) — stop feasibility from the same stop model the
generator uses, and independent of it, §18.3 · safety supervisor with layered
watchdogs, §38/§39 (`control/safety_supervisor.hpp`) — a pure decision gate that owns
the envelope, plus stale-feedback / over-temperature / deadline layers and the homing
gate · boot FSM incl. `FAULT_LOCKED`, §27 (`control/boot_fsm.{hpp,cpp}`) · park, §33
(`calibration/park_controller.{hpp,cpp}`) — ordered axes, position+velocity verify,
dwell, de-energize; refuses a park pose outside the soft limits · `controld`:
`MotorBackend` + `SimMotorBackend` + `CanMotorBackend` (§46 position-mode recipe) +
the 200 Hz `ControlLoop` + `main.cpp` (config → boot → home → hold → park,
SIGINT/SIGTERM parks cleanly, never SIGKILL-by-omission).

### Phase 3 — Trajectory
Online jerk-limited generator, §17 (`control/trajectory_generator.{hpp,cpp}`) ·
receding-horizon retargeting with C0/C1 continuity through reversals, §17.3 ·
stopping trajectories and `verify_stop_reachability`, §17.2/§48, incl. a 2000-sample
fixed-seed randomized test · `CollisionEnvelope` interface +
`RectangularCollisionEnvelope` (`control/collision_envelope.hpp`), §19, validating
the **whole path** not just the endpoint. Bang-bang variants were tried and are dead ends (limit cycling) — recorded
in the archived session log so the door is not pushed open again.

### Phase 4 — Vision daemon (Python)
`vision/visiond.py` (guarded real IMX500 + deterministic synthetic source) ·
association and selection, §12 (`vision/target_selector.py`) — class gate, confidence
threshold, IoU + centroid proximity, track age, reacquisition; exactly one selected
target · `TargetMeasurement` over UDS `SOCK_SEQPACKET`, §6.1/§6.2 (`vision/ipc.py`) —
the 58-byte LE wire, cross-checked C++↔Python in both directions · no CAN access and
no motion logic in the vision process, by construction.

**Cross-language coupling to know before changing anything:** `vision/protocol.py` and
`tracking/target_measurement.hpp` encode the *same* 58-byte struct in two languages.
There is no shared schema file, so a field added on one side is silently wrong on the
other; the round-trip tests are the only thing standing between the two.

### Phase 5 — Geometry and estimator
Camera intrinsics + projective pixel↔ray, §10 (`geometry/camera_model.*`), OpenCV
frame, distortion a named no-op hook · turret kinematics with **configurable** `R_P_C`
and LOS → (az, el), §9.2/§10.3 (`geometry/turret_kinematics.*`) · timestamp alignment
by interpolating motor history at the frame's capture time, §11 — a miss returns
false and the measurement is timing-invalid, never silently replaced with a newer
pose · alpha-beta estimator on base-frame LOS with wrap-safe azimuth + forward
prediction to actuation time, §13 (`tracking/target_estimator.*`), covariance Kalman
documented as the upgrade path · replay tests over recorded measurements, §54.3.

### Phase 6 — Closed-loop tracking
`ControlLoop` per §46 · reference arbitration `TRACKING > SEARCH > HOLD` with
confidence-scaled `v_max`, §16 (`control/reference_manager.hpp`) · tracking FSM
§34/§35 (`tracking/tracking_state_machine.hpp`) incl. **cold-start search**: a
station that has *never* seen a target enters SEARCH after the same grace a lost
target gets, because the sweep exists precisely for the case where nothing appears ·
search planner and stationary hold, §36/§37/§49 (`control/search_planner.hpp`) ·
LOS→joint solver, §14 (`geometry/los_joint_solver.hpp`) · telemetry snapshot 10–20 Hz
§6.3 + event log §43.3 + black-box ring §43.4 (`telemetry/telemetry.hpp`) ·
closed-loop integration test through the whole stack on the simulated plant
(`control/tests/test_tracking_integration.cpp`: tracks a rotating target,
loss→coast→brake→hold, search sweep, soft-limit containment, fault→safe stop) · runtime `enable_search` / `disable_search` that take effect
immediately in both directions, and refuse to be inert.

### Phase 7 — Installation orientation
`InstallationPoseProvider` + `FixedStoredPoseProvider`, §30
(`calibration/installation_pose.hpp`) · ChArUco multi-frame visual calibration with
outlier rejection and atomic commit, §29 (`vision/installation_calibration.py`) ·
world-frame telemetry §29/§30 (`calibration/world_frame_telemetry.hpp`) · `controld`
loads the stored pose at boot (`main.cpp`, `FixedStoredPoseProvider`), reporting
*uncalibrated* when the file is absent rather than assuming a level base · the
C++/Python format mismatch that made Python-written calibration files unreadable by
`controld` is fixed (see the archived queue, P9 (d)).

### Phase 8 — Web UI and diagnostics
`web/webd` FastAPI: dashboard panels System / Vision / Target / Yaw-pitch /
Calibration / Video / CAN, §42.1; developer controls §42.2 · **real IMX500 MJPEG
video** on a separate low-priority path, §42.3, opened on demand and released when
off (`web/webd/video.py`, panels in `web/webd/dashboard.py`, routes in
`web/webd/app.py`) · image corrections applied *at the source* so preview and
vision see the same pixels (`common/image_corrections.py`) · controld-side web server
§5.3/§6.1/§6.3 (`control/src/web/web_server.hpp`), JSON over UDS, never opens the CAN
bus · command validation gate §42.2 (`web/command_validation.hpp`), validated on the
web thread and executed on the control thread · hostname-based access, no hard-coded
IP, §53 (`web/webd/config.py`) + env knobs · UI load test §54.5
(`control/tests/test_web_load.cpp`) · connection supervision in the dashboard
(reconnect watchdog, close-code reporting, re-request of the video stream) · systemd
unit templates §52 (`Firmware/systemd/`) — reviewed and corrected, **not installed,
not enabled**.

### Phase 9 — Payload profiling
The §44 battery (±step / triangle / multi-speed brake / holding effort) over any
`MotorBackend` with an injectable clock (`payload/payload_profiler.{hpp,cpp}`),
response metrics `analyze_step`/`analyze_brake`/`rms_tracking_error`
(`payload/response_metrics.hpp`), `turret-payload` CLI (`tools/turret_payload.cpp`) with `--sim` rehearsal ·
schema-v1 profile store, atomic tmp+fsync+rename with `.prev` backup
(`payload/payload_profile.{hpp,cpp}`) · verifier with explicit tolerances, and only a
`Mismatch` derates (§31.3/§31.4) · in-loop payload check in `Phase::PayloadCheck`,
one axis at a time with the amplitude clamped inside the region, auto-verified once
after reaching Hold per §27 and on the manual command · derating actually lowers
`hold_speed_effective()`, ready-pose and test-motion limits and tracking `v_max`, with
`SafetySupervisor` still holding authority · four telemetry fields + a dashboard
Payload panel.

### Cross-cutting that is genuinely done
Simulated axis plant for state-machine tests, §54.2 = `SimMotorBackend` · replay
testing §54.3 · UI-load testing §54.5 · §55 acceptance-metric extraction from logs
(`tools/acceptance_metrics.py`: timing, CAN deltas, homing spans, motion) · station
staging/verification tooling (`tools/install_station.py`, `tools/webd_rehearsal.py`) ·
`tools/fake_vision.py`, a synthetic target publisher on the real wire format, for
exercising acquire/coast/lose/reacquire on demand — and explicitly **not** for
testing roaming.

## Measured on the real station

Dated, because a number without a date and a rig is folklore. Full logs are in
`docs/archive/` (station paths in the queue's P-runs); these are the ones a new
architecture revision has to design around.

**Transport and drives.**
* Feedback is **request/response, not free-running**: the drive emits no periodic
  frame by default; every command (MIT / enable / stop / reg write) is answered by
  one feedback frame. The 200 Hz loop is fire-and-forget TX + RX thread, confirmed 1:1
  at 200 Hz on both motors.
* Feedback angle uses the **±12.5 rad** mapping, not the manual's ±4π — decoding with
  ±12.5 tracks `0x7019 mechPos` to <0.001 rad.
* **Stationary velocity feedback is quantization-noise dominated**, ±0.15 rad/s
  (±9 °/s) at a held position while position is stable to 0.022°. Any "is it stopped"
  logic must threshold above ~0.2 rad/s.
* **Speed mode does not drive the loaded axis** at the commanded rate on default
  gains (0.2 rad/s command → ~0.002 rad/s of motion, ~1%), so commissioning jogs and
  the control loop use position / MIT modes. This is a *drive configuration* gap, not
  a software one, and it constrains any plan that wants speed-mode homing.
* Registers above the `0x7005..0x7020` runtime table (`0x1000/0x2000/0x3000`) read
  stale or garbage through comm 17/18; they need comm 19, which is undocumented.
* Primary bus in service is the **yousee USB-CAN** adapter (`/dev/ttyUSB0`, CH341,
  921600 UART carrying 1 Mbit/s CAN), nodes `0x64`/`0x65`, `direction_sign` pitch +1 /
  yaw −1. The **MCP2515 HAT is field-sick** (`ERROR-PASSIVE`, witness-only role) and
  is the only adapter that exposes kernel bus state; the yousee adapter reports
  `Unknown`, which is *absence, not health*. Details:
  [`can_hardware_fault_report.md`](can_hardware_fault_report.md).

**Homing, live (2026-09-03 11:30, first live homing).** 106 s end to end · pitch
travel **79.9°** (independently confirming the ~80° the config claims) · yaw
**352.8°** to a real stop on a ~360° continuous axis · temperatures +1.4 K on each
drive · peak torque pitch 2.13 N·m / yaw 0.89 N·m.

**Roaming, live (2026-09-03 12:05, §36 cold start).** With **no vision source of any
kind** and no target ever seen: `track_state=search` **1.5 s** after `start_tracking`
· swept yaw **104.4 … 193.2°**, the configured span centred on the ready yaw and
unclipped by the soft limits · **actual sweep speed 9.8 °/s on all four legs** against
the 10 °/s cap (`min(ref.v_max, envelope)` holds) · sweep bounds repeated within
**0.4°** · `disable_search` ended the sweep in **0.5 s** and the arm returned to the
ready pose · 0 CAN error frames, 0 TX failures.

**Loop timing, live.** p50 5.06 ms / p99 5.09 ms at 200 Hz, i.e. the control period
itself is comfortable — but see the first defect below.

## Defects present in shipped code

Named here so v3 does not inherit them as assumptions.

1. **Mode-entry stalls the control loop.** 28 cycles of 109–113 ms (one 224.6 ms)
   during homing, each answered by a `BRAKE` from the supervisor. Cause is pinned:
   `kRecipeDelayMs = 50` with **two** `sleep_for` calls inside
   `enter_speed_mode` / `enter_position_mode`, invoked from the control thread at
   every homing stage change. §46 forbids blocking I/O and sleeps in the loop. Fix
   direction (deliberately not done mid-run): move mode entry off the control thread,
   and give the loop a bounded expected-gap token instead of letting the watchdog read
   the self-inflicted gap as stale feedback.
2. **The dashboard's yaw-speed column is not the axis speed.** During a 10 °/s sweep,
   telemetry `v_yaw_rad_s` read median 9.6 but p90 17.2 and **max 28.0 °/s**, while the
   position trace gives 9.8 °/s on every leg. The axis was obedient; the *number the
   operator reads* spikes ~2.8× at turnarounds (the drive's velocity estimate through
   a reversal). This is not cosmetic: it invites the wrong diagnosis and trains anyone
   watching to wave off an overspeed that one day will be real. Either derive the
   displayed velocity from position over a short window or carry both — do not ship a
   number that is neither.
3. **No §43.1 high-rate control-log writer.** `ControlLogRecord` exists as a struct;
   nothing persists it. The 10 ms `motion` log lines are emitted during homing only,
   so a hold/search excursion can only be reconstructed at the 15 Hz publish rate.
   Several otherwise-obvious measurements (overspeed, the overshoot shape at a
   reversal) are unresolvable today because of this.
4. **The dashboard's truth was verified by the wrong instrument once already.** A
   parse error shipped in the inline script (`{-1: …}` is not a JS property name), so
   the page rendered but never opened `/ws`; the daemon was healthy and the operator
   saw nothing. Python and C++ tests could not see it, because none of them looked at
   the JavaScript. `node --check` on the extracted script is now a test — and it
   skipped loudly rather than passing when `node` was absent, because a guard that
   silently disappears is worse than no guard.
5. **Calibration persistence is atomic but not identified.** `installation_pose.hpp`
   writes tmp+rename, and the payload store carries a schema version with a `.prev`
   backup; neither records hardware identity or provenance timestamps, and there is no
   last-known-good fallback (§28/§41). Two stations can therefore swap pose files
   without anything noticing.
6. **`NOT MEASURED` is still the honest answer for line-of-sight tracking error.** No
   field exists for it anywhere in the telemetry schema, so no claim about tracking
   accuracy on real hardware is supportable today — on-screen or off.

## Deliberately not deployed

systemd units are templates: `turret-web.service` carries `Wants=turret-control`, so
enabling it would home the turret unattended. Nothing on this station is
`systemctl enable`d, and the deployment path is `tools/install_station.py` plus a
supervised manual start.

## What was open at cleanup, and where it went

Six tracker items were unchecked when the doc space was cleaned
(`docs/archive/progress_before_v3.md`), plus the P0–P13 live queue with its run logs,
the handoff and the run sheets (`docs/archive/`). Two of the unchecked items were
actually finished during 2026-09-03 and are recorded above rather than left as
checkboxes: bus error counters are surfaced (with the yousee adapter's honest
limitation), and §55 metric extraction exists as `tools/acceptance_metrics.py`. A
third, §54.2's simulated axis plant, is satisfied by `SimMotorBackend` — first-order
plus end stops, with no inertia or backlash modelled, so it satisfies the item and
should not be mistaken for a fidelity claim.
