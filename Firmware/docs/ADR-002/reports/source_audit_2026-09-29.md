# ADR-002 source audit — 2026-09-29

## Pinned baseline and scope

The audited baseline is branch `main`, commit `2e789c59dc9038228ac2aa91caab0f07d26d8137`. `git status --short` was empty at the start of this audit. The manifest now records that identity. This report checks the S01–S21 paths against that pinned source snapshot and compares them with the ADR and its selective source audit. It describes repository source/configuration only; it does not establish which commit or parameters are deployed, and it is not hardware validation.

The worktree is shared. A separate uncommitted edit to `gm6020_velocity.hpp` appeared during this audit; it was left untouched and is excluded from the baseline findings below. A source audit of the pinned commit should use `git show <commit>:<path>` where a concurrent edit exists.

## Current source and configuration facts

| Area | Pinned source value / behavior | ADR comparison |
|---|---|---|
| Hardware profile, yaw | `can0`, GM6020 ID 1, current mode, TX `0x1FE`, feedback `0x205`; Current Ring acknowledgment true; host limit `0.8 A`; PI `Kp=1.0 A/(rad/s)`, `Ki=0.6 A/rad`; raw temperature guard disabled (`0`) | Matches ADR-002. The gains/limit are configured commissioning values, not payload-qualified thermal limits. |
| Hardware profile, pitch | `can1`, CyberGear ID 127, declared position mode, configured current cap `5.0 A` | Matches the declared hardware configuration. |
| Host loop and service | `control.loop_hz=200`; `service_speed_control=true`; service speed `Kp=4.0`, `Ki=0.05`; position servo `Kp=4.0` | Matches the plan. Configured mode is not the runtime register mode. |
| Motion limits | Manual target: 20°/s, 15°/s², 60°/s³; manual max: 20°/s, 30°/s², 120°/s³. AUTO_ROAM target 10°/s, max 20°/s with the same acceleration/jerk caps. AUTO_TRACK target/max 20°/s and 30°/s², jerk 100°/s³. Values are independently declared for yaw and pitch. | Makes the ADR audit's general 15/60 and 20/30 observations contextual; AUTO_TRACK jerk is 100, not the manual/roam value 120. |
| Pitch runtime path | Service enables native CyberGear speed mode (`RunMode=2`); host computes position P plus velocity feed-forward. Speed setup writes and reads back `LimitCur`, neutral `SpdRef`, and mode before enabling. | Matches ADR's separation between declared `position` and runtime speed control. The source does not establish internal loop rate. |
| Pitch feedback cadence | A changed speed reference is sent as a command; steady-state register ping is age/interval gated (15 ms age, 20 ms minimum interval in the audited backend). | Consistent with the approximate 50 Hz steady-state request description; this is not a measurement of internal loop frequency. |
| Homing | Pitch full-range homing; approach in speed mode, backoff in position mode. `motion_checks_abort=false`, `mode_displacement_check=false`; `repeatability_retries=2`; adaptive current step/max are zero; pitch remains capped at 5 A. Yaw is continuous and not endpoint-homed. | Matches ADR's mode-switch and false-contact cautions. The configured disabled checks do not prove motion is harmless. |
| Payload | `active_profile: conservative`; `auto_verify: false`. Startup loads the profile and calls `set_payload_profile(...)` whose default is `commissioned=true`; the loaded profile can still supply motion v/a/j caps. It is an old CyberGear profile, not a qualified mixed-hardware result. | Matches the ADR warning: `auto_verify` does not disable load/application. Current source does not show hardware/mode/payload identity qualification for this startup trust path. |
| Yaw velocity estimator | PI differentiates encoder position per invocation and applies a 50 ms low-pass; the telemetry/control-loop estimate is a separate 100 ms estimate. A `dt > 20 ms` call invalidates the loop in the pinned implementation. | Matches the source audit. The two filters serve different consumers; do not add their time constants and call it one measured latency. |
| Guard and CAN health | `yaw_guard_loop` polls every 5 ms; no-progress uses a 5°/s demand and about 1.5 s without encoder progress. `yaw_stall_streak_` increments per guard tick while no-progress remains true, and the threshold leads to `Hold`/zero-current writes. Normal velocity commands can also write current. `buses_healthy()` rejects any nonzero cumulative RX-error or TX-failure counters. | Confirms F01/F04 in the source audit. These are source-level arbitration/health limitations, not evidence that a particular physical oscillation occurred. |
| Payload schema | Profile schema is v1; legacy gain fields are informational CyberGear gains. Runtime selection is marked uncommissioned, unlike the boot-time path. | Confirms F10 and that startup/runtime trust semantics differ. |

## S01–S21 path coverage

All manifest paths S01–S21 were checked in the pinned worktree. The table above records the configuration and operational paths S01–S03/S13–S14/S20–S21 and the main control claims. The implementation-symbol checks were:

| IDs | Symbols / facts checked |
|---|---|
| S04–S06 | `VelocityLoop::step/update_amps`, 50 ms estimate and 20 ms invalid threshold; `yaw_guard_loop`, `yaw_stall_streak_`, `send_yaw_zero_locked`, `command_yaw_velocity_locked`, cumulative `buses_healthy()` check; guard declarations/response. |
| S07/S09 | CyberGear `RunMode`, `SpdRef`, `LimitCur`, `Iqf` registers; speed-mode setup, configured/current mode separation, pitch current limit and setup readback. |
| S08/S16 | `ControlLoop::step`, distinct telemetry estimator, safety/watchdog, payload profile API default `commissioned=true`, runtime selection with false. |
| S10/S18 | `HomingController` phase changes and `jitter_suffix`; contact detector's movement history, dwell and effort conditions. |
| S11/S12 | Safety supervisor actions; GM protocol frame scaling and `kMaxContinuousA=1.62 A`. The latter is a source constant, not thermal qualification. |
| S15/S17/S19 | Schema-v1 payload type and informational gains; speed-servo position P/feed-forward, quiet/deadband and rate shaping; mixed-profile current/frame validation. |
| S20/S21 | Current runbook and docs map: operational cards are under `Firmware/docs/operations/`; ADRs belong in their own directory. No station operation was performed. |

## Differences and interpretation

The original ADR source-audit text says it had not yet fixed a local SHA. That statement is now superseded for this audit: the actual clean starting baseline was pinned above. The external source URLs in the manifest still point to mutable `ADR-001` branch content; the commit pin applies to this local audit.

The ADR's shorthand motion figures need mode context: the configured AUTO_TRACK jerk cap is 100°/s³, while manual and AUTO_ROAM use 120°/s³ as their maximum. The claimed ~50 Hz pitch feedback request cadence is an inference from the 20 ms minimum ping interval under steady state, not a measured runtime rate. The code's 1.62 A GM constant and configured 0.8 A yaw cap do not qualify current for continuous low-speed or stall operation.

The source confirms the guard tick-count and cumulative-counter concerns, but does not confirm that they caused any specific field symptom. Likewise, source/configuration cannot prove actual register mode, the active/deployed profile, electrical/thermal state, motion quality, brake performance, encoder freshness on a station, or internal motor-loop frequency.

## Offline tools

Used the project-local `.venv` (not added to version control):

```text
.venv\Scripts\python.exe -m unittest discover -s Firmware/docs/ADR-002/tests -v
Ran 45 tests — OK
.venv\Scripts\python.exe Firmware/docs/ADR-002/tools/control_math.py
Completed; output labeled ILLUSTRATIVE_ARITHMETIC_NOT_HARDWARE.
.venv\Scripts\python.exe Firmware/docs/ADR-002/tools/trace_metrics.py Firmware/docs/ADR-002/fixtures/synthetic_trace.csv --output <temporary report path>
Completed on the synthetic fixture.
```

These tests validate offline tool behavior and synthetic arithmetic/trace processing only. No C++ build, station command, CAN operation, motor movement, or hardware test was performed; all physical verification remains **NOT_RUN**.
