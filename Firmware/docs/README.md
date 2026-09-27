# Documentation Map

**For deployment/start/stop, read [`STATION_OPERATIONS.md`](STATION_OPERATIONS.md)
first.** The September 26 hardware refresh is verified in
[`HARDWARE_CURRENT.md`](HARDWARE_CURRENT.md). The station is stopped; the old
dual-CyberGear stack cannot operate the mixed-motor continuous-yaw mechanism.
Automatic roam/track remains the intended normal mode after adaptation.
September 3-9 loaded homing/tracking evidence belongs to the retired hardware.

## Hardware refresh and next implementation

| Document | Purpose |
|---|---|
| [`HARDWARE_CURRENT.md`](HARDWARE_CURRENT.md) | Hardware, live CAN identities, simultaneous camera capture, BNO085 probe and missing runtime prerequisites |
| [`GM6020_AI_Reference.md`](GM6020_AI_Reference.md) | Agent-readable protocol/firmware reference and official archived manual |
| [`HARDWARE_ADAPTATION_PLAN.md`](HARDWARE_ADAPTATION_PLAN.md) | Two buses, distinct motor drivers, continuous yaw, IMU observer, commissioning and release gates |
| [`HARDWARE_COMMISSIONING_2026_09_26.md`](HARDWARE_COMMISSIONING_2026_09_26.md) | Implemented transport, real yaw pulse/stop evidence, pitch register incompatibility and verification gaps |
| [`HARDWARE_CONTINUATION_2026_09_26.md`](HARDWARE_CONTINUATION_2026_09_26.md) | 5 A pitch limit, bounded yaw velocity experiments, Hailo installation and camera inference evidence |
| [`IMU_COMMISSIONING_2026_09_27.md`](IMU_COMMISSIONING_2026_09_27.md) | Working BNO085 acquisition/tare, paired yaw/pitch evidence and continuous energized pitch session |
| [`LARGE_MOTION_COMMISSIONING_2026_09_27.md`](LARGE_MOTION_COMMISSIONING_2026_09_27.md) | Direct-drive ±15° pitch and 30° yaw commissioning, IMU comparisons and remaining release gates |
| [`CYBERGEAR_FIRMWARE_UPGRADE.md`](CYBERGEAR_FIRMWARE_UPGRADE.md) | Identified firmware artifact, supported vendor update connection and post-update checks |
| [`AI_HAT_PERCEPTION_PLAN.md`](AI_HAT_PERCEPTION_PLAN.md) | Hailo, person/head detection, selected-person tracking and dual-camera/IMU evaluation |

## The project's own documents

All September 3-9 measurement/status rows below concern the earlier hardware.
Their software findings remain useful, but their physical approvals do not carry
over to this installation.

| Document | What it is | Status |
|---|---|---|
| [`STATION_OPERATIONS.md`](STATION_OPERATIONS.md) | Current stopped-state restrictions, inspect/stop and future deployment gates | **Current operating procedure** |
| [`homing_failure_review_2026_09_08.md`](homing_failure_review_2026_09_08.md) | Both-axis homing supervision, unknown load direction, clearance and failure coverage | **Software verified offline; physical braking/setup motion unverified** |
| [`monitored_restart_2026_09_08.md`](monitored_restart_2026_09_08.md) | Restart after operator clearance, pitch movement during mode setup and added pre-enable position check | **Station stopped again; torque-off support unresolved** |
| [`optimization_cycle_2026_09_08.md`](optimization_cycle_2026_09_08.md) | Endpoint incident, latched parking supervision, optimization trials and hardware decision | **Physical station stopped; corrected release requires secured physical verification** |
| [`latency_bottleneck_analysis_2026_09_08.md`](latency_bottleneck_analysis_2026_09_08.md) | Closed-loop latency, target-free physical response, camera timing and AI hardware priorities | Measured component boundaries; optical end-to-end timing remains unverified |
| [`travel_boundary_review_2026_09_06.md`](travel_boundary_review_2026_09_06.md) | Loaded homing, tracking response, travel limits and HUD verification | Historical dual-CyberGear evidence, not current hardware acceptance |
| [`AS_BUILT_v1.md`](AS_BUILT_v1.md) | Features and evidence as of September 3 | Historical snapshot |
| [`configurable_alignment_design.md`](configurable_alignment_design.md) | Startup aim-point and camera-to-bore configuration, tuning and offline verification | Implemented; native observations and simulated motors verified, physical alignment unverified |
| [`open_auto_turret_software_control_architecture_v1.md`](open_auto_turret_software_control_architecture_v1.md) | The v1 architecture spec (the §-numbers every source file cites) | Frozen reference. The next revision replaces it; `§` references in code point here until then |
| [`../../PROGRESS.md`](../../PROGRESS.md) | Phase-level status: what is coded, what has been verified on hardware | Tracker only — no feature detail lives there anymore |

## Operational how-tos

| Document | What it covers |
|---|---|
| [`AI_CAMERA_SETUP.md`](AI_CAMERA_SETUP.md) | Historical single-IMX500 bring-up; current dual-camera evidence is in the inventory |
| [`RS485_CAN_HAT_SETUP.md`](RS485_CAN_HAT_SETUP.md) | Historical MCP2515/RS485 HAT setup; that HAT and the subsequent USB adapter are retired |
| [`can_hardware_fault_report.md`](can_hardware_fault_report.md) | Historical MCP2515 investigation, root cause unresolved; does not diagnose the new MCP2518FD HAT. Kept because code references it |
| [`research_vision_readiness_p7.md`](research_vision_readiness_p7.md) | Vision readiness assessment (option study behind the guarded camera path and the RPK/Hailo choice). Cited by `vision/frame_source.py`, `vision/simple_detector.py`, `tools/vision_probe.py` and `systemd/turret-vision.service`, so it stays at this level even though its conclusions are dated |
| [`drive_current_friction_tuning.md`](drive_current_friction_tuning.md) | The measurements behind shipped constants. Cited by `control/src/config/turret_config.cpp`, `calibration/contact_detector.hpp` and `control/tests/test_config.cpp` — move it and those comments point at nothing |

## Vendor references

Vendor references describe protocols, not current station acceptance:
[`GM6020_AI_Reference.md`](GM6020_AI_Reference.md),
[`CyberGear_AI_Reference.md`](CyberGear_AI_Reference.md),
`CyberGear微电机使用说明书.pdf`,
[`BNO08X_AI_Reference.md`](BNO08X_AI_Reference.md),
[`SH2_AI_Reference.md`](SH2_AI_Reference.md),
[`SH2_SHTP_AI_Reference.md`](SH2_SHTP_AI_Reference.md).

Where manuals and measurements disagree, preserve both and identify the tested
device/firmware. Earlier CyberGear feedback mapping and loaded-speed observations
must be revalidated for the motor now at ID `0x7F`; they are not automatically
facts about a newly installed drive.

## `archive/` — history, kept on purpose

`docs/archive/` holds documents that were true at a moment and would mislead as
current statements. Nothing here was deleted: the measurements inside are the origin of
claims in `AS_BUILT_v1.md`, and the dead ends are the reason certain designs were
rejected.

| Document | Why it is archived |
|---|---|
| [`station_operations_dual_cybergear_2026_09_09.md`](archive/station_operations_dual_cybergear_2026_09_09.md) | Former operating runbook; yaw endpoint homing, CyberGear recovery and parking approvals are superseded by the hardware refresh |
| `post_homing_test_queue.md` | The P0–P13 live queue with 130 KB of run logs. Still the place to find *how* a measurement was taken; its statuses are superseded by `AS_BUILT_v1.md` |
| `progress_before_v3.md` | The six tracker items still open at cleanup, plus the dated session log including dead ends (bang-bang trajectory limit-cycling, speed mode on a loaded axis, comm 17/18 vs comm 19) |
| `HANDOFF_2026-09-03.md` | A shift-boundary handoff. Superseded by the documents above |
| `run_sheet_P8.md` | One supervised tracking run, pre-dating the cold-start search fix |
| `can_handover_architect.md`, `research_can_bus_error_passive.md` | Investigation writeups that led to `can_hardware_fault_report.md` v2 and the yousee adapter decision |
| [`open_auto_turret_bno085_imu_expansion_v1_1.md`](archive/open_auto_turret_bno085_imu_expansion_v1_1.md) | Earlier IMU design input. BNO085 is now installed and reports data; production integration/calibration remain pending in the current hardware plan |

One exception, learned the hard way today: **a document that live code names is not archived**, however stale it is — three of the moved files turned out to be cited by running tools and source comments, and a pointer that resolves to nothing is how a future session re-investigates a problem somebody already solved. Dated rationale stays visible; it is labelled dated instead of being hidden.

**Rules for adding documents here.** One fact, one home: measured results go in
dated validation reports, requirements go in the architecture spec, and current
operating procedures go in `STATION_OPERATIONS.md`.
A run log that earns a permanent claim gets the claim extracted and the log archived.
When a document stops describing today, it moves to `archive/` with a row above saying
why — that is a cleanup, not a deletion.
