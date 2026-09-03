# Documentation Map

**Read [`AS_BUILT_v1.md`](AS_BUILT_v1.md) first.** It is the current record of what is
implemented, what was measured on the physical turret, and which defects are shipped.
Everything else in this directory is either the specification, a vendor manual, or an
operational how-to.

## The project's own documents

| Document | What it is | Status |
|---|---|---|
| [`AS_BUILT_v1.md`](AS_BUILT_v1.md) | Implemented features with their evidence, measured station numbers, shipped defects | **Authoritative for "what does the code do today"** |
| [`open_auto_turret_software_control_architecture_v1.md`](open_auto_turret_software_control_architecture_v1.md) | The v1 architecture spec (the §-numbers every source file cites) | Frozen reference. The next revision replaces it; `§` references in code point here until then |
| [`../../PROGRESS.md`](../../PROGRESS.md) | Phase-level status: what is coded, what has been verified on hardware | Tracker only — no feature detail lives there anymore |

## Operational how-tos

| Document | What it covers |
|---|---|
| [`AI_CAMERA_SETUP.md`](AI_CAMERA_SETUP.md) | IMX500 camera bring-up |
| [`RS485_CAN_HAT_SETUP.md`](RS485_CAN_HAT_SETUP.md) | The MCP2515/RS485 HAT bring-up. **Read with the fault report below** — that hardware is field-sick and the station runs on the yousee USB-CAN adapter |
| [`can_hardware_fault_report.md`](can_hardware_fault_report.md) | The `ERROR-PASSIVE` root-cause report (v2, corrected). Referenced at runtime by `tools/can_supervisor.py`, so it stays at this level rather than in the archive |
| [`research_vision_readiness_p7.md`](research_vision_readiness_p7.md) | Vision readiness assessment (option study behind the guarded camera path and the RPK/Hailo choice). Cited by `vision/frame_source.py`, `vision/simple_detector.py`, `tools/vision_probe.py` and `systemd/turret-vision.service`, so it stays at this level even though its conclusions are dated |
| [`drive_current_friction_tuning.md`](drive_current_friction_tuning.md) | The measurements behind shipped constants. Cited by `control/src/config/turret_config.cpp`, `calibration/contact_detector.hpp` and `control/tests/test_config.cpp` — move it and those comments point at nothing |

## Vendor references

Not project status, and not mine to summarise:
[`CyberGear_AI_Reference.md`](CyberGear_AI_Reference.md),
`CyberGear微电机使用说明书.pdf`,
[`BNO08X_AI_Reference.md`](BNO08X_AI_Reference.md),
[`SH2_AI_Reference.md`](SH2_AI_Reference.md),
[`SH2_SHTP_AI_Reference.md`](SH2_SHTP_AI_Reference.md).

Where the vendor manual and the measured station disagree, the measurements in
`AS_BUILT_v1.md` win, and both are named: the manual's ±4π feedback angle mapping is
wrong for these drives (±12.5 rad), and its speed mode does not move a loaded axis at
the commanded rate on default gains.

## `archive/` — history, kept on purpose

`docs/archive/` holds documents that were true at a moment and would mislead as
current statements. Nothing here was deleted: the measurements inside are the origin of
claims in `AS_BUILT_v1.md`, and the dead ends are the reason certain designs were
rejected.

| Document | Why it is archived |
|---|---|
| `post_homing_test_queue.md` | The P0–P13 live queue with 130 KB of run logs. Still the place to find *how* a measurement was taken; its statuses are superseded by `AS_BUILT_v1.md` |
| `progress_before_v3.md` | The six tracker items still open at cleanup, plus the dated session log including dead ends (bang-bang trajectory limit-cycling, speed mode on a loaded axis, comm 17/18 vs comm 19) |
| `HANDOFF_2026-09-03.md` | A shift-boundary handoff. Superseded by the documents above |
| `run_sheet_P8.md` | One supervised tracking run, pre-dating the cold-start search fix |
| `can_handover_architect.md`, `research_can_bus_error_passive.md` | Investigation writeups that led to `can_hardware_fault_report.md` v2 and the yousee adapter decision |
| `open_auto_turret_bno085_imu_expansion_v1_1.md` | An unimplemented IMU design proposal. Opened only if that work is picked up |

One exception, learned the hard way today: **a document that live code names is not archived**, however stale it is — three of the moved files turned out to be cited by running tools and source comments, and a pointer that resolves to nothing is how a future session re-investigates a problem somebody already solved. Dated rationale stays visible; it is labelled dated instead of being hidden.

**Rules for adding documents here.** One fact, one home: measured results go in
`AS_BUILT_v1.md`, requirements go in the architecture spec, procedures go in a how-to.
A run log that earns a permanent claim gets the claim extracted and the log archived.
When a document stops describing today, it moves to `archive/` with a row above saying
why — that is a cleanup, not a deletion.
