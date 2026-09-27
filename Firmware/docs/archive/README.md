# Legacy document archive

These files predate the ADR directory convention. Their original contents and history are preserved; the folders classify the state described by each document, not a current release approval.

- [`implemented/`](implemented/) — software or procedures were implemented in the legacy system. Hardware qualification may still be missing, and some implementations belong to retired hardware.
- [`partially-implemented/`](partially-implemented/) — implementation or evidence exists, with explicit work or qualification still open.
- [`not-implemented/`](not-implemented/) — legacy design proposals whose full planned outcome was not implemented in that form.
- [`superseded/`](superseded/) — retired hardware, architecture, procedures, handoffs, or status snapshots. Consult current ADRs and `STATION_OPERATIONS.md` for current decisions and operation.

## Implemented

| Document | Classification and limit |
|---|---|
| [`implemented/architecture/open_auto_turret_v3_three_mode_target_tracking_architecture.md`](implemented/architecture/open_auto_turret_v3_three_mode_target_tracking_architecture.md) | Three-mode software architecture implemented in a prior release; historical baseline. |
| [`implemented/control/drive_current_friction_tuning.md`](implemented/control/drive_current_friction_tuning.md) | Tuning measurements behind legacy constants; measurements are hardware-specific. |
| [`implemented/control/motion_profiles.md`](implemented/control/motion_profiles.md) | Legacy service motion profile contract and configuration behavior. |
| [`implemented/control/open_auto_turret_v3_2_cybergear_tracking_control_hardening.md`](implemented/control/open_auto_turret_v3_2_cybergear_tracking_control_hardening.md) | Software hardening for the earlier CyberGear tracking configuration; not a current-hardware qualification. |
| [`implemented/control/roam_recovery_design.md`](implemented/control/roam_recovery_design.md) | Interrupted roam direction recovery implemented in the legacy controller. |
| [`implemented/design/open_auto_turret_v3_2_apache_hud_ui_revision.md`](implemented/design/open_auto_turret_v3_2_apache_hud_ui_revision.md) | HUD revision implemented in the legacy UI; visual sign-off remains a separate human decision. |
| [`implemented/hardware/CYBERGEAR_FIRMWARE_UPGRADE.md`](implemented/hardware/CYBERGEAR_FIRMWARE_UPGRADE.md) | Owner-reported update to 1.2.1.5 and subsequent register checks; the agent did not flash the motor and the bundle label is not an authenticated vendor manifest. |

## Partially implemented

| Document | Classification and limit |
|---|---|
| [`partially-implemented/acceptance/acceptance_signoff_v3_2_visual.md`](partially-implemented/acceptance/acceptance_signoff_v3_2_visual.md) | Unsigned legacy acceptance ledger; its own text records missing and invalid evidence. |
| [`partially-implemented/analysis/latency_bottleneck_analysis_2026_09_08.md`](partially-implemented/analysis/latency_bottleneck_analysis_2026_09_08.md) | Component and control latency analysis; optical end-to-end timing was not established. |
| [`partially-implemented/calibration/principal_point_method_2026-09-05_r72.md`](partially-implemented/calibration/principal_point_method_2026-09-05_r72.md) | Calibration method and old measurements; transfer to the current camera installation is unverified. |
| [`partially-implemented/commissioning/HARDWARE_COMMISSIONING_2026_09_26.md`](partially-implemented/commissioning/HARDWARE_COMMISSIONING_2026_09_26.md) | Mixed-hardware transport and motion evidence with remaining verification gaps. |
| [`partially-implemented/commissioning/HARDWARE_CONTINUATION_2026_09_26.md`](partially-implemented/commissioning/HARDWARE_CONTINUATION_2026_09_26.md) | Follow-up current-hardware evidence; not complete release acceptance. |
| [`partially-implemented/commissioning/IMU_COMMISSIONING_2026_09_27.md`](partially-implemented/commissioning/IMU_COMMISSIONING_2026_09_27.md) | Acquisition and tare observed; installation calibration and production fusion remain open. |
| [`partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md`](partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md) | Bounded yaw/pitch motion evidence with remaining release gates. |
| [`partially-implemented/control/configurable_alignment_design.md`](partially-implemented/control/configurable_alignment_design.md) | Software geometry verified in simulation; physical camera-to-bore alignment was not verified. |
| [`partially-implemented/control/control_loop_review_2026_09_06.md`](partially-implemented/control/control_loop_review_2026_09_06.md) | Legacy control-loop review and candidate changes; measurements concern earlier hardware. |
| [`partially-implemented/control/homing_failure_review_2026_09_08.md`](partially-implemented/control/homing_failure_review_2026_09_08.md) | Offline supervision work recorded; physical braking and setup motion remain unverified. |
| [`partially-implemented/control/response_tuning_followup_2026_09_08.md`](partially-implemented/control/response_tuning_followup_2026_09_08.md) | Response experiments and follow-up tuning; not current-hardware acceptance. |
| [`partially-implemented/handoffs/ARCHITECTURE_HANDOFF_2026_09_27.md`](partially-implemented/handoffs/ARCHITECTURE_HANDOFF_2026_09_27.md) | Dated architecture snapshot that informed ADR-001; use ADR-001 for the new development plan. |
| [`partially-implemented/hardware/HARDWARE_ADAPTATION_PLAN.md`](partially-implemented/hardware/HARDWARE_ADAPTATION_PLAN.md) | Mixed-hardware plan with implemented pieces and outstanding safety/release gates. |
| [`partially-implemented/hardware/HARDWARE_CURRENT.md`](partially-implemented/hardware/HARDWARE_CURRENT.md) | Dated hardware inventory snapshot, not a live status feed. |
| [`partially-implemented/imu/open_auto_turret_bno085_imu_expansion_v1_1.md`](partially-implemented/imu/open_auto_turret_bno085_imu_expansion_v1_1.md) | Earlier IMU proposal; data acquisition exists, while production calibration and fusion remain incomplete. |
| [`partially-implemented/validation/automatic_service_validation_2026_09_06.md`](partially-implemented/validation/automatic_service_validation_2026_09_06.md) | Service validation on the earlier mechanism; evidence does not transfer to the current hardware. |
| [`partially-implemented/vision/AI_HAT_PERCEPTION_PLAN.md`](partially-implemented/vision/AI_HAT_PERCEPTION_PLAN.md) | Some Hailo inference plumbing was probed; dual-camera fusion and target-quality acceptance remain open. |

## Not implemented in the planned form

| Document | Classification and limit |
|---|---|
| [`not-implemented/vision/open_auto_turret_perception_target_selection_architecture_v1.md`](not-implemented/vision/open_auto_turret_perception_target_selection_architecture_v1.md) | Legacy subsystem handover; parts were realized separately, but the complete proposed architecture is not the current system. |

## Superseded or retired

| Document | Why it is retained |
|---|---|
| [`superseded/analysis/tracking_instability_isolation_2026_09_06.md`](superseded/analysis/tracking_instability_isolation_2026_09_06.md) | Measurement report for the retired mechanism. |
| [`superseded/baselines/AS_BUILT_v1.md`](superseded/baselines/AS_BUILT_v1.md) | Historical v1 as-built snapshot. |
| [`superseded/commissioning/motion_boundary_review_2026_09_06.md`](superseded/commissioning/motion_boundary_review_2026_09_06.md) | Unloaded boundary trial on earlier hardware. |
| [`superseded/commissioning/travel_boundary_review_2026_09_06.md`](superseded/commissioning/travel_boundary_review_2026_09_06.md) | Loaded dual-CyberGear evidence; not current-hardware acceptance. |
| [`superseded/architecture/open_auto_turret_software_control_architecture_v1.md`](superseded/architecture/open_auto_turret_software_control_architecture_v1.md) | Frozen v1 specification superseded by later architecture; source references are preserved. |
| [`superseded/design/open_auto_turret_v3_reference_mock.html`](superseded/design/open_auto_turret_v3_reference_mock.html) | Historical UI mock. |
| [`superseded/design/open_auto_turret_v3_2_apache_hud_reference_mock.html`](superseded/design/open_auto_turret_v3_2_apache_hud_reference_mock.html) | Historical HUD mock. |
| [`superseded/handoffs/HANDOFF_2026-09-03.md`](superseded/handoffs/HANDOFF_2026-09-03.md) | Shift handoff superseded by later status and operation records. |
| [`superseded/handoffs/can_handover_architect.md`](superseded/handoffs/can_handover_architect.md) | Investigation handoff superseded by later CAN reports. |
| [`superseded/handoffs/implementation_takeover_2026_09_06.md`](superseded/handoffs/implementation_takeover_2026_09_06.md) | Dated implementation takeover snapshot. |
| [`superseded/handoffs/progress_before_v3.md`](superseded/handoffs/progress_before_v3.md) | Old phase tracker and session log. |
| [`superseded/hardware/RS485_CAN_HAT_SETUP.md`](superseded/hardware/RS485_CAN_HAT_SETUP.md) | Setup record for retired CAN hardware. |
| [`superseded/hardware/can_hardware_fault_report.md`](superseded/hardware/can_hardware_fault_report.md) | MCP2515 fault investigation; it does not diagnose the installed MCP2518FD HAT. |
| [`superseded/incidents/optimization_cycle_2026_09_08.md`](superseded/incidents/optimization_cycle_2026_09_08.md) | Endpoint incident and recommendations for the earlier mechanism. |
| [`superseded/operations/AI_CAMERA_SETUP.md`](superseded/operations/AI_CAMERA_SETUP.md) | Historical single-camera setup. |
| [`superseded/operations/monitored_restart_2026_09_08.md`](superseded/operations/monitored_restart_2026_09_08.md) | Dated restart review; current station operation is defined in `STATION_OPERATIONS.md`. |
| [`superseded/operations/park_home_fix_2026_09_09.md`](superseded/operations/park_home_fix_2026_09_09.md) | Park and recovery fix report for the retired hardware generation. |
| [`superseded/operations/STATION_RUNBOOK.md`](superseded/operations/STATION_RUNBOOK.md) | One-off deployment notes superseded by the root operating runbook. |
| [`superseded/operations/station_operations_dual_cybergear_2026_09_09.md`](superseded/operations/station_operations_dual_cybergear_2026_09_09.md) | Former runbook for the retired dual-CyberGear station. |
| [`superseded/operations/systemd_operations_v1.md`](superseded/operations/systemd_operations_v1.md) | Legacy service management notes. |
| [`superseded/research/research_can_bus_error_passive.md`](superseded/research/research_can_bus_error_passive.md) | Historical MCP2515 investigation and dead ends. |
| [`superseded/status/operator_status_v3_2_2026-09-05_r67.md`](superseded/status/operator_status_v3_2_2026-09-05_r67.md) | Dated operator status snapshot. |
| [`superseded/validation/post_homing_test_queue.md`](superseded/validation/post_homing_test_queue.md) | Retired-hardware test queue and raw session log. |
| [`superseded/validation/run_sheet_P8.md`](superseded/validation/run_sheet_P8.md) | Single supervised run sheet for the old setup. |
| [`superseded/vision/research_vision_readiness_p7.md`](superseded/vision/research_vision_readiness_p7.md) | Historical vision option study; source citations are retained for context, not as current hardware guidance. |
