# CyberGear firmware upgrade runbook

**September 27 update:** the owner reports upgrading the pitch CyberGear to
1.2.1.5. The subsequent agent probe matched UID `7216313130333105`, read valid
`MechPos=-0.710777 rad` (status 0), and reapplied/read back 5 A three times with
disabled, fault-free feedback. The version string is owner-reported; register
compatibility is independently verified. The agent did not perform the flash.
No repeat upgrade is required. The procedure below records the pre-upgrade
assessment and reference workflow for any future servicing.

This runbook covers the documented Windows debugger route. The Pi's SocketCAN
interface has no documented flashing procedure in this bundle, regardless of
whether the link is UP or DOWN. Before the owner's upgrade, compatible USB-CAN
availability and the live firmware version had not been established remotely.

## Candidate image and provenance

The locally supplied vendor bundle contains this application image:

```text
C:\workspace\CyberGear微电机\电机固件包（增加可读参数，修复漏帧问题，增加修改波特率功能，版本号1.2.1.5）\MCU_Motor_APP_V1.2.1.5_20231122.bin
Size:   49,348 bytes
SHA256: 3A3F767A87B64DB0F7C341E93209379A69426B7D630E0520300D341B84F77601
Label:  application firmware 1.2.1.5, dated 2023-11-22
```

The filename and enclosing folder identify this as the candidate that adds readable parameters and CAN-bitrate modification. The supplied Xiaomi manual says firmware 1.2.1.5 supports bitrate modification and reading `0x7019..0x7020`. The digest fingerprints the local file; Xiaomi does not publish a matching checksum or signed manifest in the sources checked, and the binary contains no readable version string. Treat its precise version and authenticity as **bundle-labeled, not independently authenticated**. Do not substitute the separate v1.2.0.0 APP image or v1.2.0.0 FACTORY/ST-Link image.

The image remains in the user-supplied bundle outside this repository. Do not copy firmware, debugger executables, credentials, or runtime data into Git.

## Supported connection and upgrade route

The Xiaomi [CyberGear user manual](../../../references/cybergear/CyberGear%E5%BE%AE%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf) describes the debugger connection through a CAN-to-USB adapter with the CH340 driver installed and the adapter in its default AT mode (PDF p. 12). Use the Windows CyberGear debugger from the supplied bundle, `C:\workspace\CyberGear微电机\上位机软件\CyberGear调试器\CyberGear调试器_20231101.exe`, with that supported adapter. The controller bus must be configured for the motor's current CAN bitrate; this installation's expected bitrate is 1 Mbps. The manual's upgrade sequence is **Device → Upgrade → select `.bin` → confirm → wait for completion → motor automatically restarts** (PDF p. 20). The manual specifies a 24 V motor supply (PDF pp. 6-8).

The manual does not document an image-transfer command or firmware-flashing protocol on raw CAN. The debugger's AT-mode USB-CAN serial framing is adapter-specific, not SocketCAN framing. There is no documented SocketCAN/Raspberry Pi flasher for the current pitch connection on `mcp251xfd` `can1`; do not invent update frames. Bringing the Pi link UP does not make it a supported firmware-update transport.

## Stop and protect the mechanism

Before connecting the debugger:

1. Stop the station through `Firmware/scripts/run_application.sh stop`. Confirm the controller is not issuing commands.
2. Mechanically support the gravity-loaded pitch assembly independently of the motor. Keep the support in place through the update, automatic restart, all checks, and any recovery decision. Do not rely on firmware, a motor stop frame, or holding torque to support the axis.
3. Keep the pitch motor disabled/stopped. Do not clear faults, change mode, set zero, calibrate the encoder, or send motion commands as part of an upgrade attempt.
4. Confirm the compatible CH340/AT-mode CAN-to-USB adapter is actually available, the selected COM port is correct, the adapter CAN rate is 1 Mbps, wiring and 24 V supply are sound, and no other application owns the bus.

If any item is unknown, stop before starting the upgrade. The Pi has verified the expected target UID, but the debugger must independently identify that same motor before flashing. Adapter availability and the live firmware version remain unresolved.

## Read and save a baseline before flashing

With the mechanism supported and motor stopped, use the debugger's read/upload/export facilities to save the current parameter table and record:

- Motor 64-bit MCU UID and CAN ID.
- `AppCodeVersion` (`0x1003`), `AppGitVersion` (`0x1004`), and boot-code version (`0x1000`).
- `MechPos_init` (`0x2006`), `CAN_ID` (`0x200A`), `CAN_MASTER` (`0x200B`), `CAN_TIMEOUT` (`0x200C`), and the runtime values for `run_mode` (`0x7005`) and `limit_cur` (`0x7018`).
- Current pitch position/status and the existing encoder calibration state, if the debugger reports them.

Do not overwrite the original export. Xiaomi's manual documents parameter upload/export and provides example version fields, but its sample version strings are old (`0.1.5`); the live target's values are what matter. If the motor is not positively identified, the reads are malformed, or the baseline cannot be saved, do not flash.

## Version and identity gates

The installed pitch motor must identify as UID **`0x7216313130333105`** both before and after the upgrade. A different UID means the wrong target is selected: stop immediately and do not write firmware or parameters.

The pre-flash application version is presently unknown. Do not infer it from the manual or from another motor. Record the live `AppCodeVersion` before flashing. The selected local image is labeled v1.2.1.5; after its automatic restart, independently read `AppCodeVersion` again. If the debugger cannot read it, it is not exactly `1.2.1.5`, or the motor does not return with the same UID and CAN ID, keep the mechanism supported and do not enable or move it.

## Post-flash compatibility checks, before any motion

After restart, keep pitch supported and motor stopped. Reconnect at 1 Mbps and require all of the following before considering a later motion test:

1. UID is exactly `0x7216313130333105`; CAN ID and host ID match the saved baseline.
2. `AppCodeVersion` reads exactly `1.2.1.5` and the debugger reconnects normally.
3. Read `MechPos` (`0x7019`), `run_mode` (`0x7005`), and `limit_cur` (`0x7018`) through the documented parameter interface. Require valid successful responses and plausible decoded values. Firmware 1.2.1.5 is documented as supporting reads of `0x7019..0x7020` (manual PDF p. 26). Reject malformed/status-bearing responses; do not reuse stale payload bytes as position.
4. Recheck the saved CAN ID, host ID, current CAN bitrate, encoder/calibration status, and mechanical-zero/reference fields. Do not assume that an upgrade preserves volatile state, zero, calibration, or parameters unless the live readback confirms it.
5. Set runtime `limit_cur` (`0x7018`) to **5.0 A** while still stopped, then read it back and require exactly 5.0 A before any motion. This is a volatile parameter write (`COMM_TYPE_18`); a later reboot/power cycle can lose it, so it must be re-read after every restart and before motion. The documented `limit_cur` applies to speed/position modes; it is not a universal current ceiling for every control mode. Do not enable current or motion mode on the assumption that this setting bounds it.

The commissioning record documents an earlier `MechPos` read with nonzero status and stale payload, and the exact firmware revision was not known then. The firmware upgrade is not considered successful for control use until the `MechPos`, `run_mode`, and `limit_cur` checks above all pass with fresh valid responses. If any check fails, do not send enable or motion commands.

## Recovery limits and abort conditions

The Xiaomi manual documents automatic restart after successful completion, but it does not document interrupted-flash recovery, rollback, downgrade rules, image authentication, or a bootloader recovery sequence. A separate local v1.2.0.0 FACTORY image has an ST-Link erase/write note followed by encoder calibration; it is not a documented recovery image or procedure for this v1.2.1.5 application upgrade. Do not use it as a fallback.

If transfer errors, stalls, the motor fails to restart/reconnect, the UID or CAN ID changes, or any required read fails: keep pitch mechanically supported, do not attempt motion, do not send guessed bootloader/CAN frames, and escalate for the vendor-supported service/recovery route. A failed upgrade has no verified in-project recovery path.

## Source references

- [Xiaomi CyberGear user manual](../../../references/cybergear/CyberGear%E5%BE%AE%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf): adapter connection, PDF p. 12; parameter table/export and example firmware fields, pp. 13-16; firmware update/restart, p. 20; UID protocol, p. 21; bitrate-change warning, p. 25; v1.2.1.5 readable parameter list including `0x7019..0x7020`, p. 26.
- [CyberGear AI reference](../../../references/cybergear/CyberGear_AI_Reference.md): firmware update and version caveats, §28; bitrate warning, §23; runtime parameters, §24; documented versus community behavior, §31.
- [Mixed-hardware commissioning record](../../partially-implemented/commissioning/HARDWARE_COMMISSIONING_2026_09_26.md): expected pitch UID and earlier malformed `MechPos` read, §§ actual Pi probes and pitch compatibility defect.
- [Official Xiaomi CyberGear product page](https://www.mi.com/cyber-gear): product page; no official firmware download/checksum was exposed in the sources checked.
