# RoboMaster GM6020 CAN reference

This is a concise, agent-readable reference for the RoboMaster GM6020 CAN interface. The locally archived DJI/RoboMaster guide is the authority when this summary and the guide differ:

- [Official GM6020 User Guide v1.4 (2023.10)](references/gm6020/GM6020_User_Guide_v1.4_2023-10.pdf)
- Official download URL: <https://rm-static.djicdn.com/tem/17348/RM%20GM6020%20%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E8%8B%B1%EF%BC%8920231103.pdf>
- Published version/date printed in the guide: v1.4, 2023.10. Retrieved 2026-09-26.
- SHA-256: `E41CCA5150B8710BC52AB9F89C92142D6B75A637ED8990CE76C010A9E5EAD402`
- The PDF is DJI-copyrighted. This repository copy is retained as the requested engineering reference; consult the official URL for later revisions.

Page references below are the printed page numbers in the guide. The PDF includes a cover, so its one-based PDF page number is one greater than the printed page number.

## Bus and ID basics

- CAN bitrate: **1 Mbps**. Frames are standard (11-bit) CAN data frames with DLC 8. (printed pp. 3, 7-8)
- DIP switch bits 0-2 set motor ID. `000` is invalid; `001` through `111` are IDs 1 through 7. The fourth DIP switch enables the motor's CAN terminal resistor when ON. (printed p. 6)
- Feedback identifier is `0x204 + motor_id`: ID 1 sends feedback on `0x205`, ID 2 on `0x206`, through ID 7 on `0x20B`. **ID 1 feedback is `0x205`.** (printed p. 6)
- The controller-to-motor voltage-command identifiers are `0x1FF` and `0x2FF`; they are group frames, not per-motor feedback IDs. (printed pp. 6-7)
- Installation: the owner confirms GM6020 yaw, a slip ring and no yaw endstop. On 26 September a receive-only probe verified standard `0x205` feedback at approximately 1 kHz on `can0` via `spi0.0` / `mcp251xfd`, with a 40 MHz clock and 1 Mbps CAN. Subsequent authorized `0x1FF` pulses at +/-1000 raw voltage units produced small motions in matching encoder directions; launcher stop requested zero and observed stationarity. Firmware version and current-mode support remain unknown. See [commissioning evidence and limits](HARDWARE_COMMISSIONING_2026_09_26.md).

## Integer encoding

For each two-byte field below, the guide lists a high byte followed by a low byte. Assemble it in big-endian order:

```text
u16 = (byte_hi << 8) | byte_lo
signed_value = sign_extend_int16(u16)  # signed 16-bit two's-complement interpretation
```

Use signed decoding for signed command values and signed speed/current feedback. The guide specifies negative and positive command ranges and high-byte/low-byte order, but does not spell out the signed representation algorithm; two's complement is the standard interpretation and should be confirmed against the actual controller implementation before hardware actuation. Do not byte-swap the fields.

## Voltage control (documented legacy/default CAN path)

These frames set the motor driver's **torque-voltage command**. They are not speed commands, position commands, or current commands. The motor closes its internal loop on torque voltage; speed and position loops, if needed, are external. (printed pp. 7-8)

### Standard frames, DLC 8

| CAN ID | Bytes | Motor slots |
| --- | --- | --- |
| `0x1FF` | `0..1`, `2..3`, `4..5`, `6..7` | IDs 1, 2, 3, 4 |
| `0x2FF` | `0..1`, `2..3`, `4..5`, `6..7` | IDs 5, 6, 7, unused |

Within each two-byte pair, first byte is high byte and second is low byte. Each pair is a signed voltage setpoint. Guide v1.4 gives the accepted range as **-25,000..+25,000**. A single frame can command up to four slots; unused slots should be zero-filled. (printed p. 7)

**Version caveat:** GM6020 guide v1.0 (2018.12) and v1.2 (2020.05) document a voltage range of **-30,000..+30,000**. Use the installed motor firmware's applicable documentation; do not assume the older range is valid on newer firmware. The older official guide also gives 1 kHz feedback and the same feedback field layout. The current guide v1.4 also documents 1 kHz feedback. (v1.4 printed p. 8; v1.2 printed p. 7; v1.0 printed pp. 7-8)

## Current control (firmware and setting required)

Current control is **not enabled merely by sending a current-looking frame**. DJI says the motor must run firmware **v1.0.11.2 or later**, and the **Current Ring On/Off Switch** must be enabled in RoboMaster Assistant **v2.7 or later**. In this mode, the driver closes the loop on torque current. (printed p. 7)

Guide v1.4 describes signed current setpoints with numeric range **-16,384..+16,384**, corresponding to a torque-current range **-3 A..+3 A**. It lists standard DLC-8 frames as follows (printed p. 8):

| CAN ID printed in guide | Documented slots | Bytes per slot |
| --- | --- | --- |
| `0x2FE` | IDs 1, 2, 3; bytes 6-7 are null | high, low for each ID |
| `0x1FE` | IDs 1, 2, 3, 4 | high, low for each ID |

**Documentation ambiguity:** both current-frame tables list IDs 1-3, and the guide does not explain when to choose `0x2FE` versus `0x1FE`, despite both being labelled current-control formats. The frames are not an interchangeable extension of the voltage frames. Before enabling current mode, verify the exact frame/firmware behavior with the installed motor's firmware record or a controlled bench test. Do not send these frames based only on the ID table above.

## Feedback from each motor

Each motor periodically transmits an 8-byte standard CAN frame on `0x204 + motor_id`. Fields in the guide (printed pp. 6, 8):

| Bytes | Meaning | Decode |
| --- | --- | --- |
| 0-1 | Rotor mechanical angle | unsigned big-endian, range `0..8191` |
| 2-3 | Rotational speed | signed big-endian; unit rpm |
| 4-5 | Actual torque current | signed big-endian raw value; guide does not specify a feedback scale in amperes |
| 6 | Motor temperature | one byte; unit/scale is not stated alongside the protocol table |
| 7 | Null | ignore |

The angle field is a one-turn mechanical position count in the range `0..8191` (13-bit). It wraps at the turn boundary; when tracking continuous rotation, unwrap the modulo-8192 delta in the host. Do not interpret this field as an accumulated multi-turn angle. The CCW direction viewed from the output-shaft end is positive. (printed pp. 5, 8)

The guide documents a 1 kHz feedback sending frequency. This is the reported feedback cadence; it is not a command watchdog guarantee. The guide does **not** specify what the driver does when command frames stop, any command timeout duration, or whether the last output is cleared. Do not depend on an assumed timeout for safety. (printed p. 8)

The documented feedback payload has no enable/disabled bit, fault code, fault-clear command, or explicit torque-feedback-valid flag. The guide says the driver cuts output in an abnormal state, but does not define a CAN fault reporting or clearing procedure. Therefore a decoded current field alone cannot establish that the motor is enabled, fault-free, or producing valid torque. A host integration must separately establish device identity and feedback freshness and must handle unavailable/invalid status explicitly. Do not invent CyberGear-style register reads, mode switching, or fault-clear operations for this motor; this guide documents a different CAN interface.

## Operating constraints from the guide

- Rated supply is DC 24 V. The guide lists 1.2 N·m maximum continuous rated torque, 320 rpm maximum no-load speed, and 0-55 °C operating temperature. (printed pp. 3, 12)
- DJI describes over-temperature and over-voltage protection. The status LED table marks >100 °C as a temperature warning and >125 °C as an abnormal temperature state; in an abnormal state the driver cuts its output. (printed pp. 5-6)
- CAN and PWM control modes are both supported. The guide says the motor can identify the input and switch modes automatically. Current-mode configuration is through RoboMaster Assistant, not a property established by a CAN frame alone. (printed p. 7)
- The guide does not provide a CAN command timeout/watchdog contract. It also does not identify this installation's motor ID, axis, direction after gearing, or the safety behavior required by the turret application.
- Because the yaw stage is continuous-rotation with no endstop, position limits and safe stopping behavior must come from the turret's application-level design; they are not supplied by GM6020's CAN protocol description.

## Source scope and older edition

The archived binary is the newer English guide, v1.4 (2023.10). Earlier official English guide v1.0 (2018.12) and Chinese guide v1.2 (2020.05) were consulted to verify the prior voltage range and stable frame layout. The v1.2 official source is <https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf>; v1.0 English source is <https://rm-static.djicdn.com/tem/3724/RoboMaster%20GM6020%20Brushless%20DC%20Motor%20User%20Guide.pdf>. The current official RoboMaster product page is <https://www.robomaster.com/en-US/products/components/general/gm6020>.
