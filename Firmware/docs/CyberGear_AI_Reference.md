---
title: "Xiaomi CyberGear Micro Motor - AI/Grep-Friendly Technical Reference"
product: "CyberGear 微电机"
vendor: "Xiaomi"
document_type: "normalized technical reference"
canonical_source: "CyberGear微电机使用说明书.pdf (30 pages, user-supplied Chinese manual)"
source_language: "zh-CN"
reference_language: "English with Chinese aliases"
converted_on: "2026-08-31"
web_audit_on: "2026-08-31"
web_audit_status: "No clearly newer official Xiaomi manual/protocol revision found"
canonicality:
  official_manual_sections: "Authoritative transcription/normalization of the supplied manual"
  visual_transcriptions: "Read from diagrams/tables in the supplied manual; verify drawing before manufacturing"
  web_companion_sections: "Useful but not authoritative"
  community_sections: "Unofficial/reverse-engineered; never override official behavior without hardware verification"
grep_keywords:
  - CyberGear
  - CAN_ID
  - CAN_MASTER
  - run_mode
  - iq_ref
  - spd_ref
  - loc_ref
  - limit_spd
  - limit_cur
  - limit_torque
  - torque
  - fault
  - warning
  - encoder
  - zero
  - homing
  - baud
  - SocketCAN
  - firmware
---

# Xiaomi CyberGear Micro Motor - AI/Grep-Friendly Technical Reference

This file is a normalized, grep-friendly reference made from the supplied Chinese `CyberGear 微电机使用说明书`.

The technical content from the supplied manual is preserved and reorganized for machine use. Non-technical legal/after-sales text is condensed. Information found online is kept in separate sections so that an AI or implementation agent can distinguish **official-manual facts** from **web companions** and **community/reverse-engineered behavior**.

## 0. Source trust labels

Use these labels when consuming this file:

- `[OFFICIAL-MANUAL]` = directly stated in the supplied Xiaomi manual.
- `[OFFICIAL-MANUAL-VISUAL]` = transcribed from a diagram/graph/table image in the supplied manual.
- `[NORMALIZED]` = a machine-friendly restatement or formula derived from official fields.
- `[INFERENCE]` = conclusion inferred from official example code or field layout.
- `[WEB-COMPANION]` = online documentation that appears to mirror/translate the same manual.
- `[COMMUNITY]` = third-party implementation/reference.
- `[COMMUNITY-RE]` = reverse-engineered behavior not documented in the supplied official manual.
- `[CAUTION]` = important safety/interoperability warning.

**Authority rule:** if `[COMMUNITY]` or `[WEB-COMPANION]` disagrees with `[OFFICIAL-MANUAL]`, keep the official-manual value unless hardware/firmware testing proves otherwise.

---

# 1. Quick grep index

| Search term | Meaning / section |
|---|---|
| `CAN_FRAME_LAYOUT` | 29-bit extended CAN identifier layout |
| `COMM_TYPE_0` | Device ID request/response |
| `COMM_TYPE_1` | MIT-style / motion-control command |
| `COMM_TYPE_2` | Motor feedback |
| `COMM_TYPE_3` | Enable |
| `COMM_TYPE_4` | Stop / clear fault |
| `COMM_TYPE_6` | Set mechanical zero, volatile |
| `COMM_TYPE_7` | Change CAN ID |
| `COMM_TYPE_17` / `0x11` | Read one parameter |
| `COMM_TYPE_18` / `0x12` | Write one parameter, volatile |
| `COMM_TYPE_21` / `0x15` | Fault feedback |
| `COMM_TYPE_22` / `0x16` | Change CAN bitrate, firmware 1.2.1.5 |
| `run_mode` / `0x7005` | 0 motion, 1 position, 2 speed, 3 current |
| `iq_ref` / `0x7006` | Current-mode Iq command |
| `spd_ref` / `0x700A` | Speed command |
| `limit_torque` / `0x700B` | Torque limit; source typo alias `imit_torque` |
| `loc_ref` / `0x7016` | Position command |
| `limit_spd` / `0x7017` | Position-mode speed limit |
| `limit_cur` / `0x7018` | Speed/position current limit |
| `mechPos` / `0x7019` | Multi-turn load-side mechanical position |
| `mechVel` / `0x701B` | Load-side velocity |
| `VBUS` / `0x701C` | Bus voltage |
| `rotation` / `0x701D` | Turn counter |
| `FAULT_BITS_TYPE2` | Fault bits embedded in feedback arbitration ID |
| `FAULT_BITS_TYPE21` | Dedicated fault-frame bit definitions |
| `OVERLOAD_TABLE` | Allowed overload duration vs torque |
| `MECHANICAL_DIMENSIONS` | Mounting dimensions transcribed from drawing |
| `ENDIANNESS` | Big-endian MIT fields vs little-endian parameter fields |
| `MODE_SWITCH_SAFETY` | Stop before switching control mode |
| `COMMUNITY_ADDENDUM` | Quick-move / emergency brake / encoder calibration |
| `WEB_UPDATE_AUDIT` | Online manual/firmware documentation audit |

---

# 2. Critical safety and operating rules

## MODE_SWITCH_SAFETY

`[OFFICIAL-MANUAL] [CAUTION]`

- Do **not** switch control modes while the motor/joint is running.
- To change control mode, send the **stop command** first, then switch mode.
- Check parts and wiring before use.
- Do not arbitrarily disassemble the motor.
- Ensure there is no short circuit and the connector is wired correctly.
- Do not exceed the stated working parameters.
- The manual specifically warns against casually changing:
  - torque limit,
  - protection temperature,
  - over-temperature time.

## Mechanical/electrical safety limits

`[OFFICIAL-MANUAL]`

- Rated supply: `24 VDC`
- Allowed supply range: `16..28 VDC`
- Driver maximum allowed voltage: `28 VDC`
- Rated torque: `4 N.m`
- Peak torque: `12 N.m`
- Driver rated current: `6.5 A`
- Driver maximum allowed current: `23 A`
- Ambient operating range: `-20..50 degC`
- Control board maximum allowed temperature: `80 degC`
- No condensation.

---

# 3. Motor specifications

## 3.1 Standard operating conditions

`[OFFICIAL-MANUAL]`

| Property | Value |
|---|---:|
| Rated voltage | `24 VDC` |
| Supply range | `16..28 VDC` |
| Rated load / rated torque (CW) | `4 N.m` |
| Rotation direction | `CW / CCW`, viewed from output shaft |
| Allowed mounting orientation | output shaft horizontal or vertical |
| Standard operating temperature | `25 +/- 5 degC` |
| Operating temperature | `-20..50 degC` |
| Standard humidity | `65%` |
| Operating humidity | `5..85% RH`, no condensation |
| Storage temperature | `-30..70 degC` |
| Insulation class | `Class B` |

## 3.2 Electrical characteristics

`[OFFICIAL-MANUAL]`

| Property | Value |
|---|---:|
| No-load speed | `296 rpm +/-10%` |
| No-load current | `0.5 Arms` |
| Rated torque | `4 N.m` |
| Rated-load speed | `240 rpm +/-10%` |
| Rated-load current, peak | `6.5 A +/-10%` |
| Peak torque | `12 N.m` |
| Peak current, peak | `23 A +/-10%` |
| Back EMF | `0.054..0.057 Vrms/rpm` |
| Line resistance | `0.45 ohm +/-10%` |
| Torque constant | `0.87 N.m/Arms` |
| Motor inductance | `187..339 uH` |

### Insulation-test wording

`[OFFICIAL-MANUAL] [CAUTION]`

The supplied manual prints:

- insulation resistance / stator winding: `DC 500VAC, 100 MOhm`
- high-voltage withstand / stator-to-housing: `600 VAC, 1 s, 2 mA`

The first line is internally inconsistent (`DC` and `VAC` appear together). Preserve the source wording, but do **not** design a production hipot/insulation test from that line without checking a current manufacturer document.

## 3.3 Mechanical characteristics

`[OFFICIAL-MANUAL]`

| Property | Value |
|---|---:|
| Weight | `317 g +/-3 g` |
| Pole count | `28 poles` |
| Phase count | `3 phases` |
| Drive/control | `FOC` |
| Reduction ratio | `7.75:1` |

---

# 4. MECHANICAL_DIMENSIONS

`[OFFICIAL-MANUAL-VISUAL]`

The following values are transcribed from the mechanical drawing on page 3 of the supplied PDF. For CAD/manufacturing, verify against the original drawing or a STEP model.

| Drawing feature | Value / callout |
|---|---|
| Overall outer diameter | `phi 75 +/-0.1 mm` |
| Rear circular feature | `phi 71 mm` |
| Side/body registration diameter | approximately `phi 43 +0/-0.03 mm` as drawn |
| Front cylindrical diameter | `phi 63 mm` |
| Main axial body dimension | `25 +/-0.2 mm` |
| Front protrusion axial dimension | `8.5 mm` |
| Rear axial callouts | `3 mm`, `2 mm` |
| Front central feature | `phi 24 +/-0.1 mm` |
| Front through holes | `3 x phi 4 +0.1/0 mm` |
| Front threaded holes | `9 x M3`, depth callout `8 mm` |
| Front threaded holes | `6 x M4`, depth callout `6 mm` |
| Rear threaded pattern | `6 x M3` |

`[CAUTION]` The drawing is more authoritative than this text transcription for hole PCDs, tolerances, and manufacturing fits. Use a STEP/dimension drawing if available before machining parts.

---

# 5. Torque-speed and overload behavior

## 5.1 T-N curve

`[OFFICIAL-MANUAL-VISUAL]`

The manual includes a torque-vs-speed curve. Use the numeric rated/peak specifications above as canonical. The plotted curve visually shows high available torque at low speed and decreasing torque as speed approaches the no-load region.

## OVERLOAD_TABLE

`[OFFICIAL-MANUAL-VISUAL]`

Test conditions printed in the manual:

- ambient temperature: `25 degC`
- winding limit temperature: `120 degC`
- speed: `24 rpm`

| Load torque | Maximum operating time |
|---:|---:|
| `12.0 N.m` | `28 s` |
| `11.0 N.m` | `45 s` |
| `10.0 N.m` | `60 s` |
| `9.0 N.m` | `90 s` |
| `8.0 N.m` | `160 s` |
| `7.0 N.m` | `320 s` |
| `6.0 N.m` | `700 s` |
| `5.0 N.m` | `1800 s` |
| `4.5 N.m` | `2500 s` |
| `4.0 N.m` | `rated` / continuous-rated condition |

---

# 6. Driver specifications and hardware

## 6.1 Driver

`[OFFICIAL-MANUAL]`

| Property | Value |
|---|---:|
| Rated operating voltage | `24 VDC` |
| Maximum allowed voltage | `28 VDC` |
| Rated operating current | `6.5 A` |
| Maximum allowed current | `23 A` |
| Standby consumption | `<=18 mA` |
| CAN bitrate | `1 Mbps` |
| Driver size | `phi 58 mm` |
| Ambient temperature | `-20..50 degC` |
| Maximum control-board temperature | `80 degC` |
| Encoder resolution | `14 bit`, single-turn absolute |

## 6.2 Main components

`[OFFICIAL-MANUAL]`

| Function | Part |
|---|---|
| MCU | `GD32F303RET6` |
| Gate-driver IC | `6EDL7141` |
| Magnetic encoder | `AS5047P` |
| Thermistors | `NXFT15XH103FEAB021` / `NCP18XH103F03RB` |
| Power MOSFET | `JMGG031V06A`, 6 pcs |

---

# 7. Connectors and pinout

## 7.1 Recommended connector

`[OFFICIAL-MANUAL]`

| Side | Model | Brand |
|---|---|---|
| PCB / board | `XT30PB(2+2)-M.G.B` | AMASS |
| Cable | `XT30(2+2)-F.G.B` | AMASS |

Other service interfaces:

- `2.0 mm-2P` solder pads / probe pins
- `2.54 mm-4P` solder pads / probe pins

## 7.2 Main power + CAN connector pinout

`[OFFICIAL-MANUAL]`

| Pin | Signal | Meaning |
|---:|---|---|
| 1 | `VBAT+` | power positive |
| 2 | `GND` | power negative / ground |
| 3 | `CAN_L` | CAN low |
| 4 | `CAN_H` | CAN high |

## 7.3 CAN test points

`[OFFICIAL-MANUAL]`

| Test pin | Signal |
|---:|---|
| 1 | `CAN_L` |
| 2 | `CAN_H` |

## 7.4 SWD/programming port

`[OFFICIAL-MANUAL]`

| Pin | Signal |
|---:|---|
| 1 | `SWDIO` |
| 2 | `SWCLK` |
| 3 | `3V3` |
| 4 | `GND` |

## 7.5 LEDs

`[OFFICIAL-MANUAL]`

- Red power LED:
  - indicates MCU 3.3 V supply.
  - with 24 V input, it should illuminate red.
  - if it does not illuminate with 24 V applied, disconnect power immediately.
- Blue signal LED:
  - flashing indicates MCU and driver IC are operating.

---

# 8. Official debugger / host-computer tool

`[OFFICIAL-MANUAL]`

The official manual describes CAN communication through a USB-CAN converter.

Recommended bridge:

- YourCee USB-CAN module
- CH340 driver
- converter default mode: `AT mode`
- serial frame header: `41 54`
- serial frame tail: `0D 0A`

The debugger can:

- connect/disconnect motor
- read motor information
- calibrate magnetic encoder
- change motor CAN ID
- set mechanical zero
- update firmware
- read/write/download/upload parameter table
- restore defaults
- clear warnings
- plot observed parameters
- export waveform data.

### Zero position

`[OFFICIAL-MANUAL]`

Setting mechanical zero sets the **current position to zero**.

**Persistence:** `volatile`; the manual explicitly states that the zero position is **lost on power-off**.

### Magnetic encoder calibration

`[OFFICIAL-MANUAL]`

Recalibration is required if the motor board is reinstalled on the motor, or if motor phase wiring order is changed.

---

# 9. Debugger configuration/storage parameter table: 0x2000 series

`[OFFICIAL-MANUAL]`

These are the parameter-table entries shown by the official debugger. Values below are the source's displayed/reference values, not guaranteed firmware-independent defaults.

| Index | Name | Type | Access | Min | Max | Example/current | Meaning |
|---|---|---|---|---:|---:|---:|---|
| `0x0000` | `Name` | String | R/W | - | - | - | name |
| `0x0001` | `BarCode` | String | R/W | - | - | - | barcode |
| `0x1000` | `BootCodeVersion` | String | RO | - | - | `0.1.5` | bootloader version |
| `0x1001` | `BootBuildDate` | String | RO | - | - | `Mar 16 2022` | boot build date |
| `0x1002` | `BootBuildTime` | String | RO | - | - | `20:22:09` | boot build time |
| `0x1003` | `AppCodeVersion` | String | RO | - | - | `0.1.5` | motor application version |
| `0x1004` | `AppGitVersion` | String | RO | - | - | `7b844b0fM` | app git version |
| `0x1005` | `AppBuildDate` | String | RO | - | - | `Apr 14 2022` | app build date |
| `0x1006` | `AppBuildTime` | String | RO | - | - | `20:30:22` | app build time |
| `0x1007` | `AppCodeName` | String | RO | - | - | `dog_motor` | app codename |
| `0x2000` | `echoPara1` | uint16 | config | `5` | `74` | `5` | echo parameter |
| `0x2001` | `echoPara2` | uint16 | config | `5` | `74` | `5` | echo parameter |
| `0x2002` | `echoPara3` | uint16 | config | `5` | `74` | `5` | echo parameter |
| `0x2003` | `echoPara4` | uint16 | config | `5` | `74` | `5` | echo parameter |
| `0x2004` | `echoFreHz` | uint32 | R/W | `1` | `10000` | `500` | echo/telemetry frequency |
| `0x2005` | `MechOffset` | float | set | `-7` | `7` | `4.619583` | magnetic encoder mechanical-angle offset |
| `0x2006` | `MechPos_init` | float | R/W | `-50` | `50` | `4.52` | reference angle for initial multi-turn state |
| `0x2007` | `limit_torque` | float | R/W | `0` | `12` | `12` | torque limit |
| `0x2008` | `I_FW_MAX` | float | R/W | `0` | `33` | `0` | field-weakening current; default 0 |
| `0x2009` | `motor_index` | uint8 | set | `0` | `20` | `1` | joint/index marker |
| `0x200A` | `CAN_ID` | uint8 | set | `0` | `127` | `1` | motor/node CAN ID |
| `0x200B` | `CAN_MASTER` | uint8 | set | `0` | `127` | `0` | host/master CAN ID |
| `0x200C` | `CAN_TIMEOUT` | uint32 | R/W | `0` | `100000` | `0` | CAN timeout threshold; source default 0 |
| `0x200D` | `motorOverTemp` | int16 | R/W | `0` | `1500` | `800` | motor protection temperature, value = degC x10 |
| `0x200E` | `overTempTime` | uint32 | R/W | `1000` | `100000` | `20000` | over-temperature time |
| `0x200F` | `GearRatio` | float | R/W | `1` | `64` | `7.75` | gear ratio |
| `0x2010` | `Tq_caliType` | uint8 | R/W | `0` | `1` | `1` | torque calibration method |
| `0x2011` | `cur_filt_gain` | float | R/W | `0` | `1` | `0.9` | current filter parameter |
| `0x2012` | `cur_kp` | float | R/W | `0` | `200` | `0.025` | current Kp |
| `0x2013` | `cur_ki` | float | R/W | `0` | `200` | `0.0258` | current Ki |
| `0x2014` | `spd_kp` | float | R/W | `0` | `200` | `2` | speed Kp |
| `0x2015` | `spd_ki` | float | R/W | `0` | `200` | `0.021` | speed Ki |
| `0x2016` | `loc_kp` | float | R/W | `0` | `200` | `30` | position Kp |
| `0x2017` | `spd_filt_gain` | float | R/W | `0` | `1` | `0.1` | speed filter parameter |
| `0x2018` | `limit_spd` | float | R/W | `0` | `200` | `2` | position-mode speed limit |
| `0x2019` | `limit_cur` | float | R/W | `0` | `23` | `23` | current limit for position/speed modes |

`[CAUTION]` The manual warns not to casually change torque limit, protection temperature, or over-temperature time.

---

# 10. Debugger observation/telemetry parameter table: 0x3000 series

`[OFFICIAL-MANUAL]`

These are read-only observations unless noted otherwise by the source.

| Index | Name | Type | Example | Meaning / unit |
|---|---|---|---:|---|
| `0x3000` | `timeUse0` | uint16 | `5` | internal timing |
| `0x3001` | `timeUse1` | uint16 | `0` | internal timing |
| `0x3002` | `timeUse2` | uint16 | `10` | internal timing |
| `0x3003` | `timeUse3` | uint16 | `0` | internal timing |
| `0x3004` | `encoderRaw` | int16 | `11396` | magnetic encoder sample |
| `0x3005` | `mcuTemp` | int16 | `337` | MCU temperature, degC x10 |
| `0x3006` | `motorTemp` | int16 | `333` | motor NTC temperature, degC x10 |
| `0x3007` | `vBus(mv)` | uint16 | `24195` | bus voltage in mV |
| `0x3008` | `adc1Offset` | int32 | `2084` | ADC channel 1 zero-current offset |
| `0x3009` | `adc2Offset` | int32 | `2084` | ADC channel 2 zero-current offset |
| `0x300A` | `adc1Raw` | uint16 | `1232` | ADC channel 1 raw |
| `0x300B` | `adc2Raw` | uint16 | `1212` | ADC channel 2 raw |
| `0x300C` | `VBUS` | float | `24.195` | bus voltage, V |
| `0x300D` | `cmdId` | float | `0` | d-axis current command, A |
| `0x300E` | `cmdIq` | float | `0` | q-axis current command, A |
| `0x300F` | `cmdlocref` | float | `0` | position-loop command, rad |
| `0x3010` | `cmdspdref` | float | `0` | speed-loop command, rad/s |
| `0x3011` | `cmdTorque` | float | `0` | torque command, N.m |
| `0x3012` | `cmdPos` | float | `0` | MIT-protocol position command |
| `0x3013` | `cmdVel` | float | `0` | MIT-protocol speed command |
| `0x3014` | `rotation` | int16 | `1` | turn count |
| `0x3015` | `modPos` | float | `4.363409` | unwrapped/not-counted mechanical angle, rad |
| `0x3016` | `mechPos` | float | `0.777679` | load-side multi-turn mechanical angle, rad |
| `0x3017` | `mechVel` | float | `0.036618` | load-side speed, rad/s |
| `0x3018` | `elecPos` | float | `4.714761` | electrical angle |
| `0x3019` | `ia` | float | `0` | U-line current, A |
| `0x301A` | `ib` | float | `0` | V-line current, A |
| `0x301B` | `ic` | float | `0` | W-line current, A |
| `0x301C` | `tick` | uint32 | `31600` | internal tick |
| `0x301D` | `phaseOrder` | uint8 | `0` | calibration direction marker |
| `0x301E` | `iqf` | float | `0` | filtered Iq, A |
| `0x301F` | `boardTemp` | int16 | `359` | board temperature, degC x10 |
| `0x3020` | `iq` | float | `0` | raw Iq, A |
| `0x3021` | `id` | float | `0` | raw Id, A |
| `0x3022` | `faultSta` | uint32 | `0` | fault state |
| `0x3023` | `warnSta` | uint32 | `0` | warning state |
| `0x3024` | `drv_fault` | uint16 | `0` | driver IC fault value |
| `0x3025` | `drv_temp` | int16 | `48` | driver IC temperature, degC |
| `0x3026` | `Uq` | float | `0` | q-axis voltage |
| `0x3027` | `Ud` | float | `0` | d-axis voltage |
| `0x3028` | `dtc_u` | float | `0` | U-phase duty ratio |
| `0x3029` | `dtc_v` | float | `0` | V-phase duty ratio |
| `0x302A` | `dtc_w` | float | `0` | W-phase duty ratio |
| `0x302B` | `v_bus` | float | `24.195` | closed-loop VBUS |
| `0x302C` | `v_ref` | float | `0` | vector-combined Vq/Vd reference |
| `0x302D` | `torque_fdb` | float | `0` | torque feedback, N.m |
| `0x302E` | `rated_i` | float | `8` | internal displayed rated current |
| `0x302F` | `limit_i` | float | `27` | internal displayed maximum-current limit |

`[CAUTION]` `0x302E/0x302F` are the source debugger's internal displayed values. They do not replace the external driver rating of `6.5 A rated / 23 A maximum` stated elsewhere in the manual.

---

# 11. CAN protocol

## CAN_FRAME_LAYOUT

`[OFFICIAL-MANUAL]`

- Interface: `CAN 2.0`
- Bitrate: `1 Mbps` by default
- Frame: `29-bit extended CAN frame`
- Data length: `8 bytes`

29-bit extended identifier:

```text
bits 28..24 : communication type (5 bits)
bits 23..8  : data area 2 (16 bits)
bits 7..0   : target address / target CAN ID (8 bits)
```

`[NORMALIZED]`

```text
extended_id = ((comm_type & 0x1F) << 24)
            | ((data2 & 0xFFFF) << 8)
            |  (target_id & 0xFF)

comm_type = (extended_id >> 24) & 0x1F
data2     = (extended_id >> 8)  & 0xFFFF
target_id =  extended_id        & 0xFF
```

For many host-to-motor commands:

```text
data2 = host_can_id
```

Because assigning a small host ID to the 16-bit `data2` field places it in identifier bits `15..8`.

## ENDIANNESS

`[OFFICIAL-MANUAL] [INFERENCE]`

There are two different byte-order conventions in the supplied example code:

1. **MIT / motion-control packed u16 fields** are transmitted **big-endian**:
   - high byte first,
   - low byte second.

2. **Parameter protocol (type 17/18)** uses `memcpy()` from `uint16_t`/`float` on the GD32/ARM sample platform, implying **little-endian** scalar encoding on that platform.

`[COMMUNITY]` Current independent CyberGear drivers also implement parameter indices/float values little-endian and feedback/control u16 fields big-endian.

**Do not use one byte order for every command type.**

---

# 12. Communication-type summary

| Decimal | Hex | Name | Official? |
|---:|---:|---|---|
| `0` | `0x00` | `COMM_TYPE_0` get device ID / ID response | yes |
| `1` | `0x01` | `COMM_TYPE_1` motion-control / MIT-style command | yes |
| `2` | `0x02` | `COMM_TYPE_2` motor feedback | yes |
| `3` | `0x03` | `COMM_TYPE_3` enable motor | yes |
| `4` | `0x04` | `COMM_TYPE_4` stop motor / clear fault | yes |
| `6` | `0x06` | `COMM_TYPE_6` set mechanical zero | yes |
| `7` | `0x07` | `COMM_TYPE_7` set motor CAN ID | yes |
| `17` | `0x11` | `COMM_TYPE_17` read one parameter | yes |
| `18` | `0x12` | `COMM_TYPE_18` write one parameter | yes |
| `21` | `0x15` | `COMM_TYPE_21` dedicated fault feedback | yes |
| `22` | `0x16` | `COMM_TYPE_22` change CAN bitrate | yes, firmware 1.2.1.5 |
| `5` | `0x05` | encoder calibration | community reverse-engineered |
| `19` | `0x13` | parameter-table related command | community reverse-engineered |
| `20` | `0x14` | emergency brake / torque cut | community reverse-engineered |

---

# 13. COMM_TYPE_0 - get device ID

`[OFFICIAL-MANUAL]`

Purpose: get motor ID and 64-bit MCU unique identifier.

Request identifier:

```text
comm_type = 0
identifier bits 15..8 = host CAN_ID
identifier bits 7..0 = target motor CAN_ID
data bytes = 0
```

Response:

```text
comm_type = 0
identifier carries target/motor CAN_ID in data-area-2
identifier low byte = 0xFE
data[0..7] = 64-bit MCU unique identifier
```

The manual does not describe a generic broadcast-discovery algorithm. Community libraries provide bus scanning helpers.

---

# 14. COMM_TYPE_1 - motion-control / MIT-style command

`[OFFICIAL-MANUAL]`

Purpose: command torque, position, speed, Kp and Kd in one frame.

Identifier:

```text
comm_type = 1
data2 = torque_u16
target_id = motor CAN_ID
```

Data:

```text
byte 0..1 = target position u16
byte 2..3 = target velocity u16
byte 4..5 = Kp u16
byte 6..7 = Kd u16
```

Mappings printed in the protocol table:

```text
torque   : u16 0..65535 -> -12..+12 N.m
position : u16 0..65535 -> -4*pi..+4*pi rad
velocity : u16 0..65535 -> -30..+30 rad/s
Kp       : u16 0..65535 -> 0..500
Kd       : u16 0..65535 -> 0..5
```

### Position-range source inconsistency

`[OFFICIAL-MANUAL] [CAUTION]`

The protocol table says `-4*pi..+4*pi`, while the supplied official C example defines:

```c
P_MIN = -12.5f
P_MAX =  12.5f
```

Current community implementations commonly use `+/-12.5 rad`.

For compatibility with the bundled example code and common libraries, `+/-12.5 rad` is the de-facto software mapping; treat `+/-4*pi` as a source-document inconsistency unless your firmware has been verified otherwise.

### Generic encoding

`[NORMALIZED]`

The official helper truncates after scaling:

```text
encode_u16(x, xmin, xmax):
    x = clamp(x, xmin, xmax)
    return int((x - xmin) * 65535 / (xmax - xmin))

decode_u16(u, xmin, xmax):
    return xmin + (u / 65535) * (xmax - xmin)
```

MIT frame u16 values are high-byte first.

---

# 15. COMM_TYPE_2 - motor feedback

`[OFFICIAL-MANUAL]`

Feedback identifier state:

```text
bits 8..15  : current motor CAN ID
bits 16..21 : fault flags
bits 22..23 : mode/state
bits 0..7   : host CAN ID
```

## FAULT_BITS_TYPE2

```text
bit 21 : uncalibrated
bit 20 : HALL encoder fault
bit 19 : magnetic encoder fault
bit 18 : over-temperature
bit 17 : over-current
bit 16 : under-voltage
```

Mode/state:

```text
0 = Reset mode
1 = Cali / calibration mode
2 = Motor / running mode
```

Feedback data:

```text
byte 0..1 : current angle, u16 -> nominally -4*pi..+4*pi
byte 2..3 : current angular velocity, u16 -> -30..+30 rad/s
byte 4..5 : current torque, u16 -> -12..+12 N.m
byte 6..7 : current temperature, integer = degC * 10
```

The u16 fields are transmitted high byte first in common/official handling.

---

# 16. COMM_TYPE_3 - enable motor

`[OFFICIAL-MANUAL]`

```text
comm_type = 3
identifier bits 15..8 = host CAN_ID
identifier bits 7..0 = target motor CAN_ID
data[0..7] = zero
```

Response: `COMM_TYPE_2` feedback.

### Official example-code caveat

`[OFFICIAL-MANUAL] [INFERENCE]`

The supplied C example assigns `master_id` to the ID data field and then immediately assigns zero before transmit. This effectively assumes host/master CAN ID `0`. Do not copy that accidental overwrite if your host ID is non-zero.

---

# 17. COMM_TYPE_4 - stop motor / clear fault

`[OFFICIAL-MANUAL]`

```text
comm_type = 4
identifier bits 15..8 = host CAN_ID
identifier bits 7..0 = target motor CAN_ID
```

Normal stop:

```text
data[0..7] = 00 00 00 00 00 00 00 00
```

Clear fault:

```text
data[0] = 1
```

Response: `COMM_TYPE_2` feedback.

---

# 18. COMM_TYPE_6 - set mechanical zero

`[OFFICIAL-MANUAL]`

Sets current motor position as mechanical zero.

```text
comm_type = 6
identifier bits 15..8 = host CAN_ID
identifier bits 7..0 = target motor CAN_ID
data[0] = 1
```

**Persistence:** `volatile`; **lost after power-off**.

Response: `COMM_TYPE_2`.

This is a zero-offset command, not a mechanical-endstop discovery mechanism.

---

# 19. COMM_TYPE_7 - change motor CAN ID

`[OFFICIAL-MANUAL]`

Change takes effect immediately.

```text
comm_type = 7
identifier bits 15..8  = host CAN_ID
identifier bits 23..16 = new/pre-set CAN_ID
identifier bits 7..0   = current target motor CAN_ID
```

`[NORMALIZED]`

```text
data2 = ((new_can_id & 0xFF) << 8) | (host_can_id & 0xFF)
```

Response: device/broadcast frame compatible with `COMM_TYPE_0`.

---

# 20. COMM_TYPE_17 / 0x11 - read one parameter

`[OFFICIAL-MANUAL]`

Request:

```text
comm_type = 17 / 0x11
identifier bits 15..8 = host CAN_ID
identifier bits 7..0  = target motor CAN_ID

data[0..1] = parameter index
data[2..3] = 0
data[4..7] = 0
```

Response:

```text
comm_type = 17 / 0x11
identifier identifies target motor and host

data[0..1] = parameter index
data[2..3] = 0
data[4..7] = parameter data
```

For a 1-byte parameter, the value is in `data[4]`.

See `RUNTIME_PARAMETER_TABLE`.

---

# 21. COMM_TYPE_18 / 0x12 - write one parameter

`[OFFICIAL-MANUAL]`

**Persistence:** source states **lost on power-off**.

```text
comm_type = 18 / 0x12
identifier bits 15..8 = host CAN_ID
identifier bits 7..0  = target motor CAN_ID

data[0..1] = parameter index
data[2..3] = 0
data[4..7] = parameter data
```

For `uint8`, value is placed at `data[4]`.

For `float`, the official sample copies four IEEE-754 bytes into `data[4..7]`.

Response: `COMM_TYPE_2` feedback.

---

# 22. COMM_TYPE_21 / 0x15 - dedicated fault feedback

`[OFFICIAL-MANUAL]`

```text
comm_type = 21 / 0x15
data[0..3] = fault value
data[4..7] = warning value
```

## FAULT_BITS_TYPE21

Fault value:

```text
bit 16      : phase A current-sampling over-current
bits 15..8  : overload fault field
bit 7       : encoder uncalibrated
bit 5       : phase C current-sampling over-current
bit 4       : phase B current-sampling over-current
bit 3       : over-voltage
bit 2       : under-voltage
bit 1       : driver-chip fault
bit 0       : motor over-temperature; source default threshold 80 degC
```

Warning value:

```text
bit 0       : motor over-temperature warning; source default threshold 75 degC
```

---

# 23. COMM_TYPE_22 / 0x16 - change CAN bitrate

`[OFFICIAL-MANUAL]`

The manual states this is modifiable in firmware `1.2.1.5`.

`[CAUTION]` Incorrect bitrate modification can make the motor impossible to connect to or upgrade through the expected tool.

```text
comm_type = 22 / 0x16
identifier bits 15..8 = host CAN_ID
identifier bits 7..0 = target motor CAN_ID

data[0]:
    1 = 1 Mbps
    2 = 500 kbps
    3 = 250 kbps
    4 = 125 kbps
```

Response: `COMM_TYPE_0`-style broadcast/ID frame.

---

# 24. RUNTIME_PARAMETER_TABLE - 0x7005..0x7020

`[OFFICIAL-MANUAL]`

The manual notes that `0x7019..0x7020` are readable with firmware `1.2.1.5`.

| Index | Canonical name | Source alias | Type | Bytes | Access | Range / default / meaning |
|---|---|---|---|---:|---|---|
| `0x7005` | `run_mode` | - | uint8 | 1 | R/W | `0=motion`, `1=position`, `2=speed`, `3=current` |
| `0x7006` | `iq_ref` | - | float | 4 | R/W | current command `-23..23 A` |
| `0x700A` | `spd_ref` | - | float | 4 | R/W | speed command `-30..30 rad/s` |
| `0x700B` | `limit_torque` | `imit_torque` | float | 4 | R/W | torque limit `0..12 N.m` |
| `0x7010` | `cur_kp` | - | float | 4 | R/W | source default `0.125` |
| `0x7011` | `cur_ki` | - | float | 4 | R/W | source default `0.0158` |
| `0x7014` | `cur_filt_gain` | - | float | 4 | R/W | `0..1.0`, source default `0.1` |
| `0x7016` | `loc_ref` | - | float | 4 | R/W | position-mode target angle, rad |
| `0x7017` | `limit_spd` | - | float | 4 | R/W | position-mode speed limit `0..30 rad/s` |
| `0x7018` | `limit_cur` | - | float | 4 | R/W | speed/position current limit `0..23 A` |
| `0x7019` | `mechPos` | - | float | 4 | R | load-side multi-turn mechanical angle, rad |
| `0x701A` | `iqf` | - | float | 4 | R | filtered Iq, `-23..23 A` |
| `0x701B` | `mechVel` | - | float | 4 | R | load-side speed, `-30..30 rad/s` |
| `0x701C` | `VBUS` | - | float | 4 | R | bus voltage, V |
| `0x701D` | `rotation` | - | int16 | 2 | R/W | turn count |
| `0x701E` | `loc_kp` | - | float | 4 | R/W | position Kp, source default `30` |
| `0x701F` | `spd_kp` | - | float | 4 | R/W | speed Kp, source default `1` |
| `0x7020` | `spd_ki` | - | float | 4 | R/W | speed Ki, source default `0.002` |

### Source typo normalization

The manual prints `0x700B` as `imit_torque`, apparently missing the leading `l`. This file uses canonical `limit_torque` and keeps `imit_torque` as a grep alias.

### Parameter-table caveat

The debugger/storage table (`0x2000` series) shows some gain example values that differ from the `0x7000` runtime table defaults. Do not assume these tables are interchangeable across firmware versions.

---

# 25. Control-mode sequences

## 25.1 Motion-control / MIT-style mode

`[OFFICIAL-MANUAL]`

The motor powers on in motion-control mode by default.

```text
1. send COMM_TYPE_3 enable
2. send COMM_TYPE_1 motion-control command
3. receive COMM_TYPE_2 feedback
```

## 25.2 Current mode

`[OFFICIAL-MANUAL]`

```text
1. COMM_TYPE_18: write run_mode (0x7005) = 3
2. COMM_TYPE_3: enable
3. COMM_TYPE_18: write iq_ref (0x7006) = desired current
```

## 25.3 Speed mode

`[OFFICIAL-MANUAL]`

```text
1. COMM_TYPE_18: write run_mode (0x7005) = 2
2. COMM_TYPE_3: enable
3. COMM_TYPE_18: write limit_cur (0x7018)
4. COMM_TYPE_18: write spd_ref (0x700A)
```

## 25.4 Position mode

`[OFFICIAL-MANUAL]`

```text
1. COMM_TYPE_18: write run_mode (0x7005) = 1
2. COMM_TYPE_3: enable
3. COMM_TYPE_18: write limit_spd (0x7017)
4. COMM_TYPE_18: write loc_ref (0x7016)
```

## 25.5 Stop

`[OFFICIAL-MANUAL]`

```text
send COMM_TYPE_4
```

`[CAUTION]` Stop before changing control mode.

---

# 26. Minimal raw-frame construction notes

`[NORMALIZED]`

Assume:

```text
host_can_id = 0
motor_can_id = 127 / 0x7F
```

Then arbitration IDs are:

```text
enable:     (0x03 << 24) | 0x7F = 0x0300007F
stop:       (0x04 << 24) | 0x7F = 0x0400007F
set zero:   (0x06 << 24) | 0x7F = 0x0600007F
read param: (0x11 << 24) | 0x7F = 0x1100007F
write param:(0x12 << 24) | 0x7F = 0x1200007F
```

Example set-zero data:

```text
01 00 00 00 00 00 00 00
```

Example stop data:

```text
00 00 00 00 00 00 00 00
```

### Example type-18 parameter payload

`[NORMALIZED] [INFERENCE]`

For `run_mode = speed (2)`:

```text
index = 0x7005 little-endian -> 05 70
reserved -> 00 00
uint8 value 2 in byte4 -> 02
remaining bytes -> 00 00 00

data = 05 70 00 00 02 00 00 00
```

For a float parameter, encode IEEE-754 float little-endian into bytes 4..7.

---

# 27. Official C-example concepts, normalized

`[OFFICIAL-MANUAL]`

The official sample defines:

```text
P_MIN  = -12.5
P_MAX  =  12.5
V_MIN  = -30
V_MAX  =  30
KP_MIN =  0
KP_MAX = 500
KD_MIN =  0
KD_MAX =  5
T_MIN  = -12
T_MAX  =  12
```

It uses a 29-bit extended-frame identifier represented as bit fields:

```text
id   : 8 bits
data : 16 bits
mode : 5 bits
res  : 3 bits
```

The sample provides concepts/functions equivalent to:

- `float_to_uint()`
- `motor_enable()`
- `motor_controlmode()`
- `motor_reset()`
- `motor_modechange()`
- `motor_write()`

This reference intentionally does not duplicate the entire source listing; the frame layouts above are easier to grep and less error-prone to port.

---

# 28. Firmware update

`[OFFICIAL-MANUAL]`

Official debugger flow:

```text
1. Device module -> Upgrade
2. choose .bin firmware file
3. confirm
4. wait for progress to finish
5. motor automatically restarts
```

The supplied manual's debugger screenshot/table includes old example application version `0.1.5` built in 2022, while later protocol text specifically documents firmware `1.2.1.5` features. Treat the manual as an accumulated document rather than assuming every screenshot reflects the latest firmware described by the protocol chapter.

---

# 29. USB-CAN serial bridge companion notes

`[WEB-COMPANION]`

A current AIFITLAB web mirror of the host-computer manual adds a serial-port framing example for the recommended USB-CAN bridge.

Bridge serial frame:

```text
41 54  [4-byte encoded extended CAN ID]  08  [8 data bytes]  0D 0A
```

It states that this USB-CAN module shifts the raw extended CAN ID left by 3 bits and appends binary `100` when serializing the identifier.

Example published by that companion manual:

```text
41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a
```

**Scope:** this is a property of that USB-CAN serial protocol, **not raw SocketCAN/CAN 2.0 framing**.

Companion URL:
`https://wiki.aifitlab.com/xiaomi-cybergear-docs/xiaomi-cybergear-host-computer-user-manual`

---

# 30. Raspberry Pi / Linux practical reference

`[COMMUNITY]`

A current Python driver, `grrodre/cybergear`, uses `python-can` and is tested with SocketCAN. It is useful as an implementation reference for Raspberry Pi/Linux, but it is not an official Xiaomi specification.

Typical SocketCAN setup documented by the project:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Project:
`https://github.com/grrodre/cybergear`

Another independent C/ESP-IDF implementation:
`https://github.com/cybergear-robotics/cybergear`

These are useful for checking:
- raw frame packing,
- byte order,
- parameter reads/writes,
- feedback decoding,
- practical CAN interface behavior.

---

# 31. COMMUNITY_ADDENDUM - unofficial / reverse-engineered features

`[COMMUNITY-RE] [CAUTION]`

Current community drivers expose behavior not present in the supplied official protocol chapter. Treat this as **optional experimental functionality**, not authoritative specification.

Reported/implemented examples include:

| Feature | Community representation | Official supplied manual? |
|---|---|---|
| Encoder calibration command | communication type `0x05` | no |
| Quick-move mode | community code exposes `run_mode = 7` / quick-move helper | no |
| Parameter-table command | communication type `0x13` in some implementations | no |
| Emergency brake / immediate torque cut | communication type `0x14` in some implementations | no |

The `grrodre/cybergear` project explicitly provides:
- `quick_move()` / `quick_stop()`
- `emergency_brake()`
- `encoder_calibration()`

Use these only after reading the library source and testing against the exact firmware/hardware revision.

For safety-critical motion, prefer official stop/disable behavior unless the reverse-engineered command has been validated.

---

# 32. WEB_UPDATE_AUDIT

Audit date: `2026-08-31`.

## 32.1 Official Xiaomi site

`[WEB-COMPANION]`

The official Xiaomi CyberGear product page is still online:

`https://www.mi.com/cyber-gear`

The page confirms the product remains present on Xiaomi's site, but the web-accessible page did **not** surface a newer downloadable protocol/manual revision during this audit.

## 32.2 English translation / mirror of the same manual

`[WEB-COMPANION]`

Repository:

`https://github.com/belovictor/cybergear-docs`

English manual:

`https://github.com/belovictor/cybergear-docs/blob/main/instructionmanual/instructionmanual.md`

This is useful for English searchability, but it mirrors/translates the same protocol generation. It still describes firmware `1.2.1.5` for CAN bitrate modification and `0x7019..0x7020`.

## 32.3 AIFITLAB web manuals

`[WEB-COMPANION]`

Micro-motor manual:

`https://wiki.aifitlab.com/xiaomi-cybergear-docs/xiaomi-cybergear-micro-motor-user-manual`

Host-computer manual:

`https://wiki.aifitlab.com/xiaomi-cybergear-docs/xiaomi-cybergear-host-computer-user-manual`

They are easier to browse and contain practical debugger/serial information, but they should be treated as mirrors/companion documentation rather than a newer official Xiaomi protocol specification.

**Important transcription warning:** third-party web mirrors can contain table/index transcription mistakes. Keep the supplied original PDF as canonical for register addresses. For example, the original clearly uses hexadecimal progression `0x200A`, `0x200B`, etc.

## 32.4 Current implementation-oriented references

`[COMMUNITY]`

Python / SocketCAN:

`https://github.com/grrodre/cybergear`

ESP-IDF / TWAI:

`https://github.com/cybergear-robotics/cybergear`

These are more useful than the original manual for software integration, especially on Linux/Raspberry Pi, but they include community behavior and reverse engineering.

## 32.5 Audit conclusion

**No clearly newer official Xiaomi CyberGear manual or official protocol revision was found in the public web sources checked on 2026-08-31.**

The same `1.2.1.5`-era protocol remains the common documented baseline in current mirrors and implementations.

Therefore the recommended hierarchy is:

```text
1. This supplied Xiaomi PDF / this normalized reference       -> protocol authority
2. Current community source code                              -> implementation guidance
3. Web mirrors/translations                                   -> convenience/searchability
4. Reverse-engineered extensions                              -> experimental only
```

If Xiaomi releases a newer official firmware/manual later, update this file by adding a dated delta section instead of silently overwriting old behavior.

---

# 33. Source-document irregularities worth preserving

These are not necessarily device bugs; they are documentation inconsistencies.

1. `0x700B` is printed as `imit_torque`; normalized alias is `limit_torque`.
2. Position mapping is printed as `-4*pi..+4*pi`, but official sample code uses `-12.5..+12.5`.
3. The insulation-resistance line contains contradictory `DC 500VAC` wording.
4. Section numbering duplicates `3.4.2` in the Chinese manual.
5. Official `motor_enable()` example appears to overwrite `master_id` with zero before sending, effectively assuming host ID 0.
6. Debugger screenshots/version strings (`0.1.5`, 2022) coexist with protocol features documented for firmware `1.2.1.5`.
7. Gain/reference values differ between the `0x2000` debugger table and the later `0x7000` runtime table; keep both instead of merging them.

---

# 34. Legal / warranty summary from supplied manual

`[OFFICIAL-MANUAL]`

The original PDF contains Chinese legal and after-sales terms. For technical grep use, they are condensed here:

- Use the product within manual-specified parameters.
- Damage caused by incorrect use, modification, abnormal conditions, impacts/liquid, natural disasters, exceeding peak torque, non-original product, or other non-product quality causes may be excluded from warranty.
- The source manual states consumer return/exchange/repair periods under PRC policy and notes that store policy may prevail where different.
- The manual states commercial application as an exclusion in its listed non-warranty conditions.
- Refer to the original PDF and current Xiaomi service policy for legal/warranty decisions.

---

# 35. AI-agent implementation checklist

When an AI coding agent uses this document, require it to verify these before writing motor-control code:

- [ ] `CAN 2.0 extended`, 29-bit ID, default `1 Mbps`.
- [ ] Correct `motor CAN_ID` and `host CAN_ID`.
- [ ] Big-endian u16 packing for `COMM_TYPE_1` motion fields.
- [ ] Little-endian index/value packing for type `0x11/0x12` parameter access unless firmware tests show otherwise.
- [ ] Clamp all commands to documented ranges.
- [ ] Stop motor before changing run mode.
- [ ] Set `limit_spd` before `loc_ref` in position mode.
- [ ] Set `limit_cur` before `spd_ref` in speed mode.
- [ ] Treat mechanical zero as volatile.
- [ ] Decode and act on feedback fault bits.
- [ ] Do not assume reverse-engineered `0x05/0x13/0x14` commands are official.
- [ ] Build an E-stop / power-disable path independent of high-level tracking software for physical systems.
- [ ] Log raw CAN arbitration ID + 8-byte payload for diagnosis.
- [ ] On firmware change, re-run a protocol smoke test and compare parameter behavior.

---

# 36. Suggested grep commands

```bash
# Find all runtime control parameters
grep -nE '0x700[0-9A-F]|0x701[0-9A-F]|0x7020' CyberGear_AI_Reference.md

# Find a specific command/register
grep -ni 'run_mode' CyberGear_AI_Reference.md
grep -ni 'limit_spd' CyberGear_AI_Reference.md
grep -ni 'CAN_TIMEOUT' CyberGear_AI_Reference.md

# Find communication types
grep -n 'COMM_TYPE_' CyberGear_AI_Reference.md

# Find fault definitions
grep -niE 'fault|warning|over-temperature|under-voltage' CyberGear_AI_Reference.md

# Find official vs unofficial content
grep -n '\[OFFICIAL-MANUAL\]' CyberGear_AI_Reference.md
grep -n '\[COMMUNITY-RE\]' CyberGear_AI_Reference.md

# Find version-sensitive notes
grep -niE 'firmware|1\.2\.1\.5|version' CyberGear_AI_Reference.md
```

---

# 37. Canonical source list

## Supplied source

- `CyberGear微电机使用说明书.pdf`
- 30 pages
- Chinese
- Treated as the canonical source for this reference.

## Online references audited 2026-08-31

- Xiaomi official product page: `https://www.mi.com/cyber-gear`
- English translation collection: `https://github.com/belovictor/cybergear-docs`
- AIFITLAB micro-motor manual: `https://wiki.aifitlab.com/xiaomi-cybergear-docs/xiaomi-cybergear-micro-motor-user-manual`
- AIFITLAB host-computer manual: `https://wiki.aifitlab.com/xiaomi-cybergear-docs/xiaomi-cybergear-host-computer-user-manual`
- Python/SocketCAN implementation: `https://github.com/grrodre/cybergear`
- ESP-IDF implementation: `https://github.com/cybergear-robotics/cybergear`

---

# 38. Change-log template for future updates

Append future findings here rather than silently rewriting protocol semantics.

```text
## YYYY-MM-DD update
source:
official/unofficial:
firmware:
changed fields:
compatibility impact:
hardware verified:
notes:
```

End of reference.
