---
title: "BNO08X (BNO085/BNO086) - AI/Grep-Friendly Technical Reference"
product_family: "BNO08X"
devices:
  - BNO085
  - BNO086
document_source: "BNO080_085-Datasheet.pdf"
source_document_number: "1000-3927"
source_revision: "1.17"
source_date: "2023-07-24"
source_pages: 59
vendor: "CEVA / Hillcrest Labs; physical device manufactured by Bosch Sensortec"
document_type: "normalized technical reference + source-preserving transcript"
canonicality:
  hardware_and_device_behavior: "BNO08X datasheet v1.17"
  SH2_command_details: "See companion SH2_AI_Reference.md"
  SHTP_channel_usage: "See companion SH2_SHTP_AI_Reference.md"
grep_keywords:
  - BNO085
  - BNO086
  - BNO08X
  - SHTP
  - I2C
  - SPI
  - UART-SHTP
  - UART-RVC
  - rotation vector
  - game rotation vector
  - gyro rotation vector
  - tare
  - calibration
  - DCD
  - FRS
  - system orientation
  - quaternion
  - H_INTN
  - report interval
  - sensor status
---

# BNO08X - AI/Grep-Friendly Technical Reference

This is a normalized engineering reference derived from **BNO08X Data Sheet 1000-3927, revision 1.17**.  
It is optimized for AI agents, code search, and `grep`. A page-by-page source-preserving transcript is appended so that details omitted from the normalized sections remain searchable.

## 0. Source trust labels

- `[DATASHEET]` - directly stated by BNO08X Data Sheet v1.17.
- `[NORMALIZED]` - restated into a machine-friendly table/formula without changing meaning.
- `[CROSS-DOC]` - depends on the companion SH-2 or SH-2 SHTP manual.
- `[CAUTION]` - integration detail that can cause communication, calibration, or orientation faults.
- `[SOURCE-NOTE]` - wording/version caveat preserved rather than silently reconciled.

Authority order inside this bundle:

```text
BNO08X datasheet v1.17     -> BNO08X hardware, interfaces, performance, device-level behavior
SH-2 Reference Manual v1.9 -> SH-2 reports, commands, FRS formats, tare/calibration API
SH-2 SHTP Reference v1.6   -> SH-2-specific SHTP advertisement/channel supplement
```

The generic **Sensor Hub Transport Protocol Reference Manual 1000-3535** is referenced by these documents but was **not included** in the uploaded set.

---

# 1. Quick grep index

| Search term | Meaning |
|---|---|
| `PROTOCOL_SELECT` | PS1/PS0/BOOTN interface-selection truth table |
| `PINOUT` | 28-pin BNO08X pin table |
| `I2C_INTEGRATION` | address, rate, clock stretching, polling behavior |
| `SPI_INTEGRATION` | mode 3, max clock, H_INTN, WAKE |
| `UART_SHTP` | 3 Mb/s SHTP UART |
| `UART_RVC_PACKET` | 100 Hz, 19-byte simplified robot packet |
| `SHTP_HEADER` | 4-byte SHTP header |
| `SHTP_CHANNELS` | BNO08X channel numbers 0..5 |
| `CONTROL_REPORT_IDS` | Set/Get Feature, FRS, command IDs |
| `FRS_RECORDS` | BNO08X supported Flash Record System IDs |
| `SET_FEATURE` | feature enable/report interval payload |
| `STATUS_ACCURACY` | unreliable/low/medium/high sensor status |
| `TIMESTAMPING` | 100 us delay/base-delta rules |
| `ROTATION_VECTOR_CHOICES` | RV vs Game RV vs Gyro RV |
| `SYSTEM_ORIENTATION` | FRS `0x2D3E`, Q30 quaternion |
| `TARE` | all-axis / Z-axis tare behavior |
| `CALIBRATION` | accel/gyro/mag dynamic calibration behavior |
| `CALIBRATION_STEPS` | forced calibration movements |
| `DCD_PERSISTENCE` | Dynamic Calibration Data save behavior |
| `MAX_REPORT_RATES` | maximum data rates |
| `LATENCY` | typical rotation-vector latency |
| `BOOTLOADER` | BOOTN behavior and bootloader I2C address |
| `POWER_SEQUENCE` | VDD vs VDDIO requirement |

---

# 2. Device identity and variants

`[DATASHEET]`

The BNO08X family integrates:

- triaxial accelerometer;
- triaxial gyroscope;
- triaxial magnetometer;
- 32-bit ARM Cortex-M0+ microcontroller;
- CEVA SH-2 firmware and MotionEngine sensor fusion.

Package:

```text
28-pin LGA
3.8 mm x 5.2 mm x 1.1 mm
```

### BNO085 vs BNO086

| Feature | BNO085 | BNO086 |
|---|---:|---:|
| Pinout | same | same |
| Software feature set | baseline | compatible enhancement |
| Drop-in replacement relationship | - | BNO086 may replace BNO085 |
| 14-bit accelerometer fusion | no | yes |
| Reduced idle power | baseline | yes |
| Interactive Calibration | no | yes |

`[SOURCE-NOTE]` The document family name is BNO08X and revision 1.17 focuses on BNO085/BNO086. Older BNO080 references were removed in prior revisions.

---

# 3. Physical sensors and configured ranges

`[DATASHEET]`

| Physical sensor | Internal capability / configured range |
|---|---|
| Accelerometer | triaxial; SH-2 configures `+/-8 g` |
| Gyroscope | triaxial, 16-bit; SH-2 configures `+/-2000 deg/s` |
| Magnetometer | triaxial geomagnetic sensor |

The datasheet describes the accelerometer as 12-bit in the functional overview. BNO086 adds 14-bit accelerometer **fusion**, which is a product-level enhancement and should not be confused with the physical-sensor description.

---

# 4. Electrical operating conditions

## POWER_SEQUENCE

`[DATASHEET] [CAUTION]`

The device has two supplies:

```text
VDD   = sensor supply
VDDIO = MCU core + digital interface supply
```

Recommended operating conditions:

| Parameter | Min | Max |
|---|---:|---:|
| `VDDIO` | `1.7 V` | `3.6 V` |
| `VDD` | `2.4 V` | `3.6 V` |
| Operating temperature | `-40 degC` | `+85 degC` |

Power sequencing requirement:

> VDD must be powered and at the specified level **before or at the same time as VDDIO**.

Absolute maximums from the source include:

```text
VDDIO: -0.3 .. 3.63 V
VDD:   -0.3 .. 4.25 V
logic pin: <= VDDIO + 0.3 V
```

---

# 5. PINOUT

`[DATASHEET]`

| Pin | Name | Direction | Meaning |
|---:|---|---|---|
| 1 | `RESV_NC` | NC | reserved; no connect |
| 2 | `GND` | input | ground |
| 3 | `VDD` | input | sensor supply, 2.4-3.6 V |
| 4 | `BOOTN` | input | bootloader mode select |
| 5 | `PS1` | input | protocol select 1 |
| 6 | `PS0/WAKE` | input | protocol select 0; SPI wake after reset |
| 7 | `RESV_NC` | input | reserved; no connect |
| 8 | `RESV_NC` | NC | reserved; no connect |
| 9 | `CAP` | external | `100 nF` to GND |
| 10 | `CLKSEL0` | input | clock source select; internal pulldown |
| 11 | `NRST` | input | active-low reset |
| 12 | `RESV_NC` | NC | reserved |
| 13 | `RESV_NC` | NC | reserved |
| 14 | `H_INTN` | output | active-low interrupt to host |
| 15 | `ENV_SCL` | bidirectional | environmental-sensor I2C clock |
| 16 | `ENV_SDA` | bidirectional | environmental-sensor I2C data |
| 17 | `SA0/H_MOSI` | input | I2C address bit / SPI MOSI |
| 18 | `H_CSN` | input | SPI chip select, active low |
| 19 | `H_SCL/SCK/RX` | bidirectional | host I2C SCL / SPI SCK / UART RX |
| 20 | `H_SDA/H_MISO/TX` | bidirectional | host I2C SDA / SPI MISO / UART TX |
| 21 | `RESV_NC` | NC | reserved |
| 22 | `RESV_NC` | NC | reserved |
| 23 | `RESV_NC` | NC | reserved |
| 24 | `RESV_NC` | NC | reserved |
| 25 | `GND` | input | ground |
| 26 | `XOUT32/CLKSEL1` | output/input at startup | 32 kHz crystal output / clock source selection |
| 27 | `XIN32` | input | 32 kHz crystal input / external clock |
| 28 | `VDDIO` | input | MCU + I/O supply, 1.65/1.7-3.6 V depending on source table |

`[CAUTION]` Reserved/no-connect pins should not be repurposed.

---

# 6. Clock source

`[DATASHEET]`

The BNO08X can use:

- internal oscillator;
- external `32.768 kHz` clock;
- external `32.768 kHz` crystal.

CEVA recommends approximately `50 ppm` tolerance. For a crystal, the datasheet recommends `12.5 pF` loading.

## Clock selection

| Source | `CLKSEL0` | `CLKSEL1` |
|---|---:|---|
| Crystal | `0` or unconnected | connected to crystal |
| External clock | `1` | `1` |
| Internal | `1` | `0` or unconnected |

`[CAUTION]` The internal clock **must not be used with UART-SHTP or UART-RVC**.

---

# 7. PROTOCOL_SELECT

`[DATASHEET]`

The protocol select pins and BOOTN are sampled at reset.

| `PS1` | `PS0` | Application mode (`BOOTN=1`) | Bootloader (`BOOTN=0`) |
|---:|---:|---|---|
| 0 | 0 | I2C | I2C |
| 0 | 1 | UART-RVC | reserved |
| 1 | 0 | UART-SHTP | UART |
| 1 | 1 | SPI | SPI |

After SPI selection, `PS0` is repurposed as the active-low `WAKE` signal.

Recommended boot connection:

```text
BOOTN -> 10 kOhm pull-up
optional host GPIO -> BOOTN for DFU entry
```

---

# 8. I2C_INTEGRATION

`[DATASHEET]`

```text
role: BNO08X is I2C slave to host
supported rates: 100 kbps Standard Mode, 400 kbps Fast Mode
7-bit address: 0x4A or 0x4B
address LSB: SA0 pin, sampled at reset
byte bit order: MSB first on wire
multi-byte data: little-endian (least-significant byte first)
```

Typical host-bus pullups:

```text
2 kOhm .. 4 kOhm
```

`H_INTN`:

- active low;
- should connect to host GPIO capable of wake;
- indicates BNO08X requires service.

### Required host behavior

`[CAUTION]`

- Host **must support I2C clock stretching**.
- BNO08X SHTP application is **not a normal register interface**.
- The BNO08X application does **not support the repeated-start method typical of register-based I2C devices**.

### BNO085 vs BNO086 when polled with no data

```text
BNO085: may stretch the clock until data is available.
BNO086: returns a zero-data packet; host reads length and ignores zero-length packets.
```

This BNO086 polling clarification is one of the changes in datasheet revision 1.17.

---

# 9. UART_SHTP

`[DATASHEET]`

```text
rate: 3 Mb/s
data: 8 bits
stop: 1 bit
parity: none
idle: high
bit order within byte: LSB first
protocol: SHTP with UART framing
clock requirement: external clock or crystal
```

Host-to-BNO byte spacing:

```text
>= 100 us between bytes
```

BNO-to-host bytes do not require that extra spacing.

`H_INTN` is driven low before UART transmission and can be used by the host to timestamp the start of transmission.

---

# 10. SPI_INTEGRATION

`[DATASHEET]`

```text
wire count: 4
BNO08X role: slave
SPI mode: 3
CPOL: 1
CPHA: 1
clock idle: high
capture edge: rising
max SCK: 3 MHz
byte transfer: MSB first
```

## H_INTN service timing

`[CAUTION]`

For BNO085/BNO086, if the host fails to respond to `H_INTN` within approximately `10 ms`, the BNO may time out, deassert the interrupt and retry. Repeated delays can starve processing and produce output errors.

The source recommends servicing `H_INTN` typically within:

```text
1/10 of the fastest enabled sensor period
```

## SPI wake

```text
PS0/WAKE active low
host drives WAKE low
BNO08X asserts H_INTN when awake
host asserts H_CSN / begins SPI transaction
```

---

# 11. UART_RVC_PACKET

`[DATASHEET]`

UART-RVC is a simplified robot-oriented mode.

```text
baud: 115200
format: 8N1
output rate: 100 Hz
packet length: 19 bytes
clock requirement: external clock or crystal
```

Packet:

| Byte(s) | Field | Encoding |
|---|---|---|
| 0..1 | Header | `0xAAAA` |
| 2 | Index | monotonically increasing uint8, wraps 255->0 |
| 3..4 | Yaw | signed, centidegrees |
| 5..6 | Pitch | signed, centidegrees |
| 7..8 | Roll | signed, centidegrees |
| 9..10 | X acceleration | signed, mg |
| 11..12 | Y acceleration | signed, mg |
| 13..14 | Z acceleration | signed, mg |
| 15 | MI | Motion Intent on BNO086; otherwise reserved |
| 16 | MR | Motion Request on BNO086; otherwise reserved |
| 17 | Reserved | source notes one reserved byte on BNO086; older parts use more reserved positions in the logical definition |
| 18 | Checksum | byte sum of payload fields described by source |

Angles:

```text
yaw:   +/-180 deg, 0.01 deg/count
pitch: +/-90 deg,  0.01 deg/count
roll:  +/-180 deg, 0.01 deg/count
```

To interpret the orientation, apply rotations in this order:

```text
yaw -> pitch -> roll
```

---

# 12. SHTP_HEADER

`[DATASHEET]`

All non-UART-RVC host traffic uses SHTP.

Four-byte header:

| Byte | Field |
|---:|---|
| 0 | Length LSB |
| 1 | Length MSB |
| 2 | Channel |
| 3 | Sequence number |

Length:

```text
bit 15    = continuation flag
bits 14:0 = total bytes in cargo + 4-byte header
```

The header is included in the length.

Important behavior:

- `0xFFFF` length is reserved.
- BNO08X **does not support receiving fragmented messages**.
- BNO08X **does support sending fragmented messages**.
- Partial reads are supported.
- Each channel/direction has an independent sequence number.

---

# 13. SHTP_CHANNELS

`[DATASHEET]`

BNO08X fixed application-level channel mapping:

| Channel | Purpose |
|---:|---|
| 0 | SHTP command channel |
| 1 | executable |
| 2 | SH-2 / sensor-hub control |
| 3 | normal non-wake input sensor reports, excluding gyro RV |
| 4 | wake input sensor reports |
| 5 | gyro rotation vector |

Executable channel command values:

```text
host write:
1 = reset
2 = on
3 = sleep

device read:
1 = reset complete
```

When `sleep` is issued, always-on/wake sensors continue running; other sensors are disabled. `on` reenables configured sensors.

---

# 14. CONTROL_REPORT_IDS

`[DATASHEET]`

SH-2 control channel (`channel 2`) report IDs used by BNO08X:

| ID | Direction | Meaning |
|---:|---|---|
| `0xFE` | host -> BNO | Get Feature Request |
| `0xFD` | host -> BNO | Set Feature Command |
| `0xFC` | BNO -> host | Get Feature Response |
| `0xF9` | host -> BNO | Product ID Request |
| `0xF8` | BNO -> host | Product ID Response |
| `0xF7` | host -> BNO | FRS Write Request |
| `0xF6` | host -> BNO | FRS Write Data |
| `0xF5` | BNO -> host | FRS Write Response |
| `0xF4` | host -> BNO | FRS Read Request |
| `0xF3` | BNO -> host | FRS Read Response |
| `0xF2` | host -> BNO | Command Request |
| `0xF1` | BNO -> host | Command Response |

For exact byte layouts, see `SH2_AI_Reference.md`.

---

# 15. Product ID

`[DATASHEET]`

Request report:

```text
byte 0 = 0xF9
byte 1 = reserved
```

Response `0xF8` includes:

```text
reset cause
SW version major
SW version minor
SW part number (32 bit)
SW build number (32 bit)
SW version patch (16 bit)
reserved bytes
```

This is the preferred way to identify running firmware rather than assuming a software version from the PDF.

---

# 16. FRS_RECORDS

`[DATASHEET]`

Supported configuration records explicitly listed by the BNO08X datasheet include:

| Record ID | Description |
|---:|---|
| `0x7979` | Static calibration - AGM |
| `0x4D4D` | Nominal calibration - AGM |
| `0x8A8A` | Static calibration - SRA |
| `0x4E4E` | Nominal calibration - SRA |
| `0x1F1F` | Dynamic calibration |
| `0xD3E2` | MotionEngine power management |
| `0x2D3E` | System orientation |
| `0x2D41` | Primary accelerometer orientation |
| `0x2D46` | Gyroscope orientation |
| `0x2D4C` | Magnetometer orientation |
| `0x3E2D` | AR/VR stabilization - rotation vector |
| `0x3E2E` | AR/VR stabilization - game rotation vector |
| `0xC274` | Significant Motion detector configuration |
| `0x7D7D` | Shake detector configuration |
| `0xD7D7` | Maximum fusion period |
| `0x4B4B` | Serial number |
| `0x39AF` | Pressure calibration |
| `0x4D20` | Temperature calibration |
| `0x1AC9` | Humidity calibration |
| `0x39B1` | Ambient-light calibration |
| `0x4DA2` | Proximity calibration |
| `0xD401` | ALS calibration |
| `0xD402` | Proximity-sensor calibration |
| `0xED85` | Stability detector configuration |
| `0x74B4` | User record |
| `0xD403` | MotionEngine time source selection |
| `0xA1A2` | Gyro-Integrated Rotation Vector configuration |

`[CROSS-DOC]` The SH-2 reference manual contains the fuller configuration-record catalog and field formats.

---

# 17. SET_FEATURE

`[DATASHEET]`

Set Feature command (`0xFD`) enables/configures a virtual sensor.

Payload after SHTP header:

| Byte | Field |
|---:|---|
| 0 | `0xFD` |
| 1 | Feature Report ID |
| 2 | Feature flags |
| 3..4 | Change sensitivity, LSB first |
| 5..8 | Report interval in microseconds, LSB first |
| 9..12 | Batch interval in microseconds, LSB first |
| 13..16 | Sensor-specific configuration word, LSB first |

Typical use:

```text
report_interval = 0 -> disable sensor
report_interval > 0 -> enable sensor
```

The BNO08X may return a period different from the requested value because the underlying physical sensor supports discrete operating rates. A Get Feature Response can also be emitted later if the effective rate changes.

---

# 18. STATUS_ACCURACY

`[DATASHEET]`

Sensor reports use status bits `1:0`:

```text
0 = Unreliable
1 = Accuracy low
2 = Accuracy medium
3 = Accuracy high
```

For implementation, do not treat a quaternion/vector as equally trustworthy regardless of status. This field is especially useful for detecting weak calibration or invalid startup conditions.

---

# 19. TIMESTAMPING

`[DATASHEET]`

Sensor report delay has:

```text
status bits 7:2 = upper 6 bits of delay
delay byte      = lower 8 bits
resolution      = 100 us
```

A Timebase Reference report is:

```text
Report ID = 0xFB
Base Delta = signed 32-bit value
units = 100 us ticks
```

The host should timestamp `H_INTN`, then combine/subtract the timebase reference and per-report delay as described by SH-2.

Example source logic:

```text
host interrupt time = T
base delta = 120 ticks = 12 ms

first sample, delay 0   -> T - 12 ms
second sample, delay 17 -> T - 12 ms + 1.7 ms = T - 10.3 ms
```

See the SH-2 reference for batching and Timestamp Rebase `0xFA`.

---

# 20. ROTATION_VECTOR_CHOICES

`[DATASHEET]`

## Geomagnetic Rotation Vector

Inputs:

```text
accelerometer + magnetometer
no gyroscope
```

Reference:

```text
gravity + magnetic north
```

Tradeoffs:

- lower power;
- less responsive;
- more vulnerable to magnetic-field errors.

## Game Rotation Vector

Inputs:

```text
accelerometer + gyroscope
no magnetometer
```

Reference:

```text
roll/pitch referenced to gravity
heading has no absolute reference
```

Properties:

- smooth;
- avoids magnetic correction jumps;
- yaw will drift over time.

## Rotation Vector

Inputs:

```text
accelerometer + gyroscope + magnetometer
```

Reference:

```text
gravity + magnetic north
```

The datasheet calls this the most accurate orientation estimate available. Magnetometer corrects yaw drift; gyro gives responsiveness.

## AR/VR Stabilized Game Rotation Vector

Game RV with correction behavior modified to reduce visually disturbing discontinuities/jumps.

## AR/VR Stabilized Rotation Vector

9-axis RV with correction behavior modified to reduce abrupt corrections.

## Gyro Rotation Vector / Gyro-Integrated RV

Optimized low-latency path for head tracking.

```text
maximum report rate: up to 1000 Hz
```

It may use either a magnetometer-based rotation vector or a game rotation vector as its slower reference, depending on FRS configuration.

---

# 21. SYSTEM_ORIENTATION

`[DATASHEET]`

The BNO08X can be physically mounted in an arbitrary orientation, then mapped into the device frame.

System orientation FRS:

```text
record ID: 0x2D3E
representation: unit quaternion
component storage: 32-bit signed fixed point
Q point: 30
applies to: sensor outputs + derived outputs such as rotation vectors
```

Default device coordinate convention follows Android:

```text
+X = horizontal/right
+Y = vertical/up along device face
+Z = outward from front face
positive angular rotation = counter-clockwise
```

`[CAUTION]` The datasheet examples often print quaternions as `(Qw,Qx,Qy,Qz)` for readability, while the SH-2 FRS **record word order is X, Y, Z, W**. See `SH2_AI_Reference.md` before serializing the record.

### 90-degree mounting examples from the datasheet

Assuming device axes `X=East, Y=North, Z=Up`:

| BNO physical X | BNO physical Y | BNO physical Z | Qw | Qx | Qy | Qz |
|---|---|---|---:|---:|---:|---:|
| East | North | Up | 1 | 0 | 0 | 0 |
| North | West | Up | sqrt(2)/2 | 0 | 0 | sqrt(2)/2 |
| West | South | Up | 0 | 0 | 0 | 1 |
| South | East | Up | sqrt(2)/2 | 0 | 0 | -sqrt(2)/2 |
| East | South | Down | 0 | 0 | -1 | 0 |

The original page contains the complete 24-orientation table and is preserved in the transcript.

---

# 22. TARE

`[DATASHEET]`

Tare is a runtime/user-controlled reorientation function.

Purpose:

```text
mount BNO08X arbitrarily
invoke tare
SH-2 determines output rotation needed to align with East/North/Up frame
apply to motion outputs
```

Two tare types:

```text
all-axis tare -> solves tilt + heading
Z-axis tare   -> solves heading only
```

### Magnetic-north requirement

`[CAUTION]`

If using Rotation Vector or Geomagnetic Rotation Vector, magnetic North should be resolved before taring. Otherwise heading can change later when magnetometer calibration converges.

### Persistence

A Persist Tare operation can store the orientation configuration.

Special case:

```text
Game Rotation Vector has no absolute heading reference.
Persist tare does not apply to Game Rotation Vector.
A Game-RV Z tare has special reset/runtime behavior.
```

For exact command bytes, basis-vector choices, and persistence rules see `SH2_AI_Reference.md` section `TARE_COMMAND`.

---

# 23. CALIBRATION

`[DATASHEET]`

There are two broad calibration classes:

```text
static calibration  -> non-varying sensor/system characteristics
dynamic calibration -> time/temperature-varying corrections during operation
```

Static calibration may correct:

- cross-axis/skew;
- gain/sensitivity;
- sensor orientation relative to device frame.

Dynamic calibration examples:

- gyro zero-rate offset (ZRO);
- accelerometer zero-g offset (ZGO);
- magnetic hard/soft iron compensation.

The host can enable/disable dynamic calibration routines through SH-2 commands. These enable settings **do not persist across reset**.

## Accelerometer dynamic calibration

Goal: remove zero-g offset.

Symptoms of poor calibration can include:

- apparent tilt;
- horizon/tilt correction errors.

Modes:

```text
3D calibration
planar calibration (for devices constrained to a plane)
```

## Gyroscope dynamic calibration

Goal: remove zero-rate offset.

At rest:

```text
expected gyro ~= 0 rad/s on all axes
```

A stable surface causes ZRO calibration to converge quickly.

`[CAUTION]` Slow rotation around gravity can fool the calibration algorithm. If the application can remain slowly rotating with insufficient natural tremor, the datasheet advises considering disabling gyro calibration to prevent mis-calibration. ZRO will still be corrected when the device becomes very stable/on-table.

## Magnetometer dynamic calibration

Goal: compensate hard/soft iron distortion.

The datasheet strongly recommends calibration for heading-based outputs. Without magnetometer calibration, heading is described as highly suspect.

---

# 24. CALIBRATION_STEPS

`[DATASHEET]`

Explicit forced-calibration motions:

| Sensor | Procedure |
|---|---|
| Accelerometer, 3D | place in `4-6` unique orientations; hold each about `1 s` |
| Accelerometer, planar | rotate around Z axis by at least `180 deg` |
| Gyroscope | stationary surface for approximately `2-3 s` |
| Magnetometer | rotate about `180 deg` and back on each roll/pitch/yaw axis; about `2 s` per axis |

The datasheet notes that normal use often naturally exposes the device to conditions sufficient for calibration, so explicit user calibration is not always required.

---

# 25. Recommended fusion/calibration choices

`[DATASHEET]`

### Stable magnetic environment, general orientation

Recommended:

```text
9-axis Rotation Vector
```

Gyroscope calibration may be enabled if the device motion profile provides sufficient tremor; verify application-specific behavior.

### Unstable magnetic environment

Consider:

```text
Game Rotation Vector
```

### AR/VR / smooth output

The datasheet recommends avoiding dynamic magnetometer calibration during use if correction jumps would be disturbing, and using:

```text
Game Rotation Vector
or
Gyro Rotation Vector
```

with accelerometer calibration enabled.

---

# 26. DCD_PERSISTENCE

`[DATASHEET]`

Dynamic Calibration Data (DCD):

```text
updated to RAM approximately every 5 s
```

On a non-power-up reset, the hub can persist the last RAM DCD to FRS.

SH-2 also provides:

```text
Save DCD command
Configure Periodic DCD Save command
Clear DCD and Reset command
```

See the SH-2 command reference for exact message layouts.

---

# 27. Bootloader

## BOOTLOADER

`[DATASHEET]`

At reset/power-up:

```text
BOOTN = low  -> bootloader
BOOTN = high -> application
```

Bootloader transport follows PS0/PS1 selection.

Bootloader I2C address:

```text
0x28 or 0x29
SA0 selects lowest bit
```

The application image is written in frames and each write is status-checked.

---

# 28. Startup behavior

`[DATASHEET]`

Non-UART-RVC startup:

1. device resets;
2. internal initialization;
3. BNO08X asserts `H_INTN`;
4. host reads initial SHTP advertisement;
5. executable emits reset message on channel 1;
6. SH-2 emits unsolicited initialization message on channel 2;
7. all sensors remain disabled until host configuration.

Approximate startup timing table:

```text
NRST minimum: 10 ns
internal initialization: ~90 ms typical
internal configuration: ~4 ms typical
```

Host should wait for `H_INTN` rather than relying only on a fixed delay.

---

# 29. MAX_REPORT_RATES

`[DATASHEET]`

| Sensor / virtual sensor | Maximum data rate |
|---|---:|
| Gyro rotation vector | `1000 Hz` |
| Rotation vector | `400 Hz` |
| Game rotation vector | `400 Hz` |
| Geomagnetic rotation vector | `90 Hz` |
| Gravity | `400 Hz` |
| Linear acceleration | `400 Hz` |
| Accelerometer | `500 Hz` |
| Gyroscope | `400 Hz` |
| Magnetometer | `100 Hz` |

AR/VR-stabilized versions have the same maximum report rates as their corresponding non-stabilized vectors.

`[CAUTION]` All sensors cannot necessarily operate at maximum rate simultaneously. Interface bandwidth and BNO08X processing capacity must be budgeted.

The datasheet describes the configured-rate relationship approximately as:

```text
0.9 * RequestedRate <= ConfiguredRate <= 2.1 * RequestedRate
```

---

# 30. LATENCY

`[DATASHEET]`

Typical latency table:

| Output | 100 Hz | 200 Hz |
|---|---:|---:|
| Gyro rotation vector | `6.6 ms` | `3.7 ms` |
| Rotation vector | `6.6 ms` | `3.7 ms` |
| Game rotation vector | `6.6 ms` | `3.7 ms` |

Latency sources include:

- physical sensor delay;
- BNO processing delay;
- algorithm/filter delay;
- host-interface/interrupt/communication delay.

---

# 31. Nominal orientation performance

`[DATASHEET]`

Selected source performance values:

| Output | Metric | Value |
|---|---|---:|
| Rotation Vector | dynamic rotation error | `3.5 deg` |
| Rotation Vector | static rotation error | `2.0 deg` |
| Game Rotation Vector | dynamic non-heading error | `2.5 deg` |
| Game Rotation Vector | static non-heading error | `1.5 deg` |
| Game Rotation Vector | dynamic heading drift | `0.5 deg/min` |
| Geomagnetic Rotation Vector | dynamic rotation error | `4.5 deg` |
| Geomagnetic Rotation Vector | static rotation error | `3.0 deg` |
| Gravity | static angle error | `1.5 deg` |
| Linear acceleration | dynamic accuracy | `0.35 m/s^2` |
| Accelerometer | dynamic accuracy | `0.3 m/s^2` |
| Gyroscope | dynamic accuracy | `3.1 deg/s` |
| Magnetometer | dynamic accuracy | `1.4 uT` |

`[SOURCE-NOTE]` The datasheet says the performance table was generated using simulation based on characterization of 210 physical devices, and notes real-world magnetic environment strongly affects heading.

---

# 32. Implementation checklist

- [ ] Select interface with `PS1/PS0` before reset.
- [ ] Keep `BOOTN` high for normal application.
- [ ] Respect VDD-before-or-with-VDDIO sequencing.
- [ ] Wait for `H_INTN` after reset.
- [ ] If I2C: support clock stretching and do not model device as a register peripheral.
- [ ] If SPI: use mode 3, `CPOL=1`, `CPHA=1`, <=3 MHz.
- [ ] Service `H_INTN` promptly.
- [ ] Parse the 4-byte SHTP header before cargo.
- [ ] Maintain sequence number independently per channel/direction.
- [ ] Enable sensors via `Set Feature (0xFD)`.
- [ ] Accept that returned effective period can differ from requested period.
- [ ] Decode sensor `Status` accuracy bits.
- [ ] Use timestamp/base-delta fields, not host receipt time alone, for motion verification.
- [ ] Configure `System Orientation (0x2D3E)` or runtime reorientation for installation alignment.
- [ ] Choose rotation-vector type according to magnetic environment and yaw-drift requirements.
- [ ] Do not assume tare or calibration enable settings have identical persistence semantics.
- [ ] Query Product ID/version during startup and log it.
- [ ] Log raw SHTP channel, sequence number, report ID, status, and timestamp for diagnostics.

---

# 33. Suggested grep commands

```bash
grep -ni 'SYSTEM_ORIENTATION' BNO08X_AI_Reference.md
grep -ni 'TARE' BNO08X_AI_Reference.md
grep -ni 'CALIBRATION_STEPS' BNO08X_AI_Reference.md
grep -ni 'Game Rotation Vector' BNO08X_AI_Reference.md
grep -ni 'Gyro rotation vector' BNO08X_AI_Reference.md
grep -ni 'H_INTN' BNO08X_AI_Reference.md
grep -ni '0x2D3E' BNO08X_AI_Reference.md
grep -ni '0xA1A2' BNO08X_AI_Reference.md
grep -ni '0xFD' BNO08X_AI_Reference.md
grep -ni 'BNO086' BNO08X_AI_Reference.md
```

---

# 34. Source version history checkpoint

The supplied datasheet is:

```text
Document: BNO08X Data Sheet
Document number: 1000-3927
Revision: 1.17
Revision date: July 24, 2023
```

The revision history says v1.17 changed the I2C section, including BNO086 polling behavior.

---


# Source-preserving transcript

This appendix preserves the PDF's extracted text page-by-page for exhaustive grep/search. The normalized sections above should be preferred for implementation, while this transcript is useful for locating source wording, tables, figures, and fields that were not promoted into the normalized reference.


## PDF page 1

```text
                                                                                  BNO08X Data Sheet

The BNO08X family (BNO085/BNO086) is a System in Package (SiP) that integrates a triaxial accelerometer,
triaxial gyroscope, magnetometer and a 32-bit ARM® Cortex™-M0+ microcontroller running CEVA's SH-2
firmware. The SH-2 includes the MotionEngine™ software, which provides sophisticated signal processing
algorithms to process sensor data and provide precise real-time 3D orientation, heading, calibrated acceleration
and calibrated angular velocity, as well as more advanced contextual outputs. The BNO08X is integrated into a
single 28 pin LGA 3.8mm x 5.2mm x 1.1mm package. It is compatible with Android and provides a turn-key
sensor hub solution, eliminating the complexity and investment associated with a discrete design.
The BNO08X features an enhanced sensor fusion algorithm which enables the use of add-on host-based
software products for the AR/VR and other specialized markets. The BNO086 may be used as a drop-in
replacement for the BNO085, with an identical pinout and software feature set. The BNO086 is a further
enhancement over the BNO085 offering 14-bit accelerometer fusion, reduces idle state power and Interactive
Calibration.


                                                     Turn-key Sensor Hub Solution

                                                                  BNO080
                                                                BNO08X
                                                                               SH-2

                                                       MotionEngine                                Host Interface Block

                Accelerometer
                                                                                                        Power
                                                                                                      Management
                 Gyroscope
                                           Device                                     Always-on          Sensor             Host
                                                        Calibration                   Features          Batching             I/F
                Magnetometer               Drivers
                                 SPI                                  Sensor
       I2C                                                                                              Report
                                                                      Fusion                          Management
                                                                                     Activity
                                                                                  Classification        Sensor
                                                                                                     Configuration &
                                                                                                          Rate
                                                                                                      Management




   ➢     Android compliant 4.4 KitKat and above
   ➢     Variety of 3D orientation outputs (including linear acceleration, rotation vectors, gravity)
   ➢     Includes “always-on” and classification features (step counter, stability and tap detectors, and a variety of gestures)
   ➢     Dynamically calibrates sensor data for temperature and aging




                                                 BNO08X Datasheet Revision 1.17                                                    1 / 59
© 2023 CEVA, Inc. All rights reserved                                                                              www.ceva-dsp.com
```


## PDF page 2

```text
                                                        BNO08X Datasheet                                      1000-3927 v1.17


Sensor Hub Features                                                Applications
•   Three sensors and microcontroller in a single device           •   Wearables such as head trackers for AR/VR
•   Supports new sensors and features in Android 4.4                   applications, smartwatches, fitness bands, audio
    KitKat                                                             headsets
•   Includes power management to optimize power                    •   Smartphones
•   Support for device firmware upgrade (DFU)                      •   Tablets
                                                                   •   Ultrabooks
Extensive Data Modes
                                                                   •   Robotics
•   Linear acceleration                                            •   Internet of Things (IoT)
•   Angular velocity
•   Angular position (quaternion)                                  BNO085 and BNO086
•   Data returned at configurable sample rates                     •   Enhanced sensor fusion algorithm to enable
•   Timestamps attached to sensor reports                              specialized CEVA host-based applications
•   AR/VR stabilization applied to rotation vectors provides       •   Optimized for use with virtual reality controller systems
    visually improved angular position output                      BNO086 Only
•   Low latency, 1kHz gyro rotation vector for AR/VR
    applications                                                   •   14-bit accelerometer fusion
                                                                   •   Lower idle power
“Always-on” and Classification Features                            •   Interactive Calibration
•   Built-in stability detector, tap detector, and step counter




www.ceva-dsp.com                              © 2023 CEVA, Inc. All rights reserved                                        2 / 59
```


## PDF page 3

```text
                                                                      BNO08X Datasheet                                                            1000-3927 v1.17



                                                        Table of Contents
LIST OF FIGURES .......................................................................................................................... 5

1         FUNCTIONAL OVERVIEW ................................................................................................ 7
    1.1     Reference Design Configurations ................................................................................... 8
       1.1.1 Standalone Sensor Hub Solution in Mobile Devices ................................................... 8
       1.1.2 Virtual Reality Head Tracker ........................................................................................ 8
       1.1.3 Unmanned Ground Roving Robot ................................................................................ 9
    1.2     BNO08X Connectivity ..................................................................................................... 9
       1.2.1 Pin Descriptions ......................................................................................................... 10
       1.2.2 I2C interface................................................................................................................ 12
       1.2.3 UART-SHTP interface ................................................................................................ 15
       1.2.4 SPI Interface .............................................................................................................. 17
       1.2.5 UART-RVC interface .................................................................................................. 20
    1.3     Host Communication .................................................................................................... 22
       1.3.1 SHTP .......................................................................................................................... 22
       1.3.2 Report Structure ......................................................................................................... 23
       1.3.3 BNO08X Configuration............................................................................................... 24
       1.3.4 Sensor Metadata ........................................................................................................ 25
       1.3.5 Sensor Reports .......................................................................................................... 26
    1.4     Bootloader .................................................................................................................... 29

2         SENSOR DATA PROCESSING ...................................................................................... 30
    2.1     Motion Outputs ............................................................................................................. 30
       2.1.1 Acceleration Outputs .................................................................................................. 30
       2.1.2 Angular Velocity Outputs............................................................................................ 31
       2.1.3 Magnetometer Processing ......................................................................................... 31
    2.2     Orientation Outputs ....................................................................................................... 31
       2.2.1 Geomagnetic Rotation Vector .................................................................................... 31
       2.2.2 Game Rotation Vector................................................................................................ 31
       2.2.3 AR/VR Stabilized Game Rotation vector ................................................................... 32
       2.2.4 Rotation Vector .......................................................................................................... 32
       2.2.5 AR/VR Stabilized Rotation Vector .............................................................................. 32
       2.2.6 Gyro rotation Vector ................................................................................................... 32
       2.2.7 Gyro rotation Vector Prediction .................................................................................. 32
    2.3     Environmental Sensors................................................................................................. 33
    2.4     Classification System .................................................................................................... 33
       2.4.1 Stability Detection and Classification ......................................................................... 33
       2.4.2 Tap Detector .............................................................................................................. 34
       2.4.3 Step Detector ............................................................................................................. 34
       2.4.4 Step Counter .............................................................................................................. 35
       2.4.5 Activity Classification .................................................................................................. 35
       2.4.6 Significant Motion Detector ........................................................................................ 36
       2.4.7 Shake Detector .......................................................................................................... 36

3         CALIBRATION AND INTERPRETATION ....................................................................... 37
    3.1     Calibration Effects ......................................................................................................... 37
       3.1.1 Calibration Command ................................................................................................ 37
       3.1.2 Accelerometer ............................................................................................................ 38
       3.1.3 Gyroscope .................................................................................................................. 38
       3.1.4 Magnetometer ............................................................................................................ 38
       3.1.5 Calibration Accuracy .................................................................................................. 38


www.ceva-dsp.com                                          © 2023 CEVA, Inc. All rights reserved                                                             3 / 59
```


## PDF page 4

```text
                                                                       BNO08X Datasheet                                                             1000-3927 v1.17


       3.1.6 Recommended Settings ............................................................................................. 38
    3.2     Calibration Steps .......................................................................................................... 39
    3.3     Interactive Calibration (BNO086 only) .......................................................................... 39
    3.4     Persist Dynamic Calibration Data ................................................................................. 39

4         BNO08X ORIENTATION ................................................................................................. 40
       4.1.1 Tare 42

5         GETTING STARTED WITH BNO08X .............................................................................. 43
    5.1     BNO08X in UART-RVC mode ...................................................................................... 43
    5.2     BNO08X in non UART-RVC configurations.................................................................. 43
       5.2.1 Establishing Contact .................................................................................................. 43
       5.2.2 Reading/Writing the BNO08X .................................................................................... 43

6         BNO08X CHARACTERISTICS ........................................................................................ 45
    6.1     Absolute Maximum Electrical Ratings .......................................................................... 45
    6.2     Recommended Operating Conditions .......................................................................... 45
    6.3     Power Management...................................................................................................... 45
    6.4     Electrical Characteristics .............................................................................................. 46
    6.5     AC Characteristics ........................................................................................................ 46
       6.5.1 I2C Timing................................................................................................................... 46
       6.5.2 SPI timing ................................................................................................................... 46
       6.5.3 Startup timing ............................................................................................................. 47
       6.5.4 Interrupt timing ........................................................................................................... 48
    6.6     Mechanical Characteristics ........................................................................................... 48
    6.7     Performance Characteristics ........................................................................................ 49
    6.8     Latency ......................................................................................................................... 49
    6.9     Report Rates ................................................................................................................. 50
    6.10    Power Consumption ..................................................................................................... 50

7         PACKAGING INFORMATION ......................................................................................... 53
    7.1     Package Outline ........................................................................................................... 53
    7.2     Landing Pattern Recommendation ............................................................................... 54
    7.3     Soldering Guidelines ..................................................................................................... 54
    7.4     Marking ......................................................................................................................... 54
    7.5     Soldering Guidelines ..................................................................................................... 55
    7.6     Handling Instructions .................................................................................................... 55
    7.7     Environmental Safety .................................................................................................... 55
       7.7.1 Halogen content ......................................................................................................... 55

8           VERSION HISTORY ........................................................................................................ 56

9           REFERENCES ................................................................................................................. 58

10          NOTICES .......................................................................................................................... 59




www.ceva-dsp.com                                          © 2023 CEVA, Inc. All rights reserved                                                               4 / 59
```


## PDF page 5

```text
                                                                      BNO08X Datasheet                                                          1000-3927 v1.17



                                                             List of Figures
Figure 1-1: BNO08X block diagram ...........................................................................................................................7
Figure 1-2: BNO08X in a mobile device ....................................................................................................................8
Figure 1-3: Virtual reality head tracker ......................................................................................................................8
Figure 1-4: Typical unmanned ground roving robot ..................................................................................................9
Figure 1-5: Protocol selection for BNO08X ...............................................................................................................9
Figure 1-6: BNO08X pin descriptions ..................................................................................................................... 10
Figure 1-7: 32.768kHz crystal connection .............................................................................................................. 11
Figure 1-8: Clock Source Selection ........................................................................................................................ 11
Figure 1-9: External clock connection .................................................................................................................... 11
Figure 1-10: Internal clock selection ....................................................................................................................... 12
Figure 1-11: BNO08X I2C connection diagram ....................................................................................................... 13
Figure 1-12: BNO08X I2C address ......................................................................................................................... 14
Figure 1-13: I2C START condition .......................................................................................................................... 14
Figure 1-14: I2C STOP condition ............................................................................................................................ 14
Figure 1-15: Device addressing .............................................................................................................................. 15
Figure 1-16: I2C write cycle..................................................................................................................................... 15
Figure 1-17: I2C read cycle ..................................................................................................................................... 15
Figure 1-18: BNO08X UART-SHTP connection diagram ....................................................................................... 16
Figure 1-19: UART signaling .................................................................................................................................. 17
Figure 1-20: BNO08X SPI connection diagram ...................................................................................................... 18
Figure 1-21: BNO08X SPI signaling ....................................................................................................................... 19
Figure 1-22: SPI Wake operation ........................................................................................................................... 19
Figure 1-23: BNO08X UART-RVC connection diagram ......................................................................................... 20
Figure 1-24: UART signaling .................................................................................................................................. 21
Figure 1-25: BNO08X UART-RVC packet format .................................................................................................. 21
Figure 1-26: SHTP Header ..................................................................................................................................... 22
Figure 1-27: SHTP executable commands and response...................................................................................... 23
Figure 1-28: Product ID request ............................................................................................................................. 23
Figure 1-29: Product ID Response ......................................................................................................................... 24
Figure 1-30: BNO08X Commands .......................................................................................................................... 24
Figure 1-31: FRS records ....................................................................................................................................... 25
Figure 1-32: BNO08X rotation vector metadata ..................................................................................................... 25
Figure 1-33: Set Feature command ........................................................................................................................ 27
Figure 1-34: Calibrated gyroscope input report ...................................................................................................... 28
Figure 1-35: Timebase Reference Report .............................................................................................................. 28
Figure 1-36: Timestamping example ...................................................................................................................... 28
Figure 2-1: Android co-ordinate system ................................................................................................................. 30
Figure 2-2: HMD mounted head motion prediction ................................................................................................ 33
Figure 2-3: Tap detector ......................................................................................................................................... 34
Figure 2-4: Activity classification matrix .................................................................................................................. 35
Figure 2-5: Shake gesture ...................................................................................................................................... 36
Figure 3-1: Accuracy status of sensors ................................................................................................................... 38
Figure 3-2: Calibration procedure for sensors ......................................................................................................... 39
Figure 4-1: BNO08X axis orientation ...................................................................................................................... 40
Figure 4-2: BNO08X mounted in a device .............................................................................................................. 41
Figure 4-3: Multiple 90 degree rotations ................................................................................................................. 41
Figure 5-1: BNO08X set feature report (accelerometer) including SHTP header .................................................. 44
Figure 5-2: Accelerometer & timebase input report including SHTP header ......................................................... 44
Figure 6-1: BNO08X maximum ratings................................................................................................................... 45
Figure 6-2: BNO08X operating conditions .............................................................................................................. 45
Figure 6-3: BNO08X electrical characteristics ........................................................................................................ 46
Figure 6-4: I2C timing parameters .......................................................................................................................... 46
Figure 6-5: I2C timing .............................................................................................................................................. 46
Figure 6-6: SPI timing parameters .......................................................................................................................... 47
Figure 6-7: SPI timing ............................................................................................................................................. 47

www.ceva-dsp.com                                          © 2023 CEVA, Inc. All rights reserved                                                                  5 / 59
```


## PDF page 6

```text
                                                                      BNO08X Datasheet                                                         1000-3927 v1.17


Figure 6-8: SPI timing parameters .......................................................................................................................... 47
Figure 6-9: Startup timing ........................................................................................................................................ 47
Figure 6-10: Host interrupt timing – I2C mode ........................................................................................................ 48
Figure 6-11: Host interrupt timing - SPI mode ........................................................................................................ 48
Figure 6-12: H_INTN timing .................................................................................................................................... 48
Figure 6-13: Host interrupt timing - UART-SHTP mode ......................................................................................... 48
Figure 6-14: BNO08X Performance ....................................................................................................................... 49
Figure 6-15: Typical latency measurements .......................................................................................................... 50
Figure 6-16: Maximum sensor rates ....................................................................................................................... 50
Figure 6-17: Power consumption BNO085 ............................................................................................................. 51
Figure 6-18: Power consumption BNO086 ............................................................................................................. 52
Figure 7-1: 28 pin LGA package outline (Image from Bosch) ................................................................................ 53
Figure 7-2: Landing pattern recommendation ........................................................................................................ 54
Figure 7-3: Marking of mass production parts ......................................................................................................... 55




www.ceva-dsp.com                                         © 2023 CEVA, Inc. All rights reserved                                                                 6 / 59
```


## PDF page 7

```text
                                                   BNO08X Datasheet                                         1000-3927 v1.17


1 Functional Overview
The BNO08X is manufactured by Bosch Sensortec and runs software provided by CEVA. The BNO08X integrates
a triaxial 12-bit accelerometer with a range of ±8g, triaxial 16-bit gyroscope with a range of ±2000 degrees per
second, a triaxial geomagnetic sensor, and a 32-bit ARM® Cortex™-M0+ microcontroller. The sensors are
provided by Bosch Sensortec GmbH and the Cortex M0+ processor by Atmel Corporation.
A system diagram of the BNO08X is shown in Figure 1-1.

                                                          BNO080
                                                         BNO08X
                                                                       SH-2

                                               MotionEngine                                 Host Interface Block

           Accelerometer
                                                                                                 Power
                                                                                               Management
            Gyroscope
                                     Device                                   Always-on           Sensor              Host
                                               Calibration                    Features           Batching              I/F
           Magnetometer              Drivers
                           SPI                                Sensor
  I2C                                                                                            Report
                                                              Fusion                           Management
                                                                              Activity
                                                                           Classification        Sensor
                                                                                              Configuration &
                                                                                                   Rate
                                                                                               Management




                                       Figure 1-1: BNO08X block diagram
At the heart of the BNO08X is CEVA’s SH-2 software. The SH-2 software includes MotionEngine™, ‘always-on’
features, activity classification and the host interface software. MotionEngine is digital signal processing software
that takes raw motion data from the MEMS sensors and translates this data into precise motion information.
These accurate motion outputs can be used for gesture detection and a variety of advanced motion-controlled
applications. The ‘always-on’ and activity classification features include step counter, stability detector, tap
detector, and gestures. The host interface module includes sophisticated power management functionality,
configures sensors, and handles communication with the system host. The SH-2 supports sensor types defined in
Android 4.4 KitKat. Android 5.0 defined additional methods of configuration which the BNO08X supports.
The BNO08X supports the addition of environmental sensors on a secondary I 2C interface. See 2.3.
The BNO08X can communicate with the system host over various serial interfaces: SPI, I2C and UART.




www.ceva-dsp.com                          © 2023 CEVA, Inc. All rights reserved                                       7 / 59
```


## PDF page 8

```text
                                                      BNO08X Datasheet                              1000-3927 v1.17


1.1     Reference Design Configurations
1.1.1 Standalone Sensor Hub Solution in Mobile Devices
The BNO08X can be integrated as a co-processor in a mobile device such as a smartphone, tablet or ultrabook.
This means the host processor can offload the sensor management and processing functions to the BNO08X.
The BNO08X constantly processes sensor data in the background while the host processor is in sleep state and
can provide the host processor with real-time sensor data on demand. This low power paradigm is useful to
enable a number of ‘always-on’ applications such as activity tracking, gesture recognition, fitness and pedestrian
dead reckoning.

                                      BNO080
                                      BNO085

                              Accelerometer
                                                                              Android Host
                               Gyroscope           MCU                 Application Processor (AP)

                              Magnetometer




                                       Figure 1-2: BNO08X in a mobile device



1.1.2 Virtual Reality Head Tracker
The BNO08X can be integrated into an HMD as a head tracker to allow the user to be immersed in either a virtual
reality or an augmented reality application. The BNO08X provides accurate angular position data to allow
navigation of this virtual world.



                                     BNO08X
                                     BNO080


                                    MCU for
                                                           Video
                                   Data Gather
                                                           Output


                                                                            Host SoC
                                      Head
                                     Mounted
                                     Display
                                                           USB

                                       Figure 1-3: Virtual reality head tracker




www.ceva-dsp.com                              © 2023 CEVA, Inc. All rights reserved                            8 / 59
```


## PDF page 9

```text
                                                    BNO08X Datasheet                              1000-3927 v1.17


1.1.3 Unmanned Ground Roving Robot
The BNO08X can be integrated into unmanned ground roving robot applications, providing heading information to
fuse with cameras to allow navigation (SLAM – Simultaneous Localization and Mapping application). The
BNO08X can also provide details on the tilt of the device and by virtue of the included accelerometer can serve as
a bump detector.




                                                Camera            BNO08X
                                                                  BNO080
                                                system


                                                         Navigation
                                                         Processing

                                                  IR sensing
                                                                   Wheel
                                                   (collision/
                                                                  encoders
                                                      cliff)




                             Figure 1-4: Typical unmanned ground roving robot

1.2     BNO08X Connectivity
The BNO08X can support connections to a host microcontroller through various serial interfaces:
   • I2C interface
   • UART interface
   • SPI interface
   • UART-RVC interface – a simplified UART interface for unmanned ground roving robot (such Robot
      Vacuum Cleaners)

In addition, the BNO08X includes a bootloader to allow for firmware upgrades. The bootloader can support I 2C,
SPI or UART. Access to the bootloader is achieved by setting BOOTN to 0.
Configuration of the communication interface is achieved by setting the protocol selection (PS1/0) pins
appropriately:

                                    PS1     PS0       BNO08X           BNO08X
                                                      (BOOTN=1)        bootloader
                                                                       (BOOTN=0)
                                     0      0         I2 C             I2 C
                                     0      1         UART-RVC         Reserved
                                     1      0         UART             UART
                                     1      1         SPI              SPI
                                  Figure 1-5: Protocol selection for BNO08X
The protocol selection and BOOTN pins are sampled at reset.
PS0 is repurposed as a WAKE signal in SPI mode following reset (see 6.5.3 for timing).




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                               9 / 59
```


## PDF page 10

```text
                                                 BNO08X Datasheet                                        1000-3927 v1.17


1.2.1 Pin Descriptions
Figure 1-6 describes the function of each pin.
                     Pin
                                 BNO08X Name              Mode                  Description
                   Number
                      1       RESV_NC                 NC              Reserved. No connect.
                      2       GND                     Input           Ground
                      3       VDD                     Input           Supply voltage (sensors) (2.4V
                                                                      to 3.6V)
                      4       BOOTN                   Input           Bootloader mode select
                      5       PS1                     Input           Protocol Select pin 1
                      6       PS0/WAKE                Input           Protocol Select pin 0, also
                                                                      used to wake processor in SPI
                                                                      mode
                      7       RESV_NC                 Input           Reserved. No connect.
                      8       RESV_NC                 NC              Reserved. No connect.
                      9       CAP                                     External capacitor (100nF to
                                                                      GND)
                     10       CLKSEL0                 Input           Clock source selection. Internal
                                                                      pulldown.
                     11       NRST                    Input           Active low reset
                     12       RESV_NC                 NC              Reserved. No connect.
                     13       RESV_NC                 NC              Reserved. No connect.
                     14       H_INTN                  Output          Interrupt to host device
                     15       ENV_SCL                 Bidirectional   Environmental sensor I2C clock
                     16       ENV_SDA                 Bidirectional   Environmental sensor I2C data
                     17       SA0/H_MOSI              Input           Lower address bit of device
                                                                      address. In SPI mode, data
                                                                      input
                     18       H_CSN                   Input           SPI chip select, active low
                     19       H_SCL/SCK/RX            Bidirectional   Host Interface I2C clock, SPI
                                                                      clock or UART RX
                     20       H_SDA/H_MISO/TX         Bidirectional   Host Interface I2C data, SPI
                                                                      data out or UART TX
                     21       RESV_NC                 NC              Reserved. No connect.
                     22       RESV_NC                 NC              Reserved. No connect.
                     23       RESV_NC                 NC              Reserved. No connect.
                     24       RESV_NC                 NC              Reserved. No connect.
                     25       GND                     Input           Ground
                     26       XOUT32/CLKSEL1          Output          32K crystal output / clock
                                                                      source selection. Internal
                                                                      pulldown.
                     27       XIN32                   Input           32K crystal input. / external
                                                                      clock
                     28       VDDIO                   Input           Supply voltage (core and I/O
                                                                      domain) (1.65V to 3.6V)
                                      Figure 1-6: BNO08X pin descriptions
The BNO08X can operate from an internal oscillator, an external 32.768 kHz clock or an external 32.768 kHz
crystal. If an external clock is used it must be connected to pin 27. CEVA recommends a tolerance of 50ppm. If a
crystal is used it must be connected across pins 26 and 27. CEVA recommends using a crystal with tolerance
50ppm with 12.5pF capacitor loading.




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                     10 / 59
```


## PDF page 11

```text
                                                 BNO08X Datasheet                                 1000-3927 v1.17




                                  Figure 1-7: 32.768kHz crystal connection
Clock source selection is done during startup using the CLKSEL0 and CLKSEL1 pins. Figure 1-8 shows how to
select each source. The internal clock source must not be used with the UART-SHTP or UART-RVC interfaces.
                              Source            CLKSEL0                 CLKSEL1
                              Crystal     0 or unconnected        Connected to crystal
                              External    1                       1
                              Internal    1                       0 or unconnected
                                      Figure 1-8: Clock Source Selection
CLKSEL0 is configured with an internal pulldown at startup. After CLKSEL0 has been sampled, if it is found to be
a 1, the internal pulldown is disabled to avoid unnecessary power consumption. If either the external or internal
clock sources are selected, then CLKSEL1 is configured as an input with a pulldown. CLSKSEL1 is then sample
and the clock source determined. After CLKSEL1 has been sampled, if it is found to be a 1, the internal pulldown
is disabled to avoid unnecessary power consumption.




                                    Figure 1-9: External clock connection

www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                              11 / 59
```


## PDF page 12

```text
                                             BNO08X Datasheet                                1000-3927 v1.17




                                   Figure 1-10: Internal clock selection

1.2.2 I2C interface
The BNO08X supports a standard Fast mode I2C interface and can communicate over this interface at up to
400kb/s.




www.ceva-dsp.com                     © 2023 CEVA, Inc. All rights reserved                                12 / 59
```


## PDF page 13

```text
                                                BNO08X Datasheet                                 1000-3927 v1.17




                                Figure 1-11: BNO08X I2C connection diagram
Figure 1-11 shows how the BNO08X can be connected to an external microcontroller via the I 2C interface. The
following notes are provided as guidelines for connecting the BNO08X in a system design.
1. The H_INTN pin is the application interrupt line that indicates the BNO08X requires attention. This should be
   tied to a GPIO with wake capability. The interrupt is active low.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
   reset.
3. BOOTN is sampled at reset. If low the BNO08X will enter bootloader mode.
4. Pin 4 (BOOTN) should be pulled high through a 10K Ohms resistor. To use the device firmware update (DFU)
   capability of the BNO08X, it is recommended to connect Pin 4 to a GPIO pin on the external microcontroller.
5. Pin 5 (PS1) and Pin 6 (PS0/WAKE) are the host interface protocol selection pins. These pins should be tied to
   ground to select the I2C interface.
6. Pin 17 (SA0) can be used to select the lower bit of the 7-bit I2C slave device address.



www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                               13 / 59
```


## PDF page 14

```text
                                                  BNO08X Datasheet                                     1000-3927 v1.17


7. Pullup resistors (R1 and R2) are needed on the I2C communication lines – Pin 19 (HOST_SCL) and Pin 20
   (HOST_SDA). These values may vary depending on the board design and bus capacitance, but typical
   values are between 2KΩ and 4KΩ.
8. The BNO08X supports environmental sensors (e.g. pressure sensors, ambient light sensors) on a secondary
   I2C interface. This interface should be pulled up via resistors regardless of the presence of the external
   sensor as the SW polls for sensors at reset.

1.2.2.1     I2C Operation
The I2C specification is documented in [4]. The BNO08X provides a slave interface to the application processor
and supports 100kbps Standard mode (Sm) and 400kbps Fast mode (Fm).
I2C is a two-wire serial interface that consists of a serial clock line (SCL) and a serial data line (SDA). I 2C allows
for multi-master, multi-slave communication and uses an open-drain architecture to enable this capability. All
devices that drive the SDA line or SCL line can only drive the line low (‘0’); the bus is released and pulled high by
the pull-up resistor for ‘1’s. The master device places the slave address on the data bus and the slave device with
the corresponding address acknowledges the master.
The BNO08X I2C interface answers to a 7-bit address of either 0x4A or 0x4B:
                                 6        5       4        3        2       1        0
                                 1        0       0        1        0       1       SA0
                                         Figure 1-12: BNO08X I2C address
The lower address bit of the device address is provided by the SA0 pin. This pin is sampled at reset and should
be tied to either a logic high or logic low.

1.2.2.2     I2C Protocol
In general the data line should be stable while the clock is high, data transitions should therefore occur when the
clock is low. Communication on an I2C bus is initiated with the master device presenting a START command and
terminated with a STOP command. The bus is busy between the two commands. A START command (or
condition) is defined as a transition on the SDA signal from high to low while the SCL signal is high (Figure 1-13).
A STOP command (or condition) is defined as a low to high transition on the SDA signal while the SCL signal is
high (Figure 1-14).


                        SDA

                         SCL

                           START condition
                                         Figure 1-13: I2C START condition



                     SDA

                     SCL

                                                                                 STOP condition
                                         Figure 1-14: I2C STOP condition
I2C is a byte oriented protocol. Hence each element passed between the master and slave is 8-bits long. The bits
within the byte are transmitted most-significant bit (MSB) first. For multi-byte words the data is presented in little-


www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                    14 / 59
```


## PDF page 15

```text
                                                     BNO08X Datasheet                                   1000-3927 v1.17


endian format – least-significant byte first. The master device generates the clock. Every byte transmitted must be
acknowledged. An acknowledgement is generated by the device receiving the data and is formed by the receiver
driving the SDA line low during the ninth bit.
The master device generally drives the clock. However, if the slave device requires additional time to respond it
can force the clock low, only releasing the line when it is prepared to deliver more data. The master device MUST
support clock stretching.
The first byte presented after the start condition contains the device address and the read/write bit. The least-
significant bit (LSB) of this first byte is the read/write indication (‘0’ corresponding to write)
                                SDA                   A6-0           R/W      ACK

                                  SCL      S           1-7            8           9

                                        START        Address         R/W      ACK

                                           Figure 1-15: Device addressing
For a write cycle data is provided after the device address. The slave (BNO08X in this instance) provides an ACK
for every byte received. A typical I2C write cycle is provided below (S=start, P=stop, AD=Address).

   Master     S     AD+W            DATA             DATA            DATA                            DATA            P


   Slave                    ACK                ACK             ACK          ACK                              ACK


                                               Figure 1-16: I2C write cycle
A read cycle requires that the device must first be selected with its device address. Following the device address
the BNO08X will provide data. Every byte should be acknowledged by the master:

   Master     S     AD+R                       ACK             ACK          ACK                              NAK     P


   Slave                    ACK     DATA             DATA            DATA                            DATA


                                               Figure 1-17: I2C read cycle
The BNO08X uses CEVA’s SHTP (Sensor Hub Transport Protocol) protocol to communicate. The BNO08X
application does not support the repeated start method for typical I 2C register based interfaces. More details are
available in [2]. If the BNO085 is polled and it has no data to send it will stretch the clock until it has data to send.
If the BNO086 is polled and it has no data to send, it will send zero data packets. Host should read the length field
and ignore zero length packets.

1.2.3 UART-SHTP interface
The BNO08X provides a UART communication interface that supports CEVA’s SHTP protocol. The UART
interface operates at 3Mb/s. A typical connection is shown in Figure 1-18. When using the UART interface either
the external clock or crystal must be used. The internal clock is not accurate enough to operate the UART
interface reliably.




www.ceva-dsp.com                           © 2023 CEVA, Inc. All rights reserved                                   15 / 59
```


## PDF page 16

```text
                                                BNO08X Datasheet                                    1000-3927 v1.17




                            Figure 1-18: BNO08X UART-SHTP connection diagram
Figure 1-18 shows how the BNO08X can be connected to an external microcontroller via a UART interface. The
following notes are provided as guidelines for connecting the BNO08X in a system design.
1. The H_INTN pin is driven low prior to the initial byte of UART transmission. It will deassert and reassert
   between messages. It is used by the host to timestamp the beginning of data transmission.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
   reset.
3. BOOTN is sampled at reset. If low the BNO08X will enter bootloader mode.
4. Pin 4 (BOOTN) should be pulled high through a 10K Ohms resistor. To use the device firmware update (DFU)
   capability of the BNO08X, it is recommended to connect Pin 4 to a GPIO pin on the external microcontroller.
5. Pin 5 (PS1) and Pin 6 (PS0/WAKE) are the host interface protocol selection pins. These pins should be tied to
   VDDIO and ground respectively to select the UART-SHTP interface.




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                   16 / 59
```


## PDF page 17

```text
                                                   BNO08X Datasheet                                    1000-3927 v1.17


6. The BNO08X supports environmental sensors (e.g. pressure sensors, ambient light sensors) on a secondary
   I2C interface. This interface should be pulled up via resistors regardless of the presence of the external
   sensor as the SW polls for sensors at reset.

1.2.3.1     UART operation
The UART is configured for 3Mkb/s, 8 data bits, 1 stop bit and no parity. The UART protocol relies on an idle line
being ‘high’. A transmission is started with the assertion of a start bit (pulling the line low), followed by the data,
LSB first. After the data segment is sent (in this case 8-bits), the line is pulled high (the stop signal) for a minimum
number of bits (1 for the BNO08X) to indicate end of that segment. Bytes sent from the host to the BNO08X must
be separated by at least 100us. Bytes sent from the BNO to the host have no extra spacing.

            Start       D0         D1         D2         D3         D4            D5      D6         D7         Stop

                                           Figure 1-19: UART signaling
The BNO08X uses CEVA’s SHTP protocol to communicate. The UART protocol makes use of framing bytes at
the start and end of transmission. More details are available in [2].

1.2.4 SPI Interface
The BNO08X supports 4-wire Serial Peripheral Interface (SPI) for host communication. A typical connection
diagram is provided in Figure 1-20.




www.ceva-dsp.com                          © 2023 CEVA, Inc. All rights reserved                                   17 / 59
```


## PDF page 18

```text
                                                BNO08X Datasheet                                  1000-3927 v1.17




                                Figure 1-20: BNO08X SPI connection diagram
Figure 1-20 shows how the BNO08X can be connected to an external microcontroller via a SPI interface. The
following notes are provided as guidelines for connecting the BNO08X in a system design.
1. The H_INTN pin is the application interrupt line that indicates the BNO08X requires attention. This should be
   tied to a GPIO with wake capability. The interrupt is active low.
2. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
   reset.
3. BOOTN is sampled at reset. If low the BNO08X will enter bootloader mode.
4. Pin 4 (BOOTN) should be pulled high through a 10K Ohms resistor. To use the device firmware update (DFU)
   capability of the BNO08X, it is recommended to connect Pin 4 to a GPIO pin on the external microcontroller.
5. Pin 5 (PS1) and Pin 6 (PS0/WAKE) are the host interface protocol selection pins. Both pins must be high from
   before reset until after the first assertion of H_INTN to select the SPI interface. Pin 5 may be tied to VDDIO.
   Pin 6 must be connected to a GPIO so that the WAKE functionality can be performed.




www.ceva-dsp.com                       © 2023 CEVA, Inc. All rights reserved                                18 / 59
```


## PDF page 19

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


6. After reset the PS0/WAKE signal is used as a ‘wake’ signal taking the BNO08X out of sleep if the host wants
   to initiate communication with the BNO08X.
7. The BNO08X supports environmental sensors (e.g. pressure sensors, ambient light sensors) on a secondary
   I2C interface. This interface should be pulled up via resistors regardless of the presence of the external
   sensor as the SW polls for sensors at reset.

1.2.4.1     Responding to H_INTN
On the BNO085/BNO086, if the host fails to respond to the assertion of H_INTN within approximately 10 ms, the
BNO085/BNO086 will timeout, deassert H_INTN and retry the operation. Delays in responding to H_INTN cause
lost processing time on the BNO085/BNO086. Frequent delays will cause process starvation and some
calculations will not be completed. The result is that some outputs will have errors. To avoid this problem H_INTN
should be typically handled within 1/10 of the fastest sensor period.

1.2.4.2     SPI Operation
SPI is a 4-wire synchronous serial protocol. SPI provides a full duplex communication path and has a
master/slave relationship. The master provides the clock for all transactions. Multiple slave devices can exist on a
SPI interface by the use of a chip select signal. The BNO08X is the slave in all communications.
SPI allows for two clock polarities and clock edge sampling. These options are typically called CPOL and CPHA.
The BNO08X uses CPOL = 1 and CPHA = 1. In this configuration the clock idles high and data is captured on the
rising edge of the clock:

              CS


             CLK


             MISO               D7      D6      D5       D4      D3       D2     D1       D0


             MOSI               D7      D6      D5       D4      D3       D2     D1       D0


                                      Figure 1-21: BNO08X SPI signaling
SPI transmits data MSB first and is byte oriented (i.e. all data is passed in 8-bit segments). Any number of bytes
can be transferred in a single transaction (chip select assertion).
MISO is the data transferred from the slave to the master and MOSI from master to slave.
The BNO08X uses CEVA’s SHTP protocol to communicate. More details are available at [2].

1.2.4.3     Wake operation
When the host want to initiate contact with the BNO08X it may be necessary to wake the processor from a sleep
mode. To enable this function the BNO08X uses the PS0/wake pin. The pin is active low and should be driven low
to initiate the wake procedure. The BNO08X will respond by asserting the interrupt line at which point the host can
initiate SPI accesses. The BNO08X will de-assert the interrupt line as soon as the chip select is detected.

            PS0/wake


            H_INTN


             H_CSN


                                        Figure 1-22: SPI Wake operation




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                  19 / 59
```


## PDF page 20

```text
                                                BNO08X Datasheet                                  1000-3927 v1.17


1.2.5 UART-RVC interface
The BNO08X provides a simplified UART interface for use on unmanned ground roving robot such as robot
vacuum cleaners (RVC). When configured in this mode the BNO08X simply transmits heading and sensor
information at 100Hz over the UART TX pin. A typical connection is shown in Figure 1-23. When using the UART
interface either the external clock or crystal must be used. The internal clock is not accurate enough to operate
the UART interface reliably.




                            Figure 1-23: BNO08X UART-RVC connection diagram
Figure 1-23 shows how the BNO08X can be connected to an external microcontroller via a UART interface. The
following notes are provided as guidelines for connecting the BNO08X in a system design.
1. NRST is the reset line for the BNO08X and can be either driven by the application processor or the board
   reset.
2. BOOTN is sampled at reset. If low the BNO08X will enter bootloader mode.
3. Pin 4 (BOOTN) should be pulled high through a 10K Ohms resistor. To use the device firmware update (DFU)
   capability of the BNO08X, it is recommended to connect Pin 4 to a GPIO pin on the external microcontroller.
4. Pin 5 (PS1) and Pin 6 (PS0) are the host interface protocol selection pins. These pins should be tied to
   ground and VDDIO respectively to select the UART-RVC interface.
5. The BNO08X supports environmental sensors (e.g. pressure sensors, ambient light sensors) on a secondary
   I2C interface. This interface should be pulled up via resistors regardless of the presence of the external
   sensor as the SW polls for sensors at reset.



www.ceva-dsp.com                       © 2023 CEVA, Inc. All rights reserved                                  20 / 59
```


## PDF page 21

```text
                                                               BNO08X Datasheet                                                  1000-3927 v1.17


1.2.5.1     UART-RVC operation
The UART operates at 115200 b/s, 8 data bits, 1 stop bit and no parity. The UART protocol relies on an idle line
being ‘high’. A transmission is started with the assertion of a start bit (pulling the line low), followed by the data,
LSB first. After the data segment is sent (in this case 8-bits), the line is pulled high (the stop signal) for a minimum
number of bits (1 for the BNO08X) to indicate end of that segment.



            Start          D0             D1              D2               D3             D4           D5     D6             D7         Stop

                                                      Figure 1-24: UART signaling

1.2.5.2     UART-RVC protocol
The BNO08X transmits the following data at a rate of 100Hz.




                                                                                                                          Csum
             Index




                                                                                                                   Rsvd
                                                               X-axis           Y-axis     Z-axis
  Header              Yaw         Pitch          Roll          accel            accel      accel        MI   MR
 0xAAAA
                           MSB



                                        MSB



                                                        MSB



                                                                     MSB



                                                                                    MSB



                                                                                                 MSB
                     LSB



                                 LSB



                                               LSB



                                                               LSB



                                                                            LSB



                                                                                           LSB
                                       Figure 1-25: BNO08X UART-RVC packet format
The 19-byte message has the following fields:
Header: Each report is prefixed with a 0xAAAA header
Index: A monotonically increasing 8-bit count is provided (0-255) per report
Yaw: The yaw is a measure of the rotation around the Z-axis since reset. The yaw has a range of +/- 180˚ and is
provided in 0.01˚ increments, i.e. a report of 8734 is equivalent to 87.34˚.
Pitch: The pitch is a measure of the rotation around the Y-axis. The pitch has a range of +/- 90˚ and is provided in
0.01˚ increments, i.e. a report of 1072 is equivalent to 10.72˚.
Roll: The roll is a measure of the rotation around the X-axis. The roll has a range of +/- 180˚ and is provided in
0.01˚ increments, i.e. a report of 1072 is equivalent to 10.72˚.
X-axis acceleration: The acceleration along the X-axis, presented in mg
Y-axis acceleration: The acceleration along the Y-axis, presented in mg
Z-axis acceleration: The acceleration along the Z-axis, presented in mg
MI – Motion Intent – BNO086 only. Otherwise, reserved
MR – Motion Request – BNO086 only. Otherwise, reserved.
Reserved: The message is terminated with one (BNO086) or three (otherwise) reserved bytes, currently set to
zero
Checksum (Csum): The Index, yaw, pitch, roll, acceleration and reserved data bytes are added to produce the
checksum.
To determine the actual orientation of the module, the rotations should be applied in the order yaw, pitch then roll.
An example complete message and checksum calculation is as follows:

Message: 0xAA AA DE 01 00 92 FF 25 08 8D FE EC FF D1 03 00 00 00 E7

Where:
Index = 0xDE = 222
Yaw = 00.01˚ (1 = 0x0001)
Pitch = -1.10˚ (-110 = 0xFF92)
Roll = 20.85˚ (2085 = 0x0825)
X-acceleration = -371 mg = -3.638 m/s2 (-371 = 0xFE8D)
Y-acceleration = -20 mg = -0.196 m/s2 (-20 = 0xFFEC)


www.ceva-dsp.com                                     © 2023 CEVA, Inc. All rights reserved                                                21 / 59
```


## PDF page 22

```text
                                                   BNO08X Datasheet                                      1000-3927 v1.17


Z-acceleration = 977 mg = 9.581 m/s2 (977 = 0x03D1)
Checksum = 0xE7

1.3     Host Communication
1.3.1 SHTP
The BNO08X uses CEVA’s SHTP (Sensor Hub Transport Protocol) to communicate for all interface styles except
UART-RVC. SHTP provides a means of passing data between the BNO08X and a host with support for multiple
channels. The BNO08X does not currently support the inclusion of 3rd party applications, but the SHTP protocol
allows for separation of traffic via these channels such that applications on a host can communicate over this
channel.
All data is prefixed with a 4-byte header:
                                                Byte          Field
                                                 0       Length LSB
                                                 1       Length MSB
                                                 2       Channel
                                                 3       SeqNum
                                             Figure 1-26: SHTP Header

  Length                    Bit 15 of the length field is used to indicate if a transfer is a continuation
                            of a previous transfer. Bits 14:0 are used to indicate the total number of
                            bytes in the cargo plus header, which may be spread over multiple
                            messages. The bytes in the header field are counted as part of the
                            length. A length of 65535 (0xFFFF) is reserved because a failed
                            peripheral can too easily produce 0xFFFF. Therefore, the largest cargo
                            that can be transported is 32766 minus the header bytes. The BNO08X
                            does not support receiving fragmented messages but it does support
                            sending them.

  Channel                   The channel number of the cargo. Channel 0 is the command channel
                            and is used by the SHTP.

  SeqNum                    The sequence number of the cargo. The sequence number is a
                            monotonically incrementing number that increments once for each
                            cargo sent or cargo continuation sent. Each channel and each direction
                            has its own sequence number. The sequence number is used to detect
                            duplicate or missing cargoes and to associate segmented cargoes with
                            each other.
The length field allows a host to schedule the correct number of clocks to generate for reads over I 2C and SPI.
The BNO08X supports partial reads if the host cannot accept all the bytes in one read, the length will be updated
on a subsequent read. So, for instance a host could read the first 4 bytes to determine the number of clocks to
generate and then repeat the read with the required number of clocks. The protocol is fully described in [2].

The BNO08X supports 6 channels:
   • Channel 0: the SHTP command channel
   • Channel 1: executable
   • Channel 2: sensor hub control channel
   • Channel 3: input sensor reports (non-wake, not gyroRV)
   • Channel 4: wake input sensor reports (for sensors configured as wake up sensors)
   • Channel 5: gyro rotation vector

The SHTP control channel provides information about the applications built into the BNO08X firmware image. The
BNO08X uses advertisements to publish the channel maps and the names of the built-in applications.


www.ceva-dsp.com                          © 2023 CEVA, Inc. All rights reserved                                   22 / 59
```


## PDF page 23

```text
                                                  BNO08X Datasheet                                  1000-3927 v1.17



The executable channel allows the host to reset the BNO08X and provide details of its operating mode. Figure
1-27 provides details.
:
           SHTP Channel                                   Use                                  Direction
                             0 – reserved
                             1 – reset
                             2 – on                                                              Write
                             3 – sleep
           1 (executable)
                             4-255 – reserved
                             0 – reserved
                             1 – reset complete                                                 Read
                             2-255 – reserved
                            Figure 1-27: SHTP executable commands and response
The use of the ‘on’ and ‘sleep’ commands provides support when the host is entering and exiting sleep. When the
‘sleep’ command is issued all sensors that are configured as always on or wake (see 1.3.5.1) will continue to
operate all, other sensors will be disabled. All configured sensors will be enabled when the host issues the ’on’
command.

The sensor hub control channel is used to configure the BNO08X. Responses from the BNO08X in reaction to
configuration requests are also sent over this channel.

The input sensor report channel is unidirectional, passing data from the BNO08X to the host. All input reports
pass over this channel except for sensors that are configured as wake sensors and the gyro rotation vector. A
wake sensor is defined in Android as a sensor that will remain awake during periods when a mobile device is
asleep and can wake the processor when a particular event occurs (the event being defined by the sensor).

The wake input sensor report channel is used for sensors that are configured as wake sensors.

The gyro rotation vector is a channel used for the gyro rotation vector. A separate channel is provided to allow
applications on a host processor to prioritize this data over others to ensure low latency.

1.3.2 Report Structure
The BNO08X runs CEVA’s SH2 protocol. Full details are available in [1].
All commands, responses, sensor input reports are prefixed with a report number. The report number follows the
SHTP header.
For example, the command to read the Product ID is:

                              Byte                        Description
                               0       Report ID = 0xF9
                               1       Reserved
                                        Figure 1-28: Product ID request
The BNO08X will respond to the Product ID request with the following response:
                              Byte                      Description
                               0       Report ID = 0xF8
                               1       Reset Cause
                               2       SW Version Major
                               3       SW Version Minor
                               4       SW Part Number LSB
                               5       SW Part Number …
                               6       SW Part Number …


www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                 23 / 59
```


## PDF page 24

```text
                                                      BNO08X Datasheet                               1000-3927 v1.17


                                 Byte                         Description
                                   7        SW Part Number MSB
                                   8        SW Build Number LSB
                                   9        SW Build Number …
                                  10        SW Build Number …
                                  11        SW Build Number MSB
                                  12        SW Version Patch LSB
                                  13        SW Version Patch MSB
                                  14        Reserved
                                  15        Reserved
                                            Figure 1-29: Product ID Response
The list of currently supported commands/configurations is:
             SHTP Channel          Direction         Report ID                     Description
             2 (SH-2 control)     Host to BNO          0xFE         Get Feature Request
             2 (SH-2 control)     Host to BNO          0xFD         Set Feature Command
             2 (SH-2 control)     BNO to host          0xFC         Get Feature Response
             2 (SH-2 control)     Host to BNO          0xF9         Product ID Request
             2 (SH-2 control)     BNO to host          0xF8         Product ID Response
             2 (SH-2 control)     Host to BNO          0xF7         FRS Write Request
             2 (SH-2 control)     Host to BNO          0xF6         FRS Write Data
             2 (SH-2 control)     BNO to Host          0xF5         FRS Write Response
             2 (SH-2 control)     Host to BNO          0xF4         FRS Read Request
             2 (SH-2 control)     BNO to host          0xF3         FRS Read Response
             2 (SH-2 control)     Host to BNO          0xF2         Command Request
             2 (SH-2 control)     BNO to host          0xF1         Command Response
                                            Figure 1-30: BNO08X Commands

1.3.3 BNO08X Configuration
Motion analysis systems must process data from sensors that can be mounted in an arbitrary manner and in
systems with characteristics that affect the way the data is delivered. For example a tap detector could behave
differently in a small form factor device made of metal than a tap detector in a larger form factor device made of
plastic. Another example is the static calibration record. This record provides a description of the sensor and its
orientation; necessary details to provide calibration of the sensor data.
The BNO08X contains a Flash Record System (FRS) to store these configurations.
Complete documentation of these records is provided in [1]. Supported FRS records in the BNO08X are:
                                Record ID                            Description
                                 0x7979         Static calibration – AGM
                                 0x4D4D         Nominal calibration – AGM
                                 0x8A8A         Static calibration – SRA
                                 0x4E4E         Nominal calibration - SRA
                                 0x1F1F         Dynamic calibration
                                 0xD3E2         MotionEngine power management
                                 0x2D3E         System orientation
                                 0x2D41         Primary accelerometer orientation
                                 0x2D46         Gyroscope orientation
                                 0x2D4C         Magnetometer orientation
                                 0x3E2D         AR/VR stabilization – rotation vector



www.ceva-dsp.com                             © 2023 CEVA, Inc. All rights reserved                              24 / 59
```


## PDF page 25

```text
                                                 BNO08X Datasheet                                1000-3927 v1.17


                            Record ID                          Description
                             0x3E2E        AR/VR stabilization – game rotation vector
                             0xC274        Significant Motion detector configuration
                             0x7D7D        Shake detector configuration
                             0xD7D7        Maximum fusion period
                             0x4B4B        Serial number
                             0x39AF        Environmental sensor - Pressure calibration
                             0x4D20        Environmental sensor - Temperature calibration
                             0x1AC9        Environmental sensor - Humidity calibration
                             0x39B1        Environmental sensor - Ambient light calibration
                             0x4DA2        Environmental sensor - Proximity calibration
                             0xD401        ALS Calibration
                             0xD402        Proximity Sensor Calibration
                             0xED85        Stability detector configuration
                             0x74B4        User record
                             0xD403        MotionEngine Time Source Selection
                             0xA1A2        Gyro-Integrated Rotation Vector configuration
                                           Figure 1-31: FRS records
FRS records are written by use of FRS read and write requests. Writing and reading FRS records follows a multi-
step protocol.
For writes an FRS Write Request is issued indicating which FRS record to write and the amount of data to be
written. The BNO08X will respond with a write response acknowledging the request. The host will then issue a
number of FRS Write Data packets to which the BNO08X will respond. The final response from the BNO08X will
indicate a successful conclusion to the flash writes or if the process failed a failure indication.
An FRS read follows a similar pattern. An FRS Read request is issued and FRS Read responses provide the data
with the final response indicating completion of the record retrieval.

1.3.4 Sensor Metadata
The BNO08X operates as a sensor hub and as such provides processing of sensor data. In order for a user to
understand the capabilities of the sensors, metadata is provided that describes those capabilities. The SH-2
reference manual provides full details of the metadata record but as an example Figure 1-32 provides the
metadata for the rotation vector. Metadata is stored as an FRS record and is therefore retrieved as an FRS read.
                                                           Description
                              Word        MSB                                     LSB
                               0                     Version = 0x00010001
                               1                          Range = 1.0
                               2                 Resolution = 6.103515625e-05
                               3            Revision = 4         Power = 5.3017578125
                               4                     Minimum period = 1000
                               5        FIFO reserved count =      FIFO max count = 0
                                                  1
                                6        Vendor ID length = 0    Batch buffer bytes = 14
                                7          Q point 2 = 12             Q point 1 = 14
                                8          Q point 3 = 13            Sensor-Specific
                                                                  Metadata Length = 0
                                9                   Maximum period = 10000
                               10                Sensor-Specific metadata = 0
                               N                           Vendor ID
                               Figure 1-32: BNO08X rotation vector metadata

www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                               25 / 59
```


## PDF page 26

```text
                                                 BNO08X Datasheet                                      1000-3927 v1.17


  Version                  Identifies the physical sensor/driver/fusion versions for a given sensor.
                           The elements within this field are updated when a component changes
                           in a manner that affects the output of the sensor.
                           LSB      – ME version
                           Byte 1 – MH version
                           Byte 2 – SH version
                           MSB – 0x00

  Range                    The range of the sensor. The format is unsigned fixed point. The units
                           and Q point are the same as those used in the sensor’s input report.

  Resolution               The resolution of the sensor. The format is unsigned fixed point. The
                           units and Q point are the same as those used in the sensor’s input
                           report.

  Power                    The power used by the sensor in mA when operating. The format is
                           unsigned fixed point. The Q point is 10.

  Revision                 Indicates the revision of the metadata record.

  Minimum period           An unsigned integer indicating the minimum operating period in
                           microseconds of the sensor.

  FIFO max count           The maximum number of samples that can be stored in the batch
                           buffer.

  FIFO reserved count      The number entries reserved in the batch FIFO for this sensor

  Batch buffer bytes       The number of bytes used in the batch buffer to store one entry.

  Vendor ID length         The length of the vendor ID in bytes.

  Q point 1                A signed 16-bit integer indicating the Q point of the sensor data fields.

  Q point 2                A signed 16-bit integer indicating the Q point of the sensor bias or
                           accuracy fields. This field is applicable only to sensors that have bias or
                           accuracy outputs as well as data outputs.

  Sensor-Specific          An unsigned 16-bit integer indicating how many bytes are used for
  Metadata length          sensor-specific metadata. 0 if there is no additional metadata. This
                           value must be a multiple of four.

  Q point 3                A signed 16-bit integer indicating the Q point of the sensor data change
                           sensitivity.

  Maximum period           An unsigned integer indicating the maximum operating period in
                           microseconds of the sensor.
  Sensor specific          Some sensors provide additional metadata (see [1])
  metadata
  Vendor ID                A null terminated string describing the vendor name and part number.


1.3.5 Sensor Reports
Sensors are enabled by the host issuing a Set Feature command, the feature report ID being for the sensor of
interest. The Set Feature command provides the parameters necessary for the sensor to be enabled. Typically
this is just the sampling period (report interval), but some sensors require additional configuration. A sensor can
be configured to only issue reports when a change in the sensor value occurs (change sensitivity), this can be
absolute or relative. An absolute change is a change from below to above a certain value, while relative is a

www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                   26 / 59
```


## PDF page 27

```text
                                                  BNO08X Datasheet                                    1000-3927 v1.17


change between two consecutive samples. Google introduced the concept of batching which is supported by the
BNO08X with a batch interval field.

                             Byte                           Description
                              0       Report ID = 0xFD
                              1       Feature Report ID
                              2       Feature flags
                              3       Change sensitivity [absolute | relative] LSB
                              4       Change sensitivity [absolute | relative] MSB
                              5       Report Interval LSB
                              6       Report Interval
                              7       Report Interval
                              8       Report Interval MSB
                              9       Batch Interval LSB
                              10      Batch Interval
                              11      Batch Interval
                              12      Batch Interval MSB
                              13      Sensor-specific configuration word LSB
                              14      Sensor-specific configuration word
                              15      Sensor-specific configuration word
                              16      Sensor-specific configuration word MSB
                                       Figure 1-33: Set Feature command
The host can request a feature report for a particular sensor (Get Feature command) to which the BNO08X will
respond with a Get Feature Response. This can be used to verify the configuration of a sensor. A Get Feature
Response is also issued by the BNO08X whenever the sensor’s operating mode changes which can occur due to
the host issuing a Set Feature command or because the internal rates changed due to events within the BNO08X.

1.3.5.1     Batching
Batching is a power management feature added to Android Lollipop. The concept is that the application processor
receives data in batches hence avoiding the constant processing of sensor data. This delay in processing allows
the processor to do more useful work until sufficient sensor data has accumulated for the task for which the
sensor data was requested. In addition when the application processor sleeps, sensor data can accumulate in a
batch buffer and be used to wake the processor when sufficient data has accumulated.
To facilitate the batching the BNO08X provides two batch queues. The higher priority queue is for wake sensors
and the other for all remaining enabled sensors (described as always-on in [1]). The batch interval in the set
feature command indicates how many microseconds must pass before the batch is sent to the host. If the
application processor is asleep it will be woken when the wake queue reaches the programmed interval. At that
point all data from both queues will be pushed to the host. Data in the non-wake queue will be allowed to be lost,
oldest data first in the event the application processor is sleeping. A sensor is designated as a wake sensor by a
flag in the feature flags field.
Note that the BNO08X only supports one batch queue per type of sensor. If a sensor is configured as a wake
sensor it cannot also be marked as a non-wake sensor (always-on).

1.3.5.2     Example sensor report
All input reports have a similar format ([1] provides full details). The calibrated gyroscope output is an example:
                   Byte                                     Description
                    0      Report ID = 0x02
                    1      Sequence number
                    2      Status
                    3      Delay


www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                  27 / 59
```


## PDF page 28

```text
                                                        BNO08X Datasheet                           1000-3927 v1.17


                    Byte                                    Description
                     4       Gyroscope calibrated Axis X LSB
                     5       Gyroscope calibrated Axis X MSB
                     6       Gyroscope calibrated Axis Y LSB
                     7       Gyroscope calibrated Axis Y MSB
                     8       Gyroscope calibrated Axis Z LSB
                     9       Gyroscope calibrated Axis Z MSB
                                     Figure 1-34: Calibrated gyroscope input report
The sequence number is a monotonically increasing value that is used to check for dropped samples.
The status byte is broken into two fields:
          Bits 1:0 – indicate the status of a sensor.
                   0 – Unreliable
                   1 – Accuracy low
                   2 – Accuracy medium
                   3 – Accuracy high
          Bits 7:2 - Delay upper bits: 6 most-significant bits of report delay
The delay byte is the lower 8 bits of the report delay. Delay has a resolution of 100µs.
Bytes 4-9 of the report provide the gyroscope data.

1.3.5.3       Timestamping
The sensor report delay field allows an accurate timestamp to be formed in the host application. Delay measures
the time delta from the sensor interrupt to the timebase reference. By generating a timestamp on the host
interrupt signal the host application can then determine an accurate sensor timestamp by subtracting delay.
Note that the BNO08X also provides a timebase reference report with sensor reports:
                               Byte                            Description
                                0        Report ID=0xFB
                                1        Base Delta LSB: relative to transport-defined reference
                                         point. Signed. Units are 100 microsecond ticks.
                                 2       Base Delta
                                 3       Base Delta
                                 4       Base Delta MSB
                                       Figure 1-35: Timebase Reference Report
The timebase reference functions in the same way as the delay field in the sensor report. The base delta should
be subtracted from the timestamp registered on the host for the host interrupt signal. When the timebase
reference report is provided the individual sensor report will likely have a delay of zero. However, in cases where
sensor reports are concatenated (due to delays in processing), the delay field may be populated, in which case
both that delay and the timebase reference should be taken into account when calculating the actual timestamp of
the sensor sample.
As an example, if the host receives the following report:


                                           Timebase
                                                          Sensor report   Sensor report
                                          Reference
                                                           Delay = 0       Delay = 17
                                          Delta = 120



                                          Figure 1-36: Timestamping example



www.ceva-dsp.com                            © 2023 CEVA, Inc. All rights reserved                            28 / 59
```


## PDF page 29

```text
                                                  BNO08X Datasheet                                     1000-3927 v1.17


The host will create a timestamp based on the assertion of HINT (call it T). The timebase reference provides a
baseline reference of 120 * 100µs or 12ms. All sensor reports are thus timestamped as T-12ms + their own delay.
In this example, the first sensor report should be timestamped as occurring at T-12ms and the second at T –
10.3ms (T - 12ms + 1.7ms).

1.4     Bootloader
The BNO08X provides a bootloader function to allow firmware upgrades to be applied. During reset or power-on
sequence, the bootloader first checks the status of the BOOTN pin. If the pin is pulled low during reset or power-
on, the BNO08X will enter the bootloader mode. If the BOOTN pin is pulled high, then the bootloader starts the
application.
The bootloader uses the host interface as configured by the PS0/1 pins (see Figure 1-5). If I2C is selected the
bootloader has an I2C address of either 0x28 or 0x29 (SA0 provides the lowest address bit).
After the BNO08X enters the bootloader mode, it waits for the size of the application code from the host. The
application to be written to flash is split into a number of frames. The application in flash is updated by writing
these frames to the BNO08X and reading a status back to verify the write was successful.
Reference code (on the host microcontroller) for using the bootloader to perform Device Firmware Upgrade (DFU)
is available upon request.




www.ceva-dsp.com                          © 2023 CEVA, Inc. All rights reserved                                   29 / 59
```


## PDF page 30

```text
                                                  BNO08X Datasheet                                    1000-3927 v1.17


2 Sensor Data Processing
The BNO08X analyzes the data from the 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer. It
manages the various sensors, reads data, and provides a number of outputs to the host. The BNO08X processes
motion relative to a frame of reference. It has no knowledge of the orientation of sensors within the device and
must be configured to conform to the desired frame of reference of the device. While the BNO08X is not
necessarily used within Android systems, the frame of reference defined in the Android specification is used by
the BNO08X software. In Android, the device frame of reference is represented as X axis horizontal, positive to
the right, Y axis vertical, positive along the face of the device and Z axis positive towards the outside of the front
face of the device. The frame of reference is shown in Figure 2-1.




                                     Figure 2-1: Android co-ordinate system
The BNO08X can be mounted within the device in any orientation that allows the PCB to be laid out efficiently.
The BNO08X sensor orientation is provided in Figure 4-1.
The BNO08X needs to be configured to understand what the orientation is relative to the device frame of
reference. This is achieved by modifying either the static calibration record if the device is passed through a
calibration step or by modifying the sensor orientation records. Sensor orientation is discussed further in section
4.

2.1     Motion Outputs
Motion outputs allow applications to understand the motion being applied to the device as measured by BNO08X.

2.1.1 Acceleration Outputs
The 3-axis accelerometer measures the acceleration of the device (note that the accelerometer will measure
gravity as acceleration). BNO08X calibrates the accelerometer output to improve the data reported. BNO08X will
report this acceleration aligned to the coordinate frame of the device. BNO08X can (after fusing data with the
gyroscope) separate gravity from this acceleration, thus providing the BNO08X with the following acceleration
outputs:
    • Calibrated Acceleration (m/s2). Acceleration of the device including gravity
    • Linear acceleration (m/s2). Acceleration of the device with gravity removed
    • Gravity (m/s2)
    • Raw uncalibrated accelerometer data (ADC units). Used for testing

Note that for raw data to be transmitted the accelerometer must be enabled by a separate set feature request, i.e.
the raw sensor listens to an already configured sensor. The rate specified in the raw sensor set feature request
cannot be higher than the underlying sensor configuration. The raw data will either be delivered at the same rate
as the underlying sensor or at the rate requested (within the constraints of rate decimation).




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                  30 / 59
```


## PDF page 31

```text
                                                 BNO08X Datasheet                                    1000-3927 v1.17


2.1.2 Angular Velocity Outputs
The 3-axis gyroscope measures the angular velocity of the device. The angular velocity is reported as rotations
around the X, Y, Z axes as depicted in Figure 2-1. A positive value is reported for counter-clockwise rotations.
BNO08X calibrates this data to improve the measurement and provides the following outputs:
    • Calibrated gyroscope (rad/s). The angular velocity of the device
    • Uncalibrated gyroscope (rad/s). Angular velocity without bias compensation. This output is required for
        Android 4.4 compatibility. The bias is reported as a second parameter
    • Raw uncalibrated gyroscope (ADC units). Data direct from the gyroscope, used for testing

Note that for raw data to be transmitted the gyroscope must be enabled by a separate set feature request, i.e. the
raw sensor listens to an already configured sensor. The rate specified in the raw sensor set feature request
cannot be higher than the underlying sensor configuration. The raw data will either be delivered at the same rate
as the underlying sensor or at the rate requested (within the constraints of rate decimation).

2.1.3 Magnetometer Processing
While not actually a motion sensor, the magnetometer can be used to detect motion. The magnetometer
measures the surrounding magnetic field and is used to determine absolute orientation as well as support other
fusion operations. Absolute orientation can be thought of as determining which direction is magnetic north and
evaluating the sensors orientation with respect to north.

The complication with measuring the magnetic field is that the field is distorted by the proximity of ferrous or
magnetic material. The distortion caused by these materials is often referred to as the soft-iron and hard-iron
effect. To remove this distortion the device must be moved sufficiently through all 3 axes (X, Y and Z) of 3-
dimensional space. An indication of the degree of calibration is reported in the magnetometer reports accuracy
field.
BNO08X provides the following magnetic field measurement outputs:
     • Magnetic field calibrated (in µTesla). The fully calibrated magnetic field measurement.
     • Magnetic field uncalibrated (in µTesla). The magnetic field measurement without hard-iron offset applied,
         the hard-iron estimate is provided as a separate parameter.
     • Raw magnetic field measurement (in ADC units). Direct data from the magnetometer. Used for testing.

2.2     Orientation Outputs
One of the primary outputs of BNO08X is an estimation of the device orientation. Fusing data from
accelerometers, gyroscopes and magnetometers has a rich history of usage for estimating orientation. BNO08X
provides multiple estimates that have different tradeoffs as described below (note that the term rotation vector
used below is derived from Google’s definition in Android 4.4).

2.2.1 Geomagnetic Rotation Vector
The geomagnetic rotation vector is an orientation output that is expressed as a quaternion referenced to magnetic
north and gravity. It is produced by fusing the outputs of the accelerometer and magnetometer. The gyroscope is
specifically excluded in order to produce a rotation vector output using less power than is required to produce the
rotation vector of section 2.2.4. The consequences of removing the gyroscope are:
    •   Less responsive output since the highly dynamic outputs of the gyroscope are not used
    •   More errors in the presence of varying magnetic fields

2.2.2 Game Rotation Vector
The game rotation vector is an orientation output that is expressed as a quaternion with no specific reference for
heading, while roll and pitch are referenced against gravity. It is produced by fusing the outputs of the
accelerometer and the gyroscope (i.e. no magnetometer). The game rotation vector does not use the
magnetometer to correct the gyroscopes drift in yaw. This is a deliberate omission (as specified by Google) to
allow gaming applications to use a smoother representation of the orientation without the jumps that an
instantaneous correction provided by a magnetic field update could provide. Long term the output will likely drift in


www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                  31 / 59
```


## PDF page 32

```text
                                                  BNO08X Datasheet                                    1000-3927 v1.17


yaw due to the characteristics of gyroscopes, but this is seen as preferable for this output versus a corrected
output.

2.2.3 AR/VR Stabilized Game Rotation vector
While the magnetometer is removed from the calculation of the game rotation vector, the accelerometer itself can
create a potential correction in the rotation vector produced (i.e. the estimate of gravity changes). For applications
(typically augmented or virtual reality applications) where a sudden jump can be disturbing, the output is adjusted
to prevent these jumps in a manner that takes account of the velocity of the sensor system. This process is called
AR/VR stabilization. An FRS (Flash Record System – see Figure 1-31) record is provided to allow configuration of
this feature.

2.2.4 Rotation Vector
The rotation vector provides an orientation output that is expressed as a quaternion referenced to magnetic north
and gravity. It is produced by fusing the outputs of the accelerometer, gyroscope and magnetometer. The rotation
vector is the most accurate orientation estimate available. The magnetometer provides correction in yaw to
reduce drift and the gyroscope enables the most responsive performance.

2.2.5 AR/VR Stabilized Rotation Vector
Estimates of the magnetic field and the roll/pitch of the device can create a potential correction in the rotation
vector produced. For applications (typically augmented or virtual reality applications) where a sudden jump can be
disturbing, the output is adjusted to prevent these jumps in a manner that takes account of the velocity of the
sensor system. This process is called AR/VR stabilization. An FRS (Flash Record System – see Figure 1-31)
record is provided to allow configuration of this feature.

2.2.6 Gyro rotation Vector
Head tracking systems within a virtual reality headset require low latency processing of motion. To facilitate this,
the BNO08X can provide a rotation vector at rates up to 1kHz. The gyro rotation Vector provides an alternate
orientation to the standard rotation vector. Compared to the standard rotation vector the gyro rotation vector has
an optimized processing path and correction style (correction is the adjustments made to the output based on
more accurate estimates of gravity, mag field, angular velocity) that is suitable for head tracking applications.
By default the Gyro rotation vector provides an orientation output that is expressed as a quaternion. It can be
configured via FRS record to be based on either the rotation vector (using the magnetometer) or the game
rotation vector (ignoring the magnetometer).

2.2.7 Gyro rotation Vector Prediction
In virtual reality systems reducing latency is a vital requirement to proving an immersive experience. The gyro
rotation vector provides a low latency output. However in a system there are many factors that can add to the
latency budget:
     • IMU processing
     • Data path between IMU and video rendering
     • Video rendering
     • Video buffer/display write

To aid in the reduction of the overall system latency the BNO08X can derive an expected orientation for some
time in the future. Predicting 20-30 ms into the future can significantly improve the perception of reality within the
virtual reality world. The predictor accepts the gyro rotation vector as input and produces a second rotation vector
for some time in the future. An FRS record is used to tune the predictor for the type of motion. The predictor
accepts three constants (alpha, beta, gamma) for the tuning.
The default prediction time is 28ms and is determined for motion typical of a head tracker. Some typical
parameters that can be used are captured in Figure 2-2. These values are all for motion typical of a head tracker.




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                    32 / 59
```


## PDF page 33

```text
                                                    BNO08X Datasheet                                   1000-3927 v1.17


   Prediction            Alpha                  Beta                 Gamma            Max    rms error       Noise
    Time (s)                                                                         error     (deg)        StdDev
                                                                                     (deg)                   (deg)
        0.028      0.3030725439091       0.1132958963849       0.0027762197131       1.96°     0.19°        0.019°
        0.005      0.3643672983581       0.0931655637068        0.0025557744608      0.25°     0.01°        0.003°
         0.01      0.3222569517214       0.0945467654507        0.0022830845312      0.54°     0.03°        0.005°
         0.02       0.2905555274225      0.1018361851286        0.0023953721024      1.26°      0.1°        0.011°
         0.03      0.3084584610521       0.1160861868509        0.0028497585549      2.14°     0.22°        0.021°
         0.04      0.3396316005051       0.1237361273333        0.0030844488660      3.23°      0.4°        0.034°
         0.05      0.3548447957314       0.1224432975403        0.0032236047505       4.8°     0.64°        0.051°
         0.06      0.4091677424529       0.1250943899234        0.0035648974825      7.03°     0.93°        0.072°
                                 Figure 2-2: HMD mounted head motion prediction
For other applications contact CEVA.

2.3       Environmental Sensors
The BNO08X provides support for environmental sensors connected over an I2C interface (separate to the host
interface). Currently the BNO08X supports:

    •     Bosch Sensortec BME280 pressure/humidity/temperature sensor
    •     Bosch Sensortec BMP280 pressure/temperature sensor (only BMP280 or BME280 should be populated)
    •     Capella Microsystems CM36686 proximity and ambient light sensor

If the sensors are not required for a particular application the I2C bus should be correctly terminated with pullup
resistors as the BNO08X attempts to discover the sensors at reset. Proper termination will ensure correct
behavior of the BNO08X.
The ALS and proximity sensors typically require calibration as these sensors are typically placed behind glass or
plastic. Contact CEVA for details.

2.4       Classification System
With the proliferation of sensors available within mobile devices, there is an increasing interest in providing value
by understanding the context of the device or device holder. Classifying the context based on the sensors
available is an active area of research with an emphasis of the classifiers being ‘always on’, so low power is an
inherent requirement. BNO08X supports a number of classifiers that generate events upon detection of a
particular context or motion.

2.4.1 Stability Detection and Classification
Analysis of the motion sensors allows BNO08X to classify stability. BNO08X provides two virtual sensors to
quantify stability: a stability detector and a stability classifier.
The stability classifier uses both the accelerometer and gyroscope to distinguish between three levels of stability:
    •     On table: the device is likely on a table or other stable surface
    •     Stable: the device is likely being held, but held in a stationary manner
    •     Motion: the device or device holder is in motion
The stability detector uses the accelerometer to distinguish between stability and motion.
Both the classifier and detector have configurable thresholds for the levels of stability. These thresholds are
stored as FRS records as described in the SH-2 Reference Manual [1].
The stability detector has an acceleration threshold which is the aggregate acceleration of the device. The default
value is 0.784m/s2. The stability detector also includes a time threshold. The acceleration of the device must stay
below the threshold set for the duration of the time record to register as stable. The default value is 500ms.


www.ceva-dsp.com                           © 2023 CEVA, Inc. All rights reserved                                  33 / 59
```


## PDF page 34

```text
                                                 BNO08X Datasheet                                     1000-3927 v1.17


The FRS record that configures the stability classifier is encoded in the MotionEngine power management and
stability classifier FRS record. The FRS record provides a stable threshold and a duration threshold. The data
from the gyroscope must be below the stable threshold for the duration threshold period for Stable to be declared.
The default values are 1 rad/s and 3s respectively. The static calibration record for the device contains
parameters that describe the noise floor of the gyroscope. Comparison of the gyroscope’s output to the expected
noise of the system allows for a very reliable measure of high stability, such as one might see when the device is
on a table.
Note that the stability detector is lower power than the stability classifier due to the sensors used (accelerometers
currently consume less power than gyroscopes).

2.4.2 Tap Detector
The tap detector evaluates data from the accelerometer and generates an event when either a single or double
tap is detected. The output of the tap detector indicates the axis along which the tap was detected and whether
the tap was a single or a double tap. The axes are aligned to the Android frame of reference (Figure 2-1).




                                             Figure 2-3: Tap detector

2.4.3 Step Detector
The step detector uses data from the accelerometer to detect steps. It will output a value of 1.0 whenever a step
is detected. The step detector has two optimizations, one for wrist worn devices and the second for all other use
cases.
The step detector has built in defaults (listed below). Modification of these values, while not recommended, is
possible. Consultation with CEVA is advised.
Configurable options:
    •   watchSelector: 1 for wrist worn devices, 0 otherwise
    •   allowTime: max variation of step period. Default value 250ms
    •   stepMinTime: minimum step period. Default value 300ms
    •   groupDelay: group delay of the filter. Filtering of accelerometer data is required to ensure accurate
        detection of steps, particularly if the accelerometer is rotating with respect to gravity. The group delay of
        the filter is a compromise between responsiveness and error accumulation. A value of 90ms is used and
        the user is discouraged from modifying this value without consultation with CEVA.
    •   threshold: vertical motion threshold. The threshold in meters below which a step would not be recognized.
        Default is 3.1mm
    •   thresholdWatch: A second threshold used for wrist worn devices. The default value is 0.01
    •   thresholdNonWatch: A second threshold for devices other than wrist worn. The default value is 1.2x10 -4




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                   34 / 59
```


## PDF page 35

```text
                                                  BNO08X Datasheet                                    1000-3927 v1.17


2.4.4 Step Counter
The step counter uses the step detector to detect and count steps. It provides a more accurate indication of steps
taken than the step detector. It provides increased accuracy by evaluating the data around each step event,
possibly reclassifying previous samples as either steps or non-steps, depending upon the patterns perceived. It
outputs a 16-bit step count. The application processor maintains a 64-bit count of the total number of steps taken
and must therefore manage any roll-overs in the step counter output. The step counter will assert the host
interrupt while the host processor is asleep if the 16-bit step counter is close to overflow to ensure steps are not
lost.
The step counter has two optimizations, one for wrist worn devices and the second for all other use cases.

2.4.5 Activity Classification
The activity classifier uses the accelerometer to determine if the user is walking, running or climbing stairs. The
climbing stairs feature requires the use of an external pressure sensor. The sensor is configurable to allow
customization for a particular device and/or a particular individual. The configurable options relate to the step rate
for running versus walking and to the variation of the signal strength of the accelerometer.

The configurable options are:
Thresholds related to Step Rates
    • runLowerThldStep: The lowest step rate for running. The default is 1.25 steps/s
    • walkUpperThldStep: The highest step rate for walking. The default is 2.25 steps/s
    • walkLowerThldStep: The lowest step rate for walking. The default is 0.75 step/s

Thresholds related to Step Signal Variation Strength:
    • runUpperThldStd: The highest signal variation strength for running. The default is 23.50m/s2
    • runLowerThldStd: The lowest signal variation strength for running. The default is 3.20m2/s2
    • walkUpperThldStd: The highest signal variation strength for walking. The default is 4.50m2/s2
    • walkLowerThldStd: The lowest signal variation strength for running. The default is 0.75m2/s2

The classification considers steps within a 4s window and the variation in signal strength is determined by taking
the standard deviation of the accelerometer normal over the 4s window.

With the default settings a classification matrix as depicted in Figure 2-4 can be created.




                                    Figure 2-4: Activity classification matrix


www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                   35 / 59
```


## PDF page 36

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


2.4.6 Significant Motion Detector
The significant motion detector was introduced by Google in Android 4.3 (API level 18). The definition is that an
event should be raised when a user has created motion that implies they have changed location. The typical use-
case is that the sensor be low power (as it can run while the phone is asleep) and that it alerts the main
processing element of the device when a change in location is likely. BNO08X uses the step detector and
processing based on the accelerometer to determine if the user has created significant motion. The detector
outputs a value of 1.0 when an event occurs. Per the Android specification this detector will disable itself
immediately after the event is output.
Configuration of the significant motion detector requires configuration of the following two parameters:
    •   Significant motion step threshold. The number of steps required to trigger the event. The default value is
        5 steps.
    •   Acceleration threshold. A threshold loosely based on the device’s acceleration to trigger significant motion
        processing. Default value is 10m/s2

2.4.7 Shake Detector
The shake detector determines if the device has been shaken. It can be used as a wake-up sensor or for UI
interaction. It has a number of configurable options:
     • Magnitude of acceleration that must be exceeded. Default 8m/s2
     • Number of direction changes to qualify as a shake and the axis of interest. Default of 3
     • Minimum and maximum time between direction changes. Default: 50ms < time < 400ms
     • Which device axis to detect shake along (X, Y or Z). Default: all axes.

The configuration parameters are stored in the shake detector configuration FRS record.




                                           Figure 2-5: Shake gesture




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                 36 / 59
```


## PDF page 37

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


3 Calibration and Interpretation
The BNO08X interprets the data from its sensors to model the device’s motion and ultimately determine the
device’s orientation and classify the motion of the device. The accuracy of this model is dependent on the quality
of the data provided by the sensors.
All sensors exhibit slight imperfections. These imperfections generally appear as errors in offset and scale.
Sensor systems that contain three axes have an additional variant of offset that includes skew and rotation (skew
is the non-orthogonality of the three axes and rotation is the angular difference of the coordinate frame of the
sensor to the coordinate frame of the device). Calibration of the sensor data falls into two categories:
    •   Static calibration
    •   Dynamic calibration
Static calibration is the correction of non-varying parameters to the data returned from the sensors. These static
adjustments to the data typically compensate for offsets and scaling errors that do not vary over time or with
temperature. Examples (this list is not exhaustive) of non-varying errors (or errors that are not compensated for
dynamically) are:
    •   Accelerometer (or gyroscope or magnetometer) skew (also called cross-axis sensitivity)
    •   Accelerometer (or gyroscope or magnetometer) gain or sensitivity
    •   Sensor orientation with respect to device frame of reference
A static calibration data record (SCD) is essentially a description of the sensor system used on the device and
from that record the BNO08X calibrates and interprets the data from the sensors producing data in the
appropriate SI units for use by applications. The SCD record can be generic and essentially describe the sensors
as per the datasheet, or the record can be specific to the actual sensors in the package.
Dynamic calibration is the correction to the data returned from the sensors that varies with either time or
temperature. The BNO08X calculates what the adjustment should be during use of the sensor and applies
correction factors as necessary in real time. Examples of dynamic calibration parameters are:
    •   Gyroscope zero-rate offset
    •   Accelerometer zero-g offset
In addition, as the BNO08X has applications in devices such as Robot Vacuum Cleaners it is necessary to
calibrate the accelerometer in a planar fashion (i.e. around the axis that the vacuum cleaner revolves). Selection
of 3D or planar accelerometer calibration is via command.
Magnetic field calibration is an additional capability of the BNO08X. The primary goal of using the magnetometer
is to measure the Earth’s magnetic field. The magnetic field measured by a magnetometer is distorted by the
presence of ferrous and magnetic material in the near vicinity. These distortions are referred to as soft and hard
iron effects respectively. The offset and distortion to the measured magnetic field is estimated and compensated
for continuously while the device is in motion. Removal of the distortions allows for an improved estimate of the
Earth’s magnetic field.

3.1     Calibration Effects
This section provides a brief overview of the benefits of dynamic calibration and the recommended settings for
various applications.

3.1.1 Calibration Command
The SH-2 firmware in the BNO08X allows a host microcontroller to enable or disable the dynamic calibration of
the accelerometer, gyroscope and magnetometer. Refer to the SH-2 Reference Manual [1] for the command used
to control calibration of each sensor. The command allows the host to control when calibration is performed. Note
that the calibration settings do not persist across resets of the BNO08X.




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                 37 / 59
```


## PDF page 38

```text
                                                  BNO08X Datasheet                                   1000-3927 v1.17


3.1.2 Accelerometer
Dynamic calibration for an accelerometer is the removal of zero-g offset (ZGO). An accelerometer at rest should
only report gravity, any deviation is the zero-g offset. This is most easily seen when the accelerometer is placed
with one axis perpendicular to the Earth, the other two axes should report zero. ZGO errors can manifest as tilts
or tilt corrections (e.g. the screen on a head mounted display may be tilted with respect to the expected horizon)
and this usually indicates that the accelerometer needs to be calibrated. The BNO08X provides two methods of
calibrating the accelerometer ZGO; a full 3-dimensional approach for devices that can move in freely in space and
a planar calibration for devices that are constrained to move in a plane (such as a robot vacuum cleaner). For
more information on calibrating the accelerometer, see section 3.2.

3.1.3 Gyroscope
Dynamic calibration for a gyroscope is the removal of zero rate offset (ZRO). A gyroscope at rest should report
zero rad/s on all axes. Any deviation from zero is the zero rate offset (ZRO). ZRO errors can manifest as drifts
(e.g. the screen on a head mounted display can continue moving even when the device is stationary). Note that
though the BNO08X can continuously attempt to calibrate the gyroscope for ZRO, it is possible to fool the
calibration algorithms through slow horizontal rotations (around the gravity vector). Placing the device on a stable
surface will force the ZRO calibration to converge rapidly. The gyroscope calibration algorithm attempts to remove
ZRO while the device is not on a very stable surface. For this to be successful the device to which the BNO08X is
mounted must have sufficient tremor such as the human hand. If there is insufficient tremor it is advisable to
disable the gyroscope calibration to prevent drift (i.e. by mis-calibrating ZRO). ZRO will always be corrected when
the device becomes very stable (such as when laid on a table).

3.1.4 Magnetometer
Magnetometers measure the magnetic field around them and the typical use case is to determine the position of
the Earth’s magnetic North pole. The magnetic field can be distorted by the presence of magnetic fields caused
by speakers, magnets etc. (hard-iron effects) or other ferrous materials (soft-iron effects). BNO08X can
dynamically calibrate the readings from the magnetometer to compensate for these distortions. Without calibration
the heading reported by BNO08X will be highly suspect so it is highly recommended to calibrate the
magnetometer. For more information on calibrating the magnetometer, see section 3.2.

3.1.5 Calibration Accuracy
The BNO08X provides input reports (see section 1.3.5) to send sensor data to the host microcontroller. The input
reports include a “Status” field that indicates the accuracy status of the sensor. This is especially useful when the
application needs to know the calibration status of the BNO08X outputs. Bits 1:0 of the Status field indicate the
accuracy as shown in Figure 3-1. Refer to the SH-2 Reference Manual [1] for more information.


                                   Status Bits (1:0)             Description
                                          0             Unreliable
                                          1             Accuracy Low
                                          2             Accuracy Medium
                                          3             Accuracy High
                                     Figure 3-1: Accuracy status of sensors

3.1.6 Recommended Settings
Note that by default the accelerometer and magnetometer calibration are enabled for all interface modes except
UART-RVC. In UART-RVC mode planar-ZGO calibration is enabled.
For general applications that require motion tracking in a relatively stable magnetic field it is recommended to use
9-axis sensor fusion outputs (Rotation Vector – see section 2.2.4) from the BNO08X. If the device to which the
BNO08X is attached provides sufficient tremor then the gyroscope calibration can be enabled. However the user
should verify performance with their specific motion profile.


www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                  38 / 59
```


## PDF page 39

```text
                                                   BNO08X Datasheet                                        1000-3927 v1.17


If the application requires an orientation estimate in an unstable magnetic field, the Game Rotation Vector may be
more useful.
For virtual reality applications, stability and smoothness of head tracking is important. During application use
dynamic calibration of the magnetometer can result in undesirable motion artifacts such as jumps as the BNO08X
attempts to compensate for magnetic field distortions. It is therefore recommended to use the Game Rotation
Vector (see section 2.2.2) or gyro rotation Vector (see section 2.2.6 and 2.2.7 ) and only enable the calibration of
the accelerometer.
For RVC applications it is generally expected that the UART-RVC mode be used and in this case planar ZGO will
be enabled by default. If the user has a similar application but requires higher rate outputs then alternate
interfaces can be used. The planar accelerometer calibration can be enabled by the user.

3.2     Calibration Steps
For best motion tracking performance, it is recommended to calibrate the BNO08X. Since each MEMS sensor
part has different individual characteristics, each device using the BNO08X must be calibrated individually.
Figure 3-2 summarizes the steps required to calibrate the accelerometer, gyroscope and magnetometer. For
more details on the procedure to calibrate the BNO08X, refer to the BNO08X Sensor Calibration Procedure
application note [7]. Note that in normal use the device will be exposed to conditions that will allow calibration to
occur with no explicit user input. The steps are provided below if a user wants to force a calibration.


                 Sensor                                       Calibration Procedure
              Accelerometer            •    For 3D calibration the device should be moved into 4-6 unique
                                            orientation and held in each orientation for about 1 second to
                                            calibrate the accelerometer
                                       •    For planar calibration the device should be rotated around it Z-axis
                                            by at least 180 degrees
                Gyroscope          Device should be set down on a stationary surface for approximately 2-3
                                   seconds to calibrate the gyroscope
              Magnetometer         Device should be rotated about 180° and back to the beginning position in
                                   each axis (roll, pitch and yaw). Device should be rotated about 2 seconds on
                                   each axis.
                                  Figure 3-2: Calibration procedure for sensors

3.3     Interactive Calibration (BNO086 only)
As discussed in section 3.1.3, removal of gyroscope ZRO is critical to reduce heading drift. For applications in
which low heading drift is vital, the BNO086 offers a means of calculating and removing the ZRO more frequently
and more accurately than the normal opportunistic dynamic calibration. This method is called Interactive
Calibration and involves cooperation between the host application and the BNO086 to identify emerging needs to
update calibration and provide specific opportunities to update the calibration. See reference 0 for more details.



3.4     Persist Dynamic Calibration Data
The BNO08X stores updated Dynamic Calibration Data (DCD) to RAM frequently (every 5 seconds). At non-
power-up reset, the hub will persist the last-stored DCD from RAM to FRS. To clear the DCD from RAM before
system reset, use the Clear-DCD and Reset command [1].
For SHTP applications, the last stored DCD can be saved to FRS during operation through specific commands;
the Save DCD request and the Configure Periodic DCD Save command. More details are available in [1].




www.ceva-dsp.com                           © 2023 CEVA, Inc. All rights reserved                                    39 / 59
```


## PDF page 40

```text
                                                 BNO08X Datasheet                                    1000-3927 v1.17


4 BNO08X Orientation
The BNO08X can be mounted in an arbitrary manner that facilitates the manufacture of the device it is included
within. The outputs of the BNO08X must however be aligned to a frame of reference that is practical to the user.
This essentially requires mapping the orientation of the BNO08X to the orientation of the device within which it is
housed. Re-mapping of the BNO08X’s sensor outputs to the supporting device’s frame of reference is achieved
by programming an FRS record. The system orientation FRS record (0x2D3E) applies a rotation to the sensor
outputs and all the derived outputs (e.g. rotation vectors). The system orientation record is a unit quaternion, with
each coordinate represented as a 32-bit fixed point number with a Q-point of 30 to represent a fractional number.
The default BNO08X axis orientation is shown in Figure 4-1. A positive value is reported for counter-clockwise
rotations.




                                      Figure 4-1: BNO08X axis orientation


If the BNO08X was mounted such that its positive Y-axis was aligned opposite to the X-axis of the device it was
mounted in, but with its Z-axis aligned correctly, a clockwise rotation of 90˚ around the Z-axis would be required
(see Figure 4-2).




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                  40 / 59
```


## PDF page 41

```text
                                                BNO08X Datasheet                                    1000-3927 v1.17




                                  Figure 4-2: BNO08X mounted in a device
                                                          √2       √2
This rotation would be represented by a quaternion of (       ,0,0, 2 ).
                                                          2
Assume that for the device the BNO08X is mounted in, the Z-axis is Up, the X-axis is East and the Y-axis is
North. Examples of mappings for BNO08X mounting are provided below to clarify the use of the FRS record:
                 BNO08X physical axis aligned                        Mapping quaternion
                   X        Y             Z             Qw              Qx        Qy        Qz
                 East     North           Up             1               0         0         0
                 North    West            Up          (√2)/2             0         0      (√2)/2
                 West     South           Up             0               0         0         1
                 South     East           Up          (√2)/2             0         0      −(√2)/2
                 East     South        Down              0               0        -1         0
                 North     East        Down              0           -(√2)/2   -(√2)/2       0
                 West     North        Down              0              -1         0         0
                 South    West         Down              0           -(√2)/2    (√2)/2       0
                  Up      South          East            0               0     −(√2)/2    (√2)/2
                 North      Up           East           1/2            -1/2      -1/2        1/2
                 Down     North          East         (√2)/2         -(√2)/2       0          0
                 South    Down           East           1/2            -1/2       1/2       -1/2
                  Up      North         West         -(√2)/2         -(√2)/2       0          0
                 North    Down          West           -1/2            -1/2      -1/2       -1/2
                 Down     South         West             0               0     −(√2)/2    −(√2)/2
                 South      Up          West            1/2             1/2      -1/2       -1/2
                  Up       East         North          -1/2            -1/2      -1/2        1/2
                 West       Up          North            0           −(√2)/2       0      (√2)/2
                 Down     West          North           1/2            -1/2       1/2        1/2
                 East     Down          North        -(√2)/2             0     −(√2)/2        0
                  Up      West         South            1/2             1/2      -1/2        1/2
                 West     Down         South             0           −(√2)/2       0      −(√2)/2
                 Down      East        South            1/2            -1/2      -1/2       -1/2
                 East       Up         South          (√2)/2             0     −(√2)/2        0
                                   Figure 4-3: Multiple 90 degree rotations


www.ceva-dsp.com                       © 2023 CEVA, Inc. All rights reserved                                  41 / 59
```


## PDF page 42

```text
                                                  BNO08X Datasheet                                    1000-3927 v1.17


The highlighted row is also shown pictorially in Figure 4-2. For rotations that are not a multiple of 90 degree
rotations the appropriate angular rotations should be applied.

4.1.1 Tare
The outputs generated by BNO08X can also be oriented under user control by a tare function. Taring allows a
user to mount the BNO08X in an arbitrary manner and by invoking the tare command the SH-2 software will
determine the orientation that needs to be applied to the outputs to align with an East, North, Up frame of
reference. This orientation will then be applied to all motion outputs.
If the Rotation Vector or Geomagnetic Rotation Vector is necessary for the application, the BNO08X must have
resolved magnetic North before applying the tare function. Otherwise when the magnetometer calibrates the
heading will change.

4.1.1.1     Types of Tare
There are two types of tare defined, tare around all axes and tare around the Z-axis. A tare around all axes solves
for tilt and heading. A tare around the Z-axis only solves for the heading. In an HMD application, the all-axis tare
is useful for initial adjustment, further adjustments are likely for heading reasons and then the tare-Z may be more
applicable.

The result of a tare function is applied wherever power is applied to the device. To ensure the reorientation is
permanent there is a Persist Tare function to allow storage of the orientation configuration. Note that if tare-Z is
used for the Game Rotation Vector, then the BNO08X must be reset. At startup, the BNO08X will apply the new
orientation to the Game Rotation Vector. Persist tare does not apply to the Game Rotation Vector.

Refer to the BNO08X Tare Function Usage Guide application note [8] for more information on how to use tare.




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                    42 / 59
```


## PDF page 43

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


5 Getting Started with BNO08X
5.1     BNO08X in UART-RVC mode
When the BNO08X is configured for UART-RVC mode it starts up and sends a string containing the following
information:
%Hillcrest Labs 10003608
%SW Ver 3.2.x
%(c) 2017 Hillcrest Laboratories, Inc.

This message informs the user that the BNO08X has exited reset and provides version information.
Following this message the BNO08X will issue packets according to 1.2.5.2

5.2     BNO08X in non UART-RVC configurations
The BNO08X starts up with all sensors disabled, waiting for the application processor (AP) to configure it. The
following sections provide details of typical sequences to enable features on the BNO08X.

5.2.1 Establishing Contact
After power up or reset the BNO08X will assert the interrupt (HOST_INTN) indicating that the reset routine has
completed and that the BNO08X is ready for communication. A read from the BNO08X will return the initial SHTP
advertisement packet. This packet details the SHTP channel assignments and packet length parameters (see [1]
and [2]). Following the SHTP advertisement packet, the individual applications built in to the BNO08X will send a
packet indicating they have left the reset state:
    • The executable will issue a reset message on SHTP channel 1
    • SH-2 will issue an unsolicited initialization message on SHTP channel 2

Beyond these initial messages the BNO08X will wait for configuration by the host.

5.2.2 Reading/Writing the BNO08X
After reset the sensor hub resides in a sleep state waiting for functionality to be enabled. The sensors or functions
(detailed in section 2) to enable are application dependent. For example setting the accelerometer report for a
period of 60ms (useful for screen rotation analysis in mobile phones) would be achieved by a Set Feature report
as shown in Figure 5-1. The report ID for the accelerometer is 0x1 and a period of 60ms (or 60,000 µs) equates to
a report interval of 0x0000EA60.


                             Byte                       Description
                              0      SHTP LSB = 0x15
                              1      SHTP MSB = 0x00
                              2      SHTP channel = 0x02
                              3      SHTP sequence number
                              4      Report ID = 0xFD
                              5      0x1
                              6      0
                              7      0
                              8      0
                              9      0x60
                              10     0xEA
                              11     0
                              12     0



www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                 43 / 59
```


## PDF page 44

```text
                                                BNO08X Datasheet                                        1000-3927 v1.17


                            Byte                         Description
                             13     0
                             14     0
                             15     0
                             16     0
                             17     0
                             18     0
                             19     0
                             20     0


              Figure 5-1: BNO08X set feature report (accelerometer) including SHTP header
Once set the BNO08X will issue a get feature response report and then provide input reports at the period set in
the get feature response report. Note that the get feature response may provide a different period than was
requested due to the supported rates in the underlying physical sensor. The BNO08X will also respond with a
Get Feature Response if the sensor’s reporting period is changed. The reporting period can change if a sensor is
turned on/off or if as a result of another sensor being enabled or disabled the sensor’s rate must be modified.
The sensor input report will be preceded by a timebase reference packet. A report including SHTP header will
have the format as seen in

  Byte                         Description                                            Packet type
   0       Length LSB = 19
   1       Length MSB = 0
                                                                                      SHTP header
   2       Channel = 3
   3       Sequence Number
   4       Report ID = 0xFB (Timebase Reference)
   5       Base Delta LSB
   6       Base Delta                                                              Timebase reference
   7       Base Delta
   8       Base Delta MSB
   9       Report ID = 0x01(accelerometer)
   10      Sequence number
   11      Status
   12      Delay
   13      Accelerometer Axis X LSB
                                                                                Accelerometer input report
   14      Accelerometer Axis X MSB
   15      Accelerometer Axis Y LSB
   16      Accelerometer Axis Y MSB
   17      Accelerometer Axis Z LSB
   18      Accelerometer Axis Z MSB
                Figure 5-2: Accelerometer & timebase input report including SHTP header




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                    44 / 59
```


## PDF page 45

```text
                                                   BNO08X Datasheet                                           1000-3927 v1.17


6         BNO08X Characteristics
This section describes the electrical and performance characteristics of the BNO08X. The BNO08X is a custom
part that CEVA provides based upon the Bosch Sensortec BMF055. All of the BNO08X I/O pins meet CMOS and
TTL requirements.
Note that the electrical and mechanical sections of the specification reported here are reproduced from the Bosch
Sensortec BMF055 datasheet. The data in this section is reported for convenience, the reader is encouraged to
consult the BMF055 datasheet [5] to verify all parameters.

6.1       Absolute Maximum Electrical Ratings
Exposure to maximum rating conditions for extended periods may affect device reliability.
    Parameter                                         Symbol                 Conditions           Rating             Unit
                                                      VDDIO                                     -0.3 to 3.63           V
    Voltage at supply pin
                                                      VDD                                       -0.3 to 4.25           V
    Voltage at any logic pin                          Vnon-supply                                VDDIO+0.3             V
    Storage temperature                               Trps                   ≤ 65% rel. H       -50 to +150           °C
                                                                           Duration ≤ 200μs
                                                      MechShock200μs                                 10,000           g

    Mechanical shock                                  MechShock1ms        Duration ≤ 1.0ms            2,000           g
                                                                          Free fall onto hard
                                                      MechShockfreefall                                1.8            m
                                                                              surfaces
                                                      ESDHBM               HBM at any pin              2             kV
    ESD                                               ESDCDM                     CDM                  500             V
                                                      ESDMM                        MM                 200             V
                                       Figure 6-1: BNO08X maximum ratings

6.2         Recommended Operating Conditions
            Parameter                     Symbol            Conditions        Min       Typ     Max          Unit
            Supply voltage
                                          VDDIO                               1.7               3.6           V
            (μC and I/O Domain)
            Supply voltage
                                          VDD                                 2.4               3.6           V
            (only sensors)
            Operating temperature                                             -40               85            °C
                                     Figure 6-2: BNO08X operating conditions

6.3       Power Management
The BNO08X has two distinct power supply pins:
     •    VDD is the main power supply for the internal sensors
     •    VDDIO is the power supply pin used for the supply of the internal microcontroller and the digital interfaces.
For the switching sequence of power supply VDD and VDDIO, it is mandatory that VDD is powered on and driven to
the specified level before or at the same time as VDDIO is powered ON. Otherwise there are no limitations on the
voltage levels of both pins relative to each other, as long as they are used within the specified operating range.
The reader is encouraged to read sections 1 and 3.2 of the BMF055 [5] datasheet for more information.




www.ceva-dsp.com                           © 2023 CEVA, Inc. All rights reserved                                          45 / 59
```


## PDF page 46

```text
                                                          BNO08X Datasheet                                            1000-3927 v1.17


 6.4        Electrical Characteristics
Parameter                           Symbol              Conditions                     Min               Typ              Max          Unit
                                                       VDDIO=1.7-2.7V               0.7*VDDIO                                          VDDIO
Input high voltage                    VIH
                                                       VDDIO=2.7-3.6V               0.55*VDDIO                                         VDDIO
                                                       VDDIO=1.7-2.7V                                               0.25*VDDIO         VDDIO
Input low voltage                     VIL
                                                       VDDIO=2.7-3.6V                                                0.3*VDDIO         VDDIO
Output high voltage                   VOH        VDDIO > 1.7V , IOH=10mA            0.8*VDDIO          0.9*VDDIO                       VDDIO
Output low voltage                    VOL         VDDIO > 3V, IOL=20mA                                 0.1*VDDIO     0.2*VDDIO         VDDIO
POR Voltage threshold on
                                   VDDIO_POT+                                                            1.45                            V
VDDIO-IN rising                                   VDDIO falls at 1V/ms or
POR Voltage threshold on                                   slower
                                   VDDIO_POT-                                                            0.99                            V
VDDIO-IN falling
                                        Figure 6-3: BNO08X electrical characteristics

 6.5        AC Characteristics
 6.5.1 I2C Timing
 The I2C interfaces of the BNO08X are compliant with the I2C specification [4]. The BNO08X provides master
 functionality to the environmental sensors and slave functionality to the application processor

                     Parameter                         Symbol    Conditions                      Min        Max    Unit
                     SCL clock frequency                                                                    400    kHz
                     SCL high period                   thigh                                     0.6                µs
                     SCL low period                    tlow                                      1.3                µs
                     Rise time for SCL and SDA                   10pF < Cb < 400pF            20+0.1Cb              ns
                     Fall time for SCL and SDA                   10pF < Cb < 400pF           20 + 0.1Cb             ns
                     SDA setup time                    tsu                                       0.1                µs
                     SDA hold time                     th                                         0                 µs
                     Hold time for start condition     thst                                      0.6                µs
                     Setup time for a stop condition   tsusp                                     0.6                µs
                     Idle time between accesses        tbf                                       1.3                µs
                                                Figure 6-4: I2C timing parameters
                     thst                                                                                                       tbf
                                       tsu       th

          SDA

          SCL
                                                                                                          tsusp
                                                                            thigh
                                                                                                  tlow

                                                       Figure 6-5: I2C timing

 6.5.2 SPI timing
 The Serial Peripheral Interface (SPI) is a synchronous serial interface. The BNO08X implements the slave side of
 the interface for host communications. The BNO08X supports 4 wire mode and implements SPI mode 3: CPOL =
 1 and CPHA = 1. SPI mode 3 implies that the clock idles at ‘1’ and that data is clocked in on the positive edge.




 www.ceva-dsp.com                               © 2023 CEVA, Inc. All rights reserved                                                 46 / 59
```


## PDF page 47

```text
                                                                  BNO08X Datasheet                                                   1000-3927 v1.17


                            Parameter                                Symbol          Min          Typ        Max      Unit
                            SPI (CLK) clock frequency                                                         3       MHz
                            SPI clock period                         tck             0.33                              µs
                            SCL high period                          tckh                     0.5*tck
                            SCL low period                           tckl                     0.5*tck
                            CS setup to CLK                          tcssu            0.1                             µs
                            CS hold                                  tcssh           16.83                            Ns
                            CS to MISO out                           tcsso                                    31      ns
                            CLK to MISO out valid                    tsov                                     35      ns
                            MISO hold                                tsoh            13.7                             ns
                            MISO hold after CS                       tcssoh           7.4                             ns
                            MOSI setup time                          tsisu            25                              ns
                            MOSI hold time                           tsih             5.4                             ns
                                                 Figure 6-6: SPI timing parameters
                                                                                                                             tcssh
                         tcssu
           CS                                                                 tck         twckh              twckl
                                                      tsoh          tsov
          CLK                                                                                                                        tcssoh
                 tcsso

          MISO                    D7             D6          D5        D4            D3           D2         D1         D0


          MOSI                    D7             D6          D5        D4            D3           D2         D1         D0


                                 tsisu
                                                      tsih

                                                             Figure 6-7: SPI timing

6.5.3 Startup timing
The timing for BNO08X startup for I2C and SPI modes is show below. The host may begin communicating with
the BNO08X after it has asserted H_INTN. In UART mode, the BNO08X sends an advertisement message when
it is ready to communicate.

                                   Parameter                         Symbol          Min     Typ       Max     Unit
                                   Reset signal                      tnrst           10                         ns
                                   Internal Initialization           t1              90                        ms
                                   Internal configuration            t2                       4                ms
                                                 Figure 6-8: SPI timing parameters



                                         tnrst                                  t1                                     t2


                 NRST


                H_INTN


                                                        Figure 6-9: Startup timing



www.ceva-dsp.com                                  © 2023 CEVA, Inc. All rights reserved                                                       47 / 59
```


## PDF page 48

```text
                                                              BNO08X Datasheet                                             1000-3927 v1.17


6.5.4 Interrupt timing
In SPI and I2C mode the HOST_INTN signal is used by the BNO08X to indicate to the application processor that
the BNO08X needs attention. The signal is active low and is asserted until either the end of the I2C device
address is registered or the SPI chip select is observed.

H_INTN


H_SDA                                 Start    D0        D1         D2        D3    D4         D5        D6       D7               ACK


H_SCL

                                                                                                                         tclid

                                         Figure 6-10: Host interrupt timing – I C mode    2


If the BNO08X is asleep the host can wake it by assertion of the wake signal. The BNO08X will assert the
interrupt line to indicate it is awake. If the BNO08X interrupted the host then it is already awake.
                                                                          twk


           PS0/wake

                                                                                                          tcsid
            H_INTN


            H_CSN


                                         Figure 6-11: Host interrupt timing - SPI mode
                      Parameter                                               Symbol     Min       Typ   Max      Unit
                      I2C address recognize to H_INTN dessert                 tclid                 10             µs
                      BNO08X wakeup from wake signal assert                   twk                  150             µs
                      H_CSN to H_INTN de-assert                               tcsid                800             ns
                      H_INTN assert to UART transmission                      tiatx                7.7             µs
                      H_INTN deassertion to H_INTN reassert                   tidia      1                         µs
                                                    Figure 6-12: H_INTN timing
In UART-SHTP mode the interrupt is asserted prior to the UART transmission. It is assumed that the host can
always accept data over its UART. The interrupt is asserted approx. 7.7 µs prior to the first bit of UART
transmission:


 H_INTN               tiatx


                                                                                                                                      tidia
UART_TX                       Start       D0        D1         D2        D3        D4         D5         D6       D7      Stop


                                  Figure 6-13: Host interrupt timing - UART-SHTP mode
The interrupt will be de-asserted prior to the termination of the UART transmission.

6.6       Mechanical Characteristics
The sensors within the BNO08X are specified by Bosch Sensortec. The mechanical and electrical details of the
raw sensors are specified in the BMF055 datasheet [5].
The SH-2 software within the BNO08X configures the accelerometer for a range of +/- 8g and the gyroscope with
a range of +/- 2000 ˚/s.

www.ceva-dsp.com                                © 2023 CEVA, Inc. All rights reserved                                               48 / 59
```


## PDF page 49

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


6.7     Performance Characteristics
The SH-2 software within the BNO08X calibrates and interprets the raw sensor data received from the gyroscope,
accelerometer and magnetometer. By fusing the data from all three sensors the BNO08X can provide an estimate
of the orientation of the device, the applied acceleration (i.e. gravity is removed from the accelerometer signal)
and an estimate of gravity. Figure 6-14 captures the performance of the BNO08X when using an external clock or
crystal.

             Composite Sensor            Calibration    Measurement       Performance Metric        Value
                                                        Dynamic           Rotation Error          3.5°
                Rotation Vector            Nominal
                                                        Static            Rotation Error          2.0°
                                                        Dynamic           Non-heading Error       2.5°
           Gaming Rotation Vector          Nominal      Static            Non-heading Error       1.5°
                                                        Dynamic           Heading Drift           0.5°/min
                                                        Dynamic           Rotation Error          4.5°
         Geomagnetic Rotation Vector       Nominal
                                                        Static            Rotation Error          3.0°
                    Gravity                Nominal      Static            Angle Error             1.5°
              Linear Acceleration          Nominal      Dynamic           Accuracy                0.35 m/s2
                 Accelerometer             Nominal      Dynamic           Accuracy                0.3 m/s2
                  Gyroscope                Nominal      Dynamic           Accuracy                3.1°/s
                 Magnetometer               Either      Dynamic           Accuracy                1.4uT
                                       Figure 6-14: BNO08X Performance
The results above were generated by simulation. 210 physical devices were characterized and each of these
models was subjected to simulated motion and the variation from truth catalogued. The rotation vector and
geomagnetic rotation vector are highly dependent on the environmental conditions (specifically the magnetic
field). In practice the rotation vector is typically accurate to 5˚ and the geomagnetic rotation vector to 10˚.

6.8       Latency
Latency is a measure of the response of the BNO08X to motion and is typically reserved for continuous sensors.
The time to generate an output can be divided into several parameters:
    •   Sensor delay
    •   Processing delay
    •   Algorithmic delay
    •   Communication delay
The sensors within the BNO08X will generate an output reflecting motion or a measure of the magnetic field
within the sample period just measured. The sensor interrupt is assumed to be the end of the sample.
The processing time of the BNO08X is dependent on the output of interest. The output for fused sensors (rotation
vector, gravity etc.) follows a gyroscope sample and requires additional processing to fuse the gyroscope data
with the accelerometer and magnetometer data.
Processing time is measured from data becoming available from the sensor to data being made available to the
host (HOST_INTN asserted).
The algorithms present in SH-2 apply BW limiting filtering which in turn adds a small delay to the signal.
The communication delay is dependent upon the transfer speed of the communication medium chosen and the
host’s ability to respond to interrupts and support the maximum clock rate of the BNO08X.
The measured latency for the BNO08X is provided in Figure 6-13.




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                 49 / 59
```


## PDF page 50

```text
                                                BNO08X Datasheet                                   1000-3927 v1.17


                                                              Typical latency
                                             Sensor
                                                              100Hz 200Hz
                                       Gyro rotation vector   6.6ms 3.7ms
                                       Rotation vector        6.6ms 3.7ms
                                       Game rotation vector   6.6ms 3.7ms
                                 Figure 6-15: Typical latency measurements



6.9       Report Rates
The number of reports per second that the BNO08X can reliably deliver is dependent on the interface bandwidth
and the processing time within the BNO08X for the generation of the data. The sensors also have discrete sample
rates which must be taken into account when configuring the device. The BNO08X will attempt to satisfy the
requested rate based on the following formula:
                        0.9 * RequestedRate <= ConfiguredRate <= 2.1 * RequestedRate
The maximum available data rates that can be configured per sensor are listed in Figure 6-16. Each individual
sensor can operate at its maximum rate; however, all sensors cannot operate at their maximum rate
simultaneously. For applications that require high sampling rates, the user will need to find a balance between
which sensors to operate at a high rate and which sensor to operate at a low rate to find a configuration that
meets their needs and fits within the interface bandwidth and processing power of the BNO08x.

                              Composite Sensor                Maximum Data rates (Hz)
                              Gyro rotation Vector                    1000
                                 Rotation Vector                       400
                             Gaming Rotation Vector                    400
                           Geomagnetic Rotation Vector                  90
                                     Gravity                           400
                               Linear Acceleration                     400
                                  Accelerometer                        500
                                   Gyroscope                           400
                                  Magnetometer                         100
                                      Figure 6-16: Maximum sensor rates
Note that the AR/VR stabilized versions of the various rotation vectors have the same maximum report rates as
their non-stabilized version.

6.10      Power Consumption
The power consumption of the BNO08X is dependent on the configuration of the device including the sample
rates of various sensors and even the environment in which the device is being used. For example, an
environment in which the magnetic field is constantly changing will require additional processing to resolve the
current heading if a sensor that requires the magnetic field is configured. The tables below provide typical power
consumption numbers for sample configurations using the SPI interface. Measurements are taken with VDDIO at
3V and VDD at 3.3V.




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                50 / 59
```


## PDF page 51

```text
                                            BNO08X Datasheet                           1000-3927 v1.17


                                       Sensor Rate                 Current (mA)          Power
                  Sensor
                                          (Hz)          VDDIO Rail          VDD Rail     (mW)
  Idle Power                                 —               0.12             0.01        0.39
                                             Android Sensors
                                           1000             7.14              7.50       46.17
  Gyro rotation Vector
                                            400             7.10              7.50       46.05
  6 and 9-Axis Sensor Fusion                100             3.50              7.50       35.25
  (Rotation Vector, Linear                  200             5.34              7.50       40.77
  Acceleration, Gravity, Game
  Rotation Vector)                          400                6.90           7.50       45.45
                                            100                3.22           7.50       34.41
  Calibrated Gyroscope                      200                4.83           7.50       39.24
                                            400                6.56           7.50       44.43
  Geomagnetic Rotation Vector              100           2.51                 4.70       23.04
  Calibrated Accelerometer                 125           1.23                 0.15       4.19
                                           250           2.42                 0.15       7.76
                                           500           4.81                 0.15       14.93
  Significant Motion Detector             31.25          0.40                 0.14       1.66
  Step counter/detector                   31.25          0.42                 0.14       1.72
                                         CEVA Custom Sensors
  Tap detector                              -            0.13                 0.15       0.89
  Shake gesture                           31.25          0.32                 0.14       1.42
  Stability Classifier                   100/62.5        2.85                 7.50       33.30
  Stability detector                      31.25          0.12                 0.01       0.39
                                Figure 6-17: Power consumption BNO085




www.ceva-dsp.com                    © 2023 CEVA, Inc. All rights reserved                        51 / 59
```


## PDF page 52

```text
                                            BNO08X Datasheet                           1000-3927 v1.17


                                       Sensor Rate                 Current (mA)          Power
                  Sensor
                                          (Hz)          VDDIO Rail          VDD Rail     (mW)
  Idle Power                                 —             0.047              0.01        0.17
                                             Android Sensors
                                           1000             6.84              7.50       45.27
  Gyro rotation Vector
                                            400             6.72              7.50       44.91
  6 and 9-Axis Sensor Fusion                100             3.18              7.50       34.29
  (Rotation Vector, Linear                  200             4.83              7.50       39.24
  Acceleration, Gravity, Game
  Rotation Vector)                          400                6.55           7.50       44.40
                                            100                2.92           7.50       33.51
  Calibrated Gyroscope                      200                4.39           7.50       37.92
                                            400                6.32           7.50       43.71
  Geomagnetic Rotation Vector              100           2.11                 4.70       21.84
  Calibrated Accelerometer                 125           0.98                 0.15       3.44
                                           250           1.93                 0.15       6.29
                                           500           3.84                 0.15       12.02
  Significant Motion Detector             31.25          0.34                 0.14       1.66
  Step counter/detector                   31.25          0.36                 0.14       1.54
                                         CEVA Custom Sensors
  Tap detector                              -            0.05                 0.15       0.65
  Shake gesture                           31.25          0.25                 0.14       1.21
  Stability Classifier                   100/62.5        2.62                 7.50       32.61
  Stability detector                      31.25          0.05                 0.01       0.18
                                Figure 6-18: Power consumption BNO086




www.ceva-dsp.com                    © 2023 CEVA, Inc. All rights reserved                        52 / 59
```


## PDF page 53

```text
                                                BNO08X Datasheet                                   1000-3927 v1.17


7 Packaging Information
All information in this section is reproduced for convenience. Bosch Sensortec is the manufacturer of the physical
BNO08X device. The reader should consult the BMF055 datasheet [5] for final verification.

7.1     Package Outline
The BNO08X is available in a 28-pin Land Grid Array (LGA) package. Units are in mm. Note: Unless otherwise
specified tolerance = decimal ±0.1mm.




                         Figure 7-1: 28 pin LGA package outline (Image from Bosch)




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                53 / 59
```


## PDF page 54

```text
                                               BNO08X Datasheet                                 1000-3927 v1.17


7.2    Landing Pattern Recommendation

                                                                              0.675



                                       6        5      4         3   2          1       0.25

                                       7                                       28

                                       8                                       27

                                       9                                       26

                                      10                                       25
             5.2                                                                            0.25
                                      11                                       24

                                      12                                       23
                                                                                           2.025
                                      13                                       22

                                      14                                       21

                                      15        16     17    18      19        20          0.575



                                               1.225         0.25

                                                           3.8
                               Figure 7-2: Landing pattern recommendation



7.3    Soldering Guidelines
Bosch Sensortec publishes a handling, soldering & mounting guide. The reader should consult that guide for
manufacturing guidelines [5].

7.4    Marking
The reader should consult section 10 of the BMF055 datasheet for more information. For convenience, the part
marking information has been reproduced in Figure 7-3.




www.ceva-dsp.com                      © 2023 CEVA, Inc. All rights reserved                                  54 / 59
```


## PDF page 55

```text
                                                  BNO08X Datasheet                               1000-3927 v1.17




                    Symbol         Name                              Remark
                    •          Pin 1 identifier   —
                                  Product         3 numeric digits internal identification for
                    701
                                  number          product type
                    T           Second row        4-digits Internal use
                    C            Third row        3-digits Numerical counter
                                Figure 7-3: Marking of mass production parts

7.5     Soldering Guidelines
The moisture sensitivity level of the BNO08X sensors corresponds to JEDEC Level 1, see also
   • IPC/JEDEC J-STD-020C "Joint Industry Standard: Moisture/Reflow Sensitivity Classification for non-
       hermetic Solid State Surface Mount Devices"
   • IPC/JEDEC J-STD-033A "Joint Industry Standard: Handling, Packing, Shipping and Use of
       Moisture/Reflow Sensitive Surface Mount Devices"

The sensor fulfils the lead-free soldering requirements of the above-mentioned IPC/JEDEC standard, i.e. reflow
soldering with a peak temperature up to 260°C.

7.6     Handling Instructions
Micromechanical sensors are designed to sense acceleration with high accuracy even at low amplitudes and
contain highly sensitive structures inside the sensor element. The MEMS sensor can tolerate mechanical shocks
up to several thousand g's. However, these limits might be exceeded in conditions with extreme shock loads such
as e.g. hammer blow on or next to the sensor, dropping of the sensor onto hard surfaces etc.
We recommend avoiding g-forces beyond the specified limits during transport, handling and mounting of the
sensors in a defined and qualified installation process.
This device has built-in protections against high electrostatic discharges or electric fields (e.g. 2kV HBM);
however, anti-static precautions should be taken as for any other CMOS component. Unless otherwise specified,
proper operation can only occur when all terminal voltages are kept within the supply voltage range. Unused
inputs must always be tied to a defined logic voltage level.
For more details on recommended handling, soldering and mounting please contact CEVA and ask for the
“Handling, soldering and mounting instructions” document [6].

7.7     Environmental Safety
The BNO08X sensor meets the requirements of the EC restriction of hazardous substances (RoHS and RoHS2)
directive, see also:
    • Directive 2002/95/EC of the European Parliament and of the Council of 27 January 2003 on the restriction
         of the use of certain hazardous substances in electrical and electronic equipment.

7.7.1 Halogen content
The BNO08X is halogen-free. For more details on the analysis results please contact CEVA.




www.ceva-dsp.com                       © 2023 CEVA, Inc. All rights reserved                               55 / 59
```


## PDF page 56

```text
                                              BNO08X Datasheet                                    1000-3927 v1.17


8 Version History
               Version                      Changes                                  Date
                         Section 1.2.2: Removed note
               1.17      Section 1.2.2.2: Added polling behavior for         July 24, 2023
                         BNO086
               1.16      Removed referenced to BNO080                        May 16, 2023
                         Update section 1.2.2 indicating that the
               1.15                                                          April 28, 2023
                         BNO086 does not support I2C polling.
               1.14      Added note about BNO080 being EOL                   February 2, 2023
                         Further clarified I2C responses when polled
               1.13                                                          January 12, 2023
                         in section 1.2.2.2.
                         Corrected feature list to indicate that certain
                         features are present in both BNO085 and
                         BNO086. Clarify BNO086 behavior in
               1.12                                                          November 23, 2022
                         section 1.2.4.1. Clarify sensor operations in
                         section 6.9. Clarified I2C responses when
                         polled in section 1.2.2.2.
               1.11      Update References section                           June 10, 2021
                         Minor edits. Added MI and MR to section
               1.10                                                          April 13, 2021
                         1.2.5.2.
                         Clarify need to respond to H_INTN in
               1.9       section 1.2.4.1. Add BNO086 specific                March 26, 2021
                         updates.
                         Updates to state that the internal clock may
                         not be used with either of the UART
               1.8                                                           February 5, 2020
                         interfaces. Add section 6.5.3 on startup
                         timing. Add section 1.2.4.1 on H_INTN.
               1.7       Remoted erroneous trademark in section 10           September 23, 2019
                         Removed CTS3 from section 3. Correct
                         order of axes listed in section 4.1.1.
                         Remove negative sign for Z-axis
               1.6                                                           April 30, 2019
                         acceleration in section 1.2.5.2.Update
                         capacitance loading for crystal in section
                         1.2.1.
                         Remove clutter from leftover change
               1.5       tracking. Revised section 1.1.3 to be more          January 24, 2018
                         generally applicable.
               1.4       Changes to cover BNO085                             October 23, 2017
                         Update IO voltages in Figure 6-3. Update
               1.3                                                           October 20, 2017
                         Figure 6-13
                         Added latency measurements. Clarified
                         sequence number description. Clarified
                         continuation bit in length field of SHTP
                         header. Clarified SHTP channel usage.
               1.2       Added header to Figure 5-1. Update                  May 19, 2017
                         performance table. Clarified need to support
                         clock stretching for I2C interface. Corrected
                         descriptions in pin table for ENV_SCL and
                         ENV_SDA.
                         Change VCC references to VDDIO. Minor
               1.1                                                           February 16, 2017
                         editorial updates. Update references.

www.ceva-dsp.com                     © 2023 CEVA, Inc. All rights reserved                                 56 / 59
```


## PDF page 57

```text
                                             BNO08X Datasheet                                   1000-3927 v1.17


               Version                     Changes                                  Date
               1.0       Initial release                                     February 2, 2017




www.ceva-dsp.com                     © 2023 CEVA, Inc. All rights reserved                               57 / 59
```


## PDF page 58

```text
                                                  BNO08X Datasheet                                     1000-3927 v1.17


9 References
The reference(s) listed with external links are up to date at time of the latest revision, but are subject to change
without notice. Please contact us if you need assistance (both in finding a document and general inquiries).
1.   1000-3625 – SH-2 Reference Manual, CEVA.
2.   1000-3535 – Sensor Hub Transfer Protocol, CEVA.
3.   Android Sensors HAL Overview, Google. (http://source.android.com/devices/sensors/index.html)
4.   I2C-bus specification and user manual, NXP Semiconductors. (https://www.nxp.com/docs/en/user-
     guide/UM10204.pdf)
5.   BMF055 datasheet, Bosch Sensortec. (https://www.bosch-
     sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmf055-ds000.pdf)
6.   BNO055 handling, soldering & mounting instructions. (https://www.bosch-
     sensortec.com/media/boschsensortec/downloads/handling_soldering_mounting_instructions/bst-bno055-
     hs000.pdf)
7.   1000-4044 – BNO08X Sensor Calibration Procedure, CEVA.
8.   1000-4045 – BNO08X Tare Function Usage Guide, CEVA.




www.ceva-dsp.com                         © 2023 CEVA, Inc. All rights reserved                                   58 / 59
```


## PDF page 59

```text
                                                 BNO08X Datasheet                                   1000-3927 v1.17


10 Notices
© Copyright 2023 CEVA, Inc. and/or its subsidiaries (“CEVA”) All rights reserved. All specifications are subject to
change without notice.
MotionEngine is a registered trademark of CEVA. Other company and product names mentioned in this document
may be the trademark or registered trademark of their respective owners.
Disclaimer: The information furnished herein is believed to be accurate and reliable. However, the information is
provided “AS IS”, without any express or implied warranty of any kind including warranties of merchantability, non-
infringement of intellectual property, or fitness for any particular purpose.
In no event shall CEVA or its suppliers be liable for any claims and/or damages whatsoever arising out of the use
of or inability to use the materials. CEVA and its suppliers further do not warrant the accuracy or completeness of
the information, text, graphics or other items contained within these materials. CEVA may make changes to these
materials, or to the products described within.


                                               www.ceva-dsp.com




www.ceva-dsp.com                        © 2023 CEVA, Inc. All rights reserved                                 59 / 59
```
