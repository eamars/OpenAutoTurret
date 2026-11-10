# Juxie R68 joint ("MZ17-mini"): datasheets transcribed

Transcribed 2026-11-10 from the vendor PDFs the owner obtained, so nobody has to extract them again.
The evaluation against HDP-001 is in [HDP-001 §8.8](HDP-001.md). The **R68 Lite is the MZ17-mini**;
the **R68 DNN** matches the MZ17 of the preliminary sheet (HDP-001 §8.6: Ø12 bore, 75.6 mm, 19 bit).
The PDFs themselves are not in the repository (vendor copyright); the owner keeps them under
`Downloads/MZ17 Servo Motors/`. Chinese labels are translated; the original term is kept where it
matters. Anything inferred rather than read is marked *(inferred)*.

**Outcome (owner, 2026-11-10): the series is out for the turret.** Its position covers one turn, so
it cannot drive a continuously turning yaw (HDP-001 §8.8). This file stays as the record, and as the
starting point if another Juxie part comes up.

| File | What it is | Applies to the R68? |
|---|---|---|
| `CE-RB-R68-101-Lite-CO-I.pdf` | 2D drawing + spec table, **R68 Lite**, title block Wuxi Juxie Intelligent Drive Technology (无锡巨蟹智能驱动科技有限公司), dated 2026-01-09 | yes: **the part in question** |
| `CE-RB-R68-101-DNN-CO-I.pdf` | 2D drawing + spec table, R68 DNN, title block Chengdu Alphat Machinery (成都阿尔法特机械设备有限公司), dated 2025-10-17, same drafter | an earlier/sibling variant |
| `68lite通讯电源线-50cm-402000914(1).pdf` | 50 cm power + CAN harness for the "68lite", Juxie part 402000914, 2026-03-03 | yes |
| `CE-RB-RXX-XXX模组使用说明(1).pdf` | 36-page user manual, written for the **R120** (CE-RB-R120-161-FBF). Cover says v1.0.0, revision table says 2.0.0 of 2025-11-05 | **protocol and fault handling only, by assumption**: no R68 manual was supplied. Mechanical, encoder, brake and test data are the R120's |

Both drawings call the part a "humanoid joint module" (人形关节模组).

## 1. R68 spec tables (from the two drawings)

| Item | **R68 Lite** | R68 DNN |
|---|---|---|
| Model | CE-RB-R68-101 | CE-RB-R68-101 |
| Harmonic (strain-wave) ratio | 101:1 | 101:1 |
| Output speed | 30 rpm | 30 rpm |
| Rated torque | 33 N·m | 33 N·m |
| Peak torque | 82 N·m | 82 N·m |
| Backlash | **< 30 arcsec** | < 30 arcsec |
| Operating temperature | −20 to 50 °C | 0 to 40 °C |
| Noise | ≤ 60 dB | ≤ 60 dB |
| Life | 5000 h | 5000 h |
| Mass | 740 g | 650 g |
| Velocity-loop (motor) encoder | **21 bit** | 19 bit |
| Position-loop (output) encoder | **21 bit** | 19 bit |
| Bus | CAN | CAN |
| Rated voltage | 48 V | 48 V |
| Rated current | 5.42 A | 5.42 A |
| Peak current | 16.3 A | 16.3 A |
| Back-EMF constant | 5.7 V/krpm | 5.7 V/krpm |
| Torque constant | 0.092 N·m/A (motor side *(inferred)*: × 101 = 9.3 N·m/A ideal at the output; rated 33 N·m / 5.42 A = 6.1 N·m/A) | same |

Not stated for either: torsional stiffness, positioning accuracy or repeatability, motor/output
inertia, brake, encoder type (single- or multi-turn), price.

## 2. R68 Lite mechanics (drawing CE-RB-R68-101-Lite-C0-I)

- Body Ø68 −0.05/0, overall length **73.3 ± 0.2 mm** (output flange 1 mm proud of the body).
- **Through bore Ø18** (贯穿, through), same as the eRob90.
- Output face: 12 × M4 ↓8 on a Ø60 PCD (30° pitch, first hole at 15°), Ø3 +0.015/0 ↓4 dowel hole,
  Ø52 ↓0.5 spigot recess, counterbored Ø4.5 ↓4.
- Rear (fixed) face: 6 × M4 ↓8 on Ø60 (tapped-hole depth 11.8).
- Side: two flats with 4 × Ø3.2 through holes (20 mm and 8/6 mm pitch dimensions, 61.5 across),
  a 32.5 mm section and a 23 mm ring of the housing (motor/gear split, *inferred*).
- Rear connectors: two **DC** (power in/out, XT30-type per the harness), one **CAN** (3-pin GH),
  three **STO** (safe torque off), a 6-pin connector whose mirrored label reads as 继电器 (relay,
  *reading uncertain*, purpose not stated), **USB-C** (debug).

R68 DNN differs: Ø12 bore, 75.6 ± 0.2 mm long, 8 × M4 ↓8 output holes, Ø3 +0.012 ↓6 dowel,
6 × M4 ↓10 rear (13.8 deep), labelled connectors "CAN 通讯" (two), "电源" (power), "USB 调试口".

## 3. Harness "68lite 通讯电源线", 50 cm, part 402000914

| # | Part | Spec | Qty |
|---|---|---|---|
| 1 | Housing | XT30U-F | 1 |
| 2 | Adhesive | K-1688 (at terminal roots) | — |
| 3 | Housing | GH-3Y (JST GH, 3-pin) | 1 |
| 4 | Terminal | GH-T | 3 |
| 5 | PTFE high-temperature wire | 1 mm², red and black | 100 cm |
| 6 | AFS PTFE three-core twisted | 3 × 0.12 mm² | 30 cm |
| 7–8 | Heat-shrink, wire-marker sleeve | — | — |

- Power: XT30U-F, **red +, black −**.
- CAN: GH 3-pin, **red = CAN-H, black = CAN-L, white = GND** (pin 1 white, 2 black, 3 red).
- 500 mm long, 20 mm breakouts; terminal pull-out ≥ 45 N; QC/T 29106-2014.

## 4. Manual (R120): protocol, assumed to be the same family firmware

### 4.1 Physical layer and IDs
- **CAN FD**, default: standard (11-bit) frames, **1 Mbit/s arbitration, 5 Mbit/s data** (bit-rate switch).
- Actuator ID 0x01–0x7F, must be unique before the bus is enabled. 120 Ω termination at both ends if
  needed; common ground between controller and joints; isolation (≥ 2500 Vrms) advised in noisy setups.
- Three ways to command: SDO, the custom "PDO", and the single-actuator control frame.

### 4.2 SDO (CANopen expedited SDO layout)
- Request `0x600 + ID`, response `0x580 + ID`, DLC 8, little-endian data.
- Command bytes: 2F/2B/27/23 write 1/2/3/4 bytes, 60 write ack, 40 read, 4F/4B/47/43 read reply 1/2/3/4 bytes.
- Example frames (ID 1, data hex):

| Purpose | Frame `0x601` data | Notes |
|---|---|---|
| Heartbeat/watchdog time | `2B 17 10 00 D0 07 00 00` | object 0x1017, 2000 ms; 0 disables. Watchdog default **500 ms** |
| Negative soft limit | `23 7D 60 01 C8 80 FF FF` | 0x607D:1, default −32568, range 0 … −180° |
| Positive soft limit | `23 7D 60 02 30 7F 00 00` | 0x607D:2, default 32568, range 0 … 180° |
| CAN FD data-phase rate | `23 40 25 00 01 00 00 00` | 0x2540; this page says 1 = 5M, 2 = 4M, 3 = 2M, 4 = 1M, takes effect after restart. **The dictionary (§4.4) says 1 = 1M, 2 = 500K, 3 = 250K**: contradictory |
| Set ID to 5 | `23 30 25 00 05 00 00 00` | 0x2530 |
| Set output zero | `23 31 25 00 01 00 00 00` | 0x2531 = 1; motor must be **disabled** |
| Enable sequence | `2B 40 60 00 06 00 …`, `… 07 …`, `… 0F …` | controlword 0x6040: ready → **release brake** → operation enabled |
| Profile-position mode | `2F 60 60 00 01 …` | 0x6060 = 1 |
| Accel / decel 2000 rpm/s | `23 83 60 00 D0 07 …`, `23 84 60 00 D0 07 …` | 0x6083/0x6084, 0–10000 |
| Profile velocity 10 rpm | `23 81 60 00 0A 00 …` | 0x6081, 0–30 (**output** rpm in position mode) |
| Target position | `23 7A 60 00 00 40 00 00` | 0x607A, **−32767 … 32768 = −180° … 180°** |
| Start absolute move / stop | `2B 40 60 00 4F 00 …` / `… 0F 00 …` | controlword |
| Velocity mode, 500 rpm | `2F 60 60 00 03 …`, `23 FF 60 00 F4 01 00 00` | 0x60FF, −3000 … 3000 **motor** rpm |
| Current mode, 500 mA | `2F 60 60 00 0A …`, `2B 71 60 00 F4 01 00 00` | 0x6071, ±10000 mA (model dependent) |
| Disable watchdog **and** limit protection | `23 50 26 00 01 00 00 00` | 0x2650, one switch for both |
| Max current | `23 38 25 00 …` | 0x2538 |
| Current-loop P / I | `23 32 25 00 …` / `23 33 25 00 …` | 0x2532 / 0x2533 |
| Position-loop P / I | `23 36 25 00 …` / `23 37 25 00 …` | 0x2536 / 0x2537 |

- SDO reads (reply `0x581`): actual current 0x6078 (mA, int16); **actual position 0x6064 "cnt,
  angle = cnt / 182"** (65536 / 360 = 182.04: 16 bits per turn); temperature 0x2662 (0.1 °C);
  actual speed 0x606C (rpm); fault 0x603F; status 0x6041 (0x21 ready, 0x23 brake released,
  0x27 operation enabled, 0x08 fault); torque sensor 0x2663 in 0.001 N·m (listed beside the
  temperature read; the dictionary gives 0x2663 as coil temperature: another inconsistency).

### 4.3 Custom control frames ("PDO"), CAN FD
**Feedback, joint → host: `0x300 + ID`, DLC 12**, big-endian (high byte first):

| Bytes | Field |
|---|---|
| 0–1 | **Output (load-side) position, int16, −32768 … 32767 = −180° … 180°** |
| 2–3 | Motor-side speed, int16, rpm |
| 4–5 | Iq current, int16, mA |
| 6–7 | Error code (fault table §4.5) |
| 8–9 | Coil temperature, int16, 0.1 °C |
| 10 | Mode: 1 profile position, 2 profile velocity, 3 **CSP**, 4 CSV, 5 current |
| 11 | bit7 enabled, bit6 brake released, bit5 fault, bit4 position reached, bits 3–0 reserved |

**Single-actuator command, host → joint: `0x100 + ID`, DLC 7**, big-endian:

| Bytes | Field |
|---|---|
| 0 | bit7 enable, bit6 brake release, bit5 clear fault, bits 4–1 mode (as above), bit0 reserved |
| 1–2 | Target 1, int16: position modes (1, 3) **output position −180° … 180°**; velocity modes (2, 4) motor rpm; current mode (5) Iq mA |
| 3–4 | Target 2: profile acceleration 0–20000 rpm/s (modes 1, 2) |
| 5–6 | Feed-forward: **reserved** |

**Broadcast command: `0x200`, DLC 64**: eight 7-byte commands in bytes 0–55 (same layout as the
single frame) and the eight target IDs in bytes 56–63.

### 4.4 Object dictionary (manual §4.6)

| Object | Name | Access, type | Range | Notes |
|---|---|---|---|---|
| 0x2530 | CAN node ID | RW u32 | 1–127, default 1 | restart to apply |
| 0x2540 | CAN data-phase bit rate | RW u32 | 1–3, default 1 | 1 = 1M, 2 = 500K, 3 = 250K (contradicts §4.2) |
| 0x2531 | Set output zero | RW u32 | 0–2 | write 1 while disabled |
| 0x2532 / 0x2533 | Current-loop P / I | RW u32 | 0–32768 | |
| 0x2534 / 0x2535 *(index truncated in the PDF, inferred)* | Velocity-loop P / I | RW u32 | 0–32768 | |
| 0x2536 / 0x2537 | Position-loop P / I | RW u32 | 0–32768 | |
| 0x2538 | Peak current | RW u32 | 0–32768 mA | |
| 0x2539 | Flash save code | RW u32 | | 1 save motor parameters, 2 save positive limit, 4 save negative limit |
| 0x2662 | Driver (MOSFET) temperature | RO | −400 … 1500 (0.1 °C) | |
| 0x2663 | Coil temperature | RO | −400 … 1500 (0.1 °C) | |
| 0x603F | Error code | RO u16 | | fault bits §4.5 |
| 0x6040 / 0x6041 | DS402 control / status word | RW / RO u16 | | |
| 0x6071 | Current target | RW i16 | mA | |
| 0x6078 | Current actual | RO i16 | mA | |
| 0x6079 | Bus voltage | RO u32 | 0.1 V | |
| 0x607A | Load (output) position target | RW **"INTEGER32"** | **−32767 … 32768** cnt | 16-bit range in a 32-bit object |

### 4.5 Faults and what the joint does (manual §6)

| Fault | Condition | Joint's action | Reset | Code |
|---|---|---|---|---|
| Absolute encoder fault | encoder data abnormal | safe-torque mode, then **disable + brake** | latched, manual | 0x100 (0x2000 in the 0x603F list) |
| Over/under-voltage | bus outside threshold | **power off + brake** | latched, manual | 0x01 / 0x02 |
| Stall | speed < 20 rpm and current at set peak for 2 s | safe-torque, then **disable + brake** | latched, manual | 0x08 |
| Over-temperature warning | coil > 80 °C 3 s (level 1) … > 95 °C 3 s (level 3) | keeps running | auto when cool | warn 1/2/3 |
| Over-temperature fault | coil > 110 °C | safe-torque, then **disable + brake** | command, below 110 °C | 0x04 |
| CAN error | 5 TX timeouts or 5 hardware error interrupts | re-initialises CAN; **CSP holds position**, velocity → 0 | command | 0x1000 |
| Comms watchdog | no command for **500 ms** | re-initialises CAN; **CSP holds position**, velocity → 0 | auto | — |
| Soft-limit | beyond ±limit (keep ≥ 200 cnt inside ±32767) | **power off + brake**; afterwards only moves away from the limit | command | 0x40 / 0x80 |
| CSP step too large | **new target − last target > 100 cnt** (0.55°) | ignores the command | command | 0x200 |
| Over-speed | above rated | limits speed | — | 0x20 |
| Overload | > 1.5 × rated | keeps running | — | 0x10 |

Other 0x603F bits: 0x0100 encoder comms timeout, 0x0400 power-on electrical-angle init failure,
0x1000 position error too large, 0x0020 also listed as current-sampling error.

### 4.6 R120-only content (recorded for context; **not** R68 data)
- R120: Ø120 × 115.3 mm, 161:1, 180/450 N·m, 11 rpm, backlash ≤ 15″, 4.1 kg, Ø23 bore, 720 W,
  modes IPM / CSP / PP, ≤ 3 A no-load start, −10 to 50 °C, > 10000 h. Motor: 64 pole pairs, 2/6 N·m,
  15/45 A, 3500 rpm, 8.3 V/krpm, 0.137 N·m/A, rotor 1.93 × 10⁻⁴ kg·m².
- Brake (R120): Φ60 × 16, 48 V, 0.094 A, 4.5 W, 512 Ω, > 3 N·m static (motor side), engages
  ≤ 50 ms, releases ≤ 80 ms, ~0.2 kg; static holding only, ≥ 200 emergency stops rated. Powered from
  the joint's own 48 V.
- Encoders (R120): dual **single-turn** absolute, motor 14 bit, output **16 bit**; output encoder
  angle error ≤ ±0.5°, **repeatability ≤ ±0.05° (180″)**; the 65535 → 0 wrap needs handling to avoid
  a jump in the closed loop.
- Loop tests (R120, 1 kHz sampling): current step 35 ms (no load) / 36 ms (225 N·m); speed step 40 /
  76 ms; **position step 45° in 315 / 342 ms**; −3 dB sine tracking: current 10 / 10.8 Hz, speed 7.5 /
  7.4 Hz, position **1.4 / 1.1 Hz at ±45° amplitude** (a large-signal figure: ±45° at 1.4 Hz needs
  ~400°/s, beyond the R120's 66°/s rated speed, so it is speed-limited, not a small-signal bandwidth).
- Power: star wiring for ≥ 800 W modules, daisy chain only for ≤ 200 W (≈ 1.8 % drop per node);
  nothing else on the joints' supply. Regeneration raises the bus; a dump board (48 V in, trips at
  52 V, < 1 ms, 5–100 Ω) or a dump resistor switched on > 53 V, off < 51 V; bus window 44–55 V.
- PC tool: "JuXieDriver" (TestSystem, .NET 8 desktop runtime x86), arm settings page, per-ID
  enable, current/speed/position loops.
