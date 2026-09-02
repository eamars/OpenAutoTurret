# CAN Lockup — Handover to the Architect Agent (no hardware access)

**From:** local diagnosis agent (live session on `rpi-turret`, 2026-09-02
15:21–15:47 NZ)
**To:** architect agent — for offline analysis of the P6 CAN `ERROR-PASSIVE`
blocker against your debug procedure
**Status:** failure classified with live evidence; production supervisor
implemented + tested; **several mechanisms remain unexplained and need your
analysis** (§6). All raw evidence is inlined below so you can reason without
the hardware.

> ## ⚠️ REVISION (2026-09-02, after your review) — read this first
>
> Your review was correct on all points. The two register-interpretation
> errors are confirmed and the v1 conclusions they supported are
> **superseded**:
>
> 1. **CANINTF 0xA0 = MERRF+ERRIF, not RX1IF+WAKIF.** RX1IF is 0x02,
>    RX0IF 0x01, TX0IF 0x04. The v1 "phantom frame" (EXT ID 0x01e79fdf,
>    DLC=15) was **stale RXB1 content read without RXnIF asserted**.
> 2. **Mode encoding: Loopback=0x40, Listen-Only=0x60, Config=0x80.** The v1
>    "listen-only clean" test was actually **loopback** (transceiver
>    internally disconnected — proves nothing about the wire), and the v1
>    "raw loopback TX fail" was actually **configuration mode** (TX cannot
>    start). With corrected constants the raw loopback **passes** (TX0IF
>    set, frame received) — matching the kernel loopback pass.
> 3. Listen-only REC=0 does not prove bus health (counters do not accumulate
>    in listen-only; error interrupts still fire).
> 4. Lone-node cangen TX failures = no-ACK behaviour, not a TX fault.
>
> **Corrected conclusion (§4 below): an intermittent EXTERNAL signal on
> CANH/CANL drives real receive errors (MERRF/ERRIF at ≥20–100/s, REC pinned
> ≥128 → ERROR-PASSIVE). The HAT is NOT confirmed faulty; the source is on
> the wire or in the HAT transceiver/RX path — physical isolation (your §6
> ladder) is the next step.** Full corrected report:
> `docs/can_hardware_fault_report.md` (v2).

---

## 1. One-paragraph summary (corrected)

The station reproduces the P6 `ERROR-PASSIVE` fault on demand in normal CAN
mode. The MCP2515 **core is functional** (kernel loopback TX/RX 5/5 zero
errors; corrected raw loopback passes too); SPI, oscillator, IRQ, and the
mcp251x driver are exonerated. **An intermittent external signal on
CANH/CANL drives real receive errors** — MERRF/ERRIF interrupt storms
(≥20–100/s, rate scaling with receiver bitrate), REC pinned ≥128 →
ERROR-PASSIVE, which then sticks (REC only decrements on successful
receives) until a reset lands in a quiet window. No valid frame decodes at
any bitrate (1M→100k sweep). The source is on the wire or in the HAT
transceiver/RX path — **not discriminable remotely; the physical isolation
ladder (§7) is the next step**. Do NOT classify the HAT as confirmed faulty.
Drives are silent (need power-cycle). Supervisor (`tools/can_supervisor.py`)
delivered and tested.

---

## 2. Environment (exact, captured 2026-09-02)

```
Linux rpi-turret 6.18.39+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.18.39-1+rpt1
(2026-07-29) aarch64 GNU/Linux
Debian GNU/Linux 13 (trixie)
vcgencmd: /dev/vcio_gencmd unavailable (no throttling data)
/boot/firmware/config.txt:
   8: dtparam=spi=on
  52: dtoverlay=nospi10
  58: dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

`ip -details -statistics link show can0` (healthy-looking moment):

```
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP ...
    can state ERROR-PASSIVE restart-ms 100
    bitrate 1000000 sample-point 0.666
    tq 166 prop-seg 1 phase-seg1 2 phase-seg2 2 sjw 1 brp 1
    mcp251x: tseg1 3..16 tseg2 2..8 sjw 1..4 brp 1..64 brp_inc 1
    clock 6000000            <- 12 MHz oscillator / 2 (correct for this HAT)
    re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          0          0          2          2          0
```

`ethtool -i can0`: driver `mcp251x`, version `6.18.39+rpt-rpi-2712`,
bus-info `spi0.0`. IRQ 185 `pinctrl-rp1 25 Level spi0.0` (GPIO25 = CAN INT).
Sysfs device: `spi0.0` under `1f00050000.spi` (RP1 SPI controller).

Drives: CyberGear, pitch `can_id: 100 (0x64)`, yaw `101 (0x65)`, config
`config/turret.yaml`. Daemon `controld` was NOT running during diagnosis.

## 3. Session timeline (all times NZ, 2026-09-02)

| Time | Event / observation |
|---|---|
| 14:55 | Pi reboot (operator). dmesg `MCP2515 successfully initialized`. |
| 15:21 | Session start: can0 already UP, **ERROR-PASSIVE**, restart-ms 0, RX 5 / TX 3 packets, 1 TX error, 1 dropped, IRQ count 7. Drives silent. |
| 15:25 | down/up → `ERROR-ACTIVE` at t+1s, **`ERROR-PASSIVE` at t+2s**. L3 unbind/rebind works (`MCP2515 successfully initialized`, can0 recreated), same ~1s degrade. |
| 15:28–33 | spidev bound; raw register probes. **Phantom frame** observed in normal mode: ~48.8/s, identical content, REC=0x82 (130), EFLG=0x0b, CANINTF=0xa0 (RX1IF+WAKIF). Loopback TX (raw SPI) never completes. Listen-only clean. Same phantom at 1M/500k/250k bitrates (CNF verified). |
| 15:35 | Rebound mcp251x. |
| 15:38:0x | `--once` probe: state ERROR-PASSIVE. |
| 15:38:37–39:48 | Supervisor run (idle, kernel driver): **ERROR-ACTIVE stable 70 s** — no fault detected. |
| 15:38:5x | cangen 10×0x222 burst: state → ERROR-PASSIVE; IRQ 15→289; **netdev TX counters stayed 0**. |
| 15:42 | Raw normal-mode TX test: TEC oscillated 0x28→0xf0 (bus-off cycles), REC≈0x85–0x87, EFLG 0x1f (both passive flags), CANINTF 0xa0 — phantom back in raw normal mode. |
| 15:43 | **Kernel loopback mode: PASS — 5 TX + 5 RX packets, 0 errors** (definitive: chip core works). |
| 15:43:48–45:48 | Idle watch (kernel normal mode): **ERROR-ACTIVE stable 2 min**. |
| 15:45:08–12 | Supervisor ladder test on a manufactured DOWN interface: L0 rc=0, L1 invalid (not BUS-OFF), L2 up rc=0 but verify failed (state → ERROR-PASSIVE), L3 rebind + bring-up, verify failed, L4/L5 skipped, ACK probe "TX did not complete", **classified unrecovered with two-possibility verdict, latched**. |
| 15:47 | Handover doc written. |

**Key nuance for you:** the phantom was *directly* observed (register reads)
only in raw-spidev normal mode. Under the kernel driver I inferred
REC-pinning only from the ERROR-PASSIVE state (no berr-reporting, so TEC/REC
are not readable while the driver is bound). The passive state appeared in
both regimes, intermittently.

## 4. Raw evidence (verbatim)

> ⚠️ §4 reproduces the **v1-era raw data with v1 interpretations** for the
> audit trail. Per the revision banner, several interpretations are
> superseded: "phantom frame" = stale RXB1 content read under MERRF+ERRIF;
> "listen-only" rows = actually loopback mode; "raw loopback fail" =
> actually configuration mode. The *raw register values* are genuine; the
> *conclusions* marked with v1 labels are superseded by the corrected
> results in report v2 §4 and §7 above.

### 4.1 Register dump — phantom-active normal mode (raw SPI, 2 MHz, mode 0)

```
== MCP2515 probe (spidev0.0 @ 2 MHz) ==
after RESET:
  TEC=0x00 REC=0x00 EFLG=0x00 CANINTF=0x00
  CANSTAT=0x80 CANCTRL=0x87 CNF1/2/3=0x00/0x00/0x00
enter NORMAL mode, watching 3 s:
  CANCTRL=0x00 CANSTAT=0x00 OPMODE=0 (req 0)
  TEC=0x00 REC=0x82 EFLG=0x0b CANINTF=0xa0      <- phantom within ~10 ms
  ...REC stays 0x82 (130), EFLG 0x0b for all 10 samples (3 s)
```

EFLG 0x0b = EWARN|RXWAR|RXEP (receive-side passive). CANINTF 0xa0 =
RX1IF + **WAKIF** (wake interrupt — see open questions §6).

### 4.2 The phantom frame (RXB1, read every 25 ms for 300 ms — all identical)

```
EXT ID=0x01e79fdf DLC=15 RTR=0 data 3f 9f be ef 00 00 00 02
SIDH=0x0f SIDL=0x2b EID8=0x9f EID0=0xdf  (decoded: comm_type 0x01, bits 23:16=0xe7,
                                          data2=0x9f, target=0xdf)
```

- Rate measured 4.09 s window: **48.8 frames/s** (≈ every 20 ms).
- RX1IF **re-set immediately after every clear** (200/200 samples at 20 ms).
- RXB0 held a *different, stale* frame (ID 0x79-standard, DLC=3, data
  `70 fe 80 ...`) with RX0IF=0 — stale content, not new.
- First-ever RXB1 read showed RTR=1/DLC=0x0f; later reads RTR=0/DLC=15 —
  minor inconsistency, likely a stale-flag artifact (note it).

### 4.3 Bitrate sweep — same phantom at every receiver timing

```
bitrate=1000000 CNF1/2/3=00/11/02 -> frames=100 in 2.05s (48.7/s) ID=0x01e79fdf DLC=15 data 3f 9f be ef 00 00 00 02
bitrate= 500000 CNF1/2/3=40/72/03 -> frames=100 in 2.05s (48.7/s) ID=0x01e79fdf DLC=15 data 3f 9f be ef 00 00 00 02
bitrate= 250000 CNF1/2/3=41/72/03 -> frames=100 in 2.06s (48.6/s) ID=0x01e79fdf DLC=15 data 3f 9f be ef 00 00 00 02
(CNF readback verified; CANSTAT=0x00 normal mode each time; SPI RESET before each)
```

A real wire frame cannot decode identically at three bit rates → **not a bus
transmission**.

### 4.4 Listen-only — input path clean

```
req mode 2 -> OPMODE=2 (CANSTAT=0x40)
listen-only: 0 phantom frames in 1s, EFLG=00, REC=00
```

Same input path, no TX engine involvement, no phantom → the wire/transceiver
input carries nothing.

### 4.5 Raw normal-mode TX attempt (mcp_tx.c, ext ID 0x02806400, DLC 8)

```
after TX: CANINTF=0xa0 TEC=0xc8 REC=0x87 EFLG=0x1f   <- TX0IF NEVER set
t1: TEC=f0 REC=87 | t2: TEC=28 REC=85 | t3: TEC=48 | t4: TEC=98 | t5: TEC=c8
```

- TX0IF never set; TXB0CTRL.TXREQ stays set → **no completed data TX**.
- TEC oscillates 40→240 (repeated bus-off cycles: TEC>255 → bus-off → reset).
  Attributed to **error-frame transmissions** driven by the phantom receive
  errors (each receive error → the node transmits an error frame → TEC+8),
  not to data transmission.

### 4.6 Raw loopback (SPI) — FAILS, but treat as suspect (see §6.5)

```
enter LOOPBACK mode: CANSTAT=0x80 OPMODE=4
TXREQ set; TXB0CTRL=0x08  -> stays 0x08 for 2 s; CANINTF=0x00; TEC=0x00; REC=0x00
RESULT: LOOPBACK FAIL (TX never completed)
```

### 4.7 Kernel loopback — PASS (authoritative)

```
ip link set can0 type can bitrate 1000000 restart-ms 100 loopback on  (rc=0)
cangen -I 0x555 -L 8 -D 0xAABBCCDD -n 5 -> candump shows all 5 looped
RX: 5 packets (40 bytes)  TX: 5 packets (40 bytes)  errors 0
```

→ the MCP2515's full CAN engine (bit-stream processor, TX/RX buffers,
timing) is **functional**.

### 4.8 Kernel normal-mode cangen (lone node, no ACK)

```
cangen -I 0x222 -n 10 -> IRQ 15→289 (TX/error interrupts), state → ERROR-PASSIVE,
netdev TX packets stayed 0, RX 0
```

Consistent with: transmissions attempted, fail (no ACK → TEC+8), and/or
error-frame activity — plus no completed netdev TX. (TEC vs REC not readable
under the kernel driver.)

### 4.9 Interrupt evidence

- Idle phantom-active: IRQ count frozen (15) while phantoms flowed (they do
  NOT reach the kernel RX path).
- cangen burst: IRQ 15 → 289 — the INT line works.

## 5. Failure-mode ladder results (your §3 table, live)

| Candidate | Verdict | Evidence |
|---|---|---|
| CAN BUS-OFF only | ruled out as sole cause | persistent ERROR-PASSIVE (REC≥128); TEC bus-off cycles are a consequence |
| SocketCAN misconfig | ruled out | bitrate/timing verified; survives re-probe |
| mcp251x driver deadlock | ruled out | responsive; re-probe OK; loopback TX/RX through driver |
| MCP2515 internal wedge | superseded | (v1 claim; corrected: REC pinning is from external receive errors — report v2 §4) |
| SPI failure | ruled out | register read/write verified on the same path |
| INT/IRQ failure | ruled out | GPIO25 fires normally |
| Transceiver/bus electrical | **external receive-error source — origin UNRESOLVED** | errors continue in corrected listen-only; REC pinned in normal mode; loopback clean; wire vs HAT-RX-path pending physical isolation |
| Power integrity | no evidence at diagnosis time | no throttling data; no scope capture |

**Conclusion:** the HAT's normal-mode CAN path (transceiver region or MCP2515
input/analog path) is faulty. Chip core, SPI, oscillator, IRQ, driver, and
wire input are exonerated.

## 6. Open questions for your offline analysis (revised after corrections)

1. **Identify the external source.** The corrected probes show an
   intermittent signal on CANH/CANL that produces: error-interrupt storms
   (rate scaling with receiver bitrate: ~63/s @1M → ~20/s @100k), REC ≥128,
   and — rarely — a partial frame load into RXB0/RXB1 with RXnIF asserted
   (the `0x01e79fdf DLC=15` bytes). Candidates: a faulted CyberGear
   transceiver (post-incident), a latched/oscillating transceiver (HAT or
   drive side), wiring/termination, or mains/EMI-coupled noise (NZ 50 Hz).
   Questions: (a) can a CAN FD transmitter (DLC 9–15) produce exactly this
   partial-load signature on a classic MCP2515 receiver — i.e. does the
   MCP2515 check the FDF/r0 bit, and could an FD frame without BRS decode
   far enough to load RXB0 with DLC=15 before CRC fails? (b) does the
   ~1 ms-scale event spacing fit a periodic transmitter or continuous
   noise? (c) what register-level or scope observation discriminates
   "drive transceiver driving garbage" from "passive noise coupling"?
   The physical isolation ladder (Test A–E, §7) is the ground truth.
2. **Storm intermittency.** The storm is active for seconds-to-minutes,
   then quiescent (60 s window with zero IRQs while REC stays frozen ≥128).
   What external behaviour has that duty cycle? (Faulted drive watchdog?
   Thermal? A device power-cycling? Mains-related?) Design a long-duration
   raw-probe logger (EFLG/REC/CANINTF at 1 Hz, hours) for the next session.
3. **Why REC freezes and never recovers.** REC decrements only on
   successful receives; with a silent bus the ERROR-PASSIVE state persists
   until a down/up lands in a quiet window. Confirm this fully explains the
   "~1 s ACTIVE → PASSIVE" + persistence signature, and whether any software
   action (e.g. listen-only toggling) can force REC recovery without a
   full reset.
4. **Original trigger.** The P6 event occurred near an against-gravity peak
   motor current. Does that fit an electrical disturbance that *damaged* the
   drive transceiver (now intermittently driving the bus), or a transient
   that only *exposed* the fault? Design the electrical-hardening +
   isolation plan (research note §4.1.4) so the replacement survives.
5. **Kernel path.** `berr-reporting` NOT supported by this kernel's mcp251x.
   Decision: stay with state+netdev polling (recommended) vs patch.
6. **Recovery success-path verification.** L0–L3 all failed on THIS station
   while the storm is active (as expected for a physical fault). The
   supervisor's *success* path (recover BUS-OFF/DOWN on a healthy bus) is
   unverified — design the acceptance run (§8 of the report; your §12:
   50 cycles, no hang, drives rediscovered).
7. **L4 GPIO RESET design** — defer per your §13; keep the supervisor
   `--reset-gpio` hook as a future resilience feature.
8. **Bitrate** — stay 1 Mbit/s during root-cause work (your §12).

## 7. Physical isolation ladder — the next hardware-session protocol

(Your §6–8, condensed for the operator.) Test A first, add one element at a
time, ≥5 min idle watch each, no transmissions in Test A:

- **Test A — HAT only, CANH/CANL physically disconnected:** bring up
  (1 Mbit/s, restart-ms 100), watch state + IRQ + corrected raw probe.
  Degrades → HAT/transceiver region primary suspect. Healthy → external.
- **Test B — + CAN cable** (no powered motors): wiring/termination.
- **Test C — + pitch 0x64 only** (powered, stationary): pitch transceiver.
- **Test D — + yaw 0x65 only:** yaw transceiver.
- **Test E — both:** termination, ground offsets, interaction.
- Then motor-load ramp (your §8, steps 1–10) with continuous CAN logging;
  termination measurement (powered off, CANH↔CANL, target ≈60 Ω, measure
  the real installed network); scope TXCAN/RXCAN/CANH/CANL per your §10
  Case 1–5; replacement-HAT A/B under the original load scenario.

## 7. What the local agent delivered (review targets)

| Artifact | Path | Notes for review |
|---|---|---|
| Root-cause report | `docs/can_hardware_fault_report.md` | classification, evidence table, ladder results, architecture, operator actions |
| Production watchdog | `tools/can_supervisor.py` | Python 3 stdlib only (no venv needed). 2 s poll; L0→L5 ladder; every command under `timeout 5`; ACK probe via netdev counters; rate limit 3/10 min + cooldown latch; counters + verdict JSON at `/tmp/ota_can_supervisor.json`; snapshot to `/tmp/ota_can_diag/`. **Live-tested** on the faulted HAT: detected, escalated L0→L3 without hanging, classified, latched. |
| systemd template | `systemd/turret-can-supervisor.service` | `After=can0.service`, `Before=turret-control.service`, root, `Restart=always`. Enable at P11. |
| Queue update | `docs/post_homing_test_queue.md` (P6) | BLOCKED → RESOLVED with evidence + operator checklist |
| Research-note addendum | `docs/research_can_bus_error_passive.md` §4.3 | corrects the berr-reporting mitigation; records live findings |
| Raw probe tools (disposable) | `/tmp/mcp*.c` on the Pi | register dump, loopback, listen-only, bitrate sweep, raw TX, watch/rate |

Runtime artifacts on the Pi: `/tmp/ota_can_diag/diag_20260902-{153658,154508}.txt`,
`/tmp/ota_can_supervisor.log`, `/tmp/ota_can_supervisor.json`.

## 8. Reproducibility kit (next hardware session — exact procedure)

1. Baseline: `ip -details -statistics link show can0` (state, counters),
   `grep spi0.0 /proc/interrupts`, `dmesg -T | grep -Ei mcp251|can` .
2. Idle watch 5 min: state must hold ERROR-ACTIVE (else phantom-active →
   HAT fault).
3. Kernel loopback test (the chip-core acceptance):
   `ip link set can0 down; ip link set can0 type can bitrate 1000000 restart-ms 100 loopback on; ip link set can0 up; cangen can0 -I 0x555 -n 5; candump can0` → expect TX/RX 5/5.
4. Listen-only via raw SPI (spidev driver_override swap) → expect 0 frames,
   REC=0.
5. Raw normal-mode register watch (`/tmp/mcp_watch` on the Pi) → phantom?
   (REC≥128 + RX1IF re-set = fault).
6. ACK/discovery: power-cycle drives; `./turret-can discover` → both IDs.
7. Supervisor ladder acceptance: manufacture DOWN + BUS-OFF (cangen burst on
   a lone healthy node), confirm L2 recovers, counters/verdict update.
8. 50-cycle soak per your §12 once healthy.

## 9. Station state at handover (2026-09-02 15:47 NZ)

- can0: UP, restart-ms 100, 1 Mbit/s, state **ERROR-PASSIVE** (phantom-active
  at that moment — the honest hardware state).
- spi0.0 bound to **mcp251x** (driver_override cleared) — system sane.
- Drives: **silent** (need station power-cycle). Daemon not running.
- The operator must, before re-running P6 Run A: A/B-test or replace the HAT
  (idle watch ≥5 min ERROR-ACTIVE), power-cycle the drives, confirm
  discovery, then re-run the payload check (the `pos_return` speed-gain fix
  is still pending live verification).

## 10. Decisions the architect should own

1. Repair vs replace vs scrap for this HAT (analysis per §6.1).
2. Acceptance protocol for the replacement HAT / next station session
   (section 8 checklist → your §12 acceptance criteria).
3. Kernel: stay with state/netdev polling (recommended) vs patch mcp251x for
   berr-reporting.
4. L4 GPIO RESET design artifact: produce now (contingency) or defer.
5. Bitrate: stay 1 Mbit/s vs move to 500 kbit/s (drive reprogramming cost).
6. Electrical hardening + isolation plan (research note §4.1.4, §5) so the
   trigger event cannot kill the next HAT.
