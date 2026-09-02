# CAN Bus `ERROR-PASSIVE` — Root-Cause Report (P6 blocker) — v2, corrected

**Status: CAN normal-mode ERROR-PASSIVE fault reproduced. Root cause
remains unresolved.** HAT/transceiver damage, MCP2515 RX-path damage,
external wiring, CyberGear transceiver behaviour, and motor-induced
electrical disturbance remain candidates. **Do not classify the HAT as a
confirmed hardware failure.**

This v2 supersedes v1 (2026-09-02). v1 contained two register-interpretation
errors in the raw MCP2515 probes that invalidated its central "internally
fabricated phantom frame" conclusion; the corrected probes point instead at
an **intermittent external receive-error source on the CAN wire**.
Corrections are documented in §3; all earlier reports/handovers that repeat
the v1 "phantom frame" claim should be read as superseded.

---

## 1. Environment snapshot (2026-09-02, unchanged from v1)

```
Linux rpi-turret 6.18.39+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.18.39-1+rpt1
(2026-07-29) aarch64 GNU/Linux      Debian GNU/Linux 13 (trixie)
/boot/firmware/config.txt:
   8: dtparam=spi=on     52: dtoverlay=nospi10
  58: dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
can0: driver mcp251x, spi0.0, 1 Mbit/s, 6 tq (tq 166, prop 1, ps1 2, ps2 2, sjw 1),
      sample-point 0.666, clock 6 MHz (= 12 MHz oscillator / 2)
IRQ 185: pinctrl-rp1 25 Level spi0.0  (GPIO25 = CAN INT)
Drives: CyberGear pitch id 0x64, yaw id 0x65 (config/turret.yaml)
```

## 2. Solid findings (unchanged by the correction)

| # | Finding | Evidence |
|---|---|---|
| 1 | mcp251x driver responsive; SPI works; register read/write works; GPIO25 IRQ works | re-probe succeeds; register dumps consistent; IRQ fires on activity |
| 2 | **MCP2515 digital core fully functional** | **kernel loopback: 5 TX + 5 RX packets, 0 errors**; corrected raw loopback (REQOP=010): TX0IF set, frame received |
| 3 | Oscillator + bit timing functional | loopback TX/RX works; mode transitions latch |
| 4 | CyberGear IDs unique | config: pitch 0x64, yaw 0x65 |
| 5 | **System repeatedly enters ERROR-PASSIVE in normal mode** | reproduced many times; survives down/up, re-probe, Pi reboot |
| 6 | **An intermittent external signal on CANH/CANL drives receive errors** | see §4 |
| 7 | Drives silent (no discovery response, no ACK) | `turret-can discover` timeouts; ACK probe no-ACK |
| 8 | `berr-reporting` NOT supported by this kernel's mcp251x | `ip link set can0 type can berr-reporting on` → "not supported" |

## 3. Corrections to v1 (architect review, 2026-09-02)

### 3.1 CANINTF bit mapping (v1 misread 0x80 as RX1IF)

Correct MCP2515 CANINTF (0x2C):
```
bit 7 0x80 MERRF   bit 6 0x40 WAKIF   bit 5 0x20 ERRIF
bit 4 0x10 TX2IF   bit 3 0x08 TX1IF   bit 2 0x04 TX0IF
bit 1 0x02 RX1IF   bit 0 0x01 RX0IF
```
v1 read `CANINTF=0xA0` as "RX1IF + WAKIF". Correct: **0xA0 = MERRF + ERRIF**
(message-error + error interrupts). RX1IF is bit 1 (0x02), RX0IF bit 0 (0x01).
The v1 probes therefore read RXB1/RXB0 **without RXnIF asserted** — the
"phantom frame" content (`EXT ID 0x01e79fdf DLC=15 data 3f 9f be ef 00 00
00 02`) was **stale buffer content**, not a newly received frame.

### 3.2 Operating-mode encoding (v1 swapped loopback/listen-only)

Correct OPMOD/REQOP (bits 7:5):
```
000 Normal        0x00      001 Sleep       0x20
010 Loopback      0x40      011 Listen-Only 0x60
100 Configuration 0x80
```
v1 called `CANSTAT=0x40` (Loopback) "listen-only" and `CANSTAT=0x80`
(Configuration) "loopback". Consequences:
- The v1 "listen-only is clean" result was actually **loopback mode**
  (transceiver internally disconnected) — it says nothing about the wire.
- The v1 "raw loopback TX failed" result was actually **configuration mode**
  (TX never starts) — invalid; with corrected constants the raw loopback
  **passes** (TX0IF set, frame received), matching the kernel loopback pass.

### 3.3 Listen-only REC=0 does not prove bus health

The MCP2515 does not accumulate normal error counters in Listen-Only mode.
v1's "listen-only REC=0 → wire clean" was invalid on two counts (wrong mode
and wrong counter semantics). Corrected listen-only (REQOP=0x60): error
interrupts (MERRF/ERRIF) continue firing while REC stays 0 — consistent
with an external source that the counters simply don't track in listen-only.

### 3.4 Lone-node TX failures are not evidence of a fault

`cangen` on a lone node (no ACK) → ACK error → TEC+8 → retry → bus-off. The
v1 cangen/raw-TX "TX never completes" observations are explained by **no
other node on the bus**, not by a broken TX engine (which the loopback tests
contradict).

## 4. Corrected evidence — an intermittent external receive-error source

Corrected raw probe (spidev bound to spi0.0; all readbacks re-verified):

| Test | CANSTAT | CANINTF | TEC | REC | EFLG | RX0IF | RX1IF | Result |
|---|---|---:|---:|---:|---:|---|---|---|
| After RESET | 0x80 (Config) | 0x00 | 0x00 | 0x00 | 0x00 | 0 | 0 | clean |
| Configuration | 0x80 (Config) | 0x00 | 0x00 | 0x00 | 0x00 | 0 | 0 | clean |
| Loopback (0x40) + TX | 0x40 (Loopback) | 0x05 (TX0IF+RX0IF) | 0x00 | 0x00 | 0x00 | 1 | 0 | **TX completed, frame looped — core OK** |
| Listen-Only (0x60), 3 s | 0x60 (Listen-Only) | error ints 30/30 samples | 0x00 | 0x00 | 0x40 (RX0OVR) | 1 | 0 | **external frames arriving; counters not tracked** |
| Normal (0x00), 4 s | 0x00 (Normal) | MERRF/ERRIF ~66–108/s | 0x00 | **0x81–0x82 (≥128 → PASSIVE)** | 1 (rare) | 0 | **receive-error storm; REC pinned → ERROR-PASSIVE** |

- **Error-event rate scales with receiver bitrate** (corrected sweep):
  63/s @1 Mbit/s → 39/s @500k → 23/s @250k → 20/s @100k; REC reached
  ≈0x81 (129) at every rate; **no valid frame decoded at any rate**
  (1M/800k/666k/500k/400k/333k/250k/200k/125k/100k). A bitrate-mismatched or
  corrupt external signal errors at any receiver timing — the v1
  "bitrate-independent content = internal fabrication" argument is void
  (it was stale bytes; the *error events* themselves are bitrate-independent,
  which is expected for an external garbage source).
- **Kernel confirmation:** interface up → ERROR-PASSIVE within ~1 s; during a
  quiet minute the IRQ count stayed frozen (no new error interrupts) while
  the state remained passive — **REC freezes at ≥128** (decrements only on
  successful receives; with no traffic it never recovers). A down/up during a
  quiet window recovers ERROR-ACTIVE; during an active window it re-degrades
  in ~1 s. **The intermittency explains the "recovers then degrades"
  signature.**
- Kernel **listen-only** mode (driver-supported): interface holds
  ERROR-ACTIVE (counters not accumulated), consistent with the above.

**Interpretation:** an intermittent source on CANH/CANL (or in the HAT's
transceiver RX path) delivers garbage that the MCP2515 cannot decode —
receive errors → REC ≥ 128 → ERROR-PASSIVE; the state then sticks until a
reset lands in a quiet window. The source is **external to the MCP2515
digital core** (loopback is clean). Whether it is on the wire (faulty
CyberGear transceiver / wiring / EMI) or inside the HAT's transceiver/RX
region **cannot be discriminated remotely** — it requires the physical
isolation ladder (§6).

## 5. Revised root-cause ranking (architect §17)

| Candidate | Verdict |
|---|---|
| Linux mcp251x deadlock | largely ruled out |
| SPI failure | largely ruled out |
| Oscillator failure | unlikely |
| GPIO25/IRQ failure | largely ruled out |
| Duplicate CyberGear IDs | ruled out |
| MCP2515 digital core failure | unlikely (loopback passes) |
| SN65HVD230 / HAT receive-path damage | **plausible — unconfirmed** |
| MCP2515 RXCAN/input damage | **plausible — unconfirmed** |
| CAN wiring / termination | **open** |
| Pitch CyberGear transceiver | **open** |
| Yaw CyberGear transceiver | **open** |
| Multi-motor interaction | **open** |
| Motor-load EMI / ground bounce | **open and important** (original trigger) |
| Power integrity | **open** |
| ~49 Hz internally fabricated frame | **unsupported — superseded** (§3.1) |

## 6. Immediate next step — physical isolation ladder (operator required)

The highest-information experiment needs hands at the station. Do **not**
replace/scrap the HAT before this:

- **Test A — HAT with CANH/CANL physically disconnected:** bring can0 up
  (1 Mbit/s, restart-ms 100), transmit nothing, watch ≥5 min
  (`watch -n 1 'ip -details -statistics link show can0'` + `grep spi0.0
  /proc/interrupts` + corrected raw probe). Expected healthy: ERROR-ACTIVE,
  TEC=REC=0, no error interrupts. If it degrades with nothing connected →
  HAT/transceiver region primary suspect. If healthy → fault is external.
- **Test B — CAN cable only** (no powered motors) → wiring/termination.
- **Test C/D — pitch only / yaw only** (powered, stationary) → per-drive
  transceiver.
- **Test E — both motors** → termination, ground offsets, interaction.
- Then the **motor-load test** (low→high, against-gravity pitch, reversals)
  with continuous CAN-state logging, correlating first error with motor
  current / supply / axis.
- **Termination:** powered off, measure CANH↔CANL resistance (target ≈60 Ω
  with two 120 Ω ends; measure the real installed network: HAT only, +pitch,
  +yaw, +both).
- **Scope (if available):** TXCAN / RXCAN / CANH / CANL; per the architect's
  Case 1–5 interpretation table.
- **Replacement HAT A/B test** (same Pi/cables/motors/PSU/software, only the
  HAT changes) — still recommended, but only *after* the isolation ladder,
  and repeat the original high-load scenario.
- Stay at **1 Mbit/s** during root-cause work (500 kbit/s changes drive +
  Pi + noise-margin configs and confounds the analysis).

## 7. Recovery architecture (unchanged, still delivered)

`tools/can_supervisor.py` + `systemd/turret-can-supervisor.service`:
bounded L0–L5 ladder (every command under `timeout 5`), ACK probe
(distinguishes "no node on the bus" from "controller TX failing"), rate
limit 3/10 min + cooldown latch (no reset loops), counters + verdict JSON
(`/tmp/ota_can_supervisor.json`), diagnostics snapshot
(`/tmp/ota_can_diag/`). Live-tested on the faulted station. No kernel
patches, no bitrate change, no GPIO RESET mod (deferred — SPI reset and
re-probe are proven; a RESET line cannot clear an external physical fault).
See the handover doc for review targets.

## 8. Acceptance criteria (healthy system gate, before resuming queue)

- Idle ≥30 min: ERROR-ACTIVE continuously, no unexplained errors, both
  drives discoverable.
- Motion soak (yaw/pitch/simultaneous/reversals/against-gravity/max load):
  no persistent ERROR-PASSIVE, no unrecovered BUS-OFF, no discovery loss,
  no supervisor latch.
- Recovery: ≥50/50 controlled cycles (interface DOWN, software restart,
  driver rebind, BUS-OFF) with no manual intervention and both motors
  rediscovered.
- Final verdict must name A/B/C/D/E (HAT defective / specific CyberGear /
  wiring / electrical / unresolved) — not "probably hardware".

## 9. Files

- `docs/can_handover_architect.md` — raw evidence, session timeline,
  open questions, reproducibility kit.
- `docs/post_homing_test_queue.md` (P6) and
  `docs/research_can_bus_error_passive.md` (§4.3) — updated.
- `tools/can_supervisor.py`, `systemd/turret-can-supervisor.service`.
- Corrected raw-probe tools (disposable, on the Pi): `/tmp/mcp_correct.c`
  (corrected constants + mode table + error-rate/real-frame capture),
  `/tmp/mcp_sweep.c`, `/tmp/mcp_fine.c` (bitrate sweeps).
- Station runtime artifacts: `/tmp/ota_can_diag/`,
  `/tmp/ota_can_supervisor.{log,json}`.
