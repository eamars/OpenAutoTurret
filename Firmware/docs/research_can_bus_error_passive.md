# CAN Bus `ERROR-PASSIVE` Incident — Research Note (P6 blocker)

**Incident:** 2026-09-02, P6 payload check Run A — `can0` (MCP2515) entered a
persistent `ERROR-PASSIVE` mid-run; drives fully silent; all software resets
exhausted (see `docs/post_homing_test_queue.md`, P6 BLOCKED item).
**This note:** online research into the root cause and mitigations, recorded
2026-09-02.

## Hardware context (from this repo's records)

- Adapter: Waveshare RS485 CAN HAT, MCP2515 12 MHz variant on SPI
  (`docs/RS485_CAN_HAT_SETUP.md`; `dtoverlay=mcp2515-can0,oscillator=12000000,...`).
- Host: Raspberry Pi 5, kernel 6.18.39+rpt-rpi-2712, `mcp251x` driver.
- Bus: 1 Mbit/s (CyberGear default), two CyberGear drives.
- Bring-up: `systemd/can0.service` — `ip link set can0 type can bitrate 1000000
  restart-ms 100 && ip link set up can0`. No `berr-reporting`.
- Daemon: `controld` opens `can0` via SocketCAN; CAN state is read **once, at
  open** (`control/src/can/socketcan_bus.cpp` `configure_interface()` → cached
  `can_state_`); no runtime bus-state monitoring. The RX filter already accepts
  `CAN_ERR_FLAG` error frames (`default_filters()`), but with `berr-reporting`
  off the kernel never delivers any.

## 1. What `ERROR-PASSIVE` means (protocol level)

Bosch CAN 2.0 error model (CAN Wiki — [CAN Bus-Off condition](http://can-wiki.info/doku.php?id=can_bus_off&rev=1363860418&mddo=print)):

| State | Condition |
|---|---|
| Error-Active | TEC ≤ 127 and REC ≤ 127 |
| Error-Warning | ≥ 96 on either counter (flag only) |
| **Error-Passive** | **TEC ≥ 128 or REC ≥ 128** |
| Bus-Off | TEC > 255 (transmitter only) |

Counter arithmetic: error-free receive → REC −1; successful transmit → TEC −1;
transmit error → TEC +8; receive error → REC +1 (dominant-bit error +8,
stuffing error +32).

Recovery rules (the crux of this incident):

- **Error-passive is self-driven recovery only** — the counters must fall back
  below 128. For the MCP2515, TEC decrements only on *successful
  transmissions* (Vector CAN-list: [MCP2515 — Recovering from
  Error-Passive state](https://canlist.vector-informatik.narkive.com/fuk8y5jZ/mcp2515-recovering-from-error-passive-state)).
- **Bus-off recovery needs 128 consecutive runs of 11 clean recessive bits**
  (CAN 2.0B), or an external/user reset (ISO 11898-1). A node on a still-
  corrupted bus cannot see those clean bits and stays off.
- The kernel's `restart-ms` **only fires on BUS-OFF** — it does nothing for a
  node sitting in error-passive (mcp251x reports the state via netlink, e.g.
  [kernel mcp251x.c](https://android.googlesource.com/kernel/common.git/+/9a4e328eb2bfa23b160558cff96e17ffa65ea5cf/drivers/net/can/mcp251x.c#3);
  [restart-ms handling discussion](http://mail.spinics.net/lists/linux-can/msg02650.html)).
  This is why "auto-restart re-enabled" changed nothing in the incident.

A real-world confirmation of the exact 120→127 (warn) → 128 (passive) → 95
(active) ladder, on an RPi MCP251x HAT with a motor drive: [Seeed-Studio
pi-hats issue #18](https://github.com/Seeed-Studio/pi-hats/issues/18) — heavy
traffic produced `protocol-violation{{tx-recessive-bit-error,error-on-tx}}`
storms with `error-counter-tx-rx{{128}{0}}` and "eventually, bus off motor".

## 2. Interpretation of the observed signature

- **~1 s ERROR-ACTIVE → ERROR-PASSIVE after every reset** = a *continuous*
  bus-level fault. Any reset zeroes the MCP2515 counters (→ ACTIVE); a clean
  bus would keep them low. Re-reaching 128 within ~1 s at 1 Mbit/s requires
  hundreds of bit errors per second — a persistent short, a latched
  transmitter, or a persistent common-mode/ground problem. Not a transient.
- **Silent drives** = the same fault from the other side: their TECs crossed
  255 (bus-off) and/or their firmware latched a CAN fault (de-energize +
  silence). CyberGear drives do not self-recover from that while the bus stays
  corrupted (bus-off recovery needs 128×11 clean recessive bits), and the
  ecosystem's consistent remedy for a silent CyberGear is a drive
  power-cycle/restart ([Xiaomi_CyberGear_Arduino issue
  #1](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino/issues/1):
  "after restarting the drive it came back" at the default ID).
- **Exonerated:** the MCP2515 chip, its SPI, and the oscillator — the
  `unbind`/`rebind` re-probe succeeded and the controller reaches ERROR-ACTIVE
  (it can arbitrate, transmit, receive). The fault is *outside* the HAT.
- **Exonerated:** daemon software as trigger — the P4 flood test showed the
  bus tolerates saturation with clean recovery.

## 3. Ranked physical root-cause candidates

1. **Load-induced power-rail sag / ground bounce** during the against-gravity
   move — the fault hit exactly at peak motor current / peak PWM switching. A
   shared supply path or ground return between Pi and drives shifts the CAN
   differential reference or drops the transceiver supply below common-mode
   margin → bit errors on *every* node. Fits the trigger correlation and the
   persistent-while-loaded behavior.
2. **EMI coupling** from the motor power cable / drive inverter into the CAN
   pair (routing, shield grounding, missing common-mode suppression) — worst
   at peak motor current.
3. **Latched CAN transceiver** (drive-side or HAT-side) after an overvoltage/
   ESD event — a latched transceiver continuously drives the line, producing
   exactly this continuous-error signature, and is cleared **only** by
   power-cycling the affected device. Matches the queue's "power-cycle the
   drives and/or the CAN adapter".
4. **Mechanical/wiring fault** — loose or damaged connector, partial
   CANH–CANL or line-to-GND short, broken shield, worsened by vibration
   during the move.
5. **Marginal signal integrity at 1 Mbit/s** (termination, stubs) — least
   likely as primary cause (bus ran clean for days), but the load transient
   may have exposed a borderline margin.

## 4. Mitigations

### 4.1 Station — to unblock (immediate)

1. **Power-cycle the drives first** (adapter only if drives alone don't clear
   it) — the only reset that clears a latched transceiver or a drive
   bus-off/fault latch.
2. **Verify the bus is actually healthy before re-running** (not just "daemon
   starts"):
   - `ip -details -statistics link show can0` — state `ERROR-ACTIVE`, check
     `rx-errors` / `tx-errors` / `bus-error` / `bus-off` counters.
   - `candump can0 -ea` in a background terminal during the re-run — any
     `ERRORFRAME` line (especially `tx-recessive-bit-error` /
     `dominant-bit-error`) means the fault is still present.
   - `./turret-can stats` + discovery reads of both drives (e.g. 0x7019).
3. **If it recurs on the against-gravity move, the trigger is reproducible** —
   capture at that moment: candump `-ea` output, the drive supply voltage
   (drive power input and Pi 5V), idle/dominant differential levels (≈0 V
   idle, ≈2 V dominant), and CANH–CANL resistance at the end (≈60 Ω with both
   120 Ω terminators).
4. **Hardware hardening** (if the trigger is confirmed load-related): bulk
   capacitance on the motor power rail near the drives; verify Pi↔drive ground
   bond (short, adequate gauge, star point); separate the CAN pair from motor
   power cables; shield grounded at one end (Pi side); reseat/replace the
   suspect connector.
5. If SI margin proves borderline after hardware fixes: consider dropping to
   **500 kbit/s** (2× timing margin — 1 Mbit/s is unforgiving of length/stub/
   termination margins, per the Vector thread above). Requires setting the
   drive's bitrate register to match.

### 4.2 Firmware / repo — make the next occurrence diagnosable (and maybe self-healing)

The daemon reads the CAN state once at open and never again, so a bus fault
surfaces only as "silent drives → stale-feedback Brake" — indistinguishable
from a motor fault. Proposed changes, in priority order:

1. **Enable `berr-reporting on` in `systemd/can0.service`** (one word in the
   `ip link set ... type can` line). The daemon's RX filter already accepts
   `CAN_ERR_FLAG` frames — with berr-reporting on, the kernel would deliver
   error frames with the exact error class + live TEC/REC counters
   (`error-counter-tx-rx{{128}{0}}` style) into the daemon log: the evidence
   this incident lacked.
2. **Runtime bus-state monitoring**: poll the existing `netlink_query_can`
   (`control/src/can/can_netlink.hpp`, already parses `ErrorPassive` /
   `BusOff` from `RTM_GETLINK`) at ~1 Hz, or subscribe to netlink link-change
   notifications. On `ErrorPassive`/`BusOff`: log a distinct `can bus: ...`
   diagnostic line (separate from `control fault:` motor faults); if the state
   persists (e.g. >2 s), attempt a bounded number of controlled down/up
   recoveries (the kernel `restart-ms 100` then handles bus-off), then
   escalate to a hard, operator-visible fault with the counters attached.
   Safety authority untouched — this is diagnostics + bus recovery, not a
   motion command.
3. **Expose bus error counters in the 1 Hz telemetry** — the queue (Part 2,
   S1/§55 note) already flags that `turret-can stats` counters are not in
   telemetry.
4. **Keep the power cycle as the documented station remedy**, with the §4.1.2
   verification checklist attached, so "recovered" is provable before the Run A
   re-run.

### 4.3 Addendum — live diagnosis 2026-09-02 (supersedes the proposals above)

The fault was re-diagnosed live on the station (15:21–16:10 NZ) with direct
MCP2515 register access over SPI plus a kernel loopback test. Full evidence:
`docs/can_hardware_fault_report.md` (v2). Outcomes:

1. **`berr-reporting` is NOT supported by this kernel's mcp251x driver**
   (6.18.39+rpt-rpi-2712): `ip link set can0 type can berr-reporting on`
   (link down) -> `Error: requested control mode berr-reporting not
   supported`. Mitigation 4.2.1 is therefore **not implementable on this
   kernel** — the delivered supervisor uses netdev counters + `ip` state
   polling instead.
2. **MCP2515 core exonerated at the silicon level**: kernel-level CAN
   loopback mode transmits and receives 5/5 frames with zero errors;
   corrected raw loopback (REQOP=010) passes too. SPI, oscillator, IRQ and
   the driver are all healthy.
3. **An intermittent EXTERNAL signal on CANH/CANL drives real receive
   errors**: MERRF/ERRIF interrupt storms (≥20–100/s, rate scaling with
   receiver bitrate: 1M→63/s, 500k→39/s, 250k→23/s, 100k→20/s), REC pinned
   ≥128 → ERROR-PASSIVE which then sticks (REC decrements only on successful
   receives) until a down/up lands in a quiet window. No valid frame decodes
   at any receiver bitrate (1M…100k sweep). **Correction of record:** the
   earlier "bit-rate-independent phantom frame" (EXT ID 0x01e79fdf, DLC=15)
   was a probe misinterpretation — CANINTF 0xA0 is MERRF+ERRIF (not
   RX1IF+WAKIF; RX1IF=0x02, RX0IF=0x01), and the buffer content was stale
   (read without RXnIF asserted). The v1 "listen-only clean" test was
   actually loopback mode (0x40) and the v1 "raw loopback fail" was actually
   configuration mode (0x80) — both invalid; corrected constants fix both.
4. **Drives are silent** (no discovery response, no ACK) — power-cycle
   needed at the station.
5. **Root cause remains unresolved** — HAT transceiver/RX-path damage, a
   faulted CyberGear transceiver, wiring/termination, and electrical
   disturbance all remain candidates. **Do not classify the HAT as
   confirmed faulty**; run the physical isolation ladder (report §6:
   HAT disconnected → cable → pitch → yaw → both → motor load →
   termination → scope → A/B) before replacing anything.
6. **Software recovery ladder cannot clear a physical-layer fault**; it
   remains correct and required for *bus-level* faults on a healthy HAT.
   Delivered:
## 5. Sources- [CAN Wiki — CAN Bus-Off condition/state](http://can-wiki.info/doku.php?id=can_bus_off&rev=1363860418&mddo=print) — error-state thresholds; bus-off recovery (128×11 recessive bits, CAN 2.0B §8.12 / ISO 11898-1 §6.15); "in real life a controller can switch to ErrorPassive sporadic, but BusOff should never arrive without a hardware error".
- [Vector CAN-list — MCP2515: Recovering from Error-Passive state](https://canlist.vector-informatik.narkive.com/fuk8y5jZ/mcp2515-recovering-from-error-passive-state) — MCP2515 TEC decrements only on actual successful transmissions; "taking off the node with the high error count doesn't mean that node has the problem"; bus length / termination / reflection margins at 1 Mbit/s.
- [Seeed-Studio pi-hats issue #18 — Heavy traffic makes errors and frame drops](https://github.com/Seeed-Studio/pi-hats/issues/18) — near-identical field report: RPi + MCP251x HAT + motor drive, `tx-recessive-bit-error` storm, `tx-error-passive` at counter 128, drive bus-off.
- [Xiaomi_CyberGear_Arduino issue #1](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino/issues/1) — silent CyberGear drive recovered by restarting the drive (default ID 0x7F).
- [Linux kernel — drivers/net/can/mcp251x.c](https://android.googlesource.com/kernel/common.git/+/9a4e328eb2bfa23b160558cff96e17ffa65ea5cf/drivers/net/can/mcp251x.c#3) and [linux-can mcp251x restart-ms patch discussion](http://mail.spinics.net/lists/linux-can/msg02650.html) — kernel CAN state reporting and restart-ms (bus-off only) semantics.

## 6. Occurrence log

**Occurrence 1 — 2026-09-02, P6 Run A (first incident).** Described in §§1–4:
ERROR-PASSIVE mid-check at the against-gravity `pos_return` move; drives
latched silent; survived host reboot + interface cycles + a full MCP2515
SPI re-probe (the loopback isolation test proved the host-side controller
TX/RX path healthy — frames echo perfectly when the physical bus is
isolated — pinning the fault outside the HAT).

**Station recovery — 2026-09-02 ~20:1x.** A power-cycle at the STATION (not
the host — the Pi kept its 14:55 boot) cleared the latches. Full health
checklist passed at 20:19–20:22: `can state` ERROR-ACTIVE before and after
probe, zero `ERRORFRAME` in an 8 s listen, and BOTH drives (0x64/0x65)
answered register reads (loc_kp=30, spd_kp=1.0, spd_ki=0.002 — factory
defaults, expected after a drive power-cycle; the firmware re-writes
check gains at every check start). Confirms §4.1.1: a drive/station
power-cycle is the only reset that clears it.

**Occurrence 2 — 2026-09-02 20:24:27, controld homing (P6 re-validation
attempt, "Run A2").** Cleanest timeline yet:
- 20:24:11 controld boot; homing starts; both drives' feedback flowing.
- ~20:24:16 pitch reaches its first hard-stop push (contact found at
  q≈0.0 — expected: CyberGear's mechanical zero is volatile and restarts
  from the power-on pose, `docs/CyberGear_AI_Reference.md` §Zero position;
  homing correctly re-derives endpoints from contacts).
- **20:24:27 — 16 s into the run, seconds into the first STALL push — the
  first `send_position_ref FAIL ... No buffer space available`.** Telemetry
  frozen at that instant (q_pitch=-0.0074 for the next 2.5 min), 59,625 TX
  failures, daemon SIGTERM'd at 20:26:58 (clean de-energize).
- Aftermath identical to occurrence 1: down/up → ERROR-ACTIVE for 1 s →
  ERROR-PASSIVE with zero host traffic; drives ignore all reads.

**Interpretation.** The bus fails **within seconds of the first stall-load
event** and recovers **only on a station power-cycle**. Of the ranked causes
in §3 this evidences **#1 (load-induced rail sag / ground bounce) as the
TRIGGER** and **#3 (latched transceiver / drive CAN fault) as the
self-sustaining aftermath**. Motor-load dependence was already hinted by
occurrence 1 (failed at the against-gravity move — the highest-load move of
the check) and is now explicit (fails at the stop-push stall).

**Actions before the next run (station-side, in order):**
1. Station power-cycle to unlatch drives/HAT.
2. Physical audit: drive PSU vs Pi/HAT supply topology (shared rail?),
   single common-ground point, 120 Ω termination at both bus ends, CAN pair
   twisted + routed clear of motor phase wires, connector seating
   (HAT header + drive terminals).
3. Re-run the health checklist (§4.1.2) BEFORE any motor test.
4. If it still faults at the first stall after (2): try
   `berr-reporting on` to capture error-frame types mid-fault, and consider
   the last-resort 500 kbit/s mitigation (`COMM_TYPE_22`/0x16, firmware
   ≥ 1.2.1.5; `CyberGear_AI_Reference.md` §23 — CAUTION: a wrong write
   makes the drive unreachable over CAN; needs explicit approval).

**Occurrence 3 — 2026-09-02 21:20–21:23, controld homing under a dedicated
sniffer (post station-reset #2, "Run A3").** Instrumentation: candump
(11737, live data frames), 1 Hz state poller, ip monitor. Note: mcp251x
REJECTS `berr-reporting` ("control mode not supported") — SocketCAN error
frames are therefore unobtainable on this driver; frame-level error
visibility requires the SPI-register forensics approach (MERRF/EFLG via
`can_hardware_fault_report.md`). Sniffer-logging lesson: use
`candump -l -e -L /tmp/sniffer/` (absolute-time log files); `-t d` gives
deltas only.

Timeline (absolute, from daemon log + state ticks; frame-rate from sniffer):
- 21:19:30 sniffer up; bus ERROR-ACTIVE; 6 register reads → both drives
  answered. Agent-side load: ZERO afterwards (passive logging only).
- 21:20:38 controld start; homing proceeds normally (pitch far contact
  −2.1921 ≈ historical −2.1994; both endpoints found; yaw repeatability
  sweep next). Measured steady traffic **85 fps ≈ 30 kbit/s (~3 % of the
  1 Mbit bus)** — 3× below rates this bus sustained for hours when healthy.
- 21:21:00 first **ERROR-WARNING** blip (1 s) — inside the high-power
  contact/push window. Recovered.
- ~21:21:20 sniffer socket stops receiving (degradation bursts overrunning
  its buffer) — degradation has begun. controld's actively-drained socket
  keeps pulling frames ~1 min longer.
- 21:21:58–59 ERROR-WARNING blips + supervisor BRAKE/ALLOW flap (two
  stale-feedback episodes); **21:21:59 first host TX failures** during a
  NORMAL yaw sweep (not a stall push).
- 21:22:05 kernel reports ERROR-PASSIVE. Drive→host RX keeps delivering
  fresh feedback until ~21:22:0x–21:22:16 (telemetry freezes at
  q_yaw=−3.5620) — **the failure is asymmetric: host TX dies while the
  drives' TX/ACK path still works for tens of seconds**. Last sniffer frames
  (21:22:0x): sporadic limit-current piggyback writes + drive feedback —
  then both directions silent.
- 21:23:22 daemon SIGTERM (clean de-energize). Post-mortem: drives latch
  silent to reads; down/up → ACTIVE 1 s → PASSIVE with zero host traffic;
  station power-cycle is again the only known clear.

**Refinements to §3 from occurrences 2+3:**
- NOT a single-event stall spike: the fault *ramps* across ~60–90 s of
  motor running (WARNING inside the first stall window, final TX death
  later during a low-load move) — thermal/progressive transceiver
  disturbance, consistent with §3-1 (rail sag/ground bounce) latching into
  §3-3 (latched transceiver).
- Traffic-volume / "agent software flooding" theory REFUTED by measurement:
  steady-state 85 fps; agent actions during the run were passive logging +
  6 pre-run register reads; failure onset tracks motor power phases, not
  traffic phases.
- TX-first/RX-last asymmetry points at the HAT-side transceiver's TX
  sampling being the disturbance-sensitive element (drives ACKed the host
  at first, then the host could not win the bus at all).

**Next actions, in order, before ANY motor operation:**
1. Station power-cycle (clears current latches — required now).
2. **Motorless traffic soak** (new, decisive): no controld, motors idle —
   `cangen can0 -g 5000 -e` (~200 fps) for 10 min + SPI EFLG/MERRF
   snapshot poller. Survives → traffic + HAT-alone exonerated, motor
   coupling proven by contrast; degrades → HAT/wiring/termination suspect
   independent of motors.
3. Physical audit (§ previous list): PSU topology, common ground,
   termination, pair twist/routing, terminal seating.
4. Re-run A3 with sniffer + SPI EFLG poller; correlate first WARNING with
   motor current from the daemon's 100 Hz motion log.

**Occurrence 4 — 2026-09-02 22:04–22:17, MOTORLESS traffic floods (station
reset #3 = CyberGear power-cycle only; HAT stayed powered).** Decisive
experiment set, motors NEVER energized:
- Post-reset health: link freshened, ACTIVE stable, drives 8/8 responsive
  at factory gains.
- Baseline bug found first: `cangen -g 5000` = 5000 MS (0.2 fps) — the
  5.5 min "clean soak" was a trickle, not a soak. Re-fired at `-g 5`
  (~197 fps measured both at kernel and sniffer level).
- FLOOD #1 (197 fps): **ERROR-PASSIVE at 4.2 min** (error-warn 8→9,
  error-pass 42→43 — single threshold crossings). Frames KEPT FLOWING at
  198 fps during PASSIVE, and both drives ANSWERED probes while still in
  PASSIVE → PASSIVE ≠ dead. Aftermath: stuck PASSIVE while idle (frozen
  counters — MCP2515 REC/TEC hold without traffic), but a plain link
  down/up restored stable ACTIVE with **no power-cycle and no drive latch**.
- FLOOD #2 (197 fps, immediately after, same HAT power session): **6 min
  FULLY CLEAN** (ACTIVE, counters unchanged).
- Sniffer caveat found: this candump's socket also dies (silent overrun)
  during error bursts — state-poll logs, not frame logs, are the clock.

**Model revision (v2) from occurrence 4:**
1. **Base layer (always present, HAT-side)**: an intermittent EXTERNAL
   disturbance on CANH/CANL (matches `can_hardware_fault_report.md` SPI
   forensics: real MERRF/ERRIF error storms, source not host-side)
   probabilistically walks the error counters past 128 under sustained
   ~200 fps traffic — sometimes within ~4 min, sometimes >6 min clean.
   Cleared by a simple link reset; bus keeps limping through PASSIVE.
2. **Motor layer (the P6 killers)**: motor power events (stall pushes,
   high-current phases) raise the disturbance rate/intensity enough to trip
   both nodes within ~16–100 s AND latch the CyberGear transceivers — the
   drive power-cycle in resets #1–3 was curing THIS layer; the kernel TX
   retry storm of the still-flailing controld (59,625 queued sends) made
   occurrences 1–2 look like a permanently dead HAT.
3. Implication for fixes: termination/grounding/routing work (and, if that
   fails, 500 kbit/s) target the base layer — they should also materially
   raise tolerance at the motor layer. Daemon-side TX-failure back-off is a
   required robustness fix regardless (never keep 100 Hz queueing into a
   dead TX path).

### Occurrence 5 — 2026-09-02 23:11–23:15, FIRST MOTOR RUN ON THE SECOND PHY (yousee adapter)

The decisive experiment. New PHY-agnostic transport layer (`can/can_transport.hpp`,
`YouseeTransport`, config `can.backend`) put controld on the USB-CAN AT adapter
(CH340 → /dev/ttyUSB0, AT+CAN_BAUD=1000000 verified); the MCP2515 HAT stayed on
the bus as a passive witness (`candump can0`).

Timeline (all times local; logs: `/tmp/keep/controld_yousee_incidentA.log`,
`/tmp/keep/hat_witness_incidentA.log`):

* 23:11:38 boot on yousee: `CAN transport: yousee device=/dev/ttyUSB0`,
  discovery OK (both UIDs), boot homing auto-starts. Feedback stream: full rate.
* 23:13:06.17 — **last feedback frame seen on the bus by the HAT witness**
  (`028064xx/028065xx`), mid yaw coarse sweep (q_yaw −0.933, no endstop, no
  stall). Both drives' feedback stops at the same instant.
* Supervisor: BRAKE 'stale or missing motor feedback' — correct reaction over
  the new transport; homing engine blocks in its bounded read; 16 BRAKE/17
  ALLOW flaps while q stays frozen (motors held, safe).
* 23:13:40–45 — fresh register probes sent **through the HAT** (`cansend`):
  **zero replies**. Zero feedback in witness. can0 stays ERROR-ACTIVE (self-ACK:
  HAT state cannot see drive silence).
* 23:15 — SIGTERM: clean de-energize over the adapter.

Verdict (isolates the motor layer):

1. The failure **reproduced identically on a second, independent host PHY**
   (~90 s into sustained motor+traffic load) — the host transceiver family is
   NOT the motor-layer culprit.
2. The independent witness proves the drives themselves went silent (both
   axes, simultaneously, mid-normal-motion) and stayed silent to probes from
   the other node. **Motor-layer latch is drive-side/environmental** — power,
   ground, or the drive's own transceiver supply — exactly the state that a
   CyberGear power-cycle cures (occurrences 1–3 pattern).
3. The base-layer HAT episode (Occ. 4, motorless PASSIVE crossing, cleared by
   link reset) remains a separate, lesser host-side fragility — but it did not
   occur again across ~28 min of motorless 197-fps flooding in this HAT power
   window, and it is not what blocks motor development.

Revised next actions:
1. CyberGear power-cycle (recovers Occ. 5, as before) — then re-verify adapter.
2. Physical/electrical audit focused on the DRIVES: PSU capacity/sag during
   motion, drive power vs logic power topology, common-ground point, and — if
   available — scope the drive 12V rail during a homing sweep. The failure
   signature (transceiver silence mid-sweep, MCU presumably alive since a
   plain power-cycle restores it) smells like transceiver-supply brownout or a
   common-mode event coupling into the drive CAN PHYs.
3. Keep running development on the yousee PHY (it behaves correctly;
   supervisor + clean shutdown verified over it). The HAT remains witness-only
   until a replacement arrives.
