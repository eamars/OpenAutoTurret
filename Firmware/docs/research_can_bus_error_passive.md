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

## 5. Sources

- [CAN Wiki — CAN Bus-Off condition/state](http://can-wiki.info/doku.php?id=can_bus_off&rev=1363860418&mddo=print) — error-state thresholds; bus-off recovery (128×11 recessive bits, CAN 2.0B §8.12 / ISO 11898-1 §6.15); "in real life a controller can switch to ErrorPassive sporadic, but BusOff should never arrive without a hardware error".
- [Vector CAN-list — MCP2515: Recovering from Error-Passive state](https://canlist.vector-informatik.narkive.com/fuk8y5jZ/mcp2515-recovering-from-error-passive-state) — MCP2515 TEC decrements only on actual successful transmissions; "taking off the node with the high error count doesn't mean that node has the problem"; bus length / termination / reflection margins at 1 Mbit/s.
- [Seeed-Studio pi-hats issue #18 — Heavy traffic makes errors and frame drops](https://github.com/Seeed-Studio/pi-hats/issues/18) — near-identical field report: RPi + MCP251x HAT + motor drive, `tx-recessive-bit-error` storm, `tx-error-passive` at counter 128, drive bus-off.
- [Xiaomi_CyberGear_Arduino issue #1](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino/issues/1) — silent CyberGear drive recovered by restarting the drive (default ID 0x7F).
- [Linux kernel — drivers/net/can/mcp251x.c](https://android.googlesource.com/kernel/common.git/+/9a4e328eb2bfa23b160558cff96e17ffa65ea5cf/drivers/net/can/mcp251x.c#3) and [linux-can mcp251x restart-ms patch discussion](http://mail.spinics.net/lists/linux-can/msg02650.html) — kernel CAN state reporting and restart-ms (bus-off only) semantics.
