# Post-Homing Test Queue (Phase 2)

**When to run:** after the live homing run (below) is verified on the real turret.
**Scope:** Phase 2 only — boot → home → safe hold → park. **Tracking is disabled**
in Phase 2; nothing in this queue enables it.
**Safety posture:** every command in the daemon passes the SafetySupervisor. There
is no open-loop path. Stale feedback → **Brake** (recoverable safe stop); a motor
fault / over-temp → **Disable** (de-energize, sticky). The station never homes or
moves with an unknown motor state (boot faults lock out).

Build + run from `Firmware/build`:
```
cd Firmware/build && ninja
```
Binaries: `./control/controld` (the daemon), `./turret-can` (diagnostics).

---

## P0 — Live homing run (user-run gate)

The Phase-2 deliverable: **reliable boot → homed → safe hold → park cycle.**
Everything below assumes this passes.

```
./control/controld config/turret.yaml
```

**Expected sequence (watch the log):**
1. `controld starting (config: config/turret.yaml)`
2. `boot OK: pitch uid=0x… yaw uid=0x…` (discovery + self-test pass)
3. `homing started; tracking is DISABLED in Phase 2`
4. Per-axis homing (yaw first, then pitch, per the config): coarse approach →
   contact → back-off → fine approach → contact → repeatability pass.
5. `homed + at ready pose; holding (Ctrl-C to park)` — the daemon moves to the
   logical midpoint of each axis's travel (never at a stop) and holds.

**Pass criteria:**
- Both axes home without a `control fault:` line.
- The daemon reaches `homed + at ready pose` and holds (telemetry line shows
  `phase=hold`, positions stable).
- No unexpected motion, no over-temperature, no CAN errors (see P1).

**If it fails:** the log states the fault reason (e.g. `contact not detected`,
`travel out of band`, `motor self-test failed`). Do **not** retry blindly —
inspect the fault, fix the cause (mechanical stop position, CAN wiring, config
`expected_travel_deg`), then re-run.

---

## P1 — Safe-hold verification (after homing, before anything else)

With the daemon holding at the ready pose (P0 step 5), confirm it is genuinely
stationary and the bus is healthy.

**Checks (no daemon restart needed):**
- **No drift:** in a separate terminal, `./turret-can read pitch 0x7019` and
  `./turret-can read yaw 0x7019` (MechPos) — take 3 samples ~1 s apart. The
  variance must be ≤ 0.0004 rad (at rest). Use `read`, **never** `feedback`
  (feedback re-energizes the motor and is not a valid post-stop check).
- **Bus healthy:** `./turret-can stats` — no RX/TX errors, no bitrate warnings.
- **Thermals:** the daemon telemetry + `turret-can read <axis> 0x700C` (temp) —
  motor temp well below the 75 °C fault threshold and not climbing.
- **Hold under load:** gently try to move a turret arm by hand. The position-mode
  hold should resist (the motor is enabled, holding torque). Release. Verify the
  position returns to the held value and the daemon stays in `hold`.

**Pass criteria:** position variance ≤ 0.0004 rad, no CAN errors, temp stable,
hold resists a light manual push and recovers.

---

## P2 — Shutdown / park cycle (§33)

Trigger a clean shutdown and verify the park sequence.

```
# in the daemon terminal, press Ctrl-C (SIGINT)
```

**Expected (with a *valid* park pose):** `shutdown requested; parking` → the
daemon moves yaw then pitch to the park pose, verifies position + velocity within
tolerance, dwells, then de-energizes pitch then yaw → `PARKED (motors
de-energized at the park pose)`.

> ⚠️ **FLAG — the shipped park defaults are placeholders (see Flags below).**
> `config/turret.yaml` ships `yaw_park_deg: 0` / `pitch_park_deg: 0`. Logical 0 is
> the **low mechanical stop** (homing sets logical 0 = low endpoint), and the soft
> limit is inset 5° — so the 0/0 park pose sits **outside the soft limits** and
> **violates §33.1** ("park must be inside the soft limits, not against the
> stop"). The ParkController **rejects it**, and the daemon falls back to
> **de-energizing at the current (ready) pose**, which is still safe (the ready
> pose is the travel midpoint, away from both stops) — the log will show
> `de-energized (phase=…, fault='')` instead of `PARKED`.
>
> **Before relying on the park pose**, set `yaw_park_deg` / `pitch_park_deg` to a
> safe in-travel pose (e.g. the midpoint, or a designated "stowed" side pose) and
> re-run this test to confirm the full `PARKED` path.

**Checks:**
- De-energize order: **pitch then yaw** (per §33; verify in the log / motor state).
- After de-energize: `./turret-can read <axis> 0x7019` — the position should hold
  by friction (no gravity-induced slide). If an axis slides, the park pose or the
  friction is insufficient — choose a pose where the arm is balanced.
- A second `controld` boot after a park re-homes cleanly (see P3).

**Pass criteria:** clean `PARKED` (once the park pose is fixed) or safe
de-energize at the ready pose (with the default 0/0), correct de-energize order,
no gravity slide.

---

## P3 — Reboot + re-home repeatability

Reboot the daemon and re-home; verify the endpoints repeat within tolerance.

```
# Ctrl-C the daemon (park), then:
./control/controld config/turret.yaml
```

**Checks:**
- Both axes re-home to the same endpoints as P0 (the homing log shows the same
  contact positions, within the repeatability tolerance — `repeatability_rad`
  default 0.5°).
- The ready pose is the same as P0.
- No fault on the re-home.

**Pass criteria:** re-home succeeds, endpoints repeat within 0.5°, same ready pose.

---

## P4 — Stale feedback / CAN timeout (§39.4)

Verify the recoverable **Brake** behavior when motor feedback goes stale.

**Method (pick one, least-intrusive first):**
- **Preferred:** throttle the CAN traffic. In a separate terminal, generate a
  burst of unrelated CAN frames on `can0` (or briefly reduce the feedback rate)
  so the daemon's feedback ages out past `feedback_max_age_ms` (100 ms).
- **Invasive (last resort):** briefly `ip link set can0 down` and back up. This
  drops all feedback; the daemon should Brake, and recover when the link returns.

**Expected:** the daemon logs a `Brake`-related safe stop (velocity → 0, hold at
the current safe position). This is **recoverable** — it is NOT a fault. When
feedback returns, the daemon resumes `hold` with no manual reset.

**Pass criteria:** on stale feedback → safe stop (no open-loop, no motion into a
limit); on feedback recovery → back to `hold`. No `control fault:` (a fault would
mean it was mis-classified as a Disable).

---

## P5 — Motor fault injection (§38)

Verify the sticky **Disable** behavior on a hard motor fault.

**Method:** use `./turret-can write` to set a fault bit on one motor (or trigger
a real over-current), or temporarily set the `faults` via the register. Confirm
the daemon observes the fault.

**Expected:** the daemon logs a `control fault:` (the supervisor issues
**Disable**). It de-energizes **both** axes (a fault on one axis stops the whole
station, not just that axis) and fault-locks (sticky — it does not auto-recover).
A reboot is required to clear the fault-locked state.

**Pass criteria:** fault → both axes de-energized, `phase=fault`, sticky until
reboot. No motion after the fault.

> ⚠️ This is a **destructive** test (fault-locks the station). Run it last, and
> be ready to reboot the daemon to recover.

---

## P6 — Payload response check (§27, optional)

**Phase 2: STUBBED.** The `OPTIONAL_PAYLOAD_RESPONSE_CHECK` state is a no-op in
Phase 2 (no payload attached / no payload driver). Verify only that the daemon
passes through it without fault (it does — the log shows the hold transition
with no payload-related fault). The real payload check lands in Phase 3.

---

## Flags to resolve during commissioning

1. **Park pose (P2):** `yaw_park_deg`/`pitch_park_deg` ship as `0/0` = the low
   stop (outside the soft limits, violates §33.1). The ParkController rejects it;
   the daemon safely de-energizes at the ready pose instead. **Set a safe
   in-travel park pose before relying on the park.** This is the top commissioning
   item.
2. **Expected travel bands:** `axes.*.expected_travel_deg` are conservative
   placeholders (pitch ±120°, yaw ±180°). After the first real homing, update them
   to the measured travel (the homing validates the measured travel against these
   bands; too tight a band will reject a valid home, too loose weakens the check).
3. **YAML commissioning params:** the 23 §58 commissioning parameters are
   conservative placeholders (config-driven, never compile-time). Refine them
   (contact effort thresholds, speeds, tolerances) against the real motors.
4. **Leftover virtual interface:** a `vcan9` interface was left up from earlier
   testing. Confirm it is not needed and remove it (`ip link del vcan9`) so it
   cannot shadow real traffic.

---

## Out of scope (Phase 3+)

- Collision / safety envelope tracking (§18.2/§19) — tracking is disabled here.
- Calibration persistence / reload (§28/§41).
- Structured event logging + telemetry service (§43/§55).
- HIL harness for the full control stack (§54).
- Payload driver (§27 payload check).
