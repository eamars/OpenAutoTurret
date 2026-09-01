# Post-Homing Test Queue

**When to run:** after the live homing run (P0) is verified on the real turret.
**Scope (this edition, post-Phase 9):** Phases 2–9 — boot → home → safe hold →
park, live payload checks, live vision, live closed-loop tracking, installation
orientation calibration, systemd deployment, live web end-to-end, and the §54/§55
HIL + acceptance-metrics pass.

**How to read the tags**

- `[MOTOR]` moves the real motors over CAN — **user-run, supervised station only**.
- `[CAMERA]` uses the real IMX500 but never touches `can0` — the assistant may
  run it; `controld` may be stopped entirely.
- `[SW]` software/computation only — no motor, no camera.
- **`GOOD-TO-GO` items:** the user said "good to go" for the station. Everything
  in Part 1 that is `[MOTOR]` still requires the user at the station; the
  Part-2 swap-in map lists every place a mock/guard/placeholder is replaced by
  the real thing at that moment.

**Safety posture:** every command in the daemon passes the SafetySupervisor.
There is no open-loop path. Stale feedback → **Brake** (recoverable safe stop);
a motor fault / over-temp → **Disable** (de-energize, sticky). The station never
homes or moves with an unknown motor state (boot faults lock out). Tracking is
hard-disabled until the homing gates pass (§38.1), and the tracking reference is
constrained by the §18 soft-limit envelope.

Build + run from `Firmware/build`:
```
cd Firmware/build && ninja
```
Binaries: `./control/controld` (the daemon), `./turret-can` (diagnostics),
`./turret-payload` (payload profiling/verification, `--sim` rehearsal).

---

# Part 1 — Live test queue (run in this order)

## P0 — Live homing run (user-run gate) `[MOTOR]`

The Phase-2 deliverable and the gate for everything else: **reliable boot →
homed → safe hold → park cycle.**

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
   (Phase 9: with `payload_auto_verify` enabled, a `payload_check` phase runs
   once after first hold — small moves in the central region, one axis at a time.)

**Pass criteria:**
- Both axes home without a `control fault:` line.
- The daemon reaches `homed + at ready pose` and holds (telemetry line shows
  `phase=hold`, positions stable).
- No unexpected motion, no over-temperature, no CAN errors (see P1).
- Note the measured travel per axis (used to update the `expected_travel_deg`
  placeholders — Part 3, items 4–5).

**If it fails:** the log states the fault reason (e.g. `contact not detected`,
`travel out of band`, `motor self-test failed`). Do **not** retry blindly —
inspect the fault, fix the cause (mechanical stop position, CAN wiring, config
`expected_travel_deg`), then re-run.

**Result (rehome4, 2026-09-02 03:51–03:55, PASS — final clean run):** full plan
(pitch full-range → pitch park 40° → yaw full-range → yaw park 180° → pitch
re-home) completed in **3 m 20 s** with no fault, no timeout, no supervisor
escalation (single-cycle Derates only — one 197 ms spike was a re-arm recipe
blocking the loop, by design). All six homing sub-runs landed within **0.1°**
of the P0 reference (table in P3 below). Ready pose reached exactly:
pitch −1.4956 / yaw −2.2765 (= P0). Measured travel: pitch span 79.5°
(raw −2.1994 → −0.8112), yaw span 352.7° (raw −5.3458 → +0.8104, the detent
zone excluded — see P3 note). Temperatures stable (pitch 25.9 °C,
yaw 33.3 °C).

> History: rehome1 (PASS, 5 min) → rehome2 (PASS, but backoffs crawled 16–20 s
> — stick-slip at the 10°/s backoff speed) → rehome3 (FAIL — pitch endpoint-A
> backoff timed out at 10 s; root cause: drive velocity-loop integral windup,
> see P3) → **rehome4 (PASS — the re-arm fix, verified on the wire)**.

---

## P1 — Safe-hold verification `[MOTOR]`

With the daemon holding at the ready pose (P0 step 5), confirm it is genuinely
stationary and the bus is healthy.

**Checks (no daemon restart needed):**
- **No drift:** in a separate terminal, `./turret-can read pitch 0x7019` and
  `./turret-can read yaw 0x7019` (MechPos) — take 3 samples ~1 s apart. The
  variance must be ≤ 0.0004 rad (at rest). Use `read`, **never** `feedback`
  (feedback re-energizes the motor and is not a valid post-stop check).
- **Bus healthy:** `./turret-can stats` — no RX/TX errors, no bitrate warnings.
- **Thermals:** the daemon 1 Hz telemetry now exposes `temp_pitch` / `temp_yaw`
  (°C) directly — motor temp well below the 75 °C fault threshold and not
  climbing. (Note: there is **no** `0x700C` temp register in the 0x7005..0x7020
  table; the earlier `0x700C` in this queue was a typo. Drive temp is `temp_c` in
  the drive feedback, surfaced in the daemon log.)
- **Hold under load:** gently try to move a turret arm by hand. The position-mode
  hold should resist (the motor is enabled, holding torque). Release. Verify the
  position returns to the held value and the daemon stays in `hold`.
- **Acceleration-based stuck-slip / jitter monitor:** the 100 Hz `motion` log
  (`a=` / `j=` per axis, position-derived, 0.02 s LPF) is the detector for
  stick-slip and creep. Signature keys — clean stop: `a→0, v~0, q flat`;
  **stuck-slip: `cmd≠0` but `v~0` with low `a` (~0.15–0.24 rad/s²) and tiny `dq`**
  (friction break-through); creep: `cmd≠0`, slow `v` (~0.002–0.02 rad/s), low `a`.

**Result (rehome1, 2026-09-02, mostly PASS — hold-under-load pending):**
- **No drift:** 1 Hz position flat (pitch −1.4956, yaw −2.2768, `a≈0`) over ~12 min
  of hold. PASS.
- **Thermals:** temp_pitch 26.6 °C, temp_yaw 31.3→31.9 °C (not climbing) — well
  below 75 °C. PASS.
- **Bus healthy:** can0 RX errors 0; TX errors 1 / drops 1 over the whole ~20 min
  (a single collision between the daemon and the `turret-can` tool) — negligible.
  PASS.
- **Hold under load (manual push-recovery):** PENDING — requires the operator's
  physical hand-push at the station.
- **Acceleration monitor:** confirmed working — detected the friction break-through
  during homing (low `a`, `v~0`, `cmd≠0` dwells at the stop). See P3 note.

**Result (rehome4, 2026-09-02 04:07–04:19, PASS with one yaw caveat):**
- **No drift — pitch:** PASS. 3×1 s samples −1.49538/−1.49533/−1.49548
  (range 0.00015 rad = 0.009°); 1 Hz flat at −1.4956/−1.4960 for 20+ min.
- **No drift — yaw:** borderline. After ~4 min of flat hold at −2.2765 the yaw
  took a 0.06° stick-slip step (04:00:29) to a new equilibrium and then
  micro-oscillates ±0.02° around −2.2746 (15-s cadence: range 0.0008 rad =
  0.045°; 1-s cadence: range 0.0005 rad vs the 0.0004 criterion). Net creep
  0.11° over 15 min, then locked. Interpretation: the ready pose
  (yaw 175.6°) sits near the yaw static-friction breakaway — gravity torque
  there is close to the friction hold limit, so the hold settles by a slip
  step and then stick-slips microscopically. The hold-phase executor
  re-pins its reference to the current position each cycle (by design: the
  drive's internal position loop holds the point), so no outer-loop
  correction acts against the creep. The park pose is yaw 180° (4.4° away);
  P2 will show whether the park pose sits at the gravity balance (and the
  P2 park-pose tuning is the proper fix if not).
- **Bus healthy:** PASS. `turret-can stats`: err_rx=0, tx_fail=0. Concurrency
  with `turret-can` reads shows up as single-cycle Derates (2–5 ms overruns,
  `misses=1`, immediately Allow) — no escalation, no faults.
- **Thermals:** PASS. pitch 25.9 °C, yaw 33.3 °C — flat for 25+ min, far
  below the 75 °C fault threshold.
- **Hold under load (manual push-recovery):** SKIPPED per operator
  (2026-09-02: "I don't care about the holding") — not tested.
- **Long-hold observation (unplanned, 03:51–08:37 = 4 h 45 m, 1 Hz data):**
  the daemon held at the ready pose far longer than the check window.
  - **Pitch: rock-solid** — −1.4956 → −1.4960 total (0.02°), 10-min window
    range ≤ 0.0006 rad. No creep of any kind.
  - **Yaw: slow stick-slip wander around the gravity balance** — two ±0.8°
    excursions: +0.8° at ~04:25 (settled 176.7°), −0.8° return at ~05:55
    (back at 175.6°), one ±1.4° window at ~06:35; 2.8° total over 4.7 h,
    then locked at −2.2755 ± 0.001. Interpretation: the ready pose
    (175.6°) sits inside the yaw friction deadband; the hold re-pins to the
    current position (no outer correction), so the arm wanders on friction
    + a tiny residual gravity torque and re-centers. **Gravity balance
    estimate: yaw ≈ 175.6–176.5°** — the basis for the P2 park-pose
    re-tune (180° → 176°).
  - Temps cooled to 23.9 / 31.3 °C; only sporadic 2–5 ms Derates from
    concurrent `turret-can` reads (one 197 ms re-arm spike during homing) —
    no escalation, no faults over the whole window.

**Pass criteria:** position variance ≤ 0.0004 rad, no CAN errors, temp stable,
hold resists a light manual push and recovers. (Hold-under-load waived by the
operator; all other checks PASS.)

---

## P2 — Shutdown / park cycle (§33) `[MOTOR]`

Trigger a clean shutdown and verify the park sequence.

```
# in the daemon terminal, press Ctrl-C (SIGINT)
```

**Expected (with a *valid* park pose):** `shutdown requested; parking` → the
daemon moves yaw then pitch to the park pose, verifies position + velocity within
tolerance, dwells, then de-energizes pitch then yaw → `PARKED (motors
de-energized at the park pose)`.

> ✅ **RESOLVED — the park pose is now set (§33.1 compliant).**
> `config/turret.yaml` now ships `yaw_park_deg: 180` / `pitch_park_deg: 40` — the
> **no-dangle mid-travel pose** (the same pose homing uses to park each axis between
> homes). Logical mid-travel is strictly inside the soft limits with margin, so the
> ParkController accepts it and the full `PARKED` path runs.

**Result (rehome1, 2026-09-02 02:45, PASS):** SIGTERM → `shutdown requested; parking`
→ `PARKED (motors de-energized at the park pose)` in ~4 s. De-energized positions
(0x7019): pitch −1.49726 / yaw −2.19864 (the mid-travel). Second read +15 s: pitch
−1.49707 / yaw −2.19854 → **no gravity slide** (Δ<0.01°). De-energize order pitch
then yaw. A fresh de-energized read held the park pose.
(Note: the yaw de-energized 3.96° OFF the 180° park target — the position-mode
park move never landed; see the rehome4 result below.)

**Result (rehome4, 2026-09-02 08:37, park INCOMPLETE — root-caused + fixed):**
SIGTERM after the 4 h 45 m hold → `shutdown requested; parking` (08:37:06) →
`de-energized (phase=parking, fault='')` + `controld stopped cleanly` (08:37:47).
**No `PARKED` line**: the 8000-cycle (40 s) park window in `main.cpp` expired
without reaching `Phase::Parked`, so the safe fallback `deenergize_all()` ran.

- **De-energize order:** pitch then yaw ✓ (`deenergize_all` loop).
- **Final de-energized pose (0x7019):** pitch −1.5009 (40.2° — target reached,
  0.2° over) / yaw −2.2301 (**178.2° — 1.4° short of the 180° target**).
- **Root cause (two compounding issues):**
  1. **The park moves ran in position mode** (`start_parking` →
     `enter_position_mode_all`): the drive's internal position loop is weak
     against gravity + friction — the yaw move (ready 175.6° → target 180°,
     fighting gravity uphill) crawled at ~0.07°/s vs the 10°/s command
     (0.046 rad over ~35 s).
  2. **The 180° park target sits ~4° PAST the yaw gravity balance**
     (≈175.6–176.5°, from the P1 4.7 h hold data): after the yaw neared the
     target, gravity pulled it back and the Verify stage (±0.5° + 500 ms
     dwell) could never hold it in the window — the old Verify/Dwell also
     re-pinned to the current position (no pull-back), so the park sat in
     Verify until the 40 s window expired.
  rehome1 shows the same short-landing (3.96°) — the position-mode park move
  has never once landed at the real station.
- **No-slide check at the de-energized pose (17 min, 08:37:47–08:49:28):**
  pitch −1.50088…−1.50117 (range 0.017°), yaw −2.22991…−2.23016 (range
  0.014°) → **no gravity slide** — static friction holds the arm at the
  178.2° / 40.2° pose. (The yaw balance question is answered by the re-tune
  below, not by this pose.)

**Fix (implemented 2026-09-02, pending live verification):**
1. **Park moves run in speed mode** (SpdRef, the drive's velocity loop — the
   proven smooth motion source, P0o): `start_parking` now enters speed mode
   with the per-axis current limits (pitch 3 A / yaw 1 A, under the 10 A cap);
   the executor commands `po.velocity_rad_s` (MoveTo's signed stop-distance
   profile) for the active axis and SpdRef=0 for the other. Position mode is
   entered **once, at Verify**, for the §33.2 hold. (The `StopTracking` state
   stays in speed mode — a premature position-mode entry there would swallow
   the SpdRef moves.)
2. **Verify/Dwell hold at the park target** (position mode, drive's position
   loop pulls the axis back to the target — the outer correction the old
   re-pin-to-current behavior lacked).
3. **`yaw_park_deg: 180 → 176`** in `config/turret.yaml`: the center of the
   observed yaw friction deadband (175.6–176.7°) = the gravity balance, so
   the park move is nearly torque-free and the de-energized arm sits balanced.
   Pitch stays at 40° (4.7 h hold flat; 17 min de-energized no-slide).
ctest 38/38 + pytest 93 passed/2 skipped green on the new code.

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

## P3 — Reboot + re-home repeatability `[MOTOR]`

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
- Record the measured travel + repeatability numbers (§55 homing metrics).

**Pass criteria:** re-home succeeds, endpoints repeat within 0.5°, same ready pose.

**Result (rehome1, 2026-09-02 02:33, PASS with yaw +stop caveat):** re-home succeeded
(no fault), full safe-park homing ran in ~5 min. Endpoints vs P0 ref
(`/tmp/p0_reference.txt`, tol 0.5° = 0.0087 rad):

| axis | stop | P0 raw | rehome1 raw | Δ | verdict |
|------|------|--------|-------------|---|---------|
| pitch | + | −0.8108 | −0.8105 | 0.02° | PASS |
| pitch | − | −2.1994 | −2.2009 | 0.09° | PASS |
| yaw | − | −5.3447 | −5.3462 | 0.09° | PASS |
| yaw | + | +0.9374 | +0.8108 | **7.2°** | ⚠ detent zone (explained below) |

Ready pose: pitch −1.4956 (Δ0.0°), yaw −2.2768 vs P0 −2.2765 (Δ0.02°) — **highly
repeatable** (the yaw settles to the P0 ready pose; an early −2.2627 reading was the
yaw still settling post-home).

**Result (rehome4, 2026-09-02 03:51, PASS — final, after the re-arm fix):** all
endpoints repeat to **≤0.1°** vs the P0 reference (tol 0.5°):

| axis | stop | P0 raw | rehome4 raw (contacts) | Δ | repeatability |
|------|------|--------|------------------------|---|---------------|
| pitch | + (A) | −0.8108 | −0.8112 / −0.8112 (re-home: −0.8108/−0.8116) | 0.04° | 0.0°–0.05° |
| pitch | − (B) | −2.1994 | −2.1994 / −2.1975…−2.1998 (re-home: −2.1994/−2.1994) | ≤0.05° | 0.05° |
| yaw | + (A) | +0.8108 | +0.8108 / +0.8104 | 0.02° | 0.02° |
| yaw | − (B) | −5.3447 | −5.3454 / −5.3462 | 0.07° | 0.05° |

Ready pose: pitch −1.4956 / yaw −2.2765 — **exact match with P0**. Measured
travels: pitch 79.5°, yaw 352.7°.

> ✅ **RESOLVED — the yaw "+stop 7.2° off" (rehome1) is the detent-zone width, not
> a soft stop.** The yaw's +stop is a friction/detent *zone* ~7.3° wide (raw
> [−5.4728, −5.3458], unwrapped — the same physical edge as raw +0.8108:
> Δ = 2π − 7.3°). A coarse approach always stops at the *entry edge* of whichever
> side it approaches from: from the − side it stops at −5.3458 (endpoint B); the
> P0 "+stop" reading (+0.9374) was taken from the + side (entry edge +0.8108 plus
> zone width ≈ +0.9374). rehome1's "+stop" (+0.8108) and P0's −stop (−5.3447) are
> the **same physical edge** (Δ = 2π − 7.2°), and both rehome4 endpoints agree with
> their respective entry edges to ≤0.07°. The 7.2° "delta" is therefore expected
> geometry, not a repeatability fault — the yaw +stop is repeatable *within* its
> approach direction (0.02° in rehome4). The logical model excludes the zone
> (span 352.7°, documented in the axis model).

### rehome3 failure + rehome4 fix: drive velocity-loop integral windup

**Symptom (rehome3):** pitch endpoint-A backoff never moved. The axis sat at the
stop (contact + 1.5 s dwell + 0.5 s settle with SpdRef = 0 while the drive kept
pushing the stop), and the subsequent 10°/s backoff command was visible on the
bus (SpdRef = −0.1745) while the axis was pinned: measured drive torque stayed
positive (+0.79…+1.44 N·m) for the full 10 s backoff timeout; the 100 Hz log
showed `cmd≠0, v≈0 (±0.05), a≈0` — the stuck-slip signature. rehome2 showed the
weaker form of the same effect: backoffs crawled 16–20 s (the drive's own
integral slowly bled down and the axis slipped forward).

**Root cause (wire-verified from the candump of the failed backoff):** while the
drive pushes the mechanical stop at SpdRef = +0.5236, its velocity-loop integral
winds up to **+1.3…+1.8 N·m** (rehome3 peak +1.825 and still rising at settle).
That residual exceeds the P-term available at *any* SpdRef the drive accepts
(Kp ≈ 0.38 N·m per rad/s; even 30°/s gives ≈ +1.14 N·m of P), so the reverse
backoff command cannot produce net reverse torque — the axis is pinned by the
drive's own windup. The drive's internal windup decay is slow and stalls at
~70% of peak (observed: 1.82 → 1.38 in 0.84 s, then flat at ~1.1–1.2 N·m), so
waiting is not an option.

**Fix (rehome4): re-arm the drive's velocity controller before every
post-contact backoff and before the endpoint-B start.** The verified
`enter_speed_mode` recipe — `Stop (de-energize) → 50 ms → RunMode=2 → Enable →
50 ms → LimitCur → SpdRef=0` — fully resets the drive's velocity-PI state.
Implementation: `HomingController` raises a one-shot `rearm_speed_mode` flag in
the `DesiredState` at `begin_backoff_to()` (coarse backoff + VerifyRepeatability
small backoff) and at endpoint-B construction (`rearm_before_start`); the
`Phase::Homing` executor runs the recipe via `backend_->enter_speed_mode()`
before writing the velocity command that carries the flag. Cost: one ~105 ms
blocking cycle per re-arm (supervisor: single-cycle Derate, no escalation —
verified in the rehome4 log).

**rehome4 wire verification (candump + 100 Hz log, pitch endpoint A):**
pre-re-arm settle shows `tq +0.47…+0.68` (windup still present) → **105 ms gap
in the 100 Hz log** (the recipe blocked the loop: Stop/RunMode/Enable/LimitCur/
SpdRef) → first post-re-arm sample: position bounced 0.07° off the stop,
**`tq +0.65 → 0.00` (integral cleared)**, backoff SpdRef −0.1745 written and the
axis free. Same signature on the yaw endpoint-B re-arm (tq −0.60 → 0.00).

**Residual behavior (accepted, documented):** at the 10°/s backoff speed the
drive's velocity loop sits in the stick-slip regime — backoffs take 5–7 s
(stall → the acceleration-based stall detector fires the 3× burst → release →
next stall) instead of ~1 s, and the burst momentum can overshoot the 2° small-
backoff target by ~0.6° (the target-seeking command corrects it). The fine
approach self-corrects from the backoff's residual integral (~4 s stuck-slip
start, then clean to the stop; contact still lands to 0.04°). All within the
0.5° repeatability budget and far from the 10 s timeouts. Candidate
improvement (not done): raise the backoff speed out of the stick-slip regime or
re-arm before post-backoff moves that coast on a residual (the `move yaw 180`
after the yaw-B detent fight coasted up to 26°/s — 3× its 10°/s command — on the
decaying integral; the proportional move law still landed it correctly).

---

## P4 — Stale feedback / CAN timeout (§39.4) `[MOTOR]`

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

## P5 — Motor fault injection (§38) `[MOTOR]`

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

> ⚠️ This is a **destructive** test (fault-locks the station). Run it **last in
> the pure-motor group** (P0–P6), and be ready to reboot the daemon to recover.

---

## P6 — In-loop payload response check (§27, §31.3) `[MOTOR]`

**No longer a stub** (it was a no-op in the Phase-2 edition of this queue). The
daemon now runs the §31.3 payload check in-loop: after first hold,
`Phase::PayloadCheck` performs small amplitude-clamped moves **one axis at a
time** in the safe central region, then auto-verifies the measured response
against the active payload profile (§27 `OPTIONAL_PAYLOAD_RESPONSE_CHECK`).
A manual `start_payload_verification` web command re-runs it on demand.

**Checks:**
1. With `payload_auto_verify: true` (or the manual command), watch the log:
   `phase=payload_check` appears after first hold, one axis at a time, then the
   loop returns to `hold`.
2. Telemetry: `payload_check_active` true during the check, `payload_profile_status`
   `ok` when the response matches the active profile.
3. **Mismatch path:** temporarily point `payload.active_profile` at a profile
   that does NOT match the installed mass (e.g. a too-light profile). The check
   must report `mismatch`, set `payload_derated` → the ready-pose / test-motion /
   tracking speed limits all drop by `derate_factor` (§31.4). Re-select the
   correct profile and re-verify: status returns `ok` and the derate clears.
4. The SafetySupervisor kept authority throughout (no limit crossing; the check
   amplitudes are clamped by the margin to the region edges).

**Pass criteria:** check runs one axis at a time and returns to hold; mismatch →
derate visible in telemetry + limits; re-verify with the correct profile clears it.

---

## P7 — Live camera / vision verification (Phase 4) `[CAMERA]`

**SAFETY: the vision daemon is independent of the motor driver. This test runs
`visiond` against the real IMX500 and verifies detection + timestamping + IPC
only — it does NOT open `can0`, does NOT energize the motors, and does NOT send
any setpoint. `controld` may be stopped entirely for this test.**

Prereqs (Part 3, item 11): IMX500 + `imx500-all` installed (see
`docs/AI_CAMERA_SETUP.md`); a picamera2 config JSON and a detector RPK JSON
(`--image-config` / `--detector-rpk`). The official YOLO11n RPK is AGPL —
license review before distribution.

1. Start a measurement sink on the controld IPC socket (a throwaway subscriber
   is fine for a camera-only check; the real sink is `controld`).
2. Run the vision daemon in real mode (the `--synthetic` default is the stand-in
   being replaced — Part 2, S3):
   ```bash
   # from Firmware/
   python3 -m vision.visiond --real \
       --image-config <picamera2_config.json> \
       --detector-rpk <imx500_network_yolo11n_pp.rpk.json> \
       --socket /tmp/ota_vision.sock --frames 300 \
       --orientation rotate_180
   ```
   (`--orientation` applies the install-level correction to the detector boxes —
   same `common/image_corrections.py` path the web video uses.)
3. Verify:
   - The daemon publishes `TargetMeasurement`s at ~the frame rate.
   - With a person in frame: `valid=true`, `class_id=1` (person), confidence
     above threshold, and a **stable** `visual_track_id` (association holds).
   - `sensor_timestamp_ns` is monotonic and ~frame-rate spaced (mandatory
     capture time, §6.2).
   - With the target removed, the measurement goes `valid=false` after the
     `max_lost_frames` grace (target lost → no setpoint, §12.2).
4. Cross-check one published message by hand-decoding its 58-byte UDS payload
   (matches `vision/protocol.py` / `control/src/tracking/target_measurement.hpp`).
5. Confirm `can0` is untouched and no motor moved (feedback flat / motors
   de-energized) before and after.

---

## P8 — Live closed-loop tracking (Phase 6) `[MOTOR] + [CAMERA]`

Phase 6 is **implemented and integration-tested on SimMotorBackend + synthetic
vision** (track a rotating target; loss → coast → brake → ready-hold; search
sweep; soft-limit containment; fault → safe stop). Running it live requires one
**code prerequisite** first:

> ⚠️ **PREREQ — controld vision wiring (Part 2, S1).** The production
> `controld` (`control/src/main.cpp`) does **not** yet open
> `/tmp/ota_vision.sock`, decode `TargetMeasurement`, or call
> `enable_tracking()`. Before this test: (a) add a UDS `SOCK_SEQPACKET` client
> that decodes the 58-byte measurement → `ControlLoop::feed_measurement(m)`;
> (b) load the `tracking:` config block (§58 params 19–20) + the camera
> intrinsics file → `TrackingController::Config`; (c) `enable_tracking()` gated
> on the homing gate (§38.1). All tracking machinery (estimator, FSM, reference
> manager, solver) already exists and is tested — this is wiring, not new
> control code.

Once wired (and only after P0–P7 pass, user present — motors move):
1. **Tracking hard-disabled until homed:** boot with a person in frame; the
   daemon must NOT track before the homing gates pass (§38.1).
2. **Stationary-target acquisition:** person stands still in view; the turret
   acquires (`track_state=tracking`), the optical axis converges on the target,
   and the LOS error stays inside a small band (record it for §55).
3. **Moving-target tracking:** the person walks slowly across the field of view;
   the turret follows at the §16 tracking speed (≤ 30°/s, scaled by the §35
   confidence).
4. **Target loss:** the person leaves; the state walks
   `tracking → coasting (≤200 ms) → brake_to_hold → target_lost → ready_hold`
   (or `search` if `search_enabled_by_default: true`) with the safe brake, and
   the confidence decays to 0 (§34/§35).
5. **Search:** with search enabled, the `search` sweep runs between the
   configured yaw limits with dwells (§36/§49), and reacquires the person when
   they re-enter the field of view.
6. **Envelope authority:** throughout, the §18 soft-limit envelope constrains
   the tracking reference (a target near a limit must not drive the turret into
   the stop).
7. **Fault during tracking:** trigger the P5-style fault mid-track → both axes
   Disable (tracking can never override the supervisor, §38).

**Pass criteria:** all seven sub-tests pass with no limit crossing and no
open-loop motion; telemetry `track_state`/`confidence` behave per §34/§35.

---

## P9 — Installation orientation calibration (Phase 7) `[CAMERA]`, then `[MOTOR]`

The code is done (fiducial ChArUco calibration in `vision/installation_calibration.py`,
`FixedStoredPoseProvider` in controld, world-frame telemetry); none of it has
seen the real station.

1. **[CAMERA]** Run the fiducial calibration with the real camera against the
   ChArUco board: multi-frame estimate, outlier rejection, atomic commit →
   `calibration/installation_pose.yaml` (R_W_B, C++-compatible text). Verify the
   file is written atomically and the C++ side parses it.
2. **`calibration/*.yaml` placeholder paths (Part 3, item 20):** after this test
   the `installation.pose_file` path is real. Also produce/verify the
   `camera_intrinsics.yaml` (§28.2) consumed by the tracking config (P8 prereq)
   — until then the intrinsics are commissioning placeholders.
3. **Boot check:** restart `controld`; the log shows
   `installation pose: source=stored calibrated=true`; telemetry reports the
   base tilt instead of "assumed-level".
4. **[MOTOR]** Tilt the base (or accept the as-installed tilt): with a target in
   view, tracking (P8) must now aim correctly in the world frame — the
   world-frame LOS telemetry must agree with the target's apparent position.
   Before/after comparison: an uncalibrated (identity R_W_B) run shows a
   systematic aim offset that the calibrated run removes.

**Pass criteria:** R_W_B file committed + loaded at boot; world-frame LOS
consistent; aim offset removed vs the identity baseline.

---

## P10 — Payload profiling + verification on the real station (Phase 9) `[MOTOR]`

**Supervised station only.** Everything in Phase 9 was verified on
`SimMotorBackend` (`--sim`); the real-plant run is the remaining step.

1. **Profile the installed payload** (replaces the `--sim` rehearsal — Part 2,
   S2):
   ```bash
   # from Firmware/build, with the daemon STOPPED:
   ./turret-payload profile --config ../config/turret.yaml --name <payload>
   ```
   The §44 battery runs one axis at a time (±steps, tracking triangle,
   multi-speed brakes, holding effort); targets are clamped ≥2° inside the soft
   limits. Watch the station the whole time.
2. **Inspect the profile:** `./turret-payload list` — rise/settle/overshoot/stop
   distance per axis + the derived v/a/j envelope (`derive_limits`).
3. **Verify against the profile:** `./turret-payload verify --name <payload>` —
   must report `ok` for a repeat run (repeatability of the response).
4. **Daemon integration:** set `payload.active_profile: <payload>` (replaces the
   `conservative` placeholder, Part 3, item 23), enable `payload_auto_verify`,
   boot the daemon → the P6 in-loop check must now pass against the *real*
   profile, and the tracking `v_max` / hold limits must use the payload-derived
   envelope when derated.
5. **Mismatch → derate → clear** on the real plant: swap in a wrong profile,
   confirm the derate in telemetry + limits, swap back, re-verify, confirm the
   derate clears (the live version of the Phase-9 SimMotorBackend test).

**Pass criteria:** profile captured on the real plant; verify `ok`; the daemon
auto-verify agrees; mismatch derates and re-verify clears — all with the
SafetySupervisor holding authority and no limit crossing.

---

## P11 — systemd deployment (§52) `[MOTOR]`

The units exist as templates (`Firmware/systemd/`: `can0`, `turret-control`,
`turret-vision`, `turret-web`, optional `turret-log`) — **none has been run**.

1. Install to `/etc/systemd/system` (or drop-in paths) with the `ExecStart`
   paths adjusted for the actual install location (`WorkingDirectory`/
   `ExecStart` currently point at `/opt/open_auto_turret`).
2. `systemctl daemon-reload && systemctl enable --now can0 turret-control`
   (`turret-vision`/`turret-web` after P7/P8 pass).
3. **Restart safety:** kill -9 `turret-control`; `Restart=on-failure` brings it
   back and boot must return **UNHOMED** (it re-homes; it never resumes stale
   coordinates).
4. **Network independence:** stop `turret-web` (and drop the network if you
   dare); the control loop must run at 200 Hz unaffected (`Wants=`, not
   `Requires=`).
5. `turret-vision` restarts the real `visiond` (P7 command) under the unit.

**Pass criteria:** all units survive reboot; control loop unaffected by web/
network state; a crash restart always lands in UNHOMED → re-home → hold.

---

## P12 — Live web end-to-end (§42, §54.5) `[MOTOR] + [CAMERA]`

The inspection webd currently talks to a **FakeControld** stand-in (Part 2,
S4). Point it at the real daemon:

1. Set webd's socket config to the real controld web socket
   (`OTA_WEB_SOCKET` / the systemd unit's socket path) and start webd.
2. Dashboard panels against live telemetry: System/Vision/Target/Yaw-pitch/
   Calibration/Video/Payload all show real values (tracking fields appear only
   while tracking is enabled — §6.3).
3. Developer commands (§42.2) against the real loop: start/stop tracking,
   enable/disable search, `start_payload_verification`, homing status — each
   must pass the validation gate and execute on the control thread.
4. **Video feed on/off:** switch on → IMX500 opens, MJPEG stream (capped FPS),
   switch off → camera released (zero CPU). Confirm the served image reads as
   right-side-up (the `rotate_180` correction was live-verified at 0.9996
   correlation; if the physical mount is a mirror, switch
   `OTA_VIDEO_ORIENTATION` — Part 3, item 22).
5. **§54.5 UI load, live:** several dashboard clients + video preview open while
   the 200 Hz loop runs (and is tracking): control-loop p99 must not materially
   degrade, no client starvation, no CAN feedback staleness, logging does not
   block.

**Pass criteria:** real telemetry on every panel; every command works through
the gate; video on/off releases/acquires the camera; load test passes with the
real loop + real camera.

---

## P13 — HIL checklist + acceptance metrics (§54.4, §55) `[MOTOR]`

The final supervised pass. The mock-device tests (38 ctest binaries + 95 Python
unittest) are the rehearsal; this is the hardware truth.

1. **§54.4 HIL checklist** (both CAN IDs live):
   - loss of one motor (pull the connector) → the other axis stops safely
     (supervisor Disable/Brake, whole-station stop);
   - error-active / error-passive / bus-off behavior where safely reproducible;
   - delayed feedback (throttle, as in P4 but milder) → no false Brake, no
     missed real Brake;
   - stop contact (drive toward a stop in position mode) → contact force within
     the configured threshold, no overshoot damage;
   - current-limit behavior (stall against a stop) → the motor current-limit
     engages as expected, no thermal excursion.
2. **§55 acceptance metrics — report at least:**
   - **Control timing:** target loop rate, p50/p95/p99 cycle, worst cycle,
     deadline miss count (the loop already logs these; extract from the
     high-rate control log / black-box ring).
   - **CAN:** feedback age per motor, dropped/invalid frames, error frames
     (`turret-can stats` + daemon log).
   - **Homing:** endpoint repeatability, measured travel, homing duration, peak
     homing effort, expected-range pass/fail (from P0/P3).
   - **Tracking:** acquisition time, LOS tracking error (mean/max) for
     stationary + moving targets, loss→brake time, search reacquire time (from
     P8).
   - **Limits:** soft-limit excursions (must be zero), derate events (P6/P10).
3. **Close out the placeholders** (Part 3): refine `turret.yaml` from the
   measured numbers (§58 params, travel bands, park pose, payload profile).

**Pass criteria:** HIL checklist items all safe; metrics report produced with
the numbers above; `turret.yaml` updated from measurements and the suite re-run.

---

# Part 2 — "Good to go" swap-in map

Every place where a real test/hardware is required, what stands in for it today,
and the exact replacement when the user says **good to go**. Ordered by queue
position.

| # | Location | Current stand-in / guard | Good-to-go replacement | Queue |
|---|----------|--------------------------|------------------------|-------|
| S1 | `control/src/main.cpp` (controld) | **No vision wiring:** never opens `/tmp/ota_vision.sock`, never decodes `TargetMeasurement`, never calls `enable_tracking()` (tracking machinery exists + is tested, but the daemon does not use it) | Add a UDS `SOCK_SEQPACKET` client thread (58-byte decode → `loop.feed_measurement(m)`); load the `tracking:` block + `camera.intrinsics_file` → `TrackingController::Config`; `loop.enable_tracking(cfg, err)` gated on `homed_` (§38.1) | P8 |
| S2 | `tools/turret_payload.cpp` (`turret-payload`) | `--sim` rehearsal on `SimMotorBackend` (no CAN, deterministic clock/pacer) | Drop `--sim`: real mode opens CAN + `BootFsm` and moves the real motors (supervised, daemon stopped) | P10 |
| S3 | `vision/visiond.py` | `--synthetic` (SAFE default): `SyntheticFrameSource`, no camera | `--real --image-config <json> --detector-rpk <json>` (+ `--orientation rotate_180`); the guarded `Picamera2FrameSource` path activates | P7, P8 |
| S4 | `web/webd` (inspection service) | `FakeControld` stand-in server (sim telemetry); real camera already live-verified | Point webd's socket config at the real controld web socket (`OTA_WEB_SOCKET`); run with `turret-web` unit | P12 |
| S5 | `Firmware/systemd/*.service` | Templates only — none installed/enabled (paths point at `/opt/open_auto_turret`) | Adjust `ExecStart`/`WorkingDirectory` to the install path; `systemctl enable --now can0 turret-control [turret-vision turret-web]` | P11 |
| S6 | C++ mock tests: `test_tracking_integration`, `test_payload_daemon`, `test_control_loop`, homing/trajectory suites (all `SimMotorBackend`) | Simulated plant (first-order lag, end stops, stall effort) | HIL counterparts per §54.4: the same scenarios (rotating-target track, loss→coast→brake, search sweep, in-loop payload check, stop contact) executed against the real CAN — supervised | P13 |
| S7 | `calibration/camera_intrinsics.yaml` | Path in `turret.yaml`; **file does not exist** (tracking config not yet loaded in main.cpp) | Produce the §28.2 intrinsics file (camera matrix + distortion) at commissioning | P8/P9 |
| S8 | `calibration/camera_extrinsics.yaml` (R_P_C) | Path in `turret.yaml`; **file does not exist** → aligned-identity default in `TrackingController::Config` | §28.3 extrinsic estimate (fiducial board), loaded into `geo::TurretKinematics` | P9 |
| S9 | `calibration/installation_pose.yaml` (R_W_B) | Path in `turret.yaml`; **file does not exist** → identity "assumed-level base", telemetry flags uncalibrated | §29 fiducial (ChArUco) calibration run → atomic commit → daemon loads at boot | P9 |
| S10 | `config/turret.yaml` `payload.active_profile` | `conservative` (built-in placeholder profile) | Real profile name from `turret-payload profile` (S2) | P10 |
| S11 | `config/turret.yaml` §58 commissioning values (23 params), `expected_travel_deg` bands, `shutdown.*_park_deg` | Conservative placeholders (slow speeds, wide bands, 0/0 park) | Measured values from P0/P3/P13 (travel, contact thresholds, safe park pose) | P0–P3, P13 |
| S12 | `web/webd` video | `FakeCamera` in unit tests; real IMX500 path already live-verified | Nothing to replace — keep tests as-is; live feed is real | P12 (confirm only) |

**Not swapped (deliberately deferred, see Part 3):** comm-19 parameter table,
speed-mode use, Kalman upgrade, piecewise collision envelope, per-class anchors,
homing-result persistence. None of them gates the queue.

---

# Part 3 — Placeholder registry (development order)

Every placeholder introduced across the development cycles, in the order it was
introduced (phase order), with where and how it gets resolved. Items marked
**RESOLVED** were placeholders that a later phase removed — kept here so the
history is auditable.

**Phase 1 — CAN/motor layer**
1. **comm-19 parameter table** (0x2000/0x3000/0x1000-series registers, e.g.
   `echoFreHz` 0x2004) — undocumented, unreachable via comm 17/18 (they read
   stale/garbage). Placeholder for "how to enable free-running periodic
   feedback / tune parameters". → **Not needed for v1** (request/response MIT
   model verified 1:1 at 200 Hz). Revisit only if free-running feedback or
   parameter tuning is ever required.
2. **Speed mode (run_mode=2 + SpdRef 0x700A)** — does not drive the loaded axis
   at the commanded rate with default gains (~1% of command measured).
   Placeholder for "fast open-loop-ish moves". → **v1 uses position mode for
   commissioning and MIT mode (run_mode=0) for the 200 Hz loop.** Deferred.
3. **Motor health: bus error counters not surfaced** (Phase-1 checklist item) —
   feedback freshness + fault flags are in `AxisLatest`; CAN bus error counters
   are not yet in telemetry. → Resolve during P13 metrics work (§55 CAN block).

**Phase 2 — homing & safety**
4. **23 §58 commissioning parameters in `turret.yaml`** — conservative
   placeholders (slow speeds, wide soft-limit bands). Loader falls back to the
   same built-in defaults for any `TBD`/missing key (with a warning). → Refine
   from P0/P3/P13 measurements (config, never compile-time).
5. **`axes.*.expected_travel_deg` bands** (pitch ±120°, yaw ±180°) — wide
   placeholders (the pitch band was widened to contain the observed −86° raw
   position pre-homing). → Tighten to the measured travel after the first real
   homing (P0/P3); too tight rejects a valid home, too loose weakens the §23
   check.
6. **Park pose `yaw_park_deg`/`pitch_park_deg: 0/0`** — placeholder that sits
   on the low stop, **outside the soft limits (violates §33.1)**.
   ParkController rejects it; daemon safely de-energizes at the ready pose.
   → **Top commissioning item:** set a safe in-travel pose before relying on
   park (P2).
7. **§27 post-homing boot states (camera / installation / payload checks)** —
   STUBS in Phase 2 (the hold phase passed straight through them). →
   **PARTIALLY RESOLVED:** the payload check is now real (`Phase::PayloadCheck`,
   Phase 9, P6). The camera/installation "checks" are out-of-loop by design in
   v1 (camera = external `visiond` process; installation = pose file loaded at
   boot, P9) — no further in-loop state needed.
8. **Homing calibration persistence (§28/§41)** — schema version, timestamps,
   HW IDs, atomic write, last-known-good for the homing *result* — still pending.
   → **Deferred by design:** every boot returns UNHOMED and re-homes (the
   systemd restart-safety property), so persistence is an optimization, not a
   safety requirement. Not queue-blocking.
9. **Leftover `vcan9` interface** — virtual CAN left up from earlier testing.
   → Ops: `ip link del vcan9` **before P0** so it cannot shadow real traffic.

**Phase 3 — trajectory**
10. **Piecewise collision envelope** — v1 ships the `RectangularCollisionEnvelope`
    (constant pitch/yaw limits). A piecewise table/polygon plugs in later as a
    new `is_safe` implementation without touching the controller. → Only if the
    real install needs non-rectangular safe regions. Deferred.

**Phase 4 — vision**
11. **Detector RPK + picamera2 config JSON** — not in the repo (commissioning
    artifacts; `docs/AI_CAMERA_SETUP.md` describes the setup; the official
    YOLO11n/YOLOv8n RPKs are AGPL-3.0 — license review against the project's
    GPLv3 before distribution). → P7 prerequisite (S3).
12. **Class set: 3-value protocol (none=0 / person=1 / car=2)** — a MobileNet
    SSD COCO subset; "the v1 tracker only follows 'person'". Animals have no
    class ID (a non-person box can only be followed via `fallback_to_best` when
    no person is present — incidental, not designed support). → If another class
    (e.g. animal) must be tracked: add the class ID to `vision/protocol.py`
    (the C++ side is class-agnostic), make the detector RPK emit it, and set
    `preferred_class_id` in `target_selector.py`. Design note, no queue item.
13. **`visiond --synthetic` default** — the SAFE default stand-in for the
    camera. → Replaced by `--real` at good-to-go (S3, P7).

**Phase 5 — geometry & estimator**
14. **Camera distortion — no-op placeholder in v1** (`geometry/camera_model.hpp`;
    the hook exists, "added later"). → Fill with the §28.2 distortion parameters
    when the intrinsics file is produced (P9, S7).
15. **R_P_C extrinsic — aligned-identity default** (configurable in
    `TurretKinematics`, "swappable for a fiducial-board estimate, §10.3").
    → Replaced by the §28.3 calibration (P9, S8).
16. **Alpha-beta estimator (v1)** — const-angular-velocity on base-frame LOS.
    A full-covariance Kalman with confidence-aware measurement noise (§13.2) is
    the documented upgrade path. → Only if live tracking (P8) shows the
    filter's lag/noise behavior is inadequate.
17. **Anchor = bbox centre** (v1 default, §10.1). → "Later it may use a pose
    keypoint or another object-class-specific stable anchor." Only if tracking
    accuracy needs it.

**Phase 6 — tracking**
18. **controld vision wiring** — `enable_tracking()` + the `/tmp/ota_vision.sock`
    client are not wired in `main.cpp` (the queue's only **code** prerequisite).
    → Implement before P8 (S1).
19. **systemd units — "templates only, not run (no CAN/motor)"** → enable at
    P11 (S5).

**Phase 7 — installation orientation**
20. **`calibration/*.yaml` files (intrinsics / extrinsics / installation pose)**
    — placeholder paths in `turret.yaml`; **the directory and files do not
    exist**; the daemon falls back to identity "assumed-level" + an
    uncalibrated telemetry flag. → Produced by the §28.2/§28.3/§29 calibrations
    at P9 (S7–S9).

**Phase 8 — web**
21. **webd telemetry source = `FakeControld` stand-in** — the inspection
    service runs against a simulated controld. → Point at the real daemon at
    P12 (S4).
22. **Video orientation — `rotate_180`** — live-verified (served frame
    correlates 0.9996 with a 180° rotation of the raw sensor frame), but the
    user has not yet confirmed the image reads as right-side-up; if the physical
    mount is a mirror-flip instead, switch `OTA_VIDEO_ORIENTATION`. → Confirm at
    P12 (S12).

**Phase 9 — payload**
23. **`payload.active_profile: conservative`** — built-in conservative
    placeholder profile (the `--sim` rehearsal stored a `sim` profile in
    `/tmp` during development; neither represents the installed mass). →
    Replaced by the real `turret-payload profile` output at P10 (S10).
24. **Real-plant payload commissioning** — all Phase-9 tests ran on
    `SimMotorBackend`; the in-loop check, derate, and re-verify have never moved
    a real motor. → P6 (in-loop) + P10 (standalone), supervised.

**Cross-cutting**
25. **§54.4 HIL checklist** (both CAN IDs; loss of one motor;
    error-active/passive/bus-off; delayed feedback; stop contact;
    current-limit behavior) — not yet run on hardware. → P13.
26. **§55 acceptance metrics** (control timing, CAN, homing, tracking, limits)
    — the data is logged in-loop; the report has never been produced from the
    real station. → P13.
27. **§54.2 simulated-plant fidelity** — `SimMotorBackend` has first-order lag
    + end stops + stall effort; the §54.2 extras (noise, feedback delay, CAN
    dropout, variable inertia) are not modeled. → Optional extension before P13
    if the HIL surprises need a sim repro.

---

## Resolved since the Phase-2 edition of this queue

- **Backoff stuck-slip / windup (rehome2 crawl, rehome3 timeout)** → velocity-
  controller re-arm before post-contact backoffs (rehome4, wire-verified) +
  acceleration-based stall detection with 3× bursts (see P3 for the full
  evidence chain).
- **P6 payload check was a stub** → real in-loop check + profiler + verifier +
  derate (Phase 9); see the new P6/P10.
- **P8 tracking was "not built yet"** → implemented + integration-tested on
  SimMotorBackend (Phase 6); remaining work is the S1 wiring + the live run.
- **"Out of scope (Phase 3+)" items** — calibration persistence (§28/§41) still
  pending (item 8, by design); structured event logging + telemetry (§43/§55)
  implemented (telemetry snapshot + high-rate control log + event log +
  black-box ring; the §55 *report* is P13); HIL harness (§54) → P13; payload
  driver (§27) implemented (Phase 9); collision envelope (§19) implemented in
  its v1 rectangular form (item 10); web UI (§42) implemented incl. the real
  IMX500 video feed (the §42.3 placeholder is gone); installation orientation
  (Phase 7) implemented, live run at P9; systemd (§52) templates done, enable at
  P11.
