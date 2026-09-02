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

**Result (p3, 2026-09-02 09:02 — first live verification of the speed-mode
park; park INCOMPLETE again, NEW root cause):** SIGTERM (09:02:26) → 40 s
window → `de-energized (phase=parking, fault='')` 09:03:07, no `PARKED`.
The speed-mode moves now run at command rate (no 0.07°/s crawl), but:
- **Velocity-mode point-stop overshoot:** the yaw park move (176.6° → 176°,
  0.6°) overshot to **175.16° — 0.84° PAST the target**. The drive's
  velocity loop has ~0.1–0.3 s of step-response lag (+ integral windup):
  every speed-mode point stop lands ~1° x the arrival speed past the target.
- **Verify then could never correct it:** the Verify/Dwell hold carried
  `LimitSpd=0`. The CyberGear position loop is **pinned at LimitSpd=0** —
  the yaw sat at the overshoot point (175.16°) for the full 39 s of
  Verify/Dwell, 0.84° outside the 0.5° window, and the 40 s park window
  expired. Final de-energized pose: pitch −1.4987 (40.15°) / yaw −2.28845
  (175.16°).

**Result (p3b, 2026-09-02 09:04 — boot after p3; HOMING FAILED):** boot
09:04:56 → `control fault: homing failed: yaw move to 180.000000 deg failed:
move timeout` 09:07:24. Wire capture (candump, /tmp/p3_cand.log) of the
failed `move yaw 180` plan action:
- The 180° climb from the low stop (0°) ran at 10°/s; the decel profile
  arrives at ~9°/s; the velocity-loop lag overshoots **1.2° to 181.2°**
  (5.2° past the 176° gravity balance).
- The return command (−8.3°/s = −0.1450 rad/s) then produced only
  **~0.1 N·m** (wire: tq −0.02…−0.22 N·m, v≈0, axis pinned at 181.2°):
  P-term at 0.145 rad/s error ≈ 0.055 N·m + the drive's tiny `spd_ki`
  building integral slowly. That is below the breakaway static friction at
  5.2° from balance → the move could not reverse → 30 s MoveTo timeout →
  fault. (rehome1–4 ran the same 0°→180° move successfully four times —
  the breakaway at 181.2° is marginal and run-to-run dependent; p3b lost
  the coin flip.)
- **The homing plan's mid-park was still at 180°** — the same no-dangle
  pose error already fixed for shutdown parking (`move yaw 180 ... mid-
  travel of the ~360 deg yaw`).
- Post-fault observation: the fault phase holds via SpdRef=0 (speed mode) /
  re-pinned position hold; the pitch crept +2.7° (43.2°→45.9° in 76 s, then
  decayed) = the drive velocity-loop integral wound up by the pitch's
  +climb slowly discharging against SpdRef=0. The yaw held 181.2° solid
  (static friction). De-energized at shutdown (SIGTERM, 09:22:09):
  `de-energized (phase=fault, ...)` + `controld stopped cleanly`.
- **No-slide check at the de-energized fault pose (2.5 min, 09:27–09:30):**
  pitch 45.81° (range 0.008°), yaw 181.20° (range 0.0115°) → **static
  friction holds even 5.2° past the yaw balance** (the p3b stick was
  genuinely below breakaway, not a marginal hold).

**Fix (implemented 2026-09-02, pending live verification — round 2):**
4. **Homing plan mid-park `move yaw 180` → `move yaw 176`** (config): the
   gravity balance = the true no-dangle pose. The 176° move is torque-free
   and any overshoot (~1.2° → 177.2°, 1.2° from balance) settles back on
   its own; the breakaway requirement is smallest at the balance, so the
   mid-park can no longer stick (the p3b failure mode).
5. **Verify/Dwell hold carries a NON-ZERO speed limit**
   (`shutdown.verify_speed_deg_s: 2.0`, new; wired through
   ShutdownConfig → ParkParams → executor, validated > 0 in config load):
   the position-mode hold now enters and runs at 2°/s LimitSpd, so the
   drive's position loop CAN pull a speed-move overshoot back into the
   0.5° window (a ~0.8° residual corrects in one pass: 0.4 s travel +
   0.13 s lag → ~0.26° past, inside the window). 2°/s is deliberately low:
   at 10°/s the position loop would overshoot a correction by ~1.3°
   (speed x 0.13 s velocity-lag) and limit-cycle outside the window.
ctest 38/38 + pytest 93 passed/2 skipped green on the round-2 code.

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

**Result (p3b / "rehome5" attempt, 2026-09-02 09:04, FAIL — homing fault,
root-caused from the CAN wire capture):** the boot after the failed p3 park
failed inside homing itself: `homing failed: yaw move to 180.000000 deg
failed: move timeout` (09:07:24, 30 s MoveTo timeout on the plan's `move
yaw 180` no-dangle action). Full wire-level diagnosis is in P2 (velocity-
loop stop overshoot 1.2° to 181.2°; return command at ~0.1 N·m below
breakaway static friction 5.2° from the 176° balance; the plan's mid-park
still at the wrong 180° pose). Fix: mid-park moved to 176° (P2 fix 4) + the
Verify-hold LimitSpd fix (P2 fix 5); the re-test (fresh boot → home → ready
→ SIGTERM → PARKED) is the re-run of both P2 and P3.

**Result (p3c, 2026-09-02 09:34, FAIL — homing fault, root-caused; fix
implemented, re-test = p3d):** fresh boot → homing fault at the yaw
endpoint-B backoff: `homing failed: yaw home full range failed: endpoint B
homing failed: backoff: timeout (stuck in friction? q=-5.248531
target=-5.258541 cmd_v=-0.174533 rad/s)` (10 s backoff timeout, 09:34:38).
The earlier p3 run (08:55) succeeded through the identical sequence — the
outcome was luck-dependent (see the p3c root-cause subsection below). Fix
implemented: position-mode backoff; the re-test (p3d: fresh boot → home →
ready → SIGTERM → PARKED → no-slide) doubles as the P2 park re-verification.

**Result (p3d, 2026-09-02 10:19, FAIL — homing fault; root-caused, fix
implemented, re-test = p3e → p3f):** fresh boot → homing fault at the pitch
endpoint-A coarse backoff: `homing failed: pitch home full range failed:
endpoint A homing failed: backoff: timeout (position mode, stuck in friction?
q=-0.820363 target=-0.897712 dq=0.077348 v=0.000584 rad)` (10 s timeout,
10:19:07). The axis moved only 0.13° in the whole window and the drive
commanded tq ≈ 0 (744 of 979 samples within ±0.1 N·m, max |tq| 0.5 N·m) — the
drive's position loop was not commanding at all. First hypothesis (a dropped
fire-and-forget recipe frame) was refuted by p3e (below); the TRUE root cause
is a control-loop bug: the step-6 command stage stomps the homing backoff
reference every cycle. Fix: publish the backoff reference into the phase
`q_ref/lim` (step 6 Allow becomes a no-op) + feedback keepalive for
speed-mode axes on Allow cycles (kills the Brake/Allow flap); the read-back
verify + retry + 15 s timeout are kept as defense-in-depth (see the p3d/p3e
subsection below).

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
(Superseded by the position-mode backoff below, which removes the residual
entirely.)

### p3c failure + position-mode backoff fix (2026-09-02)

**Failure:** p3c (09:34) faulted at the yaw endpoint-B backoff with a 10 s
timeout; the identical p3 run (08:55) had passed. Same drive, same pose, same
code — the difference was stick-slip luck.

**Root cause (from the 100 Hz motion log, `/tmp/controld_p3c.log`):** the
speed-mode backoff is a friction fight the velocity loop cannot win:

1. Contact at 09:34:27.9 (tq −0.64), 1.5 s dwell pushing on the stop; the
   velocity-loop integral winds to ≥1.3 N·m at the 1 A limit during the dwell
   — so 1 A is NOT the torque bottleneck, the drive can output it.
2. Re-arm at 09:34:28.185 (supervisor BRAKE, overrun 95 ms — the one blocking
   cycle). While de-energized the arm slides +3.2° off the low stop toward the
   176° gravity balance.
3. The +10°/s backoff (away from the low stop) starts 3.2° from it and
   immediately stalls ~1.9° short of the 5° target: P only (0.066 N·m at the
   10°/s error) + I at 0.0285 N·m/s build cannot reach breakaway static
   friction (~0.4–0.55 N·m, toward the balance point). 100 Hz log:
   `cmd=+0.1745`, `tq=+0.27 N·m`, `v≈0.0003`, q frozen at −5.2913 for ~4 s.
4. The stall detector's 3× burst finally slips the joint, but the burst
   momentum overshoots the 5° target to 6.6° (−5.2298; 1 Hz a_yaw −0.83
   rad/s² slip decel).
5. The −10°/s return from 6.6° must re-fight F_s + gravity + the wound-up
   integral (tq peaks −0.348, below breakaway) → creep at 0.036°/s → the axis
   is still ~0.6° short of the target (q −5.2485) when the 10 s backoff
   timeout fires at 09:34:38.1 → `control fault`.

   (Correction of record: an earlier reading of this log reported a
   "5-second motion-log gap" during the stall. There was no gap — a parser
   regex that only matched negative `cmd=` values, plus line-dedup, collapsed
   the continuous 99-line/s stall region. The yaw log is continuous through
   the whole event; per-second line counts confirm it.)

**Why p3 passed and p3c failed:** identical dynamics, different stick-slip
dice. p3's burst overshoot was 0.94° and its return creep 0.15°/s (made it
home in ~3.2 s); p3c's was 1.6° at 0.036°/s (timed out at 10 s). Any margin
that depends on creep speed is not a design.

**Fix — backoff moves run in the drive's position mode** (implemented in
`homing_controller.{hpp,cpp}`, `control_loop.cpp`, `can_motor_backend.cpp`):

- The coarse 5° and small 2° backoffs now command `LocRef` = the fixed
  backoff target with `LimitSpd = 10°/s` (a speed cap, never 0 — 0 pins the
  position loop). The position loop (loc_kp ≈ 30 N·m/rad) wants `30 × err`
  N·m from the FIRST cycle — a 5° error wants 2.6 N·m, clamped at the 1 A
  limit — so it breaks static friction immediately; there is no integral to
  wind up and no bursts (P ∝ error, so overshoot self-corrects).
- Mode sequence per endpoint: contact dwell (speed mode) →
  `enter_position_mode` (one-shot, blocking CAN recipe: stop → RunMode=1 →
  enable → LimitSpd=10°/s → pin LocRef to the just-read MechPos; the
  de-energize also resets the wound-up velocity integral — the rehome3 class
  of problem) → position backoff → settle (position mode HOLDS the pinned
  LocRef, no re-pinning) → re-arm `enter_speed_mode` (de-energize → RunMode=2
  → re-energize; LimitCur persists) → fine approach (speed mode, as before)
  → [contact 2 → small position backoff → re-arm → fine approach 2].
- Arrival uses a wide window because the position loop can stall 0.5–1.5°
  short of the target in the detent zone (P < F_s there):
  `arrive_tol = max(0.5°, 0.4 × backoff distance)` → the coarse backoff
  arrives ≥3° clear of the stop, the small ≥1.2°. The fine approach is
  "move until contact" anyway — it re-measures the stop (endpoint
  repeatability is 0.05°, so a de-energize slide before it is benign).
- A 10 s timeout on a position-mode backoff now means genuinely stuck
  (mechanical), not "velocity loop lost the friction fight"; the fault
  message says so.

Unit tests: the `test_homing`/`test_homing_plan` sim plants and
`SimMotorBackend` model the position-mode branch (drive toward LocRef at the
speed cap, full torque from the first cycle); ctest 38/38 + pytest 93/2.

### p3d/p3e failure + root cause + fix (2026-09-02)

**Failure:** p3d (10:19:07) faulted at the pitch endpoint-A coarse backoff —
the first live run of the position-mode design. 100 Hz log (`/tmp/controld_p3d.log`):
979 `move:pos` samples over 10.0 s, q −0.8104 → −0.8204 (0.13°: a residual
dwell coast for 106 ms, then a 0.04° creep), tq distribution 1 cycle > +0.5
N·m, 744 within ±0.1 N·m, 234 in −0.5…−0.1 N·m, then `backoff: timeout`.
A working position loop with a 5° error wants `loc_kp × err = 30 × 0.0873 =
2.6 N·m` (clamped at the 1…3 A limit); the drive never commanded more than
0.5 N·m — at the time this read as "the position loop was not active";
p3e's wire capture showed the loop WAS active but was commanded to hold in
place on every other frame (root cause below).

**Root cause (quiet-bus experiments, no daemon, full wire capture):**

1. The first reproduction attempt was contaminated: the p3d daemon was still
   running in its fault phase, and its fault hold loop
   (`control_loop.cpp` `Phase::Fault`: at rest `q_ref = q`, `lim = 0`;
   `command()` writes LocRef + LimitSpd write-on-change) was concurrently
   writing `LimitSpd = 0` / `0.087266` and re-pinning LocRef on the same bus
   — it *looked* like "the drive ignores LimitSpd writes". With the daemon
   killed and the bus quiet, the recipe works.
2. Quiet-bus ground truth (pitch, 3/3 runs, one of them the exact p3d
   condition — hot push into the +stop, 1.5 s dwell, recipe immediately
   after the stop command): the recipe's LimitSpd readback verifies
   (0.174533), the LocRef pin sticks, and a 5° LocRef step is tracked with
   ~3.5–4 s of breakaway creep against static friction, then an
   accelerating burst (7+°/s) that arrives ~4.5 s after the step,
   overshoots ~0.3–0.4°, and damps out slowly (still converging at +10 s).
   The post-fault register readout seen after p3d (`run_mode=1,
   limit_spd=0, loc_ref=mechPos`) is the fault-phase hold's own writes —
   not evidence about what the backoff programmed.
3. Every recipe frame is fire-and-forget: `write_reg_float` / `write_reg_u8`
   / `send_enable` are plain bus sends — the CyberGear does not ACK
   register writes on the wire, so a dropped frame is invisible. p3d's
   signature (tq ≈ 0, no tracking, LocRef register still accepting writes)
   looked exactly like a lost `enable` or a lost `RunMode=1`. **Hypothesis,
   later refuted by p3e (see below) — kept here for the record.**

**Fix v1 (f999d5f, `can_motor_backend.cpp`):** both mode recipes are now
verified live — after the recipe, the mode-defining registers are read back
(`enter_position_mode`: RunMode == 1, LimitSpd == written, LocRef == pinned;
`enter_speed_mode`: RunMode == 2, LimitCur == written, SpdRef == 0) and the
whole recipe is re-run on any mismatch, up to 3 attempts, after which the
call fails with `recipe verify failed` (a hard fault, never a silent
degraded mode). Cost: 3 register reads (~50 ms) per recipe; the recipes are
blocking setup calls (homing transitions, re-arms, hold/park entries), so
the cost is one longer supervisor cycle at most (the re-arms already take
~105–200 ms). The backoff hard timeout also moved 10 → 15 s: the measured
drive worst case is ~5 s (4.5 s arrival + 0.5 s settle), and 15 s keeps ≥2×
margin over a bad-luck breakaway while still catching a genuinely stuck
drive quickly.

**p3e (2026-09-02 10:51, FAIL — identical signature, hypothesis refuted):**
fresh boot → homing fault at the SAME pitch endpoint-A backoff, now 15 s:
`backoff: timeout (position mode, stuck in friction? q=-0.821126
target=-0.897712 dq=0.076585 v=0.000547 rad)`. The recipe VERIFY PASSED
(RunMode/LimitSpd/LocRef readback all correct — no `verify failed` log),
which a dropped recipe frame cannot explain.

**TRUE root cause (p3e wire capture, 29 127 frames decoded,
`/tmp/p3e_wire.txt`):** the control loop fights itself. In `Phase::Homing`
the homing handler issues the backoff command DIRECTLY (`command(target,
10°/s)`), but it never publishes it into the phase `q_ref[]`/`lim[]` arrays
that the common step-6 command stage uses. Step 6 re-commands every
position-mode axis from `q_ref/lim` on EVERY cycle — so on each Allow cycle
it wrote the DEFAULT hold (current position @ LimitSpd 0) immediately after
the handler's target write. Decoded pitch command pattern over the full 15 s
backoff: **`A C A C A C …` exactly 1:1 (2944 backoff frames, 2944 hold
frames, only 2 brake frames)** — the drive's position loop saw the target
flip between "backoff target @ 10°/s" and "hold in place @ 0" at 100 Hz and
could never build the sustained torque to break static friction.
Deterministic failure: p3d (10 s) and p3e (15 s) are the same bug at two
timeout settings. The unit tests could not catch it: the sim plant has no
static friction, so a 50 %-duty stomp still creeps to arrival.

Secondary finding — the carried-over **fault-phase BRAKE/ALLOW flap** (and
the p0p hold-phase flap): the wire shows `B C B C …` 1:1 (449/450) in the
fault phase, where B = the Brake override (emergency target @ 5°/s) and C =
the Allow hold. The CyberGear has NO periodic telemetry (it answers
commands only), and step 6 only issues a speed-mode command (the only thing
that pings for feedback) on non-Allow cycles. So an idle speed-mode axis
(= yaw, still in speed mode at fault time) ages past
`feedback_max_age_ms` = 100 ms after ~5 quiet cycles → supervisor BRAKE →
the brake pings the yaw but stomps the OTHER axis's reference with the
emergency target → next cycle Allow → the yaw ages out again → … A
self-sustaining ~100 ms flap that also slowly creeps the fault hold toward
the stop (the B stomp commands 5°/s toward the emergency target). During
homing the flap does not occur — the homing handler pings BOTH axes every
cycle — which is why the p3e log shows the supervisor silent through the
backoff (the stomp happened on Allow cycles, which are not logged).

**Fix v2 (implemented, `control_loop.cpp` + backend `keepalive`):**
1. Homing phase: the position-move branch now publishes
   `q_ref[ix(a)] = target; lim[ix(a)] = speed` — step 6's Allow cycle
   becomes a write-on-change no-op (zero extra CAN traffic), while a
   Brake/Hold/Derate cycle still overrides the reference (safety authority
   preserved: a genuine stale-feedback brake during a backoff still stops
   the arm — P4 depends on this).
2. Step 6: a speed-mode axis on an Allow cycle now gets `keepalive()`
   (new `MotorBackend` method; CAN impl = a rate-limited same-value LimitCur
   rewrite — inert, no reference change — that elicits a feedback response,
   same ping the `command`/`command_velocity` paths already used) so the
   feedback age stays ~40 ms and the supervisor can no longer flap.
3. Fix v1 (verify + retry, 15 s timeout) retained as defense-in-depth
   against genuine fire-and-forget drops.

ctest 38/38 + pytest 93/2 after the change.

**Live re-test p3f (2026-09-02 11:08:32, daemon 207807, cb0354a) — backoff
fix VERIFIED, new failure at the repeatability check:**

- **The p3d/p3e root cause is fixed live.** The 5° position-mode backoff
  MOVED: q −0.813 → −0.899 in **~4.4 s** (min −0.918, ~0.3° overshoot, then
  damped settle at −0.89895), no 15 s timeout, no `verify failed`, no
  A/C stomp — matching the quiet-bus ground truth (breakaway ~3.5–4 s +
  burst). Wire capture (`/tmp/p3f_candump.log`, `candump -ta`) covers the
  whole run for the no-stomp confirmation.
- **New failure — `repeatability exceeded: |q1 - q2| over the limit`** at
  11:08:50 (`/tmp/controld_p3f.log`, 1683 motion rows):
  - coarse contact 1 @ −0.81121 (tq +1.016 N·m) → 5° backoff → settle
    −0.89895 (min −0.91802);
  - first fine approach (from −0.91535, 6° travel at 15°/s): stick-slip
    stall ~1.2 s (~74.5°), burst, mini-stall 0.4 s (~76.4°), burst, contact
    @ −0.81121 (tq 0.94) — **q1, the same position as the coarse contact**;
  - small backoff → −0.83219 (only 1.18° of the 2° target — at the edge of
    the 0.8° arrival window, an open question of its own);
  - **second fine approach (1.2° of travel, starting mid-zone): CREEPT
    −0.832 → −0.822 over 1.87 s (v 0.001–0.036 rad/s, tq 0.1–0.44 N·m
    oscillating — never broke away)** → the contact detector latched the
    stall as a false contact → rep = |−0.822 − (−0.8112)| = **0.63° > 0.5°**
    → fault.
- **Root cause: the drive's static-friction zone at 75–79° pitch** (q ≈
  −0.846…−0.811). In speed mode the velocity loop's P-term at the full
  15°/s (0.2618 rad/s) error is ~0.26 N·m (Kp 0.38 N·m/(rad/s)) — below the
  ~0.44+ N·m breakaway, and spd_ki = 0.002 winds up too slowly → stall with
  tq oscillating 0.1–0.44 N·m (exactly the measured P-term + slow I).
  Breakaway is STOCHASTIC: the first fine approach needed two breakaways
  (1.2 s + 0.4 s stalls) to cross the zone; the second (1.2° of travel,
  starting mid-zone) never broke free in its 1.87 s. The detector's latch is
  not a detector bug: every gate passed, including
  `very_high_effort` 0.44 > 0.40 N·m — the stall plateau is
  **indistinguishable from a true stop plateau** (0.43–0.73 N·m measured at
  the real stop), and the creep never exceeded `v_move_threshold` (0.10) so
  no jitter "recovery" was counted.
- **Not a regression from cb0354a:** p3d/p3e never reached the fine-approach
  phases (they died at the backoff); the second approach's SpdRef comes from
  the homing handler's `command_velocity` (unchanged); and the first full
  success (p3, 08:55) shows this phase CAN pass — it is luck-dependent on
  the stochastic breakaways.
- **Fix: retry the repeatability pass.** On `rep > repeatability_rad`, the
  (small backoff + second approach) pass is re-run up to
  `repeatability_retries` times (default 2, config
  `homing.contact.repeatability_retries`) before faulting. q1 stays the
  reference; each retry re-runs the small backoff (position mode, fresh 15 s
  timeout) and the second approach. Rationale: the repeatability check
  remains the safety authority (a genuinely non-repeatable stop still faults
  after the retries), and the retries exploit the stochastic breakaway — the
  same reason the first fine approach got through on its second breakaway.
  Rejected: retuning the detector (the stall IS indistinguishable at
  0.40 N·m — that threshold is safety-critical), raising spd_ki (global drive
  behavior change affecting payload/hold/park), enlarging the small backoff
  (parameter fiddling; the retry is more general).

**Live re-test p3g** (fresh boot → home → ready → SIGTERM → PARKED, doubles
as the P2 park re-verification) on the retry fix: homing must succeed — the
second approach may now show an extra small-backoff/settle cycle in the 100
Hz log (a retry); if the retries exhaust, the fault message now reads
`repeatability exceeded: |q1 - q2|=<rep> deg > <limit> deg after <N>
retries`. The backoffs must remain A-only on the wire (no A/C stomp —
cb0354a), and the run must end `PARKED (motors de-energized at the park
pose)` at yaw 176° (raw ≈ −2.2739) / pitch 40° (raw ≈ −1.5009) with 10+ min
no-slide.

**Live re-test p3g RESULT (2026-09-02 11:32:01, daemon 210497, 3750e7e) —
homing SUCCESS (no fault), park verified, no-slide PASS:**

- **Homing succeeded end-to-end** (11:32:01 → `homing complete` 11:33:53 →
  ready-pose hold → SIGTERM → `PARKED` 11:46:31), **no `control fault`**. The pitch
  endpoint-A repeatability check — the friction-zone case that faulted in p3f —
  **passed this time, without using a retry** (the stochastic breakaway worked; the
  retry fix remains the safety net for the p3f stall case).
- **Pitch endpoint-A sequence** (`/tmp/controld_p3g.log`, 100 Hz):
  - coarse contact @ −0.81083 (tq +1.214) → 5° position-mode backoff **~4.25 s**
    (−0.81083 → −0.89284) → settle;
  - first fine approach (from −0.89437, 6° at 15°/s): stick-slip, broke away,
    contact @ −0.81121 — **q1**;
  - small backoff → −0.83219 (1.18° of the 2° target, again at the edge of the
    0.8° arrival window — the open p3f question reproduces);
  - **second fine approach (1.2° of travel, starting mid-zone): CREEPT
    −0.83219 → −0.81121 over 4.0 s (v 0.002–0.036 rad/s, tq 0.13–0.99 N·m)
    and REACHED THE STOP** — unlike p3f, where the same creep stalled at −0.822 and
    latched a false contact; contact @ −0.81159 — **q2**;
  - rep = |−0.81159 − (−0.81121)| ≈ **0.02° < 0.5°** → pass, `endpoint A homed`
    at 11:32:22. Exactly one small backoff + one second approach on the wire (no
    extra retry cycle) — confirming the retry path was NOT triggered.
- **Wire: A-only backoffs confirmed** (`/tmp/p3g_candump.log`, `candump -ta`).
  The pitch endpoint-A backoff shows the clean pattern: enter position mode →
  `lspd=0.174533` + `loc=−0.898093` (target) → **~4.2 s of silence**
  (write-on-change; the drive moves on its own, breakaway included) → at arrival
  `lspd=0` + `loc=current` (post-arrival hold) → exit to speed mode. **No A/C stomp
  during any move** — the p3d/p3e fix (cb0354a) held across the whole run (all pitch
  homing backoffs clean; the only A/Z `limit_spd` runs are the post-arrival holds
  and the ready-pose chatter below).
- **SIGTERM → PARKED cleanly** 11:46:31: `shutdown requested; parking` →
  `PARKED (motors de-energized at the park pose)` → `controld stopped cleanly`.
  Wire: park move run=2 (speed) → run=1 (position settle/dwell) → final hold
  commands (`loc_ref` = park pose) → de-energize (power-safe hold at the balance
  point). Transient one-cycle BRAKE/DERATE during the 9 s park move are expected
  (feedback stopping + one 206 ms cycle) and recovered to ALLOW.
- **Park pose (sample 1, 11:46:59, `turret-can read 0x7019`):** pitch −1.49791
  (40.2°), yaw −2.279 (175.9°) — within 0.2–0.3° of the targets (pitch −1.5009 /
  40.0°, yaw −2.2739 / 176.0°).
- **No-slide check PASS (doubles as the P2 park re-verification):** sample 2 at
  11:57:17 (10.3 min later, motors de-energized, arm undisturbed): pitch −1.49781
  (Δ **0.006°**), yaw −2.2788 (Δ **0.012°**). No measurable slide — the power-safe
  hold at the balance point is stable over 10+ min.
- **Secondary observation (not blocking, logged for follow-up):** the
  move-to-ready-pose (hold phase) took **~20 s for a 40° move** (vs ~4 s at 10°/s).
  The wire shows a `limit_spd` A/Z alternation (0.174533/0) in that window — the
  hold phase toggling between "move" (outside the 0.57° `kReadyPosTolRad`) and
  "hold" (inside) as the axis settles. The move completed and the hold is stable
  (no drift); a minor inefficiency, possibly related to position-control settling /
  the inactive-axis windup open item. Distinct from (and not a return of) the p3d/p3e
  friction backoff stomp, which is confirmed gone.

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

**Live test p4 RESULT (2026-09-02, daemon 213494, 3750e7e) — PASS:**

- **Method used:** the preferred (least-intrusive) — a saturating CAN flood:
  `timeout 3 cangen can0 -I 300 -L 8 -g 0 -p 50` (unrelated standard ID 0x300,
  max rate with ENobufs polling) while the daemon held at the ready pose.
  (Note: bare `-g 0` fails with `No buffer space available` — the tx buffer
  overflows at max rate; the `-p <ms>` poll option is what sustains the
  saturation. A partial flood, e.g. `-g 0.5` ≈ 26% of the bus, only reached
  `DERATE reason='control-loop cycle overrun'` — not enough to age the
  feedback past the 100 ms threshold.)
- **Wire (candump -ta):** in the 3 s flood window, 9302 flood frames were on the
  bus while drive feedback frames (0x02806400/0x02806500) collapsed to **3**
  (vs ~39–44/s immediately before and after) — the feedback was genuinely aged
  out. The daemon's own command frames still got through (11763 in the window),
  so it could issue the Brake hold.
- **Daemon response:** one `supervisor: BRAKE reason='stale or missing motor
  feedback'` at 12:10:35.8 (flood start) → held → one `supervisor: ALLOW
  reason='ok'` at 12:10:38.8 (flood end + ~0.8 s). The arm stayed locked at the
  ready pose the whole window (q_pitch=−1.4960, q_yaw=−2.2608, a≈0).
  **Zero `control fault:` lines.**
- **Pass criteria met:** safe stop on stale feedback (no open-loop, no motion
  into a limit — the arm was at rest at the balance-point ready pose and stayed
  there); back to `hold` on feedback recovery with no manual reset; no
  `control fault:` (not mis-classified as a Disable).
- **Recovery:** the Brake is a sustained safe-hold for the duration of the stale
  feedback, then auto-resumes to Allow/hold — recoverable, as designed. SIGTERM
  afterwards → clean `PARKED`.
- **Secondary observation (pre-existing, NOT flood-induced):** throughout homing
  and the ready-pose hold — before any flood — the supervisor showed a recurring
  single-cycle `BRAKE reason='stale or missing motor feedback'` alternating with
  `ALLOW reason='ok'` (p3g showed the same: 28 such BRAKEs at `overrun_us`
  ≈ 97.5–99.2 ms, just under the 100 ms threshold). It is recoverable (one cycle
  each, no fault), but it is the likely cause of the slow move-to-ready-pose
  crawl noted in p3g — the feedback sits just under the stale threshold during
  position-mode moves, flapping Brake/Allow. Flagged for follow-up.

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

**Live run log (in progress):**

- [x] **Safe-region fix.** The check region is now PER-AXIS, centered on each
  axis's pose at check start and intersected with the homed soft limits (§44
  "safe central region"), so a check from the ready pose is valid (it previously
  failed "start pose outside the safe central region" because the region was a
  shared static 0 ± 20 deg box).
- [x] **Unit tests.** `test_payload_check.cpp` (settle band, region guards,
  per-segment timing) + `test_payload_daemon.cpp` — including the failure-swallow
  regression: an abort must log the *failing* axis's own segment reason (not the
  next axis's) and stop at the first move's 8 s budget (not 8 s + the next axis's
  battery). Off-center ready pose and first-axis-fail-aborts-whole-check covered.
- [x] **Integration.** 39/39 ctest + 93 pytest green.
- [x] **Run A (live) — partial pass, residual root-caused.** Verified live: the
  failure-swallow fix (abort logs the failing axis reason), the inactive-axis
  (yaw) fixed-target hold (held ±0.0003 rad over ~11 s), and the 5 % settle band
  on the +step. **Residual:** the `pos_return` (against-gravity) half timed out at
  8 s.
- [x] **Root cause (Run A residual).** The drive's stock inner **speed-loop gains
  are too weak to hold the commanded 5°/s against the pitch's gravity load**:
  `spd_kp=1.0` / `spd_ki=0.002` (drive defaults). The position loop commands
  0.0873 rad/s (clamped by `LimitSpd`), but the axis only creeps ~0.02 rad/s on
  ~0.05 A — never torque-limited, so it was NOT a current-limit problem (5 A was
  plenty). The with-gravity half of the 2° step is assisted (fast burst); the
  against-gravity half crawls and times out. Gravity-neutral pitch ≈ −1.4620 rad;
  yaw is a vertical axis (no gravity torque, only friction) so it is unaffected.
- [x] **Fix (firmware, reproducible).** `MotorBackend::set_speed_loop_gains()`
  (CAN writes `SpdKp` 0x701F / `SpdKi` 0x7020; **no-op in sim**, whose plant has no
  drive-internal velocity loop) + config knobs `payload.check_spd_kp` /
  `check_spd_ki` (5.0 / 0.02 = 5× stock, with validation/clamping), applied to
  **both axes at check start**. 4 new unit tests (config default/yaml/invalid-clamp
  + a daemon test asserting the loop issues the gains on both axes). Full suite
  green (39 ctest, 93 pytest). This replaces the earlier one-off manual CAN writes
  (which are lost on a drive power-cycle) with a firmware-owned, reproducible
  write.
- [x] **RESOLVED (2026-09-02) — fault reproduced, root cause
  **unresolved-external**; re-validation gated on station hardware.** Live
  diagnosis (15:21–16:10 NZ, direct MCP2515 register access over SPI,
  corrected per architect review) — **full evidence:
  `docs/can_hardware_fault_report.md` (v2)**:
  - **MCP2515 core functional** (kernel loopback TX/RX 5/5 zero errors;
    corrected raw loopback passes; SPI, oscillator, IRQ, driver healthy).
  - **An intermittent EXTERNAL signal on CANH/CANL drives real receive
    errors**: MERRF/ERRIF storms (≥20–100/s, scaling with receiver bitrate),
    REC pinned ≥128 → ERROR-PASSIVE, which sticks until a reset lands in a
    quiet window. No valid frame decodes at any bitrate (1M→100k sweep).
    The earlier "internally fabricated phantom frame" was a probe
    misinterpretation (CANINTF 0xA0 = MERRF+ERRIF, not RX1IF+WAKIF; RXB1
    content was stale — report §3).
  - **Source not discriminable remotely** — HAT transceiver/RX-path damage,
    a faulted CyberGear transceiver, wiring/termination, and electrical
    disturbance all remain candidates. Do NOT classify the HAT as faulty
    before the isolation ladder.
  - **Drives silent** (no discovery response, no ACK) — power-cycle needed.
  - **`berr-reporting` unsupported** by this kernel's mcp251x.
  **Operator action before re-running Run A:** physical isolation ladder
  (report §6): Test A HAT disconnected ≥5 min → Test B cable → Test C pitch
  → Test D yaw → Test E both → motor-load ramp → termination measurement →
  scope if available → A/B with a replacement HAT. Then power-cycle the
  drives, confirm `./turret-can discover` returns both IDs, and re-run the
  payload check (the `pos_return` gain fix is still pending live
  verification). Supervisor (`tools/can_supervisor.py`) delivered and
  live-tested.
- [x] **Run B (live, yousee PHY, 2026-09-02 23:19–23:45) — FULL PASS.**
  Ran the whole P6 verification on the yousee USB-CAN transport after a
  drive power-cycle (logs /tmp/controld_p6_runB1..4.log):
  - **Run B1 (no profile):** homing → hold → `start_payload_verification` →
    both axes measured cleanly (pitch rise 0.53/0.45 s, settle 0.79/1.21 s;
    yaw 0.51/0.46 s, 0.71/0.55 s) → `complete: no_profile`. The Run A
    residual (against-gravity `pos_return` timeout) is GONE — the
    `check_spd_kp/ki` firmware gain fix works live.
  - Profiles captured from that run: `config/payload_profiles/conservative.yaml`
    (real baseline) and `too_light.yaml` (0.35x rise/settle — deliberate
    mismatch-test profile, never install).
  - **Run B2 (conservative):** `payload check complete: ok (derated=false)`.
  - **Run B3 (too_light):** `mismatch; rise_time ratio 3.53 ... (derated=true)`;
    telemetry `{payload_profile_status: mismatch, payload_derated: True}`.
  - **Run B4 (conservative again):** `ok (derated=false)`, telemetry derate
    cleared. Bonus negative test: a second command issued mid-return was
    correctly rejected (`system is moving; wait for hold`).
  - Zero soft-limit excursions / limit crossings in all four runs (grep-verified).
  - Mechanism note: profile selection happens at boot (no runtime selector),
    so the mismatch→clear cycle ran via config change + daemon restart; a
    `select_payload_profile` web command would make one-session operation
    possible (small follow-up).
  - Telemetry note: the 15 Hz snapshot producer blocks with the check, so
    `payload_check_active` cannot stream `true` mid-check (it reads false;
    phase transitions + `payload_profile_status` updates are correct).
    Cosmetic observability gap — worth a small fix later.
  - **Decision:** keep `auto_verify: false` (operator-triggered check; boot
    stays deterministic). Manual `start_payload_verification` is fully
    functional as the §27 path.

**Research (2026-09-02, online — full note in
  `docs/research_can_bus_error_passive.md`):** the ~1 s ACTIVE→PASSIVE
  re-degrade after every reset is the signature of a *continuous* physical-layer
  fault (hundreds of bit errors/s re-accumulate TEC/REC past 128); error-passive
  is NOT covered by the kernel's `restart-ms` (it only fires on BUS-OFF), so no
  software reset can clear it while the fault persists. The silent drives are
  the same fault from the other side: bus-off / fault-latched, unrecoverable
  until power-cycled (CAN 2.0B bus-off recovery needs 128×11 clean recessive
  bits). The MCP2515/HAT is exonerated (re-probe works, reaches ACTIVE). Leading
  candidates: load-induced power-rail sag / ground bounce or EMI at the
  against-gravity peak current (the fault's trigger), or a latched transceiver
  (cleared only by power-cycling the affected device). **After the power
  cycle, verify before re-running:** `ip -details -statistics link show can0`
  (state ERROR-ACTIVE, error counters flat) + `candump can0 -ea` in a terminal
  during the run (any `ERRORFRAME` line = the fault is still there) +
  `turret-can stats` + discovery reads of both drives. (The earlier
  "firmware follow-ups: berr-reporting on + CanIfState monitoring" are
  superseded: `berr-reporting` is unsupported on this kernel's mcp251x, and
  the monitoring/recovery is delivered as `tools/can_supervisor.py` —
  see `docs/can_hardware_fault_report.md` §6.)
- [ ] **Runs B/C/D + profiles.** `conservative.yaml` / `too_light.yaml` baselines
  must match the drive's *measured* response, so they are written from a passing
  Run A; then Run B (conservative, `auto_verify: true` → ok, derated=false), Run C
  (too_light → mismatch + derate 0.5 + telemetry `derated` true, hold drops to
  5°/s), Run D (back to conservative → ok, derate clears). Deferred until the bus
  is back.

---

## P7 — Live camera / vision verification (Phase 4) `[CAMERA]`

> **STATUS 2026-09-03 (b): camera path VERIFIED on real glass; the DETECTOR stack
> is still the gap.** `tools/camera_bringup_probe.py` streams the IMX500 through
> the shipped `Picamera2FrameSource` (45/45 frames, real `SensorTimestamp`s, same
> clock domain as controld — details below) and feeds a real-camera stream into
> `controld --sim` end to end. What remains missing is object *classification*
> (no detection API, no RPK assets, no Hailo/Dataforensics SDK): ranked options in
> **`docs/research_vision_readiness_p7.md`**. Bring-up path without the stack:
> `--detector simple` (S1/S3 in Part 2).


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

### Real-camera verification 2026-09-03 (b) — `[CAMERA]`, no CAN, no motor

`tools/camera_bringup_probe.py` (new) drives the **shipped**
`Picamera2FrameSource` on the real IMX500. Measured:

```
frames captured  : 45/45         image (1080, 1920, 4) uint8
real SensorTimestamp used: 45/45 (monotonic fallback: 0 frames)
stamp advance    : 33.314 – 33.318 ms   (30 fps requested: 33.333 ms)
|wall - sensor|  : 26.6 – 31.0 ms
```

The last line is the §6.2/§11 proof: the stamp is in the **host CLOCK_MONOTONIC
domain** controld interpolates the motor history against, offset by well under one
frame (a `CLOCK_REALTIME` stamp would read ~1.7e9 s off, not tens of ms). The
detector leg paints a bright patch at known positions **on real captured pixels**
(a probe cannot wave a target): 18/20 frames reported it, tracked 969 px of a
1083 px sweep (90 %), centre error −28 px ≈ one 30 px grid cell. That validates
format/geometry on real data; it does **not** certify real-world detection
accuracy (only a supervised session with a real target does that).

End-to-end with the daemon (no hardware at risk — `controld --sim` never opens a
CAN transport):

```
visiond --real --detector simple --frames 120 --orientation rotate_180
  -> published 120 measurements, bridge blobs 0/120 (dark, static room)
controld --sim:
  vision: 116 frames (0 dropped, seq 115, age 27 ms) | tracking=off state=ready_hold conf=0.00
  tracking ENABLED (...)            <- only after homing (§38.1)
  vision: 120 frames (0 dropped, seq 119, age 17963 ms) | tracking=on state=ready_hold conf=0.00
  loop: target=200 Hz p50=5.054 p95=5.056 p99=5.062 worst=5.112 ms (n=4096)
```

Age 23–28 ms at a 33 ms frame period → the §34 freshness window (100 ms) is met
with real sensor timestamps; after the publisher exited, age grew honestly
(+1 s/s) and the state stayed `ready_hold` — **no phantom target from a dark
room**, which is exactly the direction the design says to fail in. Camera +
ingest cost the 200 Hz loop nothing measurable (worst 5.112 ms, same as the
synthetic-session numbers).

**One practical finding for P8:** mean scene luma was **1.5 / 255** — the room is
effectively dark. A motion/blob detector (and any NN) needs light; do not read
"0 blobs" as "the detector is broken" during the first live session.

**Defect found and fixed on the way (would have looked like a dead camera):**
`request_callback` is **deprecated on picamera2 0.3.37** and silently maps onto
`post_callback`, which fires only for capture-*file* jobs — the callback
mailbox therefore received **zero frames** and `capture()` timed out after 2 s.
`Picamera2FrameSource` now **pulls** completed requests
(`capture_request()` → `get_metadata()` + `make_array("main")` → `release()`),
which is what `tools/vision_probe.py` proves works here; note
`CompletedRequest.get_metadata()` takes **no** stream argument (passing `"main"`
is a `TypeError`). Frames with no `SensorTimestamp` are counted
(`frames_without_sensor_timestamp`) rather than silently re-stamped. Regression
pinned by `vision/tests/test_picamera2_source_contract.py` (7 cases against a
fake picamera2: pull-and-release order, the deprecated callback is never
assigned again, monotonic fallback counted, orientation applied to pixels AND
boxes together, RPK failure not fatal). The fake cannot prove the camera works —
that is what the probe is for — it only makes the known failure modes loud.

---

## P8 — Live closed-loop tracking (Phase 6) `[MOTOR] + [CAMERA]`

Phase 6 is **implemented and integration-tested on SimMotorBackend + synthetic
vision** (track a rotating target; loss → coast → brake → ready-hold; search
sweep; soft-limit containment; fault → safe stop). **The code prerequisite is
done** (Part 2 S1) and the vision input is now proven against the **real camera**
into `controld --sim` (see P7's verification above: 120 frames, 0 dropped, age
23–28 ms, §38.1 gate held, loop p50 5.054 ms with camera + ingest running). What
remains for this row is the live station: a supervised window, motors armed, and
a **lit scene with a real target** (the last session measured luma 1.5/255 — the
room was dark, so "0 blobs" was not a detector failure).

> ✅ **PREREQ — controld vision wiring (Part 2, S1): BUILT.** `controld` binds the
> `SOCK_SEQPACKET` UDS (`vision.socket_path` / `$OTA_VISION_SOCKET`), decodes the
> 58-byte measurement, feeds `ControlLoop::feed_measurement`, loads the
> `tracking:` block + camera intrinsics, and calls `enable_tracking()` only after
> the homing gates pass (§38.1) — with the search span clamped inside the homed
> soft limits. Verified offline against synthetic frames *and* against the real
> IMX500 stream (sim backend, so no motor could move).

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

### Static review done offline 2026-09-03 (b) — four template defects fixed

Everything below is decidable without hardware, and each one would have cost a
supervised station visit:

| Defect | Why it bites | Fix |
|---|---|---|
| `TimeoutStartSec=120` in `turret-control` | Homing from cold is ~2.5 min (volatile mechanical zero). systemd would SIGTERM the daemon **mid-homing**, and because a timeout stop is not a "failure", `Restart=on-failure` would not bring it back: a silent, unhomed station that `systemctl status` reports as deployed | `TimeoutStartSec=infinity` + the reasoning in the unit; the daemon's own contact/stall/supervisor gates are what bound a stuck home |
| `turret-vision` / `turret-web` used `.venv/bin/python` | **Import-probed both interpreters:** the venv fails `import picamera2` (`No module named 'libcamera'`) and `uvicorn` is missing from it; `/usr/bin/python3` has picamera2/uvicorn/fastapi/numpy/av/PIL/yaml. picamera2 cannot be pip-installed into a venv (it binds the system libcamera stack), so this was not fixable by installing more into the venv | both units now run `/usr/bin/python3`, with the probe result in the comments (the README's old advice — "put fastapi/uvicorn/picamera2 in the venv" — was unachievable) |
| `turret-vision` had **no** `--orientation` | The IMX500 is mounted upside-down. A missing `rotate_180` does not crash anything: the dashboard looks right while the control geometry is 180° wrong | `--orientation rotate_180` in `ExecStart`, and `visiond --orientation` now has `choices=` so a typo exits with a usage line in the journal instead of a traceback |
| `turret-vision` had no detector choice | Silent ambiguity about what produces detections; today the platform has none | `--detector none` explicit, with the `simple` (P8 bring-up) and `rpk` (production) meanings in the unit |

Also reviewed and left as-is, deliberately: `turret-can-supervisor` watches
`can0` (the MCP2515 witness) — while the yousee adapter is primary it protects
nothing the motors use, so the README says not to expect it to; `turret-log`
tails `logs/*.log`, which the current runbook does not produce (controld logs to
stdout → journald), noted in the README rather than "fixed" by inventing files.

`systemd-analyze verify` on all five units: no structural complaints; the only
unresolved reference is `/opt/open_auto_turret/build/control/controld` (the
deploy root does not exist on a dev checkout, as expected). A new
**pre-flight checklist** (8 items: timeouts, interpreter, socket agreement,
orientation, detector, camera exclusivity, which CAN link the supervisor
actually watches, where the logs really go) is in `systemd/README.md` — read it
before the first `systemctl start`.

Related hardening found while checking the unit's restart semantics:
`visiond` used to die with a connect traceback when controld's socket was not
there yet (which under `Restart=on-failure` reads as "vision keeps restarting",
hiding the real reason). `IpcPublisher.start()` now waits for the socket
(`--connect-timeout-s`, default forever) with one legible line, honours
Ctrl-C/SIGTERM while waiting, and `visiond` exits with a hint that controld is
the side that binds. 6 new tests (`test_ipc_publisher.py`): immediate connect,
late-bind wait, timeout raises `OSError`, stop aborts the wait promptly, a dead
peer surfaces on publish (no silent queueing), and SEQPACKET boundaries survive.
The live P11 run (enable, kill -9 restart → UNHOMED, network independence) is
still the user's, on the supervised station.

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
| S1 | `control/src/vision/vision_ingest.{hpp,cpp}` + `control/src/main.cpp` | **DONE offline 2026-09-03:** controld BINDS the `SOCK_SEQPACKET` UDS (`vision.socket_path`, `$OTA_VISION_SOCKET` override), decodes 58-byte `TargetMeasurement`s on its own accept/reader threads (§46: never in the 200 Hz cycle), hands them to `loop.feed_measurement()`, auto-enables tracking once `homed_` (§38.1), loads the `tracking:` block + intrinsics/extrinsics, and publishes link health (`vision_connected/frames/dropped/last_frame_sequence/measurement_age_ms`) in the §6.3 snapshot. Sim-verified end to end + 8 unit tests (`test_vision_ingest`) | **Nothing to build — the live half is P8 itself:** run the real camera + real motors, supervised | P8 |
| S2 | `tools/turret_payload.cpp` (`turret-payload`) | `--sim` rehearsal on `SimMotorBackend` (no CAN, deterministic clock/pacer) | Drop `--sim`: real mode opens CAN + `BootFsm` and moves the real motors (supervised, daemon stopped) | P10 |
| S3 | `vision/visiond.py` + `vision/frame_source.py` | `--synthetic` (SAFE default). `--real` now speaks the picamera2 that is installed (0.3.37): camera started once, request-callback → newest-frame mailbox (same proven pattern as `web/webd/video.py`), `SensorTimestamp` in the monotonic domain, install orientation applied to frame AND boxes. `--detector rpk` (attempts the IMX500 AI API, **degrades to no detections** — measurements stay `valid=false`), `--detector simple` (classical bridge detector, `vision/simple_detector.py`), `--detector none` | The RPK/Hailo stack (platform upgrade or USB accelerator — `research_vision_readiness_p7.md`) replaces the bridge detector; then P8 is a production run | P7, P8 |
| S4 | `web/webd` (inspection service) | `FakeControld` stand-in server (sim telemetry); real camera already live-verified | Point webd's socket config at the real controld web socket (`OTA_WEB_SOCKET`); run with `turret-web` unit | P12 |
| S5 | `Firmware/systemd/*.service` | Templates only — none installed/enabled (paths point at `/opt/open_auto_turret`) | Adjust `ExecStart`/`WorkingDirectory` to the install path; `systemctl enable --now can0 turret-control [turret-vision turret-web]` | P11 |
| S6 | C++ mock tests: `test_tracking_integration`, `test_payload_daemon`, `test_control_loop`, homing/trajectory suites (all `SimMotorBackend`) | Simulated plant (first-order lag, end stops, stall effort) | HIL counterparts per §54.4: the same scenarios (rotating-target track, loss→coast→brake, search sweep, in-loop payload check, stop contact) executed against the real CAN — supervised | P13 |
| S7 | `calibration/camera_intrinsics.yaml` | Path in `turret.yaml`; **file still does not exist.** The LOADER is done (`control/src/calibration/camera_calibration.hpp`: `load_camera_intrinsics`, key=value, requires fx/fy/cx/cy/width/height) — absent it, controld logs `UNCALIBRATED` and uses the default pinhole (fx=fy=1000, cx/cy centre) | Produce the §28.2 intrinsics file (camera matrix + distortion) at commissioning | P8/P9 |
| S8 | `calibration/camera_extrinsics.yaml` (R_P_C) | Path in `turret.yaml`; **file still does not exist** → aligned-identity default. The LOADER is done (`load_camera_extrinsics`: 3×3 row parse + orthonormality check, aligned default otherwise) | §28.3 extrinsic estimate (fiducial board), loaded into `geo::TurretKinematics` | P9 |
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

---

## Offline session 2026-09-03 (b) — S1 + observability batch `[SW]`

All of this was built and measured **offline** (sim plant / unit tests; the real
camera and CAN were untouched). Nothing here satisfies a `[MOTOR]` queue item.
Probe artifacts live in `Firmware/build/probe/` (logs + JSONL); the harness
scripts are `probe2_payload.sh`, `probe3_select.sh`, `probe4_regression.sh`.

### S1 — controld vision ingest: BUILT and sim-verified

The daemon now binds the vision UDS and consumes `TargetMeasurement`s exactly as
§6.1/§6.2 specify. Sim probe (`controld config/turret_sim.yaml --sim` +
`visiond --synthetic --frames 600 --framerate 30`):

```
tracking ENABLED (v_max track=30.0 deg/s search=10.0 deg/s, ... intrinsics fx=1000.0 ...)
vision: 600 frames (0 dropped, seq 599, age 138321 ms) | tracking=on state=ready_hold
state sequence: ready_hold -> tracking (conf 1.00) -> brake_to_hold (conf 0.08) -> ready_hold
final pose: q_pitch=-1.5053 q_yaw=-2.2677 rad   (ready pose, within 0.02 deg)
```

The publisher disappearing drove §34 exactly as designed (track → coast/brake →
hold, and the axes returned to the ready pose). Unit coverage: `test_vision_ingest`
(8 cases: byte-exact round trip; short and oversized datagrams counted as dropped
and never handed to the loop; second publisher rejected; stop unlinks + counts
down; restart rebinds; counter semantics; age measured on the control clock) and
`test_tracking_observability` (5 cases, incl. `at_ready` with tracking enabled and
the §6.3 vision block).

### S1b — final regression (`probe4_regression.sh`, one daemon run)

Payload selection + the real UDS hand-off + §34 loss behaviour + park, all in one
run, with the async logger in place:

```
select_payload_profile ghost     -> {"ok":false,"error":"no payload profile named 'ghost' in config/payload_profiles"}
select sim_plant + verify        -> phase=payload_check (active=True) -> "payload check complete: ok (derated=false)"
visiond --synthetic 300 frames   -> vision: 269 frames (0 dropped, seq 268, age 18 ms) | tracking=on state=tracking conf=1.00
publisher exits (age 1007 ms)    -> state=brake_to_hold conf=0.09 -> state=ready_hold conf=0.00
after exit (state)               -> vision_connected=False, vision_frames=300, age 1123 ms ("was streaming, then stopped")
loop: target=200 Hz p50=5.055 p95=5.059 p99=5.067 worst=5.129 ms (n=4096); SLOW CYCLE count: 0
PARKED (motors de-energized at the park pose) / controld stopped cleanly
```

`measurement_age_ms` staying in the 7–29 ms band while visiond published at 30 Hz
is the §6.2 timestamp chain doing its job (capture → IPC → control cycle), and the
~1 s age after the publisher died is what makes a dead visiond obvious on the
dashboard instead of looking like "no target".

### Control-thread stalls: the synchronous logger was the cause

The first probe (before this batch's logging change) captured
`supervisor: DERATE reason='control-loop cycle overrun' overrun_us=1074018` and
`overrun_us=800159` — 0.8–1.1 s stalls of a 5 ms cycle, from spdlog writing
synchronously from the control thread (a §46 violation). controld now starts an
**async logger** (8192-message bounded queue, one writer thread, `create_async_nb`
= overrun-oldest, so a full queue drops log lines instead of stalling the cycle;
falls back to the default logger if it cannot be created) and logs per-cycle
period percentiles every second:

```
loop: target=200 Hz p50=5.054 p95=5.058 p99=5.068 worst=5.092 ms (n=4096)
```

Three further probes (~5 min of sim: homing + payload checks + tracking + park)
show `worst` <= 5.2 ms in steady state; the only remaining `SLOW CYCLE` warnings
were 2.4 ms / 5.0 ms **during homing**, i.e. the known blocking register recipes
(re-arm / backoff), not random stalls.

**Why this matters for P3/P4:** both recorded a recurring ~98 ms feedback gap that
tripped `feedback_max_age_ms` (Brake) with nothing on the wire to explain it. A
~100 ms logging stall on the same host is exactly that size, and it is now gone
offline. P3/P4 still have to be re-run live — this is a candidate root cause
removed, not a live pass, because the sim plant has no CAN transport to stall.

### `payload_check_active` "snapshot freeze" — NOT reproducible, closed

P6's note claimed the §6.3 snapshot stops during a payload check. Recorded at
~15 Hz across two real checks (`tools/telemetry_stream.py --jsonl`):

```
t=39630.823 phase=payload_check  payload_check_active=True
t=39637.036 phase=hold           payload_check_active=False payload_profile_status=mismatch payload_derated=True
t=39691.243 phase=payload_check  payload_check_active=True          (1816 frames, no gap)
```

The snapshot is rewritten every cycle and both new fields stream. The original
report is explained by the tool, not the daemon: `tools/station_ipc.py state`
asked for keys that are not on the wire (`phase`, `confidence`, `q_pitch`,
`fault`) — missing keys printed `?`, and `phase` did not exist yet. `phase` +
`fault` are now published and the tool's key list matches the real names.

### New: `select_payload_profile <name>` (§42.2) — the mismatch→clear cycle without a restart

Until now the only way out of a `mismatch` derate was editing
`payload.active_profile` and restarting the daemon. The command swaps the profile
at runtime: the new caps apply immediately, but the **status becomes
`no_profile`** until `start_payload_verification` runs (§31.3), so selecting an
unverified profile can never silently raise a limit. Verified offline, full cycle:

```
select sim_plant  -> no_profile -> verify -> payload check complete: ok (derated=false)
select too_light  -> no_profile -> verify -> mismatch; peak effort 1.00 Nm exceeds 2.0x baseline (0.35 Nm); derate 0.5
select sim_plant  -> no_profile -> verify -> ok   (derate CLEARED)
select ghost      -> ok:false, "no payload profile named 'ghost' in config/payload_profiles"
```

Two related fixes: unknown names are rejected **synchronously** (the web response
is written before the control thread executes anything, so an `ok` that decays
into a log-only warning would lie to the UI; the `stat()` runs on the web thread,
never on the control thread, §46), and `config/payload_profiles/sim_plant.yaml`
is a clearly-labelled SIM-PLANT fixture so this path stays testable offline.

### Two defects the new tests caught (both fixed)

1. `vision_measurement_age_ms` could go **hugely negative** when the arrival
   stamp and the loop clock disagreed (`-41022034` ms in a test). In the daemon
   both are CLOCK_MONOTONIC so production was unaffected, but the snapshot is the
   operator's only view: the age is clamped at 0, `-1` still means "no measurement
   has ever arrived", and the counters stay cumulative after the publisher dies
   (so "it was streaming, then it stopped" is visible).
2. **`at_ready()` was false whenever tracking was enabled**, even while the
   station was holding the ready pose. On a station running with
   `tracking.enabled: true` that silenced the operator's P0 pass line
   (`homed + at ready pose; holding`) and gated off the §27 auto payload check.
   `at_ready_` is now derived from the arbitration (`source == Hold`) plus the
   pose tolerance; regression-tested both ways.

### Vision: the platform gap is unchanged, the bridge is built

`Picamera2FrameSource` was rewritten against the picamera2 that is actually
installed (0.3.37: camera started once, request callback → newest-frame mailbox —
the old code called `capture_file()` per frame, which cannot stream). The RPK path
is now an *attempt* that degrades to "streaming WITHOUT detections" (no detections
→ `valid=false` → no tracking: the safe direction). A **classical bridge
detector** (`vision/simple_detector.py`, `--detector simple`) exists so P8's loop
can be brought up on real glass: three-frame differencing with polarity-consistent
cells and edge-band stitching, and tests that state its accuracy honestly (centre
within about half the per-frame motion; a motion detector loses a target that
stops moving; the class id is synthetic). It is not the §10.1 detector — **P7
stays open** (`research_vision_readiness_p7.md`).

### Tools + coverage

- `tools/telemetry_stream.py` (new): records/prints the §6.3 stream
  (`--changes-only`, `--count-transitions`, `--jsonl`); read-only by construction.
- `tools/station_ipc.py`: commands now pass their **arg** (it was silently
  dropped — the reason the first `select_payload_profile` probes "failed" with a
  validation error), honours `$OTA_WEB_SOCKET` / `--socket`, and `state` prints
  real field names.
- `controld --sim` never opens a CAN transport at all (it will not invent a
  `vcan0`), logs a loud `*** SIM MODE ***` line, and no sim run is ever evidence
  for a `P#` live item.
- Suite: `ctest` 42/42 (added `test_vision_ingest`, `test_tracking_observability`);
  pytest 116 passed / 2 skipped (added `test_simple_detector.py`, the `--detector`
  CLI cases, and the monotonic-stamp/pacing tests).
