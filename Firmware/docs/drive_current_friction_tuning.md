# Drive Current & Friction Tuning — Decision / Plan

**Status:** DECISION PROPOSED — pending user approval before implementation.
**Triggered by:** P0 live-homing runs p0k / p0l / p0m (2026-09-01).
**Ties to:** architecture §26 (homing safety), §31 (payload variation &
motor tuning), §8.5 (control modes), and the P10 payload-profiling stage
(Phase 9) in `post_homing_test_queue.md`.

---

## 1. Observation (what the P0 runs actually showed)

The sensorless homing FSM is logically correct (coarse → backoff → fine₁ →
small-backoff → fine₂ → repeatability, both endpoints, then travel
validation), but the **motion it drives is not smooth**. Concretely:

- **p0k** — yaw endpoint A homed cleanly at 10°/s, but the run faulted on
  travel-band validation (real 116° span vs a 360° placeholder band). The
  band was a placeholder; that part is commissioning, not a control bug.
- **p0l** — yaw fine approach (−dir) **stalled after ~1°** at raw −1.13 and
  latched it as a contact. The backoff **overshot** (moved 11° vs the
  configured 5°). Both endpoints landed 9.4° apart → travel validation
  failed. −1.13 was *not* a hard stop.
- **p0m** — after moving the yaw to a clean mid position and raising the
  fine speed to 20°/s, endpoint A homed, but endpoint B **failed on
  repeatability** (|fine₁ − fine₂| > 0.5°). The yaw ended at **−1.382, past
  the −1.31** that p0l had latched as the "stop" — proving −1.31 was itself
  a breakaway stall, not the mechanical stop.

A manual move (−1.14 → −0.25 at 20°/s) **crossed −1.13 freely and
smoothly**. So −1.13 is not a physical obstruction — the drive simply
**cannot break away from rest reliably at the current torque authority**.

### The user's reading (adopted as the working hypothesis)

> "The motion is not smooth. The current is still too small. More load adds
> friction, not just momentum."

This reframes the failures. What I had attributed to a "tiny `spd_ki` gain"
is better explained by a **current/torque limit that is too small to beat
static friction**, with **friction that grows with payload**. A position
loop that cannot overcome static friction stick-slips: it stalls, the
integrator winds up, it slips, it stalls again. That is exactly the
jitter, the breakaway stalls, the overshoots, and the non-repeatable fine
contacts we saw.

**Consequence:** the homing FSM and contact detector are being asked to
interpret a signal (position) that is itself corrupted by stick-slip. No
amount of contact-deter threshold tuning makes a stick-slipping drive
look clean. The fix is upstream: give the drive enough current to run
smoothly, and make "smooth" a *measured, sized* property rather than an
assumption.

---

## 2. The decision (to be approved)

1. **Stop treating "the axis moved" as the motion-quality gate.** The
   contact/motion detector's `had_motion` check is a *safety* gate (was
   there any movement?), not a *quality* gate (was the movement clean?).
   Add an explicit **smoothness** property and gate on it.

2. **Size the homing current limit from the plant, not from "intentionally
   low."** §26 says the homing current is "intentionally low" to keep stop
   contact low-energy. That is a *ceiling* rationale, but it has been used
   as a *floor* excuse. The current limit must be chosen so that:
   - it is **≥ the current needed for smooth (non-stick-slip) motion** at
     the homing speed and worst-case payload (the *floor*), and
   - it is **≤ the current that keeps stop-contact energy within the
     mechanical stop's rated tolerance** (the *ceiling*, §26).
   If floor > ceiling, that is a real engineering conflict that must be
   surfaced (reduce speed, add friction compensation, or re-rate the
   stop) — not silently absorbed by a stick-slipping drive.

3. **Implement three cooperating mechanisms** (below): **jitter detection**,
   **friction modeling/compensation**, and **projected smooth-running
   current headroom**. Together they turn "did it move?" into "does it have
   enough torque authority to move smoothly, and is the stop still safe?"

4. **Sequence it against the existing stage plan, don't fold it into
   homing.** Per §31.2, homing and controller tuning are *separate*
   commissioning operations. So:
   - **Now (P0 unblock):** implement jitter detection + a *manual* current
     sizing pass (measure, size, set `limit_cur`, re-home) so P0 can pass
     on the real plant. This is a supervised commissioning action, not an
     autonomous loop.
   - **Later (P10 / Phase 9):** make the sizing *automatic and
     payload-aware* (friction model + headroom projection driven by the
     P10 payload profile). The P0 manual pass is the prototype of the P10
     automatic pass.

---

## 3. Component 1 — Jitter (stick-slip) detection

### 3.1 What to detect
Stick-slip / jitter in the position command during an approach or hold.
It is the signature of insufficient torque authority.

### 3.2 Signals we capture / compute (per measurement interval, 200 Hz)
We already have, per axis, from the CyberGear feedback (no new sensor):
- `q` — `mechPos` (0x7019).
- `v` — **position-derived** `v_est_` (control loop, low-pass, §control).
  The drive's self-reported `MechVel` (0x701B) is **not** used: it has a
  ±0.05 rad/s noise band at rest (observed P0j) that chatters at-rest gates.
- `τ` — `torque_nm` (±12 N·m), **directly** in the feedback frame.
- `i_q` — `Iqf` (0x701A), filtered q-axis current.

We **add** two derived signals (the user's point):
- `a` — **acceleration**, `dv/dt` of the position-derived `v` (NOT of the
  raw position, and NOT of the drive's noisy self-reported velocity — see
  §3.4 for why the differentiation must be done carefully).
- `j` — **jerk**, `da/dt`. For a direct-drive payload, jerk is the torque-
  rate demand (§architecture: "abrupt acceleration changes translate
  directly to abrupt torque demand"), so it is the most direct stick-slip
  signature.

All five (`q, v, a, j, τ`) are logged per interval so motion quality can be
inspected offline and consumed by the P10 profile.

### 3.3 Measurable signatures (jitter = NOT smooth)
A healthy constant-speed approach has `a ≈ 0` (small transients only at
break-away and at the stop) and small `j`. Stick-slip breaks that:
- **Acceleration peak (slip):** `a` spikes positive as the drive suddenly
  slips forward. Signature: `max(a) > a_peak` over the window.
- **Acceleration dip (stall):** `a` goes strongly negative / to ~0 as the
  drive stalls. Signature: `min(a) < -a_dip` over the window.
- **Jerk spike:** `max(|j|) > j_peak` — an abrupt acceleration change
  (each stick→slip transition is a jerk spike).
- **Stall-recovery count** in a sliding window (e.g. 250 ms): number of
  single crossings of `v_move` (the jitter "moving" threshold, default
  0.10 rad/s) from below — the drive was moving at the approach speed, then
  stalled (`|v| < v_move`), and is now moving again (`|v| >= v_move`).
  One crossing = one recovery, counted once per stall→motion episode.
  `v_move` must sit BELOW the approach speed (coarse 0.175, fine 0.349
  rad/s) and above filtered rest noise — it is a DIFFERENT threshold from the
  contact gate's `stall_velocity_threshold` (0.5 rad/s, which is above the
  approach speed and therefore always "stalled" velocity-wise during the
  approach; the contact gate discriminates via position-progress instead).
  A healthy approach has 0; stick-slip has many.
- **Velocity standard deviation** over the window, normalized by the
  commanded speed. High normalized std-dev = jitter.
- **Torque oscillation (complementary, no differentiation needed):** the
  drive's `τ` oscillates (high during stall, low during slip). Signature:
  std-dev of `τ` over the window, or a stall→recovery count on `|τ|`. This
  cross-checks the position-derived signatures and is immune to
  differentiation noise.

"Smooth" for an interval = **no acceleration peak, no acceleration dip, no
jerk spike, and no stall-recovery** (within thresholds). The user's
requirement — "smooth acceleration for each measurement interval without
peak or dip" — is exactly this gate.

### 3.4 Getting a clean `a` (and `j`) — differentiation caveats
Differentiation amplifies noise, so the derived signals must be computed
carefully or they will be pure noise:
- **Do NOT differentiate the drive's self-reported `MechVel` (0x701B).** It
  has a ±0.05 rad/s noise band at rest (P0j); at 200 Hz (`dt` = 5 ms) that
  becomes a ±10 rad/s² acceleration noise band — useless.
- **Do NOT take a raw second difference of the position**
  (`(q[i+1] - 2q[i] + q[i-1])/dt²`): the encoder resolution noise is
  amplified the same way.
- **Compute `a` from the position-derived `v_est_`** (already low-pass
  filtered in the control loop), using a short smoothing window — e.g. a
  local quadratic fit of `q(t)` over ~8 samples (40 ms) yields smooth
  `v` and `a` and rejects single-sample noise; then `j = da/dt` over the
  same window. Keep the window short enough (≲ 50 ms) not to smear a real
  stick-slip event, and consistent with §architecture's "do not
  over-smooth … not introduce hundreds of ms of lag."
- **Cross-check with the directly-available `τ`:** a genuine acceleration
  peak/dip should be corroborated by a torque change (slip → torque drops,
  stall → torque rises). If `a` spikes but `τ` does not, treat it as
  estimation noise, not stick-slip. This makes the gate robust to
  differentiation artifacts.
- The thresholds (`a_peak`, `a_dip`, `j_peak`) are **tuned on real data**:
  first log `q, v, a, j, τ` on a known-clean move (p0k endpoint A class) and
  a known-jittery move (p0l/p0m class), and set the thresholds to separate
  the two, not on a priori numbers.

### 3.5 Where it is used
- **Contact detector (immediate benefit):** a *true* mechanical stop is a
  **permanent** stall (velocity stays ~0 with no recovery). Stick-slip is an
  **intermittent** stall (velocity recovers). Today the detector relies on a
  200 ms dwell, which a long breakaway stall satisfies — the p0l false
  contact at −1.13 is exactly that. Add: **if a stall window contains ≥1
  stall→recovery transition, classify it as jitter, NOT contact** (do not
  latch; keep approaching). This directly removes the p0l/p0m false
  contacts. The stall is corroborated by the acceleration/torque signature:
  a true stop holds `v≈0` with `a≈0` and **sustained high `τ`** and *no*
  recovery; stick-slip shows an `a` dip then a peak and `v` recovers.
  - **Start-up exclusion (implemented):** the initial breakaway of each
    approach (rest → motion) is a stall→motion transition that is NOT a
    stall. It is excluded by requiring the axis to have *already* been
    clearly moving before the stall (`ever_moved_`, reset per approach).
    Without this, every clean approach would record 1 spurious "recovery"
    and the fine-approach start-up would sit inside the 250 ms window.
  - **Single-threshold recovery (implemented):** a recovery is one crossing
    of the jitter threshold `v_move` (0.10 rad/s) from below, so the
    filtered velocity cannot re-trigger on every sample while it climbs back
    through a band and over-count. `v_move` is a NEW threshold, distinct from
    the contact gate's `stall_velocity_threshold` (0.5 rad/s): the latter is
    above the approach speed, so the drive is always "stalled" velocity-wise
    during the approach and could never register a recovery.
- **Homing supervisor:** if jitter persists through a full approach at the
  configured current, fail the approach with a *distinct, actionable*
  reason (`jitter: insufficient torque authority`) instead of a vague
  timeout, and report the measured jitter metrics (stall-recovery count,
  `max|a|`, `max|j|`, `std(τ)`).
- **Telemetry:** expose per-axis `a`, `j`, and a `jitter_index`
  (stall-recovery rate + `max|j|`) so P10 and the web UI can see motion
  quality; log `q, v, a, j, τ` per interval for offline analysis.

### 3.6 Decision points (need approval)
- (a) Is "≥1 recovery in the stall window ⇒ not a contact" acceptable as the
  contact/reject rule, or do you want a higher threshold (e.g. ≥2)?
- (b) Window size and thresholds (implemented: 250 ms window; a NEW
  `v_move_threshold_rad_s` — default 0.10 rad/s, yaml `v_move_threshold` —
  is the single jitter stall/recovery threshold so each stall→motion episode
  is counted exactly once. It is distinct from the contact gate's
  `stall_velocity_threshold` (0.5 rad/s), which sits ABOVE the approach speed
  and cannot be used for recovery detection. A threshold strictly between
  "stalled" and "moved" (e.g. `v_move` < `v_stall`) would over-count because
  the filtered velocity re-triggers on every sample while it climbs through
  the band, so one threshold is used for both sides of the crossing).
- (c) Confirm the **acceleration/jerk** smoothness gate (no `a` peak, no `a`
  dip, no `j` spike) as a first-class "smooth" criterion alongside the
  stall-recovery count, with thresholds tuned on real clean-vs-jittery data
  (§3.4) rather than a priori values.

---

## 4. Component 2 — Friction model & compensation

### 4.1 Model (load-dependent, per user's point)
Friction torque on an axis, as a function of velocity `v` and payload
class `L`:

```
τ_f(v, L) = τ_static(L)   (breakaway, |v| ≈ 0)
          + τ_coulomb(L) · sign(v)   (|v| > 0, opposes motion)
          + b(L) · v                 (viscous)
```

Key: **all three coefficients grow with payload `L`** (heavier payload →
more bearing/contact friction, not just more inertia). This is the user's
"more load adds friction" point, made explicit.

### 4.2 How we estimate it (commissioning, supervised)
A small **friction probe** (supervised, safe central region, §31.3-style):
- Step the axis through a small ±range at a few low speeds; log `i_q`
  (→ torque via `k_t`) vs `v`.
- **Static friction:** the torque at the instant of breakaway (the peak
  `i_q` just before `v` leaves ~0).
- **Coulomb:** the mean `|τ|` during steady slow motion, both directions.
- **Viscous `b`:** slope of `τ` vs `v` across the speed set.
- Repeat per payload class → a per-class coefficient table (this is the
  payload-aware part that P10 consumes/produces).

### 4.3 How we use it
- **Position mode (now):** we cannot inject a torque feed-forward (the
  CyberGear owns the current loop; §8.5 defers torque FF to MIT mode). So
  in position mode the friction model is used to **size the current limit**
  (Component 3): the limit must cover `τ_f` + inertia + load. The model
  converts "the drive is stick-slipping" into "the limit is X N·m short."
- **MIT mode (later, §8.5):** the same model becomes a **torque
  feed-forward** (add `τ_f(v, L)` to the commanded torque), which lets the
  current limit drop back down for safe stop contact. This is the real
  "compensation" — but it is explicitly a later-stage item, gated on the
  MIT-mode work.

### 4.4 Decision points (need approval)
- (a) Confirm friction *compensation* (feed-forward) is **deferred to MIT
  mode** and that, for now, the friction model is used only for **current
  sizing**. (This keeps P0 unblocked without taking on the MIT-mode risk.)
- (b) Approve the supervised **friction probe** as a new commissioning
  sub-step (small ±move in the safe central region, logged `i_q`/`v`).

---

## 5. Component 3 — Projected headroom for smooth-running current

This replaces "just moved" with a *sized* torque-authority check.

### 5.1 Required torque for a smooth homing approach
For an approach at speed `v_h` and acceleration `a_h` with payload `L`:

```
τ_req(v_h, a_h, L) = τ_f(v_h, L)          (friction, Component 2)
                   + J(L) · a_h            (inertia; J from P10 profile)
                   + τ_grav(pitch, L)      (gravity term, pitch axis only)
I_req = τ_req / k_t                         (k_t = torque constant, A/N·m)
```

### 5.2 Headroom
```
headroom = limit_cur / I_req
```
- `headroom < 1.0`  → **cannot** run smoothly (guaranteed stick-slip).
- `1.0 ≤ headroom < H_min` (propose `H_min = 1.5`) → marginal; expect
  jitter at the friction peaks.
- `headroom ≥ H_min` → smooth; the extra margin absorbs friction
  peaks / payload variance without hitting the limit.

### 5.3 The floor–ceiling band (the actual sizing result)
```
floor  = I_req · H_min                          (smooth-running floor)
ceiling = I_max_stop                            (safe stop-contact ceiling)
```
where `I_max_stop` is derived from the stop's rated energy:
`E_stop ≤ ½·J(L)·v_stop_max²`, and `v_stop_max` is the largest approach
speed whose impact energy is within the stop's tolerance. (For a
sensorless stop, we keep `v_h` modest so the ceiling is not the binding
constraint; the point is to *check* it, not assume it.)

**Sizing rule:**
- if `floor ≤ ceiling`: set `limit_cur = floor` (or the next standard step
  above), record the margin, proceed.
- if `floor > ceiling`: **stop and report the conflict** with the numbers.
  Options, in order of preference: (1) lower `v_h` (drops both `I_req`
  and the ceiling's `v_stop_max` tradeoff — recompute), (2) add friction
  feed-forward (MIT mode, lowers `I_req`), (3) re-rate / re-design the
  mechanical stop. This conflict must never be hidden by letting the drive
  stick-slip.

### 5.4 How this replaces "just moved"
The homing supervisor no longer asks only "did the axis move?" It asks:
1. Was the motion **smooth** (Component 1 jitter gate)?
2. Does the configured `limit_cur` have **projected headroom ≥ H_min** for
   the commanded approach (Component 3)?
3. Is the stop-contact **ceiling** respected?
A "moved but jittery" or "moved but under-limit" result is a **distinct,
actionable fault**, not a pass.

### 5.5 Decision points (need approval)
- (a) `H_min = 1.5` smooth-running headroom target — OK, or do you want a
  different margin?
- (b) Confirm the **floor > ceiling** case is a hard stop-and-report (not an
  auto-lower-speed retry), for the first pass.
- (c) `k_t` (torque constant) and the stop's rated energy: **need hardware
  numbers** (open question §8). Until then, the headroom check runs in
  *report-only* mode (logs the projected headroom, does not gate) so we can
  validate the math against observed jitter before it becomes a gate.

---

## 6. How the three components close the loop

```
        ┌────────────┐   jitter?    ┌──────────────────────┐
        │  Approach  │─────────────▶│ Jitter detector (C1) │
        │ (limit_cur)│              └──────────┬───────────┘
        └─────▲──────┘                         │ "not smooth"
              │ sized limit_cur                ▼
        ┌─────┴─────────────────────────┐  ┌──────────────────────┐
        │ Current sizing (C3):          │◀─│ Friction model (C2): │
        │ limit = max(floor, ...) with  │  │ τ_f(v, L) from probe │
        │ headroom ≥ H_min, ≤ ceiling   │  └──────────────────────┘
        └───────────────────────────────┘
```

- **C1** tells us *when* the authority is insufficient (and stops the
  contact detector from being fooled by it).
- **C2** tells us *how much* torque friction actually consumes at the real
  payload.
- **C3** converts (C1)+(C2) into a *sized, checkable* `limit_cur` with an
  explicit floor–ceiling band, and surfaces conflicts instead of hiding
  them.

---

## 7. Reconciliation with the existing architecture

- **§26 "current/torque limit is intentionally low":** re-interpreted as a
  **ceiling** (safe stop contact), with a newly explicit **floor** (smooth
  running). The doc text should be amended to say "low *enough* for safe
  stop contact, and sized *high enough* for smooth motion at the payload"
  (proposed §26 edit, pending approval).
- **§31.2 "homing ≠ tuning":** respected. The *automatic, payload-aware*
  sizing lives in P10. The P0 work is a **manual, supervised** sizing pass
  (measure → set `limit_cur` → re-home) that prototypes the P10 automation.
- **§8.5 control modes:** torque feed-forward (true friction
  *compensation*) is deferred to MIT mode. Position mode gets friction
  *sizing* now.
- **P10 (Phase 9):** the friction model (C2) and the inertia `J(L)` feed the
  P10 payload profile; the P10 `verify` step gains a "smooth-running
  headroom" check (C3) so a payload swap that pushes `I_req` above
  `limit_cur` is caught at verification, not mid-track.

---

## 8. Implementation plan (phased)

**Phase A — unblock P0 (now, supervised, small):**
1. Add the **derived-signal capture**: compute `a` and `j` per interval from
   the position-derived `v` (§3.4 method), and expose `q, v, a, j, τ` in
   telemetry + the run log. This gives us the data to (i) tune the
   thresholds and (ii) see the stick-slip directly.
2. Add the **jitter detector** (C1) to the contact detector: a stall window
   with ≥1 recovery ⇒ classify as jitter, not contact; keep approaching.
   (Directly removes the p0l/p0m false contacts.) The "smooth" gate also
   checks the acceleration (no `a` peak / `a` dip) and jerk (no `j` spike),
   corroborated by `τ`.
3. **Tune the thresholds on real data:** run one known-clean move and one
   known-jittery move, set `a_peak` / `a_dip` / `j_peak` / stall-recovery to
   separate them (§3.4), and lock those values.
4. Add a **distinct homing fault** `jitter: insufficient torque authority`
   with the measured metrics (stall-recovery count, `max|a|`, `max|j|`,
   `std(τ)`).
5. **Manual current sizing pass** (supervised): raise `limit_cur` in steps,
   re-home, and log the jitter index until the approach is clean. Record the
   smallest clean `limit_cur` per axis as the P0 commissioning value.
6. Re-run P0 (p0n) → expect both yaw endpoints clean + repeatable, then
   pitch.

**Phase B — make it a property, not a guess (after P0):**
5. Add the **friction probe** (C2) as a supervised commissioning sub-step;
   produce the per-axis/per-payload coefficient table.
6. Add the **headroom projection** (C3) in **report-only** mode (log the
   projected floor/ceiling/headroom; do not gate yet) so we validate the
   math against observed jitter.
7. Amend **§26** (floor+ceiling) and add a **§31.4 "Smooth-running current
   headroom"** subsection describing C1–C3.

**Phase C — automate & payload-aware (P10 / Phase 9):**
8. Drive C3 from the P10 payload profile (`J(L)`, `τ_f(L)`); gate homing on
   headroom ≥ `H_min` and ceiling.
9. Wire C3 into P10 `verify` (payload-swap must re-project headroom).
10. (Later, with MIT mode) implement true friction **feed-forward** so the
    limit can drop back toward the ceiling for safe stop contact.

---

## 9. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Raising `limit_cur` increases stop-contact energy | Ceiling check (C3.3); keep `v_h` modest; the stop is rated for repeated low-energy contact (§26) — verify the rating (open Q). |
| Motor/driver overheating at a higher sustained current | Homing is short/transient; add a temperature check (already active per §26) and a per-move energy budget; never run a higher limit for long tracking. |
| Jitter gate rejects a *real* slow-but-clean contact | The rule is "recovery ⇒ not contact"; a true stop has no recovery. Tune the window/thresholds (§3.4) and validate on a known clean stop (p0k endpoint A was clean — use it as the positive reference). |
| Friction model is inaccurate → wrong sizing | Start report-only (C3.5c); validate against observed jitter before gating; per-payload table (C2.4) limits extrapolation. |
| Floor > ceiling conflict on a heavy payload | Hard stop-and-report with numbers (§5.3); do not auto-hide it. |

---

## 10. Open questions (need hardware / user input)

1. **`k_t` (torque constant)** for the CyberGear in A/N·m (or N·m/A). Needed
   to convert current ↔ torque. (May be in the CyberGear reference/PDF.)
2. **Rated stop-contact energy** (or max impact speed) for the mechanical
   stops — the ceiling in C3.3.
3. **Driver thermal limit** (max sustained current / thermal fold) for the
   CyberGear — bounds how high `limit_cur` may go.
4. **Current `limit_cur` values** actually in effect (yaw 6.5 A / pitch 5 A
   per earlier notes) — confirm and confirm the driver's max.
5. Confirm the **H_min = 1.5** headroom target (§5.5a).
6. Confirm friction *compensation* is **deferred to MIT mode** (§4.4a).

---

## 11. Acceptance criteria (for Phase A, to unblock P0)

- Per-interval `q, v, a, j, τ` are captured and logged; the `a`/`j` are
  visibly smooth (no noise-band spikes) on a clean move and visibly show the
  stick-slip peaks/dips on a jittery move.
- Jitter detector present; a stick-slip stall is **not** latched as a
  contact (reproduce on the −1.13-class stall and confirm it is rejected,
  corroborated by the `a` dip + `τ` rise + `v` recovery signature).
- A clean stop (p0k endpoint A class) **is** still latched as a contact
  (no false negatives; sustained `v≈0`, `a≈0`, high `τ`, no recovery).
- Homing can fail with the distinct `jitter: insufficient torque authority`
  reason when authority is genuinely insufficient.
- A supervised manual sizing pass finds a `limit_cur` per axis at which the
  full P0 homing (yaw both endpoints + pitch both endpoints) completes
  **smoothly and repeatably** (fine₁/fine₂ within 0.5°), and that value is
  recorded in `turret.yaml` with the measured headroom.

---

## 12. Immediate next step (pending approval of §2 decisions)

On approval: implement Phase A.1 (derived-signal capture of `q, v, a, j, τ`)
+ A.2 (jitter gate in the contact detector, incl. the acceleration peak/dip
and jerk checks) + A.3 (tune thresholds on a clean vs jittery move) + A.4
(distinct fault), rebuild, then run the supervised **manual current sizing
pass** (A.5: raise `limit_cur`, re-home, log jitter) to find the clean
operating point, and re-run P0 as **p0n** (A.6).

*Nothing in Phase A changes the homing FSM, the travel bands, or the safety
supervisor — it only makes the contact detector immune to stick-slip and
makes the current limit a measured, sized quantity instead of an
assumption.*
