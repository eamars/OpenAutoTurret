# The 10 deg/s plateau is real, and it accounts for C3 — 2026-09-05, round 38

Re-analysed the recording from round 36 (`/tmp/tel_r36_dart.jsonl`, 484 frames at 15.2 Hz, 128 fields) instead
of driving the station again. The file turned out to carry the daemon's own `q_ref_accel_yaw_rad_s2`,
`q_ref_rate_yaw_rad_s`, `target_az_world_rad` and `target_az_rate_world_rad_s`, so the dart can be reconstructed
without differentiating anything twice.

## First, the error in my own first pass

My initial computation was `gap = q_ref_yaw_rad − target_az_world_rad`, which produced gaps near **180°** — not a
finding about tracking, but me **subtracting a joint angle from a world azimuth**. `q_ref_yaw_rad` is a joint
position (2.60 rad ≈ 149°); `target_az_world_rad` is a world bearing. The near-constant offset of ~179° is the
mount geometry, not an error signal. Any future gap metric must be computed in one space with the controller's
own LOS conversion, never by subtracting these two fields.

## The plateau

Reference rate through the dart, sampled from the recording:

| t (s) | target az (°) | ref rate (°/s) | ref accel (°/s²) |
|---|---|---|---|
| 1.85 | −25.10 | **10.00** | 0.0 |
| 2.31 | −17.62 | **10.00** | 0.0 |
| 2.77 | −11.11 | **10.00** | 0.0 |
| 3.24 | −6.02 | **10.00** | 0.0 |
| 3.70 | −5.69 | **10.00** | 0.0 |

During the dart the reference rate sits on **exactly 10.00 deg/s with zero acceleration**, frame after frame.
That is a clamp, not a response: a tracking response would show a varying rate and non-zero acceleration. The
target's own rate peaked at 17.4 deg/s, so the demanded motion was above the plateau.

**Arithmetic that closes C3:** at 10.00 deg/s the reference covers **16°** in the 1.60 s dart while the target
moves **25°** — a deficit of **9°**, against a measured C3 lead of **−11.6°**. The residual ~2.6° is the
estimator's own lag (−1.4°) plus the actuation/prediction term (−0.6°) measured in the same run. The numbers add
up: **C3 is the plateau, plus the known estimator lag.** Nothing about the target, the detector, or the frame
rate is implicated.

**And it explains C2 without a second mechanism.** Over the whole recorded window the target swept 24.92° and
the reference swept 23.82° — so the shortfall was eventually paid back, at 6–8 deg/s during the hold, with the
reference oscillating ±8 deg/s around the target. That is exactly what C2 measured as slow recovery: **2.85 s to
get back inside tolerance** against a 1.50 s bar. A 9° debt repaid at single-digit rates takes seconds.

## What this does and does not establish

**Established:** a 10.00 deg/s clamp is in force on the *reference rate during AUTO_TRACK on this station*, and
its magnitude is sufficient to account for C3's deficit and C2's recovery time. Round 28's *number* was right.

**Not established — and round 37 proved why this matters:** what imposes it. Round 28 blamed
`SafetyEnvelope::v_max`, which round 37 showed is a fallback reached only when travel limits are invalid. The
clamp is real; the mechanism is still unidentified. Candidates, ranked by how cheaply they can be excluded:

1. The tracking intent is built with a velocity limit derived from the hold speed (the same `10.0` constant
   appears in `main.cpp:133` and `control_loop.hpp:108`), independent of the envelope.
2. A mode authority multiplier: axis limit 30 deg/s × 1/3 = 10.0 exactly. Telemetry reported `AUTH 100%`, so the
   *published* scale does not explain it — but the published field may not be the one applied here.
3. A tracker-side rate limit (stiffness/gain cap) inside the auto-track controller itself.

**Do not raise any limit until one of these is identified with a line number and a measurement.** The clamp is
not necessarily wrong — it may be a deliberate design limit — but it is the reason the operator's lead criterion
fails, so the operator should be told it exists, what it costs (C2, C3), and that its origin is still open.

## A side result on the jerk question

Deriving jerk from the **published acceleration** (one differentiation, well-conditioned) instead of twice from
rate gives **p50 161.6, p95 300.0 °/s³** — the p95 landing on the configured `max_jerk_deg_s3: 300` exactly,
which is what a honoured jerk limit looks like on this grid. Max is again meaningless (14600). This does not
overturn round 36's C5b finding — that used the reference *rate* path, where p50/p95 agreed across two
instruments — but it does mean the jerk question depends on which signal is differentiated, and the evidence
file should not claim a single number for it.

Station untouched by this round (analysis of a file); homed, MANUAL/HOLD, READY, vision running. No code
changed; **449 pytest / 57 CTest** stand from the round-37 build.

---

## Addendum, 2026-09-05 02:5x (round 39) — three mechanisms tested and rejected; the plateau is still unexplained

The chain is now read end to end: `q_ref_rate_yaw_rad_s` is literally `ref_lim_[i].v_rad_s`
(`control_loop.cpp:1204`), limited by `lim[i]`, and on the AUTO_TRACK path
`lim[i] = std::min(tracking_ref_.v_max_rad_s, env_.max_speed_at(solved, limits_[i]))` (`control_loop.cpp:844`),
with `tracking_ref_.v_max_rad_s = in.track_v_max_rad_s * c` where `c = clamp(target_confidence, 0, 1)`
(`reference_manager.hpp:131-132`). That narrows the plateau to the two terms of that `min`. All three testable
explanations fail against the recording:

1. **Confidence scaling — rejected by measurement.** `reference_manager.hpp:132` multiplies the configured
   30 deg/s by raw confidence, which looks like the answer the moment you see it. But during the 34 plateau
   frames the published `target_confidence` is **1**, giving 30 deg/s, not 10. I had the sentence written before
   I checked. A confidence of exactly 1/3 would have produced the observed clamp — which is presumably why this
   hypothesis is attractive, and why it needed the data.
2. **Soft-limit braking — rejected by measurement.** `max_speed_at` with valid limits is pure braking-to-soft
   (`safety_envelope.hpp:119-127`). At a plateau frame the reference sat at yaw **162.29°** with soft limits
   **−22.57° … 320.15°** — **157.87°** of clear travel. The braking model at that distance permits far more than
   10 deg/s.
3. **The envelope fallback `p_.v_max` — already excluded in round 37** (reached only when `!lim.valid`; and
   `max_speed_at` uses no cruise ceiling in the valid case, so round 37's reading survives this round's fuller
   read).

**What remains on the shortlist**, untested and therefore not claimed:

- `mode_proposal_.v_max_rad_s` (`control_loop.cpp:426`) — a mode-level speed proposal that may re-limit after
  line 844.
- The post-handover authority ramp: `control_loop.cpp:609-615` multiplies intent velocity/accel scale by a
  fraction for 300 ms after a mode change (round 22 measured 0.175 → 0.524 there). If something holds that
  fraction at ~1/3 for the whole dart, `30 × 1/3 = 10.00` exactly — the arithmetic fits, the mechanism is
  unproven, and the published `intent_velocity_scale` read 1.00 ("AUTH 100%") throughout, so either that field is
  not the applied factor or the factor is applied somewhere it does not feed back into telemetry.
- `track_v_max_rad_s` itself not being 30 at runtime. It is plumbed from `tracking.track_speed_deg_s`, which has
  **no key in `turret.yaml`** and so takes the code default of 30.0 — worth confirming from the daemon's own
  loaded config rather than from the defaulting call site.

**Why this round ends without a claim.** Rounds 28, 37 and this one each produced a confident mechanism from a
partial read; two were retracted and a third died in thirty seconds of arithmetic. The plateau itself is
established (34 frames at exactly 10.00 deg/s, zero acceleration), and so is its cost (round 38's C2/C3
accounting). The cause is not. The operator-facing documents say exactly that, and no limit should be touched on
this evidence. Next round: print the daemon's *loaded* tracking speed and the authority fraction from a
measurement path, rather than inferring them from source.

---

## Addendum, 2026-09-05 03:3x (round 40) — the plateau explained, at last, by a line the code comments over-describes

Measured, in order:

1. controld's own boot log: "tracking ENABLED (v_max track=30.0 deg/s search=10.0 deg/s ...)" and
   "payload profile: loaded 'conservative' (v_max pitch=20.1 deg/s, yaw=20.1 deg/s)". So the configured tracking
   speed really is 30, and the payload profile really is 20.1.
2. During all 34 plateau frames: track_state=tracking, mode_phase=TRACKING, confidence_band=HIGH,
   tracking_active=true, selected_track_id=700, selection_ambiguous=0. **Not** the search branch (round 39's
   hope) and not confidence (30 x 1 = 30).
3. control_loop.cpp:427, inside the AUTO_TRACK branch:

       mode_proposal_.v_max_rad_s = std::min(mode_proposal_.v_max_rad_s, hold_speed_effective());

   and control_loop.cpp:1713-1721: hold_speed_effective() = min(cfg_.hold_speed_rad_s, payload pitch v_max,
   payload yaw v_max) = min(10.0, 20.1, 20.1) = **10 deg/s**.

**So the full chain is:** 30 (track speed) x 1.0 (confidence) -> min with hold_speed_effective() = 10 ->
lim[i] = min(10, max_speed_at = large) = 10 -> reference pinned at exactly 10.00 deg/s with zero acceleration.
Every link is either a log line, a telemetry field, or an expression; none is inferred.

**Round 28's substance was right; round 37's retraction was right too, and both were incomplete.** Round 28 said
the hard-coded 10 deg/s hold speed caps AUTO_TRACK while the configured tracking speed is 30 - true. It blamed
SafetyEnvelope's set_v_max - false, which round 37 correctly killed along with the HUD row that repeated it. The
real mechanism is line 427. Round 39's three rejections all stand: confidence, soft-limit braking and the
envelope fallback are each ruled out by the numbers above.

**The comment above line 427 says "the payload profile v_max caps station motion", but the expression it
applies is hold_speed_effective(), whose binding term is not the payload profile (20.1) - it is the hard-coded
hold speed (10).** The comment describes the intent; the binding constant is something else, and the code is
explicit that the ordering is deliberate ("applied before the confidence derate ... the safer ordering"). So
this is a design decision, not a slip: a hold-mode speed constant is the ceiling on tracking, on purpose.

### What the operator now has to decide

- The acceptance rule demands LEAD: a 25 deg dart in 1.60 s needs ~15.6 deg/s average, ~17.4 peak. The station's
  effective tracking ceiling is **10 deg/s**. **C2 and C3 cannot pass at that dart size, at any tuning of
  prediction or tracker gains**, because the ceiling is applied to the reference before any of that.
- The 10 deg/s comes from `main.cpp:133`, hard-coded, with **no key in `turret.yaml`** - only the *unbound*
  `tracking.track_speed_deg_s` (30) is configurable today. So "make tracking keep up" is a safety-ceiling
  decision, not a tuning knob.
- Three legitimate resolutions, all the operator's: raise the hold/payload ceiling (a safety decision about how
  fast this station may swing), size the acceptance dart to what 10 deg/s can follow, or change the design
  intent at line 427 so tracking is capped by the payload profile (20.1) rather than the hold speed. **None
  should be taken by the agent that wrote this file.**

### Follow-up worth doing, not done this round

Publish `hold_speed_effective()` (deg/s) and label it as the effective station ceiling. It is the number that
actually binds, and it is not on the operator's screen: the panel currently shows ENVELOPE V-MAX "(not in
force)" - accurate, and useless as an answer to "why won't it keep up". The round-37 row was the right fix to a
wrong label; the right *number* is still missing.
