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
