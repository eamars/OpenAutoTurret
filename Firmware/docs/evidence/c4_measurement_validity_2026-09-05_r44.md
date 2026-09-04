# C4 cannot measure what it claims to measure — 2026-09-05, round 44

No motion this round: the finding is in round 36's recording (`/tmp/tel_r36_dart.jsonl`) plus the probe's source.

## What C4 does

C4 ("no ringing") counts sign changes in the reference velocity around the recovery, and failed in both dart
runs (3 sign changes in round 27, 10 in round 36's data). It has been reported as evidence of oscillation ever
since.

## What the recording shows

Over the 91 frames where `mode_phase == TRACKING`:

| signal | p50 \|value\| | p95 | max | rate sign changes |
|---|---|---|---|---|
| target reported rate (°/s) | 1.02 | 16.08 | 17.41 | **21** |
| reference rate (°/s) | 7.36 | 10.00 | **10.00** | **10** |

**The signal being followed crosses zero twice as often as the signal doing the following.** A count of
reference zero crossings, in a window where the reference input crosses zero more often than the reference
itself, is not a measurement of ringing. The reference is the *smoother* of the two, which is the opposite of
what an oscillating loop looks like.

Two reasons the target channel wobbles, and neither is controller ringing:

1. **The probe's target genuinely holds still.** `tools/probe_track_loop.py` has no jitter, noise or random term
   anywhere (grep for `jitter|noise|random|uniform|gauss|drift` returns only comments about *feedback* noise),
   and `--hold-s` is documented as "how long the target holds still after the dart". So the wobble is introduced
   downstream of the fixture.
2. **The camera is mounted on the axis that is chasing.** While the reference still moves at 6–10 deg/s to repay
   the round-38 shortfall, a fixed world target's image position is changing, so the estimator's reported target
   bearing and rate change with it. During a fast approach, "target rate" is partly "how fast the mount is
   turning", not target motion. Layered on that is the estimator's own rate noise — around **1 deg/s at p50**
   while the target is provably stationary, which is a measured noise floor of the rate channel.

## What follows

**C4 as written cannot separate ringing from (a) estimator rate noise and (b) the geometry of a camera on a
chasing axis.** Reporting its failure as "the loop oscillates" is not supported by the data; the data show the
reference oscillating *less* than its input. The criterion is not a valid ringing test on this fixture, and
round 27's "C4 FAIL (3 sign changes)" should be read as "C4 inconclusive".

**A corrected C4, proposed — not implemented, because the acceptance criteria belong to the operator.** Sketch,
so the decision is concrete:

- Score only inside a **settled window**: after the dart, once |axis velocity| is under a stated threshold and
  the fixture target is stationary by construction — which is exactly the condition C2 already establishes when
  it says "recovered".
- Count zero crossings of the **aim error** (reference minus target, in one space — round 38's warning: not
  joint-minus-world), with a **deadband derived from the measured noise floor** (~1 deg/s rate, or the
  equivalent error band), not a fixed epsilon in source.
- Report **overshoot magnitude**, not just crossings: how far past the final bearing the reference goes, in box
  heights, so "no oscillation" has an amplitude as well as a frequency.
- Keep the existing metric printed alongside under its current name for one more round, so the operator can see
  both and choose; do not silently redefine a criterion mid-acceptance.

**Not established by this round:** that the loop does *not* ring. Only that the current measurement cannot tell.
The round-27/36 C4 failures should not be quoted as evidence of oscillation, and equally should not be quoted as
evidence against it.

Station untouched and healthy: homed, MANUAL/HOLD, READY, ceiling 10.0 deg/s on the panel, synthetic source
running. No code changed; **451 pytest / 57 CTest** stand.

## Round 52: a failed attempt at the known-truth test, recorded because the failure is the finding

C4 was read properly this round, and round 44's description of it was wrong in a way worth repeating: it scores

    signs = [1 if r["ex"] > 0 else -1 for r in hold_rows if abs(r["ex"]) > 1.0]      # ex = anchor px - centre px
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b);   c4 = flips <= 2

so it is the **anchor's horizontal offset from frame centre**, with a **1 px deadband** — not "aim error" as the
printout calls it, and not the reference-rate signal round 44 analysed.

**The attempt:** engage AUTO_TRACK against the running synthetic source, let it settle, sample 8 s, and count
flips where the truth is "nothing moves". **The premise was false.** Result: |ex| p50 595 px, p95 1045 px, max
1070 px — half a frame width from centre — with 6 flips. Either re-selection or non-engagement, but plainly not a
settled lock; and in any case the synthetic source's targets **move**, so the window was never a known-truth
segment. The analysis line printed "target static in world, axis settled" **before** the data existed: I wrote the
label for the experiment I had planned, not the one I ran. It is struck here rather than quietly deleted.

**What this does and does not establish.** It does not calibrate C4. It shows only that on a signal whose
excursions run to hundreds of pixels, a 1 px deadband cannot separate ringing from ordinary target motion or
re-selection — suggestive, not measured. **The test C4 actually needs** has to be run inside the dart probe's own
fixture, where the target is static by construction: hold the probe's static target with **no dart**, count flips
under the current rule, and use that count as the noise floor the post-dart count must exceed. That is a small
scenario addition plus one motion run; it is the next step for C4, and until it exists round 50's "FAIL (4 sign
changes)" stands as a measurement of a confounded quantity, neither confirmed nor withdrawn.

Station after the attempt: MANUAL/HOLD, READY, homed limits respected, synthetic source never stopped this round.
The brief AUTO_TRACK engagement moved the axis toward a synthetic target and was reverted to MANUAL; no controller
or config change was made.

## Round 53: the floor, measured — C4's bar of 2 is below its own noise

No new code: run the existing dart scenario with `--dart-deg 0`, which is a static hold on the fixture's own
stationary target. `/tmp/probe_r53_floor.log`.

    motion : 0.0 deg of azimuth in 1.60 s (0 deg/s), then hold 6.0 s
    hold-window aim error: p50 26.7 px / p95 44.4 px (0.071 / 0.118 of box height)
    aim-error sign changes after arrival: 11        -> C4 FAIL (bar 2)
    C1 containment: PASS (anchor and declared box edges)
    C2 recovery   : FAIL (t = 1.63 s, bar 1.50 s)

**Premise checked from the data, not asserted** — the lesson from round 52, where I printed "target static, axis
settled" before any measurement existed. Here the lock is demonstrably near centre (p50 26.7 px = 0.071 box
heights, comfortably inside the acceptance tolerance) with **no motion commanded at all**.

**Therefore: the flip metric reports 11 sign changes on a stationary, well-centred target.** The real 25 deg dart
of round 50 produced 4 — *fewer* than the no-dart floor — against a bar of 2. A criterion whose no-motion baseline
is 11 cannot certify or condemn anything near that number; C4's verdict is noise, and it is now quantified: **any
post-dart flip count must be judged against 11, not 2.** The mechanism is plain from the arithmetic: a 1 px
deadband on a signal whose jitter amplitude runs to tens of pixels (p50 26.7, p95 44.4) counts every wiggle.
Round 44 reached the right conclusion by the wrong route; this is the measurement that supports it.

### C2 carries a constant that has been read as a defect

With no dart whatsoever, recovery is reported at **t = 1.63 s**, already over the 1.50 s bar. Round 50's dart run
reported 2.32 s. So the part attributable to the dart is roughly **0.7 s**, and the criterion as written — measured
from dart start against an absolute bar — is dominated by a baseline (acquisition plus estimator convergence) that
fails the bar before any target moves. C2's FAIL is real but its cause has been misattributed to dart response.
Both numbers belong in front of the operator: the bar may need to be applied to the *incremental* recovery, or the
baseline reduced, and that is a criterion decision, not mine.

## Round 54: the floor reproduced exactly, and a contradiction between two fields of the same control loop

Three fresh no-dart runs (`--dart-deg 0 --hold-s 6.0`) and one repeat of the 25 deg dart. Logs
`/tmp/r53_floor_1.log`, `/tmp/r53_floor_2.log`, `/tmp/r53_dart.log`, plus round 53's original.

| | round 53 | floor run 1 | floor run 2 | dart repeat |
|---|---|---|---|---|
| C4 sign changes | **11** | **11** | **11** | 4 |
| hold aim error p50 (px) | 26.7 | 24.9 | 25.5 | 34.2 |
| C2 recovery (s, bar 1.50) | 1.63 | **1.61** | **1.61** | 2.27 |
| C3 lead (deg) | — | −0.275 | −0.229 | −4.702 |

**C4's floor is 11 on every no-motion run, identically** — and the real dart produced 4 both times, *below* its own
floor. The metric does not measure ringing; it measures jitter, and it did so reproducibly.

**C2's baseline is 1.61 s, reproducibly** — already past the 1.50 s bar with no dart commanded. Subtracting it, the
25 deg dart costs about **0.66 s**, not 2.27 s.

**And C6 fails with nothing moving at all.** All three no-dart runs report the reference outside its rate ceiling,
worst **17.8 deg/s from a 0.653 deg reference move over 37 ms at t = 7.79 s (hold)** and 16.9 deg/s at t = 3.82 s
(hold). Two readings of that, not yet resolved, and the difference matters:

* the published reference **position** really can outrun the station's 10 deg/s ceiling inside one sample interval,
  which would be a controller-side defect and directly relevant to smoothness; or
* the probe's sampled `dt` is biased enough to inflate the implied rate — but the tool already allows +10 %, and
  this is 1.77x, which that allowance does not cover.

Against both readings sits the daemon's own `q_ref_rate_yaw_rad_s`, which the same dart run reports as never
exceeding 10.0 deg/s. **So two fields published by the same control loop contradict each other**: position steps say
17.8, the rate field says 10.0. C6's FAIL has therefore been measuring that contradiction, not target-following.
Resolving it needs a comparison at the daemon's own 200 Hz timebase rather than at the web bridge's sampled one —
a next step, recorded as open, not a conclusion.

## Round 55: the contradiction is resolved, and it belongs to the probe, not to controld

Same question, measured at the daemon's own publish rate (`/tmp/tel_r36_dart.jsonl`, `telemetry_stream` at 15.2 Hz,
sample interval **65.7 ms**). For every consecutive pair of samples where the reference was actually moving, the
rate implied by the position step is compared with the rate controld publishes for the same instant:

    moving sample pairs: 88
    ratio implied / published rate:   p50 0.96   p95 1.00   max 1.01
    pairs exceeding +10%:             0 of 88

**Published position and published rate agree to one percent.** There is no controller-side inconsistency: the
reference does *not* outrun its own ceiling.

The 17.8 vs 10.0 deg/s contradiction from round 54 was the probe's sampling. Its dart loops sample the web bridge at
roughly **37 ms** while the bridge publishes at **65.7 ms**, so a pair can be timestamped 37 ms apart while the
position difference spans the longer real interval. 65.7 / 37 = **1.78**, against the measured excess of **1.77** —
the whole discrepancy is accounted for by arithmetic, which is the sign of a correct explanation.

**So C6 has been wrong in both directions inside one tool.** Round 47 found its ceiling was ~20.1 instead of the
binding 10 — too lenient, and fixed in round 49. Rounds 50/53/54 found its position-derived rate inflated by
sampling skew — too strict, unresolved at the time. **The FAILs recorded in those runs are therefore not evidence
about the station**: on no-motion runs they were an artifact reproducing at the bridge's sampling ratio, and the
station's own stream shows the limiter doing its job at exactly 10.0 deg/s.

**What C6 should do:** judge the ceiling against `q_ref_rate_yaw_rad_s` as published — the quantity the rate limiter
actually bounds, sampled and reported on the daemon's timebase — and keep the position-derived figure only as a
diagnostic, labelled with the skew bias and the ratio that produces it. A rate is not measured by differencing a
bridge snapshot faster than the bridge publishes.

Status of C6 after this: the reference **is** within its rate ceiling (0 of 88 moving samples over +10%, ratio p50
0.96), so the criterion passes on the station's own evidence; the probe's verdict line remains a tool defect until
it is changed to use the published rate. Not a signature, and not a claim about dart-following ability — the ceiling
itself (10 deg/s, and the C3 lead deficit) is unchanged by all of this.

## Round 56: C6 reworked to the published rate — and the new verdict is not trustworthy yet either

Change applied and executed: the differenced figure is demoted to `DIAGNOSTIC ONLY ... skews high by the
publish/sample ratio`, and a new line judges the ceiling against controld's published `q_ref_rate_yaw_rad_s`,
+5% for float dust since there is no denominator left to be wrong. Both anchors matched exactly once, the file
compiles, **461 pytest** pass, and the path was executed rather than admired — two runs, floor and dart.

    C6 rate legality (published q_ref_rate_yaw_rad_s; ceiling 10.0 deg/s, controld ... ):
        floor: PASS (max 0.2 deg/s, no sample above the ceiling plus 5%)
        dart : PASS (max 0.2 deg/s, no sample above the ceiling plus 5%)

**Those PASSes are hollow and are not offered as evidence.** A 25 deg dart against a 10 deg/s ceiling must show a
reference rate near 10, as round 54's run of this same tool printed from the same field
(`rate : p50 8.4  p95 10.0  max 10.0 deg/s`). Here the maximum of the collected list is **0.2 deg/s**, which means
the sample set my new line consumes is nearly empty or nearly zero — precisely the always-pass shape round 48's
zero-height test was written to fear. The verdict is technically what the data say and useless as a measurement.

Open, named: why `ref_v` yielded a different population this run than in round 54. Candidates are sampling density
(fewer samples, mostly at rest), the row field being absent in the window that happens to be retained, or my
block reading `all_rows` at a point where it holds fewer rows than the profile print does. Whichever it is, the fix
needs a **minimum-sample guard** — a rate-legality verdict on fewer than some number of moving samples must print
`INSUFFICIENT DATA`, not `PASS`. Until then C6 has no working verdict from this tool at all: too strict when
differenced (rounds 50/53/54), too lenient on ceiling (round 47), now too empty.

Other numbers from the same two runs, for the record: floor C4 **11 flips** again, C2 **1.62 s**; dart C4 **3**
flips, C2 **3.05 s**, C3 **-11.943 deg**. **C3's spread across three dart runs is -4.7, -6.4 and -11.9 deg** — the
sign is stable, the magnitude is not, and no single C3 number should be quoted as if it were a constant.

Station: MANUAL/HOLD, READY, synthetic source running. Probe-only change; controller and HUD untouched.

## Round 57: the always-pass shape is closed, and thinness is now printed instead of inferred

`rate_verdict(rv, ceiling, min_moving=20, tol=1.05, idle_eps=0.5)` decides C6, and it **cannot** return PASS on
data too thin to say anything:

* fewer than 20 **moving** samples (above half a degree per second — samples at rest say nothing about a 10 deg/s
  ceiling, which was precisely round 56's shape: ninety samples of 0.2 deg/s) → `INSUFFICIENT DATA`, with the
  counts printed;
* no usable ceiling value → `NO CEILING`, not a pass;
* otherwise PASS/FAIL with the moving-sample count, the maximum, the ceiling and how many samples exceeded it.

Seven unit tests in `tools/tests/test_probe_rate_verdict.py`, including the round-56 defect asserted by name
(`assertNotIn("PASS", v)` on an empty input) and an assertion that **every** verdict names its sample count — so
the next emptiness shows up in the output rather than as a suspicious maximum someone has to notice afterwards.
The decision is a pure function, which is why it can be tested without moving the station; that is the only reason
a guard like this is worth having rather than wishing for.

`py_compile` clean, the print site untouched and confirmed wired at line 1011, **7 new tests pass**. **No hardware
run this round**: what changed is the decision logic and that logic is exercised by the unit tests; the first dart
after this will show whether `ref_v` really is thin, and it will say so in words. That question — why round 56 saw
max 0.2 deg/s where round 54 saw max 10.0 from the same field — is still open, and closing the escape route is not
the same as answering it.

## Round 58: the 0.2 deg/s was my code, not the station — the same run contradicts itself

From the round-56 logs, same file, same run, twenty lines apart:

    r56_dart.log   C6 rate legality ... PASS (max 0.2 deg/s, no sample above the ceiling plus 5%)
    r56_dart.log   rate : p50 8.5  p95 10.0  max 10.0 deg/s   (track limit 30 deg/s)
    r56_floor.log  C6 rate legality ... PASS (max 0.2 deg/s, ...)
    r56_floor.log  rate : p50 5.9  p95 9.8   max 10.0 deg/s

Both statements came from the analysis block that reads `rv` (defined once, line 1002, and used by the profile
print at line 1022). A list cannot have a maximum of 0.2 and a maximum of 10.0. **So round 56's `max 0.2 deg/s`
was a defect at my verdict site, not a thin sample set from the station** — the reference really did reach
10.0 deg/s, exactly as the ceiling predicts, in both runs.

That reverses the sign of round 57's conclusion in one respect, and it is worth being exact about which part
survives:

* **Survives:** a verdict must state its evidence and must not PASS on thin data. `rate_verdict` stays, with its
  `INSUFFICIENT DATA` and `NO CEILING` paths and its sample counts — that is a real improvement in the tool.
* **Does not survive:** the reading that the station's published rate was empty. It wasn't. The number printed by
  the profile line was right all along; the verdict line was reading or transforming something else.

**Next step, concrete:** the verdict site and the profile site must be made to share one accessor, and the first run
after that should print the same maximum in both places — if it does not, the defect is visible immediately rather
than only after a PASS has been believed. Until that equality holds, **C6 still has no working verdict from this
tool**, whatever it prints.

What the corrected reading implies for acceptance, if it holds after the fix: the reference reached but did not
exceed its 10.0 deg/s ceiling during a 25 deg dart, which is round 40's plateau measured once more — a *ceiling*
problem, not a *rate-limit-violation* problem. The ceiling remains the operator's decision.

## Round 60: the two sites agree, and C6 has a working verdict for the first time — the reference sits exactly on its ceiling

Clean run, synthetic source stopped as in every successful dart run, `/tmp/r60_dart.log`:

    C6 rate legality (published q_ref_rate_yaw_rad_s; ceiling 10.0 deg/s, controld ... in force):
        PASS (148 moving samples, max 10.0 deg/s against a 10.0 deg/s ceiling +5%, 0 over)
    rate : p50 7.4  p95 10.0  max 10.0 deg/s   (track limit 30 deg/s)

**Same maximum at both sites, 10.0, on 148 moving samples.** Round 56's hollow `PASS (max 0.2 deg/s)` was the
missing rad→deg conversion; with the conversion in one shared accessor the verdict and the profile cannot disagree,
and the sample count says the data were never thin. The differenced figure still prints FAIL, now labelled
`DIAGNOSTIC ONLY ... skews high by the publish/sample ratio` — a measurement with a known bias, kept visible and no
longer a verdict.

**What C6 now says about the station:** during a 25 deg dart the reference reached **exactly 10.0 deg/s** and
exceeded it in **0 of 148** moving samples. The limiter is doing its job; the station is not smooth-thwarted by
overshoot but **bounded by the ceiling itself** — round 40's plateau, confirmed by a corrected instrument rather
than inferred from a line of source. C6 was never the station's failure: it was two defects in one tool (wrong
ceiling in round 47, wrong units in round 56), both now closed.

Other verdicts from this same run, for continuity: C2 FAIL t = 2.34 s (baseline 1.61 s, so ~0.7 s attributable to
the dart), C3 FAIL p50 lead **−5.715 deg** — the fifth dart run, whose leads read −11.6, −6.4, −4.7, −11.9, −5.7:
**sign stable, magnitude between 4.7 and 11.9 deg**, so the deficit is real and its size is not a constant;
C4 **4** sign changes, still below the measured no-motion floor of 11 and therefore not evidence of ringing.

That leaves the acceptance question where round 40 put it, now with trustworthy instruments: the reference does
what the ceiling allows, the ceiling is 10.0 deg/s, and the operator's lead criterion needs roughly 15.6 deg/s
average. The choice — raise `tracking.hold_speed_deg_s`, size the dart to what 10 deg/s can follow, or accept C3
as a documented consequence — remains unmade and unsigned.
