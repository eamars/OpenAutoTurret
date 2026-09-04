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
