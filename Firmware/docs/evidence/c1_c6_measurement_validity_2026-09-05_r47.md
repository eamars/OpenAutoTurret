# C1 and C6 are measured against the wrong predicates — 2026-09-05, round 47

No motion this round: probe source plus the existing recording. This is the third and last pass of "check the
instrument, not just the number" (C4 invalidated in round 44, C5b validated in round 45, head-aim traced in
round 46).

## C1 "the target never leaves the frame" tests a point, not the target

    tools/probe_track_loop.py:241   outside = not (0.0 <= un <= 1.0 and 0.0 <= vn <= 1.0)      # the ANCHOR, normalised
    tools/probe_track_loop.py:636   "in_frame": (0.0 <= u <= FW) and (0.0 <= vv <= FH)         # the aim PIXEL
    tools/probe_track_loop.py:675   c1 = not out_of_frame
    tools/probe_track_loop.py:839   c1 = not any((not r["in_frame"]) or r["outside"] for r in rows)

Every predicate is about the **anchor point**. The declared box is `BOX_H_NORM = 0.35` — about 378 px of a
1080-px frame — so the anchor can sit exactly on the frame edge with **189 px of target hanging outside** and
C1 still reports PASS. The objective's words are *"so the target does not leave the frame"*; what is scored is
*"so the declared anchor does not leave the frame."* **C1's PASS is therefore an upper bound on containment, not
containment.** For a criterion whose whole purpose is frame-exit margin, the margin should be measured from the
box corners (four corners, or at minimum the leading edge in the direction of travel), and reported as the
closest approach of the box to the frame edge — then the operator sees "the box came within 40 px of the edge",
not a binary that turns on a single point.

## C6 certifies darts against a ceiling that is not the one that binds

    tools/probe_track_loop.py:913   ceiling = 30.0 * (min(scales) if scales else 1.0)

30.0 is the **configured tracking speed** (which `turret.yaml` doesn't even set — 30.0 is the loader default),
scaled by the payload profile. With the `conservative` profile that gives roughly **20.1 deg/s**. Round 40
established that the ceiling actually applied to the reference is `hold_speed_effective()` = **10 deg/s**
(`control_loop.cpp:427`), and round 43 put that number on the panel.

So the pre-check prints *"envelope legal"* for darts the controller is forbidden to follow. **That is exactly the
puzzle from round 27**, whose evidence file is literally named `dart_25deg_in_1.6s_envelope_legal_...` and which
then failed C2/C3: the dart was legal against 20.1 and impossible against 10. C6 has been reporting PASS while
answering a different question from the one acceptance asks. It should compare against the **binding** ceiling —
the value controld now publishes as `effective_speed_ceiling_deg_s`, not a constant reconstructed in Python.

## What still stands

The dart verdicts after three rounds of instrument checks: **C1 PASS but point-only (weaker than claimed) · C2
FAIL · C3 FAIL · C4 inconclusive · C5a PASS · C5b FAIL, validated against a 4.2 °/s³ noise floor · C6 PASS
against the wrong ceiling.** Two of the seven criteria are now known to be measured wrongly in the *optimistic*
direction, one is inconclusive, and none is known to have been measured wrongly against the station — the errors
run in the direction of making the station look better, which is the direction worth being paranoid about.

None of this changes controller behaviour, and none of it is a §24/§110 signature. Correcting C1 and C6 means
re-running the dart once the predicates are fixed, which is the operator's call on the criteria and a probe
change, in that order.

Station untouched: homed, MANUAL/HOLD, READY, `SPEED CEILING 10.0 DEG/S` on the panel, synthetic source running.
**451 pytest / 57 CTest** stand.

## Executed at last — round 50, same day: the corrected instruments ran against real telemetry

`s3`, 25.0 deg in 1.60 s then 3.0 s hold, log `/tmp/probe_r50.log`. Both corrections behaved as intended, and the
run also produced two corrections of mine.

| criterion | round 50 (corrected instruments) | previous (2026-09-04/05) |
|---|---|---|
| C1 containment, anchor | **PASS** | PASS |
| C1 containment, declared box edges | **PASS** (never previously measured) | — |
| C2 recovery | **FAIL** t = 2.32 s (bar 1.50 s) | FAIL 2.85 s |
| C3 lead | **FAIL** p50 −6.428° (ahead in 0% of 41 samples) | FAIL −11.6° |
| C4 no ringing | **FAIL** 4 aim-error sign changes (bar 2) | "inconclusive" (wrongly, see below) |
| C5a accel | **PASS** p95 60.0, max 260.6 | PASS |
| C5b jerk | **FAIL** p95 540 (bar 300+60), max 6803 | FAIL p95 525 |
| C6 rate legality | **FAIL** 15 of 156 steps over the ceiling in force | "PASS" against ~20.1 |

**C6 changed verdict because the ceiling changed source.** It printed
`ceiling used for legality: 10.0 deg/s [controld effective_speed_ceiling_deg_s (the ceiling in force)]` and the
worst step it objected to was **18.0 deg/s** — under the old reconstructed 20.1 that step was invisible, which is
precisely how round 27 got a "legal" dart that could not be followed. The number that decides acceptance is the
one controld applies, and now it is the one being used.

**C1's box verdict passed**, so round 47's concern did not bite for this dart — a point worth having either way:
the worry was real arithmetic, and the outcome is a measurement rather than an assumption.

### Two corrections this run produced

1. **Round 44's withdrawal of C4 was made on the wrong evidence.** C4 scores **aim-error sign changes after
   arrival** (the criterion text says so, and the verdict line reports 4 against a bar of 2). Round 44 instead
   analysed zero-crossings of the *reference rate* against the *target rate* and concluded the instrument was
   contaminated. That is a different signal, so the conclusion does not belong to C4 — the sign-off sentence has
   been restored to FAIL, with the misjudgement named where it stands. The round-44 observations about a chasing
   camera and a ~1 deg/s rate noise floor remain true as measurements of those signals; they simply are not what
   C4 scores. The right validity test for C4 is round 45's method — measure aim-error crossings on a segment
   whose truth is known — and it has not been done.
2. **The C6 print label still says `30 deg/s x live derating` while the value now comes from controld.** My
   replacement anchor did not match the source text (count 0) and I left the file alone rather than editing blind,
   so the label remains wrong in the printed output while the arithmetic is right. Outstanding, cosmetic in the
   worst sense: a stale label on a safety number.

Re-running the dart moved C2 from 2.85 s to 2.32 s and C3 from −11.6° to −6.4°, with the reference rate profile
still capped at exactly 10.0 deg/s — the same plateau, the same ceiling, run to run. Station restored afterwards:
MANUAL/HOLD, READY, synthetic source running. **461 pytest / 57 CTest** stand.
