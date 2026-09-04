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
