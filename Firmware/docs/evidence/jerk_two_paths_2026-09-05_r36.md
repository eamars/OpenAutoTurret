# Jerk measured two independent ways — and what it did to the "10 deg/s cap" — 2026-09-05, round 36

Same station, same scenario (`s3`, 25° over 1.60 s, 3.0 s hold, envelope-legal, vision source stopped as the
probe requires). Nothing here is acceptance; the criteria verdicts are the probe's, and the operator signs.

## Why this was worth measuring

C5b's jerk figure had always been reported from one place: the dart probe differentiating telemetry samples.
Its own printout hedges — *"derived at probe rate, a lower bound"*. A smoothness criterion that rests on one
differentiation path, when the sampling grid is coarser than the transient, is a criterion nobody can sign.
`tools/telemetry_stream.py --jsonl` records the daemon's published frames directly, and those frames carry
`q_ref_rate_yaw_rad_s` and `v_yaw_rad_s` — the controller's own numbers. Two paths, one signal.

Recording rate, measured not assumed: **484 frames over 32.0 s = 15.2 Hz** publication (field is `ts_ns`).

## The two paths agree where it counts and disagree where they can't

| statistic | probe path (HTTP polling of the same field) | daemon path (socket stream, 15.2 Hz) | bar |
|---|---|---|---|
| jerk p50 | 147 °/s³ | **140.9 °/s³** | 300 (+60 grace) |
| jerk p95 | 542 °/s³ | **524.9 °/s³** | 300 (+60 grace) |
| jerk max | 1892 (round 27 said **576**, same parameters) | **9755** | — |
| accel p95 | 60.0 (saturated) | 34.1 (max 381) | 60 |

**C5b's failure is real.** p95 agrees to within 3% across two independent sampling paths, two different grids
(~20 ms and 66 ms), and two different transports. Jerk of ~525–542 °/s³ against a configured
`max_jerk_deg_s3: 300` is a genuine ~1.8× violation of the smoothness rule, not an artifact of differentiation.

**"Max jerk" is unquotable and I should stop printing it.** The same scenario has now produced 576, 1892 and
9755 °/s³ from the same underlying motion. That is not a signal, it is the differentiation of a grid the
transient does not fit. My earlier hedge — "a lower bound" — was the wrong hedge: it implied the number had a
direction. It doesn't. p50 and p95 do.

Accel p95 differs (60 saturated vs 34.1) because the two paths integrate over different windows: the probe's
finer grid catches the plateau, the 66 ms grid averages across it. Neither is wrong; they answer different
questions. Only the finer grid can speak to saturation, and both agree the acceleration limit is being touched.

## What this did to round 28's conclusion

**This run's published reference rate peaked at 18.56 deg/s, and actual velocity at 20.43 deg/s.** Round 28
concluded — and a commit says so — that AUTO_TRACK's reference *"can never exceed 10 deg/s — one third of its
own configured 30"*, from the envelope cap being set to the hard-coded 10 deg/s hold speed. A measured 18.56
deg/s reference rate contradicts that directly. It is not a difference in source: the probe reads
`q_ref_rate_yaw_rad_s` at `probe_track_loop.py:804`, the same field the stream recorded.

So the causal claim must be **withdrawn pending reconciliation**, and with it the advice attached to it — that
the fix is to raise the envelope's hold-speed cap. Acting on that could change safety-envelope behaviour for
nothing. Open hypotheses, in the order worth testing:

1. `env_.set_v_max(cap)` bounds something other than what generates `q_ref_rate_yaw_rad_s` — the code comment
   says it "bounds the tracking reference", and comments are not evidence when the measurement disagrees.
2. The cap is applied at a stage that a later stage can exceed (e.g. a limit applied before a re-planning step).
3. The earlier "max 10.0" was a different statistic than I reported it as (p95 versus max), or a different run
   state — payload derate, FOR clipping, or a starting position where 10 was reachable rather than limiting.

What remains genuinely true from round 28: the hard-coded `hold_v_max_rad_s = 10.0` in `main.cpp:133` and
`track_speed_deg_s` defaulting to 30 in `turret_config.cpp:713` are real, and they do disagree by construction.
What is **not** established is that this disagreement is why C2/C3 fail. Measured evidence now says the
reference moved faster than that cap would allow.

## Consequences

1. **C5b stands as a measured failure** with two agreeing instruments. The number to quote is **p95 ≈ 525–542
   °/s³ vs 300**, never a "max".
2. **C2/C3's explanation is back in the open.** The 10 deg/s story is contradicted by direct measurement of the
   field it was about. The operator should not be handed "raise the envelope cap" as a fix until the
   generator→limiter chain is read end to end.
3. The recorder is now a known-good instrument: 15.2 Hz, 128 fields per frame, `ts_ns` timestamps. Any future
   smoothness claim should be recorded this way alongside the probe, not derived from one path alone.

Probe verdicts on this run were unchanged (exit 1: C2/C3/C4/C5b fail, C1/C5a/C6 pass; envelope check printed
"achievable"). Station returned to MANUAL/HOLD, homed, vision restarted and verified. No code was changed.
