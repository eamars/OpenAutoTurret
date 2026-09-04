# Operator status — OpenAutoTurret v3.2, 2026-09-05 (round 67)

Consolidation, not measurement. Every figure below was produced and recorded earlier in this session; each line
points at where. Nothing here is signed, and nothing here asks permission I already have — it lists the decisions
that are **not mine to make**, with their measured consequences.

## 1. Acceptance criteria, as the corrected instruments report them

| # | criterion | status | the number, and where it came from |
|---|---|---|---|
| C1 | target never leaves frame | **PASS** | anchor *and* declared box edges, `dart` and `sized-dart` runs (`/tmp/r63_sized.log`); box metric added round 48 after round 47 found the old predicate tested only the anchor (`docs/evidence/c1_c6_measurement_validity_2026-09-05_r47.md`) |
| C2 | recovery inside 1/3 box within 1.50 s | **FAIL, but mis-specified** | dart runs report 2.27 / 2.32 / 2.34 s; a **0.0 deg dart reports 1.61–1.63 s** — the baseline alone breaches the bar (`/tmp/r53_floor_*.log`, `c4_measurement_validity…r44.md` round 53/54) |
| C3 | reference leads the target | **FAIL** | 25 deg darts: **−11.6, −6.4, −4.7, −11.9, −5.7 deg** across five runs — sign stable, magnitude 4.7–11.9; a ceiling-sized 12 deg dart gives **−1.74 deg** (`/tmp/r63_sized.log`) |
| C4 | no ringing | **INVALID AS IMPLEMENTED** | with a 0.0 deg dart and a locked target the present rule reports **11 sign changes** — reproduced three times identically — while real darts give 3–4, i.e. **below their own floor** (same evidence file, rounds 53/54) |
| C5a | acceleration within 60 deg/s² | **PASS** | p95 at the limit, max 260.6 during the dart (round 50/63 logs) |
| C5b | jerk within 300 deg/s³ | **FAIL, validated** | p95 **525–540**; the estimator's own noise floor is **4.2 deg/s³** on frames where jerk is zero by construction, and the violation barely moves with dart size (540 at 25 deg, 539 at 12 deg) → points at the reference profile, not at target following (`docs/evidence/ref_rate_plateau…`, round 45/63) |
| C6 | reference within its rate ceiling | **PASS** | **148 moving samples, max 10.0 deg/s against a 10.0 deg/s ceiling, 0 over** (`/tmp/r60_dart.log`). Its earlier FAILs were two defects in the probe: wrong ceiling (round 47), rad/s compared against a deg/s ceiling (round 56). Both fixed; the differenced figure is now labelled `DIAGNOSTIC ONLY` |
| — | reticle on the target **HEAD** | **PASS while holding** | all 91 TRACKING frames carry `target_aim_is_head / target_aim_valid / aim_point_valid`; the probe labels which anchor it scored (`"head aim"`, not `"anchor fallback"`). Limit: the head anchor here is what the **fixture declared**; no real detector has ever run on this station (round 46) |

## 2. The cause of the C2/C3 failures is a ceiling, not a tuning error

The tracking reference is capped at `hold_speed_effective()` = **min(hold speed 10, payload profile 20.1, 20.1) =
10 deg/s**, applied at `control_loop.cpp:427` **before** the confidence derate, so no tracker or prediction tuning
can exceed it. Confirmed by measurement: the reference sits at exactly 10.00 deg/s with zero acceleration during a
dart (rounds 38–40), and controld now publishes the value (`effective_speed_ceiling_deg_s`) and the panel prints
**`SPEED CEILING 10.0 DEG/S (min of hold + payload profile)`** (rounds 41/43).

The operator's lead criterion — 25 deg in 1.60 s — needs about **15.6 deg/s average**. It is not reachable at
10 deg/s. That is why C3 fails and why C2 recovers slowly; it is not a control-loop defect.

**Three options, all reversible, none taken by me:**

1. **Raise the ceiling.** `tracking.hold_speed_deg_s` exists as a config key (round 43), defaulting to exactly the
   old 10.0 so nothing changes until written. It is a statement about how fast this station may swing under
   payload, and it needs the payload/limit review that only the operator can do.
2. **Size the acceptance dart to what 10 deg/s can follow.** Round 62's tests and round 63's run measured a 12 deg
   dart: legal at the ceiling, C3 deficit shrinks to −1.74 deg, containment passes — but **C2 still fails on its own
   baseline and C5b jerk still fails**, so this option does not make those two criteria meaningful.
3. **Record C3 (and C5b) as documented consequences** of a safety ceiling, and accept the station as built.

## 3. Two criteria are waiting on a definition, not on hardware

* **C4** cannot be scored until a replacement is adopted. `oscillation_verdict()` is implemented, tested (7 tests),
  is implemented, tested (7 tests), and **printed beside C4 as a labelled diagnostic** (wired in round 69; the
  exit code still scores C4 exactly as written). It counts reversals beyond the *measured* jitter ruler (49 px,
  the p95 of a motionless hold) and reports peak excursion in box heights, so "ringing" would have amplitude
  as well as frequency. **Round 70 gave it the test that killed the present rule — a segment whose truth is
  known.** On a 0.0 deg dart, where nothing moves, the present rule reports `FAIL (5 sign changes)` while this
  one reports NO OSCILLATION DETECTED (0.130 box heights); on a 12 deg dart, converging, peak 0.269 box
  heights; on the 25 deg acceptance dart, converging, peak 0.818 box heights. Monotone in difficulty, never
  ringing on a motionless target, and it does not launder a large miss into ringing either. On the evidence so
  far this station does **not** ring; what adoption changes is the `<= 2 sign changes` text, which is the
  operator's call (`/tmp/r70_floor.log`, `/tmp/r70_dart.log`, `/tmp/r69b.log`).
* **C2** should probably be scored on the **incremental** recovery (dart-attributable ≈ 0.7 s) or its 1.61 s
  settling baseline reduced. Either is a criteria decision.

## 4. Not done, and what it needs

* **Principal point and boresight** (`calibration/camera_intrinsics.yaml` says so in its own header): fx/fy are
  encoder-theodolite **measured**; cx/cy are the **geometric centre by convention**, and boresight is uncommissioned.
  Needs a surveyed distant reference or a ChArUco board at wide spans for the board route — physical work at
  the station (`tools/calibrate_camera_intrinsics.py`, `make_charuco_board.py`, `probe_theodolite.py`).
  **Round 72 found that cx does not need the board**: `docs/principal_point_method_2026-09-05_r72.md` works
  out that the angular scale the existing theodolite already measures is a parabola in u whose vertex *is*
  cx, that the model reproduces this station's own 24.22 px/deg measurement to 0.1%, that the curvature at
  the frame edge is +18.7% — **RETRACTED by measurement in round 74: the curvature is real and its sign was confirmed, but recovering cx from it amplifies scale error through a square root — this rig's 8.7% residual moves cx by ~148 px at the best band, worse than the convention it would replace** — and
  that a 100 px error in cx puts ±2.9° of error on frame-edge bearings, which is what C1 is scored against.
  `--strip-at-u` now exists and the walk was run; the method is what failed. **cx therefore
  still needs the ChArUco board route** (≥8 views, board moved by hand) — there is no cheap
  theodolite path after all. Boresight still needs a surveyed reference: the theodolite
  measures differences, not absolutes.
* **Real detector**: every centring and head-aim number above is against the synthetic fixture. Plumbing evidence,
  not acceptance evidence.
* **Hardware acceptance and §24 visual fidelity**: **0 accepted by a named person.** Both counts were
  re-derived today by parsing the specs: §24 (`# 24. Acceptance criteria — visual fidelity`, an H1 — my
  earlier `^##` pattern is why round 67 could not find it) holds **16 checklist items**, matching the **16
  numbered rows** in `docs/acceptance_signoff_v3_2_visual.md`; §110 (`# 110. V3 target-state acceptance
  checklist`) holds **30 items**. Round 67 recorded the §110 figure as remembered-from-round-30 and
  unverified; it is now measured, and the round-30 memory of "30" is confirmed.
  `docs/acceptance_signoff_v3_2_visual.md` disqualifies itself on its face and carries a read-first pointer
  plus a round-61 addendum correcting its superseded dart numbers.
* IMU: **none installed**, reported as `IMU ABSENT` on the panel rather than emulated.

## 5. State of the machine and the tree as of this writing

* Suites: **484 pytest / 57 CTest**, green as reported at the round-66 commit; test runs can no longer attach to the
  live station's vision socket (`tools/fake_vision.py` refuses without an explicit socket, round 65).
* Station: controld up, `MANUAL / HOLD`, `at_ready`, ceiling 10.0 on the panel, synthetic vision source running;
  200 Hz loop overrun (~54 µs) is forgiven by design and its state is visible as `LOOP DEADLINE 0/5 (+2000us grace)`.
* The controller's behaviour, limits, and dart defaults were **never changed** by me: every change in this stretch was
  instrumentation, tests, telemetry, the config key, and documentation.

## 6. Safety findings from rounds 73–78 — this section postdates everything above

Written last because it is the part that should be read first. No acceptance criterion is involved; it is about what
the station did and what the record said about it.

* **Uncommanded motion in `MANUAL / HOLD`, not attributed.** Round 73, during a theodolite walk: after a probe aborted,
  yaw travelled tens of degrees and then wandered ±6° between samples with the target cleared and the synthetic source
  stopped — nothing I could point at was commanding it. `at_ready` flickered. I stopped motion authority and rebooted;
  the first ~140° of motion after boot was **`phase=homing`**, which is normal. **A properly homed, `ready` HOLD is
  quiet: 149.075 → 148.966° over 30 s.** The episode has not recurred and I am not claiming a cause — but a long-running
  never-rehomed station moved without a command, and that asymmetry (unhomed moves, homed does not) is worth more than
  my speculation. The axis stayed inside software limits throughout (max ≈244° against 320.2°).
* **Motion authority was cut 98 times while holding.** `/var/lib/ota/blackbox` holds **98 `BRAKE_in_hold`** scenes
  (plus 30 in homing) from one session, 08:11→15:46. This is not log spam: §80 preserves a scene on the **edge** into
  unsafe only, by explicit design ("a station sitting in a brake for a minute should hold one record, not twelve
  hundred"). The supervisor's own reason, printed by the round-78 test: **`stale or missing motor feedback`**. The
  mechanism is documented at `can_motor_backend.cpp:250-261`, where a keepalive ping exists precisely to stop this —
  and it says each BRAKE **"stomps the other axis's reference"**, which is a credible account of the ±6° above.
  **For the operator to judge:** is 98 feedback-loss brakes in a day of probing acceptable, or is the keepalive
  under-powered? That is a safety-margin question on hardware I do not own the risk of.
* **The record said the opposite of what happened — fixed, and the fix is tested.** Every one of those 98 scenes
  recorded `safety_action: ALLOW` while its reason said BRAKE. Cause: `preserve_scene(snap, …)` ran at
  `control_loop.cpp:1242` while `snap.safety_action` was assigned at 1366 — later in the same pass — and since an
  edge is a transition *from* safe, the archived action was always the last permissive one. Round 76 hoisted the
  assignment (same value, same cycle, now before its first reader; nothing else reads the field; published value
  unchanged). Round 78 added `BlackBox.ABrakeEdgeArchivesTheActionThatActuallyCausedIt`, which silences a drive through
  the documented sim hook and takes the real edge — and **the test was shown to FAIL with the fix removed and PASS
  with it restored**, then confirmed to run under `ctest` (whose entries are per-binary, so the count of 57 proves
  nothing on its own).
* **Therefore: do not trust `safety_action` in scenes written before the fix.** The 128 existing artifacts still carry
  the previous cycle's action; they are otherwise usable (selection, candidates, phase, q_ref/q_actual are all
  genuine). Only that one field is systematically stale.
* **What I did not change:** no limit, ceiling, tolerance, or behaviour. The fix is record integrity. `§22` presentation
  and `§25` staleness behaviour are unaffected on the live path — the assignment happens before publish, so the panel
  saw the right thing; it was the archived evidence that lied, which is what gets mailed in and argued over later.
