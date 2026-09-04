
---

## 2026-09-04, 00:20 — a ghost on the candidate list was selectable, and now it is not

Chasing one of the tracking items on the station turned up something better than the item.

**The defect.** The detector process died; four candidate tracks stayed listed for over four
minutes, two of them still `CONFIRMED`, and `select_target` **accepted one of them** —
`ack 'selected Person #2'`, `selected_uuid_valid=true`, `selection_visibility=VISIBLE` for
something last seen 240 s earlier. It did not move the turret only because tracking was off and
the LOS had decayed to zero; in AUTO_TRACK it is a slew toward a phantom, inside the soft limits.

**Why, exactly:** every staleness rule in `TargetSelectionManager` runs *inside* `observe()`, so
a producer that stops publishing never triggers one again and the last list received is current
forever. The unit test that appeared to cover this reached staleness by feeding 400 **empty**
frames — which keeps the producer alive. That is the case that already worked. Meanwhile
`control_loop` had been computing `last_set_receive_ns_` for `track_list_age_ms` the whole time:
**the object that knew the age and the object that made the decision were different objects, and
nothing passed the number across.** The page could therefore report four minutes of staleness in
one field while the same daemon accepted a pick out of it.

**The fix.** Age is now a *query* (`list_age_ms(now)`, `list_too_stale(now)`), so it advances with
the wall clock instead of with traffic. Selection by display index and by uuid both refuse past the
limit, with the number and the limit in the sentence (§52). The limit is §21's reacquisition
window, 3000 ms, compiled-in — `recently_known_ms` beside it is not configurable either, so this
follows its neighbour rather than inventing plumbing for one value. `selection_visibility` is now
asked *as of now*, so an aged-out selection reports `STALE` instead of going on saying `VISIBLE`.
The list-level refusal is deliberately placed only on the in-list path: a uuid that already left
the list has a more specific story ("that target was last seen too long ago"), and my first draft
put the gate ahead of it and replaced a specific reason with a general one.

`ASilentProducerAgesTheWholeListAndNotJustTheSelectedTarget` covers it, and fails when the gate is
removed — checked, not assumed. **Verified on the machine**, with visiond killed for real:
`the candidate list is 6053 ms old and nothing has been seen since, so there is nothing visible to
select (the limit is 3000 ms) — start the detector again, or roam until something is seen`, while a
fresh list still gets its specific answer (`no target # 1 in the current frame`).

**Three things found on the way and not fixed, named so they stay known:**

1. **webd serves a dead daemon's last frame as if it were now.** With controld gone, `/api/health`
   correctly said `controld_connected: false` — but `/api/state` returned `at_ready=True`, and I
   read it as a completed homing. It was the last snapshot from a process that had already died.
   Health is honest, state is not, and the page has no field that says "this is old".
2. **controld exits rather than retrying a failed CAN open**: `CAN open failed: yousee: AT
   mismatch: sent 'AT+CG…' expected 'OK' got 'ERROR'` when the replacement was launched while the
   dying daemon still held the modem. A transient AT failure therefore means no daemon at all, on
   a machine whose drives' behaviour with no master is unverified. Retried by hand after a pause
   and it came straight up.
3. **`tools/fake_vision.py` publishes `TargetMeasurement`, not a TrackSet.** So it cannot drive
   the candidate list, and the "synthetic target lifecycle" I set up never injected anything:
   the publisher was crashing on `ModuleNotFoundError: No module named 'vision'` (it needs
   `-m tools.fake_vision`) and I had sent its stderr to `/dev/null`, then read the stale
   real-detector ghosts as if they were the synthetic tracks I was waiting for. That was my
   misreading, and the AUTO_TRACK lifecycle on metal remains untested — the detector-asset gap
   below is the blocker for the honest version of it.

Also measured: homing took ~40 s warm against ~84 s cold, and v3's loop again at p50 5.054 /
p95 5.059 / p99 5.065 ms with worst 223.9 ms (the §46 recipe sleep, already on record).

§110 is unchanged: `30 items, 0 accepted on hardware by a named person, 29 simulated, 1 untried.`
54 CTest binaries green.

---

## 2026-09-04, 01:50 — v3.2 arrived; the reticle question turned out to be a calibration question

Pulled `open_auto_turret_v3_2_apache_hud_ui_revision.md` (846 lines) and its reference mock. The
operator's reading — "auto tracking means the centre of the reticle should overlap the target" —
is **not in the document set**: §2.4 only says AUTO_TRACK follows the selected target when
confidently visible, neither §24 nor §25 acceptance mentions centring, and §7 goes the other way,
insisting the reticle *never* represents target, prediction or requested LOS. So it is a controller
acceptance rule, stated without a tolerance, and therefore unfalsifiable until written down.

**The operator's rules, recorded as acceptance text.** Overlap = the reticle lands on the **head**,
tolerance **1/3 of the target box height**. Doing the arithmetic: the anchor the tracker centres on
sits at 0.5·H while a head centre sits near 0.12·H from the top, so aiming at the anchor misses the
head by **0.38·H — outside the 1/3 bar**. Head aiming is therefore not a refinement, it is required
for the criterion to pass at all; it becomes a controller aim-point parameter plus a `+` cue inside
the selected box (never on the reticle, per §7). "Smooth" was clarified as *no rapid step changes in
the reference while tracking* — dynamic, not static. And the aim must **lead** sharp target motion so
the target does not leave the frame.

**What the probe found about the lead.** The §13.3 actuation horizon exists and the reference does
take a predicted LOS "at the actuation time" — but it is fixed at `control_delay 20 ms +
motor_response 20 ms` = **40 ms**, which is latency compensation, not catch-up lead. Whether 40 ms
saves a darting target could not even be *computed*, because frame-exit margin needs the horizontal
FOV and controld boots saying `tracking geometry is UNCALIBRATED` with both `calibration/*.yaml`
absent. So commissioning became the critical path, not a side job.

**`tools/probe_theodolite.py`: the encoder as a known angle.** Step the axis, settle, and measure how
far the scene moved. Built it, and it paid for itself before measuring anything:

* Its **sign self-test caught the transform conjugated on the wrong side** — right magnitudes,
  inverted sign, which would have produced a calibration that drives the turret backwards.
* My "**the pixels are anamorphic, vertical is 2.0× horizontal**" reading was **wrong**, and the
  analytics show why: for a pinhole looking down at depression θ a pan step shifts the image by
  `f·cosβ/cos(θ−β)`, which is constant along a row and varies 0.875–1.14 across rows. Whole-frame
  correlation averages elevations and reports the scene, not the lens. Measuring a **single row** for
  yaw and a **single column** for pitch removed it: 24.22 px/deg horizontal, 25.60 px/deg vertical,
  aspect 1.057 ± 0.06 = square pixels.
* My printed "**camera roll −2.67°**" was **arithmetic**: `atan2(dy, dx)` fed signed `dx` across a
  reversal reads ~180°. Real roll ≈ 0.1–0.4°, inside noise, which is why no extrinsics file was
  written — an identity matrix would dress unmeasured up as measured.

**Now on disk and loaded:** `calibration/camera_intrinsics.yaml` — `fx=1389 fy=1467 cx=960 cy=540
1920x1080`, and controld now logs them as loaded instead of UNCALIBRATED. Derived: **HFOV 69.2°,
VFOV 40.4°**, i.e. **34.6° of image either side of the reticle** — the first real number for "does
the target fall out of frame". The file states what it does *not* know: cx/cy are the geometric
centre **by convention**, because on a rotating platform the principal point and the camera-to-axis
boresight are the same constant offset and are not separable by rotation-only observation. That
convention is load-bearing, since the closed-loop tracker drives the target *to* (cx, cy): the HUD
reticle must be drawn at exactly that pixel or "reticle overlaps target" fails by construction.

**Three defects found, all with numbers:**

1. **The field of view depends on the requested stream size** — 79.3 / 81.2 / 68–79 deg per frame
   width at 640×480 / 1280×720 / 1920×1080. The operator preview was 4:3 while visiond configures
   1920×1080 16:9, so target boxes were drawn over a *different field* than the one that produced
   them. The preview default is now 1920×1080, with the reason in `web/webd/config.py`. The real fix
   is one capture shared between preview and tracker, which still does not exist.
2. **`manual_step` refusals are reported as success.** `manual_step yaw+2` returned
   `{"ok": true}` while controld refused it — `'step size must be one of 0.5, 1, or 5 degrees'` was
   only in `cmd_ack_reason`. This is the §52 response gap costing a real run: a probe that trusted
   `ok` walked the axis seven times believing it had moved. Scripts must read `cmd_ack_seq` /
   `cmd_ack_accepted`; the response should carry the control thread's verdict.
3. **`CAN open failed: yousee: AT mismatch … got 'ERROR'` reproduced a second time** on restart,
   with controld exiting rather than retrying and leaving the station with no daemon. Two
   occurrences from quick restarts; the pause-and-retry recovery works but is manual.

**Two things the operator told me, recorded as theirs, with the measurement that agrees.** The
velocity chatter I had been measuring is an artefact of *how the axis is driven in hold* (no dead
zone / no static position mode), so optimizing it is optimizing a symptom — dropped as instructed.
The independent support for that view came out of the calibration walk: at 1° steps the out-and-back
residual was 8.7 %, i.e. **~0.2° of play in yaw**, so anything commanding finer than that cycles
without moving the camera. Separately, the channel the operator's "smooth" would be judged on — the
**reference/commanded velocity — is not published at all**; telemetry carries only
`v_yaw_rad_s`/`v_pitch_rad_s` from the drive, and the probe's own settle test had to be rewritten to
use position because the reported velocity is unusable at rest.

Also: **soft pitch limits are −74.7° to −4.9° — the camera can never look level**, which the FOR
inset and any "point it at the horizon" expectation must respect.

**Not done:** no dart/lead test yet, no centring verification on metal, no HUD code.
54 CTest, 294 pytest green. §110 unchanged: 30 items, 0 accepted on hardware by a named person.

---

## 2026-09-04, 03:20 — AUTO_TRACK centred a target for the first time, after three of my claims were killed

The operator stepped away and said: skeleton first, get a working product, do not optimise before it
works; and granted permission to run tracking against a fake target, with the instruction that the
fake target must **stay in world coordinates and not move with the video**. That instruction is the
whole method — a virtual target computed from the current pose and pinned near the image centre
makes tracking succeed trivially while proving nothing.

`tools/probe_track_loop.py` publishes **TrackSets through the production encoder** (`vision.protocol`,
not a mock), with the target as a world azimuth/elevation; only its projection depends on where the
axis is. Its own self-check proves the property numerically: the same world direction, axis 5° later,
must move ~120 px in the image — if that number is small the target is riding the video and every
result is worthless.

Three defects had to die before anything could centre, and each was invisible from the outside:

**1. The camera was mounted 90° rolled and nobody knew.** The encoder theodolite's own numbers say
it: yaw +5° at −15.8° depression moved the image **−120 px in u, v≈0**; pitch +5° at −39° moved it
**−127 px in v, u≈0**. The aligned-identity default controld had been using predicts the exact
opposite (yaw → +80 px in v, pitch → −121 px in u). `R_P_C = rot_z(−90°)` predicts −117 px and
−128 px at those poses: 2.5 % and 0.8 %. So `calibration/camera_extrinsics.yaml` now exists, controld
logs `R_P_C loaded from file`, and any LOS computed before today had its axes swapped — a horizontal
target error commanded a *pitch* slew.

**2. Joint branch vs direction.** The tracking reference was pinned to *exactly*
`q_soft_min_yaw_rad` (−0.3940) while the target sat 182 px off the reticle, reason `tracking`, for
twelve seconds. Nothing was unreachable, nothing had expired: the solver had correctly solved the
direction and returned **−4.27 rad**, equivalent to the reachable **+2.013 rad**. Angles repeat every
2π; this station's yaw travel [−0.394, 5.588] rad does not, so the far branch fell below the soft
minimum and the envelope clamped it. The turret was parked at a travel limit over a choice of
representation. `geo::wrap_near()` now picks the branch nearest the pose we are at; nothing anywhere
reports this, because `los_feasible` is consumed internally and never published (§20 gap).

**3. The analytic IK seed is wrong for this station's entire workspace.** `angular_decomposition`
returns `q_yaw = wrap(az), q_pitch = −el`, exact only for the ideal aligned gimbal. But the pitch
soft limits are −74.7°..−4.9°, so `sin(q_pitch) < 0` **everywhere**, which makes axis azimuth
`q_yaw + 180°` and axis elevation `90° + q_pitch`. Asked to solve for the direction the camera was
*already looking at*, the seed answered 180° away in azimuth — and because the seeded elevation
comes out as `90° − el`, exactly **orthogonal in 3D** (measured π/2 to 1e-16). Twelve gradient steps
cannot recover from that, so AUTO_TRACK drove away from its own target and the target left the frame.
Tracking now refines from the pose the turret is actually at (`solve_from_pose`), falling back to the
analytic seed. Physically correct and numerically sound: a tracker chases what it was just looking at.

**S1 measured, tolerance fixed before the run:** world-fixed target 12° of azimuth away (185 px),
AUTO_TRACK. Error fell to **0.3 px mean** in the last 2 s (tolerance 126 px = 1/3 of the declared box
height), first inside tolerance at **t = 0.80 s**, 2 error-sign changes in the final 2 s (noise about
zero, not hunting), peak **20.3 °/s**, daemon's own `q_ref_yaw − q_yaw` down to 0.0008 rad, and its
LOS agreeing with the probe's independent projection to a mean of 0.038°.

**My own errors, kept in the record.** (a) I claimed the seed pointed "180° away" — my test made me
look: 180° in *azimuth*, but exactly 90° in 3D, because seeded elevation is `90° − el`; my Python
print had been labelling radians as degrees. (b) I twice read "no motion" as "stalled against
friction" when the cause was a clamped reference; the mechanism the config documents — "the drive's
spd_ki is too small to build torque against static friction at a low speed-ref" — is real and
on file, but it was not what was happening here. (c) My S1 "PASS" at 6° offset was worthless because
6° starts *inside* the acceptance band; the probe now refuses to run such a test. (d) Three
self-check false alarms were all my expectation formulas, not the geometry: the axis→principal-point
check is exact to 1e-13 px, and a 12° azimuth offset at 51° depression correctly moves 183 px
(foreshortened by `cos el`), not `fx·tan(12°)`. (e) My publisher initially emitted anchors *outside*
the frame — a real detector cannot — which let the controller chase a phantom to 4842 px; off-frame
is now a LOST track.

**What this does not prove.** The target is synthetic (through the production wire format and
controld's real sensing→estimator→solver→trajectory→drive path, but it does not exercise a detector;
the detector-asset gap still stands). The operator's rule is the **head** at 1/3 box height — centring
here is anchor centring, and the aim point is not implemented yet, though `aim_point_x/y/valid` are
already published. S2 (constant-rate following) and S3 (dart, and whether the aim *leads*) have not
been run. 54 CTest, 294 pytest green. §110 unchanged: 30 items, 0 accepted on hardware by a named
person.


## 2026-09-04, 03:57 — the HUD cut its first vertical slice, and the slice caught four things

The operator view is now the v3.2 HUD: `web/webd/hud.py` served at `/`, full-viewport video with an
SVG overlay, §7 reticle, §9 candidate/selected boxes, §4.1 mode block, §8 health chips, §12 status
strip, §15 tokens, §16 typography. The card dashboard was not deleted - it moved to `/dashboard` and
keeps its numbers until the DIAG drawer can carry them. `/api/*` is untouched.

**The video is `contain`, not `cover`.** Cover fills by cropping, and the frame edge is precisely
the boundary the operator has to judge against for the lead requirement. Showing a boundary that is
not the camera's would hide the acceptance margin. The letterbox bars are the deliberate cost.

**The reticle is drawn at the measured principal point**, which required saying so in telemetry:
controld now publishes `camera_intrinsics{valid,fx,fy,cx,cy,width,height}`, `effective_hfov_deg`,
`effective_vfov_deg`, `camera_fps` (webd's `Telemetry` had to declare them - `telemetry_from_json`
drops undeclared keys, which is why the first live check showed nothing). The derived field of view
came out **69.30° x 40.42°**; the encoder-theodolite walk had measured **69.2° x 40.4°** by an
independent route. `camera_fps` reads **0** with no publisher attached: unknown, published as
unknown. A unit test pins the derivation to the theodolite numbers so the two cannot silently drift.

### Four defects, all found by running things rather than reading code

1. **`/api/video` answers 409 until the stream is asked for.** After a webd restart the HUD shipped
   a dead black panel behind perfectly correct symbology. The page now starts the preview, re-asks
   when the `<img>` errors, re-checks via `/api/video/state`, and prints the refusal reason on the
   notices layer if the camera is held by something else - rather than letting a missing picture
   look like a missing target. `tools/probe_hud_composite.py` hit the same 409, which is the proof.
2. **`z-index` on an SVG group does nothing.** My first §18 layering put `style="z-index:11"` on
   `<g>` elements: markup that reads as compliance and is ignored by the renderer. Layering inside
   the overlay is document order, and the test now asserts that order and forbids the decoration.
3. **My openness check was confounded.** Reading the centre pixel of a composite where the target
   happens to be centred reported "filled dot" because the target's own anchor cross was on the
   axis. The reticle is now drawn alone on the untouched scene and checked there: real frame, centre
   pixel (213,219,217) = scene, so **open centre, no filled dot: YES**.
4. **A stale payload was about to be reported as a measurement.** The first composite run printed
   "201.6 px - OUTSIDE" while S1 had measured 0.3 px for the same geometry. Neither number was
   wrong: the probe sampled the payload *after* the TrackSet publisher stopped, and the axis kept
   moving. Measured while the target was actually being observed, n=185 over 7 s:
   **p50 0.6 px / p95 1.1 px = 0.001 / 0.003 of box height against the 0.333 bar**, state
   `tracking` throughout. The probe now refuses to present an aged payload as an acceptance number
   (it caught a 497 s-old track list doing exactly that).

### What is verified and what is not

Verified across the real boundary: the served bytes; controld's payload keys through HTTP/WS; the
page's projection executed **under node from the page's own source string** against independent
arithmetic, including the property that a non-central principal point *moves the reticle off centre*
(today cx=960,cy=540 coincides with the centre, so the coincidence had to be separated from the
rule); controld's real payload composited onto a real 1920x1080 IMX500 frame.

Not verified: **rendering in an actual browser.** There is no browser automation on this station, so
no assertion has ever caused a real DOM/SVG paint. What is missing is exactly the last centimetre:
how the symbology looks and survives on the operator's screen. That is an operator-sign item, not
something to be inferred from a green suite.

54 CTest, 301 pytest green. §110 unchanged: 30 items, 0 accepted on hardware by a named person.


## 2026-09-04, 04:21 — the axis now aims at the head, and the rule measures

The acceptance rule is the target's **head** inside one third of the box height. Until today the
controller servo-ed the §9 anchor, which is the detector's box centroid: on a standing person that
is a torso, and every published number still called it centred. So the aim point became its own
concept: `control/src/tracking/aim_point.hpp`, driven from the detector's own box through the
intrinsics that were actually loaded, selected by station config (`tracking.aim_at_head`,
`head_fraction_from_top`) rather than a code default that predates the rule.

Written against a stub first: the stub returned the anchor unchanged and **two of the four tests
failed**, which is the only reason to trust that they can detect the behaviour's absence instead of
restating it. One of the two caught a guard of mine spot-checking the wrong field (`fy > 0` while
the test zeroed `fx`) — the intrinsics type already had `valid()`, and a guard that guesses which
field matters checks the wrong one.

### Measured on the station, target 12 deg of azimuth away, n=186 over the final 7 s

| quantity | p50 | p95 | bar |
| --- | --- | --- | --- |
| **aim point -> reticle**, in box heights | **0.002** | **0.003** | 0.333 |
| aim point vs head recomputed from the published box | 0.00 px | 0.00 px | — |
| anchor -> reticle (expected to sit BELOW the reticle) | 0.282 box heights | — | — |

`head` flag held on all 186 samples, state `tracking` throughout. The anchor row is the tell that
this is head aiming and not a relabelled centroid: (0.5 - 0.22) x 378 px = 105.8 px predicted,
106.5 px measured.

The independent recomputation matters: it checks the controller's published aim point against the
head derived from the box in the payload, so the test cannot be satisfied by the controller merely
labelling whatever it chose as "head".

### Two boundary lessons, both the same lesson

`head flag held=False` on the first run, with the geometry already correct: **webd was still running
the pre-change `protocol.py`**, whose `telemetry_from_json` drops undeclared keys. The controller was
right and the operator-facing number was missing. Then the station reported `at_ready=False` for
~90 s: homing is ~60-90 s after a boot, which is a different wait from the ~25 s CAN release, and my
notes had been treating them as one number.

### No new symbology

v3.2 mentions an aiming marker once, in §7: the reticle's open centre, which IS the optical axis. So
`target_aim_*` is published and deliberately not drawn - inventing an unspecced symbol would put
something on the operator's screen that the revision does not authorise, and it is unnecessary: when
the controller drives the head onto the axis, what the operator sees is the reticle on the head,
which is the rule as stated.

### Test-suite note, kept because it is unresolved

One `ctest` run reported 54/55 and I did not capture which test - my filter kept the summary line and
discarded the name. Seven further rounds, including `-j4` under four CPU-burning loads, were clean.
**Unattributed and unreproduced**, recorded rather than dropped.

55 CTest, 301 pytest green. §110 unchanged: 30 items, 0 accepted on hardware by a named person.


## 2026-09-04, 04:29 — S2 (following a moving target) measured, after it passed twice for nothing

`probe_track_loop.py s2` now exists: a target sweeping world azimuth at a constant rate, with its
criteria written into the file before the run - containment (never leaves the frame), following
error (<= 1/3 box height at p95 after the first second), no divergence.

### Measured on the station, -8 deg/s for 7 s, 159 steady samples

| criterion | result |
| --- | --- |
| C1 containment - target never left the frame | **PASS** |
| C2 following error, aim point to reticle | p50 **9.5 px** / p95 **13.2 px** = **0.025 / 0.035** of box height, bar 0.333 - **PASS**, n=159 |
| world azimuth error while following | p50 **0.16 deg**, max 0.52 deg |
| commanded yaw rate | p50 **7.53 deg/s** against a target rate of 8.00 deg/s |
| C0 the tracker was actually tracking | **PASS** (100% of steady samples; the `ready_hold` entries are the first second, before acquisition) |

The rate row is what makes the error row believable: the axis is moving at nearly the target's rate
rather than sitting still while the aim happens to be nearby.

### Twice this scenario passed while doing nothing, and the reason is worth keeping

First run: PASS on `p50 == p95` - one sample, left over from an earlier run, while `track_state` was
`ready_hold` and the controller's target estimate sat 34 deg away. Second: the axis was parked
against its yaw soft limit from an earlier sweep, so the guard stopped the run at t=0 and the
criteria scored an empty run.

Both were the same defect I keep finding elsewhere: **a verdict computed from a run that never
exercised the behaviour.** The scenario now (a) sweeps toward the middle of the remaining travel and
shortens itself to stay inside it, (b) walks yaw back to mid-travel with `manual_step` if a previous
run left it pinned, and (c) refuses to render any verdict - printing INVALID, not PASS or FAIL -
unless there are >=30 live aim samples and >=80% of steady samples in state `tracking`. A harness
that reports PASS on its own empty run is worse than one that fails.

### Still not measured

S3 (a dart: sudden motion, containment through it, and whether the aim *leads* it) is the part of
requirement (b) that an 8 deg/s sweep cannot speak to, and it is the predictor's whole reason for
existing. Not implemented yet. 8 deg/s is also not the fastest thing this station will be asked to
follow.

55 CTest, 301 pytest green. Section 110 unchanged: 30 items, 0 accepted on hardware by a named person.


## 2026-09-04, 04:42 — S3 falsifies the lead clause, and three of my tools were wrong first

Requirement (b) says the aim must LEAD sharp target motion by projection, so the target never
leaves the frame before the axis catches up. `probe_track_loop.py s3` now tests that: settle, dart
25 deg of azimuth, hold, then four criteria fixed in the file before the run - containment, time
back inside tolerance, whether the reference actually sits ahead of the target during the dart, and
sign changes of the aim error after arrival.

### The finding

| dart | containment | back inside 1/3 box | reference lead during dart | ringing |
| --- | --- | --- | --- | --- |
| 25 deg in 0.40 s (62 deg/s) | PASS | **FAIL 2.39 s** (bar 1.50) | **FAIL -1.305 deg** (ahead in 30% of 10 samples) | PASS (1) |
| 25 deg in 1.00 s (25 deg/s) | PASS | **FAIL 2.58 s** (bar 1.50) | +0.071 deg, ahead in 50% of 26 samples - no lead | PASS (1) |

The second row is the one that matters. 25 deg/s is a rate the axis can follow, so the first row's
lag cannot be blamed on the rate limit: at a followable rate the reference leads by **0.071 deg,
which is 1.7 px** at this station's measured 24.2 px/deg. **The aim does not lead target motion.
The lead clause of requirement (b) is not satisfied**, and this is now a measurement rather than an
impression.

Supporting numbers, both runs: peak achieved yaw rate 19.0-20.1 deg/s (config allows 30 deg/s),
hold-window aim error p50 0.38-0.61 of box height - i.e. the axis spends most of the hold still
outside the acceptance band - while the *static* target from S1 settles to 0.002 box heights. The
loop is not aiming at the wrong place; it takes ~2.5 s to get there after the target moves, and the
slower dart recovered *slower* (2.58 s vs 2.39 s), which says recovery time is set by something in
the reference generation, not by the size or speed of the disturbance.

Diagnosis for the next round, in order of cheapness: whether the estimator's actuation-horizon
prediction is actually applied while the target is moving (and over what horizon), the achieved-vs-
allowed rate (19 of 30 deg/s), and the settle time constant. Not started: this round's job was to
find out, and the operator's instruction was to reach a working product before optimising.

### Three tool defects that had to die first, all found by running the station

1. **A proximity-only safety guard refused every run.** This station's homed pitch rests about
   11 deg from its lower soft limit (soft [-74.7, -4.9] deg, homed about -63.7 deg) - sitting near a
   limit is normal here; crossing one is the hazard.
2. **A velocity-based guard and settle test could never close.** The velocity feedback ripples by
   about +-0.2 rad/s on an axis that is demonstrably holding: 3 s of sampling in MANUAL/HOLD showed
   0.022 deg of yaw drift and 0.044 deg of pitch drift. An instantaneous velocity sample on a
   holding axis is noise. Both now judge POSITION over a window (settle tolerance 0.03 deg, under
   one pixel of image travel; guard = outward drift over 1 s near a limit).
3. **Scenarios derived their "world-fixed" target from a pose that was still travelling.** Every
   scenario now passes through `wait_settled()` first, because a target computed from a moving pose
   is a target that never existed.

The INVALID verdict earned its keep: the first three S3 attempts printed it instead of a PASS, on
runs with 0 live aim samples and 0% tracking.

55 CTest, 301 pytest green. 04:42 note: no ctest/pytest production code changed this round -
the changes are in `tools/`. Section 110 unchanged: 30 items, 0 accepted on hardware by a named person.


### Addendum, same day: the lead diagnosis is limited by a missing measurement, not by guessing

Adding a second lead measure to S3 - the estimator's published LOS against the true target, separate
from the executed reference - gave the decisive number: during a 25 deg/s dart,
`target_az_world_rad` **lags by 1.307 deg** (min -2.213, max -0.764, negative in every sample). At
25 deg/s that is **52 ms of lag**, while the actuation horizon the controller is configured with is
40 ms *forward*.

What that proves, and what it does not. `target_az_world_rad` is filled from the estimator's live
filtered state, so a lag is expected of it and it cannot on its own prove the intent omits the
prediction. And `q_ref` is the **output of the slew limiter**, so a lead measured there conflates
"no lead was asked for" with "lead was asked for and the reference could not slew that fast" - which
is exactly why the same dart showed reference lead near zero while the axis never exceeded 18-20
deg/s of a permitted 30.

The signal that would settle it - the LOS actually handed to the solver as the AUTO_TRACK intent,
with the horizon applied - **is not published at all**. So §78's requirement that telemetry show the
predicted LOS is unmet, §20's `prediction.predicted_anchor_norm` gap is the same hole wearing a
different name, and the lead clause cannot be closed by tuning, because there is nothing to observe.

Next round, in this order: publish the predicted LOS plus the estimator's azimuth/elevation rates and
the horizon used (small, no behaviour change), re-run the same S3 dart, and only then decide between
a too-short horizon, a rate estimate that `beta = 0.3` smoothing has flattened, and the 19-of-30
deg/s rate ceiling. Tuning lead that cannot be observed would be another number invented to end a
conversation.

## 2026-09-04, 05:0x — the new telemetry is live, and it reported something I did not expect

`prediction_horizon_ms = 0` on a ready station.

Everything else from the new field set arrived (`camera_intrinsics`, `target_aim_valid`,
`prediction_horizon_ms`, `target_az_rate_world_rad_s` all present in `/api/state`), so the fill path
runs and the readout is not the problem. The number it reports is zero.

What makes this worth a page rather than a shrug: the code default is 20 ms + 20 ms, the loader
default is 20/20, `config/turret.yaml` sets neither key, and the handover in `make_tracking_cfg` is
`control_delay_ms * 1000000` at function scope - so a 40 ms horizon is what the source says. If the
runtime value really is zero, then `predicted_los_at_actuation` predicts "now", and **the absent lead
that S3 measured is explained by construction rather than by tuning**: no horizon, no lead, no
amount of gain work would have changed it. That is also exactly the kind of defect that survives
review, because every individual piece looks right.

Two hypotheses remain, and the discriminator is small: either the config the controller is built
with is not the one `make_tracking_cfg` returns, or `prediction_horizon_ns()` is reading a different
`cfg_` than the one populated. The next round should log the horizon at startup - one line, next to
the existing "tracking aim point: head, 22% below the top of the target box" line - and let the
station say which it is. I am not changing a control-law number on the strength of an inference.

Process notes from this round, both mine, both worth keeping:

- A `git commit -q -F - <<'MSG'` followed by more shell commands in the same invocation swallowed
  those commands into the commit message: `7e5e0f6` was born with a Python script as its subject.
  Unpushed, so it is now `a8683fc` with the intended message. Nested heredocs in one command are a
  trap; message files are not.
- `ctest` reported 55/55 PASS while the build had failed, because I grepped the summary without
  grepping the build for `error:`. It happened twice today. The suite is not evidence about a binary
  that did not link.

## 2026-09-04, 06:1x — round 4: the reference exceeds its own rate limit, and two of my own numbers were wrong first

### What is now measured, on the station, with the dart's criteria fixed beforehand

| criterion | result | number |
| --- | --- | --- |
| C1 target stays in frame | **PASS** | containment held in every dart |
| C2 back inside 1/3 box height | **FAIL** | 2.0-2.2 s against a 1.50 s bar |
| C3 reference leads the dart | **FAIL** | p50 -0.29 to -0.46 deg |
| C4 no ringing | **PASS** | 1 sign change after arrival |
| C5a/C5b reference accel and jerk | **NOT JUDGED** | derived figures are publish-quantisation, not profile |
| C6 reference within its 30 deg/s limit | **FAIL** | 4 of 152 changes over the limit, worst 42.9 deg/s |

C6 is the new one and it is the interesting result: during fast target motion the **published
reference itself moves faster than the configured track-rate limit**, up to 42.9 deg/s against
30 - so 1.43x, in about 3% of observed reference changes - while the axis peaked at 19.6 deg/s.
That is consistent with the recovery failure: if the reference runs ahead of what the axis can do,
the aim error is allowed to open and then has to be closed again.

C5 must not be reported as a failure. The accel figure came out at p95 742 deg/s^2 against a
configured 60, which looks damning, and it is mostly an artefact: I derive accel by differencing a
reference that is republished in jumps, so a 1.5 deg move followed by a hold becomes a huge positive
spike and a huge negative one. Rate survives this criticism because position over a known interval
is well defined; the second derivative does not. So "jerk-limited" is **still unverified**, exactly
as it was at the start of the round - the only difference is that I now know what would verify it:
the motion log records feedback `q v a j` at 100 Hz and does **not** record the reference, so the
profile can only be seen through a 15 Hz window. Making smoothness measurable is a logging change,
not a tuning change.

### Two numbers of mine that were wrong, and what each one teaches

1. **`prediction_horizon_ms = 0` was wrong.** webd's `Telemetry` is a dataclass, and absent keys are
   filled with *declared defaults*, so a key being present in `/api/state` says nothing about
   whether controld sent it. The zero was webd's default; while tracking, the station reports 40 ms.
   I had written that the missing lead "would be explained by construction". It is not. The fields
   added for §20 now default to `None`, so absence and zero are different answers - the same honesty
   §20 demands of the operator's data, applied to my own tooling.
2. **The first step-test figure (74.8 deg/s) was inflated by my own denominator.** I divided a
   reference change by the interval between two *polls*, but a change can accumulate across two
   publish intervals. Dividing by the interval the change actually took gave 42.9 deg/s - still a
   violation, 43% smaller. A rate-limit claim lives on its denominator.

### What the prediction is actually doing

During a 25 deg/s dart the predicted LOS sits about **1.25 deg ahead of the estimator's filtered
LOS**, which is what 25 deg/s x 40 ms predicts (1.0 deg), so the lead mechanism is alive and
roughly correctly scaled. What remains is that both are *behind the target*: the filtered estimate
lags by ~1.6 deg and the prediction cancels most but not all of it, leaving -0.29 to -0.46 deg of
net lag. So the failure is not "no prediction"; it is that a 40 ms horizon does not cover the
loop's actual latency, and the reference then also overruns what the axis can follow. Measuring
that latency end to end (detector frame -> intent -> axis motion) is the next thing, and it needs
the same timestamp discipline as the blackbox, not a gain.

55 CTest, 301 pytest green. `test_documented_telemetry_reaches_the_page` failed exactly as designed
when the predicted-LOS gap closed, and the map was updated with the reason retired rather than
deleted. Section 110: still 0 items accepted on hardware by a named person.

### Correction, minutes later: I attributed a number to the wrong machine state

The commit before this one, and the section above, both say the 665 deg/s^3 "jerk" figure came from
"an axis whose barely moved". It did not. `motion t=... ax=... q=... v=... a=... j=...` is emitted
only inside the **homing / move** path (the guard is `homing_log_cycle_`, and the `msg=` field it
prints is the drive's move message - the samples I read said `msg=moving` and `msg=arrived`, at
p50 9.8 deg/s). So:

- the 665 deg/s^3 figure belongs to a homing move, where a large differenced jerk is unsurprising,
  and it says nothing at all about tracking smoothness in either direction;
- and the real gap is larger than I described: **there is no high-rate log while AUTO_TRACK is
  running.** The 100 Hz motion log is a homing/move facility. During tracking, the reference and the
  feedback are visible only through the ~15 Hz telemetry snapshot, which is precisely why the second
  derivative came out as publish quantisation.

So the concrete next change is not "add q_ref to the motion log" but **add a throttled tracking
motion log** - reference, feedback, and the derived reference rate at the same 100 Hz cadence the
homing log already uses, emitted only while a tracking reference is active. Until that exists, the
"jerk-limited" half of requirement (b) cannot be measured on this station at all, and any number I
quote for it is an artefact of the sampling window.

The C6 rate-limit finding is unaffected by this correction: it came from published positions over
their own intervals, not from that log.

## 2026-09-04, 07:0x — round 5: the reference now has a profile, and the profile turned out to be enforcing a policy that was being bypassed

### What was wrong

`step()` published the resolver's answer as the reference, position-clamped. The velocity ceiling was
handed to the drive, so the *drive* obeyed it while the *reference* did not: measured at 99 Hz (the
new tracking motion log, whose own stamps confirmed the cadence), the reference moved at up to
**105 deg/s** with a median acceleration of **106 deg/s^2** against the 60 the same config struct
declares for braking, and the axis spent 5% of a dart run more than **15 deg** behind its own
reference — 432 px on a 69 deg frame. A stepped reference against a capped drive is a step response,
and requirement (b) names that behaviour explicitly.

### What was built

`control/src/control/reference_limiter.hpp`: a per-axis rate+acceleration limiter that follows the
solved direction instead of teleporting to it. The acceleration ceiling is `cfg_.a_brake_rad_s2` —
the figure the safety envelope already brakes with, deliberately not a second tunable number that
could drift out of sync. The velocity ceiling is `lim[i]`, which already includes the §19 confidence
derating and the envelope's reduction near a boundary.

Seven tests in `control/tests/test_reference_limiter.cpp`, and they earned their keep four times:

- they caught my "land exactly" rule zeroing velocity in a single cycle — an arrival bought with an
  acceleration the axis does not have, which is the same defect standing in a nicer place;
- they caught a permanent `v^2/2a` lag (3.3 deg at 20 deg/s, ~80 px) from braking to a *standstill*
  instead of to the target's speed, which is why the limiter now estimates the target's own velocity;
- the half-step fix was written with a signed distance floored at zero, which made the braking
  distance vanish for every move in the negative direction — **the reference refused to go left**.
  Caught by the one test that happened to aim at a negative angle; a suite that only ever moved one
  way would have shipped exactly that;
- the discretisation crossing (0.074 deg, predicted 0.075 deg by `a*dt*t_decel/2`) was fixed by
  aiming the braking curve at mid-period, and the arrival bar is now documented as two control
  periods with the arithmetic shown, because a bar tighter than one sample interval is a demand for a
  different sample rate, not a stricter requirement.

56 CTest green (55 + the new suite), verified after a clean build — twice today `ctest` reported a
pass over a failed build, and one of those was this very change.

### The result, and it is not the simple one

| | before | after |
| --- | --- | --- |
| reference rate p95 / max | 40.4 / **105.2** deg/s | 10.0 / **10.0** deg/s |
| largest reference move per 10 ms | 1.066 deg | **0.102 deg** |
| following error p50 / p95 | 10.8 / **432** px | 21.1 / **82.6** px |
| C1 stays in frame | PASS | PASS |
| C2 back inside 1/3 box | 2.08 s | 2.27 s |
| C3 reference leads | -0.45 deg | **-8.76 deg** |
| S2 steady 8 deg/s sweep | az err p50 0.16 deg | az err p50 **0.14 deg** |

Smoothness is fixed and following error is 5x better at the tail. Steady tracking is unchanged
(slightly better). **And the dart's aim now lags much further behind**, which is not a defect in the
limiter: during the dart `intent_velocity_scale` was measured at **0.217**, so the ceiling in force
was about 7-10 deg/s, and the reference is now actually held to it. Before, the stepped reference was
**riding roughshod over the §19 confidence derating** — the estimator's confidence collapses during
fast motion, the policy said "go gently", and the reference went at 105 deg/s anyway.

So requirement (b)'s two halves are in tension through a policy, not through a bug: the operator's
derating rule slows exactly the response that the lead rule demands. The resolution is the
operator's, not mine — the knob is the derating schedule and `tracking.track_speed_deg_s`, and
raising either to make a synthetic dart look better would be trading a documented safety behaviour
for a number in my own test. What I will do, and did, is make the probe judge the reference against
the ceiling *in force* (30 deg/s x live scale) rather than the nominal 30, because the previous C6
bar was measuring the reference against a limit that the derating had already legitimately lowered.

Section 110: still 0 items accepted on hardware by a named person. The tension above is recorded as
a question for the operator, not resolved by me.

### Jerk: the honest number, and why my limiter does not fix it

Computed from the new 99 Hz log's own `aref=` column rather than from probe samples — a **lower
bound**, since the published reference acceleration is EWMA-smoothed over ~20 ms and smoothing
attenuates peaks:

**reference jerk p50 793, p95 1984, max 2494 deg/s^3 against the configured 300.**

That is a real failure of the "jerk-limited" clause, and the reason is structural rather than a
tuning miss: a limiter that caps velocity and *the magnitude of* acceleration still changes
acceleration in steps - whenever the wanted velocity starts demanding more, acceleration jumps from
zero to `a_max` in one control period, so jerk is bounded only by `a_max / dt`, which at 200 Hz is
12,000 deg/s^3. Capping a quantity is not the same as ramping it. The same mistake one level down,
found by measuring it.

The fix is the third-order version of the same function, and the pieces are all there: keep the
commanded acceleration in the state, ramp it by `j_brake_rad_s3 * dt` (already in `Config`, 300
deg/s^3, already the figure the safety envelope uses), clamp it to `a_max`, and integrate velocity and
position from what survives. The existing seven tests are the safety net - the rate, accel and
convergence assertions should survive untouched, and a jerk assertion joins them.

One consequence to put in front of the operator rather than discover later: honouring 300 deg/s^3
means reaching 30 deg/s takes roughly a third of a second, and reaching full acceleration a fifth.
That is what the station's own configuration asks for, and it is the same class of trade-off as the
confidence derating - the configured smoothness limits and the "lead a sharp move" rule are pulling
against each other, and which one wins is an operator decision, not something I should settle by
picking a number that makes my own test pass.

## 2026-09-04, 08:1x — round 6: the reference is jerk-limited, and where the last 15% lives

Same dart, same station, same criteria. The jerk clause went from failing outright to sitting on its
limit, with a residue that is now identified rather than vague:

| reference profile, 99 Hz | start of round | end of round | limit |
| --- | --- | --- | --- |
| jerk p50 | 793 deg/s^3 | **282** | 300 |
| jerk max | 2494 deg/s^3 | **344** | 300 |
| accel p95 | 742 (derived, noise-dominated) | **57.9** (profile state) | 60 |
| accel max | 1354 | **60.2** | 60 |
| largest reference move per 10 ms | 1.066 deg | 0.102 deg | - |
| following error p95 | 432 px | **88 px** | - |
| C1 stays in frame / C4 ringing | PASS / PASS | **PASS / PASS** | - |
| C2 back inside tolerance | 2.2 s | 2.40 s | 1.50 s |
| C3 leads the dart | -8.8 deg | -9.9 deg | > 0 |

### What it took to get there, in the order it went wrong

1. **Braking against `v^2/2a` is wrong under bounded jerk.** The profile has to spend distance
   turning its acceleration around, and with that reserve omitted it overshot every final approach
   and never landed - it was still moving at 21 deg/s after two hundred simulated seconds. Fixed by
   computing the true stopping distance under bounded jerk (ramp a to -a_max over (a+a_max)/j, then
   decelerate) and braking against what is left of the gap.
2. **The speed ceiling has to be anticipated, not enforced.** Cap it after integrating and you must
   either cut acceleration in one cycle (measured 6,900 deg/s^3) or clip the velocity (the same jolt
   in a different variable). Reserving `a^2/2j` of speed - what will still be gained while the
   acceleration is being ramped off - makes the arrival at full speed a curve.
3. **A clamp applied *after* the jerk ramp is not jerk-limited.** The post-ramp "reach the ceiling
   exactly" clamp was putting p95 at 340 and the peak at 851 while the median sat exactly at the
   promised limit. Removing it, now that the anticipation makes it redundant, cut the peak 2.5x.
4. **The telemetry was the noise, not the motion.** The worst "jerk" events came with a real velocity
   change of 0.36 deg/s - 36 deg/s^2, well inside the limit - while the derived acceleration column
   swung 11-15 deg/s^2 between adjacent lines. Once acceleration is a state of the profile, publish
   the state. Every smoothness number quoted before this change measured my differentiator.

### What the residue is

Jerk p50 282 is the construction: the ramp allows 2 x 300 x 5 ms = 3.00 deg/s^2 per log line and the
median measures 2.87. The tail does not fit that - and I checked my first explanation instead of
publishing it: I suspected the stamp-gap denominator (a third denominator error today would have been
noteworthy), and the gaps are a solid 10.10-10.15 ms, so that was wrong. What the data actually shows
is p95 equal to max (3.44 deg/s^2), i.e. a handful of isolated events rather than a systematic
excess, which is what the two remaining code paths that zero acceleration outright look like:
`reset_at()` on re-engagement, and the no-speed-authority branch. Both are legitimate restarts of a
profile rather than violations in progress - but "legitimate" is a claim about intent, so it stays
written here as a claim, and the honest summary is: **jerk is at its limit for the body of the
profile and about 15% over in a few isolated cycles whose mechanism is known.**

### And the part that is not a defect but is the headline

C2 and C3 still fail, and the reason is now measured rather than suspected: `intent_velocity_scale`
falls to ~0.22 mid-dart, so the ceiling in force is roughly 7-10 deg/s while the dart asks for 25. A
jerk-limited profile that is also obeying the operator's confidence derating *cannot* lead a target
moving two and a half times its permitted speed, and C1 (target stays in frame) still passes - which
is the hard requirement. The knob is the derating schedule and `tracking.track_speed_deg_s`, and both
are the operator's to turn, not mine: raising them buys my synthetic dart's number at the cost of a
documented safety behaviour. Same for the steady-state cost of smoothness itself - a jerk-limited
follower matching a 20 deg/s target holds back by up to vA/2j = 2 deg (measured 0.76 deg = 18 px,
15% of the 126 px acceptance band) purely to be able to stop cleanly.

56 CTest (10 of them the limiter's own), 301 pytest. Section 110: still 0 items accepted on hardware
by a named person.

## 2026-09-04, 08:5x — round 7: §25 staleness, measured with the daemon frozen rather than killed

`controld_connected` reports a **socket**. A controld that stops publishing while still holding that
socket open therefore leaves the page showing frozen numbers under a health endpoint that says
"connected" - which is worse than a disconnection, because a disconnection announces itself. This was
on the known-defect list as "webd serves dead-daemon state as current". It is closed, and closed with
the failure actually reproduced rather than described.

### The measurement (real station, real daemon, real HTTP)

controld was **SIGSTOPped**, not killed: the process freezes with its socket open and its publish
loop silent, which is the case the old code could not see. Killing it would have exercised the path
that already worked.

| controld state | controld_connected | /api/health age | /api/state age | telemetry_stale |
| --- | --- | --- | --- | --- |
| publishing | True | 4 ms | 6 ms | False |
| frozen +0.8 s | **True** | 858 ms | 859 ms | **True** |
| frozen +1.6 s | **True** | 1661 ms | 1662 ms | **True** |
| frozen +2.4 s | **True** | 2464 ms | 2465 ms | **True** |
| resumed +0.4 s | True | 47 ms | 48 ms | False |
| resumed +1.6 s | True | 59 ms | 60 ms | False |

The `connected` column is the finding: it never moved, across all three frozen rows. Age climbed in
steps matching the sampling interval, which is the property the fix rests on - **age is computed when
the snapshot is read**, not stamped when it arrived. Stamping on arrival produces a frame that reports
itself fresh forever, because the last thing to touch it declared it recent.

### What changed

- `ControldClient.telemetry_age_s()` - monotonic, measured from the arrival stamp, `None` when no
  frame has ever landed so a daemon that never answered does not read as "0 ms old".
- `/api/state` answers on a **copy** carrying `telemetry_age_ms` + `telemetry_stale`; the cached
  Telemetry is shared by every reader, and mutating it would make a snapshot's age depend on who read
  it last. `/api/health` gained the age beside the connection flag, because "connected but silent" is
  the diagnosis and the two facts belong in one sentence.
- Threshold `TELEMETRY_STALE_AFTER_S = 0.5`, in `protocol.py` beside the field it governs: controld
  publishes near 15 Hz, so that is about seven missing frames. The document names no number, so the
  number has to state its arithmetic instead of hiding inside a comparison.
- The page's rule became **one pure function** `hudStale(o)`, in the geometry block so node can
  execute it, covering five distinct ways the picture can lie: transport down, server says stale,
  hung daemon on a live socket, link to *this* page gone quiet, and target list dead while attitude
  stays live. The last one is kept separate because it is a different emergency: the reticle would
  still be honest while the boxes were not. Previously the page had three inline comparisons, which
  is not one rule and is not testable.
- A **silence watchdog** (250 ms tick): without it the page learns it is stale only when something
  arrives to tell it. A link that goes quiet with no close event, a healthy webd, and a controld still
  publishing to everyone else is precisely the case where a stationary reticle on live video looks
  like a target that has stopped moving.

### Two errors worth keeping in the record

1. My first repair of a bad insertion in `app.py` **deleted the `@app.get("/api/state")` decorator**,
   and I looked at the leftover blank line, called it cosmetic, and moved on. The route then answered
   404 while the file still parsed and imported cleanly. A silently missing route is exactly the
   failure mode this project's own notes call undebuggable, and it survived until the new test hit it.
2. Three of the new tests first failed for a reason that is a better lesson than the tests: the
   injected "last frame arrived 3 s ago" timestamp was being overwritten within milliseconds, because
   `FakeControld` was still publishing at 20 Hz. The tests were changed to stop the publisher - to
   arrange the state under test instead of pretending - and the live SIGSTOP measurement above came
   from the same thought.

Also fixed while there: a shipped comment justifying `QUIET_AFTER_MS` as "three times the ~15 Hz frame
period", which is 200 ms and not 1500 ms. Wrong arithmetic in a comment about a timeout is worse than
no comment.

### Evidence boundary, stated plainly

The rule was executed under node from the page's own source (7 cases, `node --check` clean). Its
inputs were verified live on the station. **What was not observed is the paint itself** - there is no
browser automation here, so "the operator sees the banner" remains unverified in the same way every
other visual claim on this page is, and belongs to §24's operator-signed list.

308 pytest (7 new), 56 CTest. Station: controld resumed, publishing, `telemetry_stale` False.
Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 09:4x — round 8: §5/§6 travel tapes, and a belief about the pitch scale that did not survive contact with the repo

The tapes went in as the next item in the revision's own order, and the round turned into something
more useful than decoration: the station's live telemetry contradicted an angle convention I had been
carrying in my head and never verified.

### What was built

Both tapes are drawn by `hudTravelTape` (geometry) and `hudTravelTapeSvg` (markup), both pure and both
executed under node from the page's source. The revision specifies them numerically, so the numbers are
asserted rather than admired:

| revision claim | check | live result |
| --- | --- | --- |
| yaw "upper 10-15% of the viewport" | y/vh ∈ [0.10, 0.15] | 0.125 |
| yaw "middle 55-60% of the image width" | span/vw ∈ [0.55, 0.60] | 0.575, centred to 1e-6 |
| pitch "middle 40-45% of viewport height" | span/vh ∈ [0.40, 0.45] | 0.425 |
| pitch "close to the right image edge" | x > vw − 140 | 1814 |
| "endpoints always show the software-safe travel limits" | both endpoints present, labelled with ° | −22.6° … +320.2° |
| §6.3 hierarchy (dim fine ticks, green coarse, dark box, green outline) | colour tokens in the drawn markup | verified |
| §5.3 no compass letters | no N/E/S/W anywhere in the drawn markup | verified |
| §18 layer order | document order, since this page has no SVG z-index | candidates → selected → reticle → tapes |
| §16 one monospace stack | exactly one font stack in the CSS | verified |

Tick spacing is adaptive with a stated reason: the smallest step from {5,10,15,20,30,45,60,90} that
keeps labels ≥ 52 px apart, fine = coarse/4. A fixed 20° would collide on a narrow safe range or leave
four ticks across a 342.8° one. Live: yaw picked 20°/5° (71 ticks), pitch 10°/2.5° (30 ticks).

An un-homed axis gets **no tape**: `soft_limits_valid` false, or min ≥ max, returns null and the page
says `YAW / PITCH TAPE: TRAVEL UNRANGED (home the turret)`. Drawing endpoints for an axis that was
never homed would name a limit that does not exist.

### Live, from the station — not a fixture

```
YAW   tape: -22.6° … +320.2° | YAW +174.2° | JOINT TRAVEL, NOT HEADING   (caret x=1042, tape 408-1512)
PITCH tape:  -4.9° …  -74.7° | PITCH  -44.0° | JOINT, NOT ELEVATION      (caret y=567, tape 311-770)
caret inside its tape: true true   value inside limits: true true   no cardinal letters: true
```

### The belief that fell over

I started converting the tapes to front-relative azimuth and elevation, from `az = q_yaw + 180°`,
`el = 90° + q_pitch` — figures I remembered from the geometry commissioning. Two things killed it:

1. **Neither formula appears anywhere in the repo.** I grepped for them before using them and found
   nothing. A scale on an operator's screen cannot rest on my recollection.
2. **The theodolite probe says the offset cannot be measured this way at all** (its own docstring):
   *"Principal point and camera-to-axis boresight are NOT separable at the small angular spans
   available here — both enter as a constant pixel offset."*

So objective item (c) is **partly** commissioned and I am recording it that way rather than as
"done": effective FOV and principal point are measured (69.30° × 40.42°, 24.22 px/deg, matching the
theodolite's 69.2° × 40.4°); **the camera-to-axis boresight is not**, and the method used could not
have separated it. Centring tolerance and frame-exit margin — the reason item (c) exists — rest on the
FOV and principal point, which are measured; what is *not* available is a world elevation for the
camera's boresight, and therefore no honest elevation scale for the pitch tape.

What saved the design is the revision itself: §5.3 asks for **logical joint travel, not compass
heading**. Raw joint degrees were always the correct scale. The live oddity — yaw spanning 342.8°,
pitch entirely negative — is the machine being truthful: `config/turret.yaml` states *"YAW IS A ~360
DEG CONTINUOUS-ROTATION AXIS (user-confirmed)"*, and its ready pose sits at 176° joint because that is
where the gravity balance is, not at the travel mid. So each tape now states its own scale —
`JOINT TRAVEL, NOT HEADING` and `JOINT, NOT ELEVATION` — instead of letting −44.0 read as elevation.

### My own defects this round, all found before the page ran

- **A second `const HUD_R2D`** in the same script. In a page script that is a SyntaxError at load, and
  the whole HUD would have drawn *nothing at all* while the server happily served 200. Caught by
  `node --check` on the concatenated page source, which is why that check is now routine.
- **The vertical clamp reused the horizontal variable**, clamping the pitch caret between `x` and `x`
  and collapsing every pitch marker onto the tape's own column: expected 593.7, produced 1842.0. Hand
  arithmetic found it; the arithmetic is now in the test.
- **A CSS anchor matched inside `text.lbl {`**, splitting three rules across two conventions and
  duplicating the font stack three ways — the drift pattern I have been writing up all week.
- **A test fixture 4 px off** (`x=412` copied from an ad-hoc shell run, not the 408 the page computes),
  which made a centring assertion fail for the wrong reason. A fixture that is not what the code passes
  cannot test the code.
- **A vacuous assertion I wrote** — `assertLess(x if c else 10**9, 10**9)`, true in every universe —
  deleted and replaced with one that reads the caret's actual fill. And a stale export list naming a
  function I had deliberately never written, since both tapes share one implementation.

### Evidence boundary

Geometry, colour tokens and draw order are measured. **Resemblance to the approved reference is §24 and
remains unsigned**: no browser automation exists here, so nobody has seen these tapes painted. The
caret positions above are the page's own arithmetic on the station's own encoders, which is the
strongest statement available short of looking.

56 CTest, 321 pytest (13 new on the tapes). Station: controld publishing, `telemetry_stale` False, webd
serving the tape build, page 200. Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 10:4x — round 9: §20 prediction block + §10 cue, seen valid on the real daemon

### What went in

`prediction.{valid, predicted_los_yaw_deg, predicted_los_pitch_deg, predicted_anchor_norm,
anchor_in_frame, horizon_ms}` — the §20 shape — emitted by controld, and the §10 cue drawn from it:
amber dashed square, small amber `+`, small `PRED` label, subtle short connector only when the pair is
already close. `<g id="g-prediction">` sits between the selected target and the reticle, which is §18's
z=12 on a page whose only z-order is document order.

**No second prediction was created.** `aim_point_x/y` already projected `predicted_los_at_actuation`
through the intrinsics, behind four documented guards (estimate exists; intrinsics agree with the
detector's frame size; ray in front of the camera; point inside the frame). The new fields are filled
*inside that same block*, so the flat and nested spellings cannot disagree, and the guards exist once.
Camera-frame LOS came as `CameraModel::ray_to_los_angles`, written beside `ray_to_pixel` because it is
its inverse: tan(yaw) = (u−cx)/fx and tan(−pitch) = (v−cy)/fy, since v grows downward while elevation
grows upward. Five gtests on that helper, including a round trip across the frame and a corner check
against the 69.30 × 40.42° this station commissioned.

`FakeControld` now speaks the block too. A field the fake does not emit is a field the page tests
cannot notice going missing — and `telemetry_from_json` silently drops keys it has not seen, which has
twice produced a confident "controld does not publish X" about a field webd itself discarded.

### Live, on the station, during AUTO_TRACK on a synthetic dart

38 samples at 10 Hz with `prediction.valid` true, from the real daemon:

```
horizon_ms seen            : [40]          (the 20 ms control delay + 20 ms motor response)
lead in x                  : median 2.4 px   p95 27.1 px   max 28.5 px   (0.101 deg median)
anchor_in_frame            : 38 of 38 true
predicted LOS yaw          : -0.48 .. +13.77 deg
```

At idle the block is honestly `valid:false, anchor_in_frame:false, horizon_ms:0` — not a zero at the
centre of the frame posing as a prediction.

The two numbers worth reading together: the **cue** leads the measured anchor by 2.4–28.5 px, while the
**axis** still lags the target (C3 lead −8.9°). Both are true, and that is what §10 is for — the
operator can now see the gap between what the controller intends and what the picture confirms. The lag
is round 6's finding (confidence derating pulls the ceiling to ~7–10°/s against a dart asking 25°/s),
not a cue defect.

### Metric honesty, not metric tuning

C5a now reads **PASS (p95 60.0, max 60.0)** where it had read FAIL on identical numbers. Nothing about
the motion changed: the probe compared differenced telemetry against a bare `60.0`, and a profile
sitting exactly on its ceiling failed on floating-point dust. The tolerance is now stated in the line —
`bar 60 + 1.5 from 15 Hz differencing`, 1.5°/s² being one `j·dt` control cycle, the resolution that
derivation actually has.

C5b still reads FAIL (p95 543 vs 300+60) and I am leaving it FAIL. The same move measured from the 99 Hz
profile-state log gave p95 340. Two rates disagree because differencing an accel sequence at 15 Hz is
too coarse to resolve a 300°/s³ ramp; the 99 Hz figure is the defensible one and the probe's is a lower
bound on nothing at all. Recording the disagreement beats picking the flattering one.

C4 also flipped to FAIL (3 sign changes vs 2 last round) — the count-only metric already recorded as
too crude; a flip of one crossing is within what that metric can mean without amplitudes.

### Defects I introduced and caught this round

- **I invented `hudMapNorm`/`hudMapU`/`hudMapV`** and called them from the render path. The real mapper
  is `hudProject(u, v, lay)`. In a page script an undefined call inside `render()` kills the whole HUD
  while the server keeps returning 200 — same class as this round's duplicate `const`, and it is why
  there is now a test that every mapper the cue calls is actually defined.
- **A Python triple-quote ate the closing `"`** of a C++ string literal in `web_server.hpp`: build
  error, caught at compile.
- **Anchors I guessed wrong** — `prediction_horizon_ms` is `int64_t`, not `double`; `CameraIntrinsics`
  has no `valid_flag` (validity is derived by `valid()`). The compiler and the assert caught both.
- **My corner test contradicted my own sign test** three lines above it (I asserted positive pitch at
  v=1079, the bottom of the frame). A test that disagrees with its neighbours is usually the wrong one.
- **A sampler crash** from double-indexing an already-unpacked pair, and **another vacuous assertion**
  of mine (`assertLess(index, len(text))`) which I replaced with the real stale/invalid gate check —
  the second time this round I have caught myself writing a test that cannot fail.
- **My non-touching test asserted my own implementation** — it demanded the cue end up to the *right*
  of the box and failed when the cue correctly moved straight up. The rule is "not touching", in any
  direction; the assertion now checks rectangle intersection.

57 CTest (5 new), 330 pytest (9 new). Station: homed, controld publishing the prediction block, webd
serving the cue. Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 11:0x — round 10: §11 FOR inset, and a validator that immediately found two live defects

### What went in

`field_of_regard.{valid, kind, coordinate_frame, safe_envelope_points[]}` from controld, and §11's
inset drawn from it: low-opacity panel, `FIELD OF REGARD`, green envelope polygon, white FOV rectangle,
LOS centre marker, green target marker, amber predicted marker, three-row legend, `SAFE ENVELOPE`.

**§11.3 is what makes this publishable at all.** FOR coordinates are yaw/pitch degrees, not image
coordinates. Round 8 established that the camera-to-axis boresight is not separable from the principal
point at the spans the theodolite probe reaches, so an envelope drawn over the picture would inherit an
offset nobody has measured. In joint degrees every number comes from the encoders — and the envelope is
taken from the *same* `limits_[].q_soft_*_rad` the loop hands to the collision envelope elsewhere in
the file, so the picture of what is permitted is the permission, not a copy of it that can drift.

One scale for both axes, deliberately. Fitting yaw and pitch independently fills the box more
attractively and makes the FOV rectangle misreport the camera's field; the envelope is letterboxed
instead. The test asserts the rectangle's aspect ratio rather than how nicely it fills the box.

Live, driven through the page's own function with the station's real payload:

```
inset box             : 499 x 232 px  = 26.0% x 21.5% of viewport   (§11.2: 25-27% x 20-23%)
scale                 : 1.348 px per degree, one scale
FOV rect              : 93.4 x 54.5 px = 69.30 x 40.42 deg, aspect 1.715 (camera 1.715)
polygon               : 4 points, equal to the published soft travel to 1e-3 deg
FOV inside envelope   : true          pred marker: on the LOS (idle: reference == current)
```

### The validator, and what it found within one build

`json_balanced` already existed in `test_web_server.cpp`, but the telemetry line itself — the input the
entire UI is built from — was tested **only by substring search**, which is the weakness that helper's
own comment names while continuing to do it. Substrings cannot see a double comma, and a double comma is
perfectly balanced.

So: a structural checker over the emitted line — quote-aware, rejecting empty elements (`{,`, `,,`,
`,}`), trailing commas, and the same key twice in the same object (paths qualified, because
`prediction.valid` and `field_of_regard.valid` are different fields that must both be called `valid`).
It found two real duplicates immediately:

- **`target_aim_is_head` emitted twice** — the field carrying objective (a)'s head-aim acceptance
- **`target_el_rate_world_rad_s` emitted twice**

Both emit identical expressions, so nothing is wrong on the air today; a reader keeps the last. The
danger is the future edit: change one of a silent pair and the field starts reporting something other
than what the control loop computed, with no compiler, no test, and no parser complaining. Both de-
duplicated; each looked like `X     << X`, a collapsed line break from some earlier patch.

**I had already seen one of them.** `target_el_rate_world_rad_s` appeared twice in a `sed` listing
earlier this session and I read it as my `cut -c` truncating the line. Then `grep -c` reported "1",
which hid it again — `grep -c` counts *lines*, and both copies were on one line. Counting lines when
asking how many times something occurs is the same mistake as counting files when asking how many tests
exist.

### Defects in my own new code, caught before they shipped

- **I wrote a comment claiming a declaration that I did not write.** In `protocol.py` the patch added
  "…every container the contract names gets declared" and no field. The comment described the fix; the
  fix wasn't there. Symptom: `field_of_regard: null` from a daemon that was verifiably emitting it,
  through the exact silent-drop trap this project has now been bitten by three times.
- **The trailing comma I introduced** (`<< "]},"` while the next field also led with `,`) produced
  `},,"target_az_rate…"`. Clean build, green substring tests. Only a manual look at the live payload
  showed the UI's entire input was not JSON — which is why the validator now exists rather than because
  it seemed like a nice idea.
- **`isFinite(null)` is `true`** in JavaScript, because `Number(null)` is 0. My vertex guard accepted a
  malformed envelope point and would have plotted it at yaw 0 as a legitimate corner — a fake vertex on
  a map whose only job is saying where the turret may point. Guard now checks `typeof … === "number"`.
- **The checker's own tokenizer double-emitted strings.** I pushed a string token without clearing the
  accumulator, so the next flush leaked the word again; the stray token sat where a `:` was expected, so
  no key was ever recorded and **the duplicate detector could never find anything** — and the main
  telemetry test passed happily on that dead code. Only the self-test that asserts the checker *fails*
  on known-bad input exposed it. A validator that has never been seen failing is a decoration.
- Test bugs: `latest_telemetry()` returns a `Telemetry` object, not a dict, so three tests crashed in
  `setUp`; and I guessed `s.ts_ns` / `s.track_state = "ready_hold"` / `telemetry::format_telemetry`
  where the real ones are `sample_snapshot()`, an enum, and `ota::web::` — caught by the compiler in
  one round trip, which is cheaper than the alternative.
- Two mangled edits from my own slice arithmetic (a C++ literal's closing quote eaten by a Python
  triple-quote; an emission block half-replaced), both caught at compile or by reading the lines back.

### Counts

57 CTest entries — one entry per binary, so the two new cases inside `test_web_server.cpp` show there
as 11 cases where it had 9 (run directly, all pass). 346 pytest (16 new). Station: homed, controld
running the de-duplicated build, webd serving the inset. Section 110: still 0 items accepted on
hardware by a named person.

## 2026-09-04, 11:4x — round 11: §20 becomes checkable, and the check immediately cost a permanent red chip

### Operator input this round

Told directly: **there is no IMU installed**, skip it or stub it as nothing. That upgraded the IMU
claim from inference to fact — I had concluded "no inertial sensor" from the code (the only "imu" in
the tree is inside the word "simulation"), and the operator confirmed it as a hardware statement. The
stub therefore stays, because §20 names `imu.present / imu.gravity_valid / imu.world_elevation_deg` and
an absent answer delivered as data beats a hole every reader fills differently. `world_elevation_deg`
goes out as JSON **null, not 0** — 0.0 is the sentence "this turret is level", and nothing on this
station is entitled to say it.

### What went in

- `camera.{fps, effective_hfov_deg, effective_vfov_deg, measurement_age_ms}` — fps is the
  inter-TrackSet cadence, the only camera rate this process can observe; §12's example strip quotes
  that figure ("FPS 29"), and the browser's preview rate is a separately limited relay number.
  `measurement_age_ms` is null: the daemon loads intrinsics from a file and does not carry that file's
  timestamp into telemetry. Not 0, which would claim freshly commissioned geometry. Plumbing the file's
  own mtime through is the known, bounded fix, and the gap is declared in the ledger rather than held
  in my head.
- `imu.{present, gravity_valid, world_elevation_valid, world_elevation_deg, basis}` as described.
- **`web/webd/tests/test_section_20_ledger.py`** — the thing that mattered. Nothing in the suite had
  ever read §20 of the revision. The v3 architecture document's §50 had a ledger; §20, the list the
  revision actually demands, had none. So "the data contract is nearly complete" was a feeling. The
  names are now parsed from the document itself: every name must map to a path that resolves in the
  payload the page reads, or be a declared absence with a reason. A mapped name that stops resolving is
  a regression; a declared gap that quietly fills is drift. Both fail.

§20 result: **34 names, 32 mapped, 2 honest nulls** (`camera.measurement_age_ms`,
`imu.world_elevation_deg`).

### The defect the ledger found in the act of being written

Mapping `system.connected` required finding where the fact lives: `controld_connected` is served by
**`/api/health`**. The §8 health chip reads `t.controld_connected` — from the **state** payload, which
never carried it. So `!!undefined` was false and **the CONNECTED chip has been permanently red on every
working station since it was written** — on a turret that is homed, tracking, and publishing. An
indicator that is always wrong on healthy hardware teaches an operator to ignore the display, which is
worse than no indicator.

No test could see this because the suite checked the staleness fields and the health endpoint
separately; nobody had asserted that the field the chip reads is a field the endpoint sends. Both
endpoints now report the same `client.connected()` call — one fact behind two spellings.

### Defects in my own work, caught in place

- **My ledger's own parser silently read less than the document says.** Its regex allowed two dotted
  segments, so all six `axes.yaw.actual_deg` style names were skipped without a word — and the file
  would have gone green having ignored six requirements. It surfaced only because I also wrote the
  two-sided check (unmapped doc names *and* stale ledger entries); the "stale" list was the one that
  made nonsense, which is how I found out the parser was wrong rather than my transcription. A
  requirement-vacuating bug in the test written to prevent requirement-vacuating bugs.
- I transcribed three names wrong or missed them (`system.mode_phase`, `system.operating_mode`,
  `system.safety_action`, and `predicted_anchor_norm[]` vs `predicted_anchor_norm`) — the reason the
  ledger parses the document instead of trusting me to copy it.
- I guessed `controld_connected` was in `/api/state` from memory of the §25 work. It was in
  `/api/health`. That guess is what a mapping table with a resolving test exists to punish.
- Two string-surgery slips while emitting the new blocks: a stale anchor from last round's comma fix
  (aborted before writing, as the asserts keep doing), and a mangled line that dropped a `<<` and
  duplicated a quote — caught by the compiler, and by reading the lines back rather than trusting the
  patch's exit code.
- A `grep -c` count that reported one occurrence because it counts lines, not matches, briefly hiding a
  duplicate field again (see round 10).

### One flake, recorded rather than smoothed over

One CTest run failed; three subsequent runs were green, and the failure name was not recoverable from
the captured output. The plausible mechanism is named rather than guessed away: the socket reader in
`test_web_server.cpp` waits on a 1000 ms `poll` deadline against a publisher sending every 20 ms, so
the test asserts arrival while measuring scheduling. On a loaded station that is a coin flip that adds
nothing. The default is now 5000 ms with the reason in the source; callers that pass their own timeout
keep it, and latency assertions elsewhere keep their tolerances. Suite is 57/57 twice after.

### Also corrected

A comment beside the §12 strip claimed "FPS is not in the snapshot yet, so it reads `--`". FPS has been
in the snapshot for many rounds and the cell reads it. Comments like that are how the next reader
decides what is true, so it now says what `camera_fps` is and why §12 quotes it.

355 pytest (9 new), 57 CTest entries. Station: homed and ready, both blocks live, CONNECTED chip now
reporting a real value. Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 12:2x — round 12: §13 dock, §14 drawers, and a command path that lied about success

### What went in

The five-button dock (`TARGETS MODE MANUAL DIAG MENU`, §13's order), drawers that open one at a time,
and the first command plumbing this HUD has ever had — the page previously only ever read.

Command rows are **built as data by a pure function**, so "which commands will this drawer send" is a
question node answers without a browser or a socket. Every name and argument spelling was read out of
the daemon's own handlers first: `select_target` takes the **display index as a number** (controld's
refusal text says "the label on the screen"), `manual_jog_start` takes `yaw+/yaw-/pitch+/pitch-`,
`manual_step` takes `yaw+1`, `set_mode` takes `MANUAL/AUTO_TRACK/AUTO_ROAM` and refuses rather than
falling back, and STOP MOTION is `hold`.

Behavioural choices worth stating:
- **Gated rows are shown greyed with the reason on the row**, not hidden. A control that silently
  vanishes is how an operator learns to guess at an interface.
- **STOP MOTION is never gated.** `hold` is accepted in every mode, and a stop that works in one mode
  is not a stop.
- **The active mode and the selected target carry no command at all** — not a command the renderer
  happens to disable. My own test caught the two drawers disagreeing about this; the data is what the
  tests read, so it has to say what is true.
- **Park and supervisory shutdown ask twice**, with the row changing to CONFIRM … PRESS AGAIN so the
  waiting state is on screen rather than in someone's memory. §14 reserves red for stop and fault, so
  colour is not the only signal here.
- One `drawerOpen` variable, not five booleans: §13.2's "only one drawer at a time" becomes structural.
- `#drawer` is `position:absolute` over the video, because §13.2 requires the drawer not to resize the
  picture — the moment a drawer reflows the viewport is the moment the operator stops seeing the scene.

18 new tests execute the builders under node and assert the styling/markup rules.

### The defect the live probe caught, which no unit test could

Posting what the drawer would send — `select_target 9999`, a target that does not exist — to the live
station:

```
/api/command answered   : {"command":"select_target","ok":true,"error":""}
controld's log          : select_target 9999: REFUSED (no vision data has reached controld yet)
published ack, later    : seq 3, cmd select_target, accepted 0, reason "no vision data has reached controld yet"
```

**The socket response says ok for a command the daemon refused.** It reports that the command reached
the handler, not what the handler decided. Had the drawer rendered it, the operator would have seen
`ACCEPTED` for a target that does not exist. This is the defect my notes recorded as "manual_step
refusal returning ok:true" — and it is not specific to `manual_step`; it is the response path.

The page now reports the socket reply as `SENT` and takes `ACCEPTED`/`REFUSED` **only** from the
daemon's published `cmd_ack_accepted`, matched on `cmd_ack_seq` so a stale ack cannot answer a new
command, with a four-second `NO ACK FROM CONTROLD` path so a command that produces nothing does not sit
there looking accepted. Verified live: the drawer's own data source produces
`REFUSED: no vision data has reached controld yet`.

**Still open, deliberately:** the response itself should carry the ack. The page no longer trusts it,
but the lie is still on the wire for any other consumer. That is next round's work, not this one's.

### My own defects this round

- **I walked into the trap written in my own notes.** One command line contained both
  `pkill -f "web[.]webd[.]app"` and the plain string `web.webd.app` in the launch that followed it, so
  the kill matched my own shell and SIGTERM'd it mid-call. The note says *kill in one call, launch in
  another*. The station was fine; the call was not.
- **`python3 -c "…"` mangled again** (a `NameError` from shell-quoted code), for the second time this
  session after round 10's X-server incident. Everything with quoting now goes through files.
- **A test of mine asserted a word that also appears in prose** — `assertNotIn("ACCEPTED", body)`
  failed against my own explanatory comment. It now asserts on the string literal `"  ACCEPTED"`,
  because a test that cannot tell code from commentary passes and fails for the wrong reasons.
- Stale anchors again (my §10 anchor and a markup anchor) — the asserts aborted before writing, which is
  the only reason the file was never half-patched.
- **One sample was not enough.** Immediately after the command, `cmd_ack_seq` still read 2 — the ack is
  set on the loop thread and reaches telemetry on a later snapshot. Concluding "the ack never updates"
  from that single read would have been wrong, and is exactly the shape of error this project keeps
  meeting. It is also why the resolver is driven from the render path instead of from the send.

### Also verified live

Page 200 with `id="dock"`, `id="drawer"`, `hudDrawerActions`, `STOP MOTION` all present; the
command endpoint accepts the page's body shape; `set_mode MANUAL` (already MANUAL) acks accepted with
reason "already in MANUAL"; mode and ready state unchanged by the probes.

57 CTest entries, 373 pytest (18 new). Station: homed, ready, MANUAL, holding; webd serving the dock.
Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 13:0x — round 13: correcting my own claim about the command response, and a torn read that was really there

### My round-12 claim was wrong, and stays in the record

Round 12 said controld's command response "lies" — `select_target 9999` answering `ok:true` while the log
said REFUSED. Reading `submit_command` shows that was **my** error. The response is written on the web
thread after the validation gate and answers *"did the gate take this?"*; the control thread answers
*"what did the station do?"* through `cmd_ack_accepted`/`cmd_ack_reason`. Whether a target exists is
decided against the TrackSet actually in hand rather than a stale web-thread snapshot — the gate's own
comment says so. **Nothing was lying. My page read a receipt as a result.**

What *was* genuinely wrong is that nothing on the wire distinguished the two, so the next reader repeats
my mistake. The response now says which question it answered:

| case | live response | what the drawer shows |
|---|---|---|
| `select_target person` | `ok:false, verdict:"rejected", error:"target label must be a number"` | `REFUSED: …` at once — the gate's decision is final |
| `hold` | `ok:true, verdict:"submitted"` | `SUBMITTED`, not dressed as success |
| `select_target 9999` | `ok:true, verdict:"submitted"` → later `accepted:0, reason:"no vision data has reached controld yet"` | `REFUSED: no vision data has reached controld yet` |

webd *declares* the field (undeclared, the parser drops it — that trap again), the fake daemon emits it so
a test notices it going missing, and the client's own timeout answer leaves the verdict **null**:
asserting a refusal for a command that may have executed is the same error pointed the other way.

### A torn read on the telemetry path, found by refusing to shrug at a flake

CTest failed 1 of 4 runs. Last round I lost such a name; this round I captured it:
**`SeqLock.AdversarialContentionNoTornReads` — 3 failures in 80 direct runs (~3.8%)**, and it was not a
timeout: a read returned `true` with an inconsistent payload. This is the lock-free path the CAN RX
thread writes and the 200 Hz control loop reads, so a mismatched yaw/pitch pair could reach the
controller and the HUD together.

Two ordering faults, both specific to weakly-ordered A76:

1. **Writer:** the odd marker was stored with `release`, which orders everything *before* it and
   constrains *nothing after* it. The payload stores could therefore become visible before the marker, so
   a reader that sampled the previous even value on both sides of its copy certified a half-written
   payload. Needs a full fence between marker and payload.
2. **Reader:** the fence between copying the payload and re-sampling the sequence was `acquire`, which
   guards the opposite direction and let the copy sink past the sample meant to validate it. Needs
   `release`.

Measured: **0 tears in 240 runs** after, where the baseline rate would give 0 by luck ~1 time in 10 000
(`(1−3/80)^240 = 1.0e-4`). That is evidence plus a memory-model argument, not a formal proof, and is
recorded as such.

### Second process trap this session, and the verification it invalidated

The webd I "verified" against was **30 minutes old**. Round 12's SIGTERM'd shell had left it holding
8080; my new launch died on `Errno 98 address already in use`, and the served page had no verdict code
because the *old* module was in memory. Round 12 printed `webd still alive` and I accepted that instead
of asking *how* alive — two pids looked like parent and child, which they were, of the process I should
have killed. All the live verdict numbers above were re-taken after killing the stale process; the first
version of them is worth nothing. **A restart that fails to bind looks exactly like a restart that
succeeded, if you only check that something is answering.**

### Counts and state

57 CTest entries (`test_web_server` now 12 cases), 377 pytest. Station homed, ready, telemetry 47 ms,
no stale flag. Section 110: still 0 items accepted on hardware by a named person.

## 2026-09-04, 13:3x — round 14: §21 state wording, §22 safety ladder

**§21** — the mode block stops echoing controld's phase string. A pure builder maps mode+phase onto the
revision's wording: `AUTO TRACK / TRACKING`, `AUTO TRACK / COASTING`, **`TARGET LOST / HOLDING`** (the loss
goes on the strong line — burying it under a mode name is how an HUD hides the interesting part),
`AUTO ROAM / SWEEP`, `MANUAL / HOLD | JOG`. `JOG` is read from the published `manual_lease_active`, not
inferred from a non-zero rate, which would also light up during homing, a roam, or settling after a hold.

Two honest gaps: **no sweep direction** (`SWEEP LEFT|RIGHT` in §21.4 — controld publishes none, so no arrow
is drawn; same class as the IMU absence), and **unnamed states keep the daemon's own word, dimmed**, rather
than getting invented friendly wording.

**§22** — one chip could not carry a ladder, so there is one now:

| safety state | presentation |
|---|---|
| `ALLOW` | green chip, compact — `SAFETY ALLOW` |
| `DERATE` | amber, **naming the edge** (ref vs soft limits; breached sorts ahead of still-ahead) |
| `BRAKE` | amber, larger — `BRAKING` |
| `FAULT_STOP` / non-empty `fault` | **red, largest, with the short reason**, `aria-live="assertive"` |

The enum has six states and §22 words four: `HOLD` and `DISABLE` keep the daemon's word and are never
green, and an unrecognised action defaults to **amber caution, not green** — defaulting an unknown safety
state to safe is the one default that can kill.

**Not done, not claimed:** §22's *"including the relevant travel-tape edge"* is met by **naming** the edge,
not by highlighting it on the tape. The tape renderer has no marked-tick concept, and bolting one on from
the safety code without reading that renderer through is the blind-patch class that has cost time twice
here. Deferred with the reason.

**Own defects caught before running:** my first edge sort penalised *breached* limits — the opposite of its
own comment (ascending margin is both simpler and right); a walrus-operator class hack and a `replace()`
that made an assertion a no-op in the test file.

**Live**, as the page's own code computes it from the station's snapshot: `MANUAL / HOLD` in revision
wording, `SAFETY ALLOW` green/normal — from mode MANUAL, phase HOLD, `safety_action ALLOW`, lease 0, fault
empty. Round-13's stale-process lesson applied on purpose: separate kill/launch calls, then checked for a
bind failure and the serving process's age (both pids 28 s, no `Errno 98`).

57 CTest, 408 pytest (+31). Station homed, ready, holding. §110: still 0 items accepted on hardware by a
named person.

## 2026-09-04, 14:0x — round 15: §15 / §16 / §18 audit, and an undeclared token that silently deleted three rules

**The bug.** Three rules — dock buttons, drawer body, safety indication — read `var(--hud-mono)` inside a
`font:` **SHORTHAND**. **Nothing declared that token.** An undefined custom property in a shorthand makes
the *whole declaration* invalid at computed-value time, so those rules did not fall back to the inherited
font — the browser discarded **size and weight too**. The dock would have rendered in the UA's serif button
font at UA size: §16's typography simply absent in the newest parts, with every test green because each one
asserted a substring it already knew was there. The suite now asserts **every `var()` read is a token
declared**, and asserts the check fails on the stylesheet that actually shipped for three rounds.

**Caught by an older test:** my first fix left the stack literal in `html, body` *and* added the token; the
tapes-round test — *"the font stack is set once; three copies is how they drift"* — failed immediately and
was right. The literal is gone.

**Conformance fixes:** mode phase 13→11px and selected 12→11px so the tape value (12px) is the strongest
numeric text per §16 — **only HTML divs shrunk**; the boxed tape value is text-metric sized with a fixed
baseline constant, so enlarging SVG text blind would change layout nobody can verify. Safety indication
**45 → 50**: §18 has no row for it, and at 20 a drawer would cover a FAULT banner, making §22's "interrupt
normal operation" false the moment DIAG opens; 60 stays reserved-and-unimplemented, §17's z=1 filter absent
by choice with a test recording the absence. A **refused command is amber, not red** — §15 restricts red to
fault/stop, and spending red on inconvenience trains the operator to ignore FAULT.

**Recorded as not done:** candidate labels share `tlbl` with scale labels, so §16's small-vs-medium-small
step is unexpressed — deferred rather than fixed blind, same reason as the SVG sizing above.

15 new tests. **423 pytest, 57 CTest.** Served page: no undeclared tokens, one stack, layers
0/10/20/30/40/50, refusal amber; fresh webd (no `Errno 98`, pids 9 s old). **Still no real-browser paint in
this project's evidence** — §24 stays operator-signed. §110: 0 items accepted on hardware by a named person.

## 2026-09-04, 14:3x — round 16: §22's tape edge, closed from the running station

Round 14's deferral is done. A DERATE indication now lights **the actual tape end it names** — the tape's
endpoints *are* the soft limits, the same published numbers the limiter acts on and the same numbers the
tape is drawn from, so the highlight can't disagree with the scale beneath it (the reason I deferred it: an
amber highlight at the wrong end is worse than none — the operator believes it). Marked tick: amber, longer
than an endpoint tick, its degree label amber too. The mark is requested **only while `safety_action` is
DERATE**, only on the axis named — `BRAKE`/`FAULT` are not limit problems, so lighting an edge for them
would name a cause that isn't there — and yaw vs pitch comes from the same `hudSafetyEdge` the drawer text
reads, so chip and tape cannot drift.

Tests insist on: an **unmarked tape contains no amber anywhere** (§15 — a healthy tape carrying amber
teaches the operator that amber is decoration); the marked end out-lengths its neighbours **while they stay
green** (everything amber at once isn't a highlight); the mark is asked for only under DERATE; and **both**
tapes can carry it (a rule that only fires on yaw is half a rule).

**Verified on the station, not just in tests:** live telemetry (`ALLOW`) → the yaw tape the page's own code
generates contains **no amber at all**; the same code over the same live numbers with `safety_action` set to
`DERATE` → edge `PITCH MAX`, amber present, green still present. Deciding code on real numbers — still **no
real-browser paint**, so §24 stays operator-signed.

Still owed: candidate labels share `tlbl` with scale labels, so §16's small-vs-medium-small step is
unexpressed — deferred a third time, same reason (SVG text is sized against measured boxes with a fixed
baseline constant; changing it blind can't be verified here).

Own defects, all in my test code, all caught before becoming evidence: nested double quotes through a
heredoc (unparseable file), node helpers redeclaring the harness binding (surfaced only as an exit code),
and a label assertion that guessed an entity where the formatter emits `+100°` — nearly "fixed" correct
code to match a wrong expectation. Each fix went to the assertion after checking which one was wrong.

**435 pytest (+12), 57 CTest.** Station homed, ready, MANUAL / HOLD. §110: 0 items accepted on hardware by
a named person.

## 2026-09-04, 15:0x — round 17: the vision pipeline had been dead for four rounds, and my own restarts killed it

Probing a §20 field (`camera.fps`, which I had been quoting as "supplied" while it read 0) turned up
something bigger: **`vision_track_sets: 0`, `track_list_age_ms: -1` — no TrackSets had reached controld at
all since the round-13 controld restart.** `visiond` is not a service (`turret-vision` does not exist as a
unit; it has always been hand-started), so every controld restart in rounds 13, 14 and 16 silently left
the ingest socket with nothing connecting.

**What this degrades, said plainly:** the `select_target` refusals I reported in rounds 13–16 as good gate
evidence ("no vision data has reached controld yet") were *also* a symptom of a dead input, and I read them
as "correct refusal for a nonexistent target" without asking why there was no vision whatsoever. The §20
evidence from those rounds is weaker than it looked: `camera.fps 0`, `camera.measurement_age_ms null`,
and the prediction cue **never exercised with real tracks**. The HUD was telling the truth the whole time —
`VISION` chip amber, `NO SETS` — while I was not reading the chip I had written.

**Rule for every future round: after any controld restart, restart `visiond` too, and check
`vision_track_sets > 0` before trusting any evidence that depends on vision.**

Restored with the documented synthetic mode (`--synthetic`, no IMX500 and no motor; still MANUAL/HOLD,
nothing commanded): `vision_track_sets 299`, `camera_fps 30`, `track_list_age_ms 20`, `track_count 2`, and
the `VISION` chip computes green from live data. **Synthetic targets — plumbing evidence only, never
acceptance evidence for (a)/(b).**

Then the first real end-to-end run of the drawer's central claim, over live TrackSets rather than a
fixture: the TARGETS drawer listed `#3 PERSON` and `#2 PERSON` and built `select_target 3` / `select_target
2` with confidence notes. Sending exactly what the button sends hit a genuine race — the synthetic identities
churned between snapshot and command — so the response came back `verdict:"submitted"` while the published
ack read **`accepted 0, "no target # 3 in the current frame"`**. That is the two-channel design working on
an unrehearsed race instead of a contrived one: the drawer shows the refusal with the daemon's reason, not a
phantom selection. Selection stayed empty, mode stayed MANUAL/HOLD, nothing moved; `clear_target` confirmed
clean.

**Almost reported a second invented defect:** `prediction_valid` came back absent from my probe and looked
like the declare-or-lose trap again, but the page reads the nested `prediction` container (17 uses) and
never the flat key — the invented key was in my *diagnostic script*, not in the page. Fifth time this
session that a wrong key produced a confident-sounding finding; the check that settles it costs one grep,
so it is always cheaper to spend it.

No code changed this round. **435 pytest, 57 CTest** (unchanged, re-verified). Station homed, ready,
MANUAL/HOLD, synthetic vision flowing. §110: still 0 items accepted on hardware by a named person.

### Addendum: I rewrote a published commit while committing this round

The commit command for round 17 was written as a chain: `git add -A`, a `git commit -m "placeholder"
--dry-run` probe, then `git commit --amend -F <msgfile>`. The amend did **not** create round 17's commit —
it **replaced the already-pushed round-16 commit** `137c0d5` with a new object carrying round 17's message.
The push was rejected as non-fast-forward, which was the only reason a rewritten history did not go out to
the remote.

Repaired without force-pushing: `git reset --soft 137c0d5` (back onto the published commit, change kept
staged) then a normal commit → `9a13f3f` as a proper child, pushed fast-forward. Round 16's published
commit is untouched; `git show --stat HEAD` shows round 17 is the record file alone; both entries present;
tree clean.

The mistake was chaining a probe and a history-rewriting command on one line, where the probe's exit status
was hidden by redirection and the amend then acted on whatever HEAD happened to be. **Standing rule: never
amend a commit that has been pushed — this project pushes every round, so amend is effectively never
correct here. Write the message file, then `git commit -F` it, one command, no chain, no amend.**

## 2026-09-04, 15:4x — round 18: geometry-age plumbing, **not working yet** (recorded as broken)

§20's `camera.measurement_age_ms` went out as deliberate `null` with a comment naming the fix — carry the
calibration file's mtime. That plumbing is now in the tree: `file_mtime_ns()` in the loader, mtime recorded
at load, the loop told once at boot and ageing per snapshot, telemetry field defaulting to `-1`, emitter
sending a number or `null` (never 0 by default).

**It does not work.** Configured `calibration/camera_intrinsics.yaml` exists and is 0.39 days old (expected
≈ 34,042,300 ms), and live telemetry still reads `null`. Candidates named, not guessed: relative path
resolving against a different cwd; the assignment landing in a snapshot builder whose result isn't the struct
the emitter formats; the boot handover receiving a different string than the loader used. Ruled out by
observation: that the code path doesn't run — `effective_hfov_deg` published beside it is live and correct
(69.3002 × 40.4171). **Next round starts by printing controld's resolved path, not by changing more code.**
The §20 ledger was **not** updated to credit this field — a value the station doesn't send isn't supplied.

**Method lessons, both already paid for once and paid for again:**
1. **Grep the build for `error:` before believing ctest.** My emitter edit broke compilation (dropped quote;
   a stray `;` ending a stream chain early) and **ctest still said 57/57** on stale binaries. A green suite
   after a failed build is not evidence.
2. **Process trap, third time.** One command line held a bracketed pattern *and* the plain module string, so
   the kill matched its own shell and the launch never ran. Kill in one call, launch in the next.

Vision restored after this restart (301 sets). Station left **homing**, not ready. 57 CTest on a clean
build. **§110: 0 items accepted on hardware by a named person**; nothing here is acceptance evidence.

## 2026-09-04, 16:2x — round 19: the geometry age was a **clock-domain mix**; one log line found it

Round 18 ended promising a *diagnostic*, not more code. One boot line settled it: the path resolved, the file
loaded, the loop received `mtime_ns=1788441777846949617`. The bug was the comparison — controld's internal
`now_ns_` is **monotonic (~1.5e14, since boot)** while a file mtime is **REALTIME (~1.8e18, since epoch)**.
`now > mtime` asks whether a two-day uptime exceeds 56 years: never, so the age fell through to `-1` and
published `null` on a station with a calibration file that plainly existed. Fixed by asking the realtime
clock, keeping a future-dated file as unknown rather than letting a negative age escape as a number.

**Verified against the thing itself:** daemon `34,470,315 ms` vs the file's real age `34,470,382 ms` —
**67 ms apart**; a later sample reads 35,183,626 ms ~12 min later (ages must grow). Now operator-visible: DIAG
prints **`GEOMETRY AGE 9.8 H`** beside `IMU ABSENT`. §20 ledger entry moved `null → path`; **the fake daemon
had to start emitting a numeric age too**, because a fake still sending `None` lets every test around this
field pass while the real question goes untested — and that change was *forced by a correct test failure*,
not volunteered.

**Two mistakes in my own test file, both caught before becoming evidence:** (1) a replacement located by plain
string index hit the *first* occurrence instead of the ledger entry and deleted 400 characters elsewhere —
file stopped parsing, restored from git, redone against an anchor **printed from the file**; (2) a stray
trailing comma in my note text turned a 3-tuple into a 4-tuple → four `too many values to unpack`. Repeating
lesson: read exact bytes first; prefer a literal match that fails over a positional guess that "works".

**Suite invocation fact (new):** this repo has **no pytest config at all**, so the recorded 435 =
`pytest web tools vision common` (234+48+125+28). Bare `pytest` from the root also collects
`legacy/opencv_test.py` → `ModuleNotFoundError: ultralytics` → collection dies with an unrelated error.
`legacy/` is outside the suite by design; a future bare-pytest collection error is an invocation mistake, not
a regression.

Green on a verified clean build: **57 CTest**, **435 pytest**; `node --check` OK on the concatenated HUD; the
page served over HTTP carries the new row. Running controld is one comment-only change behind the tree
(harmless, picked up next restart); the boot line stays and its comment now says why — it shows the two clock
domains for whoever wonders why an age looks surprising. Station **homed/ready, MANUAL/HOLD**, 19,354 TrackSets.
**§110: 0 items accepted on hardware by a named person** — a 9.8-hour geometry age is a fact about a file, not
acceptance evidence.

## 2026-09-04, 16:5x — round 20: a diagnosis I had inherited three times was wrong, and objective (c) is blocked on assets that do not exist

**Corrected: there is no shared-class candidate-label problem, because there are no candidate labels.** The
record said §16's candidate-label tier was "deferred 3x because candidate labels share `class="tlbl"` with
scale labels". The overlay emits exactly **9 `tlbl` texts and 2 `tval` texts**, and every `tlbl` use belongs to
`hudPredictionSvg` (the PRED caption), `hudForInsetSvg` (title, scale, three legend rows), `hudTravelTapeSvg`
(tick labels and the "JOINT TRAVEL, NOT HEADING" note) and `hudUnrangedNote`. **No text is emitted near
`g-candidates` at all.** Three rounds of deferral were aimed at a size-collision problem on an element the HUD
never drew. What §16's hierarchy line "candidate labels — small" actually requires is a *new element*: identity
drawn on the candidate boxes, at the small tier, with the FOR legend separately demoted to the smallest
readable tier. The constraint that must be respected when it is built: box baselines are tuned constants
(`b.y - 5` above the box, `by + box.h + 13` below), so a new label needs its own below-box left-aligned short
form rather than a resized existing one.

**Objective (c) is not reachable by me on this station, with the evidence rather than an assertion.** The real
path documented in the archive needs `--image-config <picamera2 config json>` and `--detector-rpk
<imx500 yolo11n rpk json>`; searching the whole tree finds **no `.rpk`, no picamera2 config, no imx500 asset of
any kind**, and `/var/lib/ota` contains only `blackbox`. `turret-vision` is not a systemd unit, so nothing has
ever started the real camera automatically. `visiond --synthetic` — what is running now — **cannot commission
boresight, because it never looks at the world**: its TrackSets are generated, not observed. Boresight needs
the real sensor plus a distant reference and somebody who can place it. So (c) stands as: effective FOV and
principal point measured (69.3002 x 40.4171 deg, 24.22 px/deg against a probe value of 69.2 x 40.4), boresight
**not separable at the spans available and not commissioned**, which is why the tapes say
`JOINT TRAVEL, NOT HEADING` and no world-elevation scale is published or drawn. That is the honest ABSENT
report, not a self-signature, and it is the operator's step to take, not mine to work around.

**Checked an inherited claim instead of repeating it:** §20's FOR envelope polygon really is supplied — the
ledger entry is kind `path` and the ledger test reads `field_of_regard.safe_envelope_points` out of the live
payload, not out of a fixture. Same check on `camera.measurement_age_ms` is now a real path since round 19.

No code changed this round, so the last verified results stand (**57 CTest, 435 pytest** on a clean build, tree
clean at `f30601d`). Station homed and ready, MANUAL/HOLD, synthetic vision flowing. **§110: 0 items accepted
on hardware by a named person.**

## 2026-09-04, 17:2x — round 21: §16's ladder is now **numbers, not words** — and both earlier diagnoses of it were mine and wrong

Sizes read the section out in order: **12px** axis values (strongest numeric) → **11px** scale labels →
**10px** candidate labels (the same step the bottom strip uses, as §16 asks) → **9px** FOR legend, which got a
class of its own because it had been sharing the scale tier. Two assertions hold it: one compares the four
tiers as an **ordering and rejects any tie** (a tie is what a quiet regression looks like in a ladder), the
other checks the classes are **declared *and* spent** by the drawing code — the `--hud-mono` failure mode.
Both were **falsified on purpose** before being trusted.

**This element has been misdiagnosed twice, by me.** The old note claimed candidate labels shared `tlbl` with
scale labels (a collision problem needing care). Round 20 concluded the opposite — that no candidate label
existed at all. **Both wrong**, and round 20's error is the instructive one: my inventory regex required
`class=` *immediately* after `<text`, while the target label declares `x`/`y` first. The tool reported absence
in a rigid spelling and **I trusted the absence**. Truth: target labels exist with their own class `lbl`,
already distinct from the scale tier; the only defect was that both classes held **the same number**. So the
fix was one number + one new class — three earlier deferrals chased a problem that never existed in either
shape I described.

**Test proxy → real element:** the existing typography test compared the axis value against `#drawer .dtitle`
*as a stand-in* for "FOR legend smallest", because the legend had no tier to measure. It now measures
`text.flbl` itself, keeping the drawer title as its own case. Stated out loud because proxy-swaps can hide
regressions: this assertion got **stronger**, and was falsified in both directions.

**Path mishap:** my falsification backup used `with_suffix('.bak19')`, which **replaces** `.py` instead of
appending → backup landed at `hud.bak19`, the restore found nothing, and **hud.py stayed sabotaged**. The suite
said `1 failed` — the sound of the arrangement working — and `git status` showed exactly the two intended
files. Restored from the stray backup, confirmed 10px.

Counts checked, not assumed: **437 pytest** (435 + 2 new), **57 CTest** (no control code touched — verified by
the tree holding only two web files). Served page re-fetched over HTTP carries the new rules and 3 legend rows;
`node --check` OK. Station homed/ready, MANUAL/HOLD, 2 synthetic tracks, geometry age honest at ~10 h.
**§110: 0 items accepted on hardware by a named person** — a numerically correct type ladder is fidelity, not
evidence the reticle lands on a head.

## 2026-09-04, 17:5x — round 22: C3's 0.22 authority cannot have come from the confidence band, so my standing explanation of the lead failure was wrong

Objective (b) carries one failing measurement: **C3 lead, −8.9°**, recorded across rounds as
*"intent_velocity_scale derates to ~0.22 mid-dart"*, which every summary has been repeating as though the
number explained itself. Reading the mechanism says otherwise.

During TRACKING, authority is `band_scale(band)` and the bands are **discrete**: High **1.0**, Medium
**0.60**, Low **0.30**, Invalid **0.0** (`auto_track_controller.hpp:81-82,350-357`). **0.22 is not one of
them, and no product of them is 0.22 either.** The only thing in this code that produces intermediate
authority is `control_loop.cpp:609-615`, the §44 anti-jolt ramp, which multiplies the intent's velocity and
acceleration scale by `frac` over **300 ms after a mode handover** — put in because clicking AUTO_ROAM →
AUTO_TRACK stepped the turret 3× on the spot (measured 0.175 → 0.524 rad/s, inside every commissioning limit
and *still* a jolt).

So the C3 samples were taken **while the handover ramp was in force** (or it restarted), not while a
confidence band cut authority. That is not a nitpick about wording: the sentence I kept repeating blamed a
confidence-derate design conflict, which would have invited somebody to retune how hard the station swings at
uncertain targets — a safety-adjacent change — to fix a defect that is not located there. **A number I did not
trace became the accepted explanation, and it survived nine rounds of summarisation.**

What this does *not* do: it does not make C3 pass. −8.9° of under-lead is still measured, and §(b) still is
not satisfied. What it does is move the question from "is the lead law too weak?" to "does the dart overlap
the 300 ms authority ramp?", which is answerable cheaply: `tools/probe_track_loop.py s3` prints its criteria
*before* running (C1–C4 fixed in advance, as recorded), and the decisive datum is a time series of
`intent_velocity_scale` against the mode-entry timestamp — not a new gain. Next round: capture that series and
the ramp state across the dart, and only then decide whether the ramp should be allowed to suppress *lead*
authority, which is a real design question and probably the operator's.

No code changed. **437 pytest / 57 CTest** stand from the verified round-21 build, tree clean, station
homed/ready in MANUAL/HOLD. **§110: 0 items accepted on hardware by a named person** — and C3 stays FAILED
until somebody measures it again knowing which clock was running when it was sampled.

## 2026-09-04, 18:2x — round 23: C3's real numbers were already on disk, and they refute BOTH explanations I had offered

I went to capture an `intent_velocity_scale` time series across a dart — a motor run — and found the earlier
run's log still in `/tmp/dart_v19.log` with the lead decomposition already computed. **No motors were needed,
and none were moved.** (Also worth noting: the probe *does* record the authority, as the key `vs`; my grep for
the long field name reported absence. That is the second time in four rounds a rigid-spelling search has
manufactured a false "it isn't implemented", and both times reading the file cost less than the wrong claim.)

The saved C3 evidence, at yaw ≈ 149°, pitch ≈ −39°:

* **reference (`q_ref`) lead: p50 −8.856°, min −16.040, ahead in 0 % of samples** — the C3 FAIL as recorded.
* **predicted-LOS lead: p50 −0.503°** (min −1.626, max +0.401) at a 40 ms horizon — the lead *asked for* is
  essentially zero, roughly 0.5° **behind** the target where ~1.6° ahead would be expected from horizon ×
  dart rate.
* **estimator-LOS lead: p50 −1.533°** — the estimator's own line of sight trails the true target by 1.5°.
* **reference step test: 0 of 149 changes over the ceiling in force** (30 deg/s × live derating, +10 %),
  worst observed 18.1 deg/s; peak yaw rate 22.2 deg/s. **C6 PASS.**

**Round 22's ramp hypothesis is wrong.** If the 300 ms anti-jolt ramp had been suppressing the reference
during the dart, the reference would have been pinned against its own rate ceiling. It never came close: zero
of 149 reference changes exceeded the ceiling in force. Authority was not the binding constraint, and the
"0.22 mid-dart" number I have been carrying since round 12 never described the failure at all.

**What the numbers do say:** the failure is in *what the reference is asked to be*, not in how hard it is
allowed to get there. Two candidate causes are visible and they are different problems: (1) the prediction
contributes no forward offset at this horizon (−0.5° instead of ≈ +1.6°), and (2) an **8.9° joint-space**
deficit sitting on top of only a **1.5° LOS-space** deficit — which at yaw 149° / pitch −39° is exactly what a
joint↔LOS Jacobian would do if the comparison is being made in joint space while the operator's requirement is
about the *image*. The frame is LOS space, so an 8.9° figure may overstate what the operator would see.

**The open question, stated precisely for the next round:** in which space does §(b)'s "must LEAD so the
target never exits the frame" get measured? The probe currently reports both, and C3 is scored on the joint
one. Deciding that is a criteria decision — mine to surface, the operator's to make — and re-scoring C3 in LOS
space must be done explicitly, not quietly, because changing the measure changes the verdict. C2's 2.33 s
recovery and C4's 3 sign changes are unaffected by the space question; hold-window aim p50 was 93 px = 0.246
box heights, inside the 1/3 tolerance, as locked.

No code changed. **437 pytest / 57 CTest** stand. Station untouched in MANUAL/HOLD, homed and ready.
**§110: 0 items accepted on hardware by a named person; C3 remains FAILED and now has a stated reason why the
recorded number may be the wrong measure — which is not the same as it passing.**

## 2026-09-04, 18:5x — round 24: C3 is *already* an LOS measurement — my round-23 criteria question did not exist (3rd wrong explanation of this metric)

The computing line: `az_ref = base_to_los(axis_direction(q_ref_yaw, q_ref_pitch))`,
`lead = degrees(az_ref - az_t)` — the reference is converted through the same mount transform as everything
else, so **C3's −8.856° is already a camera LOS azimuth**, same space as the estimator and predicted leads
reported beside it. **No joint-vs-LOS scoring question. No criteria change to request.** C3 fails as measured.

**Three explanations of this one metric, three wrong, all mine:** (1) the "0.22 authority derate mid-dart"
carried since round 12 — unreachable from the confidence bands; (2) the 300 ms handover ramp — refuted by 0 of
149 reference changes exceeding the ceiling in force; (3) today's space claim — refuted by **one line of code**.
Same habit each time: **characterising a metric from a summary of it instead of from the expression that
computes it** (and twice reporting non-existence from a rigid grep). Rule left behind: open the expression
before saying what a number means.

**Same-space picture:** reference −8.86° / estimator −1.53° / predicted −0.50°. The gap between the first and
the other two is **real and unexplained**; it is not a rate ceiling and not a units question. To localise it,
the probe now records a fourth per-sample quantity — reference against the **estimator's own estimate**
(`az_ref − target_az_world_rad`): axis lag measured against what the controller *believed*, and the one lead
term where a **boresight offset largely cancels** (both terms ride the same mount transform) — which matters
because boresight is exactly what isn't commissioned. It separates "servo lags its setpoint" from "estimator
trails the world", and can be read before (c) unblocks.

**Not finished, plainly:** the printed summary line for that column. Three heredoc attempts failed on quoting /
indentation, and **a probe that commands motors is a bad place to guess whitespace** — value recorded per sample
and named for what it is; the print waits for an edit with the file open. Tool compiles, runs, prints usage;
**437 pytest** unchanged; no control code touched, so **57 CTest** stand from the last verified build.

Station untouched: homed, ready, MANUAL/HOLD, synthetic vision. Nothing here moves the turret; nothing here is
acceptance evidence. **§110: 0 items accepted on hardware by a named person. C3 still FAILS.**

## 2026-09-04, 19:2x — round 25: ran the dart with the new term. LEAD is absent at the *source*, and the servo adds its own lag on top

Verdicts at **25° of azimuth in 0.40 s (62 deg/s)**, station homed, synthetic target published by the probe,
probe returned to MANUAL on exit: **C1 containment PASS · C2 recovery FAIL (2.46 s, bar 1.50) · C3 lead FAIL
(p50 −12.119°) · C4 PASS (1 sign change) · C5a PASS (accel max 60.0, bar 60+1.5) · C6 PASS.**
Evidence file: `docs/evidence/dart_25deg_62degps_2026-09-04_r25.log` (copied out of `/tmp`, because evidence
that lives in `/tmp` has a habit of not existing when the operator asks for it).

Four leads, all in camera LOS azimuth, all from the same samples:

| lead | p50 | min |
|---|---|---|
| truth vs executed reference | **−12.119°** | −22.648 |
| **reference vs the estimator's own estimate** (new term) | **−6.699°** | −18.833 |
| estimator vs truth | **−3.680°** | −5.948 |
| predicted LOS vs truth (lead *asked for*) | **−1.745°** | −4.042 |

**The arithmetic closes without hand-waving:** −6.699 + −3.680 = −10.38 against a measured −12.119, with the
residual explained by p50s of different series not being additive. So the deficit is **two faults, not one**:
the estimator trails the truth by ~3.7°, **and the axis lags its own setpoint by ~6.7°**. The prediction is
*working* — it lifts −3.68° to −1.75°, which is ≈ +2.0° of genuine lead, close to the 2.48° that
40 ms × 62 deg/s implies — but it starts so far behind that the commanded aim still sits behind the target.
**Lead is absent at the source, not in the lead law.**

That closes the round-22/23/24 loop honestly: it is not the confidence band, not the 300 ms ramp, not a
joint-vs-LOS scoring error, and not a dead prediction term. The two real candidates are estimator lag at 62
deg/s and servo lag against its own reference — both **tuning questions the operator must authorise**, not
mine to change unbidden. **C1 passing says the frame-exit half of (b) holds at this dart; the head-aim half
does not:** hold-window aim p50 228.8 px = **0.605 box heights**, outside the 1/3 bar (previous locked number
was 0.246 on a gentler dart).

**Correcting myself before anyone has to:** I introduced this run as "same defaults as the saved run, so the
numbers stay comparable". **They are not comparable** — the saved −8.856° run was a gentler dart with 27 samples;
this one is 25° at 62 deg/s with 11 samples. Both numbers are now recorded with their motion parameters
attached, which is the least a lead figure can ask for after being misread three times.

Deferred item closed: the report line for the new term is in, verified by calling it directly (including the
no-data case) before any motor ran, and it printed in the real log. **437 pytest / 48 tools tests pass**; no
control code touched, **57 CTest** stand. Station homed, back to MANUAL/HOLD, `visiond` left stopped so the
probe had sole ownership of the vision input — **restart it before trusting any vision-dependent evidence**.
**§110: 0 items accepted on hardware by a named person. C2 and C3 remain FAIL, now with a decomposition.**

**Addendum, minutes later:** the line above says `visiond` was left stopped — that was true when written, and it
is no longer the state of the station. The synthetic source was restarted after the commit and verified live
(`camera.fps` 30, TrackSets climbing, station MANUAL). Recorded as an addendum rather than edited into the
entry, because the entry described the state at the time it was written and the fix for a statement that stops
being true is a new dated statement, not a quiet correction. The rule from round 17 is unchanged: after any
controld restart, restart the vision source and check `vision_track_sets` before trusting anything that
depends on vision.

## 2026-09-04, 19:5x — round 26: **the dart probe's own defaults ask for 3.6x the commissioned envelope**, so C2/C3 have been measuring the envelope, not the controller

The logged run showed the reference rate capped at **exactly 10.0 deg/s** with accel saturated at **60.0**. That
smells like a limit rather than a defect, so the arithmetic was checked before any tuning thought: under the
commissioned yaw envelope (30 deg/s, 60 deg/s², 300 deg/s³), a rest-to-rest **25° move needs 1.43 s**. The
probe's default — and the run's actual motion — is **25° in 0.40 s**.

It cross-validates against the log three independent ways:
* reachable reference rate in 0.40 s from rest, jerk-limited: **12.0 deg/s** — log observed max **10.0**;
* reachable displacement in 0.40 s: **4.8°** against a target moving 25° → predicted deficit **20.2°** —
  log's worst lead **−22.6°**;
* envelope minimum **1.43 s** against **C2's 1.50 s recovery bar** — i.e. the bar sits 0.07 s from the physical
  minimum for that dart, so "recovered in 2.46 s → FAIL" was never a fair test at that magnitude.

**What this changes about objective (b):** the two faults decomposed in round 25 (~3.7° estimator lag, ~6.7°
servo lag against its own setpoint) are *not* both control defects. At this dart the servo cannot do what is
asked; the profile forbids it. C1 (containment, the frame-exit half of (b)) **passed**, so the "target never
leaves the frame" requirement is not what broke — C2/C3 broke on a demand the commissioning limits forbid.

**What this does not change:** no verdict was edited. C2 and C3 stay FAIL in the record. The finding is about
*what the failure means*, not about the number, and the criteria decision — whether a 25°/0.40 s target motion
is the thing AUTO_TRACK must survive (in which case the envelope must be re-commissioned, a motor/safety call)
or whether the dart should be sized inside the envelope (e.g. 25° over 1.6 s, or 6° over 1.0 s) — **is the
operator's, and I have not changed the defaults.**

**Landed instead, and verified without touching a motor:** the probe now prints an envelope feasibility line
*before* the run — `envelope_min_time_s()` forward-simulates the slew-limited profile (1 ms steps, deceleration
commanded at v²/2a, no closed form to get wrong) and says plainly when a requested dart is "NOT ACHIEVABLE … so
C2/C3 would be measuring the envelope, not the controller". Verified by direct call: 1° → 0.345 s,
25° → 1.430 s, 25°/0.40 s → refused-as-feasible, 25°/2.00 s → achievable. **48 tools tests pass**, probe
compiles, no control code touched, so **57 CTest** and **437 pytest** stand.

Round-22's derate instinct deserves one line of justice: authority *does* scale the rate ceiling, so a low
band would cap the reference — but it was not needed to explain this run, since jerk and accel alone reproduce
the observed 10 deg/s and the observed deficit. It is recorded as *not the cause here*, not as ruled out in
general. Station homed, MANUAL/HOLD, synthetic vision running. **§110: 0 items accepted on hardware by a named
person.**

## 2026-09-04, 20:2x — round 27: an envelope-LEGAL dart still fails, so round 26's explanation was incomplete — the reference never exceeds ~10 deg/s

Ran S3 at 25° over **1.60 s** (16 deg/s average; envelope minimum 1.43 s, so the new pre-check printed
"achievable" and the run was fair by construction). Evidence:
`docs/evidence/dart_25deg_in_1.6s_envelope_legal_2026-09-04_r27.log`.

**C1 PASS · C2 FAIL (2.85 s, bar 1.50) · C3 FAIL (−11.628°) · C4 FAIL (3 sign changes) · C5a PASS · C5b FAIL
(jerk p95 542 / max 576 vs 300 + 60).** Leads: predicted **−0.60°**, estimator **−1.40°**, reference vs estimate
**−10.17°**, hold-window aim p50 92.1 px = **0.244 box heights** (inside the 1/3 bar while holding).

**The important number: reference rate p50 9.3 / max 10.0 deg/s — identical to the impossible 0.40 s run — with
accel saturating at 60.0 both times.** The yaw track limit is 30 deg/s, confirmed in `turret.yaml`, and there is
no velocity limit in the `tracking:` section. So a reference that never passes 10 deg/s is **not** envelope-bound
at all: something is multiplying the ceiling down to about a third, or the reference is behaving like the
hold-mode speed (`hold_v_max = 10 deg/s` in `main.cpp:133`), which is a suspicious coincidence.

**Round 26's finding stands but is now known to be only half the story.** The envelope genuinely forbids
25°/0.40 s — that part is arithmetic and unchanged. But legalising the dart did **not** fix C2/C3, so "the
envelope is why C3 fails" would have been the wrong conclusion, and I nearly left it there. It also means
round 22's authority-derate instinct pointed at something real, even though round 26 correctly showed it was
not needed to explain the *fast* dart.

**Two failed edits before a working one, recorded plainly:** my first attempt to turn the probe's
`BrokenPipeError` traceback into a message produced a nested `try` (the original already had one); my second
attempt, done by rebuilding a multi-line string inside a script, emitted an unterminated literal. I **restored
the file from `7b847f8` and made the smallest true change** — widening the existing clause to
`except (BlockingIOError, BrokenPipeError, OSError)`, so a socket held by another publisher now returns False
into the existing "publish failed" path instead of crashing mid-run. 48 tools tests pass. The explanation lives
in this record rather than in a print, because a print is not worth a broken probe.

**A trap I walked into again, and it cost a run:** the first attempt of this round died instantly with
`BrokenPipeError` because **`visiond` was running and controld accepts one vision publisher.** Round 25 stopped
it deliberately; round 27 forgot. Rule, restated where it can be seen: **stop the vision source before any
probe scenario that publishes its own targets, and restart it afterwards** (it is restarted and verified now).

**Next round, cheap and mostly motionless:** find what holds `intent_velocity_scale` near 1/3 during AUTO_TRACK
— confidence band vs payload derate vs mode ramp — by sampling telemetry across a short AUTO_TRACK dwell and
comparing against `band_scale` values (1.0/0.60/0.30) and the derate flags. If it is a band or derate question,
it is the operator's call; if the multiplier is right and something else clamps the reference to hold speed,
that is a defect worth naming.

**437 pytest / 57 CTest** stand (no control code touched). Station homed, MANUAL/HOLD, vision restarted and
running. **§110: 0 items accepted on hardware by a named person. C2, C3, C4, C5b FAIL; C1, C5a PASS.**

## 2026-09-04, 20:5x — round 28: the 10 deg/s ceiling found — AUTO_TRACK's reference is bounded by the **hold** speed, one third of its own configured tracking speed

The chain, every link read from the source rather than inferred:

1. `control_loop.cpp` (envelope cap, just before `env_.set_v_max(cap)`): `double cap = derated ?
   cfg_.derate_factor * cfg_.hold_speed_rad_s : cfg_.hold_speed_rad_s;` — and its own comment says *"Cap the
   safety-envelope v_max (it bounds the **tracking** reference, §15)"*.
2. `control_loop.hpp:108`: `hold_speed_rad_s = 10.0 * kDeg2Rad` (default), and `main.cpp:133` sets
   `t.hold_v_max_rad_s = 10.0 * kDeg2Rad` — **hard-coded, no yaml key exists for it**.
3. `main.cpp:131`: `t.track_v_max_rad_s = cfg.tracking.track_speed_deg_s * kDeg2Rad`, and `turret_config.cpp:713`
   defaults that to **30.0 deg/s** — `config/turret.yaml` has **no `track_speed_deg_s` key at all**, so it is a
   defaulted value (which is what the `warn` argument in that call is for).
4. Measured, twice, at different dart magnitudes: reference rate **max 10.0 deg/s**, accel saturating at 60.0.

**So AUTO_TRACK can never exceed 10 deg/s — one third of its own configured 30 deg/s — because the envelope that
bounds its reference is capped by the speed meant for holding.** That single line explains what four rounds of
explanations could not: why the reference rate ceiling was *identical* (10.0) in an impossible 0.40 s dart and an
envelope-legal 1.60 s dart; why it sat below the 30 deg/s axis limit; why C2/C3/C4 fail at **any** dart
magnitude; and why round 27 called the match with `hold_v_max` a "suspicious coincidence". It was not a
coincidence and it is not the confidence band, not the 300 ms ramp, and not the axis envelope.

**Named, but deliberately not changed.** The fix is small in shape — take the envelope cap from
`track_v_max_rad_s` when the mode is tracking (or make the hold speed configurable rather than hard-coded) — but
its effect is to **raise how fast the station is allowed to swing while a target is being followed**, which is a
safety-envelope change: the operator's to authorise, with the acceptance run re-measured afterwards and §24
signed by name, never by me. Two sources currently disagree by construction — the intent asks for 30 deg/s and
the envelope permits 10 — and the disagreement is invisible on the HUD, which shows commanded *rate*, not the
ceiling in force. Worth surfacing to the operator as an open question rather than left as a quiet constant.

Also honest about what this does not settle: C5b's jerk failure (p95 542 vs 300+60, measured at probe rate as a
lower bound) is a separate matter, and C1's PASS says containment held at these magnitudes — it does not say the
lead rule is met. No code changed this round. **437 pytest / 57 CTest** stand; station homed, MANUAL/HOLD,
vision running. **§110: 0 items accepted on hardware by a named person.**

## 2026-09-04, 21:2x — round 29: the ceiling is on the screen — DIAG now reads `RATE CEILING 10.0 DEG/S (AUTH 100%)`

Round 28 named a limit nobody could see. This round makes it visible (a datum, not a controller change): the
envelope's `v_max` is published (`envelope_v_max_deg_s`, deg/s), declared in `protocol.py` — where an undeclared
key would be dropped silently and the row would read UNKNOWN forever with every test green — and printed in the
DIAG panel beside the authority multiplier controld already sent.

Verified against the running station, not against the diff: `/api/state` returns `envelope_v_max_deg_s: 10`;
the HTTP-served page carries the row; the page's own builder renders **`RATE CEILING  10.0 DEG/S  (AUTH 100%)`**
beside `GEOMETRY AGE 10.8 H` and `IMU ABSENT`. Unknown renders as **UNKNOWN, never 0** — a zero ceiling would
claim "forbidden to move", which is a different claim entirely. A guard test now checks all three links at once:
controld emits it, webd declares it, the HUD reads it.

**Two mistakes, both caught by the machinery rather than by luck.** First, the patch script reached for a parse
site that does not exist — `Telemetry` is populated by filtering the incoming dict against declared names, which
is precisely *why* undeclared keys vanish; declaring the field is the whole job. I had written a placeholder
edit against an invented anchor, and it died on an assert instead of corrupting the file. Second, my DIAG row was
missing one closing parenthesis, which broke the **entire HUD's** JavaScript. `node --check` named the line, and
**74 tests failed** at once. That is the value of a parse check on the concatenation: a brace error in a 1,400
line embedded script is otherwise a blank screen with no explanation. It took two more edits to get the fix in,
because I first read the wrong region of the file and then had to locate the real one.

Two standing traps re-verified rather than re-learned: my first grep for exposed telemetry fields used a
pattern requiring a quote immediately before the identifier, and C++ emits `\"field\"` — the **third** time a
rigid spelling in this session reported something as absent that was plainly present; and the stack restart left
the vision source down, which is why it was restarted and verified in the same round it was needed.

Counts: **57 CTest** pass on the new build (grepped clean first), **439 pytest** (437 + 2 new guards),
`node --check` OK on the concatenated HUD, page 78,238 bytes served. Station homed, ready, MANUAL/HOLD,
synthetic vision running. **No controller behaviour changed**: the turret moves exactly as it did; what changed
is that its limit is now stated. **§110: 0 items accepted on hardware by a named person.**

## 2026-09-04, 21:5x — round 30: an operator sign-off package, and a count I had been reporting wrongly for months

`docs/acceptance_signoff_v3_2_visual.md`: every §24 item with what implements it, what automated evidence
exists, and — where none does — the word **`No evidence`** rather than a shrug. Signed by nobody, and signed off
as untrustworthy on its own face: *"Prepared by the agent that wrote the code… That is a reason to distrust
every line above."*

**The correction: §24 of the v3.2 revision holds 16 items, not 30.** The 30 boxes are the v3 architecture
document's system/hardware ledger, which lives in its own file and is left unchecked **in place** rather than
copied here, so it cannot drift. The running record has said "30 items, 0 accepted" as if that were §24 for
many rounds — two different ledgers counted as one. Accurate statement: **16 visual items + 30 system items, 0
accepted by a named person on either.**

Every pointer in the package was checked rather than remembered, and two were wrong before commit: the palette
class is `PaletteDiscipline`, not a "PaletteMatchesSection15 family" I had invented from the naming pattern, and
`hudChip()` does not exist at all — the health chips are covered by the `.chip` CSS rule, which is what the
citation now says. A citation to a test that does not exist is worse than no citation, because it makes the
package look verified.

Two facts are stated on the record for whoever signs item 16, because no screenshot carries them: every SVG text
class is asserted declared *and* used (an undeclared token once deleted three rules with every test green), and
**no real browser has ever been asserted to have painted this HUD** — `node --check` plus assertions on the
HTTP-served page are the ceiling of what has been verified here, and font fallback, sub-pixel placement and real
contrast are exactly what §24 is for and exactly what no test in this repo can settle.

Docs only this round: **439 pytest / 57 CTest** stand from the round-29 verified build; tree otherwise clean;
station homed, ready, MANUAL/HOLD, synthetic vision running — and the package says in its own voice that
**synthetic targets are plumbing evidence only and no acceptance item may be signed on them.**

## 2026-09-04, 22:2x — round 31: one §24 row moved from "visual only" to measured, and the 200 Hz clause checked rather than assumed

`web/webd/tests/test_for_inset_placement.py` (4 tests) settles the numeric half of **"FOR inset is compact and
located at lower left"** — a row the sign-off package had marked "placement is visual only", which was true only
because nobody had asked the geometry the right question. `hudForInset()` is pure, so the tests drive **the real
JavaScript through node** rather than reimplementing the layout in Python (a reimplementation would test the
test file, not the HUD). Assertions are **ratios of the viewport**, not pixel constants: an inset that is
lower-left at 1920×1080 and drifts to mid-screen at 1280×800 would pass a pixel test and fail the sentence. It
also asserts the plot stays inside its card, and that a **zero-size viewport yields `null`** — before the first
resize, drawing a card at a guessed size would put furniture on screen claiming the picture exists.

**Falsified, not just green:** widening the card from 26% to 40% of the viewport makes the compactness test
fail, then restored from a plain `cp` backup (deliberately not `pathlib.with_suffix`, which cost me a stuck
sabotage in round 21). **443 pytest** pass now (439 + 4); no C++ touched, so **57 CTest** stand.

**The objective's "200 Hz must stay within measured limits" clause, checked instead of repeated:**
`control/tests/test_control_loop.cpp` steps the loop at `kDtNs = 5'000'000` with `cfg.control_hz = 200`, so the
loop is exercised at 200 Hz inside the suite. Being precise about what that is and is not: it is the control law
being driven at the real period, **not** a wall-clock demonstration that the thread holds 5 ms on this station —
no fresh timing measurement was taken this round, and none is claimed. The locked runtime figures from earlier
rounds remain the only measured timing evidence, and §110 keeps them unsigned.

Sign-off row 13 now cites this and states plainly what is *still* visual: whether it **reads** as compact, and
the legend text. Row 8 (reticle brackets) stays "no pixel-level assertion exists", because that is true — the
bracket constants live inline in `render()`, which needs a DOM.

Station homed, ready, MANUAL/HOLD, synthetic vision running. **§24 visual items 16 and the v3 architecture
ledger's 30 items both remain at 0 accepted by a named person.**

## 2026-09-04, 22:5x — round 32: the reticle's anchor is proven to follow the optical axis, and the coincidence that would have hidden a wrong build is on the record

`web/webd/tests/test_reticle_anchor.py` (4 tests) settles the one part of §24 row 8 that is arithmetic: §7 puts
the centre reticle on the **optical axis**, which is the principal point — *not* the middle of the image. Those
two coincide on this station (calibration `cx 960, cy 540` of 1920×1080), which is exactly the trap: a build that
hard-coded the geometric centre would be indistinguishable from a correct one **on this hardware**. So the
mapping is tested with a deliberately off-centre principal point (1010, 505), where "follows cx/cy" and "assume
0.5" disagree.

**Falsified as designed:** replacing the mapping with `{ u: 0.5, v: 0.5 }` fails the off-centre test and nothing
else — the only test in the suite that could see that bug saw it. Restored from a plain `cp` backup.
**447 pytest** pass (443 + 4); no C++ touched, **57 CTest** stand.

Checked before testing the plumbing, per probe-first habit: **live `/api/state` carries
`camera_intrinsics: {valid: true, fx 1389, fy 1467, cx 960, cy 540, 1920x1080}`**, so the page's anchor comes
from the real calibration and the centred fallback is *not* engaged on this station. That matters for how much
the fallback test is worth: it asserts the uncalibrated path emits its own on-screen note (a code-shape claim),
but on this hardware that branch never runs, and the sign-off row says so instead of implying coverage that
does not exist.

Row 8 is now honest in both directions: the anchor is proven; **bracket separation and open centre remain
visual**, because the bracket geometry is inline in `render()` and needs a DOM. Extracting it into a pure
function would make that testable too, but it changes drawing code for a test's benefit — recorded as an option,
not taken unbidden.

Station homed, ready, MANUAL/HOLD, synthetic vision running. **§24 (16 items) and the v3 ledger (30 items): 0
accepted by a named person.**

## 2026-09-04, 23:2x — round 33: two things I had been asserting without measuring — the loop is at ~197.9 Hz with a consistent overrun, and **the camera captures**

Evidence: `docs/evidence/loop_timing_and_capture_2026-09-04_r33.md`.

**I checked what the number means before quoting it**, because rounds 23–25 were spent explaining a metric from
summaries instead of from the expression that computes it. `snap.control_cycle_us = period_ns / 1000`, where
`period_ns` is the interval the caller supplies to `step()`, and line 369 detects `period_ns > deadline_ns_` —
which is only possible if that interval is **measured**. So the field is the loop's real wake-to-wake interval.
Sampled 12× over ~22 s, homed and idle: **min 5052, p50 5054, max 5054 µs → ~197.9 Hz against a nominal 200,
with ~54 µs of consistent overrun past the 5 ms deadline.** Stated with its limits: twelve peeks at the
*latest* value is not a distribution, and the narrow spread argues against large jitter without excluding it.

**The sharper finding: the overrun is computed and then thrown away.** `overrun_us` exists at line 370, and
`grep -c overruns` is **0** in `web_server.hpp` and **0** in `protocol.py`. A station that misses its deadline on
every cycle for a whole day shows nothing on screen, and `docs/evidence/` held **zero** timing artifacts of any
kind before this file. So the objective's "200 Hz must stay within measured limits" is currently unseeable, not
merely unproven. Publishing the overrun counter plus a p95/max over a window is the next concrete step.

**A blocker I had asserted on the wrong grounds, for many rounds.** I recorded that camera-geometry
commissioning was blocked because `visiond`'s real path needs an image-config JSON and an IMX500 `.rpk`, and
neither exists. That is an argument about *assets*; **nobody had checked the hardware.** `picamera2` imports,
`rpicam-still` is installed, four `/dev/media*` nodes exist, and a 40 s probe **captured a 1920×1080 frame
(188,437 bytes) from the IMX500**. The missing detector package constrains automated *detection*, not *capture*.
Real-frame deg-per-pixel and principal-point work is therefore actionable now — the probe frame's column-edge
profile (median 824, peak 16,500 near column 826) says there is structure to phase-correlate. My first
on-the-spot threshold ("edge energy > 2.0 ⇒ not textured") said the opposite and was wrong: the meaningful
statistic is the peak-to-median ratio, which I invented after seeing it. Recorded so the bad verdict isn't
mistaken for the finding.

Two limits that survive the unblock: **absolute boresight still needs a distant feature at a surveyed bearing**
— physical, the operator's — and **this model cannot look at images** (`read_image` refuses: no image input), so
every measurement off real frames must be numeric, never "I saw the target". That is a further reason §24 stays
a human judgement.

No code changed; **447 pytest / 57 CTest** stand. Station homed, ready, MANUAL/HOLD, synthetic vision still
running (`rpicam-still` released the sensor cleanly). **§24 (16) and the v3 ledger (30): 0 accepted by a named
person.**

## 2026-09-04, 23:5x — round 34: my own record had overstated camera commissioning — the principal point was **never measured**

Started by asking how the theodolite probe gets frames, and the answers compounded:

* **`tools/probe_theodolite.py` already exists** and is careful: steps the axis, waits for settle, correlates a
  **single high-contrast strip** (a row for yaw, a column for pitch) rather than the whole frame, walks forward
  **and back** so backlash shows, and **verifies the correlation sign against a known synthetic shift before
  reporting anything** — a calibration with the wrong sign looks identical to a right one until the turret drives
  the wrong way.
* **webd's video is produced from the real IMX500 inside the webd process** (`web/webd/video.py`: picamera2 /
  libcamera, lazy, low-priority, off by default). So the probe's frames were **real optics**, not the synthetic
  pattern. Good news, and the reason to ask.
* **`calibration/camera_intrinsics.yaml` is genuinely measured — for fx/fy only.** Header: *"MEASURED on the
  station, 2026-09-04, by encoder-as-theodolite"*, and it records the correction of an earlier wrong reading (the
  "pixels are anamorphic" conclusion was whole-frame correlation averaging different elevations; strip walks
  disagreed by up to 45%). **Line 27: *"cx/cy are the GEOMETRIC CENTRE BY CONVENTION, not a measurement"***,
  because on a rotating platform the principal point and camera-to-axis boresight enter as the *same* constant
  pixel offset.

**So my record has been wrong.** Rounds have reported "FOV + principal point measured" and even the file's own
header invites it. The accurate state: **plate scale and effective FOV measured (fx 1389 / fy 1467 → 24.24 and
25.60 px/deg; hfov 69.3002°, vfov 40.4171°); principal point assumed at the centre; boresight not commissioned.**
`cx 960, cy 540` are round numbers because they are a convention, not because they are tidy measurements.

**Why that matters is the objective's own premise**: without the principal point and boresight, **centring
tolerance and frame-exit margin are not computable** — which is precisely why item (c) was listed first. So (c)
is **open**, and no amount of HUD work closes it. Round 32's reticle-anchor test is unaffected but must be read
correctly: it proves the HUD *follows* `cx/cy`, and right now `cx/cy` are a convention. The test would pass
identically on a mis-centred lens with a centred assumption.

Separation path, for when someone is standing at the station: a calibration board at **wide** spans (the tan()
curvature the probe says is only a few percent over ±16° is the only thing that splits the two constants), or a
**surveyed distant reference** at a known bearing. `make_charuco_board.py` and `calibrate_camera_intrinsics.py`
are already in `tools/`; the missing ingredient is physical.

Sign-off package's camera bullet rewritten to say all of this — including the sentence *"an assumption wearing a
MEASURED header"*. Docs only: **447 pytest / 57 CTest** stand, station homed, ready, MANUAL/HOLD, video stream
left off by default (it opens on demand).

## 2026-09-05, 00:1x — round 35: the loop's deadline state is on the screen, and round 33's recommended fix would have been a false alarm

**Where round 33 went half wrong.** It reported the overrun as "computed and then dropped" and recommended
publishing it. Reading `control_loop.cpp:360-376` first shows why that is the wrong number: the raw overrun past
the period is forgiven **by design** — counting every over-period cycle as a miss once made a **198 Hz loop Hold
all axes within five cycles** (the P0 no-motion root cause), and a fix that reset only on strictly on-time cycles
latched a permanent Hold on a host whose period is always a hair long (P0f). A miss is a cycle past
`deadline_max_us` (**2000 µs of grace**), and only **5** consecutive ones matter. Publishing "OVERRUN: ~54 µs,
every cycle" would have manufactured an alarm the architecture deliberately does not raise — the same class of
error as reporting the 198 Hz loop as broken.

**What shipped instead:** the decision-relevant triple — consecutive misses, the grace in force, the limit —
read at snapshot time from a counter the loop already maintains and two config constants, so **nothing was added
to the 200 Hz path**. Verified live after restart: `/api/state` → `misses 0, grace_us 2000, limit 5,
cycle_us 5054`, page contains the row, and the panel's own builder renders
**`LOOP DEADLINE  0/5  (+2000us grace)`** beside `RATE CEILING 10.0 DEG/S (AUTH 100%)`, `GEOMETRY AGE 11.4 H`,
`IMU ABSENT`. Absent state still renders UNKNOWN, never a reassuring 0. A dated addendum now corrects the
evidence file's own recommendation rather than leaving it standing.

**I broke my own restart rule and the page size caught it.** I restarted controld and the vision source but not
**webd**, after editing `hud.py` and `protocol.py` — which my record explicitly says requires a webd restart.
First verification returned `misses None` and a page of **78,238 bytes, byte-identical to round 29**. That
identical size is the tell worth remembering: a changed HUD that is somehow exactly the same size means the old
process is still serving, not that the change did nothing. After a real restart (one stray worker needed
`kill -9`): 78,855 bytes, row present, fields populated. `errno98=0`, no fatal in the daemon log.

Cycle times in this sample varied 5053–5067 µs, slightly wider than round 33's 5052–5054 — still the same
~198 Hz, still nowhere near the 2 ms grace. **57 CTest** on the new build (grepped clean), **449 pytest**
(447 + 2 guards), `node --check` OK — the paren discipline round 29 had to learn is now habit. Vision restarted
and checked: `vision_track_sets 418`, `supervisory READY`, station MANUAL/HOLD, homed.

## 2026-09-05, 00:5x — round 36: measured jerk two independent ways — C5b is real, **max jerk is unquotable, and round 28's "10 deg/s cap" is contradicted**

`docs/evidence/jerk_two_paths_2026-09-05_r36.md`. One scenario (s3, 25° over 1.60 s, envelope-legal, vision
source stopped), recorded twice at once: the probe's HTTP polling path, and `tools/telemetry_stream.py --jsonl`
against the daemon's socket — **484 frames / 32.0 s = 15.2 Hz publication**, field `ts_ns`, 128 fields per
frame. The daemon publishes `q_ref_rate_yaw_rad_s` and `v_yaw_rad_s`, so this is the controller's own numbers,
not differences of quantised positions.

**C5b's failure survives instrument change: jerk p95 524.9 (daemon) vs 542 (probe), ~3% apart, across ~66 ms and
~20 ms grids and two transports, against `max_jerk_deg_s3: 300`.** That is a real ~1.8× smoothness violation.

**"Max jerk" is not a number and I have been printing it.** The same motion gave 576 (round 27), 1892 (round 36
probe) and 9755 (round 36 daemon). My hedge was "a lower bound", which was the wrong hedge — it implied the
figure had a direction. It has none. p50 and p95 do; max is the differentiation of a grid the transient doesn't
fit. The sign-off package now says explicitly not to quote it.

**The bigger correction: round 28's causal claim is contradicted by the very field it was about.** This run's
**reference rate peaked at 18.56 deg/s and actual velocity at 20.43 deg/s**. Round 28 concluded — and commit
`b3dd45b` states — that AUTO_TRACK's reference "can never exceed 10 deg/s" because the envelope cap is the
hard-coded hold speed. A measured 18.56 cannot come from a 10 deg/s bound, and the probe reads
`q_ref_rate_yaw_rad_s` at `probe_track_loop.py:804`: same source, same field. So **the explanation is withdrawn
pending reconciliation, and the advice attached to it — raise the envelope cap — must not be acted on**: that
would alter safety-envelope behaviour possibly for nothing. What survives from round 28 is narrower: the two
constants really do disagree by construction. Whether that disagreement causes C2/C3 is now an open question,
with three hypotheses ranked in the evidence file (the envelope cap bounds a different quantity than the
reference generator; a later stage re-accelerates past it; or the earlier 10.0 was a different statistic or run
state). Next round: read the generator→limiter chain end to end before saying anything else about it.

No code changed. **449 pytest / 57 CTest** stand. Probe verdicts unchanged (C1/C5a/C6 pass, C2/C3/C4/C5b fail).
Station returned to MANUAL/HOLD, homed, vision restarted and verified (`vision_track_sets 8099`, READY).

## 2026-09-05, 01:3x — round 37: the ceiling story is dead, and it had infected my own HUD row

Read the generator→limiter chain as promised. Three findings, all of which I got wrong before reading:

1. **`env_.set_v_max(cap)` is in `apply_payload_derate()` (line 2200), not on any mode path.** Round 28 grepped
   `sed -n '2117,2300p'` believing that range was `build_mode_intent`; it ran past the end of that function into
   the next one. A range-limited grep read as a scoped conclusion.
2. **`SafetyEnvelope::v_max` is a *fallback*, not a limit.** `safety_envelope.hpp:70` calls it "fallback speed
   when no limits set" and line 120 reads `if (!lim.valid) return p_.v_max_rad_s;` — it is reached only when
   travel limits are unknown. This station is homed (`soft_limits_valid: true`), so the value never binds: speed
   comes from the braking model against real travel. **The code comment I quoted as authority — "it bounds the
   tracking reference, §15" — is describing the fallback case, and I read a comment as a mechanism.**
3. So round 36's measured 18.56 deg/s reference and 20.43 deg/s actual were never in conflict with anything.
   **AUTO_TRACK is not capped at a third of its configured speed. That was wrong for nine rounds.**

**The part that matters more: the wrong idea had reached the operator's screen.** Round 29 added a DIAG row
labelled `RATE CEILING 10.0 DEG/S`, and for all that time the panel showed a fallback constant as the limit in
force while the axis was tracking at twice it. The number was accurate; the *label* was false, which is worse
than no number — it turns a display into a safety statement the machine contradicts. Row renamed and now reads,
live and verified: **`ENVELOPE V-MAX  10.0 DEG/S  (not in force)  AUTH 100%`**, flipping to
`FALLBACK IN FORCE: travel limits unknown` when soft limits are gone. Old label confirmed absent from the served
page (count 0; page 79,641 bytes). The guard test was **inverted deliberately** — it once asserted the false
label existed, and now asserts it is gone and that the row states whether the value binds. The comment in the
test says why, so nobody "restores" it.

**The cost, stated plainly: C2/C3 and the C6 rate picture no longer have an explanation.** The dart still
under-reaches (reference ~10 deg/s average against a 25°/1.6 s demand), but the mechanism is unknown, and the
sign-off package says the fix is not to raise any envelope cap. Two constants that disagree by construction
(hard-coded 10 deg/s hold speed versus `track_speed_deg_s` defaulting to 30) remain a real code smell — they
just aren't shown to cause anything.

First patch attempt aborted cleanly on its own assert (anchor count 0): I had reconstructed the round-29 row
from memory instead of reading it, and the comment block I remembered wasn't in the file. No partial damage.
**449 pytest / 57 CTest** stand (no C++ changed), `node --check` OK. Station homed, MANUAL/HOLD, READY, vision
running, webd restarted cleanly (`errno98=0`).

## 2026-09-05, 02:1x — round 38: the 10 deg/s plateau is real and it accounts for C3 and C2 — from a file, not from new motion

`docs/evidence/ref_rate_plateau_2026-09-05_r38.md`, written entirely from round 36's recording (484 frames at
15.2 Hz, 128 fields — it carries `q_ref_accel_yaw_rad_s2`, `q_ref_rate_yaw_rad_s`, `target_az_world_rad`,
`target_az_rate_world_rad_s`, so the dart can be rebuilt without differentiating twice and without driving the
station again).

**Through the entire dart the reference rate sits on exactly 10.00 deg/s with zero acceleration, frame after
frame** — a clamp, not a response (a response varies and accelerates). The target demanded up to 17.4 deg/s.
The arithmetic then closes the criterion: **10.00 deg/s covers 16° in a 1.60 s dart against a 25° target = 9°
deficit**, measured C3 lead **−11.6°**, and the residual is the estimator's own lag (−1.4°) plus prediction
(−0.6°) from the same run. **C3 is the plateau plus the known lag.** C2 needs no separate mechanism either: over
the window the target swept 24.92° and the reference 23.82°, so the debt was repaid during the hold at 6–8 deg/s
while oscillating ±8°/s — which is precisely what C2 scored as **2.85 s to recover inside tolerance**.

Round 28's *number* was right all along. Round 37's *retraction* also stands: the mechanism is still not
identified, and the file says plainly that **no limit should be raised on the strength of this** — a clamp that
costs C2 and C3 may still be a deliberate design limit, and I have been wrong about the "why" here before.

**An error worth keeping in the record:** my first pass computed `gap = q_ref_yaw_rad − target_az_world_rad` and
reported gaps near 180°. That is subtracting a **joint angle** (2.60 rad ≈ 149°) from a **world azimuth** — the
~179° offset is mount geometry, not a tracking error. The near-constant gap should have warned me instantly. Any
gap metric must live in one space via the controller's own LOS conversion.

Side result: deriving jerk from the **published accel** (one differentiation) gives p50 161.6 / **p95 300.0** —
landing exactly on the configured `max_jerk_deg_s3: 300`, which is what an honoured jerk limit looks like on this
grid (max again meaningless: 14600). So C5b's verdict depends on which signal is differentiated, and no single
jerk number should be quoted across signals.

Mechanism hunt, for the next round, started but **not concluded**: every consumer of `hold_speed_rad_s` is at
`control_loop.cpp` lines 38 (an `ep.v_max_rad_s`), 103 and 822 (`enter_position_mode_all`), 928 (`lim[i] =
min(hold_speed, …)`), 1714, 2095 (`l.hold_v_max_rad_s = cap`), 2192-2193 (the derate fallback round 37 covered).
None is yet traced to the AUTO_TRACK reference — saying more now would repeat the round-28 mistake, so I stop
here. Also still unexplained: telemetry showed `AUTH 100%` while a 10 deg/s clamp was in force, so the published
authority scale is not the thing doing it.

Station untouched by this round (analysis of a file): homed, MANUAL/HOLD, READY, vision running. No code
changed; **449 pytest / 57 CTest** stand.

## 2026-09-05, 02:5x — round 39: three mechanisms for the 10 deg/s plateau, all three rejected by measurement

Detail in the dated addendum to `docs/evidence/ref_rate_plateau_2026-09-05_r38.md`. The chain is fully read now
(`q_ref_rate_yaw_rad_s` = `ref_lim_[i].v_rad_s`, limited by `min(tracking_ref_.v_max_rad_s, max_speed_at(...))`),
and **confidence looked like the answer** — `reference_manager.hpp:132` scales the configured 30 deg/s by raw
confidence. The sentence was written before the check; the check said **`target_confidence` is 1 across all 34
plateau frames**, which implies 30, not 10. Soft-limit braking also died on the data: at a plateau frame the
reference sat **157.87° clear of any yaw soft limit**, so the braking model permits far more than 10 deg/s. The
envelope fallback was already excluded in round 37, and this round's fuller read of `max_speed_at` **confirms**
round 37 rather than overturning it (no cruise ceiling when limits are valid).

Three rounds now — 28, 37, 39 — have each produced a confident mechanism from a partial read, two retracted and
one killed by arithmetic. So this round stops at triage: the shortlist is `mode_proposal_.v_max_rad_s`
(line 426), the 300 ms post-handover authority ramp (lines 609-615, where `30 × 1/3 = 10.00` fits the numbers but
telemetry reported `intent_velocity_scale 1.00` throughout — so either that field is not the applied factor, or
the factor is applied somewhere it never reaches telemetry), and the runtime value of `track_v_max_rad_s` (no
`turret.yaml` key exists, so it takes a code default — to be confirmed from the loaded daemon, not from the
defaulting call site). Next round measures the daemon's loaded speed and authority fraction instead of inferring
them from source.

The plateau and its cost stay established; the cause does not, and the operator-facing documents say so. No code
changed, station untouched: homed, MANUAL/HOLD, READY, vision running. **449 pytest / 57 CTest** stand.

## 2026-09-05, 03:3x — round 40: the plateau is closed — a *hold* speed is the ceiling on tracking, on purpose

Thirteen rounds after round 28 first noticed "10.0 deg/s", the chain is closed with every link measured: controld
logs `track=30.0` and payload profile `20.1/20.1`; the recording shows `track_state=tracking`,
`confidence_band=HIGH`, `selection_ambiguous=0`, confidence 1.0 across all 34 plateau frames (so the search
branch and the confidence derate are both excluded, as round 39 said); and `control_loop.cpp:427` caps the
AUTO_TRACK proposal at `hold_speed_effective()` = `min(10.0, 20.1, 20.1)` = **10 deg/s**, before the confidence
derate, with a comment saying the ordering is deliberate.

**Round 28's substance was right. Round 37's retraction was right too. Both were incomplete** — 28 had the
effect and the wrong line, 37 correctly killed the envelope story and threw the conclusion out with it. The
mistake pattern is the same one every time: concluding from the expression nearest to hand.

**What the operator owns now.** A 25° dart in 1.60 s needs ~15.6 deg/s average; the effective tracking ceiling
is 10 deg/s, applied to the reference *before* prediction or tracker gains can matter. **C2 and C3 cannot pass
at that dart size at any tuning.** The 10 deg/s is hard-coded in `main.cpp:133` with **no key in
`turret.yaml`** — only the unbound `track_speed_deg_s` (30) is configurable. Raising it is a statement about how
fast this station may swing with a payload, which is not mine to make. Note also that the comment above line 427
says "the payload profile v_max caps station motion" while the binding term is the *hold speed* (10), not the
profile (20.1) — the comment describes intent, the constant decides otherwise.

Left undone, deliberately, because context ran out first: publish `hold_speed_effective()` as the effective
station ceiling. The panel's `ENVELOPE V-MAX (not in force)` row is accurate and still doesn't answer "why won't
it keep up" — round 37 fixed the label; the right number is still missing.

No code changed. **449 pytest / 57 CTest** stand. Station homed, MANUAL/HOLD, READY, vision running.

## 2026-09-05, 04:0x — round 41: the binding ceiling is on the panel — `SPEED CEILING 10.0 DEG/S (min of hold + payload profile)`

The number round 40 proved is the reason C2/C3 fail is now published (`effective_speed_ceiling_deg_s`, read at
snapshot time from `hold_speed_effective()`, so nothing was added to the 200 Hz path), declared in `protocol.py`,
and printed in DIAG. Verified against the running station, not the diff: `/api/state` returns
`effective_speed_ceiling_deg_s: 10`; the page (80,289 bytes) carries the row; the page's own builder renders
**`SPEED CEILING 10.0 DEG/S (min of hold + payload profile)`** above `ENVELOPE V-MAX 10.0 DEG/S (not in force)
AUTH 100%`, beside `LOOP DEADLINE 0/5 (+2000us grace)`, `GEOMETRY AGE 12.1 H`, `IMU ABSENT`. Vision restarted and
checked (`vision_track_sets 3150`), `errno98=0`, no fatal in the daemon log, homed/READY/MANUAL/HOLD.

Two honest notes. **On this station the two ceiling numbers coincide at 10**, because the envelope fallback is
itself set from the hold speed — so the row pair does not *demonstrate* a difference here, even though the guard
test says the distinction is the point; the distinction is real in code and would show if the payload profile
bound first (it would give 20.1). Second: **one pytest failure appeared in the run immediately after the build
and did not recur in two subsequent runs (451 passed twice).** I did not identify it, and "flaky" is a label, not
an explanation — most likely a test that binds a port or reads the live daemon contending with the restart I had
just done, which is the kind of thing that should be pinned down before it hides a real failure.

The operator's decision from round 40 is unchanged and now has a number attached to it on the screen: raise the
hold/payload ceiling, size the acceptance dart to 10 deg/s, or change line 427 to use the payload profile
(20.1 deg/s). **449 → 451 pytest** with the two new guards; **57 CTest** on the new build.

## 2026-09-05, 04:4x — round 42: chased the "flaky" label, did not catch it, and caught a false claim of mine instead

**The flake stays unidentified.** Ran the full suite under four concurrent CPU hogs for its duration:
**451 passed.** So the one failure (immediately after `cmake --build -j4` + `ctest` in the same invocation) did
not reproduce under load. Facts kept on the table: one failure in six runs, always in the run sharing an
invocation with a build; `web/webd/tests` contains 24 `sleep(` calls, so the wall-clock-sensitive class exists;
nothing in the suite binds a fixed port. I am **not** calling it fixed, and I am not calling it harmless — an
unidentified intermittent failure is the kind that later disguises a real one.

**What the chase turned up instead: my own wording was false in two places.** I had written — in the operator's
sign-off package — that `test_section_20_ledger.py` reads fields "from the live payload rather than a fixture".
It does not: it instantiates **`FakeControld`** on its own unix socket with `WebConfig(port=0)` and drives a webd
it spawns itself, which is good engineering and exactly why the suite is reproducible — but it is **not the
station**, and I had used "live" to mean "the real code path" where an operator would read "the real hardware".
Both sentences now say what is true, including where the genuine station-level checks live (`docs/evidence/`,
curl-verified). Same failure class as round 34's principal point: an accurate number with an overstated provenance.

**One genuinely useful thing came out of the greps:** no test in the suite references port 8080 or the station's
socket, so "pytest is green" means something on a cold machine with no hardware — which is what the objective's
green-at-every-step clause has always needed it to mean. Docs only this round; **451 pytest / 57 CTest** stand;
station homed, READY, MANUAL/HOLD, vision running.

### Addendum to round 42, same day — I wrote "both sentences fixed" when only one was

The patch printed `count 0` for the second phrase and I still let the commit message and the entry above say both
were corrected. The sentence was at line 83 all along; my anchor straddled a line break, which is the same trap I
hit in round 29 (reconstructing text from memory instead of reading it) — except this time I also **reported the
outcome wrongly**, which is worse than a silent partial edit because the record now reads as verified when a
machine had told me otherwise in plain text. Fixed in the follow-up commit. The lesson is not "read the file"
again, it is: **when a patch reports zero matches, the sentence about that patch in the commit message is already
wrong, and it has to be written after the tool output, not before.**

## 2026-09-05, 05:2x — round 43: the safety ceiling that blocked acceptance now has a config key, and the default changes nothing

Round 40 left the operator with a decision that required editing C++ and rebuilding — a safety ceiling that
looks like a code change stops looking like a decision. `tracking.hold_speed_deg_s` is now parsed with
`opt_double(..., 10.0, warn)`, **defaulting to exactly the constant it replaces**, and `main.cpp:133` reads the
config instead of hard-coding 10.0. The `turret.yaml` entry is written but **left commented out**, so nothing
moves on this station until someone chooses.

Proved, not assumed, at both ends:
- **Parser, permanent test** (`control/tests/test_config.cpp`): absent key ⇒ 10.0 exactly; explicit `12.5` ⇒ 12.5.
  The default being exact is the point — adding the knob must not quietly change a station that never mentions it.
- **Live station, unchanged behaviour**: the new binary boots with
  `warning config: tracking.hold_speed_deg_s: not specified; using conservative default` (the parser path is
  demonstrably the one running) and the panel still reads **`SPEED CEILING 10.0 DEG/S (min of hold + payload
  profile)`**, with `/api/state` = 10. Daemon ready, no fatal. **57 CTest / 451 pytest** pass.

What is **not** yet exercised: setting the key to a real alternative on hardware and watching the reference stop
plateauing there. The unit test covers yaml→struct and the boot above covers default→panel; the middle link
(main.cpp → envelope → reference plateau) is one line and one measurement away, and belongs to the operator's
decision anyway. Recording the gap rather than implying the whole path was run.

The operator now holds three cheap options, all reversible in config with the effect visible on the panel:
raise `hold_speed_deg_s`, size the acceptance dart to what 10 deg/s can follow, or accept C2/C3 failing as a
documented consequence of the safety ceiling. Station homed, READY, MANUAL/HOLD, synthetic source restarted and
verified (track sets > 0).

## 2026-09-05, 05:5x — round 44: C4 does not measure ringing, and I have been reporting its failure as if it did

No motion this round. `docs/evidence/c4_measurement_validity_2026-09-05_r44.md`, from round 36's recording plus
the probe's source.

Over the 91 TRACKING frames: the **target's reported rate crosses zero 21 times; the reference's, 10**, and the
reference rate maxes at 10.00 (round 38's ceiling, reconfirmed). C4 counts sign changes in the *reference* and I
have reported its failure as evidence of oscillation in two rounds of summaries. **The signal being followed
crosses zero twice as often as the signal doing the following** — the reference is the smoother of the two, which
is the opposite of what an oscillating loop looks like.

Two reasons, neither of them ringing: the fixture's target genuinely holds still (the probe has no jitter term —
grep for `jitter|noise|random|uniform|gauss|drift` returns only comments about *feedback* noise — and `--hold-s`
means "how long the target holds still"), and **the camera is mounted on the axis that is still chasing**, so a
fixed world target's bearing moves while the shortfall from round 38 is being repaid. On top of that sits a
measured estimator rate noise floor near **1 deg/s p50 while the target is provably stationary**.

**Consequence: "C4 FAIL" should have read "C4 inconclusive"**, and the sign-off package now says so in place. Not
established: that the loop does *not* ring. Only that this measurement cannot tell — which matters because
objective (b) requires smoothness to be *measurable*, and one of its four criteria currently isn't. A corrected
C4 is sketched in the evidence file — scored in the settled window C2 already defines, on the aim **error** in
one space (round 38's joint-vs-world warning), with a deadband derived from the measured noise floor rather than
a constant in source, and with **overshoot amplitude** reported next to crossings — deliberately left
unimplemented, because redefining an acceptance criterion mid-acceptance is the operator's call, and the old
metric should keep printing beside it until they choose.

**Method note, applied deliberately this round:** my first two replacement anchors missed (bold markers I had
imagined, a `·` separator I hadn't). I read the exact bytes with `cat -A`, then replaced with a verified unique
anchor and confirmed `count: 1 / REPLACED` **before** writing any of it up — the round-42 lesson in practice.

Station untouched: homed, MANUAL/HOLD, READY, ceiling 10.0 on the panel, synthetic source running. Docs only;
**451 pytest / 57 CTest** stand.

## 2026-09-05, 06:2x — round 45: C5b validated by its noise floor — the opposite result to C4, from the same kind of test

Same method as round 44, applied to the other smoothness criterion, on the same recording. The question was
whether C5b's jerk figure is a differencing artifact the way C4's crossing count turned out to be.

**The test:** find frames where jerk is **zero by construction** — the reference pinned at 10.00 deg/s with
published accel under 0.5 — and see what the estimator reports there. That is the instrument's own noise floor.
33 such frames:

    jerk estimated by differencing the published rate   ->  noise floor p95 = 4.2 /s³ (p50 0.0, max 4.3)
    jerk estimated by differencing the published accel  ->  noise floor p95 = 1.1 /s³ (max 28.1)

Against that, the measured violation is **p95 525 (rate-derived) / 914 (accel-derived)** versus a configured
300. **Two orders of magnitude above the floor.** C5b is not an artifact — it is a real ~1.75x smoothness
violation, and unlike C4 it survives the only test that could have dismissed it.

**Robust to how it is derived**, which is worth more than agreement between two samplers: single-differencing the
limiter's own acceleration gives an even *larger* number (p95 914) than double-differencing the rate (525), so no
choice of derivation rescues it — the two paths disagree in size and agree in direction and magnitude class. And
for what it's worth the estimator behaves correctly at rest: jerk is exactly 0 across the plateau, which is what
a constant-rate reference should show.

**The method that produced both results in two rounds:** before believing a derived quantity, measure the
derivation on a segment where the truth is known. It invalidated C4 and validated C5b — it does not care which
way the answer falls, which is the only reason either verdict is worth having. Sign-off package updated in place
(noise floor quoted, "max" still barred). Anchor was checked (`count: 1 / REPLACED`) before writing any of it up.

Station untouched: homed, MANUAL/HOLD, READY, ceiling 10.0 on the panel, synthetic source running. Docs only;
**451 pytest / 57 CTest** stand.

## 2026-09-05, 06:5x — round 46: the head-aim numbers really are about the head (checked, after 40+ rounds of quoting them)

Objective (a) is the tolerance I have quoted approvingly since round 12 — "aim error p50 0.002/p95 0.003 box
heights, bar 1/3" — and it had never been checked that the measurement uses the **head** anchor rather than the
box centre. It could have been quietly scoring the centre for every one of those rounds.

Checked at both ends of the chain, from the recording and the probe source:

* **Daemon:** across all **91 TRACKING frames**, `target_aim_is_head = True`, `target_aim_valid = True`,
  `aim_point_valid = True`. The published aim point is the head anchor while tracking, not a fallback.
* **Probe:** the aim error is `hypot(aim_u * FW - CX, aim_v * FH - CY)` — distance from frame centre to the
  **aim point**, in pixels scaled by frame size (lines 633 and 772, for the steady and dart scenarios) — and it
  **prints which source it used**: `"head aim"` unless the aim point is missing, in which case it says
  `"anchor fallback"` (line 678). It would have said fallback. It didn't.
* **Criterion text:** `S1: steady image error <= 1/3 of the declared box height (the operator's tolerance)` —
  implemented as the objective words it, not paraphrased.

**The honest limit stays where it has been:** on this station the head anchor is the position the *fixture
declared* to be the head — there is no detector asset, so a real detector's head anchor has never been exercised.
So the number measures "the reticle holds on the declared head anchor to 0.24 box heights while holding", which
is a real closed-loop result and still plumbing evidence for §110, not a signable acceptance.

Two of the four measurement instruments have now been tested against known truth (C4 invalidated, C5b validated)
and the head-aim metric has been traced end to end. Remaining unchecked instruments: C1's containment arithmetic
and C6's ceiling comparison.

Station untouched: homed, MANUAL/HOLD, READY, ceiling 10.0 on the panel, synthetic source running. Docs only;
**451 pytest / 57 CTest** stand.

## 2026-09-05, 07:2x — round 47: the last two untested instruments both fail the check, and one of them explains round 27

Third pass of "check the instrument, not the number" (C4 invalidated · C5b validated · head-aim traced). Probe
source and the existing recording only, no motion. `docs/evidence/c1_c6_measurement_validity_2026-09-05_r47.md`.

**C1 tests a point, not the target.** Every containment predicate is on the anchor — `outside = not (0.0 <= un <=
1.0 and 0.0 <= vn <= 1.0)` at line 241, `in_frame = (0.0 <= u <= FW) and (0.0 <= vv <= FH)` at line 636, consumed
at 675 and 839. The declared box is `BOX_H_NORM = 0.35`, about **378 px of a 1080-px frame**, so the anchor may
sit on the edge with ~189 px of target outside and C1 says PASS. The objective says *"so the target does not leave
the frame"*; what is scored is *"so the anchor does not."* C1's PASS is an upper bound, not containment.

**C6 certifies against a ceiling that is not the binding one.** Line 913: `ceiling = 30.0 * (min(scales) ...)`,
i.e. the configured tracking speed (which `turret.yaml` doesn't even set — 30.0 is the loader default) scaled by
the payload profile, ≈20.1 deg/s. Round 40 proved the applied ceiling is **10**; round 43 put it on the panel.
**This is round 27's puzzle resolved**: that evidence file is named `...envelope_legal...` and then failed C2/C3
— legal against 20.1, impossible against 10. C6 should read controld's published `effective_speed_ceiling_deg_s`
instead of reconstructing a constant in Python.

**Both defects point the optimistic way.** That is the direction worth being paranoid about, and it is the third
round running that checking an instrument changed what I am allowed to say rather than what the station does.
Current honest state of the dart: C1 PASS-but-point-only · C2 FAIL · C3 FAIL · C4 inconclusive · C5a PASS · C5b
FAIL validated · C6 PASS-against-wrong-ceiling. Fixing C1/C6 is a probe change followed by one more dart run —
and the criteria themselves are the operator's, so I have written the corrections down rather than quietly
rescoring acceptance. The sign-off document got an append-only addendum saying its C1/C6 rows read stronger than
they are.

Station untouched: homed, MANUAL/HOLD, READY, `SPEED CEILING 10.0 DEG/S`, synthetic source running. Docs only;
**451 pytest / 57 CTest** stand.

## 2026-09-05, 08:0x — round 48: C1 now asks whether the box left the frame; the dart has NOT been re-run

`tools/probe_track_loop.py` gains a pure helper `box_extends_past_frame(v_px, box_h_px, frame_h_px)`, both the S2
and S3 sampling loops record `box_out`, both verdict blocks compute `c1_box`, and both print it **beside** the old
point metric, never instead of it — silently rescoring an acceptance criterion in a test tool is as bad as
measuring the wrong thing. `tools/tests/test_probe_box_containment.py` covers the centred case, the case the old
predicate passed (anchor a pixel inside the edge, ~189 px of declared box outside), the exact edge in both
directions, and a zero-height box that must degrade to the point test rather than always fail. **456 pytest / 57
CTest**; station left untouched (MANUAL/HOLD, READY, ceiling 10).

**The horizontal extent is deliberately not checked.** The fixture declares a box *height* and builds its box
from `BOX_H_NORM * 0.5` (line 258); there is no declared width anywhere in the tool, so an aspect ratio would
have to be invented, and a measurement may only contain numbers somebody measured. The printed label says
"declared box edges (height only - no width is declared)" so nobody reads it as full-box containment.

**Two real bugs came out of verifying state instead of trusting my own script.** First, the row patch replaced
*both* sampling sites (count 2, not 1), so the follow-up patch aimed at "the S3 site" correctly found nothing and
the script died mid-way — leaving the S2 print referencing `c1_box` **before any assignment existed**. That is a
runtime NameError that `py_compile` passes happily; I found it by grepping assignments against uses and reading
line numbers, not by reading the diff. Second, my own new test had its boundary case direction-inverted — I moved
the anchor *inward* and expected an overflow — so the test failed for the right reason and I fixed the test, not
the metric.

**Outstanding, deliberately not touched:** C6 still compares the dart against `30.0 * payload scale` (~20.1) at
line 913 instead of controld's published `effective_speed_ceiling_deg_s` (10). That is the next edit. And because
**no motion ran this round**, the recorded C1 verdicts are exactly as round 47 left them: the corrected metric has
never yet been executed against real telemetry, which is stated here rather than implied fixed.

## 2026-09-05, 08:4x — round 49: C6 now judges the dart against controld's ceiling, and says out loud which one it used

Line 913 read `ceiling = 30.0 * (min(scales) if scales else 1.0)` — the configured tracking speed (which
`turret.yaml` never sets; 30.0 is the loader default) times the payload derate, about **20.1 deg/s**. The ceiling
controld applies is **10** (round 40), published since round 41. That gap is the whole of round 27's confusion:
the dart was certified "envelope legal" against a limit that was never in force, then failed C2/C3.

Now `binding_ceiling_deg_s(published, configured, scales)` returns the number **and its provenance**: the
published `effective_speed_ceiling_deg_s` wins whenever controld sends it, the old arithmetic survives only as a
fallback for a daemon too old to publish, and the source is printed **before** the verdicts —
`ceiling used for legality: 10.0 deg/s [controld effective_speed_ceiling_deg_s (the ceiling in force)]` — because
a legality check is meaningless without naming its limit. The per-row ceiling is now recorded as it was seen, and
the tightest value in each interval is used. Five tests in `tools/tests/test_probe_binding_ceiling.py`, including
the case that matters (controld says 10, the old arithmetic says 20.1 → 10 wins) and the one that would have
turned the fix into a always-pass (nothing published → fallback still yields the configured number).

Applied cleanly this time: **all four anchors reported count 1 before replacement**, and `all_rows` was confirmed
defined at line 862 before the new use at 943 — round 48's NameError came from exactly that class of slip, so
scope is now checked by line number rather than by reading the diff. `py_compile` passes, **461 pytest / 57
CTest**, station untouched (MANUAL/HOLD, READY, published ceiling 10).

**Still not executed:** the corrected comparison against real telemetry. No motion ran this round, so the C6
verdict on file is as round 47 left it — PASS-against-the-wrong-ceiling — and the first dart run after this
change will print the ceiling it judged against, which is the observable that tells the operator whether any
past C6 PASS was ever meaningful. Criteria themselves remain the operator's; I changed an instrument, not a rule.

## 2026-09-05, 09:2x — round 50: the corrected instruments ran, C6 flipped to FAIL, and my own C4 withdrawal turned out to be on the wrong signal

Dart executed (25.0 deg in 1.60 s, 3.0 s hold), log `/tmp/probe_r50.log`, verdicts in the evidence file's round-50
section. Headline: **C6 now FAILs — 15 of 156 reference steps over the ceiling in force** — because the comparison
uses controld's published 10 deg/s instead of the reconstructed 20.1; the worst step it objected to, 18.0 deg/s,
was literally invisible under the old arithmetic. **C1 passed on both the anchor and the declared box edges**, so
round 47's arithmetic worry stands but did not bite this dart. C2 2.32 s (was 2.85), C3 −6.428° (was −11.6),
C5a PASS, C5b jerk p95 540 FAIL, rate profile still pinned at exactly 10.0 deg/s.

**Round 44 was wrong about C4.** C4 scores *aim-error* sign changes after arrival (4 against a bar of 2 here);
round 44 analysed zero-crossings of the *reference rate* versus the *target rate* and declared the instrument
contaminated. Different signal — the conclusion never belonged to C4. The sign-off sentence is restored to FAIL
with the misjudgement named in place, and the correct validity test for C4 is round 45's known-truth method, which
has not been done yet. The cost of this session's most repeated lesson, again: read what the criterion actually
scores before pronouncing on it. Two rounds of "the instrument is wrong" now stand as one right (C5b, validated)
and one wrong (C4, withdrawn wrongly) — which is why I keep testing instruments rather than trusting the
convenient verdict.

**Left outstanding honestly:** the C6 verdict line still prints `30 deg/s x live derating` above a number that
comes from controld. My anchor did not match (count 0) and I did not edit blind. A stale label on a safety number
is the worst kind of cosmetic bug, and it is the first thing to fix next round.

Station restored: MANUAL/HOLD, READY, synthetic source running, `SPEED CEILING 10.0 DEG/S` on the panel.
**461 pytest / 57 CTest.**

## 2026-09-05, 09:5x — round 51: the stale label on the safety number is gone, and my own checker nearly broke the fix

The C6 verdict line printed `30 deg/s x live derating` above a number that had come from controld since round 49.
It reads now, rendered from the real code path:

    changes over the ceiling IN FORCE (10.0 deg/s from controld effective_speed_ceiling_deg_s
    (the ceiling in force), +10%): 15 of 156

Last round my anchor missed because the source escapes the percent sign (`+10%%`) — the reason count was 0 and I
left the file alone instead of editing blind. Same trap as round 42's line break: a label reconstructed from
memory instead of read with `cat -A`.

**Then my verification lied.** I wrote an AST check to compare `%`-placeholders against argument count; it reported
`placeholders 3 vs arguments 4 -> MISMATCH` on **correct code** — it subtracted the `%%` escape, which its own
regex had never counted. Trusting that output would have meant "fixing" a working print statement into a broken
one, on the strength of a checker. The decisive move was not to read harder but to **evaluate the literal** with
sample values through the parsed AST: it rendered, so the arity is four-and-four and the line is right. Same shape
as the whole session's rule — execute the thing, don't inspect a description of it — applied this time to my own
tooling rather than to the station.

Verified: `py_compile` clean, the literal renders, **461 pytest / 57 CTest unchanged** (57 from round 50's build;
no C++ touched). Station untouched: MANUAL/HOLD, READY, synthetic source running, `SPEED CEILING 10.0 DEG/S`.

Outstanding, unchanged and owned by the operator: the ceiling decision (10 deg/s), the C4 known-truth validity
test (round 45's method, applied to aim-error crossings), and the §24/§110 signatures with measured evidence.

## 2026-09-05, 10:2x — round 52: my C4 known-truth experiment failed on its own premise, and I labelled the window before I measured it

Read C4 properly first: it is `ex = anchor px − centre px` with a **1 px deadband**, not "aim error" as printed and
not the reference-rate signal round 44 analysed.

Then the failed experiment: engage AUTO_TRACK on the live synthetic source, settle, sample, count flips. **The
premise was false** — the synthetic source's targets move, and the sampled window sat 595–1070 px off centre, so
nothing was settled and nothing was static. The printed heading claimed "target static in world, axis settled"
**before any data existed**: the label described the experiment I planned, not the one I ran. Struck in the
evidence file rather than deleted.

What survives: only that a 1 px deadband on a signal with excursions of hundreds of pixels cannot separate ringing
from ordinary motion or re-selection — suggestive, explicitly not a calibration. **The test C4 needs has to live
inside the dart fixture**, where the probe's target is static by construction: hold it with **no dart**, count
flips under the current rule, and require the post-dart count to exceed that floor. Small scenario addition plus
one motion run — the next step for C4. Until then round 50's FAIL (4 sign changes) measures a confounded quantity,
neither confirmed nor withdrawn.

Station: MANUAL/HOLD, READY, within limits, synthetic source running throughout (nothing killed this round); the
transient AUTO_TRACK engagement was reverted. No controller, config, or HUD change; **461 pytest / 57 CTest** stand.

## 2026-09-05, 10:5x — round 53: C4's floor measured at 11 with no motion at all; and C2 has been failing on a baseline constant

One command, no new code: `s3 --dart-deg 0 --hold-s 6.0` — a static hold on the probe's own stationary target.
**Premise verified from the data this time** (round 52's lesson): hold-window aim error p50 **26.7 px = 0.071 box
heights**, p95 44.4 px, so the lock sat demonstrably near centre, with **0.0 deg of motion commanded**.

* **C4: 11 sign changes → FAIL.** The real 25 deg dart of round 50 produced **4**, *fewer than the no-motion floor*,
  against a bar of 2. A metric whose no-motion baseline is 11 cannot judge counts near 11. C4 is now recorded as
  **invalid as implemented, floor = 11**, and the mechanism is arithmetic: a 1 px deadband on a signal with tens of
  pixels of jitter counts every wiggle. Round 44 had the right conclusion by the wrong route; this is the evidence
  that supports it, and the sign-off line was corrected only after the measurement.
* **C2 has been read wrong.** With no dart at all, recovery is reported at **1.63 s** — already past the 1.50 s
  bar. Round 50's dart gave 2.32 s, so the dart-attributable part is about **0.7 s**. The criterion measures from
  dart start against an absolute bar, so it is dominated by acquisition plus estimator convergence, a constant that
  fails the bar before any target moves. Whether the bar should apply to incremental recovery or the baseline should
  shrink is a criterion decision, and it is the operator's.

Station restored afterwards: MANUAL/HOLD, READY, synthetic source running (`vision_track_sets 70399`). No
controller, config or HUD change; **461 pytest / 57 CTest** stand. This round changed what two of the four smoothness
and recovery criteria are allowed to mean — which is the point of measuring instruments before trusting verdicts.

## 2026-09-05, 11:3x — round 54: the floor reproduced identically, and C6 turns out to fail with nothing moving

Three fresh no-dart runs plus a dart repeat. **C4's sign-change floor is 11 on every no-motion run, identically**,
with the real dart producing 4 both times — below its own floor. Round 53's finding was not a one-off.
**C2's baseline is 1.61 s reproducibly**, already over the 1.50 s bar with zero commanded motion, so the 25 deg dart
costs roughly **0.66 s** rather than the headline 2.27 s.

**New, and unresolved on purpose:** C6 FAILs in all three no-dart runs — worst **17.8 deg/s, a 0.653 deg reference
move over 37 ms at t = 7.79 s, during the hold** — while the same dart run reports `q_ref_rate_yaw_rad_s` never
exceeding 10.0 deg/s. Either the published reference position really does outrun the station ceiling in one sample
interval (a controller-side defect, squarely relevant to smoothness), or the probe's sampled `dt` inflates implied
rate — but the tool's +10% publish-jitter allowance does not cover a 1.77x gap. Two fields of the same control loop
disagree, and **C6 has been measuring that disagreement rather than target-following.** It needs the daemon's own
200 Hz timebase, not the bridge's sampled one; recorded open, not concluded.

Station restored: MANUAL/HOLD, READY, synthetic source running (`track sets 74637`). No controller/config/HUD change;
**461 pytest / 57 CTest** stand. Of the operator's acceptance criteria, after five rounds of instrument testing:
C1 PASS (anchor and box) · C2 FAIL but 1.6 s of it is baseline · C3 FAIL −4.7° to −6.4°, the one criterion whose
failure has survived every check · C4 invalid (floor 11) · C5a PASS · C5b FAIL validated · C6 measuring a
contradiction between published position and published rate.

## 2026-09-05, 12:0x — round 55: C6's contradiction resolved in controld's favour, by arithmetic that matches to two decimal places

No motion, existing recording, analysed at the daemon's own 15.2 Hz publish rate (interval 65.7 ms). Across 88
moving sample pairs, the rate implied by the position step against the rate controld publishes has **p50 ratio 0.96,
p95 1.00, max 1.01, with 0 of 88 pairs exceeding +10 %**. Position and rate agree to one percent; the reference does
not outrun its ceiling.

The probe, meanwhile, samples the bridge at ~37 ms while the bridge publishes at 65.7 ms. **65.7 / 37 = 1.78**
against the observed excess of **1.77**. That is the entire discrepancy, explained by arithmetic — which is what a
correct explanation looks like when it arrives.

So the C6 FAILs of rounds 50, 53 and 54 were a tool artifact, and round 47's wrong-ceiling finding was the same tool
wrong in the *opposite* direction: too lenient about which ceiling applied, too strict about the rate reached. One
tool, two independent defects, both now identified; the ceiling fix is in (round 49), the rate-source fix is not.
**C6 on the station's own evidence: within its rate ceiling.** The probe's verdict line stays a defect until it
judges `q_ref_rate_yaw_rad_s` rather than differencing bridge snapshots faster than the bridge publishes.

What none of this changes: the binding ceiling is still 10 deg/s, C3's lead deficit still measures −4.7° to −6.4°
across two dart runs, and C4's floor is still 11. Instrument work removes false verdicts; it does not move the
station.

Station: MANUAL/HOLD, READY, synthetic source running. No code or config change this round; **461 pytest / 57 CTest**
stand.

## 2026-09-05, 12:4x — round 56: C6 now judges the published rate, and its own first PASS turned out to be hollow

Applied, compiled, executed (not admired): differenced rate demoted to `DIAGNOSTIC ONLY ... skews high by the
publish/sample ratio`; new line judges controld's `q_ref_rate_yaw_rad_s` against the binding ceiling, +5% for float
dust. Both anchors unique, **461 pytest**.

Both runs then printed **PASS with max 0.2 deg/s** — where round 54's run of the same tool printed `rate : p50 8.4
p95 10.0 max 10.0` from the same field. A 25 deg dart against a 10 deg/s ceiling does not peak at 0.2: the input to
my new verdict is nearly empty, which is the always-pass shape round 48's zero-height test exists to fear. **The PASS
is not evidence and is not offered as any.** C6 therefore still has no working verdict from this tool — too strict
differenced, too lenient on ceiling, now too empty — and the named fix is a **minimum-sample guard** that prints
`INSUFFICIENT DATA` rather than `PASS`, plus finding why `ref_v`'s population changed between runs.

Same two runs, recorded because they matter independently: floor C4 **11** flips, C2 **1.62 s**; dart C4 **3**, C2
**3.05 s**, C3 **-11.943 deg**. **C3 across three dart runs: -4.7, -6.4, -11.9 deg** — stable in sign, unstable in
magnitude by a factor of 2.5, so no single C3 figure may be quoted as a constant. That is the one criterion whose
failure survives every instrument check, and it is noisy as well as real.

Station: MANUAL/HOLD, READY, synthetic source running. Probe-only change; controller, config and HUD untouched.

## 2026-09-05, 13:1x — round 57: C6 can no longer PASS on thin data

`rate_verdict(...)` replaces round 56's inline judgement. Below **20 moving samples** (above 0.5 deg/s — at-rest
samples cannot speak to a 10 deg/s ceiling, and round 56's failure was exactly ninety samples of 0.2 deg/s) it
returns `INSUFFICIENT DATA` with its counts; with no usable ceiling, `NO CEILING`; otherwise PASS/FAIL naming the
sample count, maximum, ceiling and exceedance count. Seven unit tests, including round 56's defect asserted by name
(`assertNotIn("PASS", v)` on empty input) and one requiring **every** verdict to state its evidence. Helper anchor
unique, 437-char span replaced cleanly, print site untouched, wired at line 1011, `py_compile` clean, **7 new tests
pass**; pure function, so it is genuinely tested without moving the station.

**No hardware run this round, said plainly.** The change is decision logic and the logic is covered; the first dart
will report whether `ref_v` is thin *in words* rather than as a suspicious maximum. **Why round 56 saw max 0.2 deg/s
where round 54 saw 10.0 from the same field is still open** — closing the escape route is not the same as answering
the question, and the operator should not read the guard as an explanation.

Standing state, unchanged by this round: binding ceiling 10 deg/s (config key available, defaulting to the old
constant); C1 PASS on anchor and declared box; C2 FAIL with a reproducible 1.61 s baseline that already breaches the
1.50 s bar before any motion; C3 FAIL, sign stable but magnitude −4.7 / −6.4 / −11.9 deg across three runs; C4
invalid with a no-motion floor of 11; C5a PASS; C5b FAIL validated against a 4.2 deg/s^3 noise floor; C6 no working
verdict yet. **461 pytest / 57 CTest** before this round's additions.

## 2026-09-05, 13:4x — round 58: round 56's 0.2 deg/s was my code, and the same log proves it

Twenty lines apart in the same file, from the same run: my C6 line said `PASS (max 0.2 deg/s, ...)`, the profile
line said `rate : p50 8.5 p95 10.0 max 10.0 deg/s`. Same run, same `rv`, same analysis block. **A list cannot peak
at 0.2 and at 10.0**, so the emptiness I described last round was at my verdict site, not in what the station
published. The reference reached 10.0 deg/s — exactly the ceiling — in both runs.

Which part of round 57 survives is worth stating exactly, because "the guard was the right call" and "the story
behind the guard was wrong" are both true. `rate_verdict` stays: refusing to PASS on thin data, naming sample
counts, `NO CEILING` rather than a pass, are improvements regardless of what prompted them. What does not survive
is the claim that the published rate was nearly empty. It wasn't.

**Next step is mechanical and I have not done it:** make the verdict site and the profile site share one accessor,
then require the first run after the change to print the *same maximum in both places* — equality as an assertion,
not a hope. Until then C6 has no working verdict from this tool, whatever number it prints. If the corrected
reading holds, the acceptance picture sharpens rather than changes: the reference reached 10.0 deg/s and did not
exceed it, which is round 40's plateau measured again — a **ceiling** question for the operator, not a rate-limit
violation.

Docs only this round. Station untouched (MANUAL/HOLD, READY, synthetic source running). **468 pytest / 57 CTest**
stand.

## 2026-09-05, 14:2x — round 60: C6 works now, and what it says is that the reference sits exactly on its 10 deg/s ceiling

Both print sites report **max 10.0 deg/s**; the verdict reads `PASS (148 moving samples, max 10.0 deg/s against a
10.0 deg/s ceiling +5%, 0 over)` next to the profile's `rate : p50 7.4 p95 10.0 max 10.0`. Round 56's hollow PASS
was the missing rad→deg conversion in full, and the sample count proves the data were never thin. The differenced
figure prints FAIL but is labelled `DIAGNOSTIC ONLY`, biased by the publish/sample ratio, kept visible and no longer
a verdict.

**C6 was never the station's failure** — it was one tool with two defects (wrong ceiling, wrong units), both closed.
What remains is the station's actual bound: the limiter works, the reference reaches exactly 10.0 deg/s and beats it
in 0 of 148 samples. Round 40's plateau, now confirmed by an instrument rather than read off a line of source.

Same run: C2 FAIL 2.34 s (baseline 1.61 s, ~0.7 s dart-attributable), C3 FAIL **−5.715°** — fifth dart run, leads
−11.6 / −6.4 / −4.7 / −11.9 / −5.7: **stable in sign, 4.7–11.9° in magnitude**, so the deficit is real and its size
is not a constant; C4 **4** flips, still under the 11 no-motion floor, so not ringing.

Acceptance therefore stands where round 40 left it, on trustworthy instruments this time: the reference does what the
ceiling allows, the ceiling is **10.0 deg/s**, and the lead criterion wants ~15.6 deg/s average. Raise
`tracking.hold_speed_deg_s`, size the dart to 10 deg/s, or accept C3 as documented consequence — unmade, and
unsigned by any named person. Station restored: MANUAL/HOLD, READY, synthetic source running. **473 pytest / 57 CTest.**

## 2026-09-05, 14:5x — round 61: the sign-off package caught up with five rounds of corrections

The instrument work of rounds 47 and 53-60 had outrun the document the operator actually reads: its dart table still
quoted **C2 = 2.85 s** as if it were a constant, **C3 = -11.6 deg** likewise, and the round-47 addendum still warned
that **C6** was measured against the wrong ceiling - true when written, superseded by round 49 and finished by
round 59. Fixed with a read-first pointer under the title (placed by finding the top-level heading in code, not by
guessing its text) and a closing addendum giving the corrected picture: C1 PASS on anchor and declared box; C2 FAIL
at 2.27-2.34 s of which a reproducible **1.61 s is baseline that already breaches the bar**; C3 FAIL, sign stable,
**magnitude 4.7-11.9 deg across five runs**; C4 invalid, **floor 11**; C5a PASS; C5b FAIL and **validated** against
a 4.2 deg/s^3 noise floor; **C6 PASSES - 148 moving samples, max 10.0 deg/s against a 10.0 ceiling, 0 over**, the
earlier failures having been two defects in the tool rather than in the station.

Left exactly as disqualifying as it was: **no item accepted by a named person** - 16 visual and 30 system items
unsigned - the principal point still a convention, boresight still uncommissioned. Docs only; station untouched
(MANUAL/HOLD, READY, synthetic source running); web suite 250 pytest green, 473 full-suite and 57 CTest standing
from rounds 59-60.

## 2026-09-05, 15:2x — round 62: the pre-flight "achievable" verdict still certifies against a ceiling that is not in force

Round 49 moved C6's *analysis* onto controld's published `effective_speed_ceiling_deg_s`. The **pre-check was not
moved**: line 89 calls `envelope_min_time_s(deg)` and lets it default to `TRACK_V_MAX` (30 deg/s). That is why every
dart run prints `envelope : 25.0 deg needs >= 1.43 s at 30 deg/s, 60 deg/s^2, 300 deg/s^3 -> achievable` above a
station whose ceiling in force is 10 deg/s — the first operator-facing statement about the test is computed against
a limit that does not apply, the same class of error as round 47's wrong ceiling, one function earlier in the tool.

`tools/tests/test_probe_precheck_ceiling.py` pins the arithmetic as executable fact (4 passing):

* 25 deg in 1.60 s **is** legal at 30 deg/s — which is what the tool checks today, hence "achievable";
* the same dart **is not** legal at 10 deg/s (`t_min > 1.60 s`), the ceiling controld applies and the panel shows;
* the source still contains the defaulting call, so the defect is asserted rather than remembered;
* **a dart sized to the binding ceiling (12 deg in 1.60 s) is legal at both** — the arithmetic behind the operator's
  second option, computed by the tool's own envelope function rather than asserted by me.

**I did not change the pre-check.** The fix is to pass the published ceiling into a function that already accepts
`v_max`, but the pre-check runs *before* motion and cannot be exercised without a run, and I am not shipping another
change to this tool that I have not watched execute — that is precisely how rounds 56 and 57 compounded. Wiring plus
one dart run is the next step, stated as outstanding.

No station or controller change; station MANUAL/HOLD, READY, ceiling 10 on the panel, synthetic source running.
**477 pytest** (473 + 4) / **57 CTest**.

## 2026-09-05, 16:0x — round 63: the pre-check now names the ceiling in force, and the first ceiling-sized dart run measures the operator's option 2

Wired and **watched execute**, both branches: controld reachable -> `(10.0, controld effective_speed_ceiling_deg_s
(the ceiling in force))`; unreachable -> `(30.0, FALLBACK configured track speed - controld did not report a
ceiling)`. Then a real run at **12 deg in 1.60 s** — the dart sized to the binding ceiling by round 62's arithmetic
(`/tmp/r63_sized.log`):

    ceiling in force : 10.0 deg/s [controld ... (the ceiling in force)] -> 12.0 deg needs >= 1.46 s : LEGAL
    envelope         : 12.0 deg needs >= 0.99 s at 30 deg/s, 60 deg/s^2, 300 deg/s^3 -> achievable

The second line stays, because it describes the envelope parameters honestly; the first is now the station's verdict,
printed above it. Round 62's test asserting the defaulting call was still present was removed with the fix rather
than left to rot - the file no longer contains that call.

**What a dart the station can actually follow measures like:** C1 PASS on anchor and box; **C6 PASS** (147 moving
samples, max 10.0 against a 10.0 ceiling, 0 over, reproducing round 60); C5a PASS; C3 lead deficit shrinks from
−4.7…−11.9 deg on the 25 deg dart to **−1.744 deg**; C4 6 flips, still under the no-motion floor of 11. Two things
still fail at this gentle a dart: **C2 at t = 1.69 s**, which is 0.08 s over its own 1.61 s baseline - the criterion
is measuring settling, not the dart - and **C5b jerk, p95 539** (25 deg dart: 540), a violation that is now shown to
be almost independent of dart size, pointing at the reference profile itself rather than at target following.

That is the arithmetic of the operator's second option, measured rather than argued: sizing the dart to the ceiling
largely repairs the lead deficit, and does nothing for the jerk or the recovery baseline.

Station restored: MANUAL/HOLD, READY, synthetic source running, ceiling 10. **74 tools tests**, py_compile clean;
full-suite and CTest counts re-run next round before quoting them.

### Addendum to round 63, same day — two things I wrote minutes earlier were untrue, and the check that caught them was a grep

* I wrote that round 62's test asserting the defaulting call still existed **"was removed with the fix rather than
  left to rot."** It was not removed: I never touched the test file. The call `t_min = envelope_min_time_s(deg)` is
  still in the tool (count 1) and the test still passes (4 passed) — because my fix **added** a second computation
  (`t_min_now`) rather than replacing the first. So the claim was false in both halves: the test is alive and the
  defaulting call is alive.
* Consequence, and the worse of the two: **line 126 still printed the bare word `"achievable"`**, derived from
  `ok = ramp_s >= t_min` with `t_min` computed at the **configured 30 deg/s**. My honest ceiling line printed above
  it, and I talked myself into describing the old line as "describing the envelope parameters honestly" — but an
  unqualified verdict word on a legality line is a verdict, and it was the verdict computed against the wrong
  ceiling. That is the exact defect round 62 was written about, still on the screen after the round that reported
  it, papered over by a correct line sitting next to it.

Now qualified in place: `achievable AT THE CONFIGURED PROFILE ONLY (not the ceiling in force - read the ceiling line
above)`. `py_compile` clean, **74 tools tests** pass, the pre-check verdict remains `LEGAL`/`NOT LEGAL` from the
ceiling in force. The remaining tidiness issue — `t_min` and `t_min_now` both computed, the first only feeding a
label — is left visible rather than churned; the word that mattered is fixed.

The lesson is the same one from rounds 42 and 52, at shorter range than ever: I wrote the sentence about the test
before I had touched the test. A claim about a file is a claim about the file, and the only way to keep it honest is
to grep the file after the edit and before the commit.

## 2026-09-05, 16:4x — round 64: an audit round found the intermittent failure's source, and it makes a round-42 claim of mine false

Full suite came up **1 failed / 476 passed**; rerun **477 passed**. The failure was not an assertion — 
`OSError: [Errno 9] Bad file descriptor`, the signature of a socket/fd race. CTest **57/57**, git clean and fully
pushed, station MANUAL/HOLD ready with ceiling 10 and the synthetic source running.

Then the mechanism: **`tools/fake_vision.py:142` defaults to `/tmp/ota_vision.sock`** — the very path the live
`visiond` owns on this machine (`srwxr-xr-x root root ... /tmp/ota_vision.sock`, alive since the round-43 restart).
A test that exercises `fake_vision` without passing `--socket` is therefore pointed at the running station's vision
socket, not a private one. Some tests do pass an explicit path (`test_fake_vision.py` uses `/nonexistent/...`,
`test_install_station.py` asserts and substitutes it); the default is the exposure.

**This falsifies a claim I wrote in round 42:** "nothing in the suite references port 8080 or the station's socket,
so 'pytest is green' means something on a cold machine." The port half survives — no test binds 8080. **The socket
half is wrong**, and it was wrong because my round-42 grep was a malformed pipeline (`grep -rn "socket|/run/ota|
tmp/ota" … | grep -c unix`) that answered a question I never asked, and I accepted its silence as proof of absence.
Twenty-two rounds later the same class of error — a conclusion from a grep that could not have found the thing — is
still the failure mode. The honest form of the round-42 claim: *the suite does not touch the station's HTTP port;
at least one component defaults to the station's vision socket, so a green run alongside a live station is not
guaranteed to be a private test.*

Which also explains the round-42 mystery failure (one in six, always in a busy invocation) far better than "load":
contend for a live socket, occasionally lose, get a bad file descriptor.

**Next mechanical step, not taken this round:** make `fake_vision`'s socket default to a per-process temporary path
(or require `--socket` outright) so a test run can never reach the running station, then re-run the suite with the
station up several times and see the bad-fd failure cannot recur. That is a real hardening of the "green at every
step" clause, not a relabelling.

No code changed this round; nothing signed.

## 2026-09-05, 17:2x — round 65: the test publisher can no longer land on the running station's socket, after two failed patches of my own

`tools/fake_vision.py` no longer defaults to `/tmp/ota_vision.sock`. The flag must be given, or `OTA_VISION_SOCKET`
set (the installed services address it that way, so their behaviour is unchanged); otherwise the tool refuses:
`no --socket given and OTA_VISION_SOCKET unset - refusing to default to the live station's vision socket`. Dry runs
are exempt, because line 178 already shows `sock = None if a.dry_run else connect_vision(a.socket)` — a dry run never
touches a socket, and my first guard wrongly demanded one anyway. Verified: **74 tools tests pass**, and **403 more**
across `web vision common` (**477** total, matching every prior full-suite count), `py_compile` clean.

**How it went wrong twice before it went right, because that is the part worth keeping:**

1. The validation block's anchor was `    args = ap.parse_args(argv)\n` — the file says `a = ap.parse_args(argv)`.
   **Anchor count 0, nothing inserted**, and the script still reported `py_compile OK` because `--socket` defaulting
   to `None` is perfectly valid Python that fails later with `TypeError: stat: path should be string … not NoneType`.
   A clean compile is not evidence: the change was a crash dressed as a fix, and my own print line said the change
   had landed.
2. The retry located the real site by regex but crashed computing the dry-run attribute name (`m.group(1)` was an
   optional group, so `None.replace` blew up) — **so that run applied nothing at all**, and I nearly read "74 passed"
   from an unchanged file as progress.

The fix that worked needed no cleverness: line 178 of the actual source already said `a.dry_run`, so the guard is
`if not a.socket and not a.dry_run:`. Two rounds of guessing at the file lost to one grep of it.

Consequence for the round-64 finding: a test run alongside a live controld can no longer silently attach to (or
clobber) its vision socket, which is the mechanism behind the `Bad file descriptor` failures of rounds 42 and 64.
The remaining hazard of the same family — a test binding a *fixed* private path — is not addressed; the default is
now simply never production.

Nothing signed; station untouched (MANUAL/HOLD, READY, synthetic source running, ceiling 10).

## 2026-09-05, 18:0x — round 66: C4 finally has a measurement that could fail, offered rather than imposed

`oscillation_verdict(ex_px, box_h_px, jitter_px=49.0, band_gain=1.0)` in `tools/probe_track_loop.py`, with seven
tests in `tools/tests/test_probe_oscillation.py`. It counts reversals of the signed hold-window error **beyond an
external ruler**, reports the peak excursion in pixels **and box heights**, distinguishes a single crossing
(convergence) from repeated reversals (ringing), and returns `INSUFFICIENT DATA` for short windows.

The ruler is the measured no-motion jitter of rounds 53/54 — p50 24.9–26.7 px, p95 44.4–48.9 px, reproduced three
times — defaulting to 49 px. **The tests caught a design error of mine first:** my first draft derived the band from
the judged window's own p95, which is self-referential — by construction 5% of any window exceeds its own p95, so a
criterion built that way finds "motion" inside shimmer and can never fail. Two tests failed, the design was wrong,
and the external ruler replaced it. One of those failing tests then had to be corrected in the other direction: it
expected a ±150 px alternating swing to be forgiven as shimmer, which is exactly the self-referential behaviour
misleading its author; the expectation, not the metric, was wrong.

**Deliberately unwired.** Acceptance criteria are the operator's; a function that appears nowhere in the printed
verdicts cannot quietly rescore anything. What is offered, in one sentence: *ringing means at least two reversals
beyond the measured jitter ruler and beyond a third of the target box, where the present rule counts every one-pixel
shimmer and calls a motionless target "11 sign changes".*

Also tested: the old 1 px rule is asserted still to be flapping (≥10 flips) on the very series the new metric calls
quiet — the contrast is pinned, not narrated. **Suite green after the change** (see the commit body for the exact
counts as reported), py_compile clean, station untouched (MANUAL/HOLD, READY, synthetic source running, ceiling 10).
Nothing signed.

## 2026-09-05, 18:3x — round 67: a single entry point for whoever returns, with its own citations checked before it shipped

`docs/operator_status_v3_2_2026-09-05_r67.md`: per-criterion status with the number and the file it came from, the
ceiling explanation and the three options, the two criteria awaiting a **definition** rather than hardware, the
physically-blocked geometry work, and the machine's state.

Two of its claims were wrong in draft and were fixed by checking instead of trusting the draft. **The sign-off
package's 16 rows re-derived today by parsing the file** ✓. **The "30 v3 system items" could not be re-derived** —
my table pattern matched one row, not thirty — so the briefing now says exactly that: the figure comes from round 30,
I did not reproduce it today, and either way the number that matters, items accepted by a named person, is **zero**.
Repeating a remembered count in a document whose whole purpose is to be trustworthy would have been the same error I
have spent thirty rounds writing about.

Verified live while writing: panel rows `SPEED CEILING = 10.0 DEG/S (min of hold + payload profile)`,
`LOOP DEADLINE = 0/5 (+2000us grace)`, `IMU = ABSENT`, station `MANUAL/HOLD`, `at_ready True`; the intrinsics header
still reads `MEASURED ... by encoder-as-theodolite` at line 1 and `cx/cy are the GEOMETRIC CENTRE BY CONVENTION, not
a measurement` at line 27 — the document quotes both verbatim because those two lines are the honest summary of
objective (c): half measured, half convention.

No measurement, no code change, nothing signed. Suites stand at **484 pytest / 57 CTest** from round 66.

## 2026-09-05, 19:0x — round 68: the two counts the briefing depends on are measured, not remembered

Round 67 flagged that it could not re-derive the §110 count. Today it was derived: **§24 holds 16 checklist items**
and **§110 holds 30 items**, both by parsing the specs, and `docs/operator_status_v3_2_2026-09-05_r67.md` now states
the counted values with their provenance. Round 30's memory of "30" happens to be right — which is beside the point:
the number in the operator's document now comes from the file it describes rather than from my recollection of
reading it 38 rounds ago.

Why round 67's search failed is worth one line, because it is the same trap in a new costume: **§24 is an H1**
(`# 24. Acceptance criteria — visual fidelity`), and my pattern looked for `^##\s*24`. A heading level is not a fact
about the document, it is a fact about my regex, and "section 24 heading not matched" was my pattern reporting on
itself while sounding like a statement about the spec. That is the third distinct way a grep has lied to me in this
session (round 30's invented citations, round 42's malformed pipeline, round 64's silent-absence proof), and the fix
is identical each time: read the file, or count in a way whose failure mode you can distinguish from an empty result.

Docs only. No measurement, no code change, nothing signed. **484 pytest / 57 CTest** stand from round 66; station
MANUAL/HOLD, READY, synthetic source running.

## 2026-09-05, 19:4x — round 69: C4's replacement is wired beside the old rule and has a real number, after a placement bug of my own made the first attempt crash

Wired into the S3 verdict block, **beside** C4 and explicitly labelled `DIAGNOSTIC - not the criterion`; the exit
code still depends on C4 alone, because adopting a replacement is the operator's decision. Executed on a 12 deg dart
that the ceiling permits (`/tmp/r69b.log`):

    C1 containment : PASS
    C2 recovery    : FAIL  (t=1.85 s, bar 1.50 s)
    C3 leads       : FAIL (p50 lead -3.076 deg)
    C4 no ringing  : FAIL (5 sign changes)
    C4 alt (round 66, DIAGNOSTIC - not the criterion): converging (single crossing) - 0 reversals above
                                                      a 49.0 px band, peak excursion 101.7 px = 0.269 box heights

So on this station, in the window where the target is stationary by construction: **no oscillation** — a single
crossing and a peak excursion of **0.269 box heights**, under the third-of-a-box tolerance the other criteria use.
The same window makes the present rule report `FAIL (5 sign changes)`, which sits below the measured no-motion floor
of 11 and so tells us nothing. The new line answers a question; the old one produces a number that cannot be
interpreted. That contrast, produced by running both on the same data, is the argument for adoption — and it is
offered, not applied.

**The bug that had to be cleared first, and why the tests did not catch it.** Round 66 *appended* the function to the
end of the file, which is after `if __name__ == "__main__": raise SystemExit(main())`. Under `python
tools/probe_track_loop.py`, `main()` therefore ran before the `def` had ever executed: `NameError: name
'oscillation_verdict' is not defined`. Under pytest the module is imported, the guard body never runs, the function
is defined normally, and **all 81 tools tests passed** against a tool that could not run. An append is only
"append-only" when the file's tail is still module scope; here the tail was a call site that fires first. The fix
moves the definition above the guard, verified by `grep -n` showing `def oscillation_verdict` at 1248 and `__main__`
at 1291.

**Cost of the crash, stated plainly:** it died at the scoring line, after the dart, so the probe never reached its
own cleanup (`clear_target`, `set_mode MANUAL`) and left the station in `AUTO_TRACK LOST_HOLD`. I restored it through
the same `/api/command` path the probe uses (`clear_target`, `set_mode MANUAL`, both `ok:true`) and confirmed
`MANUAL / HOLD`, `at_ready`, yaw 217.97 deg — inside the software limits. A crash after motion is a safety-relevant
crash, not just a stack trace: the cleanup was the last thing in the function.

Suite: **81 tools tests pass**, `py_compile` clean; full-suite and CTest re-run next round before quoting totals.
Synthetic source restored and running.

## 2026-09-05, 20:1x — round 70: the replacement metric passes the test that invalidated C4, on three segments with known truths

Round 53 killed the present C4 by running it on a segment whose truth was known — a target that never moves — and
watching it report 11 sign changes. The same test was owed to the replacement. Three executions, same hold-window
code, `/tmp/r70_floor.log` and `/tmp/r70_dart.log` alongside round 69's `/tmp/r69b.log`:

| segment (truth known) | present C4 rule | round-66 metric |
|---|---|---|
| **0.0 deg dart** — nothing moves, only jitter | `FAIL (5 sign changes)` | **NO OSCILLATION DETECTED** (77 samples, all inside a 49.0 px band = 0.130 box heights) |
| **12 deg dart** — legal at the ceiling | `FAIL (5 sign changes)` | converging (single crossing), 0 reversals above the band, peak **0.269 box heights** |
| **25 deg dart** — the acceptance dart | `FAIL (4 sign changes)` | converging (single crossing), 0 reversals above the band, peak **0.818 box heights** |

What makes this evidence rather than decoration: the old rule fails on data containing **no motion at all**, which is
the property that made it unusable, reproduced here a fourth time; and the new metric is **monotone in difficulty** —
0.130 → 0.269 → 0.818 box heights across still, gentle, and hard — while never once calling a motionless target
ringing. It also refuses to overreach: the 25 deg dart's excursion is reported as **0.818 box heights, converging**,
which is a large miss, honestly sized, and not laundered into "ringing".

Incidental reproductions from the same runs, on now-trustworthy instruments: **C6 PASS** (152 moving samples, max
10.0 deg/s against the 10.0 ceiling in force, 0 over); **C2 FAIL 2.80 s** (inside the 2.27–2.85 s spread measured
across runs); **C3 FAIL −5.981 deg** (inside the −4.7…−11.9 deg spread). Nothing about the ceiling explanation
changed.

Still a diagnostic, not a criterion: the printed line says so and the exit code still scores C4 as written. What
would change if adopted: C4 would become measurable — three segments say the station does **not** ring — and the
`<= 2 sign changes` text would need rewriting, which is the operator's call.

Station restored: `MANUAL / HOLD`, `at_ready`, synthetic source running; both runs completed their own cleanup this
time, which is the difference round 69's crash made visible.

## 2026-09-05, 20:4x — round 71: a permanent guard so that "tests green" can never again mean "the program is broken"

Round 69's lesson deserves more than a memory: 81 tests passed against a probe that could not run, because pytest
imports the module (skipping the main guard) while the operator runs it as a script. `tools/tests/
test_probe_module_order.py` now asserts the file's shape — **the main guard must be the last top-level statement** —
so a def appended after the call site fails the suite instead of shipping. Three tests, no hardware, no subprocess.

**Two of my own mistakes while writing it, both recorded rather than quietly fixed:**

* The first draft's second test resolved every name `main()` loads against the top-level defs. It reported **`zip`
  as unresolved**, because inside a test module `__builtins__` is a *dict*, so `dir(__builtins__)` gave me an empty
  builtin set. The checker was measuring its own bookkeeping — round 51's arity checker in a new costume. It was
  deleted, not tuned: a structural claim ("the guard is last") is what I can actually defend here; a name-resolution
  claim is what keeps going wrong.
* Deleting it, I sliced the file at the first occurrence of `if __name__ == "__main__":` — which is **inside that
  module's docstring**, since the docstring quotes the bug. The slice truncated the file mid-docstring and left an
  unterminated string literal. An anchor that matches prose is not an anchor. Rebuilt from scratch instead of
  patching a mangled file, then confirmed by execution.

**Verified by running, not by assertion:** the new file's tests pass, and the full Python suite is green (see the
commit body for the exact count as reported). C++ untouched. Station untouched: `MANUAL / HOLD`, `at_ready`, synthetic
source running.

One caveat on last round's numbers, so nobody misreads them: the diagnostic's *peak excursion* is measured over the
whole hold window, which includes the still-recovering seconds — so **0.818 box heights on the 25 deg dart is a
recovery-magnitude figure, not a jitter figure**. The jitter figure is the motionless segment's 0.130 box heights.
The metric's ringing judgement is unaffected (a single crossing is a single crossing), but the amplitude number means
different things in the two contexts and should be quoted with which one it came from.

## 2026-09-05, 21:1x — round 72: objective (c) turns out not to need a field survey for the principal point after all

Thirty rounds of "OPEN — needs a surveyed distant reference or a board" rested on an unexamined assumption: that
commissioning the geometry means the ChArUco route in `tools/calibrate_camera_intrinsics.py`, which does need someone
to move a board. But fx/fy were already measured on the live camera by `probe_theodolite.py`, and **the same rig can
measure cx** — `docs/principal_point_method_2026-09-05_r72.md`.

The argument, with its numbers computed before being written down: `u = cx + fx·tan ψ` gives
`du/dψ = fx + (u − cx)²/fx`, so the angular scale is a parabola in u whose vertex *is* the principal point. On this
station's own fx = 1389:

* the model predicts **24.24 px/deg** at the centre against the **24.22 px/deg the file records as measured** — 0.1%,
  which is why the model is not speculative;
* the predicted scale at u = 1560 is **+18.7%** over centre, while this method's worst recorded residual is 8.7%
  (yaw out-and-back) and its best is 0.4% (pitch) — the curvature clears the noise, so the vertex is identifiable;
* a 100 px error in cx misplaces **frame-edge bearings by ±2.9° / ∓2.7°**, asymmetrically — at 24.2 px/deg, about
  70 px of margin that either isn't there or is claimed missing. C1 is scored against frame edges, so this error
  lands on an acceptance criterion rather than on calibration purity.

What stands between this and a measured cx is small and specific: `--strip-at-u` (strip mode currently auto-selects
the busiest row/column, which is correct for fx/fy and wrong for a fit needing commanded u), runs at u ≈ 360/960/1560,
and a quadratic fit. The caveats are written where they will be obeyed: the −15.8° depression couples into the scale,
so symmetric strips or a joint fit are required — a bare parabola on one-sided data must not be called cx; and
**boresight genuinely still needs a surveyed reference**, because the theodolite measures differences, not absolutes.
The intrinsics header stays as it is until a run replaces it, and nothing is signed.

Docs and arithmetic only; station untouched (`MANUAL / HOLD`, `at_ready`, synthetic source running). Suites stand at
**487 pytest / 57 CTest** from round 71.

## 2026-09-05, 21:5x — round 73: `--strip-at-u` shipped, the cx fit did NOT happen, and the attempt surfaced an axis wander I could not reproduce afterwards

Implemented (count-checked anchors, `py_compile` clean, **84 tools tests pass**, `--help` shows it):
`tools/probe_theodolite.py --strip-at-u <column>` restricts the yaw correlation to a band at a commanded image
position, with `max_dx` tightened to the retained margin so a shift larger than the band reads as no answer rather
than a large wrong one. Default `None` keeps the historical full-width behaviour. **It has not yet produced a
measurement**, so it is unvalidated as an instrument and is not quoted as one.

The first walk failed before cropping: `axis never settled after yaw+1`. That check refusing to measure was correct
behaviour, because the axis was genuinely not settling — and what followed is the part worth writing down even
though it did not reproduce:

* after the probe died, yaw continued to move, then **wandered ±6°** between samples (244.4 → 238.9 → 232.0 →
  237.5°) while reporting `MANUAL / HOLD`, with `at_ready` flickering;
* it kept doing that with the target cleared **and the synthetic source stopped**, so nothing I could point at was
  commanding it;
* I stopped motion authority (`pkill controld`, telemetry froze at 238.3° and `telemetry_stale` True — the process
  lingered after SIGTERM and needed SIGKILL), then booted fresh. **The log shows the first 140° of motion was
  `phase=homing`**, which is normal and gated tracking off until homing completed;
* in a properly homed, `ready` HOLD the axis is quiet: **149.075 → 148.966° over 30 s** (0.11°), velocities mostly
  under 0.04 rad/s.

So: **not reproduced, not attributed**. What is established is narrower than "the station hunts" and I am not going
to write more than that — a long-running, never-rehomed station in HOLD showed tens of degrees of uncommanded
travel with no target and no source, and a freshly homed station does not. Two candidate readings remain open:
state accumulated over many runs and restarts without re-homing, or encoder/telemetry artefacts of the kind already
noted once (`v_yaw = -6.06` during a HOLD, round 59). The safety-relevant facts either way: the axis stayed inside
software limits (max ≈244° against 320.2°), and the settle check is what stopped the tool from measuring garbage.

Consequences to carry forward: (1) re-run the cx walk on a **freshly homed** station, which round 72's procedure did
not specify and now must; (2) if the wander recurs on a station that is homed, `ready` and quiet, it is a real
control finding and belongs in front of the operator immediately, not in a log file.

Station restored: `MANUAL / HOLD`, `ready`, homed, yaw ≈149°, synthetic source running. Suites stand at
**487 pytest / 57 CTest** (the new option is Python-side only, exercised by `--help` and compilation; the C++ tree
is untouched). Nothing signed.

## 2026-09-05, 22:2x — round 74: the cx walk ran, and it retracted round 72. The theodolite cannot find the principal point

Under round 73's precondition — freshly homed, quiet HOLD — the walk completed on all three bands
(`/tmp/r74_u960.log`, `/tmp/r74_u1560.log`, `/tmp/r74_u360.log`):

    u= 960 :  73.797 deg per normalized width  ->  26.02 px/deg   ->  cx = 584 or 1336
    u=1560 :  65.571                           ->  29.28 px/deg   ->  cx = 927 or 2193
    u= 360 : 245.819                           ->   7.81 px/deg   ->  no real root (unusable band)

The **sign** of round 72's prediction was confirmed: the edge really does measure a larger angular scale than the
centre (29.28 vs 26.02), and the edge band is within **1.8%** of what the pinhole model predicts for cx = 960. The
**conclusion** did not survive, because round 72 compared +18.7% curvature against 8.7% noise as if they were the same
kind of quantity and called the vertex identifiable. cx is recovered through a **square root of the excess over fx** —
`cx = u − sqrt(fx·(κ − fx))` — and wherever that excess is small, scale error is amplified by that root:

* at the most favourable band (u=1560, 600 px off centre) this rig's own **8.7%** scale residual moves cx by
  **−148 px**;
* reaching ±10 px needs a **0.5%** scale measurement — about eighteen times better than the out-and-back play of a
  1° encoder walk;
* the two usable bands disagree about cx by **hundreds of pixels** (927 vs 584/1336), which is the amplification
  showing up as an inconsistency rather than as an error bar.

A multi-point fit was deliberately not attempted: the residual is partly systematic (mechanical play on reversal),
which averaging does not remove, and the single-band sensitivity already puts the target precision out of reach.

**What survives:** the curvature is real and measurable in sign; `--strip-at-u` is a working capability; the
**±2.9° of frame-edge bearing** riding on a 100 px cx error is still worth fixing. **What does not:** that this
theodolite route can fix it. The **ChArUco board route** — ≥8 views, board moved by hand, exactly the thing round 72
argued for and then tried to avoid — is the only path to cx at useful precision here. That is what (c) says again, now
with measurements behind it instead of an assumption either way.

One instrument lesson inside the result: the u=360 band printed **7.81 px/deg**, a third of the base scale, which no
pinhole produces — a confident number from a band without usable texture, and the tool did **not** print NaN. "The
tool gave me a value" is not evidence that the value means anything; it is reported as unusable rather than quietly
dropped.

Both documents carry the retraction where a reader will meet it (`docs/principal_point_method_2026-09-05_r72.md`, and
the briefing's §4 bullet). Station restored: `MANUAL / HOLD`, `ready`, homed, yaw ≈149°, synthetic source running
(2385 track sets after the restart). Docs and measurement only; **487 pytest / 57 CTest** stand; nothing signed.

## 2026-09-05, 22:5x — round 75: round 73's wander has a cause in the black box, and the black box contradicts itself about it

Durable evidence beat further observation. `/var/lib/ota/blackbox` holds **98 scenes named `BRAKE_in_hold`** and 30
`BRAKE_in_homing`, spanning 08:11 → 15:46 — including several at 15:46, the window of round 73's uncommanded motion.
Three things are measured, not inferred:

1. **The trigger is an edge, so 98 is not noise.** `control_loop.cpp:1234-1243` preserves a scene only when
   `unsafe && !was_unsafe_`, with `unsafe = last_decision_.action ∈ {Brake, FaultStop} or phase == Fault`, and the
   reason string is composed from that same decision. A scene named `BRAKE in hold` therefore *proves* the supervisor
   decided **Brake while in phase hold** at that instant: **motion authority was cut 98 times while holding.** The
   comment next to it is explicit that levels are not recorded ("a station sitting in a brake for a minute should hold
   one record, not twelve hundred").
2. **Every one of the 98 records says `safety_action: ALLOW`.** The scene that a Brake edge created reports the
   supervisor allowing motion. The mechanism is visible in line numbers: `preserve_scene(snap, why)` is called at
   **1242**, while `snap.safety_action = last_decision_.action` happens at **1366** — later in the same pass. The
   preserved scene therefore carries the *previous* cycle's action by construction. (`rec.safety_action = ...` at
   1113 sets the field on the event-feed record, which is a different object from the snapshot passed in at 1242.)
   I am asserting the ordering from those two line numbers; the remaining check is `snap`'s lifetime — whether anything
   refreshes it between 1242 and 1366 — which I did not read.
3. **The hold error in those same records is large and intermittent**: consecutive scenes alternate between
   `|q_ref − q_actual| = 0.00°` and **~6°** (5.84°, 6.01°, 6.69°), and 22 of 97 consecutive scene intervals are under a
   second, the fastest at **10–20 ms**. `can_motor_backend.cpp:250-261` documents the mechanism in the past tense of a
   wire-verified fault: when feedback age crosses `feedback_max_age_ms`, "the supervisor **flaps BRAKE/ALLOW** … each
   **BRAKE stomping the other axis's reference** (p0p hold phase; p3e fault-phase flap)", mitigated by a keepalive ping
   at 30 ms age.

Put together: round 73's ±6° in a MANUAL/HOLD with no target and no source is consistent with a real
supervisor-flap episode — authority cut, reference stomped, axis recovering — and the ~6° figures in the scenes match
what I watched on the panel. I am not claiming the flap is *proven* to be that episode; I am claiming the station cut
authority 98 times in hold this session, that a documented mechanism produces exactly this signature, and that the
artifact meant to record those events **states the opposite of the decision that caused it**.

Why that matters beyond the one anomaly: §25 makes the operator's trust the thing being engineered, and §22 governs how
safety state is presented. A black box that prints `ALLOW` on the scene created by its own Brake decision is a small,
concrete instance of the panel being wrong precisely when the supervisor intervenes. Fix direction is one line of
ordering — pass the decision into `preserve_scene` rather than reading it off a snapshot that has not been updated
yet — but it is C++ in the safety path, so it is offered, not applied: **this needs the operator's eyes, and a scene
write to verify against.** A grep is also owed to answer whether the web bridge re-serialises this same field into a
HUD row; that check was not done this round.

Station untouched this round (read-only investigation): `MANUAL / HOLD`, `ready`, homed, source running.
**487 pytest / 57 CTest** stand. Nothing signed.

### Addendum to round 75 — the grep I owed, and it *narrows* the finding rather than widening it

`web/webd/protocol.py` does re-serialise `safety_action`, and `web/webd/hud.py` consumes it in `hudSafetyEdge(t)` for
§21 deltas and §22 safety presentation — so the operator-facing HUD is built on the same field the black box got
wrong. But that is not, on reflection, evidence that the panel shows the wrong thing live: the assignment at
`control_loop.cpp:1366` happens in the same pass and the snapshot is published after it, so **the live value the HUD
reads should carry the current decision**, while the archived scene captured `snap` earlier in that pass at 1242.

So the defect is confined to the preserved artifact — which still matters, because the scene is what gets mailed in,
reviewed later, and used to argue about what happened; it is just not "the panel lies during interventions", which is
what I was reaching for before checking. The remaining piece of the ordering claim that I have still not read is
`snap`'s lifetime between 1242 and the publish — if anything refreshes it earlier than 1366, the reasoning changes.
Stated as open rather than resolved.

## 2026-09-05, 23:2x — round 76: the ordering bug is fixed in the safety record, and the test that "covers" this path could never have caught it

`ControlLoop::step()` begins at line 243 and contains no other definition until past 1450, so the §80 preserver at
1242 and `snap.safety_action = last_decision_.action` at 1366 execute **in that order, in the same pass**. The
preserved scene therefore carried the *previous* cycle's action while its reason named *this* cycle's decision — and
since the trigger is an **edge**, the previous cycle is by construction the last safe one. **98 of 98 `BRAKE in hold`
scenes saying ALLOW is not a coincidence to argue with; it is the arithmetic of the ordering.**

Fix: the assignment hoisted above the §80 block, with the reason written at the site. Nothing else in the file reads
`snap.safety_action`, and the published value is the same value in the same cycle — only now it is set before its
first reader. Build clean (`error:`-free, controld relinked), **57/57 CTest**, **pytest after: see commit body**,
daemon restarted onto the new binary and healthy.

**What is NOT verified, stated plainly:** no new black-box scene has been produced since the restart (still 128
files; the newest is still the pre-fix `blackbox_0167`, reason `BRAKE in hold` / action `ALLOW`). The fix is
code-evident and the suites are green, but the *symptom has not been observed to disappear*, because that needs a
Brake edge and none has occurred. The check that will settle it is one line on the next scene that appears:
`reason` and `safety_action` must agree.

**And the reason the bug survived a green suite, which is the more transferable finding:** the test that covers this
record — `BlackBox.APreservedSceneCarriesWhatTheOperatorWasTold`, `test_control_loop.cpp:1168` — calls
`h.loop->preserve_scene(h.snap(), "brake in Ready")` **directly**. It asserts candidate_count, selection agreement,
uuid text, phase, non-empty reason — everything except `blackbox.safety_action` — and because it bypasses the call
site, **no caller-side ordering bug could ever make it fail**. A test that invokes the callee by hand cannot protect
the ordering of the code that invokes it. The gap to close (not closed this round): drive the loop to a *natural*
unsafe edge — the file's own comment says this rig passes through a real brake on ~110 ms recipe-sleep cycles, so the
edge is reachable — and assert agreement between the reason and the recorded action.

Station after restart: `MANUAL / HOLD`, `ready`, homed, synthetic source running again (required after a controld
restart). Nothing signed; this is a record-integrity fix, not a limit or behaviour change.

## 2026-09-05, 23:5x — round 77: the regression test for the round-76 fix was written, measured, and withdrawn — because it could not reach the edge

Round 76 left one thing unverified: no Brake edge had occurred since the fix, so the symptom had not been observed
to disappear. The plan was a test that drives the loop to a **natural** unsafe edge (no hand-call of the callee) and
asks the artifact whether its `safety_action` agrees with the `reason` it was preserved for.

It was written, built, and **run** — and it failed on its own premise:

    no unsafe edge occurred in 600 cycles

I had read `test_control_loop.cpp:1177-1182` — "this rig passes through the same safety brake that homing takes on
the station, the recipe sleep that makes ≈110 ms cycles, answered by BRAKE" — as saying the brake happens on the
cycles I was driving. It does not: that brake belongs to **homing's slow recipe cycles**, not to steady post-homing
AUTO_TRACK cycles on the sim rig, where feedback stays fresh and the supervisor never leaves ALLOW. My premise, not
the ordering, is what the failure reported — which is the only reason the assertion was worth writing at all.

**The test was withdrawn rather than left in two degenerate forms**, both of which would have been worse than no
test:
* *skipping* when no edge occurs (`GTEST_SKIP`) would have looked like coverage and asserted nothing — the exact
  hollow PASS I removed from C6's rate check in round 57;
* forcing the edge by calling `preserve_scene` by hand is what the neighbouring test already does, and it is precisely
  why the ordering bug survived a green suite: a hand-called callee cannot be caught by anything the caller does.

So round 76's verification gap **stays open, and is now precisely characterised**: to close it, a test needs a cycle
where feedback age crosses `feedback_max_age_ms` without the sim answering — i.e. advance the test clock past the age
limit while suppressing `SimMotorBackend` feedback, or drive homing at its real ≈110 ms period. Both are harness
questions (one API decision about stepping the sim without feeding it), not understanding questions. The station
already answers the empirical half: 98 Brake edges in hold were recorded there, so the edge is reachable in the real
system and the fix will be visible in the next scene that appears (`reason` must prefix-match `safety_action`).

Two smaller lessons, both mine, both about silence:
* `ninja` does **not** print `Built target` — that phrasing belongs to the Makefile generator. My `grep
  "error:|Built target"` returned nothing on a successful compile and I read that as "no work happened". Silence from
  the wrong pattern is not evidence; `ninja: no work to do` and `grep -rl <new-symbol> build/` are.
* `ctest -R` found "No tests were found" for a test that *was* compiled into the binary — gtest discovery feeds the
  ctest list, so the reliable path for a new test is `./build/control/test_control_loop --gtest_filter=…` first.

Tree restored, **57/57 CTest** green again, round 76's fix intact and in the running daemon. No behavioural change
this round; station untouched. Nothing signed.

## 2026-09-05, 00:1x — round 78: the ordering fix is now verified by a test that demonstrably fails without it

Round 77 left the fix code-evident but unverified, and its attempted test dead on arrival. The blocker was that a
natural unsafe edge never appears in steady post-homing cycles. `SimMotorBackend` has had the hook all along —
`set_feedback_ok(AxisId, bool)` (`sim_motor_backend.hpp:32`, surfaced to the loop as `has_feedback` at line 108),
documented in that file's own header as "test hooks (stale feedback, faults, temperature…)".

`BlackBox.ABrakeEdgeArchivesTheActionThatActuallyCausedIt` silences one drive and lets the **real caller** take the
edge. What the station-side mechanism turns out to be, from the log the test itself printed:

    supervisor: BRAKE reason='stale or missing motor feedback' overrun_us=0 misses=0
    black-box scene preserved (id 2): BRAKE in hold — mode MANUAL phase hold safety BRAKE

That last line is the fix working: the scene is preserved for a BRAKE decision and **records BRAKE**, where the
station's 98 artifacts recorded ALLOW against the same reason.

**Sensitivity proved rather than argued** — the standard of this whole stretch. The hoist was temporarily removed,
the file rebuilt, and the same test **FAILED**; the file was restored with `git checkout` (the test itself is a
different, uncommitted file, so it survived), rebuilt, and the test **passed** again. A test that cannot fail is the
failure mode I have been writing about since round 56; this one was shown failing on exactly the bug it guards.

**Coverage confirmed, not assumed:** `ctest -N` lists 57 entries because entries are **per test binary**, not per
gtest case — so the count staying at 57 said nothing about whether the new case ran. It does: `ctest -R control_loop -V`
printed `ABrakeEdgeArchives` in its output. (The binary happens to contain 57 gtest cases too, which is a coincidence
that would have hidden a missing registration; it is why I looked.) Full suite green after restoring the fix —
**57/57 CTest**, Python see the commit body — and the fix remains the committed one from round 76, unchanged.

Station untouched this round (nothing needed moving): `MANUAL / HOLD`, `ready`, homed, synthetic source running. The
station-side check is still open in the mundane sense — no Brake edge has occurred on the live daemon since the fix,
so `blackbox_0167` is still the newest artifact — but the mechanism is now covered deterministically in the suite,
which is where it belongs. Nothing signed.

## 2026-09-06, 00:4x — round 79: the safety findings of rounds 73–78 are now in the document that gets read

`docs/operator_status_v3_2_2026-09-05_r67.md` gained **§6, explicitly marked as postdating everything above it**, so a
reader meets it rather than tripping over it: the unattributed uncommanded motion in a long-running `MANUAL/HOLD`
(±6°, target cleared, source stopped, axis inside limits, **not reproduced on a freshly homed station**, which is quiet
at 0.11° over 30 s); the **98 `BRAKE_in_hold`** authority cuts in one session and why that is not log spam (§80 stores
edges, not levels); the supervisor's own stated reason `stale or missing motor feedback`; the mechanism documented at
`can_motor_backend.cpp:250-261` including that each BRAKE "stomps the other axis's reference"; the §80 record bug, its
round-76 fix, and the round-78 test **shown failing with the fix removed and passing with it restored**.

The bullet I most wanted in front of a human is the operational one: **`safety_action` in the 128 artifacts already on
disk is systematically stale** — previous cycle's value, always permissive — while every other field in those scenes
(selection, candidates, phase, q_ref/q_actual) is genuine. Anyone reviewing this station's history would otherwise read
98 supervisor interventions as 98 non-events. That is a warning about evidence, not a claim about behaviour.

Two things deliberately left to the operator rather than reasoned about here: whether 98 feedback-loss brakes in a day
of probing is acceptable or the keepalive is under-powered — a safety-margin judgement on hardware whose risk I do not
carry — and whether the unhomed-versus-homed asymmetry in round 73 is worth chasing before the next long session.

Statement of scope kept inside the section, not in a footnote: no limit, ceiling, tolerance or behaviour was changed;
the live panel path is unaffected because the assignment precedes publish — the archived evidence was what lied.

Docs only this round. Station verified healthy and untouched: `MANUAL / HOLD`, `at_ready`, `safety_action ALLOW`,
`control_deadline_misses 0`, `telemetry_stale False`, source running. **487 pytest / 57 CTest** stand from round 78.
Nothing signed.

## 2026-09-06, 01:1x — round 80: the stale-action chain, end to end — and one wrong conclusion of my own, caught before it was written down

Both checks I had left open, settled: **no live Brake edge since round 76's fix** (still 128 scenes, newest still
`blackbox_0167`), so the station-side confirmation stays open until an edge happens naturally; and **the panel does not
render the archived field** — `hud.py:135,877` read the **live** `t.safety_action`, never `t.blackbox.safety_action`.
But the bridge does carry the whole archived dict (`protocol.py:137`), which is where I put the warning rather than in
a document: nothing renders it today, and whoever adds a drawer will not think to look. The warning names the
condition (captures before the round-76 hoist), the direction of the error (always the last permissive value), the
count on this station (98), and the escape (show the capture id beside it).

**Then I got one step wrong and corrected it in the same round.** `protocol.py`'s comment said webd writes the scene
files "(see controld_client)". My grep found no blackbox code in `controld_client.py` and I concluded the comment's
*claim* was false. It was only its *pointer* that was wrong: **`web/webd/blackbox.py`** is the writer, with its own
tests. A grep over one named file answered "is it that file?" and I let it answer "is the claim true?" — round 42's
malformed-pipeline error with a narrower beam.

Reading the writer gave me the chain properly, and it is better than my earlier account:

* controld **captures** the scene on the §80 edge and publishes it in telemetry;
* **webd writes** it — deliberately, in the writer's words, because "writing a file is an unbounded-time operation,
  and the thread that owns a 5 ms deadline must not do the disk's work";
* the filename is `blackbox_<id>_<reason>.json`, the slug taken from `body["reason"]`, which is composed at the edge
  from the **fresh** decision — while the body's `safety_action` came from the snapshot that had not been updated yet.
  **That is the whole contradiction in one line: the filename was always fresh and the field inside was always stale**,
  which is why 98 files were named `BRAKE_in_hold` while saying `ALLOW` inside.
* also from that header, a limitation the operator should know when relying on artifacts: if **webd is down** when the
  fault happens, the scene lives only on the socket stream and in controld's memory until replaced — "a station that
  needs the artifact guaranteed should run webd as a supervised service, not as something started by hand." My own
  webd has been started by hand all session, which is worth saying out loud.

Changed: the warning comment at the bridge boundary, and the wrong module pointer in the comment above it. Comment-only
— `py_compile` clean, **250 web tests pass**, no behaviour touched, station untouched. **487 pytest / 57 CTest** stand.
Nothing signed.

## 2026-09-06, 01:4x — round 81: the artifact that this whole safety story rests on is off by default, and the installer never turns it on

Round 80 read the writer's header and took away a caveat ("run webd supervised or the scene may be lost"). Round 81
checked the supervised path and found something worse: **`tools/install_station.py` never sets `OTA_BLACKBOX_DIR` on any
unit**, `web/webd/config.py` defaults `blackbox_dir` to `""` which means *disabled*, and no test in either suite asserts
anything about it. So a correctly installed, correctly supervised station — exactly what the writer's own documentation
recommends for guaranteed artifacts — **writes no §80 files at all**. The 98 scenes behind rounds 73–79 exist only
because webd has been started by hand with the variable set, on this machine, by me.

The default-off design is deliberate and the writer says why (a side effect should not arrive because code was merged),
so this is not a bug to revert; it is an unfinished installation. The fix is a one-line env in the webd unit — and it is
left to the operator, because it is an **unbounded write with no rotation and no delete path**: this session is **128
scenes, 108 KiB total, ~855 B each**, which is nothing for a day and a slow leak over a year, and the right answer
depends on the machine and on whether someone prunes them.

What bothers me, and is the reason this is written down rather than just fixed: a data-contract item (§80, "preserve a
scene that cannot be reconstructed") can be fully implemented — capture, publish, bridge, writer, tests — and still be
inert in production, because the one line that connects it to the outside world lives in an installer no test inspects.
Every green suite in this repo was true while the feature could never fire. That is the same shape as the round-78
coverage question (ctest entries are per-binary, so the count proves nothing) and the round-69 one (tests passed against
a script that could not run): **the suite tests the code, not the deployment.**

Verification: greps above; `install_station.py` has no `blackbox`/`BLACKBOX` occurrence at all; `config.py:56,119` hold
the empty default; `tools/tests/test_install_station.py` asserts unit contents elsewhere but nothing here. Docs only —
nothing enabled, no unit changed, no disk side effect introduced without a human deciding. Station untouched;
**487 pytest / 57 CTest** stand. Nothing signed.

### Correction, same round — the numbers I typed into the entry above are not the numbers the tool printed

Measured: **128 scenes, 107 KiB total, 856 B each.** The entry above says "108 KiB total, ~855 B each" — written from
reading the output rather than copying it. The commit message carries the measured string because that one was built
from the shell variable, which is the discipline that works: **let the measurement travel into the text, do not
retype it.** Immaterial to the finding; material to the habit, which is the entire subject of rounds 42, 54, 63 and
65 in this file.

## 2026-09-06, 02:1x — round 82: the switch that makes §80 exist in production, added as a comment

Round 81 found the artifact inert because nothing set `OTA_BLACKBOX_DIR`. The reason is now visible: the environment
lives in `systemd/turret-web.service`, whose `Environment=` lines the installer deliberately does not touch
(`render_unit` rewrites exactly four keys — root, ExecStart paths, User/Group — and its own comment says why: *"silently
rewriting an `After=` or a sandbox directive is how a deploy tool ends up owning someone's outage"*). So there was no
bug to fix in the installer; the unit template simply never carried the line.

`systemd/turret-web.service` now ends its environment block with a **commented**
`#Environment=OTA_BLACKBOX_DIR=/var/lib/ota/blackbox`, above a note stating: the default-off is deliberate; with no
path, controld still captures and webd still publishes the scene on every unsafe edge while **nothing reaches disk —
the feature is inert in production and every test stays green**; and the cost of flipping it — the writer has no
rotation and no delete path, ~856 B each, 128 in one probing session, directory must be writable by the unit's user.

Same shape as `tracking.hold_speed_deg_s` from round 43, on purpose: a safety- or disk-relevant choice stays the
operator's, but it is *written down where the choice is made*, not in a chat log.

Verified as far as I can without touching the machine: **84 tools tests pass**, which includes the 11 places
`test_install_station.py` asserts against the web unit — so the comment breaks no rendered-unit expectation. **I did
not run `install_station.py` itself**: its `--root`/`--stage` default to an install root, and staging into
`/opt/open_auto_turret` to satisfy my own curiosity is not what the operator asked for. The claim is therefore "the
template carries the switch, and the installer's tests still pass", not "a staged unit was inspected".

No behaviour changed anywhere: no env set, no directory created, no daemon restarted. Station untouched, source
running. **487 pytest / 57 CTest** stand. Nothing signed.

## 2026-09-06, 02:4x — round 82 follow-up: the stack died under an interrupted call, which is round 80's caveat happening for real

A tool call was interrupted mid-flight. Before retrying anything I looked: `git log` showed round 82's commit
`f5cfb13` already present, tree clean, nothing unpushed — so the interruption had lost only the result echo, not the
work, and no retry was needed (a blind retry would have created a duplicate commit). The same check turned up
something else: **`pgrep` returned zero for controld and zero for webd**, and visiond's log ended with its clean-shutdown
summary. The whole hand-started stack was gone. The interruption appears to have taken my process group with it,
despite `setsid nohup`.

Restored in order — controld (homed, `at_ready`), webd, then visiond as the controld-restart rule requires — and
verified rather than assumed: `MANUAL / HOLD`, `ready True`, `effective_speed_ceiling_deg_s 10`, `safety_action ALLOW`,
`control_deadline_misses 0`, `telemetry_stale False`, `vision_track_sets 448` and climbing, yaw 149.21° (unchanged
across the outage), and **`GET / → 200`** on the panel.

Round 80 recorded the black-box writer's warning that *"a station that needs the artifact guaranteed should run webd as
a supervised service, not as something started by hand."* Round 82 is that sentence becoming an event: a hand-started
stack stopped, and nothing was supervising it. Two honest consequences:
* this session's uptime claims were always about *processes I started*, not a service — worth remembering when reading
  any measurement above that says "the station has been running";
* the incident cost nothing measurable here (no motion was commanded; the axis held position and telemetry resumed at
  the same yaw), but the same loss during a tracking session would have meant a silent gap with nothing supervising or
  reporting it. **Running the units (`tools/install_station.py`) is the operator's call, and this is an argument for it
  that no test could have made.**

No code or config changed in this follow-up. Commits unchanged: `f5cfb13` (round 82), `5bcb4a0`, `e3e902c`. Suites:
**487 pytest** re-run green after the commit; **57 CTest** unchanged since round 78 (no C++ touched since).

## 2026-09-06, 03:2x — round 83: the operator reported the page flashing "telemetry lost", and it was the page lying about the station

Operator, back on the line: *"I can still see the web interface flashes from time to time due to telemetry lost."*
First rule applied — measure before touching:

* **Bridge and station are clean.** 19,335 samples of `/api/state` over 25 s: `telemetry_stale` true **zero** times,
  `telemetry_age_ms` p50 33 / max **66 ms**, `feedback_age_ms` max 48, `track_list_age_ms` max 30, zero fetch errors.
* **The push path is clean too.** Over `/ws`: **p50 66 ms, max 80 ms**, *while another client simultaneously drained
  1.25 MiB/s of MJPEG video*. Server-side, nothing ever approached the 500 ms the page uses.

So the flash was being **decided inside the browser**, and reading `hudStale` + `updateStaleness` showed exactly two
ways to do it while the station was healthy:

1. `ws.onclose` sets `transportOk = false` and reconnects after **1000 ms** — and the predicate treated a closed socket
   as §25 staleness **instantly**. One dropped socket = one second of "TELEMETRY STALE / DISCONNECTED", then retraction.
2. `msgAgeMs > 1500` on a **single** 250 ms tick. A main thread stalled decoding 1080p MJPEG simply stops running
   `onmessage` for as long as it likes, and one late tick was enough to accuse the station.

Fix at the caller, `updateStaleness`, leaving the pure predicate and its existing tests untouched: a closed socket
inside the quiet grace no longer asserts staleness (the reconnect is already in flight, and if the link really died the
message-age path says so within the same grace); and verdicts resting **only on the page's clock** need **two
consecutive** overdue ticks, while anything controld itself reported — `telemetry_stale`, or its own
`telemetry_age_ms > 500` — still acts on the first tick, because that is the station's own word and must never be
softened. A stalled thread drains its queued messages the moment it resumes and so can never produce two ticks in a
row; a dead link produces all of them. Cost: a genuine loss is declared one 250 ms tick later.

**The test could have failed, and did.** `web/webd/tests/test_stale_flash_is_debounced.py` runs the page's real
`updateStaleness` in node against a scripted clock — stub world only as wide as the surface it touches. Same harness,
both versions of the page:

| case | pre-fix | post-fix | wanted |
|---|---|---|---|
| socket closed, message 200 ms old | **true** ← the flash | false | false |
| 1600 ms silent, first tick | **true** ← the flash | false | false |
| second tick | true | true | true |
| controld says stale | true | true | true |
| link dead 4 s, first tick / second | true / true | false / **true** | one-tick delay, then declare |
| fresh message again | false | false | false |

Two harness bugs of my own on the way, both caught by running it instead of trusting it: a stub element that returned
itself for every id (the page wires docks and drawers at load, so `dock.addEventListener` blew up), and `window`
without `addEventListener`. Neither state produced a *passing* run, so neither could slip through as false evidence.

Live after restart: `GET / → 200`, the served page carries the new code (`linkOverdue` appears 4×), controld
`ready`, `telemetry_stale False`, `telemetry_age_ms 6`, `control_deadline_misses 0`, and `/ws` back to p50 66 / max
84 ms with the video pane running. **488 pytest** (487 + the new one); CTest 57 untouched, no C++ changed.

**Separate finding, not the flash:** the video stream delivers **11.7 fps** against 15 requested at 1080p
(~1.25 MiB/s per client, and that client was reading flat out). The pane will therefore stutter slightly even when
telemetry is perfect. Unattributed — capture/encode limited, or network — and it is a frame-rate claim I will not make
without measuring the camera's own delivered rate.

## 2026-09-06, 04:0x — round 83b: the panel said 15 fps; the station delivered 10 — plus two self-kills of my own, logged because they were avoidable

Operator's next directive landed mid-round (drive modes: no more fake targets, shaky motion, HOLD dead zone +
fixed-position, AUTO_TRACK↔AUTO_ROAM switching, roam sloppiness). Before pivoting I closed the edit I had half-landed.

**What the video-rate investigation turned out to be.** Not a stall, not the network, not my counting:
* publish was **9.99 fps idle** and **10.00 fps with a client draining ~1 MiB/s** — 200 frames in 20.0 s, twice;
* at **1280×720** it published **121 frames in 12 s ≈ 10.1 fps**, so it was not the requested resolution;
* `_min_publish_s` is `1/15`, so the cap was never binding, and `create_video_configuration(...)` is given a **size and
  no frame rate at all** — **15 fps was never asked of the camera**, yet `/api/video/state` reported `fps: 15.00`.
So a §20 field named `fps` was publishing a wish. `VideoState` now carries **`fps_published`**, measured over a short
trailing window (`measured_fps()`, window trimmed so it falls to 0 within seconds of the stream dying — otherwise a
frozen pane advertises the rate it had when healthy). `fps` stays as the request. Both are visible; neither is hidden.
Live after restart: **requested 15.0, measured 10.4**. Four tests in `test_video_reports_measured_rate.py`, one of which
exists to fail loudly if anyone re-merges the two meanings into one field.

**An earlier number of mine was wrong and the record says so**: I first reported "0.00 fps at 720p". That was an
artifact — `frames_published` resets on `start`, and I sampled across a restart. Reproduction gave 121 frames in 12 s.
Caught because I went back and reproduced the surprising number instead of shipping it.

**Two self-kills, both avoidable, both recorded.** `pkill -f "vision.visiond"` matched **my own shell's command line**,
which contained that literal string — the call SIGTERM'd itself. Then, to relaunch in fewer calls, I put a kill by
`webd[.]app` and the launch command `-m web.webd.app` in the **same** call: the pattern matched my own cmdline again.
The bracket trick only protects against the pattern's own text, not against the same command line also containing the
thing being matched. **Rule: a kill of a component and a launch of that component never share a call.** The first cost
was a missing verification line; the second left webd down for one call until I checked rather than assumed.

Station after: controld up, **synthetic source stopped and staying stopped** (operator item 1: no fake targets), panel
`200`, video running. **255 web tests** pass (250 + the staleness test from earlier this round + 4 video). Full suite
re-run follows with the drive-mode work.

**Pivoting now to drive modes** — the operator's five items, in their order: real detector path (fake targets are
already off, which also means the vision feed will legitimately go quiet until the real path runs), motion quality
research per mode, HOLD dead zone + fixed position, AUTO_TRACK↔AUTO_ROAM switching tests, and roam range/slop.

## 2026-09-06, 05:0x — round 2: the static axis is NOT shaky, and that changes what "fix shaky motion" means

Operator's item 2 says the motion is "very shaky … static and dynamic are not good". Before changing any control
law I measured the station at rest (no motion commanded, `AUTO_TRACK/WAIT_TARGET`), twice — a 12 s sample and a 6 s sample,
n=2477 here, ~4965 in the first. Numbers below were written into this file **by the script that measured them**, not
retyped.

* actual yaw: **p2p 0.0435 deg**, sigma **0.0160 deg**, **0 reversals**. The first sample agreed: p2p 0.0877°,
  sigma 0.0172°, 0 reversals.
* commanded reference: p2p **0.0435 deg**, sigma 0.0160 deg — identical to actual, so in this phase the
  reference is holding where it is rather than fighting.
* smallest non-zero reported step: **0.02177 deg** (median step 0.02234 deg). That is the resolution floor of
  the angle as it reaches telemetry.

**Three things follow, and they matter more than the number itself.**

1. **Zero reversals over 12 s is not jitter.** A control loop oscillating around a setpoint produces sign changes by
   the hundred. What is visible here is a sub-tenth-of-a-degree monotone slide — drift, not shakiness. So "static
   control is bad" is not supported by the encoder, and if I had started tuning gains I would have been fixing
   something that is not broken (and could have broken it).
2. **The deadband in item 3 has a hard floor given by measurement, not taste:** a deadband under **~0.0218 deg**
   cannot be distinguished from quantisation of the reported angle. Any sane static deadband must sit above the noise
   floor and below what the operator can see as an error — so the design range is roughly 0.022° … 1/3 of a box
   height's worth of angle, and I will size it from the *tracking* jitter distribution once a real target is in view,
   not from a guess.
3. **What the operator is watching is probably not the axis.** At this instant the vision source is the real IMX500
   with the classical blob detector, the HUD video pane cannot run at the same time (single-owner camera), and the
   preview I measured earlier publishes ~10 fps against 15 requested. A reticle that updates at detection rate over a
   10 fps picture *looks* shaky even when the axis is dead still. Item 2 therefore needs the two paths separated
   before anything is tuned: axis shakiness (encoder) versus picture-and-reticle shakiness (detection + preview).
   The encoder side is measured and quiet; the visual side is the live suspect, and the roam preview limit is already
   known to be inside `web/webd/video.py` rather than the sensor.

Nothing changed in code this round. This entry exists so that later control work is judged against a recorded floor
instead of a memory — and so that if the operator sees shakiness I cannot reproduce, the disagreement is on the record
with both measurements, rather than resolved by whoever spoke last.

### Correction, same round — my commit message for `988d6dd` contains a number that was never measured

The message says the second 6 s sample "agreed at 0.0866 deg". **It did not.** The script printed `p2p 0.0435`, and
0.0866 was composed from memory of the *first* sample (0.0877). The file above is correct because the script wrote its
own numbers — which is exactly why that rule exists, and the commit message was written by hand a moment later, from
recollection, in the same breath as I told myself I was done with that habit.

Correct pair of measurements, both with **0 reversals**: sample 1 (12 s) p2p **0.0877°**, sigma 0.0172°; sample 2 (6 s)
p2p **0.0435°**, sigma 0.0160°; resolution floor **0.02177°** in both. The conclusion is untouched — both samples are
sub-tenth-of-a-degree with no reversals — but the number in the commit message was mine and it was wrong, so it is
stated here rather than left standing. Rounds 54, 81 and 83 said the same thing about me; the fix is not resolve, it is
that the measurement writes the text.

## 2026-09-06, 05:3x — round 3: item 3 restated, because part of it already exists and one reading of it would be unsafe

Two facts found by reading, not assuming:

* **`mode_hold_in_place_` already implements "fixed position mode"** (`control_loop.cpp:2078-2090`). When the turret
  stops moving in Manual / AutoTrack / AutoRoam at Ready, the pose is latched from the measured joints **once** and held
  — the code's own words: *"no creep (it is not re-read every cycle) and no journey (it is not the ready pose)"*. So
  HOLD does not re-chase anything today.
* **There is no deadband or dead-zone anywhere in the control loop.** The `deadband` hits in `turret.yaml` are the
  axis's *friction* deadband (backlash context), not a vision deadband. `control_loop.hpp` has no such config key.

So the live gap is narrower and different from the wording: it is a **deadband on the vision-driven reference while a
target is being tracked**, not a change to HOLD.

**And a safety reading I will not implement without saying out loud.** "Disable the closed loop feedback and switch to
fixed position mode" can mean two very different things:

1. **Stop letting vision move the aim point** while the target sits inside a band — the encoder position servo stays
   closed, the supervisor stays armed, and what is sacrificed is only *re-aiming* at small target motion: a slow drift
   below the band is no longer followed, and the axis will not chase a target that genuinely crept 0.05°.
2. **Open the position loop** — stop servoing to the reference and merely hold a commanded position with the
   driver's own hold. Then gravity sag, backlash creep (this station has a *measured* friction deadband, which is why
   `yaw_park_deg` sits at 176°), and disturbance rejection are all no longer corrected.

I will implement **(1)**: a vision deadband with hysteresis (an enter threshold and a strictly larger release
threshold, so the band cannot chatter), as an explicit config key defaulting to **0 = disabled** so no behaviour
changes until you ask for it. If you actually meant (2), that is your call to make with the friction numbers in front
of you, not mine to slip in under the word "hold".

Sizing is no longer guesswork: the measured resolution floor of the angle reaching telemetry is **0.02177 deg**, and
the measured at-rest drift is sub-tenth-of-a-degree with **0 reversals**, so the band must sit above 0.02177° to be
meaningful at all; the upper end should stay well under the 1/3-box-height acceptance tolerance. The real number comes
from the *tracking* jitter distribution once a real target is in view on the real detector path.

Where it goes: `tracking_ref_` is read at `control_loop.cpp:841-842` and constrained at 843; the write path is through
the reference owner rather than a direct assignment in `control_loop.cpp` (greps for a direct assignment found none), so
the deadband belongs at the point where a measurement becomes a reference — which is the next thing to locate.

## 2026-09-06, 06:0x — round 4: the plumbing map for the vision deadband, so implementation is one clean pass and not a hunt

Discovery this round was expensive and mostly context, so the result is a map rather than code. Every site below was
read or grepped in this round, not recalled.

**Where the aim actually comes from.** `ControlLoop::build_mode_intent()` (`control_loop.cpp:601`, defined just after
line ~600 in the switch) does **not** compute the aim: for `AutoTrack` it defers entirely to **AutoTrackController**
(`at_out_.follow_los`, `at_input_.estimator_ready`) — the comment there says so explicitly: *"this answer comes from
AutoTrackController, not from the v1 FSM"*. `tracking_ref_` is then `ref_mgr_->resolve(last_intent_, …)`
(`control_loop.cpp:619`) and is **cleared at the top of every cycle** (`:399`; there is a comment at `:443` about a bug
that came from reading it in the wrong place). So a deadband on the *reference* would be applied after the aim is
already decided — the correct place is **before the intent is formed**.

**The controller is header-only:** `control/src/tracking/auto_track_controller.hpp` (no `.cpp` exists — the `ls` for one
returned nothing). It already owns prediction: `predicted_los_at_actuation()` is what feeds
`snap.predicted_target_az_world_rad` (`control_loop.cpp:1623`), alongside `estimator_initialized()`,
`prediction_horizon_ns()`, `target_az_rate_rad_s()`. **Lead/prediction already exists** — item 2's "must LEAD" is not a
missing feature, it is a tuning/measurement question.

**The four edits, in order:**
1. `control/src/config/turret_config.cpp` (~426, where `coast_ms` / `lost_hold_ms` / `reacquire_window_ms` are parsed
   via `opt_double(at, …)`) — add `deadband_deg` **and** `deadband_release_deg`, default **0.0** meaning disabled.
2. `control/src/control/control_loop.hpp` `Config` struct (near `search_span_rad`, `:158`) — the two fields.
3. `auto_track_controller.hpp` — apply the band where its aim output is formed: hold the last commanded LOS while
   |candidate − held| < enter; release (resume following) once it exceeds the strictly larger release threshold.
   Mutable state lives in the controller, not `ControlLoop`, so `reset()/start()` clears it with the session.
4. `control/tests/test_control_loop.cpp` (or a controller-level test) — in-band wobble must not move the aim; a step
   past the release threshold must; **and a test that the default of 0.0 leaves today's behaviour byte-identical**, so
   "explicit config choice, not a silent change" is enforced by a test rather than by promise.

**Why I stopped here rather than editing now:** four files plus a build-and-test cycle is not responsible work at the
end of an exhausted context — that is the shape of the round-63/65 damage in this file (patches that "compiled" while
applying nothing). A map that took one round to produce is cheaper than a bad edit that takes two to undo.

Sizing constraints recorded last round still stand: the reported angle quantises at **0.02177°**, at-rest drift is
sub-tenth-of-a-degree with **0 reversals**, so `deadband_deg` must be > 0.02177 to mean anything; final value from
tracking jitter on the real detector path, which still needs a target in view (blob detector reports nothing on a
still scene).

## 2026-09-06, 06:3x — round 5: item 4 is not broken, it was never built — and that makes it your decision, not mine

Operator asked me to *test* the automatic switching between AUTO_TRACK and AUTO_ROAM. Testing presupposes it exists.
It does not. Evidence, from greps in this round rather than recollection:

* The **only** way the operating mode changes is `ControlLoop::request_mode()` (`control_loop.cpp:1894`, declared
  `control_loop.hpp:226`), and its callers are: `:2568` the external **`set_mode`** command, `:2590` a developer/test
  path, and `:2565` a fallback to **Manual** (safety). **Nothing inside the control thread ever passes
  `OperatingMode::AutoRoam` to it.**
* What happens today when a target is lost *stays inside AUTO_TRACK*: `lost_hold_ms` / `reacquire_window_ms` are handed
  to the AutoTrack controller (`:1755`), producing the `LOST_HOLD` phase and a reacquire window — hold and look again,
  not go hunting. The `AutoRoam` references elsewhere (`:480`, `:1413`, `:2041`, `:2049`, `:2056`) all presuppose the
  mode is *already selected*.
* Round 4 already surfaced the intent in words: *"The v1 FSM can no longer cause motion in this mode… **AUTO_ROAM owns
  roaming, and the operator owns AUTO_ROAM**."* This is deliberate architecture, consistent with §111.5 — not an
  oversight I tripped over.

So the requested behaviour — turret starts sweeping on its own when it loses the target, and hands back when it finds
it — is **new autonomy**. It is the first thing in this system that would make the machine decide to move without an
operator command, and the round-4 architecture comment is the system saying it refuses to do that by design. I will not
add it quietly under "testing".

**What it would take, if you want it** (design sketched, not built):
* an opt-in key, e.g. `v3.auto_track.auto_roam_on_loss_ms` (default **0 = off**), so today's behaviour is byte-identical
  until asked — the same rule as the deadband, enforced by a test that 0 changes nothing;
* a delay distinct from `lost_hold_ms` (2000 ms today) and from `reacquire_window_ms` (3000 ms): roaming while the
  reacquire memory is alive would fight the reacquire logic;
* **hysteresis both ways**, because the failure mode is a hunt: lost→roam after a long delay, roam→track only on a
  confirmation streak (N detections, or confidence ≥ the existing medium band), so a two-frame ghost cannot pull the
  mode back;
* roam **restricted to the region the operator named**, never widened by the automatic path — §33/§72's asymmetry
  (naming narrows, never widens) must survive this feature;
* a visible mode-change event with the reason, through §79 like the existing `ROAM_*` events, so the operator always
  knows the machine took the initiative, and a rule that any safety Brake **ends** the automatic arrangement until the
  operator re-arms it.

**Question for the operator, and it is genuinely yours to answer:** should the turret begin sweeping by itself when the
target is lost? Some people want that; some consider an unmanned search on a live mount exactly what they do not want
a machine to decide. My job here is to make sure it is not added as a side effect of a bug hunt.

Nothing changed in code this round. Blocked-with-decision items right now: this one; the picamera2 AI API for the `rpk`
detector (item 1); and a physical target in view, without which neither the deadband sizing (item 3) nor any switching
or roam measurement (items 4, 5) can be taken on real pixels.

## 2026-09-06, 07:0x — round 6: I fixed the thing that was not broken, and the measurement said so before I committed the claim

Hypothesis was specific and cheap to test: `vision/frame_source.py:314` sets `cfg["controls"]["FrameRate"]`, while
`web/webd/video.py` never named a frame rate at all — so the preview's sensor mode was chosen by size alone, and that
was my explanation for 10 fps against 15 requested. Applied the same request in `video.py`, guarded the same way the
vision path guards it, `py_compile` clean, **255 web tests pass**, restarted the daemon, measured 20 s.

**Result: 195 frames in 20.0 s = 9.75 fps.** Before the change it was 200 in 20.0 s = 10.00 fps. Within noise of
unchanged. **The hypothesis is dead**, and the only thing that would have been worse than measuring it is writing
"fixed the video frame rate" in a commit message on the strength of a plausible mechanism.

The change itself is harmless and arguably more correct (the file now asks the sensor for the rate it was given, which
is what it always should have done), so it stays — but it stays **labelled as not the fix**, in this file, not as a
win.

Surviving hypotheses, for the next pass, with the cheap test for each:
* **Request held across the encode.** `video.py`'s loop does `capture_request()` → encode → `release()`, and the code's
  own comment says holding one request starves the pipeline. If encode is ~36 ms at 1080p on top of a ~64.5 ms frame
  period, the delivered rate lands at 1/(T_frame + T_encode) ≈ **9.7** — which is uncomfortably close to both measured
  numbers. Test: time the encode around the release, or release before encoding from a copied buffer.
* **Different mode than the vision path.** Vision's 15.5 fps may come from a different size/format request entirely,
  not from the FrameRate control. Test: ask the camera what mode each path actually negotiated.
* **A single-threaded pull loop** that cannot overlap pull and encode.

Two of those predict the same ~10 fps, and they need different fixes, so the next measurement must be encode duration,
not another guess. That is the pattern this session keeps teaching: the plausible mechanism is the most dangerous
thing I have, because it reads like an explanation.

## 2026-09-06, 07:3x — round 7: encode-holding is dead too, killed by the 720p column of the same measurement

I predicted last round that if the capture request were held across the encode, encode ≈36 ms on a 64.5 ms frame period
would explain the 9.7 fps. Measured the encode directly (`array → PIL → JPEG`, median of 12), no daemon restart needed:

* **1080p: 21.1 ms** median, **720p: 9.2 ms** median (2.3× cheaper).
* Serialised-rate prediction 1/(T_frame + encode): **11.67 fps** at 1080p, **13.57 fps** at 720p.
* Measured delivery: **9.75–10.00 fps at 1080p** and **~10.1 fps at 720p**.

The 720p column is what kills it. If holding the request across the encode were the binding term, making the encode
2.3× cheaper must move the delivered rate — it would have to go to ~13.6 fps — and it did not move at all. 1080p alone
could still be argued (the prediction, 11.67, is already above the measured 9.75, so encode does not even account for
it), but a mechanism that predicts a change that does not happen is simply wrong.

Caveat, stated because it cuts against the hypothesis further rather than for it: those encodes used **random-noise
images**, which are worst-case JPEG content (1352 KiB/frame against ~105 KiB for real frames here). Real 1080p encodes
are therefore *faster* than 21 ms, which pushes the serialised prediction closer to 15 fps and makes encode explain
**less**, not more.

**Two plausible mechanisms falsified in two rounds** — "the sensor never gets a rate request" (round 6: 9.75 fps after
requesting it) and "the request is held across the encode" (this round). Both were specific, both were defensible, both
were wrong, and both were killed by a measurement that cost one call each. The surviving candidate is the one I flagged
as merely plausible and refused to bank: the **turnaround of `capture_request()` itself** in this single-in-flight
configuration (`buffer_count=3`, request → `make_array` → rotate_180 → encode → `release`), which would fix the ceiling
near 100 ms *regardless of frame size* — the only observed behaviour that matches both resolutions landing on ~10 fps.

**The decisive probe, and it needs the camera:** free the IMX500 and ask picamera2/libcamera what sensor mode and rate
are actually negotiated for a bare `create_video_configuration(main={"size": …}, buffer_count=3)` versus what
`vision/frame_source.py` negotiates (it reports **14.97–15.54 fps** from the same sensor). If the rates differ at the
mode level, the preview fix is to use the same configuration path as vision — not another guess at a control name. That
is next round's single measurement.

No code changed this round. The station is untouched: real IMX500 source still running (`camera_fps 14.9656`), controld
`AUTO_TRACK / WAIT_TARGET`, `safety_action ALLOW`.

## 2026-09-06, 08:0x — round 8: the preview's 10 fps has a mechanism now, and it exposes that my round-6 "fix" was a silent no-op

Freed the IMX500 and asked it directly (system python, libcamera chatter suppressed):

* **The sensor offers two modes: `(2028,1520) @ 30 fps` and `(4056,3040) @ 10 fps`.**
* `video.py`'s bare `create_video_configuration(main={"size": (1920,1080)}, buffer_count=3)` negotiates
  `XBGR8888`, and its advertised controls are **`FrameDurationLimits`, `NoiseReductionMode` — no `FrameRate` at all**.
* The vision path (`vision/frame_source.py:309-312`) asks differently: `main={"size": …, "format": "XRGB8888"}`,
  `buffer_count=4`, *then* sets `controls["FrameRate"]` — and delivers ~15 fps from the same sensor.

**Two conclusions, one of them about me.**

1. **Round 6's change could not have done anything, and my own guard hid the evidence.** `FrameRate` is not among the
   controls that configuration advertises, so assigning it was a no-op — inside a `try/except Exception: pass`, which
   ate any signal that the control was unknown. I wrote a defensive guard that converted a failed change into a
   successful-looking one, then measured the result and correctly reported "no change", while the *reason* was sitting
   in the code I had just written. That is the round-63/65 family of failure (patches that apply nothing) arriving by a
   new route: not a broken anchor but a **silently-ignored write**. The measurement caught the outcome; the guard
   would have let a less honest round book a win.
2. **The rate is chosen at mode-selection time, not by a control.** 1920x1080 fits inside the 30 fps mode's
   2028x1520, yet both 1080p and 720p landed on ~10 fps — matching the `(4056,3040) @ 10 fps` mode. Two resolutions
   pinned to one number, independent of encode cost (round 7), which is exactly what mode selection predicts and what
   no per-frame cost could produce.

**The fix, evidence-first:** ask for the preview the way the working path asks — **name the pixel format explicitly**
(`XRGB8888`, as `frame_source.py` does) so the fast mode can be selected, and express any rate request through a control
the configuration actually advertises (`FrameDurationLimits`, in microseconds), never `FrameRate`, which this pipeline
does not expose. Then `fps_published` decides whether it worked — it is the arbiter that stopped me booking two false
wins in this file, and it stays the arbiter.

Station restored after the probe: real IMX500 source running again, controld untouched (`AUTO_TRACK / WAIT_TARGET`,
`safety_action ALLOW`). The preview pane is intentionally still stopped; nothing is left holding the camera.

## 2026-09-06, 08:3x — round 9: third null result, and this one the change definitely applied, which makes it informative

Applied the round-8 plan: `main={"size": …, "format": "XRGB8888"}` (the form the working vision path uses) and
`FrameDurationLimits = (dur_us, dur_us)` instead of the `FrameRate` control the probe showed is not advertised — with
the `except` now **printing** instead of swallowing, because round 6 proved a silent guard is how a no-op becomes a
false win. `py_compile` clean, **255 web tests pass**.

**Measured after restart: 190 frames in 20.0 s = 9.50 fps** (state-measured `fps_published 9.60`), against 9.75–10.00
before. **Unchanged again.** And unlike round 6, the change is verifiably in effect: `pixel_format` now reports
**`XRGB8888`** (it was `XBGR8888`), and the rate-request failure log has **0** lines — so the control was accepted, or
at least not rejected, and the rate still did not move.

Third plausible mechanism, third null: (1) sensor never asked for a rate — no effect; (2) request held across the
encode — killed by the 720p column; (3) format naming selects the fast mode — **no effect, proven to have applied**.

**What the three nulls jointly say:** nothing inside the per-frame path (rate request, format, encode cost) moves this
number, across two resolutions and two pixel formats. What has never been measured is the one quantity that would
separate the remaining candidates: **the interval between completed capture requests as the sensor delivers them**,
independent of anything we do with the frame afterwards. If requests arrive ~100 ms apart, the ceiling is upstream of
this file entirely (mode selection at libcamera level, or the single-in-flight `capture_request()` pattern), and the
answer is to mirror `frame_source.py`'s *asynchronous* request discipline rather than its config arguments. If they
arrive ~64 ms apart and only 10 fps is published, the loss is inside `_encode_request`'s full per-frame work —
`make_array` of an 8 MB XRGB frame plus the `rotate_180` flip — which round 7 measured only for the JPEG step.

That is next round's single measurement: record arrival intervals of completed requests and expose them as
`sensor_fps`, so `fps_published` and `sensor_fps` together say where the frames go. Two numbers, one verdict. I am not
guessing a fourth time when one instrumentation line answers it.

Also honest about a check that did NOT pass: `colour_check` came back **"inconclusive: the first frame had almost no
colour in it, which cannot tell one channel order from another"** — because the room is static and dim. So I am *not*
claiming the format change is colour-safe; the module's own verification refused to certify it. Either the fast mode has
to be revisited with a real scene in view, or the change stays only if the operator accepts an unverified colour path —
which is their call, not something to bury in a commit.

Station: real IMX500 vision source restored and running; controld untouched; preview left as configured.

## 2026-09-06, 09:0x — round 10: verdict at last, three rounds after I started guessing — the camera gives 15.00, we publish 10.00

Instrumented the one quantity nobody had measured, then restarted and read it:

* **`sensor_fps` (completed requests arriving, before any processing): 15.00**
* **`fps_published` (after our per-frame work): 10.00**
* `pixel_format: 'XBGR8888'` — the unverified format change is **reverted** (it bought nothing measurable and
  `colour_check` refused to certify it, so it went back rather than staying as an unowned risk).

**That settles the argument three rounds should have ended with numbers instead of rhetoric.** Rounds 6, 7 and 9 each
applied a plausible mechanism and each moved nothing; the joint conclusion I wrote last round — that nothing in the
per-frame path could explain it — was *half right*: nothing in the per-frame path explains it **in the way I proposed**,
but the loss is nevertheless **inside our per-frame work**, at ~100 ms of it, because the arrival rate proves the
sensor is not the limit. The camera sends 15; we hand the browsers 10. A third of the picture is thrown away by this
file, which is exactly the kind of thing that reads as "the motion is shaky" to someone watching the pane while the
encoder next to it is dead still (round 2's 0.0877°/0-reversals).

Budget of that ~100 ms, from what is already measured: JPEG encode ≈ **21 ms** at 1080p on *worst-case* noise content
(real frames are ~105 KiB against 1352 KiB, so less), leaving roughly **70-80 ms unaccounted** in
`request.make_array()` on an 8 MB XRGB/XBGR frame plus the `rotate_180` orientation flip and the white-balance path —
all of it on the critical path of a single pull loop that also holds the request open.

Next measurement, and it is the same shape as this one: time the phases around the loop and expose them, so the fix
attacks the 70-80 ms rather than the 21 ms. Candidate fixes then rank themselves — release the request before encoding
(from a copied buffer), skip work rather than whole frames when behind, or lift the decode off the pull thread. None of
them is worth writing before the phase timings say which number to attack; I have already spent three rounds writing
plausible ones.

Also worth keeping: `sensor_fps` vs `fps_published` is now a permanent pair on `/api/video/state`. It is the kind of
instrument that would have made rounds 6-9 unnecessary, and it costs two list appends per frame.

Station: real IMX500 vision source restored; controld untouched; preview running at the reverted (verified) colour path.

## 2026-09-06, 09:3x — round 11: the missing 70–80 ms now has names and prices, measured for 5 ms of daemon churn avoided

Round 10 proved 15 arriving / 10 published and blamed "the rest of the per-frame work" — true but not actionable.
Instead of instrumenting the loop and doing another kill-restart-measure cycle, I priced the operations directly on
this machine (1920×1080×4 `uint8`, median of 10):

| operation | cost |
|---|---|
| `np.rot90(arr, 2, axes=(0,1))` — **view** | **0.0 ms** |
| that view + `ascontiguousarray` (the copy) | **18.0 ms** |
| horizontal flip + contiguous | 18.0 ms |
| BGR→RGB as a slice (`arr[:,:, [2,1,0]]`) | 5.0 ms |
| BGR→RGB slice + contiguous copy | 20.3 ms |
| `np.asarray(..., dtype=np.int16)` of a 1080p RGB view | **15.3 ms** |

Plus what was already measured: JPEG encode **≈21 ms** at 1080p (worst-case content; real frames ~105 KiB vs 1352 KiB,
so less), and `request.make_array()` handing back an 8 MB array.

**So the ~100 ms is accounted for without guessing at all**: an 8 MB copy in, a rotate copy (18 ms), a channel-order
copy (20 ms), an int16 conversion (15 ms), and an encode (~21 ms) — each individually plausible, summing to the deficit
between 15 in and 10 out. The fix list, ranked by measured price rather than by taste:

1. **Stop paying 15.3 ms for `int16` when white balance is off.** The state has reported `white_balance: "off"` all
   session ("the sensor feed is already neutral once the BGRX byte order is decoded correctly"). If the conversion runs
   regardless, that is 15 ms per frame bought for arithmetic nobody uses. *Check before changing — I have been wrong
   three rounds running on this file.*
2. **Fold rotate and channel-reorder into ONE copy instead of two.** Two full-frame copies cost 18 + 20 = 38 ms; one
   fused copy costs ~20. A `rot90` view is free; it is the materialisation that costs, so materialise once.
   `PIL.Image.transpose(ROTATE_180)` is a candidate since it is C code over the buffer we are about to encode anyway.
3. Only after those: move the work off the pull thread, or release the request before processing from a copy.

Two things to hold onto. First, this is the first explanation in this investigation that is assembled entirely from
measured prices rather than from a mechanism I found persuasive — rounds 6, 7 and 9 each had a story and each moved
nothing. Second, `sensor_fps` vs `fps_published` is already in place to *judge* the fix, so nothing on this list gets
booked as a win until those two numbers converge.

No code changed this round (no instrumentation edits were needed once the direct pricing answered it — the two brackets
I had started to place were never written, because the assert-before-write shape meant a failed anchor wrote nothing).

## 2026-09-06, 10:0x — round 12: AUTO_ROAM measured on the real axis — and round 1's hypothesis confirmed to 0.3 degrees

Entered AUTO_ROAM through `/api/command` (`{"command":"set_mode","arg":"AUTO_ROAM"}`), sampled at ~10 Hz for 55 s
(536 samples), with a `finally` that stops motion and restores the mode no matter how the script dies. Raw rows:
`/tmp/r88_roam.jsonl`.

* **Sweep: 108.82 deg to 188.77 deg, span 79.95 deg, centre 148.79 deg.**
* **Reference rate: median 10.00, p95 10.00, max 10.00 deg/s** — constant to telemetry resolution.
* **7 turnarounds**, phases `AUTO_ROAM/SWEEP` and `AUTO_ROAM/TURNAROUND`, **safety `ALLOW` throughout, 0 deadline misses**.

**Round 1's hypothesis — "the sweep is centred on wherever the station became ready ± the 45 deg default span, not on a
named sector" — is confirmed: centre 148.79 deg against a ready yaw measured at 149.07/149.21 deg earlier this
session, i.e. 0.3 deg apart.** The observed extremes sit ~4.8 deg inside the low bound and ~5.2 deg inside the high
bound of ready±45, which is what the braking inset (soft margin + stop margin) predicts, so the envelope is behaving,
not misbehaving.

**What that means for "the roam range seems strange":** the range is *not* wrong — it is the **default**, and it is an
accident of where the station happened to home and become ready. The `auto_roam: yaw_min_deg 100 / yaw_max_deg 190`
sector in `turret.yaml` is **commented documentation** (round 1), so nothing names a region and `search_span_rad = 45 deg`
governs. It lands near that documented sector only because the ready yaw sits near its middle. If the operator wants a
specific sector, it must be **written uncommented into the config** — and per §72 naming can only narrow the sweep,
never widen it, so it is safe to ask for.

**And for "the motion seems sloppy": the sweep speed is not the culprit** — 10.00 deg/s flat with zero deadline misses is
about as clean as telemetry can show. What is left, in order of likelihood now: the **turnaround behaviour** (7
reversals in 55 s with a 0.25 s dwell, so the turret is turning around every ~8 s, which reads as restless), the
**preview stutter** measured at 10 fps of 15 (rounds 6–11, the axis being 0.0877 deg/0 reversals in round 2), or the
pitch axis during the sweep, which I did not sample this run and will not claim about.

One honest gap: my restore sequence left the station in `MANUAL / HOLD` on the first attempt and the mode was set back
to `AUTO_TRACK` afterwards — the check above shows where it actually ended up, and "restore the mode" belongs in the
same `finally` as the measurement, which is where it now is.

## 2026-09-06, 10:3x — round 13: the deadband hook is now located to the line, and I am recording that instead of pretending I shipped the feature

Two discovery attempts landed on the wrong switch (there are several `case OperatingMode::AutoTrack` sites in
`control_loop.cpp` — a bare grep keeps hitting the hold-pose latching helper near :2087). Located it properly with a
function-scoped search: **`build_mode_intent()` begins at `control_loop.cpp:2132` and its AutoTrack case at :2149.**

The hook is **lines 2164–2170**, precisely:

```
2164      double az = 0.0, el = 0.0;
2165      tracking_->predicted_los_at_actuation(az, el);  // valid: estimator_ready
        <<< the deadband goes HERE >>>
2166      in.source = MotionSource::AutoTrack;
2167      in.type   = IntentType::LosDirection;
2168      in.has_los = true;
2169      in.los_az_rad = az;
2170      in.los_el_rad = el;
```

**Two facts corrected this round, both worth keeping:**

* `AutoTrackController` does **not** produce the aim. `AutoTrackOutput` (`auto_track_controller.hpp:117-126`) carries
  `follow_los` (a *permission*), `velocity_scale`, confidence, band, reason — the aim is the **predicted LOS**, obtained
  at :2165 and handed to the resolver as an `IntentType::LosDirection`. So a deadband belongs in `control_loop.cpp` at
  the hook above, not in the controller. That is the third different place I have guessed for this feature across
  rounds 3-13 (reference → controller → intent), which is exactly why I stopped editing and located it.
* Round 3's reading is confirmed at :2074-2090 again: HOLD latches *the last commanded reference*, not the measured
  feedback — "so the hold target does not wander with encoder noise" — and the comment there records why it matters: a
  STOP MOTION 14 deg into a sweep used to move the turret 14 deg on the way to the ready pose.

**The change, ready to apply, sized so it cannot change anything until asked** (5 files, one edit each):

1. `control_loop.hpp` Config: `double auto_track_deadband_deg = 0.0; double auto_track_deadband_release_deg = 0.0;`
2. `turret_config.cpp` (~426, beside `coast_ms`): parse both, default 0.
3. `control_loop.cpp` :2164 hook, holding the **last accepted LOS** in two new mutable members:
   ```cpp
   // deadband: while |delta| < release, keep pointing where we already point.
   // deadband_deg == 0  =>  pass az/el straight through (today's behaviour, exactly).
   // re-anchor whenever the target leaves the release threshold, so a real move is followed
   // at full authority; hold while it is inside, so detector jitter cannot walk the aim.
   ```
   Enter and release must be distinct, release >= enter; `release == 0` with `deadband > 0` should clamp to
   `1.5 x deadband` and log once — an accidental config where enter > release must not silently disable holding.
4. A gtest: in-band wobble does not move `in.los_az_rad` across N cycles; a step past release does, and the
   re-anchored aim equals the new target (not a partial move); **and `deadband_deg == 0` reproduces the current
   numbers**, which is the test that makes "explicit opt-in, not a silent change" enforced rather than promised.
5. Sizing floor from round 2: **0.02177 deg** quantisation, so anything below that is indistinguishable; suggested
   first real value 0.1 deg with release 0.15 deg, to be revised once a target is in view on the real detector.

**What stopped being corrected, for the operator's benefit, when this is enabled:** the aim stops re-centring on small
target motion, so a slow drift below the release threshold is not followed, and the reticle will sit slightly off a
target that crept. The **encoder position loop stays closed and the supervisor stays armed** — the axis keeps servoing
to the held aim as hard as it does now; only the *re-aiming* is damped. That is the (1) reading from round 3, not the
(2) reading that would open the position loop.

I did not apply it this round: five files plus a ninja/ctest cycle is not something I can carry honestly at the end of
this context, and "compiled cleanly" has twice in this file meant "applied nothing". This entry makes the next round one
edit-and-verify, not another hunt.

## 2026-09-06, 11:0x — round 14: item 3's aim deadband is IN, default-off, 4 new tests, CTest 57/57 — one round, one feature, as promised

Round 13's located hook made this a wiring job rather than a hunt. **One scope, no other thread opened.**

**New file `control/src/control/aim_deadband.hpp`** — a 60-line `struct AimDeadband` with the hysteresis stated in the
header prose: deviation past `enter` **enters** the hold, past `release` **releases** it (two thresholds because one
chatters at its own boundary); `enter == 0` returns the candidate untouched and clears state; an inverted pair is
clamped to 1.5×enter with `config_clamped` set so the caller can report it — because silently correcting a
safety-relevant number is the habit this project has already been burned by.

**Wired in 6 places, each anchor verified to match exactly once before writing:**
`control_loop.hpp` (include, 2 `Config` fields, `aim_hold_` member, clamp flag) · `control_loop.cpp` at the hook round
13 pinned (`predicted_los_at_actuation` → `los_az_rad`), guarded so the `0` branch only *resets* state ·
`turret_config.hpp` (`V3Config` fields) · `turret_config.cpp` (parse `deadband_deg` / `deadband_release_deg`, default 0)
· `station_wiring.cpp:116` (the `V3Config` → `ControlLoop::Config` copy that round 13 had not found, which is what the
first build failure was really telling me: `V3Config` and `ControlLoop::Config` are two different structs).

**Measured results:**
* `AimDeadband` gtest: **4/4 pass** — disabled passes through and never arms; five in-band wobbles (±0.09° at
  enter 0.10°) leave the aim **exactly** the anchor value; a 0.30° departure is followed **fully** (1.30 in → 1.30 out,
  no partial move) and re-holds about the new anchor; an inverted pair still holds and says so.
* **CTest: 57/57 pass.** `controld` target builds clean.
* No Python touched, so the 255-test web suite is unchanged (stated, not assumed).

**`config/turret.yaml` carries the keys as commented documentation** — `# deadband_deg: 0.1`,
`# deadband_release_deg: 0.15` — with the honest cost spelled out next to them: a target creeping slower than the
release threshold is **not** followed, so the reticle can sit slightly off-centre on it. **Nothing is enabled by
default**, and `DisabledByDefaultPassesTheAimThroughUntouched` is what enforces that, not the comment.

**Deliberate non-action:** I did **not** restart the live `controld` for this. With the keys at 0 the change is inert by
test, and bouncing a working daemon to prove an inert change trades real risk for a cosmetic one. The operator enables
it by uncommenting two lines and restarting, at a time they choose — which is also the point of item 3 being *"an
explicit config/mode choice rather than a silent behaviour change."*

**Sizing, still open on purpose:** the measured floor is **0.02177°** (round 2's LSB) and at-rest wobble is
**0.0877° p2p with 0 reversals**, so 0.1/0.15 sits just above the noise. That is a *starting* value, not a tuned one —
tuning it against real detector jitter needs a target in view on the real detector, which is still the operator's
unblock.

## 2026-09-06, 11:3x — round 15: item 4's automatic hand-off implemented (opt-in, default never), controld green, CTest 57/57

**What exists now:** `ControlLoop::evaluate_auto_switch(now_ns)`, called at `control_loop.cpp:604` — *after*
`at_out_ = autotrack_.update(...)` (:567) and *before* `last_intent_ = build_mode_intent(now_ns)` (:605) — so a switch
that happens takes effect **this** cycle, and the state it reads is the current cycle's, not a stale one. That ordering
was checked rather than assumed, because a one-cycle-stale read is invisible in unit tests and shows up on hardware.

Two directions, both **0 = never** (shipped default = operator-only mode changes):
* `roam_on_loss_ms` — AUTO_TRACK → AUTO_ROAM once `at_out_.state == AutoTrackState::LostHold` (§20.2, prediction
  stopped, holding) has persisted that long. **Not** Acquire/Coasting: leaving mid-coast would abandon a live track.
* `track_on_acquire_ms` — AUTO_ROAM → AUTO_TRACK once `has_selection && estimator_ready` has persisted that long: the
  same evidence AUTO_TRACK itself requires before it will move, so a hand-off can't land in a mode that then refuses to
  drive.
* **Anti-hunt reuses an existing tuned number:** after any automatic switch, no further one for a full
  `reacquire_window_ms` (default 3000). Timers count from when the condition **first** became true — a timer refreshed
  on every sighting can never expire, which is the classic way this feature silently stops working. Only from
  supervisory `Ready`; the supervisor still outranks every switch.

**Two errors the compiler caught in my own assumptions, both recorded because they are the kind I keep making:**
1. I wrote `tracking_->state()`. `tracking_` is a `std::unique_ptr<TrackingController>` — the **retired v1** tracker.
   The v3 session's state is the controller's published output, `at_out_.state`, and the enumerator is **`LostHold`**,
   not `Lost`. Asking the retired FSM a question about v3 is exactly the mistake §111.5 exists to prevent, and the
   compiler was less polite but more reliable than my notes.
2. I gave `V3Config` the names `auto_track_roam_on_loss_ms…` and `ControlLoop::Config` the names
   `auto_roam_on_loss_ms…`. The wiring assigned one to the other, and the error named the mismatch immediately.
   Two structs, two naming habits — worth remembering before the next key.

**Verified:** `controld` links; **CTest 57/57**; documented commented-out in `config/turret.yaml`.

**Not done, stated plainly:** there is **no test for the watcher yet** (only the compile-time guarantee that both keys
default to 0 and the function returns at once). A meaningful test needs a homed loop plus a driven detection sequence,
and I would rather write it properly next round than claim a switch-latency number I have not measured — the objective
asks for *measured* latency and hysteresis, and a hardware measurement of a switch needs a target in view, which is
still the operator's unblock. The switch is therefore **implemented but unaccepted**.

For the record, the ingest region the acquire-direction depends on, quoted verbatim rather than paraphrased:
```
537: // vision has gone quiet and the operator clears the target — the exact situation in
538: // which they would do it. The selection is controld's own state and it has already
539: // changed, but `at_input_` still holds the last frame's answer, in which somebody
540: // *was* selected. Before this, CLEAR_TARGET during a vision dropout left the turret
541: // aiming along the last line of sight it was given, and the only thing that would
542: // stop it was another frame. Replaying a silent session is what showed it: no
543: // detector traffic, so nothing ever told the controller the truth had changed.
544: //
545: // `just_reacquired` stays in the frame path on purpose. A reacquisition is an event
546: // in the detector stream; a cycle with no frames cannot produce a new one, and
547: // synthesising one from an age transition would be §58's forbidden fabrication.
548: {
549: const auto& sel = selection_.selection();
550: const bool visible = sel.visibility_state == tracks::Visibility::Visible;
551: at_input_.has_selection = sel.has_selection;
552: at_input_.target_visible = visible;
553: at_input_.target_occluded = sel.visibility_state == tracks::Visibility::Occluded;
```

## 2026-09-06, 12:0x — round 15b: the roam-side hand-off gate was relaxed, and the way I found out is the instructive part

Two follow-ups to `504b68f`, both caught by measurement rather than by reading:

**1. The gate relaxed on evidence, not on hope.** The roam→track condition was
`has_selection && estimator_ready`, and `estimator_ready` is
`tracking_ && tracking_->estimator_initialized()` (:526) — it depends on the **v1 estimator session existing**. I could
not locate `request_mode`'s definition inside this round's budget to confirm whether that session survives a hand-off
into roam, so the condition might have been **unreachable while roaming**: a feature that silently never fires. Safety
does not live on that line — after any hand-off, AUTO_TRACK itself still refuses to move until its estimator is ready
(:2165 returns `Hold`) — so the condition is now `has_selection` alone, with the reasoning written in place of the old
comment. Worst case is a mode change that then **holds**: visible, and never motion nobody asked for.

That also retracts the claim I shipped an hour ago — that the hand-off demands "the same evidence AUTO_TRACK itself
demands". Unverified, and probably wrong. **Retracted.**

**2. I broke the build with my own comment edit and CTest said nothing.** Replacing the comment *and* the
`const bool held = …` line in one `old_string` deleted the declaration; `control_loop.cpp:2625` then failed to compile.
What matters is that **`ctest` reported `100% tests passed, 0 failed` in the same command that reported the build
error** — because the test binary was never rebuilt and the old objects ran happily. A green suite next to a failing
build is a green suite built from stale objects. Verified properly this time: `controld` links, `test_control_loop`
rebuilt with **0 errors**, and only then **CTest 57/57**.

Standing habit worth keeping: after any edit, the build must be *seen* to recompile the changed file (I now `touch` the
file when a build looks suspiciously quiet), and `ctest` only counts once its binaries are known-fresh.

## 2026-09-06, 12:3x — round 16: item 2's research gate, done in the open — and the answer is per-mode, not one profile

Sixteen rounds in, the operator's instruction *"do your research, including searching online and manual to see what is
the best moving mode for each mode"* gets an answer. Sources are cited so every claim below can be checked rather than
trusted. **What they do NOT do is substitute for measurement on this station** — the numbers still have to come from the
axis.

**Finding 1 — trapezoidal has an acceleration discontinuity; jerk-limited profiles exist to remove it.**
A trapezoidal velocity profile "always accelerates or decelerates at the maximum defined acceleration rate", which means
acceleration steps from 0 to full in one sample: finite acceleration, **unbounded jerk at the corners**
([US20130307459A1](https://patents.google.com/patent/US20130307459A1/en#5)). Jerk is what you feel as a jolt and what
excites a compliant structure. The 7-segment S-curve ramps acceleration up and down instead of stepping it
([Frontiers, improved S-curve a/de algorithm](https://www.frontiersin.org/journals/mechanical-engineering/articles/10.3389/fmech.2026.1786455/full#3#3#2)),
at the price of a longer transition for the same peak acceleration — you trade sharpness for smoothness, which is why it
is a choice and not a default upgrade
([Technosoft on S-curve for high-inertia loads](https://technosoftmotion.com/en/the-s-curve-profile-available-in-technosoft-drives-allows-smooth-starting-and-stopping-of-high-inertia-loads/),
[motion-profile guidance](http://machinebuilding.net/mastering-motion-profiles-to-boost-precision-and-throughput),
[min-jerk trajectory generation](https://www.crowdsupply.com/iq-motion-control/iq-fortiq-bls42/updates/the-iq-minimum-jerk-trajectory-generator)).

**Finding 2 — for a *vision-following* axis, the limit is the camera's delay and sample rate, not the profile.**
Slow sampling and camera delay are the stated central problem of feature-based visual servoing
([Okayama Univ.](https://ousar.lib.okayama-u.ac.jp/en/search/p/9?all=sensor&download_id=1%3As&sort=title#20)),
and the rigorous treatment is a **sampled-data** model of the loop
([IEEE Trans. Control Systems Technology, Modeling and Control of Sampled-Data IBVS](https://www.x-mol.com/paper/1753676290649198592#1)).
Practical consequence: raising the control rate above the sensor rate buys nothing but noise amplification, and
acting on every raw measurement is how jitter becomes motion. **This is the literature's justification for exactly the
two things already in this codebase** — the estimator's prediction/lead (`predicted_los_at_actuation`) and round 14's
aim deadband — and for NOT putting an S-curve on the tracking axis, where lag is the enemy and a slower ramp makes the
reticle sit further behind a moving target.

**Therefore the per-mode answer, which is what was actually asked:**

| mode | right motion behaviour | why |
|---|---|---|
| **HOLD / MANUAL-idle** | hold a **fixed encoder setpoint**, no re-aiming | nothing is being followed; every re-aim of a stationary axis is pure disturbance |
| **AUTO_TRACK** | **keep prediction + deadband; do NOT add an S-curve** | the loop is bandwidth/delay-limited (Finding 2); a ramp adds lag and pulls the reticle further behind |
| **AUTO_ROAM** | **this is where a jerk-limited profile belongs**, especially at the turnarounds | a sweep is a *programmed* move with no target to follow — a high-inertia load being stopped and reversed every ~8 s, which is precisely Technosoft's stated case for S-curve |

**That redirects item 5's "sloppy".** Round 12 measured the sweep at a flat 10.00 deg/s with zero deadline misses —
cruising is clean. **The 7 reversals in 55 s are the part that is trapezoidal by construction**: `search_planner.hpp`
sets `v_max_rad_s = 10 deg/s` and a `dwell_s = 0.25` and names no acceleration or jerk limit at all, so each turnaround
is an acceleration step — Finding 1's exact corner. So the candidate fix is not "tune gains" (the axis measured 0
reversals at rest) but **give the roam planner an acceleration and jerk limit and shape the turnaround**.
Measurable before/after in the terms the operator named: peak |jerk| at turnaround, position overshoot past the end of
sweep, and reversal count.

**Honest limits of this research.** No controller manual for this specific drive is in hand (the station's own
`docs/` is the closest thing), the two strongest sources are behind snippets rather than full text, and **nothing here
is measured on this turret**. It says where to look and what to try, and it demotes a guess I might otherwise have
shipped: I came in tempted to put an S-curve on tracking, and the delay literature is the reason not to.

## 2026-09-06, 13:0x — round 17: **I retract round 16's turnaround claim**, and the file I blamed was the retired one

Measured the roam reference again at ~19 Hz effective telemetry for 50 s (943 samples, `/tmp/r97_roam_signed.json`), this
time keeping the **sign** of the reference rate and finite-differencing it — which is the whole point, because in round 12
I had taken `abs()`.

**Results:** signed reference rate spans **−10.00 … +10.00 deg/s**; **reference jerk median 0.0, p95 21.1, max 75.8
deg/s²**; 6 near-zero-rate samples at the reversals.

**That contradicts what I wrote yesterday:** *"the 7 reversals are trapezoidal by construction… each turnaround is an
acceleration step — Finding 1's exact corner."* An instantaneous +10 → −10 reversal at a 50 ms sample gap would show a
reference jerk of order **400+ deg/s²**. The observed peak is **75.8**, typical **21.1**, with the rate passing through
zero — so **the reversal is already rate-shaped somewhere downstream of the planner**, and there is no unbounded-jerk
corner to fix. **Retracted.** Had I acted on round 16's inference I would have "fixed" a step that measurement says is a
ramp — and then a jerk-limited profile would have been sold as a win with nothing to compare against.

**Second error in the same record:** I attributed the behaviour to `search_planner.hpp`. That planner is **retired** —
it survives in `control_loop.cpp` only inside a comment at :154. The live planner is **`mode/roam_planner.hpp`**, whose
`v_max_rad_s = 0.175` (exactly the 10.00 deg/s measured) and `turnaround_dwell_rad = 0.008` are the real parameters.
Both errors were the same habit: reasoning from the file I had just read instead of the file that is actually running.

**What the code actually does**, now read rather than assumed: `RoamPlanner` names a **waypoint**
(`IntentType::JointPosition`, :312) and the intent carries the **roam ceiling**; the safety envelope has the final word
on rate *and* position (§111.18). So the shaping is done downstream of the planner by the reference/servo layer, which is
why the jerk is finite. The `turnaround_dwell_rad = 0.008` (0.46 deg) and the settle check are deliberately there —
:94-95: *"a jittery feedback signal cannot make the turret oscillate around the turnaround point at full rate."*

**What that leaves for "sloppy", now that the reference is exonerated:** (a) the **axis actually following** that
reference through the reversal — I have measured the *reference* jerk, not the achieved jerk, and those are different
quantities; (b) the **dwell pause** at each end, which is by design and reads as hesitation; (c) the **preview stutter**
(10 of 15 fps, rounds 6-11), which remains the likeliest thing an operator *sees*. Stated as candidates, not findings.

**Caveat on my own numbers:** telemetry arrives at ~19 Hz, so a shorter-than-50 ms jerk spike would be smoothed by my
finite difference; 75.8 deg/s² is a **lower bound on the peak**, though a step reversal would still have shown several
hundred. And the sign-crossing count printed as 0 because the rate lands on exactly zero, which my `<0` test misses —
another test bug worth remembering (the 6 near-zero samples are the honest signal).

**Also corrected:** the station again ended at `MANUAL / HOLD` after the `finally`, because my restore issued MANUAL then
AUTO_TRACK and I read the state too eagerly. Restored and re-verified in this entry's commit.

## 2026-09-06, 13:3x — round 18: achieved-motion baseline on the real axis — the yaw trails its reference by 3.6 degrees at 10 deg/s

1501 samples over 50 s of AUTO_ROAM (`/tmp/r98_follow.json`), safety `ALLOW` throughout, no aborts. Telemetry publishes
`q_ref_yaw_rad`, `q_ref_rate_yaw_rad_s` **and** `q_ref_accel_yaw_rad_s2`, so following error and commanded acceleration
are read directly rather than reconstructed.

**Trustworthy (direct subtraction / published fields):**

| metric | value |
|---|---|
| following error, **SWEEP** | p50 **3.629°**, p95 **4.274°**, max **4.617°** |
| following error, **TURNAROUND** | p50 **1.350°**, max **1.776°** |
| signed mean error over SWEEP | **+0.190°** |
| commanded acceleration | p50 **0.0**, p95 **23.6**, max **60.0 deg/s²** |

**The finding: the yaw axis trails its reference by ~3.6° while sweeping.** At 10.00 deg/s that is **~0.36 s of travel**
continuously lost — the axis is not keeping up with a reference it is nominally tracking. It shrinks to 1.35° once the
axis stops (TURNAROUND, settling), which is the signature of a **velocity-proportional lag** — a following loop with no
(or negligible) velocity feed-forward. The signed mean of only +0.19° *confirms* this rather than contradicting it: a
lagging signal averages to about zero over a symmetric back-and-forth sweep.

Why this matters for the modes: the sweep **extremes are reached ~0.36 s late**, the planner's "arrived" test is
evaluated against that same reference, and in AUTO_TRACK the same lag means the reticle sits ~3.6° behind a target the
turret is following at rate — which is exactly the kind of thing an operator reads as the tracking being "not good",
entirely independent of any jitter. **Baseline before any change: 3.629° p50 / 4.274° p95.**

**And corroboration for round 17's retraction:** the *published* commanded acceleration (p95 **23.6**, max **60.0
deg/s²**) agrees with my independent finite-difference estimate of reference jerk (p95 21.1, max 75.8). Two different
measurements, same story — **the reversal is shaped**, and there is still no unbounded-jerk corner.

**What I refuse to report as a finding: the achieved-velocity figures.** My 5-point finite difference gave median
38.75 deg/s and max 63.40 deg/s on a sweep whose reference is 10.00 deg/s — physically impossible. The cause is my own
estimator: telemetry arrives unevenly, and dividing an encoder LSB of 0.02177° by an occasional 5 ms gap manufactures
huge velocities. The sign-change count (8) is roughly the right order for the number of reversals, but I am not booking
any achieved-velocity, achieved-acceleration or achieved-jerk number until I resample onto a uniform grid or fit over a
window (Savitzky-Golay) and **sanity-check that the median lands near the 10.00 deg/s the reference says it must.**
Reporting 38.75 would have been the fabricated-number habit wearing a different hat.

**Next, on the same metric so before/after is comparable:** a velocity feed-forward term in the yaw position loop is
the textbook answer to a velocity-proportional lag, and it is testable with exactly these numbers — target following
error p50 well under 3.629° at the same 10 deg/s, with no overshoot introduced at the reversal. That is a controller
change, so it goes default-off with a config key and lands only if the measured error actually moves.

## 2026-09-06, 14:0x — round 19: the 3.6 degrees is real, but it is not the lag I described — and one of my own numbers this round was mis-specified garbage

Re-examined `/tmp/r98_follow.json` with no new motion, because round 18's claim deserved to be attacked before I
"fixed" anything.

**First doubt, then elimination.** `q_ref_yaw_rad` is not a smooth setpoint: **738 distinct values over 1476 sweep
samples, half the samples unchanged, median step 0.658°, max 1.045°** — a **stepping waypoint**. If the error were
waypoint-lead quantisation, it would appear only on the samples where the waypoint jumped. It does not:

* **|error| when the waypoint MOVED this sample: p50 3.631°** (n=740)
* **|error| when the waypoint was UNCHANGED: p50 3.626°** (n=735)

Indistinguishable → **waypoint quantisation eliminated.** And `error` opposes nothing: its sign matches the direction of
motion **733 times out of 740**, so the axis genuinely trails the reference while moving. **At rest (`WAIT_TARGET`) the
error is exactly 0.0000°** — though only 3 samples landed in that phase, so treat that as indicative, not established.

**A number of mine that was wrong on the page before it was wrong in the record: τ = 0.019 s.** I had regressed the
**signed** error against **|rate|**, which cancels the two sweep directions against each other — arithmetically
meaningless, and it produced a lag estimate 20× smaller than the raw data. Redone as signed-on-signed:
**τ = 0.173 s** (n=734). **Withdrawn: the 0.019 s figure.** The two estimates still disagree (a 0.173 s linear
component versus 3.6°/10 °/s = 0.36 s of steady-state offset), and that disagreement is itself the clue — the error is
**not purely proportional to rate.**

**What fits all of it at once: a constant error while moving in either direction, zero error at rest, sign flipping with
direction.** That is the classic signature of **kinetic friction** (or equivalently a low velocity-loop gain), not of a
pure transport lag — and this station already carries independent evidence of friction: the measured friction deadband
that `yaw_park_deg: 176` exists to work around. Conditioned on the reference actually moving faster than 8 deg/s, the
error is p50 **3.647°**, max **4.617°** — a **steady-state** offset that does not grow over the sweep, i.e. the axis
keeps pace at 10 °/s but sits a fixed distance behind, which implies an effective velocity gain of about
**Kv ≈ 10 / 3.6 ≈ 2.8 s⁻¹**.

**Net for the record: round 18's headline magnitude stands (3.6° p50 / 4.27° p95 at 10 °/s, zero at rest), but its
mechanism is revised — finite velocity authority against friction, not a 0.36 s transport delay.** That changes the fix
in one respect and not in another: **velocity feed-forward is still the right first change** (it cancels the
rate-proportional demand regardless of which of the two dominates), while a friction/deadband compensation term would be
the second lever — and the distinction matters because a pure transport-lag story would have pointed at pipeline delay
instead, where nothing in the controller would have helped.

Nothing on the station was touched this round; the mode was already restored and verified (`AUTO_TRACK / WAIT_TARGET`,
`ALLOW`).

## 2026-09-06, 14:3x — round 20: the authority boundary is established, and round 19's friction story is demoted

**Who closes this loop — answered, because it decides whether item 2's dynamic fix is even ours to make.**
`MotorBackend` offers both authorities: `enter_position_mode(axis, limit_spd_rad_s, …)` (the *drive* closes the loop)
and `command_velocity(axis, velocity_rad_s)` (controld closes it). The live loop uses
**`backend_->command_velocity(...)` during motion** (`control_loop.cpp:789` for the stepping axis, `:796` holding the
other axis at zero, `:982` for pitch), and reaches for `enter_position_mode` at `:759` / `enter_position_mode_all`
(`:44`, `:103`, `:826`) for the other cases. **So the ~3.6° following error measured in round 18 lives inside controld's
own authority — it is ours to fix, and blaming it on drive parameters would have been an excuse.**

**And a demotion of yesterday's conclusion, which I over-committed to.** Round 19 said kinetic friction "fits all of it
at once". It does — but so does a **pure-rate velocity program with no proportional closing term**: command the axis at
the waypoint's own 10 °/s and any offset that opens up never closes, which predicts *exactly* the same three facts I
used as evidence (constant offset while moving, zero at rest, sign flipping with direction). **Those two explanations are
indistinguishable in the data I have**, and I stated one as if it were established. Corrected claim: the offset is
consistent with *either* kinetic/friction effects *or* a missing closing term; the fix differs (friction compensation vs a
proportional term), so the diagnosis has to be settled by reading the velocity program, not by more of the same data.

What the search says so far: outside homing, `velocity_rad_s` appears in
  * `move_to.hpp`
  * `park_controller.cpp`
  * `park_controller.hpp`
  * `can_motor_backend.cpp`
  * `can_motor_backend.hpp`
  * `control_loop.cpp`
  * `motor_backend.hpp`
  * `sim_motor_backend.hpp`
and the scan for an assignment whose right-hand side contains an error/gain/remaining-distance term found
**0** site(s) — i.e. no proportional closing term was located by that scan, which would make the "missing P-term" explanation the
live one. Not yet confirmed: a closing term could be folded into a helper call whose name contains none of the tokens I
searched for, so this is a lead, not a finding.

Round 19's honest-uncertainty habit applied to myself: the tokens above are a filter, not a proof.

## 2026-09-06, 15:0x — round 21: **round 20's authority-boundary conclusion is retracted** — I read calibration code and called it the control path

Round 20's headline was *"the live loop commands velocity during motion (`:789`), so the 3.6° following error is inside
controld's own authority and ours to fix."* Chasing the producer of that call's argument turned the claim inside out:

* `:789 backend_->command_velocity(a, ds.velocity_rad_s)` takes `ds` from **`:730 DesiredState ds = homing_->step(...)`**
  — it is the **homing** path.
* `:982/983 backend_->command_velocity(..., po.yaw.velocity_rad_s)` takes `po` from
  **`:972 ParkOutput po = park_->step(...)`** — it is the **park** path.
* `move_to.hpp`, the next suspect, is `control/src/calibration/move_to.hpp` — **calibration** too.

Every backend actuation call in `control_loop.cpp` is either bootstrap (`:48`, `:61`, `:141`, `:253`) or inside those two
calibration branches (`:759`, `:768`, `:789`, `:796`, `:803`, `:982`, `:983`, `:1007`). **The normal-operation actuation
call was not found**, and my two command_velocity witnesses were homing and park.

**Retracted: "the following error is inside controld's own authority."** It may be, but it is not established, and the
next thing I read was nearly the basis of a controller change built on the wrong loop. Note the shape of the mistake,
because it is now the third time in this investigation: *a call site that exists and is consistent with my hypothesis,
found without checking which branch reaches it.* Round 16's retired-planner attribution, round 17's `abs()`-hidden sign,
and this. The guard is the same each time and costs one grep: **establish what actually executes before attributing
behaviour to it.**

**Which lever is live is therefore unknown again**, and the two candidates differ in kind:
* if normal motion reaches the drive through a **position** command — `:768 backend_->command(a, ds.target_rad,
  ds.speed_rad_s)` is a position command and is the structural sibling of whatever the normal path calls — then the
  **drive closes the position loop**, and a controld-side SpdRef feed-forward would be aimed at a loop controld does not
  own; the lever would be drive parameters (the operator's/hardware's domain) or controld shaping its own reference.
* if it commands **speed** (`SpdRef`, and the comments at `:735-737` already describe the drive's internal velocity loop:
  integral winding to 1.3–1.8 N·m, SpdRef Kp ≈ 0.38 N·m/(rad/s)), then controld closes position and a bounded
  proportional term in the commanded rate is exactly the right change.

**One fact that survives from round 20 and matters either way:** the drive is a CyberGear running an internal **velocity**
loop (`SpdRef`, `can_motor_backend.cpp:229-237`, with a re-write dead-band at `:235`). So the position loop is closed by
*someone* upstream of SpdRef — controld or the drive's own position mode — and that is precisely the open question.

**Decisive probe for next round, named exactly:** in the cycle *after* `ref_mgr_->resolve(...)` and
`last_intent_ = build_mode_intent(now_ns)` (`:605`), find the function that turns the resolved reference into backend
calls — search the post-resolve section for the actuation call (it will not be named `ds`/`po`), and read whether the
value it sends is an **angle** or a **rate**. The 3.628° p50 metric stays the judge whatever turns out to be true; what
changes is which knob I am allowed to turn.

Round 20's *demotion* of the friction story still stands on its own: the offset is equally consistent with kinetic
friction and with a missing proportional term, and those need different fixes.

## 2026-09-06, 15:3x — round 22: the lever question is settled — normal motion sends a POSITION, so the 3.6° is the drive's error, not controld's

Round 21's named probe answered, and round 20's retraction is now confirmed rather than merely suspected. Two things I
had to fix in my own method first: my round-21 listing was **truncated by `head -12`** on a file with **21**
`backend_->` sites, and a cross-file search for actuation calls returned nothing — because the call was in
`control_loop.cpp` all along, past where I had stopped looking.

**The normal-operation actuation is `control_loop.cpp:1085`: `backend_->command(a, qr, ls);`** — the `else` arm of the
safety branch (`:1080-1083`: not-Allow → `command_velocity(a, 0.0)`, Allow-but-nothing-to-say → `keepalive(a)`). Given
`MotorBackend::command(axis, target_rad, speed_rad_s)`, that is a **position command with a speed limit**, not a rate.

**So the 3.6° following error is the CyberGear's internal position-mode error at that speed limit.** A controld-side
SpdRef feed-forward — the change I was lining up in round 20 — would have been aimed at a loop controld does not close.
`head -12` was two-thirds of the way to a wrong controller change.

**What controld can and cannot reach, from the actual plumbing:**
* **Can:** the commanded angle `qr` (shape it / lead it), the **speed limit** `ls`, and the drive's **speed-loop gains**
  — `set_speed_loop_gains` is fully plumbed (`:2253` → `can_motor_backend.cpp:357`).
* **Cannot:** the drive's **position-loop gain**. A search for any position-gain register reference returns only
  `Reg::MechPos`, which is a position *read-back*, not a gain. **There is no path in this codebase to the drive's
  position-loop P term.**

Three consequences, in order of usefulness:

1. **Item 2's dynamic half has to be argued at the levers that exist.** Lowering the commanded speed trades sweep time
   for fidelity; leading `qr` pre-compensates the trail in software; retuning the speed loop is measurable but changes
   control behaviour and is the operator's to sign. **Tuning the position loop is not available from here** — which is
   worth stating flatly, because "increase the position gain" is the reflex answer and it is not a thing this code can do.
2. **This is very likely the real mechanism behind "the roam range seems strange."** Round 12 measured the *encoder*
   sweep at 108.82–188.77° while the envelope is ready±45°; the axis **trails its commanded position by ~3.6° in the
   direction of travel**, so it reaches each extreme late and partly short, then closes the gap during the dwell. Item 5
   and item 2 are the same defect seen from two ends — not two bugs.
3. **AUTO_TRACK is affected in the worse way.** A ~3.6° trail while following a target at rate is a reticle sitting
   3.6° behind, with no controller knob in controld to shorten it except leading the command or slowing down.

Stated as inference, not proof: `qr`'s definition was not traced this round; what is proven is the call signature
(`command(axis, target_rad, speed_rad_s)`) and that it is the only actuation call outside bootstrap, homing and park.

## 2026-09-06, 16:0x — round 23: position lead implemented, default-off, 5 new tests, CTest 57/57 — the after-measurement is what's left

Took the declared default path from round 22 (lead `qr`, because the envelope keeps the final word on position).

**New `control/src/control/position_lead.hpp`** — `apply_position_lead(cmd, rate, lead_s, soft_min, soft_max)`, with two
properties stated as the reason the file exists rather than as comments:
* **No lead at zero rate** — a hold, a park, every idle cycle is bit-identical to today, and it cannot creep past what
  it stopped on.
* **Never commanded past a soft limit** — the result is clamped into `[q_soft_min_rad, q_soft_max_rad]`; a
  mis-loaded (inverted) limit pair falls back to the bare command rather than throwing on the control thread.
Plus a third guard in the caller: the rate estimate is capped by the commanded speed limit `ls`, so a **reference step**
(mode change, re-seed, a fresh waypoint) cannot manufacture a lead.

**Wired at the actuation site** (`control_loop.cpp:1085`) as `qr → cmd_pos`, deliberately **after** the hold and
emergency branches that overwrite `qr` with measured feedback or the emergency-stop target — leading a safety manoeuvre
would be wrong, and those paths are untouched. Config `auto_track.position_lead_s`, default **0**, plumbed the way the
deadband is (V3Config → parse → `station_wiring.cpp`).

**Measured:** `controld` builds clean; **`PositionLead` 5/5 pass** (disabled returns the command bit-for-bit; nothing led
at a stop; lead proportional and forward-pointing both ways; `2.9 + huge rate` clamps to exactly `3.0` and
`-2.9 - huge` to `-3.0`; inverted limits fall back); **CTest 57/57** with the test binary rebuilt from fresh objects —
the lesson from round 15b applied on purpose.

**Sizing, deliberately conservative:** the measurement brackets the needed lead at **0.173–0.36 s**, which at 10 °/s is
1.7–3.6°, and the documented first value is **0.1 s** — about a third of it, because **an overshoot at the turnaround is
worse than the shortfall it removes.**

**What is NOT claimed:** no after-number yet. The metric is defined and the before-number is fixed (**p50 3.628° /
p95 4.274° at 10.00 °/s**, round 18), but measuring the improvement needs the key **enabled**, which means editing
`turret.yaml` and **restarting the live `controld`**. I have not done that unilaterally: it changes pointing behaviour on
a station that is running, and unlike the deadband and the hand-off — which are inert until enabled — this one takes
effect the moment it is switched on, with the axis moving up to ~1° further along its sweep than it does today. The soft
clamp means it cannot cross a limit, but "cannot cross a limit" is not the same as "the operator wants it moving
differently." **That trial is one command away and needs a yes.**

One process note on my own tooling: my build verification this round was `grep -c "Linking|error:"` returning **0** on a
second invocation, which looks alarming and simply meant controld was already up to date from the first invocation
(which printed no errors). Verified by the fact that the five new tests exist and ran.

## 2026-09-06, 17:0x — AUTO_TRACK enabled on the real path and monitored: it correctly refused to track, and that is the finding

Operator instruction: enable auto tracking, monitor performance, comment. Done — and **AUTO_TRACK is left enabled**
(`AUTO_TRACK / WAIT_TARGET / ALLOW`) with the real IMX500 source feeding it.

**What had to be fixed first.** `visiond` was not running and `camera_fps` was 0; starting it failed with
`RuntimeError: Camera __init__ sequence did not complete` — **the preview pane was holding the single-owner IMX500**
(`running True, fps 10.0`, left over from the round-15 video test). Stopped the pane, source came up: **15.13 fps**,
`vision_track_sets` 443+.

**Monitored 70 s / 140 samples at 2 Hz, after issuing `select_target`:**

| | |
|---|---|
| mode phase | `WAIT_TARGET` for all 140 samples |
| safety action | **`ALLOW` always**; `control_deadline_misses` **0** |
| track count | fluctuating **0–3** blobs, `class_name='person'` at **conf 0.31–0.36** |
| `selected_confidence` | **0** in every sample |
| `confidence_band` | **`INVALID` in 140/140** |
| motion over 70 s | yaw **0.175°**, pitch **0.546°** total travel |

**The honest comment: there is no tracking-performance number from this run, because nothing was tracked** — and the
station's refusal is *correct behaviour*, not a fault. `predicted_target_los_valid` went `True` after selection, but the
confidence band never left `INVALID`, so the controller held. 0.175° of yaw in 70 s on a station watching a static room
through a noise-level detector is what a well-behaved tracker looks like. **Quoting those degrees as "tracking
performance" would be the fabricated-number habit again.**

**Why it cannot engage on this detector**, with the identity evidence: blob uuid `0:77` persisted across all 12 samples
while `0:75` vanished after ~3 s, and `selectable` oscillated **True → False → True** on the surviving blob. The blob I
selected was one of the transient ones, which is why `select_target` was accepted (`ok/submitted`) yet never became
`selected`. So selection instability here is mostly *the detector's*, not the selection logic's: threshold blobs appear
and disappear, and `selectable` correctly follows that.

**Two things that deserve naming as defects/risks, not as news:**
1. **`track_state` reports `tracking`, `coasting` and `brake_to_hold` while the band is `INVALID` and nothing is
   selected.** An operator watching the HUD sees "tracking" on a station that is holding and has no target. The
   telemetry next to it (`confidence_band INVALID`, `selected_uuid_valid False`) tells the truth; the label does not.
2. **The blob detector stamps `class_name='person'`.** That is a threshold blob wearing a class label — precisely the
   *appearance* of a fake target this project was told to stop producing. It is non-production by design, but the label
   should say so.

**What this settles for item 1:** the synthetic fixtures are gone and the real path is running, but **the `simple`
detector at conf ~0.33 cannot reach a confidence band, so AUTO_TRACK engagement cannot be measured on it at all.** Real
tracking performance needs either the `rpk` path (blocked on the picamera2 AI/`postprocessing_config` API) or a genuinely
detectable target in view — and possibly a look at the band thresholds, since a diagnostic detector that can never
qualify tells us nothing about the tracker either way.

**Operational note:** the IMX500 is single-owner, so **the HUD video pane and real vision cannot run together**. The pane
is off now; turning it back on will starve `visiond` again with that same unhelpful error.

## 2026-09-06, 18:0x — session review: what is actually shipped, what is only implemented, and what I got wrong

Written as a hand-off, because the value of this file is not the count of commits but knowing which statements can be
trusted without re-checking. New document: **`docs/STATION_RUNBOOK.md`** — one-off deployment procedure, verified
commands, and the trap list. If you read one thing, read its §1 and §7.

### Shipped, tested, and green

| change | proof |
|---|---|
| HUD staleness flash debounce (`web/webd/hud.py`) | runs the real `updateStaleness` under a scripted clock; pre-fix verdicts reproduce, post-fix do not |
| `fps_published` / `sensor_fps` on `/api/video/state` | 4 tests + hardware readings |
| **Aim deadband** `auto_track.deadband_deg` + release (item 3) — `control/src/control/aim_deadband.hpp` | `AimDeadband` 4/4, incl. disabled-passes-through |
| **Mode hand-off watcher** `roam_on_loss_ms` / `track_on_acquire_ms` (item 4) | **compiles and is inert; NO test** — see below |
| **Position lead** `auto_track.position_lead_s` (item 2) — `position_lead.hpp` | `PositionLead` 6/6, incl. the unloaded-envelope regression |
| **Dual feed / frame tap** (preview + detector simultaneously) | 380 pytest; hardware: pane `vision-tap` 9.8 fps **while** detector ran 9.55 fps; `/api/video` served **38 JPEG frames in 4 s** |
| `scripts/lead_trial.sh` self-reverting hardware trial | ran 3×, each time restored config and controld |
| Suite state | **CTest 57/57**, **pytest web+vision 380 passed** |

### The bug the review found in my own code (fixed today, `01bd65e` follow-up)

`apply_position_lead` clamped into `[q_soft_min_rad, q_soft_max_rad]` guarded by `soft_min <= soft_max`. An **unloaded
envelope has both at 0.0**, and `0 <= 0` is true — so enabling the key before the envelope loaded would have clamped
**every commanded angle into [0, 0]**, parking both axes at the origin. Inert only because the key defaults to 0. Fixed
to require `soft_max > soft_min`, with a regression test naming the failure. This is what "default-off" hides: an inert
feature can still contain the shape of a serious bug.

### Implemented but NOT accepted (do not treat as done)

* **Item 4 watcher:** no unit test, no measured switch latency, no hysteresis measurement. Default-off.
* **Item 2 lead:** no after-number. Before-baseline is fixed — **p50 3.628°, p95 4.274° at 10.00 °/s**. Run
  `scripts/lead_trial.sh` (needs a real `auto_track:` node and `start_homing` after the restart it performs).
* **Item 1 acceptance:** real path runs (`visiond --real`, 15.1 fps, `--detector simple`) but **`simple` cannot engage**:
  `select_target` is refused with **`person #1 is LOST, not CONFIRMED`**, so the intent stays `hold / "select a target"`
  and the axis never moves. `rpk` still blocked on the picamera2 AI API.
* **§24 (16) and §110 (30): 0 operator-signed.** Nothing here is acceptance.

### What I got wrong, listed so nobody re-buys it

1. Round 6: set `controls["FrameRate"]`, a control this pipeline does not expose — inside `except Exception: pass`, which
   made a dead change look applied.
2. Round 16: attributed live roam behaviour to `search_planner.hpp`, which is **retired**; the live planner is
   `mode/roam_planner.hpp`.
3. Round 16/17: claimed roam turnarounds were unbounded-jerk steps, then measured signed jerk (**max 75.8 °/s²**) —
   **already shaped**. My round-12 `abs()` had hidden the sign.
4. Round 18/19: reported τ = 0.019 s from a **mis-specified regression** (signed error on |rate|). Withdrawn; corrected
   to 0.173 s. Then over-claimed kinetic friction as *the* mechanism; a missing proportional term fits the same data.
5. Round 20: concluded controld closes the position loop, citing `command_velocity` call sites that belonged to
   **homing and park**. Normal motion is `control_loop.cpp:1085 backend_->command(a, qr, ls)` — a **position** command;
   the drive closes that loop and its position gain is not reachable from this codebase.
6. Same round-20 habit twice more: a `head -12` truncation hid the real call site, and my trial script measured a
   stationary axis and produced a triumphant **0.000** following error.

Common shape: **a plausible mechanism plus one confirming artifact, without checking what actually executes.** The
guards that work are cheap and boring — establish the executing path, keep the sign, verify the rebuild, and let a
measurement own the conclusion.

### Open decisions that block real closure (all operator-owned)

1. **`rpk` path** — upgrade picamera2 (risks the working pane) or accept `simple` as diagnostic-only.
2. **A target the detector can CONFIRM** — nothing measurable has been in view all session.
3. **Band thresholds** — at conf ≈0.33 the diagnostic detector can never qualify, so engagement is unmeasurable either way.
4. **Item 4 policy values** (`roam_on_loss_ms`, `track_on_acquire_ms`), and item 2's harder half — *"command a fixed
   encoder position"* in the sense of **opening the position loop** — deliberately **not** implemented, because that
   discards gravity-sag, backlash-creep and disturbance rejection on a station with a measured friction deadband.

## 2026-09-05, 00:3x — the prediction box, and a station that had been dark for twenty minutes

Operator: *"multiple issues with your web ui, like the predict box just doesn't work. I need you to fix it."* Fixed, and
the route there was instructive.

**First: nothing was running.** `/api/state` returned nothing because **the station had rebooted 31 minutes earlier**
(uptime 31 min, all `/tmp` evidence gone). Bringing it up reproduced the trap in full: `web server did not start: bind:
No such file or directory` — `/run/ota` does not exist until systemd makes it — logged as a **warning**, after which
controld ran, homed and tracked with **no operator interface whatsoever**. Now fixed in `main.cpp`: controld creates the
socket's parent directory (verified live: `created /run/ota for the web socket`, then `web server listening`). Still a
warning rather than a fatal exit — killing the control loop because the UI cannot bind would trade a visible fault for a
hidden one — but the fault can no longer be silent in the way it was.

**The predict box, root cause.** `control_loop.cpp:1497-1522` decided *for itself* whether a prediction existed
(estimator initialised? intrinsics sized like the frame? `r_cam.z > 0.05`? in frame?) while the motion the cue was
supposed to forecast came from `build_mode_intent` under `follow_los && estimator_ready` (`:2187`). Two copies of one
decision, and the block's own comment even warned that a second copy of those guards "would be free to drift". Measured
while the station reported `tracking`: **cue invalid in 53 of 55 samples**. The fix makes the cue the thing it claims to
be — projected from `last_intent_`, the ray actually commanded this cycle, deadband included — and makes every way it
can fail say so in a new `prediction.reason`, surfaced on the page (`PREDICTION: LIVE / <reason>`).

**Verified, on the documented synthetic bench source** (a UI-path test, *not* tracking acceptance — item 1 still needs a
real target): while `intent_type == los_direction`, **94/95** samples produced a cue, and the one that did not correctly
reported *"the predicted ray points behind the camera"*; while not aiming, **0 of 19** produced a cue. That second half
is the point: drawing a prediction while the controller holds was the lie, and it now measures zero.

**Two more of my own defects, both caught by the tests I wrote for the fix, not by me:**
1. **Stale tap served forever.** `_tap_loop` noticed a tap file *vanishing* but not one going stale. After a controld
   restart brought visiond back without `OTA_VISION_FRAME_TAP`, the pane reported `running True, error ''` while
   serving a JPEG **80 minutes old**. It now refuses to publish a stale frame and names the cause.
2. **Worse: the fallback was wrong.** With a stale tap, `start()` fell through to **opening the camera** — the exact
   sensor visiond holds, which is where `Camera __init__ sequence did not complete` comes from. Routing is now by
   *existence*; freshness decides only whether frames are served.

**And a near-miss worth recording.** `/api/video/state` showed `frames_published 1057` with `fps_published 0.0`, which
looked like a broken rate counter — I had already diagnosed a deque/list mismatch in my own `_start_tap`. It was
**wrong**: `_recent` is a plain list by design, and 0.0 was *truthful* — the last publish genuinely was 80 minutes ago,
so a 5-second window held nothing. I nearly "fixed" a correct measurement to agree with a story I liked. The lesson is
the same one from the following-error `0.000`: check what a number measures before you repair it.

Suite: **CTest 57/57** (fresh binary), **pytest web+vision 383 passed** (+3 tap tests). Left running: controld homed
`MANUAL/HOLD/ALLOW`, real vision with the tap at ~10 fps, pane on `vision-tap` at 9.8 fps.
