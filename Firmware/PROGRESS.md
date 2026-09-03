
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
