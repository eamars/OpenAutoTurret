
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
