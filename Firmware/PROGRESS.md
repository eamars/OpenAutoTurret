
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
