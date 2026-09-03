
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
