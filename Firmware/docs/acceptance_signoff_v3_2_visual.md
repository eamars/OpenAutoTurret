# Acceptance sign-off package — v3.2 Apache-HUD

Prepared for the operator. **Nothing in this file is signed.** The revision makes visual fidelity and hardware
acceptance operator judgements (§24, and the acceptance ledger in the v3 architecture document), and an
acceptance item recorded by the agent that wrote the code is worth nothing. What this file does is put every
item, the evidence that exists for it, and the evidence that does **not** exist, in one place, so signing is a
matter of looking at the screen and writing a name.

Verified counts as of this writing, correcting a figure that had been repeated loosely for many rounds:

* **§24 of the v3.2 revision holds 16 unchecked items** — the visual ledger, listed one by one below.
* **The v3 architecture document holds a separate 30-item ledger** — `docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md`, unchecked in place. Those are system and hardware acceptance items, not visual ones. For months the running record quoted "30 items, 0 accepted" as though that were §24; it was the other ledger. Both are recorded honestly below: **0 items on either ledger have been accepted by a named person.**

## How to use this

For each row: look at the station, decide, and fill in the three blanks. `Automated evidence` names a test file
under `web/webd/tests/` or a served-page check — a green test means the structure is as specified, never that
it *looks* right, which is precisely why §24 is a human call. `No evidence` is written plainly where there is
none, so the absence is not mistaken for a pass.

## §24 — visual acceptance (16 items)

| # | Item (as the revision words it) | What implements it | Automated evidence | Operator signature / date / what was observed |
|---|---|---|---|
| 1 | camera feed fills essentially the whole viewport | letterbox-fit math in `hudLayout()` + `#video` CSS | `test_travel_tapes.py` measures the fitted rect; page served at `/` carries the layout | |
| 2 | no conventional top navigation bar is visible | no nav element in `HUD_HTML`; strip is bottom-only | markup inspection; `test_dock_drawers.py` asserts the dock is the only pressed surface | |
| 3 | top-left mode block resembles the reference location and density | `#mode-block`, three lines, §16 tiers | `test_state_and_safety.py` (state wording), `test_visual_discipline.py::TypographyMatchesSection16` | |
| 4 | yaw tape occupies the upper centre and clearly shows total safe travel | `hudTravelTape()` / `hudTravelTapeSvg()` | `test_travel_tapes.py` | |
| 5 | current yaw value is boxed beneath its marker | tape value box, `text.tval` at 12px (strongest numeric) | `test_visual_discipline.py` (tier ordering), `test_travel_tapes.py` | |
| 6 | pitch tape sits on the right edge and clearly shows total safe travel | same builders, vertical orientation | `test_travel_tapes.py` | |
| 7 | yaw/pitch values are the strongest numeric text | 12px `text.tval` vs 11px scale / 10px candidate / 9px FOR legend | `TypographyMatchesSection16::test_the_current_axis_value_is_the_strongest_numeric_text`, `…_candidate_labels_sit_below_scale_labels` | |
| 8 | center reticle uses separated sensor-style brackets and open centre space | reticle brackets drawn in `render()` around the anchor from `hudAxisNorm()` | `test_reticle_anchor.py` proves the anchor follows the **optical axis** (`cx/width`, `cy/height`) using a deliberately off-centre principal point, and falsifies the tempting shortcut: hard-coding the frame centre makes it fail. It also records that **this station's calibration is dead-centre (cx 960 / cy 540 of 1920×1080), so a wrong build would pass here** — the coincidence is not evidence. Live telemetry confirms `camera_intrinsics.valid: true`, so the page uses the real calibration and not the centred fallback; the fallback's on-screen note is asserted to exist, but on this station it is never exercised. **Still visual: bracket separation, open centre, and whether it reads as a sensor reticle — the bracket geometry is inline in `render()`, which needs a DOM.** | |
| 9 | top-right health indicators are small chips, not a large card | `.chip` CSS rule (no verified builder name is cited here) | `test_visual_discipline.py` (tokens, sizes) | |
| 10 | selected target is bright green | `stroke = C.green` when `selected` | `test_visual_discipline.py::PaletteDiscipline` (red restricted to fault/stop) | |
| 11 | candidate targets are dim green | `stroke = C.dim` for unselected | same as above | |
| 12 | prediction is amber and dashed | `hudPredictionSvg()`, `stroke-dasharray="6 4"`, amber | `test_state_and_safety.py` prediction tests | |
| 13 | FOR inset is compact and located at lower left | `hudForInset()` / `hudForInsetSvg()`, legend at 9px `text.flbl` | `test_for_inset_placement.py` asserts placement against the left margin, in the lower part of the viewport, clear of the bottom strip, and inside a viewport-fraction size budget at four window sizes — and was falsified (widening the card to 40% makes it fail). `test_section_20_ledger.py` reads the FOR polygon from the payload webd serves over its real path - against a FakeControld it spawns on an ephemeral port, NOT the physical station. **What remains visual: whether it *reads* as compact, and the legend text itself.** | |
| 14 | bottom status strip is thin and subordinate | `#strip`, 10px | `test_visual_discipline.py` | |
| 15 | five compact context buttons sit at lower right | `hudDockSpecs()` → TARGETS / MODE / MANUAL / DIAG / MENU, z=30 | `test_dock_drawers.py` (five buttons, drawer contents, command payloads) | |
| 16 | drawers overlay the camera instead of resizing it, and the whole impression is a coherent sensor HUD | `#drawer` positioned absolute, z=40; §18 layer order | `test_dock_drawers.py`; `LayeringMatchesSection18`; **the "coherent HUD" half of this item has no automated evidence at all and cannot have** | |

Two structural facts the operator may want to know before signing item 16, because no screenshot conveys them:

* Every SVG text class is asserted to be *both declared and used*. An undeclared token once deleted three font
  rules while every test stayed green, so `test_visual_discipline.py` now fails on tokens read but not declared.
* **No real browser has ever been asserted to have painted this HUD.** The strongest available check is
  `node --check` on the concatenated JavaScript plus assertions on the page served over HTTP. Anything a real
  browser would catch — font fallback, sub-pixel placement, actual contrast — is exactly what §24 is for.

## The v3 architecture ledger (30 items)

Left **in place** rather than copied here, so it cannot drift from its document: all 30 boxes in
`docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md` remain unchecked. Measured evidence that
bears on some of them, and where it lives:

* **Head-aim while holding** — `docs/evidence/dart_25deg_in_1.6s_envelope_legal_2026-09-04_r27.log`: hold-window
  aim error p50 **0.244 box heights** against the 1/3 bar. Holding inside tolerance is met on this station;
  holding it *through* a sharp dart is not (see below).
* **Following a sharp dart** — two runs, recorded with their motion parameters because the first was
  misread for rounds: `dart_25deg_62degps_2026-09-04_r25.log` (25° / 0.40 s, **forbidden by the commissioned
  envelope**, which needs 1.43 s) and `dart_25deg_in_1.6s_envelope_legal_2026-09-04_r27.log` (25° / 1.60 s,
  envelope-legal). Verdicts on the legal dart: **C1 containment PASS · C2 recovery FAIL (2.85 s vs 1.50 s) ·
  C3 lead FAIL (−11.6°) · C4 FAIL (3 sign changes) · C5a PASS · C5b jerk FAIL (p95 542 vs 300+60, a lower
  bound measured at probe rate).**
* **Why C2/C3 fail: explained, and the cause is a design ceiling the operator owns.** The tracking reference is pinned at exactly 10.00 deg/s (34 telemetry frames, zero acceleration) because `control_loop.cpp:427` caps the AUTO_TRACK proposal with `hold_speed_effective()` = min(hard-coded hold speed 10.0, payload profile 20.1, 20.1) = **10 deg/s** - applied before the confidence derate, deliberately, per the code's own comment. The configured tracking speed is 30 (controld logs it) and confidence measured 1.0, so neither is binding. A 25 deg dart in 1.60 s needs ~15.6 deg/s average: **C2 and C3 cannot pass at that size whatever the tracker or prediction gains do.** The 10 deg/s has no key in `turret.yaml`; raising it is a safety decision about how fast this station may swing. Full chain, including the three mechanisms ruled out by measurement: `docs/evidence/ref_rate_plateau_2026-09-05_r38.md`.
* **Smoothness (C5b) is a measured failure, agreed by two independent instruments.** Jerk p95 is **525 °/s³**
  from the telemetry socket stream at 15.2 Hz and **542 °/s³** from the probe's own sampling path, against a
  configured `max_jerk_deg_s3: 300` — roughly 1.8× over, consistent across two grids (~20 ms and 66 ms) and two
  transports. **Do not quote "max jerk"**: the same scenario has produced 576, 1892 and 9755 °/s³, which is
  differentiation of a grid the transient does not fit, not a property of the machine.
* **Camera geometry** — **plate scale / effective FOV measured on real optics** by encoder-as-theodolite
  (`calibration/camera_intrinsics.yaml`: fx 1389, fy 1467 → 24.24 and 25.60 px/deg; hfov 69.3002°, vfov
  40.4171°). **The principal point is NOT measured**: the calibration file's own line 27 says *"cx/cy are the
  GEOMETRIC CENTRE BY CONVENTION, not a measurement"*, because on a rotating platform the principal point and the
  camera-to-axis boresight enter as the same constant offset. So `cx 960 / cy 540` is an assumption wearing a
  "MEASURED" header — true of fx/fy, not of cx/cy. **Consequence: centring tolerance and frame-exit margin are
  still not computable**, which is exactly what the objective said would happen without this step. Round 32's
  anchor test proves the HUD *follows* the calibration, not that the calibration's centre is real. The
  **camera-to-axis boresight is not commissioned** and needs either a surveyed distant reference or a calibration
  board at wide spans. Tapes therefore read `JOINT TRAVEL, NOT HEADING`.
* **Data contract (§20)** — every field is supplied, including `camera.measurement_age_ms` (the calibration
  file's own mtime, aged per snapshot, null only when there is no calibration file) and the FOR envelope
  polygon; IMU is reported honestly as `ABSENT`. `test_section_20_ledger.py` drives a **FakeControld** it spawns (own
  socket, port 0) through webd's real serving path - that is the real *code* path, not the physical
  station; station-level evidence is the curl-verified material in `docs/evidence/`.
* **Staleness (§25)** — verified by freezing the daemon: telemetry age climbed 858 → 2464 ms with
  `telemetry_stale` true and the cue going non-green.

## Standing statement

The station is homed and ready, in MANUAL/HOLD, with a synthetic vision source running. **Synthetic targets are
plumbing evidence only; no acceptance item may be signed on them.** Until the real detector path exists on this
station, head-aim and lead claims are measurements of a generated target, not of the world.

Prepared by the agent that wrote the code, on 2026-09-04. That is a reason to distrust every line above.
