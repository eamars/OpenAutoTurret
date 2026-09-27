# OpenAutoTurret v3.2
## Apache-HUD Visual Integration Revision

**Document status:** UI implementation handover / visual delta from v3.1  
**Version:** 3.2  
**Date:** 2026-09-03  
**Supersedes for UI presentation:** `open_auto_turret_v3_1_ui_hud_addendum.md` visual-layout guidance  
**Controller semantics:** unchanged from v3.0/v3.1  
**Reference mock:** `open_auto_turret_v3_2_apache_hud_reference_mock.html`  
**Visual source of truth:** generated Apache-inspired web HUD concept image approved by the user

---

# 1. Purpose of this revision

v3.2 does not change the controller architecture, mode semantics, target-selection semantics, safety authority, or telemetry ownership defined previously.

The change is specifically visual and interaction-oriented:

> the operator page shall look like a native sensor HUD laid directly over the camera feed, not a conventional web dashboard with cards placed around a video panel.

The approved concept image is now the **primary visual reference** for the operator UI.

The previous v3.1 document remains useful for functional semantics, but where its prose allows multiple visual interpretations, v3.2 narrows the implementation to the approved concept.

---

# 2. Frozen functional behavior

The following remain unchanged:

1. `controld` remains the only motor authority.
2. Normal operating modes remain:
   - `MANUAL`
   - `AUTO_TRACK`
   - `AUTO_ROAM`
3. Target selection persists independently of mode.
4. `AUTO_TRACK` follows the selected target whenever confidently visible.
5. Target loss remains `TRACKING -> COASTING -> LOST_HOLD`, with automatic reacquisition.
6. `AUTO_ROAM` scans the safe field of regard and does not automatically chase selected targets.
7. `MANUAL` requires explicit operator motion requests.
8. `STOP_MOTION` transitions through controlled braking to `MANUAL/HOLD`.
9. The browser HUD is display/interaction only; it is never safety-authoritative.
10. Production HUD rendering remains client-side and shall not be burned into the Pi video stream.

---

# 3. Primary visual rule

The default operator view shall resemble the approved concept image at first glance.

The viewport hierarchy is:

```text
camera image / video
    + sensor-style HUD symbology
    + target overlays
    + compact status symbology
    + compact Field-of-Regard inset
    + low-profile bottom status strip
    + small context controls at lower right
```

It is **not**:

```text
web page header
+ video card
+ side cards
+ large status cards
+ persistent control panels
```

The camera image must visually dominate the entire screen.

---

# 4. Reference composition

The reference concept uses a 16:9 frame. All positions below are relative to the camera viewport, not to the browser window chrome.

## 4.1 Top-left — mode block

Position: approximately `1%` from the left and `1–2%` from the top.

Presentation:

```text
AUTO TRACK
TRACKING
PERSON #1
```

Rules:

- first line uses HUD green and is the strongest line;
- second and third lines are white or near-white;
- block is compact, left aligned, and uses a thin border / low-opacity black backing;
- no product logo or conventional site header above it;
- the mode block is part of the camera HUD.

Equivalent states:

```text
MANUAL
HOLD
PERSON #1          optional if a target remains selected
```

```text
AUTO ROAM
SWEEP RIGHT
PERSON #1          optional if a target remains selected
```

---

# 5. Top-center — yaw travel tape

The yaw tape is a major visual anchor and must resemble the approved reference.

## 5.1 Placement

- horizontally centered in the upper 10–15% of the viewport;
- extends across roughly the middle 55–60% of the image width;
- visually separated from the top-left mode block and top-right health indicators.

## 5.2 Appearance

- thin horizontal baseline;
- fine tick marks at small angular increments;
- coarse ticks and labels at meaningful intervals;
- endpoints always show the software-safe travel limits;
- current yaw marker uses a small triangle/caret;
- actual numeric yaw appears in a compact outlined box directly beneath the current marker.

Reference state:

```text
-80°   -60   -40   -20    0    +20   +40   +60   +80°
                            ▲
                     YAW +22.4°
```

## 5.3 Semantics

The tape represents **logical joint travel**, not compass heading.

Do not render `N`, `E`, `S`, or `W` unless the product later gains a validated world-heading source and that feature is explicitly enabled.

---

# 6. Right-edge — pitch travel tape

## 6.1 Placement

- vertical tape close to the right image edge;
- approximately middle 40–45% of viewport height;
- labels placed to the right of the line where space permits;
- current value box sits adjacent to the current position marker.

## 6.2 Reference state

```text
+55°
  |
+30°
  |
 0°     ◄ current marker
  |
-30°
  |
-45°

PITCH
-6.8°
```

## 6.3 Visual hierarchy

- endpoint and coarse degree labels: green;
- fine ticks: dim green;
- current marker: bright green;
- value box: dark translucent fill with a thin green outline.

---

# 7. Center reticle

The center reticle must match the approved concept more closely than the earlier simple `-- ○ --` suggestion.

It shall use a sparse Apache-style sensor reticle language:

- four separated corner brackets around the central area;
- a thin vertical reference line above and below the center;
- short horizontal bars left and right of center;
- a small open central aiming/LOS marker;
- no filled center dot that hides the scene.

The center reticle always represents:

```text
actual camera optical axis
```

It never represents selected target, predicted target, requested LOS, or trajectory reference.

---

# 8. Top-right — health strip

The reference concept uses a compact row of small outlined indicators rather than one large health card.

Required items:

```text
CONNECTED
HOMED
VISION
IMU
SAFETY ALLOW
```

Visual rules:

- each item is a small translucent chip;
- healthy items contain a small green status dot;
- text remains near-white;
- chips have very low visual mass;
- no large header bar behind them.

Fault/degraded state color remains semantic:

```text
green  = healthy / allow
amber  = degraded / derate / caution
red    = fault / blocking condition
```

---

# 9. Target overlays

Targets are rendered directly over the video.

## 9.1 Candidate targets

Appearance:

- thin dashed or low-opacity green rectangle;
- compact label immediately above the box;
- label format:

```text
PERSON #2 67%
```

Candidate boxes must remain visually subordinate to the selected target.

## 9.2 Selected target

Appearance:

- solid brighter green rectangle;
- modest glow permitted;
- label immediately above box;
- label format:

```text
PERSON #1 94%
```

The selected target must be the most visually obvious detection box without becoming visually heavier than the center reticle and travel tapes.

## 9.3 Direct interaction

Visible confirmed target boxes are clickable/tappable:

```text
SELECT_TARGET(track_uuid)
```

Selection itself does not change mode.

---

# 10. Prediction cue

Prediction remains visually distinct from measurement.

The approved reference uses:

- amber dashed square;
- small amber `+` at its center;
- small `PRED` label;
- placed near the selected target but not touching its box.

Reference:

```text
PERSON #1 94%      PRED
┌────────┐       ┄ ┄ ┄ ┄
│ target │       ┆   + ┆
└────────┘       ┄ ┄ ┄ ┄
```

The prediction cue must not be green.

A long error vector across the image is not shown by default. If a short connector is used, it should be very subtle.

---

# 11. Field-of-Regard inset

The approved concept places a compact FOR inset at the lower left.

## 11.1 Visual form

The inset is not a full dashboard card.

It is a low-opacity HUD box containing:

- title `FIELD OF REGARD`;
- green safe-envelope polygon;
- white/current camera FOV rectangle;
- camera LOS center marker;
- selected target LOS marker;
- amber predicted LOS marker;
- short legend;
- `SAFE ENVELOPE` text.

## 11.2 Visual proportions

At a 16:9 desktop viewport, the inset should occupy approximately:

```text
width:  25–27% of viewport
height: 20–23% of viewport
```

It must remain small enough that the underlying camera scene is still dominant.

## 11.3 Coordinates

- video target boxes use normalized image coordinates;
- FOR coordinates use yaw/pitch degrees;
- camera FOV rectangle size derives from effective HFOV/VFOV;
- camera FOV center derives from actual yaw/pitch.

---

# 12. Bottom status strip

The approved concept uses a very thin bottom strip in the lower-left / lower-middle region.

Reference content:

```text
MODE  AUTO TRACK | STATE  TRACKING | TARGETS 3 | FPS 29 | AGE 34 ms | SAFETY ALLOW
```

Rules:

- strip must not span the entire width if not necessary;
- translucent black background;
- thin outline;
- keys in white/gray;
- state values in green;
- font substantially smaller than main yaw/pitch values;
- visually subordinate to the camera image.

If screen space becomes constrained, remove the strip before shrinking the camera viewport.

---

# 13. Lower-right context-control dock

This is the most visible interaction change from the earlier v3.1 prose.

Instead of relying only on hidden click targets or an abstract menu icon, the approved concept uses five small translucent HUD buttons anchored at the lower right:

```text
TARGETS
MODE
MANUAL
DIAG
MENU
```

## 13.1 Visual style

Each button:

- small square/vertical rectangle;
- dark translucent background;
- thin low-opacity outline;
- green line icon;
- small white label;
- no bright solid fill;
- no conventional Material-style raised card appearance.

The controls should look like part of the HUD overlay.

## 13.2 Interaction

Buttons open transient overlay drawers.

Only one drawer should be open at a time.

Drawers:

```text
TARGETS  -> target list / selection
MODE     -> MANUAL / AUTO TRACK / AUTO ROAM
MANUAL   -> dead-man jog, speed, frame, step, STOP MOTION
DIAG     -> engineering telemetry
MENU     -> HUD profile / system / supervisory actions
```

The drawer must overlay the video rather than resizing it.

---

# 14. Drawer visual language

Drawers should continue the HUD aesthetic rather than reintroducing the old dashboard design.

Required styling:

- black/translucent body;
- thin green/neutral border;
- monospaced HUD typography;
- compact spacing;
- limited use of boxes inside boxes;
- green for selected/healthy;
- amber for caution;
- red reserved for STOP/fault.

Do not turn the drawer into a conventional white settings panel.

---

# 15. Color tokens

Recommended initial palette matching the approved concept:

```css
--hud-green:       #95f58b;
--hud-green-dim:   rgba(149,245,139,.56);
--hud-green-faint: rgba(149,245,139,.22);
--hud-amber:       #f2b329;
--hud-red:         #ff5d5d;
--hud-white:       #edf2eb;
--hud-black:       rgba(3,6,5,.80);
--hud-line:        rgba(230,245,230,.24);
```

These values are reference values, not a requirement to hard-code the exact hex values in production.

The important requirement is the visual relationship:

```text
mostly monochrome image
+ sparse green HUD
+ amber prediction/caution
+ red only for fault/stop
```

---

# 16. Typography

The visual reference uses a narrow monospaced sensor-display appearance.

Recommended stack:

```css
font-family:
  "IBM Plex Mono",
  "Roboto Mono",
  "SFMono-Regular",
  Consolas,
  monospace;
```

Do not use a large proportional UI font for primary HUD labels.

Typography hierarchy:

```text
yaw/pitch current values       strongest numeric HUD text
mode name                      strong
mode phase / selected target   medium
scale labels                   medium-small
candidate labels               small
bottom status strip            small
FOR legend                     smallest normal readable size
```

---

# 17. Camera image treatment

Production video may be visible-light or another source; the approved concept is visually demonstrated with a monochrome/thermal sensor look.

The UI itself shall not assume that the source must always be thermal.

When a monochrome sensor mode is active, optional browser-only presentation may include:

- light contrast enhancement;
- subtle vignette;
- very subtle scanline/noise texture.

Do not add heavy decorative post-processing that reduces target readability.

---

# 18. HUD layering

Production DOM/render order:

```text
video stream                            z = 0
sensor presentation filter             z = 1
candidate targets                       z = 10
selected target                         z = 11
prediction cue                          z = 12
center reticle                          z = 20
yaw tape                                z = 20
pitch tape                              z = 20
mode block                              z = 20
health strip                            z = 20
FOR inset                               z = 20
bottom status strip                     z = 20
context dock                            z = 30
transient drawer                        z = 40
critical dialog                         z = 60
```

---

# 19. Recommended production component tree

```text
App
└── CameraViewport
    ├── VideoStream
    ├── SensorPresentationLayer
    ├── HudOverlay
    │   ├── ModeBlock
    │   ├── HealthStrip
    │   ├── YawTravelTape
    │   ├── PitchTravelTape
    │   ├── CenterSensorReticle
    │   ├── TargetOverlayLayer
    │   │   ├── CandidateTargetBox[]
    │   │   └── SelectedTargetBox
    │   ├── PredictionCue
    │   ├── OffscreenTargetCue
    │   ├── FieldOfRegardInset
    │   └── BottomStatusStrip
    ├── ContextDock
    └── OverlayDrawers
        ├── TargetDrawer
        ├── ModeDrawer
        ├── ManualDrawer
        ├── DiagnosticsDrawer
        └── MenuDrawer
```

The visual source of truth is the generated concept; the component boundaries above are implementation guidance.

---

# 20. Telemetry mapping

The visual revision does not require new motor-control primitives.

The existing UI snapshot remains sufficient when it provides:

```text
system.operating_mode
system.mode_phase
system.safety_action
system.connected
system.homed

axes.yaw.actual_deg
axes.yaw.soft_min_deg
axes.yaw.soft_max_deg
axes.pitch.actual_deg
axes.pitch.soft_min_deg
axes.pitch.soft_max_deg

camera.effective_hfov_deg
camera.effective_vfov_deg
camera.fps
camera.measurement_age_ms

field_of_regard.safe_envelope_points[]

target_selection.selected_track_uuid
target_selection.selected_label
target_selection.visibility
target_selection.confidence

tracks[].track_uuid
tracks[].display_label
tracks[].confidence
tracks[].state
tracks[].bbox_norm
tracks[].anchor_norm

prediction.valid
prediction.predicted_anchor_norm
prediction.predicted_los_yaw_deg
prediction.predicted_los_pitch_deg
prediction.horizon_ms

imu.present
imu.gravity_valid
imu.world_elevation_deg
```

---

# 21. State-specific visual deltas

## 21.1 AUTO_TRACK / TRACKING

Must visually match the approved concept most closely:

```text
AUTO TRACK / TRACKING / selected target
bright selected target box
dim candidate boxes
amber prediction cue
center reticle
yaw tape
pitch tape
FOR inset
health strip
bottom status strip
context dock
```

## 21.2 AUTO_TRACK / COASTING

Change only necessary semantic elements:

- top-left `TRACKING` becomes `COASTING`;
- selected measurement box fades/removes if no valid measurement remains;
- amber prediction remains while valid;
- safety/motion HUD remains stable.

## 21.3 AUTO_TRACK / LOST_HOLD

- top-left becomes `TARGET LOST / HOLDING` or equivalent compact state;
- no fake measurement box;
- last/predicted FOR marker may remain dim;
- prediction disappears when invalid/stale.

## 21.4 AUTO_ROAM

- top-left becomes `AUTO ROAM / SWEEP LEFT|RIGHT`;
- yaw/pitch tapes and FOR remain;
- detections may remain visible;
- selected target stays selected but is not visually presented as owning motion.

## 21.5 MANUAL

- top-left becomes `MANUAL / HOLD` or `MANUAL / JOG`;
- target boxes may remain visible;
- MANUAL drawer exposes jog controls;
- HUD tapes and FOR provide real-time motion feedback.

---

# 22. Safety presentation

Safety indications must fit the approved HUD without introducing large banners for normal states.

Normal:

```text
SAFETY ALLOW
```

green and compact.

Derate:

```text
DERATE
```

amber, including the relevant travel-tape edge or FOR boundary.

Brake:

```text
BRAKING
```

amber and more prominent.

Fault stop:

```text
FAULT
<short reason>
```

red and prominent enough to interrupt normal operation.

---

# 23. HTML reference mock behavior

The accompanying HTML mock is intentionally a **visual-fidelity reference**.

To guarantee that it resembles the approved concept exactly, the mock embeds the approved generated image as the underlying viewport snapshot and places semantic/interactive HTML hit regions and HUD-style drawers above it.

This is deliberate for the handover mock.

Production shall **not** use a baked screenshot for the live HUD.

Production shall instead recreate the visible elements using:

```text
live <video> / MJPEG / WebRTC stream
+
SVG/HTML overlay components
```

The mock is therefore used for:

- exact visual composition;
- approximate relative placement;
- drawer look and feel;
- target-click hit areas;
- control-dock interaction;
- semantic naming;
- source-level guidance for an implementation agent without vision capability.

---

# 24. Acceptance criteria — visual fidelity

A production implementation passes the visual acceptance check when, at normal desktop size:

- [ ] camera feed fills essentially the whole viewport;
- [ ] no conventional top navigation bar is visible;
- [ ] top-left mode block resembles the reference location and density;
- [ ] yaw tape occupies the upper center and clearly shows total safe travel;
- [ ] current yaw value is boxed beneath its marker;
- [ ] pitch tape sits on the right edge and clearly shows total safe travel;
- [ ] center reticle uses separated sensor-style brackets and open center space;
- [ ] top-right health indicators are small chips, not a large card;
- [ ] selected target is bright green;
- [ ] candidate targets are dim green;
- [ ] prediction is amber and dashed;
- [ ] FOR inset is compact and located at lower left;
- [ ] bottom status strip is thin and subordinate;
- [ ] five compact context buttons sit at lower right;
- [ ] drawers overlay the camera instead of resizing it;
- [ ] the overall impression is a coherent sensor HUD, not a dashboard.

---

# 25. Acceptance criteria — functional preservation

- [ ] mode shown from authoritative controller state;
- [ ] yaw/pitch shown from actual telemetry;
- [ ] safe endpoints shown from controller configuration/state;
- [ ] target boxes mapped correctly through rendered-video geometry;
- [ ] target click sends/selects `track_uuid` only;
- [ ] selection does not implicitly change mode;
- [ ] AUTO_TRACK uses selected valid target automatically;
- [ ] AUTO_ROAM does not chase detections by default;
- [ ] MANUAL jog remains lease/dead-man based;
- [ ] STOP MOTION retains controlled-brake semantics;
- [ ] stale telemetry stops visual interpolation and indicates stale/disconnected state;
- [ ] browser-rendered HUD never becomes the safety source of truth.

---

# 26. Explicit visual changes from the earlier reference mock

Remove from the normal viewport:

```text
sticky web header
brand/logo block
large video card frame
right-side dashboard column
large permanent target card
large permanent mode card
large permanent Field-of-Regard card
permanent diagnostics section
blue/cyan dashboard styling
```

Replace with:

```text
full-screen camera viewport
monochrome/thermal-friendly sensor presentation
green/white Apache-inspired HUD language
upper-center yaw tape
right-edge pitch tape
open sensor reticle
direct target overlays
amber prediction cue
compact lower-left FOR inset
thin lower status strip
compact lower-right context dock
transient HUD-style drawers
```

---

# 27. Final target state

The operator opens the web page and sees a single coherent sensor view.

At a glance, without opening a panel, the operator can identify:

```text
current primary mode
current mode phase
selected target
health / homed / vision / IMU / safety state
actual yaw and total safe yaw travel
actual pitch and total safe pitch travel
camera optical center
detected targets
selected target confidence
predicted target position
current camera FOV inside total field of regard
target count / FPS / measurement age
```

Only when an action is needed does the operator open one of the lower-right context controls.

That is the v3.2 operator UI target state.
