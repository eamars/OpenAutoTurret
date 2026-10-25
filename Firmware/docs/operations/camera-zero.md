# Camera zero: both crosshairs on the payload's line of sight

## What this is for

Putting each camera's crosshair (the reticle the tracker centres targets on) where the payload
points, for the calibration record in force. Do it:
- after a new payload is mounted, or an existing one is remounted;
- after a camera's installation offset changes;
- on a record that has none yet.

Design, decisions and the measurements so far: [ADR-006](../ADR-006/ADR-006.md).

Not for:
- **The servos:** use [servo-commissioning](servo-commissioning.md).
- **The cameras' rotation on the pitch arm** (the extrinsic): a payload swap does not change it,
  because the camera box sits on the pitch axis, apart from the payload rail. A remounted camera box
  needs the extrinsic refit, ADR-005 stage 3, which is not built.

**The idea in one paragraph.** The payload is pointed at a round target at a known distance. Each
camera's installation offset from the payload's aim axis is entered, measured by hand in mm. Then
the target's position in each camera's picture is given to the station, by a tap on the page or
measured from snapshots. controld solves that camera's pointing so its crosshair lands exactly on
the target at that distance (`zero_bore`, ADR-006 D1). The offsets then carry the crosshair to
every other distance, through the parallax offset/distance. The zero is stored in the record's
`bore.yaml`.

## Where the work happens

- **On the station.** controld refuses, solves and stores; it writes
  `run/state/calibration/records/<id>/bore.yaml` on its writer thread. Nothing is built, deployed or
  restarted.
- **From the page, in any browser:** the operator taps the target.
- **From the workstation, for an agent:** `Firmware/tools/camera_zero.py` reads
  `http://<station>:8080` (snapshots, state) and sends the same commands. Run it under Python with
  numpy, scipy and Pillow: the WSL test venv (`~/venvs/oattest`) has them.
- **The owner places the target and measures its distance.** The station cannot see the payload's
  own aim; the payload is pointed by the payload's means (its optic, a laser).

## The command

Before you start:
- **The record:** any but #1 AS SHIPPED, which is locked. NEW… makes a copy.
- **The station:** homed.
- **The target:** a round part of one colour (yellow by default), at a known distance from the
  payload. Indoors, 11 m was used on 2026-10-25.

### From the page (the operator)

MENU › CALIBRATION, then the BORE and ZERO sections at the bottom of the panel.

1. **HOLD HERE TO TRIM,** shown while the turret is in an AUTO mode. The turret stops where it is
   (MANUAL), and the panel says which mode comes back when you leave calibration. Already in MANUAL,
   there is nothing to press. Everything below needs this still turret, BORE ON included.
2. **BORE: ON.** With it off, the tracker aims the camera centre and nothing here acts.
3. **ZERO › WIDE and DETAIL: the installation offsets.** Each camera's position relative to the
   payload's aim axis, seen from behind looking downrange: right, up and forward, in mm. A new
   offset clears that camera's zero.
4. **ZERO › TARGET: the distance, in metres, from the payload** (0.5 to 5000). The page keeps it
   only while calibration is open: leaving forgets it, and the tap needs it typed again.
5. **Point the payload at the target** with the D-pad (FINE for the last steps). Check that the
   payload itself is on the target.
6. **Wait for TURRET: SETTLED:** both axes still within 0.3 mrad over 0.6 s. The turret settles
   inside its hold band, a few mrad from where it was sent, so check again that the payload is
   still on the target.
7. **Put the detail camera on the main display** (the PIP's *swap*), then **TAP THE TARGET ·
   DETAIL**.
   - The panel steps aside, and a bar says which picture to tap (CANCEL ends it).
   - Tap the target's centre on the main picture. The detail camera is first because it is the
     finer one: 0.12 mrad per pixel.
   - The crosshair jumps onto the tap, and the camera's row reads ZEROED <date> AT <distance>.
8. **Swap to the wide camera,** then **TAP THE TARGET · WIDE** the same way. Nothing moves when
   cameras swap.
9. **Fine clicks, if a crosshair is not quite on.** WIDE / DETAIL R and U, at STEP 0.1, 1 or
   10 mrad per press. A click marks the record's bore TRIMMED.
10. **Check:** both crosshairs on the target, before and after a swap.
11. **Leave:** ‹ MENU, or any dock button.
    - The page puts back the camera that was on the main display.
    - It resumes the mode that HOLD HERE interrupted.
    - Press STOP MOTION before leaving to stay held instead.

RANGE (in the BORE section) is the working distance: the range the tracker assumes and the
crosshairs are drawn for. A zero sets it to the target's distance. It can be changed afterwards
without a new zero.

### From the workstation (an agent, or anyone who prefers measuring to tapping)

The same zero, measured from full-resolution snapshots. About 20 pairs are averaged, and a circle is
fitted to the target's outline. On 2026-10-25 one pair spread 0.02–0.04 mrad in the detail picture
and 0.1–0.15 in the wide one, so the mean of 20 was good to about 0.06 mrad. A tap is good to about
a pixel of the picture tapped: 0.12 mrad on the detail camera, but 2.2 on the wide one, whose preview
is 640×360. The wide camera's fine clicks (step 9), or this route, finish what the tap starts.

**Who does what.** The agent cannot see where the payload points. The owner places the target,
measures its distance and says when the payload is on it. On a record with no payload (NO_LOAD),
"on the target" means the target is near the wide picture's centre. `manual_step` moves in steps
no smaller than 0.5°, about 9 mrad, which is enough to bring the target into the search window
and no finer.

**The Python.** A venv built from `Firmware/requirements-test.txt`, which brings numpy, scipy and
Pillow. On this workstation that is WSL's `~/venvs/oattest`:
`source ~/venvs/oattest/bin/activate`, then work from `Firmware/tools/`. The station's address is
`--address`, default 192.168.2.103.

1. **Hold, bore on, offsets** (skip what is already so). `send` posts one command and prints
   controld's answer:

   ```bash
   python camera_zero.py send set_mode MANUAL
   ```

   ```bash
   python camera_zero.py send set_bore_mode on
   ```

   ```bash
   python camera_zero.py send set_bore_offset detail 145 40 0
   ```

   The last line's numbers are an example (mm, seen from behind); do the same for `wide`. Leave the
   offsets alone if the record already has the right ones: a new offset clears that camera's zero.
2. **Check before measuring:**
   - the record in force is the one to zero, not #1 (the page's IN FORCE, or `/api/state`
     `calibration_record`);
   - its offsets are right for what is mounted;
   - both cameras run: the PIP shows a picture;
   - the owner says the payload is on the target;
   - the turret has settled (the tool reports how far it moved while measuring).
3. **Measure, without sending anything:**

   ```bash
   python camera_zero.py measure --pairs 20 --distance 11 --save ~/camera-zero
   ```

   - **It prints:**
     - each camera's target size and circle-fit rms;
     - the target's centre in the wide frame, and its spread;
     - how far the turret moved while measuring;
     - how far each crosshair is from the target;
     - the two `zero_bore` commands.
   - **Look at** `~/camera-zero/target_detail.png` and `target_wide.png`: the fitted pixels are
     tinted, and the circle and its centre are drawn. Go on only if the tint is the target, all of
     it, and nothing else.
4. **Apply.** This measures again and sends that measurement, so keep `--save` to see what was sent:

   ```bash
   python camera_zero.py measure --pairs 20 --distance 11 --save ~/camera-zero --apply
   ```

   - **It refuses** (exit 1, nothing sent) outside a MANUAL hold, with the bore off, on #1, while a
     switch holds, or when the measurement is not good enough. That means a spread over 0.3 mrad, a
     circle-fit rms over 1 px, or a turret that moved more than 0.3 mrad.
   - **Before sending** it prints the record and its current zero (trims, offsets, range): that is
     how to put it back: `trim_bore` with the difference restores the old trims.
   - **It sends** `zero_bore detail …`, then `zero_bore wide …`, and checks 6 fresh pairs.
5. **Done when** the check says both crosshairs are on it: within 0.1 mrad for the detail camera and
   0.2 mrad for the wide one. Otherwise measure and apply again; a zero replaces the last one.

   ```bash
   python camera_zero.py check
   ```

   `check` exits 0 when both are on, and 1 otherwise. With the bore off it says there are no
   crosshairs to compare.
6. **Leave the station as you found it:** normally `send set_mode AUTO_ROAM`, with the wide camera on
   the main display.

**If it stops halfway.**
- **HALF APPLIED** (the detail camera zeroed, the wide one refused): fix the cause the refusal
  names, then apply again.
- **"no answer from the station":** the command may still have been applied. Run `check`, and
  look at the record's bore on the page.

Options:
- `--wide-at X Y` / `--detail-at X Y`: where the target is in each picture, when it is not near
  the centre.
- `--wide-window` / `--detail-window`: the search radius in pixels, default 60 and 150.
- `--exclude-bottom 0.2`: when something cuts the target's lower edge. On 2026-10-25 a chair's seat
  did, and a whole-outline fit sat 0.6 mrad high. A cut outline shows as a circle-fit rms above
  about 1 px.

### The station's commands

The page and the tool send these through `POST /api/command` with `{"command": ..., "arg": ...}`.
controld's answer is the ack in `GET /api/state` (`cmd_ack_command`, `cmd_ack_accepted`,
`cmd_ack_reason`).

| Command | Does | Allowed |
|---|---|---|
| `set_mode MANUAL` | holds where the turret is | homed |
| `set_bore_mode on` / `off` | the tracker aims the bore / the camera centre | MANUAL hold or Shutdown, not #1 |
| `set_bore_offset <wide\|detail> <right> <up> <forward>` | the installation offset, mm, at most 1000; clears that camera's zero | the same, bore on |
| `zero_bore <wide\|detail> <metres> <u> <v>` | the zero: (u, v) is the target in the wide frame (the detail picture through its view scale) | **MANUAL hold only**, bore on, not #1 |
| `trim_bore <wide\|detail> <right\|up> <mrad>` | adds a click, rounded to 0.1 mrad, at most 100 | MANUAL hold or Shutdown, bore on, not #1 |
| `set_bore_range <metres\|inf>` | the working distance | the same |
| `manual_step yaw±<0.5\|1\|5>` / `pitch±…` | moves the turret in degrees: `yaw-` turns the cameras right, `pitch-` raises them | MANUAL hold |

The main display is `POST /api/camera/main` with `{"role": "wide"|"detail"}`.

### Two cases met so far

- **No payload (a bare turret, record NO_LOAD).** The wide camera is the reference: offsets wide 0 0
  0, detail relative to it (−10, +60, 0 on this turret). The zero puts the detail crosshair onto
  the wide one, so a swap does not step.
- **A zero copied to another record** (CALI_3_5KG_LOAD, 2026-10-25), while the real payload is
  away.
  1. Put that record in force: MENU › CALIBRATION, its chip, or `set_calibration_record <id>`.
  2. Turn its bore on, and enter its payload offsets.
  3. Add the source record's pointing as trims: `trim_bore` adds to what is there, so send the
     difference.
  4. Set the same RANGE.
  5. Put the original record back in force.

  The copied record's bore reads TRIMMED, not ZEROED, until its own payload is zeroed.

## What it proves

- **At the target's distance, each crosshair sits on the target** as that camera saw it, when the
  payload was on it. A tap or a measurement gives the station the point; controld lands the
  crosshair there exactly, by construction (ADR-006 D1). `camera_zero.py check` reads the residual;
  after the 2026-10-25 zero, fresh snapshots put the target within 0.03 mrad of both crosshairs.
- **The zero is in the record** (`bore.yaml`). It survives a restart, a deploy and a record switch,
  and comes back with the record.
- **The crosshair stays on the target through a camera swap:** each camera's crosshair comes from
  its own zero.

## What it does not prove

- **That the payload was on the target.** The zero is exactly as good as the payload's own aim when
  it was made: the station cannot see it. The hold band can leave the turret a few mrad from where
  the D-pad sent it, which is why step 6 says to look again.
- **Other distances.** These come from the offsets, by parallax. An offset 5 mm out is 0.5 mrad at
  10 m and 0.05 mrad at 100 m (ADR-006 risk 1).
- **Tracking accuracy.** That is the servo's: its limits are in the record (servo-commissioning).
- **That the tracker's bearing does not step at a swap.** ADR-006 D4 maps a detail detection through
  both cameras' zeros, and its unit tests show one bearing for both. The AUTO_TRACK check on the
  station (ADR-006 D7) was not run: the owner chose the static check.
- **Anything away from the crosshair:** roll between the cameras is not modelled (measured within
  0.5°).
- **The wide-picture defaults of `camera_zero.py`.** Its colours were checked on a real detail
  snapshot: it agrees with the 2026-10-25 zero to 0.06 mrad. In the wide picture they are the range
  that found the target on 2026-10-25, but the tool has not yet been run on a real wide picture.
  The first time, look at `target_wide.png`.

## When it fails

| What you see | Why, and what to do |
|---|---|
| "#1 is the shipped backup and stays as shipped" (controld, and the tool before it sends) | #1 is locked. NEW… makes a copy; zero the copy. |
| "turn the bore on first" | BORE: ON. |
| "zero in a MANUAL hold" / "trim in MANUAL" | HOLD HERE TO TRIM (MENU › CALIBRATION), or `set_mode MANUAL`. In Shutdown a trim works, a zero does not: the payload must be held on the target. |
| "a switch is holding the turret" | A record switch is in progress; wait for it. |
| "parked: trim in a MANUAL hold or in Shutdown" | The turret is on its rest park: leave it with MODE › MANUAL (it lifts pitch first), then zero. |
| "the zero needs R … U … mrad, more than 100" | The tap, the offsets or the distance are wrong by far more than a mounting error: check them, and that the payload is on the target. |
| "no zero: …" | The solver found no pointing that puts the target there: usually an offset or a distance far out, or a tap outside the picture. |
| "an installation offset is at most 1000 mm either way" / "a trim is at most 100 mrad either way" | A typo, or the wrong unit: offsets are mm, trims mrad. |
| "the wide/detail bore would not be valid" | The offsets put the target behind or outside a camera: check the offset signs (seen from behind, right and up are positive). |
| The tap bar says THE TURRET IS MOVING, or NO LIVE DATA FROM THE STATION | The tap waits for a still turret and live telemetry; it stays armed. A frozen video under live telemetry is not detected: watch the rate under the PIP. |
| The tap row is grey | The panel says why: the bore off, no distance, not in a MANUAL hold, or not settled. |
| The armed tap ends by itself | The cameras swapped, the hold ended, or another record came into force. Arm it again. |
| `camera_zero.py`: "no target-coloured pixels" / "touches the search window's edge" / "too small" | The target is not near the picture's centre (give `--wide-at` / `--detail-at`), the window is too small, or the target is too far away for the wide camera. |
| `camera_zero.py`: a large spread or circle-fit rms, or "not applied: …" | Something cuts the target (`--exclude-bottom`), the background has the target's colour (a smaller window), or the turret is still moving. The refusal names which. |
| `camera_zero.py`: "the station publishes no detail view scale" | The detail camera is not running in this boot: the PIP has no picture. A zero needs both. |
| One crosshair moved after the offsets changed | A new offset clears that camera's zero, and the tracker uses the new offset with the old pointing: zero that camera again. |

The calibration panel's lifecycle (what leaving restores, and what a STOP changes) is in the
runbook, [STATION_OPERATIONS](../STATION_OPERATIONS.md) "Calibration ends where it found the
station". All of calibration's failure modes are in [ADR-006](../ADR-006/ADR-006.md) section 6.
