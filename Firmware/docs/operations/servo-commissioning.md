# Servo commissioning: automatic identification, design and verification

## What this is for

Commissioning, or re-commissioning, the yaw position servo and the pitch speed-mode servo after
anything mechanical changes: a camera or payload, added or removed weights, a bearing service, a
motor swap. The pipeline does everything:

- identifies the plant;
- designs the controller from the identified plant and a simulator;
- checks the result on the station;
- keeps the accepted assets.

On the station this is a **calibration run from the page** (ADR-005 D7): MENU › CALIBRATION, VERIFY,
RE-RUN or NEW…. The accepted parts go into the calibration record in force; #1 AS SHIPPED is never
measured into. The same pipeline (`commission.py`) also runs from the workstation over ssh, for
development and for a station that has no records yet.

Not for diagnosing production tracking.

## Where the work happens

**From the page (the station's own route).** Everything runs on the Pi:
- **controld** keeps the motors for the whole run, in CALIBRATING. It runs each session itself:
  the axis under test follows the session, and production holds the other.
- **calibd** (`tools/servo_commission/calibd.py`, started by the launcher) is the orchestrator. It
  builds each session's manifest, reads the journal controld writes, and fits and designs between
  sessions with numpy, scipy and the release's `servo-sim`.
- **Runs and journals** are kept in `run/state/calibration/runs/<run>-<kind>-<stamp>/`.
- **Accepted parts** are installed into the record; the replaced files go to `history/<run>/`.

Nothing is built or deployed for a run. Build plan and protocol:
[ADR-005 stage 2](../ADR-005/STAGE2.md).

**From the workstation (development).** On the Windows workstation, with the tool running in WSL:
- **Orchestrator:** `commission.py`, with numpy/scipy and the native `servo-sim` simulator.
- **Build:** the ARM64 `commissiond` cross-build (the `run/adr0022-debian13` tree).
- **Deploy:** Windows `python.exe` runs `deploy_station.py` once per commissioning run, then every
  session runs in that one release.
- **Station access:** Windows `ssh.exe`/`scp.exe`, using this machine's own identity.
- **Production must be stopped.** commissiond and controld both take the motion lock, and each
  commissiond session leaves the axis it does not test unpowered.

The Pi compiles nothing. Read [the deploy card](deploy.md) for the cross-build tree.

One-time setup for the workstation route, in WSL from the repository root:

```bash
python3 -m venv run/servo-commission/venv && run/servo-commission/venv/bin/pip install numpy scipy matplotlib pytest
```

```bash
cmake -S Firmware -B run/adr0022-local/firmware -DCMAKE_BUILD_TYPE=Release && make -C run/adr0022-local/firmware servo-sim
```

## The command

**From the page.** The station must be homed, with the IMU gyro fresh (every session needs it).

| Press | Runs | Station time |
|---|---|---|
| VERIFY (two presses), any record | `check yaw`: one 60 s probe scored against the record's asset. `REUSE_EXACT`, or `UPDATE_PARAMETERS` when a gate fails or the friction has drifted 25 % | ~1 min |
| RE-RUN (two presses), not #1 | `pitch`, then `yaw --update` | ~45 min |
| RE-RUN +ACCURACY | then `calibrate yaw` | ~71 min |
| RE-RUN +CROSSTALK | `yaw` instead of `yaw --update`: after a yaw motor change | ~47 min |
| NEW… with MEASURE IT NOW | makes a copy, switches to it, then measures it as RE-RUN +ACCURACY | ~71 min |

- **During a run** the drawer shows the steps, the session and its time, and calibd's last line.
  The dock is dimmed and the D-pad hidden.
- **STOP** (one press; STOP MOTION does the same) ends the session at once, holds both axes,
  installs nothing, and ends in MANUAL.
- **Afterwards** the station resumes the mode the run interrupted, and the panel says LAST RUN.
- **Developer only:** `run_calibration CHECK-PITCH` (the API, not the page) runs `check pitch`.

**From the workstation**, in Git Bash or PowerShell:

```bash
wsl.exe -e bash -c "cd /mnt/c/workspace/OpenAutoTurret/Firmware/tools/servo_commission && ../../../run/servo-commission/venv/bin/python commission.py yaw"
```

Replace `yaw` with `pitch` for the other axis. Each run writes `run/servo-commission/<run id>/`,
containing `commission.log`, `report.json`, `report.md`, every session's manifest and journal, and
the candidate asset. Station time is about 25 minutes for yaw (19 of them sessions) and 25 for
pitch: commissiond's own sessions add up to about 9 minutes, and the rest is the Pi's review of
each journal around them.

Order: commission **pitch first**. On the workstation route each session leaves the other axis
unpowered, where its own friction holds it within one sensor count (measured); from the page,
production holds it. Pitch parks at its centre, so yaw is then identified at a defined pitch posture.

| What changed | Run | What happens |
|---|---|---|
| Nothing known, or a new station | `commission.py yaw` and `commission.py pitch` | Everything is identified from [`yaw_prior.json`](../../config/servo/yaw_prior.json) / [`pitch_prior.json`](../../config/servo/pitch_prior.json): hardware facts and gains that are safe on any plausible plant. |
| Payload, camera, weights, bearing service | `commission.py check yaw` first. If it says `UPDATE_PARAMETERS`, run `commission.py yaw --update` | `check` is one 60 s probe scored against the asset. `--update` re-identifies friction and inertia, redesigns and re-verifies, and reuses the encoder crosstalk table (a payload does not change it). |
| Yaw motor replaced, remounted, or its firmware changed | `commission.py yaw` | The crosstalk table is a property of that motor and its mounting, so it is measured again. |
| Pitch, anything | `commission.py pitch` | The working window is the one production's homing measures: the endstops inset by 0.14 rad, recorded in `pitch_prior.json`. It is valid while the drive is not power-cycled; after a reboot, start production once so it homes, and update the window if the endstops moved. |
| Just look | `commission.py validate yaw` (two use-case passes), `commission.py session yaw usecase` | Both accept `--asset FILE`. |
| After a deliberate hardware change, once the new asset is in place | `commission.py calibrate yaw` (about 30 min) | Re-measures the yaw accuracy limits (below). Drift alone is not a reason: a conformance shortfall without a hardware change means inspect the hardware. |

What the yaw run does, and the rule behind each step (each is a fixed computation on the measured
numbers; the report records every number):

1. **survey.** Constant-speed stretches at 0.25–65 deg/s each way, plus full turns, using the
   prior's gains. The mean current gives the friction curve (Coulomb level, Stribeck excess,
   viscous slope, and a low-speed creep term). The residual by angle gives the 24-bin friction maps.
   A thermal note appears when sliding needs more than the 0.8 A guidance level.
2. **crosstalk.** A turn each way at 10 deg/s with an 80 Hz, 0.15 A probe tone. The ratio of the
   encoder's to the current's 80 Hz component gives the reading's current sensitivity, at 2.5 deg
   resolution; the 120-bin table carries a delay. The disagreement between the two directions is
   the table's uncertainty.
3. **inertia.** Sliding at 30 deg/s while a 0.3 A current sweep is added. The response, with
   the sweep as the instrument, is fitted to rigid body + loop delay + leftover crosstalk. The
   sweep's start follows the plant: the survey's speed changes give a rough inertia first
   (accelerating minus decelerating current at the same speed), and the sweep starts at the
   nearest of 8, 5.6, 4 or 3 Hz to twice the prior loop's resonance. The bare turret keeps its
   8–30 Hz, 10.5 s sweep; a payload of about 5× gets 3–30 Hz over 18 s (the session is 45 s
   instead of 30). Why: a rigid body answers with 1/(J w²), and at 8–30 Hz a 5× payload's
   response is smaller than the encoder's noise and the table's drift (PAYLOAD 10-24: no coherent
   band). The leftover crosstalk the fit reports is the table's drift since its scan; the log says
   so when it exceeds the scan's repeatability.
4. **design (simulation).** `servo-sim` runs the identified plant through eight angles with steps
   and 5 and 20 deg/s ramps. Gains follow one loop shape scaled by inertia: kq = a·wn²,
   kv = a·wn (zeta 0.5), ki = 3·kq. For each compensation bias (1, 2, 3 mrad/A left in the
   reading on purpose) it finds the largest quiet wn with the table shifted by ± (its uncertainty +
   a drift allowance of `crosstalk_drift_fraction` × the table's largest |g|; rule 1.0, so the loop
   survives losing the compensation). It then keeps the bias whose simulated use case scores best.
   Why the drift allowance: on 2026-10-02 the reading at 326 deg moved 3 mrad/A off the morning's
   table within the day, and gains designed against the scan's own ±0.48 mrad/A buzzed at standstill.
5. **ladder (station).** At the two weakest angles, wn rises from 0.6 to 1.3 × the simulated
   boundary until the session's oscillation guard trips (fast current RMS above 0.3 A for 50 ms, stall
   rocks excluded) or the speed trip fires. The ladder gain is min(onset / 1.2, robust design wn).
   `commission.py redesign yaw` re-derives the gains offline from an asset's identified plant under
   the current rules (no station); the asset records it under `provenance.redesigned`.
   The two angles are where the crosstalk scan's two directions disagree most and where the table
   is steepest.
6. **circle check (station).** A full circle each way at 1.1 × that gain must finish without the
   guard tripping, otherwise the gain drops 10% and the circle repeats (up to three times). The
   ladder only saw two angles.
7. **validate (station).** Two use-case passes.
   - **Gates** (`score.py` `YAW_GATES`): the session completes, ramps really track (speed ratio
     within 10%), and there are at most 4 stalls (counted inside the scored script). A gate failure
     writes no asset.
   - **Conformance:** the pass is scored against the calibrated accuracy limits in
     [`yaw_accuracy.json`](../../config/servo/yaw_accuracy.json) and any shortfall is logged as
     `ACCURACY BELOW THE CALIBRATED LIMITS`. The asset is still written: on a changed plant the
     new working servo is better than the old one. The shortfall is a capability finding, and the
     limits gate ADR-003 qualification.
   - The model's prediction of the same pass is recorded next to the measurement.
   - `commission.py rescore RUN_DIR` re-evaluates a recorded run with the current scorer, without
     the station.

**Accuracy limits (`commission.py calibrate yaw`).** Owner ruling, 2026-10-02: the yaw accuracy
limits come from the real tracking performance of the feedforward + feedback servo. The campaign
and the rule are fixed in `accuracy.py` and `commission.py`:
- **Campaign:** four positions 90 deg apart. At each one, a use-case pass with the asset and one
  with every plant feedforward term removed, in alternating order.
- **Rule:** per metric, limit = 1.25 × the worst feedforward + feedback pass, over these passes
  and the asset's own two validation passes. The metrics are pointing at rest, overshoot, ramp
  RMS and jitter, speed jitter, walking-profile RMS and moving peak.

A failed feedforward pass stops the campaign and writes nothing. `--resume DIR` re-scores the
passes already recorded and runs only the missing ones. The feedback-only passes give ADR-003's
motor-feedforward comparison. The 2026-10-02 result:
- **Limits:** rest 0.44 deg, ramp RMS 0.19 deg, walking profile 0.26 deg, moving peak 0.79 deg
  (10.7, 4.6, 6.3 and 19.2 px in the tracker frame).
- **Without motor feedforward:** ramp error 1.4–2.0×, walking-profile error 1.5×, rest error
  about the same.

Pitch works the same way:

1. A slow approach to the centre of the window, then a speed-reference sweep at hold, gives the
   drive's speed-loop delay and lag.
2. kp is set for a 60 deg phase margin (at most 60/s), with ki = kp²/40.
3. A kp ladder runs from 0.6 to 1.5×.
4. Two use-case passes validate (`PITCH_LIMITS`).

## What it proves

An accepted run proves several things:

- **The asset tracks the ADR-003 use case on this station today, twice:** hold, 0.1–2 deg steps,
  1–20 deg/s ramps both ways, a reversal, ±90 deg moves and the walking profile, within the fixed
  limits.
- **Every gain was derived from measurements made in the same run,** never copied from an older
  asset.
- **The gain has margin:** at least 10% at every angle (circle), and 20% below the measured onset
  at the weakest angles.

The pipeline itself is proved offline. `tests/test_pipeline.py` runs it against simulated stations
with hidden plants in `sim_cases/` (this station's estimate, and a 2.5× payload), and requires
inertia within about 10%, crosstalk within 0.6 mrad/A, and both cases accepted. It takes about a
minute:

```bash
wsl.exe -e bash -c "cd /mnt/c/workspace/OpenAutoTurret && run/servo-commission/venv/bin/python -m unittest discover -s Firmware/tools/servo_commission/tests"
```

## What it does not prove

- **Production behaviour.** Production runs these assets (ADR-003 3b), but with production's own
  guards and limits in front of them: an accepted asset is a tuned servo, not a qualified tracking
  pipeline.
- **Thermal qualification.** The authority is 3 A peak and 1.62 A RMS (owner ruling, 2026-10-02),
  protected by the 55 C motor-temperature trip; 0.8 A is guidance only.
- **Stop or park behaviour.** commissiond's sessions end with zero yaw current and pitch STOP; a run
  from the page ends with production holding both axes.
- **That the simulator predicts tracking error.** It predicts the stability boundary to within
  about 10–15% and identifies the plant. It does not model the bearing's rest-dependent breakaway,
  its cold/warm drift or the 240–255 deg bump, so its tracking-error predictions are optimistic.
  That is why the station decides gains and acceptance, and the report records prediction next to
  measurement.

## When it fails

- **`survey ... ran out of authority`.** Sliding friction needs more than the current budget (the
  cap or the RMS limit). That is a mechanical problem (bearing preload or grease, or binding), not a
  tuning problem.
- **`HARD_ABORT: servo oscillation` in survey, crosstalk or inertia.** The prior's gains are unsafe
  for this plant (an extreme inertia or crosstalk). Lower `kq`/`kv` in the prior and rerun. The
  guard did its job.
- **Ladder ends on `yaw speed limit` instead of `servo oscillation`.** The same limit cycle grew fast
  enough to reach the 20 ms encoder speed trip first; the ladder counts either as the onset.
- **commissiond's own pitch homing (`servo_trial.home_first`) is not station-qualified.** On
  2026-10-02 it tripped the 3 N·m torque guard, which is gravity plus the push with the 3 kg payload.
  With production's homing parameters it then hit the encoder speed guard: the homing state
  machine's torque-off "rearm" lets the loaded pitch drop. Commissioning therefore uses
  production's homed window. Making commissiond's homing hold the payload through the rearm is
  ADR-003 work.
- **Pitch `outside servo trial window`.** The pitch rests more than `window_margin_rad` outside the
  window (for example, after a power cycle). Run production once so it homes.
- **`MODEL INADEQUATE` in the design step.** The identified plant is unstable in simulation at
  every gain. This usually means the survey's slow plateaus stick-slipped (look for spreads above
  ±0.2 A in `report.json` "plateaus"), making the friction curve too steep. The pipeline then ladders
  a fixed range (wn 25–75) on the station, and the station alone sets the gain; that is reported. Not
  an error, but the model's prediction is meaningless for that run.
- **Ladder oscillation at the first step.** The model is far too optimistic. The ladder still
  brackets the onset from 0.6×. Check `report.json` "inertia" bands: low coherence means a bad sweep.
- **`inertia: only N coherent frequency bands`** (the run is NOT ACCEPTED, nothing installed).
  The rigid-body response did not rise above the encoder noise and the table's drift in the
  sweep's bands. PAYLOAD 10-24 (5×) failed this way with the fixed 8–30 Hz sweep; the sweep now
  starts lower for a heavier plant. If it happens with the 3 Hz sweep, look at the survey step's
  `inertia_from_speed_changes` (is the plant heavier than the prior's gains can hold? the 10× case
  limit-cycles under the prior in simulation) and at the sweep session's peak current and speed.
- **`no gain with margin all the way round`.** Three circle checks tripped. Look for the angle in
  the circle journal; usually a local crosstalk-table error, so run `commission.py yaw` (not
  `--update`) to remeasure the table.
- **`NOT ACCEPTED` with stalls or long moves short.** The bearing stuck. Warm up (a full turn at
  30 deg/s) and rerun; if it persists, the stall-recovery thresholds in `design.servo` scale with
  the measured friction level.
- **`deploy failed`** (workstation route). Read `deploy.log` in the run directory, and
  [the deploy card](deploy.md).
- **Page: `HOME first`, `the IMU gyro is not fresh`, `#1 is the shipped backup`.** The run gate.
  HOME, check the IMU (`imu-bno085`, the stuck-I2C1 incidents), or NEW… a copy to measure.
- **Page: the run ends `calibd is not answering`.** controld heard nothing from calibd for 15 s while
  no session ran. Read `calibd.log` in the stack's run directory. Nothing was installed.
- **Page: `NOT INSTALLED: the measured parts would make the record unusable`.** The accepted parts
  did not pass the record's own checks (a servo the gate refuses). The record is unchanged; the
  run's directory keeps them.
- **Page: the station faults when a session begins** (2026-10-24, fixed in `3a0e32f`). The control
  loop waited on the web thread's preparation of the session; see STAGE2.md "As built".
- **Pitch `native register readback differs`.** The drive's as-found settings differ from the
  prior's `native_settings`; the journal's `register_read` rows show the real values. Nothing was
  enabled.
