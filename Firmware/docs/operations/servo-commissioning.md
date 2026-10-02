# Servo commissioning: automatic identification, design and verification

## What this is for

Commissioning, or re-commissioning, the yaw position servo and the pitch speed-mode servo after
anything mechanical changes: a camera or payload, added or removed weights, a bearing service, a
motor swap. One command per axis does everything:

- identifies the plant;
- designs the controller from the identified plant and a simulator;
- checks the result on the station;
- writes `config/servo/<axis>_servo.json` when the fixed acceptance limits pass.

Not for production: these are bounded commissiond sessions, and production is not touched. Not for
diagnosing production tracking.

## Where the work happens

On the Windows workstation, with the tool itself running in WSL:

- **Orchestrator:** `commission.py`, with numpy/scipy and the native `servo-sim` simulator.
- **Build:** the ARM64 `commissiond` cross-build (the `run/adr0022-debian13` tree).
- **Deploy:** Windows `python.exe` runs `deploy_station.py` once per commissioning run, then every
  session runs in that one release.
- **Station access:** Windows `ssh.exe`/`scp.exe`, using this machine's own identity.

The Pi compiles nothing. Read [the deploy card](deploy.md) for the cross-build tree.

One-time setup, in WSL from the repository root:

```bash
python3 -m venv run/servo-commission/venv && run/servo-commission/venv/bin/pip install numpy scipy matplotlib pytest
```

```bash
cmake -S Firmware -B run/adr0022-local/firmware -DCMAKE_BUILD_TYPE=Release && make -C run/adr0022-local/firmware servo-sim
```

## The command

From Git Bash or PowerShell on the workstation:

```bash
wsl.exe -e bash -c "cd /mnt/c/workspace/OpenAutoTurret/Firmware/tools/servo_commission && ../../../run/servo-commission/venv/bin/python commission.py yaw"
```

Replace `yaw` with `pitch` for the other axis. Each run writes `run/servo-commission/<run id>/`,
containing `commission.log`, `report.json`, `report.md`, every session's manifest and journal, and
the candidate asset. Station time is about 25 minutes for yaw and 10 for pitch.

Order: commission **pitch first**. Each session leaves the other axis unpowered, where its own
friction holds it within one sensor count (measured). Pitch parks at its centre, so yaw is then
identified at a defined pitch posture.

| What changed | Run | What happens |
|---|---|---|
| Nothing known, or a new station | `commission.py yaw` and `commission.py pitch` | Everything is identified from [`yaw_prior.json`](../../config/servo/yaw_prior.json) / [`pitch_prior.json`](../../config/servo/pitch_prior.json): hardware facts and gains that are safe on any plausible plant. |
| Payload, camera, weights, bearing service | `commission.py check yaw` first. If it says `UPDATE_PARAMETERS`, run `commission.py yaw --update` | `check` is one 60 s probe scored against the asset. `--update` re-identifies friction and inertia, redesigns and re-verifies, and reuses the encoder crosstalk table (a payload does not change it). |
| Yaw motor replaced, remounted, or its firmware changed | `commission.py yaw` | The crosstalk table is a property of that motor and its mounting, so it is measured again. |
| Pitch, anything | `commission.py pitch` | The working window is the one production's homing measures: the endstops inset by 0.14 rad, recorded in `pitch_prior.json`. It is valid while the drive is not power-cycled; after a reboot, start production once so it homes, and update the window if the endstops moved. |
| Just look | `commission.py validate yaw` (two use-case passes), `commission.py session yaw usecase` | Both accept `--asset FILE`. |

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
3. **inertia.** Sliding at 30 deg/s while an 8–30 Hz current sweep is added. The response, with
   the sweep as the instrument, is fitted to rigid body + loop delay + leftover crosstalk.
4. **design (simulation).** `servo-sim` runs the identified plant through eight angles with steps
   and 5 and 20 deg/s ramps. Gains follow one loop shape scaled by inertia: kq = a·wn²,
   kv = a·wn (zeta 0.5), ki = 1.5·kq. For each compensation bias (1, 2, 3 mrad/A left in the
   reading on purpose) it finds the largest quiet wn with the table shifted by ± its uncertainty.
   It then keeps the bias whose simulated use case scores best.
5. **ladder (station).** At the two weakest angles, wn rises from 0.6 to 1.3 × the simulated
   boundary until the session's oscillation guard trips (fast current RMS above 0.3 A for 50 ms, stall
   rocks excluded) or the speed trip fires. The ladder gain is min(onset / 1.2, simulated boundary).
   The two angles are where the crosstalk scan's two directions disagree most and where the table
   is steepest.
6. **circle check (station).** A full circle each way at 1.1 × that gain must finish without the
   guard tripping, otherwise the gain drops 10% and the circle repeats (up to three times). The
   ladder only saw two angles.
7. **validate (station).** Two use-case passes.
   - **Gates** (`score.py` `YAW_GATES`): the session completes, ramps really track (speed ratio
     within 10%), and there are at most 4 stalls.
   - **Accuracy** is scored against the provisional `YAW_LIMITS` and reported as *advisory*. ADR-003's
     photography spec owns the real angular budget and has not set it yet.
   - The model's prediction of the same pass is recorded next to the measurement.
   - `commission.py rescore RUN_DIR` re-evaluates a recorded run with the current scorer, without
     the station.

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

- **Production behaviour.** Production yaw still uses the old speed PI, and pitch the old
  SpeedServo path, until ADR-003 integrates the servo.
- **Thermal qualification.** The authority is 3 A peak and 1.62 A RMS (owner ruling, 2026-10-02),
  protected by the 55 C motor-temperature trip; 0.8 A is guidance only.
- **Stop or park behaviour.** Sessions end with zero yaw current and pitch STOP.
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
- **`no gain with margin all the way round`.** Three circle checks tripped. Look for the angle in
  the circle journal; usually a local crosstalk-table error, so run `commission.py yaw` (not
  `--update`) to remeasure the table.
- **`NOT ACCEPTED` with stalls or long moves short.** The bearing stuck. Warm up (a full turn at
  30 deg/s) and rerun; if it persists, the stall-recovery thresholds in `design.servo` scale with
  the measured friction level.
- **`deploy failed`.** Read `deploy.log` in the run directory, and [the deploy card](deploy.md).
- **Pitch `native register readback differs`.** The drive's as-found settings differ from the
  prior's `native_settings`; the journal's `register_read` rows show the real values. Nothing was
  enabled.
