# Servo commissioning: yaw position servo and pitch speed-mode servo

## What this is for

Running the yaw position servo (`axis_control_core/servo.*`) and the pitch speed-mode servo trial
against ADR-003-shaped reference scripts on the station, and recalibrating their assets
(`config/servo/*.json`). Not for production: these are bounded commissiond sessions; the
production stack stays off and is not touched.

## Where the work happens

On the Windows workstation, in Git Bash: WSL cross-compiles `commissiond` for ARM64 (the existing
`run/adr0022-debian13` tree) and runs the numerics; Git Bash packs, deploys a separate release and
runs it over this machine's own ssh identity. The Pi compiles nothing. Read
[the deploy card](deploy.md) first.

## The command

```bash
bash Firmware/tools/servo_commission/run.sh yaw   yaw-usecase-01   --script usecase
bash Firmware/tools/servo_commission/run.sh pitch pitch-usecase-01
```

Yaw scripts: `usecase` (hold, 0.1–2 deg steps, 1–20 deg/s ramps both ways, reversal, ±90 deg,
walking-person profile), `probe` (short), `circle` (a stop every 45 deg, both ways: angle-dependent
stability), `ladder` (with `--opts '{"gain_schedule":[...]}'`), `crosstalk_scan`. Pitch runs its own
script after a slow move to the measured centre. The helper prints the session footer and a
per-segment score (`score.py`); journals land in ignored `run/servo-commission/<label>/`.

Crosstalk recalibration (after a motor, mount or firmware change):

```bash
bash Firmware/tools/servo_commission/run.sh yaw yaw-xtalk-01 --script crosstalk_scan \
  --opts '{"vmax":30,"excitation":{"amplitude_A":0.15,"f0_hz":80.0,"f1_hz":80.0001,"begin_s":66.0,"duration_s":125.0}}'
# then, in WSL, from Firmware/tools/servo_commission:
python crosstalk.py <journal> [<second journal>] --asset ../../config/servo/yaw_servo.json
```

## What it proves

A COMPLETE footer and its score show how this binary and these assets tracked these references on
this station today: tracking error per segment, speed ratio/jitter, stops, stalls recovered
(`"stalls"` in `servo_cycle` rows). Two passes that agree are the evidence; one pass is not, because
the cross-roller bearings change friction with angle, time and temperature.

## What it does not prove

Production behaviour (3b): production yaw still uses the old speed PI and pitch the old SpeedServo
path. Not thermal qualification of the 1.5 A peak (bounded by the 0.8 A RMS budget and a 55 C
trip). Not stop/park qualification: the session ends with zero yaw current and pitch STOP.

## When it fails

- `HARD_ABORT: yaw speed limit` / `servo following error limit` — the protection worked; read the
  `servo_cycle` rows before the trip. A 14–17 Hz bang-bang current means lost phase margin: check the
  crosstalk table is loaded and gains are not above kq 90 / kv 1.6.
- `MEASUREMENT_LIMITED: IMU stream stale` before control starts — manifest too large for the Pi to
  parse while draining the IMU pipe; keep reference tables at 50 Hz (the default).
- `capture writer failed` — a journal line over 4096 bytes; parameter records use 9 digits for this.
- Pitch `native register readback differs` — the drive's as-found settings differ from the asset's
  `native_settings`; the journal's `register_read` rows show the real values. Nothing was enabled.
- Pitch `outside servo trial window` — the axis left the measured working window; the window and
  endstop facts are in [the runbook](../STATION_OPERATIONS.md).
