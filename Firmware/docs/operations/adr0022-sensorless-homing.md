# ADR-002.2 pitch sensorless calibration

## What this is for

Measure pitch endpoint contacts, repeatability, encoder/native-position differences,
IMU observations and the measured midpoint using the shared sensorless homing routine.
The owner reports about 60 degrees of physical travel and no microswitches.

## Where the work happens

Cross-compile the changed acquisition executable on the workstation. The Pi runs the
ARM64 binaries as `eamars@rpi-turret`, without sudo, through the existing launcher.
The latest owner instruction removes hashing and synthetic qualification gates from
this operation. Read [ADR-002.2](../ADR-002.2/00_CODEX_START.md), the
[runbook](../STATION_OPERATIONS.md) and [deployment card](deploy.md) first.

## The command

Create an explicit `adr0022.sensorless-homing/1` manifest with the installed pitch UID,
`can0` yaw, `can1` pitch, the existing homing method, actual native-setting readbacks
and the owner's unattended-operation authorization. Use a human session label.
Package the workstation ARM64 build with `adr0022_baseline_bundle.py pack
--session-label LABEL`, then deploy it with `deploy_station.py --baseline-bundle FILE`.
This acquisition path archives the current Firmware files into a separate release;
it preserves the dirty Pi checkout and performs no station build or production activation.

Execute the exact release/manifest command printed by deployment once:

```bash
OTA_RUN_DIR=<release>/run/stack bash <release>/Firmware/scripts/run_application.sh \
  run --establish-homing <deployed-manifest>
```

The launcher owns one IMU process and one C++ CAN sender. Freeze source edits and builds
throughout the physical session. Python never sends CAN. `FullAxisHoming` uses native
speed approaches and position backoffs; its existing filtered contact detector
processes progress and persistent stall. Fresh native `MechPos` pins position references
during mode changes. Actual mode, gains, current limit, neutral references and STOP
are read back.

Calibration records actual contacts and scatter. Expected span and small encoder/native
position discrepancies do not reject observations. Raw encoder, current, temperature,
IMU and receipt timing remain in the capture. Existing electrical, thermal, mechanical,
transport-loss and bounded-session protections remain active.

After observing both endpoints, command their measured midpoint, record the response,
confirm pitch STOP and restore the original runtime settings while disabled. This path
sends no encoder-zero/save command and does not overwrite retained calibration. Preserve
journal, manifest, attempt, process records and result. Use launcher `status`/`stop`.

## What it proves

An actual capture records endpoint contacts, span/scatter, midpoint response, mode
readbacks and STOP observations for this session.

## What it does not prove

These observations alone do not qualify encoder mapping, current dynamics, IMU mounting,
timing, plant identification or Stage 3a/3b. Yaw zero is a request without a GM6020
stopped-state confirmation.

## When it fails

Actual device faults, failed output/readback, lost feedback, mechanical protection or
an exhausted execution deadline invoke STOP. Preserve the observed failure and whether
fresh disabled feedback confirmed STOP. Do not enable or retry automatically after abort.
Measurement noise is retained and filtered for calibration, without turning a small
residual into a station fault.
