# ADR-002.2 yaw current acquisition

## What this is for

Record program-selected, finite yaw current excitation and its raw CAN/IMU response before model fitting. Pitch stays disabled at its present supported pose.

## Where the work happens

Build and run the normal-path executable probe on the workstation, then cross-compile ARM64 there. The Pi only runs the shipped binaries as `eamars@rpi-turret`, without sudo, under the existing launcher. Read the [execution contract](../ADR-002.2/00_CODEX_START.md), [yaw checklist](../ADR-002.2/YAW_TODO.md), [runbook](../STATION_OPERATIONS.md) and [deployment card](deploy.md).

## The command

From the workstation Linux environment, using the existing project venv:

```bash
cmake --build run/adr0022-local/firmware --target commissiond --parallel "$(nproc)"
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_yaw_rehearsal.py \
  --binary run/adr0022-local/firmware/axis_control_core/commissiond \
  --output run/adr0022-stage2/yaw-local-probe-01
cmake --build run/adr0022-debian13/firmware-make --target commissiond imu-bno085 --parallel "$(nproc)"
```

The physical manifest uses `adr0022.yaw-acquisition/1`, purpose `yaw_current_identification`, measured provenance, SocketCAN, installed pitch UID, `can0` yaw and `can1` pitch. Current segments are program-generated piecewise linear inputs. Record the current allowance, pre-stimulus baseline, zero-current observation, actual authorization and finite session deadline. Finite continuous-current captures can use the manufacturer's 0.9 A continuous stall allowance; a larger declared current bound remains a brief-pulse capability. No hash or synthetic qualification certificate is required.

An optional signed `yaw_displacement_target_deg` ends excitation when actual unwrapped encoder displacement reaches the requested endpoint. The reference is the fresh encoder sample at excitation start. The journal distinguishes target observation, excitation displacement and final coast after current zero. Waveform expiration without reaching the target remains recorded as `yaw_target_reached=false`; session completion does not assert angular coverage.

Package using `adr0022_baseline_bundle.py pack --session-label LABEL` and deploy using `deploy_station.py --baseline-bundle FILE`, with the existing SSH identity. Run the exact command printed by deployment:

```bash
OTA_RUN_DIR=<release>/run/stack bash <release>/Firmware/scripts/run_application.sh \
  run --acquire-yaw <deployed-manifest>
```

Freeze source edits and builds during the physical session. The C++ owner records requested, limited and successfully transmitted current, raw encoder/current/temperature, kernel receipt times and IMU samples. Pitch uses discovery, STOP and register reads only; no enable, mode, zero/save or calibration write occurs. Use launcher `status`/`stop` with the same run directory. Copy captures into ignored `run/` before analysis.

## What it proves

A completed run records the requested current sequence, actual feedback and the full zero-current observation window. The local rehearsal establishes the executable recording path with synthetic devices; the station run supplies physical observations.

## What it does not prove

Acquisition alone does not establish stopping, dynamics, sensor mounting, a complete plant model or 3a/3b acceptance. Yaw temperature remains raw. Current-zero transmission is not a disable or park certificate. All noise remains available for calibration.

## When it fails

Preserve the attempted waveform and raw observations. The C++ owner requests current zero and pitch STOP on interruption, transport/feedback failure or deadline exhaustion. Report whether output and feedback were actually observed. If an agent-added qualification gate blocks the next script, remove that gate first and rerun the same step; do not expand fault matrices or clean up unrelated code. Existing real protection remains active.
