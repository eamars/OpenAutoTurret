# ADR-002.2 baseline acquisition

## What this is for

Capture complete CAN/IMU streams and establish measurement capabilities before
excitation. Current readiness is in [the Stage 2 report](../ADR-002.2/reports/STAGE2_READINESS.md).
This is not dynamics identification, controller acceptance or production validation.

## Where the work happens

Compile, simulate, inject failures and review evidence on the **workstation**. Local
rehearsals use loopback UDP and a pipe, with no CAN/I2C or SSH access. Physical acquisition
belongs on `eamars@rpi-turret` through the launcher after readiness and the current
confidence requirement are established. This path has **not** been deployed or physically
qualified. Do not use the station to verify a correction.

## The command

From the repository root in the workstation's Linux environment:

```bash
cmake -S Firmware -B run/adr0022-local/firmware -DOTA_BUILD_TESTS=ON
cmake --build run/adr0022-local/firmware --target commissiond imu-bno085 test_commission_capture -j4
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_capture_rehearsal.py \
  --binary run/adr0022-local/firmware/axis_control_core/commissiond \
  --output run/adr0022-stage2/new-local-rehearsal --duration 120
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_capture_review.py \
  run/adr0022-stage2/new-local-rehearsal/capture.jsonl \
  --output run/adr0022-stage2/new-local-rehearsal/review.json
```

Use a fresh output directory per local rehearsal. Dependencies belong in the project
venv: [capture tests](../../commission_runtime/requirements-test.txt) and
[mathematical tests](../ADR-002.2/requirements-offline.txt).

The implemented station entry is `run_application.sh run --capture-baseline MANIFEST`;
`check` validates files and topology without opening device transports. A physical
manifest requires schema `adr0022.capture/1`, provenance `MEASURED`, transport `socketcan`,
`yaw.interface=can0`, `pitch.interface=can1`, an absolute unused output path, expected
`commissiond` and IMU executable SHA-256 values, explicit timing/quality bounds, pitch
UID `7216313130333105`, `pitch_stop_poll=true` and confirmed support when pitch is disabled.
Never copy synthetic rehearsal timing bounds into a qualified physical contract.
The launcher binds the IMU pipe descriptor.

Startup acquires the existing motion lease, checks other device consumers, creates
an exclusive attempt record, starts the IMU without recovery, and runs `commissiond`.
UID discovery precedes normal pitch STOP polling (all-zero payload, no fault clearing).
STOP supplies fresh disabled status and temperature; a stopped drive need not send
unsolicited feedback. Register reads are asynchronous and correlated, with no retry
after timeout. No enable, mode write, motion reference, mechanical zero or yaw output
is emitted. Pitch STOP does change the drive's enabled state.

## What it proves

Local success demonstrates the exercised Linux receive, pipe, recording and supervision
behavior with synthetic responses. The reviewer reports each sensor's actual sample
rate, scheduling delay, pitch Iqf and temperatures. It never adds four IMU stream rates
and labels the sum as gyro rate. Physical recording would still require calibration.

## What it does not prove

Kernel timestamps describe host receipt. Register observations preserve their request/
response interval with `device_sample_ns=null`. Yaw temperature stays raw with Celsius
unknown; pitch uses documented 0.1 C units. STOP feedback does not qualify yaw settling,
pitch current mode, protection under load, or 3a/3b. `capture_complete` never implies
`physical_parameters_qualified` or `motion_authorized`.

## When it fails

Loss, stale/reordered data, IMU generation changes, invalid source/UID, write echoes,
CAN faults, incomplete responses, queue exhaustion, disk errors and producer exit
invalidate capture. The bounded writer batches records without blocking acquisition,
rolling truncation or unbounded memory. Preserve incomplete files, attempt/result
records and stderr. Fix and verify locally. Output/attempt identities cannot be reused;
the supervisor does not restart automatically.
