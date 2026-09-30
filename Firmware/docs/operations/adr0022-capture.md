# ADR-002.2 baseline acquisition

## What this is for

Capture complete CAN/IMU streams and establish measurement capabilities before
excitation. Current readiness is in [the Stage 2 report](../ADR-002.2/reports/STAGE2_READINESS.md).
This is not dynamics identification, controller acceptance or production validation.

## Where the work happens

Compile, simulate, inject failures and review evidence on the **workstation**. Local
rehearsals use loopback UDP and a pipe, with no CAN/I2C or SSH access. Physical acquisition
belongs on `eamars@rpi-turret` through the launcher after its applicable capability
and operating checks. The owner has explicitly removed the prior inventory failure
as a gate to Step 2. This path has **not** been deployed or physically
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
The producer retains its 1 kHz yaw cadence using bounded catch-up batches with actual
kernel receipt timestamps. A successful full-rate rehearsal also requires at least
99% of that offered rate; this is a load-test criterion, not a confidence percentage
or a physical controller acceptance threshold.

Check target executables against an explicitly prepared Debian ARM64 sysroot:

```bash
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_target_abi.py \
  --binary run/adr0022-debian13/firmware-make/axis_control_core/commissiond \
  --binary run/adr0022-debian13/firmware-make/imu-bno085 \
  --sysroot run/adr0022-debian13/root \
  --output run/adr0022-debian13/new-abi-report.json
```

For a local ARM64 kernel rehearsal, prepare separate `amd64/` and `arm64/` package
roots using authenticated Debian package downloads and extraction, without installing
them on the host. The host root supplies `qemu-system-arm` and its dependencies. The
guest root supplies an ARM64 kernel with `virtio_mmio.ko.xz`, `busybox-static`, Python
with `venv`, and the target libraries. The current prepared roots and exact package
identities are indexed in the readiness evidence. Run the probe before the matrix:

```bash
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_arm_vm.py \
  --packages-root run/adr0022-debian13/vm-packages \
  --build run/adr0022-debian13/firmware-make \
  --phase probe --output run/adr0022-debian13/new-vm-probe
# After the probe passes, use --phase matrix with another new output directory.
```

The VM boots an isolated ARM64 kernel, creates a guest project venv, runs the actual
target capture executable, and exports raw evidence through a virtual serial port.
It has no network or host-device passthrough. The matrix includes a 120-second capture,
native acquisition contracts, negative-register capability replies, and all 12
injected protocol/stream failures. A correlated negative read keeps its value null
and prevents repeating that register in the same baseline context; other streams
continue and dependent measurements remain unavailable. User-mode
QEMU is insufficient for this check: the tested version rejects both `SO_TIMESTAMPNS`
and `SO_RXQ_OVFL`. Do not bypass these requirements to make an emulator pass.

The implemented station entry is `run_application.sh run --capture-baseline MANIFEST`;
`check` validates files and topology without opening device transports. A physical
manifest requires schema `adr0022.capture/2`, provenance `MEASURED`, transport `socketcan`,
`yaw.interface=can0`, `pitch.interface=can1`, an absolute unused output path, expected
`commissiond` and IMU executable SHA-256 values, explicit timing/quality bounds, pitch
UID `7216313130333105`, `pitch_stop_poll=true` and confirmed support when pitch is disabled.
Never copy synthetic rehearsal timing bounds into a qualified physical contract.
The launcher binds the IMU pipe descriptor.

Prepare an acquisition-only release from a clean committed source tree. The input
manifest contains all physical fields above except `output` and `imu_fd`: deployment
assigns a fresh output path, and the launcher binds the descriptor. Binary hashes must
refer to the already checked ARM64 executables.

```bash
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_baseline_bundle.py pack \
  --build run/adr0022-debian13/firmware-make --manifest run/physical-baseline-template.json \
  --output run/new-baseline-bundle.tar
```

Pass that archive to `Firmware/tools/deploy_station.py --baseline-bundle FILE` with
the observed station address, pinned key and existing identity as in the deploy card.
The tool verifies source and content identity before upload and again in the new
release, then runs launcher `check`. It prints the bounded capture command with a
separate run directory. It opens no device, runs no target regression tests, installs
no packages and does not activate production. Execute the printed launcher command
once for the authorized physical acquisition and preserve its result. An existing
bundle output or capture identity cannot be overwritten or retried.

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
The ABI report verifies architecture, dependency closure and version labels against the
named sysroot. VM results add execution under the recorded ARM64 kernel and libraries.

## What it does not prove

Kernel timestamps describe host receipt. Register observations preserve their request/
response interval with `device_sample_ns=null`. Yaw temperature stays raw with Celsius
unknown; pitch uses documented 0.1 C units. STOP feedback does not qualify yaw settling,
pitch current mode, protection under load, or 3a/3b. `capture_complete` never implies
`physical_parameters_qualified` or `motion_authorized`.
Neither a Debian release name nor a local VM identifies the exact libraries installed
on the station. VM scheduling and loopback traffic do not measure the Pi's SocketCAN
drivers, CAN bus load, motor firmware, IMU transport or physical stopping behavior.

## When it fails

Loss, stale/reordered data, IMU generation changes, invalid source/UID, write echoes,
CAN faults, incomplete responses, queue exhaustion, disk errors and producer exit
invalidate capture. The bounded writer batches records without blocking acquisition,
rolling truncation or unbounded memory. Preserve incomplete files, attempt/result
records and stderr. Fix and verify locally. Output/attempt identities cannot be reused;
the supervisor does not restart automatically.

Version 2 requires zero final per-socket drop counts from `SO_MEMINFO` as well as
per-packet `SO_RXQ_OVFL` notifications. This detects loss that has not yet been reported
on a later received packet. The reviewer rejects absent/nonzero final counters and
older version-1 records; those older captures remain historical evidence under their
original software identity. Timing bounds smaller than one nanosecond are rejected
before sensor startup, because they cannot be represented by the acquisition clock.
