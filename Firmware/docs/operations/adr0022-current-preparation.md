# ADR-002.2 neutral current-mode preparation

## What this is for

Verify pitch's zero-current mode transition, correlated readback, enabled observations
and return to its original disabled mode. This session supplies no excitation or
dynamic stopping, homing, calibrated-current or controller qualification.

## Where the work happens

Build, rehearse, inject failures and independently review evidence on the **workstation**.
The Pi runs only the already verified ARM64 executables, as `eamars`, under the launcher's
station motion lease. Read the [current readiness](../ADR-002.2/reports/STAGE2_READINESS.md)
and [deployment card](deploy.md). Never test a software correction on the station.

## The command

From the workstation's Linux environment, with the existing project venv:

```bash
cmake --build run/adr0022-local/firmware --target commissiond test_commission_capture -j$(nproc)
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_current_rehearsal.py \
  --binary run/adr0022-local/firmware/axis_control_core/commissiond \
  --output run/new-current-process-matrix --matrix
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_current_review.py \
  run/new-current-process-matrix/none/capture.jsonl --output run/new-current-review.json
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_arm_vm.py \
  --packages-root run/adr0022-debian13/vm-packages \
  --build run/adr0022-debian13/firmware-make --phase current-probe \
  --output run/new-current-arm-probe
# After that probe passes, run current-matrix with another unused output directory.
```

The [capture card](adr0022-capture.md) describes the prepared compiler/sysroot and ABI
checks. Cross-build on the workstation. Use the same actual ARM64 executable in the VM
and the eventual physical bundle; record its exact hash, not only its source revision.

The physical manifest uses schema `adr0022.current-preparation/1`, provenance `MEASURED`,
transport `socketcan`, purpose `neutral_current_mode_verification`, the known pitch UID,
the fixed CAN topology, pitch support when disabled and explicit timing/temperature/
neutral-current/displacement rejection bounds. These guards do not authorize nonzero
current or declare calibrated measurement uncertainty. Include executable hashes,
committed revision, source fingerprint and an embedded matching local qualification
report. Source fingerprints canonicalize CRLF to LF for their explicit text dependency
set; executable fingerprints always use exact bytes.

Record `operator_attendance.present_at_manual_cutoff` truthfully. The owner's explicit
2026-09-30 authorization permits operation regardless of presence: an unattended session
records `session_authorization.unattended_operation_authorized=true` and
`presence_required=false`, with an identity for that actual authorization. Do not claim
an attended cutoff or independent automatic protection from this authorization.

Package the clean committed source and verified binaries using the acquisition bundle
tool, then deploy through `deploy_station.py --baseline-bundle FILE` with the existing
pinned SSH identity. This acquisition path supports both baseline and neutral manifests.
Deployment assigns a new output path and prints the appropriate launcher command:

```bash
OTA_RUN_DIR=<new-release>/run/stack bash <new-release>/Firmware/scripts/run_application.sh \
  run --prepare-current <new-release>/run/baseline/manifest.json
```

Execute that bounded command once. It starts one IMU owner and one C++ CAN owner,
observes disabled pitch, writes a zero reference, selects current mode, reads mode and
reference back before enabling, reads both again after enable, and observes neutral
feedback for two seconds. It then requires fresh disabled STOP feedback before restoring
and reading back the original mode. Yaw receives only the shared current-zero frame.
Use launcher `status`/`stop`; preserve the immutable attempt, bound manifest, journal,
independent review, exact child identities and result before the next operation.

## What it proves

A local pass exercises real UDP sockets, kernel timestamps, an IMU pipe, recording,
process supervision and independent raw-wire review with synthetic devices. ARM64 VM
success adds the actual target executable and Linux kernel/library boundary. A physical
pass establishes only the observed neutral mode/readback/enable/STOP/restore sequence,
actual stream coverage and current/temperature/displacement within the stated guards.

## What it does not prove

Zero current is not a brake. Yaw zero transmission does not certify physical stopping
or disable. This session does not measure pitch endpoints, dynamic stopping, current
scale accuracy, IMU mounting or sample/filter delay. It creates no PlantSnapshot,
controller candidate or 3a/3b certificate. Manual cutoff and unqualified stopping after
complete Pi/process/CAN loss remain separate physical facts.

## When it fails

Reject wrong UID/source/binary/authorization, echoes without matching reads, receive loss,
stale or invalid streams, unexpected drive state, displacement/current/thermal violations,
and journal failure. The sole C++ owner requests neutral output and STOP on a recoverable
process abort; confirmation requires fresh valid wire evidence. The supervisor preserves
STOP as unconfirmed when the collector is lost or forcibly terminated and sends no CAN
from Python. Keep the failed attempt; do not restart it automatically, widen a guard,
restore/enable after failure, or call a process exit a physical stop certificate.
