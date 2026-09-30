# ADR-002.2 neutral current-mode preparation and measurement

## What this is for

Acquire pitch's actual zero-command current, correlated mode/reference readback,
enabled observations, STOP and original disabled-mode restore. Keep hardware offset,
noise and transients as measured calibration inputs. The historical strict
`--prepare-current` operation remains available with its original result and guards;
the separate measurement operation is `--characterize-current`.

The owner's requirement is: "You do NOT reject the hardware observation. You shall
adapt the imperfection from the hardware, and your calibration and tuning is designed
to compensate for that." This operation supplies observations for that work; it does
not supply excitation, dynamic stopping, homing, calibrated-current or controller
qualification by itself.

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
  --output run/new-characterization-probe --characterize --fault neutral_noise
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_current_review.py \
  run/new-characterization-probe/capture.jsonl --characterization \
  --output run/new-characterization-review.json
# After this executable probe passes, run the --characterize --matrix rehearsal
# with another unused output directory and independently review its captures.
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_arm_vm.py \
  --packages-root run/adr0022-debian13/vm-packages \
  --build run/adr0022-debian13/firmware-make --phase characterization-probe \
  --output run/new-characterization-arm-probe
# After that probe passes, run characterization-matrix with another unused output.
```

The [capture card](adr0022-capture.md) describes the prepared compiler/sysroot and ABI
checks. Cross-build on the workstation. Use the same actual ARM64 executable in the VM
and the eventual physical bundle; record its exact hash, not only its source revision.

The physical measurement manifest uses schema `adr0022.neutral-characterization/1`,
provenance `MEASURED`, transport `socketcan`, purpose
`neutral_current_measurement_characterization`, the known pitch UID, the fixed CAN
topology, pitch support when disabled and explicit timing/temperature/displacement
bounds. Record `transition_displacement_bound_rad=0.01`,
`pitch_maximum_temperature_C=45` and `protection_current_bound_A=6.5`.
Declare `neutral_observation_s` explicitly: a finite positive observation duration no
greater than 60 seconds. The session `limits.duration_s` must exceed
`limits.startup_s + neutral_observation_s`. A 10-second physical observation is planned
but has not been executed at this revision; the manifest records the duration actually
requested. The historical diagnostic comparison does not control this duration.
The `neutral_current_bound_A=0.1` field is only a comparison to the unmeasured assumption
in the historical attempt. Readings above it remain raw calibration data; this
comparison is not a physical acceptance line or a reason to reject the hardware.

Bind `protection_limit_basis` to kind `manufacturer_continuous_current_rating`,
`continuous_current_A=6.5`, document
`docs/references/cybergear/CyberGear微电机使用说明书.pdf`, and SHA-256
`4fe8727a690193953e62438c04abd25f8e8be232e02b4eddf3aa1f99610da495`.
The launcher checks the exact retained PDF bytes before device access. The 23 A peak
rating/protocol range does not replace the continuous rating. These protections
authorize only zero-reference observation and do not declare calibrated measurement
uncertainty.

Include executable hashes, committed revision, source fingerprint and an embedded
matching `LOCAL_CURRENT_CHARACTERIZATION_PASS` local qualification report. That
report must be synthetic, state `hardware_accessed=false`, and bind the same revision,
source and binary hashes; its canonical report hash must match the manifest. Source
fingerprints canonicalize CRLF to LF for their explicit text dependency set;
executable fingerprints always use exact bytes.

The historical strict manifest remains `adr0022.current-preparation/1`, purpose
`neutral_current_mode_verification`, with matching `LOCAL_CURRENT_PREPARATION_PASS`
evidence and its original 0.1 A abort guard. Preserve its closed attempt as historical
evidence; do not relabel it as characterization.

Record `operator_attendance.present_at_manual_cutoff` truthfully. The owner's explicit
2026-09-30 authorization permits operation regardless of presence: an unattended session
records `session_authorization.unattended_operation_authorized=true` and
`presence_required=false`, with an identity for that actual authorization. Do not claim
an attended cutoff or independent automatic protection from this authorization.

Package the clean committed source and verified binaries using the acquisition bundle
tool, then deploy through `deploy_station.py --baseline-bundle FILE` with the existing
pinned SSH identity. This acquisition path supports baseline, strict neutral and
characterization manifests.
Deployment assigns a new output path and prints the appropriate launcher command:

```bash
OTA_RUN_DIR=<new-release>/run/stack bash <new-release>/Firmware/scripts/run_application.sh \
  run --characterize-current <manifest-path-printed-by-deployment>
```

Execute that bounded command once. It starts one IMU owner and one C++ CAN owner,
observes disabled pitch, writes a zero reference, selects current mode, reads mode and
reference back before enabling, reads both again after enable, and records actual
zero-command feedback for the manifest's `neutral_observation_s`. It then requires
fresh disabled STOP feedback before restoring and reading back the original mode.
Yaw receives only the shared current-zero frame.
Use launcher `status`/`stop`; preserve the immutable attempt, bound manifest, journal,
independent review, exact child identities and result before the next operation.

## What it proves

A local pass exercises real UDP sockets, kernel timestamps, an IMU pipe, recording,
process supervision and independent raw-wire review with synthetic devices. ARM64 VM
success adds the actual target executable and Linux kernel/library boundary. A physical
completion records the observed mode/readback/enable/STOP/restore sequence, actual
stream coverage and current samples within the manufacturer protection and other
guards. The independent reviewer retains sample values, timing and statistics for
offset/noise/quantization/timing calibration. It reports the historical 0.1 A comparison
separately, without invalidating the observed hardware data.

## What it does not prove

Zero current is not a brake. Yaw zero transmission does not certify physical stopping
or disable. This session does not measure pitch endpoints, dynamic stopping, current
scale accuracy, IMU mounting or sample/filter delay. It creates no PlantSnapshot,
controller candidate or 3a/3b certificate. Characterization always leaves neutral-current,
current-mode, dynamics and physical-parameter qualification false: measured calibration
and the later prescribed dynamic evidence are still required. Manual cutoff and
unqualified stopping after complete Pi/process/CAN loss remain separate physical facts.

## When it fails

Reject wrong UID/source/binary/authorization, echoes without matching reads, receive loss,
stale or invalid streams, unexpected drive state, displacement/manufacturer-current/
thermal violations and journal failure. The sole C++ owner requests neutral output and
STOP on a recoverable
process abort; confirmation requires fresh valid wire evidence. The supervisor preserves
STOP as unconfirmed when the collector is lost or forcibly terminated and sends no CAN
from Python. Keep the failed attempt; do not restart it automatically, widen a guard,
restore/enable after failure, or call a process exit a physical stop certificate.
Preserve readings above the historical 0.1 A comparison as valid raw observations.
