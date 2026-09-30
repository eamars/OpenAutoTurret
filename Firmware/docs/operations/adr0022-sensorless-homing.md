# ADR-002.2 pitch sensorless homing

## What this is for

Observe pitch endpoints, repeatability and midpoint with the existing sensorless
homing routine; use the separate current-mode operation for mode 3 observations.

## Where the work happens

Compile, validate the manifest, run realistic executable probes, inject failures and
review raw evidence on the **workstation**. Run the same ARM64 executable in the local
ARM64 Linux VM before packaging it. The Pi runs only the verified release as `eamars`
under the launcher's sole station motion lease. Read the
[readiness report](../ADR-002.2/reports/STAGE2_READINESS.md),
[deployment card](deploy.md) and [runbook](../STATION_OPERATIONS.md) first.
Physical homing is **NOT_RUN** at this card's introduction; the commands below describe
the required local qualification path, not an existing physical certificate.

## The command

From the workstation's Linux environment, with its existing project venv:

```bash
cmake --build run/adr0022-local/firmware --target commissiond test_commission_capture -j$(nproc)
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_homing_rehearsal.py \
  --binary run/adr0022-local/firmware/axis_control_core/commissiond \
  --output run/new-homing-first-probe
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_homing_review.py \
  run/new-homing-first-probe/capture.jsonl --output run/new-homing-first-review.json
```

After the first executable probe and independent review pass, run the same rehearsal
with `--matrix` and a new output directory. Review its successful captures and failed
attempts without changing their evidence. The fixture must exercise the actual mode
transition, native position-reference readback and enable-time reference behavior;
an idealized device that omits those behaviors cannot qualify this integration.

Cross-build on the workstation as described by the
[capture card](adr0022-capture.md), then use the target binary in the isolated VM:

```bash
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_arm_vm.py \
  --packages-root run/adr0022-debian13/vm-packages \
  --build run/adr0022-debian13/firmware-make --phase homing-probe \
  --output run/new-homing-arm-probe
# After that probe and raw review pass, use homing-matrix with a new output directory.
```

The homing reviewer and VM phases are being implemented and have not been qualified
at this card's introduction. Close their evidence before the physical operation.
Preserve exact executable hashes and the source identity used by every probe. The
ARM64 executable in the eventual bundle must be the one that passed those checks.

Use schema `adr0022.sensorless-homing/1`, purpose `pitch_sensorless_homing`, provenance
`MEASURED`, transport `socketcan`, the known pitch UID, `yaw.interface=can0` and
`pitch.interface=can1`. Supply `pitch_supported_when_disabled=true`. The owner's
approximately **60-degree** travel statement and absence of microswitches describe
the mount; they are not measured endpoints. `expected_span.operator_reported_deg=60`
and explicit `minimum_deg`/`maximum_deg` establish an expected-span comparison for the
attempt. Do not reuse the historical production 140-degree span or final 40-degree
logical pose as facts for this mount.

All experiment settings come from this session's manifest. No missing value may be
filled from a synthetic fixture or the production configuration. The complete
numerical contract is checked by the C++ validator, including:

| Block | Explicit runtime fields |
|---|---|
| `homing` | Coarse/fine/backoff speeds; both backoff distances; repeatability tolerance and bounded retries; settling, approach/backoff deadlines and arrival tolerances; travel/rotation limits; opposite endpoint directions; rearm before start; torque protection; initial/step/maximum current limits. Motion checks must abort, and the existing current cap stays fixed with no current escalation. |
| `homing.contact` | Stall velocity and position progress; progress window; contact, hard-contact and hard-abort effort thresholds in Nm; prior-motion velocity; contact dwell and minimum command-active time; jitter window; moving velocity and peak acceleration. |
| `native_settings` | Expected original mode, current limit and speed/position gains; temporary homing current limit and gains; position-reference readback tolerance. |
| `guards` | Current, torque, pitch temperature, yaw displacement, total pitch displacement, encoder speed, mode-transition displacement, encoder/`MechPos` agreement, midpoint tolerance/speed/dwell/deadline. |
| `limits` | Clock uncertainty, dequeue age, CAN/IMU gaps, startup and session duration, minimum IMU status, read deadline/period and STOP period. |

Bind `native_settings_evidence={asset,sha256}` to the canonical hash of a measured
`adr0022.baseline_capabilities/1` asset with `capture_integrity=PASS`, the same pitch UID
and its raw `source.capture_sha256`. Registers `0x7005`, `0x7018`, `0x701e`, `0x701f`
and `0x7020` must each have positive observations in `PITCH_DISABLED_BASELINE` context,
with `min=max` exactly equal to the manifest's expected original mode, current limit,
position gain, speed proportional gain and speed integral gain respectively. Use a
fresh disabled baseline when these originals are missing or have changed. The older
baseline's mode 2 and the strict attempt's later disabled mode 3 must not be substituted
for current readback. During execution, the C++ owner reads and verifies every original
setting again before changing it.

The physical manifest also embeds its own synthetic, source/binary/revision-bound
`LOCAL_SENSORLESS_HOMING_PASS` qualification report and matching canonical report hash.
Record actual attendance and authorization with purpose `pitch_sensorless_homing` and
`sensorless_homing_authorized=true`. The owner's authorization permits operation
regardless of presence; an unattended session records that fact and explicit unattended
authorization without inventing attended cutoff or automatic protection.

Validate the numerical contract on the workstation with the native executable:

```bash
run/adr0022-local/firmware/axis_control_core/commissiond \
  --validate-homing run/new-homing-manifest.json
```

`--validate-homing` uses the executor's numerical contract before device, journal or
IMU-pipe access. Bundle validation checks the portable authorization, measured native
originals, committed source, binaries and local evidence. The launcher's complete
measured preflight runs on the Pi: it also checks the actual CAN/SPI topology, I2C
permissions and target executable's `--validate-homing` result before opening devices.
Do not run that physical preflight as a workstation hardware test. These checks do
not perform homing or prove the physical values are safe or accurate.

Package clean committed source and verified binaries with the acquisition bundle tool,
then deploy using `deploy_station.py --baseline-bundle FILE` and the existing pinned SSH
identity. Execute its printed command once:

```bash
OTA_RUN_DIR=<new-release>/run/stack bash <new-release>/Firmware/scripts/run_application.sh \
  run --establish-homing <manifest-path-printed-by-deployment>
```

The launcher owns one IMU process and one C++ CAN sender, mutually exclusive with all
other station control/acquisition. `FullAxisHoming` supplies native mode 2 approaches
and native mode 1 backoffs. Contact detection combines measured progress, motion
history, commanded direction, effort and dwell. Mode changes require fresh disabled
STOP feedback, zero speed/current references and correlated setting readback. Before
position-mode enable, a fresh `MechPos` read pins `LocRef` and `LimitSpd=0`; after enable,
the executor reads the enabled pose, re-pins and verifies it while the speed limit is
still zero. It then releases the bounded move. A position hold pins actual pose;
the FSM's zero-target hold sentinel is never an absolute-zero move.

After both endpoint observations, it moves to their measured midpoint, requires
settled dwell, confirms STOP and restores/readbacks the original runtime mode and
gains while disabled. Production settings and persistent calibration are unchanged:
this path sends no encoder-zero or save command and does not overwrite retained homing.
Preserve the immutable attempt, bound manifest, journal, independent review, exact child
identities and result before another operation. Use launcher `status`/`stop`.

## What it proves

A local pass proves the observed executor/protocol behavior against synthetic devices;
ARM64 VM evidence adds the actual binary and Linux boundary. A physical completion
records endpoint contact observations, measured span/repeatability, midpoint dwell,
mode transitions, STOP and original-setting readbacks for that attempt.

## What it does not prove

Completion sets `homing_observed=true` while all parameter, motion, current-mode and
encoder/`MechPos` qualification fields remain false. It does not establish current-mode
3 stopping, calibrated current/encoder uncertainty, independent cutoff, command-loss
stopping, plant identification, a controller candidate or 3a/3b. Yaw receives only zero
current requests; that does not certify yaw stopping. The measured observations must
be reviewed and bound into later calibrated acquisition without inventing ideal data.

## When it fails

Preserve faults, stale/lost streams, wrong UID or settings, invalid readback, unexpected
drive state, excessive current/torque/temperature/travel/speed, contact timeout, span
or repeatability rejection, transition motion, midpoint timeout and journal failures.
The sole C++ owner requests neutral references and STOP on recoverable abort; only fresh,
complete, fault-free Reset feedback confirms pitch disabled. After abort, do not restore
settings, enable or retry automatically. If the collector is lost or forcibly terminated,
the supervisor records STOP as unconfirmed and Python sends no CAN. Preserve physical
observations for calibration; do not widen protections to manufacture a completion or
call process exit a physical stopping certificate.
