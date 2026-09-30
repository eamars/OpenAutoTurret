# Stage 2 — baseline captured, strict neutral current attempt aborted

## Current measured-imperfection development

The hardware is the owner's camera/sensor mount. The owner's baseline is to retain
hardware observations and compensate measured imperfections through calibration and
tuning. The strict attempt above 0.1 A remains a closed historical failure under its
original rules. In the separate characterization operation, that unmeasured value is
only a diagnostic comparison; manufacturer current protection, temperature, travel,
fault and feedback-loss guards remain active.

The frozen native acquisition executable `fca1573f8a5f672125e2c57c6884b4596be84a8a10288cd7b42e8aad8d9d51ad`
passed 19 characterization process cases and a 10-second mode-3 zero-command probe.
The target executable `706883054714d7d1d0311a040c4bb39f109a0b92f6f65b4020f4b16cd370e79a`
passed the same 19-case ARM64 kernel VM matrix and 10-second probe. The ABI audit
against copied installed station libraries passed eight ELF files and 18 bindings.
The native firmware suite passed 83 tests, excluding the documented retained-homing
test. These are synthetic/local results; physical characterization remains NOT_RUN.

Shared sensorless homing passed 31 native process cases: five successful variants and
26 expected rejections. It preserves the measured encoder/native-position residual,
uses a fresh native MechPos position pin and correlated readback, tolerates pending
enable acknowledgements and reordered exact write echoes, and requires fresh Motor
feedback before motion. The reported 60-degree span remains a prior; endpoint and
repeatability observations must establish the physical geometry. The ARM64 raw
homing sequence completed with STOP/restore, and corrected independent review passed;
the VM's earlier reviewer failure remains retained. The full ARM64 matrix and
physical homing remain NOT_RUN. Before physical homing, the executor must separate
its 5 A software command ceiling from the documented 6.5 A measured-current
protection; raw feedback scatter must not be judged by the command ceiling.

The immutable neutral-observation extractor passed the final source-bound launcher
capture. It retains raw current/encoder/gyro scatter, zero empirical variance,
protocol quantization bins and host transaction latency. A zero-command current mean
is not declared sensor bias, and no correction is applied without calibration.
Mounting, physical encoder mapping, device response time/filtering, dynamic observer
uncertainty and whole-case dynamic identification remain pending. No PlantSnapshot,
controller candidate or 3a/3b certificate has been produced from these observations.

A fresh read-only inventory at 2026-09-30 06:15:21 UTC passed collection review. Its
historical launcher STOP FAILED record is retained as history; interface and process
facts do not qualify drive disable or dynamic stopping. The next bounded acquisitions
are fresh disabled native settings and a 10-second zero-command characterization,
with source and builds frozen during the physical sessions.

## Development takeover, 2026-09-30

The owner explicitly authorized station operation regardless of their presence.
Attendance at the manual cutoff is therefore not a prerequisite for this authorized
session. Manifest records retain the actual attendance fact and this authorization;
independent automatic cutoff and stopping after complete Pi/process/CAN loss remain
unqualified. This supersedes the pending attendance request below.

The inherited unfinished `--prepare-current` path has been preserved and exercised
locally. Its neutral transition now rejects receive loss immediately, requires fresh
disabled status before mode changes/enable, rejects unexpected enabled/disabled state,
bounds each receive drain, validates abort STOP frame length/time/faults, and reports a
journal completion failure truthfully. Independent review reconstructs the transition
from raw commands/readbacks/status, rather than trusting its completion footer.

The new [operation card](../../operations/adr0022-current-preparation.md) documents
single-owner launcher supervision, immutable/hash-bound manifests and acquisition
bundles. The local native/ARM64 neutral probe and 17-case process fault matrix passed.
The first physical strict neutral attempt then ended in **HARD_ABORT**, as recorded
below; it did not qualify current mode. Full calibrated acquisition, established
homing, dynamic stopping, identification and 3a/3b remain incomplete. Runtime captures
and the original dirty work snapshot are retained under
`run/adr0022-stage2/takeover-20260930/`.

The owner clarified that there are no endstop microswitches and the existing pitch
routine homes by detecting stalled motion, believed to use native speed mode. The
approximately 60-degree travel is an expected-span statement, not measured endpoints.
The production file still assumes 140 degrees and a final logical 40-degree pose;
those historical settings must not be reused as facts for this mount. Sensorless
endpoint detection, measured travel/repeatability and verified mode/stop transitions
must precede bounded dynamic acquisition.

The owner explicitly directed Step 2 to continue without treating the prior inventory
failure as a gate. A fresh read-only inventory at **2026-09-30 02:28:46 UTC passed**:
current process information, launcher status, interface facts and actual installed
library bytes were obtained. No controller or IMU consumer was observed in the account's
process list. **The first 120-second physical baseline completed successfully at
02:42:00 UTC**, using a separate acquisition release and launcher ownership. It
issued pitch discovery, reads and normal STOP requests, with one IMU startup; no
mode write, enable, excitation or production configuration change occurred.

The [Stage 1 result](STAGE1_HANDOFF.md) remains a historical mathematical-software
PASS. Its 95% engineering assessment does not certify this acquisition path. The
full ADR, including 3a and 3b, remains NOT_DONE.

## Closed strict neutral attempt (2026-09-30 04:25:25–04:25:31 UTC)

One authorized, unattended `--prepare-current` attempt ran from the separate release
`/home/eamars/workspace/OpenAutoTurret/run/releases/1b421d96b5be.qEbsMb`, committed source
`1b421d96b5becb0411a704b7ec77b5f952ed0ff9`. The bound manifest records attendance as
false and the owner's explicit authorization without a presence requirement. No
automatic retry occurred.

Pitch's original `RunMode` was 2. Correlated reads verified mode 3 and `IqRef=0` both
before and after enable. The first enabled `Iqf` readback was **0.2515328526496887 A**,
received **3.629904 ms after enable**. This exceeded the strict attempt's **0.1 A**
neutral criterion and triggered `HARD_ABORT: nonneutral measured pitch current`.
That criterion was an unmeasured zero-command expectation, not a calibrated physical
noise bound. One early sample cannot establish a steady-state current offset or its
cause; the failed capture remains evidence under its original schema and guards.

The C++ owner sent abort STOP **0.116684 ms after that readback**. A fresh, complete,
fault-free Reset feedback frame arrived **0.764139 ms after STOP**, confirming pitch
disabled in this attempt. No mode restore or subsequent enable was issued after the
abort: the last verified configured mode was 3, with pitch disabled. All **500**
transmissions succeeded; every yaw current command and pitch current reference was
zero. The capture had zero socket drops and zero interface-loss deltas, with writer
queue high water 7. Pitch position spanned one encoder count
(`25/65535 = 0.00038147554741741054` rad); yaw spanned two counts. Pitch temperature
was **23.9 C** throughout the captured feedback. These observations do not establish
dynamic stopping or calibrated measurement uncertainty.

The post-attempt launcher recorded the acquisition's exit, and the fresh account
process listing contained no controller or IMU consumer. Exact executable hashes
still matched the bound manifest. Pitch abort STOP is confirmed for this recoverable
collector failure; yaw stopping, independent cutoff, and stopping after complete
Pi/process/CAN loss remain unqualified. No current-mode qualification, PlantSnapshot,
controller candidate, 3a or 3b result was produced.

The raw journal SHA-256 is
`cfa711ec31b37093e1f8c0e6d92bdcfaa5c2a40038b79c471219691cff49ad2f`; the fetched
evidence archive SHA-256 is
`09ce24e5d004a984202edb146bf6531fdf3f8e1fef90eb207698ffda1fee0700`.
The machine-readable record indexes the manifest, attempt, result, raw journal,
closed analysis and post-state evidence. Runtime files remain outside Git.

## Separate zero-current measurement characterization

The owner's latest requirement is: **"You do NOT reject the hardware observation. You
shall adapt the imperfection from the hardware, and your calibration and tuning is
designed to compensate for that."** The 0.1 A assumption in the historical strict
attempt was not a measured physical acceptance line. Preserve the actual readings;
measure offset, noise, quantization and timing uncertainty, then use that calibration
in identification and controller compensation. Do not invent ideal hardware behavior.

The historical `adr0022.current-preparation/1` capture keeps its original abort result.
A distinct `--characterize-current` operation uses schema
`adr0022.neutral-characterization/1` and purpose
`neutral_current_measurement_characterization` to record zero-command current samples,
startup transients and noise without promoting a neutral qualification. It retains the
same zero-only commands, 0.01 rad displacement limit, 45 C temperature limit and timing
guards, followed by verified STOP and original-mode restore only on normal completion.

Characterization separately binds **6.5 A** manufacturer continuous-current protection
through `protection_current_bound_A` and `protection_limit_basis`. This is the rated
current from the retained CyberGear manual, not the 23 A peak protocol/rating value,
and not a measured noise criterion. The `neutral_current_bound_A=0.1` field is retained
only as a historical diagnostic comparison. Observations above it remain valid raw
data for calibration and do not invalidate the characterization or reject the hardware.
Its `neutral_current_qualified`, current-mode, dynamics and physical-parameter
qualification fields remain false regardless of the observed current.

The physical characterization manifest requires its own matching
`LOCAL_CURRENT_CHARACTERIZATION_PASS` source/binary-bound local report and the exact
manufacturer PDF SHA-256
`4fe8727a690193953e62438c04abd25f8e8be232e02b4eddf3aa1f99610da495`.
The launcher checks those bytes before device access. First local executable and
independent-review probes passed with synthetic devices; ARM64 and physical
characterization are **NOT_RUN** at this report revision. Current qualification and
compensation require the missing measured calibration and dynamic evidence. This new
measurement operation preserves the failed strict attempt. See the existing
[operation card](../../operations/adr0022-current-preparation.md).

The measurement duration is now a required runtime `neutral_observation_s`, independent
of the historical current comparison. It must be finite, positive and at most 60 seconds,
with a total deadline longer than startup plus the requested observation. A 10-second
physical observation is planned but has not been executed at this revision.

## Sensorless homing development

The [new operation card](../../operations/adr0022-sensorless-homing.md) uses the shared
`FullAxisHoming` routine for native mode 2 approaches and mode 1 backoffs. Runtime
manifests declare the entire homing/contact/native-setting/guard/timing contract.
Mode 1 enable requires fresh `MechPos`, a pinned reference and zero speed limit,
followed by fresh enabled pose re-pinning before bounded motion. Measured native
originals require their own disabled baseline capability asset and exact readbacks;
synthetic gains and the historical production span are not physical defaults.

The session records both endpoint contacts, repeatability, measured midpoint/dwell,
STOP and original-setting restoration on normal completion. It sends no encoder-zero
or save command and writes no retained calibration. The sole launcher lease excludes
other controllers/acquisition. Local realistic probes, independent raw review and
ARM64 binary evidence must close before physical use. **Physical sensorless homing
is NOT_RUN** at this revision. Homing observations leave parameter, motion,
current-mode and encoder/`MechPos` qualification false and do not qualify mode 3 stopping.

## Owner facts and current authorization

The owner authorized Stage 2 and reported no payload; pitch stays in place on power
loss; pitch travel is approximately 60 degrees total with homing placing it in the
middle; yaw is continuous through a slip ring. These are operator statements, not
measured endpoints, encoder zero, pitch current-mode qualification or stop evidence.

The owner confirms **manual power cutoff only**. No independent automatic cutoff or
command-loss stop is qualified; this is an operating fact, not a reason to stop the
non-exciting capability-discovery work. Software watchdogs cannot certify stopping
after complete Pi/process/CAN loss.

The owner specifies datasheet current limits with no additional imposed limit and
requests motor temperature capture. This supersedes treating the old 0.8 A yaw / 5 A
pitch host profile as a newly approved experiment envelope. Production configuration
has not been changed. Use the existing [GM6020 reference](../../references/gm6020/GM6020_AI_Reference.md)
and [CyberGear reference](../../references/cybergear/CyberGear_AI_Reference.md).
Protocol range, continuous duty and peak ratings must retain their distinct meanings.
No synthetic/default current-duration or thermal calibration becomes a measured fact.

## Historical inventory error

The process-list command incorrectly used `ps --ww`; procps rejected it. A directory
listing also returned 2 because it included an absent optional `.venv` path. The SSH
command returned 0 because the first collector did not propagate section failures.
The initial statement that inventory succeeded was incorrect and was corrected as
soon as the captured sections were inspected. The local rehearsal had also contained
the `ps` error; inspecting only its final completion marker failed to catch it.

The source-hash guard declined to execute the checkout launcher because its bytes
differed from the locally inspected source. This was the intended guard behavior.
Together with the failed process listing, it leaves current ownership unverified;
absence of `launcher.pid` must not be reported as proof that all processes stopped.

The first session remains **one failed read-only inventory, zero physical acquisition
attempts**. Corrections were tested locally. The later inventory collects current
Step 2 capability evidence under the owner's explicit continuation direction. The
previous 98.5% escalation remains historical; it is not used to block continuation.
No numeric probability of physical success is claimed from software test counts.

The preserved local raw capture is
`run/adr0022-stage2/station-inventory.txt`, SHA-256
`32ef87588a28ffe5b9f0ba5687db207503cf56dedbc1f68f30514070465d4ff9`.
The failed collector and connection record remain beside it. Raw captures stay outside
Git. The machine-readable [readiness record](STAGE2_READINESS.json) indexes the evidence.

## Facts obtained without opening motor or IMU transports

| Observation | Scope and implication |
|---|---|
| SSH host matched the existing pinned ED25519 identity; account `eamars`, UID 1000 | Host identity established for this inventory only |
| Debian 13.6, aarch64, kernel `6.18.39+rpt-rpi-2712` | Existing Ubuntu-based ARM64 compile evidence does not establish station ABI compatibility |
| Checkout `6a47f1dd696d75b878b8138dcaceb9f455ab9147`, no reported dirty files | Separate from local implementation HEAD and release identity |
| One release directory, revision `0b1b4b2b3ba7d5e45cc76f880da6049d06373337` | Directory presence is not proof of an active release |
| Captured release controller SHA-256 `3f2cabc4fe78d0de01a477ea3be3f61c49ce5090031485306982bf7019194bb4` | No Stage 2 `commissiond` binary found at the inspected release path |
| Both CAN links UP, 1 Mbps, ERROR-ACTIVE; `can0` on `spi0.0`, `can1` on `spi1.0` | Interface topology agrees with profile; motor capabilities were not queried |
| CAN error counters zero; `can0` accumulated RX drops 206,509 | A lifetime count, not current loss rate. Capture needs its own loss evidence |
| `get_throttled=0x50000` | Historical undervoltage/throttling flags, current low flags zero. Loaded power remains unqualified |
| `/dev/i2c-1` present; account belongs to `i2c` | Permission/device-node evidence only; BNO085 was not opened or reset |
| Root `run/station-venv/bin/python` symlink present; release Python executable hash obtained | Earlier reported deployment-path failure is not reproduced by this inventory; no speculative deployment fix was made |
| No launcher PID; previous shutdown record contains `STOP FAILED` and pitch disabled acknowledgement | Current ownership unknown. Last yaw settling confirmation failed despite later process-cleanup log |

The last stop log is from 23:01:25–35 UTC, before this session. It records yaw raw
speed -30 deg/s, encoder estimate -1.994 deg/s and only 5 ms of accepted dwell when
the stop deadline expired. This is not a successful stop certificate, and the saved
summary cannot establish whether motion, measurement timing or estimator behavior
caused it. No guard was loosened and no production stop fix was tested.

## Remaining acquisition blockers

1. **Active acquisition is incomplete.** `commissiond` now implements baseline CAN/IMU
   recording, discovery, normal pitch STOP polling and correlated register reads, with
   launcher ownership and single-attempt supervision, exercised locally. Current-mode
   preparation, bounded excitation, active watchdog/shutdown, physical calibration and
   automatic measured-asset injection remain open.
2. **Current-mode actuation is unqualified.** Yaw firmware/current-ring assets
   are historical and require an explicit reuse binding. Pitch's actual baseline
   `RunMode` was **2 (speed)** despite the profile's position-mode label. The strict
   neutral attempt verified mode 3 but aborted on its first enabled `Iqf` sample;
   current mode remains unqualified. Baseline `Iqf` readback was acquired at about
   24.24 Hz in the disabled context.
   Standard pitch feedback is torque, not measured
   current. `limit_cur` is specified for speed/position modes and cannot be assumed to
   limit `iq_ref` in current mode.
3. **Device time/filter calibration remains incomplete.** The capture receiver
   separates kernel receipt from userspace dequeue, bounds clock mapping uncertainty,
   observes socket overflow and checks interface loss counters. These mechanisms are
   now exercised on the station with zero capture loss. Device sample/filter timing and command-to-current relationships
   still require calibration; host receipt is not a device sampling timestamp.
4. **Physical stopping remains unqualified.** The historical stop failed; a later
   process inventory succeeded and found no active device consumers. Pitch support is owner-confirmed, but the exact
   owner handoff and mode-transition procedure still needs local implementation and
   verification before physical qualification.
5. **Measurement resolution and temperature semantics must be bound.** GM6020 encoder
   quantization is `2*pi/8192` rad; the currently implemented pitch feedback mapping
   gives `25/65535` rad and needs current-device agreement with `mechPos`. Yaw RPM is
   too coarse for low-speed truth. Record yaw's temperature byte as raw, with Celsius
   unknown until its mapping is established; pitch feedback has documented 0.1 C
   units. The captured 48.8 C is **Pi CPU temperature**, not motor temperature.
6. **Dynamic physical execution remains open.** The full firmware
   now builds against Debian ARM64 libraries, passes an offline dependency/version
   audit, and executes baseline acquisition under a local ARM64 Linux kernel. This
   is now also checked against copies of the station's actual installed libraries:
   eight ELF files and 18 dependency/version bindings pass. This does not qualify
   asynchronous devices, stop on process/communication loss or physical signal quality.

These are implementation/evidence gates, not grounds to invent physical parameters,
choose gains manually, change the Stage 1 model or raise the confidence claim.

## Local correction and verification

The reusable collector now uses `ps -ww`, records absent optional paths explicitly,
and exits unsuccessfully when any command fails. The local reviewer rejects missing,
duplicate, malformed and truncated sections, preserves prior reports, and always
leaves motion unauthorized. Eleven tests pass, including actual local Linux `ps`, the
real launcher's status branch in a temporary synthetic station tree, and injected
command failure. Platform observations in that rehearsal are fixtures, not station
evidence. See the [operation card](../../operations/adr0022-inventory.md).

Additional local resolution probes and automatic solver results are indexed in the
machine-readable record. They use synthetic plants and unmeasured sensor assumptions;
they cannot qualify this physical station or close the blockers above. No new hardware
adapter or production control change has been deployed.

The existing automatic solver was run unchanged on the saved synthetic BASELINE
snapshots, with each protocol encoder resolution and its assumed uniform quantization
variance injected. Both yaw and pitch returned `ENVELOPE_LIMITED`: none of the 256
analytic points passed all frozen checks. The rejected-point records are retained;
   no candidate was promoted, no PID was hand-selected and no threshold was relaxed.
This is an offline rejection under those stated assumptions, not proof that the
physical plant is uncontrollable. It is additional evidence against reusing the
Stage 1 synthetic sensor settings as a claim of physical readiness.

## Local baseline acquisition implementation

The full firmware build now supplies `commissiond --capture-baseline`, using the shared
GM6020/CyberGear codecs, kernel receive timestamps, bounded recording, independent IMU
streams, UID discovery, normal STOP feedback and asynchronous type-17 register reads.
The launcher supervises it through the existing motion lease. A content-checked
manifest binds executable hashes; attempt and output files cannot be reused. Commissioning
IMU startup permits one reset request and no stream recovery. This does not change
normal production startup. See the [capture operation](../../operations/adr0022-capture.md).

The first sustained local probe exposed insufficient recording throughput on the
Windows-mounted filesystem. Capture correctly failed when its bounded queue filled.
That evidence remains intact. Batching writes corrected the bottleneck without enlarging
the queue or dropping records. The subsequent 120-second local run recorded 93,184 yaw
feedback frames, 5,873 requested pitch STOP responses, 11,202 register reads and 5,927
samples from each IMU stream. Maximum queued records were 12 of 4,095 usable slots.
These counts and temperatures are **synthetic**, not station measurements.

The process suite covers source/UID mismatch, write echoes, timeout, re-enabling after
STOP, CAN faults/truncation/staleness, IMU loss/reset/EOF, evidence corruption, executable
identity, ownership supervision and refusal of a repeated attempt. Native tests exercise
real kernel socket overflow and an actual filesystem write failure. The local overflow
test was corrected to count loss observed anywhere in the drain, rather than incorrectly
expecting the last packet to carry a new loss increment.

Final local checks passed: 25 acquisition/launcher process tests, 93 mathematical tests,
and 83 CTests with `retained_homing` excluded. Seven native acquisition contracts include
the real socket-overflow and disk-write failures. The launcher lifecycle regression also
exposed its live log being moved into history; startup now preserves the prior log and
keeps the current log at its advertised path. All corrections were verified locally.

The encoder contract changed the mathematical method identity. The complete synthetic
matrix was recalculated against that identity: all 18 conditions passed, covering
417,960 native evaluations. Compatible fitted assets were verified before reuse; old
evidence was preserved. The fresh Stage 1 audit passed with 93 mathematical tests,
84 acceptance-contract tests and 83 native regressions. This revalidation does not
qualify physical acquisition or resolve the separate protocol-resolution rejection.

Pitch's finite encoder range now has an explicit `bounded_count` calibration mapping.
It preserves both endpoints and never treats a finite-range jump as modulo wraparound.
The normalization pipeline accepts it without changing the plant model, identifier,
controller synthesis or quality thresholds. The parameter catalog includes the encoding
and finite endpoints; all unknown physical values remain PENDING. Measured calibration
must explicitly name its encoding and still verify the endpoints against current-device
`mechPos` observations.

A separate local resolution diagnosis reproduced a synthetic yaw restart timeout after
a negative 5-degree step, while the frozen position/drift metrics passed. At the stalled
point, encoder quantization shifted the interpolated start-current boundary. This
narrows one rejection mechanism; it does not explain every full-solver rejection or
justify changing gains, thresholds or the model. No rejected candidate was promoted.

## Debian ARM64 execution and capture version 2

The workstation now has an isolated Debian cross-build environment, prepared from
authenticated package downloads without installing packages into the host. The full
firmware cross-build passed with GCC 14.2.0. An offline audit resolved eight ELF files
and 18 dependency/version bindings for `commissiond` and the IMU executable. The
package identities, binaries and evidence hashes are indexed in the JSON report.
The existing local release archive's controller hash differs from the inventoried
station controller, so that archive was not treated as current station evidence.

User-mode QEMU rejected both required socket metadata options. They were not disabled.
A local ARM64 VM with its own Debian `6.12.107+deb13-arm64` kernel runs the same capture
binary with the checks intact. It has no network or host-device passthrough and creates
a project venv inside the guest. Its first probe ran successfully but its initial
archive exporter lacked a gzip applet; only that attempt's console survived. The
exporter was corrected locally, then replaced with a virtual-serial export path that
preserves complete raw captures without slow console transfer.

The first sustained VM run passed at about 663 Hz, below the producer's nominal 1 kHz.
Adding a full-rate criterion exposed another 820 Hz producer limit. Bounded catch-up
batches corrected the generator; actual kernel timestamps still expose delivery
timing. The minimum 99% offered-load criterion is a test workload requirement, not a
confidence estimate or a change to the frozen controller quality thresholds.

A separate local socket probe showed that queued-packet loss can be read before a
later packet delivers its ancillary overflow notification. Capture version 2 therefore
requires zero final socket drop counters from `SO_MEMINFO`, in addition to `SO_RXQ_OVFL`
and interface counters. Eight native contracts include this real overflow case and
an actual disk-write failure. The reviewer rejects absent/nonzero final counters and
older capture schemas. Earlier files remain historical evidence under their original
software identities. Bounds smaller than one nanosecond are rejected before sensor
startup, and early process failures retain stdout, stderr and executable identity.

Final version-2 evidence:

| Local environment | Duration | Observed yaw rate | Records | Final socket drops | Queue high water |
|---|---:|---:|---:|---|---:|
| Native workstation | 120 s | 1000.001 Hz | 195,953 | yaw 0, pitch 0 | 27 / 4095 |
| ARM64 Linux VM | 120 s | 1000.024 Hz | 194,876 | yaw 0, pitch 0 | 17 / 4095 |

The VM also passed all 12 injected protocol/stream failures and eight native capture
contracts. The current host process/ABI suite passed 29 tests. These results concern
synthetic loopback traffic and recording. The VM kernel is not the Pi kernel; actual
SocketCAN drivers, motor modes, IMU timing, stop behavior and installed-library
identities remain unqualified. Active excitation/watchdog/handoff and measured asset
injection remain incomplete. **No additional station access occurred, and the required
98.5% physical-readiness assessment was not established at that checkpoint.**

## Continued Step 2 capability discovery

The latest inventory passed without opening CAN/I2C or changing station files. The
checkout remains `6a47f1dd696d75b878b8138dcaceb9f455ab9147`; the exact inspected
launcher status branch reports the historical failed stop, and no controller/IMU
consumer appears in the fresh account process listing. Six installed runtime libraries
were copied read-only and passed the offline ABI audit with the two target executables.

The saved production IMU trace was read separately. Its 2,750 samples contain no
per-sensor sequence gap, with observed status gyro 0, rotation vector 0, accelerometer 2
and game rotation vector 3. These are historical observations, not a new calibration.
Baseline acquisition preserves low-status data explicitly without declaring it calibrated.

A recorded factory CyberGear negative reply contains stale bytes where a normal reply
would carry a float. Discovery now records a correlated `register_rejected` with
`value=null`, skips that register for the remaining baseline context, and continues
unrelated streams. The independent reviewer reports `MEASUREMENT_LIMITED` for dependent
steps. Wrong source/index/status, echo, malformed data and timeout remain invalid.
This behavior passed ten native contracts, 30 host process/ABI tests and the ARM64 VM
matrix: 16 checks, including 12 fatal fault cases and the nonfatal capability case.
The sustained VM capture retained 194,975 records at 1000.034 Hz yaw input with zero
final socket drops and queue high water 16/4095.

An acquisition-only bundle packages the two verified executables and their physical
manifest under a committed source identity. The existing deployment tool installs it
in a separate release and runs launcher `check`; no package installation, target
compilation, target regression testing or production activation is involved. Local
pack/install and the real launcher check passed before this deployment path was used.
Runtime captures and installed-library copies remain outside Git; their identities
are indexed in the JSON report.

## First measured baseline (2026-09-30 02:39:55–02:42:00 UTC)

The separate `c80f84a38e55.J5AR2G` acquisition release passed launcher preflight and
completed one 120-second capture. Source revision was `c80f84a38e55101412ff7cb0cb29bbc29f586b1b`.
The raw capture SHA-256 is `2936324e32f2306f9c548e925b7ebf34cff1d33fec437c5b199d8fd3e52f4c55`.
Both socket loss counters and interface loss increments were zero. Writer queue
high water was 11/4095. The single capture ended normally, and subsequent read-only
process inspection found no controller or IMU consumer. Production was not started.

| Actual observation | Result |
|---|---|
| Yaw feedback | 120,001 frames; 1000.006 Hz; largest gap 1.200 ms |
| Pitch disabled STOP feedback | 5,806 responses; 48.478 Hz |
| Register reads | 5,805 successful; no negative replies or timeouts |
| Pitch `Iqf` | 2,900 samples; 24.239 Hz; zero in this disabled context |
| IMU gyro / rotation vectors | 5,989 samples each; 50.088 Hz; no sequence discontinuity |
| Accelerometer | 7,823 samples; 65.410 Hz |
| IMU accuracy status | gyro 0, rotation vector 0, accel 2, game rotation vector 3 throughout |
| Pitch mode / limit / bus voltage | mode 2 (speed), `limit_cur` 5 A, 24.041 V; limit does not apply as a proven mode-3 cap |
| Motor temperatures | pitch 22.6 C; yaw raw byte 28, Celsius conversion unknown |
| Encoder observations | yaw counts 5772–5774; pitch counts 30634–30636 |
| Retained homing | `/dev/shm/ota-homing-1000` absent; established homing required before bounded motion |

The current acquisition reused the shared codec's legacy yaw ampere conversion in a
derived `current_A` field. That feedback scale is not bound by this capture's calibration.
The original raw bytes remain intact; the reviewer now checks them independently and
explicitly ignores all 120,001 unqualified ampere fields. The local emitter now writes
null instead. This correction passed the real local process rehearsal and local tests;
it has not been deployed or verified by repeating the physical capture. The baseline
remains valid raw acquisition evidence, not a calibrated-current asset.

The automatic capability extractor binds the capture, original/bound manifests,
attempt and executable hashes, then writes an immutable capability asset. It records
single-pose encoder comparison and sensor scatter without promoting them to full
scale/mounting/noise uncertainty qualification. Inertia, resistance, directional load,
breakaway, delay and mounting remain null. No PlantSnapshot or controller candidate
was generated. The Stage 1 mounting estimator correctly rejects the no-excitation
identifiability probe with `INSUFFICIENT_EXCITATION`.

The next Step 2 operations are separate zero-current measurement characterization,
current-mode verification, established homing and
bounded stopping/motion qualification, then mounting/time calibration and prescribed
dynamic identification. Operator attendance at the manual cutoff has been requested
as a physical operating fact at the baseline checkpoint; the owner's later explicit
authorization permits operation regardless of presence. Record actual attendance
truthfully. The prior inventory failure is not a continuation gate.
