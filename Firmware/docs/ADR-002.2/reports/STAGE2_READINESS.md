# Stage 2 readiness — acquisition blocked

Stage 2 started with one read-only SSH inventory on 2026-09-29 at 23:39:27 UTC
(2026-09-30 local station date). **Physical acquisition has not started. Readiness
at the required confidence is not established.** No motor commands, sensor resets,
mode changes, deployment, service action or production configuration changes occurred.

The [Stage 1 result](STAGE1_HANDOFF.md) remains a historical mathematical-software
PASS. Its 95% engineering assessment does not certify this acquisition path. The
full ADR, including 3a and 3b, remains NOT_DONE.

## Owner facts and current authorization

The owner authorized Stage 2 and reported no payload; pitch stays in place on power
loss; pitch travel is approximately 60 degrees total with homing placing it in the
middle; yaw is continuous through a slip ring. These are operator statements, not
measured endpoints, encoder zero, pitch current-mode qualification or stop evidence.

The owner specifies datasheet current limits with no additional imposed limit and
requests motor temperature capture. This supersedes treating the old 0.8 A yaw / 5 A
pitch host profile as a newly approved experiment envelope. Production configuration
has not been changed. Use the existing [GM6020 reference](../../references/gm6020/GM6020_AI_Reference.md)
and [CyberGear reference](../../references/cybergear/CyberGear_AI_Reference.md).
Protocol range, continuous duty and peak ratings must retain their distinct meanings.
No synthetic/default current-duration or thermal calibration becomes a measured fact.

## Inventory error and session closure

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

The session is recorded as **one failed read-only inventory, zero physical acquisition
attempts**. There has been no station reconnection to verify a correction. The owner's
next confidence requirement is **98.5%, not yet demonstrated**; subsequent failures
raise it to 99%, then 99.95%. Test counts do not establish those probabilities.

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
2. **Current-mode measurement is unqualified.** Yaw firmware/current-ring assertions
   are historical. Pitch is configured for native position mode; current mode and
   `Iqf` acquisition are not qualified. Standard pitch feedback is torque, not measured
   current. `limit_cur` is specified for speed/position modes and cannot be assumed to
   limit `iq_ref` in current mode.
3. **Physical timing and loss evidence are incomplete.** The new capture receiver
   separates kernel receipt from userspace dequeue, bounds clock mapping uncertainty,
   observes socket overflow and checks interface loss counters. These mechanisms are
   locally exercised. Device sample/filter timing and command-to-current relationships
   still require calibration; host receipt is not a device sampling timestamp.
4. **Stop and ownership are not qualified.** The historical stop failed; the process
   listing in this inventory failed. Pitch support is owner-confirmed, but the exact
   owner handoff and mode-transition procedure still needs local implementation and
   verification before physical qualification.
5. **Measurement resolution and temperature semantics must be bound.** GM6020 encoder
   quantization is `2*pi/8192` rad; the currently implemented pitch feedback mapping
   gives `25/65535` rad and needs current-device agreement with `mechPos`. Yaw RPM is
   too coarse for low-speed truth. Record yaw's temperature byte as raw, with Celsius
   unknown until its mapping is established; pitch feedback has documented 0.1 C
   units. The captured 48.8 C is **Pi CPU temperature**, not motor temperature.
6. **Executable compatibility and acquisition rehearsal remain open.** A host build
   and ARM64 link do not prove the target ABI, asynchronous device behavior, sustained
   recording, stop on process/communication loss or complete physical signal quality.

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
