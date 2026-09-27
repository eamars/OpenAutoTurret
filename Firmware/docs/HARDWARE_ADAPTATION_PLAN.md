# Plan: split CAN buses, mixed motors and continuous yaw

Status: **mixed controller/AUTO_ROAM integration observed; two controlled moving
stops succeeded on `cae41d0`; broader stop and homing-guard qualification remain
open**. Updated 27 September 2026.
Ground truth is [the hardware inventory](HARDWARE_CURRENT.md); the operator
confirmed GM6020 on yaw and CyberGear on pitch. Follow with the
[AI perception plan](AI_HAT_PERCEPTION_PLAN.md).

The owner has now confirmed that pitch retains mechanical endstops and authorized
mechanical tests. [Implementation and evidence](HARDWARE_COMMISSIONING_2026_09_26.md)
records typed SocketCAN, GM6020 decoding/voltage framing, session-relative encoder
unwrapping, the launcher-owned probe and real bidirectional yaw motion. This
completes an initial Stage 1 slice and starts Stage 2; it does not complete the
stage exit gates or qualify automatic tracking. The default mixed probe sends no
pitch motion commands; the explicit pitch session now exercises position mode.
The initial pitch `mechPos` failure was resolved after the owner's
September 27 upgrade: the same UID now returns valid position with status 0.
Pitch-only homing has since completed in the normal mixed runtime. However,
repeated encoder-speed-ceiling and commanded-corridor warnings were not aborting
because `homing.motion_checks_abort` is false; homing-guard behavior remains
unqualified.

The owner has now confirmed the camera sits roughly at the pitch assembly's
center of mass, both axes are direct drive, and disabling pitch presents no
current support risk. The [versioned IMU acquisition and stationary tare](IMU_COMMISSIONING_2026_09_27.md)
have passed live runs and independently observed yaw and pitch movement. A
continuous pitch session completed +15°/return and −15°/return at requested
10°/s with 5 A cap readback and no guard trip. A separate yaw session completed
29.356° outbound and returned to +0.659° with no guard trip. Paired BNO085
game-RV movement was 0.994 and 0.990 of yaw encoder travel on the two legs;
pitch's four-leg mean ratio was 0.985. See the
[large-motion record](LARGE_MOTION_COMMISSIONING_2026_09_27.md). Earlier
small-motion ratios did not persist, so no IMU scale correction is retained.
The raw IMU is not a base pose and its mounting alignment remains provisional.

For motion tuning, use the full authorized current/torque headroom from the first
meaningful bounded test rather than walking up from negligible outputs. Pitch
remains limited to 5 A. Keep the motor energized between movement stages with
continuous feedback and IMU capture; avoid repeated stack/CAN shutdowns between
tests. Fault stops and an explicit end-of-session stop remain required.

The implemented commissioning slice adds a hard **5 A pitch command ceiling** and
requires current-limit and supported-mode readback before enable. A non-motion
probe has written 5 A and obtained three matching readbacks. This setting is
volatile and applies to speed/position modes; raw MIT/current mode is excluded.
It does not measure current transients. The owner has upgraded pitch to
1.2.1.5; the agent verified position compatibility and reapplied/read back 5 A.
See the [upgrade reference](CYBERGEAR_FIRMWARE_UPGRADE.md).

Hailo-8 provisioning and a real IMX477-to-YOLOv8n inference probe have passed.
The Hailo profile also passed a 60-frame application run through visiond with
motion held. Separately, the normal mixed profile completed a five-minute
AUTO_ROAM run with IMX500 (8,061 frames, zero drops), repeated AUTO_TRACK/loss
handoffs and fresh observe-only BNO085 samples. This demonstrates runtime
integration, not person/head accuracy or stop qualification. Two later controlled
stops on `cae41d0` succeeded after AUTO_TRACK near +46° yaw and AUTO_ROAM near
+76° yaw, each with fresh pitch-disabled feedback and a final GM6020 zero
request. The GM6020 disable state remains unknown, and intermittent feedback-
readiness rejection on the previous release is not proven eliminated. The
latest normal homing also completed with repeated encoder-speed-ceiling and
commanded-corridor warnings while `homing.motion_checks_abort` was false; this
is an open homing-guard qualification gap.

## Intended result

Use native SocketCAN with GM6020 ID 1 on `can0` and CyberGear ID `0x7F` on
`can1`. Support continuous yaw with no endpoint search, retain a commissioned
bounded pitch axis, and preserve the service sequence
AUTO_ROAM -> selected-person tracking -> AUTO_ROAM after loss. Manual/Hold
remains an explicit web override. Camera tracking is the application here.

The mixed motor/controller and geometry changes are now in the normal profile.
Do not roll back to the old configuration on this mechanism. Hailo provisioning
and camera-only evaluation remain separate from the IMX500-backed normal run.

## Why this needs code changes

| Current boundary | Observed assumption | Required change |
|---|---|---|
| `config/turret.yaml`, `control/src/config/turret_config.*` | One transport/device; IDs 100/101; finite travel on both axes | Explicit buses, per-axis protocol/ID/topology, validated migration schema |
| `control/src/main.cpp`, `can/cybergear_system.*` | One CyberGearSystem supplies both axes | Compose independent axis drivers over independently owned buses |
| `can/can_transport.hpp`, `can/socketcan_bus.*` | Typed frames and independent error subscription | Per-protocol filtering and refreshed health are implemented for the mixed composition |
| `control/motor_backend.hpp`, `can_motor_backend.*` | CyberGear registers, mode transitions, torque/fault/disabled feedback | Mixed backend exposes protocol capabilities and field availability; yaw disable state remains unknown |
| `calibration/*`, `control/boot_fsm.*` | Endpoint homing/finite soft limits define both axes' readiness | Continuous yaw session reference and pitch-only homing are implemented; physical stop acceptance remains open |
| `mode/roam_planner.hpp`, control/geometry/safety paths | Yaw sweep ends, soft-center park, bounded target representation | Continuous-angle planning and wrap-safe search are implemented; broader stop/park qualification remains open |
| Web telemetry, recovery and launcher/preflight | Single CAN panel; both motors acknowledge disable/fault-clear | Per-bus health, capability-aware telemetry and mixed split-bus launcher selection are implemented; two stop observations passed, prior intermittent readiness rejection remains unexplained |
| `perception/camera.py`, model adapter | IMX500-specific single camera creation | Later explicit sensor/provider selection; see AI plan |
| Host `imu-lab` versus production | Standalone BNO085 data probe outside this repository | Versioned SH-2 acquisition now runs as a launcher-supervised observe-only stream; sensor-to-camera calibration and any fusion remain open |

Paths in this table are relative to `Firmware/`. Treat the current simulated
backend as another capability implementation, rather than making it return
invented GM6020 acknowledgements just to preserve old tests.

## Stage 0: define the new installation contract

Record physical wiring, motor firmware, ratios and signs; verify pitch endstops,
load support, supply and termination, and what passes through the slip ring.
Its rating and routing, including camera connections, must support the proposed
rotation. A slip ring alone does not prove collision-free travel at every pitch.

The new `config/mixed_hardware.yaml` defines the production topology and
`config/turret_mixed.yaml` selects it; commit `56a28fe` made that mixed profile
the normal launcher default. The old `hardware_probe.yaml` remains a separate
commissioning schema:

- Bus `yaw_bus`: SocketCAN `can0`, expected parent `spi0.0`, 1 Mbps classical CAN.
- Bus `pitch_bus`: SocketCAN `can1`, expected parent `spi1.0`, 1 Mbps classical CAN.
- Yaw: protocol GM6020, motor ID 1, feedback `0x205`, voltage group `0x1FF`,
  topology `continuous`; verify command mode before selecting its driver.
- Pitch: protocol CyberGear, motor ID `0x7F`, expected UID bytes
  `7216313130333105`, topology `bounded`; no ID renumbering is necessary.
- Pitch current: finite positive limits no greater than **5 A**, including
  homing, payload checks, parking, recovery, adoption and diagnostic paths.
  Write/read back the volatile limit before every supported-mode enable after
  reset; reject unsupported current/torque modes and failed readback. Do not
  raise this ceiling to overcome load or an endpoint.
- Camera and mechanism calibration carry an installation revision, device
  identities, units, direction/ratio, origin policy and calibration validity.

Reject legacy single-bus configuration for a mixed-drive profile with a clear
preflight message. Bus bring-up is a separate host prerequisite managed by a
narrow system configuration; the station still runs as `eamars`, without sudo.
Select an OS network provisioning method that validates both interfaces and
does not require granting unrestricted network-admin capability to Python.

**Exit:** topology/schema reviewed, missing physical facts listed, no ambiguous
mapping of a GM6020 voltage unit to CyberGear amperes or torque.

## Stage 1: executable transport and protocol probe

Use the September 26 identification as the initial baseline. Before building
the whole driver, make a minimal probe exercise the actual proposed transport
on Linux virtual CAN/replayed frames, then the existing HAT:

1. Decode SFF `0x205` and EFF `0x00007FFE` with flags retained. Reject wrong
   DLC, RTR, error frames and identical numeric IDs of the wrong frame type.
2. Prove standard TX on an isolated virtual/test bus and extended TX on the
   other bus. Receive timestamping must use the controller's monotonic domain.
3. On hardware first listen to GM6020; query only CyberGear discovery. Measure
   feedback ages, frame counts and drop/error deltas with both readers active.
4. Keep RX and TX work bounded; no inference, allocation bursts, blocking
   register round trips or thread joins in the 200 Hz controller path.

Subscribe to CAN error messages through `CAN_RAW_ERR_FILTER`; a normal ID filter
alone is insufficient. Poll/query live bus state instead of reporting only the
state cached at socket open. One disconnected bus must be visible independently
and trigger the agreed coordinated-stop policy, without starving the other RX.

**Exit:** packet-level evidence of correct SFF/EFF routing and no unexplained
loss at GM6020's approximately 1 kHz feedback rate. Only then add focused
regression coverage for parsing, routing, stale feedback and bus failure.

## Stage 2: GM6020 control and generic drive capabilities

Add a GM6020 codec and driver. ID 1 occupies bytes 0-1 of standard `0x1FF`;
assemble all eight bytes deliberately, with unused slots zero. Serialize writes
if a future group has multiple axes. Decode angle modulo 8192, signed speed in
rpm and raw current separately. See the [vendor reference](GM6020_AI_Reference.md)
for v1.4's voltage range and firmware-gated current mode; current-command tables
have an unresolved slot ambiguity. Start from the installed, verified mode.

Voltage control requires host velocity/position regulation. Prototype a bounded
velocity loop with saturation and anti-windup, then add the existing trajectory
reference as an outer position target. Measure achievable loop rate/jitter on
the Pi; keep 200 Hz planning initially and choose any faster inner loop only
from measured stability/response needs. Schedule a deliberate periodic command
cadence rather than copying CyberGear's write-only-on-change cache.

Characterize no-load direction and low-output response, then supervised loaded
response, stopping distance and thermal behavior. Do not reuse old gains or
convert uncalibrated feedback current into N.m. The vendor electrical limits
are ceilings, not commissioned payload settings. In voltage mode the host
cannot claim the same independently enforced ampere limit as CyberGear LimitCur.

The generic drive contract should expose position/speed with freshness and
validity, raw effort/current when available, control strategy, and capability
flags for real device identification, fault reporting, disable confirmation,
register access and recovery. An unsupported value is **unknown**, not false,
zero or healthy. Keep the CyberGear implementation's protocol-specific recovery
and mode readback behind its own driver.

The GM6020 guide does not establish a command timeout or a disable/fault-clear
acknowledgement. Sending zero voltage is an output request, not proof of power
removal. The existing host watchdog cannot guarantee braking if its own process
or Pi fails. Measure loss-of-command behavior in a secured bench setup and
establish a verified independent cutoff/stop mechanism where required before
loaded automatic operation. A live transmitter must not indefinitely refresh
a stale nonzero command after the controller heartbeat expires.

**Exit:** explicit stop/feedback contract for each drive; bounded low-output
control demonstrated; stale-command, link-loss, process-loss and saturation
behavior established before higher-speed tuning.

## Stage 3: continuous yaw and calibration

Represent yaw internally as an unwrapped angle; use wrapped orientation only
where geometry/UI requires it. Integrate signed modulo-8192 encoder deltas with
direction/ratio applied exactly once. Require elapsed-time and speed bounds to
make wrap reconstruction unambiguous; a lost interval that permits more than
half a revolution must invalidate continuity rather than guess the turn count.
Exercise forward/reverse wraps, many turns, reordering, duplicate frames and
reconnects in the executable probe.

Choose a reference contract: initially use the observed startup orientation as
a session-local yaw origin. If an absolute repeatable heading is required,
commission a surveyed encoder offset/index or separate heading reference; the
single-turn encoder cannot recover a lost multi-turn count by itself. A restart
must revalidate reference state rather than trusting saved turn count. Store
the distinction between modulo orientation and accumulated travel explicitly.

Remove yaw endpoint search from boot and Home for this topology. Establish yaw
readiness from valid reference plus healthy fresh feedback; retain real pitch
homing after its mechanics are verified. Replace the global assumption that
`soft_limits_valid` on both axes proves readiness with per-axis readiness and
topology-specific checks. Do not fabricate min/max values or mark yaw homed to
satisfy the old FSM.

Revalidate motor-history interpolation, camera transforms, LOS filtering,
target continuity and controller error across +/-pi. A heading target maps to
the nearest reachable unwrapped equivalent consistent with the trajectory and
any exclusion sectors; wrap crossing must not cause a full-turn correction.
Velocity/acceleration/jerk and collision constraints still apply to continuous
yaw. Pitch retains finite bounds and braking margins.

Old retained calibration is rejected by installation fingerprint. Preserve its
file as historical evidence; do not overwrite it to make startup pass.

**Exit:** Continuous yaw reference, wrap-safe sector motion and pitch-only
homing have operated in the normal run. Two moving-stop observations passed on
`cae41d0`, but broader stop acceptance remains open. The normal homing emitted
repeated encoder-speed/corridor warnings without aborting because
`homing.motion_checks_abort` is false; this guard policy must be validated or
resolved before calling homing qualified.

## Stage 3B: integrate the installed BNO085 as a secondary observer

Hardware installation is complete enough for SH-2 data delivery. Versioned
`imu-bno085` runs continuously under launcher supervision and the controller
consumes its timestamped trace as an observe-only source. The five-minute run
reported fresh game-RV status 3 without gaps. This is not calibrated fusion or
motion-control authority. Use the [old IMU addendum](archive/open_auto_turret_bno085_imu_expansion_v1_1.md)
as design input after replacing its two-CyberGear/finite-yaw assumptions, and
the [BNO08X](BNO08X_AI_Reference.md), [SH-2](SH2_AI_Reference.md) and
[SHTP](SH2_SHTP_AI_Reference.md) references as protocol inputs.

The bounded, versioned executable acquisition path exists. Continue checking
product IDs/firmware, report ID, sequence,
sensor status, quaternion order, SI units, sensor timestamp and monotonic receipt
time. The existing lab probe is evidence, not production code: fix its rad/s
label, elapsed-time accounting, short-read/continuation handling and error
visibility before porting. Its uint32 microsecond HAL clock wraps after about
71.6 minutes; verify SH-2/host rollover handling in a long run rather than
silently treating a rollover as time reversal.

The IMU is a packet-oriented SHTP device, not a register bank. Respect I2C
clock-stretching, packet length, continuation headers and channel sequences.
Use one I2C owner, outside the control thread, bounded queues and explicit
staleness. Polling currently works for a brief test without INT; measure its
timing uncertainty or add a validated INT timestamp path before compensation.
Reconstruct report times using SH-2 base/delay fields in the controller's
monotonic domain. Receipt time alone is not the physical sample time.

Confirm the recorded off-axis pitch-stage mount and calibrate sensor-to-payload
and camera transforms. A base-mounted IMU cannot observe pitch-stage motion;
placement changes the available measurements. Record lever arm `r`; during
motion, accelerometer readings include `alpha x r + omega x (omega x r)` as
well as gravity/specific force. Use stationary gates or a validated compensation
model for tilt estimates. Do not classify all acceleration as gravity.

Begin in observe-only mode: compare gyro/orientation increments against encoder
kinematics while measuring sample lag, jitter and stationary drift. Rotation
vector accuracy was zero in the audit; reject unreliable absolute orientation.
Evaluate magnetic interference from both motors, wiring and the slip ring.
Game rotation vector can supply short-term relative motion without magnetic
heading, but yaw drifts; it does not supply an absolute heading or recover yaw
turn count. Do not automatically tare when sensors disagree or persist new
calibration from an unexplained fault. Any explicit reference reset invalidates
affected estimator state and requires a controlled reinitialization.

Only after the observer passes calibrated known-angle/static and slow-motion
checks should it contribute camera-motion compensation or installation tilt.
Test I2C stalls, packet loss, reset, bad status, clock wrap and magnetic upset.
An unhealthy optional IMU must be visibly excluded while encoder/camera operation
retains its own limits. Never use IMU data to bypass pitch limits, certify motor
disabled state or substitute for independent park/support evidence.

**Exit:** timestamped observe-only reporting is running with known units and
status; calibrated mount transform and measured disagreement/age thresholds
remain open before any feedback or image-motion compensation is enabled.

## Stage 4: roaming, parking, recovery and operator controls

The normal mixed runtime has exercised repeated sector sweeps and AUTO_ROAM ↔
AUTO_TRACK/loss handoffs. The web `/api/state` failure caused by NaN GM6020 yaw
effort was fixed by serializing unavailable effort as `null`; the dashboard
renders it as unavailable. The five-minute run on the previous release ended
with `STOP FAILED`. On `cae41d0`, two later controlled stops succeeded, one after
AUTO_TRACK near +46° yaw and one after AUTO_ROAM near +76° yaw, with fresh pitch-
disabled feedback and a final GM6020 zero request. Yaw disable state is unknown,
and intermittent feedback-readiness rejection on the previous release is not
proven eliminated. The earlier 176-degree yaw/lowest-pitch approval remains
historical and is not an acceptance basis.

Specify a continuous-yaw search policy: bounded speed/acceleration, chosen scan
direction, pitch coverage and optional sectors. Initially preserve the direction
across tracking/loss handoffs; Manual clears/resets that search state as before.
Test selection during a wrap, loss during rotation and reacquisition on either
side of the wrap. Pitch travel always remains constrained.

Replace yaw `soft_center` parking with an explicit policy: stop at current yaw
or move to a commissioned modulo heading using a permitted path. Select and
verify a supported pitch park pose on the new mechanism. The previous
176-degree yaw/lowest-pitch release approval does not describe this installation.
Differentiate stopped, output-zero requested, release unverified and verified
park in telemetry; do not wait forever for a GM6020 disabled bit that does not
exist or report one after a zero frame.

Keep Stop/Hold, watchdog and operator recovery authoritative. Recovery must use
each driver's supported actions, invalidate reference where necessary and avoid
automatic resumption. The web should show both bus states, physical mapping,
feedback ages, yaw reference validity/turn count, pitch calibration and the
reason motion is inhibited.

**Exit:** simulator and replay exercise the real launcher/controller/web paths:
cold start, invalid reference, Manual, Auto, selection/loss, stop, parking,
recovery, bus failure and process restart. No trial settings become defaults.

## Stage 5: controlled commissioning and release

Committed separate releases, mixed-profile preflight and normal launcher
selection are in place. The five-minute run demonstrated the normal control,
perception and web processes running together. Two controlled stops passed on
`cae41d0`; broader stop qualification and the homing-guard gap remain open.

Bounded pitch commissioning and a 30° yaw session have been completed; the
five-minute AUTO_ROAM run exercised sector sweeps and tracking/loss handoffs.
Remaining evidence includes reviewed stop behavior, additional controlled
framing/tracking cases, command/feedback timing, stopping distance, temperatures
and bus error/drop deltas under sustained load. Run camera/Hailo CPU-load trials
as well as motor-only trials before any performance claims.

Acceptance requires fresh per-axis feedback, no silent saturation or stale
command continuation, no turn discontinuity, verified stop/recovery/park behavior,
and measured payload limits. Two stop observations passed, but intermittent
feedback-readiness rejection and homing guard warnings remain to be resolved or
qualified.
Retain unresolved measurements as blockers rather
than reusing September 3-9 acceptance. Use the launcher for activation/status/
stop. Do not roll back to a dual-CyberGear release on this mechanism; rollback
means stopped state or a release verified for this hardware.

## Suggested implementation batches

1. Configuration/preflight, typed two-bus transport and diagnostic probes: implemented.
2. Capability-based mixed drivers and bounded pitch/yaw commissioning: implemented;
   two moving stops succeeded; broader stop/homing-guard acceptance remains open.
3. Continuous-yaw reference, topology-aware boot/safety/geometry and observe-only
   BNO085 acquisition: integrated; IMU calibration/fusion is not enabled.
4. Roam/tracking/loss and web integration: exercised in the five-minute run;
   park/recovery, intermittent readiness rejection and homing-guard acceptance
   still require evidence.
5. Physical commissioning and final release qualification: in progress.

Each batch records its executable evidence before adding broad tests or
optimization. No schedule or tracking-speed improvement is claimed until the
hardware measurements exist.
