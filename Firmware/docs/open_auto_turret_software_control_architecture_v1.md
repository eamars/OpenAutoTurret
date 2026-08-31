# OpenAutoTurret Vision Tracking Station
## Software Architecture and Closed-Loop Control Design

**Document status:** Architecture baseline / implementation handover  
**Version:** 1.0  
**Date:** 2026-08-31  
**Target platform:** Raspberry Pi 5 + Raspberry Pi AI Camera (Sony IMX500) + dual Xiaomi CyberGear direct-drive axes over SocketCAN  
**Scope:** Non-weaponized pan/tilt camera and sensor tracking platform

---

## 1. Executive summary

This document defines the software architecture, calibration strategy, safety model, and closed-loop control design for a two-axis direct-drive pan/tilt tracking station.

The system consists of:

- Raspberry Pi 5 running Debian GNU/Linux 13 (`aarch64`)
- Raspberry Pi AI Camera using Sony IMX500 inference
- MCP2515 CAN controller exposed as Linux `can0`
- Two Xiaomi CyberGear motors:
  - Pitch: CAN ID `100` / `0x64`
  - Yaw: CAN ID `101` / `0x65`
- 1 Mbit/s extended-frame CyberGear CAN protocol
- No dedicated IMU in the initial build
- No dedicated limit/home sensors in the initial build
- Direct-drive yaw and pitch axes with approximately balanced payloads
- Mechanical travel below 180 degrees on both axes, with yaw potentially more restricted by the payload

The proposed architecture deliberately separates perception, deterministic motor control, telemetry, and the web interface. The **control daemon is the sole owner of `can0` and the motors**. Vision and web processes can publish measurements or requests, but they cannot directly issue motor commands.

The design uses two principal timing domains:

1. **Vision measurement loop:** approximately camera inference rate (nominally around 30 Hz, measured rather than assumed).
2. **Motor/control loop:** initially 200 Hz, subject to timing and CAN profiling.

The controller does not directly map bounding-box pixel error into motor velocity. Instead, every visual observation is timestamped and converted into a camera line-of-sight ray. The ray is transformed into the calibrated turret-base frame using the **motor positions at the image capture time**, not merely the positions at the time inference completes. A target-state estimator then predicts the target line-of-sight forward to the actuation time.

Motion commands pass through a trajectory and safety layer that enforces:

- absolute calibrated joint limits;
- soft limits inside physical end stops;
- velocity limits;
- acceleration limits;
- jerk limits;
- stopping-distance / braking-envelope constraints;
- optional coupled yaw-versus-pitch collision envelopes.

Boot calibration uses an improved form of the existing sensorless hard-stop homing proof of concept. Production homing is designed as a state machine with low-energy contact detection, back-off and precision re-approach, repeatability validation, expected-travel validation, and axis-dependent safe poses. It explicitly supports multi-stage homing where one axis must be temporarily homed or repositioned before the other axis can safely traverse its range.

The whole station may be installed with arbitrary static roll/pitch tilt. Mechanical yaw/pitch coordinates are therefore kept separate from world azimuth/elevation. Initial operation does **not** require an IMU. Installation tilt can be commissioned visually using a levelled fiducial calibration board and stored as a base-to-world transform. A future IMU can be added behind the same installation-pose interface without redesigning the tracking loop.

---

# 2. Design goals

## 2.1 Primary goals

The software shall:

1. Detect and track a selected object/person using the IMX500 camera.
2. Drive two CyberGear direct-drive axes in closed loop.
3. Maintain calibrated absolute yaw and pitch after each boot.
4. Prevent high-energy impacts with mechanical end stops.
5. Continue smooth tracking between discrete camera measurements through target-state prediction.
6. Handle temporary target loss predictably.
7. Support stationary hold and optional safe free-roaming/search modes.
8. Provide a developer web interface without compromising control-loop timing.
9. Support installation where the complete station is tilted.
10. Be extendable later for:
    - an IMU;
    - radar;
    - ultrasonic ranging;
    - alternate vision models;
    - alternate target estimators.
11. Expose sufficient high-rate telemetry for control tuning and performance profiling.
12. Degrade safely when vision, CAN, timing, or motor feedback is unhealthy.

## 2.2 Non-goals for v1

The first production architecture does not require:

- ROS 2;
- a dedicated Hailo AI accelerator;
- full SLAM;
- multi-camera triangulation;
- globally referenced geographic heading;
- online aggressive automatic PID tuning;
- hard real-time Linux;
- continuous IMU fusion;
- depth estimation from monocular vision.

These can be introduced later if measured requirements justify them.

---

# 3. Hardware baseline

## 3.1 Host

- Raspberry Pi 5 Model B Rev 1.0
- Debian GNU/Linux 13 (trixie)
- `aarch64`
- Linux kernel `6.18.39+rpt-rpi-2712`
- Hostname: `rpi-turret`
- Documentation and services must not depend on a hard-coded IP address

## 3.2 Camera

- Raspberry Pi AI Camera
- Sony IMX500
- CSI-2
- Kernel identity: `imx500 10-001a`
- Existing official MobileNet SSD pipeline verified

The IMX500 performs the neural-network inference on the camera hardware, so the Pi 5 primarily performs metadata handling, target association, geometry, prediction, control, telemetry, and UI work.

## 3.3 CAN

- Waveshare RS485 CAN HAT
- MCP2515 over SPI (`spi0.0`)
- Device tree:

```text
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

- Linux interface: `can0`
- CAN bitrate: 1 Mbit/s
- Extended 29-bit CyberGear frames
- Correct physical termination is mandatory; multiple unintended 120-ohm terminators have already been demonstrated to break communication.

## 3.4 Motors

| Axis | CAN ID | Drive | Notes |
|---|---:|---|---|
| Pitch | 100 (`0x64`) | Direct | Approximately balanced payload |
| Yaw | 101 (`0x65`) | Direct | Travel likely more restricted by payload |

The control software must not assume the historical travel values from the old proof of concept. Actual expected travel bands are commissioning parameters.

---

# 4. Core architectural principles

## 4.1 One owner of the motors

Only `controld` may open the control-authoritative SocketCAN path and issue CyberGear commands.

Neither `visiond` nor `webd` may command a motor directly.

This creates one point where the following can always be enforced:

- homing validity;
- soft limits;
- braking envelope;
- fault state;
- motor feedback freshness;
- safe shutdown;
- command-rate limiting.

## 4.2 Latest-state semantics instead of unbounded queues

Real-time tracking cannot tolerate accumulated stale commands.

For target measurements:

```text
new measurement replaces older unconsumed measurement
```

The controller is interested in the **freshest estimate**, not faithfully processing a backlog of old detections.

Event logs may be queued because they are diagnostic, but control inputs shall use bounded/latest-value storage.

## 4.3 Separate mechanical and world coordinates

Do not use the words “yaw” and “azimuth” interchangeably.

- `q_yaw`, `q_pitch`: mechanical joint coordinates
- `azimuth`, `elevation`: world/gravity-frame directions

The base can be tilted, so these are not generally equal.

## 4.4 Position validity is explicit

After power-up:

```text
position_valid = false
```

Motor encoder numbers alone are not sufficient to declare the physical station homed.

Only successful boot homing/calibration changes this to:

```text
position_valid = true
```

The CyberGear “set current mechanical position to zero” operation may be used as a convenience, but the authoritative mapping remains in the host control state.

## 4.5 Safety constraints are upstream of motor commands

Every tracking/search/manual target goes through the same path:

```text
requested target
    -> target/reference arbitration
    -> joint-space command
    -> safety envelope
    -> trajectory generator
    -> motor command
```

No mode gets to bypass the trajectory and safety layers.

---

# 5. Process architecture

```text
+-----------------------------------------------------------------------+
| Raspberry Pi 5                                                        |
|                                                                       |
|   +-----------------------+                                           |
|   | visiond               |                                           |
|   | Python / Picamera2    |                                           |
|   | IMX500 adapter        |                                           |
|   | target association    |                                           |
|   +-----------+-----------+                                           |
|               | TargetMeasurement                                     |
|               | latest-value IPC                                      |
|               v                                                       |
|   +---------------------------------------------------------------+   |
|   | controld                                                      |   |
|   | C++20                                                         |   |
|   |                                                               |   |
|   |  CAN RX/TX -> MotorStateHistory                              |   |
|   |  target time alignment                                       |   |
|   |  camera ray / frame transforms                               |   |
|   |  target estimator/predictor                                  |   |
|   |  reference manager                                           |   |
|   |  trajectory generator                                        |   |
|   |  safety supervisor                                           |   |
|   |  homing/calibration state machines                           |   |
|   |  safe shutdown state machine                                 |   |
|   +------+---------------------+----------------------+-----------+   |
|          |                     |                      |               |
|          | SocketCAN           | telemetry snapshot   | commands      |
|          v                     v                      v               |
|       can0                 +--------+             +---------+         |
|          |                 | logd   |             | webd    |         |
|          |                 +--------+             | FastAPI |         |
|          |                                        +----+----+         |
|          v                                             |              |
|    MCP2515 / CAN                                       | WebSocket    |
|       /        \                                       v              |
|    pitch      yaw                                  Browser UI          |
+-----------------------------------------------------------------------+
```

## 5.1 `visiond`

Responsibilities:

- initialise Picamera2 and IMX500 model;
- consume inference metadata;
- generate detections;
- select/associate target candidates;
- attach the camera `SensorTimestamp`;
- attach frame sequence number;
- publish a `TargetMeasurement`;
- optionally generate a low-priority preview stream.

It does **not**:

- read or write CAN;
- own motion constraints;
- implement end-stop safety;
- send motor setpoints.

Python is appropriate here because the IMX500 performs inference and the main work is camera integration and moderate-rate post-processing.

## 5.2 `controld`

Responsibilities:

- sole authoritative motor/CAN control;
- parse CyberGear frames;
- maintain latest motor state and short state history;
- mechanical homing;
- coordinate transforms;
- installation-pose calibration;
- target-state estimation;
- target prediction;
- trajectory generation;
- soft limits;
- collision envelope;
- fault handling;
- safe park/shutdown;
- control-loop timing instrumentation.

Recommended implementation: **C++20**.

Rationale:

- deterministic memory/control behavior;
- direct Linux SocketCAN APIs;
- easier use of fixed-rate scheduling;
- clean separation of critical loop from Python/UI work;
- mature numerical libraries.

## 5.3 `webd`

Responsibilities:

- read telemetry;
- display state;
- submit high-level operator/developer commands;
- initiate calibration/tuning/search/hold operations;
- display preview if enabled.

It must never open `can0` for authoritative control.

Suggested implementation:

- FastAPI / Uvicorn initially;
- WebSocket telemetry;
- simple HTML/TypeScript or JavaScript UI.

The service should be documented using a hostname, for example:

```text
http://<turret-hostname>:<port>
```

and optionally mDNS where available, not a fixed numeric IP.

## 5.4 `logd`

Optional separate process or a low-priority thread/process fed by non-blocking telemetry.

Responsibilities:

- persist control traces;
- record timing;
- record motor response;
- record homing results;
- save fault snapshots;
- support offline tuning.

Loss or slowdown of logging must never block `controld`.

---

# 6. IPC design

## 6.1 Recommended transport

For v1:

- Unix-domain socket for `visiond -> controld`
- Unix-domain socket or shared-memory snapshot for `controld -> webd/logd`

A good control IPC choice is `SOCK_SEQPACKET` or a small fixed binary protocol because message boundaries are preserved.

Shared memory may be introduced for video frames, but **video frames should not traverse the critical control IPC path**.

## 6.2 Target measurement schema

Conceptually:

```text
TargetMeasurement {
    uint64 frame_sequence
    uint64 sensor_timestamp_ns

    bool valid

    int class_id
    float confidence

    float bbox_x_min_norm
    float bbox_y_min_norm
    float bbox_x_max_norm
    float bbox_y_max_norm

    float anchor_u_px
    float anchor_v_px

    optional uint64 visual_track_id
}
```

The `sensor_timestamp_ns` is mandatory.

## 6.3 Telemetry snapshot

```text
SystemTelemetry {
    timestamp_ns
    system_state

    vision_state
    target_state

    yaw_axis_state
    pitch_axis_state

    safety_state
    calibration_state
    can_state
    timing_state
}
```

`webd` should receive downsampled snapshots, e.g. 10–20 Hz. High-rate logs can run separately.

---

# 7. Timing architecture

## 7.1 Initial rates

| Component | Initial target |
|---|---:|
| Camera / IMX500 measurement | Camera model rate, nominally ~30 Hz |
| Target estimator measurement update | On every fresh vision measurement |
| Main control loop | **200 Hz** |
| Motor reference update | **200 Hz** initially |
| CAN receive | Event-driven |
| Safety evaluation | Every control cycle |
| UI telemetry | 10–20 Hz |
| High-rate diagnostics | Up to control rate |

Do not hard-code an assumption that the camera is exactly 30 Hz. Measure the real frame timestamps.

## 7.2 Why 200 Hz initially

The CyberGear contains its own lower-level motor loops. The host does not need to perform electrical commutation or a kHz current loop.

A 200 Hz host loop provides:

- 5 ms reference/safety update period;
- substantial margin above the visual update rate;
- sufficient resolution for smooth trajectory generation;
- moderate CAN traffic;
- a sensible starting point for MCP2515/SPI/Linux scheduling.

The implementation shall include profiling so 100, 200, 250, and potentially higher rates can be compared using:

- p50 loop period;
- p95;
- p99;
- worst observed jitter;
- CAN feedback age;
- missed deadlines;
- CPU utilization.

The selected production rate must be based on measurements.

## 7.3 Monotonic time

All critical timing shall use a monotonic clock.

The camera's `SensorTimestamp` is measured in nanoseconds since system boot and is associated with the camera frame timing. The motor-state history and control loop should use the same monotonic system time domain where possible.

Use absolute-time sleeping for the control loop to avoid drift:

```text
next_deadline += period
sleep_until(next_deadline)
```

rather than:

```text
sleep(period)
```

after each iteration.

## 7.4 Scheduling escalation path

Start with the standard kernel.

If profiling shows unacceptable jitter, apply progressively:

1. eliminate blocking work from control thread;
2. preallocate memory;
3. isolate logging/UI;
4. CPU affinity;
5. `SCHED_FIFO` with carefully selected priority;
6. `mlockall()` if required;
7. IRQ affinity tuning;
8. only then evaluate PREEMPT_RT if necessary.

The architecture must not require PREEMPT_RT to function correctly.

---

# 8. CAN subsystem

## 8.1 SocketCAN

`controld` should use Linux SocketCAN directly rather than a request/flush/read abstraction inherited from the Python POC.

Recommended design:

```text
CAN RX thread
    -> parse all relevant CyberGear frames
    -> timestamp
    -> atomically update motor state
    -> append to short MotorStateHistory ring

Control thread
    -> reads immutable latest snapshots
    -> computes references
    -> sends commands through CAN TX API
```

The RX path must not be blocked by the control algorithm.

## 8.2 Filtering

Install CAN RAW filters so `controld` receives the relevant CyberGear extended-frame traffic and CAN error frames required for diagnostics.

Track:

- interface state;
- RX/TX frame count;
- bus errors;
- stale motor feedback;
- motor fault flags;
- frame parse failures.

## 8.3 Motor state

Per axis:

```text
AxisMotorState {
    timestamp_ns
    raw_position_rad
    logical_position_rad
    velocity_rad_s
    torque_nm_or_reported_effort
    iq_if_available
    temperature
    mode
    fault_flags
    feedback_sequence
}
```

## 8.4 Feedback history

Retain roughly 0.5–1.0 s of axis state at high rate.

Purpose:

- interpolate motor pose at camera capture timestamp;
- diagnose latency;
- calculate measured settling/overshoot;
- support payload-response verification.

A fixed-size lock-free or low-contention ring buffer is appropriate.

## 8.5 CyberGear control mode

### v1 baseline: position mode

Use:

- host-generated smooth `q_ref(t)`;
- CyberGear position mode;
- configured `limit_spd`;
- configured `limit_cur`;
- tuned internal position/speed loop gains.

The host must **not** send a final desired angle as a step and rely on the motor to make the entire move safely. It should stream a safe trajectory.

### Future: operation/MIT mode

Evaluate only after position-mode behavior is characterized.

Potential benefits:

- position + velocity reference in one command;
- explicit PD terms;
- torque feed-forward;
- pitch gravity compensation;
- tighter host-side motion behavior.

It also moves more control responsibility to the host and therefore should not be the initial mode for an untuned heavy payload.

---

# 9. Coordinate frames

Define explicit transforms.

```text
W: world / gravity frame
B: fixed turret base frame
Y: yaw rotating frame
P: pitch rotating frame
C: camera optical frame
```

Use one transform convention throughout the codebase, e.g.:

```text
R_A_B = rotation that expresses a vector from frame B in frame A
```

Document this convention in code and tests.

## 9.1 Transform chain

For a camera ray:

```text
r_B =
    R_B_Y(q_yaw)
  * R_Y_P(q_pitch)
  * R_P_C
  * r_C
```

If world coordinates are needed:

```text
r_W = R_W_B * r_B
```

Where:

- `R_P_C`: persistent camera-to-gimbal extrinsic calibration
- `R_W_B`: installation tilt/orientation calibration
- `q_yaw`, `q_pitch`: current calibrated joint angles

## 9.2 Base tilt

The tracker should be able to operate entirely in `B` coordinates.

Therefore loss or absence of an IMU does **not** prevent basic vision tracking.

`R_W_B` is needed for features such as:

- world-horizontal search;
- world elevation;
- gravity-aware modeling;
- installation telemetry;
- future gravity feed-forward.

---

# 10. Camera geometry

## 10.1 Intrinsics

Store calibrated:

```text
fx, fy, cx, cy
distortion coefficients
image dimensions
```

The detector bounding box is converted into a configurable tracking anchor.

Default v1 anchor:

```text
bbox center
```

Later it may use a pose keypoint or another object-class-specific stable anchor.

## 10.2 Pixel to ray

After distortion correction:

```text
x = (u - cx) / fx
y = (v - cy) / fy
```

Then:

```text
r_C = normalize([x, y, 1])
```

Use full projective geometry rather than treating pixels as degrees with a constant multiplier.

## 10.3 Camera-to-gimbal extrinsic calibration

Camera optical center/axis may not be perfectly aligned with the pitch frame.

Store:

```text
T_P_C
```

or at minimum:

```text
R_P_C
```

A fiducial-board calibration procedure can estimate this.

---

# 11. Camera/motor timestamp alignment

This is a core requirement.

A target detection arriving at time `t_now` describes an image captured at `t_frame`.

During camera processing, the turret may have moved.

Therefore do not compute:

```text
target_base = current_motor_pose + current_pixel_offset
```

Instead:

1. receive `TargetMeasurement(sensor_timestamp_ns=t_frame)`;
2. query/interpolate yaw and pitch from `MotorStateHistory` at `t_frame`;
3. construct the camera transform at `t_frame`;
4. convert the visual measurement to a base-frame line of sight;
5. update the target estimator;
6. predict the target to the intended actuation time.

This avoids introducing apparent target motion caused only by pipeline latency.

## 11.1 Interpolation

For bracketing motor states:

```text
(t0, q0)
(t1, q1)
```

use linear interpolation initially:

```text
alpha = (t_frame - t0) / (t1 - t0)
q(t_frame) = q0 + alpha * (q1 - q0)
```

Velocity can be interpolated similarly.

If the history cannot cover the camera timestamp, flag the measurement as timing-invalid and do not silently use the current pose.

---

# 12. Target association

## 12.1 Detection versus tracking

The IMX500 detector provides object observations; a separate temporal association layer should provide a stable target identity.

Initial association can be lightweight:

- class gating;
- confidence threshold;
- IoU;
- predicted centroid proximity;
- track age;
- track continuity.

This is sufficient before introducing more expensive re-identification.

## 12.2 Target selection

Support:

- explicit target selected from the web UI;
- configured class preference;
- automatic reacquisition of the previous track;
- configurable fallback selection.

The motor control path receives one selected target at a time.

## 12.3 Detector-side temporal filtering

Avoid stacking several opaque temporal filters.

Prefer:

```text
relatively raw detector observations
    -> explicit association
    -> explicit estimator
```

rather than:

```text
strong detector hysteresis
    -> association smoothing
    -> Kalman smoothing
    -> motor smoothing
```

The objective is controlled, measurable latency.

---

# 13. Target-state estimator

## 13.1 Initial state

For each angular dimension, begin with:

```text
state = [angle, angular_velocity]
```

A 2D combined state can be:

```text
x = [
    azimuth_or_base_LOS_x,
    elevation_or_base_LOS_y,
    azimuth_rate,
    elevation_rate
]
```

For the initial implementation, a constant-angular-velocity Kalman filter or alpha-beta filter is appropriate.

## 13.2 Measurement update

At every valid camera measurement:

```text
z_k = measured base-frame LOS
```

Update state and covariance.

Measurement noise can depend on:

- bounding-box size;
- confidence;
- detector jitter;
- target near image edge.

## 13.3 Prediction

Between frames, propagate:

```text
angle(t + dt) =
    angle(t) + angular_velocity(t) * dt
```

Predict forward to the time the generated motor setpoint is expected to matter.

A useful effective horizon is:

```text
prediction_horizon =
    current_time - sensor_timestamp
  + estimated_control_delay
  + estimated_motor_response_delay
```

The latter two values should come from profiling.

## 13.4 Upgrade path

Only after measured evidence of benefit:

```text
[angle, angular_velocity, angular_acceleration]
```

or another motion model can be introduced.

A simpler correctly timestamped model is preferred to a more complicated model with inaccurate latency.

---

# 14. From target line of sight to desired joints

The target estimator produces a desired viewing ray.

For a two-axis gimbal, solve for:

```text
q_yaw_target
q_pitch_target
```

such that the camera optical axis aligns with the desired ray, accounting for:

- camera extrinsic transform;
- yaw axis;
- pitch axis;
- installation geometry.

In a perfectly orthogonal ideal mechanism, this reduces to angular decomposition. In implementation, use the actual frame transforms so small build misalignments can be calibrated.

The desired joint target is then passed to the reference and safety layers.

---

# 15. Control hierarchy

```text
IMX500 detections                ~camera rate
        |
        v
target association
        |
        v
timestamp-aligned LOS measurement
        |
        v
target estimator
        |
        | predicted LOS
        v
joint target solver
        |
        | requested q*
        v
reference manager
        |
        v
SafetyEnvelope
        |
        v
jerk-limited trajectory generator   200 Hz
        |
        | q_ref, optionally v_ref
        v
CyberGear internal position/speed loops
        |
        v
direct-drive mechanics
        |
        v
encoder/velocity/torque feedback
```

---

# 16. Reference manager

The reference manager arbitrates all sources:

1. calibration/homing;
2. emergency/safety stop;
3. controlled shutdown/park;
4. tracking;
5. search/free roaming;
6. stationary hold;
7. developer/manual test commands.

Priority example:

```text
FAULT / safety action
> HOMING
> SHUTDOWN
> TRACKING
> SEARCH
> HOLD
> developer request
```

A lower-priority source can never override a higher-priority safety state.

---

# 17. Motion trajectory generation

## 17.1 Requirements

Each axis trajectory shall enforce:

```text
|velocity|     <= v_max
|acceleration| <= a_max
|jerk|         <= j_max
```

as well as position constraints.

For a direct-drive heavy payload, jerk limiting is important because abrupt acceleration changes translate directly to abrupt torque demand.

## 17.2 Recommended trajectory

Use an online jerk-limited S-curve / time-optimal trajectory generator that can accept changing target positions every control cycle.

Inputs:

```text
current q, v, a
desired q
joint limits
velocity limit
acceleration limit
jerk limit
```

Outputs every cycle:

```text
q_ref
v_ref
a_ref
```

Position mode initially consumes `q_ref`, while `v_ref/a_ref` remain useful for safety and diagnostics.

## 17.3 Tracking target moves during trajectory

The generator must be online/receding-horizon.

Every cycle:

```text
new predicted target
    -> update destination
    -> continue from current planned q/v/a state
```

Do not restart a discontinuous trajectory from zero velocity after every new camera frame.

---

# 18. End-stop safety and dynamic braking

Simple position clipping is insufficient.

The controller must account for current velocity and ability to stop.

## 18.1 Physical and logical limits

For each axis store:

```text
q_hard_min_est
q_hard_max_est

q_soft_min
q_soft_max
```

where:

```text
q_soft_min > q_hard_min_est
q_soft_max < q_hard_max_est
```

The distance between hard and soft limits includes:

- homing uncertainty;
- stop repeatability;
- control delay;
- braking margin;
- mechanical tolerance.

## 18.2 Braking envelope

A first conservative speed bound is:

```text
v_allowed ~= sqrt(2 * a_brake * d_remaining)
```

with latency and margin subtracted from the usable distance.

Because the actual planner is jerk limited, production code should derive the stop from the same trajectory model rather than relying only on the constant-deceleration equation.

Every cycle, evaluate:

> Can the axis still generate a valid full-stop trajectory before the soft boundary?

If not, override tracking and enter braking.

## 18.3 Safety layers

Layer 1: trajectory planner obeys limits normally.

Layer 2: independent `SafetyEnvelope` checks predicted stop feasibility.

Layer 3: imminent boundary violation forces a controlled maximum-safe stop.

Layer 4: motor/CAN hard fault may require motor disable.

The safety checker should be independently testable from the normal trajectory generator.

---

# 19. Coupled collision envelope

The physical payload may restrict yaw differently at different pitch angles.

Therefore the architecture supports a joint-space safe region:

```text
q_yaw_min = f(q_pitch)
q_yaw_max = g(q_pitch)
```

or a more general 2D polygon/piecewise envelope.

For v1, constant yaw and pitch limits can be configured. The interface must nevertheless accept a coupled envelope later without rewriting the controller.

Examples of configuration representations:

- piecewise table of pitch vs yaw min/max;
- interpolated boundary;
- joint-space polygon.

The trajectory safety checker must validate the **path**, not only the endpoint.

---

# 20. Sensorless homing: design

## 20.1 Existing POC

The current proof of concept already demonstrates the core sensorless approach:

1. set CyberGear speed mode;
2. limit current;
3. drive toward a stop;
4. poll mechanical position and current;
5. infer stop contact from low position change + sufficiently high current;
6. establish one endpoint;
7. traverse to the opposite endpoint;
8. measure travel;
9. move toward the middle.

This is a useful starting point.

The production architecture replaces the polling loop with a dedicated homing state machine driven by continuous feedback.

## 20.2 Why the old implementation should not be used directly

The POC is intentionally rough and contains characteristics unsuitable for the production loop:

- roughly 0.5 s homing polling intervals;
- a small number of samples for contact inference;
- synchronous request/receive behavior;
- no explicit back-off/re-approach;
- no repeatability validation;
- no independent hard-contact threshold;
- no general multi-axis collision-aware homing plan;
- old historical travel assumptions that no longer match the new <180-degree mechanism;
- motor zero is treated more centrally than desired for the production logical coordinate model.

These are architecture issues, not criticisms of the POC's usefulness.

---

# 21. Production sensorless contact detector

End-stop contact should be declared only when multiple conditions agree for a configurable dwell time.

For commanded approach direction `s`:

```text
s = sign(commanded_velocity)
```

Candidate contact requires:

```text
abs(measured_velocity) < v_stall_threshold
AND
abs(delta_position_window) < q_stall_threshold
AND
signed_effort_in_stop_direction > effort_contact_threshold
AND
command has been active long enough
```

Optionally include measured `iqf`.

The detection must persist for:

```text
contact_dwell_ms
```

rather than one sample.

## 21.1 Filtered signals

Maintain short low-pass/median-filtered values for:

- velocity;
- effort/torque/current;
- position progress.

Do not excessively smooth them; the purpose is reject single-frame noise, not introduce hundreds of milliseconds of lag.

## 21.2 Independent hard abort

If:

```text
effort > effort_hard_abort
```

or a motor fault occurs, immediately command zero motion / safe action without waiting for the normal contact dwell.

This protects against a failed stall detector.

---

# 22. Precision homing sequence for one endpoint

Recommended endpoint procedure:

```text
APPROACH_COARSE
    |
    | first contact
    v
STOP_AND_SETTLE
    |
    v
BACK_OFF
    |
    v
SETTLE
    |
    v
APPROACH_FINE
    |
    | validated contact
    v
RECORD_ENDPOINT
```

Optional repeat:

```text
BACK_OFF_SMALL
    -> APPROACH_FINE
    -> RECORD_SECOND_SAMPLE
```

Then compare endpoint samples.

If:

```text
abs(q_contact_1 - q_contact_2) > homing_repeatability_limit
```

the endpoint is invalid and homing fails.

This gives a measurable homing confidence instead of a binary result from a single stall event.

---

# 23. Full-axis homing

To learn both limits:

```text
precision-home endpoint A
    -> retreat
    -> traverse safely
    -> precision-home endpoint B
    -> compute measured travel
```

Then validate:

```text
expected_travel_min
<= measured_travel
<= expected_travel_max
```

These bounds are commissioning configuration and must match the current mechanism.

If travel is out of range:

```text
HOMING_FAILED
position_valid = false
```

The station does not enter tracking mode.

---

# 24. Host logical joint coordinates

After endpoints are measured, define logical coordinates independently from the motor's volatile zero.

For example:

```text
q_logical =
    direction_sign * (q_raw - q_raw_reference)
    + q_reference_logical
```

Configuration includes:

```text
direction_sign
home_reference
logical_zero_offset
soft_min
soft_max
```

Possible logical-zero conventions:

- geometric center;
- preferred optical-forward pose;
- one physical reference endpoint plus known offset.

Use a convention meaningful to the mechanism.

CyberGear's zero command may optionally be issued after homing to keep motor diagnostics intuitive, but reboot safety does not depend on it.

---

# 25. Multi-axis homing dependency graph

The payload may require one axis to be at a safe angle before another can traverse.

Do not hard-code only:

```text
home pitch
home yaw
```

Instead define an executable homing plan.

Example:

```text
BOOT
 |
 v
COARSE_HOME_PITCH_REFERENCE
 |
 | enough position knowledge to move safely
 v
MOVE_PITCH_TO_YAW_CLEARANCE_POSE
 |
 v
FULL_HOME_YAW
 |
 v
MOVE_YAW_TO_PITCH_CLEARANCE_POSE
 |
 v
FULL_HOME_PITCH
 |
 v
OPTIONAL_REVERIFY_YAW_REFERENCE
 |
 v
MOVE_TO_READY_POSE
```

This supports “home twice at different angle” without redesign.

## 25.1 Coarse versus full home

A **coarse reference home** may find only one known stop so the axis becomes sufficiently referenced to make the next safe move.

A **full home** can then measure/revalidate both endpoints.

This is useful when a full initial sweep would risk payload collision.

## 25.2 Homing plan configuration

Example:

```yaml
homing_plan:
  - action: home_endpoint
    axis: pitch
    endpoint: lower
    precision: coarse

  - action: move
    axis: pitch
    position_deg: PITCH_YAW_CLEARANCE

  - action: home_full_range
    axis: yaw
    precision: fine

  - action: move
    axis: yaw
    position_deg: YAW_PITCH_CLEARANCE

  - action: home_full_range
    axis: pitch
    precision: fine
```

Real values are provided during commissioning.

---

# 26. Homing safety requirements

During homing:

- tracking is disabled;
- search is disabled;
- web manual motion commands are ignored except abort;
- speed is intentionally low;
- current/torque limit is intentionally low;
- normal high-speed gains/profile are not required;
- CAN feedback must be fresh;
- timeout exists for every movement;
- maximum allowed homing travel exists;
- temperature/fault checks remain active;
- a failed stage leaves the system unhomed.

A physical stop used for sensorless homing must be mechanically designed to tolerate repeated controlled low-energy contact.

If future hardware allows it, non-contact Hall/optical references remain a worthwhile upgrade because they eliminate deliberate stop contact.

---

# 27. Boot state machine

```text
POWER_ON
   |
   v
PROCESS_INIT
   |
   v
CAN_INIT
   |
   v
DISCOVER_EXPECTED_MOTORS
   |
   v
MOTOR_SELF_TEST
   |
   v
UNHOMED
   |
   v
EXECUTE_HOMING_PLAN
   |
   v
VALIDATE_TRAVEL_AND_LIMITS
   |
   +------failure------> FAULT_LOCKED
   |
   v
LOAD/VALIDATE_CAMERA_CALIBRATION
   |
   v
LOAD_INSTALLATION_POSE
   |
   v
OPTIONAL_PAYLOAD_RESPONSE_CHECK
   |
   v
MOVE_TO_READY/HOLD_POSE
   |
   v
READY_HOLD
```

The station never automatically starts tracking while position validity is unknown.

---

# 28. Calibration types and persistence

Separate calibration concepts.

## 28.1 Mechanical home calibration

Frequency:

- every cold boot/power loss;
- after motor zero/reference invalidation;
- after suspected mechanical slip.

Produces:

- raw endpoint values;
- measured travel;
- logical coordinate mapping;
- homing repeatability;
- current soft-limit envelope.

## 28.2 Camera intrinsic calibration

Frequency:

- commissioning;
- when camera/lens geometry changes.

Produces:

- camera matrix;
- distortion parameters.

## 28.3 Camera-to-gimbal extrinsic calibration

Frequency:

- commissioning;
- after camera mount changes.

Produces:

- `T_P_C`.

## 28.4 Installation orientation calibration

Frequency:

- installation;
- whenever station base is physically moved;
- optionally verified periodically.

Produces:

- `R_W_B`.

## 28.5 Payload profile

Frequency:

- commissioning per meaningful payload configuration;
- verification at boot or when payload changed.

Produces:

- controller gain profile;
- safe velocity/acceleration/jerk limits;
- response baseline.

---

# 29. Installation tilt without an IMU

## 29.1 Principle

A static installation can be calibrated visually.

The station does not need to be physically level.

A known calibration object is aligned with the real-world vertical/horizontal reference, then observed by the camera.

Recommended object:

- ChArUco board; or
- AprilTag/fiducial board with known metric geometry.

The board is physically levelled/plumb during commissioning.

## 29.2 Pose solve

From:

- camera intrinsics;
- fiducial geometry;
- detected corners;
- current calibrated yaw/pitch;
- camera extrinsics;

solve camera pose relative to the fiducial/world frame.

Then derive:

```text
R_W_B
```

and store it.

## 29.3 Multi-frame solve

Do not accept one frame.

Recommended calibration flow:

```text
detect board
    -> collect N valid frames/poses
    -> solve each
    -> reject outliers
    -> average/final optimization
    -> report reprojection error
    -> move turret to validation pose
    -> validate
    -> commit calibration atomically
```

## 29.4 Limit of arbitrary scene inference

The system should not treat arbitrary visual cues such as a person, wall, tree, or furniture as calibration-grade gravity references.

Such cues may later provide a health check, but the canonical installation transform should come from:

- a known visually level reference; or
- a future IMU.

---

# 30. Future IMU integration

Define:

```text
InstallationPoseProvider
```

with implementations:

```text
FixedStoredPoseProvider
VisualCalibrationPoseProvider
ImuPoseProvider
FusedVisualImuPoseProvider
```

The tracking/control pipeline consumes a standard:

```text
BaseOrientation {
    timestamp
    R_W_B
    covariance
    source
    valid
}
```

A future IMU therefore does not require rewriting the tracker.

For a stationary base, the IMU can mainly provide:

- gravity direction;
- movement/tilt-change detection;
- vibration diagnostics;
- revalidation of stored installation orientation.

It does not need to become part of the high-bandwidth motor servo loop.

---

# 31. Payload variation and motor tuning

## 31.1 Problem

Even with the payload approximately balanced, rotational inertia can vary substantially.

A new payload can change:

- rise time;
- overshoot;
- settling;
- current/torque demand;
- braking distance;
- stable gain range.

## 31.2 Separation of concerns

Do not combine:

```text
homing
```

and:

```text
PID/controller tuning
```

into one automatic procedure.

Every boot requires homing.

Controller tuning is a controlled commissioning operation.

## 31.3 Payload response verification

Optionally after homing, execute a small motion in a safe central region.

Example concept:

```text
small positive move
small negative move
```

using conservative limits.

Measure:

- rise time;
- overshoot;
- settling;
- peak effort;
- position error;
- velocity response.

Compare with stored profile.

If materially different:

```text
payload_profile_status = MISMATCH
```

Then either:

- select a conservative profile; or
- derate motion; or
- request a tuning procedure.

## 31.4 Tuning policy

Allow:

- manually approved gain profiles;
- scripted identification and analysis;
- automatic suggestion of gains offline.

Do **not** allow unconstrained aggressive online PID self-tuning during normal startup.

## 31.5 Axis-specific gains

Yaw and pitch must have separate profiles.

Pitch may also need gain scheduling as a function of angle if gravity/unbalance matters.

---

# 32. Pitch gravity effects

The payload should be approximately balanced, reducing static gravitational torque.

However, residual center-of-mass offset may remain.

Position-mode v1:

- use conservative tuned internal loops;
- log effort vs pitch;
- characterize whether gravity compensation is actually needed.

Future operation mode may add torque feed-forward based on:

- calibrated installation gravity vector;
- pitch angle;
- identified residual mass/lever-arm parameter.

Do not add this complexity until measurements show benefit.

---

# 33. Safe shutdown / park

The requested pitch behavior is:

> move pitch to a safe position close to a mechanical stop, then remove motor drive.

This should be an explicit state machine, not a generic `disable()`.

Example:

```text
SHUTDOWN_REQUESTED
    |
    v
STOP_TRACKING_AND_SEARCH
    |
    v
TRAJECTORY_TO_YAW_PARK
    |
    v
TRAJECTORY_TO_PITCH_PARK
    |
    | verify position + low velocity
    v
PARK_DWELL
    |
    v
DISABLE_PITCH
    |
    v
DISABLE_YAW
    |
    v
PARKED_POWER_SAFE
```

The exact order may be reversed if mechanical geometry requires it.

## 33.1 Pitch park location

Store:

```text
pitch_park_position
```

inside the calibrated soft limit, **not directly against the mechanical stop**.

The park position should leave enough margin that normal shutdown does not repeatedly load the hard stop.

## 33.2 Verification

Before disable:

```text
abs(q - q_park) < park_position_tolerance
abs(v) < park_velocity_tolerance
condition held for park_dwell_ms
```

## 33.3 Commanded versus uncontrolled power loss

Software can guarantee the park sequence only for a commanded shutdown while motor power remains available.

It cannot guarantee movement to park after abrupt power removal.

If uncontrolled power loss becomes a safety-critical case, hardware mitigation is required, such as:

- balancing/counterweighting;
- mechanical brake;
- spring/counterbalance;
- hold-up power/UPS.

---

# 34. Tracking state machine

```text
READY_HOLD
    |
    | target acquired
    v
TRACKING
    |
    | measurement temporarily missing
    v
COASTING
    |
    +---target reacquired---> TRACKING
    |
    | timeout
    v
BRAKE_TO_HOLD
    |
    v
TARGET_LOST
    |
    +---search enabled-----> SEARCH
    |
    +---otherwise----------> READY_HOLD
```

All timing thresholds are configuration.

Example starting behavior, to be tuned:

- very short dropout: continue estimator prediction;
- longer dropout: progressively reduce confidence and start braking;
- extended dropout: stop at a safe hold position or begin configured search.

Do not continue extrapolating an old target indefinitely.

---

# 35. Confidence-aware target prediction

The estimator should maintain uncertainty/confidence.

As vision goes stale:

```text
covariance increases
```

and allowed prediction aggressiveness decreases.

For example, the reference manager can scale tracking velocity based on target confidence:

```text
high confidence -> normal tracking limits
degrading confidence -> reduced acceleration/speed
lost -> controlled stop
```

This avoids a fast blind extrapolation when the target disappears.

---

# 36. Search / free-roaming mode

Search is optional and disabled by default unless configured.

It uses the exact same trajectory/safety path as tracking.

Example pattern:

- safe azimuth sweep;
- bounded elevation;
- smooth S-curve turnarounds;
- reduced velocity/acceleration relative to tracking;
- obey coupled collision envelope.

If `R_W_B` is known, search can be world-horizontal.

If it is not known, search can operate safely in base joint coordinates.

---

# 37. Stationary hold

When no target and search is disabled:

- complete a smooth stop;
- hold the current safe joint position in position mode;
- continue monitoring CAN, temperature, and faults.

Do not disable the motors merely because vision is absent unless explicitly configured.

---

# 38. Safety supervisor

The safety supervisor runs every control cycle and has authority above tracking.

Inputs:

- logical joint state;
- planned trajectory;
- motor feedback freshness;
- CAN status;
- motor faults;
- homing validity;
- target confidence;
- control timing health;
- calibration validity.

Outputs:

```text
ALLOW
DERATE
BRAKE
HOLD
FAULT_STOP
DISABLE
```

## 38.1 Typical conditions

### Vision stale

Response:

```text
COAST -> BRAKE -> HOLD/SEARCH
```

Not a motor fault.

### CAN feedback stale

Response:

- stop issuing aggressive tracking commands;
- command safe stop if communication is still possible;
- fault if required;
- never continue open-loop movement.

### Homing invalid

Response:

- no tracking;
- only restricted calibration/homing motion.

### Limit stop feasibility violated

Response:

- safety overrides tracking and commands braking.

### Motor over-temperature/fault

Response depends on fault type:

- controlled deceleration if possible;
- disable if motor/controller protection demands it.

### Control-loop deadline failures

A small isolated miss is logged.

Repeated misses:

- reduce noncritical workload;
- derate/hold;
- fault if feedback/control integrity cannot be guaranteed.

---

# 39. Watchdogs

Use layered watchdogs.

## 39.1 Vision watchdog

Tracks:

```text
age_of_last_valid_measurement
```

Transitions tracking state but does not directly disable motors.

## 39.2 Motor feedback watchdog

Per axis:

```text
age_of_last_motor_feedback
```

Stale feedback invalidates high-speed control.

## 39.3 Control-loop watchdog

Detect:

- missed deadlines;
- stalled control thread;
- abnormal cycle time.

A small external watchdog helper may monitor `controld` health.

## 39.4 Motor-side CAN timeout

CyberGear exposes a CAN-timeout-related parameter in available documentation. Its exact units and behavior on the installed firmware must be experimentally validated before using it as an independent safety layer.

Do not assume it is reliable without hardware verification.

---

# 40. Configuration architecture

Use versioned YAML.

Example structure:

```yaml
schema_version: 1

can:
  interface: can0
  bitrate: 1000000
  host_can_id: 0

motors:
  pitch:
    can_id: 100
    direction_sign: 1
  yaw:
    can_id: 101
    direction_sign: -1

control:
  loop_hz: 200

axes:
  pitch:
    expected_travel_deg:
      min: TBD
      max: TBD
    soft_margin_deg: TBD
    max_velocity_deg_s: TBD
    max_acceleration_deg_s2: TBD
    max_jerk_deg_s3: TBD

  yaw:
    expected_travel_deg:
      min: TBD
      max: TBD
    soft_margin_deg: TBD
    max_velocity_deg_s: TBD
    max_acceleration_deg_s2: TBD
    max_jerk_deg_s3: TBD

homing:
  contact:
    coarse_speed_deg_s: TBD
    fine_speed_deg_s: TBD
    current_or_effort_limit: TBD
    stall_velocity_threshold: TBD
    contact_dwell_ms: TBD
    backoff_deg: TBD
    repeatability_deg: TBD

tracking:
  search_enabled_by_default: false
  target_lost_behavior: hold

shutdown:
  yaw_park_deg: TBD
  pitch_park_deg: TBD

camera:
  intrinsics_file: calibration/camera_intrinsics.yaml
  extrinsics_file: calibration/camera_extrinsics.yaml

installation:
  pose_file: calibration/installation_pose.yaml

payload:
  active_profile: conservative
```

No mechanical value should be inferred from the old POC's historical 75/340-degree checks.

---

# 41. Calibration data integrity

Persistent calibration files should include:

```text
schema version
creation timestamp
hardware identifiers
configuration revision
calibration result
quality metrics
```

Use atomic update:

1. write temporary file;
2. fsync if appropriate;
3. rename over previous file.

Retain the previous known-good calibration where useful.

Do not silently accept a file for the wrong mechanism revision.

---

# 42. Web interface

## 42.1 Dashboard

Display:

### System

- state;
- uptime;
- `position_valid`;
- calibration validity;
- control-loop frequency;
- p50/p95/p99/worst jitter;
- CPU load.

### Vision

- camera FPS;
- frame sequence;
- measurement age;
- selected target;
- class/confidence;
- bounding box;
- track status.

### Target estimator

- measured LOS;
- predicted LOS;
- angular rates;
- estimator uncertainty;
- prediction horizon.

### Yaw/pitch

- raw motor position;
- logical position;
- velocity;
- torque/effort/current;
- temperature;
- target position;
- commanded trajectory position;
- distance to soft boundary;
- braking status;
- homing validity.

### CAN

- interface state;
- RX/TX counters;
- error counters;
- feedback age by motor.

### Calibration

- measured travel;
- homing repeatability;
- camera calibration status;
- base installation roll/pitch;
- payload profile status.

## 42.2 Developer controls

Allowed high-level operations:

- hold;
- start/stop tracking;
- enable/disable search;
- select target;
- start mechanical homing;
- start installation visual calibration;
- start payload verification;
- request safe park/shutdown;
- run restricted test motion.

Every request goes through `controld` state validation.

## 42.3 Video

Video preview is optional and low priority.

If it materially increases CPU latency:

- reduce resolution;
- reduce FPS;
- disable overlay;
- disable stream.

Control timing wins over browser video.

---

# 43. Telemetry and logging

## 43.1 High-rate control log

For tuning, capture at or near control rate:

```text
timestamp
q_actual
v_actual
effort
q_ref
v_ref
a_ref
target_requested
soft-limit distance
safety mode
CAN feedback age
control cycle duration
```

for both axes.

## 43.2 Vision log

Capture:

```text
frame_sequence
SensorTimestamp
arrival_timestamp
bbox
confidence
selected track
measured LOS
filtered LOS
predicted LOS
```

This allows offline latency analysis.

## 43.3 Event log

Structured events:

- homing start/end;
- contact detected;
- homing repeatability result;
- target acquired/lost;
- safety brake;
- limit proximity;
- CAN fault;
- motor fault;
- loop overrun;
- calibration commit;
- shutdown.

## 43.4 Black-box ring buffer

Maintain a rolling in-memory history.

On fault, persist several seconds before/after the event if storage permits.

This is extremely useful for tuning and diagnosing sporadic limit or timing behavior.

---

# 44. Motor/payload profiling

The implementation should include a profiling utility rather than embedding tuning assumptions in the architecture.

Tests:

1. low-amplitude step response;
2. low-amplitude triangular motion;
3. optional chirp in a safe central region;
4. braking test from incrementally increasing speed;
5. holding-current/effort vs pitch.

Measure:

- rise time;
- settling time;
- overshoot;
- tracking RMS error;
- peak torque/current;
- stop distance;
- control latency.

The measured braking performance feeds the commissioning values for `a_brake`, soft margins, and motion limits.

---

# 45. Commissioning workflow

Recommended order:

## Phase A: static electrical checks

- verify CAN wiring/termination;
- verify IDs 100/101;
- verify feedback;
- verify motor direction signs;
- verify temperature/fault decoding.

## Phase B: low-energy individual-axis checks

With payload removed or safely supported where possible:

- command very low speed/current;
- validate direction;
- characterize contact current/effort;
- validate emergency stop behavior.

## Phase C: sensorless homing

- tune contact detector;
- validate repeated endpoint repeatability;
- define expected travel bands;
- establish soft margins.

## Phase D: park sequence

- define safe pitch park near, but not on, the intended mechanical-stop side;
- define yaw park;
- validate shutdown ordering.

## Phase E: payload installation

- repeat conservative homing;
- profile response;
- establish safe velocity/acceleration/jerk.

## Phase F: camera calibration

- intrinsics;
- camera-to-gimbal extrinsics.

## Phase G: installation tilt

- level/plumb fiducial board;
- compute `R_W_B`;
- verify world-horizontal/world-vertical interpretation.

## Phase H: integrated tracking

- stationary target;
- slow moving target;
- faster target;
- temporary occlusion;
- target loss near a soft limit;
- search/reacquisition.

---

# 46. Control-loop pseudocode

Conceptual `controld` cycle:

```text
loop every CONTROL_PERIOD:

    now = monotonic_time()

    axis = motor_state_snapshot()

    update_timing_health(now)

    if motor_feedback_stale(axis):
        safety.request_feedback_fault()

    consume_latest_target_measurement_if_new()

    if new_measurement:
        historical_pose = motor_history.interpolate(
            target.sensor_timestamp
        )

        if historical_pose.valid:
            ray_camera = camera.pixel_to_ray(target.anchor)
            ray_base = kinematics.camera_ray_to_base(
                ray_camera,
                historical_pose
            )

            target_estimator.measurement_update(
                target.sensor_timestamp,
                ray_base,
                target.confidence
            )

    target_estimator.predict_to(now + actuation_horizon)

    requested_reference = reference_manager.compute(
        system_state,
        target_estimator,
        calibration_state
    )

    safe_reference = safety_envelope.constrain(
        requested_reference,
        axis,
        calibration_limits,
        collision_envelope
    )

    trajectory_state = trajectory_generator.step(
        axis_or_previous_plan_state,
        safe_reference,
        dt
    )

    safety_envelope.verify_stop_reachability(
        trajectory_state,
        axis
    )

    if safety_override:
        trajectory_state = safety_stop_trajectory()

    motor_command = motor_adapter.make_position_command(
        trajectory_state.q_ref
    )

    can_tx(motor_command)

    publish_nonblocking_telemetry()
```

Important:

- no file I/O in this loop;
- no HTTP;
- no blocking video work;
- no synchronous register-query chain;
- no unbounded allocation;
- no sleeping relative to the end of work.

---

# 47. Homing pseudocode

```text
home_endpoint(axis, direction):

    require state == HOMING
    require CAN healthy
    require other axis is in required clearance region

    set low-current speed mode

    command coarse approach velocity

    while not timeout:
        feedback = latest_axis_feedback()

        if hard_fault(feedback):
            abort

        if contact_detector.confirmed(feedback):
            command zero speed
            wait until low velocity
            break

    command backoff trajectory
    verify encoder progress away from stop
    settle

    command fine approach velocity

    collect continuous feedback
    detect contact using:
        low velocity
        low position progress
        sustained signed effort/current
        dwell timer

    stop

    q1 = contact position

    optional:
        back off
        fine approach again
        q2 = contact position
        verify |q2 - q1| <= repeatability limit

    return robust endpoint estimate
```

---

# 48. Safety stop pseudocode

```text
remaining = distance_to_active_soft_boundary(axis)

stop_plan = trajectory_generator.plan_stop(
    q = axis.position,
    v = axis.velocity,
    a = estimated_or_planned_acceleration,
    a_limit = configured_braking_accel,
    j_limit = configured_braking_jerk
)

if stop_plan.end_position crosses soft boundary:
    state = LIMIT_EMERGENCY
    issue strongest validated controlled braking action
```

This should be unit-tested against randomized positions/velocities.

---

# 49. Search planner

Example conceptual search:

```text
SEARCH_LEFT
    -> approach left search boundary with S-curve
    -> decelerate to zero
    -> reverse smoothly
SEARCH_RIGHT
    -> ...
```

Search boundaries must remain strictly inside tracking soft limits so tracking still has braking margin if a target is acquired during search.

---

# 50. Recommended repository structure

```text
OpenAutoTurret/
|
+-- control/
|   +-- CMakeLists.txt
|   +-- src/
|   |   +-- main.cpp
|   |   +-- can/
|   |   |   +-- socketcan_bus.*
|   |   |   +-- cybergear_protocol.*
|   |   |   +-- cybergear_axis.*
|   |   +-- control/
|   |   |   +-- control_loop.*
|   |   |   +-- trajectory_generator.*
|   |   |   +-- safety_envelope.*
|   |   |   +-- reference_manager.*
|   |   +-- calibration/
|   |   |   +-- homing_controller.*
|   |   |   +-- contact_detector.*
|   |   |   +-- installation_pose.*
|   |   +-- tracking/
|   |   |   +-- target_estimator.*
|   |   |   +-- target_association.*
|   |   +-- geometry/
|   |   |   +-- camera_model.*
|   |   |   +-- turret_kinematics.*
|   |   +-- telemetry/
|   |       +-- telemetry_publisher.*
|   +-- tests/
|
+-- vision/
|   +-- visiond.py
|   +-- imx500_detector.py
|   +-- target_selector.py
|
+-- web/
|   +-- webd/
|   +-- frontend/
|
+-- config/
|   +-- turret.yaml
|   +-- payload_profiles/
|
+-- calibration/
|   +-- camera_intrinsics.yaml
|   +-- camera_extrinsics.yaml
|   +-- installation_pose.yaml
|
+-- tools/
|   +-- can_monitor/
|   +-- homing_profiler/
|   +-- motor_response_profiler/
|   +-- replay/
|
+-- docs/
    +-- architecture/
    +-- commissioning/
```

The old Python POC can remain under a legacy/prototype directory for reference, but the production CAN/control path should not depend on it.

---

# 51. Suggested C++ dependencies

Keep dependencies modest.

Potential choices:

- Eigen for vectors/matrices/quaternions;
- yaml-cpp for configuration;
- spdlog/fmt for logging;
- Catch2 or GoogleTest for unit tests.

SocketCAN itself can be used through Linux headers directly.

For online jerk-limited trajectory generation, either:

- implement and thoroughly test a bounded S-curve generator; or
- use a well-maintained real-time trajectory library after license/API review.

Do not make the web framework or Python environment a dependency of `controld`.

---

# 52. System service deployment

Recommended systemd units:

```text
turret-control.service
turret-vision.service
turret-web.service
turret-log.service   (optional)
```

Dependencies:

```text
turret-control
    After=network-independent local hardware init / can0 setup

turret-vision
    After=turret-control or camera availability

turret-web
    After=turret-control
```

`turret-control.service` should be restart-managed carefully. A crash/restart must return to:

```text
UNHOMED
```

not silently resume old coordinates.

---

# 53. Startup networking

Do not hard-code an IP in source or documentation.

Use:

- bind-all or configured interface for `webd`;
- hostname-based examples;
- optional mDNS (`rpi-turret.local`) if enabled;
- configuration/environment variable for port.

Motor control does not depend on network availability.

Wi-Fi/Ethernet failure must not disturb the deterministic local tracking loop.

---

# 54. Testing strategy

## 54.1 Unit tests

Required for:

- CyberGear frame encode/decode;
- bit fields;
- logical/raw coordinate conversion;
- angle wrap behavior;
- interpolation of motor history;
- camera ray conversion;
- frame transforms;
- Kalman/alpha-beta estimator;
- trajectory limit enforcement;
- stop-distance feasibility;
- collision envelope;
- contact detector;
- homing repeatability calculations;
- state transitions.

## 54.2 Simulation

Build a simple simulated axis plant:

```text
command -> motor lag -> inertia -> position/velocity
```

Add:

- end stops;
- noise;
- feedback delay;
- CAN dropout;
- variable inertia.

Use it to test state machines before repeated hardware contact.

## 54.3 Recorded vision replay

Record:

- detections;
- timestamps;
- motor traces.

Allow deterministic replay into `controld` without a live camera.

This is valuable for estimator and latency tuning.

## 54.4 Hardware-in-loop tests

Test:

- both CAN IDs;
- loss of one motor;
- error-active/passive/bus-off behavior where safely reproducible;
- delayed feedback;
- stop contact;
- current-limit behavior.

## 54.5 UI load test

Open multiple dashboard clients / video preview and verify:

- control-loop p99 does not materially degrade;
- no CAN feedback staleness;
- logging does not block.

---

# 55. Acceptance metrics

Exact numerical thresholds require commissioning, but the implementation should report at least:

## Control timing

- target loop rate;
- p50/p95/p99 cycle;
- worst cycle;
- deadline miss count.

## CAN

- feedback age per motor;
- dropped/invalid frames;
- error frames.

## Homing

- endpoint repeatability;
- measured travel;
- homing duration;
- peak homing effort;
- expected-range pass/fail.

## Tracking

- visual measurement latency;
- target LOS RMS error;
- mechanical pointing RMS error;
- overshoot;
- reacquisition time;
- lost-target stop behavior.

## Limits

- minimum observed distance to soft boundary during braking;
- measured stopping distance versus model;
- no hard-stop contact outside the deliberate homing procedure.

---

# 56. Implementation phases

## Phase 0 — Preserve and instrument the POC

- keep existing code as reference;
- record current CyberGear behavior;
- verify feedback units and signs;
- verify exact installed motor firmware behavior.

Deliverable:
- motor protocol test report.

## Phase 1 — Production CAN/motor layer

Implement in C++:

- SocketCAN;
- CyberGear encode/decode;
- async feedback;
- motor health;
- state history;
- basic position-mode commands.

Deliverable:
- two motors controllable independently with high-rate telemetry.

## Phase 2 — Homing and safety foundation

Implement:

- contact detector;
- coarse/fine endpoint homing;
- travel validation;
- host logical coordinates;
- soft limits;
- multi-axis homing plan;
- safe park/shutdown.

Deliverable:
- reliable boot -> homed -> safe hold -> park cycle.

This phase should be completed before integrated vision tracking.

## Phase 3 — Trajectory generator

Implement:

- velocity/acceleration/jerk limits;
- online target updates;
- stopping trajectories;
- braking envelope;
- coupled collision envelope interface.

Deliverable:
- arbitrary safe joint target tracking without camera.

## Phase 4 — Vision daemon

Implement:

- Picamera2/IMX500;
- detector parsing;
- `SensorTimestamp`;
- target selection/association;
- IPC.

Deliverable:
- timestamped target measurements independent of motor control.

## Phase 5 — Geometry and estimator

Implement:

- camera intrinsics;
- motor history interpolation;
- camera-to-base transforms;
- target LOS estimator;
- prediction.

Deliverable:
- predicted base-frame target LOS with replay tests.

## Phase 6 — Closed-loop tracking

Connect:

```text
vision -> estimator -> joint target -> trajectory -> motors
```

Tune:

- prediction horizon;
- target filter noise;
- tracking dynamics.

Deliverable:
- smooth object/person tracking.

## Phase 7 — Installation orientation calibration

Implement:

- fiducial detection;
- board pose solve;
- multi-frame outlier rejection;
- `R_W_B` persistence;
- world-frame telemetry.

Deliverable:
- correct tracking/world coordinate interpretation with a tilted base.

## Phase 8 — Web UI and diagnostics

Implement:

- dashboard;
- target selection;
- calibration controls;
- timing plots;
- low-priority preview.

Deliverable:
- complete developer interface without control-loop degradation.

## Phase 9 — Payload profiling/tuning

Implement:

- response tests;
- payload profile storage;
- mismatch detection;
- automatic derating.

Deliverable:
- repeatable commissioning for multiple payload configurations.

---

# 57. Key differences from the current POC

| Area | Current POC concept | Production architecture |
|---|---|---|
| CAN access | Python synchronous request/read | Dedicated async SocketCAN control daemon |
| Motor ownership | Turret/Python objects | Single authoritative `controld` |
| Homing sample rate | coarse polling | continuous feedback |
| Contact detection | position stability + current | filtered velocity + progress + signed effort + dwell |
| Endpoint | single contact | back-off + fine re-approach + repeatability |
| Axis order | fixed pitch then yaw | configurable collision-aware homing graph |
| Position zero | motor zero central | host logical coordinates authoritative |
| Travel values | historical constants | commissioning bounds |
| Limit safety | basic position guard | braking envelope + trajectory reachability |
| Motion command | direct position-style behavior | online jerk-limited reference |
| Vision timing | not integrated | camera timestamp + motor history interpolation |
| Target model | not integrated | angular LOS estimator + prediction |
| Tilt | not modeled | base/world transform with visual calibration |
| UI | not critical-loop isolated | separate low-priority web service |

---

# 58. Remaining commissioning parameters — not architecture blockers

No further architecture decision is required before implementation, but the following values must be measured or chosen before full-speed operation:

1. Actual pitch mechanical travel band.
2. Actual yaw mechanical travel band.
3. Soft-limit margins.
4. Pitch and yaw park positions.
5. Safe pitch pose for yaw homing.
6. Safe yaw pose for pitch homing.
7. Whether a second reference pass is required after cross-axis repositioning.
8. Coarse/fine sensorless homing speeds.
9. Homing current/torque limits.
10. Contact velocity/progress thresholds.
11. Contact dwell time.
12. Back-off angle.
13. Homing repeatability tolerance.
14. Maximum tracking velocity.
15. Maximum tracking acceleration.
16. Maximum tracking jerk.
17. Validated braking acceleration/jerk.
18. Payload profile gain values.
19. Target-loss timing thresholds.
20. Desired search pattern/range.
21. Camera intrinsics and camera-to-gimbal extrinsics.
22. Whether the base installation orientation is needed for v1 operation or only commissioning/telemetry.
23. Real measured camera/inference/control latency.

These should be configuration, not compile-time assumptions.

---

# 59. Architectural decisions frozen by this document

Unless later profiling demonstrates a specific reason to change them:

1. **IMX500 remains the v1 inference engine; no AI HAT is required for the initial architecture.**
2. **`controld` is the sole motor/CAN authority.**
3. **The critical control path is C++ and the vision integration is Python.**
4. **Host control starts at 200 Hz and is profiled.**
5. **CyberGear position mode is the initial motor mode.**
6. **The host streams a constrained trajectory; it does not send raw destination steps.**
7. **Mechanical coordinates are host-calibrated every boot.**
8. **Sensorless hard-stop homing is retained but redesigned as a low-energy multi-stage state machine.**
9. **Homing supports axis dependency and multiple passes at different safe angles.**
10. **Soft limits are protected by braking reachability, not position clipping alone.**
11. **Velocity, acceleration, and jerk are constrained.**
12. **Camera observations are time-aligned using frame timestamps and motor history.**
13. **Tracking is based on an estimated/predicted line of sight, not raw pixel error.**
14. **Base tilt is represented by a separate world-to-base transform.**
15. **Visual installation calibration is supported without an IMU.**
16. **A future IMU enters through an installation-pose provider abstraction.**
17. **No target -> short prediction -> controlled hold; search is configurable.**
18. **Web/video are lower priority than motor/control timing.**
19. **Payload change is verified and derated; aggressive auto-PID tuning is not performed at every boot.**
20. **Commanded shutdown parks pitch near a safe mechanical side before disabling.**

---

# 60. References and implementation notes

The architecture was informed by the following project/hardware documentation and the existing proof of concept:

1. Existing OpenAutoTurret POC:
   - `https://github.com/eamars/OpenAutoTurret/blob/main/Firmware/turret.py`
   - `https://github.com/eamars/OpenAutoTurret/blob/main/Firmware/cybergear_motor_controller.py`

2. Raspberry Pi AI Camera documentation:
   - `https://www.raspberrypi.com/documentation/accessories/ai-camera.html`

3. Raspberry Pi Picamera2 manual (`SensorTimestamp` and camera metadata):
   - `https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf`

4. Linux kernel SocketCAN documentation:
   - `https://docs.kernel.org/networking/can.html`
   - equivalent kernel documentation mirrors may resolve to the current version.

5. Xiaomi CyberGear protocol/manual translations and independent driver references:
   - `https://github.com/belovictor/cybergear-docs`
   - `https://github.com/grrodre/cybergear`

CyberGear protocol behavior, parameter units, timeout behavior, and firmware-specific edge cases should be validated against the exact installed motor firmware before being treated as safety guarantees.

---

# Appendix A — Suggested core state enums

```text
SystemState:
    BOOT
    CAN_INIT
    SELF_TEST
    UNHOMED
    HOMING
    CALIBRATING
    READY_HOLD
    TRACKING
    COASTING
    SEARCH
    BRAKING
    PARKING
    PARKED
    FAULT

AxisHomeState:
    UNKNOWN
    APPROACH_COARSE
    CONTACT_COARSE
    BACKOFF
    SETTLE
    APPROACH_FINE
    CONTACT_FINE
    VERIFY_REPEATABILITY
    COMPLETE
    FAILED

TargetState:
    NONE
    CANDIDATE
    TRACKED
    COASTING
    LOST

SafetyAction:
    ALLOW
    DERATE
    BRAKE
    HOLD
    FAULT_STOP
    DISABLE
```

---

# Appendix B — Suggested configuration ownership

| Parameter | Set by |
|---|---|
| Motor IDs | hardware configuration |
| Axis direction signs | commissioning |
| Mechanical expected travel | mechanical commissioning |
| Homing current/speed | homing characterization |
| Soft margins | safety/braking characterization |
| v/a/j limits | payload profiling |
| CyberGear gains | payload tuning |
| Camera intrinsics | camera calibration |
| Camera extrinsics | gimbal calibration |
| Installation tilt | visual calibration / future IMU |
| Search limits | application configuration |
| Target class/thresholds | application configuration |
| Web port | deployment configuration |

---

# Appendix C — Design rule for future sensors

Future sensors must produce a timestamped observation through a standard measurement interface rather than writing directly to the motor controller.

Concept:

```text
SensorMeasurement {
    sensor_type
    measurement_timestamp
    observation
    covariance
    validity
}
```

Possible future adapters:

```text
IMX500VisionProvider
RadarBearingRangeProvider
UltrasonicRangeProvider
ImuBaseOrientationProvider
```

The estimator decides how observations are fused.

This preserves the same safety/trajectory/motor architecture regardless of sensor expansion.

---

# Appendix D — Recommended first implementation milestone

The first implementation agent should **not start with AI tracking**.

The highest-value milestone is:

```text
boot
 -> discover both motors
 -> robust sensorless multi-axis homing
 -> establish logical coordinates
 -> execute safe jerk-limited moves
 -> demonstrate stopping envelope
 -> park safely
 -> log timing/feedback
```

Only after this is reliable should the camera be permitted to generate motion targets.

This sequencing minimizes debugging ambiguity: vision errors and motor/safety errors are separated until the motion foundation is trustworthy.
