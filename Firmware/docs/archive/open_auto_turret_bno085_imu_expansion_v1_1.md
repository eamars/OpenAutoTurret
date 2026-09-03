# OpenAutoTurret BNO085 IMU Expansion
## Architecture and Control Design Addendum to v1

**Document status:** Implementation handover / v1 expansion  
**Version:** 1.1-IMU  
**Date:** 2026-09-01  
**Depends on:** `open_auto_turret_software_control_architecture_v1.md`  
**Assumption:** The v1 architecture is fully implemented and working.  
**Target IMU:** CEVA/Hillcrest BNO085 / BNO08X family  
**Scope:** Non-weaponized pan/tilt camera and sensor tracking platform

---

# 1. Purpose

This document defines the BNO085 IMU expansion for the existing OpenAutoTurret v1 architecture.

It does **not** replace the v1 architecture. All existing v1 principles remain authoritative unless this document explicitly extends them.

The expansion adds four requested capabilities:

1. **Automatic tare/reference recovery when the IMU orientation does not make sense.**
2. **Useful operation without requiring a formal IMU calibration procedure.**
3. **Compensation for a tilted turret installation so a commanded yaw sweep can maintain constant world/gravity elevation by automatically adjusting pitch.**
4. **Independent secondary verification of yaw and pitch motion using the IMU against the CyberGear encoder/kinematic prediction.**

The central design principle is:

> The IMU is an independent orientation and motion observer, not the primary joint-position sensor.

CyberGear joint encoders and the existing v1 mechanical homing remain authoritative for:

- joint position;
- mechanical soft limits;
- hard-stop avoidance;
- joint-space braking;
- shutdown/park;
- boot position validity.

The BNO085 adds:

- gravity direction;
- installation tilt;
- short-timescale angular motion;
- independent motion verification;
- optional gravity/world-frame reference.

If the BNO085 is absent, unhealthy, untrusted, or temporarily unavailable, the turret shall fall back to v1 behavior rather than lose basic operation.

---

# 2. Mandatory mounting assumption

A single BNO085 can satisfy all four requested functions only if it is mounted on the **moving pitch/camera stage**.

Recommended physical placement:

```text
STATIC BASE
   |
   +-- yaw motor / yaw joint
          |
          +-- YAW FRAME
                 |
                 +-- pitch motor / pitch joint
                        |
                        +-- PITCH / PAYLOAD FRAME
                               |
                               +-- camera
                               |
                               +-- BNO085   <--- recommended
```

The BNO085 shall be:

- rigidly attached to the pitch/payload frame;
- mechanically coupled to the same structure as the camera as directly as practical;
- mounted away from flexible panels if possible;
- mounted such that local vibration is representative of payload motion rather than a loose bracket.

## 2.1 Why this mounting is required

If the BNO085 is mounted on the static base:

- it can measure base tilt;
- it cannot independently observe yaw rotation;
- it cannot independently observe pitch rotation.

If mounted on the yaw frame before the pitch joint:

- it can observe yaw;
- it cannot observe pitch.

If mounted on the pitch/camera stage:

- it observes yaw motion;
- it observes pitch motion;
- it observes gravity;
- it can validate the complete camera-stage angular motion.

## 2.2 Optional future second IMU

A second IMU on the static base could later provide:

- direct base motion/tilt change detection;
- separation of base vibration from gimbal motion;
- stronger inertial diagnostics.

It is not required by this v1.1 design.

---

# 3. Relationship to v1 architecture

The v1 architecture already defined:

```text
W: world / gravity frame
B: fixed turret base frame
Y: yaw frame
P: pitch frame
C: camera frame
```

This expansion adds:

```text
S: BNO085 reported sensor frame
G: gravity-aligned frame when heading is unknown
```

`S` deliberately means **reported frame**, not necessarily the physical PCB axes.

This distinction is important because the BNO085 may have:

- an unknown previous persistent tare;
- a runtime tare;
- a reorientation quaternion;
- imperfect physical mounting.

The v1.1 host estimates a transform that absorbs all of those effects.

The v1 transform:

```text
R_P_C
```

continues to describe camera mounting.

The IMU expansion adds:

```text
R_P_S
```

which maps vectors expressed in the BNO085's current reported frame into the physical pitch frame.

This transform includes:

- physical BNO085 mounting error;
- axis permutation/sign;
- any effective SH-2 output reorientation/tare that changes the reported axes.

Therefore an old or imperfect BNO085 tare does not automatically make the IMU unusable.

---

# 4. Important distinction: calibration, tare, and alignment

These must be treated as different concepts.

## 4.1 Sensor calibration

BNO085 internal calibration concerns biases and sensor fusion quality, including:

- accelerometer;
- gyroscope;
- magnetometer.

Formal calibration can improve accuracy.

It is **not required for the turret to boot or track**.

The system uses quality checks and confidence weighting instead of requiring:

```text
calibration_status == HIGH
```

before all IMU features become available.

## 4.2 BNO085 tare

A BNO085 tare changes the reported orientation/reference frame.

It can be:

- runtime;
- persistent, depending on SH-2 operation and report type.

The turret shall not assume the BNO085 tare is correct at boot.

## 4.3 Host alignment / host tare

The host maintains:

```text
R_P_S
```

and an optional orientation-reference alignment.

This is the primary mechanism used to make IMU output useful.

The host can recover from:

- a wrong device tare;
- an unknown device tare;
- a slightly crooked physical IMU installation.

This document uses the phrase **host auto-tare** for this process.

## 4.4 Policy

Default policy:

```text
HOST AUTO-TARE / ALIGNMENT
        first

BNO085 RUNTIME TARE
        only if required

BNO085 PERSIST TARE
        never automatic
```

This minimizes the chance of a bad automatically-generated tare being written to nonvolatile storage and becoming a new persistent failure mode.

---

# 5. Why Game Rotation Vector should be the primary reference

The turret does not require magnetic North for its requested IMU functions.

In fact, the motors, steel structure, wiring, payload, and environment can make magnetometer-based heading less trustworthy.

Therefore the default fusion strategy should avoid dependence on magnetic heading.

Recommended BNO085 reports:

| Report | Purpose | Initial requested rate |
|---|---|---:|
| Gyro-Integrated Rotation Vector `0x2A` | High-rate orientation + angular velocity | 200 Hz |
| Game Rotation Vector `0x08` or ARVR-Stabilized Game RV `0x29` | Low-drift 6-axis orientation reference | 100 Hz |
| Gravity vector | Installation tilt / sanity | 50–100 Hz |
| Accelerometer / linear acceleration | Optional diagnostics | 50–100 Hz |
| Rotation Vector `0x05` | Optional magnetic-heading diagnostics only | off by default |

The Gyro-Integrated Rotation Vector can use the Game Rotation Vector as its slower reference.

This gives:

- high-rate angular motion;
- no requirement for a valid magnetic heading;
- less sensitivity to motor magnetic fields;
- useful pitch/roll/gravity behavior.

The exact achievable report rates must be measured on the chosen Linux driver/transport. The v1 control loop remains 200 Hz even if the IMU report rate is lower.

---

# 6. Transport architecture

The BNO085 transport is configurable.

Supported architecture choices:

```text
SPI
I2C
UART
```

The rest of the control stack must be transport-independent.

## 6.1 Recommended preference

If wiring and driver support allow it:

```text
1. dedicated SPI controller
2. I2C with validated Raspberry Pi clock-stretching behavior
3. UART if the selected board/driver makes it preferable
```

The existing MCP2515 already uses `spi0.0`, so a BNO085 SPI implementation should use a separate available SPI controller/chip-select rather than sharing an unsafe configuration by accident.

## 6.2 Raspberry Pi I2C note

BNO085 is known to use I2C clock stretching, and Raspberry Pi hardware I2C has historically had clock-stretching interoperability issues.

If I2C is used:

- make transport health visible;
- detect read/packet timeout;
- automatically restart the IMU service on persistent failure;
- verify actual report rate and packet loss;
- test the known 400 kHz workaround where appropriate for the specific Pi/board/driver;
- never allow a stalled I2C transaction to block the main motor-control loop.

## 6.3 No blocking sensor I/O in `controld`

The BNO085 must not be read synchronously from the 200 Hz control thread.

Preferred architecture:

```text
BNO085
  |
  v
imud
  |
  | latest-value IMU state
  v
controld
```

---

# 7. New process: `imud`

Add:

```text
turret-imu.service
```

Recommended implementation:

- C++ if using a native SH-2/SHTP implementation;
- alternatively a robust standalone process in another language if it meets timing requirements.

The important requirement is process isolation from the critical motor loop.

## 7.1 `imud` responsibilities

`imud` owns the BNO085 transport.

It shall:

- initialize/reset the BNO085;
- identify product/version where available;
- configure requested reports;
- decode SHTP/SH-2 reports;
- recover/report sensor-hub reset;
- maintain sensor timestamps;
- map the sensor timebase to host monotonic time;
- detect dropped/stale packets;
- normalize quaternions;
- expose raw report status;
- expose high-rate angular velocity;
- expose gravity;
- accept explicit tare/reorientation requests from `controld`;
- never issue motor commands.

## 7.2 `imud` output

Conceptual message:

```text
ImuSample {
    uint64 host_timestamp_ns
    uint64 sensor_timestamp_ns
    uint64 sample_sequence
    uint32 reset_generation

    bool orientation_valid
    quaternion q_F_S

    bool angular_velocity_valid
    vec3 omega_S_rad_s

    bool gravity_valid
    vec3 gravity_S_m_s2

    optional vec3 accel_S_m_s2
    optional vec3 linear_accel_S_m_s2

    uint8 game_rv_status
    uint8 gravity_status

    bool packet_timing_valid
    bool transport_healthy
}
```

`F` means the current BNO085 fusion reference frame.

The host should not attach physical meaning such as “North” to `F` unless a specific magnetically referenced mode has been deliberately enabled and validated.

---

# 8. IPC

Use latest-value semantics.

At 100–200 Hz, either:

- Unix-domain `SOCK_SEQPACKET`; or
- shared-memory seqlock snapshot

is sufficient.

The main control loop reads the newest complete immutable IMU snapshot without blocking.

No unbounded queue of old IMU reports may accumulate between `imud` and `controld`.

For offline diagnostics, `imud` may separately feed a logger.

---

# 9. IMU health model

Add:

```text
ImuHealthState:
    ABSENT
    STARTING
    RAW_VALID
    ALIGNING
    VALID
    DEGRADED
    SUSPECT_REFERENCE
    RETARING
    FAILED
```

The IMU health state is distinct from the overall turret state.

The turret can be:

```text
READY_HOLD + IMU_DEGRADED
TRACKING + IMU_VALID
```

etc.

## 9.1 Raw sanity checks

Before orientation data are used:

### Quaternion norm

Require approximately:

```text
||q|| ~= 1
```

Reject/renormalize only inside a configured tolerance.

A large norm error is a transport/decoding failure, not a calibration problem.

### Finite values

Reject:

- NaN;
- infinity;
- impossible fixed-point decode.

### Report age

Require fresh reports.

Example initial thresholds:

```text
warning_age_ms = 30
invalid_age_ms = 100
```

for a nominal 100–200 Hz stream.

These are tunable.

### Gravity magnitude

When gravity report is valid:

```text
||g|| approximately 9.81 m/s^2
```

Use a generous plausibility band initially.

Example:

```text
7.5 < ||g|| < 12.0 m/s^2
```

The exact bounds should be tuned from real hardware.

### Angular-rate plausibility

Reject values significantly beyond physically possible turret rates plus margin.

---

# 10. BNO085 report accuracy is advisory, not a hard gate

SH-2 common report status encodes:

```text
0 = unreliable
1 = accuracy low
2 = accuracy medium
3 = accuracy high
```

The system shall record this status.

However:

```text
status < 3
```

does not automatically disable the IMU.

Instead, combine:

- BNO085 report status;
- transport health;
- gravity plausibility;
- encoder/IMU consistency;
- stationarity consistency;
- orientation continuity.

For example:

```text
status = low
but
gravity stable
and
gyro matches encoders
```

may still be sufficient for:

- base tilt;
- secondary motion verification with wider thresholds.

A formal manual calibration is therefore not mandatory.

---

# 11. IMU reference manager

Add to `controld`:

```text
ImuReferenceManager
```

Responsibilities:

- estimate `R_P_S`;
- identify invalid/stale reference;
- perform host auto-tare;
- request optional BNO085 runtime tare;
- manage reference-generation IDs;
- ensure all downstream consumers know when the IMU frame mapping changes.

Conceptual state:

```text
ImuReferenceState {
    bool valid
    uint32 reference_generation

    R_P_S
    covariance

    alignment_source
    fit_error

    bool device_tare_known
    bool device_tare_persisted_unknown
}
```

---

# 12. Critical design feature: absorb unknown device tare into `R_P_S`

Suppose the BNO085 board is physically mounted 4 degrees crooked.

Suppose it also boots with a previously saved 9-degree SH-2 reorientation.

Do not try to reason about those separately.

Treat the BNO085 output frame `S` as whatever frame it currently reports.

Then estimate:

```text
R_P_S
```

from known turret motion.

This one transform absorbs:

```text
physical mounting rotation
+
axis permutation/sign
+
existing BNO085 output reorientation/tare
```

as long as the BNO output is internally consistent.

That is why manual calibration is not strictly required.

---

# 13. Automatic IMU alignment during existing v1 homing

The best opportunity to estimate the IMU mounting/reference transform is the existing controlled mechanical homing motion.

No separate calibration routine needs to be mandatory.

During safe portions of the v1 homing sequence:

- only yaw moves;
- or only pitch moves;
- joint velocity is known from the CyberGear encoder;
- BNO085 angular velocity is simultaneously observed.

Exclude samples:

- during actual hard-stop contact;
- during large impact/vibration;
- when joint velocity is below useful threshold;
- during reversal transient if necessary.

## 13.1 Kinematic angular velocity prediction

For each sample:

```text
q = [q_yaw, q_pitch]
qdot = [qdot_yaw, qdot_pitch]
```

The existing turret kinematics can predict the angular velocity of the physical pitch stage expressed in `P`:

```text
omega_P_pred =
    J_omega_P(q) * qdot
```

The BNO085 measures:

```text
omega_S_meas
```

We seek:

```text
omega_P_pred ~= R_P_S * omega_S_meas
```

## 13.2 Solve `R_P_S`

Collect many pairs:

```text
(omega_S_meas_i, omega_P_pred_i)
```

and solve the rotation that minimizes:

```text
sum_i w_i *
|| R_P_S * omega_S_meas_i - omega_P_pred_i ||^2
```

Use a Wahba/Kabsch/SVD rotation solve with determinant constrained to `+1`.

Weight samples by:

- angular speed;
- BNO report health;
- distance from hard-stop contact;
- absence of acceleration transient.

## 13.3 Excitation requirement

A single rotational axis cannot fully determine a 3D frame.

Both yaw and pitch motion are required.

The normal v1 homing process already provides both.

Therefore, after one complete valid homing cycle, the system can usually have:

```text
IMU alignment valid
```

without a separate IMU calibration action.

## 13.4 Fit validation

Require:

- sufficient number of yaw samples;
- sufficient number of pitch samples;
- adequate angular excitation;
- low vector fit residual;
- predicted axes not degenerate;
- resulting matrix close to orthonormal;
- determinant approximately +1.

Store:

```text
alignment_rms_rad_s
alignment_peak_rad_s
sample_count_yaw
sample_count_pitch
```

If fit is poor, do not silently accept it.

---

# 14. Optional boot alignment without a full new homing sweep

If the v1 boot homing is already always performed, use it.

If some future mode reuses a valid mechanical home without sweeping both axes, the stored `R_P_S` can be loaded provisionally.

Then run a small safe verification motion:

```text
small yaw dither
small pitch dither
```

in a central safe region.

If residuals pass, reuse stored alignment.

If not, re-estimate.

---

# 15. Host auto-tare definition

The preferred auto-tare does **not** immediately modify BNO085 flash or persistent device orientation.

Host auto-tare means:

1. Declare the current IMU reference suspect.
2. Continue collecting raw orientation/gyro reports.
3. Use encoder-known motion to re-estimate `R_P_S`.
4. Establish a new orientation-reference alignment generation.
5. Validate against a second motion segment.
6. Atomically commit the new host transform.

This procedure is:

- reversible;
- measurable;
- independent of previous BNO085 tare state;
- robust to imperfect physical installation.

---

# 16. What means “measurement does not make sense”

Auto-tare/reference recovery may be triggered by any persistent combination of the following.

## 16.1 Impossible raw report

Examples:

- invalid quaternion norm;
- non-finite value;
- transport framing errors;
- stale stream.

This is **not** a tare problem.

Action:

```text
restart/reinitialize IMU
```

before attempting tare.

## 16.2 Orientation discontinuity unsupported by gyro

Example:

```text
orientation jumps 25 degrees
but
integrated gyro says 0.5 degree
```

Action:

```text
SUSPECT_REFERENCE
```

Possible causes:

- BNO sensor-hub reset;
- fusion reinitialization;
- device tare changed;
- bad packet decode.

## 16.3 Encoder/gyro axis disagreement

Example:

```text
yaw encoder = +20 deg/s
predicted IMU angular motion = known vector
measured IMU = mostly another axis
```

Persistent mismatch can indicate:

- bad `R_P_S`;
- device tare/reorientation changed;
- IMU moved physically;
- loose mount;
- wrong axis/sign configuration.

## 16.4 Base tilt impossible relative to recent history

The static turret base should not suddenly change by many degrees.

If IMU-derived gravity produces:

```text
base tilt jump > configured threshold
```

without:

- base movement being expected;
- corresponding mechanical/visual evidence;

mark the reference suspect.

Do not immediately move pitch based on the new tilt.

## 16.5 Gravity disagreement after known motion

When the turret returns to a known pose, the base gravity estimate should agree with the previously estimated base gravity within tolerance.

Persistent disagreement triggers reference recovery.

---

# 17. Auto-tare/recovery escalation

Use escalation rather than one blind action.

```text
REFERENCE SUSPECT
      |
      v
freeze IMU-derived control correction
      |
      v
raw sensor health check
      |
      +--bad--> reinitialize BNO085
      |
      v
attempt host re-alignment from encoder/gyro
      |
      +--valid--> VALID
      |
      v
optional runtime device tare
      |
      v
invalidate old R_P_S
      |
      v
re-run host alignment
      |
      +--valid--> VALID
      |
      v
IMU DEGRADED / FALL BACK TO v1
```

---

# 18. Optional actual BNO085 runtime tare

If host re-alignment cannot obtain an internally consistent reference and the sensor is otherwise healthy, `controld` may request an SH-2 runtime tare through `imud`.

Conditions:

- turret is mechanically homed;
- current pose is known;
- axes are stationary;
- not near a mechanically dangerous position;
- no hard-stop contact occurring;
- report stream is healthy;
- tare operation is explicitly enabled in config.

Recommended basis:

```text
Game Rotation Vector
```

rather than magnetically referenced Rotation Vector.

Recommended action:

```text
all-axis runtime tare
```

when recovering a clearly incorrect orientation frame.

After device tare:

```text
R_P_S = INVALID
```

and the host must re-estimate the transform.

## 18.1 Never automatically persist tare

Do not automatically issue Persist Tare.

Reason:

- the device may have been tared in a temporary installation;
- a poor automatic result should not survive reboot;
- Game Rotation Vector does not provide an absolute heading;
- host alignment already handles mounting/reference corrections.

A developer may explicitly choose to persist a validated device tare later, but it is not required by this architecture.

---

# 19. Device-reset detection

The BNO085/SH-2 can reset independently.

`imud` shall detect:

- unsolicited sensor-hub initialization response where supported;
- transport reset;
- product/reset sequence restart;
- report sequence discontinuity consistent with reset.

Increment:

```text
reset_generation
```

After any IMU reset:

```text
orientation reference = suspect
host alignment = provisional
tilt correction = frozen
motion verification = degraded
```

until revalidated.

If the reported frame after reset is demonstrably unchanged, stored `R_P_S` may be reused after a short verification.

---

# 20. Gravity-based base tilt

The base is physically static but may be installed tilted.

The IMU is on the moving pitch stage.

Once `R_P_S` is known, the gravity vector can be transformed back through the joint kinematics into the base frame.

At time `t`:

```text
gravity_S
    |
    v
gravity_P = R_P_S * gravity_S
    |
    v
gravity_B =
    R_B_P(q_yaw(t), q_pitch(t))
    * gravity_P
```

Normalize:

```text
down_B = normalize(gravity_B)
up_B   = -down_B
```

The exact sign must be verified against the BNO085 gravity convention and unit-tested with a known physical pose.

## 20.1 Key benefit

This computation does **not** require magnetic heading.

The turret only needs gravity direction to know what “horizontal” means.

---

# 21. Gravity frame `G`

Introduce a gravity-aligned frame:

```text
G
```

where:

- `+Z_G` is vertical/up;
- yaw heading is arbitrary unless another reference exists.

This is intentionally weaker than a full world frame.

The system can know:

```text
level
up
down
elevation
horizontal plane
```

without knowing:

```text
true North
absolute compass azimuth
```

This is sufficient for horizontal yaw compensation.

## 21.1 Relationship to v1 `W`

If v1 visual installation calibration provides a full world transform:

```text
R_W_B
```

then:

- preserve visual/world yaw heading;
- allow IMU gravity to continuously validate/correct roll/pitch.

If no full world calibration exists:

- expose gravity orientation;
- mark `heading_valid = false`.

---

# 22. Updated orientation data structure

Extend the v1 `BaseOrientation`.

Recommended:

```text
BaseOrientation {
    uint64 timestamp_ns

    vec3 up_B

    optional R_W_B

    bool gravity_valid
    bool heading_valid

    covariance gravity_covariance
    optional covariance orientation_covariance

    enum source:
        FIXED_CONFIG
        VISUAL
        IMU_GRAVITY
        VISUAL_PLUS_IMU
        NONE

    bool valid
}
```

This avoids forcing an arbitrary yaw heading into `R_W_B` when the IMU only knows gravity.

---

# 23. When to update installation tilt

The physical base is expected to be static.

Therefore do **not** continuously chase every IMU gravity estimate at high bandwidth.

Update the base gravity estimate primarily while the turret is quasi-static.

Stationarity conditions can include:

```text
abs(qdot_yaw)   < threshold
abs(qdot_pitch) < threshold
abs(omega_imu)  < threshold
linear_accel small
```

Then collect a time window and robustly average.

Example:

```text
0.5–2 seconds
```

depending on noise.

## 23.1 During motion

During normal tracking:

- continue to calculate an instantaneous gravity estimate for diagnostics;
- do not rapidly modify the canonical base tilt;
- allow only a very slow correction if confidence is high.

This prevents turret acceleration/vibration from becoming an apparent moving “gravity” direction.

---

# 24. Base moved detection

Even though installation is intended to be static, the IMU can detect if someone physically moves the whole station.

If, during a verified stationary joint state:

```text
angle(up_B_new, up_B_stored) > base_move_threshold
```

for a configured dwell:

```text
BASE_ORIENTATION_CHANGED
```

Then:

1. freeze world-level compensation;
2. controlled hold;
3. re-estimate gravity;
4. validate the new base orientation;
5. update `BaseOrientation`;
6. resume only when safe.

This is an additional benefit of the IMU.

---

# 25. Horizontal yaw compensation requirement

The requested behavior is:

> If the base is tilted, a yaw motion should remain horizontal in the gravity/world frame by changing pitch automatically.

This cannot be implemented correctly as:

```text
pitch_compensation = base_roll * sin(yaw)
```

with an ad-hoc formula.

Use the existing v1 kinematics.

---

# 26. World-level pointing constraint

Let:

```text
d_B(q_yaw, q_pitch)
```

be the camera optical forward unit vector expressed in the base frame.

Let:

```text
up_B
```

be the IMU-estimated world-up vector expressed in the base frame.

The elevation of the camera optical axis is:

```text
elevation =
asin( dot(up_B, d_B) )
```

For a horizontal/constant-elevation yaw operation, maintain:

```text
dot(up_B, d_B(q_yaw, q_pitch))
    =
sin(elevation_reference)
```

For true horizontal pointing:

```text
elevation_reference = 0
```

For “hold current elevation while yawing”:

```text
elevation_reference =
asin(dot(up_B, d_B(current_q)))
```

---

# 27. Pitch compensation solver

For every desired yaw target:

```text
q_yaw_des
```

solve one scalar unknown:

```text
q_pitch_des
```

such that:

```text
f(q_pitch) =
dot(
    up_B,
    d_B(q_yaw_des, q_pitch)
)
- sin(elevation_reference)
= 0
```

Use:

- bounded Newton iteration if derivative is reliable;
- otherwise bisection/Brent method over the allowed pitch interval.

Use previous pitch solution as the initial seed for continuity.

The solution runs in the reference-generation layer before the v1 trajectory generator.

---

# 28. Feasibility

A tilted 2-axis gimbal cannot necessarily satisfy a requested constant elevation over its full yaw range.

The required pitch may exceed:

- mechanical limits;
- soft limits;
- coupled collision envelope;
- braking margin.

The solver must return:

```text
VALID
NO_SOLUTION_WITHIN_LIMITS
NEAR_LIMIT
```

If no valid pitch exists:

- do not command the yaw target blindly;
- reduce the allowed yaw range;
- stop at the reachable boundary;
- expose the restriction to the search planner/UI.

---

# 29. Two-axis limitation: camera roll cannot be fully corrected

A yaw/pitch gimbal has only two rotational DOFs.

If the base is tilted, pitch compensation can keep the **camera optical axis at constant world elevation**, but it cannot generally keep the camera image horizon perfectly roll-level for all yaw angles.

That would require a roll axis or digital image rotation.

This expansion guarantees:

```text
level pointing trajectory
```

not:

```text
physical camera roll stabilization
```

unless geometry happens to make them coincide.

---

# 30. Integration into v1 Reference Manager

Add a constraint layer:

```text
requested behavior
    |
    v
ReferenceManager
    |
    +-- TRACK target
    +-- SEARCH
    +-- manual world-horizontal yaw
    +-- HOLD
    |
    v
WorldLevelConstraint
    |
    v
q_yaw_target, q_pitch_target
    |
    v
existing v1 SafetyEnvelope
    |
    v
existing jerk-limited trajectory
```

The IMU never bypasses the v1 safety envelope.

---

# 31. Which modes should use tilt compensation

## 31.1 Search/free roaming

Strongly recommended.

A world-horizontal scan becomes:

```text
vary yaw
solve pitch continuously
keep elevation constant
```

## 31.2 Developer/manual “yaw only”

Add two distinct commands:

```text
JOINT_YAW_ONLY
WORLD_HORIZONTAL_YAW
```

Do not make their semantics ambiguous.

## 31.3 Tracking

Pure image tracking does not fundamentally require base tilt because the existing v1 base-frame line-of-sight controller already tracks the target.

Optional enhancement:

- express target line-of-sight in gravity frame for motion-model interpretation;
- maintain world elevation when vision is temporarily lost.

The initial IMU expansion should avoid unnecessarily rewriting the mature v1 vision tracker.

## 31.4 Hold

Hold remains joint-position hold.

No need to continuously move just to compensate tilt unless the desired hold is explicitly world-referenced.

---

# 32. Secondary motion verification

The IMU provides an independent observation of payload angular motion.

The main verification compares:

```text
CyberGear encoder-derived motion
```

against:

```text
BNO085 inertial motion
```

This is independent of visual tracking.

---

# 33. Fast angular-velocity residual

Every control cycle with a fresh IMU sample:

Predict pitch-stage angular velocity from joint state:

```text
omega_P_enc =
    J_omega_P(q)
    * qdot_encoder
```

Transform IMU gyro:

```text
omega_P_imu =
    R_P_S
    * omega_S_imu
```

Residual:

```text
e_omega =
    omega_P_imu - omega_P_enc
```

Metrics:

```text
absolute_error =
    ||e_omega||

relative_error =
    ||e_omega|| /
    max(||omega_P_enc||, omega_floor)

direction_error =
    angle(omega_P_imu, omega_P_enc)
```

Do not rely on one metric only.

---

# 34. Axis-specific verification

For commanded yaw-dominant motion:

```text
expected_yaw_axis_P =
    kinematic_yaw_axis_expressed_in_P(q)
```

Project:

```text
omega_yaw_imu =
dot(omega_P_imu, expected_yaw_axis_P)
```

Compare to encoder yaw contribution.

For pitch:

```text
omega_pitch_imu =
dot(omega_P_imu, pitch_axis_P)
```

Compare to encoder pitch contribution.

This produces intuitive diagnostics such as:

```text
yaw encoder: +32.0 deg/s
yaw inertial: +30.7 deg/s
residual:      -1.3 deg/s
```

---

# 35. Orientation-delta verification

Gyro residual is fast but can be noisy.

Add a slower relative-orientation check over a short window.

For two times:

```text
t0
t1
```

IMU relative rotation:

```text
DeltaR_S_imu
```

Transform to the mechanical frame using `R_P_S`.

The encoder/kinematics produce:

```text
DeltaR_P_enc
```

Compute:

```text
R_error =
inverse(DeltaR_P_enc)
* DeltaR_P_imu
```

Convert to rotation-vector magnitude:

```text
theta_error =
|| log_SO3(R_error) ||
```

Because this is a **relative** rotation test, an unknown absolute BNO heading largely cancels.

This is particularly valuable when formal calibration is poor.

---

# 36. What secondary verification can detect

Potential detected failures include:

- loose BNO085 mount;
- loose camera/payload structure;
- mechanical coupling slip;
- encoder sign/configuration error;
- unexpected external movement;
- motor encoder says moving but payload is physically stuck;
- payload moves while encoder reports stationary;
- unexpected cross-axis motion;
- major structural flex;
- BNO reference discontinuity.

It does **not** replace:

- hard-stop soft limits;
- CyberGear motor fault detection;
- v1 CAN watchdog;
- mechanical safety design.

---

# 37. Motion mismatch classification

Add:

```text
MotionVerificationState:
    DISABLED
    LEARNING
    HEALTHY
    WARNING
    MISMATCH
    IMU_UNTRUSTED
```

Possible classifications:

## 37.1 Encoder moves, IMU does not

Potential causes:

- payload/mechanical decoupling;
- IMU frozen;
- IMU transport stale;
- motor rotor moving against a mechanical issue.

## 37.2 IMU moves, encoder does not

Potential causes:

- external disturbance;
- base motion;
- structural slip after encoder;
- bad IMU reference;
- loose IMU.

## 37.3 Magnitudes match but direction does not

Potential causes:

- bad `R_P_S`;
- tare/reference changed;
- axis mapping problem;
- mount shifted.

Prefer reference recovery before declaring mechanical fault.

---

# 38. Mismatch debounce

Never fault on one sample.

Use:

```text
warning threshold + dwell
fault threshold + dwell
```

Example conceptual values:

```text
warning:
    5 deg/s residual for 100 ms

fault:
    15 deg/s residual for 150 ms
```

These are illustrative only.

Real values must come from hardware logs.

At low speed, use absolute-angle delta residual because percentage error becomes unstable near zero.

---

# 39. Different thresholds by motion state

Verification thresholds should vary by mode.

## Normal smooth tracking

Tight thresholds.

## Aggressive acceleration/deceleration

Allow transient lag based on measured structural dynamics.

## Sensorless homing approach

Use wide verification thresholds.

## Hard-stop contact

Temporarily suppress normal motion-mismatch faulting because impact/vibration is expected.

Still check:

- IMU transport health;
- obviously impossible rotation.

## Park/hold

Use a low-motion disturbance detector.

---

# 40. Safety response to IMU mismatch

The IMU is secondary.

Therefore response depends on whether the failure appears to be:

```text
IMU failure
```

or:

```text
mechanical motion disagreement
```

## 40.1 IMU clearly unhealthy

Examples:

- stale data;
- invalid quaternion;
- sensor reset;
- failed transport.

Action:

```text
disable IMU-dependent correction
fall back to v1
raise warning
```

Tracking may continue if all v1 safety inputs are healthy.

## 40.2 IMU healthy but repeated mechanical mismatch

Action:

```text
DERATE
then controlled HOLD
```

if residual persists.

Do not immediately disable motor torque unless another v1 fault requires it.

## 40.3 Severe unexpected motion

If IMU detects substantial motion inconsistent with encoders while the turret should be stationary:

```text
controlled hold / fault
```

and require operator inspection or re-homing depending on severity.

---

# 41. Updated Safety Supervisor inputs

Extend v1 `SafetySupervisor` inputs:

```text
imu_health
imu_reference_valid
base_orientation_valid
motion_verification_state
motion_residual
orientation_delta_residual
base_move_detected
```

New outputs remain within existing v1 actions:

```text
ALLOW
DERATE
BRAKE
HOLD
FAULT_STOP
DISABLE
```

No new motor-bypass path is created.

---

# 42. New safety policy matrix

| Condition | IMU feature | Turret response |
|---|---|---|
| IMU absent at boot | unavailable | v1 operation |
| IMU stale | disabled | warn, v1 operation |
| IMU raw invalid | disabled | restart IMU, v1 operation |
| IMU low calibration but consistent | enabled with lower confidence | normal/derated |
| Reference/tare suspect | freeze tilt correction | host re-align |
| Runtime tare in progress | disabled | hold or restricted known motion |
| Base gravity estimate invalid | no horizontal compensation | use v1/static orientation |
| Mechanical mismatch warning | verification | derate |
| Mechanical mismatch persistent | verification | controlled hold |
| Base physically moved | tilt/world | hold and re-estimate |

---

# 43. Startup state-machine expansion

The v1 boot state becomes:

```text
POWER_ON
   |
   +--------------------------+
   |                          |
   v                          v
v1 CAN/MOTOR INIT         IMU START
   |                          |
   |                      RAW HEALTH
   |                          |
   +------------+-------------+
                |
                v
         V1 HOMING PLAN
                |
                +--> collect yaw/pitch IMU motion samples
                |
                v
       FIT R_P_S / HOST AUTO-TARE
                |
          +-----+------+
          |            |
        valid        invalid
          |            |
          |        optional runtime tare
          |            |
          |        re-fit / verify
          |            |
          +-----+------+
                |
                v
         ESTIMATE BASE GRAVITY
                |
                v
        IMU MOTION VERIFY TEST
                |
                v
       READY_HOLD / v1 fallback
```

The IMU must never prevent mechanical homing unless it has been explicitly configured as mandatory.

Default:

```text
imu.required = false
```

---

# 44. Homing integration

The v1 homing controller should publish:

```text
HomingMotionSegment {
    axis
    phase
    safe_for_imu_alignment
}
```

Examples:

```text
COARSE_TRAVEL -> yes, away from contact
FINE_APPROACH -> possibly yes
CONTACT -> no
BACKOFF -> yes
SETTLE -> stationary gravity sample
```

The IMU alignment collector only uses approved samples.

This prevents hard-stop vibration from corrupting `R_P_S`.

---

# 45. Alignment sample selection

Use a sample only when:

```text
imu sample fresh
AND
transport healthy
AND
abs(commanded joint speed) > minimum excitation
AND
abs(measured joint speed) > minimum excitation
AND
not in stop-contact window
AND
other axis movement within expected bound
```

Prefer multiple speed levels.

Reject outliers using:

- robust median/MAD;
- RANSAC if necessary;
- residual threshold after initial rotation solve.

---

# 46. Gravity sampling after homing

After both axes are referenced:

1. move to a safe known pose;
2. come to full stop;
3. wait for vibration to decay;
4. collect gravity samples;
5. transform each to base coordinates;
6. reject outliers;
7. average directions on the unit sphere;
8. compute covariance;
9. set `BaseOrientation.gravity_valid`.

This can take place automatically on every boot.

No external calibration fixture is required.

---

# 47. Calibration optionality

The system has three quality levels.

## Level A — no formal calibration

Requirements:

- BNO stream healthy;
- host `R_P_S` auto-fit valid;
- gravity stable;
- encoder/gyro consistency acceptable.

Capabilities:

- base tilt;
- horizontal yaw compensation;
- relative motion verification.

This is the default desired operating mode.

## Level B — BNO internal calibration improved

Benefits:

- better gyro bias;
- better gravity/orientation stability;
- tighter verification thresholds.

Capabilities otherwise unchanged.

## Level C — external visual + IMU calibration

Adds:

- high-confidence full world orientation;
- visual cross-check of gravity/world pose;
- best long-term consistency.

Not mandatory.

---

# 48. How to use BNO accuracy status

Suggested confidence weighting:

```text
status 3:
    full nominal weight

status 2:
    moderate-high weight

status 1:
    reduced weight
    wider residual thresholds

status 0:
    do not use as sole source for base tilt update
    gyro may still be observed diagnostically if internally consistent
```

Do not hard-code these without validating actual BNO report behavior.

The architecture should expose the weight mapping in config.

---

# 49. Magnetometer policy

Default:

```text
magnetic heading not used
```

Reasons:

- nearby CyberGear motors;
- steel;
- current-carrying wiring;
- installation environment;
- heading is not needed for level yaw motion.

If enabled later:

- use as advisory absolute heading;
- monitor heading-accuracy report;
- compare against visual/world reference where available;
- do not let magnetic heading jumps directly command motor motion.

---

# 50. Tilt compensation smoothing

The canonical:

```text
up_B
```

shall change slowly because the base is static.

Use a directional low-pass or complementary update, not direct Euler-angle filtering.

Concept:

```text
up_B_filtered =
normalize(
    (1 - alpha) * up_B_filtered
    + alpha * up_B_measurement
)
```

only while stationarity/quality conditions are satisfied.

For larger legitimate installation changes, use the base-moved recovery state rather than a slow multi-second incorrect transition while actively tracking.

---

# 51. Avoid Euler singularities internally

The IMU returns a quaternion.

Keep transformations as:

- quaternions;
- SO(3) matrices;
- rotation vectors.

Use roll/pitch/yaw only for:

- UI display;
- diagnostics;
- configuration where convenient.

Do not build the core compensation algorithm using chained Euler-angle additions.

---

# 52. Time synchronization

The v1 architecture already emphasized camera/motor timestamp alignment.

The IMU expansion must follow the same rule.

Every BNO measurement needs a host-monotonic timestamp representing the sensor sample time as accurately as the driver permits.

## 52.1 `imud` time model

Maintain:

```text
sensor_time -> host_monotonic_time
```

mapping.

Track:

- offset;
- drift if measurable;
- report transport delay;
- reset generation.

## 52.2 Why timing matters

At 100 deg/s:

```text
10 ms timing error = 1 degree motion error
```

which is large enough to create false encoder/IMU mismatch.

Do not compare:

```text
latest encoder state
```

with:

```text
old IMU sample
```

without time alignment.

---

# 53. Motor history reuse

Reuse v1 `MotorStateHistory`.

For IMU sample time:

```text
t_imu
```

interpolate:

```text
q(t_imu)
qdot(t_imu)
```

Then compute predicted IMU-stage motion at the same time.

This makes the secondary verifier structurally similar to the existing camera timestamp alignment.

---

# 54. IMU history

Retain approximately:

```text
0.5–2 seconds
```

of IMU samples in a fixed-size ring buffer.

Uses:

- orientation delta checks;
- reference discontinuity detection;
- alignment fitting;
- fault black-box log;
- time-offset profiling.

---

# 55. Updated control-loop pseudocode

The v1 loop becomes conceptually:

```text
loop every CONTROL_PERIOD:

    now = monotonic_time()

    axis = motor_state_snapshot()
    imu  = latest_imu_snapshot()

    update_v1_timing_health(now)

    # existing v1 checks
    check_motor_feedback(axis)
    process_latest_camera_measurement()

    # IMU health path
    imu_health.update(imu, now)

    if imu_health.raw_valid:

        aligned_motor_state =
            motor_history.interpolate(imu.timestamp)

        if aligned_motor_state.valid:

            imu_reference_manager.observe_motion(
                imu,
                aligned_motor_state
            )

            if imu_reference_manager.valid:

                motion_verifier.update(
                    imu,
                    aligned_motor_state,
                    R_P_S
                )

                base_orientation_estimator.observe(
                    imu,
                    aligned_motor_state,
                    R_P_S
                )

    if imu_reference_manager.reference_suspect:
        freeze_imu_control_contribution()
        schedule_reference_recovery()

    target_estimator.predict_to(
        now + actuation_horizon
    )

    requested_reference =
        reference_manager.compute(...)

    if requested_reference.requires_world_level_constraint:

        if base_orientation.gravity_valid:
            requested_reference =
                world_level_constraint.solve(
                    requested_reference,
                    up_B
                )
        else:
            requested_reference =
                fallback_policy(...)

    safe_reference =
        existing_v1_safety_envelope.constrain(...)

    trajectory =
        existing_v1_trajectory_generator.step(...)

    safety_supervisor.evaluate(
        v1_inputs,
        imu_health,
        motion_verification,
        base_orientation
    )

    apply_existing_v1_safety_override()

    can_tx(...)
    publish_nonblocking_telemetry()
```

---

# 56. Reference recovery must not occur inside the hard real-time calculation

The 200 Hz loop may:

- flag reference recovery;
- consume already-computed transforms;
- freeze IMU contribution.

Potentially expensive operations such as:

- SVD/Kabsch fit;
- multi-second data-window analysis;
- device tare command sequence;

should run in:

- a lower-priority calibration worker;
- or a bounded non-control thread.

Commit a new `R_P_S` atomically.

---

# 57. Tare/reference generation IDs

Every time the effective IMU reference changes:

```text
reference_generation++
```

Every `ImuSample`/derived state consumed by the controller should be associated with the relevant generation.

Do not compare orientation-delta samples across:

- BNO reset;
- device tare;
- host frame change.

Flush orientation verification windows on generation change.

---

# 58. Persistence

Store:

```yaml
imu:
  alignment:
    R_P_S: ...
    source: auto_homing_fit
    fit_rms: ...
    created_at: ...
    sensor_identity: ...
    mechanism_revision: ...
```

This is provisional at the next boot until verified.

Also store:

```yaml
installation:
  gravity:
    up_B: [...]
    covariance: [...]
    source: bno085
    created_at: ...
```

Do not automatically store a BNO085 device tare to flash.

---

# 59. Configuration expansion

Suggested addition to `turret.yaml`:

```yaml
imu:
  enabled: true
  required: false

  type: bno085

  mounting:
    frame: pitch_stage

  transport:
    type: spi   # spi | i2c | uart
    # transport-specific values under this node

  reports:
    gyro_integrated_rotation_vector_hz: 200
    game_rotation_vector_hz: 100
    gravity_hz: 100

  magnetometer_heading:
    enabled: false

  health:
    warning_age_ms: 30
    invalid_age_ms: 100
    quaternion_norm_tolerance: 0.05
    gravity_min_m_s2: 7.5
    gravity_max_m_s2: 12.0

  alignment:
    auto_from_homing: true
    minimum_yaw_samples: TBD
    minimum_pitch_samples: TBD
    minimum_joint_speed_deg_s: TBD
    max_fit_rms_deg_s: TBD
    verify_at_boot: true

  auto_tare:
    enabled: true
    host_realign_first: true
    allow_runtime_device_tare: true
    persist_device_tare_automatically: false
    orientation_jump_deg: TBD
    orientation_jump_dwell_ms: TBD

  base_orientation:
    estimate_from_gravity: true
    stationary_joint_speed_deg_s: TBD
    stationary_imu_speed_deg_s: TBD
    sample_window_ms: TBD
    base_move_threshold_deg: TBD

  motion_verification:
    enabled: true

    warning:
      angular_velocity_residual_deg_s: TBD
      orientation_delta_deg: TBD
      dwell_ms: TBD

    fault:
      angular_velocity_residual_deg_s: TBD
      orientation_delta_deg: TBD
      dwell_ms: TBD

  world_level_yaw:
    enabled: true
    fallback_if_imu_invalid: use_static_v1_pose
```

All `TBD` values are commissioning parameters.

---

# 60. BNO085 driver interface

Create a transport-independent C++ or IPC-facing interface:

```text
class ImuProvider {
public:
    virtual bool healthy() const = 0;
    virtual ImuSample latest() const = 0;

    virtual DeviceInfo deviceInfo() const = 0;

    virtual Result reinitialize() = 0;
    virtual Result runtimeTare(TareRequest) = 0;
};
```

Transport implementations:

```text
Bno085SpiProvider
Bno085I2cProvider
Bno085UartProvider
```

If `imud` is a process, these concepts live behind its command protocol rather than directly inside `controld`.

---

# 61. Runtime tare command interface

Concept:

```text
TareRequest {
    enum axes:
        ALL
        Z_ONLY

    enum basis:
        GAME_ROTATION_VECTOR
        ROTATION_VECTOR
        GEOMAGNETIC_ROTATION_VECTOR

    bool persist
}
```

Production policy rejects:

```text
persist = true
```

from automatic recovery code.

Only a developer-authorized explicit command may request persistence.

---

# 62. `ImuAlignmentEstimator`

Add:

```text
ImuAlignmentEstimator
```

Inputs:

```text
timestamped imu angular velocity
timestamped q/qdot
homing phase metadata
```

Outputs:

```text
R_P_S
covariance
fit error
sample coverage
valid
```

## 62.1 Algorithm

1. Collect motion vector pairs.
2. Normalize or retain magnitude depending on weighting scheme.
3. Remove near-zero samples.
4. Robustly reject outliers.
5. Form cross-covariance.
6. SVD/Kabsch solve.
7. Force proper rotation.
8. Recompute residuals.
9. Reject if quality thresholds fail.
10. Validate on held-out samples.

Use some samples for fitting and some for verification so the result is not only self-consistent on its training window.

---

# 63. `BaseOrientationEstimator`

Add:

```text
BaseOrientationEstimator
```

Inputs:

- `gravity_S`;
- `R_P_S`;
- timestamp-aligned v1 joint state;
- stationarity state;
- optional existing visual `R_W_B`.

Outputs:

```text
up_B
gravity_covariance
base_moved
optional updated R_W_B
```

## 63.1 Visual + IMU mode

If a visual world transform exists:

- preserve visual heading;
- use IMU gravity to validate/correct tilt slowly;
- flag large disagreement.

Do not silently rotate the world heading from magnetometer data.

---

# 64. `WorldLevelConstraint`

Add:

```text
WorldLevelConstraint
```

Inputs:

```text
desired yaw / desired horizontal motion
desired elevation
up_B
v1 turret kinematics
joint limits
collision envelope
```

Outputs:

```text
q_yaw_target
q_pitch_target
validity
margin_to_constraint_failure
```

This module must be pure/testable and contain no hardware I/O.

---

# 65. `MotionVerifier`

Add:

```text
MotionVerifier
```

Inputs:

- timestamped IMU orientation/gyro;
- timestamp-aligned joint position/velocity;
- `R_P_S`;
- kinematic Jacobian;
- current turret mode.

Outputs:

```text
MotionVerification {
    state

    omega_error_vector
    omega_error_magnitude
    yaw_rate_encoder
    yaw_rate_imu
    pitch_rate_encoder
    pitch_rate_imu

    orientation_delta_error

    confidence
    fault_reason
}
```

---

# 66. Telemetry expansion

Add dashboard panels.

## IMU device

- present;
- transport;
- report rates requested/actual;
- packet age;
- reset generation;
- dropped/error count;
- Game RV status;
- gravity status.

## IMU orientation

- quaternion;
- displayed roll/pitch/yaw for diagnostics;
- `R_P_S` validity;
- alignment fit RMS;
- host reference generation;
- device runtime tare state if known.

## Installation

- `up_B`;
- base roll/pitch display;
- heading valid/invalid;
- source:
  - visual;
  - IMU;
  - fused;
  - static;
- base-moved alarm.

## Motion verification

- yaw encoder rate vs IMU rate;
- pitch encoder rate vs IMU rate;
- gyro residual;
- orientation-delta residual;
- state;
- fault dwell.

## Level-yaw

- enabled;
- elevation target;
- compensated pitch target;
- solution margin;
- infeasible state.

---

# 67. Developer UI controls

Add:

```text
IMU enable/disable
Restart IMU
Run host re-alignment
Run verification dither
Runtime tare now
Clear/freeze host alignment
Re-estimate base gravity
Enable/disable world-level yaw compensation
```

Dangerous/persistent operations should require explicit confirmation.

Do not expose automatic Persist Tare as a one-click default action.

---

# 68. Logging expansion

High-rate log:

```text
imu_timestamp
imu_q
imu_omega
imu_gravity
imu_status
R_P_S generation

q_yaw
q_pitch
qdot_yaw
qdot_pitch

omega_encoder_pred
omega_imu_aligned
omega_residual

up_B
base_orientation_confidence

level_constraint_pitch
level_constraint_error
```

Event log:

```text
IMU_CONNECTED
IMU_RESET
IMU_STALE
IMU_REFERENCE_SUSPECT
HOST_AUTOTARE_STARTED
HOST_AUTOTARE_SUCCEEDED
HOST_AUTOTARE_FAILED
DEVICE_RUNTIME_TARE
IMU_ALIGNMENT_VALID
IMU_ALIGNMENT_REJECTED
BASE_TILT_UPDATED
BASE_MOVED
MOTION_MISMATCH_WARNING
MOTION_MISMATCH_FAULT
IMU_FALLBACK_TO_V1
```

---

# 69. Black-box fault data

On:

- motion mismatch;
- base moved;
- reference jump;
- IMU reset;

persist several seconds of:

- raw IMU;
- encoder state;
- motor commands;
- target trajectory;
- safety state.

This allows determining whether the discrepancy came from:

- IMU;
- mechanics;
- encoder;
- control timing.

---

# 70. Failover behavior

The IMU is optional by default.

## 70.1 IMU unavailable at startup

```text
continue v1 boot
```

Use:

- stored visual installation pose if available;
- otherwise base-frame operation.

World-horizontal yaw compensation is disabled unless a valid stored gravity/world pose exists.

## 70.2 IMU fails during tracking

Freeze:

```text
last trusted base tilt
```

Do not continuously use stale IMU data.

Then either:

- continue v1 target tracking;
- disable world-level free-roam compensation;
- warn developer.

## 70.3 IMU returns

Do not immediately trust it.

Sequence:

```text
RAW_VALID
-> verify stored R_P_S
-> gravity consistency
-> motion consistency
-> VALID
```

---

# 71. Interaction with v1 visual installation calibration

The IMU expansion does not remove the v1 visual calibration feature.

They are complementary.

## Visual calibration is strong at

- full orientation;
- potentially heading;
- camera/world geometry;
- one-time installation reference.

## IMU is strong at

- gravity;
- relative angular motion;
- dynamic verification;
- detecting physical change after installation.

## Recommended fusion

If both exist:

```text
visual:
    heading / full reference

IMU:
    gravity / roll-pitch validation

encoders:
    mechanical joints
```

No single source should be forced to perform all three roles.

---

# 72. Interaction with target prediction

The initial implementation should not directly insert IMU gyro into the visual target Kalman filter.

Reason:

- target LOS prediction already has a working v1 solution;
- camera observations are transformed with timestamped encoder state;
- the IMU primarily validates mechanics and supplies gravity.

Possible future enhancement:

- use IMU orientation in camera pose interpolation if encoder timestamps are insufficient;
- use gravity-frame target motion priors.

This is outside the first IMU expansion unless profiling demonstrates a benefit.

---

# 73. Interaction with homing sensorless stop detection

Do not replace v1 current/velocity-based hard-stop detection with IMU impact detection.

IMU can provide an auxiliary signal:

```text
impact/vibration detected
```

but it should not become the primary stop-contact criterion.

Reason:

- payload compliance varies;
- vibration varies;
- accelerometer response depends on mounting;
- deliberate stop contact is already detected by motor feedback.

IMU impact may be logged for homing characterization.

---

# 74. Interaction with braking envelope

Mechanical joint braking remains encoder-based.

IMU does not redefine:

```text
q_soft_min
q_soft_max
```

However, severe motion disagreement can force:

```text
DERATE
BRAKE
HOLD
```

through the existing safety supervisor.

---

# 75. Horizontal-yaw trajectory timing

Do not calculate pitch compensation only when a new yaw destination arrives.

At each 200 Hz reference update:

1. obtain current planned yaw state;
2. compute corresponding desired world elevation constraint;
3. solve pitch reference for that yaw;
4. feed yaw/pitch pair into the existing online trajectory layer.

This produces a smooth curved joint-space path that corresponds to a level world-space sweep.

---

# 76. Coupled trajectory issue

When pitch is used to compensate yaw, yaw and pitch become intentionally coupled.

The existing v1 per-axis trajectory constraints still apply, but a world-horizontal path can be limited by whichever axis reaches:

- velocity;
- acceleration;
- jerk;
- soft limit;

first.

Therefore the level-yaw planner should expose a path-speed scale:

```text
0.0 ... 1.0
```

such that the requested world yaw rate is reduced when pitch cannot keep up.

Do not let yaw outrun the required pitch compensation.

---

# 77. Feed-forward pitch velocity

For better coordination, compute the local derivative:

```text
dq_pitch / dq_yaw
```

along the level constraint.

Then:

```text
qdot_pitch_ff =
(dq_pitch / dq_yaw) *
qdot_yaw
```

The trajectory generator can use this as a desired coordinated velocity hint if its interface supports it.

This is optional for initial implementation.

Position targets at 200 Hz may already be sufficient.

---

# 78. Level-yaw acceptance criterion

During a test sweep, calculate world/gravity elevation:

```text
elevation(t) =
asin(dot(up_B, d_B(q(t))))
```

Acceptance should be based on:

```text
RMS elevation error
peak elevation error
```

rather than only encoder pitch error.

Example commissioning target:

```text
RMS < 0.5 deg
peak < 1.5 deg
```

These numbers are examples and should be adjusted to the actual mechanical accuracy and application.

---

# 79. Motion-verification acceptance criterion

Characterize normal residuals across:

- slow yaw;
- fast yaw;
- slow pitch;
- fast pitch;
- combined motion;
- acceleration;
- braking;
- different payloads.

Set warning/fault thresholds above measured normal envelopes.

Do not pick thresholds from theory only.

---

# 80. Payload dependence

IMU motion residuals will include structural dynamics.

A heavier/flexible payload may cause:

- phase lag;
- vibration;
- small elastic motion.

Therefore motion-verification profiles may be payload-specific, similar to v1 motor tuning profiles.

Extend payload profile:

```yaml
payload_profiles:
  heavy_camera:
    imu_verification:
      omega_warning_deg_s: ...
      omega_fault_deg_s: ...
      orientation_delta_warning_deg: ...
      orientation_delta_fault_deg: ...
```

---

# 81. Auto-tare acceptance criteria

Host auto-tare succeeds only if:

1. enough yaw excitation exists;
2. enough pitch excitation exists;
3. rotation solve is valid;
4. held-out residual passes;
5. gravity is plausible after transform;
6. subsequent small known motion validates direction and scale.

It must not succeed solely because the SVD returned a matrix.

---

# 82. Auto-tare hysteresis

Avoid repeated tare loops.

After a successful reference:

```text
minimum_time_before_re_tare
```

unless there is a severe reset/discontinuity.

Keep counters:

```text
auto_tare_attempts
auto_tare_failures
```

If repeated failures exceed a threshold:

```text
IMU_FAILED
```

and fall back to v1.

---

# 83. Prevent tare during unsafe movement

Device tare or alignment commit is forbidden while:

- tracking at high speed;
- braking near a limit;
- hard-stop contact;
- safe park sequence;
- CAN fault;
- motor feedback stale.

For host re-alignment from normal motion, data may be collected during movement, but the new transform should only be committed after validation.

---

# 84. Orientation jump handling

When an orientation quaternion changes abruptly:

1. compare the jump with integrated measured angular velocity;
2. if supported by gyro, treat as real motion;
3. if unsupported, classify as reference discontinuity;
4. flush delta-orientation windows;
5. freeze world-level compensation;
6. revalidate reference.

Never command an immediate pitch correction from an unexplained 20-degree quaternion jump.

---

# 85. Base-tilt jump handling

Likewise:

```text
new gravity implies base suddenly tilted
```

must not instantly cause the pitch motor to move.

Use:

```text
candidate base orientation
```

then verify while stationary.

Only commit after dwell/validation.

This prevents an IMU fusion transient from producing a large mechanical motion.

---

# 86. No direct IMU attitude servo in v1.1

Do **not** implement:

```text
pitch_motor_error =
imu_world_pitch_error
```

as a direct inner control loop.

The authoritative joint controller remains encoder-based.

The IMU generates a **reference correction**:

```text
world-level target
    -> desired joint angles
```

The existing motor servo then tracks those joint angles.

This preserves the safety properties of v1.

---

# 87. Why not fuse encoders and IMU into one joint EKF immediately

A full encoder/IMU EKF could eventually estimate:

- flex;
- bias;
- dynamic lag.

It is unnecessary for the first expansion.

The direct-drive CyberGear encoders already provide high-quality joint position.

The largest value from BNO085 is:

- gravity;
- reference validation;
- independent angular motion.

Therefore the v1.1 architecture uses **cross-checking**, not aggressive sensor fusion.

---

# 88. Repository expansion

Add:

```text
OpenAutoTurret/
|
+-- imu/
|   +-- imud/
|   |   +-- bno085_device.*
|   |   +-- shtp_transport.*
|   |   +-- sh2_reports.*
|   |   +-- time_sync.*
|   |   +-- imu_publisher.*
|   |
|   +-- tests/
|
+-- control/
|   +-- src/
|       +-- imu/
|       |   +-- imu_reference_manager.*
|       |   +-- imu_alignment_estimator.*
|       |   +-- imu_health.*
|       |   +-- base_orientation_estimator.*
|       |   +-- motion_verifier.*
|       |
|       +-- geometry/
|       |   +-- world_level_constraint.*
|       |
|       +-- control/
|           +-- reference_manager.*   # extended
|           +-- safety_envelope.*     # inputs extended
|
+-- tools/
    +-- imu_monitor/
    +-- imu_alignment_profiler/
    +-- level_yaw_test/
    +-- imu_encoder_replay/
```

---

# 89. Systemd expansion

Add:

```text
turret-imu.service
```

Recommended behavior:

- starts independently of network;
- restarts on crash;
- does not restart `turret-control.service` when it fails;
- `controld` handles IMU disconnect/reconnect dynamically.

Do not make:

```text
turret-control.service Requires=turret-imu.service
```

unless the deployment intentionally sets `imu.required=true`.

---

# 90. Unit tests

Required new tests:

## Quaternion/SO(3)

- normalization;
- inverse/composition;
- shortest-angle residual;
- log map;
- transform convention.

## Alignment estimator

Synthetic known `R_P_S` with:

- noise;
- outliers;
- wrong signs;
- different yaw/pitch paths.

Verify recovered transform.

## Gravity/base tilt

Generate known base tilt and joint pose.

Verify recovered:

```text
up_B
```

across many yaw/pitch positions.

## World-level constraint

For random base tilts and yaw values:

- solve pitch;
- verify elevation error;
- verify no solution when limits prevent it.

## Motion verifier

Inject:

- perfect match;
- gyro bias;
- encoder lag;
- frozen IMU;
- wrong axis;
- orientation jump.

Verify classifications.

## State-machine tests

- IMU absent;
- reset during tracking;
- wrong initial tare;
- runtime tare;
- re-alignment success/failure;
- repeated failure fallback.

---

# 91. Simulation tests

Extend the v1 plant simulator.

Simulate:

```text
base tilt
BNO mounting rotation
BNO old tare
gyro bias
orientation noise
packet delay
packet dropout
sensor reset
structural lag
encoder/physical decoupling
```

Use the same `ImuSample` interface as hardware.

This allows validating auto-tare logic without repeatedly manipulating the physical turret.

---

# 92. Hardware commissioning sequence

## Phase 1 — transport

- verify stable BNO085 communication;
- measure actual report rate;
- measure packet jitter;
- test sensor reset recovery.

## Phase 2 — raw orientation

- manually rotate BNO board;
- verify quaternion/gyro/gravity decode;
- verify units/signs.

## Phase 3 — mount on pitch stage

- perform normal v1 homing;
- collect alignment data;
- recover `R_P_S`;
- inspect fit.

## Phase 4 — repeatability

Power-cycle repeatedly with:

- no device tare change;
- deliberately altered runtime tare if practical.

Verify host auto-alignment recovers consistent mechanical behavior.

## Phase 5 — base tilt

Physically tilt station by known approximate angles.

Verify:

```text
up_B
```

changes correctly.

## Phase 6 — level yaw

Sweep yaw with base tilted.

Verify pitch automatically compensates and optical elevation stays nearly constant.

## Phase 7 — motion verifier

Introduce controlled discrepancies where safe:

- manually hold payload slightly;
- loosen only a test coupling fixture if available;
- simulate stale/frozen IMU in software.

Verify warning/hold logic.

---

# 93. Deliberate bad-tare test

This is an important acceptance test.

1. Put turret in a safe known pose.
2. Apply an intentionally incorrect BNO runtime tare/reorientation.
3. Restart IMU integration state.
4. Verify:
   - system marks reference suspect;
   - v1 mechanical control remains safe;
   - host auto-alignment recovers `R_P_S`;
   - base gravity becomes sensible;
   - motion verification passes;
   - no dangerous immediate pitch jump occurs.

Then test the optional runtime auto-tare escalation.

---

# 94. Poor-calibration test

Start the BNO085 with low reported accuracy.

Verify:

- system does not refuse to boot;
- host alignment still works if gyro data are consistent;
- gravity estimate becomes valid only after consistency checks;
- confidence is lower;
- residual thresholds are widened;
- UI shows low sensor status.

This directly verifies requirement 2.

---

# 95. Tilted installation test

Install base with both roll and pitch tilt.

Test:

```text
WORLD_HORIZONTAL_YAW
```

over the usable yaw range.

Verify:

- pitch changes as yaw changes;
- world elevation remains approximately constant;
- pitch limits are respected;
- infeasible yaw regions are detected;
- yaw slows if pitch compensation cannot keep up.

---

# 96. Secondary yaw verification test

Command yaw motion with pitch held by trajectory.

Compare:

```text
qdot_yaw_encoder
```

and:

```text
IMU-projected yaw rate
```

through:

- acceleration;
- constant speed;
- braking.

Characterize residual envelope.

---

# 97. Secondary pitch verification test

Repeat for pitch.

Because the BNO is on the pitch stage, pitch motion should be directly visible.

Characterize:

- gravity effects;
- payload dependence;
- structural oscillation.

---

# 98. Combined motion test

Command simultaneous yaw/pitch motion.

The full 3D angular velocity prediction:

```text
J_omega(q) qdot
```

must match IMU angular velocity after `R_P_S`.

Do not verify combined motion by simply adding scalar yaw and pitch magnitudes.

---

# 99. IMU reset during motion test

Software-trigger or physically reset the BNO085 during a safe slow trajectory.

Expected:

1. `imud` detects reset.
2. `controld` marks IMU reference suspect.
3. world-level correction freezes.
4. v1 trajectory remains safe.
5. IMU recovers/revalidates.
6. no position jump is commanded.

---

# 100. Acceptance metrics

## Transport

- actual report rate;
- p95/p99 inter-report time;
- stale events;
- reset recovery time.

## Alignment

- `R_P_S` fit RMS;
- held-out RMS;
- repeatability across boots;
- repeatability after bad runtime tare.

## Base tilt

- static gravity variation;
- estimated tilt repeatability;
- disagreement with visual/physical reference if available.

## Level yaw

- RMS world elevation error;
- peak elevation error;
- pitch compensation smoothness;
- no soft-limit violation.

## Motion verification

- normal residual distribution;
- detection latency for injected mismatch;
- false warning/fault rate.

---

# 101. Implementation phases

## IMU-1 — transport and raw telemetry

Implement:

- `imud`;
- BNO085 reports;
- timestamps;
- health;
- dashboard.

No control contribution.

## IMU-2 — encoder/IMU alignment

Implement:

- homing sample collection;
- `R_P_S` fit;
- validation;
- persistence.

Still no control contribution.

## IMU-3 — secondary motion verification

Implement:

- gyro residual;
- orientation delta residual;
- warnings;
- logging.

Initially warning-only.

## IMU-4 — base gravity

Implement:

- `BaseOrientationEstimator`;
- stationarity gating;
- base-moved detection.

Still no automatic pitch correction.

## IMU-5 — level yaw compensation

Implement:

- `WorldLevelConstraint`;
- reference-manager integration;
- feasibility;
- pitch/yaw coordination.

Enable first in slow developer/search mode.

## IMU-6 — auto-tare recovery

Implement:

- suspect reference detection;
- host re-alignment;
- optional runtime BNO tare;
- fall back to v1.

## IMU-7 — promote verification to safety action

After sufficient logs:

- warning thresholds;
- derate thresholds;
- controlled hold thresholds.

Do not enable fault response before normal residuals are characterized.

---

# 102. Recommended first implementation order for the agent

The agent should **not begin by commanding pitch from the BNO085**.

Recommended sequence:

```text
read BNO085
    ->
timestamp correctly
    ->
log BNO vs encoder
    ->
auto-fit R_P_S
    ->
prove yaw/pitch motion agreement
    ->
derive base gravity
    ->
prove tilt estimate
    ->
run level-yaw in simulation
    ->
run slow hardware level-yaw
    ->
enable automatic reference recovery
    ->
enable secondary safety actions
```

This prevents a wrong tare or frame convention from immediately becoming a motor-motion problem.

---

# 103. Design decisions frozen by this expansion

Unless hardware testing demonstrates a specific need to change them:

1. **One BNO085 is mounted on the moving pitch/camera stage.**
2. **The IMU remains secondary to the v1 CyberGear encoder/joint system.**
3. **The turret remains operational without the IMU by default.**
4. **BNO085 data acquisition runs outside the critical motor-control thread.**
5. **Gyro-Integrated Rotation Vector referenced to Game Rotation Vector is the preferred high-rate orientation source.**
6. **Magnetic heading is disabled/not trusted by default.**
7. **Formal BNO calibration is not a boot requirement.**
8. **Host auto-alignment is the primary tare/recovery mechanism.**
9. **`R_P_S` absorbs both imperfect physical mounting and unknown BNO output reorientation.**
10. **Normal v1 homing motion is reused to auto-estimate `R_P_S`.**
11. **Automatic Persist Tare is forbidden.**
12. **Optional runtime BNO all-axis tare is an escalation step only.**
13. **Gravity direction, not magnetic heading, defines horizontal.**
14. **Base tilt is updated mainly while mechanically stationary.**
15. **World-horizontal yaw is implemented by solving pitch from the existing kinematics and the gravity vector.**
16. **World-level compensation is a reference-generation feature, not a direct IMU inner servo.**
17. **The two-axis mechanism guarantees level optical-axis elevation, not full camera roll leveling.**
18. **Encoder/IMU verification compares timestamp-aligned 3D angular motion.**
19. **IMU failure falls back to v1; credible mechanical mismatch can derate/hold.**
20. **Unexplained orientation/tilt jumps never directly generate a corrective motor jump.**

---

# 104. Open commissioning values — not architecture blockers

The following remain to be measured/tuned:

1. Selected BNO085 transport.
2. Actual achievable Gyro-Integrated RV report rate.
3. Actual Game RV/gravity report rate.
4. IMU transport timeout limits.
5. Quaternion plausibility tolerance.
6. Gravity magnitude tolerance.
7. Minimum joint speed for alignment fitting.
8. Required yaw alignment sample count.
9. Required pitch alignment sample count.
10. Maximum alignment-fit RMS.
11. Held-out validation threshold.
12. Stationarity thresholds.
13. Base-move angular threshold.
14. Base-move dwell.
15. Reference jump threshold.
16. Runtime auto-tare retry count.
17. Motion verification warning thresholds.
18. Motion verification fault thresholds.
19. Mode-specific residual envelopes.
20. Level-yaw allowed elevation error.
21. Pitch/yaw coordinated path limits.
22. Fallback policy when level-yaw becomes infeasible.
23. Payload-specific verification thresholds.

These values belong in configuration and should be derived from logs.

---

# 105. External references

The implementation should be checked against the exact current BNO085 firmware and driver used on the Raspberry Pi.

Primary references:

## CEVA SH-2 Reference Manual

`https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf`

Relevant items:

- Tare command `0x03`
- Tare Now
- Persist Tare
- Set Reorientation
- Game Rotation Vector `0x08`
- Gyro-Integrated Rotation Vector `0x2A`
- common report status:
  - 0 unreliable
  - 1 low
  - 2 medium
  - 3 high

The manual documents that Gyro-Integrated Rotation Vector provides high-rate, low-latency orientation plus calibrated angular velocity and may use Game Rotation Vector as its reference.

## CEVA BNO08X Datasheet

`https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf`

Relevant items:

- arbitrary mounting / tare behavior;
- all-axis versus Z-axis tare;
- persistence behavior;
- interaction with magnetic North for magnetically referenced rotation vectors.

## Raspberry Pi / BNO085 I2C clock-stretching guidance

`https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching`

`https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes`

This is relevant only if I2C is chosen.

---

# Appendix A — Example IMU reference state machine

```text
ABSENT
  |
  | sensor discovered
  v
STARTING
  |
  | raw reports sane
  v
RAW_VALID
  |
  | collect encoder-known motion
  v
ALIGNING
  |
  +----fit valid----------> VALID
  |
  +----fit invalid--------> SUSPECT_REFERENCE
                                  |
                                  v
                           HOST_REALIGN_RETRY
                                  |
                    +-------------+-------------+
                    |                           |
                  valid                       invalid
                    |                           |
                    v                           v
                  VALID                  DEVICE_TARE_PENDING
                                                |
                                                v
                                         RUNTIME_TARE
                                                |
                                                v
                                            ALIGNING
                                                |
                                    +-----------+----------+
                                    |                      |
                                  valid                  invalid
                                    |                      |
                                    v                      v
                                  VALID                 DEGRADED
```

---

# Appendix B — Example base-orientation state machine

```text
UNKNOWN
  |
  | R_P_S valid + gravity valid
  v
COLLECTING_STATIONARY
  |
  | enough stable samples
  v
VALID
  |
  | candidate tilt jump
  v
CHANGE_SUSPECT
  |
  +--not persistent-------> VALID
  |
  +--persistent------------> HOLD_AND_REESTIMATE
                                  |
                                  v
                               VALID
```

---

# Appendix C — Example motion-verification logic

```text
if imu not valid:
    state = DISABLED

elif R_P_S not valid:
    state = LEARNING

else:
    motor = motor_history.interpolate(imu.timestamp)

    omega_enc =
        J_omega(motor.q) * motor.qdot

    omega_imu =
        R_P_S * imu.omega

    e = omega_imu - omega_enc

    update residual filters

    if current_mode == HARD_STOP_CONTACT:
        suppress normal mismatch fault

    elif severe residual persists:
        state = MISMATCH

    elif moderate residual persists:
        state = WARNING

    else:
        state = HEALTHY
```

---

# Appendix D — Example level-yaw solver

```text
function solve_level_pitch(
    yaw_target,
    elevation_target,
    up_B,
    pitch_min,
    pitch_max
):

    function f(pitch):
        d_B = camera_forward_vector(
            yaw_target,
            pitch
        )

        return dot(up_B, d_B) \
             - sin(elevation_target)

    if no sign change / no interior root:
        return NO_SOLUTION

    pitch =
        bounded_root_solve(
            f,
            pitch_min,
            pitch_max,
            previous_pitch_seed
        )

    return pitch
```

The actual implementation should use the v1 calibrated kinematic transform including camera extrinsics rather than an idealized yaw/pitch formula.

---

# Appendix E — Example host alignment solve

For collected samples:

```text
a_i = omega_S_imu_i
b_i = omega_P_encoder_pred_i
```

Find:

```text
R_P_S =
arg min_R
sum_i w_i || R a_i - b_i ||^2
```

subject to:

```text
R^T R = I
det(R) = +1
```

Use a standard Wahba/Kabsch solve.

Then validate:

```text
e_i = R_P_S a_i - b_i
```

on both fitting and held-out samples.

---

# Appendix F — Key implementation warning

A BNO085 tare can make the orientation output look numerically reasonable while still being referenced to the wrong frame.

Therefore the acceptance test is **not**:

```text
roll/pitch/yaw look plausible
```

The acceptance test is:

```text
does the BNO085 predict the same 3D physical angular motion
as the independently measured yaw/pitch encoders and kinematics?
```

That is the core reason this architecture can tolerate an unknown tare and an imperfect IMU installation.

---

# Appendix G — Key control warning

The IMU-derived base tilt is never allowed to create a sudden motor reference discontinuity.

Every newly accepted base orientation must enter control through:

```text
validated orientation
    ->
world-level reference solver
    ->
existing v1 safety envelope
    ->
existing jerk-limited trajectory
```

Never:

```text
new IMU angle
    ->
raw motor command
```

---

# Appendix H — Definition of completion

The BNO085 expansion is complete when all of the following are demonstrated:

1. Turret boots and operates normally with BNO085 disconnected.
2. Turret boots with BNO085 connected but poorly calibrated.
3. `R_P_S` is automatically recovered from normal homing motion.
4. A deliberately wrong BNO runtime tare is detected/recovered without unsafe motion.
5. Base roll/pitch tilt is recovered without requiring magnetic North.
6. With the base tilted, a world-horizontal yaw sweep automatically moves pitch and keeps optical elevation approximately constant.
7. Yaw encoder motion is independently verified by the BNO085.
8. Pitch encoder motion is independently verified by the BNO085.
9. A frozen/reset/stale BNO085 falls back to v1 without disrupting motor safety.
10. A credible persistent encoder-versus-IMU mechanical mismatch produces the configured derate/controlled-hold response.
11. IMU reset/re-tare does not cause a sudden pitch command.
12. Web telemetry and logs clearly show IMU health, reference quality, base tilt, and motion residuals.

