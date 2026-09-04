# OpenAutoTurret v3.2
## CyberGear Quiet-Drive, Predictive Tracking, and Closed-Loop Control Hardening Plan

**Document status:** Detailed implementation handover  
**Version:** 3.2  
**Date:** 2026-09-04  
**Depends on:** OpenAutoTurret v1, v3.0, v3.1, and the station handover at repo revision `ee1b33b`  
**Target:** Raspberry Pi 5 + IMX500 + two Xiaomi CyberGear direct-drive pan/tilt axes  
**Scope:** Non-weaponized camera/sensor tracking system  
**Primary concerns:** vibration/humming at hold, tracking lag, poor prediction, gearbox/encoder nonlinearity, and safe mode-specific motor control

---

# 1. Executive conclusion

The present behavior should not be treated as a single “PID tuning” problem.

There are at least three separate closed-loop problems:

1. **CyberGear inner servo behavior**
   - internal position loop;
   - internal speed PI loop;
   - internal current/FOC loop;
   - 7.75:1 reduction;
   - finite encoder/feedback resolution;
   - friction, gearbox play, and structural compliance.

2. **Host gimbal control**
   - 200 Hz motion reference;
   - mode selection;
   - soft-limit braking;
   - hold behavior;
   - position-versus-speed command selection;
   - payload-specific tuning.

3. **Visual tracking**
   - camera and detector latency;
   - track confirmation;
   - camera-motion compensation;
   - timestamp alignment;
   - target state estimation;
   - prediction to the time at which the actuator can actually respond.

The recommended final structure is:

```text
IMX500 / detector
        |
        | timestamped target observation
        v
TrackManager
        |
        v
base-frame angular target estimator
        |
        | q_target(t), qdot_target(t), covariance
        v
latency-ahead target predictor
        |
        v
mode-specific host controller
        |
        +-- AUTO_TRACK: host outer position loop -> CyberGear speed mode
        |
        +-- AUTO_ROAM / MANUAL GOTO: shaped position reference
        |
        +-- HOLD: quiet latched position hold
        |
        v
v1 SafetyEnvelope / braking / limits
        |
        v
CyberGear internal loops
        |
        v
payload
```

The immediate priorities are:

```text
1. make HOLD quiet and deterministic;
2. audit and expose the real CyberGear gains;
3. instrument motor command/current/position oscillation;
4. make prediction timestamp-correct;
5. reach a real 20–30 fps perception path;
6. A/B-test position mode versus host speed-mode tracking.
```

Do not implement MIT/torque control first.

---

# 2. Evidence levels used in this plan

The local implementation agent shall keep these categories separate.

```text
MEASURED
    reproduced on this station and recorded in the handover

DOCUMENTED
    described by the CyberGear manual/protocol or component documentation

INFERRED
    technically plausible explanation derived from measurements and architecture

UNVERIFIED
    must be tested before becoming a design assumption
```

Important examples:

- The station's 200 Hz control loop and measured tracking error are **MEASURED**.
- CyberGear's exposed `loc_kp`, `spd_kp`, and `spd_ki` registers are **DOCUMENTED**.
- “The hum is caused by the position loop crossing gearbox backlash” is currently **INFERRED**, not proven.
- CyberGear's actual backlash magnitude is **UNVERIFIED**; no authoritative backlash specification was found.

---

# 3. CyberGear facts relevant to this problem

The CyberGear manual describes:

- FOC drive;
- 7.75:1 reduction;
- AS5047P magnetic encoder;
- 14-bit single-turn absolute encoder resolution;
- position, speed, current, and operation/MIT-style modes;
- runtime parameters for:
  - `loc_kp`;
  - `spd_kp`;
  - `spd_ki`;
  - `limit_spd`;
  - `limit_cur`;
  - current-loop gains and filters;
- observed/internal signals including:
  - `cmdlocref`;
  - `cmdspdref`;
  - `cmdIq`;
  - `mechPos`;
  - `mechVel`;
  - `iqf`;
  - phase currents;
  - voltage and temperature.

The translated manual gives two sets of example/default gain values in different tables:

```text
persistent/config table:
    loc_kp = 30
    spd_kp = 2
    spd_ki = 0.021

runtime register table:
    loc_kp = 30
    spd_kp = 1
    spd_ki = 0.002
```

Therefore:

> Never assume the gain values from documentation. Read back the actual values from each installed motor and record the firmware revision.

---

# 4. Critical backend audit: `spd_kd` may actually be `spd_ki`

The station handover reports a backend function conceptually named:

```text
set_speed_loop_gains(axis, spd_kp, spd_kd)
```

However, CyberGear exposes:

```text
0x701F = spd_kp
0x7020 = spd_ki
```

not a speed-loop derivative gain.

The implementation agent must inspect the actual register written by the second argument.

If it writes `0x7020`, then:

```text
spd_kd
```

is a misleading name and must be renamed:

```text
spd_ki
```

throughout:

- C++ API;
- YAML;
- telemetry;
- tests;
- UI;
- payload profiles;
- tuning scripts.

This is not cosmetic.

A developer who believes they are changing damping (`Kd`) while actually changing integral action (`Ki`) can tune in exactly the wrong direction and make low-speed hunting worse.

Add a compile-time/register-map test:

```cpp
static_assert(kSpdKpRegister == 0x701F);
static_assert(kSpdKiRegister == 0x7020);
```

and unit tests for frame encoding.

---

# 5. Why the present HOLD behavior is suspicious

The handover reports that HOLD currently does approximately:

```text
command(axis, measured_position, 1.0e-6)
```

every 200 Hz cycle.

This combines three questionable behaviors:

1. **The hold target follows measured encoder noise.**
   - The hold target is not a fixed latched pose.
   - A one-count change can move the commanded target back and forth.

2. **The position command is recommitted every 5 ms.**
   - Rewriting an unchanged value may be harmless.
   - Rewriting a slightly changing quantized value is not harmless.

3. **`limit_spd = 1e-6 rad/s` is physically meaningless at this encoder scale.**

A 14-bit revolution has:

```text
2π / 16384
= 0.0003835 rad
= 0.02197°
```

per count.

At `1e-6 rad/s`, one encoder count nominally takes:

```text
0.0003835 / 0.000001
≈ 383.5 seconds
```

to traverse.

This is not a useful hold velocity.

It can leave the internal cascaded controller in a contradictory state:

```text
position error exists
but permitted speed is essentially zero
```

which may produce current effort, integrator accumulation, micro-corrections, or audible excitation without useful position motion.

This is an engineering inference and must be verified, but it is important enough to test first.

---

# 6. Likely causes of vibration and humming

Do not assume one cause.

## 6.1 Closed-loop hunting / limit cycle

Plausible mechanism:

```text
encoder changes one/few counts
    ->
position controller requests correction
    ->
speed PI/current loop builds torque
    ->
static friction or gearbox play prevents immediate output motion
    ->
torque crosses friction/backlash threshold
    ->
output jumps across target
    ->
error reverses
    ->
cycle repeats
```

Gear-driven gimbals are known to suffer from:

- backlash;
- hysteresis;
- flexibility;
- friction;
- limit cycles during reversing tracking motion.

## 6.2 Speed-loop integral action near zero

A speed PI integrator can continue accumulating while:

- position loop asks for tiny correction;
- speed is clamped;
- output is stuck in stiction;
- gearbox is within a dead zone.

When motion finally occurs, stored integral can overshoot and reverse.

## 6.3 Position stiffness too high

`loc_kp` may be unnecessarily high for a lightly loaded, balanced camera gimbal.

High stiffness can turn:

- encoder quantization;
- mounting flex;
- gear tooth compliance;
- cable drag;

into audible torque modulation.

## 6.4 Encoder/calibration issue

The motor uses an AS5047P and exposes an “encoder not calibrated” fault.

If the motor board, phase wiring, or motor relationship has changed, the manual calls for magnetic encoder/electrical calibration.

A bad electrical offset can create:

- inefficient FOC;
- excess current;
- torque ripple;
- noise;
- heating.

Do not run gain experiments until encoder calibration state and fault flags are confirmed.

## 6.5 Mechanical resonance

The direct-drive gimbal assembly can have resonant modes from:

- printed brackets;
- camera support;
- long payload arms;
- cable loom;
- motor mount;
- bearing/shaft compliance.

A closed loop can excite those modes even if the motor itself is healthy.

## 6.6 Gearbox play

The planetary reduction is a plausible contributor.

However:

> No authoritative CyberGear backlash number was found.

Do not write a guessed backlash value into the controller.

Measure the installed assembly.

## 6.7 Normal electromagnetic/acoustic noise

Some humming may be FOC/PWM-related rather than mechanical hunting.

Distinguish it experimentally.

---

# 7. Diagnostic decision tree for the noise

Run one axis at a time, near the middle of safe travel, with low current limits and the payload mechanically supported where appropriate.

```text
Motor disabled
    |
    +-- noise/vibration present?
            |
            yes -> mechanical/external source
            no
            |
Motor enabled but zero-current/current mode 0
    |
    +-- noise?
            |
            yes -> drive/PWM/calibration/electrical source
            no
            |
Position hold, fixed latched LocRef
    |
    +-- noise?
            |
            yes -> closed-loop stiffness/integral/quantization/friction
            no
            |
Current implementation: q_meas recommitted + 1e-6 limit
    |
    +-- noise appears?
            |
            yes -> current HOLD implementation implicated
```

Then compare:

```text
unloaded vs payload
yaw vs pitch
high vs reduced loc_kp
normal vs reduced spd_ki
repeated writes vs cached hold target
```

---

# 8. Required motor telemetry

The current normal feedback frame is not enough for diagnosis.

Add optional diagnostic reads/logging for:

```text
run_mode
loc_ref
limit_spd
limit_cur
loc_kp
spd_kp
spd_ki
mechPos
mechVel
iqf

if firmware supports observed parameters:
cmdlocref
cmdspdref
cmdIq
cmdTorque
encoderRaw
motorTemp
boardTemp
VBUS
faultSta
warnSta
```

Store with a monotonic timestamp.

Minimum useful diagnostic rate:

```text
position / velocity / torque feedback:
    as fast as reliably available

control command log:
    200 Hz

slower parameter reads:
    10–50 Hz where CAN/firmware permits
```

Do not overload the bus with repeated diagnostic parameter requests during normal operation.

Use a dedicated tuning mode.

---

# 9. Frequency analysis

The agent shall characterize the noise in both time and frequency domains.

Log:

```text
q_meas
q_ref
q_hold
mechVel
iqf / torque
commanded speed
commanded mode
```

Optional:

```text
microphone audio
BNO085 angular velocity when installed
```

Calculate:

- peak-to-peak position jitter;
- RMS position jitter;
- RMS and peak `iqf`;
- dominant frequency in position;
- dominant frequency in current;
- coherence between current and position;
- dominant audio frequency.

Interpretation:

```text
same peak in iqf and q_meas
    -> servo-induced mechanical hunting likely

audio peak but no q/iq modulation
    -> PWM/electromagnetic acoustic source more likely

q movement with low encoder change but external load movement
    -> compliance/backlash/external sensor issue

strong oscillation only with payload
    -> payload resonance or gain mismatch
```

---

# 10. Parameter snapshot before tuning

Create:

```text
tools/cybergear_snapshot
```

It reads and records per motor:

```yaml
motor_id:
firmware:
app_git_version:
encoder_calibrated:
run_mode:

persistent:
  loc_kp:
  spd_kp:
  spd_ki:
  spd_filt_gain:
  cur_kp:
  cur_ki:
  cur_filt_gain:
  limit_spd:
  limit_cur:
  limit_torque:
  can_timeout:

runtime:
  loc_kp:
  spd_kp:
  spd_ki:
  limit_spd:
  limit_cur:
```

The tool shall:

- refuse to write while motor is running;
- save original values;
- generate a restoration file;
- verify every write by readback;
- restore automatically after a failed trial.

Documentation indicates parameter-table configuration should occur while the motor is in standby, and the manual explicitly says not to switch control mode while the joint is running.

---

# 11. Safe tuning harness

Create self-reverting experiments.

Every trial shall:

1. verify mechanical home and safe central pose;
2. verify software envelope unchanged;
3. lower current and speed to the trial profile;
4. apply exactly one parameter change;
5. run for a bounded duration;
6. monitor temperature/current/velocity;
7. stop on any safety warning;
8. restore original values;
9. verify restoration by readback;
10. save results.

Never tune at a hard stop.

Never tune both axes simultaneously at first.

---

# 12. Quiet HOLD redesign

## 12.1 New HOLD contract

HOLD means:

> Electrically maintain one fixed safe pose with the minimum stiffness/current needed for the current payload, without chasing encoder noise.

It does not mean:

- continually set target equal to current measurement;
- use an effectively zero speed limit;
- disable feedback;
- freewheel.

## 12.2 Latch the hold target once

On entry:

```text
BRAKE_TO_HOLD
    ->
verify |velocity| < hold_entry_velocity for dwell
    ->
collect short stable position window
    ->
q_hold = robust median/mean
    ->
quantize/latch once
    ->
enter HOLD
```

During HOLD:

```text
q_hold remains constant
```

unless:

- operator requests new motion;
- disturbance exceeds a separately defined recenter threshold;
- calibration/reference changes;
- safety supervisor replaces it.

Do not set:

```text
q_hold = q_meas
```

every cycle.

## 12.3 Cache motor parameter writes

In HOLD:

- write `LocRef` once on entry;
- write `LimitSpd` only when profile changes;
- write `LimitCur` only when profile changes;
- continue health/feedback supervision;
- send only the minimum required command/heartbeat behavior verified for the firmware.

If continuous command frames are required, send the same bit-identical command, not a newly sampled target.

## 12.4 Use a meaningful hold speed limit

Replace `1e-6 rad/s` with a low but physically resolvable configured value.

The value must be derived experimentally.

It should be:

- low enough to make disturbance recovery gentle;
- high enough that a one-count/few-count correction can complete without minutes of saturation;
- independently limited by current and gain.

## 12.5 Hold hysteresis

Use two thresholds:

```text
quiet-enter error: e_enter
correction-exit error: e_exit
e_exit > e_enter
```

Within the quiet zone:

- do not rewrite target;
- do not command a corrective host velocity;
- allow small encoder variation.

Outside the exit threshold:

- perform a slow bounded recenter;
- return to quiet hold.

Initial threshold derivation:

```text
e_enter =
max(
    3 * measured_stationary_encoder_sigma,
    a fraction of measured load-side lost motion
)

e_exit =
1.5 to 2.5 * e_enter
```

These are tuning rules, not fixed numbers.

## 12.6 Axis-specific hold

Yaw and pitch need different hold profiles.

Yaw may tolerate:

- lower stiffness;
- larger quiet deadband.

Pitch may require:

- higher minimum stiffness/current;
- gravity support;
- eventual gravity feed-forward.

Do not apply one universal HOLD profile.

---

# 13. Internal hold profiles

Add internal controller profiles, not new operator modes:

```text
HOLD_QUIET
HOLD_STIFF
```

### HOLD_QUIET

Use when:

- station is undisturbed;
- payload balanced;
- tracking inactive.

Goals:

- low noise;
- low heat;
- no hunting.

### HOLD_STIFF

Use when:

- external disturbance detected;
- calibration/test requires stiffness;
- payload needs stronger support.

Goals:

- stronger rejection;
- noise is secondary.

The operator still sees:

```text
MANUAL / HOLD
AUTO_TRACK / LOST_HOLD
```

not a fourth/fifth mode.

---

# 14. CyberGear gain tuning order

Do not start by modifying the current loop.

Recommended order:

```text
1. verify encoder calibration and mechanics
2. correct HOLD command behavior
3. reduce speed-loop integral if hunting remains
4. tune speed proportional response
5. tune position proportional stiffness
6. evaluate speed filter
7. only then consider current-loop tuning
```

## 14.1 Tune `spd_ki` first for low-speed hunting

Because integral action can accumulate across:

- static friction;
- backlash;
- a speed clamp;
- a deadband;

test reductions relative to the actual readback:

```text
100%
50%
25%
0% for a brief diagnostic trial
```

Do not persist immediately.

Observe:

- hum;
- `iqf`;
- static position error;
- disturbance recovery;
- slow velocity tracking.

If removing/reducing `spd_ki` strongly reduces hum, integral-driven hunting is implicated.

## 14.2 Tune `loc_kp`

After speed-loop behavior is stable, test reduced position stiffness.

Relative test sequence:

```text
100%
75%
50%
25%
```

Stop when:

- hold becomes too compliant;
- tracking/goto settling becomes unacceptable;
- gravity disturbance cannot be rejected.

Do not assume `loc_kp=30` is appropriate for the payload merely because it is documented as a default.

## 14.3 Tune `spd_kp`

Tune for:

- adequate velocity response;
- no high-frequency oscillation;
- acceptable disturbance rejection.

Increasing `spd_kp` can increase stiffness/damping of velocity response, but may amplify measurement noise.

Use the measured response, not generic PID rules.

## 14.4 Speed filter

The motor exposes a speed filter gain.

Its exact firmware convention must be confirmed.

Do not assume “higher value means more filtering” without a step test/read of documentation/firmware behavior.

Test only after base gains are understood.

## 14.5 Current limit

Set per mode and payload.

The current limit is not a tuning gain, but it limits the energy available to a bad loop.

Initial tuning trials should use a conservative value just above that required for:

- controlled motion;
- static pitch support;
- expected disturbance.

Do not change thermal/over-current protection thresholds.

---

# 15. Motor-command write policy

The current backend reportedly writes:

```text
LocRef + LimitSpd
```

on every 200 Hz normal-motion cycle.

Refactor the backend so each property has a cache:

```cpp
struct MotorCommandCache {
    RunMode mode;
    float loc_ref;
    float spd_ref;
    float limit_spd;
    float limit_cur;
    float loc_kp;
    float spd_kp;
    float spd_ki;
};
```

Write when:

```text
value changed by more than protocol/control epsilon
OR
refresh timeout elapsed
OR
motor reset generation changed
OR
readback mismatch detected
```

Suggested policy:

```text
LocRef during dynamic position motion:
    up to control rate, only if target changed meaningfully

LocRef in HOLD:
    once on entry, then low-rate verified refresh if needed

LimitSpd / LimitCur / gains:
    only on mode/profile transition

SpdRef during speed control:
    control rate, with change threshold and watchdog refresh
```

This reduces:

- CAN traffic;
- parameter churn;
- accidental reference noise;
- ambiguity about which frame caused feedback.

---

# 16. Motor mode policy by controller state

| Controller state | Initial CyberGear mode | Reason |
|---|---|---|
| Sensorless homing sweep | Speed mode | existing proven approach |
| Homing contact/fine approach | Speed/current-limited | low-energy contact |
| Manual step/GOTO | Position mode | bounded point-to-point control |
| AUTO_ROAM | Position mode initially | measured stable shaped sweep |
| AUTO_TRACK | A/B-test speed-mode outer loop | reduce drive-side position lag |
| COASTING | Speed mode, derated | preserve smooth short prediction |
| BRAKE_TO_HOLD | active speed/trajectory stop | controlled transition |
| LOST_HOLD | Quiet position hold | no drift |
| Safe park motion | existing validated park strategy | preserve v1 behavior |
| Parked pitch before disable | position hold then disable | mechanical safety |
| Future precision/compliant tracking | Operation/MIT candidate | later phase |

---

# 17. Safe mode transition state machine

The CyberGear manual says not to switch mode while the joint is running.

Use:

```text
REQUEST_MODE_CHANGE
    ->
BRAKE_CURRENT_MODE
    ->
verify |mechVel| < threshold for dwell
    ->
latch current q
    ->
send motor stop
    ->
write run_mode
    ->
write target-mode limits/gains
    ->
read back
    ->
enable
    ->
send bumpless initial command
    ->
verify feedback
    ->
ACTIVE
```

## 17.1 Position -> speed

Initial speed command:

```text
spd_ref = current measured velocity
```

then smoothly blend toward controller output.

Do not begin with a sudden nonzero velocity.

## 17.2 Speed -> position hold

Sequence:

```text
ramp spd_ref toward zero
verify low velocity
q_hold = robust current position
stop
switch mode
write LocRef = q_hold
enable
```

Do not switch to a stale previous LocRef.

## 17.3 Transition failure

Any failed write/readback/feedback timeout:

```text
SafetySupervisor -> HOLD/FAILSAFE
```

Do not continue with an uncertain RunMode.

---

# 18. AUTO_TRACK motor control: host outer loop

The most promising practical improvement is a host outer position loop using CyberGear speed mode.

Per axis:

```math
e_q = q_d - q
```

```math
v_cmd_raw = v_d + K_p^{host} e_q
```

where:

- `q_d`: latency-compensated desired joint position;
- `v_d`: predicted/feed-forward desired joint angular velocity;
- `q`: measured logical joint position.

Then:

```text
v_cmd =
    safety_velocity_limit(
        jerk_accel_limit(
            deadband_and_confidence_gate(v_cmd_raw)
        )
    )
```

Send:

```text
SpdRef = v_cmd
```

## 18.1 No host integral initially

Do not add `Ki_host` initially.

Reasons:

- drive already contains speed integral;
- backlash/stiction create windup risk;
- target is moving;
- limits introduce saturation;
- HOLD is handled by position mode.

If steady dynamic bias remains later, add a tightly bounded conditional integrator only when:

- not near limits;
- not saturated;
- target confidence high;
- direction stable.

## 18.2 Do not differentiate quantized position directly

Avoid:

```text
Kd * derivative(raw q error)
```

at the host.

Use:

- CyberGear `mechVel`;
- a filtered velocity estimate;
- MIT-mode `Kd` only in a later experiment.

Raw finite differences of ~0.022° feedback can be noisy.

## 18.3 Host gain bandwidth

Start with low `Kp_host`.

Increase until:

- lag materially improves;
- no hunting;
- no oscillation during reversal;
- speed/current remain acceptable.

The outer loop must be substantially slower than the motor's internal velocity/current loops and below dominant mechanical resonance.

Determine this experimentally.

---

# 19. Speed-command safety envelope

For each axis, compute permitted speed toward each boundary.

At minimum:

```math
v_{limit}(d) = sqrt(2 a_{brake} max(0, d - d_{margin}))
```

For production, use the same jerk-limited stop model as v1.

Then:

```text
if v_cmd > 0:
    v_cmd <= positive_boundary_speed_limit

if v_cmd < 0:
    |v_cmd| <= negative_boundary_speed_limit
```

Also apply:

```text
|v_cmd| <= v_mode_max
|dv_cmd/dt| <= a_mode_max
|d²v_cmd/dt²| <= j_mode_max
```

A speed-mode controller must never be allowed to rely on position clipping alone.

---

# 20. Tracking quiet zone

Do not move the gimbal for every subpixel/bbox-centroid fluctuation.

Define an angular dead zone in camera LOS space.

```text
enter quiet zone:
    |e_los| < e_enter
    AND target angular speed confidence is low

exit quiet zone:
    |e_los| > e_exit
    OR target angular speed confidently nonzero
```

with:

```text
e_exit > e_enter
```

Derive `e_enter` from measured target observation noise.

Example:

```text
e_enter_yaw =
max(
    2–3 * stationary target LOS sigma,
    one meaningful output/gear correction threshold
)
```

The predictor continues to update while motor output is quiet.

This prevents a closed mechanical loop from chasing vision noise.

---

# 21. Operation/MIT mode: later option

CyberGear operation mode supports one CAN frame containing:

```text
target position
target velocity
torque feed-forward
Kp
Kd
```

Potential advantages:

- explicit low stiffness;
- explicit damping;
- no hidden position-loop default gain;
- gravity feed-forward;
- a single dynamic command frame.

Potential use later:

```text
q_cmd = predicted target
v_cmd = predicted angular rate
kp = modest stiffness
kd = damping
torque = pitch gravity feed-forward
```

## 21.1 Why it is not phase one

Risks/unknowns:

- current backend lacks it;
- exact firmware behavior must be verified;
- command quantization;
- 200 Hz host/MCP2515 timing;
- mode-switch safety;
- tuning Kp/Kd/torque incorrectly can create high-energy oscillation.

First fix HOLD and test speed-mode AUTO_TRACK.

Proceed to MIT mode only if those cannot meet the target.

---

# 22. Mechanical backlash/compliance measurement

No published CyberGear backlash specification was located.

Measure the assembled axis.

## 22.1 Determine where the useful encoder information is

The manual labels `mechPos` as load-end mechanical angle.

Verify empirically whether it follows:

- actual output housing/shaft angle;
- motor-side angle transformed by ratio;
- an internally estimated angle.

This distinction matters.

## 22.2 External load-angle reference

Before BNO085 is installed, use one of:

- printed fiducial observed by a fixed external camera;
- dial indicator against a long rigid lever;
- high-resolution external angle sensor;
- optical reference marker.

After BNO085 is installed:

- use gyro/orientation as secondary motion reference;
- still recognize IMU drift/noise at static very small angles.

## 22.3 Reversal/lost-motion test

At a safe central pose:

1. apply a very slow positive motion until external load motion is clear;
2. stop;
3. reverse at very low speed/current;
4. record:
   - CyberGear `mechPos`;
   - external load angle;
   - `iqf`;
   - time;
5. identify how much motor-reported movement/current occurs before the load reverses.

Repeat in both directions.

This estimates:

- lost motion;
- stiction;
- hysteresis;
- direction-dependent cable force.

Do not use a violent relay test on the assembled camera.

---

# 23. Structural resonance identification

Run low-amplitude, safe tests around the central pose:

```text
sine sweep / chirp
small amplitude
low current
frequency below unsafe range
```

Measure:

- commanded q/v;
- encoder q/v;
- current;
- BNO085 later;
- external marker if available.

Find resonance peaks.

Set control bandwidth comfortably below the first poorly damped structural mode unless an explicit notch/input shaper is implemented.

Input shaping or a notch filter may be useful later, but only after a repeatable resonance is measured.

---

# 24. Why prediction currently “barely works”

Based on station evidence, prediction is not yet being evaluated under a healthy tracking chain.

Current blockers include:

- detector + preview around 9.55 fps;
- frame-based warmup/confirmation;
- no confirmed selectable track during the handover session;
- confidence invalid in observed samples;
- AUTO_TRACK produced no meaningful motion;
- vision stage latency is uninstrumented;
- position-mode motor lag is substantial.

Therefore:

> Do not conclude that the Kalman model alone is the primary failure.

Prediction cannot repair:

- a target identity that never confirms;
- a 500 ms acquisition delay;
- a stale timestamp;
- a wrong motor pose at capture time;
- a motor loop with ~0.17 s or greater effective lag;
- unstable bbox centers.

Fix the measurement chain and actuator model together.

---

# 25. Tracking timing model

Define explicit timestamps:

```text
t_exposure
    SensorTimestamp / frame capture reference

t_vision_receive
    camera request/callback available

t_detection_done

t_track_publish

t_control_receive

t_control_cycle

t_command_send

t_motor_effect
    estimated time command meaningfully affects output
```

Per frame, calculate:

```text
camera/vision age:
    t_control_cycle - t_exposure

vision processing:
    t_track_publish - t_exposure
    or callback-relative subdivisions

transport:
    t_control_receive - t_track_publish

command path:
    t_command_send - t_control_cycle

actuator delay:
    identified from command/encoder response
```

Do not use publish→receive as “inference latency”.

---

# 26. Target coordinate system

Do not filter raw screen pixel error as the primary state while the camera is moving.

For every selected target observation:

1. use its capture `SensorTimestamp`;
2. interpolate yaw/pitch at that timestamp;
3. convert target anchor pixel to a calibrated camera ray;
4. transform the ray into the fixed turret-base frame;
5. convert to base-frame angular LOS;
6. update the target estimator at the capture time.

This removes camera motion from estimated target motion.

State:

```text
theta_yaw_target_B
theta_pitch_target_B
```

or an equivalent unit-vector representation.

Given the mechanism's limited travel, unwrapped angular coordinates are adequate if discontinuities are handled carefully.

---

# 27. Recommended first target estimator

Use a constant-angular-velocity Kalman filter.

State:

```math
x =
[theta_y,
 theta_p,
 omega_y,
 omega_p]^T
```

Measurement:

```math
z =
[theta_y,
 theta_p]^T
```

For time step `dt`:

```math
F =
[1 0 dt 0
 0 1 0 dt
 0 0 1  0
 0 0 0  1]
```

```math
H =
[1 0 0 0
 0 1 0 0]
```

Use a continuous white-acceleration process model.

For one axis:

```math
Q_axis = sigma_a^2 *
[dt^4/4  dt^3/2
 dt^3/2  dt^2]
```

Build block-diagonal `Q` for yaw/pitch initially.

This is simple, computationally cheap, and appropriate before adding an acceleration state.

---

# 28. Filter state-time rule

Keep the authoritative estimator state at:

```text
the timestamp of the most recently accepted camera measurement
```

When a new frame arrives in timestamp order:

```text
propagate previous state to t_capture
update with measurement
store state at t_capture
```

At each 200 Hz control cycle:

```text
copy estimator state
predict copy to t_effective
```

Do not mutate the measurement-state time to “now” on every control query.

This avoids double-counting latency.

If a frame arrives out of order:

- either reject it if very old;
- or use a bounded fixed-lag replay buffer.

Given one ordered camera stream, rejection plus diagnostics may be sufficient initially.

---

# 29. Effective prediction horizon

Define:

```text
t_effective =
    current_control_time
  + estimated_CAN_command_delay
  + estimated_motor_response_lead
```

Then:

```text
prediction_horizon =
    t_effective
  - estimator_state_timestamp
```

This naturally contains:

- age of the latest image;
- detector/association delay;
- transport delay;
- control scheduling;
- motor response lead.

Do not separately add camera age again if the estimator state is still at capture time.

---

# 30. Actuator delay model

Identify per axis and payload.

Start with:

```math
G(s) = e^{-T_d s} / (tau s + 1)
```

for the relationship of command to measured motion.

Parameters:

```text
T_d:
    pure delay / communication + internal update

tau:
    dominant response time constant
```

Estimate from:

- small step response;
- small sine sweep;
- cross-correlation between command and velocity/position;
- least-squares fit.

Maintain separate models for:

```text
position mode
speed mode
yaw
pitch
payload profile
```

Do not use the measured position-mode `tau` for speed mode without re-identifying it.

---

# 31. Measurement covariance from detector quality

Do not use one fixed `R` for every bounding box.

Estimate angular measurement uncertainty from:

- detector confidence;
- track association score;
- bbox width/height;
- target near image edge;
- motion blur proxy;
- partial occlusion;
- bbox-center jitter history.

Basic form:

```math
R =
diag(sigma_yaw^2, sigma_pitch^2)
```

Approximate conversion:

```math
sigma_angle_yaw ≈ sigma_pixel_x / fx
sigma_angle_pitch ≈ sigma_pixel_y / fy
```

for small angles.

Larger uncertainty means the filter trusts prediction more and measurement less.

---

# 32. Innovation gating

Innovation:

```math
nu = z - H x_pred
```

Innovation covariance:

```math
S = H P H^T + R
```

Mahalanobis distance:

```math
d² = nu^T S^{-1} nu
```

For a 2D measurement, useful gates are approximately:

```text
95%: 5.99
99%: 9.21
```

Policy:

- below normal gate: update;
- moderate outlier: inflate `R` / soft reject;
- severe outlier: reject;
- several coherent outliers: declare maneuver/reacquisition instead of clinging to old model.

Do not reset the track on one bad box.

---

# 33. Adaptive process noise

A fixed low process noise gives smooth but lagging prediction.

A fixed high process noise follows maneuvers but jitters.

Use innovation-based adaptation:

```text
small consistent innovation:
    reduce/hold sigma_a

large coherent innovation:
    increase sigma_a temporarily

random alternating innovation:
    increase measurement R instead
```

Keep bounded:

```text
sigma_a_min <= sigma_a <= sigma_a_max
```

Log the active value.

---

# 34. Prediction confidence and lead limiting

The predicted covariance grows with horizon.

Use it to scale motion.

```text
high confidence:
    normal velocity/acceleration
    full identified lead

medium:
    reduced lead and acceleration

low:
    coast gently

invalid:
    brake/hold
```

Cap prediction horizon.

Initial engineering cap:

```text
300–500 ms
```

depending on target maneuver and measured frame rate.

This is not an acceptance constant; tune it.

A one-second straight-line extrapolation of a person is usually not trustworthy.

---

# 35. Better maneuver handling

After the CV filter is correct and measured, consider:

## Option A — adaptive CV

Preferred first.

- simplest;
- good at 20–30 fps;
- process noise increases on maneuver.

## Option B — constant acceleration

State:

```text
angle, rate, acceleration
```

Needs more stable observations.

Can overshoot badly after abrupt stops.

## Option C — IMM

Two models:

```text
stationary/slow
constant velocity or maneuvering
```

or:

```text
CV
CA
```

An interacting multiple-model filter can switch between smooth and maneuvering behavior.

Only implement if logs show CV failure during real target maneuvers.

## Option D — switching predictor

A transition detector can temporarily change to a filter better suited to velocity steps.

This is supported by visual-servoing literature, but it is phase two.

---

# 36. Target prediction output

The estimator shall expose:

```text
measured_los
filtered_los_at_last_capture
predicted_los_at_effective_time

predicted_angular_velocity
prediction_horizon_ms
covariance
innovation
innovation_gate_state
confidence
```

The HUD can show measured versus predicted target.

The motor controller consumes:

```text
q_d
qdot_d
confidence
```

not raw bbox pixels.

---

# 37. Camera/motor latency calibration

## 37.1 Vision timing

Instrument each frame first.

## 37.2 Motor timing

Use a safe small target/reference waveform.

Record:

```text
command timestamp
feedback timestamp
q command
v command
q measured
v measured
```

Fit delay.

## 37.3 End-to-end validation

Use a target moving at known angular rate.

Examples:

- monitor displaying a horizontally moving high-contrast object at calibrated geometry;
- rotating target fixture;
- controlled person/marker movement across known FOV.

Measure:

```text
target angle at SensorTimestamp
predicted angle at motor-effective time
actual camera LOS
```

---

# 38. Acquisition latency versus steady tracking

Treat separately.

## Acquisition

Includes:

- detector warmup;
- track confirmation;
- target selection;
- estimator initialization.

## Steady tracking

Includes:

- current frame age;
- filter update;
- actuator response.

Prediction can substantially improve steady tracking.

It cannot eliminate the requirement to obtain enough evidence to confirm a new track.

Convert confirmation to time and reach 20–30 fps.

---

# 39. Estimator initialization

On first confirmed measurement:

```text
angle = measured angle
angular velocity = 0
large velocity covariance
```

After a second/third timestamped measurement:

- estimate velocity;
- reduce covariance.

During early acquisition:

- cap motor acceleration;
- avoid a large lead based on uncertain velocity.

Use an `ACQUIRING` state until velocity is credible.

---

# 40. Lost-target handling

Recommended:

```text
VISIBLE
    -> normal filter updates

brief missing
    -> predict / COASTING
    -> increasing covariance
    -> reducing motion limits

coast timeout
    -> BRAKE_TO_HOLD

LOST_HOLD
    -> quiet position hold

confident reacquisition
    -> update/initialize filter
    -> bumpless speed-mode entry
```

Do not continue extrapolating indefinitely.

---

# 41. Stable target identity is prerequisite

Prediction must follow one physical object.

The current simple detector:

- emits only one largest moving blob;
- cannot validate multiple people;
- did not produce a reliably confirmed selectable target in the handover.

Production path:

```text
NN detections
    ->
class-aware NMS
    ->
TrackManager
    ->
selected target
```

Before prediction, duplicate detections must be suppressed.

Association should combine:

- predicted position;
- IoU;
- scale;
- short-lived non-biometric appearance.

Do not let identity changes masquerade as high target acceleration.

---

# 42. Vision frame rate and predictor semantics

Current tracking thresholds are frame-based.

That is wrong for a system whose fps may change from 10 to 30.

Convert to time:

```yaml
tracking:
  confirm:
    minimum_observations: 3
    minimum_visible_ms: 120

  coast_ms: 300

  reacquire_ttl_ms: 3000
```

Then:

- 15 fps and 30 fps retain similar temporal meaning;
- increased fps gives more observations rather than silently halving identity lifetime.

---

# 43. Preview architecture

The preview currently reduces detector rate substantially.

Implement:

```text
camera owner
  |
  +--> detection stream / metadata path
  |
  +--> latest-only preview buffer
          |
          v
      separate encoder/web worker
```

Rules:

- preview may drop;
- detector does not wait;
- no 8 MB/frame copy chain if avoidable;
- use a lower-resolution ISP stream if supported;
- JPEG conversion outside the vision critical thread.

Target:

```text
detector/TrackSet:
    final ~30 fps
    interim >=20 fps

web preview:
    10–15 fps acceptable
```

Control response matters more than preview smoothness.

---

# 44. Control-rate structure

Recommended:

```text
camera/detector:
    20–30 Hz

TrackManager:
    once per detector frame

target estimator measurement update:
    once per accepted selected-target observation

target prediction query:
    200 Hz

motor feedback:
    asynchronous

motor command:
    up to 200 Hz during dynamic motion
    cached/minimized during hold
```

Do not run detection association at 200 Hz.

---

# 45. Detailed AUTO_TRACK cycle

```text
every 200 Hz:

1. read latest timestamped motor state

2. if a new selected-target observation exists:
       pose_at_capture =
           motor_history.interpolate(sensor_timestamp)

       ray_camera =
           pixel_to_camera_ray(anchor, intrinsics)

       ray_base =
           transform_camera_ray_to_base(
               ray_camera,
               pose_at_capture,
               camera_extrinsics
           )

       z =
           ray_to_base_angles(ray_base)

       TargetEstimator.update(
           measurement_timestamp,
           z,
           detector_derived_R
       )

3. predict target to:
       now + actuator_delay

4. solve predicted LOS to joint:
       q_d

5. obtain:
       qdot_d
       covariance/confidence

6. AutoTrackController:
       v_raw = qdot_d + Kp_host * (q_d - q_meas)

7. apply:
       target-confidence derate
       visual dead zone
       v/a/j constraints
       stopping-distance limits
       coupled collision envelope

8. send CyberGear speed command

9. log:
       target/filter/control/motor state
```

---

# 46. Bumpless reacquisition

When target returns from LOST_HOLD:

1. obtain several valid observations if association uncertainty is high;
2. initialize/update target velocity cautiously;
3. remain in position hold until:
   - target identity valid;
   - target covariance below threshold;
   - axis mode transition allowed;
4. transition position→speed at zero/low velocity;
5. begin with:
   ```text
   v_cmd = measured velocity
   ```
6. blend feed-forward and proportional correction over a short ramp.

Avoid a sudden chase command on one reacquired frame.

---

# 47. Direction reversal and gearbox play

Target tracking frequently reverses.

This is the worst region for backlash/stiction.

On predicted velocity sign change:

- reduce acceleration around zero;
- avoid integrating through the dead zone;
- require the target velocity sign to be credible;
- use hysteresis before reversing for very small errors;
- log reversal current and lost motion.

Do not add a full backlash inverse compensation until backlash has been measured.

An incorrect inverse can amplify impact and vibration.

---

# 48. Optional backlash-aware controller after measurement

If a repeatable deadband `b` is measured, consider a backlash state:

```text
ENGAGED_POSITIVE
FREE_PLAY
ENGAGED_NEGATIVE
```

During reversal:

```text
command low controlled take-up velocity
current limit reduced
do not use aggressive position gain
detect load motion engagement
then resume normal tracking
```

With BNO085/load observation, engagement can be detected more reliably.

This is phase three.

---

# 49. Pitch-specific design

Pitch is not just another yaw axis.

Potential effects:

- residual gravity torque;
- position-dependent load;
- backdrive risk;
- static support current;
- different resonance.

Before IMU:

- use a separate gain/current profile;
- keep position HOLD;
- do not disable pitch except after safe park.

After BNO085:

- estimate gravity orientation;
- characterize static `iqf` versus pitch;
- optionally add torque feed-forward in operation/MIT mode later.

---

# 50. Yaw-specific design

Yaw is likely more tolerant of compliant hold.

Recommended:

- larger quiet deadband;
- lower hold stiffness;
- lower current cap where practical;
- more aggressive use of speed mode for tracking;
- cable-drag compensation only after measured.

---

# 51. Telemetry additions

Add these to high-rate logs:

## Motor/controller

```text
axis
run_mode
q_meas
v_meas
torque/iqf
q_hold
q_requested
q_reference
v_feedforward
v_feedback_correction
v_cmd_before_limits
v_cmd_after_limits
loc_kp
spd_kp
spd_ki
limit_spd
limit_cur
mode_transition_state
```

## Target estimator

```text
SensorTimestamp
receive_timestamp
measurement_los
filtered_los
predicted_los
angular_velocity
prediction_horizon
P diagonal/covariance summary
innovation
Mahalanobis distance
measurement accepted/rejected
process_noise_scale
measurement_noise_scale
```

## End-to-end

```text
measurement age
vision processing latency
transport latency
command timestamp
motor response delay estimate
```

---

# 52. HUD/diagnostics changes

Normal Apache-style HUD stays clean.

Diagnostics drawer adds:

```text
MOTOR QUIETNESS
    hold jitter
    iq RMS
    dominant oscillation frequency
    active hold profile

TRACK PREDICTION
    horizon
    covariance
    innovation
    target angular velocity
    actuator delay estimate

CYBERGEAR
    actual readback gains
    run mode
    current/speed limits
    encoder calibration status
```

Never expose gain editing in normal operator mode.

---

# 53. Code modules to add

```text
control/src/motor/
    cybergear_parameter_map.*
    cybergear_profile_manager.*
    cybergear_mode_transition.*
    motor_command_cache.*

control/src/control/
    quiet_hold_controller.*
    dynamic_tracking_controller.*
    actuator_delay_model.*
    command_rate_limiter.*

control/src/tracking/
    angular_target_estimator.*
    adaptive_process_noise.*
    target_measurement_model.*

tools/
    cybergear_snapshot
    cybergear_restore
    quiet_hold_trial
    gain_sweep_trial
    actuator_identification
    tracking_latency_report
```

---

# 54. Backend API changes

Add explicit names:

```cpp
Result set_position_gain(Axis, float loc_kp);
Result set_speed_pi_gains(Axis, float spd_kp, float spd_ki);
Result set_speed_filter_gain(Axis, float value);

Result read_runtime_parameter(Axis, Parameter);
Result write_runtime_parameter(Axis, Parameter, Value);
Result verify_parameter(Axis, Parameter, Expected);

Result request_mode_transition(Axis, RunMode, MotorProfile);
```

Do not keep an API called:

```text
set_speed_loop_gains(kp, kd)
```

if the second register is `spd_ki`.

---

# 55. Configuration example

```yaml
cybergear:
  parameter_readback_required: true

  yaw:
    hold_profile: quiet_yaw
    track_profile: dynamic_yaw

  pitch:
    hold_profile: quiet_pitch
    track_profile: dynamic_pitch

motor_profiles:
  quiet_yaw:
    run_mode: position
    loc_kp: READBACK_AND_TUNE
    spd_kp: READBACK_AND_TUNE
    spd_ki: READBACK_AND_TUNE
    limit_current_a: COMMISSION
    hold_speed_limit_rad_s: COMMISSION
    hold_deadband_enter_deg: COMMISSION
    hold_deadband_exit_deg: COMMISSION

  quiet_pitch:
    run_mode: position
    loc_kp: READBACK_AND_TUNE
    spd_kp: READBACK_AND_TUNE
    spd_ki: READBACK_AND_TUNE
    limit_current_a: COMMISSION

  dynamic_yaw:
    run_mode: speed
    host_kp_s_inv: COMMISSION
    max_velocity_deg_s: EXISTING_SAFE_VALUE
    max_acceleration_deg_s2: EXISTING_SAFE_VALUE
    max_jerk_deg_s3: EXISTING_SAFE_VALUE

  dynamic_pitch:
    run_mode: speed
    host_kp_s_inv: COMMISSION
    max_velocity_deg_s: EXISTING_SAFE_VALUE
    max_acceleration_deg_s2: EXISTING_SAFE_VALUE
    max_jerk_deg_s3: EXISTING_SAFE_VALUE

tracking_estimator:
  model: constant_angular_velocity

  process_noise:
    angular_accel_sigma_min_deg_s2: COMMISSION
    angular_accel_sigma_max_deg_s2: COMMISSION
    adaptive: true

  prediction:
    max_horizon_ms: 400
    actuator_delay_source: identified_profile

  innovation_gate:
    soft_chi2: 5.99
    hard_chi2: 9.21

  quiet_zone:
    enabled: true
    enter_deg: MEASURE_FROM_STATIONARY_TARGET
    exit_deg: GREATER_THAN_ENTER

vision:
  target_detector_fps: 30
  interim_min_fps: 20
  preview_fps: 10

  tracking:
    confirm_min_observations: 3
    confirm_visible_ms: 120
    coast_ms: 300
    lost_retain_ms: 3000
```

Do not automatically apply placeholder values.

The agent must refuse startup if a required commissioning placeholder remains.

---

# 56. Quiet HOLD pseudocode

```cpp
HoldOutput QuietHoldController::step(
    const AxisState& axis,
    TimePoint now)
{
    switch (state_) {
    case State::BRAKING:
        return makeControlledStop(axis);

    case State::SETTLING:
        stable_samples_.push(axis.position);

        if (abs(axis.velocity) < cfg_.entry_velocity &&
            stableFor(cfg_.settle_dwell)) {
            q_hold_ = robustMedian(stable_samples_);
            q_hold_ = quantizeToStableReference(q_hold_);
            state_ = State::HOLDING;
            return writeHoldReferenceOnce(q_hold_);
        }

        return makeControlledStop(axis);

    case State::HOLDING: {
        const double error = q_hold_ - axis.position;

        if (abs(error) <= cfg_.deadband_enter) {
            return HoldOutput{
                .position_reference = q_hold_,
                .rewrite_reference = false,
                .profile = HoldProfile::QUIET
            };
        }

        if (abs(error) >= cfg_.deadband_exit) {
            state_ = State::RECENTERING;
        }

        return holdAt(q_hold_);
    }

    case State::RECENTERING:
        if (nearHoldTarget(axis)) {
            state_ = State::HOLDING;
        }
        return slowBoundedPositionMove(q_hold_);
    }
}
```

---

# 57. AUTO_TRACK speed controller pseudocode

```cpp
DynamicTrackOutput DynamicTrackingController::step(
    const PredictedTarget& target,
    const AxisState& axis,
    const SafetyLimits& limits,
    double dt)
{
    if (!target.valid || target.confidence < cfg_.min_confidence) {
        return requestBrakeToHold();
    }

    const double e =
        wrapOrBoundedDifference(target.q_desired, axis.position);

    double v_raw =
        target.qdot_desired
        + cfg_.host_kp * e;

    if (insideVisualQuietZone(target, e)) {
        v_raw = 0.0;
    }

    const double confidence_scale =
        confidenceToScale(target.covariance);

    v_raw *= confidence_scale;

    const double v_position_safe =
        limits.velocityAllowedByStoppingDistance(
            axis.position,
            axis.velocity);

    double v_bounded =
        clamp(v_raw, -v_position_safe, v_position_safe);

    v_bounded =
        jerkLimitedVelocityUpdate(
            previous_velocity_command_,
            previous_acceleration_command_,
            v_bounded,
            limits.max_acceleration,
            limits.max_jerk,
            dt);

    previous_velocity_command_ = v_bounded;

    return {
        .run_mode = RunMode::SPEED,
        .speed_reference = v_bounded
    };
}
```

---

# 58. Target estimator pseudocode

```cpp
void AngularTargetEstimator::onMeasurement(
    const TargetObservation& obs,
    const MotorStateHistory& history)
{
    const auto pose =
        history.interpolate(obs.sensor_timestamp);

    if (!pose.valid) {
        counters_.timing_invalid++;
        return;
    }

    const Vec3 ray_c =
        camera_model_.pixelToRay(obs.anchor_pixel);

    const Vec3 ray_b =
        kinematics_.cameraRayToBase(ray_c, pose);

    const Vec2 z =
        rayToYawPitch(ray_b);

    if (!initialized_) {
        initializeAt(obs.sensor_timestamp, z);
        return;
    }

    if (obs.sensor_timestamp <= state_timestamp_) {
        handleOutOfOrder(obs);
        return;
    }

    propagateTo(obs.sensor_timestamp);

    const Mat2 R =
        measurement_model_.covariance(obs);

    const Innovation innovation =
        calculateInnovation(z, R);

    if (innovation.mahalanobis > cfg_.hard_gate) {
        counters_.hard_reject++;
        maneuver_monitor_.observeRejected(innovation);
        return;
    }

    update(z, R, innovation);
    adaptProcessNoise(innovation);

    state_timestamp_ = obs.sensor_timestamp;
}

PredictedTarget AngularTargetEstimator::predictForControl(
    TimePoint now,
    const ActuatorDelayModel& actuator)
{
    const TimePoint t_effective =
        now
        + actuator.command_delay()
        + actuator.response_lead();

    const Duration horizon =
        clamp(
            t_effective - state_timestamp_,
            Duration::zero(),
            cfg_.max_prediction_horizon);

    return propagateCopy(horizon);
}
```

---

# 59. Hardware experiment matrix

| Trial | Mode/condition | Purpose |
|---|---|---|
| A | motor disabled | mechanical/acoustic baseline |
| B | enabled, current mode 0 A | electrical/PWM baseline |
| C | present HOLD implementation | establish hum/jitter baseline |
| D | fixed latched LocRef, same gains | isolate q_meas-chasing effect |
| E | D with meaningful low LimitSpd | isolate 1e-6 clamp effect |
| F | E with lower `spd_ki` | test integral hunting |
| G | F with lower `loc_kp` | test stiffness |
| H | speed mode, 0 reference | characterize speed-loop zero behavior |
| I | speed mode small sine | identify speed response |
| J | position mode small sine | identify position response |
| K | AUTO_TRACK position path | current tracking baseline |
| L | AUTO_TRACK speed outer loop | candidate improved path |
| M | operation mode low Kp/Kd | later compliant-control experiment |

For every trial:

- yaw and pitch separately;
- unloaded and payload;
- restored parameters afterward;
- no limit changes;
- no thermal protection changes.

---

# 60. Acceptance criteria: quiet hold

Set project-specific thresholds after baseline, but require:

```text
no visible periodic oscillation
substantially reduced audible hum
lower iq RMS than current implementation
no unacceptable static sag
no repeated direction reversal
no temperature increase beyond accepted profile
hold survives a small external disturbance
```

Quantitative reporting:

```text
position RMS
position p2p
velocity RMS
iq RMS
dominant oscillation amplitude/frequency
temperature rise over 5–10 min
```

---

# 61. Acceptance criteria: dynamic tracking

Test target angular rates:

```text
10 deg/s
20 deg/s
direction reversal
stop/start
brief occlusion
```

Report:

```text
acquisition latency
p50/p95/max pointing error
phase lag
overshoot
reversal settling
target-loss stop distance
reacquisition response
motor current/temperature
```

Candidate speed mode should materially improve tracking error over position mode without worsening:

- limit safety;
- vibration;
- thermal behavior;
- target-stop overshoot.

---

# 62. Acceptance criteria: prediction

Prediction is accepted only if an A/B test shows:

```text
prediction OFF
versus
timestamp-correct prediction ON
```

with:

- lower p50/p95 tracking error;
- lower phase lag;
- no unacceptable overshoot on target stop/reversal;
- robust behavior under dropped frames;
- correct confidence reduction when measurements age.

Do not accept prediction because the HUD marker “looks ahead”.

---

# 63. Recommended implementation sequence for the local agent

## Phase 0 — freeze safety and baseline

- confirm existing signed safety invariants;
- record current config, firmware, gain readbacks;
- record hum/tracking baseline;
- create self-reverting trial harness.

## Phase 1 — CyberGear register correctness

- implement/read `loc_kp`, `spd_kp`, `spd_ki`;
- audit `spd_kd` naming;
- add readback verification;
- add firmware/encoder-calibration telemetry.

## Phase 2 — quiet HOLD

- latch fixed q_hold;
- remove q_meas-following hold;
- replace `1e-6` limit with commissioned low value;
- cache property writes;
- implement hysteresis;
- tune yaw/pitch hold profiles.

## Phase 3 — motor identification

- run position and speed small-signal tests;
- fit delay/time constants;
- measure reversal behavior;
- characterize resonance.

## Phase 4 — vision timing

- add per-stage timestamps;
- decouple preview;
- convert track state thresholds to time;
- achieve ≥20 fps, then approach 30 fps.

## Phase 5 — estimator correctness

- filter base-frame LOS at SensorTimestamp;
- implement CV Kalman;
- use detector-derived covariance;
- add innovation gate;
- predict to effective actuator time.

## Phase 6 — speed-mode AUTO_TRACK

- implement safe RunMode transition;
- implement host P + velocity feed-forward;
- no host integral;
- add safety velocity envelope;
- A/B against position mode.

## Phase 7 — identity and production detector

- unblock NN/rpk path;
- NMS;
- multi-object association;
- stable selected target;
- real target tests.

## Phase 8 — optional advanced control

Only if required:

- IMM/switching predictor;
- backlash take-up state;
- notch/input shaping;
- MIT mode;
- pitch torque feed-forward after IMU.

---

# 64. Explicit prohibitions

The local agent shall not:

- widen any safety limit to make a test pass;
- tune at or near a hard stop;
- change current-loop gains first;
- change thermal or over-current protection values;
- switch CyberGear mode while moving;
- treat `spd_ki` as `spd_kd`;
- persist unverified gains automatically;
- send stale LocRef after a brake/hold;
- use a 1e-6 speed clamp as the long-term hold design;
- chase every encoder count or bbox-centroid fluctuation;
- filter raw pixels without camera-motion compensation;
- calculate prediction from message-arrival time instead of `SensorTimestamp`;
- double-count camera latency in the prediction horizon;
- enable long-horizon blind extrapolation;
- declare multi-object tracking complete with the one-blob simple detector;
- implement MIT/torque mode before simpler paths are measured.

---

# 65. Research-supported conclusions

1. Gear-driven gimbal literature identifies backlash, friction, flexibility, and hysteresis as causes of degraded reciprocal tracking and closed-loop limit cycles.

2. Visual-servoing literature consistently identifies image acquisition/processing delay as a primary limit on tracking velocity and supports timestamp-aware predictive filtering.

3. Delayed visual measurements should constrain the state at capture time, not arrival time.

4. CyberGear exposes internal position P, speed PI, current-loop parameters, operation/position/speed/current modes, and diagnostic variables useful for identifying hunting.

5. CyberGear's manual requires stopping before changing control mode.

6. CyberGear uses a 14-bit magnetic encoder, implying approximately 0.022° per revolution count; this is consistent with the station's measured encoder step.

7. Operation/MIT mode can provide position, velocity, torque, Kp, and Kd in one frame, but the current OpenAutoTurret backend does not yet implement it.

8. No authoritative CyberGear backlash specification was found, so installed-system backlash must be measured.

---

# 66. Sources

## CyberGear

- Xiaomi CyberGear official product page:
  `https://www.mi.com/cyber-gear`

- Translated CyberGear instruction manual and protocol:
  `https://github.com/belovictor/cybergear-docs/blob/main/instructionmanual/instructionmanual.md`

- Independent open-source CyberGear SocketCAN driver demonstrating operation, position, speed, current, asynchronous feedback, and parameter access:
  `https://github.com/grrodre/cybergear`

- Xiaomi_CyberGear_Arduino driver:
  `https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino`

## Encoder

- ams OSRAM AS5047P information:
  `https://ams-osram.com/news/press-releases/unparalleled-accuracy-over-full-temperature-range-at-high-speed-in-latest-version-of-ams-47-series-magnetic-position-sensor`

## Gearbox/gimbal control

- Tang et al., “Combining Load and Motor Encoders to Compensate Nonlinear Disturbances for High Precision Tracking Control of Gear-Driven Gimbal,” Sensors 2018:
  `https://doi.org/10.3390/s18030754`

- Ramírez and Guesalaga, “A Friction Model Describing Limit Cycles in Positioning Servos,” 2009:
  `https://doi.org/10.1177/1548512909354614`

- Vered and Elliott, “Robust internal model control approach for position control of systems with sandwiched backlash,” 2023:
  `https://arxiv.org/abs/2307.06030`

## Visual prediction / delayed measurements

- Chroust and Vincze, “Improvement of the Prediction Quality for Visual Servoing with a Switching Kalman Filter,” 2003:
  `https://doi.org/10.1177/027836490302210008`

- Liu, Huang, and Wang, “Target Tracking for Visual Servoing Systems Based on an Adaptive Kalman Filter,” 2012:
  `https://doi.org/10.5772/52035`

- Ranganathan, Kaess, and Dellaert, “Fast 3D Pose Estimation With Out-of-Sequence Measurements,” IROS 2007:
  `https://publications.ri.cmu.edu/fast-3d-pose-estimation-with-out-of-sequence-measurements`

---

# 67. Definition of completion

This v3.2 hardening work is complete only when:

## Quiet drive

- HOLD uses a fixed latched target.
- `1e-6` hold speed behavior is removed/replaced.
- actual motor gains are readable and correctly named.
- hold hum/jitter/current are quantified.
- hold profile is quiet enough without unsafe sag.
- mode transitions are stopped, verified, and bumpless.

## Tracking

- real selected target confirms reliably.
- detector/preview path reaches ≥20 fps and is designed for 30 fps.
- all tracking state timing is elapsed-time based.
- target estimator updates in base-frame LOS at capture time.
- prediction horizon uses measured latency and actuator delay.
- prediction A/B test improves tracking.
- target loss and reacquisition remain stable.

## Motor control

- position-mode and speed-mode AUTO_TRACK are A/B tested.
- selected production mode meets tracking and quietness goals.
- v/a/j and stopping-distance safety remain authoritative.
- stale destinations cannot reappear after HOLD.
- all gain changes are reversible and payload-specific.

