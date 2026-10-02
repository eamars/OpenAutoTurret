# ADR-003 implementation: plan, decisions and status

Owner of the implementation: Claude, from 2026-10-02, at the station owner's request. The
architect's package in this directory ([README](README.md), [ADR-003.md](ADR-003.md),
[docs/](docs/), [contracts/](contracts/)) is the design guidance and stays as delivered (its
checksums cover it). This file records how the work is carried out, where the code or the station
said otherwise, and the state of each work item.

## Rulings this work follows

- **Owner, 2026-10-02: accuracy limits come from measured performance.** The yaw accuracy limits
  are calibrated from the real tracking performance of the FF+FB controllers. This replaces the
  photography template's "do not infer tolerances from achieved performance" for this station.
  Three things keep it honest:
  - the rule (1.25 × the worst pass of a fixed campaign) and the campaign are fixed before any
    data is taken;
  - nothing is repeated or dropped;
  - a re-calibration happens only after a deliberate hardware change.
- **Owner, 2026-10-02: tracking mode is limited only by safety.** The cap is 100 RPM, and
  AUTO_TRACK's speed and acceleration are not limited by a motion profile. The tracking asset's
  Level-1 limits are therefore physical capability:
  - **Yaw:** 100 RPM. Acceleration is half of (3 A peak − 1 A worst friction) / inertia 0.041,
    which is 24.6 rad/s². Jerk is half of the servo's 200 A/s current slew / inertia. The other
    half is left for feedback.
  - **Pitch:** 100 RPM, with the 60°/s² its commissioning demonstrated, because its capability
    under the 3 kg payload is unmeasured.
  - **Enforcement:** the backend trips the yaw servo above 100 RPM, and the pitch servo's command
    clamp is the same cap.
- **Owner, 2026-10-02: never drive the pitch into its mechanical end stop.** Three layers protect
  it, independently of each other:
  1. Level 1 clamps its target to this boot's soft envelope (5° inside the homed ends) and brakes
     in time.
  2. Inside the 1 kHz pitch servo, a travel governor caps the commanded speed toward either end
     so that a stop at the demonstrated 60°/s² fits before a guard 2° beyond the soft limit
     (`axis::travel_governor`, unit-tested).
  3. If the measured pitch is ever past that guard, SpdRef goes to 0 and the axis faults.
- **Earlier owner rulings still hold.** The working product comes first and the ADRs are guidance.
  Speeds stay below 100 RPM. Yaw current is 3 A peak and 1.62 A continuous. Repeat measurements,
  because the bearing is inconsistent. Do not fall into try-and-error loops: analyse the recorded
  data before running the station again.

## The code as it stands (re-audited on 0007dbe, 2026-10-02)

[01_SOURCE_AUDIT.md](docs/01_SOURCE_AUDIT.md) read a public snapshot. The local tree adds:

**Production does not run the ADR-002.2 servos.**
- Yaw is a GM6020 in CAN current mode. A host velocity PI (0.8 A, 200 Hz, 50 ms filtered speed)
  drives it from `SpeedServo`.
- Pitch is a CyberGear in speed mode (SpdRef). `mixed_hardware.yaml` labels it "position".
- `axis_control_core` (Servo, PositionLoop) is linked into controld but used only by commissiond.

**The prediction lead is stacked.**
- The estimator is extrapolated by capture age + 20 ms + 120 ms (`motor_response_ms`) at a rate
  shrunk by 2σ.
- That rate is fed forward again through `track_reference` and the `SpeedServo` feedforward.

**Smaller gaps.**
- The pose history is sampled once per 200 Hz tick, not per CAN frame.
- No record joins the LOS estimate, q_ref, q and the observation time. Only the 15 Hz web
  snapshot carries LOS fields, and nothing persists it.
- The observation time is libcamera's SensorTimestamp (exposure start, first row), passed through
  without an exposure or row correction.

## Decisions where the implementation differs from the package

1. **The tracking core is its own library,** [`tracking_core/`](../../tracking_core/), the
   sibling of `axis_control_core`. Without spdlog or the control loop, commissiond (3a), controld
   (3b) and the simulator all link the same objects. The old `control/src/tracking/target_estimator`
   is removed when W4 switches production (one owner, D01).
2. **The reference is a constant-jerk segment.** Level 1 publishes {q, v, a, j} at each 5 ms tick.
   The 1 kHz servos evaluate that polynomial, and the next tick integrates it to its actual time.
   The ADR's single integral (sec. 5.2) therefore holds across the two rates, without
   interpolating or differentiating anything.
3. **The lead limit has hysteresis.** Beyond `lead_limit` of reference over axis, the reference
   stops advancing and holds. It releases at half the limit and continues toward the *current*
   target, so no old waypoints are replayed (sec. 5.2).
4. **The servo's accuracy limits do not block a working servo.** Commissioning writes the asset
   whenever the servo works (tracking, stall and stability gates). It reports conformance to
   `config/servo/yaw_accuracy.json` separately. After a payload change, refusing the new working
   servo would leave the old, wrong one in place. A shortfall is a capability finding instead:
   inspect the hardware, or re-calibrate after a deliberate change. The limits gate ADR-003
   qualification.
5. **Stage-1 checks test properties.** Each checks the property its scenario must show (the
   "必须证明" column), with bounds derived from design quantities: the coast horizon H, the
   settling time 4/λ, the pixel noise, and the reference's own stop and braking distance. Framing
   errors, lags and peaks are reported against the photography budget, not invented as gates.

## Work items

| Item | State | Evidence |
|---|---|---|
| Yaw accuracy limits (owner request) | **done 2026-10-02** | [`config/servo/yaw_accuracy.json`](../../config/servo/yaw_accuracy.json), [`config/tracking/photography_spec.json`](../../config/tracking/photography_spec.json), `commission.py calibrate` |
| W1 stage 1: math, interfaces, simulator, 14 scenarios | **done 2026-10-02** | `tracking_core/` (+ `test-tracking`), `tools/tracking/` (`tracking.py stage1`, pytest), [stage-1 report](reports/STAGE1_2026-10-02.md) |
| W2 stage 2: timing, noise, q_a, λ from data | **timing done 2026-10-02**; noise, q_a, λ need a subject | [`config/tracking/camera_timing.json`](../../config/tracking/camera_timing.json), `tracking.py timing` (below) |
| W3 stage 3a: independent program on the station | next | commissiond plus the unchanged visiond: real camera, then tracking_core, then both ADR-002.2 servos |
| W4 stage 3b: production | **built 2026-10-02, not yet on the station** | details below |

**Stage 3b in production (controld).** Software is done and tested offline; physical 3b has not
started.

- **The ADR-002.2 servos own both axes** at Hold in every mode. The mixed backend:
  - steps the yaw `Servo` on every GM6020 frame;
  - runs the pitch `PositionLoop` at 1 kHz;
  - takes one constant-jerk reference segment per 5 ms tick through `command_reference`;
  - keeps the production guards in front of the servos and the commissioning trips behind them
    (following error, oscillation, temperature, 100 RPM, end-stop guard).

  Homing, braking, emergency stop and fault keep the legacy paths, and any legacy command
  releases the servo.
- **AUTO_TRACK runs on the tracking core** (`tracking.core` in `turret_mixed.yaml`). One estimator
  works at each frame's measured optical time, and the observation age is the only lead: the
  configured 140 ms is not used. Level 1 is the only reference, within the limits ruled above.
  The 2-σ-gated legacy lead is no longer on this path.
- **Per-tick trace:** `OTA_TRACKING_TRACE` holds one JSON line per Level-1 tick, with the goal,
  the reference, its flags, e_track and e_servo.
- **Tests:**
  - *`test_reference_servo`:* against the commissioned plant model, it follows a smooth move,
    releases on a legacy command, holds a stale segment, and trips on following error.
  - *`test_tracking_integration`:* the full stack following a 10°/s subject. The core's worst
    late error is 0.05°; the legacy estimator with its lead and `track_reference` reaches 2.09°.
  - *`travel_governor`:* covered in `servo_core`.
- **Still to do for 3b:**
  - the V2 observation wire, so visiond sends each frame's exposure and optical time;
  - removing the legacy estimator once 3b qualifies;
  - the station runs themselves.

## Stage 2: camera timing (2026-10-02)

**Method.** `tracking.py timing` runs a fixed plan. The yaw follows two sines (±5 deg, about
15 deg/s peak) under the commissioned servo, with pitch disabled at its centre. Meanwhile the
tracking camera records with production's own opener and profile. This runs twice: once with the
camera's auto-exposure, once at a fixed 8 ms.
- **Per frame:** the horizontal flow between consecutive frames (global Lucas-Kanade) is fitted
  against the encoder's angle increments at a scanned delay, in six row bands.
- **Rolling shutter:** the bands give the row time.
- **Exposure term:** the two exposures separate it.
- **Gates:** R², the band scatter about the rolling-shutter line, and the exposure model. Nothing
  is written unless they hold.
- **Offline proof:** synthetic frames with a known delay are recovered within 2 ms.

**Result.**

| Quantity | Value |
|---|---|
| Model | t_o = SensorTimestamp + 3.4 ms − 0.33 × exposure + 11.6 µs × row |
| Uncertainty | ±3.9 ms |
| Clocks | CLOCK_BOOTTIME equals CLOCK_MONOTONIC on the station |

**The stamp is not what libcamera documents.** Its definition (start of exposure) predicts
+exposure/2 to the image centre; the measurement says the stamp behaves like start-of-frame. At
the room's 33 ms auto-exposure:
- the frame centre is imaged 1.2 ms *before* SensorTimestamp;
- production's raw stamp is therefore within about 1 ms of right;
- the package's assumed +exposure/2 would have been 16.5 ms wrong.

**Methods that failed on this scene (2026-10-02), recorded so nobody repeats them.**
- *Phase correlation* locked onto a zero-shift peak: a low-texture wall, 33 ms blur, and a near
  stand with large parallax.
- *Frame-difference energy* is biased by about a third of the exposure. Consecutive blurred frames
  also differ by their blur width.

**For the photography budget.** The room's auto-exposure runs at the full 33 ms frame time. At
15 deg/s that is about 12 px of motion blur in the 1920×1080 frame.

## What needs the owner

- **A moving subject for the real-optics runs** (W2's noise and q_a, and the moving scenarios of
  3a and 3b). The detector sees people only. A person walking at a few repeatable speeds, or a
  screen showing a person, is enough. Everything that can run without one (timing calibration,
  static scenes, injected delay, drops and saturation) is done first.
