# Service motion profiles

`motion.modes` gives **each** of MANUAL, AUTO_ROAM and AUTO_TRACK a `maximum`
and a `target` pair. Speeds are magnitudes in degrees/second, accelerations in
degrees/second². Jerk is in degrees/second³. The loader converts to radians
once; runtime code uses radians throughout.

| Mode | Target speed | Target acceleration | Maximum speed | Maximum acceleration |
|---|---:|---:|---:|---:|
| Manual (Fast/base) | 20 | 15 | 20 | 30 |
| Auto Roam | 10 | 15 | 20 | 30 |
| Auto Track | 20 | 30 | 20 | 30 |

Target values constrain planned motion, not a demand to move continuously at
that rate. Maximum values constrain the final speed servo and permit bounded
position-error correction. Roam retains correction headroom. Auto Track's
target speed and acceleration now equal the existing service ceilings, as
requested; this removes extra speed headroom at its maximum planned rate.
Tracking target jerk remains 100; Manual/Roam target jerk remains 60. Maximum
servo jerk remains 120. Drive gains, estimation and prediction tuning are
separate from these motion profiles.

## Configuration and precedence

All three mode entries and both complete speed/acceleration pairs are
required when `motion` is present. Each pair can explicitly set `jerk_deg_s3`;
otherwise the mode defaults above apply. Values must be finite and positive,
and each target rate must not exceed its corresponding maximum. Unknown keys
and mixed legacy/new motion settings fail loading. The supported service
envelope remains 20°/s, 30°/s², 120°/s³; exceeding it requires a separately
validated engineering change. `v3.service_speed_control: true` is required so
the acceleration contract is enforceable through the host servo.

Per-axis overrides use the same pairs, for example inside `auto_track`:

```yaml
axes:
  pitch:
    maximum: {speed_deg_s: 8, acceleration_deg_s2: 12, jerk_deg_s3: 120}
    target: {speed_deg_s: 6, acceleration_deg_s2: 10, jerk_deg_s3: 100}
```

The override supplies both pairs. An omitted axis inherits its mode's pairs;
an override never mutates the other axis or another mode. For each axis:

1. Intersect mode maximums with `axes.<axis>.max_*` and applicable payload
   `v_max_rad_s`, `a_max_rad_s2`, `j_max_rad_s3` caps.
2. Clamp targets to those effective maximums. Payload mismatch derating reduces
   speed and target acceleration; it retains maximum braking acceleration.
3. Apply the intent's confidence/preset/handoff scales and supervisor derating.
4. Shape the reference using target limits. Follow it using maximum limits.
5. Bound the final signed command using measured pose, estimated actual
   velocity, command acceleration, and the same capped acceleration/jerk
   supplied to the servo. The existing 200 ms response reserve and 0.05 rad
   positional reserve remain.

The shared scalar reference interface carries the larger per-axis speed;
each axis is independently constrained before reference generation and motor
commands. A slower pitch no longer silently caps yaw. Positive and negative
boundary speed limits are computed separately, including during reversals.

Manual Fine/Normal/Fast retain their existing multipliers: speed
0.15/0.45/1.0, target acceleration 0.25/0.60/1.0, target jerk
0.40/0.80/1.0. They apply to Manual's target pair, and manual correction speed
remains capped to the selected target rate. The servo retains the Manual
maximum acceleration for following and stopping. Hold is a stationary intent
inside the owning mode, with bounded correction authority, not a zero-valued
configuration profile.

Reducing a mode/preset speed while already moving requests deceleration;
carried velocity is not instantaneously reset or clipped to the new speed.
The incoming speed can therefore be exceeded transiently while decelerating.
The fixed service ceiling and measured-pose boundary still apply. A newly
reduced hard acceleration cap takes priority over jerk continuity. Safety
Brake/Disable actions retain their existing override path and are not delayed
by ordinary smoothing.

## Migration and scope

This implements the **service-mode stage** of consolidation. The production
YAML replaces `tracking.hold_speed_deg_s`, `track_speed_deg_s`,
`track_acceleration_deg_s2`, `track_jerk_deg_s3`, `search_speed_deg_s`,
`v3.service_max_speed_deg_s`, and `v3.auto_roam.velocity_deg_s` with the new
profiles. Old configuration files without `motion` retain the legacy path.
Runtime wiring and startup logging use the resolved profiles; the old fields
are not competing tuning controls in a new configuration.

Homing, parking, payload verification and fault stopping are supervisory
operations, not additional operator modes. Their existing commissioned
settings and execution paths are preserved, including coarse/fine/backoff
homing speeds 5/3/3°/s and park/verification speeds 10/2°/s. Giving their
position-mode/direct-velocity paths a new acceleration contract is the next
stage and requires separate physical validation; this change does not pretend
they already enforce the service profile contract.

## Observability and validation

During speed-controlled service, `motion_profile.pitch` and `.yaw` telemetry
contain `configured` and `effective` target/maximum pairs, signed boundary
speed limits, and `limit_reason`. Reasons identify mode, axis, payload,
derating, intent scaling, boundary limiting, and deceleration into a lower
speed. Outside that path the object is null. The legacy scalar speed-ceiling field remains for
older clients; per-axis telemetry is authoritative for explaining a limit.

Offline checks exercise YAML -> production wiring -> simulated homing ->
native target input -> reference generation -> motor commands. They verify
20°/s commands and 30°/s² tracking reference acceleration, independent lower
pitch limits, transition into a lower Manual ceiling, lease-expiry stopping,
and boundary reserves with 50/120/200 ms synthetic motor lag in both directions.
Configuration checks cover malformed/missing pairs, typoed keys, nonfinite
values, unsupported maximums, axis overrides, and legacy conflicts.

These are software/simulation results. Full-speed stopping under the installed
3 kg payload and moving-target overshoot have **not** been established by this
change. No new source was activated on the station. Commissioning must use the
normal committed-release/launcher workflow, retain calibration validation,
and measure stopping in both directions on each axis before describing these
settings as verified physical maxima.
