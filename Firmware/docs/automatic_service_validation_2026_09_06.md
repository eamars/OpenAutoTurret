# Automatic camera station: implementation and validation

Historical validation record. Current operation:
[station runbook](STATION_OPERATIONS.md). Later loaded-control evidence:
[travel boundary review](travel_boundary_review_2026_09_06.md).

This continues [the initial takeover review](implementation_takeover_2026_09_06.md).
The user confirmed an unloaded camera/sensor station, attended the physical trials,
and asked to prioritize smooth rotation over further acoustic tuning.

## Operation

On `rpi-turret`, from `/home/eamars/workspace/OpenAutoTurret/Firmware`:

```bash
bash scripts/run_application.sh        # detached start; no mode arguments
bash scripts/run_application.sh status # from another shell
bash scripts/run_application.sh stop   # park, disable, and stop owned processes
```

Use **http://rpi-turret:8080/** for the real camera feed and controls.

- Startup validates retained calibration against the configuration, motor IDs,
  live energized feedback, drive mode/gains and calibrated travel. If validation
  fails, it homes before entering service. Normal stop invalidates calibration.
- After homing, Auto roams at a planned speed of 10 degrees/s. Tracking has a
  15 degrees/s planning ceiling, with a 20 degrees/s motor ceiling for recovery.
  Confidence, payload and travel limits can reduce these speeds. A fresh
  selected target starts tracking. Sustained loss returns to roaming and releases
  the missing selection so a new target can be acquired. Targets outside calibrated
  travel hold the station and report TARGET_UNREACHABLE.
- **Manual / Hold** stops automatic behavior. The D-pad appears only in Manual
  while the controller is in Hold. Press and hold an arrow to jog; release to stop.
  Jog commands have a 300 ms lease, renewed by the page. Focus loss also stops jog.
- **Auto** explicitly resumes automatic roaming and acquisition.
- **MENU → Home → Confirm Home** recalibrates both axes. The second press must
  occur while the confirmation is displayed. Use the launcher stop command for
  full shutdown; Manual / Hold does not abort supervisory homing.
- Green boxes are camera detections. The small amber prediction cross is the
  controller's actual projected tracking point, without decorative displacement.

The launcher owns its controller, camera process and web service. The Python
runtime is the project-local `run/station-venv`; no global packages were installed.

## Motion and integration changes

Service modes use the CyberGear speed loop with a host position correction and
trajectory velocity feed-forward. Planned speeds stay below the motor ceiling
to leave recovery authority. Reference acceleration is limited to 15 degrees/s²
and jerk to 60 degrees/s³. Motor-command shaping allows 30 degrees/s² and
120 degrees/s³, within the safety envelope; applying the same ramp twice delayed
the response and produced a surge after each reversal.
The host has no integral term, bounds its position correction, and latches quiet
hold with hysteresis. Runtime service gains are SpdKp 4 and SpdKi 0.05. Homing and
parking retain SpdKi 0.002. Current ceilings remain pitch 3 A and yaw 1 A.

The previous service planner could run at 10 degrees/s while the motor command
was capped at 3. This accumulated a large position error and delayed reversals.
The matched-speed live run reduced maximum reference error from 32.77 to 2.74
degrees on yaw and from 8.26 to 2.01 degrees on pitch. The two trials used different
live human movements: these measurements demonstrate the removed mismatch, not
a controlled comparison of tracking accuracy or a claim that all jitter is gone.

The final estimator candidate uses measurement sigma 0.012 rad, acceleration sigma
0.20 rad/s² and 120 ms motor response allowance. These are provisional unloaded
settings, not a fitted actuator model. Further acoustic tuning was stopped at the
user's request.

Motor mode transitions now progress across control cycles: brake, confirm stillness,
disable, configure and verify registers, then enable and confirm fresh feedback.
The hardware transition probe completed pitch/yaw speed-position-speed transitions
in 303–480 ms, with a maximum individual call of 0.0415 ms. Cancellation inhibited
late enable. A separate production watchdog checks heartbeat and motor feedback
every 5 ms, latching stop after a 100 ms failure. It cannot act after host process
death or through a physically broken communication link.

Two live-only perception failures were repaired: equivalent UUID sessions with and
without hyphens now match, and the camera's 180-degree mounting correction is
applied at the sensor. This keeps neural input, preview and detection geometry
consistent. A 260-frame sensor-orientation probe retained one person throughout;
the real preview was checked for upright imagery and aligned boxes.

Retained homing uses an owner-checked, checksummed cache in pinned tmpfs memory at
`/dev/shm/ota-homing-<uid>`. It survives application restarts and never enables a
motor. Reuse requires fresh evidence that both motors are already energized and
within the saved travel. Every commanded disable invalidates it before Stop.
Controlled application restart trials preserved position to within 0.022 degrees
pitch and 0.044 degrees yaw. A full Pi reboot discards the cache and conservatively
requires homing. Cache reuse across instances and rejection of a different motor
ID are covered by the cache regression test.

Two intermediate starts stopped during homing, before service gains were applied.
The second captured a 5.66-second host cycle stall; the independent watchdog
stopped both axes when the inactive axis's feedback age crossed 100 ms. The exact
blocking site was not reproduced on the next start. Additional invalidation
timing and first-trip watchdog diagnostics remain enabled. Moving the cache from
an SD-backed mapping to pinned tmpfs removes a possible filesystem writeback wait
from the motor-disable path; it is not evidence that the historical stall's cause
has been conclusively identified.

## Live aiming failure and correction

The first no-argument production trial exposed a failure hidden by checking only
reference error. Pitch hardly moved and yaw could pursue the wrong direction,
even while telemetry said TRACKING. Replaying the actual captured values
`az=2.90598, el=0.927651, yaw=0.590715, pitch=-0.69295` demonstrated the cause:
12 unscaled gradient steps failed the solver tolerance, after which the analytic
fallback returned yaw `+2.90598`, pitch `+0.643145`. That is the opposite joint
branch; pitch lies outside this station's negative travel.

Damped Gauss-Newton now converges on yaw `-0.235613`, pitch `-0.643145` for the
same input. The motion resolver no longer falls back to the unrelated analytic
branch. It refuses solutions outside calibrated travel. Reachability is checked
from the current prediction while holding as well as moving; the earlier code
read a result that had already been cleared, making every target look reachable.
Regression cases cover this exact recording, both yaw directions across the
station's pitch range, malformed angles, and out-of-travel hold.

The Home confirmation failure was also reproduced in the rendered UI: comparing
`HOME` with the changed text `CONFIRM HOME` meant a second press never submitted
anything. Confirmation now matches the stable command ID. The earlier failed
Home attempt is not counted as a successful homing trial.
After the fix, both presses on the actual web page produced `start_homing`
ACCEPTED, cleared the valid limits, completed the physical homing sequence and
returned through Auto Roam to Auto Track. Evidence is in
`run/service-web-home-start-fixed.json` and `run/service-web-home-completed.json`.

## Evidence and acceptance limits

### Uninterrupted horizontal motion

Following the user's observation that motor smoothness must be judged separately
from an unstable prediction, two 95-second horizontal roam recordings were made
with target acquisition suppressed. Both kept a 3 degrees/s reference on the
straight portions, reported ALLOW throughout, and completed without motor faults.
Encoder rate was calculated over 200 ms to limit quantization noise. Turnarounds
and startup were excluded from the rate comparison.

| Runtime SpdKp / SpdKi | Straight rate error RMS | Absolute reference error p95 |
|---|---:|---:|
| 1 / 0.05 | 0.929 degrees/s | 2.829 degrees |
| 4 / 0.05 | 0.688 degrees/s | 2.421 degrees |
| 4 / 0.02 | 0.952 degrees/s | 6.009 degrees |
| 5 / 0.05 | 0.681 degrees/s | 3.453 degrees |

The higher proportional gain reduced measured speed ripple by 26%, but visible
periodic acceleration remained. This is an improvement, not smooth-motion
acceptance. Plots and metric summaries are in
[`evidence/2026-09-06/long-roam-01.png`](evidence/2026-09-06/long-roam-01.png) and
[`long-roam-kp4-02.png`](evidence/2026-09-06/long-roam-kp4-02.png).
The first trial paused the camera process and it did not recover afterwards;
subsequent trials set `track_on_acquire_ms: 0` in a temporary configuration while
keeping the camera live. This override must not become the normal station config.
Reducing integral gain to 0.02 worsened both ripple and lag and was rejected.
Kp 5 made essentially no further improvement in ripple and increased lag, so
Kp 4 / Ki 0.05 was selected. All four completed sweeps reported ALLOW throughout.

The launcher cleanup now waits for the controller to park and exit, then gives
remaining nonmotor children five seconds to exit before terminating them. The
motor controller is never killed by that timeout.

### Prediction uncertainty

The measurement covariance now passes through the camera and current capture-time
joint geometry. The previous code used image-angle variance directly as world
azimuth/elevation variance. At a recorded pitch of -0.29 rad, a pixel-noise Monte
Carlo probe produced 2.03 degrees of yaw uncertainty, versus the previous assumed
0.78 degrees. The new Jacobian gives 2.018 degrees for the same pixel uncertainty.
The diagonal estimator conservatively includes the magnitude of cross covariance.

A stationary synthetic 25 Hz replay through the actual C++ estimator reduced
predicted-image radial RMS from 39.47 to 14.15 px. This demonstrates the conversion
error under controlled noise; it is not a live tracking accuracy measurement.
The geometry regression covers the station pose and azimuth wrapping.

### Camera motion compensation

A separate scene-flow check during uninterrupted roaming found that the stored
camera rotation omitted a pitch boresight offset. Its signs looked plausible,
but it predicted strong image roll during almost horizontal scene motion. The
median residual across 136 background-feature pairs was 15.89 px. Detected people
were excluded from the feature mask.

Two stopped yaw moves at pitch approximately -0.697 rad fitted an additional
rotation of -0.860 rad about the pitch frame's Y axis. Independent yaw moves at
pitch approximately -0.64 rad reduced median feature residuals from 72.54 / 53.45
px to 8.23 / 10.80 px. Pitch moves retained their direction with 4.30--5.33 px
residuals. This corrects the pose transform used by both estimation and aiming,
not just the displayed cue. It is a provisional rotational correction;
translation/parallax and lens distortion remain unmeasured.

The old near-vertical geometry also explains the large azimuth uncertainty in
the synthetic covariance probe above. Covariance propagation remains necessary
for arbitrary camera poses, but those old numerical values are not a claim about
the corrected station's ready pose.

Raw correspondence records are `run/roam-projection-geometry.json` and
`run/dwell-projection-geometry.json` on the Pi. The new rotation and provenance
are recorded in `calibration/camera_extrinsics.yaml`.

The following 90-second normal-launch recording entered Auto Track, recovered
through Roam on losses, and remained free of motor faults. A further 60-second
seated-subject recording stayed in Auto Track throughout. The user still observed
prediction movement, so these runs do not constitute stable-prediction acceptance.
On request, further work shifted back to actuator smoothness and tracking speed.

### Tracking speed follow-up

The initial 3 degrees/s commissioning cap was too slow for nearby people. Service
speed is now an explicit validated `v3.service_max_speed_deg_s` setting, default 3
and bounded at 20 degrees/s. The control loop now receives the configured
`tracking.hold_speed_deg_s`; previously that setting reached the tracker but the
actuator's hold-speed ceiling stayed at its compiled 10 degrees/s default.
The configured roam speed also now survives the full intent-to-reference path.
Previously the roam controller received it, but the reference resolver replaced
it with the shared search speed. The intermediate recordings named `speed10` and
`catchup10` actually contain **20 degrees/s references**; their filenames are not
evidence of a 10 degrees/s trial.

The corrected 95-second `headroom10` sweep kept a true 10 degrees/s reference
under a 20 degrees/s motor ceiling. It removed persistent straight-line lag but
exposed the identical second ramp in the motor servo. A second 95-second trial,
`servo30`, allowed the motor command to respond at 30 degrees/s² and 120 degrees/s³
while keeping the reference at 15 degrees/s² and 60 degrees/s³.

| True 10 degrees/s roam | Straight rate error RMS | Position error p95 | Maximum position error |
|---|---:|---:|---:|
| Identical reference and servo ramps | 1.732 degrees/s | 2.903 degrees | 5.374 degrees |
| Quicker servo response | 0.553 degrees/s | 0.730 degrees | 1.174 degrees |

Both runs completed without faults and reported ALLOW throughout. Rate uses
200 ms encoder differences, excluding startup and turnarounds. The measured rate
error fell 68%, but the retained restarts centered the sweeps at different yaw
poses, so this is not a perfectly matched mechanical-sector experiment. Residual
speed ripple remains. The operator subsequently reported that faster live tracking
was much better and motion was much smoother, while reporting excessive tracking
overshoot. That accepts the improvement in motor response, not overall tracking
stability. The subsequent [boundary isolation report](tracking_instability_isolation_2026_09_06.md)
records separate measurement, geometry, estimator and planner probes, including a
reproduced and corrected software limit cycle.
See [the full trace](evidence/2026-09-06/long-roam-servo30.png) and
[comparison with source hashes](evidence/2026-09-06/roam-servo-response.json).

The normal configuration now uses 10 degrees/s roam, 15 degrees/s tracking and
20 degrees/s motor recovery authority, with automatic acquisition restored to
250 ms. A motor speed is not a demonstrated person-tracking bandwidth. At closest
approach, a person walking sideways at 1 m/s needs approximately 29 degrees/s at
2 m range, or 14 degrees/s at 4 m. The current planned tracking ceiling therefore
cannot keep up with every nearby walking person. Camera estimation and confidence
derating further affect the achieved response.

Regression checks cover configured hold/service speed conversion and the full
control-loop path retaining a named roam speed below the shared search ceiling.
The final temporary-config stop parked and disabled both motors cleanly before
the normal no-argument script was launched with the faster configuration.

Baseline commit is `d643b7b`. Changes are uncommitted in the local checkout and
deployed to `/home/eamars/workspace/OpenAutoTurret` on the Pi.

- Pi CMake build and all **66 CTest targets passed**, including the projection covariance regression.
- **780 Python tests and 14 subtests passed** across perception, vision and web.
- Actual web/IPC/controller trials verified automatic acquisition, tracking,
  coasting, loss and return to roam. The default-start physical trial also acquired new UUIDs after loss.
- Four physical jog directions were accepted after correcting the command grammar
  to `axis:profile`. An earlier probe with `axis profile` was rejected and is not
  counted as a successful jog test.
- Normal script stop has physically parked and disabled both motors.

Raw earlier-cycle evidence is in the isolated Pi checkout
`/home/eamars/workspace/OpenAutoTurret-takeover-20260906/run/`:
`real-auto-ki05-02.json`, `real-auto-speed-matched-01.json`,
`service-ki05-manual-jogs-fixed.json`, `warm-restart-result.json`,
`mode-transition-physical.log` and `hardening/servo-*-ki05-01/`.
Local copies of key telemetry are under `run/takeover-evidence/automatic-service/`.
Final-checkout build/test logs are `run/service-{cmake,build,ctest,pytest}.log`.

The two architecture plans remain only partially accepted. Full labeled detector
quality/crossing tests, production YOLO acceptance, controlled prediction A/B,
loaded payload trials, drive-side communication timeout verification and an
external measurement of mechanical jitter remain outstanding. SSD is the current
live detector; its thresholds remain provisional.
