# Travel range, end approaches and HUD — 2026-09-06

**Status: partially verified.** The UI has been exercised in the browser using
recorded station data. The bounded solver and service boundary governor have
executable probe evidence; full-loop simulated roaming covers both ends. The
pitch homing backoff failure recorded below has since been resolved in a slow
loaded trial. Physical full-speed end approaches remain a separate verification
gap. No calibration was adopted after a failed homing attempt.

## Changes

- Pitch presentation subtracts the midpoint of the calibrated soft travel.
  The tape, diagnostics and field-of-regard use the same offset. At the last
  successful calibration, the display endpoints are −34.76° and +34.76°.
  Positive remains the existing joint direction (camera downward). Raw control,
  calibration and API angles retain their existing coordinate system.
- The optical-axis reticle and amber prediction square/cross use 3 px strokes
  with dark outlines. Prediction remains distinct from measured detection and
  disappears when invalid. Its source remains the actual controller prediction.
- `v3.auto_roam.full_yaw_travel: true` derives the sweep from homed yaw limits.
  Each sweep endpoint keeps the 0.05 rad service stopping reserve plus 0.5°.
  Arrival tolerance is 0.4°; reversal requires half that positional tolerance.
  With the previous calibration this requests approximately −158.57° to
  +177.43°, a 336.00° sweep. These are usable software endpoints inside the
  mechanical stops, not permission to traverse a full circular revolution.
- A service boundary governor caps the final signed velocity independently of
  target direction. It uses the actual service servo bounds (30°/s² and
  120°/s³), measured position-derived velocity, current commanded acceleration,
  a 200 ms response allowance and the existing 0.05 rad reserve. Acceleration
  building speed toward an end tapers with clearance; braking authority remains
  available. A newly inward goal cannot remove the cap on still-outward motion.
- Reference speed depends on clearance where the axis is moving, rather than
  slowing an entire traversal merely because its destination is near an end.
- The LOS solver searches valid bounded joint branches. A reachable equivalent
  yaw at the other end is preserved instead of wrapping it back outside the
  limits. A direction in the forbidden gap remains unreachable.
- A fresh goal requiring more than 180° of legal yaw travel initiates
  `AUTO_ROAM / YAW_REPOSITION` toward the other end through the legal interior.
  Loss during that transit does not restart a nearest-end search. Fresh,
  reachable observations are required to reacquire Auto Track. A long occlusion
  does not guarantee identity continuity. Manual remains an explicit override.
- Auto Track's planning speed is 15°/s with 20°/s service recovery headroom.
  A subsequent operator request increased tracking-only reference acceleration
  from 15 to 25°/s² and jerk from 60 to 100°/s³. The final servo and boundary
  governor remain at 30°/s² and 120°/s³. See the loaded trial below for limits
  on what has been physically verified.

The service governor supplements the supervisor. It does not run during contact
homing, replace feedback/deadline watchdogs, or derate emergency braking.

## Evidence

1. Before this change, `run/range-review-tracking-before-01/capture.json` on the
   Pi recorded 90 seconds of continuous person tracking without fault. There
   are 1,285 distinct telemetry timestamps among 3,382 reads and 742 preview
   frames. Position-reference errors were approximately 0.3° p95 on both axes;
   total measured yaw/pitch spans were 2.36°/2.80°. The telemetry stream has a
   maximum 2.71 s sampling gap, so this is not a high-frequency jitter claim.
   Person/anchor movement is not stationary-target ground truth.
2. `run/probe_boundary_governor.cpp` drove the production governor and servo
   into simulated first-order plants with 50 ms and 150 ms lag, continuously
   requesting a position beyond each end. All four runs stopped at the
   2.865° software reserve. This does not establish the installed motor's
   worst-case stopping distance under every load.
3. `run/probe_bounded_yaw.cpp` recovered the correct bounded joint pose for all
   nine combinations of start/goal yaw −159°, 0°, +178°, and rejected 190° in
   the forbidden gap. Regression coverage adds rotated camera extrinsics.
4. `TrackingIntegration.FullTravelRoamReversesAtBothEndsWithoutTouchingReserve`
   runs the actual ControlLoop against SimMotorBackend for 80 simulated seconds,
   covers both safe endpoints and asserts the 0.05 rad reserve throughout.
5. The browser check on a separately labelled recorded fixture verified the
   thicker cues, pitch 0° at the midpoint, and diagnostic limits ±34.8°. This
   fixture had no motor connection. The normal station was not represented as
   operating while its motors were disabled.
6. The final build passed all 66 CTest groups, including the new full-loop
   sweep, bounded branch and braking regressions. The 222-test web run found
   one obsolete assertion requiring 1.2 px prediction strokes; after updating
   it to the requested 3 px, all nine prediction tests passed. The eight HUD
   tests also passed, including midpoint and missing-calibration behavior.

## Physical commissioning interruption

Changing configuration invalidated the retained calibration as designed. The
normal stop/start scripts were used. Startup reached the first pitch endpoint,
but the 5° position-mode backoff timed out after 15 seconds:

| Trial | Final pitch | Backoff target | Result |
|---|---:|---:|---|
| Existing homing gains | −0.008965 rad | −0.086313 rad | Timeout; disabled |
| Accepted service gains in backoff only | −0.009346 rad | −0.086313 rad | Same timeout; disabled |

The second trial kept the 3 A current cap, target, speed and timeout unchanged.
It falsified the proposal that restoring service speed-loop gains would resolve
this failure; that code experiment was reverted. Logs show roughly −1.9 Nm
reported effort without useful backoff motion. This does not by itself prove a
mechanical obstruction, a drive defect or a particular control-law defect.

After shutdown, `run/range-disabled-registers-01/before.yaml` confirms pitch
LimitCur 3 A, LocKp 30, bus voltage about 24 V and zero filtered current while
disabled. Disabled snapshots do not prove the register values during motion.
The failed logs are preserved as `run/range-homing-backoff-failed-01.log` and
`-02.log`. The operator was asked to inspect the arm/cable at the endpoint.

### Retry with the added payload

On the operator's request, the unchanged 3 A pitch profile was retried through
the normal launcher (`run/range-activation-03.json`). Pitch again failed its
15-second backoff: final −0.008965 rad versus target −0.085931 rad, with about
−1.9 Nm reported effort while nearly stationary. The fault disabled both
motors. At the end of that attempt the controller/web remained available in
Fault with a live camera feed (approximately 26 fps).

The operator confirmed an **added 3 kg, front-heavy camera payload** and requested
slow homing. A fixed 5 A pitch limit, 5°/s coarse approach and 3°/s fine/backoff
profile still failed with stock homing gains 1/0.002; reported backoff torque
was only about 0.5 Nm. Increasing the current cap alone did not resolve that
slow-profile failure. Giving homing independent speed-loop gains 4/0.05 at the
same 5 A cap completed both axis ranges and the final pitch repeat in 318 s.
The capture contains no fault, valid calibrated limits, and ends in Manual/Hold.
The largest 0.25–0.4 s position-window speeds were 7.31°/s pitch and 6.62°/s yaw;
command caps are not claims that the physical drive never overshoots its rate.
Evidence: `run/loaded-homing-05a-gains.json` and its controller log.

The trial also temporarily reduced service to 3°/s and roaming to 2°/s. The
operator correctly identified this unwanted coupling. The normal station YAML
now retains slow homing but restores tracking 15°/s, service 20°/s, roam 10°/s,
and automatic startup. Tracking acceleration and jerk have separate bounded
settings; homing no longer determines these service settings. Detection,
estimation, velocity confidence gating, damping and prediction horizon were
not changed for this acceleration trial.

The production reference plus speed-servo probe uses first-order motor lags
50/120/200 ms with exact moving targets at ±3/10/15°/s, followed by abrupt stops.
The 25/100 candidate settles in all cases and reduces peak following lag, but
does not eliminate overshoot: worst modeled stop overshoot is 4.35° versus
5.76° with 15/60; at 15°/s and 120 ms lag it increases from 0.75° to 1.72°.
These are synthetic target and plant results, not measured installed-load
stopping distances. Evidence: `run/tracking-acceleration-probe.csv`.

Remaining physical checks include bounded approaches on both axes in both
directions, a full horizontal Roam sweep, and opposite-end reacquisition with
this payload. Do not infer these from the unloaded, simulated or recorded UI
checks, and do not bypass invalidated calibration to resume service.

### Normal launcher and restored tracking verification

The no-argument launcher then repeated loaded homing in approximately 317 s,
with no fault and valid soft limits, and entered automatic service. Its first
sampler stopped on an assertion because it saw `Hold` just before the one-shot
Auto handoff; a subsequent live read confirmed `AUTO_TRACK` and the restored
20°/s service ceiling. The sampler now waits for the handoff itself. This was
an observation timing error, not a homing failure.

Public web/API 5° manual steps completed in both directions on both axes.
Within each manual segment, observed target overshoot was at most 0.14°;
95th-percentile actual-to-reference error was 0.32–0.40°. One proposed pitch
step was rejected by the probe's 10° interior guard before being sent; the
later pitch trial began with a move inward. Operator mode changes end a manual
segment in the analysis, so subsequent roaming is not counted as step motion.

A separate 60 s read-only observation recorded 1,397 reads / 905 distinct
telemetry timestamps, automatic roam/track transitions, no fault, and zero
reported deadline misses. Both tracking reference acceleration peaks reached
25°/s². Of 545 active tracking samples, actual-to-reference error p95 was
2.87° yaw and 0.62° pitch. This confirms the higher setting is exercised;
it does not establish negligible moving-target overshoot or eliminate the
remaining visual tracking instability. The camera continued at about 26 fps.

The normal browser showed the live feed, detection box, amber controller
prediction, centered pitch scale and stronger reticle. The direction pad was
visible in Manual and absent in Auto. Final CTest passed all 66 groups,
including both tracking response settings and independent homing configuration.
The offline replay's 300 s total homing budget was insufficient for the slow
full-range plan; it is now 600 simulated seconds. Production per-action
timeouts, contact checks and motor watchdogs still apply.

Compact results are committed-file candidates under `docs/evidence/2026-09-06/`:
`homing-tracking-separation-summary.json`, `restored-tracking-service-summary.json`,
`restored-tracking-service-02-summary.json` and `restored-tracking-readonly-summary.json`.
Full captures and logs remain in `run/` on the station.
