# Roam recovery direction

## Decision

Tracking interrupts a coverage sweep. Automatic loss recovery resumes the
interrupted sweep direction instead of choosing a boundary from current yaw on
every return. This implements the last-roam-direction option. It does not infer
a lost target's travel heading.

Previously, leaving roam erased the planner direction and re-entry chose the
nearest end. Tracking interruptions on one half of the envelope could repeatedly
send the camera back toward that end, reducing coverage. The direction was a
function of pose, not intrinsically clockwise; both signs can exhibit the defect.

## State and priority

1. Supervisory and safety authority remain above all motion modes.
2. Existing forbidden-yaw-gap repositioning retains its explicit legal-interior
   direction. Ordinary loss recovery does not replace that waypoint.
3. On an accepted ROAM -> TRACK transition, ControlLoop records the active
   planner's direction in `interrupted_roam_dir_`. This is desired sweep direction,
   not measured motor velocity or the last telemetry event's direction.
4. After the existing continuous loss dwell and anti-hunt gate, automatic return
   enters roam with that preference. Entry uses current feedback and the current
   validated envelope. At/within reach tolerance of an end it chooses inward;
   outside the sweep bounds it approaches the nearest end before continuing
   inward. Direction cannot authorize motion outside the bounds.
5. Without an interrupted leg, use the existing nearest-boundary entry rule.

Manual, STOP, and leaving supervisory Ready clear coverage memory. The memory is
process-local and is not retained calibration or configuration. Explicit operator
entry into roam starts a fresh nearest-boundary sweep. Accepted repeated requests
for the same mode do not replace the saved direction. Rejected requests do not
change it. Each subsequent ROAM -> TRACK transition records the latest leg,
including direction changes made at a turnaround.

`RoamPlanner::exit()` still clears the active planner direction. Idle and tracking
telemetry therefore continue to report no active sweep. The next sweep uses a new
waypoint and the existing reference/trajectory/safety path; no old trajectory,
velocity, acceleration, or target reference is restored. Pitch reference and speed
limits remain configuration controlled. Acquisition, coast, loss, and anti-hunt
timings are unchanged.

`ROAM_RECOVERY` logs the source (`interrupted_sweep` or `nearest_boundary`),
requested direction and effective entry direction. A boundary recovery can make
the effective direction differ from the saved preference. Existing telemetry
reports the active waypoint and direction.

## Why target heading is not used in this revision

The published tracking confidence bands describe whether a selected identity is
trustworthy enough to follow. They are not confidence bounds on the sign of target
angular velocity. Camera-relative displacement also includes turret motion, and
motor direction can reflect tracking error correction rather than target motion.
By the automatic loss transition, the latest measurement may be stale and the
estimator may no longer support prediction.

A future heading preference needs a separately validated contract: accepted
measurements from one identity, turret-motion compensation in a common frame,
enough displacement and time support to resolve the sign above uncertainty,
bounded evidence age through coast/loss, invalidation on ambiguity or identity
change, and an explicit mapping into the legal bounded yaw branch. Low-confidence,
stationary, reversing, stale, and unreachable cases must fall back to the sweep
direction. Detection confidence or last motor speed alone is insufficient.

## Verification contract

Design hypothesis: direction survives the actual automatic-mode hand-off and
produces continued coverage through the production reference path.

The executable probe is `RoamMode.AutomaticLossResumesInterruptedSweep` in
`test_control_loop`. It homes simulated axes through the production homing path,
runs a sweep until it is travelling away from its nearer end, enters tracking
without a target, waits for the real automatic loss timer, and checks direction,
waypoint and simulated movement. It repeats for +1, -1, +1. The original code
fails the direction/waypoint checks in both signs; the candidate passes.

Additional regressions exercise automatic acquisition with fresh TrackSets then
stale-target loss, Manual/STOP reset, explicit operator entry, both end boundaries,
outside-region approach and inward continuation, and changed envelope bounds.
Existing turnaround, forbidden-gap planner, safety, mode and control-loop tests
remain applicable.

Validation on 2026-09-07 used an isolated Pi build of baseline `f7cf9b8` plus
this change, with source hashes checked against the Windows workspace. The
final run passed all 8 roam integration tests, all 17 planner tests, and the
complete 66-entry CTest suite (which re-exercised the probe after hardening).
Logs are retained locally under ignored `run/roam-recovery-validation/`.
No release was activated and no hardware motion was exercised by these tests.

Run from a configured simulation build:

```bash
cmake --build build --target test_control_loop test_roam_planner -j2
build/control/test_control_loop --gtest_filter='RoamMode.*'
build/control/test_roam_planner
ctest --test-dir build --output-on-failure
```

This is partial system verification: production control code and simulated motors
are exercised, not live camera tracking or loaded physical motion. A future live
acceptance run must use the station launcher/deployment runbook. Observe an
acquisition/loss on each sweep leg away from its end, compare `ROAM_RECOVERY` with
the pre-track direction and actual resumed yaw, and confirm subsequent travel
toward both ends. Include a target loss near a boundary and a Manual/STOP reset.
Do not infer hardware validation from the simulation result.
