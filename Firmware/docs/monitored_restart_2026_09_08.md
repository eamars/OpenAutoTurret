# Monitored restart after operator clearance — 8 September 2026

**Outcome: restarted once, then automatically stopped on unexpected pitch
motion.** The user said the station was safe to operate again. That clearance
was used to start the previously verified `680b13a62dc0` release through the
launcher, with a temporary Manual startup configuration. The stack was confirmed
stopped afterward. No camera image or video feed was inspected; no target or
tracking stimulus controlled this test.

The restart monitor used the controller's existing numeric socket, both encoder
positions and their feedback timestamps. It rejected measured speed above
8 degrees/second, stale feedback, controller faults and unexpected startup modes.
It remained active during normal homing. Its calculations had first accepted
329 saved small-step telemetry states and rejected injected excessive motion.

## Physical observations

| Time, NZST | Observation |
|---|---|
| 20:01:34.060 | Controller starts; release `680b13a62dc0`; temporary Manual config |
| 20:01:35.197 | Existing recovery gate confirms both drives disabled and healthy; normal homing begins |
| 20:01:35.849 | Pitch approaches its first endpoint at a commanded +5 degrees/second |
| 20:01:37.578 | First coarse contact settles near +0.00515 rad |
| 20:01:38.447–41.188 | Position-mode backoff; measured speed remains around or below 4.3 degrees/second in the active-axis log |
| 20:01:41.694 | Pitch is settled near -0.04864 rad, with reported holding torque approximately -1.8 N·m |
| Approximately 20:01:41.982 | During the next drive-mode transition, fresh encoder evidence is +0.002098 rad: about **2.91 degrees** back toward the prior endpoint. The monitor estimates **17.54 degrees/second** over its recent encoder window and aborts. |
| 20:01:41.987 | Controller accepts Stop Motion during homing; motor disable and calibration invalidation occur |
| 20:01:41.992–42.209 | Launcher shutdown de-energizes and exits; no new park move or homing retry |

The active-axis homing log pauses during asynchronous mode setup. The monitor
therefore caught a movement absent from that log's final settled samples.
There is no completed homing calibration or independent output-angle observation
for this run. These records establish an encoder excursion during setup, not a
new independent impact/damage measurement.

The event is consistent with losing holding torque while switching from the
position-mode backoff to the fine speed-mode approach. It is not evidence of
slow object recognition, Pi compute saturation or a tracking-reference failure.
The cause remains an inference: the source records the stop/re-enable recipe,
but individual transmitted CAN frames and independent physical angle were not
captured during this run. The operator subsequently described the payload as
front-heavy and the near-endpoint braking distance as inadequate. They also
required that yaw homing account for other load conditions. That description
must not be used as a fixed load model for either axis.

## Why another blind restart is inappropriate

The [supplied Xiaomi manual](CyberGear微电机使用说明书.pdf), page 2, requires a
stop before switching control mode and prohibits switching while the joint is
running. The production backend accordingly sends stop, writes/verifies setup,
then enables. Removing that stop would contradict the documented interface and
would not be a validated repair.

The missing evidence is bounded setup motion and adequate clearance on both
axes. The review found software gaps in backoff clearance and both-axis
supervision; see the [general homing review](homing_failure_review_2026_09_08.md).
If torque-off motion cannot be contained, support or brake changes may be needed.
That is not established solely by the load description. A faster Pi cannot
supply holding torque while a drive is disabled. The 1.8 N·m sample is
insufficient to size a brake for all poses and disturbances.

## Additional backend correction and offline proof

The asynchronous mode recipe previously checked register values and feedback
age but did not bound position movement after stop. It could update its pinned
position after drift and proceed to enable. A fresh-looking COMM_TYPE_2 sample
could also retain the position from before the load moved.

The correction in [can_motor_backend.cpp](../control/src/control/can_motor_backend.cpp):

- Preserves the last position before stop and rejects reported movement greater
  than 0.25 degrees throughout the remaining setup sequence.
- Requires fresh finite encoder-register (`MechPos`) readback immediately before
  enable; rejects displacement greater than 0.25 degrees or missing readback.
- Uses the existing failure path to cancel the pending recipe and disable.
  It neither retries nor bypasses the stop-before-mode-change requirement.

The [executable probe](../tools/probe_mode_motion_guard.cpp) runs the actual CAN
backend, register request/response matching and protocol encoding against a
simulated motor transport. It does not open physical hardware. Before the fix,
the backend sent enable even with 2.9 degrees of simulated stopped-axis drift,
and never requested the encoder position register before enable.

After the fix, both position- and speed-mode cases passed:

| Simulated condition | Result |
|---|---|
| Stationary axis, fresh position | Transition completes; one enable and a pre-enable position read |
| 2.9-degree drift visible in feedback | Transition fails; zero enable frames |
| 2.9-degree drift hidden behind frozen position feedback | Fresh encoder read detects movement; zero enable frames |
| No encoder-register reply | Transition fails; zero enable frames |

The initial backend correction passed **73 CTest entries locally**. The later
both-axis work is documented in the general homing review. This proves the
backend's command gating under injected evidence. It does not prove physical
load holding, prevent an unpowered fall, or establish that the motor obeys a stop.
This additional correction has not been exercised on the physical mechanism.

## Evidence and next boundary

Evidence is local under `run/optimization-resume-20260908/`: the startup monitor,
failure state, full controller log, extracted numeric samples, old/new backend
probe results and test output. The controller log SHA-256 is
`2cf174acc1c42cce377db76f5ac06ea3c10e935d15557cf4a04a8e9f7de37127`.
The temporary configuration changes only `v3.default_mode` to Manual. Captures
and the project virtual environment remain outside Git.

Further physical tuning depends on establishing bounded motion and adequate
clearance through torque-off/setup under the installed load conditions.
The safety monitor's 8-degree/second threshold will not be raised to make this
run pass. A new physical trial must demonstrate bounded pose and stationary
feedback through setup before small interior response tests resume.
