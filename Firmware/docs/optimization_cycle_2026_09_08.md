# Optimization cycle and endpoint incident — 8 September 2026

**Physical operation suspended.** The launcher finished stopping the station at
19:25:33 NZST after parking failed. The subsequent investigation used saved
numeric logs, offline replay and simulated motors. No camera image or video feed
was inspected during this cycle. Do not activate the staged optimization release
or restart homing until the installed load and stopping arrangements have been
physically checked. A commanded zero speed or motor disable does not certify
that this front-heavy mechanism has stopped or will remain supported.

**Status: partially verified.** The controller corrections pass executable
software probes. Their physical braking behavior and the proposed faster
tracking configuration remain unverified. The optimization cycle has not reached
a defensible “hardware upgrade is mandatory” conclusion.

## Endpoint incident: observations and cause assessment

The last running diagnostic release was `c81a96fba5a6`, with a temporary Manual
startup configuration. Its launcher shutdown began at 19:21:46.878. The requested
parking speed was 3 degrees/second, with yaw moving first and then pitch.

| Numeric evidence | Observation |
|---|---|
| Yaw | An approximately 7.8-degree return took about 68 seconds before pitch movement began. Yaw overshot the park target by about 10.1 degrees. Its sampled position-derived speed reached 11.6 degrees/second. |
| Pitch lower boundary | At 19:23:14.252 the encoder was -1.389524 rad, beyond the -1.303784 rad soft minimum and close to the calibrated mechanical endpoint. |
| Pitch upper boundary | At 19:24:11.984 it was +0.000954 rad, beyond the -0.084405 rad soft maximum and close to the opposite calibrated endpoint. |
| Pitch speed | The recorded filtered estimate reached 1.127621 rad/s, approximately 64.6 degrees/second, during the nominal 3-degree/second parking sequence. |
| Supervision | The log contains 340 BRAKE transitions during shutdown. The sequence repeatedly resumed on ALLOW and eventually expired after about 226 seconds. |
| Final shutdown | `PARK FAILED: parking deadline exceeded`; launcher-controlled emergency-disable fallback and process exit followed. No verified PARKED result. |

The 2 Hz parking log does not resolve individual impacts or peak instantaneous
speed. It does establish travel beyond both pitch soft limits. The user reported
endpoint impacts; an independent output-angle or contact sensor was not available
to inspect damage or certify the mechanism's final position remotely.

![Numeric parking record](../../../run/optimization-20260908/endpoint-motion.png)

Two definite software defects were found:

1. Parking passed integral gain **0.002**, leaving the proportional gain at its
   default **1.0**, to the drive mode-transition recipe. Normal service and the
   configured homing profile use **4.0 / 0.05**. The recipe writes and verifies
   those requested registers. Parking therefore changed the loaded drive's
   control behavior. This mismatch is a plausible contributor to the recorded
   delayed motion and overshoot; changing it has not yet proved physical causality.
2. BRAKE was temporary during park moves. Returning ALLOW resumed the move.
   The parking command gate also rejected **Stop Motion**, despite its intended
   unconditional stop semantics. A stalled or unstable drive could consequently
   remain in the parking sequence until the long travel-based deadline.

The previous simulator followed speed commands immediately and assumed zero
speed held position. It could not expose the physical drive/load behavior. An
earlier launcher stop in this cycle had also failed near a boundary. That failure
should have blocked further physical trials; continuing after it was an error.

## Controller corrections and proof

The candidate changes are in [control_loop.cpp](../control/src/control/control_loop.cpp)
and [command_validation.hpp](../control/src/web/command_validation.hpp):

- Preserve configured service drive gains during parking move and verification
  mode setup. Neutralize both axes while sequential setup is pending, so the
  other axis cannot retain its preceding tracking speed command.
- On every parking control cycle, check measured soft-limit containment. After
  mode setup, check filtered measured speed against the larger park/verify speed
  plus 2 degrees/second: **5 degrees/second for this station**.
- Check the active axis against its starting-position-to-target interval, with
  a 1-degree tolerance. Check the waiting axis against its starting position,
  and the completed axis against its park target, with the same tolerance.
- Latch a controlled-stop fault on unexpected travel, overspeed, invalid motor
  feedback, or a parking safety action stronger than DERATE. A later ALLOW
  cannot resume parking. Home cannot bypass explicit motor recovery after this
  motion fault. Hard-fault and watchdog disable authority remains in force.
- Accept Stop Motion and Hold during parking and cancel the sequence into the
  same latched stop. Do not turn cancellation into another parking attempt.
- Include parking speed commands and position-derived speed (`vest`) in the
  existing bounded numeric trace. A motion fault logs both axes' measured pose,
  estimated speed, previous command and soft limits once.

These checks issue a zero-speed stop through the existing speed-mode backend.
If drive mode setup is still pending, cancellation uses the existing disable
path to cancel that recipe: ordinary references are suppressed during setup.
They do **not** independently stop a drive that ignores that command or support
an unpowered load. Launcher termination still retains its documented disable
fallback. Neither the guard thresholds nor the braking model are newly certified
physical limits. The release remains inactive for that reason.

The [production-loop probe](../tools/probe_parking_motion.cpp) first failed on
the old implementation: gains became 1/0.002, an injected inactive-axis drift
accumulated to 5 degrees, injected overspeed did not latch, and Stop Motion did
not cancel parking. With the correction:

| Executed probe | Result |
|---|---|
| Drive setup arguments | 4.0 / 0.05 retained |
| Inactive pitch disturbance, 0.5 degrees/second | Latched at approximately 1.0025 degrees; zero speed commanded on both axes; no automatic restart |
| Injected measured overspeed | Detected within 40 ms in this synthetic case |
| Recorded shutdown position replay | Yaw overspeed latched at approximately 16.27 seconds, versus the first logged pitch boundary crossing at 87.37 seconds |
| HTTP through real controller and web processes, simulated motors | Successful park and verification-only failure lifecycle retained; Stop Motion accepted during parking, both speed commands zero, fault remained latched, Home rejected |
| Cancellation during pending mode setup | Both axes disabled, no subsequent mode-transition calls, fault latched |
| C++ regression suite | 72/72 CTest entries passed |

Replay interpolates the saved 2 Hz position log onto controller ticks and ignores
the resulting commands. It demonstrates earlier detection, **not a counterfactual
proof that the physical collision would have been prevented**. The HTTP probe
opens no physical devices and disables video.

Offline reproduction from `Firmware`, using the project virtual environment:

```bash
build/probe-parking-motion
PYTHONPATH=. ../run/station-venv/bin/python tools/probe_park_service.py \
  --controld build/control/controld --output ../run/park-safety-service
```

An optional whitespace-separated `seconds pitch_rad yaw_rad` numeric replay can
be passed as the sole argument to `probe-parking-motion`.

## Latency optimization evidence collected before stopping

The [initial analysis](latency_bottleneck_analysis_2026_09_08.md) remains the
baseline component budget. The follow-up used a bounded six-second fixed-angle
`response_probe` through the production tracking reference, host servo and
motor backend. It bypassed target selection and camera direction input. Entry
required Manual/Hold, fresh feedback, no fault, stationary commands and at least
15 degrees of soft-limit clearance. Original current, speed, acceleration,
jerk and watchdog limits were retained. This does not make an unsuccessful stop
safe; the subsequent endpoint incident takes precedence over the earlier trials.

Sixteen steps were accepted across five captures; twelve had at least 5.8 seconds
of uninterrupted ALLOW evidence. Adverse and incomplete trials are retained.

| Tested reference | Selected real encoder evidence | Decision |
|---|---|---|
| Original omega 2.5, 1-degree steps | Yaw t90 about 1.58–1.84 s; pitch about 2.08–2.29 s in these trials | Substantial reference/loaded-axis delay exists independently of recognition. Sparse directional trials are not a latency percentile. |
| Constant omega 4, 1-degree steps | Yaw t90 about 1.13–1.57 s; pitch about 1.43–1.92 s | Improvement in some cases, but this does not establish stability across step sizes. |
| Constant omega 6 | Some positive 1-degree steps reached t90 in about 0.71 s; other trials were interrupted by control-cycle DERATE | Rejected as a deployable setting on this evidence. |
| Constant omega 4, 5-degree steps | One completed step overshot by 1.43 degrees. Another was cancelled and continued moving afterward: observed overshoot reached **2.28 degrees**, total excursion **7.28 degrees**. | Reject a simple global increase to omega 4. Do not omit motion after cancellation. |

[analyze_response_cycle.py](../tools/analyze_response_cycle.py) now reports
overshoot through the available post-cancellation observations as well as the
active-trial interval. The 2.28-degree result includes later numeric state
samples after trace collection aborted. It is not a certified final maximum:
the recording ends about 3.28 seconds after that trial started.

The unactivated optimization candidate `f8999584f4bb` contains:

- **Adaptive reference response:** requested omega 4 is reduced for larger
  position error according to available acceleration, retaining the original
  lower bound 2.5. Actual C++ reference probes showed no overshoot across signed
  0.5/1/5/10/25-degree steps. For a 1-degree software reference, t90 fell from
  about 1.58 to 1.03 seconds; for 5 degrees, from about 1.67 to 1.37 seconds.
  These are reference-generator results, not new physical measurements.
- **Acquisition handoff:** 250 ms to 50 ms, preserving confirmation, selection,
  distinct-frame and stale-evidence checks. A synthetic production-loop handoff
  reached tracking in 125 ms versus 325 ms from the same supplied selection.
- **Coordinate conversion:** batch geometry agrees exactly with the installed
  Picamera2 SDK for 9,072 tested combinations. An actual-adapter synthetic
  100-box comparison fell from approximately 6.6 to 3.2 ms. First-frame SDK
  equivalence checking and fallback remain. The live pipeline saving has not
  been measured because this candidate was not activated.

Normal deployment remains AUTO_ROAM. Bench Manual overrides were disposable
configuration files and were not persisted into the shipped default.

## Object → recognition → Pi, and the hardware decision

The diagnostic release exported latest-only numeric camera timing with no image
inspection. Its stationary live baseline measured:

| Boundary or component | Median | p95 |
|---|---:|---:|
| Sensor timestamp → wire completion | 52.89 ms | 65.73 ms |
| Camera request return → wire completion | 13.23 ms | 14.86 ms |
| Pi image copy | 1.94 ms | 2.28 ms |
| Pi perception pipeline | 10.62 ms | 12.06 ms |
| IMX500 DNN KPI | 13.57 ms | 13.95 ms |
| IMX500 DSP KPI | 12.36 ms | 12.37 ms |

Exposure was 38.173 ms in a 38.450 ms frame, approximately 26 Hz. These
overlapping stages and different sample populations must not be summed.
Object appearance is not the sensor timestamp: frame phase, rolling exposure
and the first correct recognition still require a clocked optical stimulus
and independent transition timing. That optical test was designed in the
baseline report but has not been performed. No real target controlled a trial.

**Recommendation:** retain Pi 5/IMX500 for now. Restore safe and repeatable loaded
motion before trying to shorten it. Then validate adaptive reference behavior,
acquisition policy and host geometry savings against agreed settling/error and
recognition requirements. The present evidence does not show Pi compute or
neural inference as the primary bottleneck.

The existing detector runs on the IMX500 sensor. AI HAT+ 2 is a different Hailo
model/pipeline path; its advertised 40 TOPS INT4 is not this detector's measured
latency. It cannot repair drive overspeed, load imbalance or parking supervision.
See [Raspberry Pi AI Camera documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)
and [AI HAT+ 2 documentation](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html).

Mechanical support, balance, mount integrity and a dependable stopping method
are the immediate physical investigation. Independent output sensing is already
required by the station's parking-release contract and is currently absent.
If the load cannot remain supported when drive torque is removed, passive
support or a suitable brake is needed before relying on shutdown. Motor,
transmission or accelerator replacement cannot be specified from these records
alone. A mandatory performance upgrade also needs an agreed angular-step,
settling tolerance, target angular speed and detection-quality objective.

## Evidence custody and restart prerequisites

Local evidence is under `run/optimization-20260908/`: raw numeric trials,
camera summaries, all interrupted runs, full parking logs, replay input,
derived summaries, plot and software probe results. Captures and virtual
environments remain outside Git. Transferred captures were hash-checked before
Pi deletion; roughly 36.6 MB of this cycle's captures and the older latency
capture directory were removed. Large build/source artifacts in an older
release were also reclaimed while preserving an operator's working directory.

The endpoint archive SHA-256 is
`e99aa2536e5fcc1d5fe962e48864a8bb6204d83cd4f06aa0a864c2bc82366049`.
The extracted controller log SHA-256 is
`e97f9f0aaa4adb4faccd2e57644fa11d4a792b607e802ce0f7814f6bb3bc26ab`.

Before renewed physical work, the endpoint impacts and unpowered support must
be checked at the station. Then a restrained, observable bench must establish
that both axes obey a zero-speed stop and remain inside a narrow interior range,
including after cancellation. An on-site means of stopping/supporting the load
is required because the current motor feedback/command path cannot independently
guarantee that outcome. No automatic homing or parking retry should be used to
discover whether the problem has cleared.
