# Response tuning follow-up — 8 September 2026

This follows the [optimization cycle](optimization_cycle_2026_09_08.md).
Physical experiments use fixed angular commands and numeric motor telemetry.
No camera image or video is inspected and no detected subject drives a trial.

**Physical result:** homing completed on `adf447e0b5c5.cmdOMu` in approximately
6.3 minutes, with valid soft limits, Manual readiness and no fault. Forty
target-free step trials then exercised 0.5/1/5-degree commands in both axes and
directions. Thirty-eight completed their full six-second measurement interval;
one was interrupted by a cycle-timing intervention and one by excessive reverse
yaw excursion. The selected operating profile is adaptive reference omega 4,
position-servo gain 4. Homing speeds, current limits and inner drive gains are
unchanged by this tuning selection.

| Physical test | Encoder t90 | Active overshoot | Result |
|---|---:|---:|---|
| 0.5 degrees, original omega 2.5 / gain 3, four directions | 1.57–3.02 s | 0.09–0.22 degrees | Baseline |
| 0.5 degrees, omega 4 / gain 3, four directions | 1.20–2.37 s | 0.05–0.13 degrees | Faster reference helps |
| 0.5 degrees, omega 6 / gain 6, four directions | 0.85–1.40 s | 0.07–0.18 degrees | Insufficient basis for general deployment |
| 1 degree, selected omega 4 / gain 4, four directions | 1.03–1.56 s | 0.11–0.20 degrees | Completed |
| 5 degrees, conservative omega 2.5 / gain 2, four directions | 1.52–1.96 s | 0.05–0.27 degrees | Comparison baseline |
| 5 degrees, selected omega 4 / gain 4, four directions | 1.20–1.54 s | 0.14–0.18 degrees | Completed |
| Repeated reverse 5-degree yaw / pitch at selected profile | 1.41 / 1.29 s | 0.11 / 0.18 degrees | Completed |

These are individual observed ranges, not statistical p95 estimates. The final
0.5-degree positive yaw/pitch checks at gain 4 took 2.08/0.94 s respectively,
showing residual pose/friction sensitivity. Settling uses a separate +/-0.2-degree
window; t90 only means sustained crossing of 90% displacement, not settlement.

The aggressive omega 6 / gain 6 profile is rejected: a repeated 1-degree pitch
move overshot by 0.75 degrees, and the reverse 5-degree yaw move exceeded the
seven-degree excursion observer. Stop Motion retained Manual service; subsequent
readback showed zero service-rate commands, healthy feedback and valid limits.
Recorded yaw overshoot including post-cancellation motion was 3.28 degrees, with
8.28 degrees total excursion. This is a tuning failure, not a successful fast
response. No inference or target selection drove these angular commands.

The selected 5-degree reference itself reached t90 at about 1.36 s, close to the
1.20–1.54 s encoder response. The remaining response is substantially shaped by
the command trajectory and installed drive/load dynamics. These tests do not
establish that all software optimization is exhausted or that new hardware is
mandatory. Faster aggressive gains already lose overshoot control. Detection
quality and appearance latency require the separate evidence described below.

**Configuration history:** the 21:16 attempt on `da2f55eeb787.KcQiPy` tripped
the added homing speed gate during coarse approach; the homing fault path
disabled both drives. The operator then required these checks to warn and
continue. `homing.motion_checks_abort: false` now makes added speed, corridor
and reverse-motion findings warnings, while feedback and drive health remain
fatal. The separate mode displacement switch remains false. The next physical
probe, rather than another broad test suite, was the acceptance gate. That
attempt reached yaw's second endpoint but exposed another added stationary-window
deadline. `adf447e0b5c5` extends the same override to endpoint arrival/settling,
restoring the earlier timed-settling and backoff procedure; its physical home
then completed.

## Homing configuration and first result

Release `e3cacb2d1d6c.Rm5qjM` ran one monitored Manual startup at 20:58.
Recovery passed; homing stopped at pitch coarse-contact to backoff mode setup.
The last settled encoder position was +0.00553 rad and the fault sample was
-0.00134 rad: approximately -0.394 degrees, away from the positive approach.
During the preceding approximately 496 ms of logged settling, position stayed
at the same rounded value and reported effort ranged from 2.537 to 2.956 N.m.
The 0.25-degree mode-displacement gate latched a fault. The launcher stopped.
Calibration was invalid and no response trial ran in that attempt.

This is compatible with mechanical recoil when effort is removed; the record
does not separate preload, compliance, backlash and external load. It does not
establish a universal load direction, independent output angle or final stopping
distance. The 217 ms between the last settle and fault log samples includes
mode setup; it is not a measured disable interval or guard reaction time.

The operator then requested disabling the check to proceed with response tuning.
Commit `da2f55eeb787` adds `homing.mode_displacement_check`, with default true
when omitted and explicit false in the station configuration. Only the
quarter-degree displacement gate in homing mode transitions is omitted,
including the final transition into service mode. Both-axis speed/corridor
supervision, current/torque limits, feedback and drive-fault checks, disabled
confirmation, finite pre-enable encoder readback and the watchdog remain.
Parking calls retain the displacement gate. Normal startup remains AUTO_ROAM;
physical tests use a disposable Manual configuration.

The production CAN backend probe passed 44 simulated transport cases, including
signed 0.4-degree recoil with the override, missing readback and ignored stop.
All 74 local CTest entries passed. These results establish configuration and
protocol behavior, not physical homing success.

## Hardware decision includes detection speed and quality

The user's two AI upgrade objectives are valid independently of motor response:
shorter object-appearance to first prediction, and more reliable person versus
non-person detection. Neither is measured by the target-free motor experiments.
The current numeric setup also cannot measure the first objective: a sensor
timestamp is not an independently timed appearance event. An inference KPI
cannot establish time to first correct recognition. Detection confidence alone
cannot establish accuracy without labels. No optical or labelled-scene test is
claimed here.

The earlier recommendation to keep Pi 5/IMX500 is therefore limited to the
measured host/control workload. It does **not** show that perception hardware
is adequate for the user's two objectives, or that a replacement cannot help.
Likewise, larger TOPS figures do not establish either improvement.

Before choosing an accelerator, compare candidate detector models on matched,
labelled inputs with motor commands disconnected. Record person precision and
recall, false person detections on non-person examples, missed detections by
size/lighting/occlusion, and unique prediction cadence. Match recall when
comparing false positives; raising a threshold can otherwise hide missed people.
Keep frame ingestion, neural/DSP runtime and postprocessing timestamps separate.
Appearance-to-first-correct-prediction remains an external acceptance test
requiring independently timed optical input; motor tuning is not its substitute.

There is still an existing-camera comparison path. Raspberry Pi's model zoo lists
COCO detection mAP 0.218 for SSD MobileNetV2 FPNLite 320 and 0.374 for YOLO11n
640. These are aggregate benchmarks, not station person accuracy or a latency
comparison. [Official IMX500 model zoo](https://github.com/raspberrypi/imx500-models).
The station already has a hash-pinned YOLO11n comparison contract. Its prior
reference run produced 241 tensor frames from 482 camera frames in 30 seconds,
about 8 predictions/s despite requested 16 camera fps. Quality remains
uncommissioned. See [the recorded comparison](implementation_takeover_2026_09_06.md#yolo11n-reference-model).
Do not replace the SSD profile blindly or equate camera fps with prediction fps.

For conventional object detection, shortlist the 26-TOPS AI HAT+ alongside AI
HAT+ 2 and compare the actual supported detector. Raspberry Pi describes HAT+ 2
vision performance as broadly comparable to its 26-TOPS predecessor. HAT+ 2 adds
8 GB memory and generative-model support; its 40-TOPS specification is INT4.
That alone does not justify preferring it for person detection.
[Raspberry Pi's HAT+ 2 announcement](https://www.raspberrypi.com/news/introducing-the-raspberry-pi-ai-hat-plus-2-generative-ai-on-raspberry-pi-5/).
The Hailo route requires compatible compiled models and ingestion integration;
it does not accelerate the existing sensor-side model automatically.
[Official AI HAT documentation](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html).

An AI upgrade becomes necessary for an agreed requirement when a detector that
meets the quality target cannot run at the required prediction latency/cadence
on the existing camera after supported model/pipeline optimization. This cycle
has not established that boundary. A mechanical upgrade has a separate criterion:
required motion or holding performance cannot be achieved within installed
drive limits. Neither purchase can substitute for evidence about the other.

## Evidence

The first attempt's ignored local directory is
`run/optimization-homing-20260908b/`. `analyze_attempt.py` derives the motor-only
summary from the copied log and checks its SHA-256 against the Pi:
`daa81bed9b6add674712ff178a9d8aa65646e7ae7d7c58cfae0f8d61f7eb59ea`.
The controller log is 44,428 bytes. The Pi had approximately 2.5 GB free after
the preceding cleanup. Captures remain outside version control.
