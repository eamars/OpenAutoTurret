# Tracking latency and hardware priorities — 8 September 2026

**Historical baseline within this day's work.** Later physical trials,
separate DNN/DSP measurements, endpoint failures, storage cleanup and the current
stopped state are recorded in the [optimization-cycle follow-up](optimization_cycle_2026_09_08.md).
Its operational status supersedes the running-state and Pi artifact locations
at the end of this baseline report.

**Finding:** The measurements do not support buying a faster Pi or an AI HAT as the first response to slow tracking. The largest identifiable contributors are acquisition policy, the tracking trajectory filter, and the installed drive/load response. Camera delivery and host perception processing are tens of milliseconds; software acquisition and substantial physical movement are hundreds of milliseconds to seconds.

**Status: partially verified across the complete optical loop.** Real target-free motor response and live camera/publication timing were measured. Production selection and reference-generation code were exercised independently with synthetic inputs. No person or other real target was required to move, and detections never controlled the physical tests. Photon-to-recognition timing, independent output-angle metrology, and tracking under full-speed load remain unverified. These are component measurements, not a measured end-to-end optical acquisition time.

The active release was `7aabc2cb3a494963408ab9daf0669b1383ac6b17`, at `/home/eamars/workspace/OpenAutoTurret/run/releases/7aabc2cb3a49.FOB3cZ/`. The station is a Raspberry Pi 5 Model B, IMX500 AI Camera, SSD MobileNetV2 FPNLite 320 detector, and CyberGear motors through the Yousee USB/UART-to-CAN adapter. Source, drive gains, current limits, calibration, camera configuration and service limits were unchanged. All motion used the existing web command API under the running launcher.

## What “latency” means here

Three different questions need separate answers:

1. **First acquisition:** how long until a new eligible object is selected and tracking is permitted?
2. **Following response:** how quickly does an already accepted direction change become a trajectory and then movement?
3. **Arrival/settling:** how long until the mechanism reaches a requested direction? This depends on distance, speed, acceleration, jerk, load and controller dynamics; it is not a fixed communication delay.

The closed loop is:

```text
scene → exposure/readout + IMX500 inference → CSI/libcamera → Pi image copy,
tensor parsing, coordinate conversion, deduplication, association → selection
→ local native socket → capture-time encoder/geometry transform → estimator
→ automatic mode/measurement gates → prediction → tracking reference filter
→ host position/speed servo → USB/UART/CAN → drive velocity loop + mechanism
→ encoder feedback and changed camera view → next observation
```

The browser preview is a separate, slower observation path. Its refresh rate and HTTP delays must not be counted as vision-to-controller delay.

## Static analysis, checked against executable probes

| Stage | Active behavior | Consequence |
|---|---|---|
| Camera | 26 Hz model setting; actual cadence 26.01 Hz; exposure 38.173 ms | Frame phase alone introduces up to about 38.45 ms of sampling wait, depending on when the scene changes. Exposure/readout and inference overlap; do not add their durations blindly. |
| Candidate confirmation | At least three observations and 120 ms visibility | At 26 Hz, ideal continuous detections first confirm after 153.846 ms, on the fifth observation. |
| Automatic selection | One eligible candidate continuously present for 500 ms, with identity/confidence gates | Ideal first selection occurred 653.846 ms after the first synthetic detection. Multiple candidates, gaps or insufficient confidence extend or restart the wait. |
| Roam-to-track handoff | A fresh, reachable selected target must persist for 250 ms | Adds about 250 ms for normal AUTO_ROAM acquisition; already-tracking updates bypass this dwell. |
| Anti-hunting gate | 1,000 ms since the last automatic mode switch | Can postpone reacquisition after a recent handoff. It is conditional, not a charge on every frame. |
| AUTO_TRACK acquisition | Requires two further distinct accepted capture timestamps after entering ACQUIRE | An isolated production state-machine probe reached TRACKING after 80 ms at 26 Hz and 40 ms at 52 Hz, with 5 ms control ticks and immediately valid evidence. These are new frames, not merely two 200 Hz ticks. |
| Estimator | Capture-time pose compensation, Kalman update, confidence-dependent covariance and uncertainty-gated target velocity | Filtering/rejection can delay or weaken response to changing observations. There is no single fixed delay to infer from these parameters. |
| Prediction | Configured 120 ms motor response plus 20 ms control look-ahead, with measurement age | These are prediction assumptions, **not sleeps and not measured actuator latency**. Do not add 140 ms as a serial waiting stage. |
| Tracking reference | `track_reference`, a damped second-order response with `omega=2.5/s`, followed by motion limits | A substantial software response time persists even with an instantaneous detector and perfect motor. See the executed reference probe below. |
| Motion limits | AUTO_TRACK target 20°/s, 30°/s², 100°/s³; maximum 20°/s, 30°/s², 120°/s³ | Larger movements and reversals take time. Confidence and boundary handling can reduce authority. At the full target speed there is no extra speed headroom for error correction. |
| Host servo | Velocity feed-forward plus position correction gain 3/s, acceleration/jerk limiting, quiet-hold hysteresis 0.08°/0.15° | Small-error correction and actuator commands have their own dynamics. |
| Communication | 200 Hz controller; native local socket; UART 921,600 baud; CAN 1 Mbit/s | No architectural network round-trip in the tracking path. A 17-byte UART command needs about 0.184 ms for 8N1 serialization alone; this excludes USB buffering, arbitration, drive handling and feedback. |

Relevant implementations: [configuration](../config/turret.yaml), [camera ownership](../perception/camera.py), [pipeline](../perception/pipeline.py), [track lifecycle](../perception/tracking/track_manager.py), [selection policy](../perception/selection/policy.py), [mode/servo integration](../control/src/control/control_loop.cpp), [AUTO_TRACK acquisition](../control/src/tracking/auto_track_controller.hpp), [tracking filter](../control/src/control/tracking_reference.hpp), [speed servo](../control/src/control/speed_servo.hpp), and [UART transport](../control/src/can/yousee_transport.cpp).

**Ideal selection-rate comparison, executed through production association and selection:**

| Supplied detection cadence | First confirmed/selectable | First automatically selected |
|---|---:|---:|
| 26 Hz | 153.846 ms | 653.846 ms |
| 52 Hz | 134.615 ms | 634.615 ms |
| 1,000 Hz, hypothetical | 120 ms | 620 ms |

Thus doubling detector cadence saved only **19.23 ms at the selection boundary** in the ideal case. It can also reduce frame-phase wait and the later two-frame acquisition gate. It cannot remove the 120/500/250 ms time-based requirements. For a brand-new sole candidate, the healthy path is already of order one second before a following trajectory is authorized, including current camera delivery and the later acquisition gates. This is a budget estimate assembled from components, not a measured optical event-to-motion result. A retained, selected target takes a different path.

**Tracking-reference response, executed against the active release's actual C++ header:**

| Stationary direction step | Time to 10% | Time to 50% | Time to 90% |
|---|---:|---:|---:|
| 1° | 240 ms | 695 ms | 1,580 ms |
| 5° | 320 ms | 785 ms | 1,670 ms |
| 10° | 395 ms | 820 ms | 1,660 ms |

This probe starts the reference at rest, supplies zero target velocity, advances at 200 Hz and applies the configured AUTO_TRACK target limits with full confidence and no boundary reduction. It contains no camera, estimator or motor. It demonstrates the software filter's step response, not an unavoidable mechanical limit. A moving target with credible velocity feed-forward behaves differently. The slightly shorter 10° versus 5° 90% result reflects the nonlinear limits and is not a general speed advantage.

## Real test design and execution

**Hypothesis:** If Pi/inference throughput is the principal delay, camera delivery or host processing should approach the observed motion delay. If commanded reference/speed change substantially before encoder position, the drive/load path contributes independently. A slow reference with no motor would establish a separate software contributor.

The first executable probe was a 15-second read-only baseline. A single 2-second yaw+ FINE jog then established that the production API, timestamped controller events, planned reference and encoder feedback could be observed together. It completed under ALLOW with no fault. The repeated design used yaw+, yaw−, pitch+, pitch−, each for two seconds with a renewed 300 ms lease, an explicit jog stop, four seconds of observation and another second between trials. FINE requests at most 3°/s while jogging; the normal service controller retains authority during stopping/hold correction.

Entry checks required Manual/Hold, healthy fresh feedback, valid calibrated limits and at least 15° clearance on both axes. Monitoring aborted on a fault, non-ALLOW decision, stale telemetry or more than 10° excursion from a trial's starting pose. Cleanup used `manual_jog_stop`; the existing lease and controller watchdog remained active. The probe never opened the motors or camera, bypassed homing, or changed automatic mode.

The measurement acceptance criteria were a recorded controller acceptance event, separate changing reference/command/encoder signals, no faults, and valid telemetry throughout a completed trial. Motor velocity telemetry is noisy at rest, so physical response is evaluated from **encoder displacement**, with sustained threshold crossings. Both 0.25° and 0.5° crossings are retained; the table uses 0.5° because pre-trial encoder spans ranged from approximately 0.022° to 0.175°. This is time to a visible amount of movement, not first microscopic movement or pure drive dead time.

There were **13 completed physical trials**: four yaw+, three yaw−, three pitch+ and three pitch−. A fourteenth trial was excluded because the capture aborted during post-stop observation. The last controller sample was approximately 556 ms old when the probe rejected it; recorded feedback remained healthy, with no sampled fault or derating. The exact cause of the telemetry interruption was not isolated. After reducing observer HTTP polling from roughly 50 to at most 20 requests/s, eight additional trials completed. That outcome does not establish causation. The aborted capture is retained, not silently discarded.

**Measured intervals from controller jog acceptance to 0.5° displacement:**

| Direction | Completed trials | Planned position crosses 0.5° | Encoder crosses 0.5° |
|---|---:|---:|---:|
| yaw+ | 4 | 496–659 ms | 575–976 ms |
| yaw− | 3 | 497–663 ms | 575–743 ms |
| pitch+ | 3 | 578–724 ms | 821–993 ms |
| pitch− | 3 | 491–658 ms | 570–1,805 ms |

Each range encloses the before/after sampling brackets across those trials; it is not a confidence interval or p95 estimate. Typical telemetry spacing was about 66 ms, sometimes 80–121 ms. Encoder sample timestamps, rather than HTTP arrival time, locate position observations. Command acceptance was established from the controller event timestamp, excluding SSH and initial HTTP dispatch from these motion intervals.

The slow pitch− trial reached 0.5° in **1.739–1.805 s**, while its reference crossed that threshold in **0.496–0.562 s**: an additional **1.18–1.31 s between comparable displacement crossings**. Another pitch− trial took 1.325–1.405 s; the third was much faster. This variability is inconsistent with one fixed camera or UART delay. The evidence localizes a substantial contributor downstream of the planned reference; it does not distinguish load imbalance, friction, drive velocity-loop behavior and other mechanical effects sufficiently to specify a replacement motor.

The optional integrated-command time-shift fits in the JSON are exploratory, not identified plant dead times. Their residuals are significant, and one hits the search bound. They must not be used to replace the configured prediction lead with one fitted constant.

![Repeated physical response](../../../run/latency-20260908/analysis/physical-overview.png)

These tests characterize the installed drive's encoder response under small, slow movements. They do not certify independent camera-platform angle, mount flex, backlash, high-speed tracking performance or settling at arbitrary load/pose. Manual and AUTO_TRACK share the host speed-servo/backend path, but their reference generation and limits differ; the jog times are not AUTO_TRACK latency measurements.

## Camera → recognition → Pi measurements and research

The final complete physical run supplied 1,559 unique perception publications, 523 preview metadata records, 522 exactly paired metadata/publication frames and 907 unique controller snapshots. The initial stationary baseline gave similar timings.

| Measured boundary | Median | p95 | Meaning |
|---|---:|---:|---|
| Sensor timestamp → camera request return | 39.13 ms | 41.12 ms | Exposure/readout, sensor processing, delivery and request scheduling together; not isolated neural inference. |
| Request return → publication, matched frame | 17.70 ms | 19.60 ms | Pi-side image copy, tensor handling, geometry, filtering, association and selection before the publication timestamp. |
| Sensor timestamp → publication, all sampled frames | 58.34 ms | 73.87 ms | Live perception delivery latency. Its maximum was 84.33 ms. |
| Publication → controller receipt | 1 ms | 1 ms | Integer-millisecond controller telemetry; maximum 11 ms. Zero values mean sub-millisecond rounding, not instantaneous transfer. |
| Sensor age at sampled control cycles | 83 ms | 105 ms | Includes time between successive observations; not a second stage to add to sensor-to-publication delay. |
| Motor-feedback age | 6 ms | 17 ms | Age of the latest available feedback; not a measurement of command-to-motion latency. |
| Control-cycle interval | 5.057 ms | 5.070 ms | Scheduling interval, not CPU execution time. No deadline misses were present in the completed capture's snapshots. |

Stage medians have different sample populations and must not be summed as if they described one representative frame. Preview metadata is sampled more sparsely than native publications. The JPEG is used only as a carrier for original timestamps; JPEG publication delay is excluded from the matched camera/request-to-publication calculation.

The camera exposed for 38.173 ms with a 38.450 ms frame duration throughout these recordings. `SensorTimestamp` refers to the exposure of the first active sensor row and uses `CLOCK_BOOTTIME`; it is not object appearance time. The Pi's boottime/monotonic comparison was effectively zero at the precision needed here, so subtraction against host monotonic timestamps was valid for this unsuspended run. Rolling-shutter row timing and the scene-change phase remain outside the measurement. See the [libcamera control definitions](https://docs.libcamera.org/master/internal-api/namespacelibcamera_1_1controls.html).

For this hardware the physical ordering differs from CPU detection: the IMX500 includes its own image preprocessing and neural accelerator and transmits inference tensors alongside the image stream. The installed SSD model performs detection on the camera; the Python `infer()` call largely retrieves and interprets existing tensors. Timing that call alone would miss the sensor's computation. [Raspberry Pi AI Camera architecture](https://www.raspberrypi.com/documentation/accessories/ai-camera.html).

**What is not measured:** the sensor's DNN runtime separately from its DSP/postprocessing and the actual scene-change-to-first-correct-detection interval. Picamera2 exposes `get_kpi_info(metadata)`, returning DNN and DSP times from `CnnKpiInfo`, but the current station's exported preview metadata does not carry this field. Reading an existing snapshot cannot reconstruct it. No competing camera owner or service restart was introduced to obtain it. [Picamera2 IMX500 implementation](https://github.com/raspberrypi/picamera2/blob/main/picamera2/devices/imx500/imx500.py).

A future optical proof can avoid a real subject: use a clocked LED transition for photon-to-image timing, and a display switching between blank and a prerecorded/synthetic recognizable image for semantic timing. Record the actual display/LED transition with a photodiode on the same timebase, collect sensor timestamps and `CnnKpiInfo` from the sole camera owner, and record first correct detections before association. Randomize transition phase and repeat under several light levels. This separates display delay, exposure/frame phase, sensor DNN/DSP time, host work, confirmation and selection. A software command to change the display is not itself proof of when photons changed.

The post-test five-second CPU sample was 24.3% busy across four cores, with the busiest core at 30.6%, and CPU temperature 54°C; `vcgencmd get_throttled` reported `0x0`. This short held-camera sample supports available CPU headroom under the observed workload. It is not a full-load or real-time scheduling guarantee. The telemetry interruption and occasional historical log overruns remain separate reliability observations.

## Hardware decision

| Candidate action | What it could improve | What this evidence supports |
|---|---|---|
| Review acquisition and reference-filter behavior | The 120/500/250 ms gates and slow direction-step response | Highest-priority software investigation before spending on compute. Changes would need false-acquisition and closed-loop validation; none were made here. |
| Investigate pitch balance, friction, transmission and drive/load response | Variable delayed movement after a command/reference exists | Higher-priority hardware investigation than an AI accelerator. Repeat at fixed poses with independent output sensing before choosing a motor or transmission. |
| Improve illumination / evaluate shorter exposure | Exposure blur and possibly optical response/recognition reliability | Current exposure nearly fills the 38.45 ms frame. A new accelerator alone cannot remove optical integration time. Sensor delivery latency must be remeasured after any camera changes. |
| Faster Pi | Host processing, scheduling, concurrent workload capacity | No evidence it is the primary bottleneck here; host processing is about 18 ms and the installed Pi is already a Pi 5. |
| AI HAT+ / AI HAT+ 2 | Compatible detector/pose models, larger models, potentially better quality or faster inference | A model/pipeline replacement project, not a plug-in speedup of the existing sensor-side SSD. Benchmark comparable model accuracy and batch-one latency before buying for latency. |

The **AI HAT+ 2** uses Hailo-10H with 8 GB onboard memory and advertises **40 TOPS at INT4**. That number does not predict the latency of this detector and is not directly comparable to an INT8 TOPS figure. Raspberry Pi's product brief describes computer-vision performance as equivalent or superior to the 26 TOPS AI HAT+, while the older HAT+ has 13/26 TOPS variants. [AI HAT+ 2 product brief](https://pip-assets.raspberrypi.com/categories/1319-raspberry-pi-ai-hat-2/documents/RP-009655-MM-3-raspberry-pi-ai-hat-plus-2-product-brief.pdf), [AI HAT+ product brief](https://datasheets.raspberrypi.com/ai-hat-plus/raspberry-pi-ai-hat-plus-product-brief.pdf).

A Hailo pipeline would require a compatible model artifact and changes to the current IMX500-specific ingestion/coordinate mapping. It could be justified by better detection/pose accuracy, which may improve tracking indirectly, even if latency is not the current bottleneck. This investigation did not benchmark a HAT, compare detector accuracy, or establish how much faster a replacement pipeline would be. The present evidence supports **keeping the Pi 5/IMX500 while first addressing policy/reference response and characterizing the pitch mechanism**.

## Evidence, reproduction and final state

Raw data, plots and derived results are retained under `run/latency-20260908/` locally and on the Pi; runtime captures are not added to Git. The local [analysis summary](../../../run/latency-20260908/analysis/summary.json) includes SHA-256 hashes of each JSONL capture, stage statistics, individual trial brackets and the excluded trial. [Selection replay](../../../run/latency-20260908/acquisition-timing.json), [reference samples](../../../run/latency-20260908/reference.csv), [AUTO_TRACK gate result](../../../run/latency-20260908/auto-track-acquire.csv), [environment](../../../run/latency-20260908/environment.json) and [final launcher status](../../../run/latency-20260908/final-status.txt) are separate artifacts.

Reusable tools added:

- [measure_station_latency.py](../tools/measure_station_latency.py): default read-only; `--single` runs the first yaw probe; `--jogs N` runs N four-direction cycles. On the Pi, use `run/station-venv/bin/python` and an output under `run/`.
- [analyze_station_latency.py](../tools/analyze_station_latency.py): offline analysis and plots, using the project venv with NumPy and Matplotlib.
- [probe_acquisition_timing.py](../tools/probe_acquisition_timing.py): synthetic timestamps/detections through production association and selection; no device access.
- [probe_latency_reference.cpp](../tools/probe_latency_reference.cpp): production-header step response; `--acquire` exercises the distinct-frame AUTO_TRACK gate. Compiled with `g++ -O2 -std=c++20 -I <active-release>/Firmware/control/src` into ignored `run/`.

The Python tools passed compilation checks, the offline probes executed, and plotted encoder/reference traces were visually inspected. No production behavior was changed, so an unrelated full regression suite was not run.

The original launcher and its three children remain running. Final status was Manual/Hold, valid soft limits, ALLOW, no fault and no active jog lease. The runtime manual jog preset is now FINE as used by the test; normal deployment remains AUTO_ROAM with its original profiles. No calibration was invalidated or motor power cycle requested.
