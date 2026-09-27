# Plan: Hailo-assisted people and head tracking

Status: **Hailo camera-only visiond and normal IMX500 mixed-stack operation are
observed; head/person accuracy evaluation, dual-camera operation and stop
qualification remain incomplete**.
Updated 27 September 2026. Uses the [verified hardware inventory](../hardware/HARDWARE_CURRENT.md).
Mixed motor and continuous-yaw runtime integration are now observed; physical
stop and homing-guard qualification remain open in the
[hardware adaptation plan](../hardware/HARDWARE_ADAPTATION_PLAN.md).

The explicit `hailo_yolov8n` profile now runs IMX477 640x480 through the existing
DetectionSet, tracker, selection and preview path. A real 60-frame launcher run
delivered all frames with no inference, publishing or clock-domain errors, at
the configured 15 Hz. Model inference p50/p95 was 7.09/8.60 ms; sensor-to-publish
p50/p95 was 22.19/25.79 ms. One frame contained a permitted detection and no
track was confirmed, so the scene does not qualify person/head accuracy.

The normal mixed profile separately completed a five-minute AUTO_ROAM run using
the IMX500 stream: 8,061 frames, zero frame drops, repeated AUTO_ROAM ↔
AUTO_TRACK/loss handoffs, and fresh BNO085 observer samples (game-RV status 3,
no gaps). The run establishes integration/throughput for that scene, not
detector precision/recall, head localization, selected-person continuity or
stop qualification. That earlier release ended with stop verification failed.
On release `cae41d0`, two later controlled stops succeeded after AUTO_TRACK near
+46° yaw and AUTO_ROAM near +76° yaw, each confirming fresh pitch-disabled
feedback and issuing the final GM6020 zero request. GM6020 disable state remains
unknown, and intermittent feedback-readiness rejection from the previous release
is not proven eliminated. Final station readiness remains pending. A web
telemetry bug that made NaN GM6020 yaw effort
break `/api/state` was fixed by encoding unavailable effort as JSON `null`; the
dashboard now shows it as unavailable.

Use `--hold-motion --profile hailo_yolov8n` for this profile until IMX477
intrinsics and camera-to-axis geometry are commissioned. The controller's old
1920x1080 calibration does not cover this sensor/stream. Simultaneous dual-camera
ownership and head localization remain follow-up work.

## Requested behavior and scope

Detect people separately from other objects, locate a visible person's head for
camera framing, and keep following an operator-selected person. For finding the
selected person again, use session-local track continuity, motion and temporary
non-biometric appearance cues such as clothing color. This supports searching
for the selected subject during the session; it does not establish a person's
real-world identity or recognize them across sessions. Similar clothing or a
long off-screen interval can make the answer uncertain. Show that uncertainty
and request reselection instead of substituting a different person.

Separate three outputs: **person classification**, **head localization**, and
**selected-track association**. A high person-class score does not establish
head position or identity continuity. Head/pose geometry is used for framing,
not biometric identification. Persistent face galleries and biometric matching
are outside this plan.

## Baseline and recommended architecture

The launcher uses `person_detect_available` for the default perception profile:
IMX500 SSD MobileNetV2 FPN Lite 320 x 320, configured at 26 inference Hz. The JSON's
standalone default `person_detect` instead names YOLO11n PP, 640 x 640 at 16 Hz,
with uncommissioned thresholds. Record the actual launched profile, model hash
and settings in every comparison; neither configured rate is an achieved rate.

The vision service has an explicit Hailo/IMX477 adapter profile in addition to
the IMX500 and mock paths. Keep the current normalized detections, BYTE-style
association, UUID selection, latest-only preview and sensor-time contracts
wherever they remain suitable. The default station run remains IMX500; Hailo's
camera-only test does not promote it to the motion-authoritative stream.

Recommended experiment: retain IMX500 as the baseline/person search stream and
evaluate IMX477 + Hailo as a person/head-detail stream. Determine their final
roles from measured lens FOV, head pixel size, blur and inference quality. The
owner reports a 25 mm F1.4 lens on the IMX477 HQ Camera. Its 6.287 × 4.712 mm
full sensor area gives a nominal rectilinear FOV near 14.33° × 10.77°, much
narrower than the existing IMX500 1920 × 1080 calibration (~69.2° × 40.4°).
Measure the effective FOV and crop for the actual 640 × 480 Hailo stream before
using these approximate angles in association or control.
[Raspberry Pi camera specifications](https://www.raspberrypi.com/documentation/accessories/camera.html#hardware-specifications).

Current Hailo executable evidence includes both a single-camera probe and a
60-frame visiond integration run: the minimal Hailo-8 runtime survives reboot,
the project venv
imports HailoRT, and a real IMX477 stream produced finite YOLOv8n outputs with
valid capture timestamps. Thirty 640 x 480 RGB frames ran at 15 fps after
letterboxing to 640 x 640; inference p50/p95 was 6.86/7.05 ms and
sensor-to-result p50/p95 was 20.99/21.69 ms. No box reached 0.5 confidence in
the current view, so these timings say nothing about person accuracy or useful
recall. The camera was released after the probe. The HEF and raw run artifacts
are retained only under ignored `run/hailo-probe/`; a repeatable probe script
and configuration manifest are now available in `Firmware/tools/probe_hailo_camera.py`
and `Firmware/config/hailo_yolov8n_manifest.json`.

```mermaid
flowchart LR
  A[IMX500 owner and sensor inference] --> N[Normalized observations with camera ID and capture time]
  B[IMX477 owner] --> H[Hailo person detector]
  H --> N
  H --> P[Selected-person pose or head localization]
  N --> T[Person tracks and explicit selection]
  P --> G[Time and geometry validation]
  T --> G
  G --> C[Selected framing observation]
  C --> M[Motion controller with its own limits]
  T --> W[Web candidates and confidence]
```

This is a staged design: first prove one Hailo camera path. The diagram's
cross-camera combination is enabled only after time and geometry validation.
Each camera has one owner; web preview consumes those owners' frames.

The installed BNO085 is an additional launcher-supervised, observe-only motion
source. A fresh timestamped trace ran alongside the five-minute normal stack,
but mount calibration and image-motion compensation are not qualified. It does
not increase detector class accuracy by itself.

## Phase A: provision and prove the accelerator

The minimal Hailo-8 stack is now provisioned on the existing
`6.18.39+rpt-rpi-2712` kernel: DKMS 3.2.2, HailoRT and
`hailort-pcie-driver` 4.23.0, and `python3-hailort` 4.23.0-1. The package
transaction added 8 packages and upgraded/removed none; `hailo-all` and Tappas
were not installed. After reboot, `/dev/hailo0` returned, `hailortcli
fw-control identify` reported HAILO8 firmware 4.23.0, and the project venv
import worked. This satisfies basic device/runtime identification for this
kernel, not long-run health or production service integration. A repeatable
`Firmware/tools/probe_hailo_camera.py` and config manifest are being added.
[Official Pi setup and package compatibility](https://www.raspberrypi.com/documentation/computers/ai.html).

Use project-local environments for added Python dependencies, with system camera
and supported OS-packaged Hailo bindings available (for example, a compatible
venv created with `--system-site-packages`). Run both the Hailo binding import
and a real inference through the **same interpreter the launcher selects for
production visiond**, including any `OTA_PYTHON` choice. Record that interpreter,
binding path/version and native runtime version; a system-Python-only demo does
not establish that the project venv works. Keep package/HEF versions matched, record the resolved
versions, and fail clearly on incompatibility. Prefer precompiled vendor HEFs
for the first executable probe. Hailo Model Zoo **v2.x** and Dataflow Compiler
**v3.x** target Hailo-8/8L; current Model Zoo master targets newer hardware.
Compile custom models on a supported development host only when necessary.
[Official Model Zoo compatibility notice](https://github.com/hailo-ai/hailo_model_zoo).

**Basic runtime exit passed:** the runtime identifies HAILO8 and real camera
pipelines produced finite outputs using the compatible recorded HEF. The
repeatable probe/manifest and camera-only visiond run exist. Longer sustained
health and person/head quality remain open. PCI identity and the 26 TOPS rating
alone do not establish useful accuracy or frame rate.

## Phase B: choose models using representative evidence

An official Hailo Model Zoo 2.17.0 YOLOv8n HEF has now run on the HAILO8 device
as an execution candidate. SHA-256:
`e893b0f9dcae366fe1bc9ebce25e32ad889acf2bc58cfe1f73a572f78f7ec055`.
The 30-frame hardware benchmark reported 3.36 ms inference latency. The real
IMX477 pipeline measurements above are useful for timing only; no output reached
0.5 confidence in the current view. No person/head accuracy has been measured.
Continue with a compatible Hailo YOLOv8m person detector and a smaller model
comparison on representative labelled scenes. Hailo's Pi examples include H8
detection and pose pipelines; these are starting points for evaluation, not
station acceptance.
[Official Pi example pipelines](https://github.com/hailo-ai/hailo-rpi5-examples/blob/main/doc/basic-pipelines.md).

For head position compare two approaches: a person pose model with confident
head-related keypoints, and a dedicated head detector applied to the selected
person's crop. Official Hailo pose examples include YOLOv8s/m pose. Keypoints
such as nose/eyes/ears are geometric measurements; they do not directly specify
an anatomical head center, and back-facing heads need a tested fallback. A
dedicated head model requires its own compatible export/HEF and quality evidence.
[Official Hailo pose example](https://github.com/hailo-ai/hailo-apps/blob/main/hailo_apps/python/standalone_apps/pose_estimation/README.md).

Build a labelled evaluation set with small/distant people, standing/sitting,
front/profile/back views, hats, partial heads, occlusion, people crossing,
similar clothing, fast motion, indoor low light and bright backgrounds. Include
empty scenes and difficult non-person negatives such as chairs, displays,
posters and human-shaped objects. Split tuning and acceptance scenes; report
counts and uncertainty, not just a single average score.

Compare models on the same images where the execution paths support replay.
For sensor-only inference, use repeated controlled scenes and document the
unavoidable capture difference; do not label two unrelated live streams an
identical-input comparison. Separate detector accuracy from tracker behavior.
The existing `perception/replay`, `tools/bakeoff.py` under `perception/`, and
commissioning tools are useful starting points.

Record model origin/license, checksum, target architecture, runtime/compiler
versions, input dimensions/layout/color, quantization, label indexing, outputs,
NMS location and thresholds in a manifest. Tune thresholds independently per
model. A larger model is selected only if the measured quality gain justifies
its capture-to-observation delay and resource cost.

**Exit:** chosen detector/head strategy beats or usefully complements the
measured baseline on the held-out set. No FPS or accuracy uplift is assumed.

## Phase C: integrate one camera and correct head geometry

Add a Hailo adapter behind an experimental profile. Separate capture from
inference and preview; bound every queue, drop stale work and release camera
buffers promptly. Avoid blocking motor control on Hailo scheduling. Preserve
sensor timestamp, host receipt time, frame sequence and camera identity across
asynchronous inference and reject delayed/out-of-order results.

Reverse letterboxing/cropping/resize exactly once. Carry the active sensor crop,
orientation, stream dimensions and intrinsics through detection to LOS. Verify
color order and model label mapping with real frames before trusting scores.
Expose preprocessing, inference, postprocessing, tracking and publication
timings separately. Integrate through the existing detection/track protocols;
version those protocols if camera IDs, anchor provenance or uncertainties need
new fields. Do not change packet layouts without both ends accepting the version.

The current `tracking.aim_point` at 0.22 of a person box and the existing
shoulder/torso anchor are not measured head detections. Add explicit sources
such as `head_box`, `head_keypoints`, `torso_fallback`, with visibility,
confidence and capture age. Keep person association based on the body track
while using the head point only for framing. Reject a head that belongs to a
neighboring person's overlapping box. On missing/occluded head, transition
smoothly to a labelled torso fallback or hold; do not extrapolate indefinitely.

**Exit:** real Hailo observations traverse the production pipeline into replay/
simulated control; overlays agree with pixels and calibrated rays; stalls and
bad geometry cannot produce a fresh selected observation.

## Phase D: keep the selected person through short interruptions

Reuse selected UUID protection and motion/IoU/scale gates. Evaluate the existing
`tracking/appearance.py` upper/lower-body HSV descriptor as a low-weight
association cue, in memory only and expiring with the track. Keep person score,
association confidence and head-point confidence distinct in UI and telemetry.
No persistent identity store or face embedding is needed for this workflow.

When the operator explicitly selects someone, latch that selection policy.
The current `AUTO_SELECT_SINGLE` must not silently choose the next sole person
after the selected one disappears. Roaming/search can resume while selection
is retained for its bounded lifetime. A plausible short-gap return must pass
motion, time and ambiguity checks; two similar candidates means no reacquisition.
After expiry, report selection lost and require a new operator selection for
this mode. Separate this from the normal optional automatic-any-person mode.

**Exit:** crossing/occlusion trials show no observed target steals; ambiguous
and expired selection states are visible and cannot silently redirect tracking.

## Phase E: add the second camera only for a measured benefit

First run concurrent independent pipelines and measure CPU, memory bandwidth,
thermal throttling and observation age. The September 26 30-frame test proves
basic dual capture only. Pi supports concurrent cameras, but exposure and 3A
are not automatically synchronized; software sync is not exact simultaneity.
[Official multiple-camera guidance](https://www.raspberrypi.com/documentation/computers/camera_software.html#use-multiple-cameras).

Commission per-camera intrinsics/distortion, crop/orientation and rigid mounting
extrinsics. Determine whether both cameras move with the head and test mounting
flex at representative pitch/yaw. Pair observations by measured timestamp skew
and sensor exposure, not arrival time. Account for each sensor's rolling-shutter
and exposure contribution during motion.

Different optical centers introduce range-dependent parallax. Without depth,
a 2D box in camera A is not a unique pixel or ROI in camera B; use calibrated
ray/epipolar candidate gating or a validated limited-range approximation with
uncertainty bounds. Do not assume stereo depth or a universal homography. First
use camera B as independently checked corroboration/detail for a selected
candidate in overlapping view. Missing, stale or ambiguous correspondence
produces no refinement. Do not merge independent camera UUIDs by numeric ID.

**Exit:** the extra stream measurably improves person recall/head localization
without stale observations, duplicate people or false cross-camera association.
Otherwise retain one motion-authoritative stream and a secondary operator view.

## Phase F: use the IMU to distinguish camera motion from subject motion

The BNO085 observer now runs alongside the normal mixed controller, but it is
not yet calibrated or used for compensation. Once its observe-only validation
and mount calibration pass, interpolate BNO085 orientation/rates
to each camera's sensor timestamp and project camera rotation into the image.
Use this as a motion-compensation hint for the existing
`perception/tracking/camera_motion.py` path, combined with encoder kinematics.
This can improve association while the camera pans and help estimate subject
motion; measure that effect instead of assuming an improvement.

Carry validity, status and age with each IMU hint. The observed rotation-vector
accuracy status of zero is not suitable for authoritative orientation. A short-
term gyro/game-rotation estimate may be evaluated after bias/timing checks;
its yaw drift prevents use as a guaranteed global heading. Inertial rotation
does not remove parallax, translational image motion or rolling-shutter effects.
Do not feed the same encoder evidence into an allegedly independent IMU check.

Compare labelled pan/tilt replays with and without IMU compensation, measuring
ID switches, angular framing error and head-anchor jitter. Drop stale/bad hints
and retain the existing visual/encoder fallback. No automatic tare, persistence
or motion-control authority is introduced by this perception feature.

**Exit:** measurable association/framing benefit with no target steals or
regression during sensor dropout, magnetic disturbance or timestamp faults.

## Acceptance and performance gates

The existing [perception architecture](../../not-implemented/vision/open_auto_turret_perception_target_selection_architecture_v1.md)
provides initial tracking gates. Keep the following measurements distinct:

| Measure | Proposed acceptance approach |
|---|---|
| Person precision/recall | Report by range/head size, light and occlusion on held-out labels; improve the selected failure cases without degrading agreed baseline cases |
| Empty-scene selectable people | Aim for zero; existing initial maximum is fewer than 1 per 10 minutes |
| Duplicate visible candidates | Existing target below 1% |
| Selected-person continuity | Zero observed ID switches/target steals in crossing set; retain selection through 300 ms occlusion in at least 95% of trials |
| Head localization | Report median/p95 head-center error normalized by labelled head size and in calibrated angle; proposed initial p95 below 25% of head-box diagonal for visible heads, subject to scene commissioning |
| Head unavailable | Every fallback labelled; zero fresh-head claims from stale/occluded measurements |
| First valid detection to selectable | Existing preferred p95 <=250 ms; measure separately from automatic selection dwell |
| Actual automatic acquisition | Report full exposure-to-selection and selection-to-motion delay; current 500 ms single-candidate dwell cannot meet a 250 ms automatic-acquisition target unchanged |
| Latency and throughput | Capture-to-publication p50/p95/p99, achieved inference/publication rates, queue age and drops for each camera/model; no growing backlog |
| System load | Sustained dual-camera/Hailo run, including motor control once commissioned; no controller deadline/watchdog regression, unexplained CAN drop growth or active undervoltage/throttling. The host was lost during a concurrent release build and subsequently reported active `0x50005` undervoltage/throttling during AUTO_TRACK. Correct and requalify the Pi/HAT power path before adding accelerator load to normal motion. |
| IMU contribution | Compare compensation enabled/disabled; only calibrated, correctly timestamped hints may be used, and bad/stale IMU data must revert visibly to the validated fallback |

Historical IMX500 sensor-to-publication measurements were median 58.34 ms /
p95 73.87 ms with approximately 38 ms exposure. They belong to the previous
station and provide a comparison method, not an acceptance result for this one.
See [latency analysis](../analysis/latency_bottleneck_analysis_2026_09_08.md). Measure optical
scene-change-to-correct-observation separately; accelerator inference time alone
does not include exposure, queueing, confirmation or mechanical response.

Release in order: camera-only benchmark -> replay/simulated control -> bounded
Manual tracking after motor commissioning -> operator-selected search -> normal
automatic operation. Keep the IMX500 fallback explicit and report its reduced
capabilities if Hailo fails. Switch camera/model only through a controlled reset
of geometry/track state; never silently substitute a stream with different
calibration. Store runtime datasets outside Git and preserve approved versioned
model/profile manifests with the release.
