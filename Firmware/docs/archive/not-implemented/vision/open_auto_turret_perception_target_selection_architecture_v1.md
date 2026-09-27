# OpenAutoTurret Perception and Target Selection Architecture
## Independent IMX500 vision subsystem plan

**Document status:** Implementation handover / independent subsystem architecture  
**Version:** Vision 1.0  
**Date:** 2026-09-05  
**Target platform:** Raspberry Pi 5 + Raspberry Pi AI Camera (Sony IMX500)  
**Compatible controller:** OpenAutoTurret v3.x  
**Explicitly independent from:** CyberGear drive tuning, position/speed-loop selection, motor PID, braking implementation, and actuator prediction  
**Primary problem:** noisy detections, duplicate identities for one person, unreliable target retention, and ambiguous target selection

---

# 1. Purpose

This document defines an independent vision, multi-object tracking, and target-selection subsystem for OpenAutoTurret.

It deliberately stops at the following output boundary:

```text
SelectedTargetObservation
```

It does not decide:

- CyberGear operating mode;
- motor gain;
- motor reference shape;
- servo damping;
- actuator latency compensation;
- braking or stopping distance;
- yaw/pitch command.

Those belong to the independent motor/control subsystem.

This subsystem decides only:

1. What objects or people are detected?
2. Which detections are duplicates?
3. Which detections belong to the same identity over time?
4. Which identity has the operator selected?
5. Is that selected identity currently measured, temporarily occluded, ambiguously reacquired, or lost?
6. What measured image anchor should be published to the controller?
7. How confident is each distinct part of that decision?

The entire subsystem shall be executable and testable with:

```text
recorded camera data
+
a mock consumer
```

while all motors are disabled.

---

# 2. Independent subsystem boundary

```text
Raspberry Pi AI Camera / IMX500
               |
               v
       Imx500ModelAdapter
               |
               v
       DetectionNormalizer
               |
               v
       Person/Class Filter
               |
               v
      DetectionDeduplicator
               |
               v
     CameraMotionCompensator
          (optional input)
               |
               v
          TrackManager
               |
               v
     DuplicateTrackResolver
               |
               v
    TargetSelectionManager
               |
        +------+------+
        |             |
        v             v
     TrackSet   SelectedTargetObservation
        |             |
        v             v
      webd        controller IPC
```

The required controller-facing API contains no:

- motor ID;
- CAN frame;
- CyberGear term;
- servo gain;
- drive mode.

The controller is merely a consumer of target measurements.

---

# 3. Findings from the station handover

The existing handover establishes several important facts.

## 3.1 Current target choice is operator-driven

The v3 contract currently requires the operator to choose an identity.

Without a selection:

```text
AUTO_TRACK -> WAIT_TARGET
```

There is no implemented “best person” scorer.

That contract is retained as the default in this document.

## 3.2 The current simple detector is not a person detector

The current `simple_detector.py`:

- emits zero or one largest moving blob;
- uses frame differencing;
- stamps the result as `class_name="person"`;
- does not semantically classify a person;
- cannot represent two simultaneous people correctly;
- is intrinsically unsafe as a production detector while the camera is moving.

During turret movement, much of the scene changes between frames, so a frame-difference detector can interpret camera self-motion as object motion.

## 3.3 Duplicate “people” are primarily duplicate retained identities

Because the simple detector can emit at most one box per frame, seeing multiple tracks does not prove multiple detections.

The handover observed:

```text
tracks = 2
```

while only one live blob could be emitted.

The likely mechanism is:

```text
old identity remains retained as LOST
+
new current identity is created
```

The old LOST identity must not be rendered as another visible person.

## 3.4 Selection by recycled display index is not reliable enough

Observed anomalies include:

```text
select_target 2 -> "selected Person #1"
```

and labels may be recycled.

Therefore display labels such as:

```text
Person #1
Person #2
```

shall never be control identifiers.

## 3.5 Current confidence semantics are model-inappropriate

The simple detector produced scores around:

```text
0.31–0.45
```

while `AUTO_TRACK` used a medium threshold of `0.50`.

A model-specific detector score was being combined with track/reacquisition/ambiguity factors into one value.

The new architecture separates these values rather than treating one opaque “confidence” as universal truth.

## 3.6 Two tracking state machines are currently exposed through one HUD label

The handover reports that a v1 tracking state is translated to v3 vocabulary while the v3 AutoTrack FSM independently owns permission.

This makes labels such as `LOST_HOLD` diagnostically ambiguous.

The perception subsystem shall publish one authoritative target lifecycle state.

The controller may publish its separate AutoTrack phase.

The UI must show them as separate concepts.

---

# 4. Architecture decisions frozen by this document

1. **The frame-difference detector is a diagnostic/fallback motion detector only.**
2. **Production AUTO_TRACK requires an IMX500 neural detector or pose model.**
3. **The first production detector baseline is the official IMX500 YOLO11n model with on-sensor post-processing.**
4. **A model bake-off is mandatory; YOLO11n is not automatically the final deployed model.**
5. **The final human-focused candidate is a custom one-class person model, preferably YOLO11n or YOLOv8n, quantized for IMX500 with NMS included.**
6. **YOLO11n-pose is a human-specialized alternative when keypoints are reliable at the intended distance.**
7. **Detection, identity tracking, and target selection are distinct modules.**
8. **Detector-level duplicate suppression occurs before TrackManager.**
9. **TrackManager uses a ByteTrack-style two-stage association as the first implementation.**
10. **Camera motion compensation is pluggable and optional, but camera motion must never be mistaken for semantic detection.**
11. **Track lifecycle timing is expressed in milliseconds plus minimum observation count, not frames alone.**
12. **Only currently measured, confirmed tracks are shown as visible target candidates.**
13. **LOST identities remain internal/reacquirable and are not drawn as extra visible people.**
14. **Selection uses immutable `track_uuid`, never display index.**
15. **Display numbers are monotonic during one vision session and are never reused.**
16. **Selecting an already-selected UUID is idempotent and must not reset acquisition.**
17. **The default target-selection policy is explicit operator selection.**
18. **Optional automatic selection is allowed only when exactly one eligible target is stable for a dwell period.**
19. **No automatic “best of multiple people” policy is enabled by default.**
20. **Ambiguous reacquisition holds the existing selection and does not switch to another person.**
21. **Measured detection score, association quality, identity confidence, and ambiguity remain separate fields.**
22. **The vision subsystem publishes measured target data; motor lead prediction remains outside this subsystem.**
23. **The same vision pipeline can be replayed without camera or motor hardware.**
24. **The browser receives target metadata but does not own identity.**
25. **The current Picamera2/IMX500 API and package versions are pinned and verified before integration.**

---

# 5. Detection-engine strategy

The IMX500 can load different neural-network firmware profiles, but only one production model should own inference at a time.

Do not dynamically switch models per frame.

Define model profiles selected at `visiond` startup:

```text
GENERAL_OBJECT
PERSON_DETECT
PERSON_POSE
LOW_LATENCY
SIMPLE_DIAGNOSTIC
```

Each profile has:

- RPK/model path;
- task;
- label map;
- permitted classes;
- input dimensions;
- inference rate;
- post-processing type;
- bbox format/order;
- normalization;
- threshold set;
- model hash;
- license metadata.

---

# 6. Immediate production baseline: YOLO11n with on-sensor post-processing

Use the official model:

```text
/usr/share/imx500-models/imx500_network_yolo11n_pp.rpk
```

with:

```text
bbox normalization enabled
bbox order = xy
```

Reasons:

- true multi-object neural detection;
- highest reported detection mAP among the current official Raspberry Pi IMX500 object-detection zoo entries;
- on-sensor post-processing in the `_pp` package;
- a current, supported Picamera2 example path;
- direct person-class filtering;
- no dependency on frame motion.

This is the **compatibility and accuracy baseline**, not necessarily the final latency winner.

## 6.1 Expected rate

Published Ultralytics IMX500 benchmarking gives approximately:

```text
YOLO11n: 58.82 ms/image
```

which is approximately 17 inference results per second.

Therefore:

- do not promise 30 neural detections per second from YOLO11n at 640;
- distinguish 30 fps camera video from 16–17 fps inference metadata;
- measure the actual station rate.

---

# 7. Model candidate bake-off

| Model/profile | Input | Official role | Strength | Risk/limitation |
|---|---:|---|---|---|
| YOLO11n `_pp` | 640×640 | General object detection | Best official-zoo detection accuracy among listed candidates; integrated postprocess | Approximately 17 inference/s in published benchmark; AGPL model |
| NanoDet Plus `_pp` | 416×416 | General object detection | Smaller input; respectable official-zoo mAP; Apache-2.0 | Must validate exact packaged parser/runtime; not necessarily person-specialized |
| EfficientDet Lite0 `_pp` | 320×320 | General object detection | Smaller input, permissive license | Lower official-zoo mAP |
| SSD MobileNetV2 FPN Lite `_pp` | 320×320 | General object detection | Compatibility/speed floor; permissive license | Lowest official-zoo mAP among listed detectors |
| HigherHRNet | 288×384 | Human pose | Human-only keypoints; stable torso anchor possible | Official example defaults to 10 inference/s; relatively low model-zoo pose mAP |
| YOLO11n-pose custom export | nominally 640 | Human pose | Person box + 17 keypoints; better anchor and pose-based dedup | Roughly 62.5 ms/image in published benchmark; custom deployment integration |
| Custom one-class YOLO11n | 416/512/640 candidate | Person detection | Best task specialization and confidence calibration | Requires training, quantization, conversion, and real validation |

Do not compare mAP values across different datasets/evaluation scripts as if they were directly interchangeable.

The bake-off uses the project's own recorded test set.

---

# 8. Recommended model progression

## Phase A — restore a supported neural pipeline

1. Pin the software environment.
2. Run the current official Picamera2 YOLO11n example unmodified.
3. Confirm multiple people produce multiple detections.
4. Confirm inference metadata contains:
   - score;
   - class;
   - coordinates;
   - `SensorTimestamp`.
5. Integrate behind `Imx500ModelAdapter`.

## Phase B — benchmark official model zoo

Run:

```text
YOLO11n pp
NanoDet Plus pp
EfficientDet Lite0 pp
SSD MobileNetV2 FPN Lite pp
```

Use exactly the same physical test recordings/scenes.

Choose the fastest candidate that passes quality gates.

## Phase C — human-specialized profile

Test:

```text
YOLO11n-pose
```

when:

- humans are the primary target;
- torso/head keypoints remain reliable at intended distance;
- 16–17 inference/s is acceptable.

## Phase D — custom one-class person detector

Fine-tune and quantize a task-specific model.

This is the expected long-term profile if pretrained COCO models remain noisy in the installation environment.

---

# 9. Current IMX500 runtime compatibility

The implementation must stop relying on stale API assumptions.

The current official Picamera2 example uses:

```python
imx500 = IMX500(model_path)
intrinsics = imx500.network_intrinsics
picam2 = Picamera2(imx500.camera_num)
```

It does not use:

```text
Picamera2.postprocessing_config
```

The current example also uses:

- `imx500.get_outputs(...)`;
- `imx500.get_input_size()`;
- `imx500.convert_inference_coords(...)`;
- network intrinsics for:
  - postprocess;
  - bbox normalization;
  - bbox order;
  - labels;
  - inference rate.

## 9.1 Environment manifest

At startup, record:

```yaml
os:
python:
kernel:

packages:
  python3-picamera2:
  imx500-all:
  imx500-firmware:
  imx500-models:
  imx500-tools:
  rpicam-apps-imx500-postprocess:
  numpy:
  opencv:

model:
  path:
  sha256:
  task:
  labels_hash:
  network_intrinsics:
```

The agent shall not “upgrade until it works” without recording the full compatibility set.

## 9.2 Compatibility oracle

Before integrating any model, run its upstream reference example.

The model is not admitted to OpenAutoTurret until:

```text
reference example works
+
OpenAutoTurret adapter produces identical normalized detections
```

on a captured test sequence.

## 9.3 Model manifest

Create a checked-in manifest:

```yaml
model_id: imx500-yolo11n-pp-coco
task: object_detection
input_width: 640
input_height: 640
bbox_order: xy
bbox_normalized: true
postprocess: on_sensor
inference_rate_hz: 16
labels: coco
permitted_classes:
  - person
```

Fail fast when model output and manifest disagree.

---

# 10. Picamera2 versus Sony `modlib`

Sony's Application Module Library (`modlib`) supports:

- Raspberry Pi AI Camera;
- custom models;
- custom post-processing;
- tracker examples;
- object detection and pose applications;
- Trixie/Python 3.13.

Recommended policy:

```text
production camera owner:
    remain Picamera2 initially

modlib:
    use as a reference/validation implementation
    or optional future adapter
```

Reason:

- the current project already has Picamera2 camera ownership, timestamping, and preview integration;
- changing the entire camera runtime while solving detection identity creates unnecessary simultaneous variables.

However:

- use Sony's post-process implementations as a correctness reference for custom YOLO exports;
- compare one model under both paths if output parsing remains uncertain.

---

# 11. Custom person detector

## 11.1 Recommended architecture

Start from:

```text
YOLO11n detection
```

Fine-tune to:

```text
one class: person
```

Candidate input:

```text
416×416
512×512
640×640
```

Do not assume smaller input will compile or run faster enough to justify quality loss.

Benchmark all successfully converted candidates.

## 11.2 Why one-class training

Expected benefits:

- no irrelevant class confusion;
- task-specific confidence calibration;
- hard-negative learning for the installation;
- potentially smaller output/post-processing load;
- training can emphasize partial people and expected viewpoints.

It does **not** guarantee a proportionate inference-speed improvement.

## 11.3 Training data

Use:

1. COCO person examples.
2. CrowdHuman or another person/occlusion dataset, subject to its terms.
3. Station-specific recordings.

Station data must include:

- empty room/yard;
- curtains;
- chairs;
- plants;
- shadows;
- reflections;
- monitors;
- pets;
- person standing;
- walking;
- crouching;
- partially hidden;
- backlit;
- low light;
- close and far;
- one and multiple people;
- camera stationary;
- camera yawing/pitching;
- motion blur;
- entering/exiting frame.

## 11.4 Dataset split

Do not randomly split adjacent frames from the same video into train and validation.

Split by:

```text
recording session
location/time
subject sequence
```

to prevent temporal leakage.

## 11.5 Annotation semantics

Pick one consistent person-box meaning:

```text
visible-body box
```

is recommended for direct image support.

If using full-body boxes under occlusion, document it and evaluate anchor stability.

For aim control, keep a separate target anchor from the detector box.

## 11.6 Quantization calibration

Use representative images from the actual installation for INT8 calibration.

Validate:

```text
floating model
versus
quantized ONNX
versus
IMX500 deployment
```

on the same held-out set.

Reject the export if quantization materially increases:

- missed people;
- duplicate boxes;
- box jitter;
- false positives.

## 11.7 Export path

Current Ultralytics documentation supports IMX export for YOLOv8n and YOLO11n variants and supports:

```text
detection
pose
classification
segmentation
```

A candidate export command is conceptually:

```text
yolo export \
  model=person_yolo11n.pt \
  format=imx \
  data=person_dataset.yaml \
  imgsz=416 \
  nms=True
```

The local agent must validate current CLI syntax and generated artifacts.

Do not embed a command in deployment automation until it has passed a clean-environment test.

---

# 12. Human pose alternative

A pose model can be more useful than a generic person box when the person occupies enough pixels.

Potential benefits:

- stable upper-torso aim anchor;
- distinguish two overlapping people by skeleton;
- suppress duplicate poses using keypoint similarity;
- detect partial body more intelligently;
- track posture-independent center.

## 12.1 Anchor policy

When reliable keypoints exist:

Priority:

```text
1. midpoint of shoulders
2. midpoint between shoulder center and hip center
3. hip center
4. bbox fallback
```

Use confidence-weighted keypoints.

Do not aim at face/head by default; torso is generally more stable and less jitter-prone.

## 12.2 Pose duplicate suppression

Use:

- box overlap;
- shared high-confidence keypoint locations;
- Object Keypoint Similarity-like score;
- torso-center distance.

If two pose outputs describe the same skeleton, retain one.

## 12.3 When not to use pose

Reject pose as primary profile when:

- target is too small for reliable keypoints;
- inference rate is unacceptable;
- keypoint dropout creates worse identity fragmentation than box detection;
- non-person objects must also be tracked.

---

# 13. Detection output contract

Every model adapter emits the same structure.

```text
DetectionSet {
    model_id
    model_generation

    frame_sequence
    sensor_timestamp_ns
    publish_timestamp_ns

    stream_width
    stream_height

    roi
    preserve_aspect_ratio

    detections[] {
        detection_id_in_frame

        class_id
        class_name
        detector_score

        bbox_norm {
            x_min
            y_min
            x_max
            y_max
        }

        measured_anchor_norm {
            x
            y
        }

        optional keypoints[]
        optional pose_score
    }

    counters {
        raw_outputs
        class_filtered
        post_nms
        malformed_rejected
    }
}
```

All coordinates are normalized against the visible stream after correct inference-to-stream mapping.

No coordinate may be published without:

- source dimensions;
- coordinate convention;
- validity.

---

# 14. Coordinate correctness

The existing system has shown an impossible value in a field named radians.

The new vision subsystem applies fail-fast checks.

For normalized boxes:

```text
0 <= x_min < x_max <= 1
0 <= y_min < y_max <= 1
```

For anchors:

```text
0 <= x <= 1
0 <= y <= 1
```

For dimensions:

```text
width > 0
height > 0
```

For target angles, if this subsystem optionally calculates them:

```text
finite
within physically representable camera FOV
```

Do not publish a number with a validity flag omitted.

---

# 15. Person/class filtering

For human tracking:

```text
permitted class = person
```

Filter immediately after model normalization.

Do not create tracks for:

- chair;
- dog;
- potted plant;
- car;

unless the current model profile explicitly permits those classes.

Other object tracking can use a separate allowed-class profile.

This reduces:

- tracker load;
- target-selector clutter;
- irrelevant false identities.

---

# 16. Detector-level duplicate suppression

On-sensor `_pp` or `nms=True` output is preferred.

Host-side deduplication still remains as a defensive layer.

## 16.1 Class-aware NMS

For each permitted class:

1. sort by detector score;
2. select highest;
3. suppress overlapping lower-score boxes;
4. repeat.

Threshold is model-specific and evaluated on the station dataset.

Do not copy an arbitrary IoU threshold from a generic demo.

## 16.2 Containment suppression

Two boxes can describe one person while having only moderate IoU when one is nested inside another.

Calculate:

```text
intersection / min(area_a, area_b)
```

If containment is high and:

- class is equal;
- centers are close;
- scale relationship is plausible;

suppress or merge the lower-quality box.

## 16.3 Pose NMS

For pose profiles, use keypoint similarity in addition to box IoU.

## 16.4 Dedup counters

Publish:

```text
raw_detection_count
post_model_nms_count
host_duplicates_suppressed
containment_suppressed
pose_duplicates_suppressed
```

No silent duplicate removal.

---

# 17. Tracker decision: ByteTrack-style first

The first TrackManager shall use a ByteTrack-style two-stage association.

Reason:

- high-score detections establish/update tracks;
- lower-score detections can preserve an occluded track;
- low-score detections do not automatically create strong new identities;
- implementation is relatively lightweight for Pi 5;
- it directly addresses fragmented tracks caused by confidence fluctuation.

Do not copy the full upstream implementation blindly.

Implement the association behavior behind the project's own data structures and tests.

---

# 18. Tracker lifecycle

Use:

```text
TENTATIVE
CONFIRMED_VISIBLE
OCCLUDED
LOST_REACQUIRABLE
RETIRED
```

## 18.1 TENTATIVE

A new high-score detection creates a tentative track.

It is not selectable.

## 18.2 CONFIRMED_VISIBLE

Requires:

```text
minimum observation count
+
minimum visible duration
+
recent measurement
```

It is selectable and shown as a video candidate.

## 18.3 OCCLUDED

No high-score detection, but a plausible low-score association or a very short miss exists.

Selected target may coast according to controller policy.

## 18.4 LOST_REACQUIRABLE

No current measurement.

Identity remains internal for a bounded TTL.

It is not rendered as another visible person.

## 18.5 RETIRED

Identity and ephemeral appearance data are discarded.

---

# 19. Time-based lifecycle

Replace frame-count semantics.

Initial configuration:

```yaml
tracking:
  tentative:
    min_observations: 3
    min_visible_ms: 120
    max_gap_ms: 180

  occluded:
    max_ms: 350

  lost:
    retain_ms: 3000
```

These are starting engineering values.

They must be tuned using real detector cadence and recordings.

A minimum observation count remains useful.

Time is the primary semantic.

---

# 20. Association passes

## Pass 1 — high-confidence detections

Match to:

```text
CONFIRMED_VISIBLE
OCCLUDED
```

using:

- predicted position;
- IoU;
- center distance;
- scale/aspect change;
- optional appearance;
- optional camera-motion compensation.

## Pass 2 — lower-confidence detections

Attempt to recover unmatched established tracks.

Low-score detections:

- may update an existing track;
- may not immediately create a selectable new track.

## Pass 3 — new track creation

Only unmatched detections above a model-specific creation threshold create tentative tracks.

This reduces identity explosion from noise.

---

# 21. Association cost

Recommended normalized cost:

```text
C =
    w_motion * motion_distance
  + w_iou    * (1 - IoU)
  + w_scale  * scale_change
  + w_shape  * aspect_change
  + w_app    * appearance_distance
  + w_pose   * pose_distance
```

Terms are enabled only when available.

Before assignment, apply hard gates:

- wrong class;
- impossible displacement;
- impossible scale jump;
- incompatible pose;
- stale beyond reacquisition TTL.

Use Hungarian/min-cost assignment for the remaining matrix.

---

# 22. Motion model

Initial state per track:

```text
cx
cy
width
height
vx
vy
```

Use elapsed `dt` from `SensorTimestamp`.

Do not assume constant fps.

OC-SORT is a candidate if long occlusion and nonlinear motion remain problematic.

The baseline TrackManager should remain pluggable:

```text
AssociationEngine:
    BYTE_STYLE
    OC_SORT
```

Do not deploy full ReID-heavy BoT-SORT first.

---

# 23. Camera motion

A moving turret shifts every image-plane track.

The neural detector remains semantically valid, but TrackManager association can still suffer if all boxes jump.

Define:

```text
CameraMotionProvider
```

implementations:

```text
NONE
IMAGE_GMC
EXTERNAL_POSE_HINT
```

## 23.1 IMAGE_GMC

Estimate global image motion from background features.

Potential methods:

- sparse optical flow;
- affine transform;
- homography where appropriate.

Exclude detected person regions from background feature estimation.

This adds CPU load and must be profiled.

## 23.2 EXTERNAL_POSE_HINT

Optional generic input:

```text
CameraMotionHint {
    timestamp
    delta_yaw
    delta_pitch
}
```

The interface contains no motor brand or drive details.

The vision subsystem must still run without it.

## 23.3 Baseline policy

Start with:

```text
NONE
```

for stationary-camera tracker tests.

Then test:

```text
EXTERNAL_POSE_HINT
```

or image GMC during roaming/panning.

Do not use frame differencing as camera-motion compensation.

---

# 24. Ephemeral appearance descriptor

Use only if box/motion association is insufficient.

Recommended inexpensive descriptor:

```text
coarse HSV histogram
split into upper / lower body regions
```

or a similarly small non-biometric embedding.

Policy:

- memory only;
- no image crop persistence;
- no face embedding;
- no identity name;
- deleted when track retires;
- reset when `visiond` restarts;
- used only for short-term association/reacquisition.

Do not call this “person identification.”

It is short-lived visual track association.

---

# 25. Duplicate-track resolver

Detector NMS cannot prevent every identity split.

After association, detect two tracks likely representing one physical object.

Candidate duplicate condition:

```text
same class
+
high box overlap or containment
+
similar velocity
+
similar appearance/pose
+
persistent for a dwell
```

Do not merge after one frame.

## 25.1 Merge policy

Choose survivor by:

1. selected identity, if the evidence supports equivalence;
2. older confirmed track;
3. longer visible history;
4. higher recent association quality.

Create an alias:

```text
retired_uuid -> surviving_uuid
```

for the remaining session.

If the selected target's UUID is merged:

- TargetSelectionManager follows the alias atomically;
- UI receives a `TRACK_MERGED` event;
- AUTO_TRACK is not reset.

---

# 26. Track capacity

Set explicitly:

```yaml
max_tracks: 16
```

Initial eviction order:

1. RETIRED/stale;
2. oldest LOST_REACQUIRABLE, unselected;
3. weakest TENTATIVE;
4. weakest unselected confirmed track only if absolutely necessary.

Never evict:

```text
selected target
```

while its identity TTL is active.

Publish:

```text
track_capacity
track_capacity_used
detections_dropped_capacity
tracks_evicted
```

No silent truncation.

---

# 27. Display labels

Internal identity:

```text
track_uuid
```

Human display:

```text
Person #17
```

Rules:

- index increments monotonically for one `visiond` session;
- never reuse an index in that session;
- process restart creates a new `session_uuid`;
- display label is never accepted by selection API.

This removes ambiguity from label recycling.

---

# 28. Target-selection policy

## 28.1 Default: EXPLICIT_ONLY

The default target-selection policy is:

```text
operator must select a confirmed visible target
```

When multiple people are visible, the system does not invent a preference.

This is the clearest and most predictable behavior.

## 28.2 Optional: AUTO_SELECT_SINGLE

An optional policy may automatically select when:

```text
mode is AUTO_TRACK
AND no target is selected
AND exactly one eligible confirmed target exists
AND it remains the only eligible target for dwell_ms
AND detector/identity confidence passes high threshold
```

Initial candidate dwell:

```text
400–700 ms
```

If another eligible target appears during dwell:

```text
cancel auto-selection
```

Default:

```yaml
target_selection:
  policy: EXPLICIT_ONLY
```

## 28.3 Rejected default: AUTO_SELECT_BEST

Do not automatically choose:

- largest box;
- highest detector score;
- nearest image center;
- newest person;

when multiple people exist.

Any such rule will appear arbitrary and can switch subjects unexpectedly.

It may be added later as a named application policy.

---

# 29. Selection API

Never send:

```text
select_target 2
```

as the authoritative protocol.

Use:

```text
SelectTargetRequest {
    request_id
    track_uuid
    track_set_sequence_seen_by_ui
}
```

Response:

```text
SelectTargetAck {
    request_id
    accepted
    reason

    selected_track_uuid
    selected_display_label

    authoritative_track_set_sequence
}
```

Possible rejections:

```text
TRACK_NOT_FOUND
TRACK_NOT_CONFIRMED
TRACK_NOT_CURRENTLY_SELECTABLE
STALE_UI_TRACK_SET
TRACK_MERGED_USE_SURVIVOR
```

The browser displays selection only after authoritative state confirms it.

---

# 30. Idempotent selection

If:

```text
requested_uuid == already_selected_uuid
```

then:

```text
accepted = true
selection_unchanged = true
```

Do not:

- reset AutoTrack;
- clear estimator;
- restart acquisition;
- create a new selection generation.

This directly fixes the observed problem where repeated scripts reselected the same apparent person and repeatedly reset acquisition.

---

# 31. Selection persistence

Selection is independent of visibility.

State:

```text
NO_SELECTION
SELECTED_VISIBLE
SELECTED_OCCLUDED
SELECTED_LOST_REACQUIRABLE
SELECTED_STALE
SELECTED_AMBIGUOUS
```

When selected track becomes LOST:

- selection remains;
- selected target card shows LOST;
- no duplicate visible box is drawn;
- new targets are not automatically substituted.

When identity TTL expires:

```text
SELECTED_STALE
```

Operator can:

- clear;
- reselect;
- wait for an application-specific long-term ReID feature added later.

---

# 32. Ambiguous reacquisition

If two candidates have similar reacquisition scores:

```text
SELECTED_AMBIGUOUS
```

Behavior:

- do not select either;
- do not change UUID;
- publish ambiguity candidates for UI/diagnostics;
- retain LOST selection until TTL;
- controller receives no valid new measurement.

Do not encode ambiguity by simply multiplying a generic confidence by 0.5.

Publish it explicitly.

---

# 33. TargetSelectionManager state

```text
TargetSelectionState {
    selection_generation

    optional selected_track_uuid
    optional selected_display_label

    selection_status
    target_class

    last_measured_timestamp_ns
    age_since_measurement_ms

    identity_confidence
    association_quality

    ambiguous
    ambiguity_candidates[]

    optional alias_resolution
}
```

---

# 34. Selected target output

The controller-facing output is:

```text
SelectedTargetObservation {
    protocol_version

    selection_generation
    track_uuid

    frame_sequence
    sensor_timestamp_ns
    publish_timestamp_ns

    target_state:
        CONFIRMED_VISIBLE
        OCCLUDED
        LOST
        AMBIGUOUS

    measurement_valid

    bbox_norm
    measured_anchor_norm

    detector_score
    association_quality
    identity_confidence

    ambiguity
    just_reacquired

    class_id
    class_name
}
```

No motor lead or motor command is included.

---

# 35. Aim anchor

Identity box and aim anchor are separate.

For a person detector:

```text
x = bbox horizontal center
y = configurable fraction from top
```

Initial candidate:

```text
y = 0.42–0.48 of bbox height
```

to target upper torso rather than legs.

Tune from recordings.

For pose:

- use shoulder/torso center;
- fall back to bbox anchor when keypoints are weak.

Publish:

```text
anchor_source:
    BBOX_TORSO
    POSE_SHOULDERS
    POSE_TORSO
    BBOX_CENTER_FALLBACK
```

---

# 36. Anchor stability

Do not heavily smooth the measured anchor in `visiond`.

TrackManager may provide a lightly filtered box for UI/association.

The controller receives:

- measured anchor;
- optional track velocity;
- timestamp;
- quality.

The motor-control subsystem owns final target-motion prediction.

This prevents prediction being applied twice.

---

# 37. Confidence architecture

Replace one opaque value with explicit fields.

```text
detector_score
    model output

association_quality
    how well this detection matched the track

identity_confidence
    continuity/uniqueness confidence

measurement_quality
    bbox/keypoint suitability for aim

ambiguity
    explicit boolean/score

track_state
    lifecycle state
```

## 37.1 Selectable gate

A target is selectable when:

```text
state == CONFIRMED_VISIBLE
AND detector_score >= model.select_min
AND identity_confidence >= identity.select_min
AND not duplicate_resolving
AND not ambiguous
```

## 37.2 Model-specific thresholds

Never use one universal `0.50` across:

- simple detector;
- YOLO;
- NanoDet;
- SSD;
- pose.

Each model profile has:

```yaml
score_thresholds:
  low_association:
  new_track:
  confirmed_update:
  selectable:
```

Tune from validation data.

---

# 38. One authoritative target lifecycle

The vision subsystem publishes:

```text
track_state
selection_status
```

The controller independently publishes:

```text
auto_track_phase
```

HUD example:

```text
TARGET: OCCLUDED
AUTO TRACK: COASTING
```

Do not translate an older v1 tracking FSM into a v3 phase.

Deprecate duplicated state sources.

---

# 39. Preview isolation

The current frame tap reduces the detector pipeline from approximately 15 fps to approximately 10 fps.

The production architecture uses:

```text
one camera owner
    |
    +--> inference metadata / detection
    |
    +--> latest-only preview buffer
             |
             v
        independent encoder/web worker
```

Rules:

- preview cannot block detection;
- preview queue depth is one;
- old preview frame is overwritten;
- preview may be 10–15 fps;
- inference runs at model-native rate;
- browser HUD is rendered client-side.

---

# 40. Per-stage timing

Every frame records:

```text
sensor_timestamp
metadata_receive_timestamp
model_output_parse_start/end
coordinate_normalization_start/end
dedup_start/end
association_start/end
selection_update_start/end
publish_timestamp
preview_enqueue_timestamp
```

Report:

```text
p50
p95
p99
max
```

for each stage.

Do not call publish→controller time “inference latency.”

---

# 41. Association diagnostics

Add structured per-frame diagnostics in a bounded debug mode.

For each detection:

```text
detection_id
candidate tracks
gate pass/fail
cost terms
assigned track
new-track reason
duplicate-suppression reason
```

For each track:

```text
state before/after
matched detection
miss age
reacquisition score
ambiguity
merge decision
```

This directly resolves the handover's uncertainty about why a selected identity stops receiving measurements.

Do not enable verbose per-frame logs indefinitely in production.

Use a ring buffer and persist on fault/selection-loss event.

---

# 42. Event log

Add:

```text
MODEL_LOADED
MODEL_REJECTED_INCOMPATIBLE

DETECTION_DUPLICATE_SUPPRESSED
TRACK_CREATED
TRACK_CONFIRMED
TRACK_OCCLUDED
TRACK_LOST
TRACK_REACQUIRED
TRACK_RETIRED
TRACK_MERGED
TRACK_CAPACITY_DROP

TARGET_SELECTED
TARGET_SELECTION_REJECTED
TARGET_SELECTION_IDEMPOTENT
TARGET_CLEARED
TARGET_AMBIGUOUS
TARGET_STALE
```

---

# 43. Recorded-data architecture

Add:

```text
visiond --record-dataset
visiond --replay <recording>
```

A recording must preserve:

```text
frame or encoded image
SensorTimestamp
camera metadata required for coordinate conversion
raw model outputs or normalized DetectionSet
model ID/hash
configuration
```

Two replay levels:

## Level A — detector replay

Recorded images pass through the model where feasible.

Useful for model comparison.

## Level B — detection replay

Stored DetectionSets pass through:

```text
dedup
TrackManager
selection
```

Useful for deterministic association tests without IMX500 hardware.

---

# 44. Test scenario set

Record and annotate:

1. Empty scene for 10 minutes.
2. One stationary person.
3. One person walking horizontally.
4. One person moving toward/away.
5. Person stopping and reversing.
6. Brief occlusion:
   - 100 ms;
   - 300 ms;
   - 700 ms;
   - 2 s.
7. Person leaves and re-enters.
8. Two people separated.
9. Two people crossing.
10. Two people overlapping.
11. One person partly hidden.
12. Person plus dog/cat.
13. Backlit person.
14. Low-light person.
15. Similar clothing.
16. Different clothing.
17. Camera yaw/pitch movement with stationary people.
18. AUTO_ROAM-like continuous camera movement.
19. Curtains/tree motion with no person.
20. Reflections/screens/posters as hard negatives.

Ground truth includes:

```text
person boxes
stable ground-truth ID
visibility/occlusion
selected target ID
```

---

# 45. Evaluation metrics

## Detector

```text
precision
recall
false positives per minute
missed-person frames
duplicate detections per person-frame
box jitter
first detection latency
inference cadence
```

## Tracker

```text
ID switches
fragmentations
duplicate active identities
selected-track retention
reacquisition correctness
reacquisition latency
IDF1/HOTA if evaluation tooling supports them
```

## Selection

```text
request ACK correctness
wrong-subject selection count
stale-label race count
same-UUID reselection resets
target stealing count
ambiguous reacquisition behavior
```

## Performance

```text
model inference rate
vision pipeline p95/p99
CPU
memory
preview impact
TrackSet publish rate
```

---

# 46. Initial engineering acceptance gates

These are starting targets, subject to operator revision.

## Empty scene

```text
false selectable people:
    0 preferred
    <1 per 10 minutes initial maximum
```

## One-person scene

```text
duplicate visible candidate rate:
    <1% of person-visible frames

ID switches:
    0

selection mismatch:
    0
```

## Selection API

```text
requesting UUID A:
    selects A or rejects
    never selects B
```

## Confirmation

```text
p95 from first valid detection to selectable:
    <=250 ms preferred
```

subject to model inference rate.

## Occlusion

```text
300 ms occlusion:
    selected identity retained in >=95% of trials
```

## Crossing people

```text
selected target stealing:
    0 in acceptance set
```

If zero cannot be achieved with the lightweight tracker, add ephemeral appearance/pose before relaxing it.

---

# 47. Model selection scorecard

Do not choose solely by COCO mAP or FPS.

A model must pass all mandatory gates:

```text
API-compatible
stable class mapping
acceptable false-positive rate
acceptable duplicate rate
selection continuity
acceptable latency
acceptable license
```

Then choose based on:

1. selected-target retention;
2. duplicate-free output;
3. reacquisition correctness;
4. confirmation latency;
5. inference rate;
6. CPU/preview impact.

A faster model that repeatedly changes identity is not better for this application.

---

# 48. License decision

Current official model-zoo licenses differ:

```text
YOLO11n / YOLOv8n:
    AGPL-3.0 model distribution in the Raspberry Pi zoo

NanoDet:
    Apache-2.0

EfficientDet Lite0:
    Apache-2.0

SSD MobileNetV2 FPN Lite:
    MIT
```

The project owner must choose a model whose license fits the intended distribution/deployment.

Do not treat model license as an implementation afterthought.

---

# 49. Source tree

Recommended:

```text
vision/
├── visiond.py
├── model/
│   ├── adapter.py
│   ├── imx500_yolo.py
│   ├── imx500_nanodet.py
│   ├── imx500_ssd.py
│   ├── imx500_pose.py
│   ├── manifest.py
│   └── compatibility_probe.py
│
├── detection/
│   ├── types.py
│   ├── normalize.py
│   ├── class_filter.py
│   ├── nms.py
│   ├── containment.py
│   └── pose_nms.py
│
├── tracking/
│   ├── track.py
│   ├── track_manager.py
│   ├── byte_association.py
│   ├── oc_sort_association.py
│   ├── camera_motion.py
│   ├── appearance.py
│   ├── duplicate_track_resolver.py
│   └── diagnostics.py
│
├── selection/
│   ├── target_selection_manager.py
│   ├── policy.py
│   ├── protocol.py
│   └── alias_map.py
│
├── protocol/
│   ├── detection_set.py
│   ├── track_set.py
│   └── selected_target.py
│
├── replay/
│   ├── recorder.py
│   ├── replay_source.py
│   └── evaluator.py
│
└── tests/
```

No file in this tree imports:

```text
CyberGear
SocketCAN
motor backend
```

Protocol packages may be shared with the controller.

---

# 50. Configuration example

```yaml
vision:
  profile: person_detect

  camera:
    frame_rate: model_intrinsics
    preserve_aspect_ratio: true

  preview:
    enabled: true
    fps: 10
    latest_queue_depth: 1

  models:
    person_detect:
      adapter: imx500_yolo
      path: /usr/share/imx500-models/imx500_network_yolo11n_pp.rpk
      manifest: config/models/yolo11n_pp.yaml
      permitted_classes: [person]

      thresholds:
        low_association: COMMISSION
        new_track: COMMISSION
        confirmed_update: COMMISSION
        selectable: COMMISSION

  dedup:
    class_aware_nms: true
    nms_iou: COMMISSION
    containment: true
    containment_ratio: COMMISSION
    center_distance_norm: COMMISSION

  tracking:
    engine: byte_style

    tentative:
      min_observations: 3
      min_visible_ms: 120
      max_gap_ms: 180

    occluded:
      max_ms: 350

    lost:
      retain_ms: 3000

    appearance:
      enabled: false
      type: hsv_upper_lower
      persist: false

    camera_motion:
      provider: none

    max_tracks: 16

  selection:
    policy: explicit_only
    auto_select_single_dwell_ms: 500
    display_index_reuse: never_within_session
```

The implementation shall reject unresolved `COMMISSION` values for a production profile.

---

# 51. Implementation phases

## Vision-0 — preserve evidence and isolate subsystem

- create replay/mocked consumer boundary;
- preserve current simple detector under `SIMPLE_DIAGNOSTIC`;
- prevent it from being the default AUTO_TRACK detector;
- add model/environment manifest.

## Vision-1 — restore official neural baseline

- update to current Picamera2 IMX500 API;
- run official YOLO11n `_pp` example;
- integrate `Imx500ModelAdapter`;
- validate normalized boxes and timestamps;
- filter to `person`.

Deliverable:

```text
multiple people -> multiple valid Detection records
```

## Vision-2 — detector deduplication

- on-sensor PP/NMS validation;
- host class-aware NMS;
- containment suppression;
- dedup counters.

Deliverable:

```text
one person does not create multiple current detection boxes
```

## Vision-3 — time-based tracker

- ByteTrack-style two-pass association;
- timestamp-based prediction;
- no frame-count lifecycle;
- no display-index reuse;
- selected track protected from eviction.

Deliverable:

```text
stable UUIDs on recorded single and two-person sequences
```

## Vision-4 — target selection repair

- UUID-based commands;
- authoritative ACK;
- idempotent same-target selection;
- selection persists through loss;
- no AutoTrack reset on same UUID;
- alias map on track merge.

Deliverable:

```text
request UUID A -> select A or reject, never B
```

## Vision-5 — diagnostics

- per-assignment ring buffer;
- lifecycle and merge events;
- explicit target state;
- remove v1/v3 tracking-state ambiguity;
- value/unit sanity checks.

Deliverable:

```text
every identity loss has an observable cause
```

## Vision-6 — model bake-off

- YOLO11n;
- NanoDet PP;
- EfficientDet Lite0 PP;
- SSD MobileNet PP;
- optional pose.

Deliverable:

```text
evidence-based selected model profile
```

## Vision-7 — camera-motion testing

- panning/roaming recordings;
- optional pose hint or image GMC;
- select the cheapest method that prevents ID fragmentation.

## Vision-8 — custom person model

Only if official models fail quality/latency requirements:

- build dataset;
- fine-tune one-class model;
- export/quantize/package;
- validate quantization;
- repeat acceptance suite.

---

# 52. Tests

## Unit tests

- model manifest validation;
- bbox order/normalization;
- ROI/aspect mapping;
- NMS;
- containment suppression;
- pose dedup;
- time-based track lifecycle;
- ByteTrack high/low association;
- display index monotonicity;
- UUID selection;
- stale UI request;
- idempotent reselection;
- track merge alias;
- selected-track eviction protection;
- ambiguity.

## Deterministic replay tests

- duplicate boxes in one frame;
- detector score dips;
- one-frame miss;
- multi-frame occlusion;
- same person reappears outside old spatial gate;
- two people cross;
- old LOST identity plus new detection;
- display index race;
- camera pan;
- reordered/delayed metadata.

## Hardware tests

- exact packaged model on IMX500;
- timestamp monotonicity;
- inference cadence;
- multiple live people;
- preview on/off;
- long-running memory/track capacity.

---

# 53. Target-selection decision table

| Situation | Decision |
|---|---|
| No target selected, no candidates | WAIT_TARGET |
| No target selected, one candidate, EXPLICIT_ONLY | wait for operator |
| No target selected, one candidate, AUTO_SELECT_SINGLE | select after stable dwell |
| No target selected, multiple candidates | wait for operator |
| Operator clicks visible confirmed UUID | select exact UUID |
| Operator re-clicks same UUID | acknowledge idempotently; no reset |
| Operator submits stale/retired UUID | reject |
| Selected target briefly occluded | retain selection |
| Selected target LOST but reacquirable | retain selection, no measurement |
| Two plausible reacquisition candidates | ambiguous; choose neither |
| Duplicate track merged into survivor | atomically follow alias |
| Display label reused or stale | irrelevant; protocol uses UUID |
| AUTO_ROAM sees selected target | selection remains; roaming continues |
| AUTO_TRACK sees selected target | valid measurements published automatically |

---

# 54. Explicitly rejected approaches

Do not:

- use frame differencing as production person detection;
- stamp an arbitrary motion blob as `person`;
- track display indices;
- recycle display labels in one session;
- show LOST tracks as live candidate boxes;
- use one universal confidence threshold for every detector;
- multiply every uncertainty into one opaque score;
- reset AutoTrack when the same UUID is selected again;
- automatically pick the “largest” person when several exist;
- allow low-confidence detections to create immediate selectable tracks;
- rely only on center distance for association;
- rely only on IoU for reacquisition;
- persist face/appearance embeddings;
- run a heavy ReID model before lightweight association is tested;
- apply motor prediction in both vision and control;
- let preview block inference;
- accept a custom quantized model without post-quantization validation;
- compare model-zoo mAP and Ultralytics sample mAP as equivalent measurements;
- declare completion using synthetic/fake targets only.

---

# 55. Completion criteria

The independent perception/selection work is complete when:

1. An official neural IMX500 model runs through the current supported API.
2. Multiple people produce multiple semantic detections.
3. Camera motion does not create fake people.
4. One person does not appear as multiple visible candidates in normal operation.
5. LOST retained identities are not rendered as additional visible people.
6. Target identity uses UUID.
7. Display labels are never selection keys.
8. Selecting target A selects A or rejects; never B.
9. Re-selecting A is idempotent.
10. Selected identity survives ordinary short detector score dips.
11. Selected identity does not switch to another person during crossing.
12. Ambiguous reacquisition chooses neither.
13. Track lifecycle is time-based.
14. Every association/identity loss can be diagnosed from a bounded trace.
15. Target state and AutoTrack phase are separate in telemetry.
16. Preview does not measurably reduce inference cadence.
17. A real-model bake-off has selected the deployed profile.
18. All perception acceptance tests run with motors disabled.
19. The controller consumes only the documented `SelectedTargetObservation`.
20. No CyberGear/control implementation decision is required to complete this subsystem.

---

# 56. Immediate next action for the local agent

Do not modify motor control.

Do not tune the current simple detector further as the production answer.

The next steps are:

```text
1. capture the current package/version manifest;
2. replace stale IMX500 API usage;
3. run the official YOLO11n-PP Picamera2 example;
4. integrate it behind Imx500ModelAdapter;
5. publish person-only DetectionSet with valid timestamps/coordinates;
6. add host dedup;
7. replace display-index selection with UUID selection;
8. add association decision logging;
9. run recorded one-person and two-person tests;
10. only then compare alternate models or train a custom person model.
```

---

# 57. Research references

## Raspberry Pi AI Camera and model deployment

- Raspberry Pi AI Camera documentation:  
  `https://www.raspberrypi.com/documentation/accessories/ai-camera.html`

- Raspberry Pi IMX500 model zoo:  
  `https://github.com/raspberrypi/imx500-models`

- Current Picamera2 IMX500 object-detection example:  
  `https://github.com/raspberrypi/picamera2/blob/main/examples/imx500/imx500_object_detection_demo.py`

- Current Picamera2 HigherHRNet pose example:  
  `https://github.com/raspberrypi/picamera2/blob/main/examples/imx500/imx500_pose_estimation_higherhrnet_demo.py`

## Sony/AITRIOS

- Sony Application Module Library for Raspberry Pi AI Camera:  
  `https://github.com/SonySemiconductorSolutions/aitrios-rpi-application-module-library`

- Sony Raspberry Pi sample applications:  
  `https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-apps`

## Ultralytics IMX500 export

- Official Ultralytics Sony IMX500 integration:  
  `https://github.com/ultralytics/ultralytics/blob/main/docs/en/integrations/sony-imx500.md`

## Multi-object tracking

- ByteTrack, ECCV 2022:  
  `https://www.ecva.net/papers/eccv_2022/papers_ECCV/html/315_ECCV_2022_paper.php`

- OC-SORT:  
  `https://arxiv.org/abs/2203.14360`

- BoT-SORT:  
  `https://arxiv.org/abs/2206.14651`

## Person dataset

- CrowdHuman:  
  `https://arxiv.org/abs/1805.00123`

---

# Appendix A — Target selector pseudocode

```python
def select_target(request: SelectTargetRequest, latest: TrackSet) -> SelectTargetAck:
    track = latest.find_uuid(request.track_uuid)

    if request.track_uuid == selection.selected_track_uuid:
        return SelectTargetAck(
            accepted=True,
            reason="selection unchanged",
            selected_track_uuid=request.track_uuid,
            selection_unchanged=True,
        )

    if track is None:
        alias = alias_map.resolve(request.track_uuid)
        if alias is not None:
            track = latest.find_uuid(alias)

    if track is None:
        return reject("TRACK_NOT_FOUND")

    if track.state is not CONFIRMED_VISIBLE:
        return reject("TRACK_NOT_CURRENTLY_SELECTABLE")

    if track.ambiguous or track.duplicate_resolving:
        return reject("TRACK_IDENTITY_UNRESOLVED")

    selection.commit(track.uuid)
    return accept(track.uuid, track.display_label)
```

---

# Appendix B — Track rendering rule

```python
def visible_candidates(track_set):
    return [
        track
        for track in track_set.tracks
        if track.state == CONFIRMED_VISIBLE
        and track.selectable
        and not track.duplicate_resolving
    ]
```

Do not render:

```text
OCCLUDED
LOST_REACQUIRABLE
RETIRED
```

as additional live people.

The selected LOST identity appears only in:

- target summary;
- optional field-of-regard last/predicted marker;
- diagnostics.

---

# Appendix C — ByteTrack-style update

```python
def update_tracks(high_detections, low_detections, timestamp):
    predicted = predict_tracks_to(timestamp)

    # Pass 1: stable high-quality detections.
    matches_1, unmatched_tracks, unmatched_high = assign(
        tracks=predicted.active_and_occluded,
        detections=high_detections,
        cost=association_cost,
        hard_gates=association_gates,
    )

    apply_matches(matches_1)

    # Pass 2: low-score boxes may rescue existing tracks.
    matches_2, still_unmatched, _ = assign(
        tracks=unmatched_tracks,
        detections=low_detections,
        cost=low_score_association_cost,
        hard_gates=stricter_low_score_gates,
    )

    apply_matches(matches_2)
    age_unmatched(still_unmatched, timestamp)

    # Only unmatched HIGH detections can create tentative tracks.
    for det in unmatched_high:
        if det.score >= config.new_track_threshold:
            create_tentative_track(det, timestamp)

    resolve_duplicate_tracks()
    enforce_capacity()
```

---

# Appendix D — Model adapter compatibility probe

```python
def probe_model(model_path: str) -> ModelManifest:
    imx500 = IMX500(model_path)
    intrinsics = imx500.network_intrinsics

    assert intrinsics is not None
    assert intrinsics.task in {"object detection", "pose estimation"}
    assert intrinsics.inference_rate is not None
    assert intrinsics.labels is not None

    return ModelManifest(
        task=intrinsics.task,
        inference_rate=intrinsics.inference_rate,
        bbox_order=getattr(intrinsics, "bbox_order", None),
        bbox_normalization=getattr(intrinsics, "bbox_normalization", None),
        preserve_aspect_ratio=intrinsics.preserve_aspect_ratio,
        labels=intrinsics.labels,
    )
```

The production code must handle model-specific output explicitly rather than assuming every network has identical tensors.

---

# Appendix E — Model bake-off report template

```yaml
model:
  id:
  path:
  hash:
  license:
  task:
  input:
  postprocess:

runtime:
  inference_hz_p50:
  inter_result_ms_p95:
  parse_ms_p95:
  dedup_ms_p95:
  association_ms_p95:
  cpu_percent:
  preview_impact_percent:

detector:
  precision:
  recall:
  false_positive_per_minute:
  duplicate_person_frame_percent:
  first_detection_ms_p95:
  confirmation_ms_p95:

tracking:
  id_switches:
  fragmentations:
  duplicate_active_track_percent:
  selected_retention_percent:
  reacquisition_correct_percent:
  reacquisition_ms_p95:

selection:
  wrong_uuid_selection_count:
  stale_request_rejection_count:
  same_uuid_reset_count:
  target_stealing_count:

decision:
  pass_mandatory_gates:
  selected_for_profile:
  reason:
```
