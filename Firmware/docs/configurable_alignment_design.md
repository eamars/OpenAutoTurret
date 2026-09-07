# Configurable measurement point and virtual laser alignment

Implemented 2026-09-07: startup configuration, native/legacy aiming policy,
production C++ geometry and reference integration, telemetry and HUD rendering.
**Verified in simulation; physical alignment and real detector performance are
not verified.** No video feed, physical station, rangefinder or IMU was used.

## Configure and tune

Edit [turret.yaml](../config/turret.yaml). Values are loaded at startup:

~~~yaml
tracking:
  aim_point:
    mode: box_fraction
    x_fraction: 0.50
    y_fraction: 0.22

alignment:
  mode: off
  camera_from_laser_mm: {right: 75, up: 75, forward: 0}
  laser_axis_deg: {right: 0, up: 0}
  assumed_depth_m: 10
~~~

These are blocks within the existing configuration, not a complete replacement
file. The shipped measurement point explicitly uses 22% on **both native and
legacy observation paths**. Previously, native observations bypassed the head
setting and followed perception's anchor (45% box fallback, or pose anchor).

| Setting | Meaning |
|---|---|
| tracking.aim_point.mode | box_fraction selects a point in the detected box. perception_anchor follows the point supplied by perception. |
| x_fraction | Fraction from the box's left edge; [0,1]. 0.50 is horizontally centred. |
| y_fraction | Fraction down from its top edge; [0,1]. 0.22 is the requested head/framing approximation. |
| alignment.mode | off points the camera optical axis. manual_depth points a virtual laser sight at an assumed depth. No measured-range mode exists yet. |
| camera_from_laser_mm.right/up/forward | Camera displacement from the laser, expressed in corrected camera axes; positive right/up/forward. The 75/75/0 values are provisional mounting inputs. |
| laser_axis_deg.right/up | Independent horizontal/vertical laser angles relative to the optical axis. Positive projects right/up. Each must be strictly inside +/-45 degrees. Zero assumes parallel axes. |
| assumed_depth_m | Positive distance to a plane along the camera optical axis. **10 m is an example, not a measured range or recommended operating distance.** |

The absent laser is why compensation ships off. To exercise compensation in
simulation or a later controlled trial, use manual_depth and deliberately choose
the reference depth. The HUD shows an amber laser crosshair labelled
**ASSUMED 10.0 m** (or the configured value), a small camera-centre marker and a
white diamond at the requested measurement point. Invalid/stale alignment does
not produce a valid laser marker. Telemetry always reports range_measured=false.

Tuning is **startup-only**. No live settings editor, Apply command or Save endpoint
is implemented. File edits take effect on the next normal restart. This avoids
interpreting a changed box fraction as target velocity or changing reference
geometry halfway through tracking. Active changes to aim/alignment through the
internal startup setter are rejected; idempotent calls remain allowed. Runtime
Apply/Save was a future extension in the proposal and remains separate work.

When hardware operation is appropriate later, follow
[STATION_OPERATIONS.md](STATION_OPERATIONS.md) for committed-source deployment
and controlled restart. Nothing was deployed or activated in this session.

## Measurement point and compatibility

For a validated selected box (xmin, ymin, xmax, ymax):

~~~
u_norm = xmin + x_fraction * (xmax - xmin)
v_norm = ymin + y_fraction * (ymax - ymin)
~~~

Perception's association anchor, identity, selection authority, confidence and
capture timestamp remain independent. The controller applies box_fraction after
validating the observation, including when the native wire marks its anchor
authoritative. Pose anchors do not override an explicit box fraction.

An unusable box falls back to a valid perception anchor with source
invalid_box_anchor_fallback. An invalid anchor rejects the measurement. Valid
boxes touching an image edge are marked target_aim_box_clipped; the code does not
infer missing anatomy or silently move the requested point. The measurement
marker expires when its measurement becomes stale.

A percentage is **not anatomical head localization**. Moving toward 50% balances
framing above and below the object but changes the measurement location. Because
camera and laser are rigidly mounted, software cannot independently centre the
complete object and place the laser at an arbitrary location on it.

Missing new blocks preserve legacy behavior. An explicit tracking.aim_point
block overrides aim_at_head/head_fraction_from_top, with a warning when both
old and new keys are present. Malformed, incomplete or unknown new settings
are rejected. Explicit aim policies require valid camera intrinsics; active laser
alignment additionally requires valid camera extrinsics and an in-frame projected
sight. The daemon checks these before opening its motor transport.

## Geometry and distance limitation

The corrected detector frame is C = (right, down, forward). In metres:

~~~
o_C = (-camera_right_mm, +camera_up_mm, -camera_forward_mm) / 1000
d_C = normalize(tan(laser_right_angle), -tan(laser_up_angle), 1)
s = (assumed_depth_m - o_C.z) / d_C.z
p_C = o_C + s * d_C
r_sight_C = normalize(p_C)
u_laser = cx + fx * p_C.x / p_C.z
v_laser = cy + fy * p_C.y / p_C.z
~~~

The reference plane must be ahead of both devices. Geometry uses the corrected
detector frame once; it does not apply the existing rotate_180 correction again
to the marker. An orientation or mount change needs calibration checked.

At 10 m, a parallel 75/75 mm mount with the stored 1920x1080 intrinsics projects
to (949.5825, 551.0025) pixels, left/below the optical axis (960, 540).
Camera intrinsics are not altered to create this offset.

Automatic pointing aligns R_WC(q) * r_sight_C with the estimated target LOS.
The bounded and iterative solvers, internal reference path and joint-rate
feed-forward use the same sight ray. The shared ReferenceManager honors it only
for AutoTrack intents. Manual, roaming, homing and parking retain their existing
reference interpretation and limits. An unreachable sight produces the existing
unreachable/hold result.

For a rigid mount and fixed assumed depth, the crosshair stays at a fixed image
location while the assembly rotates. It changes when depth or alignment changes.
Translation's angular effect shrinks with depth; mounting-angle error remains.

Without measured range, correct alignment across arbitrary distances cannot be
guaranteed. The object can sit on the virtual crosshair while the beam misses.
For the parallel 75/75 mm mount configured at **10 m**, the reference simulation
gives these diagonal errors:

| Actual camera-axis depth | Beam miss |
|---|---:|
| 2 m | 84.853 mm |
| 5 m | 53.033 mm |
| 10 m | 0 mm |
| 30 m | 212.132 mm |
| 100 m | 954.594 mm |

This is a direction-only model, consistent with the existing LOS stack.
Simulation assumes the camera origin is at the rotation pivot and uses ideal
pinhole projection. Camera-to-pivot translation, axis-origin translation, mount
flex and lens distortion are not established here. Stored calibration includes
provisional values. The t_P_C camera-to-mechanism translation is not repurposed as
the camera-to-laser offset.

No IMU is needed for base-relative encoder geometry. This does not provide
inertial stabilization of a moving base. A future rangefinder return is laser
slant range rho, giving p_C = o_C + rho*d_C; its camera depth is p_C.z. That
integration also needs timestamps, validity and association with the selected
object, since a background return is not the object's distance.

## Implementation map

| Code | Responsibility |
|---|---|
| config/turret_config.*, config/tracking_setup.hpp, main.cpp | Strict YAML parsing, legacy precedence, shared boot setup and validation before opening motors. |
| tracking/aim_point.hpp, control/tracking_controller.hpp | Explicit measurement point on native/legacy paths; point source, clipping and freshness. |
| geometry/laser_alignment.hpp | Pure geometry for mounting offsets, assumed depth and projected virtual laser marker. |
| geometry/los_joint_solver.hpp, control/motion_intent.hpp, control/reference_manager.hpp, control/control_loop.cpp | Configured sight through the active control path and bounded motion references. |
| telemetry/telemetry.hpp, web/web_server.hpp, webd/protocol.py | Policy revision, effective parameters, marker projection, validity/source and assumed-depth status. Available even without a vision transport. |
| webd/hud.py | Controller-projected laser crosshair, measurement-point diamond and separate optical-axis marker. |

## Simulation and regression evidence

The first production probe exercised **45 scenes** through the actual C++
TrackingController and ReferenceManager using stored camera calibration.
Authoritative-anchor measurements exercised the native-path policy override.
Maximum image discrepancy was **0.000163 px**, with maximum geometric beam miss
**0.011731 mm**; float32 box/anchor fields account for the difference from the
double-precision reference model.

The integrated test goes further: encoded native perception bytes pass through
the production decoder, ControlLoop, estimator, reference generation and
SimMotorBackend. Each run homes simulated axes, feeds a fixed synthetic world
point at 25 Hz while control runs at 200 Hz, then removes measurements.
Compensation on/off and fractions 22%, 45% and 60% form six scenarios. Final
image error was **0.2087-0.2092 px**. Enabled cases also pass an independent
world-space beam-miss gate of **4 mm at 2 m**. Loss handling remains nonfaulting;
active policy changes are rejected while idempotent configuration calls pass.

Checks performed:

- Full daemon and C++ test build in local Ubuntu WSL.
- **68/68 CTest entries passed**, including the production alignment probe,
  configuration compatibility, calibration, solver, native simulation and
  existing control/reference regressions.
- **78 selected Python/HUD tests passed**, executing actual HUD JavaScript under
  Node and exercising Python telemetry serialization and fake web services.
  Tests use synthetic frames/services, never the video feed.
- Telemetry captured from the actual C++ simulated-motor run was passed through
  Python's protocol and the HUD JavaScript. Policy, marker position and the
  ASSUMED 2.0 m label survived that complete data path unchanged.
- The earlier Python reference model provides 180 ideal geometry scenes,
  a wrong-depth sweep and a synthetic framing comparison. Its tiny numeric
  residuals are mathematical results, not physical accuracy claims.

The synthetic large-box example clips after alignment at 22% and 35%, but remains
in frame at 45% and 50%. That illustrates a tradeoff only; it does not establish
real detection rates, tracking loss, or a preferred operating percentage.

## Reproduce locally

From the repository root in Linux/WSL, with CMake, a C++20 compiler, yaml-cpp,
spdlog and GoogleTest installed:

~~~bash
cmake -S Firmware -B run/alignment-build/cmake -DOTA_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build run/alignment-build/cmake -j 6
ctest --test-dir run/alignment-build/cmake --output-on-failure -j 4
~~~

The alignment_geometry_probe CTest entry automatically runs in Firmware/ so it
can load calibration files. For the full simulated-motor path alone:

~~~bash
run/alignment-build/cmake/control/test_tracking_integration --gtest_filter=AlignmentSimulation.*
~~~

The optional Python reference tool remains tunable without starting the stack:

~~~powershell
& run/takeover-analysis-venv/Scripts/python.exe Firmware/tools/probe_alignment_design.py --depth-m 30 --y-fraction 0.35 --output run/alignment-design/trial.json
~~~

For its mounting inputs, copy
[alignment_design.json](../tools/fixtures/alignment_design.json) into ignored
run/, edit it, and pass --config. That JSON is a reference-model fixture,
distinct from production YAML.

Python checks in WSL use the project-local run/alignment-venv-linux environment
(FastAPI, uvicorn, httpx, PyYAML, numpy, Pillow, pytest; Node runs the JavaScript).
From Firmware/, invoke that venv's Python with -m pytest and the selected
web/webd/tests/test_alignment.py, test_reticle_anchor.py, test_hud.py,
test_visual_discipline.py, test_protocol.py, test_dashboard_js_parses.py, and
test_prediction_cue.py files. Venvs, binaries and reports stay under ignored run/.

Do not use the station launcher's --sim for these tests: it still opens the
real camera. The probes above never start the station stack.

## Remaining physical verification

Simulation verifies implemented software behavior within the stated model. It
does not establish actual laser accuracy, detector visibility, real motor
stability or video-overlay registration. Later commissioning needs nonliving
calibration targets at known depths and measured mounting angles/translations.
The provisional camera calibration, IMU absence and missing range sensor remain
explicit limitations.

