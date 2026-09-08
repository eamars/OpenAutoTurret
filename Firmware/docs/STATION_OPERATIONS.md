# Deploy and operate the camera station

This is the current operating runbook. Use this procedure rather than dated
commissioning scripts or the legacy individual systemd units.

**Current commissioning setting (8 September 2026):** after the 20:58 monitored
home stopped on approximately 0.39 degrees of pitch encoder recoil during mode
setup, the operator requested disabling that check and proceeding to response
tuning. `homing.mode_displacement_check: false` now omits the optional 0.25-degree
displacement gate in homing mode recipes, including final service-mode setup.
The next attempt at 21:16 stopped on the separate speed gate. The operator then
required the added motion checks to warn and continue homing. The station now
also sets `homing.motion_checks_abort: false`: speed, corridor and reverse-motion
observations log warnings without changing the homing state or motor commands.
The same switch restores the preceding arrival/settling procedure in the endpoint
FSM: timed settling, the prior backoff arrival window and fine-approach travel
bounds. Added stationary-window, clearance and coarse/fine comparison gates do
not abort this procedure. Existing contact, repeatability and drive-health
requirements remain.
Both fields default to true if omitted. Feedback/drive-fault checks,
current/torque limits, the watchdog and parking checks remain active.
The backend still verifies disabled state, mode
registers and fresh finite encoder readback before enabling. No load direction
is assumed. See the [both-axis homing review](homing_failure_review_2026_09_08.md),
[prior restart](monitored_restart_2026_09_08.md) and
[earlier incident](optimization_cycle_2026_09_08.md) for historical evidence.

## Station and ownership

- SSH: `eamars@rpi-turret`; use the configured SSH key. Do not put passwords in scripts or Git.
- Main checkout: `/home/eamars/workspace/OpenAutoTurret`.
- Web control and live camera: **http://rpi-turret:8080/**.
- Runtime: `/tmp/ota-stack-1000` for `eamars`. Use that account for all operations;
  running the script under `sudo` selects a different runtime and is not the operating procedure.
- One launcher owns `controld`, `perception.visiond`, and `web.webd.app`.
  Vision alone owns the IMX500 camera; web reads its preview.

## Start, inspect and stop

For an early physical design probe, `Firmware/tools/deploy_station.py --probe-build`
uses the launcher to build only the runtime controller and run read-only preflight.
It defers the regression suite and labels the release probe-ready. The normal
deployment path still builds and tests all targets. This option does not start
motors unless activation is separately requested.

On the Pi, from the checkout or deployed release directory:

```bash
bash Firmware/scripts/run_application.sh          # detached start; same as start
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh stop
```

Start returns after the supervisor has launched its children, **not after
homing**. It survives SSH disconnection. Repeating start in the same checkout
reports the existing launcher. Starting another release while one is running
is refused. For interactive logs and Ctrl-C shutdown:

```bash
bash Firmware/scripts/run_application.sh run
```

`status` reports the active checkout, configuration, process IDs and live JSON
telemetry. Verify `controld_connected`, `soft_limits_valid`, an empty `fault`,
and `operating_mode` of `AUTO_ROAM` or `AUTO_TRACK` after startup. `phase=hold`
is the controller's service phase; it does not mean the user selected Manual.
The web shows homing progress and the current mode.

The launcher `stop` command requests controlled parking and motor disable, then waits for the owned
camera and web processes to exit. It never force-kills the motor controller.
If the caller's 120-second wait expires, shutdown remains in progress: inspect
the log and status. Do not start another controller or use `pkill`/`kill -9`.
Repeated stop is harmless. A failed child also shuts down its sibling processes.

The web/API action `request_shutdown` is **motor parking**, not launcher stop.
It leaves `controld`, webd and perception running after success or failure.
Home is rejected while parking runs, and can start a new calibration after
PARKED or a park-only failure if both drives have fresh, healthy, stationary
feedback. For a drive fault or latched watchdog, use **MENU → RECOVER MOTORS →
Confirm**, then **HOME → Confirm Home** after recovery succeeds. Home alone
does not clear a latched controller fault. An explicit launcher stop terminates
the services and retains its emergency-disable fallback, reported as PARK FAILED
when the park was not verified. It must not be mistaken for a successful release.

The corrected controller accepts **Stop Motion / Hold during parking** and
latches a controlled-stop fault. Parking overspeed, unexpected travel, stale
feedback and BRAKE/HOLD interventions also latch; a later ALLOW cannot resume
the park. These motion failures require Recover Motors before Home. Ordinary
verification-only failures retain the separate Home recovery described above.
The zero-speed command does not certify a stopped or supported load. The
controller cancels an interrupted drive mode-setup recipe through the existing
disable path because ordinary speed writes are suppressed while it is pending.
An offline pass does not establish physical validation of the corrected release.

### Motor fault recovery

`recover_motors` is an explicit operator action, available in Fault, Idle or
Parked. It inhibits motion, invalidates retained calibration, stops both drives,
and sends the documented CyberGear `COMM_TYPE_4` fault-clear command (`data[0]=1`)
once to each motor. This is fault clearing, not a firmware reboot or encoder-zero
command. It does not automatically retry or resume previous tracking/roaming.

The controller remains online in `phase=recovering`. Before releasing its
watchdog latch it requires both motors to report disabled, fault-free feedback
no older than 50 ms, temperatures within the configured limit, and at least ten
distinct samples per axis spanning a one-second position window of at most
0.25 degrees. A five-second deadline bounds the attempt. Failure reports
`RECOVERY FAILED` with the affected feedback gate and keeps motion disabled.
Missing communication, continuing drive faults, heat, or movement must resolve
before recovery can succeed; repeatedly resetting cannot repair those causes.

Successful operator recovery leaves `phase=idle`, Manual, motors disabled and
calibration invalid. Home is a separate confirmed action. Hardware Home also
runs the same clear/verify sequence before starting the homing plan. Normal
startup runs it when homing is required; a validated retained calibration still
avoids unnecessary motor disable and homing. Recovery does not certify a park
pose or the stability of an unpowered load.

Home and other motion commands are rejected while recovery is running. Stop
Motion (or Hold during recovery) cancels the attempt and leaves Fault; an
explicit Recover Motors action can retry later. The 100 ms watchdog remains
enabled and unchanged. Resetting is not evidence that the recurring feedback
outage has been repaired. Preserve runtime logs before any launcher restart.

Offline verification, with no physical motors or camera:

```bash
build/probe-motor-recovery
PYTHONPATH=. ../run/station-venv/bin/python tools/probe_motor_recovery_service.py \
  --controld build/control/controld --output ../run/motor-recovery-service
```

The first probe runs the actual watchdog/backend/UART path against a PTY motor
emulator, including an existing watchdog latch, silent pitch timeout and retry.
The second runs the actual controller/web services with simulated motors and
checks HTTP recovery, command rejection, cancellation, retry and re-homing.
Neither establishes that the physical feedback-loss mechanism is resolved.

Parking motion supervision can be probed without hardware with
`build/probe-parking-motion`. The numeric control trace includes actual parking
speed commands and position-derived estimated speed (`vest`), both in rad/s.

Parking targets are configurable under `shutdown` in `config/turret.yaml`:

- `yaw_park_mode` and `pitch_park_mode`: `logical_degrees`, `soft_center`,
  `soft_min`, or `soft_max`.
- `logical_degrees` uses the corresponding `yaw_park_deg` / `pitch_park_deg`.
- The other modes use calibrated **raw** soft limits. `soft_min` / `soft_max`
  are inset by `park_end_clearance_deg`; inadequate braking clearance is rejected.

The selected front-heavy-load pose is yaw `soft_center`, pitch `soft_min`,
with a 5-degree inset from the pitch soft minimum (in addition to the homing
soft-limit margin). The resolved raw and logical targets are logged after
homing. Configuration alone does not establish load stability at this pose.

Automatic parking release requires observed travel of at least 0.25 degrees
on each axis during its own park move, fresh motor feedback, and a trusted
independent output-position measurement including its uncertainty. The release
window is half the configured position tolerance: 0.25 degrees for a configured
0.5 degrees. Both axes are checked during dwell, before each disable, and for
another dwell after disable. No-motion, borderline, missing or stale evidence
reports PARK FAILED; no artificial movement is introduced to satisfy the gate.

**The current CAN backend has no independent position source connected.**
It therefore cannot satisfy the automatic release gate. Park verification
failure leaves healthy drives under fault-hold and keeps the services online;
hard-fault, temperature and watchdog emergency-disable authority remains.
The simulator supplies explicitly labelled simulated plant-position evidence;
its success is not physical verification. Integrate and validate an independent
sensor before claiming hardware PARKED or post-disable stability.

Offline lifecycle verification (from `Firmware`, after building):

```bash
PYTHONPATH=. ../run/station-venv/bin/python tools/probe_park_service.py \
  --controld build/control/controld --output ../run/park-service-probe
```

This starts the real controller and web processes with simulated motors and
video disabled. It uses a disposable fast-homing Manual configuration, exercises
park success and no-motion failure through HTTP, verifies Home rejection during
parking and recovery afterward, and checks both processes remain alive. Its
final cleanup explicitly terminates those simulator processes; it does not
operate the physical station or inspect a camera feed.

The script in any release can stop the active stack because ownership is shared
by account/runtime directory, not by checkout. It reports the active checkout
so an agent can find it without reconstructing past commands.

## Automatic mode and homing

The shipped `Firmware/config/turret.yaml` sets `v3.default_mode: AUTO_ROAM`.
The normal launcher checks this before enabling motors. No `--auto` flag or web
command is needed. After calibration validation/homing, the station roams,
acquires an eligible target, tracks, and returns to roaming after target loss.
Automatic loss recovery resumes the interrupted sweep direction. At a sweep end
it continues inward; outside the sweep region it first approaches the nearest end.
Manual/STOP clears that direction memory, and explicit Auto starts a fresh sweep.
See [roam recovery design](roam_recovery_design.md) for the policy and validation.

**Manual / Hold** is an explicit web override. Its D-pad appears only in Manual
service; hold an arrow to jog, release to stop. **Auto** resumes automatic
roaming/acquisition. **MENU → Home → Confirm Home** requests homing.
Manual/Hold is not an emergency stop or a reliable way to abort supervisory
homing; use the normal stop command for full controlled shutdown.

Motor disable invalidates retained calibration. Application restart skips homing
only when retained calibration and live energized motor state validate. Never
copy a retained homing file, manually mark axes homed, or bypass validation.
Slow loaded homing currently takes about 5–6 minutes: coarse 5°/s, fine/backoff
and between-axis moves 3°/s. This does not cap tracking speed. Tracking is
configured with target and maximum pairs of 20°/s and 30°/s² under
`motion.modes.auto_track`. These are the existing service command ceilings,
not verified installed-load maxima. At full target speed there is no extra
speed headroom for correcting lag. Axis, payload, confidence and boundary
limits remain authoritative. Full-speed loaded stopping and moving-target
overshoot remain unverified. See [motion profiles](motion_profiles.md) for the
configuration contract and offline validation.

## Deploy a committed revision from Windows or Linux

Deployment requires Git, OpenSSH (`ssh`, `scp`), and a local Python interpreter.
It uses only Python's standard library. Commit first; deployment refuses a dirty
source tree. No push to GitHub is required.

From this Windows workspace (using its existing project-local interpreter):

```powershell
run/takeover-analysis-venv/Scripts/python.exe Firmware/tools/deploy_station.py
```

From a Linux checkout with a project-local venv:

```bash
.venv/bin/python Firmware/tools/deploy_station.py
```

The command archives `HEAD`, uploads it into a new directory under
`/home/eamars/workspace/OpenAutoTurret/run/releases/`, records `REVISION`,
reuses the station's project-local `run/station-venv`, builds C++, runs CTest,
and performs read-only preflight. It does not overwrite the Pi checkout,
discard dirty files, or change the running station. It prints the exact
release path and activation/status commands.

To deploy and activate in one command, append **`--activate`**. Only after
build/tests/preflight succeed does it stop the old stack and start the new one.
Activation may home and move the station. If build/preflight fails, the running
release remains active; if activation fails, inspect the retained release and
runtime logs. There is no automatic rollback that unexpectedly starts motors.

For an existing inactive checkout directly on the Pi:

```bash
bash Firmware/scripts/run_application.sh deploy  # build, CTest, preflight; no start
bash Firmware/scripts/run_application.sh start
```

In-place deployment is refused while that checkout runs. A separate release can
be built while the old release runs. To roll back, use the printed path of the
previous release with `stop`, then `start`; normal homing validity rules apply.
Never merge/reset the Pi's dirty checkout merely to make a deployment command pass.

## Preflight, prerequisites and diagnostics

```bash
bash Firmware/scripts/run_application.sh check  # imports/config/files only; no motor or camera open
tail -n 80 /tmp/ota-stack-1000/controller.log
tail -n 80 /tmp/ota-stack-1000/vision.log
tail -n 80 /tmp/ota-stack-1000/web.log
tail -n 80 /tmp/ota-stack-1000/launcher.log
```

The commissioned Pi already has CMake, a C++20 compiler, yaml-cpp, spdlog, GTest,
RPi libcamera/Picamera2, the IMX500 model files named by the perception profile,
and permission for `eamars` to access `/dev/ttyUSB0` and the camera devices.
Deployment does not install OS packages or change hardware permissions.
For a replacement Pi, follow [AI camera setup](AI_CAMERA_SETUP.md) for OS/camera
packages. Create the project runtime with system-site-packages so it can import
the OS camera bindings:

```bash
python3 -m venv --system-site-packages run/station-venv
run/station-venv/bin/python -m pip install fastapi uvicorn PyYAML numpy Pillow
```

Reuse an existing venv; never install pip dependencies globally. Run `check`
after provisioning. Missing model/calibration files or imports must be resolved
before startup. `check` does not prove camera/CAN device availability or physical
motion; live startup and telemetry provide those checks.

Advanced overrides: `OTA_PYTHON`, `OTA_CONTROL_CONFIG`, `OTA_RUN_DIR`,
`OTA_WEB_HOST`, `OTA_WEB_PORT`, `OTA_BUILD_JOBS` (default 2). Keep normal operation
free of commissioning overrides. A custom control config may explicitly select
Manual; preflight prints the chosen startup mode. Status/stop require the same
account and `OTA_RUN_DIR`, and obtain the active web port from runtime metadata.

`--sim` simulates motors but still uses the real camera. `--hold-motion` starts
only perception and is stopped by the same script. Neither is the normal default.
`--production` invokes the perception model's production-readiness gate; the
commissioned `person_detect_available` profile is the ordinary default.

## Evidence and historical documents

### Numeric response diagnostics

The controller socket accepts the read-only `read_control_trace` command. It
returns at most 256 recent control records (about 1.28 seconds at 200 Hz), with
pitch/yaw encoder position, reference position/rate, commanded speed, reported
torque, feedback timestamps, command acknowledgement sequence and cycle interval.
The diagnostic writer uses a nonblocking lock: a reader can cause a missing
diagnostic sample, never a wait in motor control. Deduplicate by timestamp and
report gaps. Encoder readings do not certify independent platform angle.

Perception writes a latest-only `perception/timing.json` in the runtime directory.
It contains numeric capture/publication timestamps, exposure, frame duration,
image-copy time and available IMX500 DNN/DSP KPI values. No image access is needed.

`tools/measure_response_cycle.py --output ../run/response.jsonl` captures these
diagnostics without moving the station. Explicit `--probe yaw:1:2.5` requests a
target-free six-second fixed angular step using the tracking reference filter and
the existing AUTO_TRACK motion profile through the normal safety envelope. This
is a Manual commissioning command, not an automatic tracking mode. It requires
healthy homed speed-mode service, fresh feedback, near-zero commanded speed, and
15 degrees clearance at both endpoints. Allowed signed steps are 0.5, 1 and 5
degrees; the filter response rate is bounded to 2.5–6 per second. An optional
fourth argument (`yaw:1:4:3`) sets the host position correction gain for that
trial only, within 2–6 per second. The drive's internal gains are not changed.
Any subsequent
controller command cancels it, as do expiry, mode change and a safety intervention.
The tool also sends Stop Motion on exit. The trial does not alter deployed gains,
current limits, homing, calibration, or startup mode. Move captures off the Pi
after analysis; runtime data does not belong in Git.

The deployed `v3.tracking_reference_omega` controls small-correction response
(allowed 2.5–4 per second). Large corrections retain the original stiffness
until the requested acceleration fits the configured profile. A constant faster
gain produced excessive reference and encoder overshoot in physical 5-degree
steps, so it is not the production algorithm. `v3.position_servo_kp` defaults to
3 per second; the station configuration selects the physically evaluated value
4 per second. Probe overrides are not retained. See the
[response tuning follow-up](response_tuning_followup_2026_09_08.md) for measured
response, final deployment and remaining timing/settling limitations.
Current, speed, acceleration,
jerk, thermal, boundary and watchdog limits remain authoritative.

The automatic hand-off wait is 50 ms after a fresh selected measurement, followed
by AUTO_TRACK's distinct-frame acquisition checks. Perception's confirmation and
500 ms single-candidate selection dwell remain in force. This changes response
to a fresh selection; it does not establish detector accuracy on new scenes.

See [travel and loaded-control validation](travel_boundary_review_2026_09_06.md)
for measured motion limits and remaining verification gaps. The September 3
as-built document and earlier run reports are historical snapshots, not operating
instructions. The individual systemd templates use an older vision path; do not
run them alongside this launcher. Boot-time systemd activation is not installed
by this workflow.
