# Deploy and operate the camera station

This is the current operating runbook. Use this procedure rather than dated
commissioning scripts or the legacy individual systemd units.

## Station and ownership

- SSH: `eamars@rpi-turret`; use the configured SSH key. Do not put passwords in scripts or Git.
- Main checkout: `/home/eamars/workspace/OpenAutoTurret`.
- Web control and live camera: **http://rpi-turret:8080/**.
- Runtime: `/tmp/ota-stack-1000` for `eamars`. Use that account for all operations;
  running the script under `sudo` selects a different runtime and is not the operating procedure.
- One launcher owns `controld`, `perception.visiond`, and `web.webd.app`.
  Vision alone owns the IMX500 camera; web reads its preview.

## Start, inspect and stop

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

Stop requests controlled parking and motor disable, then waits for the owned
camera and web processes to exit. It never force-kills the motor controller.
If the caller's 120-second wait expires, shutdown remains in progress: inspect
the log and status. Do not start another controller or use `pkill`/`kill -9`.
Repeated stop is harmless. A failed child also shuts down its sibling processes.

The script in any release can stop the active stack because ownership is shared
by account/runtime directory, not by checkout. It reports the active checkout
so an agent can find it without reconstructing past commands.

## Automatic mode and homing

The shipped `Firmware/config/turret.yaml` sets `v3.default_mode: AUTO_ROAM`.
The normal launcher checks this before enabling motors. No `--auto` flag or web
command is needed. After calibration validation/homing, the station roams,
acquires an eligible target, tracks, and returns to roaming after target loss.

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
15°/s with 20°/s service headroom; track acceleration is 25°/s². Config and
safety limits remain authoritative. Moving-target overshoot is not fully verified.

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

See [travel and loaded-control validation](travel_boundary_review_2026_09_06.md)
for measured motion limits and remaining verification gaps. The September 3
as-built document and earlier run reports are historical snapshots, not operating
instructions. The individual systemd templates use an older vision path; do not
run them alongside this launcher. Boot-time systemd activation is not installed
by this workflow.
