# Operate and adapt the camera station

Current operating runbook, **27 September 2026**. Read this before deploying,
starting, stopping or diagnosing the station. Dated run reports are historical.

## Current deployment gate

**Current station state:** release `f8bcdb6` is running in operator-selected
MANUAL/HOLD after bounded D-pad tests. The owner raised the Pi input to 5.25 V
after release `8901808` reported active undervoltage/throttling (`0x50005`).
The subsequent normal IMX500/CAN/IMU run lasted about 5.5 minutes in
AUTO_ROAM/AUTO_TRACK with repeated `get_throttled=0x0`; a 60-frame IMX477/Hailo
camera-only run and a release build beside the active stack also returned 0x0.
PMIC EXT5V samples under these loads were about 4.87–5.11 V. This clears the
observed current power fault for those loads, but simultaneous dual-camera +
Hailo + motor peaks remain unmeasured. See [Raspberry Pi's bit definitions](https://www.raspberrypi.com/documentation/usage/raspberry-pi-os/raspberry-pi.html#get_throttled).

The latest full startup completed pitch homing and reached READY after the
continuous-yaw *pitch-homing-only* displacement tolerance was changed from
0.5° to 2°. An earlier attempt faulted at 0.527° yaw drift with fresh CAN
feedback; its launcher stop sent pitch STOP/yaw zero but could not confirm the
normal stopped state because the controller was already faulted. Preserve that
case for stop-path qualification. The latest manual yaw tests moved about 7.5°
in six seconds and 17.3° in twelve seconds, with no fault. A pitch manual
out/return test reached 15.63° above its initial pose after release at about
12° and transiently overshot 3.3° past its initial pose on return, then settled
within about 0.6°. Do not interpret working D-pad motion as pitch overshoot
qualification; see the [architect handoff](archive/partially-implemented/handoffs/ARCHITECTURE_HANDOFF_2026_09_27.md).

**The normal launcher selects the mixed split-bus profile. On release `cae41d0`,
two controlled stops succeeded after motion: one from AUTO_TRACK near +46° yaw,
and one from AUTO_ROAM near +76° yaw. Both confirmed fresh pitch-disabled
feedback and issued the final GM6020 zero request; GM6020 disable state remains
unavailable. These two observations do not complete stop qualification. An
intermittent feedback-readiness rejection seen on the previous release has not
yet been shown eliminated. Normal pitch homing completed, but repeated encoder
speed-ceiling/corridor warnings did not abort with
`homing.motion_checks_abort: false`; that guard behavior also remains
unqualified.**

**Current release:** `f8bcdb6` passed committed-source probe build and mixed
preflight, then was started through the launcher and reached READY. Its full
regression suite was deferred by `--probe-build`. The 13 targeted manual
controller tests and the isolated launcher lifecycle test passed; the
commissioning-ownership test cannot acquire the global station lock while the
live stack owns it. Release `43193dc` passed the
earlier 77-test suite, before these control changes. Current D-pad evidence
comes from the web command API and controller feedback, not a browser pointer
event trace. The live API reports valid pitch limits and a session-relative
±80° yaw operating sector; CAN errors are zero and BNO085 observation is fresh.

The installation has GM6020 yaw on `can0`, CyberGear pitch on `can1`, continuous
yaw without an endstop, IMX500 + IMX477 cameras, a PCIe Hailo device, and a
BNO085 on I2C. See [verified hardware and probes](archive/partially-implemented/hardware/HARDWARE_CURRENT.md).

The mixed runtime uses GM6020 yaw on CAN0 and CyberGear pitch on CAN1, with
pitch limited to 5 A. Yaw has no confirmed disable state; a zero request is not
proof of motor de-energization. The September 27 run confirms the mixed control,
perception and web path operated together; two subsequent controlled stops
passed the observed pitch-disable/yaw-zero checks, while broader stop
qualification and the intermittent readiness-rejection question remain open.
See the [hardware adaptation plan](archive/partially-implemented/hardware/HARDWARE_ADAPTATION_PLAN.md).

The project venv and a separate commissioning release are built on the Pi.
The minimal Hailo-8 kernel/runtime stack is now installed and passed a reboot;
an experimental IMX477-to-Hailo detector probe also passed finite-output and
timing checks. This does not establish detection accuracy, tracking identity or
production perception integration. See the [hardware inventory](archive/partially-implemented/hardware/HARDWARE_CURRENT.md)
and [AI plan](archive/partially-implemented/vision/AI_HAT_PERCEPTION_PLAN.md).

With the launcher stopped and camera ownership clear, the separate no-motion
probe exercises the pinned Hailo model without starting the controller/web:

```bash
run/station-venv/bin/python Firmware/tools/probe_hailo_camera.py \
  --hef /home/eamars/workspace/OpenAutoTurret/run/hailo-probe/yolov8n.hef --frames 30
```

Run from a committed release with the station project venv. It holds that
runtime directory's launcher lock, verifies the model SHA and HAILO8 identity,
and saves no images. Do not use another `OTA_RUN_DIR` to bypass ownership.

## Account, ownership and preserved operating contract

- SSH as `eamars@rpi-turret` using the existing key. Run station operations
  without `sudo`; uid 1000 owns `/tmp/ota-stack-1000` when the launcher runs.
- Checkout: `/home/eamars/workspace/OpenAutoTurret`. Preserve its local changes.
- After the Pi reboot, Windows DNS resolution for `rpi-turret` failed. The
  observed address was `192.168.2.100`; when needed, deploy with
  `--connect-address 192.168.2.100`. This is an observed address, not a static
  network setting. The option preserves the known `rpi-turret` SSH host-key
  identity while connecting to that address.
- One `Firmware/scripts/run_application.sh` launcher owns controller,
  `perception.visiond` and `web.webd.app`. Do not run old systemd services beside it.
- Normal startup uses AUTO_ROAM -> target tracking -> AUTO_ROAM after loss.
  Manual/Hold is an explicit web override, not a saved trial default.
- The web address is `http://rpi-turret:8080/` when the stack is running. A
  telemetry serialization fix now represents unavailable GM6020 torque as JSON
  `null`; the dashboard displays an em dash, and `/api/state` no longer fails on
  NaN yaw effort.
- Each physical camera has one owner; preview reads that owner's frames.
  The observed five-minute normal run used IMX500 and delivered 8,061 frames
  with zero drops while AUTO_ROAM and AUTO_TRACK/loss handoffs repeated. This is
  integration evidence, not an accuracy benchmark. The explicit
  `hailo_yolov8n` profile has passed a 60-frame IMX477 run through visiond in
  `--hold-motion` mode. A continuous BNO085 observer is launcher-supervised and
  observe-only; it has no motion-control authority. Simultaneous dual-camera
  operation remains unqualified.

## Inspect the stopped installation

Run from the Pi checkout/release as `eamars`:

```bash
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh check
ip -details -statistics link show can0
ip -details -statistics link show can1
rpicam-hello --list-cameras
lspci -nn
```

`check` inspects imports/config/files; it does not open motors or cameras and
does not prove motion readiness. The normal profile is now the mixed profile;
use `check --commission-hardware` for bounded commissioning preflight. Logs under
`/tmp/ota-stack-1000` exist only after a run.

After the September 27 reboot, both CAN links were DOWN; neither NetworkManager
nor the previous installation had a CAN startup profile. The one-time authorized
administrator setup installed and enabled `ota-can-links.service` from
[`../systemd/ota-can-links.service`](../systemd/ota-can-links.service) and its
[`../scripts/configure_can_links.sh`](../scripts/configure_can_links.sh) helper.
It brings `can0` and `can1` up at 1 Mbps classical CAN on boot, or validates an
already-up link without cycling it. It opens no motor transport. Check it as
`eamars` with `systemctl is-enabled ota-can-links.service` and
`systemctl is-active ota-can-links.service`, then inspect both links above.
Routine station operation remains unprivileged and uses the launcher. The
service also completed successfully at monotonic 5.76–5.82 s on the next
observed boot, and both links were UP, ERROR-ACTIVE, 1 Mbps. One successful
boot does not establish long-term recovery reliability. The Pi's idle
`get_throttled=0x0` after that boot does not replace a loaded power check.

Earlier September 27 large-motion sessions left both CAN links UP at 1 Mbps.
Their pitch drives ended with verified disabled feedback; yaw ended with zero
voltage requested and stationary feedback, but its disable state is unknown.
The current release is running in MANUAL/HOLD after D-pad tests. Inspect live
ownership/state before another session; do not cycle CAN links between tests.

The owner authorized the September 27 one-time privileged CAN boot setup after
the reboot. This does not change unprivileged launcher ownership.
Never put credentials in scripts or Git. For motor probes, identify the exact
protocol first; discovery must not enable, zero, home or actuate a motor.
GM6020 `0x1FF` is a voltage command, not a discovery request. See the
[GM6020 reference](references/gm6020/GM6020_AI_Reference.md) and
[CyberGear reference](references/cybergear/CyberGear_AI_Reference.md).

The existing IMU probe is `/home/eamars/workspace/imu-lab/imu_main`, with source
and README beside it. It soft-resets the IMU and enables three sensor reports;
it is not a passive bus read. Run it only when it owns the sensor and its reset
cannot disrupt a running consumer. Its gyro output label is wrong: values are
rad/s, not deg/s. See the inventory for measured results and remaining gaps.

Use the versioned replacement for further IMU work:

```bash
bash Firmware/scripts/run_application.sh run --probe-imu --imu-seconds 30
```

Deploy it with `deploy_station.py --probe-build --probe-imu`. It needs only
unprivileged I2C access, records `/tmp/ota-stack-1000/imu.ndjson`, and opens no
camera or motor transport. It establishes a stationary **host reference**, not
a mounting calibration. `--commission-hardware --with-imu` adds the same capture
to bounded motor probes. See [IMU evidence and coordinate meaning](archive/partially-implemented/commissioning/IMU_COMMISSIONING_2026_09_27.md).
Do not assign the pitch-mounted IMU pose directly to the base orientation.

For the tested camera-only Hailo application slice, use
`run --hold-motion --profile hailo_yolov8n --frames 60 --no-web`.
The shared model remains under the original checkout's `run/hailo-probe` and is
linked into releases, with SHA verification before use. IMX477 camera-to-axis
calibration is still required before its detections may guide physical motion;
the existing 1920x1080 camera calibration does not certify this 640x480 profile.

## Stop and preserve evidence

If a launcher-owned stack is running, stop it through the launcher:

```bash
bash Firmware/scripts/run_application.sh stop
bash Firmware/scripts/run_application.sh status
```

The launcher requests the controller's axis-specific safe stop action, then
shuts down its children. It never force-kills the controller. Pitch disable is
feedback-confirmed when fresh; GM6020 receives a zero request but its disable
state is unavailable. If its 120-second caller wait
expires, inspect status/logs; shutdown may still be in progress. Do not start a
second controller or use broad process kills. Stop can be issued from another
checkout because ownership is shared by account/runtime directory.

**Stop qualification includes two successful moving-stop observations on release
`cae41d0` and one further stop on release `8901808`; broader qualification
remains open.**
The previous middle-yaw/lowest-pitch release contract and CyberGear disabled-bit
verification do not transfer to GM6020. A zero GM6020 command does not certify
power removal or a supported load. Complete stop/park qualification before
unattended operation. The web's parking request is not equivalent to full
launcher stop.

On `cae41d0`, controlled stop completed successfully after AUTO_TRACK motion at
about +46° yaw and after AUTO_ROAM motion at about +76° yaw. Both recorded fresh
pitch-disabled feedback and a final GM6020 zero request. GM6020 disable state
remains unknown; zero request is never power-removal certification. The prior
release had intermittent feedback-readiness rejection. The two successes do
not prove that issue is eliminated or establish full stop/recovery/park
qualification.
Do not interpret a zero-voltage request as power removal.
The separate `--apply-pitch-limit` option only writes the configured volatile
CyberGear `LimitCur` value (5 A maximum) and checks three matching readbacks; it
does not enable or move pitch. It cannot be combined with yaw voltage or speed
actuation. Reapply and verify volatile pitch settings after reset and before
enable; the production position/speed mode paths must establish and verify the
current cap before enabling the drive.

Preserve numeric logs before restarting. Never overwrite retained homing data,
manually mark axes homed or bypass validation. Invalidate old calibration by
installation identity. Yaw needs reference initialization instead of endpoint
homing. Pitch homing has completed in the normal mixed controller, but repeated
speed-ceiling/corridor warnings did not abort with
`homing.motion_checks_abort: false`; this guard behavior remains unqualified.
IMU orientation is a secondary observation, not a replacement for
motor/reference validity.

## Deployment and operation

Use the following launcher path for the mixed profile. Two controlled stops
have passed on `cae41d0`; do not regard normal operation as fully commissioned
until broader stop/recovery evidence and remaining acceptance gates are
reviewed, including the intermittent readiness rejection seen on the prior
release.

Use the existing project-local virtual environment with OS camera bindings and
install station requirements there. Never install pip dependencies globally or
commit the environment. `run/station-venv` exists with system camera bindings;
builds live in separate `run/releases/.../Firmware/build` directories. The
minimal Hailo-8 driver/runtime is installed and verified after reboot; do not
replace it with `hailo-all` or install Tappas as part of this minimal profile.

Deploy committed source with `Firmware/tools/deploy_station.py`. It archives
`HEAD`, creates a separate release under `run/releases`, records `REVISION`,
builds/tests and performs preflight while preserving the Pi checkout. It refuses
dirty source. Use a project-local Python interpreter; no push is required.

Without `--activate`, deployment does not start motors. `--probe-build` builds
the controller and commissioning probe and runs preflight while deferring regression tests; it is
probe-ready evidence only. `--activate` additionally stops/starts through the
launcher and verifies readiness. It can move motors and is inappropriate for
unadapted source.

After implementation and commissioning, the usual entry points are:

```bash
bash Firmware/scripts/run_application.sh deploy  # inactive checkout: build/test/check
bash Firmware/scripts/run_application.sh start   # also the no-argument default
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh stop
```

Start returns after child launch, not after readiness. Current runtime uses a
session-relative continuous-yaw reference, pitch-only homing, fresh feedback,
bus identity checks and valid perception. A successful AUTO_ROAM run does not
prove stop behavior, tracking accuracy or final station readiness.

`--sim` still opens the real camera; `--hold-motion` is perception-only and also
opens it. Neither replaces camera ownership checks or verifies the new backend.
Keep trial mode/speed/gain overrides out of normal releases. Rollback must select
a release qualified for this hardware or leave the station stopped; never
restart a dual-CyberGear build on the new mechanism.

## Bounded commissioning, without automatic startup

See [the September 26 implementation and test record](archive/partially-implemented/commissioning/HARDWARE_COMMISSIONING_2026_09_26.md)
for the tested revision and release path. Deploy a committed commissioning build
with the local project Python:

```bash
python Firmware/tools/deploy_station.py --probe-build --commission-hardware
```

If `rpi-turret` does not resolve from Windows after reboot, the observed
connection workaround is:

```bash
python Firmware/tools/deploy_station.py --connect-address 192.168.2.100 --probe-build --commission-hardware
```

The address is evidence from this session, not a static configuration promise.

On that release, as `eamars`, with both links already at 1 Mbps and UP:

```bash
bash Firmware/scripts/run_application.sh check --commission-hardware
bash Firmware/scripts/run_application.sh run --commission-hardware
# Optional non-motion operation: apply and verify the pitch LimitCur ceiling.
bash Firmware/scripts/run_application.sh run --commission-hardware --apply-pitch-limit
# Explicit motion: repeat only within the commissioned envelope and clear mechanism.
bash Firmware/scripts/run_application.sh run --commission-hardware --yaw-voltage 1000 --pulse-ms 150
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh stop
```

The default probe only receives yaw and queries pitch discovery/mechanical
position. A rejected pitch register read is reported unavailable, never treated
as a position. The probe does not home, enable, zero or actuate pitch. With
`--apply-pitch-limit`, it writes only volatile `LimitCur=5 A`, requires three
matching readbacks, and leaves pitch disabled; this must be reapplied and
verified after a reset before any enable. Following the owner's September 27
CyberGear 1.2.1.5 upgrade, the same UID returned valid `MechPos` (-0.710777 rad)
with status 0, and raw feedback reported mode 0/faults 0. The earlier rejected
register read is historical. Later normal mixed runtime completed pitch-only
homing, but repeated encoder-speed-ceiling and commanded-corridor warnings were
non-aborting under `homing.motion_checks_abort: false`; treat the guard behavior
as unresolved.
Do not combine limit setup with yaw actuation.

### Pitch motion session

The owner's tuning preference is to establish meaningful motion using the full
authorized output headroom first. For pitch that means **5 A maximum**, never the
motor's larger factory limit. Use a clear bounded target instead of escalating
from tiny current/speed commands. A current limit is available headroom; it does
not mean the controller must draw 5 A continuously.

Keep pitch enabled between movements, and keep CAN and IMU acquisition live
through the session. Do not cycle the stack, lower the CAN links or disable the
motor between individual stages. Stop on a fault or explicit session completion.
No persistent gain, homing, encoder-zero or calibration writes are part of this
probe. The commissioned ±15° pitch session is:

```bash
bash Firmware/scripts/run_application.sh run --commission-hardware --with-imu \
  --pitch-step-mdeg 15000 --pitch-test-gains
```

It verifies the 5 A cap and position mode, then enables once for two step/return
pairs: +15°, start, −15°, start, at a requested 10°/s. Pitch remains
energized while settling and between all four stages. The explicit gain trial
uses speed-loop Kp=4, Ki=0.05 and restores nominal 1/0.002 at session completion.
The final stop is not a parking/homing certification. Numeric traces are
`pitch-probe.csv`, `controller.log` and `imu.ndjson` in the launcher runtime.

The probe bounds excursion from initial position to 17°, encoder-derived speed
over at least 50 ms to 20°/s, feedback/heartbeat age to 100 ms, and temperature
to 45°C. The firmware's raw speed field has shown noise inconsistent with small
encoder changes; it remains logged but does not alone establish actual speed.
These bounds do not qualify an unknown pitch endpoint or automatic homing.
See [the paired large-motion and IMU record](archive/partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md).

### Yaw motion session

The commissioned yaw excursion is 30° out and back in one continuous CAN0
session with a fresh BNO085 host tare:

```bash
bash Firmware/scripts/run_application.sh run --commission-hardware --with-imu \
  --yaw-step-deg 30
```

The successful run moved 29.356° outbound and returned to +0.659° relative to
its start; the IMU independently measured +29.183° and −28.446° on the two
legs. The GM6020 voltage output ceiling is the vendor-documented ±25,000 raw,
while actual commands stayed within −4,268..+5,643 raw. It guards travel,
speed, stale feedback and stalled progress, then requests zero voltage and
observes a stationary motor. GM6020 zero voltage is not a verified disable or
mechanical park. See the [large-motion record](archive/partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md)
for bounds, failures and raw evidence.

`config/hardware_probe.yaml` is a separate probe schema, **not** a production
controller configuration. Fixed ceilings are |voltage| <= 3000 raw, pulse <=
500 ms, travel <= 5 degrees, speed <= 20 degrees/s, feedback age <= 20 ms and
heartbeat gap <= 40 ms. Recorded trials include +/-1000 and +/-1500 raw for
150 ms, +2000 raw for 100 ms, and bounded +/-3 deg/s PI requests for 500 ms.
The PI trial stayed within the guards but did not achieve its requested speed;
its 1500 raw ceiling and gains are not production-qualified. See the
[continuation evidence](archive/partially-implemented/commissioning/HARDWARE_CONTINUATION_2026_09_26.md). These raw voltage
commands are not amperes.

The probe verifies SPI parents, bitrate, ERROR-ACTIVE state, UID and stationary
yaw baseline before output. The 200 Hz pulse loop and separate in-process guard
serialize commands, stop on stale/invalid feedback or CAN error frames, and
request zero after pulse deadline/interruption. This guard cannot survive loss
of the process or Pi. Automatic operation and process-loss behavior remain
unqualified. No independent power-cutoff capability has been established.

The launcher and probe hold station-wide locks independent of `OTA_RUN_DIR`.
Do not run other motor transmitters alongside them. Numeric evidence is written
to `/tmp/ota-stack-1000/hardware-probe.csv` and `controller.log`; copy it into
ignored `run/` before the next probe replaces it. No camera or web process is
started in commissioning mode.

## Historical procedures

Detailed September 8-9 homing, recovery, tuning and parking instructions are
preserved in the [retired dual-CyberGear runbook](archive/superseded/operations/station_operations_dual_cybergear_2026_09_09.md).
Their measurements remain background, not certification of this mechanism.
`STATION_RUNBOOK.md`, MCP2515 setup/fault reports and `AS_BUILT_v1.md` describe
prior installations. The earlier BNO085 proposal is also historical design
input; its hardware-absent status is superseded, while integration remains open.
See [the documentation map](README.md).
