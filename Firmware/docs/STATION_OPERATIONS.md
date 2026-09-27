# Operate and adapt the camera station

Current operating runbook, **27 September 2026**. Read this before deploying,
starting, stopping or diagnosing the station. Dated run reports are historical.

## Current deployment gate

**The automatic station remains stopped. Only the explicit, bounded
`--commission-hardware` path is qualified for the current motor probes.**
Normal hardware preflight rejects the legacy yousee configuration when the
split-bus installation is present. This is not a complete mixed-drive backend.

The installation has GM6020 yaw on `can0`, CyberGear pitch on `can1`, continuous
yaw without an endstop, IMX500 + IMX477 cameras, a PCIe Hailo device, and a
BNO085 on I2C. See [verified hardware and probes](HARDWARE_CURRENT.md).

The source still selects the retired `/dev/ttyUSB0` yousee adapter and CyberGear
IDs 100/101. Its yaw endpoint homing, soft limits and soft-center parking describe
the old mechanism. Changing only `can.backend` or IDs is insufficient: the
transport now supports both frame types, but the automatic backend still assumes
CyberGear control/feedback on both axes. Follow the
[hardware adaptation plan](HARDWARE_ADAPTATION_PLAN.md) before activation.

The project venv and a separate commissioning release are built on the Pi.
The minimal Hailo-8 kernel/runtime stack is now installed and passed a reboot;
an experimental IMX477-to-Hailo detector probe also passed finite-output and
timing checks. This does not establish detection accuracy, tracking identity or
production perception integration. See the [hardware inventory](HARDWARE_CURRENT.md)
and [AI plan](AI_HAT_PERCEPTION_PLAN.md).

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
- The future normal startup remains AUTO_ROAM -> target tracking -> AUTO_ROAM
  after loss. Manual/Hold is an explicit web override, not a saved trial default.
- The web address is `http://rpi-turret:8080/` when the stack is running. During
  this audit no web service was listening there.
- Each physical camera has one owner; preview reads that owner's frames.
  Normal production still selects IMX500. The explicit `hailo_yolov8n` profile
  has passed a 60-frame IMX477 run through visiond in `--hold-motion` mode.
  IMU acquisition/tare runs through the launcher; simultaneous dual-camera
  operation and IMU/controller fusion remain unimplemented.

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
does not prove motion readiness. Use `check --commission-hardware` on the new
release for commissioning preflight. Normal `check` deliberately rejects the old
motor configuration. Logs under
`/tmp/ota-stack-1000` exist only after a run.

The latest September 27 large-motion sessions left both CAN links UP at
1 Mbps. Pitch ended with verified disabled feedback; yaw ended with zero
voltage requested and stationary feedback, but its disable state is unknown.
Inspect current ownership/state before another session; do not cycle CAN links
between tests.

The owner's September 26 elevation authorization covers temporary CAN link
setup for these probes; mechanical tests were subsequently authorized explicitly.
This does not change unprivileged launcher ownership.
Never put credentials in scripts or Git. For motor probes, identify the exact
protocol first; discovery must not enable, zero, home or actuate a motor.
GM6020 `0x1FF` is a voltage command, not a discovery request. See the
[GM6020 reference](GM6020_AI_Reference.md) and
[CyberGear reference](CyberGear_AI_Reference.md).

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
to bounded motor probes. See [IMU evidence and coordinate meaning](IMU_COMMISSIONING_2026_09_27.md).
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

The launcher requests controlled parking and motor disable, then shuts down its
children. It never force-kills the controller. If its 120-second caller wait
expires, inspect status/logs; shutdown may still be in progress. Do not start a
second controller or use broad process kills. Stop can be issued from another
checkout because ownership is shared by account/runtime directory.

**The old stack's parking/disable result is not validated on this hardware.**
The previous middle-yaw/lowest-pitch release contract and CyberGear disabled-bit
verification do not transfer to GM6020. A zero GM6020 command does not certify
power removal or a supported load. Commission the new stop/park contract before
operation. The web's parking request is not equivalent to full launcher stop.

The default mixed-probe stop requests zero GM6020 voltage and observes feedback; its
terminal result explicitly says **not a park/disable certification**. It never
enables or moves pitch. The explicit pitch session below does enable pitch.
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
homing; pitch homing/support must be re-commissioned. IMU orientation is a
secondary observation, not a replacement for motor/reference validity.

## Deployment after adaptation gates pass

The following remains the deployment path, but **activation is deferred until
hardware adaptation and physical commissioning gates pass**.

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

Start returns after child launch, not after homing/reference establishment. The
revised implementation must require per-axis valid reference/calibration, fresh
feedback, empty faults, both bus identities and valid perception before normal
operation. The old finite-yaw `soft_limits_valid` check does not prove continuous
yaw readiness.

`--sim` still opens the real camera; `--hold-motion` is perception-only and also
opens it. Neither replaces camera ownership checks or verifies the new backend.
Keep trial mode/speed/gain overrides out of normal releases. Rollback must select
a release qualified for this hardware or leave the station stopped; never
restart a dual-CyberGear build on the new mechanism.

## Bounded commissioning, without automatic startup

See [the September 26 implementation and test record](HARDWARE_COMMISSIONING_2026_09_26.md)
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
register read is historical; pitch motion/homing still needs commissioning.
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
See [the paired large-motion and IMU record](LARGE_MOTION_COMMISSIONING_2026_09_27.md).

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
mechanical park. See the [large-motion record](LARGE_MOTION_COMMISSIONING_2026_09_27.md)
for bounds, failures and raw evidence.

`config/hardware_probe.yaml` is a separate probe schema, **not** a production
controller configuration. Fixed ceilings are |voltage| <= 3000 raw, pulse <=
500 ms, travel <= 5 degrees, speed <= 20 degrees/s, feedback age <= 20 ms and
heartbeat gap <= 40 ms. Recorded trials include +/-1000 and +/-1500 raw for
150 ms, +2000 raw for 100 ms, and bounded +/-3 deg/s PI requests for 500 ms.
The PI trial stayed within the guards but did not achieve its requested speed;
its 1500 raw ceiling and gains are not production-qualified. See the
[continuation evidence](HARDWARE_CONTINUATION_2026_09_26.md). These raw voltage
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
preserved in the [retired dual-CyberGear runbook](archive/station_operations_dual_cybergear_2026_09_09.md).
Their measurements remain background, not certification of this mechanism.
`STATION_RUNBOOK.md`, MCP2515 setup/fault reports and `AS_BUILT_v1.md` describe
prior installations. The earlier BNO085 proposal is also historical design
input; its hardware-absent status is superseded, while integration remains open.
See [the documentation map](README.md).
