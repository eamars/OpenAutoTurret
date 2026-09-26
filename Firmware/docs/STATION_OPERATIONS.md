# Operate and adapt the camera station

Current operating runbook, **26 September 2026**. Read this before deploying,
starting, stopping or diagnosing the station. Dated run reports are historical.

## Current deployment gate

**The station is stopped, and the checked-in motor stack does not yet support
the installed hardware. Do not start it on the new mechanism.** This is a
documented deployment restriction, not a guard already implemented in software.

The installation has GM6020 yaw on `can0`, CyberGear pitch on `can1`, continuous
yaw without an endstop, IMX500 + IMX477 cameras, a PCIe Hailo device, and a
BNO085 on I2C. See [verified hardware and probes](HARDWARE_CURRENT.md).

The source still selects the retired `/dev/ttyUSB0` yousee adapter and CyberGear
IDs 100/101. Its yaw endpoint homing, soft limits and soft-center parking describe
the old mechanism. Changing only `can.backend` or IDs is insufficient: SocketCAN
currently transmits extended frames only, and the backend assumes CyberGear
control/feedback on both axes. Follow the
[hardware adaptation plan](HARDWARE_ADAPTATION_PLAN.md) before activation.

The Pi also lacks the project runtime/build and Hailo driver/runtime. Launcher
`check` currently stops at missing project Python. Simultaneous camera capture
and the standalone IMU probe passed their basic data-delivery checks; neither
establishes complete application, orientation-calibration or AI readiness.
Follow the [AI plan](AI_HAT_PERCEPTION_PLAN.md).

## Account, ownership and preserved operating contract

- SSH as `eamars@rpi-turret` using the existing key. Run station operations
  without `sudo`; uid 1000 owns `/tmp/ota-stack-1000` when the launcher runs.
- Checkout: `/home/eamars/workspace/OpenAutoTurret`. Preserve its local changes.
- One `Firmware/scripts/run_application.sh` launcher owns controller,
  `perception.visiond` and `web.webd.app`. Do not run old systemd services beside it.
- The future normal startup remains AUTO_ROAM -> target tracking -> AUTO_ROAM
  after loss. Manual/Hold is an explicit web override, not a saved trial default.
- The web address is `http://rpi-turret:8080/` when the stack is running. During
  this audit no web service was listening there.
- Each physical camera has one owner; preview reads that owner's frames.
  Production currently owns only IMX500. Dual-camera/Hailo/IMU integration is
  planned, not implemented production behavior.

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
does not prove motion readiness. Missing Python is a real failure, not evidence
that the old configuration would otherwise pass for this hardware. Logs under
`/tmp/ota-stack-1000` exist only after a run.

At audit completion both CAN links were restored `DOWN` / `STOPPED` at 1 Mbps;
both cameras were closed and the IMU probe exited. A future probe must inspect
current ownership/state rather than assume these conditions persist.

The owner's September 26 one-off elevation authorization was used only to bring
up/down CAN links for identification. It does not change launcher ownership.
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

Preserve numeric logs before restarting. Never overwrite retained homing data,
manually mark axes homed or bypass validation. Invalidate old calibration by
installation identity. Yaw needs reference initialization instead of endpoint
homing; pitch homing/support must be re-commissioned. IMU orientation is a
secondary observation, not a replacement for motor/reference validity.

## Deployment after adaptation gates pass

The following remains the deployment path, but **activation is deferred until
hardware adaptation and physical commissioning gates pass**.

Provision a project-local virtual environment with OS camera bindings and install
station requirements there. Never install pip dependencies globally or commit
the environment. This Pi has neither the expected runtime venv nor
`Firmware/build`. Hailo OS driver/runtime provisioning is separately planned.

Deploy committed source with `Firmware/tools/deploy_station.py`. It archives
`HEAD`, creates a separate release under `run/releases`, records `REVISION`,
builds/tests and performs preflight while preserving the Pi checkout. It refuses
dirty source. Use a project-local Python interpreter; no push is required.

Without `--activate`, deployment does not start motors. `--probe-build` builds
only the controller and runs preflight while deferring regression tests; it is
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

## Historical procedures

Detailed September 8-9 homing, recovery, tuning and parking instructions are
preserved in the [retired dual-CyberGear runbook](archive/station_operations_dual_cybergear_2026_09_09.md).
Their measurements remain background, not certification of this mechanism.
`STATION_RUNBOOK.md`, MCP2515 setup/fault reports and `AS_BUILT_v1.md` describe
prior installations. The earlier BNO085 proposal is also historical design
input; its hardware-absent status is superseded, while integration remains open.
See [the documentation map](README.md).
