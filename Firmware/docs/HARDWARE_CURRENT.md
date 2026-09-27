# Current station hardware

Updated **27 September 2026** with large pitch/yaw and paired IMU motion checks. This is
the current hardware inventory; dated September 3-9 reports describe the previous
mechanism. Operation is governed by [STATION_OPERATIONS.md](STATION_OPERATIONS.md).
Bounded mixed-bus probes, non-motion pitch current-limit application, and a first
IMX477/Hailo measurement now have evidence; production motor/perception adaptation
remains pending:
[hardware adaptation plan](HARDWARE_ADAPTATION_PLAN.md) and
[AI perception plan](AI_HAT_PERCEPTION_PLAN.md).

## Evidence and status

Verification ran over key-based SSH as `eamars@rpi-turret`, beginning at
21:02 NZST (UTC+12), with an additional IMU probe later in the same audit.
Both local and Pi source were at
`6a47f1dd696d75b878b8138dcaceb9f455ab9147` (`Add v2 CAD`). The Pi checkout was
clean. The station launcher reported `Stopped (last park outcome unavailable)`.
No controller, perception daemon or web daemon was running.

The authorized mechanical session built a project-local runtime and separate
committed release, verified bidirectional low-output GM6020 motion and launcher
stop. Later non-motion pitch-limit and Hailo checks are recorded below. See
[the implementation record](HARDWARE_COMMISSIONING_2026_09_26.md) for the earlier
commissioning session; this document records only current distilled evidence.

**Verified** below means observed on this host during this audit.
**Owner-confirmed** means supplied by the owner, without physical inspection.
**Pending** means a specific remaining verification task, not a completed feature.

| Component | Current installation | Evidence |
|---|---|---|
| Host/account | `eamars@rpi-turret`, uid 1000 | Verified by SSH, `id`, `hostname` |
| Computer | Raspberry Pi 5 Model B Rev 1.1 | Verified device-tree model |
| OS/kernel | Debian 13.6 Trixie, aarch64, `6.18.39+rpt-rpi-2712` | Verified OS release and `uname` |
| CAN HAT | Waveshare 2-CH CAN FD HAT Rev2.1 | Model/revision owner-confirmed; two MCP2518FD controllers verified in kernel log |
| CAN transport | Two independent SocketCAN interfaces; `mcp251xfd`; 40 MHz controller clock | Verified netlink, sysfs, kernel log |
| Yaw drive | RoboMaster GM6020, ID 1, `can0` | Axis owner-confirmed; standard `0x205` feedback verified |
| Pitch drive | Xiaomi CyberGear, ID `0x7F`, `can1` | Axis owner-confirmed; extended discovery and UID verified |
| Yaw mechanics | Direct drive, continuous rotation, slip ring, no yaw endstop | Owner-confirmed; +29.356° and return verified with IMU; full-turn clearance unverified |
| Pitch mechanics | Direct drive, bounded pitch with mechanical endstops | Owner-confirmed; ±15° and return verified with 5 A cap; exact endpoints and homing pending |
| Camera A | Sony IMX500, index 0 at this boot | Enumerated and simultaneous capture verified |
| Camera B | Sony IMX477, index 1 at this boot | Enumerated and simultaneous capture verified; lens/FOV/mount geometry unknown |
| Accelerator | Hailo-8 AI HAT | `hailortcli fw-control identify` reported HAILO8, firmware 4.23.0, through `/dev/hailo0` after reboot |
| IMU | One BNO085 on I2C-1, address `0x4A` | Model owner-confirmed; SH-2 acceleration/gyro/rotation reports verified using the installed probe |
| Retired adapter | yousee/YouCee USB-to-CAN | Owner-confirmed retirement; no `/dev/ttyUSB*`, `lsusb` showed only root hubs |

The two motors are **not on a shared CAN bus**. The HAT supports CAN FD, but
the observed motor links use classical CAN, MTU 16, at 1 Mbps. Do not enable
FD/BRS merely because the adapter supports it.

## CAN wiring and boot configuration

The following active lines were read under `[all]` in
`/boot/firmware/config.txt`:

```ini
dtparam=spi=on
dtoverlay=spi1-3cs
dtoverlay=mcp251xfd,spi0-0,interrupt=25
dtoverlay=mcp251xfd,spi1-0,interrupt=24
```

An earlier commented `#dtparam=spi=on` is not the active setting. The pre-existing
`[pi5]` entry `dtoverlay=nospi10` is also present. This audit changed no boot file.

| Interface | Kernel parent | Interrupt in active overlay | Clock | CAN bitrate | Motor protocol |
|---|---|---|---|---|---|
| `can0` | `spi0.0` | GPIO25 | 40,000,000 Hz | 1,000,000 bit/s | GM6020 standard 11-bit frames |
| `can1` | `spi1.0` | GPIO24 | 40,000,000 Hz | 1,000,000 bit/s | CyberGear extended 29-bit frames |

Both kernel initialization messages identify **MCP2518FD** through the
`mcp251xfd` driver. Both interfaces reported sample point 0.750 and
`restart-ms 0`. Interface names should be checked against their SPI parents on
subsequent deployments; an interface name alone is not a hardware identity.

The owner's later authorization of elevated access was used only for temporary
CAN link up/down during the original audit. Station commands remained unprivileged.
Both links started and ended `DOWN` / `STOPPED` in that audit.

## Live motor identification, without motor actuation

After checking that the stack was stopped, a bounded Python standard-library
SocketCAN probe opened the links at their already configured bitrate:

| Probe | Observation | What it establishes |
|---|---|---|
| Receive only on `can0` for 1.2 seconds | 1,201 standard `0x205`, DLC-8 frames in 1.1999 s; sample `05520000ff5a1700` | Approximately 1 kHz traffic consistent with GM6020 ID 1; no command sent on this bus |
| One device-ID request on `can1` | Extended ID `0x0000007F`, DLC 8, payload `0000000000000000` | Documented CyberGear COMM_TYPE_0 query, host ID 0 / target ID `0x7F` |
| Discovery response | Extended `0x00007FFE`, DLC 8, payload `7216313130333105` | Motor ID `0x7F`; UID matches the owner's supplied byte string |

Record the UID unambiguously as **hexadecimal bytes `72 16 31 31 30 33 31 05`**,
or `0x7216313130333105` in the repository parser's big-endian representation.
Do not interpret the digits as a decimal serial number.

In this initial identification probe, no enable, zero, homing, register-write, speed, current or voltage command was
sent. `0x1FF` is the GM6020's documented voltage-command group for ID 1; it was
not exercised until the subsequent [commissioning session](HARDWARE_COMMISSIONING_2026_09_26.md).
That session establishes limited voltage response and encoder direction, not
motor firmware, full-load behavior or physical clockwise sign.
See [GM6020 reference](GM6020_AI_Reference.md).

The links were restored down in the probe's `finally` block and checked again
over a separate SSH command. GM6020 TX counter stayed 0; CyberGear TX rose by
one frame. Both bus-error counters remained 0. `can0` already had 10,649 RX
drops before this audit and still had 10,649 afterward; that historical counter
is not proof of a new HAT fault or proof of sustained-load health.

## Pitch current ceiling and non-motion limit application

The owner-set pitch CyberGear current ceiling is **5 A maximum**. This applies
to every software path that can configure the pitch drive; backend/config
enforcement now passes the regression suite. Do not treat a YAML setting or a successful
register write as proof that a later reset or mode change preserved it.

The bounded commissioning probe's explicit `--apply-pitch-limit` operation was
run without yaw actuation. It wrote volatile `LimitCur=5 A` and obtained three
matching readbacks. Pitch raw feedback reported mode 0 and faults 0. Before
upgrade, `MechPos` (`0x7019`) returned error reply `0x11017F00`; stale payload
was rejected. On September 27 the owner confirmed upgrading **pitch CyberGear
to 1.2.1.5**. A new live probe matched the same UID and read valid `MechPos`
at -0.710777 rad with response status 0. It reapplied 5 A and verified three
readbacks; feedback remained mode 0/faults 0. No pitch enable or motion command
was sent. This verifies register compatibility and limit setup, not homing or
loaded behavior; the version number itself is owner-reported.

The limit is volatile. Reapply and verify it after reset and before any enable.
Volatile speed/position settings also require verification before enable and
reapplication after reset. The CLI rejects combining pitch-limit setup with yaw
actuation. The owner's upgrade resolves the observed position-register blocker.
The agent did not flash firmware. The candidate image and vendor procedure remain
in [the upgrade reference](CYBERGEAR_FIRMWARE_UPGRADE.md); no further flash is
needed to repeat the now-passing register check.

## Cameras and capture evidence

Device paths reported by `rpicam-hello --list-cameras`:

```text
0 imx500 /base/axi/pcie@1000120000/rp1/i2c@88000/imx500@1a
1 imx477 /base/axi/pcie@1000120000/rp1/i2c@80000/imx477@1a
```

Bind future configuration to verified sensor identity/path rather than assuming
indices never change. Both report 4056 x 3040 sensor dimensions. Selected
advertised modes include IMX500 2028 x 1520 at 30.02 fps and IMX477
2028 x 1080 at 74.74 fps (10-bit); these are enumeration capabilities, not
measured application throughput.

A short, in-memory Picamera2 probe ran both cameras concurrently in separate
processes, requested RGB888 640 x 480 at 15 fps, captured 30 requests each, and
closed both devices. No images were saved. Results:

| Camera | Frames | Measured sensor cadence | Sensor timestamps | Host receipt minus sensor timestamp |
|---|---:|---:|---|---|
| IMX500 | 30 | 15.005 fps | All present and strictly increasing | 22.514-27.014 ms |
| IMX477 | 30 | 15.006 fps | All present and strictly increasing | 11.349-11.598 ms |

This verifies basic simultaneous delivery, not long-run reliability, correct
exposure synchronization, detection performance, full-resolution throughput,
optical alignment or the previous station's calibration. First-frame timestamps
differed by about 25.1 ms; no synchronization was configured.

## Hailo provisioning and first camera benchmark

The minimal Hailo-8 core was installed on the existing kernel
`6.18.39+rpt-rpi-2712`, which was not changed: DKMS 3.2.2, HailoRT and
`hailort-pcie-driver` 4.23.0, and `python3-hailort` 4.23.0-1. The package
transaction added 8 packages and upgraded/removed none. Neither `hailo-all` nor
Tappas/full Hailo applications were installed. A reboot passed: `/dev/hailo0`
returned, HailoRT identified HAILO8 firmware 4.23.0, and the project's venv
could import the Hailo binding.

An official Hailo Model Zoo 2.17.0 YOLOv8n HEF was used for a first hardware
measurement. Its SHA-256 is
`e893b0f9dcae366fe1bc9ebce25e32ad889acf2bc58cfe1f73a572f78f7ec055`; the HEF
and run artifacts are under ignored `run/hailo-probe/`, not source control.
The 30-frame hardware benchmark reported 3.36 ms inference latency. A separate
real IMX477 pipeline captured 30 RGB 640 x 480 frames at 15 fps, letterboxed to
640 x 640, and produced finite 80-class outputs with valid timestamps. Measured
inference p50/p95 was 6.86/7.05 ms; sensor-to-result p50/p95 was
20.99/21.69 ms. No detection score reached 0.5 for the current view. These
results prove pipeline execution and timing only: they are not accuracy evidence,
do not establish useful person recall, and do not implement person identity or
tracking. The camera was released after the probe. A repeatable
`Firmware/tools/probe_hailo_camera.py` and its pinned configuration manifest
provide the repeatable camera-only probe.

The repository's existing `Firmware/tools/camera_bringup_probe.py` is present
on the Pi. It exercises the older single-camera `vision` path and a synthetic
bright patch over a real frame; it cannot establish human-recognition accuracy.
A separate dual-camera probe was not located by filename/content searches of
accessible project/home, `/opt`, `/usr/local`, `/tmp` and `/var/tmp` locations
(excluding dependency/cache trees). Its location remains unresolved; the new
in-memory check above was independent of it.

Installed camera packages include Picamera2 0.3.37-1, libcamera
0.7.2+rpt20260817-1 and rpicam-apps 1.13.0-1. IMX500 model assets are present.

## Hailo and application readiness

Earlier PCI-only evidence was insufficient to identify the SKU. The later
`hailortcli fw-control identify` result resolves the runtime architecture as
HAILO8; see the installed stack and benchmark above.

Initial audit: launcher `check` failed at missing project Python; no station/CAN
systemd unit files were listed. Subsequent implementation provisioned
`run/station-venv` with system camera bindings and installed station Python
requirements there. Separate releases now contain working C++ builds and the
commissioning probe. The original Pi checkout was preserved. Normal hardware
preflight now rejects the legacy motor configuration on this installation.

The current production controller still lacks the commissioned mixed-drive,
continuous-yaw topology and pitch-only homing behavior. The normal automatic
startup gate remains closed. Old calibration, gains, loaded limits and parking
sign-off do not transfer to the new mechanism.

## Installed BNO085 and existing host probe

**Current integration:** versioned `tools/imu_bno085.c` and the SH-2 library now
run through the station launcher. A 30-second whole-packet I2C capture delivered
1,481 samples per gyro/orientation stream with no sequence gaps or I2C errors.
Game RV reported status 3; magnetic RV and gyro accuracy remained 0. A stationary
host tare and paired yaw/encoder measurements passed, including 1.01074° encoder
peak versus 1.01360° game-RV peak. This is relative-motion evidence, not a completed
mount calibration or controller fusion. Product part 10004148 reported version
3.2.13/build 6. See [the IMU commissioning record](IMU_COMMISSIONING_2026_09_27.md).

A later continuous energized pitch session completed +15°/return and
−15°/return at requested 10°/s and a verified 5 A cap, with no fault or guard
trip. The four paired game-RV/encoder angle ratios were 0.967, 0.995, 0.995
and 0.984; filtered current peaked at 1.034 A. The earlier +3° magnitude
shortfall did not persist and no scale correction is stored. A subsequent
30° yaw session reached +29.356° and returned to +0.659°; game-RV observed
+29.183° and −28.446° on the two legs against encoder +29.356° and
−28.740°. See the [large-motion record](LARGE_MOTION_COMMISSIONING_2026_09_27.md).

The following describes the earlier host-lab audit, retained as provenance.

The owner confirmed that the previously proposed BNO085 is now installed.
The working probe is **`/home/eamars/workspace/imu-lab/imu_main`**, with
`main.c`, `README.md` and an SH-2/SHTP library copy in `rd/`. Earlier experiments
`probe2.py`, `imu_read.py`, `imu_sh2.py` and `raw_dump.py` are also present.
The lab's project-local `.venv` is separate from OpenAutoTurret's runtime.
No probe files had been copied into production at that initial audit.

Source inspection shows the C probe opens `/dev/i2c-1`, selects `0x4A`, issues
an IMU soft reset, and configures acceleration, calibrated gyro and rotation
vector at requested 20,000 us intervals. It contains no motor commands, tare,
FRS writes or calibration-save operation. It was run as `eamars` after checking
that no other IMU consumer was active, and exited normally with code 0:

| Observation | Result |
|---|---|
| Report configuration | Zero configuration failures reported |
| Decoded reports | 333 accelerometer, 252 gyro, 252 rotation-vector reports |
| Acceleration norm | 9.825-9.909 m/s^2, mean 9.878 m/s^2 |
| Example quaternion | `(i,j,k,real) = (0.6242, 0.1683, -0.7471, 0.1548)` |
| Rotation-vector status | `0` throughout (unreliable accuracy status) |
| Gyro units | Library values are rad/s; the probe incorrectly prints `deg/s` |

The final output calls the run "3 seconds", but the source loops 600 times with
5 ms sleeps **plus I/O work**. It does not log per-report timestamps/sequence or
measure actual elapsed sample intervals. Counts therefore do not establish
50 Hz delivered cadence, uniqueness or latency. No stationary truth reference
or physical orientation check accompanied these readings.

The lab README previously recorded acceleration magnitude near 11.20 m/s^2 and
attributed it to calibration. Today's magnitude differs; neither that prior
explanation nor a claim that calibration is now correct follows from this short
sample. The exact firmware/product ID was not queried. The observed transport
and decoded reports establish SH-2 communication, not qualified orientation.

The lab README records placement at the top of the pitch stage, off-axis, and
polling without an INT connection. Confirm the current physical placement,
sensor-to-camera rotation and lever arm before using it for motion compensation.
The sensor probe exited; its volatile report configuration may remain active
until reconfigured/reset. No production IMU process is installed.

Inspected-source SHA-256:
`2da7fee0de0b6c7831bf4c52950b4230033be559a738532af3ebbd3c9d3c9fc8`.
Executed-binary SHA-256:
`c73c21c830f556f97a119d1e856d98a5834b2aa8d31a98dfa1dfbd421b2e252e`.
These identify the audit inputs; this audit did not rebuild or establish their
build reproducibility. The
[earlier IMU proposal](archive/open_auto_turret_bno085_imu_expansion_v1_1.md)
remains design input, with hardware absence superseded by this evidence.

## Remaining physical facts to collect

- GM6020 firmware and Assistant settings, especially voltage/current mode;
  command-loss behavior and an independently effective stop path.
- Pitch endstop geometry, new direction signs, yaw mechanical reference,
  supply/termination and slip-ring ratings. Both axes are owner-confirmed direct drive.
  The owner confirms the camera is near the pitch center of mass and disabling
  pitch presents no current support risk.
- Camera lenses, focus/FOV, mounts/orientation, intrinsics/extrinsics and overlap.
- HAT label/SKU and sustained Hailo/camera performance under production load.
- IMU mount/lever arm, accuracy and timestamp quality,
  magnetic behavior with motors energized, and long-run I2C reliability.

No runtime captures, credentials, retained calibration or virtual environments
belong in source control. This document records distilled observations only.
