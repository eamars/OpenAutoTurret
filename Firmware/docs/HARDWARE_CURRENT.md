# Current station hardware

Updated **26 September 2026**. This is the current hardware inventory; dated
September 3-9 reports describe the previous mechanism. Operation is governed by
[STATION_OPERATIONS.md](STATION_OPERATIONS.md). Implementation remains pending:
[hardware adaptation plan](HARDWARE_ADAPTATION_PLAN.md) and
[AI perception plan](AI_HAT_PERCEPTION_PLAN.md).

## Evidence and status

Verification ran over key-based SSH as `eamars@rpi-turret`, beginning at
21:02 NZST (UTC+12), with an additional IMU probe later in the same audit.
Both local and Pi source were at
`6a47f1dd696d75b878b8138dcaceb9f455ab9147` (`Add v2 CAD`). The Pi checkout was
clean. The station launcher reported `Stopped (last park outcome unavailable)`.
No controller, perception daemon or web daemon was running.

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
| Yaw mechanics | Continuous rotation, slip ring, no yaw endstop | Owner-confirmed; no motion/clearance inspection performed |
| Pitch mechanics | Intended bounded pitch axis | Exact new limits, direction, load/support and endstop arrangement pending commissioning |
| Camera A | Sony IMX500, index 0 at this boot | Enumerated and simultaneous capture verified |
| Camera B | Sony IMX477, index 1 at this boot | Enumerated and simultaneous capture verified; lens/FOV/mount geometry unknown |
| Accelerator | Owner reports 26 TOPS AI HAT, implying Hailo-8; PCIe Hailo presence verified | PCI `1e60:2864` at `0001:01:00.0`; PCI ID/description alone cannot distinguish H8 from H8L; runtime architecture/SKU pending |
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
CAN link up/down during this audit. Station commands remained unprivileged.
Both links started and ended `DOWN` / `STOPPED`.

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

No enable, zero, homing, register-write, speed, current or voltage command was
sent. `0x1FF` is the GM6020's documented voltage-command group for ID 1; it was
not exercised. Motor firmware, torque/voltage response, direction and loaded
stopping behavior remain unverified. See [GM6020 reference](GM6020_AI_Reference.md).

The links were restored down in the probe's `finally` block and checked again
over a separate SSH command. GM6020 TX counter stayed 0; CyberGear TX rose by
one frame. Both bus-error counters remained 0. `can0` already had 10,649 RX
drops before this audit and still had 10,649 afterward; that historical counter
is not proof of a new HAT fault or proof of sustained-load health.

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

PCIe prints `Hailo Technologies Ltd. Hailo-8 AI Processor [1e60:2864]`.
This is the PCI database's family label, not definitive H8-versus-H8L
identification: the official Pi documentation also shows this PCI ID for an
H8L example. The owner's 26 TOPS specification implies H8, but verify
`Device Architecture` using `hailortcli fw-control identify` before choosing a
HEF. [Official identify example](https://www.raspberrypi.com/documentation/computers/ai.html).
However, no driver is bound at its PCI function, `/dev/hailo*` is absent,
`hailortcli` is absent, and `/sbin/modinfo hailo_pci` reports module not found.
Package queries found no `hailo-all`, `hailort`, `hailo-dkms`,
`python3-hailort` or `rpicam-apps-hailo-postprocess` installation.
**Physical enumeration is verified; usable Hailo inference is not.**

The launcher `check` fails at missing project Python
`/home/eamars/workspace/OpenAutoTurret/.venv/bin/python`. Neither
`run/station-venv` nor `Firmware/build` exists on this Pi checkout. No station/CAN
systemd unit files were listed. Re-provisioning and build are needed in addition
to the code adaptation. No package installation was performed in this audit.

The source still selects `yousee`, `/dev/ttyUSB0`, pitch ID 100 and yaw ID 101,
both CyberGears, endpoint yaw homing and soft-center yaw parking. It cannot run
this installation by changing the bus name alone. Old calibration, gains,
loaded limits and parking sign-off do not transfer to the new mechanism.

## Installed BNO085 and existing host probe

The owner confirmed that the previously proposed BNO085 is now installed.
The working probe is **`/home/eamars/workspace/imu-lab/imu_main`**, with
`main.c`, `README.md` and an SH-2/SHTP library copy in `rd/`. Earlier experiments
`probe2.py`, `imu_read.py`, `imu_sh2.py` and `raw_dump.py` are also present.
The lab's project-local `.venv` exists; this does not provision OpenAutoTurret's
missing venv. No probe files were copied into production.

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
- Pitch endstop geometry and load support, new direction signs and transmission
  ratios, yaw mechanical reference, supply/termination and slip-ring ratings.
- Camera lenses, focus/FOV, mounts/orientation, intrinsics/extrinsics and overlap.
- HAT label/SKU and Hailo runtime architecture after driver provisioning.
- IMU product/firmware identity, mount/lever arm, accuracy and timestamp quality,
  magnetic behavior with motors energized, and long-run I2C reliability.

No runtime captures, credentials, retained calibration or virtual environments
belong in source control. This document records distilled observations only.
