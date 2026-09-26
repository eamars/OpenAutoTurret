# Hardware adaptation continuation: current ceiling and Hailo

26–27 September 2026. **Partially verified.** This continues the
[first commissioning record](HARDWARE_COMMISSIONING_2026_09_26.md), without
changing its historical results. Automatic operation remains stopped.

## Pitch: 5 A policy and device evidence

The owner requires a maximum pitch current setting of 5 A. The commissioning
configuration uses 5 A; `--apply-pitch-limit` writes only volatile `LimitCur`
and requires three matching successful register responses. It is mutually
exclusive with yaw actuation and does not enable, zero or home pitch.

The real drive at ID `0x7F`, UID `7216313130333105`, accepted 5 A and returned
three matching readbacks, both before and after the Pi reboot. After reboot,
type-2 feedback reported mode 0, no fault bits, position -0.71088 rad and
temperature 22.6 C. The probe sent zero yaw commands and saw no CAN error
frames. The earlier observed 27 A was replaced, never adopted for operation.

This is verification of the configured limit, **not a measurement of peak
physical current**. `LimitCur` applies to speed/position modes and is volatile.
The implementation rejects raw pitch MIT/current-mode commands and requires
supported-mode/current-limit readback before enable. Configuration, homing,
payload checks, adoption, keepalive and adaptive current requests must remain
within the same 5 A ceiling. A reset or firmware update requires new readback.

The motor initially rejected `MechPos` (`0x7019`) with response status 1.
On September 27 the owner confirmed upgrading pitch CyberGear to 1.2.1.5.
The subsequent real probe matched the same UID, obtained `MechPos=-0.710777 rad`
with status 0, and reapplied/read back 5 A three times. Raw feedback reported
mode 0, faults 0 and 19.3 C. This resolves the observed position-register
blocker; the precise version number remains owner-reported. No pitch enable
or motion trial was performed, and the agent did not flash the motor. See the
[firmware reference](CYBERGEAR_FIRMWARE_UPGRADE.md). Numeric evidence is retained
in `run/hardware-adaptation/pitch-post-upgrade.log`.

## GM6020 velocity experiment

A bounded encoder-derived velocity PI probe now sits alongside fixed-voltage
pulses. It uses a 50 ms filter, saturation and anti-windup, rejecting invalid
timing/state. The physical raw-rpm feedback is quantized in 6 deg/s steps.
The existing 500 ms pulse, 5 deg travel, 20 deg/s raw-speed and freshness guards
still apply. None of these values is a production tracking qualification.

The first +5 deg/s, 500 ms experiment with Kp=8500, Ki=1500 and a 1000 raw
ceiling stalled near static friction: peak travel 0.132 deg, final +0.088 deg.
It did **not** demonstrate speed tracking. Fixed +1500/-1500 raw, 150 ms
characterization then measured +0.352/-1.099 deg final displacement, with
peak absolute travel 0.483/1.494 deg. The negative run did not meet the short
stationarity window; a subsequent four-second passive observation passed,
with only one encoder-count variation. Neither run reported a CAN error or
failed zero transmission. This motivated another bounded gain trial, rather
than adopting the initial gains for production.

The revised loop (Kp=35000, Ki=20000, output ceiling 1500 raw) was exercised
on September 27 using +/-3 deg/s requests for 500 ms, followed by four seconds
of observation:

| Request | Peak absolute travel | Final displacement | Peak raw-rpm speed | Stationary afterward |
|---|---:|---:|---:|---|
| +3 deg/s | 0.176 deg | +0.132 deg | 0 deg/s | Yes |
| -3 deg/s | 0.747 deg | -0.483 deg | 12 deg/s | Yes |
| +2000 raw voltage, 100 ms characterization | 1.143 deg | +1.099 deg | 12 deg/s | Yes |

All reported zero CAN error frames and no failed zero-output requests. The
positive velocity trial spent most of its active interval near the output
ceiling (mean 1455 raw), yet still fell short of its requested speed. This is
evidence that the current commissioning gains/ceiling do **not** establish
usable speed regulation. The stronger short pulse establishes another bounded
response point; friction, direction asymmetry and voltage-to-motion behavior
need characterization before choosing production feedforward/gains. The source
retains the 1500 raw velocity ceiling; the 2000 pulse did not raise it.
Traces are `yaw-speed-positive-revised.csv`, `yaw-speed-negative-revised.csv`
and `yaw-positive-2000.csv` in the ignored evidence directory.

## Hailo: installed and exercised with the actual camera

Minimal OS packages installed: `dkms` 3.2.2, `hailort` and
`hailort-pcie-driver` 4.23.0, and `python3-hailort` 4.23.0-1 plus dependencies.
The transaction added eight packages, upgraded/removed none, and built the
driver for the existing `6.18.39+rpt-rpi-2712` kernel. The kernel did not change.
No global pip installation, TAPPAS or full `hailo-all` stack was used.

After a controlled reboot, `/dev/hailo0`, firmware 4.23.0 and architecture
**HAILO8** were verified. The project venv imports the OS Python bindings.
The official Hailo-8 Model Zoo v2.17.0 YOLOv8n artifact has SHA-256
`e893b0f9dcae366fe1bc9ebce25e32ad889acf2bc58cfe1f73a572f78f7ec055`.
Its input is UINT8 NHWC 640x640x3 and output is FLOAT32 NMS by 80 classes.
The binary remains in ignored `run/hailo-probe/` on the workstation and Pi.

The 30-frame Hailo CLI benchmark measured 3.36 ms device latency, but used
synthetic input. The meaningful camera probe then acquired IMX477 RGB frames
at 640x480, 15 FPS, letterboxed with RGB (114,114,114), and executed the real
Python Hailo pipeline. All 30 timestamps increased and all output values were
finite. Inference time p50/p95 was **6.86/7.05 ms**; sensor timestamp to result
was **20.99/21.69 ms**. These are component timings, not optical tracking latency.

The first Python attempt failed because PIL's NumPy view was read-only; an
explicit writable contiguous copy corrected the input buffer. No device or
driver change was required. No boxes exceeded 0.5 confidence in the observed
view, so this establishes transport/inference viability, not person/head
detection accuracy or identity recognition. No camera images were retained.
Representative scene evaluation and application integration are next.

The committed reusable probe passed a second 30-frame IMX477 run while the Pi
was compiling the controller: inference p50/p95 **8.47/12.41 ms**, sensor to
result **26.18/31.02 ms**. These timings include CPU contention and startup;
its aggregate run rate of 10.42 FPS is not steady-state camera throughput.
The probe verified the live HAILO8 identity and pinned model hash, then released
the camera. Numeric output is in `hailo-camera-repeat.json`.

## Regression verification

All **77 CTest targets passed on Linux/WSL and on the actual Pi** after the
current-guard changes (Pi: 49.93 s). Both Linux launcher pytest tests also
passed on the Pi (4.19 s).
The mode-transition probe additionally simulates a drive ignoring the limit
write and retaining 27 A: both pitch modes must refuse enable. The regression
run also exposed and corrected an initial-stop displacement check and a
redundant post-park STOP. Neither was tested by energizing the real pitch motor.
Software verification does not certify the upgraded motor's loaded response.

Tested runtime revision: `1003bdc894f1b0176e32c8156c0898b085927767`, deployed
from committed source, without activation, into:

```text
/home/eamars/workspace/OpenAutoTurret/run/releases/1003bdc894f1.UREuRU
```

After testing, that release repeated the non-motion pitch setup successfully:
same UID, valid MechPos, three 5 A readbacks, mode 0/faults 0. The launcher was
stopped, both CAN links restored DOWN/STOPPED at 1 Mbps, cameras closed, and
no controller/perception/probe process remained. Both buses had zero kernel
error counters. Yaw's zero request and observed stationarity do not certify
electrical disable or process-loss stopping. The original Pi checkout remained
clean. See `pitch-limit-final.log`, `final-host-state.log`,
`deploy-five-amp.log` and `launcher-five-amp.log` in the ignored evidence folder.

## Host and evidence

The original Pi checkout and retained calibration are preserved. Station
processes and probes run as `eamars`; elevation was used only for the OS
driver/runtime installation, reboot and temporary CAN link configuration.
After reboot, Windows hostname resolution failed; mDNS identified the Pi at
192.168.2.100 and SSH verified its existing `rpi-turret` host key. Deployment
now accepts `--connect-address` while retaining the configured host identity.
The observed address is not a newly assigned static address.

Numeric logs remain under ignored `run/hardware-adaptation/`, including
`pitch-limit-first.log`, `pitch-limit-after-reboot-controller.log`,
`yaw-speed-positive.csv`, `yaw-positive-1500.csv`, `yaw-negative-1500.csv`,
`yaw-settle-after-1500.log`, `hailo-model-run.log` and
`hailo-camera-first.json`. Runtime traces and firmware binaries are not committed.

Remaining gates: supported-load pitch homing; production mixed-drive backend;
continuous-yaw reference/planning; qualified process/link-loss stop behavior;
camera geometry and person/head accuracy; IMU and perception integration.
