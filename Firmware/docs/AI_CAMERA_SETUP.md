# Raspberry Pi AI Camera Setup Record

Date: 2026-08-30
Host: `192.168.2.79` (`rpi-turret`)
Guide: [Raspberry Pi AI Camera documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)

## Result

Successful. The Pi detects the Sony IMX500 AI Camera, the official IMX500 software is installed, the camera driver loads the IMX500 firmware, and the official MobileNet SSD pipeline completes successfully.

## Process and Pi output

1. Preflight checks

   - Model: `Raspberry Pi 5 Model B Rev 1.0`
   - OS: `Debian GNU/Linux 13 (trixie)`
   - Architecture: `aarch64`
   - Camera enumeration: `0 : imx500 [4056x3040 10-bit RGGB]`
   - Initial package state: `imx500-all` was not installed.

2. Updated the operating system using the guide command:

   ```text
   sudo apt update && sudo apt full-upgrade
   ```

   Pi output: `All packages are up to date.`; `Upgrading: 0, Installing: 0, Removing: 0`; exit `0`.

3. Installed the official IMX500 support:

   ```text
   sudo apt install imx500-all
   ```

   Pi output: installed successfully, exit `0`. Installed versions included:

   - `imx500-all 1.13.0-1`
   - `imx500-firmware 0.FF23+3`
   - `imx500-models 1:1.0.0-1`
   - `imx500-tools 0~20241022+2-1+trixie`
   - `rpicam-apps-imx500-postprocess 1.13.0-1`

4. Rebooted the Pi as required by the guide.

   Pi output: SSH connection reset during reboot. After reconnecting, the Pi reported kernel `6.18.39+rpt-rpi-2712`.

5. Verified the installation and camera.

   - Firmware files present: `/lib/firmware/imx500_loader.fpk` and `/lib/firmware/imx500_firmware.fpk`.
   - Model files present: `23`.
   - Official post-processing file present: `/usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json`.
   - Camera enumeration still reports the IMX500 sensor.
   - Kernel output reports: `imx500 ... main firmware version: 02.07.00`.

6. Ran the official MobileNet SSD command in headless mode for SSH verification:

   ```text
   rpicam-hello -n -t 5s --post-process-file /usr/share/rpi-camera-assets/imx500_mobilenet_ssd.json --viewfinder-width 1920 --viewfinder-height 1080 --framerate 30
   ```

   The first run uploaded the network firmware to 99% and exited with `139` (segmentation fault). A normal camera stream then passed with exit `0`; retrying the official AI command uploaded the network firmware to `100%` and exited with `0`.

## Final verification

The Raspberry Pi AI Camera setup is operational. No application or service was left running by the verification commands.

## Follow-up model notes — setup deferred

No YOLO model was loaded or configured during this workflow. The only AI model exercised was the official MobileNet SSD pipeline.

For OpenAutoTurret, the recommended next model is:

```text
/usr/share/imx500-models/imx500_network_yolo11n_pp.rpk
```

Reason: `Firmware/tracking.py` uses YOLOv8 segmentation but only consumes person bounding boxes and their centres; it does not use segmentation masks. YOLO11n is therefore a better fit than a segmentation model, and the post-processing variant is intended to run post-processing on the IMX500.

Implementation notes for a future change:

- The current OpenCV/Ultralytics `model.track()` pipeline cannot consume an IMX500 `.rpk` directly.
- A future integration should use Picamera2/IMX500 to load the RPK and pass bounding-box detections into the existing target-selection/controller logic.
- YOLOv8n post-processed is the fallback option if matching the current YOLOv8 model family is preferred.
- The official YOLO11n and YOLOv8n RPKs are AGPL-3.0 licensed; review this against the project GPLv3 license before distribution.
- YOLO setup and project-code changes were intentionally deferred at the user's request.
