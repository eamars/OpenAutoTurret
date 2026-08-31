# Waveshare RS485 CAN HAT Setup Record

Date: 2026-08-30
Guide: [Waveshare RS485 CAN HAT Wiki](https://www.waveshare.com/wiki/RS485_CAN_HAT)

## Scope

This workflow checks and configures the HAT locally. No external CAN device was connected, no CAN frames were transmitted, and no end-to-end RS485 test was performed.

## Result

The CAN side of the HAT is available and communicating with the Pi over SPI. The MCP2515 driver initialized successfully and created `can0`. The interface was intentionally left `DOWN/STOPPED` because no CAN bus or peer is connected.

## Process and Pi output

1. Read-only pre-check

   - Board: `Raspberry Pi 5 Model B Rev 1.0`
   - Kernel: `6.18.39+rpt-rpi-2712`, `aarch64`
   - SPI was disabled in `/boot/firmware/config.txt` (`#dtparam=spi=on`).
   - No `/dev/spidev*` or `can0` existed.
   - No MCP2515 probe message was present.

2. Confirmed local support

   - `/boot/firmware/overlays/mcp2515-can0.dtbo` exists.
   - The Pi 5 `dtoverlay=nospi10` entry was left unchanged; it did not prevent the Waveshare overlay from working.

3. Applied the current Waveshare configuration for the 12 MHz HAT variant:

   ```text
   dtparam=spi=on
   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
   ```

   Pi output:

   ```text
   backup: /boot/firmware/config.txt.waveshare-backup-20260830-212219
   8:dtparam=spi=on
   58:dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
   [ACTION 3 COMPLETE]
   ```

4. Rebooted the Pi to load the overlay.

   Pi output: `Connection reset`, followed by a successful SSH reconnect.

5. Verified the CAN HAT after reboot.

   Pi output:

   ```text
   mcp251x spi0.0 can0: MCP2515 successfully initialized.
   /sys/bus/spi/devices/spi0.0 exists
   can0 exists; state DOWN; can state STOPPED
   mcp251x and can_dev modules loaded
   traffic test: skipped (no CAN device connected)
   ```

6. Checked the RS485 UART path.

   Pi output:

   ```text
   /dev/serial0 -> ttyAMA10
   serial-getty@ttyAMA10.service loaded active running
   end-to-end RS485 transmission not tested because no RS485 peer is connected
   ```

   The serial console getty currently owns the UART. No UART configuration was changed in this workflow. It must be disabled before using the HAT’s RS485 port from an application.

7. Confirmed the active oscillator property.

   Pi output: `/sys/firmware/devicetree/base/can0_osc/clock-frequency` contains `00 b7 1b 00`, which is 12,000,000 Hz.

## Final verification

The Waveshare HAT’s MCP2515 CAN controller is detected and initialized over SPI. `can0` is available but deliberately not opened or used on a bus until a CAN device and appropriate bus wiring/termination are connected.
