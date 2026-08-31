# Systemd units (architecture §52)

Deployment units for the four OpenAutoTurret services plus the CAN bring-up they
depend on. These are **templates**: they are not enabled or run in this repo
(no CAN, no motor). Before deploying, adjust the `User`/`Group` and the
`/opt/open_auto_turret` install root in each file.

| Unit                  | What it runs                          | After (soft `Wants=`)     |
|-----------------------|---------------------------------------|---------------------------|
| `can0.service`        | Configure + bring up `can0` @ 1 Mbit/s | — (local-filesystem)      |
| `turret-control.service` | `controld` (sole owner of can0)     | `can0.service`, network   |
| `turret-vision.service`  | `python -m vision.visiond --real`    | `turret-control.service`  |
| `turret-web.service`     | `python -m web.webd.app`             | `turret-control.service`  |
| `turret-log.service`     | tail the daemon logs (optional)       | the three daemons         |

## The UNHOMED-on-restart guarantee (§52)

`turret-control.service` uses `Restart=on-failure`. This is safe because
`controld` **always starts at UNHOMED**: every (re)start re-runs CAN discovery +
self-test + homing from a known stop before any motion is allowed. A crash can
therefore never silently resume stale coordinates — the station returns to
UNHOMED and re-homes. `KillSignal=SIGINT` maps a stop to the §33 safe-park path
(de-energize) rather than a hard abort.

## Network independence (§53)

`Wants=` (not `Requires=`) is used everywhere so the deterministic local loop
never depends on Wi-Fi/Ethernet or the web UI being up. The web bind host, port,
and controld socket are set via `OTA_WEB_*` env (nothing hard-coded); access is
by hostname, not IP.

## Install (on the target)

```sh
# 1. Install the firmware to /opt/open_auto_turret (build + config + venv), and
#    make sure /opt/open_auto_turret/.venv has fastapi/uvicorn/picamera2.
# 2. Copy the units and reload systemd.
sudo cp /path/to/Firmware/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
# 3. Enable the required units (turret-log is optional).
sudo systemctl enable can0.service turret-control.service turret-vision.service turret-web.service
# 4. Start (and watch control return to UNHOMED, then home, then safe hold).
sudo systemctl start can0.service
sudo systemctl start turret-control.service
sudo systemctl start turret-vision.service turret-web.service
journalctl -u turret-control.service -f
```

## Notes

- **can0** is an MCP2515 kernel driver (dtoverlay); `can0.service` waits for the
  interface, sets 1 Mbit/s, and brings it UP. If the driver is missing, control
  fails its CAN-open step and stays UNHOMED (no motion).
- **Vision** runs `--real` (IMX500) on the target; drop `--real` for the
  synthetic, hardware-free development mode.
- **web** and **control** share `/run/ota/controld-web.sock`; `control` creates
  `/run/ota` in its `ExecStartPre`.
- `PrivateTmp` is left off for control + vision because they share
  `/tmp/ota_vision.sock`.
